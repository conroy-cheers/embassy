use embassy_hal_internal::into_ref;

use super::{
    blocking_delay_us, Adc, AdcChannel, AnyAdcChannel, Instance, Resolution, RxDma, SampleTime, SealedAdcChannel,
};
use crate::dma::Transfer;
use crate::{rcc, Peripheral};

/// Default VREF voltage used for sample conversion to millivolts.
pub const VREF_DEFAULT_MV: u32 = 3300;
/// No calibration data for C011, voltage should be 1212mV
pub const VREF_INT: u32 = 1212;

pub struct VrefInt;
impl<T: Instance> AdcChannel<T> for VrefInt {}
impl<T: Instance> SealedAdcChannel<T> for VrefInt {
    fn channel(&self) -> u8 {
        10
    }
}

impl<'d, T: Instance> Adc<'d, T> {
    pub fn new(adc: impl Peripheral<P = T> + 'd) -> Self {
        into_ref!(adc);
        rcc::enable_and_reset::<T>();

        T::regs().cfgr1().modify(|reg| {
            reg.set_chselrmod(false);
        });

        blocking_delay_us(20);

        // Perform calibration
        T::regs().cr().modify(|reg| {
            reg.set_adcal(true);
        });

        while T::regs().cr().read().adcal() {
            // spin
        }

        blocking_delay_us(1);

        Self {
            adc,
            sample_time: SampleTime::from_bits(0),
        }
    }

    // Enable ADC only when it is not already running.
    fn enable(&mut self) {
        // Make sure bits are off
        while T::regs().cr().read().addis() {
            // spin
        }

        if !T::regs().cr().read().aden() {
            // Enable ADC
            T::regs().isr().modify(|reg| {
                reg.set_adrdy(true);
            });
            T::regs().cr().modify(|reg| {
                reg.set_aden(true);
            });

            while !T::regs().isr().read().adrdy() {
                // spin
            }
        }
    }

    pub fn enable_vrefint(&self) -> VrefInt {
        T::regs().ccr().modify(|reg| {
            reg.set_vrefen(true);
        });

        // "Table 26. Embedded internal voltage reference" states that it takes a maximum of 12 us
        // to stabilize the internal voltage reference.
        blocking_delay_us(15);

        VrefInt {}
    }

    /// Set the ADC sample time.
    pub fn set_sample_time(&mut self, sample_time: SampleTime) {
        self.sample_time = sample_time;
    }

    /// Get the ADC sample time.
    pub fn sample_time(&self) -> SampleTime {
        self.sample_time
    }

    /// Set the ADC resolution.
    pub fn set_resolution(&mut self, resolution: Resolution) {
        T::regs().cfgr1().modify(|reg| reg.set_res(resolution.into()));
    }

    /// Perform a single conversion.
    fn convert(&mut self) -> u16 {
        T::regs().isr().modify(|reg| {
            reg.set_eos(true);
            reg.set_eoc(true);
        });

        // Start conversion
        T::regs().cr().modify(|reg| {
            reg.set_adstart(true);
        });

        while !T::regs().isr().read().eos() {
            // spin
        }

        T::regs().dr().read().0 as u16
    }

    /// Read an ADC channel.
    pub fn blocking_read(&mut self, channel: &mut impl AdcChannel<T>) -> u16 {
        self.read_channel(channel)
    }

    /// Read one or multiple ADC channels using DMA.
    ///
    /// `sequence` iterator and `readings` must have the same length.
    ///
    /// Example
    /// ```rust,ignore
    /// use embassy_stm32::adc::{Adc, AdcChannel}
    ///
    /// let mut adc = Adc::new(p.ADC1);
    /// let mut adc_pin0 = p.PA0.degrade_adc();
    /// let mut adc_pin1 = p.PA1.degrade_adc();
    /// let mut measurements = [0u16; 2];
    ///
    /// adc.read_async(
    ///     p.DMA1_CH2,
    ///     [
    ///         (&mut *adc_pin0, SampleTime::CYCLES160_5),
    ///         (&mut *adc_pin1, SampleTime::CYCLES160_5),
    ///     ]
    ///     .into_iter(),
    ///     &mut measurements,
    /// )
    /// .await;
    /// defmt::info!("measurements: {}", measurements);
    /// ```
    pub async fn read(
        &mut self,
        rx_dma: &mut impl RxDma<T>,
        sequence: impl ExactSizeIterator<Item = (&mut AnyAdcChannel<T>, SampleTime)>,
        readings: &mut [u16],
    ) {
        assert!(sequence.len() != 0, "Asynchronous read sequence cannot be empty");
        assert!(
            sequence.len() == readings.len(),
            "Sequence length must be equal to readings length"
        );
        assert!(
            sequence.len() <= 16,
            "Asynchronous read sequence cannot be more than 16 in length"
        );

        // Ensure no conversions are ongoing and ADC is enabled.
        Self::cancel_conversions();
        self.enable();

        let mut channel_mask = 0;

        // Configure channels and ranks
        for (_i, (channel, sample_time)) in sequence.enumerate() {
            Self::set_channel_sample_time(channel.channel(), sample_time);
            channel_mask |= 1 << channel.channel();
        }

        // Enabled channels are sampled from 0 to last channel.
        // It is possible to add up to 8 sequences if CHSELRMOD = 1.
        // However for supporting more than 8 channels alternative CHSELRMOD = 0 approach is used.
        T::regs().chselr().modify(|reg| {
            reg.set_chsel(channel_mask);
        });

        // Set continuous mode with oneshot dma.
        // Clear overrun flag before starting transfer.
        T::regs().isr().modify(|reg| {
            reg.set_ovr(true);
        });

        T::regs().cfgr1().modify(|reg| {
            reg.set_discen(false);
            reg.set_cont(true);
            reg.set_dmacfg(false);  // oneshot
            reg.set_dmaen(true);
        });

        let request = rx_dma.request();
        let transfer = unsafe {
            Transfer::new_read(
                rx_dma,
                request,
                T::regs().dr().as_ptr() as *mut u16,
                readings,
                Default::default(),
            )
        };

        // Start conversion
        T::regs().cr().modify(|reg| {
            reg.set_adstart(true);
        });

        // Wait for conversion sequence to finish.
        transfer.await;

        // Ensure conversions are finished.
        Self::cancel_conversions();

        // Reset configuration.
        T::regs().cfgr1().modify(|reg| {
            reg.set_cont(false);
        });
    }

    fn read_channel(&mut self, channel: &mut impl AdcChannel<T>) -> u16 {
        self.enable();
        Self::set_channel_sample_time(channel.channel(), self.sample_time);

        // Select channel
        T::regs().chselr().write(|reg| reg.set_chsel(1 << channel.channel()));

        let val = self.convert();

        T::regs().cr().modify(|reg| reg.set_addis(true));

        val
    }

    fn set_channel_sample_time(_ch: u8, sample_time: SampleTime) {
        // All channels use the same sampling time.
        T::regs().smpr().modify(|reg| reg.set_smp1(sample_time.into()));
    }

    fn cancel_conversions() {
        if T::regs().cr().read().adstart() && !T::regs().cr().read().addis() {
            T::regs().cr().modify(|reg| {
                reg.set_adstp(true);
            });
            while T::regs().cr().read().adstart() {}
        }
    }
}
