#![no_std]

// feature approach cribbed from https://github.com/proman21/samd-dma
#[cfg(not(feature = "samd21"))]
compile_error!("Please use this crate's feature flags to select a target.");

#[cfg(feature = "samd21")]
extern crate atsamd_hal as hal;

extern crate defmt_rtt;

use hal::clock;
use hal::gpio;
use hal::target_device as pac;
use hal::time::Hertz;

pub use pac::i2s::clkctrl::SLOTSIZE_A as SlotSize;
use pac::i2s::serctrl::CLKSEL_A as ClockUnitID;

pub struct ClockUnit0 {}
pub struct ClockUnit1 {}

/// Allows compile-time associations between pins and clock units
pub trait ClockUnit {
    fn id() -> ClockUnitID;
}

impl ClockUnit for ClockUnit0 {
    fn id() -> ClockUnitID {
        ClockUnitID::CLK0
    }
}

impl ClockUnit for ClockUnit1 {
    fn id() -> ClockUnitID {
        ClockUnitID::CLK1
    }
}

pub trait MasterClock<ClockUnit> {
    fn freq(&self) -> Hertz;
}
impl MasterClock<ClockUnit0> for hal::clock::I2S0Clock {
    fn freq(&self) -> Hertz {
        self.freq()
    }
}
impl MasterClock<ClockUnit1> for hal::clock::I2S1Clock {
    fn freq(&self) -> Hertz {
        self.freq()
    }
}

pub trait SerialClock<ClockUnit> {}
impl SerialClock<ClockUnit0> for gpio::Pa10<gpio::PfG> {}
impl SerialClock<ClockUnit1> for gpio::Pb11<gpio::PfG> {}

pub trait FrameSync<ClockUnit> {}
impl FrameSync<ClockUnit0> for gpio::Pa11<gpio::PfG> {}
impl FrameSync<ClockUnit1> for gpio::Pb12<gpio::PfG> {}

// TODO encode the transmitter/receiver/disabled for the two serializers in the type
pub struct I2s<SerialClockPin, FrameSyncPin> {
    hw: pac::I2S,
    serial_clock_pin: SerialClockPin,
    frame_sync_pin: FrameSyncPin,
    data_in_pin: gpio::Pa7<gpio::PfG>,
    data_out_pin: gpio::Pa8<gpio::PfG>,
}

impl<SerialClockPin, FrameSyncPin> I2s<SerialClockPin, FrameSyncPin> {
    /// master_clock_generator, serial_clock_pin, and frame_sync_pin must be attached to the same clock unit
    pub fn tdm_master<ClockGenerator, ClkUnit: ClockUnit, Freq: Into<Hertz>>(
        hw: pac::I2S,
        pm: &mut hal::target_device::PM,
        master_clock_generator: ClockGenerator,
        serial_freq: Freq,
        number_of_slots: u8,
        serial_clock_pin: SerialClockPin,
        frame_sync_pin: FrameSyncPin,
        data_in_pin: gpio::Pa7<gpio::PfG>,
        data_out_pin: gpio::Pa8<gpio::PfG>,
    ) -> Self
    where
        ClockGenerator: MasterClock<ClkUnit>,
        SerialClockPin: SerialClock<ClkUnit>,
        FrameSyncPin: FrameSync<ClkUnit>,
    {
        // Turn on the APB clock to the I2S peripheral
        pm.apbcmask.modify(|_, w| w.i2s_().set_bit());

        Self::reset(&hw);

        let clock_unit = ClkUnit::id();

        // unsafe is due to the bits() calls
        unsafe {
            hw.clkctrl[clock_unit as usize].write(|clock_unit| {
                clock_unit
                    .fswidth()
                    .bit_()
                    .nbslots()
                    .bits(number_of_slots - 1)
                    .slotsize()
                    .variant(SlotSize::_32);
                let divisor = (master_clock_generator.freq().0 / serial_freq.into().0 - 1) as u8;
                clock_unit.mckdiv().bits(divisor)
            });
        }

        hw.serctrl[0].write(|w| w.clksel().variant(clock_unit));

        hw.serctrl[1].write(|w| w.clksel().variant(clock_unit).sermode().tx());

        // Synchronization doesn't seem to happen until the peripheral is enabled

        match clock_unit {
            ClockUnitID::CLK0 => {
                hw.ctrla.modify(|_, w| w.cken0().set_bit());
            }

            ClockUnitID::CLK1 => {
                hw.ctrla.modify(|_, w| w.cken1().set_bit());
            }
        }

        hw.ctrla
            .modify(|_, w| w.seren0().set_bit().seren1().set_bit());

        Self {
            hw,
            serial_clock_pin,
            frame_sync_pin,
            data_in_pin,
            data_out_pin,
        }
    }

    pub fn send(&self, v: u32) {
        while self.hw.intflag.read().txrdy1().bit_is_clear() {}

        unsafe {
            self.hw.data[1].write(|reg| reg.data().bits(v));
        }
    }

    /// Gives the peripheral and pins back
    pub fn free(
        self,
    ) -> (
        pac::I2S,
        SerialClockPin,
        FrameSyncPin,
        gpio::Pa7<gpio::PfG>,
        gpio::Pa8<gpio::PfG>,
    ) {
        (
            self.hw,
            self.serial_clock_pin,
            self.frame_sync_pin,
            self.data_in_pin,
            self.data_out_pin,
        )
    }

    /// Blocking software reset of the peripheral
    fn reset(hw: &pac::I2S) {
        hw.ctrla.write(|w| w.swrst().set_bit());

        while hw.syncbusy.read().swrst().bit_is_set() || hw.ctrla.read().swrst().bit_is_set() {}
    }

    /// Enable the peripheral
    pub fn enable(&self) {
        self.hw.ctrla.modify(|_, w| w.enable().set_bit());

        while self.hw.syncbusy.read().cken0().bit_is_set()
            || self.hw.syncbusy.read().cken1().bit_is_set()
            || self.hw.syncbusy.read().seren0().bit_is_set()
            || self.hw.syncbusy.read().seren1().bit_is_set()
            || self.hw.syncbusy.read().enable().bit_is_set()
        {}
    }
}
