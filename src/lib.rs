#![no_std]

// feature approach cribbed from https://github.com/proman21/samd-dma
#[cfg(not(feature = "samd21"))]
compile_error!("Please use this crate's feature flags to select a target.");

#[cfg(feature = "samd21")]
extern crate atsamd_hal as hal;

extern crate defmt_rtt;

use core::marker::PhantomData;
use hal::gpio;

// use hal::prelude::*;
use hal::target_device as pac;

pub use hal::target_device::i2s::clkctrl::SLOTSIZE_A as SlotSize;
use hal::target_device::i2s::serctrl::CLKSEL_A as ClockUnitID;

pub struct ClockUnit0 {}
pub struct ClockUnit1 {}
pub trait ClockUnit {
    fn id() -> ClockUnitID;
}

impl ClockUnit for ClockUnit0 {
    fn id() -> ClockUnitID { ClockUnitID::CLK0 }
}

impl ClockUnit for ClockUnit1 {
    fn id() -> ClockUnitID { ClockUnitID::CLK1 }
}

pub trait SerialClock<ClockUnit> {}
pub trait FrameSync<ClockUnit> {}

impl SerialClock<ClockUnit0> for gpio::Pa10<gpio::PfG> {}
impl SerialClock<ClockUnit1> for gpio::Pb11<gpio::PfG> {}

impl FrameSync<ClockUnit0> for gpio::Pa11<gpio::PfG> {}
impl FrameSync<ClockUnit1> for gpio::Pb12<gpio::PfG> {}

// TODO encode the transmitter/receiver/disabled for the two serializers in the type
pub struct I2s<ClkUnit, SerialClockPin, FrameSyncPin> {
    hw: pac::I2S,
    serial_clock_pin: SerialClockPin,
    frame_sync_pin: FrameSyncPin,
    data_in_pin: gpio::Pa7<gpio::PfG>,
    data_out_pin: gpio::Pa8<gpio::PfG>,
    phantom: PhantomData<ClkUnit>
}

impl<ClkUnit: ClockUnit, SerialClockPin, FrameSyncPin> I2s<ClkUnit, SerialClockPin, FrameSyncPin> {
    // TODO figure out how to convey frequency of the connected gclk ** the LCD via IIC example **
    // data_in
    // data_out
    // sck
    // frame_sync
    pub fn tdm_master(
        hw: pac::I2S,
        number_of_slots: u8,
        serial_clock_pin: SerialClockPin,
        frame_sync_pin: FrameSyncPin,
        data_in_pin: gpio::Pa7<gpio::PfG>, // TODO use option<>
        data_out_pin: gpio::Pa8<gpio::PfG>, // TODO use option<>
        ) -> Self
    where
        SerialClockPin: SerialClock<ClkUnit>,
        FrameSyncPin: FrameSync<ClkUnit>
    {
        let ret = Self {
            hw,
            serial_clock_pin,
            frame_sync_pin,
            data_in_pin,
            data_out_pin,
            phantom: PhantomData,
        };

        ret.reset();

        let clock_unit = ClkUnit::id();

        // unsafe is due to nbslots().bits()
        unsafe {
            ret.hw.clkctrl[clock_unit as usize].write(|clock_unit| { clock_unit
                .fswidth().bit_()
                .nbslots().bits(number_of_slots - 1)
                .slotsize().variant(SlotSize::_32) // TODO take as argument, reexport unambiguously
            });
        }

        ret.hw.serctrl[0].write(|serializer| { serializer
            .clksel().variant(clock_unit)
        });

        ret.hw.serctrl[1].write(|serializer| { serializer
            .clksel().variant(clock_unit)
            .sermode().tx()
        });

        // Enable the used Clock Unit and Serializers
        ret.hw.ctrla.write(|control| { control
            .seren1().set_bit()
            .seren0().set_bit()
            .cken1().set_bit()
        });

        ret
    }

    /// Gives the peripheral and pins back
    pub fn free(self) -> (
        pac::I2S,
        SerialClockPin,
        FrameSyncPin,
        gpio::Pa7<gpio::PfG>,
        gpio::Pa8<gpio::PfG>,) {(
            self.hw,
            self.serial_clock_pin,
            self.frame_sync_pin,
            self.data_in_pin,
            self.data_out_pin
    )}

    /// Blocking software reset of the peripheral
    pub fn reset(&self) {
        self.hw.ctrla.write(|w| w.swrst().set_bit());

        while self.hw.ctrla.read().swrst().bit_is_set() {
            // spin
        }
    }

    /// Enable the peripheral
    pub fn enable(&self) {
        self.hw.ctrla.modify(|_, w| w.enable().set_bit());
    }
}