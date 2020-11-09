#![no_std]

// feature approach cribbed from https://github.com/proman21/samd-dma
#[cfg(not(feature = "samd21"))]
compile_error!("Please use this crate's feature flags to select a target.");

#[cfg(feature = "samd21")]
extern crate atsamd_hal as hal;

extern crate defmt_rtt;

use hal::gpio;

// use hal::prelude::*;
use hal::target_device as pac;

pub use hal::target_device::i2s::clkctrl::SLOTSIZE_A as SlotSize;

#[derive(Debug)]
pub enum ClockUnit {
    Unit0 = 0,
    Unit1 = 1,
    All
}

// TODO encode the transmitter/receiver/disabled for the two serializers in the type
pub struct I2s {
    hw: pac::I2S,
    serial_clock_pin: gpio::Pa10<gpio::PfG>,
    frame_sync_pin: gpio::Pa11<gpio::PfG>,
    data_in_pin: gpio::Pa7<gpio::PfG>,
    data_out_pin: gpio::Pa8<gpio::PfG>,
}

impl I2s {
    // TODO figure out how to convey frequency of the connected gclk ** the LCD via IIC example **
    // TODO maybe this could allow for either clock unit?
    // data_in
    // data_out
    // sck
    // frame_sync
    pub fn tdm_master(hw: pac::I2S,
        number_of_slots: u8,
        serial_clock_pin: gpio::Pa10<gpio::PfG>,
        frame_sync_pin: gpio::Pa11<gpio::PfG>,
        data_in_pin: gpio::Pa7<gpio::PfG>,
        data_out_pin: gpio::Pa8<gpio::PfG>,
        ) -> Self {
        let ret = Self {
            hw,
            serial_clock_pin,
            frame_sync_pin,
            data_in_pin,
            data_out_pin,
        };

        ret.reset();

        // Just need one clock unit, unsafe is due to nbslots().bits()
        unsafe {
            ret.hw.clkctrl[0].write(|clock_unit| { clock_unit
                .fswidth().bit_()
                .nbslots().bits(number_of_slots - 1)
                .slotsize().variant(SlotSize::_32) // TODO take as argument, reexport unambiguously
            });
        }

        // Configure the Serializers
        // Both serializers use clock unit 0 by default
        ret.hw.serctrl[1].write(|serializer| { serializer
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
        gpio::Pa10<gpio::PfG>,
        gpio::Pa11<gpio::PfG>,
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

    // TODO look in to modifying the PAC to use an enum for this
    pub fn set_num_slots(&self, unit: ClockUnit, num: u8) {
        if num == 0 || num > 8 {
            panic!("Must have between 1 and 8 slots, inclusive");
        }

        self.hw.clkctrl[unit as usize].modify(|_, w| unsafe{w.nbslots().bits(num-1)});
    }
}