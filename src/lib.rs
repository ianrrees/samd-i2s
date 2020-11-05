#![no_std]

// feature approach cribbed from https://github.com/proman21/samd-dma
#[cfg(not(feature = "samd21"))]
compile_error!("Please use this crate's feature flags to select a target.");

#[cfg(feature = "samd21")]
extern crate atsamd_hal as hal;

extern crate defmt_rtt;

use hal::prelude::*;
use hal::target_device as pac;

#[derive(Debug)]
pub enum ClockUnit {
    Unit0 = 0,
    Unit1 = 1,
    All
}

pub struct I2s {
    hw: pac::I2S
}

impl I2s {
    pub fn new(hw: pac::I2S) -> Self {
        Self {
            hw,
        }
    }

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