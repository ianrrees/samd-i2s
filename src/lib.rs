#![no_std]

// feature approach cribbed from https://github.com/proman21/samd-dma
#[cfg(not(feature = "samd21"))]
compile_error!("Please use this crate's feature flags to select a target.");

#[cfg(feature = "samd21")]
extern crate atsamd_hal as hal;

extern crate defmt_rtt;

use core::marker::PhantomData;

use hal::gpio;
use hal::target_device as pac;
use hal::time::Hertz;

// TODO for samd5x parts, this will need to be pac::dmac::chctrla::TRIGSRC_A
#[cfg(any(feature = "samd11", feature = "samd21"))]
pub use pac::dmac::chctrlb::TRIGSRC_A as DmaTriggerSource;
// Havent 
#[cfg(any(
    feature = "samd51",
    feature = "same51",
    feature = "same53",
    feature = "same54"
))]
pub use pac::dmac::chctrla::TRIGSRC_A as DmaTriggerSource;

pub use pac::i2s::clkctrl::SLOTSIZE_A as BitsPerSlot;
use pac::i2s::serctrl::CLKSEL_A as ClockUnitID;

//////////// This probably belongs in an I2S trait crate? ////////////
#[derive(Debug)]
pub enum I2SError {
    /// An operation would block because the device is currently busy or there is no data available.
    WouldBlock,
}

/// Result for I2S operations.
pub type Result<T> = core::result::Result<T, I2SError>;

pub struct ClockUnit0;
pub struct ClockUnit1;

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

// TODO perhaps have something like this in gpio?
pub struct ExternalClock<PinType> {
    frequency: Hertz,
    pin: PhantomData<PinType>,
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

impl MasterClock<ClockUnit0> for ExternalClock<gpio::Pa9<gpio::PfG>> {
    fn freq(&self) -> Hertz {
        self.frequency
    }
}

#[cfg(any(feature = "min-samd21j"))] // TODO pick min packages for each GPIO pin, also for samd5x
impl MasterClock<ClockUnit0> for ExternalClock<gpio::Pb17<gpio::PfG>> {
    fn freq(&self) -> Hertz {
        self.frequency
    }
}

impl MasterClock<ClockUnit1> for ExternalClock<gpio::Pb10<gpio::PfG>> {
    fn freq(&self) -> Hertz {
        self.frequency
    }
}

pub trait SerialClock<ClockUnit> {}
impl SerialClock<ClockUnit0> for gpio::Pa10<gpio::PfG> {}
impl SerialClock<ClockUnit1> for gpio::Pb11<gpio::PfG> {}

pub trait FrameSync<ClockUnit> {}
impl FrameSync<ClockUnit0> for gpio::Pa11<gpio::PfG> {}
#[cfg(any(feature = "min-samd21j"))]
impl FrameSync<ClockUnit1> for gpio::Pb12<gpio::PfG> {}

/// The I2S peripheral has two serializers; refer to them using this enum
pub enum Serializer {
    M0, // 'm' is the datasheet convention
    M1,
}

// TODO could this be an enum instead of a trait?
/// The I2S peripheral has two serializers, each can be used as either an input or an output.  The
/// SerializerOrientation trait is used to indicate which serializer is used for each direction.
pub trait SerializerOrientation {
    fn tx_id() -> Serializer;
    fn rx_id() -> Serializer;
}

/// Transmit from serializer 0, receive on serializer 1
pub struct Tx0Rx1;
impl SerializerOrientation for Tx0Rx1 {
    fn tx_id() -> Serializer {
        Serializer::M0
    }
    fn rx_id() -> Serializer {
        Serializer::M1
    }
}

/// Transmit from serializer 1, receive on serializer 0
pub struct Tx1Rx0;
impl SerializerOrientation for Tx1Rx0 {
    fn tx_id() -> Serializer {
        Serializer::M1
    }
    fn rx_id() -> Serializer {
        Serializer::M0
    }
}

// TODO make these optional, in particular the Tx one to support PDM mics
pub trait SerializerTx<SerializerOrientation> {}
impl SerializerTx<Tx0Rx1> for gpio::Pa7<gpio::PfG> {}
impl SerializerTx<Tx1Rx0> for gpio::Pa8<gpio::PfG> {}
impl SerializerTx<Tx0Rx1> for gpio::Pa19<gpio::PfG> {}
#[cfg(any(feature = "min-samd21j"))]
impl SerializerTx<Tx1Rx0> for gpio::Pb16<gpio::PfG> {}

pub trait SerializerRx<SerializerOrientation> {}
impl SerializerRx<Tx1Rx0> for gpio::Pa7<gpio::PfG> {}
impl SerializerRx<Tx0Rx1> for gpio::Pa8<gpio::PfG> {}
impl SerializerRx<Tx1Rx0> for gpio::Pa19<gpio::PfG> {}
#[cfg(any(feature = "min-samd21j"))]
impl SerializerRx<Tx0Rx1> for gpio::Pb16<gpio::PfG> {}


pub struct I2s<MasterClockSource, SerialClockPin, FrameSyncPin, RxPin, TxPin> {
    hw: pac::I2S,
    serial_clock_pin: SerialClockPin,
    frame_sync_pin: FrameSyncPin,
    data_in_pin: RxPin,
    data_out_pin: TxPin,
    master_clock_source: MasterClockSource,
}

// Need to support three clocking configurations:
//   gclck master clock (frequency) => serial clock is output (pin+frequency)
//   external master clock (pin+frequency) => serial clock is output (pin+frequency)
//   No master clock => serial clock is input (pin)

impl<MasterClockSource, SerialClockPin, FrameSyncPin, RxPin, TxPin>
    I2s<MasterClockSource, SerialClockPin, FrameSyncPin, RxPin, TxPin> {
    /// master_clock_source, serial_clock_pin, and frame_sync_pin must be attached to the same clock unit
    /// TxPin and RxPin need to be connected to different serializer units
    pub fn tdm_master<ClkUnit: ClockUnit, SerializerCfg: SerializerOrientation, Freq: Into<Hertz>>(
        hw: pac::I2S,
        pm: &mut hal::target_device::PM,
        master_clock_source: MasterClockSource,
        serial_freq: Freq,
        number_of_slots: u8,
        bits_per_slot: BitsPerSlot,
        serial_clock_pin: SerialClockPin,
        frame_sync_pin: FrameSyncPin,
        data_in_pin: RxPin,
        data_out_pin: TxPin,
    ) -> Self
    where
        MasterClockSource: MasterClock<ClkUnit>,
        SerialClockPin: SerialClock<ClkUnit>,
        FrameSyncPin: FrameSync<ClkUnit>,
        RxPin: SerializerRx<SerializerCfg>,
        TxPin: SerializerTx<SerializerCfg>, 
    {
        // Turn on the APB clock to the I2S peripheral
        pm.apbcmask.modify(|_, w| w.i2s_().set_bit());

        Self::reset(&hw);

        let clock_unit = ClkUnit::id();

        defmt::info!("Master clock running at {:u32}", master_clock_source.freq().0);

        let master_clock_divisor = (master_clock_source.freq().0 / serial_freq.into().0 - 1) as u8;
        defmt::info!("divisor is {:u8}", master_clock_divisor);

        // unsafe is due to the bits() calls
        unsafe {
            hw.clkctrl[clock_unit as usize].write(|w|
                w
                    .mckdiv().bits(master_clock_divisor)
                    // .mcksel().mckpin() // Use MCK pin as master clock input
                    // .scksel().sckpin() // Uses SCK pin as input
                    .fswidth().bit_()
                    .nbslots().bits(number_of_slots - 1)
                    .slotsize().variant(bits_per_slot)
            );
        }

        hw.serctrl[SerializerCfg::rx_id() as usize].write(|w| w.clksel().variant(clock_unit));

        hw.serctrl[SerializerCfg::tx_id() as usize].write(|w| w.clksel().variant(clock_unit).sermode().tx());

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
            master_clock_source,
        }
    }

    pub fn send<SerializerCfg: SerializerOrientation>(&self, v: u32) -> Result<()>
    where
        RxPin: SerializerRx<SerializerCfg>,
        TxPin: SerializerTx<SerializerCfg>,
    {
        match SerializerCfg::tx_id() {
            Serializer::M0 => {
                if self.hw.intflag.read().txrdy0().bit_is_clear() {
                    return Err(I2SError::WouldBlock);
                }
                unsafe {
                    self.hw.data[0].write(|reg| reg.data().bits(v));
                }

                while self.hw.syncbusy.read().data0().bit_is_set() {}
            }

            Serializer::M1 => {
                if self.hw.intflag.read().txrdy1().bit_is_clear() {
                    return Err(I2SError::WouldBlock);
                }
                unsafe {
                    self.hw.data[1].write(|reg| reg.data().bits(v));
                }

                while self.hw.syncbusy.read().data1().bit_is_set() {}
            }
        }

        Ok(())
    }

    /// Gives the peripheral and pins back
    pub fn free(
        self,
    ) -> (
        pac::I2S,
        SerialClockPin,
        FrameSyncPin,
        RxPin,
        TxPin,
        MasterClockSource,
    ) {
        (
            self.hw,
            self.serial_clock_pin,
            self.frame_sync_pin,
            self.data_in_pin,
            self.data_out_pin,
            self.master_clock_source,
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

    /// Intended as a DMA destination; if writing single values use send()
    // TODO Figure out how to make these DMA functions const
    pub fn transmit_dma_ptr<SerializerCfg: SerializerOrientation>(&self) -> *mut u32
    where
        RxPin: SerializerRx<SerializerCfg>,
        TxPin: SerializerTx<SerializerCfg>,
    {
        &self.hw.data[SerializerCfg::tx_id() as usize] as *const _ as *mut u32
    }

    pub fn transmit_dma_trigger<SerializerCfg: SerializerOrientation>(&self) -> DmaTriggerSource
    where
        RxPin: SerializerRx<SerializerCfg>,
        TxPin: SerializerTx<SerializerCfg>,
    {
        match SerializerCfg::tx_id() {
            Serializer::M0 => DmaTriggerSource::I2S_TX_0,
            Serializer::M1 => DmaTriggerSource::I2S_TX_1,
        }
    }

    /// Intended as a DMA source; if writing single values use receive()
    pub fn receive_dma_ptr<SerializerCfg: SerializerOrientation>(&self) -> *mut u32
    where
        RxPin: SerializerRx<SerializerCfg>,
        TxPin: SerializerTx<SerializerCfg>,
    {
        &self.hw.data[SerializerCfg::rx_id() as usize] as *const _ as *mut u32
    }

    pub fn receive_dma_trigger<SerializerCfg: SerializerOrientation>(&self) -> DmaTriggerSource
    where
        RxPin: SerializerRx<SerializerCfg>,
        TxPin: SerializerTx<SerializerCfg>,
    {
        match SerializerCfg::rx_id() {
            Serializer::M0 => DmaTriggerSource::I2S_RX_0,
            Serializer::M1 => DmaTriggerSource::I2S_RX_1,
        }
    }
}
