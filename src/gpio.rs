//! General-purpose I/O.
//!
//! Each pin is a distinct zero-sized type, `Pin<PORT, NUMBER, MODE>`, so the mode a
//! pin is in is checked at compile time and a pin can only be owned by one place.
//!
//! ```ignore
//! let p1 = dp.port1.split();
//! let mut led = p1.p102.into_push_pull_output();
//! let button = p1.p105.into_pull_up_input();
//! ```
//!
//! # How this talks to the hardware
//!
//! Mode changes go through the pin's `PmnPFS` register, which holds direction,
//! pull-up, open-drain, analog and peripheral-mux selection in one 32-bit word. That
//! makes a mode change a single write with no read-modify-write, so configuring one
//! pin can never disturb another. `PmnPFS` is write-protected, so each change is
//! bracketed by a `PWPR` unlock.
//!
//! Level changes go through `PCNTR3`, whose low half sets pins and whose high half
//! clears them. Writing a one-hot mask there is inherently atomic with respect to the
//! other fifteen pins on the port, so `set_high` needs no critical section either.
//!
//! Note that the PAC's byte-addressable port register aliases (`podr()`, `pdr()`,
//! `pidr()`, `posr()`, `porr()`) are generated from an SVD that has their offsets
//! wrong — each names its neighbour's register. This module uses only the 32-bit
//! `PCNTR1`/`PCNTR2`/`PCNTR3` views, whose field definitions are correct.
//!
//! # Pins that are not bonded out
//!
//! [`split`](GpioExt::split) hands back all sixteen pins of every port, because the
//! port registers are sixteen bits wide regardless of the package. The 64-pin
//! R7FA4M1AB on the Uno R4 bonds out only a subset; configuring one of the others is
//! harmless (the write goes to an unimplemented PFS register and nothing happens),
//! but it will not do anything either. The package pin list in the datasheet is the
//! authority on which pins exist.

use core::marker::PhantomData;

use embedded_hal::digital::{ErrorType, InputPin, OutputPin, StatefulOutputPin};

pub use embedded_hal::digital::PinState;

use crate::pac;

/// I/O port base address. Ports are 0x20 bytes apart.
const PORT_BASE: usize = 0x4004_0000;
/// `PmnPFS` base address. Ports are 0x40 bytes apart, pins 4 bytes.
const PFS_BASE: usize = 0x4004_0800;

// `PmnPFS` bit positions.
const PFS_PODR: u32 = 1 << 0;
const PFS_PDR: u32 = 1 << 2;
const PFS_PCR: u32 = 1 << 4;
const PFS_NCODR: u32 = 1 << 6;
const PFS_DSCR: u32 = 1 << 10;
const PFS_ISEL: u32 = 1 << 14;
const PFS_ASEL: u32 = 1 << 15;
const PFS_PMR: u32 = 1 << 16;
const PFS_PSEL_SHIFT: u32 = 24;

/// A peripheral function that can be muxed onto a pin (`PmnPFS.PSEL`).
///
/// These are *mux groups*, not peripheral instances. A pin reaches at most one
/// function per group, and which instance that is depends on the pin: the
/// "Multi-Function Pin Controller" table in the RA4M1 hardware manual is the
/// authority. [`crate::board`] records the right group for every bus on the Uno R4
/// headers, so prefer the constants there over guessing.
///
/// In particular `SciGroup1` and `SciGroup2` are **not** SCI channels 1 and 2. They
/// are the two alternate SCI mux groups; on most pins group 1 carries the
/// even-numbered channels (SCI0, SCI2, ...) and group 2 the odd ones, but there are
/// per-pin exceptions.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum AltFunction {
    /// AGT timer I/O.
    Agt = 0b00001,
    /// GPT timer I/O, mux group 1. FSP calls this `IOPORT_PERIPHERAL_GPT0`.
    GptGroup1 = 0b00010,
    /// GPT timer I/O, mux group 2. FSP calls this `IOPORT_PERIPHERAL_GPT1`.
    ///
    /// This is the group the Uno R4's GPT header pins use.
    GptGroup2 = 0b00011,
    /// SCI mux group 1: `RXDn`, `TXDn`, `SCKn`, `CTSn`.
    ///
    /// FSP calls this `IOPORT_PERIPHERAL_SCI0_2_4_6_8`, after the channels it
    /// usually carries. Not a channel number; see the type-level docs.
    SciGroup1 = 0b00100,
    /// SCI mux group 2. FSP calls this `IOPORT_PERIPHERAL_SCI1_3_5_7_9`.
    SciGroup2 = 0b00101,
    /// SPI: `RSPCKn`, `MOSIn`, `MISOn`, `SSLn`.
    Spi = 0b00110,
    /// IIC: `SCLn`, `SDAn`.
    Iic = 0b00111,
    /// Key interrupt input.
    Kint = 0b01000,
    /// CLKOUT, comparator, RTC output.
    ClkOut = 0b01001,
    /// CAC, ADC trigger.
    CacAdc = 0b01010,
    /// External bus.
    Bus = 0b01011,
    /// Capacitive touch sensing unit.
    Ctsu = 0b01100,
    /// Segment LCD controller.
    Lcdc = 0b01101,
    /// CAN.
    Can = 0b10000,
    /// Serial sound interface.
    Ssi = 0b10010,
    /// USB full-speed.
    UsbFs = 0b10011,
    /// Trace output.
    Trace = 0b11010,
}

/// Output drive strength (`PmnPFS.DSCR`).
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum DriveStrength {
    /// Normal drive.
    #[default]
    Normal,
    /// High drive. Only supported on some pins.
    High,
}

// --- Type states -----------------------------------------------------------------

/// Input mode (type state).
#[derive(Debug)]
pub struct Input<MODE> {
    _mode: PhantomData<MODE>,
}
/// No pull resistor (type state).
#[derive(Debug)]
pub struct Floating;
/// Internal pull-up enabled (type state).
#[derive(Debug)]
pub struct PullUp;

/// Output mode (type state).
#[derive(Debug)]
pub struct Output<MODE> {
    _mode: PhantomData<MODE>,
}
/// Push-pull output (type state).
#[derive(Debug)]
pub struct PushPull;
/// N-channel open-drain output (type state).
#[derive(Debug)]
pub struct OpenDrain;

/// Analog input, digital buffer disconnected (type state).
#[derive(Debug)]
pub struct Analog;

/// Driven by a peripheral rather than the port (type state).
#[derive(Debug)]
pub struct Alternate;

// --- Register helpers ------------------------------------------------------------

/// The port register block for port `p`.
///
/// `port0::RegisterBlock` and `port1::RegisterBlock` differ only in the event-link
/// registers, which this module never touches; `PCNTR1`/`PCNTR2`/`PCNTR3` sit at the
/// same offsets in both. Using the `port0` view for every port is therefore sound and
/// lets one generic implementation cover all ten.
#[inline(always)]
fn port(p: u8) -> &'static pac::port0::RegisterBlock {
    // SAFETY: `PORT_BASE + 0x20 * p` is the documented base of port `p`. Every caller
    // reaches here through a `Pin`, `PartiallyErasedPin` or `ErasedPin`, and the only
    // way to obtain one is `GpioExt::split`, which is implemented for ports 0..=9.
    unsafe { &*((PORT_BASE + 0x20 * p as usize) as *const pac::port0::RegisterBlock) }
}

/// The `PmnPFS` register for port `p`, pin `n`.
#[inline(always)]
fn pfs(p: u8, n: u8) -> &'static pac::pfs::P000pfs {
    // SAFETY: the PFS block is a flat array of 32-bit registers, 16 per port, and
    // `pac::pfs::P000pfs` is a `#[repr(transparent)]` wrapper around a volatile u32.
    // Every PmnPFS register has the same layout, so the P000 spec describes them all.
    unsafe {
        &*((PFS_BASE + 0x40 * p as usize + 4 * n as usize) as *const pac::pfs::P000pfs)
    }
}

/// Run `f` with `PmnPFS` writes unlocked.
///
/// `PWPR.PFSWE` is only writable while `PWPR.B0WI` is clear, so unlocking takes two
/// writes. Runs in a critical section: `PWPR` is a single global register, and an
/// interrupt that reconfigured another pin in the middle would leave it locked.
#[inline]
fn with_pfs_unlocked<R>(f: impl FnOnce() -> R) -> R {
    critical_section::with(|_| {
        // SAFETY: `Pmisc` is a zero-sized token and `PWPR` is restored before we
        // return, so no other holder can observe it changed.
        let pmisc = unsafe { pac::Pmisc::steal() };
        pmisc.pwpr().write(|w| unsafe { w.bits(0x00) }); // B0WI = 0
        pmisc.pwpr().write(|w| unsafe { w.bits(0x40) }); // PFSWE = 1
        let r = f();
        pmisc.pwpr().write(|w| unsafe { w.bits(0x00) }); // PFSWE = 0
        pmisc.pwpr().write(|w| unsafe { w.bits(0x80) }); // B0WI = 1
        r
    })
}

/// Write a complete `PmnPFS` value.
///
/// Clears `PMR` first: the manual forbids changing `PSEL` while the pin is still
/// handed to a peripheral, so a mux change is always mux-out then mux-in.
fn write_pfs(p: u8, n: u8, value: u32) {
    let reg = pfs(p, n);
    with_pfs_unlocked(|| {
        reg.write(|w| unsafe { w.bits(value & !PFS_PMR) });
        if value & PFS_PMR != 0 {
            reg.write(|w| unsafe { w.bits(value) });
        }
    });
}

/// Read back a `PmnPFS` value.
fn read_pfs(p: u8, n: u8) -> u32 {
    pfs(p, n).read().bits()
}

// --- The pin ---------------------------------------------------------------------

/// A single I/O pin.
///
/// `P` is the port index (0..=9) and `N` the pin number within the port (0..=15).
/// Both are const generics, so a pin carries no runtime data at all.
#[derive(Debug)]
pub struct Pin<const P: u8, const N: u8, MODE> {
    _mode: PhantomData<MODE>,
}

impl<const P: u8, const N: u8, MODE> Pin<P, N, MODE> {
    /// The port index.
    pub const PORT: u8 = P;
    /// The pin number within the port.
    pub const NUMBER: u8 = N;

    #[inline(always)]
    const fn new() -> Self {
        Self { _mode: PhantomData }
    }

    #[inline(always)]
    const fn mask(&self) -> u32 {
        1 << N
    }

    /// Reconfigure the pin, consuming it and handing back the new mode.
    #[inline]
    fn convert<NEW>(self, pfs_value: u32) -> Pin<P, N, NEW> {
        write_pfs(P, N, pfs_value);
        Pin::new()
    }

    /// High-impedance input with no pull resistor.
    pub fn into_floating_input(self) -> Pin<P, N, Input<Floating>> {
        self.convert(0)
    }

    /// Input with the internal pull-up enabled.
    pub fn into_pull_up_input(self) -> Pin<P, N, Input<PullUp>> {
        self.convert(PFS_PCR)
    }

    /// Push-pull output, initially low.
    pub fn into_push_pull_output(self) -> Pin<P, N, Output<PushPull>> {
        self.into_push_pull_output_in_state(PinState::Low)
    }

    /// Push-pull output, driven to `state` before the direction is switched.
    ///
    /// Use this instead of [`into_push_pull_output`](Self::into_push_pull_output)
    /// when a glitch to the wrong level would matter, such as a chip-select or a
    /// reset line: the level is programmed in the same write that makes the pin an
    /// output, so the pin never briefly drives low.
    pub fn into_push_pull_output_in_state(
        self,
        state: PinState,
    ) -> Pin<P, N, Output<PushPull>> {
        let level = if state == PinState::High { PFS_PODR } else { 0 };
        self.convert(PFS_PDR | level)
    }

    /// N-channel open-drain output, initially released (high-impedance).
    pub fn into_open_drain_output(self) -> Pin<P, N, Output<OpenDrain>> {
        self.into_open_drain_output_in_state(PinState::High)
    }

    /// N-channel open-drain output, driven to `state` as the direction is switched.
    pub fn into_open_drain_output_in_state(
        self,
        state: PinState,
    ) -> Pin<P, N, Output<OpenDrain>> {
        let level = if state == PinState::High { PFS_PODR } else { 0 };
        self.convert(PFS_PDR | PFS_NCODR | level)
    }

    /// Analog input. Disconnects the digital input buffer, which is required before
    /// the pin is used as an ADC or DAC channel.
    pub fn into_analog(self) -> Pin<P, N, Analog> {
        self.convert(PFS_ASEL)
    }

    /// Hand the pin over to a peripheral.
    ///
    /// See [`AltFunction`] for the encoding, and the hardware manual for which
    /// functions this particular pin supports.
    pub fn into_alternate(self, function: AltFunction) -> Pin<P, N, Alternate> {
        self.convert(PFS_PMR | ((function as u32) << PFS_PSEL_SHIFT))
    }

    /// Hand the pin to a peripheral with the output stage in open-drain mode.
    ///
    /// What I2C needs: [`AltFunction::Iic`] plus `NCODR`, so SCL and SDA are only
    /// ever driven low and released.
    pub fn into_alternate_open_drain(self, function: AltFunction) -> Pin<P, N, Alternate> {
        self.convert(PFS_PMR | PFS_NCODR | ((function as u32) << PFS_PSEL_SHIFT))
    }

    /// Hand the pin to a peripheral, additionally enabling the analog buffer.
    ///
    /// Needed for the analog-ish peripheral pins (comparator, CTSU).
    pub fn into_alternate_analog(self, function: AltFunction) -> Pin<P, N, Alternate> {
        self.convert(PFS_PMR | PFS_ASEL | ((function as u32) << PFS_PSEL_SHIFT))
    }

    /// Set the output drive strength.
    ///
    /// Only meaningful on pins that support high drive; the bit is ignored elsewhere.
    pub fn set_drive_strength(&mut self, strength: DriveStrength) {
        let mut v = read_pfs(P, N);
        match strength {
            DriveStrength::Normal => v &= !PFS_DSCR,
            DriveStrength::High => v |= PFS_DSCR,
        }
        write_pfs(P, N, v);
    }

    /// Route this pin to the IRQ input of the ICU (`PmnPFS.ISEL`).
    ///
    /// The HAL does not manage the interrupt controller for you; this only connects
    /// the pin to the IRQn input so that you can configure the ICU through the PAC.
    pub fn set_irq_enabled(&mut self, enabled: bool) {
        let mut v = read_pfs(P, N);
        if enabled {
            v |= PFS_ISEL;
        } else {
            v &= !PFS_ISEL;
        }
        write_pfs(P, N, v);
    }

    /// Erase the pin number, keeping the port in the type.
    pub fn erase_number(self) -> PartiallyErasedPin<P, MODE> {
        PartiallyErasedPin {
            n: N,
            _mode: PhantomData,
        }
    }

    /// Erase both port and pin number, for storing pins in an array.
    pub fn erase(self) -> ErasedPin<MODE> {
        ErasedPin {
            p: P,
            n: N,
            _mode: PhantomData,
        }
    }
}

// --- Level access ----------------------------------------------------------------
//
// Factored out so `Pin`, `PartiallyErasedPin` and `ErasedPin` share one implementation
// and cannot drift apart.

#[inline(always)]
fn set_level(p: u8, mask: u32, high: bool) {
    // PCNTR3: POSR in bits 0..15 (write 1 to drive high), PORR in bits 16..31
    // (write 1 to drive low). Zeroes have no effect, so this one-hot write cannot
    // disturb the other pins on the port and needs no critical section.
    let bits = if high { mask } else { mask << 16 };
    port(p).pcntr3().write(|w| unsafe { w.bits(bits) });
}

#[inline(always)]
fn is_set_high(p: u8, mask: u32) -> bool {
    // PCNTR1: PDR in bits 0..15, PODR in bits 16..31.
    port(p).pcntr1().read().bits() & (mask << 16) != 0
}

#[inline(always)]
fn is_high(p: u8, mask: u32) -> bool {
    // PCNTR2: PIDR in bits 0..15.
    port(p).pcntr2().read().bits() & mask != 0
}

#[inline(always)]
fn toggle(p: u8, mask: u32) {
    set_level(p, mask, !is_set_high(p, mask));
}

// --- embedded-hal impls ----------------------------------------------------------

impl<const P: u8, const N: u8, MODE> ErrorType for Pin<P, N, MODE> {
    type Error = core::convert::Infallible;
}

impl<const P: u8, const N: u8, MODE> OutputPin for Pin<P, N, Output<MODE>> {
    fn set_high(&mut self) -> Result<(), Self::Error> {
        set_level(P, self.mask(), true);
        Ok(())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        set_level(P, self.mask(), false);
        Ok(())
    }
}

impl<const P: u8, const N: u8, MODE> StatefulOutputPin for Pin<P, N, Output<MODE>> {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(is_set_high(P, self.mask()))
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!is_set_high(P, self.mask()))
    }

    fn toggle(&mut self) -> Result<(), Self::Error> {
        toggle(P, self.mask());
        Ok(())
    }
}

impl<const P: u8, const N: u8, MODE> InputPin for Pin<P, N, Input<MODE>> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(is_high(P, self.mask()))
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!is_high(P, self.mask()))
    }
}

// An open-drain output can be read back: the input buffer stays connected, so this
// is how you do a bidirectional single-wire bus.
impl<const P: u8, const N: u8> InputPin for Pin<P, N, Output<OpenDrain>> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(is_high(P, self.mask()))
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!is_high(P, self.mask()))
    }
}

// --- Erased pins -----------------------------------------------------------------

/// A pin whose number is a runtime value but whose port is still in the type.
#[derive(Debug)]
pub struct PartiallyErasedPin<const P: u8, MODE> {
    n: u8,
    _mode: PhantomData<MODE>,
}

impl<const P: u8, MODE> PartiallyErasedPin<P, MODE> {
    #[inline(always)]
    const fn mask(&self) -> u32 {
        1 << self.n
    }

    /// Erase the port as well.
    pub fn erase(self) -> ErasedPin<MODE> {
        ErasedPin {
            p: P,
            n: self.n,
            _mode: PhantomData,
        }
    }
}

impl<const P: u8, MODE> ErrorType for PartiallyErasedPin<P, MODE> {
    type Error = core::convert::Infallible;
}

impl<const P: u8, MODE> OutputPin for PartiallyErasedPin<P, Output<MODE>> {
    fn set_high(&mut self) -> Result<(), Self::Error> {
        set_level(P, self.mask(), true);
        Ok(())
    }
    fn set_low(&mut self) -> Result<(), Self::Error> {
        set_level(P, self.mask(), false);
        Ok(())
    }
}

impl<const P: u8, MODE> StatefulOutputPin for PartiallyErasedPin<P, Output<MODE>> {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(is_set_high(P, self.mask()))
    }
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!is_set_high(P, self.mask()))
    }
    fn toggle(&mut self) -> Result<(), Self::Error> {
        toggle(P, self.mask());
        Ok(())
    }
}

impl<const P: u8, MODE> InputPin for PartiallyErasedPin<P, Input<MODE>> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(is_high(P, self.mask()))
    }
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!is_high(P, self.mask()))
    }
}

/// A pin with both port and number erased, so pins from different ports can share an
/// array or a `[&mut dyn OutputPin]`.
#[derive(Debug)]
pub struct ErasedPin<MODE> {
    p: u8,
    n: u8,
    _mode: PhantomData<MODE>,
}

impl<MODE> ErasedPin<MODE> {
    #[inline(always)]
    const fn mask(&self) -> u32 {
        1 << self.n
    }

    /// The port index this pin came from.
    pub const fn port(&self) -> u8 {
        self.p
    }

    /// The pin number this pin came from.
    pub const fn number(&self) -> u8 {
        self.n
    }
}

impl<MODE> ErrorType for ErasedPin<MODE> {
    type Error = core::convert::Infallible;
}

impl<MODE> OutputPin for ErasedPin<Output<MODE>> {
    fn set_high(&mut self) -> Result<(), Self::Error> {
        set_level(self.p, self.mask(), true);
        Ok(())
    }
    fn set_low(&mut self) -> Result<(), Self::Error> {
        set_level(self.p, self.mask(), false);
        Ok(())
    }
}

impl<MODE> StatefulOutputPin for ErasedPin<Output<MODE>> {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(is_set_high(self.p, self.mask()))
    }
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!is_set_high(self.p, self.mask()))
    }
    fn toggle(&mut self) -> Result<(), Self::Error> {
        toggle(self.p, self.mask());
        Ok(())
    }
}

impl<MODE> InputPin for ErasedPin<Input<MODE>> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(is_high(self.p, self.mask()))
    }
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!is_high(self.p, self.mask()))
    }
}

// --- Splitting a port ------------------------------------------------------------

/// Consume a PAC port and split it into individual pins.
pub trait GpioExt {
    /// The struct of pins this port splits into.
    type Parts;

    /// Split the port. Consuming the PAC peripheral means the returned pins are the
    /// only route to this port's registers.
    fn split(self) -> Self::Parts;
}

macro_rules! gpio {
    ($($PortTy:ident, $mod:ident, $p:literal, [$($pin:ident: $n:literal),+ $(,)?];)+) => {
        $(
            #[doc = concat!("Pins of port ", stringify!($p), ".")]
            pub mod $mod {
                use super::{GpioExt, Input, Floating, Pin};
                use crate::pac;

                /// The pins of this port, all in their reset state.
                #[derive(Debug)]
                pub struct Parts {
                    $(
                        #[doc = concat!("Pin P", stringify!($p), stringify!($n), ".")]
                        pub $pin: Pin<$p, $n, Input<Floating>>,
                    )+
                }

                impl GpioExt for pac::$PortTy {
                    type Parts = Parts;

                    fn split(self) -> Parts {
                        // I/O ports have no module-stop bit, so there is no clock to
                        // ungate here. `self` is dropped: the pins are now the only
                        // handle on this port.
                        Parts { $($pin: Pin::new(),)+ }
                    }
                }
            }
        )+

        $( pub use $mod::Parts as $PortTy; )+
    };
}

gpio! {
    Port0, port0, 0, [
        p000:  0, p001:  1, p002:  2, p003:  3, p004:  4, p005:  5, p006:  6, p007:  7,
        p008:  8, p009:  9, p010: 10, p011: 11, p012: 12, p013: 13, p014: 14, p015: 15,
    ];
    Port1, port1, 1, [
        p100:  0, p101:  1, p102:  2, p103:  3, p104:  4, p105:  5, p106:  6, p107:  7,
        p108:  8, p109:  9, p110: 10, p111: 11, p112: 12, p113: 13, p114: 14, p115: 15,
    ];
    Port2, port2, 2, [
        p200:  0, p201:  1, p202:  2, p203:  3, p204:  4, p205:  5, p206:  6, p207:  7,
        p208:  8, p209:  9, p210: 10, p211: 11, p212: 12, p213: 13, p214: 14, p215: 15,
    ];
    Port3, port3, 3, [
        p300:  0, p301:  1, p302:  2, p303:  3, p304:  4, p305:  5, p306:  6, p307:  7,
        p308:  8, p309:  9, p310: 10, p311: 11, p312: 12, p313: 13, p314: 14, p315: 15,
    ];
    Port4, port4, 4, [
        p400:  0, p401:  1, p402:  2, p403:  3, p404:  4, p405:  5, p406:  6, p407:  7,
        p408:  8, p409:  9, p410: 10, p411: 11, p412: 12, p413: 13, p414: 14, p415: 15,
    ];
    Port5, port5, 5, [
        p500:  0, p501:  1, p502:  2, p503:  3, p504:  4, p505:  5, p506:  6, p507:  7,
        p508:  8, p509:  9, p510: 10, p511: 11, p512: 12, p513: 13, p514: 14, p515: 15,
    ];
    Port6, port6, 6, [
        p600:  0, p601:  1, p602:  2, p603:  3, p604:  4, p605:  5, p606:  6, p607:  7,
        p608:  8, p609:  9, p610: 10, p611: 11, p612: 12, p613: 13, p614: 14, p615: 15,
    ];
    Port7, port7, 7, [
        p700:  0, p701:  1, p702:  2, p703:  3, p704:  4, p705:  5, p706:  6, p707:  7,
        p708:  8, p709:  9, p710: 10, p711: 11, p712: 12, p713: 13, p714: 14, p715: 15,
    ];
    Port8, port8, 8, [
        p800:  0, p801:  1, p802:  2, p803:  3, p804:  4, p805:  5, p806:  6, p807:  7,
        p808:  8, p809:  9, p810: 10, p811: 11, p812: 12, p813: 13, p814: 14, p815: 15,
    ];
    Port9, port9, 9, [
        p900:  0, p901:  1, p902:  2, p903:  3, p904:  4, p905:  5, p906:  6, p907:  7,
        p908:  8, p909:  9, p910: 10, p911: 11, p912: 12, p913: 13, p914: 14, p915: 15,
    ];
}
