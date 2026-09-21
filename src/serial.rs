//! Asynchronous serial (UART) on the SCI peripherals.
//!
//! ```ignore
//! let tx = p4.p401.into_alternate(AltFunction::Sci1);
//! let rx = p4.p402.into_alternate(AltFunction::Sci1);
//! let mut serial = Serial::new(dp.sci0, (tx, rx), Config::baud(115_200), &clocks);
//! writeln!(serial, "hello").unwrap();
//! ```
//!
//! Implements [`embedded_io::Read`]/[`Write`](embedded_io::Write),
//! [`embedded_hal_nb::serial`] and [`core::fmt::Write`]. Transfers are polled, not
//! interrupt driven.
//!
//! Which pins carry `TXDn`/`RXDn` for a given SCI is fixed in silicon; look it up in
//! the hardware manual's pin function table. The driver requires that the pins have
//! been switched to [`AltFunction::Sci1`](crate::gpio::AltFunction::Sci1) and takes
//! ownership of them, but it cannot check that you picked pins this SCI can reach.

use core::convert::Infallible;
use core::marker::PhantomData;

use crate::clock::Clocks;
use crate::gpio::{Alternate, Pin};
use crate::mstp::{self, ModuleStop};
use crate::pac;

/// Number of data bits per character.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum DataBits {
    /// 7 data bits.
    Seven,
    /// 8 data bits.
    #[default]
    Eight,
}

/// Parity checking and generation.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum Parity {
    /// No parity bit.
    #[default]
    None,
    /// Even parity.
    Even,
    /// Odd parity.
    Odd,
}

/// Number of stop bits.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum StopBits {
    /// One stop bit.
    #[default]
    One,
    /// Two stop bits.
    Two,
}

/// UART configuration.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Config {
    /// Bit rate in bits per second.
    pub baudrate: u32,
    /// Data bits per character.
    pub data_bits: DataBits,
    /// Parity.
    pub parity: Parity,
    /// Stop bits.
    pub stop_bits: StopBits,
    /// Enable the digital noise filter on RXD.
    pub noise_filter: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self::baud(115_200)
    }
}

impl Config {
    /// 8N1 at `baudrate`.
    pub const fn baud(baudrate: u32) -> Self {
        Self {
            baudrate,
            data_bits: DataBits::Eight,
            parity: Parity::None,
            stop_bits: StopBits::One,
            noise_filter: false,
        }
    }

    /// Set the number of data bits.
    pub const fn data_bits(mut self, data_bits: DataBits) -> Self {
        self.data_bits = data_bits;
        self
    }

    /// Set the parity.
    pub const fn parity(mut self, parity: Parity) -> Self {
        self.parity = parity;
        self
    }

    /// Set the number of stop bits.
    pub const fn stop_bits(mut self, stop_bits: StopBits) -> Self {
        self.stop_bits = stop_bits;
        self
    }

    /// Enable the digital noise filter on RXD.
    pub const fn noise_filter(mut self, enabled: bool) -> Self {
        self.noise_filter = enabled;
        self
    }
}

/// A receive error reported through `SSR`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Error {
    /// A character arrived before the previous one was read.
    Overrun,
    /// The stop bit was not where it should have been.
    Framing,
    /// The parity bit did not match.
    Parity,
    /// The requested baud rate cannot be produced from the current PCLKB.
    UnsupportedBaudRate,
}

impl embedded_hal_nb::serial::Error for Error {
    fn kind(&self) -> embedded_hal_nb::serial::ErrorKind {
        use embedded_hal_nb::serial::ErrorKind;
        match self {
            Error::Overrun => ErrorKind::Overrun,
            Error::Framing => ErrorKind::FrameFormat,
            Error::Parity => ErrorKind::Parity,
            Error::UnsupportedBaudRate => ErrorKind::Other,
        }
    }
}

impl embedded_io::Error for Error {
    fn kind(&self) -> embedded_io::ErrorKind {
        match self {
            Error::UnsupportedBaudRate => embedded_io::ErrorKind::InvalidInput,
            _ => embedded_io::ErrorKind::Other,
        }
    }
}

// --- Instances -------------------------------------------------------------------

/// An SCI channel usable as a UART.
///
/// # Safety
///
/// `ptr` must return the base address of a real SCI register block, and no two
/// implementations may return the same address.
pub unsafe trait Instance {
    /// This channel's module-stop bit.
    const MODULE: ModuleStop;

    /// Base address of the channel's registers.
    ///
    /// `sci2::RegisterBlock` is a subset of `sci0::RegisterBlock` with identical
    /// offsets for every register this driver touches, so SCI2 and SCI9 are viewed
    /// through the `sci0` layout too.
    fn ptr() -> *const pac::sci0::RegisterBlock;
}

macro_rules! sci_instance {
    ($($SCI:ident => $module:ident,)+) => {
        $(
            // SAFETY: each `pac::$SCI::PTR` is the distinct, documented base address
            // of that SCI channel.
            unsafe impl Instance for pac::$SCI {
                const MODULE: ModuleStop = ModuleStop::$module;
                #[inline(always)]
                fn ptr() -> *const pac::sci0::RegisterBlock {
                    pac::$SCI::PTR as *const pac::sci0::RegisterBlock
                }
            }
        )+
    };
}

sci_instance! {
    Sci0 => SCI0,
    Sci1 => SCI1,
    Sci2 => SCI2,
    Sci9 => SCI9,
}

/// Pins that can be handed to a [`Serial`].
///
/// Implemented for a `(tx, rx)` tuple, for a lone TX or RX pin, and for `()` when
/// the pins are managed elsewhere. All pins must already be in
/// [`crate::gpio::Alternate`] mode.
pub trait Pins<SCI> {}

impl<SCI> Pins<SCI> for () {}
impl<SCI, const P: u8, const N: u8> Pins<SCI> for Pin<P, N, Alternate> {}
impl<SCI, const P1: u8, const N1: u8, const P2: u8, const N2: u8> Pins<SCI>
    for (Pin<P1, N1, Alternate>, Pin<P2, N2, Alternate>)
{
}

// --- Baud rate -------------------------------------------------------------------

/// Register settings that produce a bit rate.
///
/// The rate is `pclkb / (divisor * 4^cks * (brr + 1))`, optionally scaled by
/// `mddr / 256`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
struct BaudSetting {
    /// `SMR.CKS`: prescales PCLKB by 4^cks.
    cks: u8,
    /// `SEMR.BGDM`.
    bgdm: bool,
    /// `SEMR.ABCS`: 8 base clock cycles per bit instead of 16.
    abcs: bool,
    /// `BRR`.
    brr: u8,
    /// `MDDR`, when bit-rate modulation is needed. `SEMR.BRME` follows this.
    mddr: Option<u8>,
}

/// The `(BGDM, ABCS, divisor)` combinations. `divisor` is 32 with both bits clear
/// and halves for each one that is set.
const BAUD_DIVISORS: [(bool, bool, u32); 4] = [
    (false, false, 32),
    (true, false, 16),
    (false, true, 16),
    (true, true, 8),
];

/// Find the register combination whose bit rate is closest to `baud`.
///
/// Two families of solution are considered for each prescaler. The plain one solves
/// `brr = pclkb / (divisor * 4^cks * baud) - 1` and rounds to nearest. The other uses
/// the bit-rate modulator: it picks a `brr` that lands *above* the target, then
/// scales the result down by `MDDR / 256`, which buys roughly eight extra bits of
/// resolution. Modulation is what makes rates like 460800 from a 24 MHz PCLKB
/// possible at all, so it is used whenever it is strictly closer.
fn calc_baud(pclkb: u32, baud: u32) -> Option<BaudSetting> {
    if baud == 0 {
        return None;
    }

    let mut best: Option<(BaudSetting, u32)> = None;
    let mut consider = |setting: BaudSetting, actual: u32| {
        let error = actual.abs_diff(baud);
        match best {
            // `<=` keeps the first, least exotic setting on a tie: the plain
            // solutions are offered before the modulated ones.
            Some((_, best_err)) if best_err <= error => {}
            _ => best = Some((setting, error)),
        }
    };

    for (bgdm, abcs, divisor) in BAUD_DIVISORS {
        for cks in 0u8..4 {
            // `unit` is everything the bit rate is divided by except `BRR + 1`.
            let unit = divisor << (2 * cks as u32);
            let Some(den) = unit.checked_mul(baud) else {
                continue;
            };

            // Plain: round `brr` to nearest.
            let n = (pclkb + den / 2) / den;
            if (1..=256).contains(&n) {
                consider(
                    BaudSetting {
                        cks,
                        bgdm,
                        abcs,
                        brr: (n - 1) as u8,
                        mddr: None,
                    },
                    pclkb / (unit * n),
                );
            }

            // Modulated: round `brr` down, so the unmodulated rate is at or above
            // the target, then bring it back down with MDDR.
            let n = pclkb / den;
            if (1..=256).contains(&n) {
                let base = u64::from(pclkb) / u64::from(unit * n);
                // mddr = round(256 * baud / base)
                if let Some(mddr) = (256 * u64::from(baud) + base / 2).checked_div(base) {
                    // 0..=127 is prohibited, and 256 is just the unmodulated case,
                    // which was already offered above.
                    if (128..=255).contains(&mddr) {
                        let actual = (u64::from(pclkb) * mddr) / (u64::from(unit * n) * 256);
                        consider(
                            BaudSetting {
                                cks,
                                bgdm,
                                abcs,
                                brr: (n - 1) as u8,
                                mddr: Some(mddr as u8),
                            },
                            actual as u32,
                        );
                    }
                }
            }
        }
    }

    best.map(|(setting, _)| setting)
}

// --- Driver ----------------------------------------------------------------------

/// A polled UART on one SCI channel.
#[derive(Debug)]
pub struct Serial<SCI, PINS> {
    sci: SCI,
    pins: PINS,
}

impl<SCI: Instance, PINS: Pins<SCI>> Serial<SCI, PINS> {
    /// Configure the channel for asynchronous serial and enable TX and RX.
    ///
    /// Returns [`Error::UnsupportedBaudRate`] if the requested rate cannot be reached
    /// from the current PCLKB.
    pub fn new(
        sci: SCI,
        pins: PINS,
        config: Config,
        clocks: &Clocks,
    ) -> Result<Self, Error> {
        let baud = calc_baud(clocks.pclkb().raw(), config.baudrate)
            .ok_or(Error::UnsupportedBaudRate)?;

        mstp::start(SCI::MODULE);

        let regs = unsafe { &*SCI::ptr() };

        // Stop the channel before touching anything else; SMR and BRR are only
        // writable while TE and RE are clear.
        regs.scr().write(|w| unsafe { w.bits(0) });
        // Plain UART: not simple-I2C, not smart-card, internal baud rate generator.
        regs.simr1().write(|w| w.iicm().clear_bit());
        regs.spmr().write(|w| unsafe { w.bits(0) });

        regs.smr().write(|w| {
            w.cm().clear_bit(); // asynchronous
            w.chr().bit(config.data_bits == DataBits::Seven);
            w.pe().bit(config.parity != Parity::None);
            w.pm().bit(config.parity == Parity::Odd);
            w.stop().bit(config.stop_bits == StopBits::Two);
            w.mp().clear_bit();
            unsafe { w.cks().bits(baud.cks) }
        });

        // SCMR.CHR1 = 1 together with SMR.CHR picks 8- or 7-bit characters; CHR1 = 0
        // would mean 9-bit, which this driver does not expose.
        regs.scmr().write(|w| {
            w.smif().clear_bit();
            w.sinv().clear_bit();
            w.sdir().clear_bit(); // LSB first
            w.chr1().set_bit();
            w.bcp2().set_bit() // reset value; only meaningful in smart-card mode
        });

        regs.semr().write(|w| {
            w.abcs().bit(baud.abcs);
            w.bgdm().bit(baud.bgdm);
            w.nfen().bit(config.noise_filter);
            w.abcse().clear_bit();
            w.brme().bit(baud.mddr.is_some());
            w.rxdesel().set_bit()
        });

        regs.brr().write(|w| unsafe { w.brr().bits(baud.brr) });
        if let Some(mddr) = baud.mddr {
            // MDDR is only consulted while SEMR.BRME is set, and the manual requires
            // it to be written after BRR.
            regs.mddr().write(|w| unsafe { w.mddr().bits(mddr) });
        }

        regs.scr().write(|w| {
            // CKE = 0: on-chip baud rate generator, SCK pin unused.
            unsafe { w.cke().bits(0) };
            w.te().set_bit();
            w.re().set_bit()
        });

        Ok(Self { sci, pins })
    }

    /// Split into independent halves so TX and RX can live in different places.
    pub fn split(self) -> (Tx<SCI>, Rx<SCI>) {
        (
            Tx {
                _sci: PhantomData,
            },
            Rx {
                _sci: PhantomData,
            },
        )
    }

    /// Stop the channel and give back the peripheral and pins.
    pub fn release(self) -> (SCI, PINS) {
        let regs = unsafe { &*SCI::ptr() };
        regs.scr().write(|w| unsafe { w.bits(0) });
        mstp::stop(SCI::MODULE);
        (self.sci, self.pins)
    }

    /// Borrow the transmit half.
    pub fn tx(&mut self) -> Tx<SCI> {
        Tx {
            _sci: PhantomData,
        }
    }

    /// Borrow the receive half.
    pub fn rx(&mut self) -> Rx<SCI> {
        Rx {
            _sci: PhantomData,
        }
    }
}

/// The transmit half of a [`Serial`].
#[derive(Debug)]
pub struct Tx<SCI> {
    _sci: PhantomData<SCI>,
}

/// The receive half of a [`Serial`].
#[derive(Debug)]
pub struct Rx<SCI> {
    _sci: PhantomData<SCI>,
}

impl<SCI: Instance> Tx<SCI> {
    #[inline(always)]
    fn regs(&self) -> &'static pac::sci0::RegisterBlock {
        // SAFETY: `Tx` is only constructed from a configured `Serial`, which owns the
        // channel. The transmit and receive halves touch disjoint registers apart
        // from SSR, whose status bits are write-1-to-clear per direction.
        unsafe { &*SCI::ptr() }
    }

    /// Is the transmit data register empty?
    pub fn is_tx_empty(&self) -> bool {
        self.regs().ssr().read().tdre().bit_is_set()
    }

    /// Has the last character been shifted all the way out?
    ///
    /// Check this before powering the channel down or releasing the TXD pin.
    pub fn is_tx_complete(&self) -> bool {
        self.regs().ssr().read().tend().bit_is_set()
    }

    /// Queue one byte if there is room.
    pub fn write_byte(&mut self, byte: u8) -> nb::Result<(), Infallible> {
        if !self.is_tx_empty() {
            return Err(nb::Error::WouldBlock);
        }
        self.regs().tdr().write(|w| unsafe { w.tdr().bits(byte) });
        Ok(())
    }

    /// Block until everything queued has left the shift register.
    pub fn flush(&mut self) -> nb::Result<(), Infallible> {
        if self.is_tx_complete() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl<SCI: Instance> Rx<SCI> {
    #[inline(always)]
    fn regs(&self) -> &'static pac::sci0::RegisterBlock {
        // SAFETY: see `Tx::regs`.
        unsafe { &*SCI::ptr() }
    }

    /// Is there a received byte waiting?
    pub fn is_rx_ready(&self) -> bool {
        self.regs().ssr().read().rdrf().bit_is_set()
    }

    /// Take one byte if one has arrived.
    ///
    /// Errors are reported before the byte, and clear the corresponding flag. An
    /// overrun does not consume `RDR`, so the next call still returns the byte that
    /// was waiting.
    pub fn read_byte(&mut self) -> nb::Result<u8, Error> {
        let ssr = self.regs().ssr().read();

        if ssr.orer().bit_is_set() {
            self.clear_flag(|w| w.orer().clear_bit());
            return Err(nb::Error::Other(Error::Overrun));
        }
        if ssr.fer().bit_is_set() {
            self.clear_flag(|w| w.fer().clear_bit());
            return Err(nb::Error::Other(Error::Framing));
        }
        if ssr.per().bit_is_set() {
            self.clear_flag(|w| w.per().clear_bit());
            return Err(nb::Error::Other(Error::Parity));
        }
        if !ssr.rdrf().bit_is_set() {
            return Err(nb::Error::WouldBlock);
        }
        // Reading RDR clears RDRF.
        Ok(self.regs().rdr().read().rdr().bits())
    }

    /// Clear one SSR status flag.
    ///
    /// `SSR` error flags are cleared by writing 0 after reading a 1, so this has to
    /// be a read-modify-write rather than a plain write.
    fn clear_flag(&mut self, f: impl FnOnce(&mut pac::sci0::ssr::W) -> &mut pac::sci0::ssr::W) {
        self.regs().ssr().modify(|_, w| f(w));
    }
}

// --- embedded-hal-nb -------------------------------------------------------------

impl<SCI> embedded_hal_nb::serial::ErrorType for Tx<SCI> {
    type Error = Error;
}

impl<SCI: Instance> embedded_hal_nb::serial::Write<u8> for Tx<SCI> {
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        self.write_byte(word).map_err(|e| match e {
            nb::Error::WouldBlock => nb::Error::WouldBlock,
            nb::Error::Other(e) => match e {},
        })
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        Tx::flush(self).map_err(|e| match e {
            nb::Error::WouldBlock => nb::Error::WouldBlock,
            nb::Error::Other(e) => match e {},
        })
    }
}

impl<SCI> embedded_hal_nb::serial::ErrorType for Rx<SCI> {
    type Error = Error;
}

impl<SCI: Instance> embedded_hal_nb::serial::Read<u8> for Rx<SCI> {
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.read_byte()
    }
}

impl<SCI, PINS> embedded_hal_nb::serial::ErrorType for Serial<SCI, PINS> {
    type Error = Error;
}

impl<SCI: Instance, PINS> embedded_hal_nb::serial::Write<u8> for Serial<SCI, PINS> {
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        embedded_hal_nb::serial::Write::write(&mut self.tx_half(), word)
    }
    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        embedded_hal_nb::serial::Write::flush(&mut self.tx_half())
    }
}

impl<SCI: Instance, PINS> embedded_hal_nb::serial::Read<u8> for Serial<SCI, PINS> {
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        self.rx_half().read_byte()
    }
}

impl<SCI: Instance, PINS> Serial<SCI, PINS> {
    #[inline(always)]
    fn tx_half(&mut self) -> Tx<SCI> {
        Tx {
            _sci: PhantomData,
        }
    }
    #[inline(always)]
    fn rx_half(&mut self) -> Rx<SCI> {
        Rx {
            _sci: PhantomData,
        }
    }
}

// --- embedded-io -----------------------------------------------------------------

impl<SCI> embedded_io::ErrorType for Tx<SCI> {
    type Error = Error;
}

impl<SCI: Instance> embedded_io::Write for Tx<SCI> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }
        // `embedded_io::Write` may return a short write, but must block until it has
        // written at least one byte.
        nb::block!(self.write_byte(buf[0])).ok();
        let mut written = 1;
        while written < buf.len() && self.is_tx_empty() {
            nb::block!(self.write_byte(buf[written])).ok();
            written += 1;
        }
        Ok(written)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        nb::block!(Tx::flush(self)).ok();
        Ok(())
    }
}

impl<SCI: Instance> embedded_io::WriteReady for Tx<SCI> {
    fn write_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(self.is_tx_empty())
    }
}

impl<SCI> embedded_io::ErrorType for Rx<SCI> {
    type Error = Error;
}

impl<SCI: Instance> embedded_io::Read for Rx<SCI> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }
        // Must block for at least one byte, then drain whatever else is ready.
        buf[0] = nb::block!(self.read_byte())?;
        let mut read = 1;
        while read < buf.len() && self.is_rx_ready() {
            match self.read_byte() {
                Ok(b) => {
                    buf[read] = b;
                    read += 1;
                }
                // Report the bytes we already have; the error will surface on the
                // next call, with its flag still set.
                Err(_) => break,
            }
        }
        Ok(read)
    }
}

impl<SCI: Instance> embedded_io::ReadReady for Rx<SCI> {
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(self.is_rx_ready())
    }
}

impl<SCI, PINS> embedded_io::ErrorType for Serial<SCI, PINS> {
    type Error = Error;
}

impl<SCI: Instance, PINS> embedded_io::Write for Serial<SCI, PINS> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        embedded_io::Write::write(&mut self.tx_half(), buf)
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        embedded_io::Write::flush(&mut self.tx_half())
    }
}

impl<SCI: Instance, PINS> embedded_io::WriteReady for Serial<SCI, PINS> {
    fn write_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(self.tx_half().is_tx_empty())
    }
}

impl<SCI: Instance, PINS> embedded_io::Read for Serial<SCI, PINS> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        embedded_io::Read::read(&mut self.rx_half(), buf)
    }
}

impl<SCI: Instance, PINS> embedded_io::ReadReady for Serial<SCI, PINS> {
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(self.rx_half().is_rx_ready())
    }
}

// --- core::fmt -------------------------------------------------------------------

impl<SCI: Instance> core::fmt::Write for Tx<SCI> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for byte in s.as_bytes() {
            nb::block!(self.write_byte(*byte)).map_err(|_| core::fmt::Error)?;
        }
        Ok(())
    }
}

impl<SCI: Instance, PINS> core::fmt::Write for Serial<SCI, PINS> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        core::fmt::Write::write_str(&mut self.tx_half(), s)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The rate the driver will actually produce for a given `BaudSetting`.
    fn actual(pclkb: u32, s: BaudSetting) -> u32 {
        let divisor: u64 = match (s.bgdm, s.abcs) {
            (false, false) => 32,
            (true, true) => 8,
            _ => 16,
        };
        let unit = divisor * (1 << (2 * s.cks as u32)) * (s.brr as u64 + 1);
        let base = u64::from(pclkb) / unit;
        match s.mddr {
            None => base as u32,
            Some(mddr) => ((u64::from(pclkb) * u64::from(mddr)) / (unit * 256)) as u32,
        }
    }

    #[test]
    fn common_rates_are_within_two_percent() {
        // PCLKB is 24 MHz in the stock Uno R4 clock configuration.
        for pclkb in [24_000_000u32, 48_000_000] {
            for baud in [9_600u32, 19_200, 38_400, 57_600, 115_200, 230_400, 460_800] {
                let s = calc_baud(pclkb, baud).expect("representable");
                let got = actual(pclkb, s);
                // Async serial tolerates about 2% total skew across a frame; every
                // rate here should land far inside that.
                let error = got.abs_diff(baud) * 1000 / baud;
                assert!(error <= 10, "{pclkb} -> {baud}: got {got} ({error}/1000 off)");
            }
        }
    }

    #[test]
    fn rejects_zero_and_absurd_rates() {
        assert!(calc_baud(24_000_000, 0).is_none());
        // Below PCLKB / (32 * 64 * 256) nothing fits in BRR.
        assert!(calc_baud(24_000_000, 10).is_none());
    }
}
