//! SPI master on the SPI (RSPI) peripherals.
//!
//! ```ignore
//! let sck  = p1.p102.into_alternate(AltFunction::Spi);
//! let mosi = p1.p101.into_alternate(AltFunction::Spi);
//! let miso = p1.p100.into_alternate(AltFunction::Spi);
//! let mut spi = Spi::new(dp.spi0, (sck, mosi, miso), Config::mode0(1_000_000), &clocks);
//! spi.transfer_in_place(&mut buf).unwrap();
//! ```
//!
//! Implements [`embedded_hal::spi::SpiBus`] over 8-bit frames. Chip select is left to
//! the caller: pair the bus with
//! `embedded_hal_bus::spi::ExclusiveDevice` and an ordinary output pin, or drive
//! `SSL` through the peripheral by muxing it yourself.
//!
//! Transfers are polled, one frame at a time. The peripheral has a one-frame buffer
//! in each direction, so a polled loop leaves one bit-time of idle between frames.

use embedded_hal::spi::{ErrorKind, ErrorType, Mode, SpiBus, MODE_0, MODE_1, MODE_2, MODE_3};

use crate::clock::Clocks;
use crate::gpio::{Alternate, Pin};
use crate::mstp::{self, ModuleStop};
use crate::pac;
use crate::spin_until;

/// Poll budget for one frame. At the slowest supported bit rate a frame is a few
/// thousand cycles, so this only trips if the peripheral has stopped responding.
const TIMEOUT: u32 = 1_000_000;

/// `SPCMD0.SPB` value for an 8-bit frame. Encodings 0b0100..0b0111 all mean 8 bits.
const SPB_8_BIT: u8 = 0b0100;

/// SPI bus errors.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Error {
    /// A frame arrived before the previous one was read.
    Overrun,
    /// Another master drove `SSL` while we were driving the bus.
    ModeFault,
    /// Parity error on the received frame.
    Parity,
    /// The peripheral did not complete a frame in time.
    Timeout,
    /// The requested bit rate cannot be produced from the current PCLKA.
    UnsupportedFrequency,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            Error::Overrun => "receive overrun",
            Error::ModeFault => "mode fault",
            Error::Parity => "parity error",
            Error::Timeout => "transfer timed out",
            Error::UnsupportedFrequency => "bit rate not reachable from the current PCLKA",
        })
    }
}

impl core::error::Error for Error {}

impl embedded_hal::spi::Error for Error {
    fn kind(&self) -> ErrorKind {
        match self {
            Error::Overrun => ErrorKind::Overrun,
            Error::ModeFault => ErrorKind::ModeFault,
            Error::Parity | Error::Timeout | Error::UnsupportedFrequency => ErrorKind::Other,
        }
    }
}

/// Bit order within a frame.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum BitOrder {
    /// Most significant bit first. What almost every SPI device expects.
    #[default]
    MsbFirst,
    /// Least significant bit first.
    LsbFirst,
}

/// Bus configuration.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Config {
    /// Clock polarity and phase.
    pub mode: Mode,
    /// Target SCK frequency in Hz. The achieved rate is the closest available that
    /// does not exceed this.
    pub frequency: u32,
    /// Bit order.
    pub bit_order: BitOrder,
}

impl Default for Config {
    fn default() -> Self {
        Self::mode0(1_000_000)
    }
}

impl Config {
    /// SPI mode 0 (CPOL = 0, CPHA = 0) at `frequency`.
    pub const fn mode0(frequency: u32) -> Self {
        Self {
            mode: MODE_0,
            frequency,
            bit_order: BitOrder::MsbFirst,
        }
    }
    /// SPI mode 1 (CPOL = 0, CPHA = 1) at `frequency`.
    pub const fn mode1(frequency: u32) -> Self {
        Self {
            mode: MODE_1,
            frequency,
            bit_order: BitOrder::MsbFirst,
        }
    }
    /// SPI mode 2 (CPOL = 1, CPHA = 0) at `frequency`.
    pub const fn mode2(frequency: u32) -> Self {
        Self {
            mode: MODE_2,
            frequency,
            bit_order: BitOrder::MsbFirst,
        }
    }
    /// SPI mode 3 (CPOL = 1, CPHA = 1) at `frequency`.
    pub const fn mode3(frequency: u32) -> Self {
        Self {
            mode: MODE_3,
            frequency,
            bit_order: BitOrder::MsbFirst,
        }
    }

    /// Send the least significant bit of each frame first.
    pub const fn lsb_first(mut self) -> Self {
        self.bit_order = BitOrder::LsbFirst;
        self
    }
}

// --- Instances -------------------------------------------------------------------

/// An SPI channel.
///
/// # Safety
///
/// `ptr` must return the base address of a real SPI register block, and no two
/// implementations may return the same address.
pub unsafe trait Instance {
    /// This channel's module-stop bit.
    const MODULE: ModuleStop;

    /// Base address of the channel's registers.
    fn ptr() -> *const pac::spi0::RegisterBlock;
}

// SAFETY: `pac::Spi0::PTR` and `pac::Spi1::PTR` are the distinct, documented base
// addresses of the two SPI channels, and `spi1::RegisterBlock` has the same layout
// as `spi0::RegisterBlock`.
unsafe impl Instance for pac::Spi0 {
    const MODULE: ModuleStop = ModuleStop::SPI0;
    #[inline(always)]
    fn ptr() -> *const pac::spi0::RegisterBlock {
        pac::Spi0::PTR
    }
}

unsafe impl Instance for pac::Spi1 {
    const MODULE: ModuleStop = ModuleStop::SPI1;
    #[inline(always)]
    fn ptr() -> *const pac::spi0::RegisterBlock {
        pac::Spi1::PTR as *const pac::spi0::RegisterBlock
    }
}

/// Pins that can be handed to an [`Spi`].
///
/// Implemented for `(sck, mosi, miso)`, for `(sck, mosi)` when the bus is write-only,
/// and for `()` when the pins are managed elsewhere.
pub trait Pins<SPI> {}

impl<SPI> Pins<SPI> for () {}
impl<SPI, const P1: u8, const N1: u8, const P2: u8, const N2: u8> Pins<SPI>
    for (Pin<P1, N1, Alternate>, Pin<P2, N2, Alternate>)
{
}
impl<
    SPI,
    const P1: u8,
    const N1: u8,
    const P2: u8,
    const N2: u8,
    const P3: u8,
    const N3: u8,
> Pins<SPI>
    for (
        Pin<P1, N1, Alternate>,
        Pin<P2, N2, Alternate>,
        Pin<P3, N3, Alternate>,
    )
{
}

// --- Bit rate --------------------------------------------------------------------

/// `SPBR` and `SPCMD0.BRDV`. SCK is `PCLKA / (2 * (spbr + 1) * 2^brdv)`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
struct BitRate {
    spbr: u8,
    brdv: u8,
}

/// Pick the fastest setting that does not exceed `frequency`.
///
/// Running an SPI slave faster than it is rated for is a hardware fault, so the
/// search rounds down rather than to nearest.
fn calc_bitrate(pclka: u32, frequency: u32) -> Option<BitRate> {
    if frequency == 0 {
        return None;
    }

    let mut best: Option<(BitRate, u32)> = None;

    for brdv in 0u8..4 {
        let base = 2u32 * (1 << brdv);
        // spbr = pclka / (base * frequency) - 1, rounded up so we stay at or below
        // the requested rate.
        let den = base.checked_mul(frequency)?;
        let n = pclka.div_ceil(den);
        if n == 0 {
            continue;
        }
        let spbr = n - 1;
        if spbr > 255 {
            continue;
        }
        let actual = pclka / (base * (spbr + 1));
        match best {
            Some((_, best_hz)) if best_hz >= actual => {}
            _ => {
                best = Some((
                    BitRate {
                        spbr: spbr as u8,
                        brdv,
                    },
                    actual,
                ))
            }
        }
    }

    best.map(|(rate, _)| rate)
}

// --- Driver ----------------------------------------------------------------------

/// A polled SPI master.
#[derive(Debug)]
pub struct Spi<SPI, PINS> {
    spi: SPI,
    pins: PINS,
}

impl<SPI: Instance, PINS: Pins<SPI>> Spi<SPI, PINS> {
    /// Bring up the channel as a master.
    pub fn new(spi: SPI, pins: PINS, config: Config, clocks: &Clocks) -> Result<Self, Error> {
        let rate = calc_bitrate(clocks.pclka().to_raw(), config.frequency)
            .ok_or(Error::UnsupportedFrequency)?;

        mstp::start(SPI::MODULE);

        let this = Spi { spi, pins };
        let regs = this.regs();

        // Everything below SPE must be configured with the peripheral disabled.
        regs.spcr().write(|w| unsafe { w.bits(0) });
        regs.sslp().write(|w| unsafe { w.bits(0) }); // SSL active low
        regs.sppcr().write(|w| unsafe { w.bits(0) }); // no loopback
        regs.spbr().write(|w| unsafe { w.spr().bits(rate.spbr) });

        // SPLW = 0: SPDR is accessed as a byte or halfword. This driver only does
        // 8-bit frames and writes SPDR one byte at a time.
        regs.spdcr().write(|w| {
            w.sprdtd().clear_bit();
            w.splw().clear_bit()
        });

        // No inter-transfer delays; the RSPI inserts them between sequences and we
        // are running a single command repeatedly.
        regs.spckd().write(|w| unsafe { w.bits(0) });
        regs.sslnd().write(|w| unsafe { w.bits(0) });
        regs.spnd().write(|w| unsafe { w.bits(0) });
        regs.spcr2().write(|w| unsafe { w.bits(0) });

        regs.spcmd0().write(|w| {
            w.cpha().bit(config.mode.phase == embedded_hal::spi::Phase::CaptureOnSecondTransition);
            w.cpol().bit(config.mode.polarity == embedded_hal::spi::Polarity::IdleHigh);
            unsafe { w.brdv().bits(rate.brdv) };
            unsafe { w.ssla().bits(0) };
            unsafe { w.spb().bits(SPB_8_BIT) };
            w.lsbf().bit(config.bit_order == BitOrder::LsbFirst);
            w.spnden().clear_bit();
            w.slnden().clear_bit();
            w.sckden().clear_bit()
        });

        regs.spcr().write(|w| {
            w.spms().clear_bit(); // SPI operation (4-wire)
            w.txmd().clear_bit(); // full duplex
            w.modfen().clear_bit(); // no mode-fault detection: we are the only master
            w.mstr().set_bit();
            w.speie().clear_bit();
            w.sptie().clear_bit();
            w.sprie().clear_bit();
            w.spe().set_bit()
        });

        Ok(this)
    }

    /// Stop the channel and give back the peripheral and pins.
    pub fn release(self) -> (SPI, PINS) {
        self.regs().spcr().write(|w| unsafe { w.bits(0) });
        mstp::stop(SPI::MODULE);
        (self.spi, self.pins)
    }

    #[inline(always)]
    fn regs(&self) -> &'static pac::spi0::RegisterBlock {
        // SAFETY: `Spi` owns the peripheral token for this channel.
        unsafe { &*SPI::ptr() }
    }

    /// Byte-wide view of `SPDR`.
    ///
    /// The PAC only exposes `SPDR` as a word and a halfword, but with `SPDCR.SPLW`
    /// clear and an 8-bit frame length the register is meant to be accessed a byte at
    /// a time: a halfword write would queue a 16-bit frame's worth of shift data.
    #[inline(always)]
    fn spdr_byte(&self) -> *mut u8 {
        // SAFETY: SPDR is at offset 0x04 in the SPI register block, and byte access
        // to it is explicitly supported when SPLW = 0.
        unsafe { (SPI::ptr() as *mut u8).add(0x04) }
    }

    /// Exchange one frame: shift `out` out and return whatever came back.
    fn transfer_frame(&mut self, out: u8) -> Result<u8, Error> {
        if !spin_until(TIMEOUT, || self.regs().spsr().read().sptef().bit_is_set()) {
            return Err(Error::Timeout);
        }
        // SAFETY: `spdr_byte` points at SPDR, which is valid for byte writes here.
        unsafe { self.spdr_byte().write_volatile(out) };

        if !spin_until(TIMEOUT, || self.regs().spsr().read().sprf().bit_is_set()) {
            return Err(Error::Timeout);
        }
        self.check_errors()?;
        // SAFETY: as above; reading SPDR also clears SPRF.
        Ok(unsafe { self.spdr_byte().read_volatile() })
    }

    fn check_errors(&mut self) -> Result<(), Error> {
        let sr = self.regs().spsr().read();
        if sr.ovrf().bit_is_set() {
            self.clear_errors();
            return Err(Error::Overrun);
        }
        if sr.modf().bit_is_set() {
            self.clear_errors();
            return Err(Error::ModeFault);
        }
        if sr.perf().bit_is_set() {
            self.clear_errors();
            return Err(Error::Parity);
        }
        Ok(())
    }

    /// `SPSR` error flags are cleared by writing 0 after reading a 1.
    fn clear_errors(&mut self) {
        self.regs().spsr().modify(|_, w| {
            w.ovrf().clear_bit();
            w.modf().clear_bit();
            w.perf().clear_bit();
            w.udrf().clear_bit()
        });
    }
}

impl<SPI, PINS> ErrorType for Spi<SPI, PINS> {
    type Error = Error;
}

impl<SPI: Instance, PINS: Pins<SPI>> SpiBus<u8> for Spi<SPI, PINS> {
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        // Full duplex: something has to be clocked out to clock anything in.
        for word in words {
            *word = self.transfer_frame(0xFF)?;
        }
        Ok(())
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        for &word in words {
            self.transfer_frame(word)?;
        }
        Ok(())
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        // The buffers may differ in length: pad the short side. Clocking out 0xFF
        // past the end of `write` keeps MOSI idle-high, which is what most devices
        // expect on a read-only tail.
        let len = read.len().max(write.len());
        for i in 0..len {
            let out = write.get(i).copied().unwrap_or(0xFF);
            let got = self.transfer_frame(out)?;
            if let Some(slot) = read.get_mut(i) {
                *slot = got;
            }
        }
        Ok(())
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        for word in words {
            *word = self.transfer_frame(*word)?;
        }
        Ok(())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        // IDLNF stays set while a frame is still in the shift register.
        if spin_until(TIMEOUT, || self.regs().spsr().read().idlnf().bit_is_clear()) {
            Ok(())
        } else {
            Err(Error::Timeout)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn achieved(pclka: u32, r: BitRate) -> u32 {
        pclka / (2 * (1 << r.brdv) * (r.spbr as u32 + 1))
    }

    #[test]
    fn never_exceeds_the_requested_rate() {
        let pclka = 48_000_000;
        for target in [100_000u32, 1_000_000, 4_000_000, 8_000_000, 24_000_000] {
            let r = calc_bitrate(pclka, target).expect("representable");
            let got = achieved(pclka, r);
            assert!(got <= target, "{target}: got {got}, which is too fast");
        }
    }

    #[test]
    fn picks_the_fastest_available_setting() {
        // 48 MHz / 2 is the ceiling, and it is exactly representable.
        let r = calc_bitrate(48_000_000, 24_000_000).unwrap();
        assert_eq!(achieved(48_000_000, r), 24_000_000);
    }

    #[test]
    fn rejects_impossible_rates() {
        assert!(calc_bitrate(48_000_000, 0).is_none());
        // Below PCLKA / (2 * 8 * 256) nothing fits.
        assert!(calc_bitrate(48_000_000, 1_000).is_none());
    }
}
