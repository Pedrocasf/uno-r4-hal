//! 14-bit analog-to-digital converter (ADC140).
//!
//! ```ignore
//! let _a0 = p0.p000.into_analog();
//! let mut adc = Adc::new(dp.adc140, Config::default(), &clocks);
//! let raw = adc.read(Channel::AN000).unwrap();
//! ```
//!
//! `embedded-hal` 1.0 has no ADC trait, so this is a plain inherent API: one
//! single-scan conversion per [`read`](Adc::read), polled to completion.
//!
//! The pin belonging to a channel must be switched to
//! [`into_analog`](crate::gpio::Pin::into_analog) first, which disconnects its
//! digital input buffer. The driver cannot do that for you, because the channel
//! number to pin mapping is not something it can check.

use crate::clock::Clocks;
use crate::mstp::{self, ModuleStop};
use crate::pac;
use crate::spin_until;

/// Poll budget for one conversion.
const TIMEOUT: u32 = 1_000_000;

/// Base address of the `ADDRn` array.
const ADDR_BASE: usize = 0x4005_c000 + 0x20;

/// Conversion errors.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Error {
    /// The conversion did not finish in time.
    Timeout,
    /// The channel number is outside the range the hardware implements.
    InvalidChannel,
}

/// Result resolution.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum Resolution {
    /// 14 bits. Results are 0..=16383.
    #[default]
    Bits14,
    /// 12 bits. Results are 0..=4095.
    Bits12,
    /// 10 bits. Results are 0..=1023.
    Bits10,
    /// 8 bits. Results are 0..=255.
    Bits8,
}

impl Resolution {
    /// The largest value a conversion can produce.
    pub const fn max_value(self) -> u16 {
        match self {
            Resolution::Bits14 => 16383,
            Resolution::Bits12 => 4095,
            Resolution::Bits10 => 1023,
            Resolution::Bits8 => 255,
        }
    }

    /// `ADCER.ADPRC`.
    const fn adprc(self) -> u8 {
        match self {
            Resolution::Bits14 => 0b00,
            Resolution::Bits12 => 0b01,
            Resolution::Bits10 => 0b10,
            Resolution::Bits8 => 0b11,
        }
    }
}

/// ADC configuration.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Config {
    /// Result resolution.
    pub resolution: Resolution,
    /// Sampling time in ADC clock (PCLKC) cycles, 5..=255.
    ///
    /// Longer sampling lets a higher-impedance source settle. The reset value of 13
    /// suits sources up to a few kilohms; a 100 kΩ divider wants far more.
    pub sample_cycles: u8,
    /// Convert at high speed. Draws more current and requires high-speed operating
    /// mode, which the stock 48 MHz clock configuration already selects.
    pub high_speed: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            resolution: Resolution::Bits14,
            sample_cycles: 13,
            high_speed: true,
        }
    }
}

/// An analog input channel.
///
/// Which `ANnnn` inputs are bonded out depends on the package; see the pin list in
/// the datasheet.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Channel(u8);

impl Channel {
    /// AN000.
    pub const AN000: Self = Self(0);
    /// AN001.
    pub const AN001: Self = Self(1);
    /// AN002.
    pub const AN002: Self = Self(2);
    /// AN003.
    pub const AN003: Self = Self(3);
    /// AN004.
    pub const AN004: Self = Self(4);
    /// AN005.
    pub const AN005: Self = Self(5);
    /// AN006.
    pub const AN006: Self = Self(6);
    /// AN007.
    pub const AN007: Self = Self(7);
    /// AN008.
    pub const AN008: Self = Self(8);
    /// AN009.
    pub const AN009: Self = Self(9);
    /// AN016.
    pub const AN016: Self = Self(16);
    /// AN017.
    pub const AN017: Self = Self(17);
    /// AN018.
    pub const AN018: Self = Self(18);
    /// AN019.
    pub const AN019: Self = Self(19);
    /// AN020.
    pub const AN020: Self = Self(20);
    /// AN021.
    pub const AN021: Self = Self(21);
    /// AN022.
    pub const AN022: Self = Self(22);
    /// AN023.
    pub const AN023: Self = Self(23);
    /// AN024.
    pub const AN024: Self = Self(24);
    /// AN025.
    pub const AN025: Self = Self(25);

    /// An arbitrary channel number.
    ///
    /// Returns `None` for numbers the `ADANSA` registers cannot select: 10..=15 and
    /// anything above 25.
    pub const fn new(n: u8) -> Option<Self> {
        if n <= 9 || (16 <= n && n <= 25) {
            Some(Self(n))
        } else {
            None
        }
    }

    /// The channel number.
    pub const fn number(self) -> u8 {
        self.0
    }
}

/// A polled single-scan ADC.
#[derive(Debug)]
pub struct Adc {
    adc: pac::Adc140,
    resolution: Resolution,
}

impl Adc {
    /// Bring up the converter for single-scan, software-triggered conversions.
    pub fn new(adc: pac::Adc140, config: Config, _clocks: &Clocks) -> Self {
        mstp::start(ModuleStop::ADC140);

        // ADCSR must be cleared before anything else is configured.
        adc.adcsr().write(|w| unsafe { w.bits(0) });

        adc.adcer().write(|w| {
            unsafe { w.adprc().bits(config.resolution.adprc()) };
            w.adrfmt().clear_bit(); // right-aligned results
            w.ace().clear_bit(); // no automatic clearing of ADDR on read
            w.diagm().clear_bit()
        });

        // AVCC0 / AVSS0 as the reference.
        adc.adhvrefcnt().write(|w| unsafe { w.bits(0) });

        // One sampling-state register covers every channel we expose.
        let sst = config.sample_cycles.max(5);
        for n in 0..15 {
            adc.adsstr(n).write(|w| unsafe { w.bits(sst) });
        }
        adc.adsstrl().write(|w| unsafe { w.sst().bits(sst) });

        adc.adcsr().write(|w| {
            // ADCS = 00: single scan.
            unsafe { w.adcs().bits(0b00) };
            w.adhsc().bit(!config.high_speed);
            w.trge().clear_bit();
            w.extrg().clear_bit();
            w.dble().clear_bit();
            w.gbadie().clear_bit()
        });

        Self {
            adc,
            resolution: config.resolution,
        }
    }

    /// Stop the converter and give the peripheral back.
    pub fn release(self) -> pac::Adc140 {
        self.adc.adcsr().write(|w| unsafe { w.bits(0) });
        mstp::stop(ModuleStop::ADC140);
        self.adc
    }

    /// The largest value [`read`](Self::read) can return, for scaling.
    pub const fn max_value(&self) -> u16 {
        self.resolution.max_value()
    }

    /// Convert one channel and return the result.
    pub fn read(&mut self, channel: Channel) -> Result<u16, Error> {
        let n = channel.number();
        if !(n <= 9 || (16..=25).contains(&n)) {
            return Err(Error::InvalidChannel);
        }

        // Select exactly this channel. Writing both halves unconditionally clears
        // whatever the previous call selected.
        if n < 16 {
            self.adc.adansa0().write(|w| unsafe { w.bits(1 << n) });
            self.adc.adansa1().write(|w| unsafe { w.bits(0) });
        } else {
            self.adc.adansa0().write(|w| unsafe { w.bits(0) });
            self.adc
                .adansa1()
                .write(|w| unsafe { w.bits(1 << (n - 16)) });
        }

        // ADST stays set for the duration of the scan and clears itself at the end.
        self.adc.adcsr().modify(|_, w| w.adst().set_bit());
        if !spin_until(TIMEOUT, || self.adc.adcsr().read().adst().bit_is_clear()) {
            return Err(Error::Timeout);
        }

        Ok(self.read_addr(n))
    }

    /// Read `ADDRn`.
    ///
    /// The PAC's generated accessors put every register above `ADDR15` at the same
    /// offset, which is an SVD defect. The array is a flat run of 16-bit registers at
    /// `ADDR_BASE + 2 * n`, so it is indexed directly here.
    fn read_addr(&self, n: u8) -> u16 {
        let ptr = (ADDR_BASE + 2 * n as usize) as *const u16;
        // SAFETY: `n` is in 0..=25, so this lands inside the ADDR array, and `Adc`
        // owns the peripheral.
        unsafe { ptr.read_volatile() }
    }
}
