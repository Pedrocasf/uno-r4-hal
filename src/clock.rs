//! Clock tree configuration.
//!
//! Out of reset the RA4M1 runs from MOCO (8 MHz). [`Config`] walks the chip through
//! the sequence the hardware manual requires to get to a faster clock: raise the
//! flash wait states and operating power mode *first*, start the oscillator, wait for
//! it to stabilise, program the dividers, and only then switch the system clock
//! source over.
//!
//! The resulting [`Clocks`] is a `Copy` snapshot of the frequencies every other
//! driver needs to compute its own dividers.
//!
//! Every wait on an oscillator is bounded. Selecting a source that never arrives —
//! most obviously the main oscillator on a board with no crystal fitted, which is
//! every Uno R4 — returns an [`Error`] instead of hanging the boot.

use crate::pac;
use crate::{Hertz, spin_until};

/// Poll budget for one oscillator or mode transition.
///
/// These waits all happen before the system clock is switched, so the core is still
/// on MOCO at 8 MHz: a few hundred thousand polls is well over the millisecond or
/// two that even a crystal needs, and still gives up in a fraction of a second when
/// the source is never going to arrive.
const TIMEOUT: u32 = 1_000_000;

/// Why [`Config::freeze`] could not bring the clock tree up.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Error {
    /// The main oscillator never reported stable.
    ///
    /// Usually means no crystal or external clock is fitted. Neither Uno R4 has one:
    /// both run from HOCO, so [`Config::uno_r4`] never touches this oscillator.
    MainOscTimeout,
    /// The high-speed on-chip oscillator never reported stable.
    HocoTimeout,
    /// The PLL never locked.
    PllTimeout,
    /// The operating power mode transition did not complete.
    PowerModeTimeout,
    /// `SCKSCR` did not read back the requested source.
    SwitchTimeout,
    /// [`SysClk::Pll`] was selected without filling in [`Config::pll`].
    MissingPllConfig,
    /// The PLL multiplier is outside the 2..=31 the hardware encodes.
    InvalidPllMultiplier,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            Error::MainOscTimeout => "main oscillator did not stabilise (no crystal fitted?)",
            Error::HocoTimeout => "high-speed on-chip oscillator did not stabilise",
            Error::PllTimeout => "PLL did not lock",
            Error::PowerModeTimeout => "operating power mode transition did not complete",
            Error::SwitchTimeout => "system clock source did not switch",
            Error::MissingPllConfig => "SysClk::Pll selected without a PLL configuration",
            Error::InvalidPllMultiplier => "PLL multiplier outside 2..=31",
        })
    }
}

impl core::error::Error for Error {}

/// System clock source (`SCKSCR.CKSEL`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum SysClk {
    /// High-speed on-chip oscillator.
    Hoco,
    /// Middle-speed on-chip oscillator (8 MHz). The reset default.
    Moco,
    /// Low-speed on-chip oscillator (32.768 kHz).
    Loco,
    /// Main clock oscillator (external crystal or clock input).
    MainOsc,
    /// Sub-clock oscillator (32.768 kHz watch crystal).
    SubOsc,
    /// PLL output.
    Pll,
}

impl SysClk {
    const fn cksel(self) -> u8 {
        match self {
            SysClk::Hoco => 0,
            SysClk::Moco => 1,
            SysClk::Loco => 2,
            SysClk::MainOsc => 3,
            SysClk::SubOsc => 4,
            SysClk::Pll => 5,
        }
    }
}

/// A `SCKDIVCR` divider field.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum Div {
    /// Divide by 1.
    Div1 = 0,
    /// Divide by 2.
    Div2 = 1,
    /// Divide by 4.
    Div4 = 2,
    /// Divide by 8.
    Div8 = 3,
    /// Divide by 16.
    Div16 = 4,
    /// Divide by 32.
    Div32 = 5,
    /// Divide by 64.
    Div64 = 6,
}

impl Div {
    const fn divisor(self) -> u32 {
        1 << (self as u32)
    }
}

/// PLL output divider (`PLLCCR2.PLODIV`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum PllDiv {
    /// Divide the VCO output by 1.
    Div1 = 0,
    /// Divide the VCO output by 2.
    Div2 = 1,
    /// Divide the VCO output by 4.
    Div4 = 2,
}

impl PllDiv {
    const fn divisor(self) -> u32 {
        1 << (self as u32)
    }
}

/// PLL settings. The output is `main_osc * mul / div`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Pll {
    /// Multiplication factor, 2..=31. Written to `PLLCCR2.PLLMUL` as `mul - 1`.
    pub mul: u8,
    /// Output divider.
    pub div: PllDiv,
}

/// What the main clock oscillator is connected to.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum MainOscKind {
    /// A crystal or ceramic resonator across XTAL/EXTAL.
    Resonator,
    /// A square wave driven into EXTAL.
    ExternalClock,
}

/// Clock tree configuration.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Config {
    /// Which source drives ICLK.
    pub sysclk: SysClk,
    /// Frequency of the crystal / external clock on the main oscillator, if used.
    pub main_osc: Hertz,
    /// Whether the main oscillator drives a resonator or takes a clock input.
    pub main_osc_kind: MainOscKind,
    /// PLL settings. Required when `sysclk` is [`SysClk::Pll`]; the PLL always takes
    /// the main oscillator as its source on this part.
    pub pll: Option<Pll>,
    /// HOCO frequency.
    ///
    /// The HOCO runs at 24, 32, 48 or 64 MHz depending on `OFS1.HOCOFRQ1` in the
    /// option-setting memory, which is fixed when the part is programmed and is not
    /// readable through a normal register. If you select [`SysClk::Hoco`] you must
    /// tell the HAL what that value is: every baud rate and timer period in the
    /// crate is derived from it.
    pub hoco: Hertz,
    /// ICLK divider (CPU, DTC, flash).
    pub ick: Div,
    /// PCLKA divider (SPI, SCI, GPT).
    pub pcka: Div,
    /// PCLKB divider (SCI, IIC, AGT and most other peripherals).
    pub pckb: Div,
    /// PCLKC divider (ADC sampling clock).
    pub pckc: Div,
    /// PCLKD divider (ADC, GPT).
    pub pckd: Div,
    /// FCLK divider (flash programming).
    pub fck: Div,
}

impl Default for Config {
    fn default() -> Self {
        Self::moco()
    }
}

impl Config {
    /// The reset configuration: MOCO at 8 MHz, every divider at 1.
    pub const fn moco() -> Self {
        Self {
            sysclk: SysClk::Moco,
            main_osc: Hertz::from_raw(12_000_000),
            main_osc_kind: MainOscKind::Resonator,
            pll: None,
            hoco: Hertz::from_raw(48_000_000),
            ick: Div::Div1,
            pcka: Div::Div1,
            pckb: Div::Div1,
            pckc: Div::Div1,
            pckd: Div::Div1,
            fck: Div::Div1,
        }
    }

    /// The Arduino Uno R4 configuration: HOCO at 48 MHz.
    ///
    /// ICLK, PCLKA, PCLKC and PCLKD run at 48 MHz; PCLKB and FCLK at 24 MHz.
    ///
    /// **Neither Uno R4 has a crystal fitted.** The Minima and the WiFi both leave
    /// XTAL/EXTAL unpopulated and run the whole part from the high-speed on-chip
    /// oscillator, which is also why HOCO is trimmed to 48 MHz rather than the more
    /// common 24: USB full-speed needs a 48 MHz UCLK and has nowhere else to get it.
    /// Starting the main oscillator on one of these boards waits for a stabilisation
    /// flag that can never assert.
    ///
    /// Matches `BSP_CFG_CLOCK_SOURCE`, `BSP_CFG_HOCO_FREQUENCY`, `BSP_CFG_XTAL_HZ`
    /// and the divider settings in ArduinoCore-renesas,
    /// `variants/*/includes/ra_gen/bsp_clock_cfg.h` — identical for both boards.
    pub const fn uno_r4() -> Self {
        Self {
            sysclk: SysClk::Hoco,
            // Unused: there is no crystal. Left at a sane value so a caller who
            // switches `sysclk` over has a starting point.
            main_osc: Hertz::from_raw(12_000_000),
            main_osc_kind: MainOscKind::Resonator,
            pll: None,
            hoco: Hertz::from_raw(48_000_000),
            ick: Div::Div1,
            pcka: Div::Div1,
            pckb: Div::Div2,
            pckc: Div::Div1,
            pckd: Div::Div1,
            fck: Div::Div2,
        }
    }

    /// Frequency of the selected system clock source, before `SCKDIVCR`.
    pub const fn source_hz(&self) -> u32 {
        match self.sysclk {
            SysClk::Hoco => self.hoco.to_raw(),
            SysClk::Moco => 8_000_000,
            SysClk::Loco => 32_768,
            SysClk::MainOsc => self.main_osc.to_raw(),
            SysClk::SubOsc => 32_768,
            SysClk::Pll => match self.pll {
                Some(pll) => self.main_osc.to_raw() / pll.div.divisor() * pll.mul as u32,
                // `freeze` rejects this combination; a hand-built struct that hits it
                // would produce a nonsense frequency rather than a wrong one.
                None => 0,
            },
        }
    }

    /// Apply the configuration and return the resulting frequencies.
    ///
    /// Consumes the `SYSTEM` peripheral, so the clock tree cannot be reconfigured out
    /// from under a driver that has already cached a frequency.
    ///
    /// Every hardware wait is bounded. On a timeout the part is left on whatever
    /// source it was already running (MOCO at 8 MHz out of reset), the protection
    /// registers are re-locked, and the [`Error`] says which step gave up — so a
    /// board with no crystal reports [`Error::MainOscTimeout`] rather than hanging
    /// before `main` gets anywhere.
    pub fn freeze(self, system: pac::System) -> Result<Clocks, Error> {
        if self.sysclk == SysClk::Pll {
            let pll = self.pll.ok_or(Error::MissingPllConfig)?;
            if pll.mul < 2 || pll.mul > 31 {
                return Err(Error::InvalidPllMultiplier);
            }
        }

        let source_hz = self.source_hz();
        let iclk = source_hz / self.ick.divisor();

        unprotect(&system);

        // Raising the operating power mode and the flash wait state has to happen
        // *before* the clock speeds up. `freeze` is one-shot and only ever raises.
        if iclk > 32_000_000 {
            // OPCM = 0: high-speed mode.
            system.opccr().write(|w| unsafe { w.bits(0) });
            if !spin_until(TIMEOUT, || system.opccr().read().opcmtsf().bit_is_clear()) {
                protect(&system);
                return Err(Error::PowerModeTimeout);
            }
            system.memwait().write(|w| w.memwait().set_bit());
        }

        let started = match self.sysclk {
            SysClk::MainOsc => self.start_main_osc(&system),
            SysClk::Pll => self
                .start_main_osc(&system)
                .and_then(|()| self.start_pll(&system)),
            SysClk::Hoco => start_hoco(&system),
            // MOCO, LOCO and the sub-clock are either already running out of reset or
            // are outside what this driver configures.
            SysClk::Moco | SysClk::Loco | SysClk::SubOsc => Ok(()),
        };
        if let Err(e) = started {
            // Leave the part on the source it is already running from.
            protect(&system);
            return Err(e);
        }

        let divs = ((self.fck as u32) << 28)
            | ((self.ick as u32) << 24)
            | ((self.pcka as u32) << 12)
            | ((self.pckb as u32) << 8)
            | ((self.pckc as u32) << 4)
            | (self.pckd as u32);
        system.sckdivcr().write(|w| unsafe { w.bits(divs) });

        let cksel = self.sysclk.cksel();
        system.sckscr().write(|w| unsafe { w.bits(cksel) });
        // The switch is not instantaneous; spin until the register reads back.
        if !spin_until(TIMEOUT, || system.sckscr().read().bits() == cksel) {
            protect(&system);
            return Err(Error::SwitchTimeout);
        }

        protect(&system);

        Ok(Clocks {
            iclk: Hertz::from_raw(iclk),
            pclka: Hertz::from_raw(source_hz / self.pcka.divisor()),
            pclkb: Hertz::from_raw(source_hz / self.pckb.divisor()),
            pclkc: Hertz::from_raw(source_hz / self.pckc.divisor()),
            pclkd: Hertz::from_raw(source_hz / self.pckd.divisor()),
            fclk: Hertz::from_raw(source_hz / self.fck.divisor()),
        })
    }

    fn start_main_osc(&self, system: &pac::System) -> Result<(), Error> {
        if system.mosccr().read().mostp().bit_is_clear() {
            return Ok(()); // already running
        }
        // MOMCR must be written while the oscillator is stopped.
        //   MOSEL  (b6): 0 = resonator, 1 = external clock input
        //   MODRV1 (b3): 0 = 10..20 MHz, 1 = 1..10 MHz
        let mosel = matches!(self.main_osc_kind, MainOscKind::ExternalClock);
        let modrv1 = self.main_osc.to_raw() < 10_000_000;
        system.momcr().write(|w| {
            w.mosel().bit(mosel);
            w.modrv1().bit(modrv1)
        });
        // Longest available stabilisation wait. Only paid once, at boot.
        system.moscwtcr().write(|w| unsafe { w.bits(0x09) });
        system.mosccr().write(|w| w.mostp().clear_bit());
        if spin_until(TIMEOUT, || system.oscsf().read().moscsf().bit_is_set()) {
            Ok(())
        } else {
            // Stop it again so the caller is left on a known-good source.
            system.mosccr().write(|w| w.mostp().set_bit());
            Err(Error::MainOscTimeout)
        }
    }

    fn start_pll(&self, system: &pac::System) -> Result<(), Error> {
        let pll = self.pll.ok_or(Error::MissingPllConfig)?;
        // PLLCCR2 may only be written while the PLL is stopped.
        system.pllcr().write(|w| w.pllstp().set_bit());
        let pllccr2 = (pll.mul - 1) | ((pll.div as u8) << 6);
        system.pllccr2().write(|w| unsafe { w.bits(pllccr2) });
        system.pllcr().write(|w| w.pllstp().clear_bit());
        if spin_until(TIMEOUT, || system.oscsf().read().pllsf().bit_is_set()) {
            Ok(())
        } else {
            system.pllcr().write(|w| w.pllstp().set_bit());
            Err(Error::PllTimeout)
        }
    }
}

fn start_hoco(system: &pac::System) -> Result<(), Error> {
    if system.hococr().read().hcstp().bit_is_clear() {
        return Ok(());
    }
    system.hococr().write(|w| w.hcstp().clear_bit());
    if spin_until(TIMEOUT, || system.oscsf().read().hocosf().bit_is_set()) {
        Ok(())
    } else {
        system.hococr().write(|w| w.hcstp().set_bit());
        Err(Error::HocoTimeout)
    }
}

/// Unlock the clock-generation registers (`PRCR.PRC0` and `PRC1`).
fn unprotect(system: &pac::System) {
    system.prcr().write(|w| unsafe { w.bits(0xA503) });
}

/// Re-lock the clock-generation registers.
fn protect(system: &pac::System) {
    system.prcr().write(|w| unsafe { w.bits(0xA500) });
}

/// Frozen clock frequencies.
///
/// Handed to every driver that needs to compute a divider. There is no way to build
/// one except through [`Config::freeze`], so a `Clocks` value is always an accurate
/// description of the hardware.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Clocks {
    iclk: Hertz,
    pclka: Hertz,
    pclkb: Hertz,
    pclkc: Hertz,
    pclkd: Hertz,
    fclk: Hertz,
}

impl Clocks {
    /// CPU / system clock.
    pub const fn iclk(&self) -> Hertz {
        self.iclk
    }
    /// Peripheral clock A: SPI, SCI, GPT.
    pub const fn pclka(&self) -> Hertz {
        self.pclka
    }
    /// Peripheral clock B: SCI, IIC, AGT and most other peripherals.
    pub const fn pclkb(&self) -> Hertz {
        self.pclkb
    }
    /// Peripheral clock C: ADC conversion clock.
    pub const fn pclkc(&self) -> Hertz {
        self.pclkc
    }
    /// Peripheral clock D: ADC, GPT.
    pub const fn pclkd(&self) -> Hertz {
        self.pclkd
    }
    /// Flash clock.
    pub const fn fclk(&self) -> Hertz {
        self.fclk
    }
}

/// Lets you write `dp.system.freeze(config)` instead of `config.freeze(dp.system)`.
pub trait ClockConfigExt {
    /// Apply `config` to this `SYSTEM` peripheral.
    fn freeze(self, config: Config) -> Result<Clocks, Error>;
}

impl ClockConfigExt for pac::System {
    fn freeze(self, config: Config) -> Result<Clocks, Error> {
        config.freeze(self)
    }
}
