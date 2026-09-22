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

use crate::Hertz;
use crate::pac;

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

    /// The Arduino Uno R4 configuration: 12 MHz crystal, PLL x8 /2, 48 MHz ICLK.
    ///
    /// ICLK, PCLKA and PCLKD run at 48 MHz; PCLKB, PCLKC and FCLK at 24 MHz.
    pub const fn uno_r4() -> Self {
        Self {
            sysclk: SysClk::Pll,
            main_osc: Hertz::from_raw(12_000_000),
            main_osc_kind: MainOscKind::Resonator,
            pll: Some(Pll {
                mul: 8,
                div: PllDiv::Div2,
            }),
            hoco: Hertz::from_raw(48_000_000),
            ick: Div::Div1,
            pcka: Div::Div1,
            pckb: Div::Div2,
            pckc: Div::Div2,
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
    /// # Panics
    ///
    /// If `sysclk` is [`SysClk::Pll`] but `pll` is `None`, or if the PLL multiplier
    /// is outside 2..=31.
    pub fn freeze(self, system: pac::System) -> Clocks {
        if self.sysclk == SysClk::Pll {
            let pll = self.pll.expect("SysClk::Pll selected without Config::pll");
            assert!(
                pll.mul >= 2 && pll.mul <= 31,
                "PLL multiplier must be in 2..=31"
            );
        }

        let source_hz = self.source_hz();
        let iclk = source_hz / self.ick.divisor();

        unprotect(&system);

        // Raising the operating power mode and the flash wait state has to happen
        // *before* the clock speeds up. `freeze` is one-shot and only ever raises.
        if iclk > 32_000_000 {
            // OPCM = 0: high-speed mode.
            system.opccr().write(|w| unsafe { w.bits(0) });
            while system.opccr().read().opcmtsf().bit_is_set() {}
            system.memwait().write(|w| w.memwait().set_bit());
        }

        match self.sysclk {
            SysClk::MainOsc => self.start_main_osc(&system),
            SysClk::Pll => {
                self.start_main_osc(&system);
                self.start_pll(&system);
            }
            SysClk::Hoco => start_hoco(&system),
            // MOCO, LOCO and the sub-clock are either already running out of reset or
            // are outside what this driver configures.
            SysClk::Moco | SysClk::Loco | SysClk::SubOsc => {}
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
        while system.sckscr().read().bits() != cksel {}

        protect(&system);

        Clocks {
            iclk: Hertz::from_raw(iclk),
            pclka: Hertz::from_raw(source_hz / self.pcka.divisor()),
            pclkb: Hertz::from_raw(source_hz / self.pckb.divisor()),
            pclkc: Hertz::from_raw(source_hz / self.pckc.divisor()),
            pclkd: Hertz::from_raw(source_hz / self.pckd.divisor()),
            fclk: Hertz::from_raw(source_hz / self.fck.divisor()),
        }
    }

    fn start_main_osc(&self, system: &pac::System) {
        if system.mosccr().read().mostp().bit_is_clear() {
            return; // already running
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
        while system.oscsf().read().moscsf().bit_is_clear() {}
    }

    fn start_pll(&self, system: &pac::System) {
        let pll = self.pll.expect("checked at the top of freeze");
        // PLLCCR2 may only be written while the PLL is stopped.
        system.pllcr().write(|w| w.pllstp().set_bit());
        let pllccr2 = (pll.mul - 1) | ((pll.div as u8) << 6);
        system.pllccr2().write(|w| unsafe { w.bits(pllccr2) });
        system.pllcr().write(|w| w.pllstp().clear_bit());
        while system.oscsf().read().pllsf().bit_is_clear() {}
    }
}

fn start_hoco(system: &pac::System) {
    if system.hococr().read().hcstp().bit_is_clear() {
        return;
    }
    system.hococr().write(|w| w.hcstp().clear_bit());
    while system.oscsf().read().hocosf().bit_is_clear() {}
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
    fn freeze(self, config: Config) -> Clocks;
}

impl ClockConfigExt for pac::System {
    fn freeze(self, config: Config) -> Clocks {
        config.freeze(self)
    }
}
