//! PWM on the GPT peripherals.
//!
//! ```ignore
//! // D3 on an Uno R4 Minima is P104 = GTIOC1B.
//! let _ = pins.d3.into_alternate(AltFunction::GptGroup2);
//! let pwm = Pwm::new(dp.gpt321, 1.kHz(), &clocks).unwrap();
//! let (_a, mut b) = pwm.split();
//! b.enable();
//! b.set_duty_cycle_percent(25).unwrap();
//! ```
//!
//! Each GPT channel drives two outputs, `GTIOCnA` and `GTIOCnB`, which share one
//! period but have independent duty cycles. Channel `n` is `pac::Gpt32n` for n in
//! 0..=1 and `pac::Gpt16n` for n in 2..=7; [`crate::board`] records which channel
//! and output each header pin reaches. Both are exposed as
//! [`PwmChannel`]s implementing [`embedded_hal::pwm::SetDutyCycle`].
//!
//! The timer runs in saw-wave up-counting mode: the output goes high at the start of
//! each cycle and low at the compare match, so duty 0 is a constant low and duty
//! `max_duty_cycle()` is a constant high.

use core::marker::PhantomData;

use embedded_hal::pwm::{ErrorType, SetDutyCycle};
use fugit::HertzU32 as Hertz;

use crate::clock::Clocks;
use crate::mstp::{self, ModuleStop};
use crate::pac;

/// `GTIOR` setting for active-high saw-wave PWM: initial output low, high at cycle
/// end, low at compare match.
const GTIO_ACTIVE_HIGH: u8 = 0b01001;

/// PWM errors.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Error {
    /// The requested frequency cannot be produced from the current PCLKD with a
    /// period that fits in 16 bits.
    UnsupportedFrequency,
}

impl embedded_hal::pwm::Error for Error {
    fn kind(&self) -> embedded_hal::pwm::ErrorKind {
        embedded_hal::pwm::ErrorKind::Other
    }
}

/// A GPT channel.
///
/// # Safety
///
/// `ptr` must return the base address of a real GPT register block, and no two
/// implementations may return the same address.
pub unsafe trait Instance {
    /// This channel's module-stop bit. GPT320/321 share one bit, GPT162..167 another.
    const MODULE: ModuleStop;

    /// Base address of the channel's registers.
    ///
    /// The 16-bit and 32-bit GPT variants have identical register layouts; only the
    /// width of the counter, period and compare registers differs. This driver keeps
    /// every value inside 16 bits, so the `gpt320` view describes both.
    fn ptr() -> *const pac::gpt320::RegisterBlock;
}

macro_rules! gpt_instance {
    ($($GPT:ident => $module:ident,)+) => {
        $(
            // SAFETY: each `pac::$GPT::PTR` is the distinct, documented base address
            // of that GPT channel.
            unsafe impl Instance for pac::$GPT {
                const MODULE: ModuleStop = ModuleStop::$module;
                #[inline(always)]
                fn ptr() -> *const pac::gpt320::RegisterBlock {
                    pac::$GPT::PTR as *const pac::gpt320::RegisterBlock
                }
            }
        )+
    };
}

gpt_instance! {
    Gpt320 => GPT32,
    Gpt321 => GPT32,
    Gpt162 => GPT16,
    Gpt163 => GPT16,
    Gpt164 => GPT16,
    Gpt165 => GPT16,
    Gpt166 => GPT16,
    Gpt167 => GPT16,
}

/// The `GTIOCnA` output (type state).
#[derive(Debug)]
pub struct ChannelA;
/// The `GTIOCnB` output (type state).
#[derive(Debug)]
pub struct ChannelB;

/// A configured GPT channel running as a PWM generator.
#[derive(Debug)]
pub struct Pwm<GPT> {
    gpt: GPT,
    /// Counts per cycle. `GTPR` holds this minus one.
    period: u16,
}

impl<GPT: Instance> Pwm<GPT> {
    /// Configure the channel for saw-wave PWM at `frequency`, with both outputs
    /// disabled and at zero duty.
    pub fn new(gpt: GPT, frequency: Hertz, clocks: &Clocks) -> Result<Self, Error> {
        let (tpcs, period) =
            divider_for(clocks.pclkd().raw(), frequency.raw()).ok_or(Error::UnsupportedFrequency)?;

        mstp::start(GPT::MODULE);

        let this = Self { gpt, period };
        let regs = this.regs();

        // Unlock the write-protected registers. The key must accompany the write.
        regs.gtwp().write(|w| unsafe { w.bits(0x0000_A500) });

        // Stop and clear before reconfiguring.
        regs.gtcr().write(|w| unsafe { w.bits(0) });
        regs.gtcnt().write(|w| unsafe { w.gtcnt().bits(0) });

        // Up-counting, with the forcible-update bit so the direction takes effect now
        // rather than at the next cycle boundary.
        regs.gtuddtyc().write(|w| {
            w.ud().set_bit();
            w.udf().set_bit()
        });

        regs.gtpr()
            .write(|w| unsafe { w.gtpr().bits(u32::from(period) - 1) });
        regs.gtccra().write(|w| unsafe { w.gtccra().bits(0) });
        regs.gtccrb().write(|w| unsafe { w.gtccrb().bits(0) });

        // Buffer the compare registers so a duty change mid-cycle cannot produce a
        // runt pulse: the new value is taken up at the next cycle boundary.
        regs.gtber().write(|w| unsafe {
            w.ccra().bits(0b01);
            w.ccrb().bits(0b01);
            w.pr().bits(0b00);
            w.bd().bits(0)
        });

        // Outputs stay disabled (OAE/OBE clear) until `enable`.
        regs.gtior().write(|w| unsafe {
            w.gtioa().bits(GTIO_ACTIVE_HIGH);
            w.oae().clear_bit();
            w.gtiob().bits(GTIO_ACTIVE_HIGH);
            w.obe().clear_bit()
        });

        // MD = 000: saw-wave PWM. CST starts the counter.
        regs.gtcr().write(|w| unsafe {
            w.md().bits(0b000);
            w.tpcs().bits(tpcs);
            w.cst().set_bit()
        });

        Ok(this)
    }

    /// Stop the timer and give the peripheral back.
    pub fn release(self) -> GPT {
        let regs = self.regs();
        regs.gtior().write(|w| unsafe { w.bits(0) });
        regs.gtcr().write(|w| unsafe { w.bits(0) });
        mstp::stop(GPT::MODULE);
        self.gpt
    }

    /// Split into the two independently controllable outputs.
    pub fn split(self) -> (PwmChannel<GPT, ChannelA>, PwmChannel<GPT, ChannelB>) {
        let period = self.period;
        (
            PwmChannel {
                period,
                _gpt: PhantomData,
                _channel: PhantomData,
            },
            PwmChannel {
                period,
                _gpt: PhantomData,
                _channel: PhantomData,
            },
        )
    }

    #[inline(always)]
    fn regs(&self) -> &'static pac::gpt320::RegisterBlock {
        // SAFETY: `Pwm` owns the peripheral token for this channel.
        unsafe { &*GPT::ptr() }
    }
}

/// One of a GPT channel's two PWM outputs.
#[derive(Debug)]
pub struct PwmChannel<GPT, C> {
    period: u16,
    _gpt: PhantomData<GPT>,
    _channel: PhantomData<C>,
}

impl<GPT: Instance, C> PwmChannel<GPT, C> {
    #[inline(always)]
    fn regs(&self) -> &'static pac::gpt320::RegisterBlock {
        // SAFETY: a `PwmChannel` can only come from splitting a `Pwm`, which owned
        // the peripheral. The two channels write disjoint registers apart from
        // `GTIOR`, whose enable bits are touched under a critical section.
        unsafe { &*GPT::ptr() }
    }
}

impl<GPT: Instance> PwmChannel<GPT, ChannelA> {
    /// Drive the `GTIOCnA` pin.
    pub fn enable(&mut self) {
        // GTIOR holds both channels' settings, so this is a read-modify-write that
        // the other channel could race.
        critical_section::with(|_| self.regs().gtior().modify(|_, w| w.oae().set_bit()));
    }

    /// Release the `GTIOCnA` pin.
    pub fn disable(&mut self) {
        critical_section::with(|_| self.regs().gtior().modify(|_, w| w.oae().clear_bit()));
    }
}

impl<GPT: Instance> PwmChannel<GPT, ChannelB> {
    /// Drive the `GTIOCnB` pin.
    pub fn enable(&mut self) {
        critical_section::with(|_| self.regs().gtior().modify(|_, w| w.obe().set_bit()));
    }

    /// Release the `GTIOCnB` pin.
    pub fn disable(&mut self) {
        critical_section::with(|_| self.regs().gtior().modify(|_, w| w.obe().clear_bit()));
    }
}

impl<GPT, C> ErrorType for PwmChannel<GPT, C> {
    type Error = Error;
}

impl<GPT: Instance> SetDutyCycle for PwmChannel<GPT, ChannelA> {
    fn max_duty_cycle(&self) -> u16 {
        self.period
    }

    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        self.regs()
            .gtccra()
            .write(|w| unsafe { w.gtccra().bits(u32::from(duty.min(self.period))) });
        Ok(())
    }
}

impl<GPT: Instance> SetDutyCycle for PwmChannel<GPT, ChannelB> {
    fn max_duty_cycle(&self) -> u16 {
        self.period
    }

    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        self.regs()
            .gtccrb()
            .write(|w| unsafe { w.gtccrb().bits(u32::from(duty.min(self.period))) });
        Ok(())
    }
}

/// `GTCR.TPCS` values, in order of increasing division.
const TPCS: [(u8, u32); 6] = [
    (0b000, 1),
    (0b001, 4),
    (0b010, 16),
    (0b011, 64),
    (0b100, 256),
    (0b101, 1024),
];

/// Pick the finest prescaler whose period still fits in 16 bits.
///
/// Staying inside 16 bits keeps the duty resolution addressable by
/// `SetDutyCycle`'s `u16`, and works on the 16-bit GPT channels as well as the
/// 32-bit ones.
fn divider_for(pclkd: u32, frequency: u32) -> Option<(u8, u16)> {
    if frequency == 0 {
        return None;
    }
    for (tpcs, div) in TPCS {
        let counts = pclkd / div / frequency;
        if counts < 2 {
            return None; // too fast to have any duty resolution at all
        }
        if counts <= 0x1_0000 {
            return Some((tpcs, counts as u16));
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn picks_the_finest_prescaler_that_fits() {
        // 48 MHz / 1 kHz = 48000 counts, which fits with no prescaling.
        assert_eq!(divider_for(48_000_000, 1_000), Some((0b000, 48_000)));
        // 100 Hz needs 480000 counts, so the /16 prescaler (30000 counts).
        assert_eq!(divider_for(48_000_000, 100), Some((0b010, 30_000)));
    }

    #[test]
    fn resolution_is_never_below_one_bit() {
        // 24 MHz would give a single count per cycle: refuse rather than emit a
        // square wave with no controllable duty.
        assert!(divider_for(48_000_000, 48_000_000).is_none());
    }

    #[test]
    fn rejects_frequencies_below_the_slowest_prescaler() {
        // 48 MHz / 1024 / 65536 is about 0.72 Hz.
        assert!(divider_for(48_000_000, 0).is_none());
        assert!(divider_for(48_000_000, 1).is_some());
    }
}
