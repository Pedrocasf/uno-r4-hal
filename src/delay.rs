//! Blocking delays.
//!
//! [`Delay`] drives the Cortex-M SysTick counter, which is clocked from ICLK. It is
//! the cheapest accurate delay available and needs no peripheral, but it takes
//! exclusive ownership of SysTick, so it cannot coexist with an RTOS tick. If you
//! need SysTick for something else, [`crate::timer::Timer`] offers the same
//! `DelayNs` implementation on an AGT channel instead.

use cortex_m::peripheral::SYST;
use cortex_m::peripheral::syst::SystClkSource;
use embedded_hal::delay::DelayNs;

use crate::clock::Clocks;

/// SysTick is a 24-bit down-counter.
const MAX_RELOAD: u32 = 0x00FF_FFFF;

/// A blocking delay backed by SysTick.
pub struct Delay {
    syst: SYST,
    iclk: u32,
}

// `SYST` is an opaque register-block token with no `Debug`, so name it rather than
// deriving through it.
impl core::fmt::Debug for Delay {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Delay").field("iclk", &self.iclk).finish()
    }
}

impl Delay {
    /// Take over SysTick and clock it from the core clock.
    pub fn new(mut syst: SYST, clocks: &Clocks) -> Self {
        syst.set_clock_source(SystClkSource::Core);
        syst.disable_counter();
        syst.disable_interrupt();
        Self {
            syst,
            iclk: clocks.iclk().to_raw(),
        }
    }

    /// Give SysTick back, leaving the counter stopped.
    pub fn free(mut self) -> SYST {
        self.syst.disable_counter();
        self.syst
    }

    /// Block for exactly `ticks` core clock cycles.
    ///
    /// Split into chunks because the reload value is only 24 bits wide.
    fn delay_ticks(&mut self, mut ticks: u64) {
        while ticks != 0 {
            // `- 1` because the counter reloads on the tick *after* it hits zero, so
            // a reload of N produces N+1 cycles.
            let chunk = ticks.min(MAX_RELOAD as u64) as u32;
            self.syst.set_reload(chunk - 1);
            self.syst.clear_current();
            self.syst.enable_counter();
            // COUNTFLAG latches when the counter reaches zero and clears on read.
            while !self.syst.has_wrapped() {
                core::hint::spin_loop();
            }
            self.syst.disable_counter();
            ticks -= chunk as u64;
        }
    }
}

impl DelayNs for Delay {
    fn delay_ns(&mut self, ns: u32) {
        // 64-bit intermediate: at 48 MHz, `ns * iclk` overflows u32 above ~90 ns.
        // Round up so a requested delay is never shorter than asked for, which is
        // what callers waiting on a peripheral timing requirement need.
        let ticks = (u64::from(ns) * u64::from(self.iclk)).div_ceil(1_000_000_000);
        self.delay_ticks(ticks);
    }

    fn delay_us(&mut self, us: u32) {
        let ticks = (u64::from(us) * u64::from(self.iclk)).div_ceil(1_000_000);
        self.delay_ticks(ticks);
    }

    fn delay_ms(&mut self, ms: u32) {
        let ticks = (u64::from(ms) * u64::from(self.iclk)).div_ceil(1_000);
        self.delay_ticks(ticks);
    }
}
