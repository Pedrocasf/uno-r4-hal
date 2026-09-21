//! Periodic timers and delays on the AGT peripherals.
//!
//! ```ignore
//! let mut timer = Timer::new(dp.agt0, &clocks);
//! timer.start(1.kHz());
//! nb::block!(timer.wait()).unwrap();
//! ```
//!
//! AGT is a 16-bit down-counter clocked from PCLKB with a /1, /2 or /8 prescaler. At
//! the stock 24 MHz PCLKB that gives periods from about 42 ns up to 21.8 ms in one
//! reload; [`Timer`] also implements [`DelayNs`], looping over as many reloads as a
//! longer delay needs.

use embedded_hal::delay::DelayNs;
use fugit::HertzU32 as Hertz;

use crate::clock::Clocks;
use crate::mstp::{self, ModuleStop};
use crate::pac;
use crate::spin_until;

/// Poll budget for the start/stop handshake, which takes a few PCLKB cycles.
const HANDSHAKE_TIMEOUT: u32 = 10_000;

/// Timer errors.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Error {
    /// The requested period is longer than one reload can cover at the slowest
    /// prescaler, or shorter than one count.
    UnsupportedPeriod,
}

/// An AGT channel.
///
/// # Safety
///
/// `ptr` must return the base address of a real AGT register block, and no two
/// implementations may return the same address.
pub unsafe trait Instance {
    /// This channel's module-stop bit.
    const MODULE: ModuleStop;

    /// Base address of the channel's registers.
    fn ptr() -> *const pac::agt0::RegisterBlock;
}

// SAFETY: `pac::Agt0::PTR` and `pac::Agt1::PTR` are the distinct, documented base
// addresses of the two AGT channels, which share a register block layout.
unsafe impl Instance for pac::Agt0 {
    const MODULE: ModuleStop = ModuleStop::AGT0;
    #[inline(always)]
    fn ptr() -> *const pac::agt0::RegisterBlock {
        pac::Agt0::PTR
    }
}

unsafe impl Instance for pac::Agt1 {
    const MODULE: ModuleStop = ModuleStop::AGT1;
    #[inline(always)]
    fn ptr() -> *const pac::agt0::RegisterBlock {
        pac::Agt1::PTR
    }
}

/// `AGTMR1.TCK` values that divide PCLKB.
///
/// The encoding is not monotonic: /2 is 0b011 and /8 is 0b001.
const TCK_BY_SHIFT: [(u8, u32); 3] = [(0b000, 1), (0b011, 2), (0b001, 8)];

/// A periodic 16-bit down-counter.
#[derive(Debug)]
pub struct Timer<AGT> {
    agt: AGT,
    pclkb: u32,
}

impl<AGT: Instance> Timer<AGT> {
    /// Take the channel and leave it stopped.
    pub fn new(agt: AGT, clocks: &Clocks) -> Self {
        mstp::start(AGT::MODULE);
        let this = Self {
            agt,
            pclkb: clocks.pclkb().raw(),
        };
        this.stop();
        this
    }

    /// Stop the channel and give the peripheral back.
    pub fn release(self) -> AGT {
        self.stop();
        mstp::stop(AGT::MODULE);
        self.agt
    }

    #[inline(always)]
    fn regs(&self) -> &'static pac::agt0::RegisterBlock {
        // SAFETY: `Timer` owns the peripheral token for this channel.
        unsafe { &*AGT::ptr() }
    }

    /// Start counting with the given period.
    ///
    /// Restarting an already running timer reloads it from the top.
    pub fn start(&mut self, frequency: Hertz) -> Result<(), Error> {
        let (tck, reload) = self.divider_for(frequency.raw()).ok_or(Error::UnsupportedPeriod)?;

        self.stop();

        // Timer mode, count from the reload value down to zero.
        self.regs().agtmr1().write(|w| unsafe {
            w.tmod().bits(0b000);
            w.tedgpl().clear_bit();
            w.tck().bits(tck)
        });
        self.regs().agtmr2().write(|w| unsafe { w.bits(0) });
        self.regs().agt().write(|w| unsafe { w.agt().bits(reload) });

        self.clear_underflow();
        self.regs().agtcr().write(|w| w.tstart().set_bit());
        // TCSTF follows TSTART once the counter is actually running.
        spin_until(HANDSHAKE_TIMEOUT, || {
            self.regs().agtcr().read().tcstf().bit_is_set()
        });
        Ok(())
    }

    /// Stop counting.
    pub fn stop(&self) {
        self.regs().agtcr().write(|w| w.tstop().set_bit());
        spin_until(HANDSHAKE_TIMEOUT, || {
            self.regs().agtcr().read().tcstf().bit_is_clear()
        });
    }

    /// Has the counter underflowed since the last call?
    ///
    /// Consumes the flag, so each period is reported exactly once.
    pub fn wait(&mut self) -> nb::Result<(), core::convert::Infallible> {
        if self.regs().agtcr().read().tundf().bit_is_set() {
            self.clear_underflow();
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    /// The current counter value.
    pub fn counter(&self) -> u16 {
        self.regs().agt().read().agt().bits()
    }

    /// Clear `AGTCR.TUNDF`.
    ///
    /// `AGTCR` mixes write-1 control bits (TSTART, TSTOP) with write-0-to-clear
    /// status flags, so this reads, clears the one flag and writes back without
    /// re-triggering a start or stop.
    fn clear_underflow(&self) {
        self.regs().agtcr().modify(|_, w| {
            w.tstart().clear_bit();
            w.tstop().clear_bit();
            w.tundf().clear_bit()
        });
    }

    /// Choose the finest prescaler whose reload value still fits in 16 bits.
    fn divider_for(&self, frequency: u32) -> Option<(u8, u16)> {
        if frequency == 0 {
            return None;
        }
        for (tck, div) in TCK_BY_SHIFT {
            let counts = self.pclkb / div / frequency;
            if counts == 0 {
                return None; // faster than the timer can count
            }
            if counts <= 0x1_0000 {
                // The counter underflows one cycle after reaching zero, so a reload
                // of N gives N+1 counts.
                return Some((tck, (counts - 1) as u16));
            }
        }
        None
    }

    /// Busy-wait for `ticks` PCLKB cycles using the /1 prescaler.
    fn delay_pclkb_ticks(&mut self, mut ticks: u64) {
        const MAX: u64 = 0x1_0000;
        if ticks == 0 {
            return;
        }
        self.stop();
        self.regs().agtmr1().write(|w| unsafe {
            w.tmod().bits(0b000);
            w.tedgpl().clear_bit();
            w.tck().bits(0b000) // PCLKB / 1
        });
        self.regs().agtmr2().write(|w| unsafe { w.bits(0) });

        while ticks != 0 {
            let chunk = ticks.min(MAX);
            self.regs()
                .agt()
                .write(|w| unsafe { w.agt().bits((chunk - 1) as u16) });
            self.clear_underflow();
            self.regs().agtcr().write(|w| w.tstart().set_bit());
            while self.regs().agtcr().read().tundf().bit_is_clear() {
                core::hint::spin_loop();
            }
            ticks -= chunk;
        }

        self.stop();
        self.clear_underflow();
    }
}

impl<AGT: Instance> DelayNs for Timer<AGT> {
    fn delay_ns(&mut self, ns: u32) {
        // Round up: a delay must never come back early.
        let ticks = (u64::from(ns) * u64::from(self.pclkb)).div_ceil(1_000_000_000);
        self.delay_pclkb_ticks(ticks);
    }

    fn delay_us(&mut self, us: u32) {
        let ticks = (u64::from(us) * u64::from(self.pclkb)).div_ceil(1_000_000);
        self.delay_pclkb_ticks(ticks);
    }

    fn delay_ms(&mut self, ms: u32) {
        let ticks = (u64::from(ms) * u64::from(self.pclkb)).div_ceil(1_000);
        self.delay_pclkb_ticks(ticks);
    }
}
