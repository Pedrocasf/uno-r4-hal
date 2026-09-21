//! A [`embedded-hal`] 1.0 implementation for the Renesas RA4M1 (Arduino Uno R4).
//!
//! Built on top of the [`ra4m1_pac`] peripheral access crate, which is re-exported
//! as [`pac`].
//!
//! # Getting started
//!
//! ```ignore
//! use uno_r4_hal::{clock, delay::Delay, prelude::*};
//!
//! let dp = uno_r4_hal::take_peripherals().unwrap();
//! let cp = cortex_m::Peripherals::take().unwrap();
//!
//! // 12 MHz crystal -> PLL -> 48 MHz ICLK.
//! let clocks = clock::Config::uno_r4().freeze(dp.system);
//! let mut delay = Delay::new(cp.SYST, &clocks);
//!
//! let p1 = dp.port1.split();
//! let mut led = p1.p102.into_push_pull_output();
//! loop {
//!     led.set_high().unwrap();
//!     delay.delay_ms(500);
//!     led.set_low().unwrap();
//!     delay.delay_ms(500);
//! }
//! ```
//!
//! # A note on the PAC's port registers
//!
//! The SVD this PAC was generated from mis-describes the byte-addressable aliases of
//! the I/O port registers (`PODR`/`PDR`, `PIDR`, `POSR`/`PORR` are all swapped with
//! their neighbour). The 32-bit `PCNTR1`/`PCNTR2`/`PCNTR3` views *are* correct, so
//! [`gpio`] drives the ports exclusively through those. Don't reach for
//! `pac::Port1::posr()` and friends directly — they do not do what their names say.
//!
//! # Memory layout
//!
//! The bundled `memory.x` links the image at 0x4000, where the stock Arduino
//! bootloader loads a sketch, and [`take_peripherals`] moves the vector table there
//! with [`relocate_vector_table`]. To flash over SWD with no bootloader instead,
//! switch `memory.x` to the bare-metal layout it documents.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal/1.0.0/embedded_hal/

#![no_std]
#![deny(missing_debug_implementations)]
#![warn(missing_docs)]

pub use ra4m1_pac as pac;

pub mod adc;
pub mod clock;
pub mod delay;
pub mod gpio;
pub mod i2c;
pub mod mstp;
pub mod pwm;
pub mod serial;
pub mod spi;
pub mod timer;

pub mod prelude {
    //! Re-exports of the traits you almost always want in scope.

    pub use embedded_hal::delay::DelayNs as _;
    pub use embedded_hal::digital::{InputPin as _, OutputPin as _, StatefulOutputPin as _};
    pub use embedded_hal::i2c::I2c as _;
    pub use embedded_hal::pwm::SetDutyCycle as _;
    pub use embedded_hal::spi::SpiBus as _;
    pub use embedded_hal_nb::serial::{Read as _, Write as _};
    pub use embedded_io::{Read as _, ReadReady as _, Write as _, WriteReady as _};

    pub use crate::clock::ClockConfigExt as _;
    pub use crate::gpio::GpioExt as _;
}

/// Convenience alias for a frequency in Hz.
pub type Hertz = fugit::HertzU32;

/// Point `SCB.VTOR` at this image's vector table.
///
/// The default [`memory.x`] links the image at 0x4000, where the stock Arduino
/// bootloader loads a sketch. The Cortex-M core fetches its vector table from
/// address 0 out of reset, and `cortex-m-rt` does not move it, so until `VTOR` is
/// updated every exception and interrupt vectors into the *bootloader's* table.
/// Renesas' own FSP does the same thing in `SystemInit`.
///
/// [`take_peripherals`] calls this, so most programs never need it directly. Call it
/// yourself if you skip `take_peripherals`, and call it before enabling any
/// interrupt.
///
/// On a bare-metal image linked at 0 this writes the value the core already has, so
/// it is harmless either way.
///
/// [`memory.x`]: https://github.com/Pedro-Starling-F/uno-r4-hal/blob/master/memory.x
#[inline]
pub fn relocate_vector_table() {
    // Defined by `cortex-m-rt`'s linker script at the start of `.vector_table`.
    unsafe extern "C" {
        static __vector_table: u32;
    }

    // Only the address is taken, never a read, so this needs no `unsafe`. The
    // linker has already asserted the section is aligned well enough for VTOR:
    // `link.x` requires it be aligned to at least its own size rounded up to a
    // power of two, and to at least 128 bytes.
    let base = (&raw const __vector_table) as u32;

    // SAFETY: writing our own correctly aligned table address is the documented use
    // of VTOR. `SCB` is a fixed-address core peripheral, so stealing a reference to
    // it cannot alias anything the caller owns.
    unsafe {
        let scb = &*cortex_m::peripheral::SCB::PTR;
        scb.vtor.write(base);
    }
}

/// Take the peripheral singletons, once.
///
/// Returns `None` on every call after the first, so the ownership guarantee the rest
/// of the HAL relies on holds.
///
/// Also calls [`relocate_vector_table`], because this is the first thing a program
/// does and the vector table has to be pointed at this image before any interrupt
/// can fire. See that function for why.
///
/// The PAC has its own `Peripherals::take`, but it is behind
/// `#[cfg(feature = "critical-section")]` and the PAC's manifest never declares that
/// feature, so it is never compiled in. Use this instead.
pub fn take_peripherals() -> Option<pac::Peripherals> {
    use core::sync::atomic::{AtomicBool, Ordering};

    static TAKEN: AtomicBool = AtomicBool::new(false);

    if TAKEN.swap(true, Ordering::AcqRel) {
        None
    } else {
        relocate_vector_table();
        // SAFETY: the swap above succeeded, so this is the first call and no other
        // `Peripherals` exists.
        Some(unsafe { pac::Peripherals::steal() })
    }
}

/// Spin until `cond` returns `true`, giving up after `limit` polls.
///
/// Returns `false` on timeout. Used by the peripheral drivers so that a stuck bus
/// can't wedge the whole program.
#[inline]
pub(crate) fn spin_until<F: FnMut() -> bool>(limit: u32, mut cond: F) -> bool {
    for _ in 0..limit {
        if cond() {
            return true;
        }
        core::hint::spin_loop();
    }
    cond()
}
