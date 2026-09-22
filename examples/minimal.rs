//! Smallest possible sign of life: toggle the LED with no clock configuration.
//!
//! A bisection aid rather than a demo. It runs at whatever clock the bootloader
//! left set up and busy-waits on a counted loop, so it exercises only
//! [`take_peripherals`](uno_r4_hal::take_peripherals) (which relocates `VTOR`) and
//! the GPIO layer. If this blinks but `blinky` does not, the fault is in
//! [`clock`](uno_r4_hal::clock) or [`delay`](uno_r4_hal::delay); if neither blinks,
//! it is in startup, the vector table, or the pin mapping.
//!
//! The blink is deliberately fast and hard-edged so it cannot be confused with the
//! bootloader's slow PWM fade on the same pin.
//!
//! ```sh
//! cargo build --example minimal --release
//! ```

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;

use uno_r4_hal::{board::minima, prelude::*};

#[entry]
fn main() -> ! {
    let dp = uno_r4_hal::take_peripherals().unwrap();

    let pins = minima::Pins::new(dp.port0, dp.port1, dp.port3, dp.port5);
    let mut led = pins.d13.into_push_pull_output();

    loop {
        led.toggle().unwrap();
        // No timer, no clock setup: just burn cycles. The exact period depends on
        // whatever ICLK the bootloader handed over, which is the point.
        for _ in 0..400_000 {
            cortex_m::asm::nop();
        }
    }
}
