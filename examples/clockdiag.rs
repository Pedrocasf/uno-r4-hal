//! Report what [`clock::Config::freeze`] does, on the LED.
//!
//! A diagnostic for boards with no debugger and no serial adapter. The LED is set
//! up *before* the clock is touched, and the result is signalled as a blink count
//! rather than a panic, so a failing `freeze` is still observable.
//!
//! - Continuous fast blink: `freeze` succeeded, and the fault is elsewhere.
//! - N blinks, long pause, repeat: `freeze` returned an error, where N is
//!   - 1 `MainOscTimeout`
//!   - 2 `HocoTimeout`
//!   - 3 `PllTimeout`
//!   - 4 `PowerModeTimeout`
//!   - 5 `SwitchTimeout`
//!   - 6 `MissingPllConfig`
//!   - 7 `InvalidPllMultiplier`
//!
//! Timing comes from counted loops, not from a timer, so it works whatever the
//! clock ends up at.
//!
//! ```sh
//! cargo build --example clockdiag --release
//! ```

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;

use uno_r4_hal::{board::minima, clock, prelude::*};

/// Roughly 0.12 s at 48 MHz. Only has to be visible, not accurate.
const SHORT: u32 = 1_500_000;
/// Long enough to separate one repetition of the code from the next.
const GAP: u32 = 9_000_000;

fn spin(n: u32) {
    for _ in 0..n {
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    let dp = uno_r4_hal::take_peripherals().unwrap();

    // The LED comes first: whatever the clock does next, we can still report it.
    let pins = minima::Pins::new(dp.port0, dp.port1, dp.port3, dp.port5);
    let mut led = pins.d13.into_push_pull_output();

    let code = match clock::Config::uno_r4().freeze(dp.system) {
        Ok(_) => 0,
        Err(clock::Error::MainOscTimeout) => 1,
        Err(clock::Error::HocoTimeout) => 2,
        Err(clock::Error::PllTimeout) => 3,
        Err(clock::Error::PowerModeTimeout) => 4,
        Err(clock::Error::SwitchTimeout) => 5,
        Err(clock::Error::MissingPllConfig) => 6,
        Err(clock::Error::InvalidPllMultiplier) => 7,
    };

    loop {
        if code == 0 {
            led.toggle().unwrap();
            spin(SHORT / 3);
        } else {
            for _ in 0..code {
                led.set_high().unwrap();
                spin(SHORT);
                led.set_low().unwrap();
                spin(SHORT);
            }
            spin(GAP);
        }
    }
}
