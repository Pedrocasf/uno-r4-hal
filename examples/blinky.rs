//! Blink the built-in LED on an Uno R4 Minima, faster while `D2` is pulled low.
//!
//! `LED_BUILTIN` is `D13`, which is P111 on the Minima. On a WiFi it is P102, so
//! swap `board::minima` for `board::wifi` and the pin map follows.
//!
//! ```sh
//! cargo build --example blinky --release
//! ```

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;

use uno_r4_hal::{board::minima, clock, delay::Delay, prelude::*};

#[entry]
fn main() -> ! {
    let dp = uno_r4_hal::take_peripherals().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let clocks = clock::Config::uno_r4().freeze(dp.system).unwrap();
    let mut delay = Delay::new(cp.SYST, &clocks);

    let pins = minima::Pins::new(dp.port0, dp.port1, dp.port3, dp.port5);
    let mut led = pins.d13.into_push_pull_output();
    let mut button = pins.d2.into_pull_up_input();

    loop {
        let period = if button.is_low().unwrap() { 100 } else { 500 };
        led.toggle().unwrap();
        delay.delay_ms(period);
    }
}
