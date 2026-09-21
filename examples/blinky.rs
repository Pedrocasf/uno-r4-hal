//! Blink a LED on P102 and read a button on P105.
//!
//! ```sh
//! cargo build --example blinky --release
//! ```

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;

use uno_r4_hal::{clock, delay::Delay, prelude::*};

#[entry]
fn main() -> ! {
    let dp = uno_r4_hal::take_peripherals().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let clocks = clock::Config::uno_r4().freeze(dp.system);
    let mut delay = Delay::new(cp.SYST, &clocks);

    let p1 = dp.port1.split();
    let mut led = p1.p102.into_push_pull_output();
    let mut button = p1.p105.into_pull_up_input();

    loop {
        led.set_high().unwrap();
        delay.delay_ms(if button.is_low().unwrap() { 100 } else { 500 });
        led.set_low().unwrap();
        delay.delay_ms(if button.is_low().unwrap() { 100 } else { 500 });
    }
}
