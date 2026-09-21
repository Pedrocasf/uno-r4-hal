//! Brings up every driver in the crate at once: UART, I2C, SPI, ADC, PWM and a
//! hardware timer.
//!
//! The pin choices below are placeholders. Check the RA4M1 hardware manual's
//! multi-function pin table for the pins your SCI/IIC/SPI/GPT channel can actually
//! reach, and the Uno R4 schematic for where those land on the headers.
//!
//! `writeln!` here resolves to [`embedded_io::Write::write_fmt`], which the
//! prelude brings into scope. Importing `core::fmt::Write` as well would make the
//! call ambiguous; pick one.
//!
//! ```sh
//! cargo build --example peripherals --release
//! ```

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use fugit::RateExtU32;
use panic_halt as _;

use uno_r4_hal::{
    adc::{self, Adc},
    clock,
    gpio::{AltFunction, PinState},
    i2c::{self, I2c},
    prelude::*,
    pwm::Pwm,
    serial::{self, Serial},
    spi::{self, Spi},
    timer::Timer,
};

#[entry]
fn main() -> ! {
    let dp = uno_r4_hal::take_peripherals().unwrap();

    let clocks = clock::Config::uno_r4().freeze(dp.system);

    let p1 = dp.port1.split();
    let p4 = dp.port4.split();
    let p0 = dp.port0.split();

    // --- UART -------------------------------------------------------------------
    let tx = p1.p101.into_alternate(AltFunction::Sci1);
    let rx = p1.p102.into_alternate(AltFunction::Sci1);
    let mut serial = Serial::new(
        dp.sci0,
        (tx, rx),
        serial::Config::baud(115_200),
        &clocks,
    )
    .unwrap();
    writeln!(serial, "uno-r4-hal up at {} Hz", clocks.iclk().raw()).unwrap();

    // --- I2C --------------------------------------------------------------------
    let scl = p4.p400.into_alternate_open_drain(AltFunction::Iic);
    let sda = p4.p401.into_alternate_open_drain(AltFunction::Iic);
    let mut i2c = I2c::new(dp.iic0, (scl, sda), i2c::Config::standard(), &clocks).unwrap();

    // Probe the bus: a device that acknowledges its address answers a zero-length
    // write.
    for address in 0x08..0x78u8 {
        if i2c.write(address, &[]).is_ok() {
            writeln!(serial, "i2c device at {address:#04x}").unwrap();
        }
    }

    // --- SPI --------------------------------------------------------------------
    let sck = p1.p111.into_alternate(AltFunction::Spi);
    let mosi = p1.p112.into_alternate(AltFunction::Spi);
    let miso = p1.p110.into_alternate(AltFunction::Spi);
    let mut cs = p1.p103.into_push_pull_output_in_state(PinState::High);
    let mut spi = Spi::new(
        dp.spi0,
        (sck, mosi, miso),
        spi::Config::mode0(1_000_000),
        &clocks,
    )
    .unwrap();

    let mut frame = [0x9F, 0x00, 0x00, 0x00];
    cs.set_low().unwrap();
    spi.transfer_in_place(&mut frame).unwrap();
    cs.set_high().unwrap();
    writeln!(serial, "spi read {frame:?}").unwrap();

    // --- ADC --------------------------------------------------------------------
    let _a0 = p0.p000.into_analog();
    let mut adc = Adc::new(dp.adc140, adc::Config::default(), &clocks);

    // --- PWM --------------------------------------------------------------------
    let _pwm_pin = p1.p105.into_alternate(AltFunction::Gpt1);
    let pwm = Pwm::new(dp.gpt162, 1.kHz(), &clocks).unwrap();
    let (mut pwm_a, _pwm_b) = pwm.split();
    pwm_a.enable();

    // --- Timer ------------------------------------------------------------------
    let mut timer = Timer::new(dp.agt0, &clocks);

    loop {
        let raw = adc.read(adc::Channel::AN000).unwrap();
        // Mirror the pot on the PWM output.
        let duty = (u32::from(raw) * u32::from(pwm_a.max_duty_cycle())
            / u32::from(adc.max_value())) as u16;
        pwm_a.set_duty_cycle(duty).unwrap();

        writeln!(serial, "an000 = {raw}, duty = {duty}").unwrap();
        timer.delay_ms(250);
    }
}
