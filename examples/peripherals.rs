//! Brings up every driver in the crate at once on an Uno R4 Minima, using the real
//! header pins from [`uno_r4_hal::board::minima`].
//!
//! The peripheral behind each Arduino bus name is fixed by the board wiring:
//! `Serial1` is SCI2, `Wire` is IIC1, `SPI` is SPI1. The `mux` constants say which
//! `AltFunction` each of those pins needs.
//!
//! `SCK` and `LED_BUILTIN` are the same pin (`D13`), so this uses the SPI bus and
//! leaves the LED alone.
//!
//! `writeln!` here resolves to [`embedded_io::Write::write_fmt`], which the prelude
//! brings into scope. Importing `core::fmt::Write` as well would make the call
//! ambiguous; pick one.
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
    board::minima::{self, mux},
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

    let clocks = clock::Config::uno_r4().freeze(dp.system).unwrap();
    let pins = minima::Pins::new(dp.port0, dp.port1, dp.port3, dp.port5);

    // --- Serial1: SCI2 on D1/D0 -------------------------------------------------
    let tx = pins.d1.into_alternate(mux::SERIAL1);
    let rx = pins.d0.into_alternate(mux::SERIAL1);
    let mut serial = Serial::new(dp.sci2, (tx, rx), serial::Config::baud(115_200), &clocks)
        .unwrap();
    writeln!(serial, "uno-r4-hal up at {} Hz", clocks.iclk().to_raw()).unwrap();

    // --- Wire: IIC1 on A4/A5 ----------------------------------------------------
    let sda = pins.a4.into_alternate_open_drain(mux::WIRE);
    let scl = pins.a5.into_alternate_open_drain(mux::WIRE);
    let mut i2c = I2c::new(dp.iic1, (scl, sda), i2c::Config::standard(), &clocks).unwrap();

    // A device that acknowledges its address answers a zero-length write.
    for address in 0x08..0x78u8 {
        if i2c.write(address, &[]).is_ok() {
            writeln!(serial, "i2c device at {address:#04x}").unwrap();
        }
    }

    // --- SPI: SPI1 on D11/D12/D13, chip select on D10 ---------------------------
    let mosi = pins.d11.into_alternate(mux::SPI);
    let miso = pins.d12.into_alternate(mux::SPI);
    let sck = pins.d13.into_alternate(mux::SPI);
    // D10 is Arduino's SS, but the RA4M1 cannot drive it as SSL, so it is an
    // ordinary output.
    let mut cs = pins.d10.into_push_pull_output_in_state(PinState::High);
    let mut spi = Spi::new(
        dp.spi1,
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

    // --- ADC on A0 --------------------------------------------------------------
    let _a0 = pins.a0.into_analog();
    let mut adc = Adc::new(dp.adc140, adc::Config::default(), &clocks);

    // --- PWM: D3 is GPT channel 1, output B -------------------------------------
    let _pwm_pin = pins.d3.into_alternate(AltFunction::GptGroup2);
    let pwm = Pwm::new(dp.gpt321, 1.kHz(), &clocks).unwrap();
    let (_pwm_a, mut pwm_b) = pwm.split();
    pwm_b.enable();

    // --- Timer ------------------------------------------------------------------
    let mut timer = Timer::new(dp.agt0, &clocks);

    loop {
        let raw = adc.read(minima::analog::A0).unwrap();
        // Mirror the pot on the PWM output.
        let duty = (u32::from(raw) * u32::from(pwm_b.max_duty_cycle())
            / u32::from(adc.max_value())) as u16;
        pwm_b.set_duty_cycle(duty).unwrap();

        writeln!(serial, "A0 = {raw}, duty = {duty}").unwrap();
        timer.delay_ms(250);
    }
}
