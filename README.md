# uno-r4-hal

An [`embedded-hal`] 1.0 implementation for the Renesas RA4M1 (Arduino Uno R4),
built on the [`ra4m1-pac`] peripheral access crate.

## What is implemented

| Module | Peripheral | Traits |
| --- | --- | --- |
| [`clock`] | SYSTEM (MOSC, HOCO, PLL, `SCKDIVCR`) | — |
| [`gpio`] | PORT0..PORT9, PFS | `InputPin`, `OutputPin`, `StatefulOutputPin` |
| [`delay`] | Cortex-M SysTick | `DelayNs` |
| [`serial`] | SCI0, SCI1, SCI2, SCI9 as UART | `embedded-io` `Read`/`Write`, `embedded-hal-nb` `serial`, `core::fmt::Write` |
| [`i2c`] | IIC0, IIC1 as master | `I2c` (7-bit, with repeated starts) |
| [`spi`] | SPI0, SPI1 as master | `SpiBus<u8>` |
| [`adc`] | ADC140 | inherent (`embedded-hal` 1.0 has no ADC trait) |
| [`timer`] | AGT0, AGT1 | `DelayNs`, periodic `wait` |
| [`pwm`] | GPT320/321, GPT162..167 | `SetDutyCycle` |
| [`mstp`] | module-stop (clock gating) | — |
| [`board`] | Arduino pin maps for the Minima and the WiFi | — |

All drivers are polled. Nothing here uses interrupts or DMA.

## Getting started

```rust
use uno_r4_hal::{clock, delay::Delay, prelude::*};

let dp = uno_r4_hal::take_peripherals().unwrap();
let cp = cortex_m::Peripherals::take().unwrap();

// 12 MHz crystal -> PLL x8 /2 -> 48 MHz ICLK.
let clocks = clock::Config::uno_r4().freeze(dp.system);
let mut delay = Delay::new(cp.SYST, &clocks);

let p1 = dp.port1.split();
let mut led = p1.p102.into_push_pull_output();

loop {
    led.toggle().unwrap();
    delay.delay_ms(500);
}
```

Build with the target already set in `.cargo/config.toml`:

```bash
cargo build --example blinky --release
```

The arithmetic-only parts (the baud rate, I2C bit rate, SPI bit rate and PWM
prescaler solvers) have unit tests that run on the host:

```bash
cargo test-host   # = cargo test --lib --target <your host triple>
```

## Things to know before you trust this on hardware

**None of this has been run on a board.** It compiles, links to a valid image, and
the divider solvers are unit tested, but no register sequence here has been
confirmed against real silicon.

**Pin assignments are yours to check, except on the headers.** For the Uno R4 header
pins, [`board`] records the MCU pin, the peripheral each bus lands on and the mux
group it needs. Anywhere else, `into_alternate` takes any
[`AltFunction`](gpio::AltFunction) for any pin, and which SCI, IIC, SPI or GPT
channel a given pin can actually reach is fixed in silicon and listed in the hardware
manual's multi-function pin table. The drivers take ownership of the pins you hand
them and require them to be in alternate mode, but they cannot check that you picked
pins the peripheral can see.

**The PAC has two SVD defects this crate works around**, both documented at the point
of use:

- The byte-addressable port register aliases (`podr`, `pdr`, `pidr`, `posr`, `porr`)
  each sit at their neighbour's offset. [`gpio`] uses the 32-bit `PCNTR1`/`PCNTR2`/
  `PCNTR3` views instead, whose fields are correct.
- The `PFS` and `ADDR16`+ accessors have wrong offsets. [`gpio`] and [`adc`] compute
  those addresses directly.

**`Peripherals::take` is not available from the PAC.** It is behind
`#[cfg(feature = "critical-section")]`, and the PAC's manifest declares no features,
so it is never compiled in. Use [`take_peripherals`] instead.

**No interrupt vector table.** The PAC's vector table is behind its own `rt` cfg,
which is likewise never enabled, so `#[interrupt]` handlers will not be wired up.
Fixing that needs a change in the PAC.

## Board pin maps

[`board::minima`] and [`board::wifi`] carry the Arduino silkscreen names, transcribed
from [ArduinoCore-renesas][core] (`variant.cpp` for the header mapping, `pinmux.inc`
for the per-pin peripheral capabilities).

**The two boards are not pin-compatible underneath.** `D0`, `D1`, `D8`, `D9` and all
the analog pins share MCU pins, but `D2`..`D7` and `D10`..`D13` do not, and `SPI` is
SPI1 on the Minima against SPI0 on the WiFi:

| | Minima | WiFi |
| --- | --- | --- |
| `LED_BUILTIN` (`D13`) | P111 | P102 |
| `SPI` MOSI/MISO/SCK | P109 / P110 / P111 (SPI1) | P411 / P410 / P102 (SPI0) |
| `Serial1` TX/RX | `D1`/`D0` = P302/P301 (SCI2) | `D22`/`D23` = P109/P110 (SCI9) |
| `Wire` SDA/SCL | `A4`/`A5` = P101/P100 (IIC1) | same |

Each module also has a `mux` submodule giving the exact `AltFunction` every bus
needs, and an `analog` submodule mapping the analog pins to ADC channels:

```rust
let pins = board::minima::Pins::new(dp.port0, dp.port1, dp.port3, dp.port5);
let tx = pins.d1.into_alternate(board::minima::mux::SERIAL1);
let raw = adc.read(board::minima::analog::A0)?;
```

Note that `SciGroup1`/`SciGroup2` are mux *groups*, not channel numbers: group 1
usually carries the even-numbered SCI channels and group 2 the odd ones, but a few
Uno R4 pins break that pattern, which is why the `mux` constants are transcribed
per pin rather than derived from a rule.

`Pins::new` consumes whole ports, so any pin it does not name becomes unreachable.
Split the ports yourself with `GpioExt` if you need one of those; the type aliases
(`minima::D13<Output<PushPull>>`) work either way.

[core]: https://github.com/arduino/ArduinoCore-renesas

## Memory layout

`memory.x` targets the **stock Arduino bootloader**: the image is linked at
`0x4000`, with 240 KB of flash above it and 32 KB of SRAM at `0x2000_0000`. Those
match `FLASH_IMAGE_START`, `FLASH_LENGTH` and `RAM_LENGTH` in
[ArduinoCore-renesas, `variants/MINIMA/memory_regions.ld`][mem].

Because the image no longer starts at 0, the vector table is not where the core
looks for it out of reset, and `cortex-m-rt` does not move it. Until `SCB.VTOR` is
updated, every exception and interrupt vectors into the *bootloader's* table.
[`take_peripherals`] calls [`relocate_vector_table`] to fix that before it hands
back the peripherals; Renesas' own FSP does the same thing in `SystemInit`. If you
skip `take_peripherals`, call `relocate_vector_table` yourself before enabling any
interrupt.

The option-setting memory (OFS0, OFS1 and the security-MPU settings at
`0x400..0x440`) belongs to the bootloader in this layout, not to the sketch —
Arduino's own linker script gives the application an `OPTION_SETTING` region of
length 0 for the same reason.

### Flashing over SWD instead

Switch `memory.x` to the bare-metal layout it documents:

```
FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 256K
```

and add `_stext = ORIGIN(FLASH) + 0x440;`. That second line matters: the RA4M1 reads
its option-setting memory out of flash at `0x400..0x440` during reset, before the
first instruction runs, and `cortex-m-rt` reserves a full kilobyte for the vector
table — so without the nudge `.text` begins at exactly `0x400` and the boot sequence
reads your first instructions as option settings. An arbitrary OFS0 can start the
independent watchdog with a short timeout and reset the part in a loop. Leaving the
region erased gives the factory defaults, which is what this HAL expects: watchdogs
stopped, HOCO stopped after reset, voltage detection off.

[mem]: https://github.com/arduino/ArduinoCore-renesas/blob/main/variants/MINIMA/memory_regions.ld

## Not implemented

CAN, USB, RTC, DAC, CTSU, the comparators, the data flash, DTC/DMAC, and interrupt- or
DMA-driven variants of the drivers that are here.

## Licence

MIT OR Apache-2.0.

[`embedded-hal`]: https://docs.rs/embedded-hal/1.0.0/embedded_hal/
[`ra4m1-pac`]: https://github.com/Pedro-Starling-F/ra4m1-pac
