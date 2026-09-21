//! Arduino Uno R4 board pin maps.
//!
//! The silkscreen names (`D0`..`D13`, `A0`..`A5`) are a board-level fact, not an MCU
//! one, so they live here rather than in [`gpio`](crate::gpio). Each board gets a
//! module: [`minima`] and [`wifi`].
//!
//! **The two boards are not pin-compatible underneath.** `D0`, `D1`, `D8`, `D9` and
//! every analog pin land on the same MCU pins, but `D2`..`D7` and `D10`..`D13` do
//! not, and the SPI bus is on different silicon: SPI1 on the Minima, SPI0 on the
//! WiFi. Code written against one module will not work on the other board.
//!
//! # Two ways to use this
//!
//! [`minima::Pins::new`] is the convenience path. It takes the PAC port peripherals, splits
//! them and hands back everything named:
//!
//! ```ignore
//! use uno_r4_hal::board::minima::Pins;
//!
//! let pins = Pins::new(dp.port0, dp.port1, dp.port3, dp.port5);
//! let mut led = pins.d13.into_push_pull_output();
//! ```
//!
//! That consumes whole ports, so any pin this struct does not name becomes
//! unreachable. If you need one of those, split the ports yourself with
//! [`GpioExt`](crate::gpio::GpioExt) and use the type aliases to label what you care
//! about:
//!
//! ```ignore
//! use uno_r4_hal::{board::minima, gpio::{GpioExt, Output, PushPull}};
//!
//! let p1 = dp.port1.split();
//! let led: minima::D13<Output<PushPull>> = p1.p111.into_push_pull_output();
//! let spare = p1.p108; // not on a header, still available
//! ```
//!
//! # Source
//!
//! Transcribed from [ArduinoCore-renesas], `variants/MINIMA` and
//! `variants/UNOWIFIR4`: `variant.cpp` for the header mapping and `pinmux.inc` for
//! the per-pin peripheral capabilities quoted in the docs below.
//!
//! [ArduinoCore-renesas]: https://github.com/arduino/ArduinoCore-renesas

/// Build a board's type aliases and `Pins` struct from a pin list.
macro_rules! board {
    (
        ports: [ $( $pf:ident : $PortTy:ident ),+ $(,)? ],
        pins: [ $(
            $(#[$pdoc:meta])*
            $field:ident : $Alias:ident = $srcport:ident . $srcpin:ident @ ($p:literal, $n:literal)
        ),+ $(,)?
        ] $(,)?
    ) => {
        $(
            $(#[$pdoc])*
            pub type $Alias<MODE = $crate::gpio::Input<$crate::gpio::Floating>> =
                $crate::gpio::Pin<$p, $n, MODE>;
        )+

        /// Every pin this board names, in its reset state (floating input).
        #[derive(Debug)]
        pub struct Pins {
            $(
                $(#[$pdoc])*
                pub $field: $Alias,
            )+
        }

        impl Pins {
            /// Split the ports this board uses and label their pins.
            ///
            /// Consumes the ports whole, so pins this struct does not name are no
            /// longer reachable. Split the ports yourself if you need one of them.
            pub fn new($( $pf: $crate::pac::$PortTy ),+) -> Self {
                use $crate::gpio::GpioExt as _;
                $( let $pf = $pf.split(); )+
                Pins { $( $field: $srcport.$srcpin, )+ }
            }
        }
    };
}

pub mod minima {
    //! Arduino Uno R4 Minima.
    //!
    //! # Buses
    //!
    //! | Arduino name | Pins | Peripheral |
    //! | --- | --- | --- |
    //! | `Serial1` | `D1` TX, `D0` RX | SCI2 |
    //! | `Wire` | `A4` SDA, `A5` SCL | IIC1 |
    //! | `SPI` | `D11` MOSI, `D12` MISO, `D13` SCK | SPI1 |
    //!
    //! `SPI` has no hardware chip select on the header: Arduino's `SS` is `D10`,
    //! which the RA4M1 cannot drive as `SSL`, so drive it as an ordinary output.
    //!
    //! Note that `SCK` and `LED_BUILTIN` are both `D13` — the long-standing Arduino
    //! conflict. Using the SPI bus makes the LED flicker, and driving the LED
    //! corrupts SPI traffic.

    board! {
        ports: [port0: Port0, port1: Port1, port3: Port3, port5: Port5],
        pins: [
            /// `D0` — P301. RX of `Serial1` (SCI2). GPT4B, IRQ6.
            d0: D0 = port3.p301 @ (3, 1),
            /// `D1` — P302. TX of `Serial1` (SCI2). GPT4A, IRQ5.
            d1: D1 = port3.p302 @ (3, 2),
            /// `D2` — P105. GPT1A, IRQ0.
            d2: D2 = port1.p105 @ (1, 5),
            /// `D3` — P104, PWM capable. GPT1B, IRQ1, SCI0 RXD.
            d3: D3 = port1.p104 @ (1, 4),
            /// `D4` — P103. AN019, CAN0 TX, GPT2A, SCI0 CTS.
            d4: D4 = port1.p103 @ (1, 3),
            /// `D5` — P102, PWM capable. AN020, CAN0 RX, GPT2B, SCI0 SCK,
            /// SCI2 TXD, SPI0 SCK.
            d5: D5 = port1.p102 @ (1, 2),
            /// `D6` — P106, PWM capable. GPT0B.
            d6: D6 = port1.p106 @ (1, 6),
            /// `D7` — P107. GPT0A.
            d7: D7 = port1.p107 @ (1, 7),
            /// `D8` — P304. GPT7A, IRQ9.
            d8: D8 = port3.p304 @ (3, 4),
            /// `D9` — P303, PWM capable. GPT7B.
            d9: D9 = port3.p303 @ (3, 3),
            /// `D10` — P112, PWM capable. Arduino's `SS`, but software-driven: this
            /// pin has no `SSL` function. GPT3B, SCI1 SCK, SCI2 TXD.
            d10: D10 = port1.p112 @ (1, 12),
            /// `D11` — P109, PWM capable. `MOSI` (SPI1). CAN0 TX, GPT1A, SCI1 SCK,
            /// SCI9 TXD.
            d11: D11 = port1.p109 @ (1, 9),
            /// `D12` — P110. `MISO` (SPI1). CAN0 RX, GPT1B, IRQ3, SCI2 CTS,
            /// SCI9 RXD.
            d12: D12 = port1.p110 @ (1, 10),
            /// `D13` — P111. `SCK` (SPI1) *and* `LED_BUILTIN`; they conflict.
            /// GPT3A, IRQ4, SCI2 SCK, SCI9 SCK.
            d13: D13 = port1.p111 @ (1, 11),

            /// `A0` — P014. AN009, and the DAC12 output.
            a0: A0 = port0.p014 @ (0, 14),
            /// `A1` — P000. AN000, IRQ6.
            a1: A1 = port0.p000 @ (0, 0),
            /// `A2` — P001. AN001, IRQ7.
            a2: A2 = port0.p001 @ (0, 1),
            /// `A3` — P002. AN002, IRQ2.
            a3: A3 = port0.p002 @ (0, 2),
            /// `A4` — P101. `SDA` (IIC1). AN021, GPT5A, IRQ1, SCI0 TXD, SPI0 MOSI.
            a4: A4 = port1.p101 @ (1, 1),
            /// `A5` — P100. `SCL` (IIC1). AN022, GPT5B, IRQ2, SCI0 RXD, SPI0 MISO.
            a5: A5 = port1.p100 @ (1, 0),

            /// P500 — the on-board AVCC divider, AN016. Not on a header.
            vcc_measure: VccMeasure = port5.p500 @ (5, 0),
            /// P012 — the TX indicator LED. Not on a header.
            led_tx: LedTx = port0.p012 @ (0, 12),
            /// P013 — the RX indicator LED. Not on a header.
            led_rx: LedRx = port0.p013 @ (0, 13),
            /// P501 — TX on the SWD connector. SCI1 TXD.
            swd_tx: SwdTx = port5.p501 @ (5, 1),
            /// P502 — RX on the SWD connector. SCI1 RXD.
            swd_rx: SwdRx = port5.p502 @ (5, 2),
            /// P108 — SWDIO. Taking this over costs you the debug connection.
            swdio: Swdio = port1.p108 @ (1, 8),
            /// P300 — SWCLK. Taking this over costs you the debug connection.
            swclk: Swclk = port3.p300 @ (3, 0),
        ],
    }

    /// The on-board LED, on `D13`. Shared with `SCK`.
    pub type LedBuiltin<MODE = crate::gpio::Input<crate::gpio::Floating>> = D13<MODE>;
    /// `SDA` of `Wire` (IIC1), on `A4`.
    pub type Sda<MODE = crate::gpio::Input<crate::gpio::Floating>> = A4<MODE>;
    /// `SCL` of `Wire` (IIC1), on `A5`.
    pub type Scl<MODE = crate::gpio::Input<crate::gpio::Floating>> = A5<MODE>;
    /// `MOSI` of `SPI` (SPI1), on `D11`.
    pub type Mosi<MODE = crate::gpio::Input<crate::gpio::Floating>> = D11<MODE>;
    /// `MISO` of `SPI` (SPI1), on `D12`.
    pub type Miso<MODE = crate::gpio::Input<crate::gpio::Floating>> = D12<MODE>;
    /// `SCK` of `SPI` (SPI1), on `D13`. Shared with the LED.
    pub type Sck<MODE = crate::gpio::Input<crate::gpio::Floating>> = D13<MODE>;
    /// Arduino's `SS`/`CS`, on `D10`. Software-driven; not a hardware `SSL`.
    pub type Cs<MODE = crate::gpio::Input<crate::gpio::Floating>> = D10<MODE>;
    /// TX of `Serial1` (SCI2), on `D1`.
    pub type Tx<MODE = crate::gpio::Input<crate::gpio::Floating>> = D1<MODE>;
    /// RX of `Serial1` (SCI2), on `D0`.
    pub type Rx<MODE = crate::gpio::Input<crate::gpio::Floating>> = D0<MODE>;

    /// The [`AltFunction`](crate::gpio::AltFunction) each bus needs.
    ///
    /// Taken from the Arduino variant's per-pin mux table, not inferred. Pass these
    /// to [`into_alternate`](crate::gpio::Pin::into_alternate) (or
    /// [`into_alternate_open_drain`](crate::gpio::Pin::into_alternate_open_drain)
    /// for I2C) so you do not have to read the MPC table yourself.
    pub mod mux {
        use crate::gpio::AltFunction;

        /// `Serial1` on `D0`/`D1`, which is SCI2.
        pub const SERIAL1: AltFunction = AltFunction::SciGroup1;
        /// `Wire` on `A4`/`A5`, which is IIC1.
        pub const WIRE: AltFunction = AltFunction::Iic;
        /// `SPI` on `D11`/`D12`/`D13`, which is SPI1.
        pub const SPI: AltFunction = AltFunction::Spi;
    }

    /// ADC channel for each analog-capable header pin.
    ///
    /// Pass these to [`Adc::read`](crate::adc::Adc::read) after putting the matching
    /// pin in [`Analog`](crate::gpio::Analog) mode with
    /// [`into_analog`](crate::gpio::Pin::into_analog).
    pub mod analog {
        use crate::adc::Channel;

        /// `A0` (P014).
        pub const A0: Channel = Channel::AN009;
        /// `A1` (P000).
        pub const A1: Channel = Channel::AN000;
        /// `A2` (P001).
        pub const A2: Channel = Channel::AN001;
        /// `A3` (P002).
        pub const A3: Channel = Channel::AN002;
        /// `A4` (P101), if you are not using it for I2C.
        pub const A4: Channel = Channel::AN021;
        /// `A5` (P100), if you are not using it for I2C.
        pub const A5: Channel = Channel::AN022;
        /// `D4` (P103), which is also analog capable.
        pub const D4: Channel = Channel::AN019;
        /// `D5` (P102), which is also analog capable.
        pub const D5: Channel = Channel::AN020;
        /// The on-board AVCC divider (P500).
        pub const VCC_MEASURE: Channel = Channel::AN016;
    }
}

pub mod wifi {
    //! Arduino Uno R4 WiFi.
    //!
    //! # Buses
    //!
    //! | Arduino name | Pins | Peripheral |
    //! | --- | --- | --- |
    //! | `Serial1` | `D22` TX, `D23` RX | SCI9 |
    //! | `Serial2` | `D1` TX, `D0` RX | SCI2 |
    //! | `Serial3` | `D24` TX, `D25` RX (to the ESP32) | SCI1 |
    //! | `Wire` | `A4` SDA, `A5` SCL | IIC1 |
    //! | `Wire1` | `D27` SDA, `D26` SCL (Qwiic connector) | IIC0 |
    //! | `SPI` | `D11` MOSI, `D12` MISO, `D13` SCK | SPI0 |
    //!
    //! `SPI` has no hardware chip select on the header: Arduino's `SS` is `D10`,
    //! which the RA4M1 cannot drive as `SSL`, so drive it as an ordinary output.
    //!
    //! `SCK` and `LED_BUILTIN` are both `D13`, so they conflict.
    //!
    //! `D28`..`D38` drive the charlieplexed 12x8 LED matrix and are not on headers.

    board! {
        ports: [port0: Port0, port1: Port1, port2: Port2, port3: Port3, port4: Port4, port5: Port5],
        pins: [
            /// `D0` — P301. RX of `Serial2` (SCI2). GPT4B, IRQ6.
            d0: D0 = port3.p301 @ (3, 1),
            /// `D1` — P302. TX of `Serial2` (SCI2). GPT4A, IRQ5.
            d1: D1 = port3.p302 @ (3, 2),
            /// `D2` — P104. GPT1B, IRQ1, SCI0 RXD.
            d2: D2 = port1.p104 @ (1, 4),
            /// `D3` — P105, PWM capable. GPT1A, IRQ0.
            d3: D3 = port1.p105 @ (1, 5),
            /// `D4` — P106. GPT0B.
            d4: D4 = port1.p106 @ (1, 6),
            /// `D5` — P107, PWM capable. GPT0A.
            d5: D5 = port1.p107 @ (1, 7),
            /// `D6` — P111, PWM capable. GPT3A, IRQ4, SCI2 SCK, SCI9 SCK, SPI1 SCK.
            d6: D6 = port1.p111 @ (1, 11),
            /// `D7` — P112. GPT3B, SCI1 SCK, SCI2 TXD.
            d7: D7 = port1.p112 @ (1, 12),
            /// `D8` — P304. GPT7A, IRQ9.
            d8: D8 = port3.p304 @ (3, 4),
            /// `D9` — P303, PWM capable. GPT7B.
            d9: D9 = port3.p303 @ (3, 3),
            /// `D10` — P103, PWM capable. Arduino's `SS`, but software-driven: this
            /// pin has no `SSL` function. AN019, CAN0 TX, GPT2A, SCI0 CTS.
            d10: D10 = port1.p103 @ (1, 3),
            /// `D11` — P411, PWM capable. `MOSI` (SPI0). GPT6A, IRQ4, SCI0 TXD.
            d11: D11 = port4.p411 @ (4, 11),
            /// `D12` — P410. `MISO` (SPI0). GPT6B, IRQ5, SCI0 RXD.
            d12: D12 = port4.p410 @ (4, 10),
            /// `D13` — P102. `SCK` (SPI0) *and* `LED_BUILTIN`; they conflict.
            /// AN020, CAN0 RX, GPT2B, SCI0 SCK, SCI2 TXD.
            d13: D13 = port1.p102 @ (1, 2),

            /// `A0` — P014. AN009, and the DAC12 output.
            a0: A0 = port0.p014 @ (0, 14),
            /// `A1` — P000. AN000, IRQ6.
            a1: A1 = port0.p000 @ (0, 0),
            /// `A2` — P001. AN001, IRQ7.
            a2: A2 = port0.p001 @ (0, 1),
            /// `A3` — P002. AN002, IRQ2.
            a3: A3 = port0.p002 @ (0, 2),
            /// `A4` — P101. `SDA` (IIC1). AN021, GPT5A, IRQ1, SCI0 TXD, SPI0 MOSI.
            a4: A4 = port1.p101 @ (1, 1),
            /// `A5` — P100. `SCL` (IIC1). AN022, GPT5B, IRQ2, SCI0 RXD, SPI0 MISO.
            a5: A5 = port1.p100 @ (1, 0),

            /// P500 — the on-board AVCC divider, AN016. Not on a header.
            vcc_measure: VccMeasure = port5.p500 @ (5, 0),
            /// P408 — the USB mux switch. Drive it high to give the RA4M1 the port.
            usb_switch: UsbSwitch = port4.p408 @ (4, 8),

            /// `D22` — P109. TX of `Serial1` (SCI9). CAN0 TX, GPT1A, SPI1 MOSI.
            d22: D22 = port1.p109 @ (1, 9),
            /// `D23` — P110. RX of `Serial1` (SCI9). CAN0 RX, GPT1B, IRQ3,
            /// SPI1 MISO.
            d23: D23 = port1.p110 @ (1, 10),
            /// `D24` — P501. TX of `Serial3` (SCI1), wired to the ESP32.
            d24: D24 = port5.p501 @ (5, 1),
            /// `D25` — P502. RX of `Serial3` (SCI1), wired to the ESP32.
            d25: D25 = port5.p502 @ (5, 2),
            /// `D26` — P400. `SCL` of `Wire1` (IIC0), on the Qwiic connector.
            d26: D26 = port4.p400 @ (4, 0),
            /// `D27` — P401. `SDA` of `Wire1` (IIC0), on the Qwiic connector.
            d27: D27 = port4.p401 @ (4, 1),

            /// `D28` — P003. LED matrix.
            d28: D28 = port0.p003 @ (0, 3),
            /// `D29` — P004. LED matrix.
            d29: D29 = port0.p004 @ (0, 4),
            /// `D30` — P011. LED matrix.
            d30: D30 = port0.p011 @ (0, 11),
            /// `D31` — P012. LED matrix.
            d31: D31 = port0.p012 @ (0, 12),
            /// `D32` — P013. LED matrix.
            d32: D32 = port0.p013 @ (0, 13),
            /// `D33` — P015. LED matrix.
            d33: D33 = port0.p015 @ (0, 15),
            /// `D34` — P204. LED matrix.
            d34: D34 = port2.p204 @ (2, 4),
            /// `D35` — P205. LED matrix.
            d35: D35 = port2.p205 @ (2, 5),
            /// `D36` — P206. LED matrix.
            d36: D36 = port2.p206 @ (2, 6),
            /// `D37` — P212. LED matrix.
            d37: D37 = port2.p212 @ (2, 12),
            /// `D38` — P213. LED matrix.
            d38: D38 = port2.p213 @ (2, 13),
        ],
    }

    /// The on-board LED, on `D13`. Shared with `SCK`.
    pub type LedBuiltin<MODE = crate::gpio::Input<crate::gpio::Floating>> = D13<MODE>;
    /// `SDA` of `Wire` (IIC1), on `A4`.
    pub type Sda<MODE = crate::gpio::Input<crate::gpio::Floating>> = A4<MODE>;
    /// `SCL` of `Wire` (IIC1), on `A5`.
    pub type Scl<MODE = crate::gpio::Input<crate::gpio::Floating>> = A5<MODE>;
    /// `SDA` of `Wire1` (IIC0), on the Qwiic connector (`D27`).
    pub type QwiicSda<MODE = crate::gpio::Input<crate::gpio::Floating>> = D27<MODE>;
    /// `SCL` of `Wire1` (IIC0), on the Qwiic connector (`D26`).
    pub type QwiicScl<MODE = crate::gpio::Input<crate::gpio::Floating>> = D26<MODE>;
    /// TX of `Serial1` (SCI9), on `D22`.
    pub type Tx<MODE = crate::gpio::Input<crate::gpio::Floating>> = D22<MODE>;
    /// RX of `Serial1` (SCI9), on `D23`.
    pub type Rx<MODE = crate::gpio::Input<crate::gpio::Floating>> = D23<MODE>;
    /// `MOSI` of `SPI` (SPI0), on `D11`.
    pub type Mosi<MODE = crate::gpio::Input<crate::gpio::Floating>> = D11<MODE>;
    /// `MISO` of `SPI` (SPI0), on `D12`.
    pub type Miso<MODE = crate::gpio::Input<crate::gpio::Floating>> = D12<MODE>;
    /// `SCK` of `SPI` (SPI0), on `D13`. Shared with the LED.
    pub type Sck<MODE = crate::gpio::Input<crate::gpio::Floating>> = D13<MODE>;
    /// Arduino's `SS`/`CS`, on `D10`. Software-driven; not a hardware `SSL`.
    pub type Cs<MODE = crate::gpio::Input<crate::gpio::Floating>> = D10<MODE>;

    /// The [`AltFunction`](crate::gpio::AltFunction) each bus needs.
    ///
    /// Taken from the Arduino variant's per-pin mux table, not inferred. Note that
    /// `SERIAL1` and `SERIAL2` land in different SCI mux groups even though both are
    /// UARTs — the group is a property of the pin, not of the peripheral.
    pub mod mux {
        use crate::gpio::AltFunction;

        /// `Serial1` on `D22`/`D23`, which is SCI9.
        pub const SERIAL1: AltFunction = AltFunction::SciGroup2;
        /// `Serial2` on `D0`/`D1`, which is SCI2.
        pub const SERIAL2: AltFunction = AltFunction::SciGroup1;
        /// `Serial3` on `D24`/`D25`, which is SCI1, wired to the ESP32.
        pub const SERIAL3: AltFunction = AltFunction::SciGroup2;
        /// `Wire` on `A4`/`A5`, which is IIC1.
        pub const WIRE: AltFunction = AltFunction::Iic;
        /// `Wire1` on the Qwiic connector (`D26`/`D27`), which is IIC0.
        pub const WIRE1: AltFunction = AltFunction::Iic;
        /// `SPI` on `D11`/`D12`/`D13`, which is SPI0.
        pub const SPI: AltFunction = AltFunction::Spi;
    }

    /// ADC channel for each analog-capable header pin.
    ///
    /// Pass these to [`Adc::read`](crate::adc::Adc::read) after putting the matching
    /// pin in [`Analog`](crate::gpio::Analog) mode with
    /// [`into_analog`](crate::gpio::Pin::into_analog).
    pub mod analog {
        use crate::adc::Channel;

        /// `A0` (P014).
        pub const A0: Channel = Channel::AN009;
        /// `A1` (P000).
        pub const A1: Channel = Channel::AN000;
        /// `A2` (P001).
        pub const A2: Channel = Channel::AN001;
        /// `A3` (P002).
        pub const A3: Channel = Channel::AN002;
        /// `A4` (P101), if you are not using it for I2C.
        pub const A4: Channel = Channel::AN021;
        /// `A5` (P100), if you are not using it for I2C.
        pub const A5: Channel = Channel::AN022;
        /// `D10` (P103), which is also analog capable.
        pub const D10: Channel = Channel::AN019;
        /// `D13` (P102), which is also analog capable.
        pub const D13: Channel = Channel::AN020;
        /// The on-board AVCC divider (P500).
        pub const VCC_MEASURE: Channel = Channel::AN016;
    }
}
