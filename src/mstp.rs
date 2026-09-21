//! Module-stop (clock gating) control.
//!
//! Every peripheral on the RA4M1 comes out of reset with its clock gated off by a
//! bit in `MSTPCRB`/`MSTPCRC`/`MSTPCRD`. Clearing the bit starts the module; setting
//! it again stops it. The drivers in this crate call [`start`] for you, so you only
//! need this module if you are driving a peripheral through the PAC directly.

use crate::pac;

/// One peripheral's module-stop bit.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct ModuleStop {
    reg: Reg,
    bit: u8,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Reg {
    B,
    C,
    D,
}

macro_rules! modules {
    ($($(#[$doc:meta])* $name:ident => ($reg:ident, $bit:literal),)+) => {
        impl ModuleStop {
            $(
                $(#[$doc])*
                pub const $name: Self = Self { reg: Reg::$reg, bit: $bit };
            )+
        }
    };
}

modules! {
    /// CAN0
    CAN0 => (B, 2),
    /// IIC1 (I2C bus interface 1)
    IIC1 => (B, 8),
    /// IIC0 (I2C bus interface 0)
    IIC0 => (B, 9),
    /// USB 2.0 full-speed interface
    USBFS => (B, 11),
    /// SPI1
    SPI1 => (B, 18),
    /// SPI0
    SPI0 => (B, 19),
    /// SCI9
    SCI9 => (B, 22),
    /// SCI2
    SCI2 => (B, 29),
    /// SCI1
    SCI1 => (B, 30),
    /// SCI0
    SCI0 => (B, 31),
    /// Clock frequency accuracy measurement circuit
    CAC => (C, 0),
    /// CRC calculator
    CRC => (C, 1),
    /// Capacitive touch sensing unit
    CTSU => (C, 3),
    /// Segment LCD controller
    SLCDC => (C, 4),
    /// Synchronous serial interface 0
    SSIE0 => (C, 8),
    /// Data operation circuit
    DOC => (C, 13),
    /// Event link controller
    ELC => (C, 14),
    /// AGT1
    AGT1 => (D, 2),
    /// AGT0
    AGT0 => (D, 3),
    /// GPT320 .. GPT323
    GPT32 => (D, 5),
    /// GPT164 .. GPT169
    GPT16 => (D, 6),
    /// Port output enable for GPT
    POEG => (D, 14),
    /// 14-bit A/D converter
    ADC140 => (D, 16),
    /// 8-bit D/A converter
    DAC8 => (D, 19),
    /// 12-bit D/A converter
    DAC12 => (D, 20),
    /// Low-power analog comparator
    ACMPLP => (D, 29),
    /// Operational amplifier
    OPAMP => (D, 31),
}

/// Ungate the module's clock (clear its module-stop bit).
///
/// A read-modify-write on a register shared by every peripheral, so it runs in a
/// critical section.
pub fn start(module: ModuleStop) {
    write_bit(module, false);
}

/// Gate the module's clock again (set its module-stop bit).
pub fn stop(module: ModuleStop) {
    write_bit(module, true);
}

fn write_bit(module: ModuleStop, stop: bool) {
    critical_section::with(|_| {
        // SAFETY: we hold a critical section, and only touch the one bit that
        // identifies this module. `Mstp` is a zero-sized token, so stealing it here
        // cannot invalidate anything the caller still holds.
        let mstp = unsafe { pac::Mstp::steal() };
        let mask = 1u32 << module.bit;
        let apply = |bits: u32| if stop { bits | mask } else { bits & !mask };
        match module.reg {
            Reg::B => mstp
                .mstpcrb()
                .modify(|r, w| unsafe { w.bits(apply(r.bits())) }),
            Reg::C => mstp
                .mstpcrc()
                .modify(|r, w| unsafe { w.bits(apply(r.bits())) }),
            Reg::D => mstp
                .mstpcrd()
                .modify(|r, w| unsafe { w.bits(apply(r.bits())) }),
        };
    });
}
