//! I2C master on the IIC peripherals.
//!
//! ```ignore
//! let scl = p4.p400.into_alternate_open_drain(AltFunction::Iic);
//! let sda = p4.p401.into_alternate_open_drain(AltFunction::Iic);
//! let mut i2c = I2c::new(dp.iic0, (scl, sda), Config::standard(), &clocks);
//! let mut buf = [0u8; 2];
//! i2c.write_read(0x68, &[0x00], &mut buf).unwrap();
//! ```
//!
//! Implements [`embedded_hal::i2c::I2c`] for 7-bit addresses, including
//! multi-operation [`transaction`](embedded_hal::i2c::I2c::transaction)s with
//! repeated starts. Transfers are polled, and every wait is bounded, so a stuck bus
//! surfaces as [`Error::Timeout`] rather than a hang.
//!
//! 10-bit addressing is not implemented.

use embedded_hal::i2c::{
    ErrorKind, ErrorType, I2c as I2cTrait, NoAcknowledgeSource, Operation, SevenBitAddress,
};

use crate::clock::Clocks;
use crate::gpio::{Alternate, Pin};
use crate::mstp::{self, ModuleStop};
use crate::pac;
use crate::spin_until;

/// Poll budget for any single wait inside a transfer.
///
/// Generous enough for a 10 kHz bus at 48 MHz, small enough that a wedged bus is
/// reported in well under a second.
const TIMEOUT: u32 = 1_000_000;

/// I2C bus errors.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Error {
    /// The addressed device did not acknowledge its address.
    NoAcknowledgeAddress,
    /// A data byte was not acknowledged.
    NoAcknowledgeData,
    /// Lost arbitration to another master.
    ArbitrationLoss,
    /// The bus never reached the state we were waiting for.
    Timeout,
    /// The bus was still busy when the transfer was started.
    Busy,
    /// A zero-length read was requested. The hardware cannot clock in no bytes.
    ZeroLengthRead,
    /// The requested bus frequency cannot be produced from the current PCLKB.
    UnsupportedFrequency,
}

impl embedded_hal::i2c::Error for Error {
    fn kind(&self) -> ErrorKind {
        match self {
            Error::NoAcknowledgeAddress => {
                ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address)
            }
            Error::NoAcknowledgeData => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data),
            Error::ArbitrationLoss => ErrorKind::ArbitrationLoss,
            Error::Timeout | Error::Busy => ErrorKind::Bus,
            Error::ZeroLengthRead | Error::UnsupportedFrequency => ErrorKind::Other,
        }
    }
}

/// Bus configuration.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Config {
    /// Target SCL frequency in Hz.
    ///
    /// The achieved frequency is always a little lower than this, because the
    /// hardware counts only the driven low period and the release of SCL is followed
    /// by a rise time set by the bus pull-ups, which the counters do not model.
    pub frequency: u32,
    /// Enable the digital noise filter on SCL and SDA.
    pub noise_filter: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self::standard()
    }
}

impl Config {
    /// Standard mode, 100 kHz.
    pub const fn standard() -> Self {
        Self {
            frequency: 100_000,
            noise_filter: true,
        }
    }

    /// Fast mode, 400 kHz.
    pub const fn fast() -> Self {
        Self {
            frequency: 400_000,
            noise_filter: true,
        }
    }

    /// An arbitrary SCL frequency.
    pub const fn hz(frequency: u32) -> Self {
        Self {
            frequency,
            noise_filter: true,
        }
    }
}

// --- Instances -------------------------------------------------------------------

/// An IIC channel.
///
/// # Safety
///
/// `ptr` must return the base address of a real IIC register block, and no two
/// implementations may return the same address.
pub unsafe trait Instance {
    /// This channel's module-stop bit.
    const MODULE: ModuleStop;

    /// Base address of the channel's registers.
    ///
    /// `iic1::RegisterBlock` has the same layout as `iic0::RegisterBlock` for every
    /// register this driver touches, so IIC1 is viewed through the `iic0` layout.
    fn ptr() -> *const pac::iic0::RegisterBlock;
}

// SAFETY: `pac::Iic0::PTR` and `pac::Iic1::PTR` are the distinct, documented base
// addresses of the two IIC channels.
unsafe impl Instance for pac::Iic0 {
    const MODULE: ModuleStop = ModuleStop::IIC0;
    #[inline(always)]
    fn ptr() -> *const pac::iic0::RegisterBlock {
        pac::Iic0::PTR
    }
}

unsafe impl Instance for pac::Iic1 {
    const MODULE: ModuleStop = ModuleStop::IIC1;
    #[inline(always)]
    fn ptr() -> *const pac::iic0::RegisterBlock {
        pac::Iic1::PTR as *const pac::iic0::RegisterBlock
    }
}

/// Pins that can be handed to an [`I2c`]: an `(scl, sda)` tuple, or `()`.
pub trait Pins<IIC> {}

impl<IIC> Pins<IIC> for () {}
impl<IIC, const P1: u8, const N1: u8, const P2: u8, const N2: u8> Pins<IIC>
    for (Pin<P1, N1, Alternate>, Pin<P2, N2, Alternate>)
{
}

// --- Bit rate --------------------------------------------------------------------

/// `ICMR1.CKS`, `ICBRH` and `ICBRL` for a target frequency.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
struct BitRate {
    cks: u8,
    brh: u8,
    brl: u8,
}

/// Split one SCL period into a high and a low count.
///
/// The counters run at `PCLKB / 2^cks` and count `BRH + 1` high and `BRL + 1` low,
/// each capped at 32 counts. We pick the smallest prescaler whose period fits, then
/// give the low phase the larger share, because every I2C mode specifies a longer
/// minimum low period than high period.
fn calc_bitrate(pclkb: u32, frequency: u32) -> Option<BitRate> {
    if frequency == 0 {
        return None;
    }

    for cks in 0u8..8 {
        let counter_hz = pclkb >> cks;
        let total = counter_hz / frequency;
        // Each half needs at least 1 count and at most 32.
        if total < 2 {
            continue;
        }
        if total > 64 {
            continue;
        }
        let low = (total * 5 / 9).clamp(1, 32);
        let high = (total - low).clamp(1, 32);
        return Some(BitRate {
            cks,
            brh: (high - 1) as u8,
            brl: (low - 1) as u8,
        });
    }
    None
}

// --- Driver ----------------------------------------------------------------------

/// A polled I2C master.
#[derive(Debug)]
pub struct I2c<IIC, PINS> {
    iic: IIC,
    pins: PINS,
}

impl<IIC: Instance, PINS: Pins<IIC>> I2c<IIC, PINS> {
    /// Bring up the channel as a master at `config.frequency`.
    pub fn new(iic: IIC, pins: PINS, config: Config, clocks: &Clocks) -> Result<Self, Error> {
        let rate = calc_bitrate(clocks.pclkb().raw(), config.frequency)
            .ok_or(Error::UnsupportedFrequency)?;

        mstp::start(IIC::MODULE);

        let this = Self { iic, pins };
        let regs = this.regs();

        // Reset sequence: the internal state machine and the SCL/SDA output latches
        // are only reset while IICRST is asserted, and ICE must be set for the rest
        // of the registers to be writable.
        regs.iccr1().write(|w| {
            w.ice().clear_bit();
            w.iicrst().set_bit()
        });
        regs.iccr1().write(|w| {
            w.ice().set_bit();
            w.iicrst().set_bit()
        });

        regs.icmr1().write(|w| unsafe { w.cks().bits(rate.cks) });
        regs.icbrh().write(|w| unsafe { w.brh().bits(rate.brh) });
        regs.icbrl().write(|w| unsafe { w.brl().bits(rate.brl) });

        // NACKE: stop transmitting when a slave NACKs. NFE/SCLE: noise filter and
        // SCL synchronisation. The arbitration-loss detections are left off; we are
        // the only master in the configurations this driver supports.
        regs.icfer().write(|w| {
            w.tmoe().clear_bit();
            w.male().clear_bit();
            w.nale().clear_bit();
            w.sale().clear_bit();
            w.nacke().set_bit();
            w.nfe().bit(config.noise_filter);
            w.scle().set_bit()
        });

        // Master only: no slave addresses recognised, no interrupts.
        regs.icser().write(|w| unsafe { w.bits(0) });
        regs.icier().write(|w| unsafe { w.bits(0) });

        // RDRFS = 0: receiving a byte does not stall the clock by itself; WAIT
        // controls that explicitly during the last bytes of a read.
        regs.icmr3().write(|w| {
            w.rdrfs().clear_bit();
            w.wait().clear_bit();
            w.ackwp().set_bit();
            w.ackbt().clear_bit()
        });

        // Release the reset; the bus is now idle and usable.
        regs.iccr1().modify(|_, w| w.iicrst().clear_bit());

        Ok(this)
    }

    /// Stop the channel and give back the peripheral and pins.
    pub fn release(self) -> (IIC, PINS) {
        self.regs().iccr1().write(|w| w.ice().clear_bit());
        mstp::stop(IIC::MODULE);
        (self.iic, self.pins)
    }

    #[inline(always)]
    fn regs(&self) -> &'static pac::iic0::RegisterBlock {
        // SAFETY: `I2c` owns the peripheral token for this channel, so it is the only
        // thing writing these registers.
        unsafe { &*IIC::ptr() }
    }

    /// Wait for the bus to go idle, then issue a START.
    fn start(&mut self) -> Result<(), Error> {
        if !spin_until(TIMEOUT, || self.regs().iccr2().read().bbsy().bit_is_clear()) {
            return Err(Error::Busy);
        }
        self.clear_status();
        self.regs().iccr2().modify(|_, w| w.st().set_bit());
        Ok(())
    }

    /// Issue a repeated START.
    ///
    /// Only valid while we already hold the bus, which is why it does not wait on
    /// `BBSY` the way [`start`](Self::start) does.
    fn restart(&mut self) -> Result<(), Error> {
        self.regs().iccr2().modify(|_, w| w.rs().set_bit());
        Ok(())
    }

    /// Issue a STOP and wait for it to appear on the bus.
    fn stop(&mut self) -> Result<(), Error> {
        let regs = self.regs();
        regs.icsr2().modify(|_, w| w.stop().clear_bit());
        regs.iccr2().modify(|_, w| w.sp().set_bit());
        if !spin_until(TIMEOUT, || regs.icsr2().read().stop().bit_is_set()) {
            return Err(Error::Timeout);
        }
        self.clear_status();
        Ok(())
    }

    /// Clear the latched STOP, NACK and arbitration-loss flags.
    fn clear_status(&self) {
        self.regs().icsr2().modify(|_, w| {
            w.stop().clear_bit();
            w.nackf().clear_bit();
            w.al().clear_bit()
        });
    }

    /// Abandon a transfer: force a STOP so the bus is left usable.
    ///
    /// Used on every error path. Its own result is discarded because the error that
    /// got us here is the one worth reporting.
    fn abort(&mut self) {
        let _ = self.stop();
    }

    fn wait_tdre(&self) -> Result<(), Error> {
        if spin_until(TIMEOUT, || self.regs().icsr2().read().tdre().bit_is_set()) {
            Ok(())
        } else {
            Err(Error::Timeout)
        }
    }

    fn wait_tend(&self) -> Result<(), Error> {
        if spin_until(TIMEOUT, || self.regs().icsr2().read().tend().bit_is_set()) {
            Ok(())
        } else {
            Err(Error::Timeout)
        }
    }

    fn wait_rdrf(&self) -> Result<(), Error> {
        if spin_until(TIMEOUT, || self.regs().icsr2().read().rdrf().bit_is_set()) {
            Ok(())
        } else {
            Err(Error::Timeout)
        }
    }

    fn check_nack(&self, source: Error) -> Result<(), Error> {
        let sr = self.regs().icsr2().read();
        if sr.al().bit_is_set() {
            return Err(Error::ArbitrationLoss);
        }
        if sr.nackf().bit_is_set() {
            return Err(source);
        }
        Ok(())
    }

    /// Put the address byte on the bus and confirm it was acknowledged.
    fn send_address(&mut self, address: u8, read: bool) -> Result<(), Error> {
        self.wait_tdre()?;
        let byte = (address << 1) | u8::from(read);
        self.regs().icdrt().write(|w| unsafe { w.icdrt().bits(byte) });
        if read {
            // For a read the address phase is complete once the first data byte has
            // started arriving; NACKF is still the way an unanswered address shows up.
            if !spin_until(TIMEOUT, || {
                let sr = self.regs().icsr2().read();
                sr.rdrf().bit_is_set() || sr.nackf().bit_is_set() || sr.al().bit_is_set()
            }) {
                return Err(Error::Timeout);
            }
        } else {
            self.wait_tend()?;
        }
        self.check_nack(Error::NoAcknowledgeAddress)
    }

    /// Write every byte of the run's operations, with no repeated start between them.
    fn write_run(&mut self, ops: &[Operation<'_>]) -> Result<(), Error> {
        for op in ops {
            let Operation::Write(buf) = op else {
                unreachable!("run is all writes");
            };
            for &byte in *buf {
                self.wait_tdre()?;
                self.regs().icdrt().write(|w| unsafe { w.icdrt().bits(byte) });
                self.check_nack(Error::NoAcknowledgeData)?;
            }
        }
        self.wait_tend()?;
        self.check_nack(Error::NoAcknowledgeData)
    }

    /// Read `total` bytes into the run's operations, finishing with a STOP or a
    /// repeated START.
    ///
    /// The RIIC needs software to stay a byte ahead of the bus: reading `ICDRR`
    /// releases the clock for the *next* byte, so the NACK that ends the transfer has
    /// to be armed while the second-to-last byte is still in the register, and the
    /// clock has to be held (`WAIT`) so the STOP lands immediately after it. That is
    /// why `total` is computed across the whole run before any byte is read.
    fn read_run(
        &mut self,
        ops: &mut [Operation<'_>],
        total: usize,
        finish: Finish,
    ) -> Result<(), Error> {
        if total == 0 {
            return Err(Error::ZeroLengthRead);
        }
        let regs = self.regs();

        if total == 1 {
            // There is no second-to-last byte, so arm everything before the dummy
            // read that starts the one and only byte.
            regs.icmr3().modify(|_, w| w.wait().set_bit());
            regs.icmr3().modify(|_, w| {
                w.ackwp().set_bit();
                w.ackbt().set_bit()
            });
        }

        // Reading ICDRR here discards the address-phase placeholder and starts
        // clocking in the first data byte.
        let _ = regs.icdrr().read().icdrr().bits();

        let mut index = 0usize;
        for op in ops.iter_mut() {
            let Operation::Read(buf) = op else {
                unreachable!("run is all reads");
            };
            for slot in buf.iter_mut() {
                self.wait_rdrf()?;

                if index + 2 == total {
                    // The next ICDRR read starts the final byte: NACK it, and hold
                    // the clock once it has arrived.
                    regs.icmr3().modify(|_, w| w.wait().set_bit());
                    regs.icmr3().modify(|_, w| {
                        w.ackwp().set_bit();
                        w.ackbt().set_bit()
                    });
                }

                if index + 1 == total {
                    // SCL is held low. Queue the stop or restart *before* reading, so
                    // it is emitted the moment the clock is released.
                    match finish {
                        Finish::Stop => {
                            regs.icsr2().modify(|_, w| w.stop().clear_bit());
                            regs.iccr2().modify(|_, w| w.sp().set_bit());
                        }
                        Finish::Restart => {
                            regs.iccr2().modify(|_, w| w.rs().set_bit());
                        }
                    }
                }

                *slot = regs.icdrr().read().icdrr().bits();
                index += 1;
            }
        }

        // Release the clock and re-arm ACK for the next transfer.
        regs.icmr3().modify(|_, w| w.wait().clear_bit());
        regs.icmr3().modify(|_, w| {
            w.ackwp().set_bit();
            w.ackbt().clear_bit()
        });

        if finish == Finish::Stop {
            if !spin_until(TIMEOUT, || regs.icsr2().read().stop().bit_is_set()) {
                return Err(Error::Timeout);
            }
            self.clear_status();
        }
        Ok(())
    }
}

/// How a run of operations ends.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Finish {
    /// Last run of the transaction: emit a STOP.
    Stop,
    /// Another run follows with the opposite direction: emit a repeated START.
    Restart,
}

impl<IIC, PINS> ErrorType for I2c<IIC, PINS> {
    type Error = Error;
}

impl<IIC: Instance, PINS: Pins<IIC>> I2cTrait<SevenBitAddress> for I2c<IIC, PINS> {
    fn transaction(
        &mut self,
        address: SevenBitAddress,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        if operations.is_empty() {
            return Ok(());
        }

        let result = self.run_transaction(address, operations);
        if result.is_err() {
            self.abort();
        }
        result
    }
}

impl<IIC: Instance, PINS: Pins<IIC>> I2c<IIC, PINS> {
    fn run_transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Error> {
        let len = operations.len();
        let mut i = 0;
        let mut first = true;

        while i < len {
            let is_read = matches!(operations[i], Operation::Read(_));
            // `embedded-hal` says consecutive operations of the same type are merged
            // into one bus transfer with no repeated start between them.
            let mut j = i + 1;
            while j < len && matches!(operations[j], Operation::Read(_)) == is_read {
                j += 1;
            }
            let last_run = j == len;

            if first {
                self.start()?;
                first = false;
            } else {
                self.restart()?;
            }
            self.send_address(address, is_read)?;

            if is_read {
                // The NACK has to be armed a byte early, so the run's total length is
                // needed before the first byte is clocked in.
                let total: usize = operations[i..j]
                    .iter()
                    .map(|op| match op {
                        Operation::Read(buf) => buf.len(),
                        Operation::Write(_) => 0,
                    })
                    .sum();
                let finish = if last_run { Finish::Stop } else { Finish::Restart };
                self.read_run(&mut operations[i..j], total, finish)?;
            } else {
                self.write_run(&operations[i..j])?;
                if last_run {
                    self.stop()?;
                }
            }

            i = j;
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn achieved(pclkb: u32, r: BitRate) -> u32 {
        let counter = pclkb >> r.cks;
        counter / (r.brh as u32 + 1 + r.brl as u32 + 1)
    }

    #[test]
    fn standard_and_fast_mode_land_close() {
        for pclkb in [24_000_000u32, 48_000_000] {
            for target in [100_000u32, 400_000] {
                let r = calc_bitrate(pclkb, target).expect("representable");
                let got = achieved(pclkb, r);
                let error = got.abs_diff(target) * 100 / target;
                assert!(error <= 10, "{pclkb} -> {target}: got {got}");
                // Every I2C mode wants the low period at least as long as the high.
                assert!(r.brl >= r.brh, "low period must not be shorter than high");
            }
        }
    }

    #[test]
    fn rejects_impossible_frequencies() {
        assert!(calc_bitrate(24_000_000, 0).is_none());
        // Faster than PCLKB/2 cannot be counted out.
        assert!(calc_bitrate(24_000_000, 20_000_000).is_none());
    }
}
