//! Adapter abstraction between the emulated UART and a diagnostic line.
//!
//! Modelled on CASCADE's `interface.h`. Differences from the C++ version:
//! - every call carries the emulated cycle count, so a backend can timestamp
//!   traffic and time its replies deterministically;
//! - instead of holding a `Serial *`, every call gets the UART's receive
//!   buffer as an [`RxBuf`]. Upstream backends add bytes to it and sometimes
//!   flush it in the middle of a call; the handle allows exactly that.

#![forbid(unsafe_code)]

use std::cell::RefCell;
use std::rc::Rc;

mod fake;
pub mod iso14230;
pub mod kwp;

pub use fake::Fake;
pub use kwp::{KwpConfig, KwpEcu};

/// Emulated CPU state-time count (the core's clock, 20 MHz nominal).
pub type Cycles = u64;

/// The UART receive buffer as seen by a backend.
pub trait RxBuf {
    /// Append a received byte (`Serial::addRxData`).
    fn push(&mut self, byte: u8);
    /// Drop everything buffered (`Serial::flushRxBuf`).
    fn flush(&mut self);
}

/// One line-level event, the unit of tracing and replay.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Event {
    /// Byte sent by the scanner.
    Tx(u8),
    /// Byte the backend put into the UART receive buffer.
    Rx(u8),
    /// The backend flushed the UART receive buffer.
    RxFlush,
    /// UART baud divisor written by the firmware.
    BaudDivisor(u32),
    /// Firmware is about to start a 5-baud slow init.
    SlowInitImminent,
    /// Whole-address slow init requested.
    SlowInit(u8),
    /// One bit of a bitwise slow init (`true` = line high / break off).
    SlowInitBit(bool),
    /// L line level.
    L(bool),
    /// CAN mode on or off.
    Can(bool),
    /// Diagnostic line selected by the firmware (I/O 0x250): OBD pin
    /// number as upstream decodes it (7 = K, 8 = ABS/TCS, 12 = SRS, ...;
    /// 42 = CAN; negative = unknown, -(~value)).
    Line(i32),
}

/// A timestamped [`Event`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Stamped {
    pub at: Cycles,
    pub event: Event,
}

/// Diagnostic adapter backend. Optional operations have no-op defaults,
/// matching the C++ base class.
pub trait Interface {
    fn set_baud_divisor(&mut self, now: Cycles, divisor: u32, rx: &mut dyn RxBuf);

    fn send_byte(&mut self, now: Cycles, byte: u8, rx: &mut dyn RxBuf);

    /// Deliver whatever has arrived from the line by `now` (`checkInput`).
    fn poll_input(&mut self, now: Cycles, rx: &mut dyn RxBuf);

    fn slow_init_imminent(&mut self, _now: Cycles, _rx: &mut dyn RxBuf) {}

    /// Whole-address slow init, for adapters that do it themselves.
    fn send_slow_init(&mut self, _now: Cycles, _target: u8, _rx: &mut dyn RxBuf) {}

    /// Drive one bit of a 5-baud init. Returns `false` if unsupported.
    fn send_slow_init_bitwise(&mut self, _now: Cycles, _bit: bool, _rx: &mut dyn RxBuf) -> bool {
        false
    }

    fn set_l(&mut self, _now: Cycles, _level: bool) {}

    fn set_can(&mut self, _now: Cycles, _on: bool, _rx: &mut dyn RxBuf) {}

    /// The firmware switched the diagnostic line (not in upstream's
    /// interface; upstream only shows it in the UI).
    fn select_line(&mut self, _now: Cycles, _line: i32) {}

    /// Instantaneous RX line level, if the backend can sample it.
    fn rx_state(&mut self, _now: Cycles) -> Option<bool> {
        None
    }

    fn set_rx_bitbang(&mut self, _now: Cycles, _on: bool) {}
}

impl<T: Interface + ?Sized> Interface for Box<T> {
    fn set_baud_divisor(&mut self, now: Cycles, divisor: u32, rx: &mut dyn RxBuf) {
        (**self).set_baud_divisor(now, divisor, rx)
    }
    fn send_byte(&mut self, now: Cycles, byte: u8, rx: &mut dyn RxBuf) {
        (**self).send_byte(now, byte, rx)
    }
    fn poll_input(&mut self, now: Cycles, rx: &mut dyn RxBuf) {
        (**self).poll_input(now, rx)
    }
    fn slow_init_imminent(&mut self, now: Cycles, rx: &mut dyn RxBuf) {
        (**self).slow_init_imminent(now, rx)
    }
    fn send_slow_init(&mut self, now: Cycles, target: u8, rx: &mut dyn RxBuf) {
        (**self).send_slow_init(now, target, rx)
    }
    fn send_slow_init_bitwise(&mut self, now: Cycles, bit: bool, rx: &mut dyn RxBuf) -> bool {
        (**self).send_slow_init_bitwise(now, bit, rx)
    }
    fn set_l(&mut self, now: Cycles, level: bool) {
        (**self).set_l(now, level)
    }
    fn set_can(&mut self, now: Cycles, on: bool, rx: &mut dyn RxBuf) {
        (**self).set_can(now, on, rx)
    }
    fn select_line(&mut self, now: Cycles, line: i32) {
        (**self).select_line(now, line)
    }
    fn rx_state(&mut self, now: Cycles) -> Option<bool> {
        (**self).rx_state(now)
    }
    fn set_rx_bitbang(&mut self, now: Cycles, on: bool) {
        (**self).set_rx_bitbang(now, on)
    }
}

/// A line with nothing attached: accepts everything, never answers.
#[derive(Debug, Default)]
pub struct Null;

impl Interface for Null {
    fn set_baud_divisor(&mut self, _: Cycles, _: u32, _: &mut dyn RxBuf) {}
    fn send_byte(&mut self, _: Cycles, _: u8, _: &mut dyn RxBuf) {}
    fn poll_input(&mut self, _: Cycles, _: &mut dyn RxBuf) {}
}

/// [`RxBuf`] adapter that records what passes through it.
struct Recording<'a> {
    inner: &'a mut dyn RxBuf,
    events: &'a EventLog,
    at: Cycles,
}

impl RxBuf for Recording<'_> {
    fn push(&mut self, byte: u8) {
        self.events.borrow_mut().push(Stamped {
            at: self.at,
            event: Event::Rx(byte),
        });
        self.inner.push(byte);
    }
    fn flush(&mut self) {
        self.events.borrow_mut().push(Stamped {
            at: self.at,
            event: Event::RxFlush,
        });
        self.inner.flush();
    }
}

/// Shared event log, so the caller can read it while the interface is owned
/// by the emulator.
pub type EventLog = Rc<RefCell<Vec<Stamped>>>;

/// Wraps another interface and records every call and receive-buffer change.
pub struct Traced<I> {
    pub inner: I,
    events: EventLog,
}

impl<I> Traced<I> {
    pub fn new(inner: I) -> Self {
        Self {
            inner,
            events: EventLog::default(),
        }
    }

    /// Handle to the log; stays valid after `self` is boxed and moved.
    pub fn log_handle(&self) -> EventLog {
        self.events.clone()
    }

    fn log(&mut self, at: Cycles, event: Event) {
        self.events.borrow_mut().push(Stamped { at, event });
    }
}

macro_rules! recording {
    ($self:ident, $now:ident, $rx:ident) => {
        &mut Recording {
            inner: $rx,
            events: &$self.events,
            at: $now,
        }
    };
}

impl<I: Interface> Interface for Traced<I> {
    fn set_baud_divisor(&mut self, now: Cycles, divisor: u32, rx: &mut dyn RxBuf) {
        self.log(now, Event::BaudDivisor(divisor));
        self.inner
            .set_baud_divisor(now, divisor, recording!(self, now, rx));
    }

    fn send_byte(&mut self, now: Cycles, byte: u8, rx: &mut dyn RxBuf) {
        self.log(now, Event::Tx(byte));
        self.inner.send_byte(now, byte, recording!(self, now, rx));
    }

    fn poll_input(&mut self, now: Cycles, rx: &mut dyn RxBuf) {
        self.inner.poll_input(now, recording!(self, now, rx));
    }

    fn slow_init_imminent(&mut self, now: Cycles, rx: &mut dyn RxBuf) {
        self.log(now, Event::SlowInitImminent);
        self.inner
            .slow_init_imminent(now, recording!(self, now, rx));
    }

    fn send_slow_init(&mut self, now: Cycles, target: u8, rx: &mut dyn RxBuf) {
        self.log(now, Event::SlowInit(target));
        self.inner
            .send_slow_init(now, target, recording!(self, now, rx));
    }

    fn send_slow_init_bitwise(&mut self, now: Cycles, bit: bool, rx: &mut dyn RxBuf) -> bool {
        self.log(now, Event::SlowInitBit(bit));
        self.inner
            .send_slow_init_bitwise(now, bit, recording!(self, now, rx))
    }

    fn set_l(&mut self, now: Cycles, level: bool) {
        self.log(now, Event::L(level));
        self.inner.set_l(now, level);
    }

    fn set_can(&mut self, now: Cycles, on: bool, rx: &mut dyn RxBuf) {
        self.log(now, Event::Can(on));
        self.inner.set_can(now, on, recording!(self, now, rx));
    }

    fn select_line(&mut self, now: Cycles, line: i32) {
        self.log(now, Event::Line(line));
        self.inner.select_line(now, line);
    }

    fn rx_state(&mut self, now: Cycles) -> Option<bool> {
        self.inner.rx_state(now)
    }

    fn set_rx_bitbang(&mut self, now: Cycles, on: bool) {
        self.inner.set_rx_bitbang(now, on);
    }
}

/// Simple `Vec`-backed [`RxBuf`], for tests and tools.
impl RxBuf for Vec<u8> {
    fn push(&mut self, byte: u8) {
        Vec::push(self, byte);
    }
    fn flush(&mut self) {
        self.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Answers each sent byte with its complement on the next poll.
    #[derive(Default)]
    struct Echo(Vec<u8>);

    impl Interface for Echo {
        fn set_baud_divisor(&mut self, _: Cycles, _: u32, rx: &mut dyn RxBuf) {
            rx.flush();
        }
        fn send_byte(&mut self, _: Cycles, b: u8, _: &mut dyn RxBuf) {
            self.0.push(!b);
        }
        fn poll_input(&mut self, _: Cycles, rx: &mut dyn RxBuf) {
            for b in self.0.drain(..) {
                rx.push(b);
            }
        }
    }

    #[test]
    fn traced_records_calls_and_buffer_changes() {
        let mut t = Traced::new(Echo::default());
        let mut buf: Vec<u8> = vec![9];
        t.set_baud_divisor(1, 42, &mut buf);
        t.send_byte(10, 0x81, &mut buf);
        t.poll_input(20, &mut buf);
        assert_eq!(buf, [0x7e]);
        let ev = |at, event| Stamped { at, event };
        assert_eq!(
            *t.log_handle().borrow(),
            [
                ev(1, Event::BaudDivisor(42)),
                ev(1, Event::RxFlush),
                ev(10, Event::Tx(0x81)),
                ev(20, Event::Rx(0x7e)),
            ]
        );
    }

    #[test]
    fn null_is_silent() {
        let mut n = Null;
        let mut buf = Vec::new();
        n.send_byte(0, 1, &mut buf);
        n.poll_input(1, &mut buf);
        assert!(buf.is_empty());
        assert!(!n.send_slow_init_bitwise(2, true, &mut buf));
        assert_eq!(n.rx_state(3), None);
    }
}
