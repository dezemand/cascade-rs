//! Adapter abstraction between the emulated UART and a diagnostic line.
//!
//! Modelled on CASCADE's `interface.h`. Differences from the C++ version:
//! every call carries the emulated cycle count, so a backend can timestamp
//! traffic and a replay backend can deliver bytes at the same emulated
//! moment; and received bytes are returned to the caller instead of being
//! pushed into the UART from inside the backend.

#![forbid(unsafe_code)]

/// Emulated CPU state-time count (the core's clock, 20 MHz nominal).
pub type Cycles = u64;

/// One line-level event, the unit of tracing and replay.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Event {
    /// Byte sent by the scanner.
    Tx(u8),
    /// Byte received from the line (after any echo cancellation).
    Rx(u8),
    /// UART baud divisor written by the firmware.
    BaudDivisor(u32),
    /// Firmware is about to start a 5-baud slow init.
    SlowInitImminent,
    /// One bit of a bitwise slow init (`true` = line high / break off).
    SlowInitBit(bool),
    /// L line level.
    L(bool),
    /// CAN mode on or off.
    Can(bool),
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
    fn set_baud_divisor(&mut self, now: Cycles, divisor: u32);

    fn send_byte(&mut self, now: Cycles, byte: u8);

    /// Collect bytes that have arrived from the line by `now`.
    /// (`checkInput` in the C++ code.)
    fn poll_input(&mut self, now: Cycles, rx: &mut dyn FnMut(u8));

    fn slow_init_imminent(&mut self, now: Cycles);

    /// Whole-address slow init, for adapters that do it themselves.
    fn send_slow_init(&mut self, _now: Cycles, _target: u8) {}

    /// Drive one bit of a 5-baud init. Returns `false` if unsupported.
    fn send_slow_init_bitwise(&mut self, _now: Cycles, _bit: bool) -> bool {
        false
    }

    fn set_l(&mut self, _now: Cycles, _level: bool) {}

    fn set_can(&mut self, _now: Cycles, _on: bool) {}

    /// Instantaneous RX line level, if the backend can sample it.
    fn rx_state(&mut self, _now: Cycles) -> Option<bool> {
        None
    }

    fn set_rx_bitbang(&mut self, _now: Cycles, _on: bool) {}
}

/// A line with nothing attached: accepts everything, never answers.
#[derive(Debug, Default)]
pub struct Null;

impl Interface for Null {
    fn set_baud_divisor(&mut self, _: Cycles, _: u32) {}
    fn send_byte(&mut self, _: Cycles, _: u8) {}
    fn poll_input(&mut self, _: Cycles, _: &mut dyn FnMut(u8)) {}
    fn slow_init_imminent(&mut self, _: Cycles) {}
}

/// Wraps another interface and records every call and received byte.
#[derive(Debug, Default)]
pub struct Traced<I> {
    pub inner: I,
    pub events: Vec<Stamped>,
}

impl<I> Traced<I> {
    pub fn new(inner: I) -> Self {
        Self {
            inner,
            events: Vec::new(),
        }
    }

    fn log(&mut self, at: Cycles, event: Event) {
        self.events.push(Stamped { at, event });
    }
}

impl<I: Interface> Interface for Traced<I> {
    fn set_baud_divisor(&mut self, now: Cycles, divisor: u32) {
        self.log(now, Event::BaudDivisor(divisor));
        self.inner.set_baud_divisor(now, divisor);
    }

    fn send_byte(&mut self, now: Cycles, byte: u8) {
        self.log(now, Event::Tx(byte));
        self.inner.send_byte(now, byte);
    }

    fn poll_input(&mut self, now: Cycles, rx: &mut dyn FnMut(u8)) {
        let events = &mut self.events;
        self.inner.poll_input(now, &mut |b| {
            events.push(Stamped {
                at: now,
                event: Event::Rx(b),
            });
            rx(b);
        });
    }

    fn slow_init_imminent(&mut self, now: Cycles) {
        self.log(now, Event::SlowInitImminent);
        self.inner.slow_init_imminent(now);
    }

    fn send_slow_init(&mut self, now: Cycles, target: u8) {
        self.inner.send_slow_init(now, target);
    }

    fn send_slow_init_bitwise(&mut self, now: Cycles, bit: bool) -> bool {
        self.log(now, Event::SlowInitBit(bit));
        self.inner.send_slow_init_bitwise(now, bit)
    }

    fn set_l(&mut self, now: Cycles, level: bool) {
        self.log(now, Event::L(level));
        self.inner.set_l(now, level);
    }

    fn set_can(&mut self, now: Cycles, on: bool) {
        self.log(now, Event::Can(on));
        self.inner.set_can(now, on);
    }

    fn rx_state(&mut self, now: Cycles) -> Option<bool> {
        self.inner.rx_state(now)
    }

    fn set_rx_bitbang(&mut self, now: Cycles, on: bool) {
        self.inner.set_rx_bitbang(now, on);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Answers each sent byte with its complement on the next poll.
    #[derive(Default)]
    struct Echo(Vec<u8>);

    impl Interface for Echo {
        fn set_baud_divisor(&mut self, _: Cycles, _: u32) {}
        fn send_byte(&mut self, _: Cycles, b: u8) {
            self.0.push(!b);
        }
        fn poll_input(&mut self, _: Cycles, rx: &mut dyn FnMut(u8)) {
            self.0.drain(..).for_each(rx);
        }
        fn slow_init_imminent(&mut self, _: Cycles) {}
    }

    #[test]
    fn traced_records_tx_rx_in_order() {
        let mut t = Traced::new(Echo::default());
        t.set_baud_divisor(1, 42);
        t.send_byte(10, 0x81);
        let mut got = vec![];
        t.poll_input(20, &mut |b| got.push(b));
        assert_eq!(got, [0x7e]);
        assert_eq!(
            t.events,
            [
                Stamped {
                    at: 1,
                    event: Event::BaudDivisor(42)
                },
                Stamped {
                    at: 10,
                    event: Event::Tx(0x81)
                },
                Stamped {
                    at: 20,
                    event: Event::Rx(0x7e)
                },
            ]
        );
    }

    #[test]
    fn null_is_silent() {
        let mut n = Null;
        n.send_byte(0, 1);
        n.poll_input(1, &mut |_| panic!("Null produced input"));
        assert!(!n.send_slow_init_bitwise(2, true));
        assert_eq!(n.rx_state(3), None);
    }
}
