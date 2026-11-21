//! A K-line in virtual time, attached to an emulated ECU (`cascade-iface`
//! backend such as `KwpEcu`). Used for tests and `kdiag --sim`.

use std::collections::VecDeque;
use std::io;
use std::time::Duration;

use cascade_iface::{Cycles, Interface, RxBuf};

use crate::line::{Line, byte_time};

/// Emulated cycles per millisecond, as in `cascade_iface::kwp`.
const CYCLES_PER_MS: u64 = 10_000;

#[derive(Default)]
struct Rx(VecDeque<u8>);

impl RxBuf for Rx {
    fn push(&mut self, byte: u8) {
        self.0.push_back(byte);
    }
    fn flush(&mut self) {
        self.0.clear();
    }
}

pub struct SimLine<I: Interface> {
    pub ecu: I,
    now: Cycles,
    baud: u32,
    rx: Rx,
}

impl<I: Interface> SimLine<I> {
    pub fn new(ecu: I) -> Self {
        Self {
            ecu,
            now: 0,
            baud: 10_400,
            rx: Rx::default(),
        }
    }

    fn advance(&mut self, d: Duration) {
        self.now += (d.as_nanos() as u64 * CYCLES_PER_MS) / 1_000_000;
    }
}

impl<I: Interface> Line for SimLine<I> {
    fn elapsed(&self) -> Duration {
        Duration::from_nanos(self.now * 1_000_000 / CYCLES_PER_MS)
    }

    fn set_baud(&mut self, baud: u32) -> io::Result<()> {
        self.baud = baud;
        Ok(())
    }

    fn wake_up(&mut self, low: Duration, high: Duration) -> io::Result<()> {
        self.ecu
            .send_slow_init_bitwise(self.now, false, &mut self.rx);
        self.advance(low);
        self.ecu
            .send_slow_init_bitwise(self.now, true, &mut self.rx);
        self.advance(high);
        Ok(())
    }

    fn write(&mut self, data: &[u8]) -> io::Result<()> {
        for &b in data {
            self.advance(byte_time(self.baud));
            self.ecu.send_byte(self.now, b, &mut self.rx);
        }
        Ok(())
    }

    fn read_byte(&mut self, timeout: Duration) -> io::Result<Option<u8>> {
        let deadline = self.now + (timeout.as_nanos() as u64 * CYCLES_PER_MS) / 1_000_000;
        loop {
            self.ecu.poll_input(self.now, &mut self.rx);
            if let Some(b) = self.rx.0.pop_front() {
                self.advance(byte_time(self.baud));
                return Ok(Some(b));
            }
            if self.now >= deadline {
                return Ok(None);
            }
            self.now = (self.now + CYCLES_PER_MS / 10).min(deadline);
        }
    }

    fn flush_input(&mut self) -> io::Result<()> {
        self.rx.0.clear();
        Ok(())
    }

    fn idle(&mut self, d: Duration) {
        self.advance(d);
    }
}
