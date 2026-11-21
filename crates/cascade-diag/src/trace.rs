//! JSONL tracing of K-line traffic: one object per event, with the line's
//! time in microseconds. Written by hand to avoid a JSON dependency; all
//! fields are numbers or fixed strings.

use std::io::{self, Write};
use std::time::Duration;

use crate::kwp::hex;
use crate::line::Line;

pub struct TracedLine<L: Line, W: Write> {
    inner: L,
    out: W,
}

impl<L: Line, W: Write> TracedLine<L, W> {
    pub fn new(inner: L, out: W) -> Self {
        Self { inner, out }
    }

    fn event(&mut self, body: &str) {
        let t = self.inner.elapsed().as_micros();
        // tracing must never break the session; drop write errors
        let _ = writeln!(self.out, "{{\"t_us\":{t},{body}}}");
    }
}

impl<L: Line, W: Write> Line for TracedLine<L, W> {
    fn elapsed(&self) -> Duration {
        self.inner.elapsed()
    }

    fn set_baud(&mut self, baud: u32) -> io::Result<()> {
        self.event(&format!("\"ev\":\"baud\",\"baud\":{baud}"));
        self.inner.set_baud(baud)
    }

    fn wake_up(&mut self, low: Duration, high: Duration) -> io::Result<()> {
        self.event(&format!(
            "\"ev\":\"wake\",\"low_us\":{},\"high_us\":{}",
            low.as_micros(),
            high.as_micros()
        ));
        self.inner.wake_up(low, high)
    }

    fn write(&mut self, data: &[u8]) -> io::Result<()> {
        self.event(&format!("\"ev\":\"tx\",\"data\":\"{}\"", hex(data)));
        self.inner.write(data)
    }

    fn read_byte(&mut self, timeout: Duration) -> io::Result<Option<u8>> {
        let r = self.inner.read_byte(timeout)?;
        match r {
            Some(b) => self.event(&format!("\"ev\":\"rx\",\"byte\":\"{b:02X}\"")),
            None => self.event(&format!(
                "\"ev\":\"timeout\",\"after_us\":{}",
                timeout.as_micros()
            )),
        }
        Ok(r)
    }

    fn flush_input(&mut self) -> io::Result<()> {
        self.inner.flush_input()
    }

    fn idle(&mut self, d: Duration) {
        self.inner.idle(d)
    }
}

impl<T: Line + ?Sized> Line for Box<T> {
    fn elapsed(&self) -> Duration {
        (**self).elapsed()
    }
    fn set_baud(&mut self, baud: u32) -> io::Result<()> {
        (**self).set_baud(baud)
    }
    fn wake_up(&mut self, low: Duration, high: Duration) -> io::Result<()> {
        (**self).wake_up(low, high)
    }
    fn write(&mut self, data: &[u8]) -> io::Result<()> {
        (**self).write(data)
    }
    fn read_byte(&mut self, timeout: Duration) -> io::Result<Option<u8>> {
        (**self).read_byte(timeout)
    }
    fn flush_input(&mut self) -> io::Result<()> {
        (**self).flush_input()
    }
    fn idle(&mut self, d: Duration) {
        (**self).idle(d)
    }
}
