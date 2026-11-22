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

    fn five_baud(&mut self, addr: u8) -> io::Result<()> {
        self.event(&format!("\"ev\":\"five_baud\",\"addr\":{addr}"));
        self.inner.five_baud(addr)
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
    fn five_baud(&mut self, addr: u8) -> io::Result<()> {
        (**self).five_baud(addr)
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

/// Frame-level tracing for any [`Link`] (used for J2534, where single bytes
/// are not visible). Writes the same JSONL events as [`TracedLine`]: a
/// `wake` per fast init, `tx` per frame, `rx` per received byte, so the
/// recordings replay the same way (`cascade_iface::replay`).
pub struct TracedLink<K: crate::kwp::Link, W: Write> {
    inner: K,
    out: W,
}

impl<K: crate::kwp::Link, W: Write> TracedLink<K, W> {
    pub fn new(inner: K, out: W) -> Self {
        Self { inner, out }
    }

    fn event(&mut self, body: &str) {
        let t = self.inner.elapsed().as_micros();
        let _ = writeln!(self.out, "{{\"t_us\":{t},{body}}}");
    }

    fn rx(&mut self, frame: &[u8]) {
        for b in frame {
            self.event(&format!("\"ev\":\"rx\",\"byte\":\"{b:02X}\""));
        }
    }
}

impl<K: crate::kwp::Link, W: Write> crate::kwp::Link for TracedLink<K, W> {
    fn elapsed(&self) -> Duration {
        self.inner.elapsed()
    }

    fn idle(&mut self, d: Duration) {
        self.inner.idle(d)
    }

    fn fast_init(
        &mut self,
        start_frame: &[u8],
        timing: &crate::kwp::Timing,
    ) -> Result<Vec<u8>, crate::kwp::Error> {
        self.event(&format!(
            "\"ev\":\"wake\",\"low_us\":{},\"high_us\":{}",
            timing.wake_low.as_micros(),
            timing.wake_high.as_micros()
        ));
        self.event(&format!("\"ev\":\"tx\",\"data\":\"{}\"", hex(start_frame)));
        let r = self.inner.fast_init(start_frame, timing);
        match &r {
            Ok(f) => self.rx(f),
            Err(e) => self.event(&format!(
                "\"ev\":\"timeout\",\"error\":\"{}\"",
                e.to_string().replace('"', "'")
            )),
        }
        r
    }

    fn send(&mut self, frame: &[u8]) -> Result<(), crate::kwp::Error> {
        self.event(&format!("\"ev\":\"tx\",\"data\":\"{}\"", hex(frame)));
        self.inner.send(frame)
    }

    fn recv(&mut self, first: Duration, p1: Duration) -> Result<Vec<u8>, crate::kwp::Error> {
        let r = self.inner.recv(first, p1);
        match &r {
            Ok(f) => self.rx(f),
            Err(_) => self.event(&format!(
                "\"ev\":\"timeout\",\"after_us\":{}",
                first.as_micros()
            )),
        }
        r
    }

    fn flush_input(&mut self) -> Result<(), crate::kwp::Error> {
        self.inner.flush_input()
    }
}
