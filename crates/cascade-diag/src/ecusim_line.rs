//! A [`Line`] to an `ecusim` server (the GK 2.0 ECU dump running as its own
//! process): bytes, the fast-init pulse and 5-baud bits go over its socket,
//! stamped with wall time. The server does not echo the tester's bytes.

use std::collections::VecDeque;
use std::io;
use std::path::Path;
use std::time::{Duration, Instant};

use ecusim::client::Conn;
use ecusim::proto::{ToClient, ToServer};

use crate::line::{Line, byte_time};
use crate::serial::precise_sleep;

pub struct EcusimLine {
    conn: Conn,
    start: Instant,
    baud: u32,
    rx: VecDeque<u8>,
}

impl EcusimLine {
    /// Connect as a tester on a Hi-Scan line number (7 engine, 8 ABS/TCS,
    /// 12 SRS).
    pub fn connect(socket: &Path, line: i32) -> io::Result<Self> {
        let mut conn = Conn::connect(socket)?;
        conn.send(&ToServer::KLine)?;
        conn.send(&ToServer::Line(line))?;
        Ok(Self {
            conn,
            start: Instant::now(),
            baud: 10_400,
            rx: VecDeque::new(),
        })
    }

    fn us(&self) -> u64 {
        self.start.elapsed().as_micros() as u64
    }

    fn take(&mut self, m: ToClient) {
        if let ToClient::Rx(b) = m {
            self.rx.extend(b);
        }
    }
}

impl Line for EcusimLine {
    fn elapsed(&self) -> Duration {
        self.start.elapsed()
    }

    fn set_baud(&mut self, baud: u32) -> io::Result<()> {
        self.baud = baud;
        self.conn.send(&ToServer::Baud(baud))
    }

    fn wake_up(&mut self, low: Duration, high: Duration) -> io::Result<()> {
        let t = self.us();
        self.conn.send(&ToServer::Low { us: t })?;
        precise_sleep(low);
        self.conn.send(&ToServer::High {
            us: t + low.as_micros() as u64,
        })?;
        precise_sleep(high);
        Ok(())
    }

    fn five_baud(&mut self, addr: u8) -> io::Result<()> {
        let bit = 200_000u64;
        let t = self.us();
        let bits = std::iter::once(false)
            .chain((0..8).map(|i| addr >> i & 1 == 1))
            .chain(std::iter::once(true));
        let mut low_since = None;
        for (i, high) in bits.enumerate() {
            let at = t + i as u64 * bit;
            match (high, low_since) {
                (false, None) => {
                    self.conn.send(&ToServer::Low { us: at })?;
                    low_since = Some(at);
                }
                (true, Some(_)) => {
                    self.conn.send(&ToServer::High { us: at })?;
                    low_since = None;
                }
                _ => {}
            }
            precise_sleep(Duration::from_micros(bit));
        }
        self.conn.send(&ToServer::Slow {
            us: t + 10 * bit,
            addr,
        })?;
        Ok(())
    }

    fn write(&mut self, data: &[u8]) -> io::Result<()> {
        let t = self.us();
        self.conn.send(&ToServer::Tx {
            us: t,
            bytes: data.to_vec(),
        })?;
        // the bytes take their time on the wire
        precise_sleep(byte_time(self.baud) * data.len() as u32);
        Ok(())
    }

    fn read_byte(&mut self, timeout: Duration) -> io::Result<Option<u8>> {
        let deadline = Instant::now() + timeout;
        loop {
            if let Some(b) = self.rx.pop_front() {
                return Ok(Some(b));
            }
            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                return Ok(None);
            }
            if let Some(m) = self.conn.recv_timeout(left)? {
                self.take(m);
            }
        }
    }

    fn flush_input(&mut self) -> io::Result<()> {
        while let Some(m) = self.conn.try_recv() {
            self.take(m);
        }
        self.rx.clear();
        Ok(())
    }

    fn idle(&mut self, d: Duration) {
        precise_sleep(d);
    }
}
