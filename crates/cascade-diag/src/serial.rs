//! A real K-line through a serial adapter (KKL cable, Chipsoft in K-line
//! mode, ...), via the `serialport` crate. macOS arbitrary baud rates
//! (10,400) go through the IOSSIOSPEED ioctl inside `serialport`.
//!
//! **Untested on hardware**: no adapter has been connected yet. The Phase 3
//! checklist in CLAUDE.md lists what to verify (baud, break, echo).

use std::io::{self, Read, Write};
use std::time::{Duration, Instant};

use serialport::{ClearBuffer, DataBits, FlowControl, Parity, SerialPort, StopBits, TTYPort};

use crate::line::{Line, byte_time};

/// How to produce the fast-init low pulse.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WakeMethod {
    /// TIOCSBRK/TIOCCBRK around a timed sleep. Precise if the adapter
    /// honours BREAK promptly.
    Break,
    /// Send 0x00 at a baud rate where start bit plus 8 data bits last the
    /// low time (360 baud for 25 ms). For adapters that ignore BREAK.
    ZeroByte,
}

pub struct SerialLine {
    port: TTYPort,
    start: Instant,
    baud: u32,
    /// The adapter hears its own transmissions (normal for K-line).
    pub echo: bool,
    pub wake: WakeMethod,
    /// Pseudo-terminals cannot set baud rates (ENOTTY); for tests only.
    /// Never set this for a real adapter.
    pub ignore_baud_errors: bool,
}

impl SerialLine {
    pub fn open(path: &str, echo: bool) -> io::Result<Self> {
        let port = serialport::new(path, 10_400)
            .data_bits(DataBits::Eight)
            .parity(Parity::None)
            .stop_bits(StopBits::One)
            .flow_control(FlowControl::None)
            .timeout(Duration::from_millis(100))
            .open_native()
            .map_err(io::Error::from)?;
        Ok(Self::from_port(port, echo))
    }

    pub fn from_port(port: TTYPort, echo: bool) -> Self {
        Self {
            port,
            start: Instant::now(),
            baud: 10_400,
            echo,
            wake: WakeMethod::Break,
            ignore_baud_errors: false,
        }
    }

    fn read_exact_timeout(&mut self, n: usize, timeout: Duration) -> io::Result<Vec<u8>> {
        let deadline = Instant::now() + timeout;
        let mut got = Vec::with_capacity(n);
        while got.len() < n {
            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                break;
            }
            if let Some(b) = self.read_byte(left)? {
                got.push(b);
            }
        }
        Ok(got)
    }
}

/// Sleep with sub-millisecond accuracy: OS sleep, then spin the last 2 ms.
pub fn precise_sleep(d: Duration) {
    let end = Instant::now() + d;
    if d > Duration::from_millis(3) {
        std::thread::sleep(d - Duration::from_millis(2));
    }
    while Instant::now() < end {
        std::hint::spin_loop();
    }
}

impl Line for SerialLine {
    fn elapsed(&self) -> Duration {
        self.start.elapsed()
    }

    fn set_baud(&mut self, baud: u32) -> io::Result<()> {
        match self.port.set_baud_rate(baud) {
            Err(_) if self.ignore_baud_errors => {}
            r => r.map_err(io::Error::from)?,
        }
        self.baud = baud;
        Ok(())
    }

    fn wake_up(&mut self, low: Duration, high: Duration) -> io::Result<()> {
        match self.wake {
            WakeMethod::Break => {
                self.port.set_break().map_err(io::Error::from)?;
                precise_sleep(low);
                self.port.clear_break().map_err(io::Error::from)?;
                precise_sleep(high);
            }
            WakeMethod::ZeroByte => {
                let normal = self.baud;
                // 9 bit times (start + 8 zero data bits) = low
                let baud = (9_000_000 / low.as_micros().max(1) as u64) as u32;
                self.port.set_baud_rate(baud).map_err(io::Error::from)?;
                let t0 = Instant::now();
                self.port.write_all(&[0x00])?;
                self.port.flush()?;
                // the stop bit already belongs to the high phase
                let spent = t0.elapsed();
                self.port.set_baud_rate(normal).map_err(io::Error::from)?;
                precise_sleep((low + high).saturating_sub(spent));
                self.port
                    .clear(ClearBuffer::Input)
                    .map_err(io::Error::from)?;
            }
        }
        Ok(())
    }

    fn write(&mut self, data: &[u8]) -> io::Result<()> {
        self.port.write_all(data)?;
        self.port.flush()?;
        if self.echo {
            let timeout = byte_time(self.baud) * data.len() as u32 + Duration::from_millis(50);
            let back = self.read_exact_timeout(data.len(), timeout)?;
            if back != data {
                return Err(io::Error::other(format!(
                    "K-line echo mismatch: sent {:02X?}, heard {:02X?} (collision, wrong wiring, or adapter does not echo: try --no-echo)",
                    data, back
                )));
            }
        }
        Ok(())
    }

    fn read_byte(&mut self, timeout: Duration) -> io::Result<Option<u8>> {
        self.port
            .set_timeout(timeout.max(Duration::from_millis(1)))
            .map_err(io::Error::from)?;
        let mut b = [0u8; 1];
        match self.port.read(&mut b) {
            Ok(1) => Ok(Some(b[0])),
            Ok(_) => Ok(None),
            Err(e) if e.kind() == io::ErrorKind::TimedOut => Ok(None),
            Err(e) => Err(e),
        }
    }

    fn flush_input(&mut self) -> io::Result<()> {
        self.port.clear(ClearBuffer::Input).map_err(io::Error::from)
    }

    fn idle(&mut self, d: Duration) {
        precise_sleep(d);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A pseudo-terminal pair stands in for the adapter: no baud rates or
    /// BREAK, but real reads, writes and timeouts through the same code.
    /// The far end is read on a thread, because tcdrain on a pty waits for
    /// the other side to read.
    #[test]
    fn pty_write_read_and_echo_check() {
        let (a, mut b) = TTYPort::pair().expect("pty pair");
        b.set_timeout(Duration::from_millis(500)).unwrap();
        let mut far = b.try_clone_native().unwrap();
        let reader = std::thread::spawn(move || {
            let mut got = [0u8; 5];
            far.read_exact(&mut got).map(|_| got)
        });
        let mut line = SerialLine::from_port(a, false);
        line.write(&[0x81, 0x11, 0xf1, 0x81, 0x04]).unwrap();
        assert_eq!(
            reader.join().unwrap().unwrap(),
            [0x81, 0x11, 0xf1, 0x81, 0x04]
        );

        b.write_all(&[0x83, 0xf1]).unwrap();
        assert_eq!(
            line.read_byte(Duration::from_millis(200)).unwrap(),
            Some(0x83)
        );
        assert_eq!(
            line.read_byte(Duration::from_millis(200)).unwrap(),
            Some(0xf1)
        );
        let t = Instant::now();
        assert_eq!(line.read_byte(Duration::from_millis(30)).unwrap(), None);
        assert!(t.elapsed() >= Duration::from_millis(25), "timeout honoured");

        // with echo expected but nobody echoing, write must fail loudly
        let mut drain = b.try_clone_native().unwrap();
        let drainer = std::thread::spawn(move || {
            let mut x = [0u8; 1];
            let _ = drain.read(&mut x);
        });
        line.echo = true;
        assert!(line.write(&[0x3e]).is_err());
        drainer.join().unwrap();
    }

    /// The whole stack on a real serial path: KWP client and SerialLine on
    /// one end of a pty, the KwpEcu model served in real time on the other.
    #[test]
    fn kwp_session_over_pty_in_real_time() {
        use crate::kwp::Client;
        use cascade_iface::{Interface, KwpConfig, KwpEcu, RxBuf};
        use std::sync::Arc;
        use std::sync::atomic::{AtomicBool, Ordering};

        struct Out(Vec<u8>);
        impl RxBuf for Out {
            fn push(&mut self, b: u8) {
                self.0.push(b);
            }
            fn flush(&mut self) {
                self.0.clear();
            }
        }
        let (mut ecu_end, tester_end) = TTYPort::pair().expect("pty pair");
        ecu_end.set_timeout(Duration::from_millis(1)).unwrap();
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = stop.clone();
        let server = std::thread::spawn(move || {
            let mut cfg = KwpConfig {
                dtcs: vec![(0x0130, 0x20)],
                ..Default::default()
            };
            cfg.local_data.insert(0x01, (0..80).collect());
            let mut ecu = KwpEcu::new(cfg);
            let t0 = Instant::now();
            let now = || t0.elapsed().as_micros() as u64 * 10; // 10 cycles per us
            let (mut out, mut b) = (Out(Vec::new()), [0u8; 64]);
            while !stop2.load(Ordering::Relaxed) {
                if let Ok(n) = ecu_end.read(&mut b) {
                    for &x in &b[..n] {
                        ecu.send_byte(now(), x, &mut out);
                    }
                }
                ecu.poll_input(now(), &mut out);
                if !out.0.is_empty() {
                    ecu_end.write_all(&out.0).unwrap();
                    out.0.clear();
                }
            }
        });
        let mut line = SerialLine::from_port(tester_end, false);
        line.ignore_baud_errors = true;
        let mut c = Client::new(line);
        let result = (|| {
            assert_eq!(c.start()?, [0xef, 0x8f]);
            assert_eq!(c.read_dtcs()?, [(0x0130, 0x20)]);
            assert_eq!(c.read_local(0x01)?, (0..80).collect::<Vec<u8>>()); // long frame
            c.tester_present()?;
            c.stop()
        })();
        stop.store(true, Ordering::Relaxed);
        server.join().unwrap();
        result.expect("session over pty");
    }

    #[test]
    fn precise_sleep_is_precise() {
        let t = Instant::now();
        precise_sleep(Duration::from_millis(25));
        let e = t.elapsed();
        assert!(
            e >= Duration::from_millis(25) && e < Duration::from_millis(35),
            "{e:?}"
        );
    }
}
