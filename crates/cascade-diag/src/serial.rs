//! A real K-line through a serial adapter (KKL cable, Chipsoft in K-line
//! mode, ...), via the `serialport` crate. macOS arbitrary baud rates
//! (10,400) go through the IOSSIOSPEED ioctl inside `serialport`.
//!
//! **Untested on hardware**: no adapter has been connected yet. To verify
//! on one: 10,400 baud, BREAK on the line, and the echo (`kdiag selftest`).

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

/// Wait until `n` bytes just written have left the driver, then one more
/// byte time for the last one to shift out. Bounded: tcdrain(3) (what
/// `flush()` does) can block forever (seen on macOS pseudo-terminals in
/// about 1 of 10 test runs), and a hang on the car is worse than an error.
fn drain(port: &mut TTYPort, baud: u32, n: usize) -> io::Result<()> {
    let deadline = Instant::now() + byte_time(baud) * n as u32 + Duration::from_millis(200);
    loop {
        match port.bytes_to_write() {
            Ok(0) => break,
            Ok(_) if Instant::now() < deadline => std::thread::sleep(Duration::from_micros(100)),
            Ok(left) => {
                return Err(io::Error::new(
                    io::ErrorKind::TimedOut,
                    format!("serial output did not drain ({left} bytes left)"),
                ));
            }
            Err(e) => return Err(io::Error::from(e)),
        }
    }
    precise_sleep(byte_time(baud));
    Ok(())
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
        let mut line = Self::from_port(port, echo);
        line.kkl_mode();
        Ok(line)
    }

    /// Reset the adapter and put it in K-line (KKL) mode: DTR on and off,
    /// then DTR and RTS low, 100 ms each. Some knock-off adapters do no
    /// fast init without it (as found by gkbus). Ports without modem
    /// lines (pseudo-terminals) are left alone.
    fn kkl_mode(&mut self) {
        let pause = || std::thread::sleep(Duration::from_millis(100));
        if self.port.write_data_terminal_ready(true).is_err() {
            return;
        }
        pause();
        let _ = self.port.write_data_terminal_ready(false);
        pause();
        let _ = self.port.write_request_to_send(false);
        pause();
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

/// Serial devices that could be an adapter: `/dev/cu.*`, without the
/// Mac's built-in Bluetooth and debug ports.
pub fn list_ports() -> Vec<String> {
    serialport::available_ports()
        .map(|v| {
            v.into_iter()
                .map(|p| p.port_name)
                .filter(|n| {
                    n.starts_with("/dev/cu.")
                        && !n.contains("Bluetooth")
                        && !n.contains("debug-console")
                })
                .collect()
        })
        .unwrap_or_default()
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
                drain(&mut self.port, baud, 1)?;
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

    fn five_baud(&mut self, addr: u8) -> io::Result<()> {
        // Bit-banged with break (K low) and no break (K high), whatever the
        // wake method: USB serial chips cannot run at 5 baud. Bits are 200 ms,
        // so the USB latency of each break toggle (about 1 ms) is harmless.
        let bit = Duration::from_millis(200);
        let start = Instant::now();
        let bits = std::iter::once(false)
            .chain((0..8).map(|i| addr >> i & 1 == 1))
            .chain(std::iter::once(true));
        for (i, high) in bits.enumerate() {
            if high {
                self.port.clear_break().map_err(io::Error::from)?;
            } else {
                self.port.set_break().map_err(io::Error::from)?;
            }
            // sleep to the bit boundary, so toggle delays do not add up
            precise_sleep((bit * (i as u32 + 1)).saturating_sub(start.elapsed()));
        }
        // a break can leave 00 bytes in the receive buffer
        self.port.clear(ClearBuffer::Input).map_err(io::Error::from)
    }

    fn write_unchecked(&mut self, data: &[u8]) -> io::Result<()> {
        self.port.write_all(data)?;
        drain(&mut self.port, self.baud, data.len())
    }

    fn write(&mut self, data: &[u8]) -> io::Result<()> {
        self.port.write_all(data)?;
        drain(&mut self.port, self.baud, data.len())?;
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

/// Outcome of one self-test check.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Status {
    Pass,
    Warn,
    Fail,
    Info,
}

/// One line of the self-test report.
#[derive(Clone, Debug)]
pub struct Check {
    pub status: Status,
    pub name: &'static str,
    pub detail: String,
}

fn check(status: Status, name: &'static str, detail: impl Into<String>) -> Check {
    Check {
        status,
        name,
        detail: detail.into(),
    }
}

/// Bytes the adapter heard within `timeout`, stopping early once `n` arrived.
fn hear(line: &mut SerialLine, n: usize, timeout: Duration) -> io::Result<Vec<u8>> {
    line.read_exact_timeout(n, timeout)
}

/// Adapter self-test: what the Phase 3 checklist asks of a K-line adapter.
///
/// Sends only bytes that are not valid KWP2000 frames, and wake-up pulses
/// without a StartCommunication, so a connected ECU ignores it. Run it with
/// 12 V on OBD pin 16 (car with ignition off, or a bench supply): without
/// that most adapters' K-line transceivers are dead and nothing echoes.
pub fn selftest(line: &mut SerialLine) -> Vec<Check> {
    let mut out = Vec::new();

    // 1. baud rates
    match line.port.set_baud_rate(10_400) {
        Ok(()) => out.push(check(
            Status::Pass,
            "10,400 baud",
            "accepted by the driver (IOSSIOSPEED)",
        )),
        Err(e) => out.push(check(
            Status::Fail,
            "10,400 baud",
            format!("refused: {e}. KWP2000 on K-line needs it."),
        )),
    }
    let slow = line.port.set_baud_rate(360);
    out.push(match slow {
        Ok(()) => check(
            Status::Pass,
            "360 baud",
            "accepted (needed for --wake zero)",
        ),
        Err(e) => check(
            Status::Warn,
            "360 baud",
            format!("refused: {e}. --wake zero will not work; BREAK must."),
        ),
    });
    let _ = line.port.set_baud_rate(10_400);
    line.baud = 10_400;

    // 2. echo
    let pattern = [0x55u8, 0xaa, 0x0f, 0xf0, 0x00, 0xff];
    let _ = line.flush_input();
    let echo = line
        .port
        .write_all(&pattern)
        .and_then(|_| drain(&mut line.port, 10_400, pattern.len()))
        .and_then(|_| hear(line, pattern.len(), Duration::from_millis(300)));
    let echoes = match &echo {
        Ok(b) if b.as_slice() == pattern => {
            out.push(check(
                Status::Pass,
                "echo",
                "the adapter hears its own bytes: transceiver powered, wiring sane",
            ));
            true
        }
        Ok(b) if b.is_empty() => {
            out.push(check(
                Status::Warn,
                "echo",
                "nothing heard. Either the adapter has no 12 V on OBD pin 16 (most K-line \
                 transceivers need it), or it filters the echo (then use --no-echo). \
                 The remaining line checks need an echo and are skipped.",
            ));
            false
        }
        Ok(b) => {
            out.push(check(
                Status::Fail,
                "echo",
                format!("sent {pattern:02X?}, heard {b:02X?}: wrong baud rate, a noisy line, or another tester on the bus"),
            ));
            false
        }
        Err(e) => {
            out.push(check(Status::Fail, "echo", format!("I/O error: {e}")));
            false
        }
    };

    if echoes {
        // 3. echo latency: one byte out, its echo back
        let mut rtts = Vec::new();
        for _ in 0..10 {
            let _ = line.flush_input();
            let t = Instant::now();
            if line
                .port
                .write_all(&[0x5a])
                .and_then(|_| drain(&mut line.port, 10_400, 1))
                .is_err()
            {
                break;
            }
            if let Ok(b) = hear(line, 1, Duration::from_millis(200))
                && b == [0x5a]
            {
                rtts.push(t.elapsed());
            }
            precise_sleep(Duration::from_millis(20));
        }
        rtts.sort();
        if let (Some(min), Some(max)) = (rtts.first(), rtts.last()) {
            let med = rtts[rtts.len() / 2];
            let status = if med < Duration::from_millis(8) {
                Status::Pass
            } else {
                Status::Warn
            };
            out.push(check(
                status,
                "echo latency",
                format!(
                    "{} of 10 bytes, median {:.1} ms (min {:.1}, max {:.1}); one byte takes 0.96 ms on the wire. \
                     Above ~8 ms, USB latency eats into the ECU's 25-50 ms reply window.",
                    rtts.len(),
                    med.as_secs_f64() * 1e3,
                    min.as_secs_f64() * 1e3,
                    max.as_secs_f64() * 1e3
                ),
            ));
        }

        // 4. BREAK: the line should be pulled low, which the receiver reads
        // as 0x00 bytes (with framing errors) at 10,400 baud
        let _ = line.flush_input();
        let brk = line
            .port
            .set_break()
            .map(|_| precise_sleep(Duration::from_millis(25)))
            .and_then(|_| line.port.clear_break());
        let heard = hear(line, 8, Duration::from_millis(60)).unwrap_or_default();
        out.push(match (brk, heard.contains(&0)) {
            (Err(e), _) => check(Status::Fail, "BREAK", format!("driver refused: {e}. Use --wake zero.")),
            (Ok(()), true) => check(
                Status::Pass,
                "BREAK",
                format!("line went low (heard {heard:02X?}); --wake break (default) should work"),
            ),
            (Ok(()), false) => check(
                Status::Warn,
                "BREAK",
                "accepted by the driver but not seen on the line; the adapter may ignore BREAK. Try --wake zero.",
            ),
        });

        // 5. zero-byte wake-up: 0x00 at 360 baud is ~25 ms low
        let _ = line.flush_input();
        let zero = line
            .port
            .set_baud_rate(360)
            .and_then(|_| line.port.write_all(&[0x00]).map_err(Into::into))
            .map_err(io::Error::from)
            .and_then(|_| drain(&mut line.port, 360, 1));
        let heard = hear(line, 1, Duration::from_millis(150)).unwrap_or_default();
        let _ = line.port.set_baud_rate(10_400);
        out.push(match (zero, heard.as_slice()) {
            (Err(e), _) => check(
                Status::Warn,
                "zero-byte wake",
                format!("could not send: {e}"),
            ),
            (Ok(()), [0x00]) => check(
                Status::Pass,
                "zero-byte wake",
                "0x00 at 360 baud echoed; --wake zero works",
            ),
            (Ok(()), h) => check(
                Status::Warn,
                "zero-byte wake",
                format!("heard {h:02X?} instead of 00"),
            ),
        });
    }

    // 6. host timing of a 25 ms pulse (what we control on our side)
    let mut errs = Vec::new();
    for _ in 0..10 {
        let t = Instant::now();
        precise_sleep(Duration::from_millis(25));
        errs.push(t.elapsed().as_secs_f64() * 1e3 - 25.0);
    }
    let worst = errs.iter().cloned().fold(0.0f64, f64::max);
    out.push(check(
        if worst < 1.0 {
            Status::Pass
        } else {
            Status::Warn
        },
        "host timing",
        format!("25 ms pulse overshoots by at most {worst:.2} ms (ISO 14230 allows 24-26 ms)"),
    ));
    out
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

    /// Self-test against a pty whose far end echoes everything back: the
    /// echo and latency checks pass; baud and BREAK cannot pass on a pty.
    #[test]
    fn selftest_with_echoing_pty() {
        let run = || {
            let (a, mut b) = TTYPort::pair().expect("pty pair");
            b.set_timeout(Duration::from_millis(5)).unwrap();
            let stop = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
            let stop2 = stop.clone();
            let echoer = std::thread::spawn(move || {
                let mut buf = [0u8; 64];
                while !stop2.load(std::sync::atomic::Ordering::Relaxed) {
                    if let Ok(n) = b.read(&mut buf) {
                        let _ = b.write_all(&buf[..n]);
                    }
                }
            });
            let mut line = SerialLine::from_port(a, true);
            let report = selftest(&mut line);
            stop.store(true, std::sync::atomic::Ordering::Relaxed);
            echoer.join().unwrap();
            report
        };
        // A macOS pty occasionally stops passing bytes (output stuck in the
        // queue although the far end reads). It used to hang in tcdrain; now
        // `drain` reports it. That is the pty, not the code under test: retry.
        let stalled = |r: &[Check]| r.iter().any(|c| c.detail.contains("did not drain"));
        let mut report = run();
        for _ in 0..2 {
            if !stalled(&report) {
                break;
            }
            report = run();
        }
        let get = |n: &str| report.iter().find(|c| c.name == n).map(|c| c.status);
        assert_eq!(get("echo"), Some(Status::Pass), "{report:#?}");
        assert_eq!(get("echo latency"), Some(Status::Pass), "{report:#?}");
        // macOS ptys refuse IOSSIOSPEED; Linux ptys accept any baud rate
        let expected = if cfg!(target_os = "macos") {
            Status::Fail
        } else {
            Status::Pass
        };
        assert_eq!(get("10,400 baud"), Some(expected), "{report:#?}");
        assert!(get("host timing").is_some());
    }

    #[test]
    fn selftest_without_echo_warns_and_skips() {
        // the far end reads and discards (an adapter that does not echo);
        // without a reader, tcdrain on a pty blocks forever
        let (a, mut b) = TTYPort::pair().expect("pty pair");
        b.set_timeout(Duration::from_millis(5)).unwrap();
        let stop = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        let stop2 = stop.clone();
        let sink = std::thread::spawn(move || {
            let mut buf = [0u8; 64];
            while !stop2.load(std::sync::atomic::Ordering::Relaxed) {
                let _ = b.read(&mut buf);
            }
        });
        let mut line = SerialLine::from_port(a, true);
        let report = selftest(&mut line);
        stop.store(true, std::sync::atomic::Ordering::Relaxed);
        sink.join().unwrap();
        let echo = report.iter().find(|c| c.name == "echo").unwrap();
        assert_eq!(echo.status, Status::Warn);
        assert!(
            report.iter().all(|c| c.name != "BREAK"),
            "line checks need an echo"
        );
    }

    #[test]
    fn precise_sleep_is_precise() {
        let t = Instant::now();
        precise_sleep(Duration::from_millis(25));
        let e = t.elapsed();
        assert!(e >= Duration::from_millis(25), "{e:?}");
        // shared CI runners stall for tens of ms; check the upper bound locally
        if std::env::var_os("CI").is_none() {
            assert!(e < Duration::from_millis(35), "{e:?}");
        }
    }
}
