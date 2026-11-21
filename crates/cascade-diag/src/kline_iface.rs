//! A real K-line as a `cascade_iface::Interface`, so the emulated Hi-Scan
//! firmware can talk to a car through a serial adapter. Port of upstream
//! `iface_kl_tty.cpp`:
//! - a reader thread collects received bytes;
//! - bytes we sent are cancelled from the adapter's echo (echo ring);
//! - zero bytes are ignored around BREAK (slow/fast init), as upstream does;
//! - init bits become BREAK set/clear; the L line is RTS.
//!
//! Timing: upstream calls `sync(true)` before every line action so that
//! wall time matches emulated time at that moment. Here the interface keeps
//! its own line clock: before each physical action it waits until the wall
//! time since the previous action has caught up with the emulated time since
//! it. A 25 ms emulated init pulse is therefore ~25 ms on the wire even
//! though the runner paces in coarser slices.
//!
//! **Untested on hardware.** Tested end to end over a pseudo-terminal.

use std::collections::VecDeque;
use std::io::{Read, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use cascade_iface::{Cycles, Interface, RxBuf};
use serialport::{ClearBuffer, SerialPort, TTYPort};

use crate::serial::precise_sleep;

/// Emulated cycles per millisecond (the core's clock bookkeeping).
const CYCLES_PER_MS: u64 = 10_000;
/// Emulated gaps longer than this are not reproduced on the wire (pauses,
/// menus); the line clock simply restarts.
const MAX_SYNC: Duration = Duration::from_secs(2);

#[derive(Default)]
struct Shared {
    rx: VecDeque<u8>,
    echo: VecDeque<u8>,
    ignore_breaks: bool,
}

pub struct SerialKLine {
    port: TTYPort,
    shared: Arc<Mutex<Shared>>,
    stop: Arc<AtomicBool>,
    reader: Option<JoinHandle<()>>,
    /// The adapter hears its own bytes and they must be cancelled.
    hw_echo: bool,
    /// Pseudo-terminals cannot set baud rates; for tests only.
    pub ignore_baud_errors: bool,
    baud: u32,
    clock: Option<(Instant, Cycles)>,
}

impl SerialKLine {
    /// Open a serial adapter by path (`/dev/cu.…`).
    pub fn open(path: &str, hw_echo: bool) -> std::io::Result<Self> {
        let port = serialport::new(path, 10_400)
            .timeout(Duration::from_millis(5))
            .open_native()
            .map_err(std::io::Error::from)?;
        Self::from_port(port, hw_echo)
    }

    /// Use an already open port (tests use one end of a pty pair).
    pub fn from_port(port: TTYPort, hw_echo: bool) -> std::io::Result<Self> {
        let mut reader_port = port.try_clone_native().map_err(std::io::Error::from)?;
        reader_port
            .set_timeout(Duration::from_millis(5))
            .map_err(std::io::Error::from)?;
        let shared = Arc::new(Mutex::new(Shared::default()));
        let stop = Arc::new(AtomicBool::new(false));
        let (s2, stop2) = (shared.clone(), stop.clone());
        let reader = std::thread::Builder::new()
            .name("kline-rx".into())
            .spawn(move || {
                let mut buf = [0u8; 64];
                while !stop2.load(Ordering::Relaxed) {
                    let n = match reader_port.read(&mut buf) {
                        Ok(n) => n,
                        Err(e) if e.kind() == std::io::ErrorKind::TimedOut => 0,
                        Err(_) => {
                            std::thread::sleep(Duration::from_millis(5));
                            0
                        }
                    };
                    if n == 0 {
                        continue;
                    }
                    let Ok(mut sh) = s2.lock() else { return };
                    for &b in &buf[..n] {
                        if sh.echo.front() == Some(&b) {
                            sh.echo.pop_front(); // our own byte coming back
                        } else if sh.ignore_breaks && b == 0 {
                            // BREAK read back as 0x00 during init
                        } else {
                            sh.rx.push_back(b);
                        }
                    }
                }
            })
            .map_err(std::io::Error::other)?;
        Ok(Self {
            port,
            shared,
            stop,
            reader: Some(reader),
            hw_echo,
            ignore_baud_errors: false,
            baud: 10_400,
            clock: None,
        })
    }

    /// Wait until wall time catches up with emulated time since the
    /// previous line action, then restart the line clock at `now`.
    fn sync(&mut self, now: Cycles) {
        if let Some((t0, c0)) = self.clock
            && now >= c0
        {
            let emu = Duration::from_nanos((now - c0) * 1_000_000 / CYCLES_PER_MS);
            if emu <= MAX_SYNC {
                let target = t0 + emu;
                let wall = Instant::now();
                if target > wall {
                    precise_sleep(target - wall);
                }
                // keep the ideal timeline when we were early enough
                self.clock = Some((target.max(Instant::now() - Duration::from_millis(1)), now));
                return;
            }
        }
        self.clock = Some((Instant::now(), now));
    }

    fn with_shared(&self, f: impl FnOnce(&mut Shared)) {
        if let Ok(mut sh) = self.shared.lock() {
            f(&mut sh);
        }
    }
}

impl Drop for SerialKLine {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(h) = self.reader.take() {
            let _ = h.join();
        }
    }
}

/// Upstream's divisor mapping: 0x77/0x76 are 10,400 baud; others are
/// `clock / 2 / divisor / 8` with the 20 MHz clock.
pub fn divisor_to_baud(divisor: u32) -> u32 {
    match divisor {
        0x77 | 0x76 => 10_400,
        0 => 10_400,
        d => (1_250_000 + d / 2) / d,
    }
}

impl Interface for SerialKLine {
    fn set_baud_divisor(&mut self, _now: Cycles, divisor: u32, rx: &mut dyn RxBuf) {
        let baud = divisor_to_baud(divisor);
        if baud == self.baud {
            return;
        }
        self.baud = baud;
        if let Err(e) = self.port.set_baud_rate(baud)
            && !self.ignore_baud_errors
        {
            eprintln!("kline: cannot set {baud} baud: {e}");
        }
        // as upstream: drop everything in flight, including expected echoes
        let _ = self.port.clear(ClearBuffer::Input);
        rx.flush();
        self.with_shared(|sh| {
            sh.rx.clear();
            sh.echo.clear();
        });
    }

    fn send_byte(&mut self, now: Cycles, byte: u8, _rx: &mut dyn RxBuf) {
        self.sync(now);
        let hw_echo = self.hw_echo;
        self.with_shared(|sh| {
            sh.ignore_breaks = false;
            if hw_echo {
                sh.echo.push_back(byte);
            }
        });
        if let Err(e) = self.port.write_all(&[byte]) {
            eprintln!("kline: write failed: {e}");
        }
    }

    fn poll_input(&mut self, _now: Cycles, rx: &mut dyn RxBuf) {
        let mut got = Vec::new();
        self.with_shared(|sh| got.extend(sh.rx.drain(..)));
        for b in got {
            rx.push(b);
        }
    }

    fn slow_init_imminent(&mut self, _now: Cycles, _rx: &mut dyn RxBuf) {
        self.with_shared(|sh| sh.ignore_breaks = true);
    }

    fn send_slow_init_bitwise(&mut self, now: Cycles, bit: bool, _rx: &mut dyn RxBuf) -> bool {
        self.sync(now);
        self.with_shared(|sh| sh.ignore_breaks = true);
        let r = if bit {
            self.port.clear_break()
        } else {
            self.port.set_break()
        };
        if let Err(e) = r {
            eprintln!("kline: BREAK {}: {e}", if bit { "clear" } else { "set" });
        }
        true
    }

    fn set_l(&mut self, now: Cycles, level: bool) {
        self.sync(now);
        let _ = self.port.write_request_to_send(level);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn divisors() {
        assert_eq!(divisor_to_baud(0x77), 10_400);
        assert_eq!(divisor_to_baud(0x81), 9_690); // the firmware's "9600"
        assert_eq!(divisor_to_baud(0x102), 4_845); // its "4800"
    }

    /// Echo cancellation and BREAK handling against a pty whose far end
    /// echoes every byte and then answers.
    #[test]
    fn echo_is_cancelled_and_replies_pass() {
        let (a, mut b) = TTYPort::pair().expect("pty pair");
        b.set_timeout(Duration::from_millis(5)).unwrap();
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = stop.clone();
        let far = std::thread::spawn(move || {
            let mut buf = [0u8; 64];
            while !stop2.load(Ordering::Relaxed) {
                if let Ok(n) = b.read(&mut buf) {
                    // echo what the tester sent, then reply 0xC1 per byte
                    let _ = b.write_all(&buf[..n]);
                    for _ in 0..n {
                        let _ = b.write_all(&[0xc1]);
                    }
                }
            }
        });
        let mut k = SerialKLine::from_port(a, true).unwrap();
        k.ignore_baud_errors = true;
        let mut rx: Vec<u8> = Vec::new();
        k.send_byte(0, 0x81, &mut rx);
        k.send_byte(10_000, 0x11, &mut rx);
        let t = Instant::now();
        while rx.len() < 2 && t.elapsed() < Duration::from_secs(2) {
            k.poll_input(0, &mut rx);
            std::thread::sleep(Duration::from_millis(5));
        }
        stop.store(true, Ordering::Relaxed);
        far.join().unwrap();
        assert_eq!(rx, [0xc1, 0xc1], "echoes removed, replies kept");
    }

    /// The line clock reproduces emulated spacing on the wire.
    #[test]
    fn line_clock_spaces_actions() {
        let (a, mut b) = TTYPort::pair().expect("pty pair");
        b.set_timeout(Duration::from_millis(5)).unwrap();
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = stop.clone();
        let sink = std::thread::spawn(move || {
            let mut buf = [0u8; 64];
            while !stop2.load(Ordering::Relaxed) {
                let _ = b.read(&mut buf);
            }
        });
        let mut k = SerialKLine::from_port(a, false).unwrap();
        let mut rx: Vec<u8> = Vec::new();
        let t = Instant::now();
        k.send_slow_init_bitwise(1_000_000, false, &mut rx); // K low
        k.send_slow_init_bitwise(1_250_000, true, &mut rx); // 25 ms later: high
        let low = t.elapsed();
        k.send_byte(1_500_000, 0x81, &mut rx); // 25 ms after that
        let total = t.elapsed();
        stop.store(true, Ordering::Relaxed);
        sink.join().unwrap();
        // never early; the upper bounds only where the scheduler is quiet
        // (shared CI runners stall for tens of ms)
        let ci = std::env::var_os("CI").is_some();
        assert!(low >= Duration::from_millis(25), "low {low:?}");
        assert!(total >= Duration::from_millis(50), "total {total:?}");
        if !ci {
            assert!(low < Duration::from_millis(30), "low {low:?}");
            assert!(total < Duration::from_millis(57), "total {total:?}");
        }
    }
}
