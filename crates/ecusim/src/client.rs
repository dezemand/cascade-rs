//! Clients of an `ecusim` server.

use std::io::{self, BufRead, BufReader, Write};
use std::os::unix::net::UnixStream;
use std::path::Path;
use std::path::PathBuf;
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, TryRecvError};
use std::time::{Duration, Instant};

use cascade_iface::{Cycles, Interface, RxBuf};

use crate::proto::{ToClient, ToServer};

/// A connection: messages out, a reader thread for messages in.
pub struct Conn {
    w: UnixStream,
    rx: Receiver<ToClient>,
}

impl Conn {
    pub fn connect(path: &Path) -> io::Result<Self> {
        let s = UnixStream::connect(path)?;
        let r = s.try_clone()?;
        let (tx, rx) = mpsc::channel();
        std::thread::spawn(move || {
            for line in BufReader::new(r).lines() {
                let Ok(line) = line else {
                    break;
                };
                if let Ok(m) = ToClient::parse(&line)
                    && tx.send(m).is_err()
                {
                    break;
                }
            }
        });
        Ok(Self { w: s, rx })
    }

    pub fn send(&mut self, m: &ToServer) -> io::Result<()> {
        writeln!(self.w, "{}", m.line())
    }

    pub fn try_recv(&self) -> Option<ToClient> {
        self.rx.try_recv().ok()
    }

    /// Next message if one is waiting; an error once the server is gone.
    pub fn poll(&self) -> io::Result<Option<ToClient>> {
        match self.rx.try_recv() {
            Ok(m) => Ok(Some(m)),
            Err(TryRecvError::Empty) => Ok(None),
            Err(TryRecvError::Disconnected) => Err(io::Error::new(
                io::ErrorKind::BrokenPipe,
                "ecusim server closed the connection",
            )),
        }
    }

    /// Next message, `None` after `timeout`. Errors when the server is gone.
    pub fn recv_timeout(&self, timeout: Duration) -> io::Result<Option<ToClient>> {
        match self.rx.recv_timeout(timeout) {
            Ok(m) => Ok(Some(m)),
            Err(RecvTimeoutError::Timeout) => Ok(None),
            Err(RecvTimeoutError::Disconnected) => Err(io::Error::new(
                io::ErrorKind::BrokenPipe,
                "ecusim server closed the connection",
            )),
        }
    }
}

/// The Hi-Scan emulator's diagnostic line, connected to an `ecusim` server.
/// Timestamps are the emulator's cycles (10,000 per ms) in microseconds.
///
/// If the server goes away (stopped, restarted) the line is dead, as with
/// an unplugged cable, and it reconnects by itself, at most once a second,
/// when a server answers on the socket again.
pub struct SocketKLine {
    path: PathBuf,
    conn: Option<Conn>,
    baud: Option<u32>,
    pending: Vec<u8>,
    retry_at: Instant,
}

fn open_kline(path: &Path) -> io::Result<Conn> {
    let mut conn = Conn::connect(path)?;
    conn.send(&ToServer::KLine)?;
    Ok(conn)
}

impl SocketKLine {
    pub fn connect(path: &Path) -> io::Result<Self> {
        Ok(Self {
            path: path.to_path_buf(),
            conn: Some(open_kline(path)?),
            baud: None,
            pending: Vec::new(),
            retry_at: Instant::now(),
        })
    }

    /// A line that connects when a server answers on `path` (and again
    /// after it restarts); dead until then.
    pub fn waiting(path: &Path) -> Self {
        let mut l = Self {
            path: path.to_path_buf(),
            conn: None,
            baud: None,
            pending: Vec::new(),
            retry_at: Instant::now(),
        };
        l.reconnect();
        l
    }

    /// Whether a server is connected now.
    pub fn is_connected(&self) -> bool {
        self.conn.is_some()
    }

    fn reconnect(&mut self) {
        if self.conn.is_some() || Instant::now() < self.retry_at {
            return;
        }
        self.retry_at = Instant::now() + Duration::from_secs(1);
        if let Ok(mut c) = open_kline(&self.path) {
            if let Some(b) = self.baud {
                let _ = c.send(&ToServer::Baud(b));
            }
            self.conn = Some(c);
        }
    }

    fn send(&mut self, m: ToServer) {
        self.reconnect();
        if let Some(c) = &mut self.conn
            && c.send(&m).is_err()
        {
            self.conn = None;
        }
    }
}

fn us(now: Cycles) -> u64 {
    now / 10
}

/// The Hi-Scan's UART divisor as a baud rate (as in `kline_iface`).
fn divisor_to_baud(d: u32) -> u32 {
    match d {
        0x77 | 0x76 | 0 => 10_400,
        d => (1_250_000 + d / 2) / d,
    }
}

impl Interface for SocketKLine {
    fn set_baud_divisor(&mut self, _now: Cycles, divisor: u32, _rx: &mut dyn RxBuf) {
        let b = divisor_to_baud(divisor);
        self.baud = Some(b);
        self.send(ToServer::Baud(b));
    }

    fn send_byte(&mut self, now: Cycles, byte: u8, _rx: &mut dyn RxBuf) {
        self.send(ToServer::Tx {
            us: us(now),
            bytes: vec![byte],
        });
    }

    fn poll_input(&mut self, _now: Cycles, rx: &mut dyn RxBuf) {
        self.reconnect();
        while let Some(c) = &self.conn {
            match c.poll() {
                Ok(Some(ToClient::Rx(b))) => self.pending.extend(b),
                Ok(Some(_)) => {}
                Ok(None) => break,
                Err(_) => self.conn = None,
            }
        }
        for b in self.pending.drain(..) {
            rx.push(b);
        }
    }

    fn send_slow_init_bitwise(&mut self, now: Cycles, bit: bool, _rx: &mut dyn RxBuf) -> bool {
        self.send(if bit {
            ToServer::High { us: us(now) }
        } else {
            ToServer::Low { us: us(now) }
        });
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::os::unix::net::UnixListener;

    struct Sink;
    impl RxBuf for Sink {
        fn push(&mut self, _: u8) {}
        fn flush(&mut self) {}
    }

    #[test]
    fn line_reconnects_after_the_server_restarts() {
        let dir = std::env::temp_dir().join(format!("ecusim-t{}", std::process::id()));
        let _ = std::fs::create_dir(&dir);
        let sock = dir.join("s");
        let l1 = UnixListener::bind(&sock).unwrap();
        let mut line = SocketKLine::connect(&sock).unwrap();
        // server goes away
        let (s1, _) = l1.accept().unwrap();
        drop(s1);
        drop(l1);
        std::fs::remove_file(&sock).unwrap();
        let mut rx = Sink;
        for _ in 0..50 {
            line.poll_input(0, &mut rx);
            if !line.is_connected() {
                break;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        assert!(!line.is_connected(), "a closed server is noticed");
        // a new server on the same socket
        let l2 = UnixListener::bind(&sock).unwrap();
        line.retry_at = Instant::now();
        line.poll_input(0, &mut rx);
        assert!(line.is_connected(), "reconnects");
        let (s2, _) = l2.accept().unwrap();
        let mut first = String::new();
        std::io::BufRead::read_line(&mut std::io::BufReader::new(s2), &mut first).unwrap();
        assert_eq!(first.trim(), "KLINE");
        let _ = std::fs::remove_dir_all(&dir);
    }
}
