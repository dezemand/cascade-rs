//! ISO-TP (ISO 15765-2): diagnostic messages of up to 4095 bytes over
//! 8-byte CAN frames, with normal (11- or 29-bit, no address byte)
//! addressing, as OBD-II on CAN (ISO 15765-4) uses it.
//!
//! Frame types, by the high nibble of the first byte:
//! - `0` single frame: length (1..=7), data
//! - `1` first frame: 12-bit length, first 6 data bytes
//! - `2` consecutive frame: sequence number (1, 2, .. 15, 0, 1, ..), 7 bytes
//! - `3` flow control from the receiver: `30` continue / `31` wait /
//!   `32` overflow, block size, STmin
//!
//! Done in user space over any [`CanBus`] (not the kernel's `can-isotp`),
//! so the same code runs on Linux SocketCAN and on the in-memory bus.

use std::fmt;
use std::io;
use std::thread;
use std::time::{Duration, Instant};

use crate::{CanBus, Frame};

/// Longest ISO-TP message (12-bit length).
pub const MAX_LEN: usize = 4095;

/// How many "wait" flow controls the sender accepts in a row (N_WFTmax).
const MAX_WAITS: u32 = 10;

#[derive(Debug)]
pub enum Error {
    Io(io::Error),
    /// No (complete) message in time; says what was being waited for.
    Timeout(&'static str),
    /// The other side broke the protocol (wrong sequence number, ...).
    Protocol(String),
    /// The receiver answered flow control "overflow".
    Overflow,
    /// The message does not fit ISO-TP.
    TooLong(usize),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Io(e) => write!(f, "CAN I/O: {e}"),
            Error::Timeout(what) => write!(f, "timeout waiting for {what}"),
            Error::Protocol(m) => write!(f, "ISO-TP: {m}"),
            Error::Overflow => write!(f, "ISO-TP: receiver reported overflow"),
            Error::TooLong(n) => write!(f, "ISO-TP: {n} bytes is longer than {MAX_LEN}"),
        }
    }
}

impl std::error::Error for Error {}

impl From<io::Error> for Error {
    fn from(e: io::Error) -> Self {
        Error::Io(e)
    }
}

/// The separation time an STmin byte asks for between consecutive frames.
pub fn st_min(b: u8) -> Duration {
    match b {
        0x00..=0x7f => Duration::from_millis(b as u64),
        0xf1..=0xf9 => Duration::from_micros((b as u64 - 0xf0) * 100),
        // reserved values: use the longest valid time
        _ => Duration::from_millis(0x7f),
    }
}

/// The physical request id of the ECU answering on `rx` when `tx` is an
/// OBD-II functional id (ISO 15765-4): `7DF` → response id − 8, and for
/// 29-bit `18DB33F1` → `18DA<ecu>F1` from response `18DAF1<ecu>`. `None`
/// when `tx` is not functional.
pub fn physical_request_id(tx: u32, rx: u32) -> Option<u32> {
    use crate::EXTENDED;
    if tx == 0x7df && (0x7e8..=0x7ef).contains(&rx) {
        return Some(rx - 8);
    }
    if tx == 0x18db_33f1 | EXTENDED && rx & 0xffff_ff00 == (0x18da_f100 | EXTENDED) {
        return Some(0x18da_00f1 | EXTENDED | (rx & 0xff) << 8);
    }
    None
}

/// One ISO-TP channel: sends on `tx_id`, receives on `rx_id`.
pub struct IsoTp<B: CanBus> {
    pub bus: B,
    pub tx_id: u32,
    pub rx_id: u32,
    /// Where this side sends flow control when receiving, if not `tx_id`:
    /// after a functional request (`7DF`) the ECU expects it on its
    /// physical address (`7E0` for an answer on `7E8`).
    pub fc_id: Option<u32>,
    /// Also accept single frames on this id (an ECU's functional request
    /// address, `7DF` for OBD-II).
    pub functional_id: Option<u32>,
    /// Pad every frame to 8 bytes with this byte (`None`: send only the
    /// bytes used).
    pub pad: Option<u8>,
    /// Longest wait for a flow control or the next consecutive frame
    /// (N_Bs, N_Cr; 1000 ms in ISO 15765-2).
    pub n_timeout: Duration,
    /// Block size and STmin this side asks for when receiving.
    pub block_size: u8,
    pub st_min: u8,
}

impl<B: CanBus> IsoTp<B> {
    pub fn new(bus: B, tx_id: u32, rx_id: u32) -> Self {
        Self {
            bus,
            tx_id,
            rx_id,
            fc_id: None,
            functional_id: None,
            pad: Some(0xaa),
            n_timeout: Duration::from_millis(1000),
            block_size: 0,
            st_min: 0,
        }
    }

    fn put(&mut self, mut data: Vec<u8>) -> Result<(), Error> {
        if let Some(p) = self.pad {
            data.resize(8, p);
        }
        self.bus.send(&Frame {
            id: self.tx_id,
            data,
        })?;
        Ok(())
    }

    /// The next frame on `rx_id` (or a single frame on the functional id)
    /// before `deadline`; other traffic is skipped.
    fn next(
        &mut self,
        deadline: Instant,
        sf_only_functional: bool,
    ) -> Result<Option<Frame>, Error> {
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                return Ok(None);
            }
            let Some(f) = self.bus.recv(left)? else {
                return Ok(None);
            };
            if f.data.is_empty() {
                continue;
            }
            if f.id == self.rx_id
                || (sf_only_functional && Some(f.id) == self.functional_id && f.data[0] >> 4 == 0)
            {
                return Ok(Some(f));
            }
        }
    }

    /// Wait for the receiver's flow control: (block size, STmin).
    fn flow_control(&mut self) -> Result<(u8, Duration), Error> {
        let mut waits = 0;
        loop {
            let deadline = Instant::now() + self.n_timeout;
            let Some(f) = self.next(deadline, false)? else {
                return Err(Error::Timeout("flow control"));
            };
            match f.data[0] {
                0x30 => {
                    let bs = f.data.get(1).copied().unwrap_or(0);
                    let st = f.data.get(2).copied().unwrap_or(0);
                    return Ok((bs, st_min(st)));
                }
                0x31 => {
                    waits += 1;
                    if waits > MAX_WAITS {
                        return Err(Error::Protocol("too many flow control waits".into()));
                    }
                }
                0x32 => return Err(Error::Overflow),
                // anything else from the receiver now is not flow control
                b if b >> 4 == 3 => {
                    return Err(Error::Protocol(format!("bad flow control {b:02X}")));
                }
                _ => {}
            }
        }
    }

    /// Send one message.
    pub fn send(&mut self, msg: &[u8]) -> Result<(), Error> {
        if msg.is_empty() {
            return Err(Error::Protocol("empty message".into()));
        }
        if msg.len() > MAX_LEN {
            return Err(Error::TooLong(msg.len()));
        }
        if msg.len() <= 7 {
            let mut d = vec![msg.len() as u8];
            d.extend_from_slice(msg);
            return self.put(d);
        }
        let mut d = vec![0x10 | (msg.len() >> 8) as u8, msg.len() as u8];
        d.extend_from_slice(&msg[..6]);
        self.put(d)?;
        let mut rest = &msg[6..];
        let mut seq = 1u8;
        while !rest.is_empty() {
            let (bs, gap) = self.flow_control()?;
            let mut sent = 0u32;
            while !rest.is_empty() && (bs == 0 || sent < bs as u32) {
                if sent > 0 || seq != 1 {
                    thread::sleep(gap);
                }
                let n = rest.len().min(7);
                let mut d = vec![0x20 | (seq & 0x0f)];
                d.extend_from_slice(&rest[..n]);
                self.put(d)?;
                rest = &rest[n..];
                seq = seq.wrapping_add(1) & 0x0f;
                sent += 1;
            }
        }
        Ok(())
    }

    fn send_flow_control(&mut self) -> Result<(), Error> {
        let mut d = vec![0x30, self.block_size, self.st_min];
        if let Some(p) = self.pad {
            d.resize(8, p);
        }
        let id = self.fc_id.unwrap_or(self.tx_id);
        self.bus.send(&Frame { id, data: d })?;
        Ok(())
    }

    /// Receive one message; the first frame must come within `timeout`.
    /// `Ok(None)` when nothing came.
    pub fn recv(&mut self, timeout: Duration) -> Result<Option<Vec<u8>>, Error> {
        let deadline = Instant::now() + timeout;
        loop {
            let Some(f) = self.next(deadline, true)? else {
                return Ok(None);
            };
            let d = &f.data;
            match d[0] >> 4 {
                0 => {
                    let n = (d[0] & 0x0f) as usize;
                    if n == 0 || n > 7 || n >= d.len() {
                        continue; // malformed: ignore it
                    }
                    return Ok(Some(d[1..=n].to_vec()));
                }
                1 => {
                    if d.len() < 8 {
                        continue;
                    }
                    let n = ((d[0] as usize & 0x0f) << 8) | d[1] as usize;
                    if n < 8 {
                        continue;
                    }
                    return self.recv_rest(n, &d[2..8]).map(Some);
                }
                // consecutive frames without a first frame, stray flow
                // control: not ours
                _ => continue,
            }
        }
    }

    fn recv_rest(&mut self, len: usize, first: &[u8]) -> Result<Vec<u8>, Error> {
        let mut msg = first.to_vec();
        self.send_flow_control()?;
        let mut seq = 1u8;
        let mut in_block = 0u32;
        while msg.len() < len {
            let deadline = Instant::now() + self.n_timeout;
            let Some(f) = self.next(deadline, false)? else {
                return Err(Error::Timeout("consecutive frame"));
            };
            let d = &f.data;
            if d[0] >> 4 != 2 {
                return Err(Error::Protocol(format!(
                    "expected a consecutive frame, got {f:?}"
                )));
            }
            if d[0] & 0x0f != seq {
                return Err(Error::Protocol(format!(
                    "consecutive frame {} where {seq} was due",
                    d[0] & 0x0f
                )));
            }
            let take = (len - msg.len()).min(d.len() - 1);
            msg.extend_from_slice(&d[1..1 + take]);
            seq = (seq + 1) & 0x0f;
            in_block += 1;
            if self.block_size != 0 && in_block == self.block_size as u32 && msg.len() < len {
                in_block = 0;
                self.send_flow_control()?;
            }
        }
        Ok(msg)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vbus::Hub;

    fn pair() -> (IsoTp<crate::vbus::Endpoint>, IsoTp<crate::vbus::Endpoint>) {
        let hub = Hub::new();
        (
            IsoTp::new(hub.endpoint(), 0x7e0, 0x7e8),
            IsoTp::new(hub.endpoint(), 0x7e8, 0x7e0),
        )
    }

    #[test]
    fn single_frame_padded() {
        let hub = Hub::new();
        let mut tap = hub.endpoint();
        let mut a = IsoTp::new(hub.endpoint(), 0x7e0, 0x7e8);
        a.send(&[0x01, 0x0c]).unwrap();
        let f = tap.recv(Duration::from_millis(100)).unwrap().unwrap();
        assert_eq!(
            f,
            Frame::new(0x7e0, &[2, 1, 0x0c, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa])
        );
        a.pad = None;
        a.send(&[0x03]).unwrap();
        let f = tap.recv(Duration::from_millis(100)).unwrap().unwrap();
        assert_eq!(f, Frame::new(0x7e0, &[1, 3]));
    }

    #[test]
    fn long_message_both_ways_with_sequence_wrap() {
        let (mut a, mut b) = pair();
        // 200 bytes: 1 first frame + 28 consecutive frames, sequence wraps
        let msg: Vec<u8> = (0..200u32).map(|i| i as u8).collect();
        let m = msg.clone();
        let t = std::thread::spawn(move || {
            b.block_size = 5; // flow control every 5 frames
            b.st_min = 0xf1; // 100 µs
            let got = b.recv(Duration::from_secs(2)).unwrap().unwrap();
            assert_eq!(got, m);
            b.send(&got[..100]).unwrap();
        });
        a.send(&msg).unwrap();
        let back = a.recv(Duration::from_secs(2)).unwrap().unwrap();
        assert_eq!(back, msg[..100]);
        t.join().unwrap();
    }

    #[test]
    fn other_ids_are_ignored_and_functional_single_frames_accepted() {
        let hub = Hub::new();
        let mut other = hub.endpoint();
        let mut ecu = IsoTp::new(hub.endpoint(), 0x7e8, 0x7e0);
        ecu.functional_id = Some(0x7df);
        other.send(&Frame::new(0x7e9, &[2, 0x41, 0])).unwrap();
        other.send(&Frame::new(0x123, &[1, 2, 3])).unwrap();
        other.send(&Frame::new(0x7df, &[2, 0x01, 0x00])).unwrap();
        assert_eq!(
            ecu.recv(Duration::from_millis(200)).unwrap(),
            Some(vec![0x01, 0x00])
        );
        assert_eq!(ecu.recv(Duration::from_millis(50)).unwrap(), None);
    }

    #[test]
    fn missing_flow_control_times_out_and_overflow_is_reported() {
        let hub = Hub::new();
        let mut ecu = hub.endpoint();
        let mut a = IsoTp::new(hub.endpoint(), 0x7e0, 0x7e8);
        a.n_timeout = Duration::from_millis(50);
        assert!(matches!(a.send(&[0; 20]), Err(Error::Timeout(_))));
        let t = std::thread::spawn(move || {
            let _ff = ecu.recv(Duration::from_secs(1)).unwrap();
            ecu.send(&Frame::new(0x7e8, &[0x31, 0, 0])).unwrap(); // wait
            ecu.send(&Frame::new(0x7e8, &[0x32, 0, 0])).unwrap(); // overflow
        });
        a.n_timeout = Duration::from_secs(1);
        assert!(matches!(a.send(&[0; 20]), Err(Error::Overflow)));
        t.join().unwrap();
    }

    #[test]
    fn wrong_sequence_number_is_an_error() {
        let hub = Hub::new();
        let mut ecu = hub.endpoint();
        let mut a = IsoTp::new(hub.endpoint(), 0x7e0, 0x7e8);
        let t = std::thread::spawn(move || {
            ecu.send(&Frame::new(
                0x7e8,
                &[0x10, 20, 0x49, 2, 1, 0x4b, 0x4d, 0x48],
            ))
            .unwrap();
            let _fc = ecu.recv(Duration::from_secs(1)).unwrap();
            ecu.send(&Frame::new(0x7e8, &[0x22, 1, 2, 3, 4, 5, 6, 7]))
                .unwrap();
        });
        let r = a.recv(Duration::from_secs(1));
        assert!(matches!(r, Err(Error::Protocol(_))), "{r:?}");
        t.join().unwrap();
    }

    #[test]
    fn functional_request_long_answer() {
        let hub = Hub::new();
        let mut ecu = IsoTp::new(hub.endpoint(), 0x7e8, 0x7e0);
        ecu.functional_id = Some(0x7df);
        let t = std::thread::spawn(move || {
            let req = ecu.recv(Duration::from_secs(1)).unwrap().unwrap();
            assert_eq!(req, vec![0x09, 0x02]);
            ecu.send(&[0x49; 20]).unwrap();
        });
        let mut a = IsoTp::new(hub.endpoint(), 0x7df, 0x7e8);
        a.fc_id = physical_request_id(0x7df, 0x7e8);
        assert_eq!(a.fc_id, Some(0x7e0));
        a.send(&[0x09, 0x02]).unwrap();
        assert_eq!(
            a.recv(Duration::from_secs(1)).unwrap(),
            Some(vec![0x49; 20])
        );
        t.join().unwrap();
    }

    #[test]
    fn physical_ids() {
        use crate::EXTENDED;
        assert_eq!(physical_request_id(0x7e0, 0x7e8), None);
        assert_eq!(physical_request_id(0x7df, 0x7e9), Some(0x7e1));
        assert_eq!(
            physical_request_id(0x18db_33f1 | EXTENDED, 0x18da_f110 | EXTENDED),
            Some(0x18da_10f1 | EXTENDED)
        );
    }

    #[test]
    fn st_min_values() {
        assert_eq!(st_min(0x14), Duration::from_millis(20));
        assert_eq!(st_min(0xf3), Duration::from_micros(300));
        assert_eq!(st_min(0x80), Duration::from_millis(127));
    }
}
