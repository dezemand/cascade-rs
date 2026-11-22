//! Diagnostics over CAN: raw frames on a [`CanBus`], ISO-TP (ISO 15765-2)
//! messages on top in [`isotp`], and the buses to run them on:
//!
//! - [`socketcan::SocketCan`] (Linux only): a raw SocketCAN socket on an
//!   interface such as `can0` or `vcan0`. The interface (bitrate, up) is set
//!   up outside, with `ip link`.
//! - [`vbus::Hub`]: an in-memory bus for tests and simulation, any OS.
//!
//! [`ecu`] answers requests as an ECU would (a simulated OBD-II engine ECU,
//! or anything that implements [`ecu::Responder`]).

use std::fmt;
use std::io;
use std::time::Duration;

pub mod ecu;
pub mod isotp;
#[cfg(target_os = "linux")]
pub mod socketcan;
pub mod vbus;

/// Set in [`Frame::id`] for a 29-bit identifier (as in SocketCAN).
pub const EXTENDED: u32 = 0x8000_0000;

/// One classic CAN data frame (up to 8 bytes).
#[derive(Clone, PartialEq, Eq)]
pub struct Frame {
    /// 11-bit identifier, or 29-bit with [`EXTENDED`] set.
    pub id: u32,
    pub data: Vec<u8>,
}

impl Frame {
    pub fn new(id: u32, data: &[u8]) -> Self {
        assert!(data.len() <= 8, "classic CAN carries at most 8 bytes");
        Self {
            id,
            data: data.to_vec(),
        }
    }
}

impl fmt::Debug for Frame {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.id & EXTENDED != 0 {
            write!(f, "{:08X}", self.id & !EXTENDED)?;
        } else {
            write!(f, "{:03X}", self.id)?;
        }
        write!(f, " [{}]", self.data.len())?;
        for b in &self.data {
            write!(f, " {b:02X}")?;
        }
        Ok(())
    }
}

/// A CAN bus as one node sees it: it sends frames and receives the frames
/// of the other nodes (not its own).
pub trait CanBus {
    fn send(&mut self, frame: &Frame) -> io::Result<()>;

    /// The next frame from another node, or `None` after `timeout`.
    fn recv(&mut self, timeout: Duration) -> io::Result<Option<Frame>>;
}

impl<B: CanBus + ?Sized> CanBus for Box<B> {
    fn send(&mut self, frame: &Frame) -> io::Result<()> {
        (**self).send(frame)
    }

    fn recv(&mut self, timeout: Duration) -> io::Result<Option<Frame>> {
        (**self).recv(timeout)
    }
}

/// Parse a CAN identifier as written by people: `7E0`, `0x7e0`, or 8 hex
/// digits (`18DA10F1`) for a 29-bit one.
pub fn parse_id(s: &str) -> Result<u32, String> {
    let h = s.trim_start_matches("0x").trim_start_matches("0X");
    let v = u32::from_str_radix(h, 16).map_err(|e| format!("CAN id {s:?}: {e}"))?;
    if h.len() > 3 || v > 0x7ff {
        if v > 0x1fff_ffff {
            return Err(format!("CAN id {s:?} is longer than 29 bits"));
        }
        Ok(v | EXTENDED)
    } else {
        Ok(v)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ids() {
        assert_eq!(parse_id("7E0"), Ok(0x7e0));
        assert_eq!(parse_id("0x7df"), Ok(0x7df));
        assert_eq!(parse_id("18DA10F1"), Ok(0x18da_10f1 | EXTENDED));
        assert!(parse_id("3FFFFFFF").is_err());
        assert_eq!(
            format!("{:?}", Frame::new(0x7e8, &[2, 0x41, 0])),
            "7E8 [3] 02 41 00"
        );
    }
}
