//! KWP2000 (ISO 14230) client over a [`Line`].

use std::fmt;
use std::io;
use std::time::Duration;

use cascade_iface::iso14230;

use crate::line::Line;

/// ISO 14230 default timing.
#[derive(Clone, Copy, Debug)]
pub struct Timing {
    /// Longest wait for the first response byte (P2max).
    pub p2_max: Duration,
    /// Longest wait after a "response pending" (0x78) negative response.
    pub p2_ext: Duration,
    /// Longest gap between bytes of one ECU frame (P1max).
    pub p1_max: Duration,
    /// Shortest pause between a response and the next request (P3min).
    pub p3_min: Duration,
    /// Fast-init low and high times (TiniL, TWuP - TiniL).
    pub wake_low: Duration,
    pub wake_high: Duration,
}

impl Default for Timing {
    fn default() -> Self {
        Self {
            p2_max: Duration::from_millis(50),
            p2_ext: Duration::from_millis(5000),
            p1_max: Duration::from_millis(20),
            p3_min: Duration::from_millis(55),
            wake_low: Duration::from_millis(25),
            wake_high: Duration::from_millis(25),
        }
    }
}

#[derive(Debug)]
pub enum Error {
    Io(io::Error),
    /// No (complete) response in time. Carries what was received.
    Timeout(Vec<u8>),
    /// Bad checksum or framing. Carries the raw bytes.
    BadFrame(Vec<u8>),
    /// ECU answered with 7F sid code.
    Negative {
        sid: u8,
        code: u8,
    },
    /// Response does not belong to the request.
    Unexpected(Vec<u8>),
    /// Refused by the read-only guard before anything was sent.
    Blocked(crate::guard::Blocked),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Io(e) => write!(f, "I/O: {e}"),
            Error::Blocked(b) => write!(f, "{b}"),
            Error::Timeout(b) => write!(f, "timeout (received {})", hex(b)),
            Error::BadFrame(b) => write!(f, "bad frame {}", hex(b)),
            Error::Negative { sid, code } => {
                write!(f, "negative response to {sid:02X}: {code:02X}")
            }
            Error::Unexpected(b) => write!(f, "unexpected response {}", hex(b)),
        }
    }
}

impl std::error::Error for Error {}

impl From<io::Error> for Error {
    fn from(e: io::Error) -> Self {
        Error::Io(e)
    }
}

pub fn hex(b: &[u8]) -> String {
    b.iter()
        .map(|x| format!("{x:02X}"))
        .collect::<Vec<_>>()
        .join(" ")
}

pub struct Client<L: Line> {
    pub line: L,
    pub target: u8,
    pub source: u8,
    pub timing: Timing,
    /// Always send the separate length byte (`80 tgt src len ...`), as
    /// the Hi-Scan does for ABS and SRS.
    pub length_byte: bool,
    /// Let ClearDiagnosticInformation through the read-only guard.
    pub allow_clear: bool,
    last_rx_end: Duration,
}

impl<L: Line> Client<L> {
    /// Client for the engine ECU (0x11) as tester F1, the addresses the
    /// Hi-Scan uses for the Tiburon engine.
    pub fn new(line: L) -> Self {
        Self::with_addresses(line, 0x11, 0xf1)
    }

    pub fn with_addresses(line: L, target: u8, source: u8) -> Self {
        Self {
            line,
            target,
            source,
            timing: Timing::default(),
            length_byte: false,
            allow_clear: false,
            last_rx_end: Duration::ZERO,
        }
    }

    /// Fast init at 10,400 baud plus StartCommunication. Returns key bytes.
    pub fn start(&mut self) -> Result<[u8; 2], Error> {
        self.line.set_baud(10_400)?;
        // the bus must be idle (high) before the wake-up pattern
        self.line.idle(Duration::from_millis(300));
        self.line.flush_input()?;
        self.line
            .wake_up(self.timing.wake_low, self.timing.wake_high)?;
        let r = self.exchange(&[0x81])?;
        match r.as_slice() {
            [0xc1, kb1, kb2] => Ok([*kb1, *kb2]),
            _ => Err(Error::Unexpected(r)),
        }
    }

    /// Send a request and return the positive response data (with SID).
    pub fn request(&mut self, data: &[u8]) -> Result<Vec<u8>, Error> {
        self.exchange(data)
    }

    fn exchange(&mut self, data: &[u8]) -> Result<Vec<u8>, Error> {
        // nothing reaches the line without passing the read-only guard
        crate::guard::check(data, self.allow_clear).map_err(Error::Blocked)?;
        let since = self.line.elapsed().saturating_sub(self.last_rx_end);
        if since < self.timing.p3_min {
            self.line.idle(self.timing.p3_min - since);
        }
        let sid = data[0];
        self.line.flush_input()?;
        let frame = if self.length_byte {
            let mut f = vec![0x80, self.target, self.source, data.len() as u8];
            f.extend_from_slice(data);
            f.push(iso14230::checksum(&f));
            f
        } else {
            iso14230::encode(self.target, self.source, data)
        };
        self.line.write(&frame)?;
        let mut wait = self.timing.p2_max;
        loop {
            let frame = self.read_frame(wait)?;
            self.last_rx_end = self.line.elapsed();
            let f = iso14230::parse(&frame).ok_or_else(|| Error::BadFrame(frame.clone()))?;
            if f.target != self.source || f.source != self.target {
                continue; // someone else's traffic
            }
            match f.data.as_slice() {
                [0x7f, s, 0x78] if *s == sid => wait = self.timing.p2_ext,
                [0x7f, s, code] if *s == sid => return Err(Error::Negative { sid, code: *code }),
                [r, ..] if *r == sid.wrapping_add(0x40) => return Ok(f.data),
                _ => return Err(Error::Unexpected(f.data)),
            }
        }
    }

    fn read_frame(&mut self, first_byte_timeout: Duration) -> Result<Vec<u8>, Error> {
        let mut frame = Vec::new();
        let mut timeout = first_byte_timeout;
        loop {
            match self.line.read_byte(timeout)? {
                Some(b) => frame.push(b),
                None => return Err(Error::Timeout(frame)),
            }
            if iso14230::expected_len(&frame) == Some(frame.len()) {
                return Ok(frame);
            }
            timeout = self.timing.p1_max;
        }
    }

    pub fn tester_present(&mut self) -> Result<(), Error> {
        self.exchange(&[0x3e, 0x01]).map(|_| ())
    }

    /// Stored DTCs as the Hi-Scan asks for them (`18 00 00 00`):
    /// (code, status) pairs.
    pub fn read_dtcs(&mut self) -> Result<Vec<(u16, u8)>, Error> {
        let r = self.exchange(&[0x18, 0x00, 0x00, 0x00])?;
        let n = *r.get(1).ok_or_else(|| Error::Unexpected(r.clone()))? as usize;
        let body = &r[2..];
        if body.len() < n * 3 {
            return Err(Error::Unexpected(r));
        }
        Ok(body
            .chunks(3)
            .take(n)
            .map(|c| (((c[0] as u16) << 8) | c[1] as u16, c[2]))
            .collect())
    }

    /// DTCs as the Hi-Scan asks the Tiburon ABS for them
    /// (`13 40 00`): two-byte codes, no status.
    pub fn read_dtcs_13(&mut self, group: u16) -> Result<Vec<u16>, Error> {
        let r = self.exchange(&[0x13, (group >> 8) as u8, group as u8])?;
        let n = *r.get(1).ok_or_else(|| Error::Unexpected(r.clone()))? as usize;
        let body = &r[2..];
        if body.len() < n * 2 {
            return Err(Error::Unexpected(r));
        }
        Ok(body
            .chunks(2)
            .take(n)
            .map(|c| ((c[0] as u16) << 8) | c[1] as u16)
            .collect())
    }

    /// ReadDTCsByStatus with explicit status and group (SRS: `18 00 FF 00`
    /// and `18 01 FF 00`). Assumes (code, status) records as for the engine;
    /// unverified for SRS.
    pub fn read_dtcs_by_status(&mut self, status: u8, group: u16) -> Result<Vec<(u16, u8)>, Error> {
        let r = self.exchange(&[0x18, status, (group >> 8) as u8, group as u8])?;
        let n = *r.get(1).ok_or_else(|| Error::Unexpected(r.clone()))? as usize;
        let body = &r[2..];
        if body.len() < n * 3 {
            return Err(Error::Unexpected(r));
        }
        Ok(body
            .chunks(3)
            .take(n)
            .map(|c| (((c[0] as u16) << 8) | c[1] as u16, c[2]))
            .collect())
    }

    /// ReadDataByLocalIdentifier: the block after `61 id`.
    pub fn read_local(&mut self, id: u8) -> Result<Vec<u8>, Error> {
        let r = self.exchange(&[0x21, id])?;
        match r.as_slice() {
            [0x61, rid, rest @ ..] if *rid == id => Ok(rest.to_vec()),
            _ => Err(Error::Unexpected(r)),
        }
    }

    /// ClearDiagnosticInformation for a group, as the Hi-Scan sends it
    /// (engine `14 FF 00`, ABS `14 40 00`, both observed in the emulator).
    /// Refused unless `allow_clear` is set.
    pub fn clear_dtcs(&mut self, group: u16) -> Result<(), Error> {
        let r = self.exchange(&[0x14, (group >> 8) as u8, group as u8])?;
        match r.as_slice() {
            [0x54, ..] => Ok(()),
            _ => Err(Error::Unexpected(r)),
        }
    }

    pub fn stop(&mut self) -> Result<(), Error> {
        self.exchange(&[0x82]).map(|_| ())
    }
}

/// Format a DTC code the way the Hi-Scan shows it (P0301 and so on).
pub fn dtc_name(code: u16) -> String {
    let letter = ['P', 'C', 'B', 'U'][(code >> 14) as usize];
    format!("{letter}{:04X}", code & 0x3fff)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::SimLine;
    use cascade_iface::{KwpConfig, KwpEcu};

    fn client(cfg: KwpConfig) -> Client<SimLine<KwpEcu>> {
        Client::new(SimLine::new(KwpEcu::new(cfg)))
    }

    #[test]
    fn session_dtcs_and_block() {
        let mut cfg = KwpConfig {
            dtcs: vec![(0x0301, 0xe0), (0x0130, 0x20)],
            ..Default::default()
        };
        cfg.local_data.insert(0x01, vec![7; 80]);
        let mut c = client(cfg);
        assert_eq!(c.start().unwrap(), [0xef, 0x8f]);
        c.tester_present().unwrap();
        let d = c.read_dtcs().unwrap();
        assert_eq!(d, [(0x0301, 0xe0), (0x0130, 0x20)]);
        assert_eq!(dtc_name(d[0].0), "P0301");
        assert_eq!(c.read_local(0x01).unwrap(), vec![7; 80]); // long frame
        c.stop().unwrap();
    }

    #[test]
    fn negative_and_silence() {
        let mut c = client(KwpConfig::default());
        // before StartCommunication the ECU stays silent
        assert!(matches!(c.request(&[0x3e, 0x01]), Err(Error::Timeout(_))));
        c.start().unwrap();
        assert!(matches!(
            c.read_local(0x05),
            Err(Error::Negative {
                sid: 0x21,
                code: 0x31
            })
        ));
        // P3max: after 6 s of silence the session is gone
        c.line.idle(Duration::from_secs(6));
        assert!(matches!(c.tester_present(), Err(Error::Timeout(_))));
    }

    #[test]
    fn abs_session_with_length_byte_headers() {
        let mut cfg = KwpConfig {
            address: 0x28,
            key_bytes: [0xea, 0x8f],
            length_byte: true,
            ..Default::default()
        };
        cfg.responses.insert(
            vec![0x13, 0x40, 0x00],
            vec![0x53, 0x02, 0x50, 0x1f, 0x40, 0x15],
        );
        let mut c = Client::with_addresses(SimLine::new(KwpEcu::new(cfg)), 0x28, 0xf1);
        c.length_byte = true;
        assert_eq!(c.start().unwrap(), [0xea, 0x8f]);
        let codes = c.read_dtcs_13(0x4000).unwrap();
        assert_eq!(
            codes.iter().map(|&c| dtc_name(c)).collect::<Vec<_>>(),
            ["C101F", "C0015"]
        );
    }

    #[test]
    fn guard_blocks_before_sending() {
        let mut c = client(KwpConfig {
            dtcs: vec![(0x0301, 0xe0)],
            ..Default::default()
        });
        c.start().unwrap();
        let t = c.line.elapsed();
        assert!(matches!(c.request(&[0x11, 0x01]), Err(Error::Blocked(_))));
        assert!(matches!(c.clear_dtcs(0xff00), Err(Error::Blocked(_))));
        assert_eq!(c.line.elapsed(), t, "nothing was sent");
        assert_eq!(c.read_dtcs().unwrap().len(), 1, "codes still stored");
        c.allow_clear = true;
        c.clear_dtcs(0xff00).unwrap();
        assert!(c.read_dtcs().unwrap().is_empty());
    }

    #[test]
    fn requests_respect_p3_min() {
        let mut c = client(KwpConfig::default());
        c.start().unwrap();
        let t0 = c.line.elapsed();
        c.tester_present().unwrap();
        c.tester_present().unwrap();
        // two exchanges: each at least P3min + frame times + P2 (25 ms in the model)
        assert!(c.line.elapsed() - t0 >= Duration::from_millis(2 * (55 + 25)));
    }
}
