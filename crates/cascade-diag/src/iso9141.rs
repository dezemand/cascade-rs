//! ISO 9141-2 client: 5-baud initialisation and `68 6A F1` framing, as the
//! Hi-Scan firmware talks to the Tiburon's traction control module (2003-,
//! address 0x32, key bytes 08 08) [emulator-derived].
//!
//! Frames carry no length: a request is `68 6A F1 data sum`, a response
//! `48 6B addr data sum`, and a frame ends when the line goes quiet. One
//! request can have several response frames (mode 03 sends at most three
//! codes per frame).

use std::time::Duration;

use crate::guard;
use crate::kwp::Error;
use crate::line::Line;

/// ISO 9141-2 timing, with some slack for USB adapters.
#[derive(Clone, Copy, Debug)]
pub struct Timing {
    /// Longest wait for `55` after the address (W1max 300 ms).
    pub w1_max: Duration,
    /// Longest wait for each key byte (W2max/W3max 20 ms, plus slack).
    pub w2_max: Duration,
    /// Pause before sending the inverted key byte (W4 25-50 ms).
    pub w4: Duration,
    /// Longest wait for the first response byte (P2max 50 ms, plus slack).
    pub p2_max: Duration,
    /// A gap this long ends a response frame (P1max is 20 ms, P2min 25 ms).
    pub frame_gap: Duration,
    /// Pause between the last response and the next request (P3min).
    pub p3_min: Duration,
}

impl Default for Timing {
    fn default() -> Self {
        Self {
            w1_max: Duration::from_millis(400),
            w2_max: Duration::from_millis(50),
            w4: Duration::from_millis(30),
            p2_max: Duration::from_millis(100),
            frame_gap: Duration::from_millis(22),
            p3_min: Duration::from_millis(55),
        }
    }
}

fn checksum(b: &[u8]) -> u8 {
    b.iter().fold(0u8, |a, &x| a.wrapping_add(x))
}

/// A request frame: `68 6A F1 data sum`.
pub fn request_frame(data: &[u8]) -> Vec<u8> {
    let mut f = vec![0x68, 0x6a, 0xf1];
    f.extend_from_slice(data);
    f.push(checksum(&f));
    f
}

/// The data of a response frame `48 6B addr data sum`.
pub fn parse_response(frame: &[u8]) -> Result<Vec<u8>, Error> {
    if frame.len() < 5 || frame[0] != 0x48 || frame[1] != 0x6b {
        return Err(Error::BadFrame(frame.to_vec()));
    }
    let (body, sum) = frame.split_at(frame.len() - 1);
    if checksum(body) != sum[0] {
        return Err(Error::BadFrame(frame.to_vec()));
    }
    Ok(body[3..].to_vec())
}

pub struct Client<L: Line> {
    pub line: L,
    pub address: u8,
    pub timing: Timing,
    /// Every request goes through this check before it is sent.
    pub guard: fn(&[u8]) -> Result<(), guard::Blocked>,
}

impl<L: Line> Client<L> {
    pub fn new(line: L, address: u8, guard: fn(&[u8]) -> Result<(), guard::Blocked>) -> Self {
        Self {
            line,
            address,
            timing: Timing::default(),
            guard,
        }
    }

    fn byte(&mut self, timeout: Duration, got: &[u8]) -> Result<u8, Error> {
        self.line
            .read_byte(timeout)?
            .ok_or_else(|| Error::Timeout(got.to_vec()))
    }

    /// 5-baud init; returns the key bytes.
    pub fn start(&mut self) -> Result<[u8; 2], Error> {
        let t = self.timing;
        self.line.set_baud(10_400)?;
        // W0/W5: the bus idles before the address
        self.line.idle(Duration::from_millis(300));
        self.line.flush_input()?;
        self.line.five_baud(self.address)?;
        let sync = self.byte(t.w1_max, &[])?;
        if sync != 0x55 {
            return Err(Error::Unexpected(vec![sync]));
        }
        let kb1 = self.byte(t.w2_max, &[sync])?;
        let kb2 = self.byte(t.w2_max, &[sync, kb1])?;
        self.line.idle(t.w4);
        self.line.write(&[!kb2])?;
        let inv = self.byte(t.w1_max, &[sync, kb1, kb2])?;
        if inv != !self.address {
            return Err(Error::Unexpected(vec![sync, kb1, kb2, inv]));
        }
        self.line.idle(t.p3_min);
        Ok([kb1, kb2])
    }

    /// Send a request and collect every response frame's data.
    pub fn request(&mut self, data: &[u8]) -> Result<Vec<Vec<u8>>, Error> {
        (self.guard)(data).map_err(Error::Blocked)?;
        let frame = request_frame(data);
        self.line.flush_input()?;
        self.line.write(&frame)?;
        let mut frames: Vec<Vec<u8>> = Vec::new();
        let mut cur = Vec::new();
        let mut timeout = self.timing.p2_max;
        loop {
            match self.line.read_byte(timeout)? {
                Some(b) => {
                    cur.push(b);
                    timeout = self.timing.frame_gap;
                }
                None if cur.is_empty() => break,
                None => {
                    frames.push(std::mem::take(&mut cur));
                    // the next frame may follow after P2
                    timeout = self.timing.p2_max;
                }
            }
        }
        if frames.is_empty() {
            return Err(Error::Timeout(Vec::new()));
        }
        let out = frames
            .iter()
            .map(|f| parse_response(f))
            .collect::<Result<Vec<_>, _>>()?;
        // positive responses echo the mode + 0x40, or the B1 request bytes as F1 ..
        for r in &out {
            let ok = match data[0] {
                0xb1 => r.first() == Some(&0xf1) && r.get(1) == data.get(1),
                m => r.first() == Some(&(m + 0x40)),
            };
            if !ok {
                return Err(Error::Unexpected(r.clone()));
            }
        }
        self.line.idle(self.timing.p3_min);
        Ok(out)
    }

    /// Stored fault codes (mode 03): two bytes per code, `00 00` padding.
    pub fn read_dtcs(&mut self) -> Result<Vec<u16>, Error> {
        let frames = self.request(&[0x03])?;
        Ok(frames
            .iter()
            .flat_map(|f| f[1..].as_chunks::<2>().0)
            .map(|&c| u16::from_be_bytes(c))
            .filter(|&c| c != 0)
            .collect())
    }

    /// Keep-alive (`B1 01` → `F1 01`).
    pub fn keep_alive(&mut self) -> Result<(), Error> {
        self.request(&[0xb1, 0x01]).map(|_| ())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::SimLine;
    use cascade_iface::{KwpConfig, KwpEcu};

    fn tcs() -> Client<SimLine<KwpEcu>> {
        let mut cfg = KwpConfig {
            address: 0x32,
            iso9141: true,
            ..Default::default()
        };
        cfg.responses.insert(vec![0xb1, 0x01], vec![0xf1, 0x01]);
        cfg.responses
            .insert(vec![0x03], vec![0x43, 0x52, 0x00, 0x52, 0x03, 0x00, 0x00]);
        cfg.responses
            .insert(vec![0xb1, 0x0e], vec![0xf1, 0x0e, 10, 20, 30, 40]);
        cfg.responses
            .insert(vec![0xb1, 0x2c], vec![0xf1, 0x2c, 0x05]);
        Client::new(SimLine::new(KwpEcu::new(cfg)), 0x32, guard::check_tcs)
    }

    #[test]
    fn session_codes_and_data_against_the_model() {
        let mut c = tcs();
        assert_eq!(c.start().unwrap(), [0x08, 0x08]);
        c.keep_alive().unwrap();
        assert_eq!(c.read_dtcs().unwrap(), [0x5200, 0x5203]);
        assert_eq!(
            c.request(&[0xb1, 0x0e]).unwrap(),
            [vec![0xf1, 0x0e, 10, 20, 30, 40]]
        );
        assert_eq!(c.request(&[0xb1, 0x2c]).unwrap(), [vec![0xf1, 0x2c, 0x05]]);
    }

    #[test]
    fn guard_stops_requests_before_the_line() {
        let mut c = tcs();
        c.start().unwrap();
        let before = c.line.elapsed();
        assert!(matches!(c.request(&[0xb1, 0x10]), Err(Error::Blocked(_))));
        assert!(matches!(c.request(&[0x04]), Err(Error::Blocked(_))));
        assert_eq!(c.line.elapsed(), before, "nothing was sent");
    }

    #[test]
    fn wrong_address_times_out() {
        let mut c = tcs();
        c.address = 0x33;
        assert!(matches!(c.start(), Err(Error::Timeout(_))));
    }

    #[test]
    fn frames() {
        assert_eq!(request_frame(&[0x03]), [0x68, 0x6a, 0xf1, 0x03, 0xc6]);
        let resp = [0x48, 0x6b, 0x32, 0x43, 0x52, 0x00, 0x00, 0x00, 0x00, 0x00];
        let mut f = resp.to_vec();
        f.push(checksum(&resp));
        assert_eq!(parse_response(&f).unwrap(), [0x43, 0x52, 0, 0, 0, 0, 0]);
        *f.last_mut().unwrap() ^= 1;
        assert!(parse_response(&f).is_err());
    }
}
