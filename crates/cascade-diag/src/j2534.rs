//! KWP2000 over a J2534 PassThru device: the adapter does the fast init,
//! framing, checksums and inter-byte timing, so this [`Link`] moves whole
//! messages. On Windows the device is a vendor DLL
//! (`cascade_j2534::dll::DllPassThru`); [`MockPassThru`] stands in for it on
//! any platform.
//!
//! J2534 messages carry the ISO 14230 header and data without the checksum;
//! the client's frames include it, so it is removed on the way out and
//! added on the way in. That the device strips the checksum from received
//! messages is J2534-1 behaviour **[assumed for the Chipsoft]**.

use std::time::Duration;

use cascade_iface::iso14230;
pub use cascade_j2534::{PassThru, config};

use crate::kwp::{Error, Link, Timing};
use crate::line::Line;

/// ISO 14230 timing for the adapter, from [`Timing`] (J2534 units).
pub fn iso14230_config(t: &Timing) -> Vec<(u32, u32)> {
    let half_ms = |d: Duration| (d.as_micros() / 500) as u32;
    let ms = |d: Duration| d.as_millis() as u32;
    vec![
        (config::LOOPBACK, 0),
        (config::P1_MAX, half_ms(t.p1_max)),
        (config::P3_MIN, half_ms(t.p3_min)),
        (config::P4_MIN, 0),
        (config::TIDLE, 300),
        (config::TINIL, ms(t.wake_low)),
        (config::TWUP, ms(t.wake_low + t.wake_high)),
    ]
}

fn strip(frame: &[u8]) -> &[u8] {
    &frame[..frame.len().saturating_sub(1)]
}

fn with_checksum(mut msg: Vec<u8>) -> Vec<u8> {
    msg.push(iso14230::checksum(&msg));
    msg
}

/// A J2534 failure as a client error; "no answer" stays a timeout, which the
/// client and kdiag treat as such (retry, re-init).
fn io(e: cascade_j2534::Error) -> Error {
    if e.code == cascade_j2534::status::ERR_TIMEOUT {
        Error::Timeout(Vec::new())
    } else {
        Error::Io(std::io::Error::other(e.to_string()))
    }
}

pub struct J2534Link<P: PassThru> {
    pub pt: P,
    connected: bool,
}

impl<P: PassThru> J2534Link<P> {
    pub fn new(pt: P) -> Self {
        Self {
            pt,
            connected: false,
        }
    }
}

impl<P: PassThru> Link for J2534Link<P> {
    fn elapsed(&self) -> Duration {
        self.pt.elapsed()
    }

    fn idle(&mut self, d: Duration) {
        self.pt.idle(d)
    }

    fn fast_init(&mut self, start_frame: &[u8], timing: &Timing) -> Result<Vec<u8>, Error> {
        if !self.connected {
            self.pt
                .connect_iso14230(10_400, &iso14230_config(timing))
                .map_err(io)?;
            self.connected = true;
        }
        self.pt.clear_rx().map_err(io)?;
        let r = self.pt.fast_init(strip(start_frame)).map_err(io)?;
        if r.is_empty() {
            return Err(Error::Timeout(r));
        }
        Ok(with_checksum(r))
    }

    fn send(&mut self, frame: &[u8]) -> Result<(), Error> {
        self.pt
            .write(strip(frame), Duration::from_millis(1000))
            .map_err(io)
    }

    fn recv(&mut self, first: Duration, p1: Duration) -> Result<Vec<u8>, Error> {
        // the adapter assembles the frame; allow it the whole frame time
        match self.pt.read(first + p1 * 64).map_err(io)? {
            Some(m) => Ok(with_checksum(m)),
            None => Err(Error::Timeout(Vec::new())),
        }
    }

    fn flush_input(&mut self) -> Result<(), Error> {
        if self.connected {
            self.pt.clear_rx().map_err(io)?;
        }
        Ok(())
    }
}

/// A J2534 device in front of a simulated byte-level K-line (an ECU model
/// or the SIMK41 code behind `SimLine`): behaves like an adapter that does
/// the init, framing and checksums. For tests and `kdiag --j2534-mock`.
pub struct MockPassThru<L: Line> {
    pub line: L,
    pub timing: Timing,
    pub connected: Option<(u32, Vec<(u32, u32)>)>,
}

impl<L: Line> MockPassThru<L> {
    pub fn new(line: L) -> Self {
        Self {
            line,
            timing: Timing::default(),
            connected: None,
        }
    }
}

fn mock_err(e: Error) -> cascade_j2534::Error {
    match e {
        Error::Timeout(_) => {
            cascade_j2534::Error::new(cascade_j2534::status::ERR_TIMEOUT, "timeout")
        }
        e => cascade_j2534::Error::new(0x07, e.to_string()),
    }
}

impl<L: Line> PassThru for MockPassThru<L> {
    fn connect_iso14230(
        &mut self,
        baud: u32,
        config: &[(u32, u32)],
    ) -> Result<(), cascade_j2534::Error> {
        self.connected = Some((baud, config.to_vec()));
        Ok(())
    }

    fn fast_init(&mut self, request: &[u8]) -> Result<Vec<u8>, cascade_j2534::Error> {
        let frame = with_checksum(request.to_vec());
        let r = Link::fast_init(&mut self.line, &frame, &self.timing).map_err(mock_err)?;
        Ok(strip(&r).to_vec())
    }

    fn write(&mut self, msg: &[u8], _timeout: Duration) -> Result<(), cascade_j2534::Error> {
        Link::send(&mut self.line, &with_checksum(msg.to_vec())).map_err(mock_err)
    }

    fn read(&mut self, timeout: Duration) -> Result<Option<Vec<u8>>, cascade_j2534::Error> {
        match Link::recv(&mut self.line, timeout, self.timing.p1_max) {
            Ok(f) if iso14230::parse(&f).is_some() => Ok(Some(strip(&f).to_vec())),
            Ok(_) => Ok(None), // a real adapter drops frames with a bad checksum
            Err(Error::Timeout(_)) => Ok(None),
            Err(e) => Err(mock_err(e)),
        }
    }

    fn clear_rx(&mut self) -> Result<(), cascade_j2534::Error> {
        Link::flush_input(&mut self.line).map_err(mock_err)
    }

    fn battery_mv(&mut self) -> Result<u32, cascade_j2534::Error> {
        Ok(13_800)
    }

    fn elapsed(&self) -> Duration {
        Line::elapsed(&self.line)
    }

    fn idle(&mut self, d: Duration) {
        Line::idle(&mut self.line, d)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kwp::Client;
    use crate::sim::SimLine;
    use cascade_iface::{KwpConfig, KwpEcu};

    fn client(cfg: KwpConfig) -> Client<J2534Link<MockPassThru<SimLine<KwpEcu>>>> {
        Client::new(J2534Link::new(MockPassThru::new(SimLine::new(
            KwpEcu::new(cfg),
        ))))
    }

    #[test]
    fn session_over_j2534() {
        let mut cfg = KwpConfig {
            dtcs: vec![(0x0130, 0x20)],
            ..Default::default()
        };
        cfg.local_data.insert(0x01, vec![9; 78]);
        let mut c = client(cfg);
        assert_eq!(c.start().unwrap(), [0xef, 0x8f]);
        assert_eq!(c.read_dtcs().unwrap(), [(0x0130, 0x20)]);
        assert_eq!(c.read_local(0x01).unwrap(), vec![9; 78], "long frame");
        c.tester_present().unwrap();
        c.stop().unwrap();
        // the adapter got ISO 14230 timing: P1max 20 ms = 40, TiniL 25 ms, TWuP 50 ms
        let (baud, cfg) = c.line.pt.connected.clone().unwrap();
        assert_eq!(baud, 10_400);
        assert!(cfg.contains(&(config::P1_MAX, 40)));
        assert!(cfg.contains(&(config::TINIL, 25)));
        assert!(cfg.contains(&(config::TWUP, 50)));
    }

    #[test]
    fn guard_and_negative_responses_still_apply() {
        let mut c = client(KwpConfig::default());
        c.start().unwrap();
        assert!(matches!(c.request(&[0x27, 0x01]), Err(Error::Blocked(_))));
        assert!(matches!(
            c.request(&[0x1a, 0x99]),
            Err(Error::Negative { sid: 0x1a, .. })
        ));
    }

    #[test]
    fn silent_ecu_times_out() {
        let mut c = Client::new(J2534Link::new(MockPassThru::new(SimLine::new(
            cascade_iface::Null,
        ))));
        assert!(matches!(c.start(), Err(Error::Timeout(_))));
    }
}
