//! KWP2000 (ISO 14230) client over a [`Link`]: whole frames. Every byte-level
//! [`Line`] is a `Link` (this module frames the bytes and times the init);
//! a J2534 PassThru adapter is another (`crate::j2534`), where the adapter
//! does the wake-up, framing and timing itself.

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
            // ISO 14230 says 55; the SIMK43 drops requests that come
            // 55 ms after its answer at fast baud rates (gkbus), so 60
            p3_min: Duration::from_millis(60),
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

/// A diagnostic link that moves whole ISO 14230 frames. Frames are always
/// complete: header, data and checksum.
pub trait Link {
    /// Time since the link was opened (virtual for simulated links).
    fn elapsed(&self) -> Duration;

    /// Wait without traffic.
    fn idle(&mut self, d: Duration);

    /// Wake the bus (ISO 14230 fast init, 10,400 baud) and send
    /// `start_frame` (StartCommunication); returns the ECU's first frame.
    fn fast_init(&mut self, start_frame: &[u8], timing: &Timing) -> Result<Vec<u8>, Error>;

    /// Send one frame.
    fn send(&mut self, frame: &[u8]) -> Result<(), Error>;

    /// The next frame from the ECU: it must start within `first`, and its
    /// bytes may be at most `p1` apart.
    fn recv(&mut self, first: Duration, p1: Duration) -> Result<Vec<u8>, Error>;

    /// Drop anything already received.
    fn flush_input(&mut self) -> Result<(), Error>;
}

/// Extra time on top of P1/P2 when reading through a serial adapter: USB
/// serial chips hold received bytes back (an FTDI's latency timer is 16 ms
/// by default), so bytes the ECU sent in time can reach us late.
pub const ADAPTER_SLACK: Duration = Duration::from_millis(40);

/// How long fast init listens for the StartCommunication answer.
pub const FAST_INIT_LISTEN: Duration = Duration::from_millis(400);

/// Wake-up low/high time changes tried in turn when no answer is found
/// (cheap adapters' BREAK timing is not exact; values from gkbus).
pub const FAST_INIT_OFFSETS_MS: [i64; 3] = [0, 2, -2];

/// The ECU's StartCommunication answer somewhere in `heard`: adapters may
/// put a 00 (from the BREAK) and the request's echo before it, or none of
/// them, or lose bytes.
fn find_start_answer(heard: &[u8], tester: u8, ecu: u8) -> Option<Vec<u8>> {
    (0..heard.len()).find_map(|off| {
        let s = &heard[off..];
        let n = iso14230::expected_len(s)?;
        let f = iso14230::parse(s.get(..n)?)?;
        (f.target == tester && f.source == ecu && f.data.first() == Some(&0xc1))
            .then(|| s[..n].to_vec())
    })
}

fn shifted(d: Duration, ms: i64) -> Duration {
    if ms >= 0 {
        d + Duration::from_millis(ms as u64)
    } else {
        d.saturating_sub(Duration::from_millis((-ms) as u64))
    }
}

/// Fast init on a byte-level line, tolerant of adapters: see
/// [`Link::fast_init`] for `impl Link for Line`.
fn line_fast_init<L: Line>(
    l: &mut L,
    start_frame: &[u8],
    timing: &Timing,
) -> Result<Vec<u8>, Error> {
    let req = iso14230::parse(start_frame).ok_or_else(|| Error::BadFrame(start_frame.to_vec()))?;
    let (tester, ecu) = (req.source, req.target);
    l.set_baud(10_400)?;
    let mut last = Vec::new();
    for offset in FAST_INIT_OFFSETS_MS {
        // the bus must be idle (high) before the wake-up pattern
        Line::idle(l, Duration::from_millis(300));
        Line::flush_input(l)?;
        l.wake_up(
            shifted(timing.wake_low, offset),
            shifted(timing.wake_high, offset),
        )?;
        l.write_unchecked(start_frame)?;
        let deadline = l.elapsed() + FAST_INIT_LISTEN;
        let mut heard = Vec::new();
        while let Some(left) = deadline.checked_sub(l.elapsed()).filter(|d| !d.is_zero()) {
            let Some(b) = l.read_byte(left)? else {
                break;
            };
            heard.push(b);
            if let Some(f) = find_start_answer(&heard, tester, ecu) {
                return Ok(f);
            }
        }
        if !heard.is_empty() {
            last = heard;
        }
    }
    if last.is_empty() {
        return Err(Error::Timeout(last));
    }
    // Something answered, but not a readable StartCommunication answer
    // (some adapters mangle it). Whether the session is up, the next
    // request tells: TesterPresent, as the client would send it.
    let mut probe = if start_frame[0] == 0x80 {
        vec![0x80, ecu, tester, 2, 0x3e, 0x01]
    } else {
        vec![0x82, ecu, tester, 0x3e, 0x01]
    };
    probe.push(iso14230::checksum(&probe));
    Line::idle(l, timing.p3_min);
    Line::flush_input(l)?;
    l.write(&probe)?;
    let r = Link::recv(l, timing.p2_max, timing.p1_max)?;
    match iso14230::parse(&r) {
        Some(f) if f.target == tester && f.source == ecu && f.data.first() == Some(&0x7e) => {
            // the session is up; the key bytes were lost (reported as 00 00)
            let mut ok = vec![0x83, tester, ecu, 0xc1, 0x00, 0x00];
            ok.push(iso14230::checksum(&ok));
            Ok(ok)
        }
        _ => Err(Error::BadFrame(last)),
    }
}

impl<L: Line> Link for L {
    fn elapsed(&self) -> Duration {
        Line::elapsed(self)
    }

    fn idle(&mut self, d: Duration) {
        Line::idle(self, d)
    }

    /// Wake-up, StartCommunication, then listen up to 400 ms and pick the
    /// ECU's answer out of whatever the adapter delivers (a 00 from the
    /// BREAK, the echo, or neither). Without an answer the wake-up is tried
    /// again 2 ms longer, then 2 ms shorter. If only unreadable bytes come
    /// back, a TesterPresent decides whether the session is up.
    fn fast_init(&mut self, start_frame: &[u8], timing: &Timing) -> Result<Vec<u8>, Error> {
        line_fast_init(self, start_frame, timing)
    }

    fn send(&mut self, frame: &[u8]) -> Result<(), Error> {
        Ok(self.write(frame)?)
    }

    fn recv(&mut self, first: Duration, p1: Duration) -> Result<Vec<u8>, Error> {
        let mut frame = Vec::new();
        let mut timeout = first + ADAPTER_SLACK;
        let p1 = p1 + ADAPTER_SLACK;
        loop {
            match self.read_byte(timeout)? {
                Some(b) => frame.push(b),
                None => return Err(Error::Timeout(frame)),
            }
            if iso14230::expected_len(&frame) == Some(frame.len()) {
                return Ok(frame);
            }
            timeout = p1;
        }
    }

    fn flush_input(&mut self) -> Result<(), Error> {
        Ok(Line::flush_input(self)?)
    }
}

impl Link for Box<dyn Link> {
    fn elapsed(&self) -> Duration {
        (**self).elapsed()
    }
    fn idle(&mut self, d: Duration) {
        (**self).idle(d)
    }
    fn fast_init(&mut self, start_frame: &[u8], timing: &Timing) -> Result<Vec<u8>, Error> {
        (**self).fast_init(start_frame, timing)
    }
    fn send(&mut self, frame: &[u8]) -> Result<(), Error> {
        (**self).send(frame)
    }
    fn recv(&mut self, first: Duration, p1: Duration) -> Result<Vec<u8>, Error> {
        (**self).recv(first, p1)
    }
    fn flush_input(&mut self) -> Result<(), Error> {
        (**self).flush_input()
    }
}

pub struct Client<L: Link> {
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

impl<L: Link> Client<L> {
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
        crate::guard::check(&[0x81], self.allow_clear).map_err(Error::Blocked)?;
        let frame = self.encode(&[0x81]);
        let first = self.line.fast_init(&frame, &self.timing)?;
        let r = self.response(0x81, Some(first))?;
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
        let frame = self.encode(data);
        self.line.send(&frame)?;
        self.response(sid, None)
    }

    fn encode(&self, data: &[u8]) -> Vec<u8> {
        if self.length_byte {
            let mut f = vec![0x80, self.target, self.source, data.len() as u8];
            f.extend_from_slice(data);
            f.push(iso14230::checksum(&f));
            f
        } else {
            iso14230::encode(self.target, self.source, data)
        }
    }

    /// Wait for the answer to `sid` (after any "response pending"); `first`
    /// is a frame already received.
    fn response(&mut self, sid: u8, mut first: Option<Vec<u8>>) -> Result<Vec<u8>, Error> {
        let mut wait = self.timing.p2_max;
        loop {
            let frame = match first.take() {
                Some(f) => f,
                None => self.line.recv(wait, self.timing.p1_max)?,
            };
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

    /// What a cheap K-line adapter does around a fast init.
    #[derive(Clone, Copy, PartialEq, Debug)]
    enum Quirk {
        /// a 00 from the BREAK, then the request's echo, then the answer
        ZeroAndEcho,
        /// the first `n` bytes after the first wake-up are lost
        LoseFirst(usize),
        /// the StartCommunication answer arrives with a bad checksum
        Mangled,
        /// the ECU only wakes for a low phase of 27 ms or more
        WantsLongerLow,
    }

    struct Adapter {
        inner: SimLine<KwpEcu>,
        quirk: Quirk,
        injected: std::collections::VecDeque<u8>,
        drop: usize,
        mangle: bool,
        asleep: bool,
        wakes: Vec<Duration>,
    }

    impl Adapter {
        fn new(quirk: Quirk) -> Self {
            Self {
                inner: SimLine::new(KwpEcu::new(KwpConfig::default())),
                quirk,
                injected: Default::default(),
                drop: 0,
                mangle: false,
                asleep: false,
                wakes: Vec::new(),
            }
        }
    }

    impl Line for Adapter {
        fn elapsed(&self) -> Duration {
            Line::elapsed(&self.inner)
        }
        fn set_baud(&mut self, baud: u32) -> io::Result<()> {
            self.inner.set_baud(baud)
        }
        fn wake_up(&mut self, low: Duration, high: Duration) -> io::Result<()> {
            self.wakes.push(low);
            self.asleep = self.quirk == Quirk::WantsLongerLow && low < Duration::from_millis(27);
            if self.asleep {
                Line::idle(&mut self.inner, low + high);
                return Ok(());
            }
            match self.quirk {
                Quirk::LoseFirst(n) if self.wakes.len() == 1 => self.drop = n,
                Quirk::Mangled => self.mangle = true,
                _ => {}
            }
            self.inner.wake_up(low, high)
        }
        fn write(&mut self, data: &[u8]) -> io::Result<()> {
            self.inner.write(data)
        }
        fn write_unchecked(&mut self, data: &[u8]) -> io::Result<()> {
            if self.asleep {
                return Ok(());
            }
            if self.quirk == Quirk::ZeroAndEcho {
                self.injected.push_back(0x00);
                self.injected.extend(data);
            }
            self.inner.write(data)
        }
        fn read_byte(&mut self, timeout: Duration) -> io::Result<Option<u8>> {
            if let Some(b) = self.injected.pop_front() {
                return Ok(Some(b));
            }
            loop {
                let Some(mut b) = self.inner.read_byte(timeout)? else {
                    return Ok(None);
                };
                if self.drop > 0 {
                    self.drop -= 1;
                    continue;
                }
                if self.mangle {
                    // corrupt the first answer's key byte
                    if b == 0xef {
                        b = 0x55;
                        self.mangle = false;
                    }
                }
                return Ok(Some(b));
            }
        }
        fn flush_input(&mut self) -> io::Result<()> {
            self.injected.clear();
            Line::flush_input(&mut self.inner)
        }
        fn idle(&mut self, d: Duration) {
            Line::idle(&mut self.inner, d)
        }
    }

    #[test]
    fn fast_init_survives_adapter_quirks() {
        // 00 and echo before the answer (genuine FTDI)
        let mut c = Client::new(Adapter::new(Quirk::ZeroAndEcho));
        assert_eq!(c.start().unwrap(), [0xef, 0x8f]);
        c.tester_present().unwrap();
        assert_eq!(c.line.wakes.len(), 1);

        // the answer's first byte lost on the first try: wake-up again
        let mut c = Client::new(Adapter::new(Quirk::LoseFirst(1)));
        assert_eq!(c.start().unwrap(), [0xef, 0x8f]);
        assert_eq!(c.line.wakes.len(), 2);
        c.tester_present().unwrap();

        // unreadable answer: TesterPresent shows the session is up
        let mut c = Client::new(Adapter::new(Quirk::Mangled));
        assert_eq!(c.start().unwrap(), [0x00, 0x00]);
        c.tester_present().unwrap();

        // retried 2 ms longer
        let mut c = Client::new(Adapter::new(Quirk::WantsLongerLow));
        assert_eq!(c.start().unwrap(), [0xef, 0x8f]);
        assert_eq!(
            c.line.wakes,
            [Duration::from_millis(25), Duration::from_millis(27)]
        );
    }

    #[test]
    fn fast_init_gives_up_when_nothing_answers() {
        // the request goes to 0x11: nobody answers
        let cfg = KwpConfig {
            address: 0x28,
            ..Default::default()
        };
        let mut c = Client::new(SimLine::new(KwpEcu::new(cfg)));
        assert!(matches!(c.start(), Err(Error::Timeout(_))));
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
        Link::idle(&mut c.line, Duration::from_secs(6));
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
        let t = Link::elapsed(&c.line);
        assert!(matches!(c.request(&[0x11, 0x01]), Err(Error::Blocked(_))));
        assert!(matches!(c.clear_dtcs(0xff00), Err(Error::Blocked(_))));
        assert_eq!(Link::elapsed(&c.line), t, "nothing was sent");
        assert_eq!(c.read_dtcs().unwrap().len(), 1, "codes still stored");
        c.allow_clear = true;
        c.clear_dtcs(0xff00).unwrap();
        assert!(c.read_dtcs().unwrap().is_empty());
    }

    #[test]
    fn requests_respect_p3_min() {
        let mut c = client(KwpConfig::default());
        c.start().unwrap();
        let t0 = Link::elapsed(&c.line);
        c.tester_present().unwrap();
        c.tester_present().unwrap();
        // two exchanges: each at least P3min + frame times + P2 (25 ms in the model)
        assert!(Link::elapsed(&c.line) - t0 >= Duration::from_millis(2 * (55 + 25)));
    }
}
