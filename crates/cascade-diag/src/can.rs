//! Diagnostics over CAN (ISO 15765): a [`Link`] that carries the client's
//! requests as ISO-TP messages, so [`crate::kwp::Client`] (and with it the
//! read-only guard, response-pending handling and the ELM327 front) works
//! unchanged on a CAN bus.
//!
//! CAN has no wake-up and no StartCommunication: [`CanLink`] answers `81`
//! (and `82`, StopCommunication) itself, and nothing is put on the bus for
//! them. The ISO 14230 header the client builds is dropped on the way out
//! and rebuilt from the request's addresses on the way in.
//!
//! The Tiburon GK has no CAN diagnostics (all its modules are on the K
//! line); this is for OBD-II on CAN cars (ISO 15765-4, `7DF`/`7E0`/`7E8`)
//! and for simulation.

use std::collections::VecDeque;
use std::thread;
use std::time::{Duration, Instant};

use cascade_can::CanBus;
use cascade_can::isotp::{self, IsoTp};
use cascade_iface::iso14230;

use crate::kwp::{Error, Link, Timing};

/// Extra time allowed on top of the client's P2 for USB/serial CAN
/// adapters, whose latency the ECU's 50 ms does not include.
const ADAPTER_SLACK: Duration = Duration::from_millis(50);

pub struct CanLink<B: CanBus> {
    pub tp: IsoTp<B>,
    opened: Instant,
    /// Answers made up locally (StartCommunication, StopCommunication),
    /// as ISO 14230 frames.
    local: VecDeque<Vec<u8>>,
    /// Addresses of the last request (tester, ECU), for the answers.
    addr: (u8, u8),
}

fn io(e: isotp::Error) -> Error {
    match e {
        isotp::Error::Io(e) => Error::Io(e),
        isotp::Error::Timeout(_) => Error::Timeout(Vec::new()),
        e => Error::Io(std::io::Error::other(e.to_string())),
    }
}

impl<B: CanBus> CanLink<B> {
    pub fn new(tp: IsoTp<B>) -> Self {
        Self {
            tp,
            opened: Instant::now(),
            local: VecDeque::new(),
            addr: (0xf1, 0x11),
        }
    }

    /// Positive answer to a session request handled here.
    fn local_answer(&mut self, data: &[u8]) -> bool {
        let answer: &[u8] = match data {
            // key bytes EF 8F: what a KWP2000 ECU would send
            [0x81] => &[0xc1, 0xef, 0x8f],
            [0x82] => &[0xc2],
            _ => return false,
        };
        let (tester, ecu) = self.addr;
        self.local.push_back(iso14230::encode(tester, ecu, answer));
        true
    }

    fn take(&mut self, frame: &[u8]) -> Result<Vec<u8>, Error> {
        let f = iso14230::parse(frame).ok_or_else(|| Error::BadFrame(frame.to_vec()))?;
        self.addr = (f.source, f.target);
        Ok(f.data)
    }
}

impl<B: CanBus> Link for CanLink<B> {
    fn elapsed(&self) -> Duration {
        self.opened.elapsed()
    }

    fn idle(&mut self, d: Duration) {
        thread::sleep(d)
    }

    fn fast_init(&mut self, start_frame: &[u8], timing: &Timing) -> Result<Vec<u8>, Error> {
        self.flush_input()?;
        self.send(start_frame)?;
        self.recv(timing.p2_max, timing.p1_max)
    }

    fn send(&mut self, frame: &[u8]) -> Result<(), Error> {
        let data = self.take(frame)?;
        if self.local_answer(&data) {
            return Ok(());
        }
        self.tp.send(&data).map_err(io)
    }

    fn recv(&mut self, first: Duration, _p1: Duration) -> Result<Vec<u8>, Error> {
        if let Some(f) = self.local.pop_front() {
            return Ok(f);
        }
        match self.tp.recv(first + ADAPTER_SLACK).map_err(io)? {
            Some(data) if data.len() <= 255 => {
                let (tester, ecu) = self.addr;
                Ok(iso14230::encode(tester, ecu, &data))
            }
            Some(data) => Err(Error::BadFrame(data)),
            None => Err(Error::Timeout(Vec::new())),
        }
    }

    fn flush_input(&mut self) -> Result<(), Error> {
        self.local.clear();
        // drop frames already waiting (a late answer to an earlier
        // request); bounded, for a busy bus
        for _ in 0..1000 {
            if self.tp.bus.recv(Duration::ZERO)?.is_none() {
                break;
            }
        }
        Ok(())
    }
}

/// OBD-II trouble codes from a mode 03/07/0A answer on CAN
/// (`43 count code code ..`); `0000` padding is left out.
pub fn obd_codes(r: &[u8]) -> Vec<u16> {
    let Some(&n) = r.get(1) else {
        return Vec::new();
    };
    r[2..]
        .as_chunks::<2>()
        .0
        .iter()
        .take(n as usize)
        .map(|c| u16::from_be_bytes(*c))
        .filter(|&c| c != 0)
        .collect()
}

/// Mode 01 PIDs `kdiag log --can` records: (PID, column name, unit).
pub const LOG_PIDS: [(u8, &str, &str); 7] = [
    (0x0c, "engine_speed", "rpm"),
    (0x0d, "vehicle_speed", "km/h"),
    (0x05, "coolant_temp", "C"),
    (0x0f, "intake_air_temp", "C"),
    (0x0b, "map", "kPa"),
    (0x11, "throttle", "%"),
    (0x42, "module_voltage", "V"),
];

/// The value of a mode 01 PID from its data bytes (after `41 pid`), SAE
/// J1979 scaling.
pub fn obd_value(pid: u8, d: &[u8]) -> Option<f64> {
    let a = *d.first()? as f64;
    let ab = || Some(u16::from_be_bytes([*d.first()?, *d.get(1)?]) as f64);
    Some(match pid {
        0x05 | 0x0f => a - 40.0,
        0x0b | 0x0d => a,
        0x0c => ab()? / 4.0,
        0x11 => a * 100.0 / 255.0,
        0x42 => ab()? / 1000.0,
        _ => return None,
    })
}

/// Whether a "PIDs supported" answer (`41 base m0 m1 m2 m3`) lists `pid`.
pub fn pid_supported(r: &[u8], pid: u8) -> bool {
    let (Some(&base), Some(m)) = (r.get(1), r.get(2..6)) else {
        return false;
    };
    if pid <= base || pid as u16 > base as u16 + 32 {
        return false;
    }
    let i = (pid - base - 1) as usize;
    m[i / 8] & (0x80 >> (i % 8)) != 0
}

/// An ECU on CAN that is really a K-line ECU: requests from CAN testers go
/// to the client (through its read-only guard) and the answers back, with
/// OBD mode 03/07 answers put in CAN form (count byte). The K line is far
/// slower than CAN's 50 ms P2, so every request is first answered
/// `7F sid 78` (response pending). A refused request is answered `7F sid
/// 11` (service not supported). On a K-line failure the session is started
/// again and the request tried once more; then no answer.
pub struct KlineBridge<'a, L: Link> {
    pub client: &'a mut crate::kwp::Client<L>,
    session: bool,
}

impl<'a, L: Link> KlineBridge<'a, L> {
    pub fn new(client: &'a mut crate::kwp::Client<L>) -> Self {
        Self {
            client,
            session: false,
        }
    }

    fn exchange(&mut self, req: &[u8]) -> Result<Vec<u8>, Error> {
        if !self.session {
            self.client.start()?;
            self.session = true;
        }
        self.client.request(req)
    }
}

impl<L: Link> cascade_can::ecu::Responder for KlineBridge<'_, L> {
    fn slow(&self, _req: &[u8]) -> bool {
        true
    }

    fn answer(&mut self, req: &[u8]) -> Vec<Vec<u8>> {
        let Some(&sid) = req.first() else {
            return vec![];
        };
        let mut r = self.exchange(req);
        if let Err(Error::Timeout(_) | Error::Io(_) | Error::BadFrame(_) | Error::Unexpected(_)) =
            &r
        {
            // the K-line session may have expired: once more, from the start
            self.session = false;
            r = self.exchange(req);
        }
        match r {
            Ok(r) if matches!(req, [0x03] | [0x07]) && (r.len() - 1) % 2 == 0 => {
                // K-line form: codes without a count byte
                let codes: Vec<u8> = r[1..]
                    .chunks(2)
                    .filter(|p| p != &[0, 0])
                    .flatten()
                    .copied()
                    .collect();
                let mut out = vec![r[0], (codes.len() / 2) as u8];
                out.extend(codes);
                vec![out]
            }
            Ok(r) => vec![r],
            Err(Error::Negative { sid, code }) => vec![vec![0x7f, sid, code]],
            Err(Error::Blocked(b)) => {
                eprintln!("bridge: refused {b}");
                vec![vec![0x7f, sid, 0x11]]
            }
            Err(e) => {
                eprintln!("bridge: {e}");
                self.session = false;
                vec![]
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kwp::Client;
    use cascade_can::ecu::{self, OBD_ENGINE_REQUEST, OBD_ENGINE_RESPONSE, ObdSim};
    use cascade_can::vbus::{Endpoint, Hub};
    use std::sync::Arc;
    use std::sync::atomic::{AtomicBool, Ordering};

    struct Car {
        run: Arc<AtomicBool>,
        ecu: Option<thread::JoinHandle<()>>,
    }

    impl Drop for Car {
        fn drop(&mut self) {
            self.run.store(false, Ordering::Relaxed);
            if let Some(t) = self.ecu.take() {
                t.join().unwrap();
            }
        }
    }

    fn car() -> (Car, Client<CanLink<Endpoint>>) {
        let hub = Hub::new();
        let run = Arc::new(AtomicBool::new(true));
        let mut tp = IsoTp::new(hub.endpoint(), OBD_ENGINE_RESPONSE, OBD_ENGINE_REQUEST);
        tp.functional_id = Some(ecu::OBD_FUNCTIONAL);
        let r = run.clone();
        let t = thread::spawn(move || {
            ecu::serve(&mut tp, &mut ObdSim::default(), &|| {
                r.load(Ordering::Relaxed)
            })
            .unwrap()
        });
        let link = CanLink::new(IsoTp::new(
            hub.endpoint(),
            OBD_ENGINE_REQUEST,
            OBD_ENGINE_RESPONSE,
        ));
        let mut c = Client::new(link);
        c.timing.p3_min = Duration::ZERO;
        (Car { run, ecu: Some(t) }, c)
    }

    #[test]
    fn client_over_can() {
        let (_car, mut c) = car();
        assert_eq!(c.start().unwrap(), [0xef, 0x8f]);
        assert_eq!(
            c.request(&[0x01, 0x0c]).unwrap(),
            vec![0x41, 0x0c, 0x0c, 0x80]
        );
        // multi-frame answer after a "response pending"
        let vin = c.request(&[0x09, 0x02]).unwrap();
        assert_eq!(&vin[3..], b"KMHSIMULATED00001");
        let codes = c.request(&[0x03]).unwrap();
        assert_eq!(obd_codes(&codes), vec![0x0117]);
        assert!(matches!(
            c.request(&[0x22, 0xf1, 0x90]),
            Err(Error::Negative {
                sid: 0x22,
                code: 0x11
            })
        ));
        c.stop().unwrap();
    }

    #[test]
    fn guard_still_applies_on_can() {
        let (_car, mut c) = car();
        assert!(matches!(c.request(&[0x04]), Err(Error::Blocked(_))));
        assert!(matches!(c.request(&[0x2e, 1, 2]), Err(Error::Blocked(_))));
        assert!(matches!(c.request(&[0x27, 1]), Err(Error::Blocked(_))));
        c.allow_clear = true;
        assert_eq!(c.request(&[0x04]).unwrap(), vec![0x44]);
        assert_eq!(obd_codes(&c.request(&[0x03]).unwrap()), Vec::<u16>::new());
    }

    #[test]
    fn silent_ecu_times_out() {
        let hub = Hub::new();
        let _nobody = hub.endpoint();
        let mut c = Client::new(CanLink::new(IsoTp::new(
            hub.endpoint(),
            OBD_ENGINE_REQUEST,
            OBD_ENGINE_RESPONSE,
        )));
        c.start().unwrap(); // answered locally
        assert!(matches!(c.request(&[0x01, 0x0c]), Err(Error::Timeout(_))));
    }

    /// A K-line link that takes real time: 400 ms for the wake-up and
    /// StartCommunication, 120 ms per answer.
    struct Slow<L: Link>(L);

    impl<L: Link> Link for Slow<L> {
        fn elapsed(&self) -> Duration {
            self.0.elapsed()
        }
        fn idle(&mut self, d: Duration) {
            self.0.idle(d)
        }
        fn fast_init(&mut self, f: &[u8], t: &Timing) -> Result<Vec<u8>, Error> {
            thread::sleep(Duration::from_millis(400));
            self.0.fast_init(f, t)
        }
        fn send(&mut self, f: &[u8]) -> Result<(), Error> {
            self.0.send(f)
        }
        fn recv(&mut self, first: Duration, p1: Duration) -> Result<Vec<u8>, Error> {
            thread::sleep(Duration::from_millis(120));
            self.0.recv(first, p1)
        }
        fn flush_input(&mut self) -> Result<(), Error> {
            self.0.flush_input()
        }
    }

    #[test]
    fn k_line_ecu_bridged_onto_can() {
        use crate::sim::SimLine;
        use cascade_iface::{KwpConfig, KwpEcu};
        let mut cfg = KwpConfig::default();
        cfg.responses
            .insert(vec![0x01, 0x0c], vec![0x41, 0x0c, 0x0c, 0x80]);
        // K-line mode 03: three code slots, no count
        cfg.responses
            .insert(vec![0x03], vec![0x43, 0x01, 0x17, 0x00, 0x00, 0x00, 0x00]);
        let hub = Hub::new();
        let run = Arc::new(AtomicBool::new(true));
        let mut tp = IsoTp::new(hub.endpoint(), OBD_ENGINE_RESPONSE, OBD_ENGINE_REQUEST);
        let r = run.clone();
        let t = thread::spawn(move || {
            // a K line as slow as the real one, in wall time
            let mut k = Client::new(Slow(SimLine::new(KwpEcu::new(cfg))));
            let mut bridge = KlineBridge::new(&mut k);
            ecu::serve(&mut tp, &mut bridge, &|| r.load(Ordering::Relaxed)).unwrap()
        });
        let mut c = Client::new(CanLink::new(IsoTp::new(
            hub.endpoint(),
            OBD_ENGINE_REQUEST,
            OBD_ENGINE_RESPONSE,
        )));
        // default (CAN) timing on the tester side
        c.start().unwrap();
        assert_eq!(
            c.request(&[0x01, 0x0c]).unwrap(),
            vec![0x41, 0x0c, 0x0c, 0x80]
        );
        assert_eq!(c.request(&[0x03]).unwrap(), vec![0x43, 1, 0x01, 0x17]);
        // refused by the bridge's own guard, even if the CAN tester allows it
        c.allow_clear = true;
        assert!(matches!(
            c.request(&[0x04]),
            Err(Error::Negative {
                sid: 0x04,
                code: 0x11
            })
        ));
        run.store(false, Ordering::Relaxed);
        t.join().unwrap();
    }

    #[test]
    fn pid_decoding() {
        assert_eq!(obd_value(0x0c, &[0x0c, 0x80]), Some(800.0));
        assert_eq!(obd_value(0x05, &[128]), Some(88.0));
        assert_eq!(obd_value(0x42, &[0x37, 0x14]), Some(14.1));
        let r = [0x41, 0x00, 0x88, 0x3a, 0x80, 0x01];
        assert!(pid_supported(&r, 0x0c) && pid_supported(&r, 0x20));
        assert!(!pid_supported(&r, 0x0e) && !pid_supported(&r, 0x42));
    }

    #[test]
    fn codes() {
        assert_eq!(
            obd_codes(&[0x43, 2, 0x01, 0x17, 0x43, 0x00]),
            vec![0x0117, 0x4300]
        );
        assert_eq!(obd_codes(&[0x43, 0]), Vec::<u16>::new());
    }
}
