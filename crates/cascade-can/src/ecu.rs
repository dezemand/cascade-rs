//! The ECU side: answer diagnostic requests on a CAN bus. [`serve`] runs
//! the ISO-TP side, a [`Responder`] decides the answers; [`ObdSim`] is a
//! simulated OBD-II engine ECU (ISO 15765-4 addressing: requests on `7E0`
//! or functional `7DF`, answers on `7E8`).

use std::time::Duration;

use crate::CanBus;
use crate::isotp::{Error, IsoTp};

/// OBD-II on CAN, 11-bit: functional request, engine ECU request/response.
pub const OBD_FUNCTIONAL: u32 = 0x7df;
pub const OBD_ENGINE_REQUEST: u32 = 0x7e0;
pub const OBD_ENGINE_RESPONSE: u32 = 0x7e8;

/// Decides an ECU's answers.
pub trait Responder {
    /// The messages to send for `req` (service data, SID first), in order:
    /// usually one, `7F sid 78` (response pending) before a slow one,
    /// none to stay silent.
    fn answer(&mut self, req: &[u8]) -> Vec<Vec<u8>>;

    /// Whether answering `req` takes longer than P2 (50 ms): [`serve`]
    /// then sends `7F sid 78` (response pending) before asking [`Self::answer`].
    fn slow(&self, _req: &[u8]) -> bool {
        false
    }
}

impl<F: FnMut(&[u8]) -> Vec<Vec<u8>>> Responder for F {
    fn answer(&mut self, req: &[u8]) -> Vec<Vec<u8>> {
        self(req)
    }
}

/// Answer requests until `running()` turns false. Testers that break the
/// ISO-TP rules are logged and ignored; bus errors end it.
pub fn serve<B: CanBus, R: Responder + ?Sized>(
    tp: &mut IsoTp<B>,
    r: &mut R,
    running: &dyn Fn() -> bool,
) -> Result<(), Error> {
    while running() {
        let req = match tp.recv(Duration::from_millis(100)) {
            Ok(Some(req)) => req,
            Ok(None) => continue,
            Err(Error::Io(e)) => return Err(Error::Io(e)),
            Err(e) => {
                eprintln!("ecu: {e}");
                continue;
            }
        };
        if r.slow(&req) {
            match tp.send(&[0x7f, req[0], 0x78]) {
                Ok(()) => {}
                Err(Error::Io(e)) => return Err(Error::Io(e)),
                Err(e) => eprintln!("ecu: pending: {e}"),
            }
        }
        for msg in r.answer(&req) {
            match tp.send(&msg) {
                Ok(()) => {}
                Err(Error::Io(e)) => return Err(Error::Io(e)),
                Err(e) => eprintln!("ecu: answer: {e}"),
            }
        }
    }
    Ok(())
}

/// A simulated OBD-II engine ECU with plausible warm-idle values.
#[derive(Clone, Debug)]
pub struct ObdSim {
    pub rpm: f64,
    pub coolant_c: f64,
    pub speed_kmh: f64,
    pub map_kpa: f64,
    pub intake_c: f64,
    pub throttle_pct: f64,
    pub battery_v: f64,
    /// 17 characters.
    pub vin: String,
    /// Stored (mode 03) and pending (mode 07) codes, `P0117` = `0x0117`.
    pub stored: Vec<u16>,
    pub pending: Vec<u16>,
    /// Answer `7F 09 78` (response pending) before the VIN, as slow ECUs do.
    pub vin_pending: bool,
}

impl Default for ObdSim {
    fn default() -> Self {
        Self {
            rpm: 800.0,
            coolant_c: 88.0,
            speed_kmh: 0.0,
            map_kpa: 33.0,
            intake_c: 30.0,
            throttle_pct: 0.0,
            battery_v: 14.1,
            // a made-up VIN (check digit not computed)
            vin: "KMHSIMULATED00001".into(),
            stored: vec![0x0117],
            pending: vec![],
            vin_pending: true,
        }
    }
}

/// Mode 01 PIDs this ECU answers (besides the "supported" ones).
const PIDS: [u8; 10] = [0x01, 0x05, 0x0b, 0x0c, 0x0d, 0x0f, 0x11, 0x20, 0x40, 0x42];

/// A "PIDs supported" bitmap for `base+1 ..= base+32`.
fn supported(base: u8, pids: &[u8]) -> [u8; 4] {
    let mut m = [0u8; 4];
    for &p in pids {
        if p > base && p as u16 <= base as u16 + 32 {
            let i = (p - base - 1) as usize;
            m[i / 8] |= 0x80 >> (i % 8);
        }
    }
    m
}

impl ObdSim {
    fn pid(&self, pid: u8) -> Option<Vec<u8>> {
        let byte = |v: f64| v.round().clamp(0.0, 255.0) as u8;
        Some(match pid {
            0x00 => supported(0x00, &PIDS).to_vec(),
            0x20 => supported(0x20, &PIDS).to_vec(),
            0x40 => supported(0x40, &PIDS).to_vec(),
            // MIL on when codes are stored; spark ignition, tests complete
            0x01 => vec![
                (!self.stored.is_empty() as u8) << 7 | self.stored.len().min(127) as u8,
                0x07,
                0x65,
                0x00,
            ],
            0x05 => vec![byte(self.coolant_c + 40.0)],
            0x0b => vec![byte(self.map_kpa)],
            0x0c => ((self.rpm * 4.0).round().clamp(0.0, 65535.0) as u16)
                .to_be_bytes()
                .to_vec(),
            0x0d => vec![byte(self.speed_kmh)],
            0x0f => vec![byte(self.intake_c + 40.0)],
            0x11 => vec![byte(self.throttle_pct * 2.55)],
            0x42 => ((self.battery_v * 1000.0).round() as u16)
                .to_be_bytes()
                .to_vec(),
            _ => return None,
        })
    }

    fn codes(sid: u8, codes: &[u16]) -> Vec<u8> {
        let mut r = vec![sid + 0x40, codes.len() as u8];
        for c in codes {
            r.extend_from_slice(&c.to_be_bytes());
        }
        r
    }
}

impl Responder for ObdSim {
    fn answer(&mut self, req: &[u8]) -> Vec<Vec<u8>> {
        let Some(&sid) = req.first() else {
            return vec![];
        };
        let r = match (sid, &req[1..]) {
            // up to 6 PIDs per request on CAN; unsupported ones are left out
            (0x01, pids) if !pids.is_empty() && pids.len() <= 6 => {
                let mut r = vec![0x41];
                for &p in pids {
                    if let Some(d) = self.pid(p) {
                        r.push(p);
                        r.extend(d);
                    }
                }
                if r.len() == 1 {
                    return vec![]; // nothing supported: no answer
                }
                r
            }
            (0x03, []) => Self::codes(0x03, &self.stored),
            (0x07, []) => Self::codes(0x07, &self.pending),
            (0x04, []) => {
                self.stored.clear();
                self.pending.clear();
                vec![0x44]
            }
            (0x09, [0x00]) => {
                let mut r = vec![0x49, 0x00];
                r.extend(supported(0x00, &[0x02, 0x0a]));
                r
            }
            (0x09, [0x02]) => {
                let mut r = vec![0x49, 0x02, 0x01];
                r.extend(format!("{:<17.17}", self.vin).bytes());
                if self.vin_pending {
                    return vec![vec![0x7f, 0x09, 0x78], r];
                }
                r
            }
            (0x09, [0x0a]) => {
                let mut r = vec![0x49, 0x0a, 0x01];
                let mut name = b"ECM\0-EngineControl".to_vec();
                name.resize(20, 0);
                r.extend(name);
                r
            }
            (0x3e, [sub]) if sub & 0x80 != 0 => return vec![],
            (0x3e, [sub]) => vec![0x7e, *sub],
            (0x10, [0x01]) => vec![0x50, 0x01, 0x00, 0x32, 0x01, 0xf4],
            (0x10, _) => vec![0x7f, 0x10, 0x12],
            (0x01 | 0x03 | 0x04 | 0x07 | 0x09, _) => vec![0x7f, sid, 0x12],
            _ => vec![0x7f, sid, 0x11],
        };
        vec![r]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vbus::Hub;
    use std::sync::Arc;
    use std::sync::atomic::{AtomicBool, Ordering};

    #[test]
    fn obd_answers() {
        let mut e = ObdSim::default();
        assert_eq!(e.answer(&[0x01, 0x0c]), vec![vec![0x41, 0x0c, 0x0c, 0x80]]);
        assert_eq!(
            e.answer(&[0x01, 0x05, 0x0d, 0x99]),
            vec![vec![0x41, 0x05, 128, 0x0d, 0]]
        );
        assert_eq!(
            e.answer(&[0x01, 0x00]),
            vec![vec![0x41, 0, 0x88, 0x3a, 0x80, 0x01]]
        );
        assert_eq!(e.answer(&[0x01, 0x99]), Vec::<Vec<u8>>::new());
        assert_eq!(e.answer(&[0x03]), vec![vec![0x43, 1, 0x01, 0x17]]);
        assert_eq!(e.answer(&[0x04]), vec![vec![0x44]]);
        assert_eq!(e.answer(&[0x03]), vec![vec![0x43, 0]]);
        assert_eq!(e.answer(&[0x2e, 1, 2]), vec![vec![0x7f, 0x2e, 0x11]]);
        let vin = e.answer(&[0x09, 0x02]);
        assert_eq!(vin[0], vec![0x7f, 0x09, 0x78]);
        assert_eq!(&vin[1][3..], b"KMHSIMULATED00001");
    }

    #[test]
    fn serves_over_isotp() {
        let hub = Hub::new();
        let run = Arc::new(AtomicBool::new(true));
        let mut ecu = IsoTp::new(hub.endpoint(), OBD_ENGINE_RESPONSE, OBD_ENGINE_REQUEST);
        ecu.functional_id = Some(OBD_FUNCTIONAL);
        let r = run.clone();
        let t = std::thread::spawn(move || {
            serve(&mut ecu, &mut ObdSim::default(), &|| {
                r.load(Ordering::Relaxed)
            })
            .unwrap()
        });
        let mut tester = IsoTp::new(hub.endpoint(), OBD_FUNCTIONAL, OBD_ENGINE_RESPONSE);
        tester.send(&[0x01, 0x0c]).unwrap();
        let got = tester.recv(Duration::from_secs(1)).unwrap();
        assert_eq!(got, Some(vec![0x41, 0x0c, 0x0c, 0x80]));
        tester.tx_id = OBD_ENGINE_REQUEST;
        tester.send(&[0x09, 0x02]).unwrap();
        let pending = tester.recv(Duration::from_secs(1)).unwrap().unwrap();
        assert_eq!(pending, vec![0x7f, 0x09, 0x78]);
        let vin = tester.recv(Duration::from_secs(1)).unwrap().unwrap();
        assert_eq!(vin.len(), 20);
        run.store(false, Ordering::Relaxed);
        t.join().unwrap();
    }
}
