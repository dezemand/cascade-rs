//! The rest of the car: ABS, traction control and airbag modules beside
//! the real engine ECU, each on the diagnostic line the Hi-Scan uses for it.
//!
//! These are models (`cascade_iface::KwpEcu`), not ECU code: their requests
//! and data layouts were learned by running the Hi-Scan firmware against
//! them (`protocol/tiburon-2003-abs-srs.md`, `tiburon-2003-tcs-atx-bcm.md`).
//! Their live data follows the engine model and road speed; their fault
//! codes follow the injected faults and are cleared by a tester.

use cascade_iface::{Cycles, Interface, KwpConfig, KwpEcu};

use crate::plant::{Controls, Fault, Plant};

/// Model time: `cascade_iface` cycles, 10,000 per ms.
pub const CYCLES_PER_MS: u64 = 10_000;

/// Status of a present airbag code, and of one that is history (as the
/// Hi-Scan shows them; emulator-derived).
const SRS_PRESENT: u8 = 0x71;
const SRS_HISTORY: u8 = 0x60;

#[derive(Clone)]
pub struct Module {
    pub name: &'static str,
    /// Hi-Scan line numbers it answers on (7 engine, 8 ABS/TCS, 12/10 SRS).
    pub lines: &'static [i32],
    pub ecu: KwpEcu,
}

impl Module {
    pub fn on(&self, line: i32) -> bool {
        self.lines.contains(&line)
    }
}

fn abs() -> Module {
    let cfg = KwpConfig {
        address: 0x28,
        key_bytes: [0xea, 0x8f],
        length_byte: true,
        ..Default::default()
    };
    Module {
        name: "abs",
        lines: &[8],
        ecu: KwpEcu::new(cfg),
    }
}

fn tcs() -> Module {
    let mut cfg = KwpConfig {
        address: 0x32,
        iso9141: true,
        ..Default::default()
    };
    cfg.responses.insert(vec![0xb1, 0x01], vec![0xf1, 0x01]);
    cfg.responses.insert(vec![0xb1, 0x2c], vec![0xf1, 0x2c, 0]);
    Module {
        name: "tcs",
        lines: &[8],
        ecu: KwpEcu::new(cfg),
    }
}

fn srs() -> Module {
    let mut cfg = KwpConfig {
        address: 0x58,
        key_bytes: [0xea, 0x8f],
        length_byte: true,
        ..Default::default()
    };
    // identification laid out as the Hi-Scan reads it (see kdiag `ident`)
    let mut id = vec![0x5a, 0x80];
    id.extend(b"GK-03");
    id.extend(b"95910-2C000");
    id.extend(b"SRSCM (SIMULATED)   ");
    id.resize(68, b' ');
    id.extend(b"12");
    cfg.responses.insert(vec![0x1a, 0x80], id);
    let mut crash = vec![0x61, 0x00];
    crash.extend([0u8; 40]);
    cfg.responses.insert(vec![0x21, 0x00, 0x01], crash.clone());
    cfg.responses.insert(vec![0x21, 0x00], crash);
    Module {
        name: "srs",
        lines: &[12, 10],
        ecu: KwpEcu::new(cfg),
    }
}

/// The modules beside the engine ECU.
pub fn modules() -> Vec<Module> {
    vec![abs(), tcs(), srs()]
}

/// Which module stores which code for a fault (two-byte codes, letter in
/// the top bits: C = 0x4000, B = 0x8000).
fn module_codes(f: Fault) -> &'static [(&'static str, u16)] {
    match f {
        Fault::AbsWheelSensorFl => &[("abs", 0x5200), ("tcs", 0x5200)],
        Fault::AbsPumpMotor => &[("abs", 0x6402)],
        Fault::SrsDriverAirbag => &[("srs", 0x9346)],
        Fault::SrsPassengerAirbag => &[("srs", 0x9352)],
        _ => &[],
    }
}

/// Follow the model: live data and fault codes.
pub fn update(mods: &mut [Module], plant: &Plant, c: &Controls) {
    let s = plant.sensors(c);
    let kmh = c.vehicle_kmh.round().clamp(0.0, 255.0) as u8;
    let fl = if c.faults.contains(&Fault::AbsWheelSensorFl) {
        0
    } else {
        kmh
    };
    for m in mods.iter_mut() {
        let cfg = &mut m.ecu.config;
        match m.name {
            "abs" => {
                // the 17-byte `21 01` block (tiburon-2003-abs-current-data.csv)
                let mut b = vec![0u8; 17];
                b[0..2].copy_from_slice(&((plant.rpm * 4.0) as u16).to_le_bytes());
                b[2] = kmh;
                b[3] = (c.throttle / 100.0 * 255.0) as u8;
                b[5] = (s.battery_v * 16.0).clamp(0.0, 255.0) as u8;
                b[6] = 213; // 4.99 V reference
                b[7..11].copy_from_slice(&[fl, kmh, kmh, kmh]);
                b[12] = 128; // 0 g
                b[13] = 128;
                cfg.local_data.insert(0x01, b);
            }
            "tcs" => {
                cfg.responses
                    .insert(vec![0xb1, 0x0e], vec![0xf1, 0x0e, fl, kmh, kmh, kmh]);
            }
            _ => {}
        }
        // codes: a present fault is (re)stored; the airbag module moves a
        // code to history once the fault is gone
        for f in Fault::ALL {
            for &(name, code) in module_codes(f) {
                if name != m.name {
                    continue;
                }
                let present = c.faults.contains(&f);
                let stored = cfg.dtcs.iter().position(|d| d.0 == code);
                match (present, stored, name) {
                    (true, None, "srs") => {
                        cfg.dtcs_history.retain(|d| d.0 != code);
                        cfg.dtcs.push((code, SRS_PRESENT));
                    }
                    (true, None, _) => cfg.dtcs.push((code, 0x60)),
                    (false, Some(i), "srs") => {
                        cfg.dtcs.remove(i);
                        if !cfg.dtcs_history.iter().any(|d| d.0 == code) {
                            cfg.dtcs_history.push((code, SRS_HISTORY));
                        }
                    }
                    _ => {}
                }
            }
        }
    }
}

/// Stored codes of a module for the status line: "C1200,C2402".
pub fn codes_text(m: &Module) -> String {
    let all: Vec<String> = m
        .ecu
        .config
        .dtcs
        .iter()
        .map(|d| {
            let kind = if m.name == "srs" { "present" } else { "stored" };
            format!("{}:{kind}", crate::ecu_view::dtc_name(d.0))
        })
        .chain(
            m.ecu
                .config
                .dtcs_history
                .iter()
                .map(|d| format!("{}:{}", crate::ecu_view::dtc_name(d.0), "history")),
        )
        .collect();
    if all.is_empty() {
        "none".into()
    } else {
        all.join(",")
    }
}

/// Model time now, from milliseconds since the server started.
pub fn now(ms: f64) -> Cycles {
    (ms * CYCLES_PER_MS as f64) as Cycles
}

/// Feed a 5-baud address to the modules on `line` (each checks it is
/// addressed).
pub fn slow_init(mods: &mut [Module], line: i32, addr: u8, now: Cycles) {
    let mut rx = Vec::new();
    for m in mods.iter_mut().filter(|m| m.on(line)) {
        m.ecu.send_slow_init(now, addr, &mut rx);
    }
}
