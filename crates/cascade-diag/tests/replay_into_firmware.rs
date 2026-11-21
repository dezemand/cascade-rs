//! A `kdiag` session recorded as JSONL, replayed into the emulated Hi-Scan:
//! the firmware's DTC and Current Data screens get exactly the recorded
//! answers. Deterministic (key presses are cycle-stamped). Needs CASCADE_ROM.

use cascade_core::{Cpu, Key, KeyEvent, StepEvent};
use cascade_diag::kwp::Client;
use cascade_diag::sim::SimLine;
use cascade_diag::trace::TracedLine;
use cascade_iface::{KwpConfig, KwpEcu, replay};

/// Record a short session against the ECU model, as `kdiag --trace` would.
fn record() -> (String, Vec<u8>) {
    let mut cfg = KwpConfig::default();
    // distinctive values: 1234 rpm at offset 18 (u16 LE / 4), 0x99 elsewhere
    let mut block = vec![0x99u8; 60];
    block[18..20].copy_from_slice(&(1234u16 * 4).to_le_bytes());
    cfg.local_data.insert(0x01, block.clone());
    cfg.dtcs = vec![(0x0130, 0x20)];
    let mut out = Vec::new();
    {
        let line = TracedLine::new(SimLine::new(KwpEcu::new(cfg)), &mut out);
        let mut c = Client::new(line);
        c.start().unwrap();
        c.read_dtcs().unwrap();
        for _ in 0..3 {
            c.read_local(0x01).unwrap();
        }
        c.stop().unwrap();
    }
    let mut resp = vec![0x61, 0x01];
    resp.extend_from_slice(&block);
    (String::from_utf8(out).unwrap(), resp)
}

/// Drive the firmware down `path` (screen.py's timing) and return the
/// emulated ECU's exchange log.
fn run(rom: &[u8], cfg: KwpConfig, path: &[Key], until: u64) -> Vec<cascade_iface::kwp::Exchange> {
    let ecu = KwpEcu::new(cfg);
    let log = ecu.log_handle();
    let mut cpu = Cpu::new(rom.to_vec(), Box::new(ecu), false);
    let hold = 1_572_865;
    let mut keys = vec![(59_899_906, Key::ENTER)];
    keys.extend(
        path.iter()
            .enumerate()
            .map(|(i, &k)| (300_000_000 + i as u64 * 30_000_000, k)),
    );
    for (at, key) in keys {
        cpu.queue_key(KeyEvent {
            at,
            key,
            down: true,
        });
        cpu.queue_key(KeyEvent {
            at: at + hold,
            key,
            down: false,
        });
    }
    while cpu.cycles() < until {
        match cpu.step(None) {
            Ok(StepEvent::Executed) => {}
            other => panic!("unexpected {other:?} at {} cycles", cpu.cycles()),
        }
    }
    log.borrow().clone()
}

#[test]
fn recorded_session_drives_the_hiscan_screens() {
    let Ok(rom) = std::env::var("CASCADE_ROM") else {
        eprintln!("CASCADE_ROM not set; skipping");
        return;
    };
    let rom = std::fs::read(rom).expect("read ROM");
    let (trace, block_resp) = record();
    let rec = replay::from_jsonl(&trace).expect("parse trace");
    assert_eq!(rec.sequences[&vec![0x21, 0x01]].len(), 3);

    // Tiburon / Engine L4 / 2003, then 02 CURRENT DATA
    let mut path = vec![Key::ENTER];
    path.extend([Key::DOWN; 8]);
    path.extend([
        Key::ENTER,
        Key::ENTER,
        Key::DOWN,
        Key::DOWN,
        Key::DOWN,
        Key::ENTER,
    ]);
    path.extend([Key::DOWN, Key::ENTER]);
    let until = 300_000_000 + (path.len() as u64 + 3) * 30_000_000;
    let log = run(&rom, rec.config(), &path, until);

    let answered: Vec<_> = log
        .iter()
        .filter(|e| e.request == [0x21, 0x01])
        .map(|e| e.response.clone())
        .collect();
    assert!(
        answered.len() >= 3,
        "Current Data polled: {}",
        answered.len()
    );
    assert!(
        answered
            .iter()
            .all(|r| r.as_deref() == Some(&block_resp[..])),
        "every 21 01 got the recorded block"
    );
    assert_eq!(
        log.iter()
            .find(|e| e.request[0] == 0x81)
            .unwrap()
            .response
            .as_deref(),
        Some(&[0xc1, 0xef, 0x8f][..])
    );
}
