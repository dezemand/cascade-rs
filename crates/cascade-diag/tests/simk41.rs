//! kdiag and the Hi-Scan firmware against the engine ECU's own code (the
//! GK-20 SIMK41 dump in `ecuc167`), offline. Needs CASCADE_ECU_ROM (the
//! SIMK41 dump) and, for the Hi-Scan part, CASCADE_ROM. Both are proprietary
//! and stay outside the repository.

use cascade_core::{Cpu, Key, KeyEvent, StepEvent};
use cascade_diag::defs::{self, Value};
use cascade_diag::kwp::Client;
use cascade_diag::sim::SimLine;
use cascade_iface::{Event, Traced};
use ecuc167::iface::Simk41;

fn ecu() -> Option<Simk41> {
    let p = std::env::var("CASCADE_ECU_ROM").ok()?;
    Some(Simk41::new(
        std::fs::read(p).expect("read ECU ROM"),
        16_000_000,
    ))
}

#[test]
fn kdiag_client_against_simk41() {
    let Some(ecu) = ecu() else {
        eprintln!("CASCADE_ECU_ROM not set; skipping");
        return;
    };
    let mut c = Client::new(SimLine::new(ecu));
    assert_eq!(c.start().expect("StartCommunication"), [0xef, 0x8f]);
    assert_eq!(c.read_dtcs().expect("18 00 00 00"), vec![]);
    let block = c.read_local(0x01).expect("21 01");
    assert_eq!(block.len(), 78, "78 data bytes after 61 01");
    let params = defs::parse_csv(defs::TIBURON_2003_ENGINE_L4).unwrap();
    let get = |n: &str| match params.iter().find(|p| p.name == n).unwrap().decode(&block) {
        Some(Value::Number(v)) => v,
        other => panic!("{n}: {other:?}"),
    };
    // all ADC inputs at mid-scale (2.5 V)
    assert!((get("battery_voltage") - 13.0).abs() < 0.01);
    assert!((get("o2_front_b1") - 2497.5).abs() < 1.0);
    assert!((get("map_voltage") - 2500.0).abs() < 2.0);
    assert_eq!(get("engine_speed"), 0.0, "no crank signal");
    c.tester_present().expect("3E");
    c.stop().expect("82");
}

#[test]
fn hiscan_firmware_against_simk41() {
    let (Some(ecu), Ok(rom)) = (ecu(), std::env::var("CASCADE_ROM")) else {
        eprintln!("CASCADE_ROM / CASCADE_ECU_ROM not set; skipping");
        return;
    };
    let traced = Traced::new(ecu);
    let log = traced.log_handle();
    let mut cpu = Cpu::new(
        std::fs::read(rom).expect("read ROM"),
        Box::new(traced),
        false,
    );
    // screen.py's tiburon-l4 path, then 02 CURRENT DATA
    let mut path = vec![Key::ENTER];
    path.extend([Key::DOWN; 8]);
    path.extend([
        Key::ENTER,
        Key::ENTER,
        Key::DOWN,
        Key::DOWN,
        Key::DOWN,
        Key::ENTER,
        Key::DOWN,
        Key::ENTER,
    ]);
    let mut keys = vec![(59_899_906u64, Key::ENTER)];
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
            at: at + 1_572_865,
            key,
            down: false,
        });
    }
    let until = 300_000_000 + (path.len() as u64 + 3) * 30_000_000;
    while cpu.cycles() < until {
        let ev = cpu.step(None);
        assert!(matches!(ev, Ok(StepEvent::Executed)), "{ev:?}");
    }
    let rx: Vec<u8> = log
        .borrow()
        .iter()
        .filter_map(|e| match e.event {
            Event::Rx(b) => Some(b),
            _ => None,
        })
        .collect();
    let key_bytes = [0x83, 0xf1, 0x11, 0xc1, 0xef, 0x8f, 0xc4];
    assert!(
        rx.windows(7).any(|w| w == key_bytes),
        "StartCommunication answered"
    );
    let blocks = rx
        .windows(6)
        .filter(|w| *w == [0x80, 0xf1, 0x11, 0x50, 0x61, 0x01])
        .count();
    assert!(blocks >= 3, "Current Data polled the ECU: {blocks} blocks");
}

#[test]
fn j2534_link_against_simk41() {
    use cascade_diag::j2534::{J2534Link, MockPassThru};
    let Some(ecu) = ecu() else {
        eprintln!("CASCADE_ECU_ROM not set; skipping");
        return;
    };
    // the message-level path kdiag --j2534 uses, with a mock adapter in front
    // of the ECU code doing the init, framing and checksums
    let mut c = Client::new(J2534Link::new(MockPassThru::new(SimLine::new(ecu))));
    assert_eq!(c.start().expect("StartCommunication"), [0xef, 0x8f]);
    assert_eq!(c.read_local(0x01).expect("21 01").len(), 78);
    assert_eq!(c.read_dtcs().expect("18 00 00 00"), vec![]);
    c.stop().expect("82");
}
