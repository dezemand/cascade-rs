//! The engine ECU's CAN broadcasts on the emulated C167 CAN controller:
//! which identifiers, how often, and that the engine speed in them follows
//! the crank. Needs a SIMK41 dump without immobiliser:
//! set `CASCADE_ECU_ROM_NOIMMO`.

use cascade_can::Frame;
use ecuc167::cpu::{Cam, Cpu, Pin, Wheel};

const FCPU: u64 = 16_000_000;

#[test]
fn broadcasts_every_10_ms_with_the_engine_speed() {
    let Ok(p) = std::env::var("CASCADE_ECU_ROM_NOIMMO") else {
        eprintln!("skipped: set CASCADE_ECU_ROM_NOIMMO");
        return;
    };
    let rom = std::fs::read(&p).unwrap_or_else(|e| panic!("{p}: {e}"));
    let mut c = Cpu::new(rom, FCPU);
    let mut crank = Wheel::new(
        vec![
            Pin::Cc { unit: 0, ch: 15 },
            Pin::T7In,
            Pin::T0In,
            Pin::CapIn,
        ],
        60,
        2,
    );
    crank.cam = Some(Cam {
        targets: vec![Pin::Cc { unit: 0, ch: 14 }],
        lows: vec![(50, 110)],
    });
    crank.rpm = 800.0;
    c.wheels = vec![crank];
    c.can.acked = true; // another node on the bus
    let end = 3 * FCPU;
    let mut frames: Vec<(u64, Frame)> = Vec::new();
    while c.cycles < end {
        c.step().expect("ECU code stopped");
        c.events.clear();
        frames.append(&mut c.can.tx);
    }
    // the last second, after start-up
    let late: Vec<&(u64, Frame)> = frames.iter().filter(|(t, _)| *t >= 2 * FCPU).collect();
    for (id, len) in [(0x316, 8), (0x329, 7), (0x545, 4)] {
        let mine: Vec<_> = late.iter().filter(|(_, f)| f.id == id).collect();
        assert!(
            (95..=105).contains(&mine.len()),
            "{id:03X}: {} frames in 1 s",
            mine.len()
        );
        assert!(
            mine.iter().all(|(_, f)| f.data.len() == len),
            "{id:03X} length"
        );
    }
    let (_, dme1) = late.iter().rev().find(|(_, f)| f.id == 0x316).unwrap();
    let rpm = u16::from_le_bytes([dme1.data[2], dme1.data[3]]) as f64 * 0.25;
    assert!((rpm - 800.0).abs() < 10.0, "DME1 engine speed {rpm}");
}
