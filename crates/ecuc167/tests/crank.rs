//! The engine ECU's own engine speed follows an emulated crank wheel.
//! Needs a SIMK41 dump without immobiliser (the immobiliser dump stops crank
//! processing when no immobiliser answers): set `CASCADE_ECU_ROM_NOIMMO`.

use ecuc167::cpu::{Cam, Cpu, Pin, Wheel};

const FCPU: u64 = 16_000_000;
/// Engine speed in rpm, as the ECU puts it in its `21 01` block (RAM E354).
const ENGINE_SPEED: u32 = 0xE354;

fn rom() -> Option<Vec<u8>> {
    let p = std::env::var("CASCADE_ECU_ROM_NOIMMO").ok()?;
    Some(std::fs::read(&p).unwrap_or_else(|e| panic!("{p}: {e}")))
}

fn run_to(c: &mut Cpu, ms: u64) {
    while c.cycles < ms * FCPU / 1000 {
        c.step().expect("ECU code stopped");
        c.events.clear();
    }
}

#[test]
fn engine_speed_tracks_the_crank_without_dropouts() {
    let Some(rom) = rom() else {
        eprintln!("skipped: set CASCADE_ECU_ROM_NOIMMO");
        return;
    };
    let mut c = Cpu::new(rom, FCPU);
    let mut crank = Wheel::new(
        vec![Pin::Cc { unit: 0, ch: 15 }, Pin::T7In, Pin::CapIn],
        60,
        2,
    );
    crank.cam = Some(Cam {
        targets: vec![Pin::Cc { unit: 0, ch: 14 }],
        fall: 30,
        rise: 90,
    });
    c.wheels = vec![crank];
    let mut t = 3000;
    run_to(&mut c, t);
    for rpm in [250.0, 800.0, 3000.0, 6000.0] {
        c.wheels[0].rpm = rpm;
        t += 1000; // settle
        run_to(&mut c, t);
        for _ in 0..20 {
            t += 50;
            run_to(&mut c, t);
            let got = c.peek16(ENGINE_SPEED) as f64;
            assert!(
                (got - rpm).abs() <= rpm * 0.01 + 2.0,
                "{rpm} rpm: ECU says {got} at {t} ms"
            );
        }
    }
    c.wheels[0].rpm = 0.0;
    t += 1500;
    run_to(&mut c, t);
    assert_eq!(c.peek16(ENGINE_SPEED), 0, "stopped engine reads 0");
}
