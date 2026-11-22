//! Reading the ECU's fault memory and readings straight from its RAM.
//! Needs a SIMK41 dump without immobiliser: CASCADE_ECU_ROM_NOIMMO.

use ecuc167::cpu::Cpu;
use ecusim::ecu_view::{readings, stored_codes};

const FCPU: u64 = 16_000_000;

fn run_to(c: &mut Cpu, ms: u64) {
    while c.cycles < ms * FCPU / 1000 {
        c.step().expect("ECU code stopped");
        c.events.clear();
    }
}

#[test]
fn stored_codes_and_readings_match_the_ecu() {
    let Ok(p) = std::env::var("CASCADE_ECU_ROM_NOIMMO") else {
        eprintln!("CASCADE_ECU_ROM_NOIMMO not set; skipping");
        return;
    };
    let mut c = Cpu::new(std::fs::read(p).expect("read ECU ROM"), FCPU);
    // 85 °C coolant, 14 V battery (sensor curves from ecusim::plant)
    c.adc[11] = ecusim::plant::adc_ntc(85.0);
    c.adc[4] = ecusim::plant::adc_battery(14.0);
    c.adc[14] = ecusim::plant::adc_ntc(25.0);
    run_to(&mut c, 4000);
    assert_eq!(stored_codes(&c), vec![], "healthy");
    let (rpm, _, coolant, battery) = readings(&c);
    assert_eq!(rpm, 0);
    assert!((coolant - 85.0).abs() < 1.5, "coolant {coolant}");
    assert!((battery - 14.0).abs() < 0.2, "battery {battery}");
    // throttle sensor shorted: P0122, present
    c.adc[10] = 0;
    run_to(&mut c, 20_000);
    let codes = stored_codes(&c);
    assert_eq!(codes.len(), 1, "{codes:?}");
    assert_eq!(codes[0].code, "P0122");
    assert!(codes[0].present());
}
