//! The CAN gateway against the simulated car's engine ECU in wall time:
//! a CAN tester (default OBD-on-CAN timing) asks over an in-memory bus,
//! `KlineBridge` forwards over ecusim's K-line (fast init, 10,400 baud).
//!
//! Needs a running `ecusim serve` with the ignition on; set
//! CASCADE_ECUSIM_SOCKET to its socket (`auto` for the default one).

#![cfg(all(feature = "serial", unix))]

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;

use cascade_can::ecu::{self, OBD_ENGINE_REQUEST, OBD_ENGINE_RESPONSE};
use cascade_can::isotp::IsoTp;
use cascade_can::vbus::Hub;
use cascade_diag::can::{CanLink, KlineBridge, obd_codes};
use cascade_diag::ecusim_line::EcusimLine;
use cascade_diag::kwp::Client;

#[test]
fn can_tester_through_gateway_to_ecusim() {
    let Ok(sock) = std::env::var("CASCADE_ECUSIM_SOCKET") else {
        eprintln!("skipped: set CASCADE_ECUSIM_SOCKET");
        return;
    };
    let path = if sock == "auto" {
        ecusim::default_socket()
    } else {
        sock.into()
    };
    let hub = Hub::new();
    let run = Arc::new(AtomicBool::new(true));
    let mut tp = IsoTp::new(hub.endpoint(), OBD_ENGINE_RESPONSE, OBD_ENGINE_REQUEST);
    let r = run.clone();
    let gw = thread::spawn(move || {
        let line = EcusimLine::connect(&path, 7).expect("ecusim socket");
        let mut k = Client::new(line);
        let mut bridge = KlineBridge::new(&mut k);
        ecu::serve(&mut tp, &mut bridge, &|| r.load(Ordering::Relaxed)).unwrap();
    });
    let mut c = Client::new(CanLink::new(IsoTp::new(
        hub.endpoint(),
        OBD_ENGINE_REQUEST,
        OBD_ENGINE_RESPONSE,
    )));
    c.start().unwrap();
    let rpm = c.request(&[0x01, 0x0c]).expect("01 0C through the gateway");
    assert_eq!(&rpm[..2], &[0x41, 0x0c]);
    let codes = c.request(&[0x03]).expect("03 through the gateway");
    eprintln!(
        "rpm answer {rpm:02X?}, codes {:04X?} (raw {codes:02X?})",
        obd_codes(&codes)
    );
    // refused by the gateway's guard
    c.allow_clear = true;
    assert!(c.request(&[0x04]).is_err());
    run.store(false, Ordering::Relaxed);
    gw.join().unwrap();
}
