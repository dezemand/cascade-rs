//! The emulated Hi-Scan firmware talking to an ECU over a real serial path:
//! runner -> SerialKLine -> pseudo-terminal -> bus thread (echo + KwpEcu in
//! wall-clock time). Needs CASCADE_ROM; takes ~10 s.

#![cfg(feature = "serial")]

use std::io::{Read, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, mpsc};
use std::time::{Duration, Instant};

use cascade_core::Key;
use cascade_diag::kline_iface::SerialKLine;
use cascade_iface::{Interface, KwpConfig, KwpEcu, RxBuf};
use cascade_runner::{Command, IfaceKind, Runner, Snapshot, Speed};
use serialport::{SerialPort, TTYPort};

struct Out(Vec<u8>);
impl RxBuf for Out {
    fn push(&mut self, b: u8) {
        self.0.push(b);
    }
    fn flush(&mut self) {
        self.0.clear();
    }
}

fn wait_for(r: &Runner, what: impl Fn(&Snapshot) -> bool) -> Snapshot {
    let t = Instant::now();
    loop {
        let s = r.snapshot();
        if what(&s) {
            return s;
        }
        assert!(
            t.elapsed() < Duration::from_secs(60),
            "timed out; stopped: {:?}",
            s.stopped
        );
        std::thread::sleep(Duration::from_millis(10));
    }
}

#[test]
fn firmware_opens_kwp_session_over_serial() {
    let Ok(rom) = std::env::var("CASCADE_ROM") else {
        eprintln!("CASCADE_ROM not set; skipping");
        return;
    };
    let rom = std::fs::read(rom).expect("read ROM");
    let (tester_end, mut bus_end) = TTYPort::pair().expect("pty pair");
    bus_end.set_timeout(Duration::from_millis(1)).unwrap();

    // the bus: echo every byte (as a real K-line does) and run the ECU model
    let stop = Arc::new(AtomicBool::new(false));
    let stop2 = stop.clone();
    let (tx, rx) = mpsc::channel();
    let bus = std::thread::spawn(move || {
        let mut cfg = KwpConfig::default();
        cfg.local_data.insert(0x01, vec![0; 80]);
        let mut ecu = KwpEcu::new(cfg);
        let log = ecu.log_handle();
        let t0 = Instant::now();
        let now = || t0.elapsed().as_micros() as u64 * 10;
        let (mut out, mut buf) = (Out(Vec::new()), [0u8; 64]);
        while !stop2.load(Ordering::Relaxed) {
            if let Ok(n) = bus_end.read(&mut buf) {
                let _ = bus_end.write_all(&buf[..n]); // echo
                for &b in &buf[..n] {
                    ecu.send_byte(now(), b, &mut out);
                }
            }
            ecu.poll_input(now(), &mut out);
            if !out.0.is_empty() {
                let _ = bus_end.write_all(&out.0);
                out.0.clear();
            }
        }
        let summary: Vec<(Vec<u8>, bool)> = log
            .borrow()
            .iter()
            .map(|e| (e.request.clone(), e.response.is_some()))
            .collect();
        tx.send(summary).unwrap();
    });

    let port = Arc::new(Mutex::new(Some(tester_end)));
    let make: cascade_runner::IfaceFactory = Arc::new(move || {
        let p = port.lock().unwrap().take().ok_or("port already used")?;
        let mut k = SerialKLine::from_port(p, true).map_err(|e| e.to_string())?;
        k.ignore_baud_errors = true; // a pty has no baud rate
        Ok(Box::new(k) as Box<dyn Interface>)
    });
    let iface = IfaceKind::Custom {
        label: "pty K-line".into(),
        make,
    };
    let r = Runner::spawn_with(rom, iface, Speed::Max, None);

    let press = |k: Key| {
        r.send(Command::Key { key: k, down: true });
        r.send(Command::Key {
            key: k,
            down: false,
        });
    };
    let settle = || {
        let t0 = r.snapshot().total_cycles;
        wait_for(&r, |s| s.total_cycles > t0 + 40_000_000);
    };
    wait_for(&r, |s| s.total_cycles > 100_000_000);
    // info screen -> main menu -> Hyundai diagnosis -> Tiburon -> Engine L4
    press(Key::ENTER);
    settle();
    press(Key::ENTER);
    settle();
    for _ in 0..8 {
        press(Key::DOWN);
    }
    settle();
    press(Key::ENTER);
    settle();
    press(Key::ENTER);
    settle();
    for _ in 0..3 {
        press(Key::DOWN);
    }
    settle();
    // the session runs against a wall-clock ECU: real time from here
    r.send(Command::Speed(Speed::Real(1.0)));
    press(Key::ENTER); // 2003
    std::thread::sleep(Duration::from_secs(6));
    let snap = r.snapshot();
    drop(r);
    stop.store(true, Ordering::Relaxed);
    bus.join().unwrap();
    let exchanges = rx.recv().unwrap();

    let start = exchanges.iter().position(|(req, ok)| req == &[0x81] && *ok);
    assert!(
        start.is_some(),
        "no answered StartCommunication; exchanges {exchanges:?}; log {:?}",
        snap.log
    );
    let answered_after = exchanges[start.unwrap() + 1..]
        .iter()
        .filter(|(_, ok)| *ok)
        .count();
    assert!(
        answered_after >= 3,
        "session did not continue after StartCommunication: {exchanges:?}"
    );
}
