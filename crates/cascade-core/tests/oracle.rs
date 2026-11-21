//! Tests against traces produced by the C++ oracle (`oracle/`).
//!
//! Gated: they run only when `CASCADE_ORACLE_TRACE` points at an
//! uncompressed oracle log (`zstd -d traces/boot-5m.trace.zst`), so the
//! default `cargo test` works without proprietary ROMs or oracle output.

use std::fs::File;
use std::io::{BufRead, BufReader};

use cascade_core::trace::Reader;

fn oracle_trace() -> Option<String> {
    std::env::var("CASCADE_ORACLE_TRACE").ok()
}

/// Every record parses, and formatting it reproduces the oracle's two lines
/// exactly. This pins the format the Rust core will emit.
#[test]
fn oracle_trace_roundtrips() {
    let Some(path) = oracle_trace() else {
        eprintln!("CASCADE_ORACLE_TRACE not set; skipping");
        return;
    };
    let lines = || {
        BufReader::new(File::open(&path).expect("open trace"))
            .lines()
            .map(|l| l.expect("read trace"))
    };
    let mut expected = lines().filter(|l| l.starts_with("PC ") || l.starts_with("   "));
    let mut n = 0u64;
    for rec in Reader::new(lines()) {
        let rec = rec.expect("parse");
        let text = rec.to_string();
        let mut got = text.lines();
        assert_eq!(got.next(), expected.next().as_deref(), "record {n}");
        assert_eq!(got.next(), expected.next().as_deref(), "record {n}");
        n += 1;
    }
    assert!(n > 0, "no records in {path}");
    eprintln!("{n} records round-tripped");
}

/// The Rust core reproduces the oracle trace record for record.
/// Needs `CASCADE_ROM` (the Hyundai pack) and `CASCADE_ORACLE_TRACE`.
/// Runs as many cycles as the trace covers (the oracle's -m value is
/// inferred: the Rust run stops after the trace's last record).
#[test]
fn rust_core_matches_oracle_trace() {
    let (Some(trace), Ok(rom)) = (oracle_trace(), std::env::var("CASCADE_ROM")) else {
        eprintln!("CASCADE_ORACLE_TRACE / CASCADE_ROM not set; skipping");
        return;
    };
    let rom = std::fs::read(&rom).expect("read ROM");
    // the oracle's debug build runs with the fake interface
    let mut cpu = cascade_core::Cpu::new(rom, Box::new(cascade_iface::Fake::default()), false);
    let lines = BufReader::new(File::open(&trace).expect("open trace"))
        .lines()
        .map(|l| l.expect("read trace"));
    let mut n = 0u64;
    for expected in Reader::new(lines) {
        let expected = expected.expect("parse");
        let mut got = None;
        cpu.step(Some(&mut |r: &cascade_core::trace::Record| {
            got = Some(r.clone())
        }))
        .unwrap_or_else(|stop| panic!("stopped at record {n}: {stop}"));
        let got = got.expect("step produced no trace record");
        assert!(
            got.same_state(&expected),
            "record {n} differs\noracle:\n{expected}\nrust:\n{got}"
        );
        n += 1;
    }
    eprintln!("{n} records match");
}

/// Menu path Tiburon > Engine L4 > 2003 with the oracle's own key cycles
/// (from a `-d iface,serial` oracle run). The firmware does a KWP2000 fast
/// init and sends StartCommunication and TesterPresent; the bytes and the
/// cycle of every write must match what the oracle logged. Needs `CASCADE_ROM`.
#[test]
fn kwp_fast_init_matches_oracle() {
    use cascade_core::{Cpu, Key, KeyEvent, StepEvent};
    use cascade_iface::{Event, Fake, Traced};

    let Ok(rom) = std::env::var("CASCADE_ROM") else {
        eprintln!("CASCADE_ROM not set; skipping");
        return;
    };
    // (press cycle, key, hold cycles) as logged by the oracle
    let keys: &[(u64, Key, u64)] = &[
        (59899906, Key::ENTER, 1703937),
        (151781377, Key::ENTER, 1703938),
        (168820742, Key::DOWN, 1703931),
        (185729030, Key::DOWN, 1572866),
        (202637315, Key::DOWN, 1703936),
        (219545601, Key::DOWN, 1703937),
        (236453892, Key::DOWN, 1703944),
        (253362176, Key::DOWN, 1703936),
        (270270464, Key::DOWN, 1703938),
        (287309828, Key::DOWN, 1572864),
        (304218113, Key::ENTER, 1572870),
        (317194246, Key::ENTER, 1572867),
        (334102535, Key::DOWN, 1572857),
        (351010818, Key::DOWN, 1703936),
        (367919104, Key::DOWN, 1572869),
        (384827395, Key::ENTER, 1703937),
    ];
    let expected_tx: &[(u64, u8)] = &[
        (403529683, 0x81),
        (403584263, 0x11),
        (403639784, 0xf1),
        (403694364, 0x81),
        (403750511, 0x04),
        (404575980, 0x81),
        (404631501, 0x11),
        (404686081, 0xf1),
        (404741602, 0x3e),
        (404796182, 0xc1),
    ];

    let traced = Traced::new(Fake::default());
    let log = traced.log_handle();
    let mut cpu = Cpu::new(
        std::fs::read(rom).expect("read ROM"),
        Box::new(traced),
        false,
    );
    for &(at, key, hold) in keys {
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
    while cpu.cycles() < 420_000_000 {
        match cpu.step(None) {
            Ok(StepEvent::Executed) => {}
            other => panic!("unexpected {other:?} at {} cycles", cpu.cycles()),
        }
    }
    let tx: Vec<(u64, u8)> = log
        .borrow()
        .iter()
        .filter_map(|e| match e.event {
            Event::Tx(b) => Some((e.at, b)),
            _ => None,
        })
        .collect();
    assert_eq!(tx, expected_tx);
}
