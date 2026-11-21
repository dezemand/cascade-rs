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
    let mut cpu = cascade_core::Cpu::with_null_interface(rom);
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
