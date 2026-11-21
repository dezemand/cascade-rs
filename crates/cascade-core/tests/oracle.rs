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
