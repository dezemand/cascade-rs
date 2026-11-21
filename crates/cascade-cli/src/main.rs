//! `cascade` command-line tool.
//!
//! Commands:
//!   cascade trace-diff <left.log> <right.log>
//!       Compare two oracle-format traces by machine state and report the
//!       first divergence. Exit 0 if equal, 1 if they diverge, 2 on error.
//!   cascade trace-stats <log>
//!       Count records and distinct physical PCs.

use std::collections::HashSet;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::process::ExitCode;

use cascade_core::trace::{Divergence, ParseError, Reader, Record, first_divergence};

fn records(path: &str) -> Result<impl Iterator<Item = Result<Record, ParseError>>, String> {
    let f = File::open(path).map_err(|e| format!("{path}: {e}"))?;
    let lines = BufReader::new(f).lines().map_while(Result::ok);
    Ok(Reader::new(lines))
}

fn trace_diff(left: &str, right: &str) -> Result<ExitCode, String> {
    let (n, d) = first_divergence(records(left)?, records(right)?).map_err(|e| e.to_string())?;
    match d {
        None => {
            println!("identical: {n} records");
            Ok(ExitCode::SUCCESS)
        }
        Some(Divergence::State {
            index,
            left: a,
            right: b,
        }) => {
            println!("state diverges at record {index}");
            println!("left:\n{a}\nright:\n{b}");
            Ok(ExitCode::from(1))
        }
        Some(Divergence::Length { len, left_longer }) => {
            let side = if left_longer { "right" } else { "left" };
            println!("{side} trace ends after {len} records; states equal up to there");
            Ok(ExitCode::from(1))
        }
    }
}

fn trace_stats(path: &str) -> Result<ExitCode, String> {
    let mut n = 0u64;
    let mut pcs = HashSet::new();
    for r in records(path)? {
        let r = r.map_err(|e| e.to_string())?;
        pcs.insert(r.phys);
        n += 1;
    }
    println!("{n} records, {} distinct physical PCs", pcs.len());
    Ok(ExitCode::SUCCESS)
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let result = match args
        .iter()
        .map(String::as_str)
        .collect::<Vec<_>>()
        .as_slice()
    {
        ["trace-diff", l, r] => trace_diff(l, r),
        ["trace-stats", p] => trace_stats(p),
        _ => Err("usage: cascade trace-diff <left> <right> | cascade trace-stats <log>".into()),
    };
    result.unwrap_or_else(|e| {
        eprintln!("cascade: {e}");
        ExitCode::from(2)
    })
}
