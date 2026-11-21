//! `cascade` command-line tool.
//!
//! Commands:
//!   cascade run <rom> [-m CYCLES] [--trace FILE] [--dump DIR]
//!               [--key CYCLE:NAME[:HOLD]]... [--screen]
//!       Run the emulator headless with nothing on the diagnostic line.
//!       Stops after CYCLES (like the oracle's -m). --trace writes the
//!       oracle trace format; --dump writes `dump` and `dump.ram` like the
//!       oracle; --key presses NAME at CYCLE for HOLD cycles (default 3M);
//!       --screen prints the LCD as ASCII at the end.
//!   cascade trace-diff <left.log> <right.log>
//!       Compare two oracle-format traces by machine state and report the
//!       first divergence. Exit 0 if equal, 1 if they diverge, 2 on error.
//!   cascade trace-stats <log>
//!       Count records and distinct physical PCs.

use std::collections::HashSet;
use std::fs::File;
use std::io::{BufRead, BufReader, BufWriter, Write};
use std::process::ExitCode;

use cascade_core::trace::{Divergence, ParseError, Reader, Record, first_divergence};
use cascade_core::{Cpu, Key, KeyEvent, StepEvent, Stop};

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

fn parse_num(s: &str) -> Result<u64, String> {
    let s = s.replace('_', "");
    let r = if let Some(h) = s.strip_prefix("0x") {
        u64::from_str_radix(h, 16)
    } else if let Some(m) = s.strip_suffix('M') {
        m.parse::<u64>().map(|v| v * 1_000_000)
    } else {
        s.parse()
    };
    r.map_err(|e| format!("{s:?}: {e}"))
}

fn run(args: &[&str]) -> Result<ExitCode, String> {
    let mut rom_path = None;
    let mut max = u64::MAX;
    let mut trace_path = None;
    let mut dump_dir = None;
    let mut screen = false;
    let mut keys = Vec::new();
    let mut it = args.iter();
    while let Some(&a) = it.next() {
        let mut val = || {
            it.next()
                .copied()
                .ok_or_else(|| format!("{a} needs a value"))
        };
        match a {
            "-m" => max = parse_num(val()?)?,
            "--trace" => trace_path = Some(val()?),
            "--dump" => dump_dir = Some(val()?),
            "--screen" => screen = true,
            "--key" => {
                let v = val()?;
                let parts: Vec<&str> = v.split(':').collect();
                let (at, name, hold) = match parts.as_slice() {
                    [at, name] => (*at, *name, "3M"),
                    [at, name, hold] => (*at, *name, *hold),
                    _ => return Err(format!("bad --key {v:?}, want CYCLE:NAME[:HOLD]")),
                };
                let key = Key::from_name(name).ok_or_else(|| format!("unknown key {name:?}"))?;
                let at = parse_num(at)?;
                keys.push(KeyEvent {
                    at,
                    key,
                    down: true,
                });
                keys.push(KeyEvent {
                    at: at + parse_num(hold)?,
                    key,
                    down: false,
                });
            }
            _ if rom_path.is_none() && !a.starts_with('-') => rom_path = Some(a),
            _ => return Err(format!("unexpected argument {a:?}")),
        }
    }
    let rom_path = rom_path.ok_or("run needs a ROM path")?;
    let rom = std::fs::read(rom_path).map_err(|e| format!("{rom_path}: {e}"))?;

    let mut cpu = Cpu::with_null_interface(rom);
    cpu.max_cycles = max;
    for k in keys {
        cpu.queue_key(k);
    }

    let mut out = match trace_path {
        Some(p) => Some(BufWriter::new(
            File::create(p).map_err(|e| format!("{p}: {e}"))?,
        )),
        None => None,
    };
    let mut write_err = None;
    let mut sink = |r: &Record| {
        if let Some(w) = out.as_mut()
            && let Err(e) = writeln!(w, "{r}")
        {
            write_err.get_or_insert(e);
        }
    };
    let trace: Option<&mut dyn FnMut(&Record)> = if trace_path.is_some() {
        Some(&mut sink)
    } else {
        None
    };
    let stop = cpu.run(trace, |ev| {
        if let StepEvent::EndlessLoopReset { pc, phys, cycles } = ev {
            eprintln!("ENDLESS LOOP at {pc:04X} ({phys:08X}) at {cycles} cycles! (reset)");
        }
    });
    if let Some(mut w) = out {
        w.flush().map_err(|e| e.to_string())?;
    }
    if let Some(e) = write_err {
        return Err(format!("writing trace: {e}"));
    }

    eprintln!("{stop} at {} cycles, PC {:04X}", cpu.cycles(), cpu.pc());
    if let Some(dir) = dump_dir {
        std::fs::create_dir_all(dir).map_err(|e| e.to_string())?;
        std::fs::write(format!("{dir}/dump"), cpu.ram()).map_err(|e| e.to_string())?;
        std::fs::write(format!("{dir}/dump.ram"), cpu.mapped_ram()).map_err(|e| e.to_string())?;
    }
    if screen {
        print!("{}", cpu.lcd.render_ascii(2, 4));
    }
    Ok(if stop == Stop::MaxCycles {
        ExitCode::SUCCESS
    } else {
        ExitCode::from(1)
    })
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let args: Vec<&str> = args.iter().map(String::as_str).collect();
    let result = match args.as_slice() {
        ["run", rest @ ..] => run(rest),
        ["trace-diff", l, r] => trace_diff(l, r),
        ["trace-stats", p] => trace_stats(p),
        _ => Err("usage: cascade run <rom> [-m N] [--trace F] [--dump DIR] [--key C:NAME[:HOLD]] [--screen]\n       \
                  cascade trace-diff <left> <right>\n       cascade trace-stats <log>"
            .into()),
    };
    result.unwrap_or_else(|e| {
        eprintln!("cascade: {e}");
        ExitCode::from(2)
    })
}
