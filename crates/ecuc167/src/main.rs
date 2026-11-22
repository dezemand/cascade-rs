//! ecuc167: disassemble and run the GK SIMK41 engine ECU dump (Infineon
//! C167 class) offline, with a simulated KWP2000 tester on ASC0.
//!
//! Offline analysis only. Nothing here talks to a car.
//!
//! ```text
//! ecuc167 dis  ROM ADDR [COUNT]        linear disassembly at a physical address
//! ecuc167 scan ROM [--refs HEX]...     recursive traversal from the vectors;
//!                                      lists code that references the values
//! ecuc167 run  ROM [options]           emulate; see `run` below
//! ```
//!
//! `run` options (`ecuc167 help run` lists them with their defaults):
//! - `-m MS` emulated time (default 3000), `--fcpu HZ` (default 16 MHz: the boot code's S0BG 0x2F
//!   is then 10,417 baud and the CAPCOM2 CC26 tick exactly 1 ms)
//! - `--init-at MS` start the tester: KWP2000 fast init (25 ms low, 25 ms
//!   high), then `81 11 F1 81 04`, then each `--req "HEX ..."` (service
//!   bytes; header and checksum are added), one after each ECU reply
//! - `--p3 MS` gap after an ECU reply before the next request (default 60)
//! - `--gap MS` tester inter-byte gap (default 2), `--wake-low MS` /
//!   `--wake-high MS` fast-init pattern (default 25 / 25)
//! - `--adc CH=VAL` ADC input (10 bit), `--no-echo`
//! - `--wheel PINS:TEETH:MISSING:RPM` a toothed wheel on timer inputs, `+`
//!   separated: `t0in`, `t7in`, `capin`, `cc0`..`cc31` (e.g.
//!   `t0in+capin:60:2:800`; `cc30` is the vehicle speed input)
//! - `--cam PINS:FALL:RISE[:FALL:RISE...]` a cam level locked to the last
//!   wheel: low from crank tooth position FALL to RISE over 720 degrees
//!   (0..2 x teeth), several intervals for a multi-tooth cam
//! - `--compares` print every CAPCOM compare match
//! - `--ram-in FILE@ADDR` / `--ram-out FILE@ADDR:LEN` load memory before
//!   reset / save it at the end (keep-alive RAM across power cycles)
//! - `--watch-from MS` start the `--watch` log at that time (the printed log
//!   is limited to 200,000 entries)
//! - `--ssc-trace` print every SSC byte sent, with the P6 latch (chip selects)
//! - `--p2-in HEX` levels of undriven P2 input pins (default 0)
//! - `--no-compare-on-write` compare matches only on timer counts, not when
//!   software writes a CAPCOM timer
//! - `--adc-at MS:CH=VAL` set an ADC input from that time on (later entries
//!   for the same channel win once their time is reached)
//! - `--rpm-at MS=RPM` change the first wheel's speed at that time
//! - `--trace-from MS --trace-n N` instruction trace, `--prof` hot spots,
//!   `--watch LO-HI` log accesses to a physical address range (`--watch-w`:
//!   writes only),
//!   `--stop-at PC`, `--dump ADDR LEN` print memory at the end,
//!   `--poke-at PC ADDR=HEX..` store bytes (physical) whenever PC is reached

use ecuc167::{cpu, isa};

use cpu::{Cpu, Event};
use std::collections::BTreeMap;
use std::process::ExitCode;

mod cli;
use cli::{Cmd, RunOpts};

fn hx(b: &[u8]) -> String {
    b.iter()
        .map(|x| format!("{x:02X}"))
        .collect::<Vec<_>>()
        .join(" ")
}

fn main() -> ExitCode {
    let (cli, m) = cli::parse();
    let r = match cli.cmd {
        Cmd::Dis { rom, addr, count } => cmd_dis(&rom, addr, count),
        Cmd::Scan { rom, refs, from } => cmd_scan(&rom, &refs, &from),
        Cmd::Run(a) => {
            cli::parse_run(*a, m.subcommand_matches("run").expect("run matches")).and_then(cmd_run)
        }
    };
    match r {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("{e}");
            ExitCode::from(2)
        }
    }
}

fn load(path: &str) -> Result<Vec<u8>, String> {
    let r = std::fs::read(path).map_err(|e| format!("{path}: {e}"))?;
    if !r.len().is_power_of_two() {
        return Err("ROM size must be a power of two".into());
    }
    Ok(r)
}

fn line(c: &Cpu, pc: u32) -> (u8, String) {
    let b = c.fetch4(pc);
    let (len, s) = c.disasm_at(pc);
    (
        len,
        format!("{pc:06X}  {:<11}  {s}", hx(&b[..len as usize]),),
    )
}

fn cmd_dis(rom: &str, addr: u32, n: usize) -> Result<(), String> {
    let rom = load(rom)?;
    let mut pc = addr;
    let c = Cpu::new(rom, 16_000_000);
    let mut extr = 0u8;
    for _ in 0..n {
        let b = c.fetch4(pc);
        let d = isa::decode(pc as u16, b);
        let s = isa::Fmt {
            d: &d,
            ext: extr > 0,
        };
        println!("{pc:06X}  {:<11}  {s}", hx(&b[..d.len as usize]));
        extr = match d.insn {
            isa::Insn::Extr(k) => k,
            isa::Insn::Ext {
                kind: isa::Ext::Extsr | isa::Ext::Extpr,
                cnt,
                ..
            } => cnt,
            _ => extr.saturating_sub(1),
        };
        pc += d.len as u32;
    }
    Ok(())
}

/// Recursive traversal from the reset/trap vectors. Static DPPs as set by
/// the boot code (0x12, 0x14, 0x13, 0x03) are used only for printing.
fn traverse(c: &Cpu, extra: &[u32]) -> BTreeMap<u32, isa::Decoded> {
    use isa::Insn::*;
    let mut seen = BTreeMap::new();
    let mut work: Vec<u32> = (0..0x200).step_by(4).collect();
    work.extend((0..0x80).map(|k| 0x50210 + 4 * k));
    work.extend_from_slice(extra);
    while let Some(mut pc) = work.pop() {
        loop {
            if seen.contains_key(&pc) || pc >= 0x80000 {
                break;
            }
            let d = isa::decode(pc as u16, c.fetch4(pc));
            if matches!(d.insn, Bad) {
                break;
            }
            seen.insert(pc, d);
            let seg = pc & 0xFF0000;
            let next = seg | (pc as u16).wrapping_add(d.len as u16) as u32;
            match d.insn {
                Jmpr { cc, t } | Jmpa { cc, t } => {
                    work.push(seg | t as u32);
                    if cc == 0 {
                        break;
                    }
                }
                Jbit { t, .. } => work.push(seg | t as u32),
                Jmps { seg, t } => {
                    work.push(((seg as u32) << 16) | t as u32);
                    break;
                }
                Calla { t, .. } | Callr { t } | Pcall { t, .. } => work.push(seg | t as u32),
                Calls { seg, t } => work.push(((seg as u32) << 16) | t as u32),
                Jmpi { cc: 0, .. } | Ret | Rets | Retp(_) | Reti => break,
                Sys("SRST") => break,
                _ => {}
            }
            pc = next;
        }
    }
    seen
}

fn cmd_scan(rom: &str, refs: &[u16], extra: &[u32]) -> Result<(), String> {
    let rom = load(rom)?;
    let c = Cpu::new(rom, 16_000_000);
    let code = traverse(&c, extra);
    println!("{} instructions reached", code.len());
    use isa::Op;
    let hit = |o: Op, v: u16| match o {
        Op::Mem(a) | Op::Imm(a) | Op::Off(_, a) => a == v,
        Op::Reg(r) => r < 0xF0 && isa::reg_addr(r, false) == v,
        _ => false,
    };
    for (&pc, d) in &code {
        use isa::Insn::*;
        let ops: Vec<Op> = match d.insn {
            Alu { d, s, .. } | Mov { d, s, .. } | Movx { d, s, .. } => vec![d, s],
            Cmpid { s, .. } | Scxt { s, .. } => vec![s],
            Push(r) | Pop(r) => vec![Op::Reg(r)],
            Calla { t, .. } | Jmpa { t, .. } | Callr { t } | Jmpr { t, .. } => {
                vec![Op::Imm(t)]
            }
            Calls { t, .. } | Jmps { t, .. } => vec![Op::Imm(t)],
            Bset { q, .. } | Bclr { q, .. } | Jbit { q, .. } | Bfld { q, .. } => {
                if q < 0xF0 {
                    vec![Op::Mem(isa::bitoff_addr(q, false))]
                } else {
                    vec![]
                }
            }
            Bit2 { zq, qq, .. } => [zq, qq]
                .iter()
                .filter(|&&q| q < 0xF0)
                .map(|&q| Op::Mem(isa::bitoff_addr(q, false)))
                .collect(),
            _ => vec![],
        };
        for &v in refs {
            if ops.iter().any(|&o| hit(o, v)) {
                println!("{}", line(&c, pc).1);
            }
        }
    }
    Ok(())
}

// ------------------------------------------------------------------ tester

/// ISO 14230 frame with target 0x11, source 0xF1 (length in the format
/// byte when it fits, else the extra length byte).
fn frame(data: &[u8]) -> Vec<u8> {
    let mut f = if data.len() < 64 {
        vec![0x80 | data.len() as u8, 0x11, 0xF1]
    } else {
        vec![0x80, 0x11, 0xF1, data.len() as u8]
    };
    f.extend_from_slice(data);
    let sum = f.iter().fold(0u8, |s, &b| s.wrapping_add(b));
    f.push(sum);
    f
}

enum Phase {
    Idle,
    /// Fast init: line low until .0, then idle high until .1.
    Wake(u64),
    /// Waiting for the ECU to answer; `last` = cycle of its last byte.
    Await {
        since: u64,
        last: Option<u64>,
    },
    Done,
}

fn cmd_run(o: RunOpts) -> Result<(), String> {
    let rom = load(&o.rom)?;
    let mut c = Cpu::new(rom, o.fcpu);
    c.echo = o.echo;
    for &(ch, v) in &o.adc {
        c.adc[ch & 15] = v;
    }
    c.wheels = o.wheels.clone();
    c.log_compares = o.compares;
    c.compare_on_write = o.cmp_wr;
    for (f, a) in &o.ram_in {
        let data = std::fs::read(f).map_err(|e| format!("{f}: {e}"))?;
        c.mem[*a as usize..*a as usize + data.len()].copy_from_slice(&data);
    }
    if o.ssc_trace {
        c.ssc_trace = Some(Vec::new());
    }
    if let Some(p) = o.p2_in {
        c.p2_in = p;
    }
    c.watch = if o.watch_from > 0.0 {
        Vec::new()
    } else {
        o.watch.clone()
    };
    c.watch_writes_only = o.watch_w;
    c.can.acked = o.can_ack;
    let mut can_next: Vec<u64> = vec![0; o.can_rx.len()];
    if o.prof {
        c.prof = Some(Default::default());
    }
    let cpm = o.fcpu as f64 / 1000.0;
    let ms = |cy: u64| cy as f64 / cpm;
    let max = (o.max_ms * cpm) as u64;
    // 10,400 baud bit time for the tester, independent of the ECU's S0BG.
    let tbit = (o.fcpu as f64 / 10_400.0) as u64;
    let send = |c: &mut Cpu, at: u64, bytes: &[u8], gap: u64| -> u64 {
        let mut t = at;
        for &b in bytes {
            c.line.frame(t, b, tbit);
            t += 10 * tbit + gap;
        }
        t
    };
    let mut phase = Phase::Idle;
    let mut reqs: std::collections::VecDeque<Vec<u8>> = o.reqs.iter().cloned().collect();
    let mut rx: Vec<u8> = Vec::new();
    let mut ecu_tx: Vec<(u64, u8)> = Vec::new();
    let mut tracing_left = 0u64;
    let p3 = (o.p3_ms * cpm) as u64;
    let gap = (o.gap_ms * cpm) as u64;
    let mut stop = None;
    while c.cycles < max {
        if let Some(tf) = o.trace_from
            && tracing_left == 0
            && !c.trace
            && ms(c.cycles) >= tf
        {
            c.trace = true;
            tracing_left = o.trace_n;
        }
        if c.trace {
            tracing_left = tracing_left.saturating_sub(1);
            if tracing_left == 0 {
                c.trace = false;
            }
        }
        if Some(c.pc()) == o.stop_at {
            println!("{:9.3} ms  stop at {:06X}", ms(c.cycles), c.pc());
            break;
        }
        for (pc, a, b) in &o.pokes {
            if c.pc() == *pc {
                for (i, &x) in b.iter().enumerate() {
                    c.mem[*a as usize + i] = x;
                }
            }
        }
        if let Some(&(t, r)) = o.rpm_at.iter().rev().find(|(t, _)| ms(c.cycles) >= *t)
            && let Some(w) = c.wheels.first_mut()
            && w.rpm != r
            && t >= 0.0
        {
            w.rpm = r;
        }
        if o.watch_from > 0.0 && c.watch.is_empty() && ms(c.cycles) >= o.watch_from {
            c.watch = o.watch.clone();
        }
        for &(t, ch, val) in &o.adc_at {
            if ms(c.cycles) >= t {
                c.adc[ch & 15] = val;
            }
        }
        if let Err(e) = c.step() {
            stop = Some(e);
            break;
        }
        for ev in c.events.drain(..) {
            match ev {
                Event::Tx { at, byte } => {
                    ecu_tx.push((at, byte));
                    rx.push(byte);
                    if let Phase::Await { last, .. } = &mut phase {
                        *last = Some(at);
                    }
                }
                Event::Compare { at, unit, ch, mode } => {
                    if o.compares {
                        println!(
                            "{:9.3} ms  CC{} mode {mode}",
                            ms(at),
                            unit as u32 * 16 + ch as u32
                        );
                    }
                }
                Event::Baud { at, s0bg, s0con } => println!(
                    "{:9.3} ms  ASC0 S0BG={s0bg:04X} S0CON={s0con:04X} ({:.0} baud)",
                    ms(at),
                    o.fcpu as f64
                        / ((s0bg as u64 + 1) * if s0con & 0x2000 != 0 { 48 } else { 32 }) as f64
                ),
                Event::Rx {
                    at,
                    byte,
                    fe,
                    enabled,
                } => {
                    if o.show_rx {
                        println!(
                            "{:9.3} ms  ECU rx {byte:02X}{}{}",
                            ms(at),
                            if fe { " FE" } else { "" },
                            if enabled { "" } else { " (receiver off)" }
                        )
                    }
                }
            }
        }
        for (at, f) in c.can.tx.drain(..) {
            if o.can_log {
                println!("{:9.3} ms  CAN tx {f:?}", ms(at));
            }
        }
        for (i, (id, data, period)) in o.can_rx.iter().enumerate() {
            if c.cycles >= can_next[i] {
                can_next[i] = c.cycles + (period * cpm) as u64;
                let f = cascade_can::Frame::new(*id, data);
                let taken = c.can.rx(&f);
                if o.can_log {
                    println!(
                        "{:9.3} ms  CAN rx {f:?}{}",
                        ms(c.cycles),
                        if taken { "" } else { " (no object)" }
                    );
                }
            }
        }
        let now = c.cycles;
        match phase {
            Phase::Idle => {
                if let Some(t) = o.init_at
                    && ms(now) >= t
                {
                    let lo_end = now + (o.wake_low * cpm) as u64;
                    c.line.pull_low(now, lo_end);
                    println!(
                        "{:9.3} ms  tester: fast init ({} ms low, {} ms high)",
                        ms(now),
                        o.wake_low,
                        o.wake_high
                    );
                    phase = Phase::Wake(lo_end + (o.wake_high * cpm) as u64);
                }
            }
            Phase::Wake(hi_end) => {
                if now >= hi_end {
                    let f = frame(&o.start);
                    println!("{:9.3} ms  tester  -> {}", ms(now), hx(&f));
                    send(&mut c, now, &f, gap);
                    rx.clear();
                    phase = Phase::Await {
                        since: now,
                        last: None,
                    };
                }
            }
            Phase::Await { since, last } => {
                let quiet = last.is_some_and(|l| now > l + (25.0 * cpm) as u64);
                let timeout = now > since + (1000.0 * cpm) as u64;
                if quiet || timeout {
                    if rx.is_empty() {
                        println!("{:9.3} ms  ECU     <- (no answer)", ms(now));
                    } else {
                        let t0 = last.unwrap_or(now);
                        println!("{:9.3} ms  ECU     <- {}", ms(t0), hx(&rx));
                    }
                    rx.clear();
                    match reqs.pop_front() {
                        Some(r) => {
                            let at = last.map_or(now, |l| l + p3).max(now);
                            let f = frame(&r);
                            println!("{:9.3} ms  tester  -> {}", ms(at), hx(&f));
                            send(&mut c, at, &f, gap);
                            phase = Phase::Await {
                                since: at,
                                last: None,
                            };
                        }
                        None => phase = Phase::Done,
                    }
                }
            }
            Phase::Done => {}
        }
    }
    if let Some(e) = stop {
        println!("{:9.3} ms  stopped: {e}", ms(c.cycles));
        println!("{}", line(&c, c.cur_pc).1);
    }
    println!(
        "{:9.3} ms  end: pc {:06X}, {} instructions, {} flash writes ignored, {} SSC bytes",
        ms(c.cycles),
        c.pc(),
        c.insns,
        c.flash_writes,
        c.ssc_log.len()
    );
    for (f, a, n) in &o.ram_out {
        std::fs::write(f, &c.mem[*a as usize..(*a + *n) as usize])
            .map_err(|e| format!("{f}: {e}"))?;
    }
    if let Some(t) = &c.ssc_trace {
        for (at, p6, con, w) in t {
            println!("{:9.3} ms  SSC P6={p6:04X} CON={con:04X} {w:04X}", ms(*at));
        }
    }
    for (i, w) in c.wheels.iter().enumerate() {
        println!(
            "           wheel {i}: {} edges, {} rpm at the end",
            w.edges, w.rpm
        );
    }
    if let Some(p) = &c.prof {
        let mut v: Vec<_> = p.iter().collect();
        v.sort_by(|a, b| b.1.cmp(a.1));
        for (pc, n) in v.iter().take(40) {
            println!("{n:10}  {}", line(&c, **pc).1);
        }
    }
    for (t, pc, a, v, w) in c.watch_log.iter().take(200_000) {
        println!(
            "{:9.3} ms  {pc:06X}  {} {a:06X} = {v:04X}",
            ms(*t),
            if *w { "W" } else { "R" }
        );
    }
    for &(a, n) in &o.dumps {
        for row in (0..n).step_by(16) {
            let b: Vec<u8> = (0..16.min(n - row)).map(|i| c.peek8(a + row + i)).collect();
            println!("{:06X}: {}", a + row, hx(&b));
        }
    }
    let _ = ecu_tx;
    Ok(())
}
