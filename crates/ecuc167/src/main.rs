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
//! `run` options:
//! - `-m MS` emulated time (default 3000), `--fcpu HZ` (default 16 MHz: the boot code's S0BG 0x2F
//!   is then 10,417 baud and the CAPCOM2 CC26 tick exactly 1 ms)
//! - `--init-at MS` start the tester: KWP2000 fast init (25 ms low, 25 ms
//!   high), then `81 11 F1 81 04`, then each `--req "HEX ..."` (service
//!   bytes; header and checksum are added), one after each ECU reply
//! - `--p3 MS` gap after an ECU reply before the next request (default 60)
//! - `--gap MS` tester inter-byte gap (default 2), `--wake-low MS` /
//!   `--wake-high MS` fast-init pattern (default 25 / 25)
//! - `--adc CH=VAL` ADC input (10 bit), `--no-echo`
//! - `--trace-from MS --trace-n N` instruction trace, `--prof` hot spots,
//!   `--watch LO-HI` log accesses to a physical address range (`--watch-w`:
//!   writes only),
//!   `--stop-at PC`, `--dump ADDR LEN` print memory at the end,
//!   `--poke-at PC ADDR=HEX..` store bytes (physical) whenever PC is reached

use ecuc167::{cpu, isa};

use cpu::{Cpu, Event};
use std::collections::BTreeMap;
use std::process::ExitCode;

fn hex_bytes(s: &str) -> Result<Vec<u8>, String> {
    s.split([' ', ','])
        .filter(|t| !t.is_empty())
        .map(|t| u8::from_str_radix(t, 16).map_err(|e| format!("{t}: {e}")))
        .collect()
}

fn hexnum(s: &str) -> Result<u32, String> {
    u32::from_str_radix(s.trim_start_matches("0x"), 16).map_err(|e| format!("{s}: {e}"))
}

fn hx(b: &[u8]) -> String {
    b.iter()
        .map(|x| format!("{x:02X}"))
        .collect::<Vec<_>>()
        .join(" ")
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let r = match args.first().map(String::as_str) {
        Some("dis") => cmd_dis(&args[1..]),
        Some("scan") => cmd_scan(&args[1..]),
        Some("run") => cmd_run(&args[1..]),
        _ => Err("usage: ecuc167 dis|scan|run ROM ... (see source header)".into()),
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

fn cmd_dis(a: &[String]) -> Result<(), String> {
    let rom = load(a.first().ok_or("ROM")?)?;
    let mut pc = hexnum(a.get(1).ok_or("ADDR")?)?;
    let n: usize = a
        .get(2)
        .map_or(Ok(40), |s| s.parse().map_err(|e| format!("{e}")))?;
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

fn cmd_scan(a: &[String]) -> Result<(), String> {
    let rom = load(a.first().ok_or("ROM")?)?;
    let mut refs = Vec::new();
    let mut extra = Vec::new();
    let mut it = a[1..].iter();
    while let Some(x) = it.next() {
        match x.as_str() {
            "--refs" => refs.push(hexnum(it.next().ok_or("--refs HEX")?)? as u16),
            "--from" => extra.push(hexnum(it.next().ok_or("--from PC")?)?),
            s => return Err(format!("unknown option {s}")),
        }
    }
    let c = Cpu::new(rom, 16_000_000);
    let code = traverse(&c, &extra);
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
        for &v in &refs {
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

struct RunOpts {
    rom: String,
    max_ms: f64,
    fcpu: u64,
    init_at: Option<f64>,
    reqs: Vec<Vec<u8>>,
    p3_ms: f64,
    gap_ms: f64,
    wake_low: f64,
    wake_high: f64,
    adc: Vec<(usize, u16)>,
    echo: bool,
    trace_from: Option<f64>,
    trace_n: u64,
    prof: bool,
    watch: Vec<(u32, u32)>,
    stop_at: Option<u32>,
    dumps: Vec<(u32, u32)>,
    start: Vec<u8>,
    show_rx: bool,
    pokes: Vec<(u32, u32, Vec<u8>)>,
    watch_w: bool,
}

fn parse_run(a: &[String]) -> Result<RunOpts, String> {
    let mut o = RunOpts {
        rom: String::new(),
        max_ms: 3000.0,
        fcpu: 16_000_000,
        init_at: None,
        reqs: Vec::new(),
        p3_ms: 60.0,
        gap_ms: 2.0,
        wake_low: 25.0,
        wake_high: 25.0,
        adc: Vec::new(),
        echo: true,
        trace_from: None,
        trace_n: 2000,
        prof: false,
        watch: Vec::new(),
        stop_at: None,
        dumps: Vec::new(),
        start: vec![0x81],
        show_rx: false,
        pokes: Vec::new(),
        watch_w: false,
    };
    let mut it = a.iter();
    let f = |s: &str| s.parse::<f64>().map_err(|e| format!("{s}: {e}"));
    while let Some(x) = it.next() {
        let mut v = || it.next().cloned().ok_or(format!("{x} needs a value"));
        match x.as_str() {
            "-m" => o.max_ms = f(&v()?)?,
            "--fcpu" => o.fcpu = v()?.parse().map_err(|e| format!("{e}"))?,
            "--init-at" => o.init_at = Some(f(&v()?)?),
            "--req" => o.reqs.push(hex_bytes(&v()?)?),
            "--start" => o.start = hex_bytes(&v()?)?,
            "--p3" => o.p3_ms = f(&v()?)?,
            "--gap" => o.gap_ms = f(&v()?)?,
            "--wake-low" => o.wake_low = f(&v()?)?,
            "--wake-high" => o.wake_high = f(&v()?)?,
            "--no-echo" => o.echo = false,
            "--trace-from" => o.trace_from = Some(f(&v()?)?),
            "--trace-n" => o.trace_n = v()?.parse().map_err(|e| format!("{e}"))?,
            "--prof" => o.prof = true,
            "--show-rx" => o.show_rx = true,
            "--poke-at" => {
                let pc = hexnum(&v()?)?;
                let s = v()?;
                let (a, b) = s.split_once('=').ok_or("--poke-at PC ADDR=HEX..")?;
                o.pokes.push((pc, hexnum(a)?, hex_bytes(b)?));
            }
            "--stop-at" => o.stop_at = Some(hexnum(&v()?)?),
            "--adc" => {
                let s = v()?;
                let (c, val) = s.split_once('=').ok_or("--adc CH=VAL")?;
                o.adc
                    .push((c.parse().map_err(|e| format!("{e}"))?, hexnum(val)? as u16));
            }
            "--watch-w" => o.watch_w = true,
            "--watch" => {
                let s = v()?;
                let (l, h) = s.split_once('-').unwrap_or((&s, &s));
                o.watch.push((hexnum(l)?, hexnum(h)?));
            }
            "--dump" => {
                let a = hexnum(&v()?)?;
                let n = hexnum(&v()?)?;
                o.dumps.push((a, n));
            }
            s if s.starts_with('-') => return Err(format!("unknown option {s}")),
            s => o.rom = s.to_string(),
        }
    }
    if o.rom.is_empty() {
        return Err("run: ROM missing".into());
    }
    Ok(o)
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

fn cmd_run(a: &[String]) -> Result<(), String> {
    let o = parse_run(a)?;
    let rom = load(&o.rom)?;
    let mut c = Cpu::new(rom, o.fcpu);
    c.echo = o.echo;
    for &(ch, v) in &o.adc {
        c.adc[ch & 15] = v;
    }
    c.watch = o.watch.clone();
    c.watch_writes_only = o.watch_w;
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
    if let Some(p) = &c.prof {
        let mut v: Vec<_> = p.iter().collect();
        v.sort_by(|a, b| b.1.cmp(a.1));
        for (pc, n) in v.iter().take(40) {
            println!("{n:10}  {}", line(&c, **pc).1);
        }
    }
    for (t, pc, a, v, w) in c.watch_log.iter().take(5000) {
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
