//! `cascade` command-line tool.
//!
//! Commands:
//!   cascade run <rom> [-m CYCLES] [--trace FILE] [--dump DIR]
//!               [--key CYCLE:NAME[:HOLD]]... [--screen]
//!       Run the emulator headless. The diagnostic line is upstream's fake
//!       ECU by default (as in the oracle's debug build); --iface null for
//!       nothing attached.
//!       Stops after CYCLES (like the oracle's -m). --trace writes the
//!       oracle trace format; --dump writes `dump` and `dump.ram` like the
//!       oracle; --key presses NAME at CYCLE for HOLD cycles (default 3M);
//!       --screen prints the LCD as ASCII at the end; --png FILE saves it;
//!       --line-log FILE writes every line event (TX/RX/baud/init) with
//!       its emulated cycle; --iface kwp attaches a KWP2000 engine ECU
//!       model and --kwp-log FILE writes its decoded request/response log;
//!       --dtc CODE:STATUS (repeatable, hex with 0x) stores a DTC in it;
//!       --local ID=HEX sets its ReadDataByLocalIdentifier block;
//!       --respond REQ=RESP (hex, SID first) answers one exact request;
//!       --kwp-addr ADDR sets the ECU address (engine 0x11, ABS 0x28, SRS 0x58);
//!       --kwp-keybytes HEX and --kwp-length-byte vary its StartComm reply;
//!       --pcs CYCLE:FILE writes every executed code address (physical for
//!       the code window) from CYCLE on, with counts;
//!       --rom-reads CYCLE:FILE writes every ROM offset read through the
//!       data window from CYCLE on, with counts;
//!       --exrom F (or -x F) loads an extended ROM, as upstream's -x: a
//!       Hyundai "TROUBLE SHOOTING PACK" (i-info) behind the main pack;
//!       --ecu-rom F runs a SIMK41 engine ECU dump (ecuc167) as the ECU
//!       (implies --iface simk41); --ecu-adc CH=VAL sets its ADC inputs (hex
//!       with 0x, 10 bit; default 0x200 = 2.5 V);
//!       --replay F.jsonl answers from a `kdiag --trace` recording (implies
//!       --iface kwp; address, key bytes and framing come from the trace); --stats prints
//!       coverage counters. Exit 0 at the cycle cap, 1 on any other stop,
//!       3 when the firmware's endless-loop reset fires (unless
//!       --continue-after-reset).
//!   cascade trace-diff <left.log> <right.log>
//!       Compare two oracle-format traces by machine state and report the
//!       first divergence. Exit 0 if equal, 1 if they diverge, 2 on error.
//!   cascade trace-stats <log>
//!       Count records and distinct physical PCs.
//!   cascade disasm <rom> <ADDR> [-n N] [--bank PHYS]
//!       Linear disassembly of N instructions (default 40) from virtual
//!       ADDR. Below 0xC000 the ROM maps 1:1; 0xC000-0xFFFF is the code
//!       window, whose physical start is --bank (default 0xC000).

mod png;

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

fn parse_hex(s: &str) -> Result<Vec<u8>, String> {
    let h: String = s.chars().filter(|c| !c.is_whitespace()).collect();
    if !h.len().is_multiple_of(2) {
        return Err(format!("odd hex length in {s:?}"));
    }
    (0..h.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&h[i..i + 2], 16).map_err(|e| format!("{s:?}: {e}")))
        .collect()
}

fn run(args: &[&str]) -> Result<ExitCode, String> {
    let mut rom_path = None;
    let mut max = u64::MAX;
    let mut trace_path = None;
    let mut dump_dir = None;
    let mut screen = false;
    let mut png_path = None;
    let mut lcd_dump_path = None;
    let mut line_log_path = None;
    let mut kwp_log_path = None;
    let mut dtcs: Vec<(u16, u8)> = Vec::new();
    let mut local_data = std::collections::BTreeMap::new();
    let mut responses = std::collections::BTreeMap::new();
    let mut kwp_addr: Option<u8> = None;
    let mut replay = None;
    let mut ecu_rom: Option<String> = None;
    let mut exrom: Option<String> = None;
    let mut adc: Vec<(usize, u16)> = Vec::new();
    let mut rom_reads: Option<(u64, String)> = None;
    let mut pcs_out: Option<(u64, String)> = None;
    let mut kwp_kb: Option<[u8; 2]> = None;
    let mut kwp_len_byte = false;
    let mut stats = false;
    let mut continue_after_reset = false;
    let mut iface_name = "fake";
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
            "--png" => png_path = Some(val()?),
            "--lcd-dump" => lcd_dump_path = Some(val()?),
            "--line-log" => line_log_path = Some(val()?),
            "--kwp-log" => kwp_log_path = Some(val()?),
            "--local" => {
                let v = val()?;
                let (id, hex) = v
                    .split_once('=')
                    .ok_or_else(|| format!("bad --local {v:?}, want ID=HEX"))?;
                let hex: String = hex.chars().filter(|c| !c.is_whitespace()).collect();
                if !hex.len().is_multiple_of(2) {
                    return Err(format!("odd hex length in --local {v:?}"));
                }
                let bytes = (0..hex.len())
                    .step_by(2)
                    .map(|i| u8::from_str_radix(&hex[i..i + 2], 16))
                    .collect::<Result<Vec<u8>, _>>()
                    .map_err(|e| format!("--local {v:?}: {e}"))?;
                local_data.insert(parse_num(id)? as u8, bytes);
            }
            "--kwp-addr" => kwp_addr = Some(parse_num(val()?)? as u8),
            "--pcs" => {
                let v = val()?;
                let (from, file) = v
                    .split_once(':')
                    .ok_or_else(|| format!("bad --pcs {v:?}, want FROM_CYCLE:FILE"))?;
                pcs_out = Some((parse_num(from)?, file.to_string()));
            }
            "--rom-reads" => {
                let v = val()?;
                let (from, file) = v
                    .split_once(':')
                    .ok_or_else(|| format!("bad --rom-reads {v:?}, want FROM_CYCLE:FILE"))?;
                rom_reads = Some((parse_num(from)?, file.to_string()));
            }
            "--exrom" | "-x" => exrom = Some(val()?.to_string()),
            "--ecu-rom" => {
                ecu_rom = Some(val()?.to_string());
                iface_name = "simk41";
            }
            "--ecu-adc" => {
                let v = val()?;
                let (ch, x) = v
                    .split_once('=')
                    .ok_or_else(|| format!("bad --ecu-adc {v:?}, want CH=VALUE"))?;
                adc.push((parse_num(ch)? as usize, parse_num(x)? as u16));
            }
            "--replay" => {
                let p = val()?;
                let text = std::fs::read_to_string(p).map_err(|e| format!("{p}: {e}"))?;
                let rec =
                    cascade_iface::replay::from_jsonl(&text).map_err(|e| format!("{p}: {e}"))?;
                eprintln!(
                    "replay: {} answers to {} distinct requests from {p} (address {:02X?}, {} unanswered)",
                    rec.pairs(),
                    rec.sequences.len(),
                    rec.address,
                    rec.unanswered
                );
                replay = Some(rec);
                iface_name = "kwp";
            }
            "--kwp-keybytes" => {
                let kb = parse_hex(val()?)?;
                kwp_kb = Some(kb.try_into().map_err(|_| "--kwp-keybytes needs 2 bytes")?);
            }
            "--kwp-length-byte" => kwp_len_byte = true,
            "--respond" => {
                let v = val()?;
                let (req, resp) = v
                    .split_once('=')
                    .ok_or_else(|| format!("bad --respond {v:?}, want REQHEX=RESPHEX"))?;
                responses.insert(parse_hex(req)?, parse_hex(resp)?);
            }
            "--dtc" => {
                let v = val()?;
                let (c, st) = v
                    .split_once(':')
                    .ok_or_else(|| format!("bad --dtc {v:?}, want CODE:STATUS"))?;
                dtcs.push((parse_num(c)? as u16, parse_num(st)? as u8));
            }
            "--stats" => stats = true,
            "--iface" => iface_name = val()?,
            "--continue-after-reset" => continue_after_reset = true,
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

    let mut kwp_log = None;
    let inner: Box<dyn cascade_iface::Interface> = match iface_name {
        "fake" => Box::new(cascade_iface::Fake::default()),
        "null" => Box::new(cascade_iface::Null),
        "kwp" => {
            let base = replay.as_ref().map(|r| r.config()).unwrap_or_default();
            let config = cascade_iface::KwpConfig {
                dtcs: dtcs.clone(),
                local_data: local_data.clone(),
                responses: responses.clone(),
                address: kwp_addr.unwrap_or(base.address),
                key_bytes: kwp_kb.unwrap_or(base.key_bytes),
                length_byte: kwp_len_byte || base.length_byte,
                ..base
            };
            let ecu = cascade_iface::KwpEcu::new(config);
            kwp_log = Some(ecu.log_handle());
            Box::new(ecu)
        }
        "simk41" => {
            let p = ecu_rom
                .as_deref()
                .ok_or("--iface simk41 needs --ecu-rom PATH")?;
            let rom = std::fs::read(p).map_err(|e| format!("{p}: {e}"))?;
            let mut ecu = ecuc167::iface::Simk41::new(rom, 16_000_000);
            for &(ch, v) in &adc {
                ecu.set_adc(ch, v);
            }
            Box::new(ecu)
        }
        other => {
            return Err(format!(
                "unknown --iface {other:?} (fake, null, kwp, simk41)"
            ));
        }
    };
    let traced = cascade_iface::Traced::new(inner);
    let line_log = traced.log_handle();
    let iface: Box<dyn cascade_iface::Interface> = Box::new(traced);
    let mut cpu = Cpu::new(rom, iface, false);
    if let Some(p) = &exrom {
        let image = std::fs::read(p).map_err(|e| format!("{p}: {e}"))?;
        cpu.load_extended_rom(image);
    }
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
    let tracing = trace_path.is_some();
    // Upstream resets the machine (cycles back to 0) on an endless loop, so
    // a cycle cap alone would never end the run. Stop there by default.
    let mut reset_seen = false;
    let mut reads_on = false;
    let mut pcs = std::collections::BTreeMap::<u32, u64>::new();
    let stop = loop {
        if let Some((from, _)) = &rom_reads
            && !reads_on
            && cpu.cycles() >= *from
        {
            cpu.record_rom_reads();
            reads_on = true;
        }
        if let Some((from, _)) = &pcs_out
            && cpu.cycles() >= *from
        {
            let pc = cpu.pc();
            *pcs.entry(cpu.code_phys(pc).unwrap_or(pc as u32))
                .or_default() += 1;
        }
        let t: Option<&mut dyn FnMut(&Record)> = if tracing { Some(&mut sink) } else { None };
        match cpu.step(t) {
            Ok(StepEvent::Executed) => {}
            Ok(StepEvent::EndlessLoopReset { pc, phys, cycles }) => {
                eprintln!(
                    "ENDLESS LOOP at {pc:04X} ({phys:08X}) at {cycles} cycles! (machine reset)"
                );
                if !continue_after_reset {
                    reset_seen = true;
                    break None;
                }
            }
            Err(stop) => break Some(stop),
        }
    };
    if let Some(mut w) = out {
        w.flush().map_err(|e| e.to_string())?;
    }
    if let Some((_, file)) = &pcs_out {
        let text: String = pcs.iter().map(|(p, n)| format!("{p:06X} {n}\n")).collect();
        std::fs::write(file, text).map_err(|e| format!("{file}: {e}"))?;
    }
    if let Some((_, file)) = &rom_reads {
        let mut text = String::new();
        for (phys, n) in cpu.take_rom_reads() {
            text.push_str(&format!("{phys:06X} {n}\n"));
        }
        std::fs::write(file, text).map_err(|e| format!("{file}: {e}"))?;
    }
    if let Some(e) = write_err {
        return Err(format!("writing trace: {e}"));
    }

    match &stop {
        Some(stop) => eprintln!("{stop} at {} cycles, PC {:04X}", cpu.cycles(), cpu.pc()),
        None => eprintln!("stopped after firmware reset"),
    }
    if stats {
        let st = &cpu.stats;
        let used: Vec<String> = (0..256)
            .filter(|&o| st.opcodes[o] > 0 && o != 0xfe)
            .map(|o| format!("{o:02X}"))
            .chain(
                (0..256)
                    .filter(|&o| st.fe_opcodes[o] > 0)
                    .map(|o| format!("FE{o:02X}")),
            )
            .collect();
        let total: u64 = st.opcodes.iter().sum();
        eprintln!("instructions: {total}");
        eprintln!(
            "distinct opcodes executed: {} [{}]",
            used.len(),
            used.join(" ")
        );
        eprintln!(
            "SWT interrupts: {}, rand() port reads: {}, UART TX bytes: {}",
            st.swt_interrupts, st.rand_reads, st.uart_tx
        );
    }
    if let Some(dir) = dump_dir {
        std::fs::create_dir_all(dir).map_err(|e| e.to_string())?;
        std::fs::write(format!("{dir}/dump"), cpu.ram()).map_err(|e| e.to_string())?;
        std::fs::write(format!("{dir}/dump.ram"), cpu.mapped_ram()).map_err(|e| e.to_string())?;
    }
    if let Some(p) = line_log_path {
        let mut w = BufWriter::new(File::create(p).map_err(|e| format!("{p}: {e}"))?);
        for e in line_log.borrow().iter() {
            writeln!(w, "{} {:?}", e.at, e.event).map_err(|e| e.to_string())?;
        }
        w.flush().map_err(|e| e.to_string())?;
    }
    if let Some(p) = kwp_log_path {
        let log = kwp_log.ok_or("--kwp-log needs --iface kwp")?;
        let mut w = BufWriter::new(File::create(p).map_err(|e| format!("{p}: {e}"))?);
        let hex = |b: &[u8]| {
            b.iter()
                .map(|x| format!("{x:02X}"))
                .collect::<Vec<_>>()
                .join(" ")
        };
        for e in log.borrow().iter() {
            let resp = e
                .response
                .as_deref()
                .map_or("(no response)".to_string(), hex);
            writeln!(
                w,
                "{} {:02X}->{:02X} {} => {}",
                e.at,
                e.source,
                e.target,
                hex(&e.request),
                resp
            )
            .map_err(|e| e.to_string())?;
        }
        w.flush().map_err(|e| e.to_string())?;
    }
    if let Some(p) = lcd_dump_path {
        std::fs::write(p, cpu.lcd.memory()).map_err(|e| format!("{p}: {e}"))?;
    }
    if let Some(p) = png_path {
        let lcd = &cpu.lcd;
        let img = png::encode(
            cascade_core::lcd::LCD_WIDTH,
            cascade_core::lcd::LCD_HEIGHT,
            2,
            |x, y| lcd.pixel(x, y),
        );
        std::fs::write(p, img).map_err(|e| format!("{p}: {e}"))?;
    }
    if screen {
        print!("{}", cpu.lcd.render_ascii(2, 4));
    }
    Ok(match stop {
        Some(Stop::MaxCycles) => ExitCode::SUCCESS,
        _ if reset_seen => ExitCode::from(3),
        _ => ExitCode::from(1),
    })
}

fn disasm(args: &[&str]) -> Result<ExitCode, String> {
    use cascade_core::disasm::{Flow, decode};
    let (mut rom_path, mut addr, mut n, mut bank) = (None, None, 40u64, 0xc000u64);
    let mut it = args.iter();
    while let Some(&a) = it.next() {
        let mut val = || it.next().copied().ok_or(format!("{a} needs a value"));
        match a {
            "-n" => n = parse_num(val()?)?,
            "--bank" => bank = parse_num(val()?)?,
            _ if rom_path.is_none() => rom_path = Some(a),
            _ if addr.is_none() => addr = Some(parse_num(a)? as u16),
            _ => return Err(format!("unexpected argument {a:?}")),
        }
    }
    let rom_path = rom_path.ok_or("disasm needs a ROM path")?;
    let mut pc = addr.ok_or("disasm needs an address")?;
    let rom = std::fs::read(rom_path).map_err(|e| format!("{rom_path}: {e}"))?;
    let phys = |v: u16| -> usize {
        if v >= 0xc000 {
            bank as usize + (v - 0xc000) as usize
        } else {
            v as usize
        }
    };
    for _ in 0..n {
        let p = phys(pc);
        let bytes: Vec<u8> = (0..6u16)
            .map(|i| rom.get(phys(pc.wrapping_add(i))).copied().unwrap_or(0xff))
            .collect();
        let Some(insn) = decode(&bytes, pc) else {
            break;
        };
        let hex: String = bytes[..insn.len as usize]
            .iter()
            .map(|b| format!("{b:02X} "))
            .collect();
        println!("{p:06X}  {pc:04X}  {hex:<18} {}", insn.text);
        if matches!(
            insn.flow,
            Flow::Ret | Flow::Jump(_) | Flow::Indirect | Flow::Stop
        ) {
            println!();
        }
        pc = pc.wrapping_add(insn.len as u16);
    }
    Ok(ExitCode::SUCCESS)
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let args: Vec<&str> = args.iter().map(String::as_str).collect();
    let result = match args.as_slice() {
        ["run", rest @ ..] => run(rest),
        ["trace-diff", l, r] => trace_diff(l, r),
        ["trace-stats", p] => trace_stats(p),
        ["disasm", rest @ ..] => disasm(rest),
        _ => Err("usage: cascade run <rom> [-m N] [--trace F] [--dump DIR] [--key C:NAME[:HOLD]] [--screen] [--png F] [--line-log F] [--stats] [--iface fake|null|kwp] [--kwp-log F] [--dtc C:S] [--local ID=HEX] [--respond REQ=RESP] [--kwp-addr A] [--kwp-keybytes HEX] [--kwp-length-byte] [--replay KDIAG.jsonl] [--exrom F] [--ecu-rom F] [--ecu-adc CH=VAL] [--rom-reads CYCLE:FILE] [--pcs CYCLE:FILE] [--continue-after-reset]\n       \
                  cascade trace-diff <left> <right>\n       cascade trace-stats <log>\n       cascade disasm <rom> <ADDR> [-n N] [--bank PHYS]"
            .into()),
    };
    result.unwrap_or_else(|e| {
        eprintln!("cascade: {e}");
        ExitCode::from(2)
    })
}
