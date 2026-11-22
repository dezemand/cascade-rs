//! `cascade` command-line tool.
//!
//! Commands:
//! (`cascade help run` lists every option with its default.)
//!
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
//!       --kwp-p3-max MS changes its session timeout (default 5000);
//!       --pcs CYCLE:FILE writes every executed code address (physical for
//!       the code window) from CYCLE on, with counts;
//!       --rom-reads CYCLE:FILE writes every ROM offset read through the
//!       data window from CYCLE on, with counts;
//!       --eeprom F keeps the EEPROM in F between runs (loaded if it exists,
//!       written at the end; upstream uses ROM.eep), otherwise it starts blank;
//!       --exrom F (or -x F) loads an extended ROM, as upstream's -x: a
//!       Hyundai "TROUBLE SHOOTING PACK" (i-info) behind the main pack;
//!       --ecusim SOCKET|auto talks to `ecusim serve` (the ECU dump running in
//!       real time as its own process; implies --realtime); --realtime paces
//!       the emulator to wall time
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

mod cli;
mod png;

use std::collections::HashSet;
use std::fs::File;
use std::io::{BufRead, BufReader, BufWriter, Write};
use std::process::ExitCode;

use clap::ArgMatches;

use cli::{Cmd, RunArgs, iface_choice};

use cascade_core::trace::{Divergence, ParseError, Reader, Record, first_divergence};
use cascade_core::{Cpu, KeyEvent, StepEvent, Stop};

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

fn run(a: RunArgs, m: &ArgMatches) -> Result<ExitCode, String> {
    let RunArgs {
        rom: rom_path,
        max,
        trace: trace_path,
        dump: dump_dir,
        key,
        screen,
        png: png_path,
        lcd_dump: lcd_dump_path,
        line_log: line_log_path,
        stats,
        iface,
        kwp_log: kwp_log_path,
        dtc: dtcs,
        local,
        respond,
        kwp_addr,
        kwp_keybytes: kwp_kb,
        kwp_length_byte: kwp_len_byte,
        kwp_p3_max,
        kwp_5baud_keybytes: kwp_5baud_kb,
        kwp_iso9141,
        replay,
        exrom,
        eeprom: eeprom_file,
        ecu_rom,
        ecu_adc: adc,
        ecusim,
        realtime,
        rom_reads,
        pcs: pcs_out,
        continue_after_reset,
    } = a;
    let max = max.unwrap_or(u64::MAX);
    let local_data: std::collections::BTreeMap<u8, Vec<u8>> = local.into_iter().collect();
    let responses: std::collections::BTreeMap<Vec<u8>, Vec<u8>> = respond.into_iter().collect();
    let keys: Vec<KeyEvent> = key.into_iter().flatten().collect();
    let iface_name = iface_choice(&iface, m);
    // the ECU process runs in real time
    let realtime = realtime || ecusim.is_some();
    #[cfg(unix)]
    let ecusim_sock: Option<std::path::PathBuf> = ecusim.map(|v| {
        if v == "auto" {
            ecusim::default_socket()
        } else {
            v.into()
        }
    });
    #[cfg(not(unix))]
    if ecusim.is_some() {
        return Err("--ecusim needs a Unix build (ecusim serve)".into());
    }
    let replay = match replay {
        Some(p) => {
            let text = std::fs::read_to_string(&p).map_err(|e| format!("{p}: {e}"))?;
            let rec = cascade_iface::replay::from_jsonl(&text).map_err(|e| format!("{p}: {e}"))?;
            eprintln!(
                "replay: {} answers to {} distinct requests from {p} (address {:02X?}, {} unanswered)",
                rec.pairs(),
                rec.sequences.len(),
                rec.address,
                rec.unanswered
            );
            Some(rec)
        }
        None => None,
    };
    let rom = std::fs::read(&rom_path).map_err(|e| format!("{rom_path}: {e}"))?;

    let mut kwp_log = None;
    let inner: Box<dyn cascade_iface::Interface> = match iface_name {
        "fake" => Box::new(cascade_iface::Fake::default()),
        // experimental: the CAN Interface Module with a KWP2000-on-CAN ECU
        "canmod" => Box::new(cascade_iface::canmod::CanModule::new(
            cascade_iface::KwpConfig {
                dtcs: dtcs.clone(),
                local_data: local_data.clone(),
                responses: responses.clone(),
                ..Default::default()
            },
        )),
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
                p3_max: kwp_p3_max.map_or(base.p3_max, |ms| ms * cascade_iface::kwp::CYCLES_PER_MS),
                slow_init_key_bytes: kwp_5baud_kb.unwrap_or(base.slow_init_key_bytes),
                iso9141: kwp_iso9141 || base.iso9141,
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
        #[cfg(unix)]
        "ecusim" => {
            let p = ecusim_sock.ok_or("--iface ecusim needs --ecusim SOCKET")?;
            Box::new(
                ecusim::client::SocketKLine::connect(&p)
                    .map_err(|e| format!("{}: {e} (is `ecusim serve` running?)", p.display()))?,
            )
        }
        other => {
            return Err(format!(
                "unknown --iface {other:?} (fake, null, kwp, canmod, simk41, ecusim)"
            ));
        }
    };
    let traced = cascade_iface::Traced::new(inner);
    let line_log = traced.log_handle();
    let iface: Box<dyn cascade_iface::Interface> = Box::new(traced);
    let mut cpu = Cpu::new(rom, iface, false);
    if let Some(p) = &eeprom_file {
        match std::fs::read(p) {
            Ok(img) => cpu.eeprom.load_image(&img),
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
            Err(e) => return Err(format!("{p}: {e}")),
        }
    }
    if let Some(p) = &exrom {
        let image = std::fs::read(p).map_err(|e| format!("{p}: {e}"))?;
        cpu.load_extended_rom(image);
    }
    cpu.max_cycles = max;
    for k in keys {
        cpu.queue_key(k);
    }

    let mut out = match &trace_path {
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
    let started = std::time::Instant::now();
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
        // real time: 10,000 cycles per ms, checked every 10,000 cycles
        if realtime && cpu.cycles() % 10_000 < 8 {
            let ahead = (cpu.cycles() / 10).saturating_sub(started.elapsed().as_micros() as u64);
            if ahead > 2_000 {
                std::thread::sleep(std::time::Duration::from_micros(ahead));
            }
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
    if let Some(p) = &eeprom_file {
        std::fs::write(p, cpu.eeprom.to_image()).map_err(|e| format!("{p}: {e}"))?;
    }
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
    if let Some(dir) = &dump_dir {
        std::fs::create_dir_all(dir).map_err(|e| e.to_string())?;
        std::fs::write(format!("{dir}/dump"), cpu.ram()).map_err(|e| e.to_string())?;
        std::fs::write(format!("{dir}/dump.ram"), cpu.mapped_ram()).map_err(|e| e.to_string())?;
    }
    if let Some(p) = &line_log_path {
        let mut w = BufWriter::new(File::create(p).map_err(|e| format!("{p}: {e}"))?);
        for e in line_log.borrow().iter() {
            writeln!(w, "{} {:?}", e.at, e.event).map_err(|e| e.to_string())?;
        }
        w.flush().map_err(|e| e.to_string())?;
    }
    if let Some(p) = &kwp_log_path {
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
    if let Some(p) = &lcd_dump_path {
        std::fs::write(p, cpu.lcd.memory()).map_err(|e| format!("{p}: {e}"))?;
    }
    if let Some(p) = &png_path {
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

fn disasm(rom_path: &str, addr: u64, n: u64, bank: u64) -> Result<ExitCode, String> {
    use cascade_core::disasm::{Flow, decode};
    let mut pc = addr as u16;
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
    let (cli, m) = cli::parse();
    let result = match cli.cmd {
        Cmd::Run(a) => run(*a, m.subcommand_matches("run").expect("run matches")),
        Cmd::TraceDiff { left, right } => trace_diff(&left, &right),
        Cmd::TraceStats { log } => trace_stats(&log),
        Cmd::Disasm { rom, addr, n, bank } => disasm(&rom, addr, n, bank),
    };
    result.unwrap_or_else(|e| {
        eprintln!("cascade: {e}");
        ExitCode::from(2)
    })
}
