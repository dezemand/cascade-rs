//! kdiag: K-line diagnostics for the 2002 Hyundai Coupe (Tiburon GK).
//!
//!   kdiag dtc  (--sim | --port DEV) [--system engine|abs|srs] [line options]
//!   kdiag selftest --port DEV   (adapter checks, no ECU session)
//!   kdiag clear (--sim | --port DEV) [--system engine|abs] --allow-clear [--yes]
//!   kdiag log  (--sim | --port DEV) [--system engine|abs] [--out FILE.csv]
//!              [--interval-ms N] [--duration-s N] [line options]
//!
//! Line options: --trace FILE.jsonl (every byte with a timestamp),
//! --no-echo (adapter does not hear its own bytes), --wake break|zero
//! (fast-init method).
//!
//! Systems (addresses and framing as the Hi-Scan uses them, learned in the
//! emulator; see protocol/):
//!   engine  0x11 on the K line (OBD pin 7)
//!   abs     0x28 on OBD pin 8: the cable must connect pin 8 to K
//!   srs     0x58 on OBD pin 12: the cable must connect pin 12 to K
//!
//! Read-only by default: every request passes `cascade_diag::guard`, which
//! refuses anything but session handling and reads. `clear` is the one
//! exception and needs --allow-clear plus a typed confirmation (or --yes).
//!
//! --sim talks to the KwpEcu model in virtual time, with plausible values.
//! J2534: --j2534 DLL|auto (Windows; a vendor PassThru DLL, `auto` = the
//! first installed driver; build for i686-pc-windows-msvc for 32-bit DLLs)
//! replaces --port: the adapter does the fast init, framing and timing.
//! --j2534-mock (any OS) runs the same message-level path against --sim or
//! --sim-ecu. `kdiag j2534-list` shows installed drivers (Windows).
//!
//! --sim-ecu ROM runs the engine ECU's own code instead (a SIMK41 dump, in the
//! `ecuc167` emulator, virtual time; engine only). Sensors read mid-scale.
//! `log` writes one CSV row per `21 01` response: time, decoded values from
//! the embedded protocol definitions, and the raw block in hex, so logs can
//! be re-decoded if the definitions change.

use std::fs::File;
use std::io::{BufWriter, Write};
use std::process::ExitCode;
use std::time::Duration;

use cascade_diag::defs::{self, Value};
use cascade_diag::j2534::{J2534Link, MockPassThru};
use cascade_diag::kwp::{Client, Link, dtc_name, hex};
use cascade_diag::line::Line;
use cascade_diag::sim::SimLine;
use cascade_diag::trace::{TracedLine, TracedLink};
use cascade_iface::{KwpConfig, KwpEcu};

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum System {
    Engine,
    Abs,
    Srs,
}

impl System {
    fn address(self) -> u8 {
        match self {
            System::Engine => 0x11,
            System::Abs => 0x28,
            System::Srs => 0x58,
        }
    }

    /// ABS and SRS only accept the session with key bytes EA 8F and
    /// length-byte headers, and the Hi-Scan sends them that way.
    fn length_byte(self) -> bool {
        self != System::Engine
    }

    fn data_defs(self) -> Result<&'static str, String> {
        match self {
            System::Engine => Ok(defs::TIBURON_2003_ENGINE_L4),
            System::Abs => Ok(defs::TIBURON_2003_ABS),
            System::Srs => Err("no live data is known for SRS".into()),
        }
    }
}

#[derive(Clone)]
struct Opts {
    sim: bool,
    sim_ecu: Option<String>,
    port: Option<String>,
    j2534: Option<String>,
    j2534_mock: bool,
    out: Option<String>,
    trace: Option<String>,
    interval: Duration,
    duration: Option<Duration>,
    echo: bool,
    wake: String,
    system: System,
    allow_clear: bool,
    yes: bool,
}

fn parse(args: &[String]) -> Result<Opts, String> {
    let mut o = Opts {
        sim: false,
        sim_ecu: None,
        port: None,
        j2534: None,
        j2534_mock: false,
        out: None,
        trace: None,
        interval: Duration::from_millis(170),
        duration: None,
        echo: true,
        wake: "break".into(),
        system: System::Engine,
        allow_clear: false,
        yes: false,
    };
    let mut it = args.iter();
    while let Some(a) = it.next() {
        let mut val = || {
            it.next()
                .cloned()
                .ok_or_else(|| format!("{a} needs a value"))
        };
        match a.as_str() {
            "--sim" => o.sim = true,
            "--j2534" => o.j2534 = Some(val()?),
            "--j2534-mock" => o.j2534_mock = true,
            "--sim-ecu" => {
                o.sim_ecu = Some(val()?);
                o.sim = true;
            }
            "--port" => o.port = Some(val()?),
            "--out" => o.out = Some(val()?),
            "--trace" => o.trace = Some(val()?),
            "--no-echo" => o.echo = false,
            "--wake" => o.wake = val()?,
            "--allow-clear" => o.allow_clear = true,
            "--yes" => o.yes = true,
            "--system" => {
                o.system = match val()?.as_str() {
                    "engine" => System::Engine,
                    "abs" => System::Abs,
                    "srs" => System::Srs,
                    s => return Err(format!("unknown --system {s:?} (engine, abs, srs)")),
                }
            }
            "--interval-ms" => {
                o.interval = Duration::from_millis(val()?.parse().map_err(|e| format!("{e}"))?)
            }
            "--duration-s" => {
                o.duration = Some(Duration::from_secs(
                    val()?.parse().map_err(|e| format!("{e}"))?,
                ))
            }
            _ => return Err(format!("unexpected argument {a:?}")),
        }
    }
    let sources = o.sim as u8 + o.port.is_some() as u8 + o.j2534.is_some() as u8;
    if sources != 1 {
        return Err("give exactly one of --sim, --port DEV or --j2534 DLL|auto".into());
    }
    if o.j2534_mock && !o.sim {
        return Err("--j2534-mock goes with --sim or --sim-ecu".into());
    }
    Ok(o)
}

/// ABS simulator: standing still, 800 rpm, 14 V, brake pressed.
fn sim_abs_block() -> Vec<u8> {
    let mut b = vec![0u8; 17];
    b[0..2].copy_from_slice(&3200u16.to_le_bytes()); // 800 rpm
    b[5] = 224; // 14.0 V
    b[6] = 213; // 4.99 V
    b[12] = 128; // 0 g
    b[13] = 128;
    b[15] = 0b0100; // brake switch on
    b
}

fn sim_config(system: System) -> KwpConfig {
    let mut cfg = KwpConfig {
        address: system.address(),
        ..Default::default()
    };
    match system {
        System::Engine => {
            cfg.dtcs = vec![(0x0130, 0x20)];
            cfg.local_data
                .insert(0x01, cascade_diag::sim::engine_idle_block());
        }
        System::Abs | System::Srs => {
            cfg.key_bytes = [0xea, 0x8f];
            cfg.length_byte = true;
            cfg.local_data.insert(0x01, sim_abs_block());
            cfg.responses
                .insert(vec![0x13, 0x40, 0x00], vec![0x53, 0x01, 0x50, 0x1f]);
        }
    }
    cfg
}

/// The diagnostic link: a byte-level line (simulated or serial), or a J2534
/// device (real on Windows, or a mock in front of a simulated line).
fn open_link(o: &Opts) -> Result<Box<dyn Link>, String> {
    let traced = |l: Box<dyn Link>| -> Result<Box<dyn Link>, String> {
        Ok(match &o.trace {
            Some(p) => Box::new(TracedLink::new(
                l,
                File::create(p).map_err(|e| format!("{p}: {e}"))?,
            )),
            None => l,
        })
    };
    if let Some(dll) = &o.j2534 {
        return traced(open_j2534(dll)?);
    }
    if o.j2534_mock {
        let inner = Opts {
            trace: None,
            ..o.clone()
        };
        let line = open_line(&inner)?;
        return traced(Box::new(J2534Link::new(MockPassThru::new(line))));
    }
    Ok(Box::new(open_line(o)?))
}

#[cfg(windows)]
fn open_j2534(dll: &str) -> Result<Box<dyn Link>, String> {
    use cascade_j2534::PassThru;
    let path = if dll == "auto" {
        let d = cascade_j2534::drivers()
            .map_err(|e| format!("reading the J2534 registry: {e}"))?
            .into_iter()
            .next()
            .ok_or("no J2534 driver installed (see kdiag j2534-list)")?;
        eprintln!("J2534 driver: {} ({})", d.name, d.library);
        d.library
    } else {
        dll.to_string()
    };
    let mut pt = cascade_j2534::dll::DllPassThru::open(&path).map_err(|e| e.to_string())?;
    match pt.battery_mv() {
        Ok(mv) => eprintln!("adapter reads {:.1} V on pin 16", mv as f64 / 1000.0),
        Err(e) => eprintln!("battery voltage not available: {e}"),
    }
    Ok(Box::new(J2534Link::new(pt)))
}

#[cfg(not(windows))]
fn open_j2534(_dll: &str) -> Result<Box<dyn Link>, String> {
    Err(
        "--j2534 needs Windows and the vendor's J2534 DLL; on macOS use --port \
         (K-line mode) or --j2534-mock with --sim to rehearse"
            .into(),
    )
}

#[cfg(windows)]
fn j2534_list() -> Result<(), String> {
    let ds = cascade_j2534::drivers().map_err(|e| e.to_string())?;
    if ds.is_empty() {
        println!("no J2534 04.04 drivers registered");
    }
    for d in ds {
        println!("{}  {}", d.name, d.library);
    }
    Ok(())
}

#[cfg(not(windows))]
fn j2534_list() -> Result<(), String> {
    Err("J2534 drivers are registered in the Windows registry; run this on Windows".into())
}

fn open_line(o: &Opts) -> Result<Box<dyn Line>, String> {
    let line: Box<dyn Line> = if o.sim {
        match &o.sim_ecu {
            Some(rom_path) => {
                if o.system != System::Engine {
                    return Err("--sim-ecu runs the engine ECU only".into());
                }
                let rom = std::fs::read(rom_path).map_err(|e| format!("{rom_path}: {e}"))?;
                Box::new(SimLine::new(ecuc167::iface::Simk41::new(rom, 16_000_000)))
            }
            None => Box::new(SimLine::new(KwpEcu::new(sim_config(o.system)))),
        }
    } else {
        open_serial(o)?
    };
    Ok(match &o.trace {
        Some(p) => Box::new(TracedLine::new(
            line,
            File::create(p).map_err(|e| format!("{p}: {e}"))?,
        )),
        None => line,
    })
}

#[cfg(all(feature = "serial", unix))]
fn open_serial(o: &Opts) -> Result<Box<dyn Line>, String> {
    use cascade_diag::serial::{SerialLine, WakeMethod};
    let port = o.port.as_deref().unwrap_or_default();
    let mut l = SerialLine::open(port, o.echo).map_err(|e| format!("{port}: {e}"))?;
    l.wake = match o.wake.as_str() {
        "break" => WakeMethod::Break,
        "zero" => WakeMethod::ZeroByte,
        w => return Err(format!("unknown --wake {w:?} (break, zero)")),
    };
    Ok(Box::new(l))
}

#[cfg(not(all(feature = "serial", unix)))]
fn open_serial(_o: &Opts) -> Result<Box<dyn Line>, String> {
    Err("the serial K-line backend is built for macOS/Unix with the `serial` feature; on Windows use --j2534".into())
}

fn connect(o: &Opts) -> Result<Client<Box<dyn Link>>, String> {
    let mut c = Client::with_addresses(open_link(o)?, o.system.address(), 0xf1);
    c.length_byte = o.system.length_byte();
    let kb = c.start().map_err(|e| format!("start: {e}"))?;
    eprintln!("{:?} session up, key bytes {}", o.system, hex(&kb));
    Ok(c)
}

/// Adapter self-test (no car session; see `cascade_diag::serial::selftest`).
#[cfg(all(feature = "serial", unix))]
fn selftest(port: &str) -> Result<(), String> {
    use cascade_diag::serial::{SerialLine, Status, selftest};
    let mut line = SerialLine::open(port, true).map_err(|e| {
        let ports = cascade_diag::serial::list_ports();
        let list = if ports.is_empty() {
            "none found".to_string()
        } else {
            ports.join(", ")
        };
        format!("{port}: {e}\nadapter ports: {list}")
    })?;
    println!("Adapter self-test on {port}");
    println!("Needs 12 V on OBD pin 16 (car with ignition OFF, or a bench supply).\n");
    let report = selftest(&mut line);
    for c in &report {
        let tag = match c.status {
            Status::Pass => "PASS",
            Status::Warn => "WARN",
            Status::Fail => "FAIL",
            Status::Info => "INFO",
        };
        println!("{tag}  {:<15} {}", c.name, c.detail);
    }
    if report.iter().any(|c| c.status == Status::Fail) {
        return Err("adapter self-test failed".into());
    }
    Ok(())
}

#[cfg(not(all(feature = "serial", unix)))]
fn selftest(_port: &str) -> Result<(), String> {
    Err("the serial K-line backend is built for macOS/Unix with the `serial` feature; on Windows use --j2534".into())
}

/// Clear stored fault codes, the way the Hi-Scan's ERAS key does.
fn clear(o: &Opts) -> Result<(), String> {
    let group = match o.system {
        System::Engine => 0xff00, // Hi-Scan sends 14 FF 00
        System::Abs => 0x4000,    // Hi-Scan sends 14 40 00
        System::Srs => {
            return Err("clearing SRS codes is not supported (request not observed)".into());
        }
    };
    if !o.allow_clear {
        return Err("clearing fault codes changes the ECU; rerun with --allow-clear".into());
    }
    if !o.yes {
        eprint!("Clear all stored {:?} fault codes? Type yes: ", o.system);
        let mut answer = String::new();
        std::io::stdin()
            .read_line(&mut answer)
            .map_err(|e| e.to_string())?;
        if answer.trim() != "yes" {
            return Err("not confirmed; nothing sent".into());
        }
    }
    let mut c = connect(o)?;
    c.allow_clear = true;
    c.clear_dtcs(group).map_err(|e| format!("clear: {e}"))?;
    println!("fault codes cleared");
    let _ = c.stop();
    Ok(())
}

fn dtc(o: &Opts) -> Result<(), String> {
    let mut c = connect(o)?;
    let list: Vec<String> = match o.system {
        System::Engine => c
            .read_dtcs()
            .map_err(|e| format!("DTCs: {e}"))?
            .into_iter()
            .map(|(code, st)| format!("{} status {st:02X}", dtc_name(code)))
            .collect(),
        System::Abs => c
            .read_dtcs_13(0x4000)
            .map_err(|e| format!("DTCs: {e}"))?
            .into_iter()
            .map(dtc_name)
            .collect(),
        System::Srs => {
            // The Hi-Scan asks twice; it marks codes from filter 01 as
            // history (H), whatever their status byte (emulator-derived).
            let mut v = Vec::new();
            for (status, kind) in [(0x00, "current"), (0x01, "history")] {
                for (code, st) in c
                    .read_dtcs_by_status(status, 0xff00)
                    .map_err(|e| format!("DTCs: {e}"))?
                {
                    v.push(format!("{} {kind}, status {st:02X}", dtc_name(code)));
                }
            }
            v
        }
    };
    if list.is_empty() {
        println!("no stored DTCs");
    }
    for l in list {
        println!("{l}");
    }
    let _ = c.stop();
    Ok(())
}

fn log(o: &Opts) -> Result<(), String> {
    let params = defs::parse_csv(o.system.data_defs()?)?;
    let mut c = connect(o)?;
    let mut w: Box<dyn Write> = match &o.out {
        Some(p) => Box::new(BufWriter::new(
            File::create(p).map_err(|e| format!("{p}: {e}"))?,
        )),
        None => Box::new(std::io::stdout().lock()),
    };
    let header: Vec<String> = params
        .iter()
        .map(|p| format!("{}[{}]", p.name, p.unit))
        .collect();
    writeln!(w, "t_s,{},raw", header.join(",")).map_err(|e| e.to_string())?;
    let start = c.line.elapsed();
    let mut errors = 0;
    loop {
        let t = c.line.elapsed() - start;
        if o.duration.is_some_and(|d| t >= d) {
            break;
        }
        match c.read_local(0x01) {
            Ok(block) => {
                errors = 0;
                let cols: Vec<String> = params
                    .iter()
                    .map(|p| match p.decode(&block) {
                        Some(Value::Number(v)) => format!("{v:.3}"),
                        Some(Value::Flag(f)) => (f as u8).to_string(),
                        None => String::new(),
                    })
                    .collect();
                let raw: String = block.iter().map(|b| format!("{b:02X}")).collect();
                writeln!(w, "{:.3},{},{raw}", t.as_secs_f64(), cols.join(","))
                    .map_err(|e| e.to_string())?;
            }
            Err(e) => {
                errors += 1;
                eprintln!("{:.3}s: {e}", t.as_secs_f64());
                if errors >= 3 {
                    eprintln!("re-initialising session");
                    c.start().map_err(|e| format!("restart: {e}"))?;
                    errors = 0;
                }
            }
        }
        let spent = c.line.elapsed() - start - t;
        if spent < o.interval {
            c.line.idle(o.interval - spent);
        }
    }
    w.flush().map_err(|e| e.to_string())?;
    let _ = c.stop();
    Ok(())
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let r = match args.split_first() {
        Some((cmd, rest)) if cmd == "dtc" => parse(rest).and_then(|o| dtc(&o)),
        Some((cmd, rest)) if cmd == "log" => parse(rest).and_then(|o| log(&o)),
        Some((cmd, rest)) if cmd == "clear" => parse(rest).and_then(|o| clear(&o)),
        Some((cmd, _)) if cmd == "j2534-list" => j2534_list(),
        Some((cmd, rest)) if cmd == "selftest" => match rest {
            [flag, dev] if flag == "--port" => selftest(dev),
            _ => Err("usage: kdiag selftest --port DEV".into()),
        },
        _ => Err("usage: kdiag dtc|log|clear|selftest|j2534-list (--sim [--sim-ecu ROM] [--j2534-mock] | --port DEV | --j2534 DLL|auto) [--system engine|abs|srs] [--out F] \
                  [--trace F.jsonl] [--interval-ms N] [--duration-s N] [--no-echo] [--wake break|zero]"
            .into()),
    };
    match r {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("kdiag: {e}");
            ExitCode::from(2)
        }
    }
}
