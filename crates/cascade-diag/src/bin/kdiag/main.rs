//! kdiag: K-line diagnostics for the 2002 Hyundai Coupe (Tiburon GK).
//!
//!   kdiag dtc  (--sim | --port DEV) [--system engine|abs|srs|tcs] [line options]
//!   kdiag selftest --port DEV   (adapter checks, no ECU session)
//!   kdiag ident (--sim | --port DEV) [--system engine|abs|srs]
//!   kdiag crash (--sim | --port DEV) --system srs   (crash records, raw)
//!   kdiag elm  (--sim | --port DEV | --ecusim auto) [--listen 127.0.0.1:35000]
//!              ELM327 over TCP for OBD apps, through the read-only guard
//!   kdiag clear (--sim | --port DEV) [--system engine|abs] --allow-clear [--yes]
//!   kdiag log  (--sim | --port DEV) [--system engine|abs|tcs] [--out FILE.csv]
//!              [--interval-ms N] [--duration-s N] [line options]
//!
//! `kdiag help COMMAND` lists every option with its default.
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
//!   tcs     0x32 on OBD pin 8 (traction control, 2003-): 5-baud init and
//!           ISO 9141-2 frames; serial or --sim only, not J2534
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
//! CAN (OBD-II on ISO 15765-4 cars; the Tiburon GK itself has no CAN
//! diagnostics): --can IFACE uses a Linux SocketCAN interface (`can0`,
//! `vcan0`; bring it up first with `ip link`), --can sim an in-memory bus
//! with a simulated OBD-II ECU (any OS). dtc, ident, log, clear and elm then
//! use OBD-II modes 03/07, 09, 01 and 04. --can-ids TX:RX (default 7E0:7E8;
//! 7DF:7E8 for a functional request), --can-pad XX|none (default AA).
//!   kdiag can-ecu --can IFACE [--sim | --port DEV | --ecusim auto]
//!              answer on CAN as an engine ECU: the simulated one, or a
//!              K-line ECU behind the read-only guard (a CAN gateway)
//!
//! --ecusim SOCKET|auto talks to `ecusim serve` (the ECU dump running in
//! real time as its own process; `auto` = its default socket), in wall time.
//! --sim-ecu ROM runs the engine ECU's own code instead (a SIMK41 dump, in the
//! `ecuc167` emulator, virtual time; engine only). Sensors read mid-scale.
//! `log` writes one CSV row per `21 01` response: time, decoded values from
//! the embedded protocol definitions, and the raw block in hex, so logs can
//! be re-decoded if the definitions change.

use std::fs::File;
use std::io::{BufWriter, Write};
use std::process::ExitCode;

use cascade_diag::defs::{self, Value};
use cascade_diag::j2534::{J2534Link, MockPassThru};
use cascade_diag::kwp::{Client, Link, dtc_name, hex};
use cascade_diag::line::Line;
use cascade_diag::sim::SimLine;
use cascade_diag::trace::{TracedLine, TracedLink};
use cascade_diag::{guard, iso9141};
use cascade_iface::{KwpConfig, KwpEcu};
use clap::Parser;

mod cli;
use cli::{Cli, Cmd, Opts, System, check};

impl System {
    fn address(self) -> u8 {
        match self {
            System::Engine => 0x11,
            System::Abs => 0x28,
            System::Srs => 0x58,
            System::Tcs => 0x32,
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
            System::Tcs => Err("traction control data is read by tcs_log".into()),
        }
    }
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
            // what the GK-20 SIMK41 code answers (ecuc167)
            cfg.ecu_ident.insert(0x8e, b"G2T0MK2B----".to_vec());
            cfg.local_data
                .insert(0x01, cascade_diag::sim::engine_idle_block());
        }
        System::Abs => {
            cfg.key_bytes = [0xea, 0x8f];
            cfg.length_byte = true;
            cfg.local_data.insert(0x01, sim_abs_block());
            cfg.responses
                .insert(vec![0x13, 0x40, 0x00], vec![0x53, 0x01, 0x52, 0x00]); // C1200
        }
        System::Srs => {
            // an airbag module with one current and one stored code
            cfg.key_bytes = [0xea, 0x8f];
            cfg.length_byte = true;
            cfg.responses.insert(
                vec![0x18, 0x00, 0xff, 0x00],
                vec![0x58, 0x01, 0x93, 0x46, 0x71], // B1346 DAB resistance high
            );
            cfg.responses.insert(
                vec![0x18, 0x01, 0xff, 0x00],
                vec![0x58, 0x01, 0x91, 0x02, 0x60], // B1102 battery voltage low
            );
            // laid out as the Hi-Scan reads it: vehicle 1-6 (from the option
            // byte on), part no 7-17, part name 18-37, version at 68/69
            let mut id = vec![0x5a, 0x80];
            id.extend(b"GK-03");
            id.extend(b"95910-2C000");
            id.extend(b"SRSCM (SIMULATED)   ");
            id.resize(68, b' ');
            id.extend(b"12");
            cfg.responses.insert(vec![0x1a, 0x80], id);
            let mut crash = vec![0x61, 0x00];
            crash.extend([0u8; 40]);
            cfg.responses.insert(vec![0x21, 0x00, 0x01], crash.clone());
            cfg.responses.insert(vec![0x21, 0x00], crash);
        }
        System::Tcs => {
            // standing still, one stored code, valves off
            cfg.iso9141 = true;
            cfg.responses.insert(vec![0xb1, 0x01], vec![0xf1, 0x01]);
            cfg.responses.insert(
                vec![0x03],
                vec![0x43, 0x52, 0x00, 0x00, 0x00, 0x00, 0x00], // C1200
            );
            cfg.responses
                .insert(vec![0xb1, 0x0e], vec![0xf1, 0x0e, 0, 0, 0, 0]);
            cfg.responses.insert(vec![0xb1, 0x2c], vec![0xf1, 0x2c, 0]);
        }
    }
    cfg
}

/// Traction control session: 5-baud init, ISO 9141-2 frames, read-only
/// allowlist of its own (`guard::check_tcs`).
fn connect_tcs(o: &Opts) -> Result<iso9141::Client<Box<dyn Line>>, String> {
    let mut c = iso9141::Client::new(open_line(o)?, System::Tcs.address(), guard::check_tcs);
    let kb = c.start().map_err(|e| format!("5-baud init: {e}"))?;
    // the Hi-Scan's keep-alive, also its first request; recorded so a
    // replay can answer it
    c.keep_alive()
        .map_err(|e| format!("keep-alive B1 01: {e}"))?;
    eprintln!("Tcs session up, key bytes {}", hex(&kb));
    Ok(c)
}

fn tcs_dtc(o: &Opts) -> Result<(), String> {
    let mut c = connect_tcs(o)?;
    let names = defs::dtc_texts(defs::TCS_DTC_NAMES);
    let codes = c.read_dtcs().map_err(|e| format!("DTCs: {e}"))?;
    if codes.is_empty() {
        println!("no stored DTCs");
    }
    for code in codes {
        let name = dtc_name(code);
        let text = names
            .get(&name)
            .map_or("(not in the Hi-Scan's list)", String::as_str);
        println!("{name} {text}");
    }
    Ok(())
}

/// Wheel speeds (`B1 0E`) and valve outputs (`B1 2C`), one CSV row per pair.
fn tcs_log(o: &Opts) -> Result<(), String> {
    let speeds = defs::parse_csv(defs::TIBURON_2003_TCS_WHEEL_SPEEDS)?;
    let valves = defs::parse_csv(defs::TIBURON_2003_TCS_VALVES)?;
    let mut c = connect_tcs(o)?;
    let mut w: Box<dyn Write> = match &o.out {
        Some(p) => Box::new(BufWriter::new(
            File::create(p).map_err(|e| format!("{p}: {e}"))?,
        )),
        None => Box::new(std::io::stdout().lock()),
    };
    let header: Vec<String> = speeds
        .iter()
        .chain(&valves)
        .map(|p| format!("{}[{}]", p.name, p.unit))
        .collect();
    writeln!(w, "t_s,{},raw_b10e,raw_b12c", header.join(",")).map_err(|e| e.to_string())?;
    let start = Line::elapsed(&c.line);
    let mut errors = 0;
    loop {
        let t = Line::elapsed(&c.line) - start;
        if o.duration.is_some_and(|d| t >= d) {
            break;
        }
        let read = |c: &mut iso9141::Client<Box<dyn Line>>, req: &[u8]| {
            c.request(req)
                .map(|f| f.into_iter().next().unwrap_or_default().split_off(2))
        };
        match read(&mut c, &[0xb1, 0x0e]).and_then(|a| Ok((a, read(&mut c, &[0xb1, 0x2c])?))) {
            Ok((a, b)) => {
                errors = 0;
                let cols: Vec<String> = speeds
                    .iter()
                    .map(|p| p.decode(&a))
                    .chain(valves.iter().map(|p| p.decode(&b)))
                    .map(|v| match v {
                        Some(Value::Number(v)) => format!("{v:.3}"),
                        Some(Value::Flag(f)) => (f as u8).to_string(),
                        None => String::new(),
                    })
                    .collect();
                let raw = |x: &[u8]| x.iter().map(|b| format!("{b:02X}")).collect::<String>();
                writeln!(
                    w,
                    "{:.3},{},{},{}",
                    t.as_secs_f64(),
                    cols.join(","),
                    raw(&a),
                    raw(&b)
                )
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
        let spent = Line::elapsed(&c.line) - start - t;
        if spent < o.interval {
            Line::idle(&mut c.line, o.interval - spent);
        }
    }
    w.flush().map_err(|e| e.to_string())?;
    Ok(())
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
    if let Some(iface) = &o.can {
        return traced(Box::new(cascade_diag::can::CanLink::new(open_isotp(
            o, iface,
        )?)));
    }
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
    } else if let Some(sock) = &o.ecusim {
        // the Hi-Scan's line numbers: engine 7, ABS and TCS 8, SRS 12
        let line = match o.system {
            System::Engine => 7,
            System::Abs | System::Tcs => 8,
            System::Srs => 12,
        };
        open_ecusim(sock, line)?
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
fn open_ecusim(sock: &str, line: i32) -> Result<Box<dyn Line>, String> {
    let path = if sock == "auto" {
        ecusim::default_socket()
    } else {
        sock.into()
    };
    let l = cascade_diag::ecusim_line::EcusimLine::connect(&path, line)
        .map_err(|e| format!("{}: {e} (is `ecusim serve` running?)", path.display()))?;
    Ok(Box::new(l))
}

#[cfg(not(all(feature = "serial", unix)))]
fn open_ecusim(_sock: &str, _line: i32) -> Result<Box<dyn Line>, String> {
    Err("--ecusim needs a Unix build with the `serial` feature".into())
}

#[cfg(all(feature = "serial", unix))]
fn open_serial(o: &Opts) -> Result<Box<dyn Line>, String> {
    use cascade_diag::serial::{SerialLine, WakeMethod};
    let port = o.port.as_deref().unwrap_or_default();
    let mut l = SerialLine::open(port, o.echo).map_err(|e| format!("{port}: {e}"))?;
    l.wake = match o.wake {
        cli::Wake::Break => WakeMethod::Break,
        cli::Wake::Zero => WakeMethod::ZeroByte,
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
    if o.can.is_some() {
        return can_clear(o);
    }
    let group = match o.system {
        System::Engine => 0xff00, // Hi-Scan sends 14 FF 00
        System::Abs => 0x4000,    // Hi-Scan sends 14 40 00
        System::Srs => {
            return Err("clearing SRS codes is not supported (request not observed)".into());
        }
        System::Tcs => {
            return Err("clearing traction control codes is not supported".into());
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
    if o.can.is_some() {
        return can_dtc(o);
    }
    if o.system == System::Tcs {
        return tcs_dtc(o);
    }
    let mut c = connect(o)?;
    // the texts the Hi-Scan shows for this system's codes
    let names = defs::dtc_texts(match o.system {
        System::Engine => defs::ENGINE_DTC_NAMES,
        System::Abs => defs::ABS_DTC_NAMES,
        System::Srs => defs::SRS_DTC_NAMES,
        System::Tcs => unreachable!("handled by tcs_dtc"),
    });
    let named = |code: u16| -> String {
        let name = dtc_name(code);
        let text = names
            .get(&name)
            .map_or("(not in the Hi-Scan's list)", String::as_str);
        format!("{name} {text}")
    };
    let list: Vec<String> = match o.system {
        System::Engine => c
            .read_dtcs()
            .map_err(|e| format!("DTCs: {e}"))?
            .into_iter()
            .map(|(code, st)| format!("{} (status {st:02X})", named(code)))
            .collect(),
        System::Abs => c
            .read_dtcs_13(0x4000)
            .map_err(|e| format!("DTCs: {e}"))?
            .into_iter()
            .map(named)
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
                    v.push(format!("{} ({kind}, status {st:02X})", named(code)));
                }
            }
            v
        }
        System::Tcs => unreachable!("handled by tcs_dtc"),
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

/// ECU identification as the Hi-Scan asks for it.
fn ident(o: &Opts) -> Result<(), String> {
    if o.can.is_some() {
        return can_ident(o);
    }
    if o.system == System::Tcs {
        return Err("the Hi-Scan has no identification screen for traction control".into());
    }
    let mut c = connect(o)?;
    let printable = |b: &[u8]| -> String {
        b.iter()
            .map(|&x| {
                if (0x20..0x7f).contains(&x) {
                    x as char
                } else {
                    '.'
                }
            })
            .collect()
    };
    match o.system {
        System::Engine => {
            // the Hi-Scan's Identification Check: 1A 8E, shown as text
            let r = c
                .request(&[0x1a, 0x8e])
                .map_err(|e| format!("1A 8E: {e}"))?;
            println!("identification: {}", printable(r.get(2..).unwrap_or(&[])));
            println!("raw: {}", hex(&r));
        }
        System::Srs => {
            let r = c
                .request(&[0x1a, 0x80])
                .map_err(|e| format!("1A 80: {e}"))?;
            let id = defs::parse_srs_ident(&r);
            println!("vehicle:     {}", id.vehicle);
            println!("part no:     {}", id.part_no);
            println!("part name:   {}", id.part_name);
            println!(
                "ACU version: {}",
                id.acu_version.as_deref().unwrap_or("(reply too short)")
            );
            println!("raw: {}", hex(&r));
            println!("(layout as the Hi-Scan shows it; unverified on a real module)");
        }
        System::Abs => {
            return Err("the ABS identification request has not been observed yet".into());
        }
        System::Tcs => unreachable!("refused above"),
    }
    let _ = c.stop();
    Ok(())
}

/// SRS crash records (Hi-Scan "Crashing Information"): read-only, printed raw
/// because the record layout is not known yet.
fn crash(o: &Opts) -> Result<(), String> {
    if o.can.is_some() {
        return Err(
            "crash records are read from the Tiburon's SRS on the K line, not over CAN".into(),
        );
    }
    if o.system != System::Srs {
        return Err("crash records are an SRS function: use --system srs".into());
    }
    let mut c = connect(o)?;
    // the Hi-Scan sends 21 00 01 first, then polls 21 00
    for req in [&[0x21, 0x00, 0x01][..], &[0x21, 0x00]] {
        match c.request(req) {
            Ok(r) => println!("{} -> {}", hex(req), hex(&r)),
            Err(e) => println!("{} -> {e}", hex(req)),
        }
    }
    println!("(record layout unknown: keep this output and the --trace file)");
    let _ = c.stop();
    Ok(())
}

fn log(o: &Opts) -> Result<(), String> {
    if o.can.is_some() {
        return can_log(o);
    }
    if o.system == System::Tcs {
        return tcs_log(o);
    }
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

/// Serve ELM327 clients (OBD apps) on `--listen`, through the read-only
/// guard (`--allow-clear` lets mode 04 through).
fn elm(o: &Opts) -> Result<(), String> {
    if o.system != System::Engine {
        return Err("elm talks to the engine ECU (KWP2000)".into());
    }
    if !o.listen.starts_with("127.") && !o.listen.starts_with("localhost") {
        eprintln!("kdiag: elm listens on {} (not only this machine)", o.listen);
    }
    let mut c = Client::new(open_link(o)?);
    c.allow_clear = o.allow_clear;
    let sock = o.ecusim.clone();
    let battery = move || -> Option<f64> { ecusim_battery(sock.as_deref()?) };
    let proto = match o.can {
        Some(_) => cascade_diag::elm::Proto::Can { rx_id: o.can_ids.1 },
        None => cascade_diag::elm::Proto::Kwp,
    };
    cascade_diag::elm::serve(&mut c, &battery, proto, &o.listen).map_err(|e| e.to_string())
}

/// Battery voltage from an ecusim server's status (for ATRV).
#[cfg(all(feature = "serial", unix))]
fn ecusim_battery(sock: &str) -> Option<f64> {
    use ecusim::proto::{ToClient, ToServer};
    let path = if sock == "auto" {
        ecusim::default_socket()
    } else {
        sock.into()
    };
    let mut c = ecusim::client::Conn::connect(&path).ok()?;
    c.send(&ToServer::Watch).ok()?;
    match c
        .recv_timeout(std::time::Duration::from_millis(300))
        .ok()??
    {
        ToClient::Status(kv) => kv.iter().find(|(k, _)| k == "battery")?.1.parse().ok(),
        _ => None,
    }
}

#[cfg(not(all(feature = "serial", unix)))]
fn ecusim_battery(_sock: &str) -> Option<f64> {
    None
}

type Bus = Box<dyn cascade_can::CanBus + Send>;

/// The CAN bus for `--can`: a SocketCAN interface, or with `sim` an
/// in-memory bus with a simulated OBD-II engine ECU on it.
fn open_bus(iface: &str) -> Result<Bus, String> {
    if iface == "sim" {
        use cascade_can::ecu::{self, OBD_ENGINE_REQUEST, OBD_ENGINE_RESPONSE, ObdSim};
        let hub = cascade_can::vbus::Hub::new();
        let mut tp =
            cascade_can::isotp::IsoTp::new(hub.endpoint(), OBD_ENGINE_RESPONSE, OBD_ENGINE_REQUEST);
        tp.functional_id = Some(ecu::OBD_FUNCTIONAL);
        // lives as long as kdiag
        std::thread::spawn(move || ecu::serve(&mut tp, &mut ObdSim::default(), &|| true));
        return Ok(Box::new(hub.endpoint()));
    }
    open_socketcan(iface)
}

#[cfg(target_os = "linux")]
fn open_socketcan(iface: &str) -> Result<Bus, String> {
    cascade_can::socketcan::SocketCan::open(iface)
        .map(|b| Box::new(b) as Bus)
        .map_err(|e| format!("{iface}: {e}"))
}

#[cfg(not(target_os = "linux"))]
fn open_socketcan(iface: &str) -> Result<Bus, String> {
    Err(format!(
        "--can {iface}: SocketCAN needs Linux; use --can sim to try the CAN path here"
    ))
}

fn open_isotp(o: &Opts, iface: &str) -> Result<cascade_can::isotp::IsoTp<Bus>, String> {
    let (tx, rx) = o.can_ids;
    let mut tp = cascade_can::isotp::IsoTp::new(open_bus(iface)?, tx, rx);
    tp.pad = o.can_pad;
    // after a functional request, flow control goes to the ECU's own id
    tp.fc_id = cascade_can::isotp::physical_request_id(tx, rx);
    Ok(tp)
}

fn can_session(o: &Opts) -> Result<Client<Box<dyn Link>>, String> {
    let mut c = Client::new(open_link(o)?);
    c.start().map_err(|e| format!("start: {e}"))?;
    let (tx, rx) = o.can_ids;
    eprintln!(
        "CAN on {}: requests {tx:03X}, answers {rx:03X}",
        o.can.as_deref().unwrap_or("?")
    );
    Ok(c)
}

/// OBD-II stored (mode 03) and pending (mode 07) codes.
fn can_dtc(o: &Opts) -> Result<(), String> {
    use cascade_diag::can::obd_codes;
    let mut c = can_session(o)?;
    let stored = c.request(&[0x03]).map_err(|e| format!("mode 03: {e}"))?;
    let stored = obd_codes(&stored);
    let pending = match c.request(&[0x07]) {
        Ok(r) => obd_codes(&r),
        Err(e) => {
            eprintln!("mode 07 (pending codes): {e}");
            Vec::new()
        }
    };
    if stored.is_empty() && pending.is_empty() {
        println!("no stored or pending DTCs");
    }
    for code in stored {
        println!("{} (stored)", dtc_name(code));
    }
    for code in pending {
        println!("{} (pending)", dtc_name(code));
    }
    Ok(())
}

/// OBD-II vehicle information: VIN (09 02) and ECU name (09 0A).
fn can_ident(o: &Opts) -> Result<(), String> {
    let mut c = can_session(o)?;
    let text = |b: &[u8]| -> String {
        b.iter()
            .filter(|&&x| x != 0)
            .map(|&x| {
                if (0x20..0x7f).contains(&x) {
                    x as char
                } else {
                    '.'
                }
            })
            .collect()
    };
    for (pid, name) in [(0x02u8, "VIN"), (0x0a, "ECU name")] {
        match c.request(&[0x09, pid]) {
            // 49 pid count data..
            Ok(r) => println!(
                "{name:9} {}   (raw {})",
                text(r.get(3..).unwrap_or(&[])),
                hex(&r)
            ),
            Err(e) => println!("{name:9} ({e})"),
        }
    }
    Ok(())
}

/// OBD-II live data (mode 01) as CSV, one PID per request.
fn can_log(o: &Opts) -> Result<(), String> {
    use cascade_diag::can::{LOG_PIDS, obd_value, pid_supported};
    let mut c = can_session(o)?;
    let mut maps = Vec::new();
    for base in [0x00u8, 0x20, 0x40] {
        match c.request(&[0x01, base]) {
            Ok(r) => maps.push(r),
            Err(_) => break,
        }
        if !maps.last().is_some_and(|r| pid_supported(r, base + 0x20)) {
            break;
        }
    }
    let pids: Vec<_> = LOG_PIDS
        .iter()
        .filter(|(p, ..)| maps.iter().any(|m| pid_supported(m, *p)))
        .collect();
    if pids.is_empty() {
        return Err("the ECU lists none of the logged PIDs as supported".into());
    }
    let mut w: Box<dyn Write> = match &o.out {
        Some(p) => Box::new(BufWriter::new(
            File::create(p).map_err(|e| format!("{p}: {e}"))?,
        )),
        None => Box::new(std::io::stdout().lock()),
    };
    let header: Vec<String> = pids.iter().map(|(_, n, u)| format!("{n}[{u}]")).collect();
    writeln!(w, "t_s,{}", header.join(",")).map_err(|e| e.to_string())?;
    let start = c.line.elapsed();
    loop {
        let t = c.line.elapsed() - start;
        if o.duration.is_some_and(|d| t >= d) {
            break;
        }
        let cols: Vec<String> = pids
            .iter()
            .map(|(p, ..)| match c.request(&[0x01, *p]) {
                Ok(r) if r.get(1) == Some(p) => {
                    obd_value(*p, &r[2..]).map_or(String::new(), |v| format!("{v:.2}"))
                }
                _ => String::new(),
            })
            .collect();
        writeln!(w, "{:.3},{}", t.as_secs_f64(), cols.join(",")).map_err(|e| e.to_string())?;
        let spent = c.line.elapsed() - start - t;
        if spent < o.interval {
            c.line.idle(o.interval - spent);
        }
    }
    w.flush().map_err(|e| e.to_string())
}

/// OBD-II mode 04: clears codes and freeze frames, resets readiness.
fn can_clear(o: &Opts) -> Result<(), String> {
    if !o.allow_clear {
        return Err("clearing fault codes changes the ECU; rerun with --allow-clear".into());
    }
    if !o.yes {
        eprint!("Clear all OBD-II codes (also resets readiness monitors)? Type yes: ");
        let mut answer = String::new();
        std::io::stdin()
            .read_line(&mut answer)
            .map_err(|e| e.to_string())?;
        if answer.trim() != "yes" {
            return Err("not confirmed; nothing sent".into());
        }
    }
    let mut c = can_session(o)?;
    c.allow_clear = true;
    c.request(&[0x04]).map_err(|e| format!("mode 04: {e}"))?;
    println!("fault codes cleared");
    Ok(())
}

/// Answer on a CAN bus as the engine ECU (ISO 15765-4 ids, or --can-ids
/// swapped: requests on TX, answers on RX): the simulated OBD-II ECU, or a
/// K-line ECU through the read-only guard.
fn can_ecu(o: &Opts) -> Result<(), String> {
    use cascade_can::ecu::{self, ObdSim};
    let iface = o.can.as_deref().unwrap_or_default();
    if iface == "sim" {
        return Err(
            "can-ecu needs a real interface (vcan0, can0); --can sim already has one".into(),
        );
    }
    let (req, resp) = o.can_ids;
    let mut tp = cascade_can::isotp::IsoTp::new(open_bus(iface)?, resp, req);
    tp.pad = o.can_pad;
    tp.functional_id = Some(ecu::OBD_FUNCTIONAL);
    let kline = o.sim || o.port.is_some() || o.ecusim.is_some();
    eprintln!(
        "can-ecu on {iface}: requests {req:03X} and {:03X}, answers {resp:03X}, {}",
        ecu::OBD_FUNCTIONAL,
        if kline {
            "bridged to the K-line ECU (read-only guard on)"
        } else {
            "simulated OBD-II ECU"
        }
    );
    if kline {
        let inner = Opts {
            can: None,
            ..o.clone()
        };
        let mut c = Client::new(open_link(&inner)?);
        c.allow_clear = o.allow_clear;
        let mut bridge = cascade_diag::can::KlineBridge::new(&mut c);
        ecu::serve(&mut tp, &mut bridge, &|| true).map_err(|e| e.to_string())
    } else {
        ecu::serve(&mut tp, &mut ObdSim::default(), &|| true).map_err(|e| e.to_string())
    }
}

fn main() -> ExitCode {
    let r = match Cli::parse().cmd {
        Cmd::Dtc(o) => check(o, false).and_then(|o| dtc(&o)),
        Cmd::Log(o) => check(o, false).and_then(|o| log(&o)),
        Cmd::Clear(o) => check(o, false).and_then(|o| clear(&o)),
        Cmd::Ident(o) => check(o, false).and_then(|o| ident(&o)),
        Cmd::Crash(o) => check(o, false).and_then(|o| crash(&o)),
        Cmd::Elm(o) => check(o, false).and_then(|o| elm(&o)),
        Cmd::CanEcu(o) => check(o, true).and_then(|o| can_ecu(&o)),
        Cmd::J2534List => j2534_list(),
        Cmd::Selftest { port } => selftest(&port),
    };
    match r {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("kdiag: {e}");
            ExitCode::from(2)
        }
    }
}
