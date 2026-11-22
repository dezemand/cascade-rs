//! ecusim: run a GK 2.0 SIMK41 engine ECU dump as its own process.
//!
//! ```text
//! ecusim serve --rom ROM [--socket PATH]    run the ECU (Ctrl-C to stop)
//! ecusim set NAME VALUE [NAME VALUE]...     change controls, e.g.
//!                                           ecusim set ignition on starter on
//! ecusim status [--follow]                  print the state line(s)
//! ecusim play SCENARIO                      run a scenario file (below)
//! ecusim can [-n N] [ID HEX]                show the engine CAN bus (and
//!                                           send one frame first)
//! ```
//!
//! Controls: ignition on|off, starter on|off, throttle 0-100 (%), coolant and
//! intake (°C), battery (V, engine off), speed (km/h), idle (rpm), rpm (fixed
//! engine speed; 0 = engine model), battery_connected on|off (off wipes the
//! ECU's memory), fault.NAME on|off (wiring faults: map_short, map_open,
//! coolant_short, coolant_open, throttle_short, throttle_open, o2_front_open,
//! o2_rear_open, evap_short, evap_open, battery_low, wheel_speed_circuit,
//! cam_missing, can_gearbox, can_esp, can_bus (CAN nodes silent / none),
//! abs_wheel_sensor_fl, abs_pump_motor, srs_driver_airbag,
//! srs_passenger_airbag, oil_temp_short, oil_temp_open; value on, off or
//! intermittent), time 1-10 (time factor while no tester is connected),
//! snapshot NAME / restore NAME (whole simulator state, in memory).
//! The socket defaults to `$TMPDIR/ecusim.sock`.
//!
//! A scenario file has one step per line, `SECONDS NAME VALUE` (seconds
//! from the start of the scenario, wall time), `#` comments; see
//! `scenarios/`.
//! Testers connect to the same socket: the Hi-Scan GUI's line option "ECU
//! process", `kdiag --ecusim`.

use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::time::Duration;

use ecusim::client::Conn;
use ecusim::proto::{ToClient, ToServer};

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut socket = ecusim::default_socket();
    let mut rom = None;
    let mut follow = false;
    let mut rest = Vec::new();
    let mut it = args.iter().skip(1);
    while let Some(a) = it.next() {
        match a.as_str() {
            "--socket" => socket = it.next().map(PathBuf::from).unwrap_or(socket),
            "--rom" => rom = it.next().map(PathBuf::from),
            "--follow" => follow = true,
            _ => rest.push(a.clone()),
        }
    }
    let r = match args.first().map(String::as_str) {
        Some("serve") => serve(rom, &socket),
        Some("set") => set(&socket, &rest),
        Some("status") => status(&socket, follow),
        Some("can") => can(&socket, &rest),
        Some("play") => match rest.first() {
            Some(f) => play(&socket, f),
            None => Err("play SCENARIO_FILE".into()),
        },
        _ => Err(
            "usage: ecusim serve --rom ROM [--socket P] | set NAME VALUE... | status [--follow]"
                .into(),
        ),
    };
    match r {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("ecusim: {e}");
            ExitCode::from(2)
        }
    }
}

fn serve(rom: Option<PathBuf>, socket: &Path) -> Result<(), String> {
    let rom = rom.ok_or("serve needs --rom ROM (a SIMK41 dump without immobiliser)")?;
    let data = ecusim::server::load_rom(&rom).map_err(|e| e.to_string())?;
    eprintln!("ecusim: {} on {}", rom.display(), socket.display());
    // a fault profile beside the dump (tools/fault_sweep.py --csv), if any
    let mut profile_path = rom.clone().into_os_string();
    profile_path.push(".faults.csv");
    let profile = std::fs::read_to_string(&profile_path).unwrap_or_default();
    if !profile.is_empty() {
        eprintln!(
            "ecusim: fault profile {}",
            std::path::Path::new(&profile_path).display()
        );
    }
    ecusim::server::Server::new(data)
        .with_profile(&profile)
        .serve(socket)
        .map_err(|e| format!("{}: {e}", socket.display()))
}

fn connect(socket: &Path) -> Result<Conn, String> {
    Conn::connect(socket)
        .map_err(|e| format!("{}: {e} (is `ecusim serve` running?)", socket.display()))
}

fn set(socket: &Path, pairs: &[String]) -> Result<(), String> {
    if pairs.is_empty() || !pairs.len().is_multiple_of(2) {
        return Err("set NAME VALUE [NAME VALUE]...".into());
    }
    let mut c = connect(socket)?;
    for p in pairs.chunks(2) {
        c.send(&ToServer::Set {
            name: p[0].clone(),
            value: p[1].clone(),
        })
        .map_err(|e| e.to_string())?;
    }
    // errors come back asynchronously
    while let Some(m) = c
        .recv_timeout(Duration::from_millis(150))
        .map_err(|e| e.to_string())?
    {
        if let ToClient::Err(e) = m {
            return Err(e);
        }
    }
    Ok(())
}

fn status(socket: &Path, follow: bool) -> Result<(), String> {
    let mut c = connect(socket)?;
    c.send(&ToServer::Watch).map_err(|e| e.to_string())?;
    loop {
        match c
            .recv_timeout(Duration::from_secs(2))
            .map_err(|e| e.to_string())?
        {
            Some(m @ ToClient::Status(_)) => {
                println!("{}", m.line().trim_start_matches("STATUS "));
                if !follow {
                    return Ok(());
                }
            }
            Some(_) => {}
            None => return Err("no status from the server".into()),
        }
    }
}

/// Print the engine CAN bus traffic, candump style; optionally send one
/// frame first. `-n N` stops after N frames.
fn can(socket: &Path, args: &[String]) -> Result<(), String> {
    let mut limit = None;
    let mut send = Vec::new();
    let mut it = args.iter();
    while let Some(a) = it.next() {
        if a == "-n" {
            limit = Some(
                it.next()
                    .ok_or("-n N")?
                    .parse::<usize>()
                    .map_err(|e| e.to_string())?,
            );
        } else {
            send.push(a.as_str());
        }
    }
    let mut c = connect(socket)?;
    let first = match send[..] {
        [] => ToServer::CanSub,
        [id, data] => ToServer::parse(&format!("CAN {id} {data}"))?,
        _ => return Err("can [-n N] [ID HEX]".into()),
    };
    c.send(&first).map_err(|e| e.to_string())?;
    let mut n = 0;
    loop {
        match c
            .recv_timeout(Duration::from_secs(2))
            .map_err(|e| e.to_string())?
        {
            Some(ToClient::Can { id, data }) => {
                println!(
                    "{}  [{}]  {}",
                    ToClient::Can { id, data: vec![] }
                        .line()
                        .trim_start_matches("CAN ")
                        .trim(),
                    data.len(),
                    data.iter()
                        .map(|b| format!("{b:02X}"))
                        .collect::<Vec<_>>()
                        .join(" ")
                );
                n += 1;
                if limit.is_some_and(|l| n >= l) {
                    return Ok(());
                }
            }
            Some(ToClient::Err(e)) => return Err(e),
            Some(_) => {}
            None => return Err("no CAN traffic (ECU off or battery disconnected?)".into()),
        }
    }
}

fn play(socket: &Path, file: &str) -> Result<(), String> {
    let text = std::fs::read_to_string(file).map_err(|e| format!("{file}: {e}"))?;
    let mut steps = Vec::new();
    for (n, line) in text.lines().enumerate() {
        let line = line.split('#').next().unwrap_or("").trim();
        if line.is_empty() {
            continue;
        }
        let f: Vec<&str> = line.split_whitespace().collect();
        let [t, name, value] = f[..] else {
            return Err(format!("{file}:{}: expected SECONDS NAME VALUE", n + 1));
        };
        let t: f64 = t.parse().map_err(|e| format!("{file}:{}: {e}", n + 1))?;
        steps.push((t, name.to_string(), value.to_string()));
    }
    let mut c = connect(socket)?;
    let start = std::time::Instant::now();
    for (t, name, value) in steps {
        let wait = t - start.elapsed().as_secs_f64();
        if wait > 0.0 {
            std::thread::sleep(Duration::from_secs_f64(wait));
        }
        eprintln!("{t:7.1} s  {name} {value}");
        c.send(&ToServer::Set { name, value })
            .map_err(|e| e.to_string())?;
        if let Some(ToClient::Err(e)) = c
            .recv_timeout(Duration::from_millis(50))
            .map_err(|e| e.to_string())?
        {
            return Err(e);
        }
    }
    Ok(())
}
