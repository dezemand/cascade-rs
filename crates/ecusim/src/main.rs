//! ecusim: run a GK 2.0 SIMK41 engine ECU dump as its own process.
//!
//! ```text
//! ecusim serve --rom ROM [--socket PATH]    run the ECU (Ctrl-C to stop)
//! ecusim set NAME VALUE [NAME VALUE]...     change controls, e.g.
//!                                           ecusim set ignition on starter on
//! ecusim status [--follow]                  print the state line(s)
//! ```
//!
//! Controls: ignition on|off, starter on|off, throttle 0-100 (%), coolant and
//! intake (°C), battery (V, engine off), speed (km/h), idle (rpm), rpm (fixed
//! engine speed; 0 = engine model). The socket defaults to `$TMPDIR/ecusim.sock`.
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
    ecusim::server::Server::new(data)
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
