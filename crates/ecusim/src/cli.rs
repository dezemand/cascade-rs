//! ecusim's command line (the binary's, not part of the library).

use std::path::PathBuf;

use clap::{Parser, Subcommand};

#[derive(Parser)]
#[command(
    name = "ecusim",
    about = "Run a GK 2.0 SIMK41 engine ECU dump as its own process",
    arg_required_else_help = true,
    args_override_self = true
)]
pub struct Cli {
    /// The server's socket (default `$TMPDIR/ecusim.sock`)
    #[arg(long, global = true, value_name = "PATH")]
    pub socket: Option<PathBuf>,
    #[command(subcommand)]
    pub cmd: Cmd,
}

#[derive(Subcommand)]
pub enum Cmd {
    /// Run the ECU (Ctrl-C to stop)
    Serve {
        /// A SIMK41 dump without immobiliser
        #[arg(long, value_name = "ROM")]
        rom: PathBuf,
    },
    /// Change controls, e.g. `ecusim set ignition on starter on`
    ///
    /// Controls: ignition on|off, starter on|off, throttle 0-100 (%), coolant
    /// and intake (°C), battery (V, engine off), speed (km/h), idle (rpm), rpm
    /// (fixed engine speed; 0 = engine model), battery_connected on|off (off
    /// wipes the ECU's memory), fault.NAME on|off|intermittent (wiring
    /// faults), time 1-10 (time factor while no tester is connected),
    /// snapshot NAME / restore NAME (whole simulator state, in memory).
    #[command(allow_negative_numbers = true)]
    Set {
        #[arg(value_names = ["NAME", "VALUE"], required = true, num_args = 2..)]
        pairs: Vec<String>,
    },
    /// Print the state line
    Status {
        /// Keep printing state lines
        #[arg(long)]
        follow: bool,
    },
    /// Run a scenario file: one `SECONDS NAME VALUE` step per line (wall
    /// time from the start), `#` comments; see `scenarios/`
    Play {
        #[arg(value_name = "SCENARIO")]
        file: String,
    },
    /// Show the engine CAN bus, candump style (and send one frame first)
    Can {
        /// Stop after N frames
        #[arg(short = 'n', value_name = "N")]
        limit: Option<usize>,
        /// A frame to send first: ID and data bytes in hex
        #[arg(value_names = ["ID", "HEX"], num_args = 2)]
        send: Vec<String>,
    },
}

#[cfg(test)]
mod tests {
    use super::*;
    use clap::CommandFactory;

    #[test]
    fn cli_is_consistent() {
        Cli::command().debug_assert();
    }

    fn parse(args: &[&str]) -> Result<Cli, clap::Error> {
        Cli::try_parse_from(["ecusim"].iter().chain(args))
    }

    #[test]
    fn command_lines() {
        // as ecusim-gui starts it
        let c = parse(&["serve", "--rom", "r.bin", "--socket", "/tmp/s"]).unwrap();
        assert_eq!(c.socket, Some(PathBuf::from("/tmp/s")));
        assert!(matches!(c.cmd, Cmd::Serve { .. }));
        assert!(parse(&["serve"]).is_err());
        let c = parse(&[
            "set", "coolant", "-10", "ignition", "on", "--socket", "/tmp/s",
        ])
        .unwrap();
        let Cmd::Set { pairs } = c.cmd else { panic!() };
        assert_eq!(pairs, ["coolant", "-10", "ignition", "on"]);
        assert_eq!(c.socket, Some(PathBuf::from("/tmp/s")));
        let Cmd::Can { limit, send } = parse(&["can", "-n", "5", "4F0", "0102"]).unwrap().cmd
        else {
            panic!()
        };
        assert_eq!((limit, send.len()), (Some(5), 2));
        assert!(parse(&["status", "--follow"]).is_ok());
        assert!(parse(&["play", "scenarios/airbag-light.txt"]).is_ok());
    }
}
