//! kdiag's command line: the subcommands, the options they share, and the
//! source rules checked after parsing.

use std::time::Duration;

use clap::{ArgAction, Args, Parser, Subcommand, ValueEnum};

#[derive(Clone, Copy, PartialEq, Eq, Debug, ValueEnum)]
pub enum System {
    /// 0x11 on the K line (OBD pin 7)
    Engine,
    /// 0x28 on OBD pin 8: the cable must connect pin 8 to K
    Abs,
    /// 0x58 on OBD pin 12: the cable must connect pin 12 to K
    Srs,
    /// 0x32 on OBD pin 8 (traction control, 2003-): 5-baud init and ISO
    /// 9141-2 frames; serial or --sim only, not J2534
    Tcs,
}

/// Fast-init method on a serial adapter.
#[derive(Clone, Copy, PartialEq, Eq, Debug, ValueEnum)]
pub enum Wake {
    /// hold BREAK for the low time
    Break,
    /// send 0x00 at 360 baud (for adapters that ignore BREAK)
    Zero,
}

#[derive(Parser)]
#[command(
    name = "kdiag",
    about = "K-line diagnostics for the 2002 Hyundai Coupe (Tiburon GK), read-only by default",
    arg_required_else_help = true
)]
pub struct Cli {
    #[command(subcommand)]
    pub cmd: Cmd,
}

#[derive(Subcommand)]
pub enum Cmd {
    /// Read stored fault codes
    #[command(args_override_self = true)]
    Dtc(Opts),
    /// Log live data as CSV
    ///
    /// One row per `21 01` response: time, decoded values from the embedded
    /// protocol definitions, and the raw block in hex, so logs can be
    /// re-decoded if the definitions change.
    #[command(args_override_self = true)]
    Log(Opts),
    /// Clear stored fault codes (needs --allow-clear and a typed confirmation)
    #[command(args_override_self = true)]
    Clear(Opts),
    /// ECU identification as the Hi-Scan asks for it
    #[command(args_override_self = true)]
    Ident(Opts),
    /// SRS crash records, printed raw (--system srs)
    #[command(args_override_self = true)]
    Crash(Opts),
    /// ELM327 over TCP for OBD apps, through the read-only guard
    #[command(args_override_self = true)]
    Elm(Opts),
    /// Answer on CAN as an engine ECU
    ///
    /// The simulated OBD-II ECU, or with --sim, --port or --ecusim a K-line
    /// ECU behind the read-only guard (a CAN gateway).
    #[command(args_override_self = true)]
    CanEcu(Opts),
    /// Adapter checks, no ECU session
    #[command(args_override_self = true)]
    Selftest {
        /// The adapter's serial port
        #[arg(long, value_name = "DEV")]
        port: String,
    },
    /// Show installed J2534 drivers (Windows)
    J2534List,
}

/// Give exactly one source: --sim (or --sim-ecu), --port, --ecusim, --j2534
/// or --can.
#[derive(Args, Clone)]
pub struct Opts {
    /// Talk to the KwpEcu model in virtual time, with plausible values
    #[arg(long, help_heading = "Source")]
    pub sim: bool,
    /// Run the engine ECU's own code instead (a SIMK41 dump, in the `ecuc167`
    /// emulator, virtual time; engine only; implies --sim). Sensors read
    /// mid-scale.
    #[arg(long, value_name = "ROM", help_heading = "Source")]
    pub sim_ecu: Option<String>,
    /// A serial K-line adapter
    #[arg(long, value_name = "DEV", help_heading = "Source")]
    pub port: Option<String>,
    /// Talk to `ecusim serve` (the ECU dump running in real time as its own
    /// process; `auto` = its default socket), in wall time
    #[arg(long, value_name = "SOCKET|auto", help_heading = "Source")]
    pub ecusim: Option<String>,
    /// A vendor PassThru DLL (Windows; `auto` = the first installed driver;
    /// build for i686-pc-windows-msvc for 32-bit DLLs) replaces --port: the
    /// adapter does the fast init, framing and timing
    #[arg(long, value_name = "DLL|auto", help_heading = "Source")]
    pub j2534: Option<String>,
    /// Run the J2534 message-level path (any OS) against --sim, --sim-ecu or
    /// --ecusim
    #[arg(long, help_heading = "Source")]
    pub j2534_mock: bool,
    /// OBD-II over CAN (ISO 15765-4 cars; the Tiburon GK itself has no CAN
    /// diagnostics): a Linux SocketCAN interface (`can0`, `vcan0`; bring it
    /// up first with `ip link`), or `sim`, an in-memory bus with a simulated
    /// OBD-II ECU (any OS). dtc, ident, log, clear and elm then use OBD-II
    /// modes 03/07, 09, 01 and 04. For can-ecu, the bus to answer on.
    #[arg(long, value_name = "IFACE|sim", help_heading = "Source")]
    pub can: Option<String>,
    /// ISO-TP request and response identifiers (7DF:7E8 for a functional
    /// request)
    #[arg(long, value_name = "TX:RX", default_value = "7E0:7E8", value_parser = parse_can_ids)]
    pub can_ids: (u32, u32),
    /// ISO-TP padding byte (hex), or none
    #[arg(long, value_name = "XX|none", default_value = "AA", value_parser = parse_can_pad)]
    pub can_pad: ::std::option::Option<u8>,
    /// Which control unit (addresses and framing as the Hi-Scan uses them,
    /// learned in the emulator; see protocol/)
    #[arg(long, value_enum, default_value_t = System::Engine)]
    pub system: System,
    /// `log`: write the CSV here instead of stdout
    #[arg(long, value_name = "FILE.csv")]
    pub out: Option<String>,
    /// Record every byte with a timestamp
    #[arg(long, value_name = "FILE.jsonl")]
    pub trace: Option<String>,
    /// `log`: time between requests
    #[arg(long = "interval-ms", value_name = "N", default_value = "170", value_parser = parse_ms)]
    pub interval: Duration,
    /// `log`: stop after N seconds
    #[arg(long = "duration-s", value_name = "N", value_parser = parse_secs)]
    pub duration: Option<Duration>,
    /// The adapter does not hear its own bytes
    #[arg(long = "no-echo", action = ArgAction::SetFalse)]
    pub echo: bool,
    /// Fast-init method
    #[arg(long, value_enum, default_value_t = Wake::Break)]
    pub wake: Wake,
    /// Let a fault code clear through (`clear`; mode 04 for `elm` and
    /// `can-ecu`)
    #[arg(long)]
    pub allow_clear: bool,
    /// `clear`: skip the typed confirmation
    #[arg(long)]
    pub yes: bool,
    /// `elm`: where to serve ELM327 clients
    #[arg(long, value_name = "ADDR:PORT", default_value = "127.0.0.1:35000")]
    pub listen: String,
}

fn parse_can_ids(v: &str) -> Result<(u32, u32), String> {
    let (tx, rx) = v.split_once(':').ok_or("want TX:RX, like 7E0:7E8")?;
    Ok((cascade_can::parse_id(tx)?, cascade_can::parse_id(rx)?))
}

fn parse_can_pad(v: &str) -> Result<Option<u8>, String> {
    if v == "none" {
        Ok(None)
    } else {
        u8::from_str_radix(v, 16)
            .map(Some)
            .map_err(|e| e.to_string())
    }
}

fn parse_ms(v: &str) -> Result<Duration, String> {
    v.parse()
        .map(Duration::from_millis)
        .map_err(|e| format!("{e}"))
}

fn parse_secs(v: &str) -> Result<Duration, String> {
    v.parse()
        .map(Duration::from_secs)
        .map_err(|e| format!("{e}"))
}

/// The source rules clap cannot say: --sim-ecu counts as --sim, and
/// `can_ecu` (--can is the bus to answer on, and the other source, if any,
/// is the K-line ECU behind it) differs from the other commands.
pub fn check(mut o: Opts, can_ecu: bool) -> Result<Opts, String> {
    o.sim |= o.sim_ecu.is_some();
    let sources =
        o.sim as u8 + o.port.is_some() as u8 + o.j2534.is_some() as u8 + o.ecusim.is_some() as u8;
    if can_ecu {
        return match &o.can {
            None => Err("can-ecu needs --can IFACE (the bus to answer on)".into()),
            Some(_) if sources > 1 => {
                Err("can-ecu bridges at most one K-line source (--sim, --port or --ecusim)".into())
            }
            Some(_) => Ok(o),
        };
    }
    if o.can.is_some() {
        if sources != 0 || o.j2534_mock {
            return Err("--can replaces --sim, --port, --ecusim and --j2534".into());
        }
        if o.system != System::Engine {
            return Err(
                "over CAN kdiag speaks OBD-II to the engine ECU only (--system engine)".into(),
            );
        }
        return Ok(o);
    }
    if sources != 1 {
        return Err(
            "give exactly one of --sim, --port DEV, --ecusim SOCKET|auto, --j2534 DLL|auto or --can IFACE|sim"
                .into(),
        );
    }
    if o.system == System::Tcs && (o.j2534.is_some() || o.j2534_mock) {
        return Err("traction control (ISO 9141-2, 5-baud init) is not done over J2534 yet".into());
    }
    if o.j2534_mock && !o.sim && o.ecusim.is_none() {
        return Err("--j2534-mock goes with --sim, --sim-ecu or --ecusim".into());
    }
    Ok(o)
}

#[cfg(test)]
mod tests {
    use super::*;
    use clap::CommandFactory;

    #[test]
    fn cli_is_consistent() {
        Cli::command().debug_assert();
    }

    fn opts(args: &[&str]) -> Result<Opts, String> {
        let cli = Cli::try_parse_from(["kdiag"].iter().chain(args)).map_err(|e| e.to_string())?;
        match cli.cmd {
            Cmd::Dtc(o) | Cmd::Log(o) | Cmd::Clear(o) | Cmd::Ident(o) | Cmd::Crash(o) => {
                check(o, false)
            }
            Cmd::Elm(o) => check(o, false),
            Cmd::CanEcu(o) => check(o, true),
            _ => Err("not a session command".into()),
        }
    }

    #[test]
    fn car_day_rehearsal_command_lines() {
        // tools/car_day_rehearsal.sh
        let o = opts(&[
            "log",
            "--ecusim",
            "auto",
            "--system",
            "tcs",
            "--duration-s",
            "2",
        ])
        .unwrap();
        assert_eq!(o.ecusim.as_deref(), Some("auto"));
        assert_eq!(o.system, System::Tcs);
        assert_eq!(o.duration, Some(Duration::from_secs(2)));
        assert_eq!(o.interval, Duration::from_millis(170));
        let o = opts(&["dtc", "--ecusim", "auto", "--j2534-mock"]).unwrap();
        assert!(o.j2534_mock && o.echo);
        assert_eq!(o.wake, Wake::Break);
        assert_eq!(o.can_ids, (0x7e0, 0x7e8));
        assert_eq!(o.can_pad, Some(0xaa));
        assert!(opts(&["crash", "--ecusim", "auto", "--system", "srs"]).is_ok());
    }

    #[test]
    fn sources() {
        assert!(opts(&["dtc"]).is_err());
        assert!(opts(&["dtc", "--sim", "--port", "/dev/x"]).is_err());
        // --sim-ecu is a kind of --sim
        let o = opts(&["dtc", "--sim", "--sim-ecu", "rom.bin"]).unwrap();
        assert!(o.sim);
        assert!(opts(&["dtc", "--sim-ecu", "rom.bin"]).unwrap().sim);
        assert!(opts(&["dtc", "--j2534-mock", "--port", "/dev/x"]).is_err());
        assert!(opts(&["dtc", "--can", "sim", "--sim"]).is_err());
        assert!(opts(&["dtc", "--can", "sim", "--system", "abs"]).is_err());
        let o = opts(&[
            "dtc",
            "--can",
            "sim",
            "--can-ids",
            "7DF:7E8",
            "--can-pad",
            "none",
        ])
        .unwrap();
        assert_eq!(o.can_ids, (0x7df, 0x7e8));
        assert_eq!(o.can_pad, None);
        assert!(opts(&["can-ecu", "--ecusim", "auto"]).is_err());
        assert!(opts(&["can-ecu", "--can", "vcan0", "--ecusim", "auto"]).is_ok());
        assert!(opts(&["can-ecu", "--can", "vcan0", "--sim", "--port", "p"]).is_err());
        assert!(opts(&["dtc", "--sim", "--system", "tcs", "--j2534-mock"]).is_err());
        assert!(opts(&["dtc", "--sim", "--wake", "bogus"]).is_err());
    }

    #[test]
    fn clear_flags_stay_plain() {
        let o = opts(&["clear", "--sim", "--system", "abs"]).unwrap();
        assert!(!o.allow_clear && !o.yes);
        let o = opts(&["clear", "--sim", "--allow-clear", "--yes"]).unwrap();
        assert!(o.allow_clear && o.yes);
    }
}
