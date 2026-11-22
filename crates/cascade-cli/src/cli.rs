//! The command line: `cascade run|trace-diff|trace-stats|disasm` and the
//! value parsers for their options.

use cascade_core::{Key, KeyEvent};
use clap::{ArgMatches, Args, CommandFactory, FromArgMatches, Parser, Subcommand};

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

/// `FROM_CYCLE:FILE`, for `--pcs` and `--rom-reads`.
fn parse_from_file(v: &str) -> Result<(u64, String), String> {
    let (from, file) = v.split_once(':').ok_or("want FROM_CYCLE:FILE")?;
    Ok((parse_num(from)?, file.to_string()))
}

fn parse_u8(v: &str) -> Result<u8, String> {
    Ok(parse_num(v)? as u8)
}

fn parse_local(v: &str) -> Result<(u8, Vec<u8>), String> {
    let (id, hex) = v.split_once('=').ok_or("want ID=HEX")?;
    Ok((parse_num(id)? as u8, parse_hex(hex)?))
}

fn parse_respond(v: &str) -> Result<(Vec<u8>, Vec<u8>), String> {
    let (req, resp) = v.split_once('=').ok_or("want REQHEX=RESPHEX")?;
    Ok((parse_hex(req)?, parse_hex(resp)?))
}

fn parse_dtc(v: &str) -> Result<(u16, u8), String> {
    let (c, st) = v.split_once(':').ok_or("want CODE:STATUS")?;
    Ok((parse_num(c)? as u16, parse_num(st)? as u8))
}

fn parse_adc(v: &str) -> Result<(usize, u16), String> {
    let (ch, x) = v.split_once('=').ok_or("want CH=VALUE")?;
    Ok((parse_num(ch)? as usize, parse_num(x)? as u16))
}

fn parse_keybytes(v: &str) -> Result<[u8; 2], String> {
    parse_hex(v)?
        .try_into()
        .map_err(|_| "needs 2 bytes".to_string())
}

/// `CYCLE:NAME[:HOLD]`: the key goes down at CYCLE and up HOLD cycles later.
fn parse_key(v: &str) -> Result<[KeyEvent; 2], String> {
    let parts: Vec<&str> = v.split(':').collect();
    let (at, name, hold) = match parts.as_slice() {
        [at, name] => (*at, *name, "3M"),
        [at, name, hold] => (*at, *name, *hold),
        _ => return Err("want CYCLE:NAME[:HOLD]".into()),
    };
    let key = Key::from_name(name).ok_or_else(|| format!("unknown key {name:?}"))?;
    let at = parse_num(at)?;
    Ok([
        KeyEvent {
            at,
            key,
            down: true,
        },
        KeyEvent {
            at: at + parse_num(hold)?,
            key,
            down: false,
        },
    ])
}

/// Parse the process arguments; the matches tell where each option was given.
pub fn parse() -> (Cli, ArgMatches) {
    let m = Cli::command().get_matches();
    let cli = Cli::from_arg_matches(&m).unwrap_or_else(|e| e.exit());
    (cli, m)
}

#[derive(Parser)]
#[command(
    name = "cascade",
    about = "The CASCADE Hi-Scan emulator, headless, and its trace tools",
    args_override_self = true,
    arg_required_else_help = true
)]
pub struct Cli {
    #[command(subcommand)]
    pub cmd: Cmd,
}

#[derive(Subcommand)]
pub enum Cmd {
    /// Run the emulator headless
    ///
    /// The diagnostic line is upstream's fake ECU by default (as in the
    /// oracle's debug build). Exit 0 at the cycle cap, 1 on any other stop, 3 when the firmware's endless-loop reset fires (unless
    /// --continue-after-reset).
    #[command(args_override_self = true)]
    Run(Box<RunArgs>),
    /// Compare two oracle-format traces by machine state
    ///
    /// Reports the first divergence. Exit 0 if equal, 1 if they diverge, 2 on error.
    TraceDiff { left: String, right: String },
    /// Count records and distinct physical PCs.
    TraceStats { log: String },
    /// Linear disassembly from a virtual address
    ///
    /// Below 0xC000 the ROM maps
    /// 1:1; 0xC000-0xFFFF is the code window, whose physical start is --bank.
    #[command(args_override_self = true)]
    Disasm {
        rom: String,
        #[arg(value_name = "ADDR", value_parser = parse_num)]
        addr: u64,
        /// Number of instructions
        #[arg(short = 'n', value_name = "N", default_value = "40", value_parser = parse_num)]
        n: u64,
        /// Physical start of the code window
        #[arg(long, value_name = "PHYS", default_value = "0xC000", value_parser = parse_num)]
        bank: u64,
    },
}

/// Numbers take `0x` hex, an `M` suffix (millions) and `_` separators.
#[derive(Args)]
pub struct RunArgs {
    /// The Hi-Scan ROM pack
    pub rom: String,
    /// Stop after CYCLES (like the oracle's -m)
    #[arg(short = 'm', value_name = "CYCLES", value_parser = parse_num)]
    pub max: Option<u64>,
    /// Write the oracle trace format
    #[arg(long, value_name = "FILE")]
    pub trace: Option<String>,
    /// Write `dump` and `dump.ram` into DIR, like the oracle
    #[arg(long, value_name = "DIR")]
    pub dump: Option<String>,
    /// Press NAME at CYCLE for HOLD cycles (default 3M); repeatable
    #[arg(long, value_name = "CYCLE:NAME[:HOLD]", value_parser = parse_key)]
    pub key: Vec<[KeyEvent; 2]>,
    /// Print the LCD as ASCII at the end
    #[arg(long)]
    pub screen: bool,
    /// Save the LCD as a PNG at the end
    #[arg(long, value_name = "FILE")]
    pub png: Option<String>,
    /// Save the raw LCD memory at the end
    #[arg(long, value_name = "FILE")]
    pub lcd_dump: Option<String>,
    /// Write every line event (TX/RX/baud/init) with its emulated cycle
    #[arg(long, value_name = "FILE")]
    pub line_log: Option<String>,
    /// Print coverage counters
    #[arg(long)]
    pub stats: bool,
    /// What is on the diagnostic line: fake (upstream's fake ECU), null
    /// (nothing), kwp (a KWP2000 engine ECU model), canmod (the CAN Interface
    /// Module, experimental), simk41 (--ecu-rom), ecusim (--ecusim). The last
    /// of --iface, --ecu-rom, --ecusim and --replay decides.
    #[arg(long, value_name = "NAME", default_value = "fake")]
    pub iface: String,
    /// Write the KWP model's decoded request/response log (--iface kwp)
    #[arg(long, value_name = "FILE")]
    pub kwp_log: Option<String>,
    /// Store a DTC in the KWP model (hex with 0x); repeatable
    #[arg(long, value_name = "CODE:STATUS", value_parser = parse_dtc)]
    pub dtc: Vec<(u16, u8)>,
    /// Set the KWP model's ReadDataByLocalIdentifier block; repeatable
    #[arg(long, value_name = "ID=HEX", value_parser = parse_local)]
    pub local: Vec<(u8, Vec<u8>)>,
    /// Answer one exact request (hex, SID first); repeatable
    #[arg(long, value_name = "REQ=RESP", value_parser = parse_respond)]
    pub respond: Vec<(Vec<u8>, Vec<u8>)>,
    /// The KWP model's address (engine 0x11, ABS 0x28, SRS 0x58)
    #[arg(long, value_name = "ADDR", value_parser = parse_u8)]
    pub kwp_addr: Option<u8>,
    /// The key bytes of its StartComm reply
    #[arg(long, value_name = "HEX", value_parser = parse_keybytes)]
    pub kwp_keybytes: Option<[u8; 2]>,
    /// Length-byte headers in its replies
    #[arg(long)]
    pub kwp_length_byte: bool,
    /// Its session timeout in ms (default 5000)
    #[arg(long, value_name = "MS", value_parser = parse_num)]
    pub kwp_p3_max: Option<u64>,
    /// Its key bytes after a 5-baud init
    #[arg(long = "kwp-5baud-keybytes", value_name = "HEX", value_parser = parse_keybytes)]
    pub kwp_5baud_keybytes: Option<[u8; 2]>,
    /// ISO 9141-2 framing (traction control)
    #[arg(long)]
    pub kwp_iso9141: bool,
    /// Answer from a `kdiag --trace` recording (implies --iface kwp; address,
    /// key bytes and framing come from the trace)
    #[arg(long, value_name = "KDIAG.jsonl")]
    pub replay: Option<String>,
    /// Load an extended ROM, as upstream's -x: a Hyundai "TROUBLE SHOOTING
    /// PACK" (i-info) behind the main pack
    #[arg(long, short = 'x', value_name = "FILE")]
    pub exrom: Option<String>,
    /// Keep the EEPROM in FILE between runs (loaded if it exists, written at
    /// the end; upstream uses ROM.eep), otherwise it starts blank
    #[arg(long, value_name = "FILE")]
    pub eeprom: Option<String>,
    /// Run a SIMK41 engine ECU dump (ecuc167) as the ECU (implies --iface simk41)
    #[arg(long, value_name = "FILE")]
    pub ecu_rom: Option<String>,
    /// Set the SIMK41 ECU's ADC inputs (hex with 0x, 10 bit; default 0x200 =
    /// 2.5 V); repeatable
    #[arg(long, value_name = "CH=VAL", value_parser = parse_adc)]
    pub ecu_adc: Vec<(usize, u16)>,
    /// Talk to `ecusim serve` (the ECU dump running in real time as its own
    /// process; `auto` = its default socket; implies --iface ecusim and
    /// --realtime). Unix only.
    #[arg(long, value_name = "SOCKET|auto")]
    pub ecusim: Option<String>,
    /// Pace the emulator to wall time
    #[arg(long)]
    pub realtime: bool,
    /// Write every ROM offset read through the data window from CYCLE on,
    /// with counts
    #[arg(long, value_name = "CYCLE:FILE", value_parser = parse_from_file)]
    pub rom_reads: Option<(u64, String)>,
    /// Write every executed code address (physical for the code window) from
    /// CYCLE on, with counts
    #[arg(long, value_name = "CYCLE:FILE", value_parser = parse_from_file)]
    pub pcs: Option<(u64, String)>,
    /// Keep running after the firmware's endless-loop reset
    #[arg(long)]
    pub continue_after_reset: bool,
}

/// --iface, --ecu-rom, --ecusim and --replay all choose the interface; the
/// last one on the command line wins.
pub fn iface_choice<'a>(iface: &'a str, m: &ArgMatches) -> &'a str {
    let last = |id: &str| m.indices_of(id).and_then(|i| i.max());
    [
        ("iface", iface),
        ("ecu_rom", "simk41"),
        ("ecusim", "ecusim"),
        ("replay", "kwp"),
    ]
    .into_iter()
    .filter_map(|(id, name)| Some((last(id)?, name)))
    .max()
    .map_or(iface, |(_, name)| name)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cli_is_consistent() {
        Cli::command().debug_assert();
    }

    fn run_args(args: &[&str]) -> (RunArgs, ArgMatches) {
        let m = Cli::command()
            .try_get_matches_from(["cascade", "run"].iter().chain(args))
            .unwrap();
        let sub = m.subcommand_matches("run").unwrap().clone();
        match Cli::from_arg_matches(&m).unwrap().cmd {
            Cmd::Run(a) => (*a, sub),
            _ => unreachable!(),
        }
    }

    fn iface(args: &[&str]) -> String {
        let (a, m) = run_args(args);
        iface_choice(&a.iface, &m).to_string()
    }

    #[test]
    fn last_interface_option_wins() {
        // tools/screen.py sends --iface kwp first, then its extra options
        assert_eq!(
            iface(&["rom", "--iface", "kwp", "--ecu-rom", "e"]),
            "simk41"
        );
        assert_eq!(iface(&["rom", "--ecu-rom", "e", "--iface", "kwp"]), "kwp");
        assert_eq!(iface(&["rom", "--iface", "kwp", "--iface", "null"]), "null");
        assert_eq!(
            iface(&["rom", "--iface", "kwp", "--replay", "t.jsonl"]),
            "kwp"
        );
        assert_eq!(
            iface(&["rom", "--replay", "t.jsonl", "--iface", "fake"]),
            "fake"
        );
        assert_eq!(iface(&["rom"]), "fake");
    }

    #[test]
    fn screen_py_command_line() {
        let (a, _) = run_args(&[
            "roms/hyundai.his.dat",
            "-m",
            "280000000",
            "--iface",
            "kwp",
            "--lcd-dump",
            "traces/screen.lcd",
            "--local",
            "0x01=00ff",
            "--dtc",
            "0x0130:0x20",
            "--respond",
            "1A8E=5A8E41",
            "--kwp-keybytes",
            "EA8F",
            "--kwp-length-byte",
            "--kwp-5baud-keybytes",
            "0808",
            "--key",
            "59899906:enter:1572865",
            "--key",
            "89000000:down",
        ]);
        assert_eq!(a.max, Some(280_000_000));
        assert_eq!(a.local, vec![(1, vec![0, 0xff])]);
        assert_eq!(a.dtc, vec![(0x0130, 0x20)]);
        assert_eq!(a.respond, vec![(vec![0x1a, 0x8e], vec![0x5a, 0x8e, 0x41])]);
        assert_eq!(a.kwp_keybytes, Some([0xea, 0x8f]));
        assert_eq!(a.kwp_5baud_keybytes, Some([0x08, 0x08]));
        assert!(a.kwp_length_byte);
        assert_eq!(a.key.len(), 2);
        assert_eq!(a.key[0][1].at, 59_899_906 + 1_572_865);
        assert_eq!(a.key[1][1].at, 89_000_000 + 3_000_000);
    }
}
