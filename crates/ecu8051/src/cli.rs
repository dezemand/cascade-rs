//! ecu8051's command line and the value parsers for it.

use clap::{ArgAction, Parser};

pub struct Opts {
    pub rom: String,
    pub max_ms: f64,
    pub fosc: u64,
    pub chan: usize,
    pub init_at_ms: Option<f64>,
    pub reqs: Vec<Vec<u8>>,
    pub watch: Vec<u8>,
    pub xlog: Option<(u16, u16)>,
    pub trace_from_ms: Option<f64>,
    pub trace_n: u64,
    pub new_pcs: bool,
    pub hot: bool,
    pub adc: Vec<(usize, u16)>,
    pub pin: Option<(u8, u8)>,
    pub echo: bool,
    pub gap_ms: f64,
    pub baud: f64,
    pub stop_at_pc: Option<u16>,
    pub five_baud: bool,
    pub wake06: bool,
    pub addr: u8,
    pub kwp: bool,
    pub show_rx: bool,
    pub kw1281: bool,
    pub acks: u32,
}

fn hex_bytes(s: &str) -> Result<Vec<u8>, String> {
    s.split([' ', ','])
        .filter(|t| !t.is_empty())
        .map(|t| u8::from_str_radix(t, 16).map_err(|e| format!("{t}: {e}")))
        .collect()
}

fn num(s: &str) -> Result<u64, String> {
    let r = if let Some(h) = s.strip_prefix("0x") {
        u64::from_str_radix(h, 16)
    } else {
        s.parse()
    };
    r.map_err(|e| format!("{s}: {e}"))
}

/// Hex bytes as one option value (`Vec<u8>` would make clap take many).
type Bytes = Vec<u8>;

fn hex8(s: &str) -> Result<u8, String> {
    u8::from_str_radix(s, 16).map_err(|e| format!("{e}"))
}

fn hex16(s: &str) -> Result<u16, String> {
    u16::from_str_radix(s, 16).map_err(|e| format!("{e}"))
}

fn float(s: &str) -> Result<f64, String> {
    s.parse().map_err(|e| format!("{e}"))
}

fn sfr_list(s: &str) -> Result<Bytes, String> {
    s.split(',').map(hex8).collect()
}

fn xlog_range(s: &str) -> Result<(u16, u16), String> {
    let (l, h) = s.split_once('-').ok_or("want LO-HI")?;
    Ok((hex16(l)?, hex16(h)?))
}

fn adc_input(s: &str) -> Result<(usize, u16), String> {
    let (c, val) = s.split_once('=').ok_or("want CH=VAL")?;
    Ok((num(c)? as usize, num(val)? as u16))
}

fn pin_bit(s: &str) -> Result<(u8, u8), String> {
    let (r, b) = s.split_once('.').ok_or("want SFR.BIT")?;
    Ok((hex8(r)?, b.parse().map_err(|e| format!("{e}"))?))
}

/// Times are emulated milliseconds. Tester modes, all after --init-at: by
/// default a 5-baud address and a KW1281-style block link; --raw sends each
/// --req as raw bytes after the 5-baud init; --fast --kwp does a KWP2000
/// fast init and wraps each --req in an ISO 14230 header.
#[derive(Parser)]
#[command(
    name = "ecu8051",
    args_override_self = true,
    arg_required_else_help = true
)]
pub struct Cli {
    pub rom: String,
    /// Emulated time
    #[arg(short = 'm', value_name = "MS", default_value = "2000", value_parser = float)]
    pub max_ms: f64,
    /// Oscillator (the ROM's 5-baud decoder only syncs for 15-16 MHz here)
    #[arg(long, value_name = "HZ", default_value = "16000000", value_parser = num)]
    pub fosc: u64,
    /// Serial channel the tester is on
    #[arg(long, value_name = "N", default_value = "1", value_parser = num)]
    pub chan: u64,
    /// Start the tester
    #[arg(long = "init-at", value_name = "MS", value_parser = float)]
    pub init_at_ms: Option<f64>,
    /// A request: a block body (title and data), or raw bytes with --raw,
    /// or service bytes with --kwp; repeatable
    #[arg(long, value_name = "HEX ...", value_parser = hex_bytes)]
    pub req: Vec<Bytes>,
    /// 5-baud address (hex)
    #[arg(long, value_name = "HEX", default_value = "10", value_parser = hex8)]
    pub addr: u8,
    /// ACK blocks to wait for before the requests
    #[arg(long, value_name = "N", default_value = "6", value_parser = num)]
    pub acks: u64,
    /// Send each --req as raw bytes after the 5-baud init (ISO 9141-2)
    #[arg(long)]
    pub raw: bool,
    /// Wrap each --req in an ISO 14230 header (with --fast)
    #[arg(long)]
    pub kwp: bool,
    /// KWP2000 fast init instead of the 5-baud address
    #[arg(long = "fast", action = ArgAction::SetFalse)]
    pub five_baud: bool,
    /// Wake with an 06 byte at the idle baud rate instead
    #[arg(long)]
    pub wake06: bool,
    /// Tester baud rate
    #[arg(long, value_name = "BAUD", default_value = "10400", value_parser = float)]
    pub baud: f64,
    /// Tester inter-byte gap
    #[arg(long, value_name = "MS", default_value = "1", value_parser = float)]
    pub gap: f64,
    /// ADC input; repeatable
    #[arg(long, value_name = "CH=VAL", value_parser = adc_input)]
    pub adc: Vec<(usize, u16)>,
    /// Mirror the K-line level on an input port pin
    #[arg(long, value_name = "SFR.BIT", value_parser = pin_bit)]
    pub pin: Option<(u8, u8)>,
    /// Log SFR accesses (hex, comma separated; 0 logs interrupts); repeatable
    #[arg(long = "watch-sfr", value_name = "9B,9C,...", value_parser = sfr_list)]
    pub watch: Vec<Bytes>,
    /// Log XRAM accesses in a range (hex)
    #[arg(long, value_name = "LO-HI", value_parser = xlog_range)]
    pub xlog: Option<(u16, u16)>,
    /// Instruction trace from that time
    #[arg(long = "trace-from", value_name = "MS", value_parser = float)]
    pub trace_from_ms: Option<f64>,
    /// Length of the instruction trace
    #[arg(long, value_name = "N", default_value = "2000", value_parser = num)]
    pub trace_n: u64,
    /// Stop when PC is reached (hex)
    #[arg(long = "stop-at", value_name = "PC", value_parser = hex16)]
    pub stop_at_pc: Option<u16>,
    /// Print the most executed PCs of the second half of the run
    #[arg(long)]
    pub hot: bool,
    /// Print the code ranges first reached after --init-at
    #[arg(long)]
    pub new_pcs: bool,
    /// Print every byte the ECU receives
    #[arg(long)]
    pub show_rx: bool,
    /// The tester does not hear its own bytes
    #[arg(long = "no-echo", action = ArgAction::SetFalse)]
    pub echo: bool,
}

pub fn parse() -> Opts {
    let a = Cli::parse();
    Opts {
        rom: a.rom,
        max_ms: a.max_ms,
        fosc: a.fosc,
        chan: a.chan as usize,
        init_at_ms: a.init_at_ms,
        reqs: a.req,
        watch: a.watch.concat(),
        xlog: a.xlog,
        trace_from_ms: a.trace_from_ms,
        trace_n: a.trace_n,
        new_pcs: a.new_pcs,
        hot: a.hot,
        adc: a.adc,
        pin: a.pin,
        echo: a.echo,
        gap_ms: a.gap,
        baud: a.baud,
        stop_at_pc: a.stop_at_pc,
        five_baud: a.five_baud,
        wake06: a.wake06,
        addr: a.addr,
        kwp: a.kwp,
        show_rx: a.show_rx,
        kw1281: !(a.kwp || a.raw),
        acks: a.acks as u32,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use clap::CommandFactory;

    #[test]
    fn cli_is_consistent() {
        Cli::command().debug_assert();
    }

    #[test]
    fn command_line() {
        let a = Cli::try_parse_from([
            "ecu8051",
            "rom.bin",
            "--fast",
            "--kwp",
            "--req",
            "1a 8e",
            "--watch-sfr",
            "99,9b",
            "--watch-sfr",
            "0",
            "--addr",
            "33",
            "--fosc",
            "0xF42400",
        ])
        .unwrap();
        assert!(!a.five_baud && a.kwp && a.echo);
        assert_eq!(a.watch.concat(), vec![0x99, 0x9b, 0]);
        assert_eq!(a.addr, 0x33);
        assert_eq!(a.fosc, 16_000_000);
        assert_eq!(a.max_ms, 2000.0);
    }
}
