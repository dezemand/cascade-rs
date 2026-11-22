//! ecuc167's command line (the binary's, not part of the library): `dis`, `scan` and `run` with its options,
//! and the value parsers for them.

use clap::{ArgAction, ArgMatches, Args, CommandFactory, FromArgMatches, Parser, Subcommand};
use ecuc167::cpu;

fn hex_bytes(s: &str) -> Result<Vec<u8>, String> {
    s.split([' ', ','])
        .filter(|t| !t.is_empty())
        .map(|t| u8::from_str_radix(t, 16).map_err(|e| format!("{t}: {e}")))
        .collect()
}

fn hexnum(s: &str) -> Result<u32, String> {
    u32::from_str_radix(s.trim_start_matches("0x"), 16).map_err(|e| format!("{s}: {e}"))
}

fn hexnum16(s: &str) -> Result<u16, String> {
    Ok(hexnum(s)? as u16)
}

fn float(s: &str) -> Result<f64, String> {
    s.parse::<f64>().map_err(|e| format!("{s}: {e}"))
}

/// Hex bytes as one option value (`Vec<u8>` would make clap take many).
type Bytes = Vec<u8>;

/// Parse the process arguments; the matches tell where each option was
/// given (see `parse_run`).
pub fn parse() -> (Cli, ArgMatches) {
    let m = Cli::command().get_matches();
    let cli = Cli::from_arg_matches(&m).unwrap_or_else(|e| e.exit());
    (cli, m)
}

#[derive(Parser)]
#[command(
    name = "ecuc167",
    about = "Disassemble and run the GK SIMK41 engine ECU dump (C167 class) offline",
    arg_required_else_help = true
)]
pub struct Cli {
    #[command(subcommand)]
    pub cmd: Cmd,
}

#[derive(Subcommand)]
pub enum Cmd {
    /// Linear disassembly at a physical address
    Dis {
        rom: String,
        /// Physical address (hex)
        #[arg(value_parser = hexnum)]
        addr: u32,
        /// Number of instructions
        #[arg(default_value_t = 40)]
        count: usize,
    },
    /// Recursive traversal from the vectors; lists code that references the
    /// values
    #[command(args_override_self = true)]
    Scan {
        rom: String,
        /// A value to look for (hex); repeatable
        #[arg(long, value_name = "HEX", value_parser = hexnum16)]
        refs: Vec<u16>,
        /// Another entry point for the traversal (hex); repeatable
        #[arg(long = "from", value_name = "PC", value_parser = hexnum)]
        from: Vec<u32>,
    },
    /// Emulate, with a simulated KWP2000 tester on ASC0
    #[command(args_override_self = true)]
    Run(Box<RunArgs>),
}

pub struct RunOpts {
    pub rom: String,
    pub max_ms: f64,
    pub fcpu: u64,
    pub init_at: Option<f64>,
    pub reqs: Vec<Vec<u8>>,
    pub p3_ms: f64,
    pub gap_ms: f64,
    pub wake_low: f64,
    pub wake_high: f64,
    pub adc: Vec<(usize, u16)>,
    pub wheels: Vec<cpu::Wheel>,
    pub compares: bool,
    pub rpm_at: Vec<(f64, f64)>,
    pub p2_in: Option<u16>,
    pub cmp_wr: bool,
    pub ssc_trace: bool,
    pub watch_from: f64,
    pub adc_at: Vec<(f64, usize, u16)>,
    pub ram_in: Vec<(String, u32)>,
    pub ram_out: Vec<(String, u32, u32)>,
    pub echo: bool,
    pub trace_from: Option<f64>,
    pub trace_n: u64,
    pub prof: bool,
    pub watch: Vec<(u32, u32)>,
    pub stop_at: Option<u32>,
    pub dumps: Vec<(u32, u32)>,
    pub start: Vec<u8>,
    pub show_rx: bool,
    pub pokes: Vec<(u32, u32, Vec<u8>)>,
    pub watch_w: bool,
    pub can_log: bool,
    pub can_ack: bool,
    /// Frames another node sends: (id, data, period ms).
    pub can_rx: Vec<(u32, Vec<u8>, f64)>,
}

/// Times are emulated milliseconds; addresses and values are hex.
#[derive(Args)]
pub struct RunArgs {
    pub rom: String,
    /// Emulated time
    #[arg(short = 'm', value_name = "MS", default_value = "3000", value_parser = float)]
    pub max_ms: f64,
    /// CPU clock (16 MHz: the boot code's S0BG 0x2F is then 10,417 baud and
    /// the CAPCOM2 CC26 tick exactly 1 ms)
    #[arg(long, value_name = "HZ", default_value_t = 16_000_000)]
    pub fcpu: u64,
    /// Start the tester: KWP2000 fast init (--wake-low, --wake-high), then
    /// the --start request, then each --req, one after each ECU reply
    #[arg(long, value_name = "MS", value_parser = float, help_heading = "Tester")]
    pub init_at: Option<f64>,
    /// A request (service bytes; header and checksum are added); repeatable
    #[arg(long, value_name = "HEX ...", value_parser = hex_bytes, help_heading = "Tester")]
    pub req: Vec<Bytes>,
    /// The first request after the fast init
    #[arg(long, value_name = "HEX ...", default_value = "81", value_parser = hex_bytes, help_heading = "Tester")]
    pub start: Bytes,
    /// Gap after an ECU reply before the next request
    #[arg(long, value_name = "MS", default_value = "60", value_parser = float, help_heading = "Tester")]
    pub p3: f64,
    /// Tester inter-byte gap
    #[arg(long, value_name = "MS", default_value = "2", value_parser = float, help_heading = "Tester")]
    pub gap: f64,
    /// Fast-init low time
    #[arg(long, value_name = "MS", default_value = "25", value_parser = float, help_heading = "Tester")]
    pub wake_low: f64,
    /// Fast-init high time
    #[arg(long, value_name = "MS", default_value = "25", value_parser = float, help_heading = "Tester")]
    pub wake_high: f64,
    /// The tester does not hear its own bytes
    #[arg(long = "no-echo", action = ArgAction::SetFalse, help_heading = "Tester")]
    pub echo: bool,
    /// Print every byte the ECU receives
    #[arg(long, help_heading = "Tester")]
    pub show_rx: bool,
    /// ADC input (10 bit); repeatable
    #[arg(long, value_name = "CH=VAL", value_parser = parse_adc, help_heading = "Inputs")]
    pub adc: Vec<(usize, u16)>,
    /// Set an ADC input from that time on (later entries for the same
    /// channel win once their time is reached); repeatable
    #[arg(long, value_name = "MS:CH=VAL", value_parser = parse_adc_at, help_heading = "Inputs")]
    pub adc_at: Vec<(f64, usize, u16)>,
    /// A toothed wheel on timer inputs, `+` separated: `t0in`, `t7in`,
    /// `capin`, `cc0`..`cc31` (e.g. `t0in+capin:60:2:800`; `cc30` is the
    /// vehicle speed input); repeatable
    #[arg(long, value_name = "PINS:TEETH:MISSING:RPM", value_parser = parse_wheel, help_heading = "Inputs")]
    pub wheel: Vec<cpu::Wheel>,
    /// A cam level locked to the --wheel before it: low from crank tooth
    /// position FALL to RISE over 720 degrees (0..2 x teeth), several
    /// intervals for a multi-tooth cam
    #[arg(long, value_name = "PINS:FALL:RISE[:FALL:RISE...]", value_parser = parse_cam, help_heading = "Inputs")]
    pub cam: Vec<CamSpec>,
    /// Change the first wheel's speed at that time; repeatable
    #[arg(long, value_name = "MS=RPM", value_parser = parse_rpm_at, help_heading = "Inputs")]
    pub rpm_at: Vec<(f64, f64)>,
    /// Levels of undriven P2 input pins (default 0)
    #[arg(long, value_name = "HEX", value_parser = hexnum16, help_heading = "Inputs")]
    pub p2_in: Option<u16>,
    /// Acknowledge CAN frames the ECU sends
    #[arg(long, help_heading = "CAN")]
    pub can_ack: bool,
    /// Print CAN frames
    #[arg(long, help_heading = "CAN")]
    pub can_log: bool,
    /// A frame another node sends every PERIOD_MS (implies --can-ack);
    /// repeatable
    #[arg(long, value_name = "ID:HEX:PERIOD_MS", value_parser = parse_can_rx, help_heading = "CAN")]
    pub can_rx: Vec<(u32, Bytes, f64)>,
    /// Print every CAPCOM compare match
    #[arg(long, help_heading = "Analysis")]
    pub compares: bool,
    /// Compare matches only on timer counts, not when software writes a
    /// CAPCOM timer
    #[arg(long = "no-compare-on-write", action = ArgAction::SetFalse, help_heading = "Analysis")]
    pub cmp_wr: bool,
    /// Load memory before reset (keep-alive RAM across power cycles);
    /// repeatable
    #[arg(long, value_name = "FILE@ADDR", value_parser = parse_ram_in, help_heading = "Analysis")]
    pub ram_in: Vec<(String, u32)>,
    /// Save memory at the end; repeatable
    #[arg(long, value_name = "FILE@ADDR:LEN", value_parser = parse_ram_out, help_heading = "Analysis")]
    pub ram_out: Vec<(String, u32, u32)>,
    /// Print every SSC byte sent, with the P6 latch (chip selects)
    #[arg(long, help_heading = "Analysis")]
    pub ssc_trace: bool,
    /// Instruction trace from that time
    #[arg(long, value_name = "MS", value_parser = float, help_heading = "Analysis")]
    pub trace_from: Option<f64>,
    /// Length of the instruction trace
    #[arg(
        long,
        value_name = "N",
        default_value_t = 2000,
        help_heading = "Analysis"
    )]
    pub trace_n: u64,
    /// Print the hot spots
    #[arg(long, help_heading = "Analysis")]
    pub prof: bool,
    /// Log accesses to a physical address range; repeatable
    #[arg(long, value_name = "LO-HI", value_parser = parse_range, help_heading = "Analysis")]
    pub watch: Vec<(u32, u32)>,
    /// --watch logs writes only
    #[arg(long, help_heading = "Analysis")]
    pub watch_w: bool,
    /// Start the --watch log at that time (the printed log is limited to
    /// 200,000 entries)
    #[arg(long, value_name = "MS", default_value = "0", value_parser = float, help_heading = "Analysis")]
    pub watch_from: f64,
    /// Stop when PC is reached
    #[arg(long, value_name = "PC", value_parser = hexnum, help_heading = "Analysis")]
    pub stop_at: Option<u32>,
    /// Print memory at the end; repeatable
    #[arg(long, num_args = 2, value_names = ["ADDR", "LEN"], help_heading = "Analysis")]
    pub dump: Vec<String>,
    /// Store bytes (physical) whenever PC is reached; repeatable
    #[arg(long, num_args = 2, value_names = ["PC", "ADDR=HEX.."], help_heading = "Analysis")]
    pub poke_at: Vec<String>,
}

/// `--cam` pins and (fall, rise) intervals.
type CamSpec = (Vec<cpu::Pin>, Vec<(u32, u32)>);

fn parse_adc(s: &str) -> Result<(usize, u16), String> {
    let (c, val) = s.split_once('=').ok_or("want CH=VAL")?;
    Ok((c.parse().map_err(|e| format!("{e}"))?, hexnum(val)? as u16))
}

fn parse_adc_at(a: &str) -> Result<(f64, usize, u16), String> {
    let (t, r) = a.split_once(':').ok_or("want MS:CH=VAL")?;
    let (ch, val) = r.split_once('=').ok_or("want MS:CH=VAL")?;
    Ok((
        t.parse::<f64>().map_err(|e| format!("{t}: {e}"))?,
        ch.parse::<usize>().map_err(|e| format!("{ch}: {e}"))?,
        hexnum(val)? as u16,
    ))
}

fn parse_can_rx(s: &str) -> Result<(u32, Bytes, f64), String> {
    let p: Vec<&str> = s.split(':').collect();
    let [id, data, period] = p[..] else {
        return Err("want ID:HEX:PERIOD_MS".into());
    };
    Ok((cascade_can::parse_id(id)?, hex_bytes(data)?, float(period)?))
}

fn parse_range(s: &str) -> Result<(u32, u32), String> {
    let (l, h) = s.split_once('-').unwrap_or((s, s));
    Ok((hexnum(l)?, hexnum(h)?))
}

fn parse_ram_in(a: &str) -> Result<(String, u32), String> {
    let (f, addr) = a.split_once('@').ok_or("want FILE@ADDR")?;
    Ok((f.to_string(), hexnum(addr)?))
}

fn parse_ram_out(a: &str) -> Result<(String, u32, u32), String> {
    let (f, r) = a.split_once('@').ok_or("want FILE@ADDR:LEN")?;
    let (addr, len) = r.split_once(':').ok_or("want FILE@ADDR:LEN")?;
    Ok((f.to_string(), hexnum(addr)?, hexnum(len)?))
}

fn parse_rpm_at(a: &str) -> Result<(f64, f64), String> {
    let (t, r) = a.split_once('=').ok_or("want MS=RPM")?;
    Ok((float(t)?, float(r)?))
}

fn parse_pins(s: &str) -> Result<Vec<cpu::Pin>, String> {
    s.split('+').map(str::parse).collect()
}

/// `PINS:FALL:RISE[:FALL:RISE...]` in crank tooth positions over 720 degrees.
fn parse_cam(a: &str) -> Result<CamSpec, String> {
    let f: Vec<&str> = a.split(':').collect();
    if f.len() < 3 || f.len().is_multiple_of(2) {
        return Err("want PINS:FALL:RISE[:FALL:RISE...]".into());
    }
    let n = |x: &str| x.parse::<u32>().map_err(|e| format!("{x}: {e}"));
    let lows = f[1..]
        .chunks(2)
        .map(|c| Ok((n(c[0])?, n(c[1])?)))
        .collect::<Result<Vec<_>, String>>()?;
    Ok((parse_pins(f[0])?, lows))
}

/// `PINS:TEETH:MISSING:RPM`, e.g. `t0in+capin:60:2:800`.
fn parse_wheel(a: &str) -> Result<cpu::Wheel, String> {
    let f: Vec<&str> = a.split(':').collect();
    let [pins, t, m, rpm] = f[..] else {
        return Err("want PINS:TEETH:MISSING:RPM".into());
    };
    let mut w = cpu::Wheel::new(parse_pins(pins)?, float(t)? as u32, float(m)? as u32);
    w.rpm = float(rpm)?;
    Ok(w)
}

pub fn parse_run(a: RunArgs, m: &ArgMatches) -> Result<RunOpts, String> {
    let mut wheels = a.wheel;
    // each --cam goes with the last --wheel before it on the command line
    let wheel_at: Vec<usize> = m.indices_of("wheel").into_iter().flatten().collect();
    let cam_at = m.indices_of("cam").into_iter().flatten();
    for (at, (targets, lows)) in cam_at.zip(a.cam) {
        let w = wheel_at
            .iter()
            .rposition(|&w| w < at)
            .ok_or("--cam needs a --wheel first")?;
        wheels[w].cam = Some(cpu::Cam { targets, lows });
    }
    let dumps = a
        .dump
        .chunks(2)
        .map(|v| Ok((hexnum(&v[0])?, hexnum(&v[1])?)))
        .collect::<Result<_, String>>()?;
    let pokes = a
        .poke_at
        .chunks(2)
        .map(|v| {
            let (addr, b) = v[1].split_once('=').ok_or("--poke-at PC ADDR=HEX..")?;
            Ok((hexnum(&v[0])?, hexnum(addr)?, hex_bytes(b)?))
        })
        .collect::<Result<_, String>>()?;
    Ok(RunOpts {
        rom: a.rom,
        max_ms: a.max_ms,
        fcpu: a.fcpu,
        init_at: a.init_at,
        reqs: a.req,
        p3_ms: a.p3,
        gap_ms: a.gap,
        wake_low: a.wake_low,
        wake_high: a.wake_high,
        adc: a.adc,
        wheels,
        compares: a.compares,
        rpm_at: a.rpm_at,
        p2_in: a.p2_in,
        cmp_wr: a.cmp_wr,
        ssc_trace: a.ssc_trace,
        watch_from: a.watch_from,
        adc_at: a.adc_at,
        ram_in: a.ram_in,
        ram_out: a.ram_out,
        echo: a.echo,
        trace_from: a.trace_from,
        trace_n: a.trace_n,
        prof: a.prof,
        watch: a.watch,
        stop_at: a.stop_at,
        dumps,
        start: a.start,
        show_rx: a.show_rx,
        pokes,
        watch_w: a.watch_w,
        can_log: a.can_log,
        // a node that sends also acknowledges
        can_ack: a.can_ack || !a.can_rx.is_empty(),
        can_rx: a.can_rx,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cli_is_consistent() {
        Cli::command().debug_assert();
    }

    fn run(args: &[&str]) -> Result<RunOpts, String> {
        let m = Cli::command()
            .try_get_matches_from(["ecuc167", "run"].iter().chain(args))
            .map_err(|e| e.to_string())?;
        let Cmd::Run(a) = Cli::from_arg_matches(&m).unwrap().cmd else {
            unreachable!()
        };
        parse_run(*a, m.subcommand_matches("run").unwrap())
    }

    #[test]
    fn fault_sweep_command_line() {
        let o = run(&[
            "rom.bin",
            "--fcpu",
            "16000000",
            "-m",
            "31000",
            "--wheel",
            "cc15+t7in+t0in+capin:60:2:800",
            "--cam",
            "cc14:50:110",
            "--init-at",
            "30000",
            "--req",
            "18 00 00 00",
            "--adc",
            "1=0x200",
            "--adc-at",
            "3000:2=0x3ff",
        ])
        .unwrap();
        assert_eq!(o.max_ms, 31000.0);
        assert_eq!(o.reqs, vec![vec![0x18, 0, 0, 0]]);
        assert_eq!(o.start, vec![0x81]);
        assert_eq!(o.wheels.len(), 1);
        assert!(o.wheels[0].cam.is_some());
        assert_eq!(o.adc, vec![(1, 0x200)]);
        assert_eq!(o.adc_at, vec![(3000.0, 2, 0x3ff)]);
        assert!(o.echo && o.cmp_wr && !o.can_ack);
    }

    #[test]
    fn cam_goes_with_the_wheel_before_it() {
        let o = run(&[
            "rom.bin",
            "--wheel",
            "t0in:60:2:800",
            "--wheel",
            "cc30:4:0:100",
            "--cam",
            "cc14:50:110",
        ])
        .unwrap();
        assert!(o.wheels[0].cam.is_none() && o.wheels[1].cam.is_some());
        let o = run(&[
            "rom.bin",
            "--wheel",
            "t0in:60:2:800",
            "--cam",
            "cc14:50:110",
            "--wheel",
            "cc30:4:0:100",
        ])
        .unwrap();
        assert!(o.wheels[0].cam.is_some() && o.wheels[1].cam.is_none());
        assert!(
            run(&[
                "rom.bin",
                "--cam",
                "cc14:50:110",
                "--wheel",
                "t0in:60:2:800"
            ])
            .is_err()
        );
    }

    #[test]
    fn two_value_options() {
        let o = run(&[
            "rom.bin",
            "--dump",
            "f000",
            "20",
            "--poke-at",
            "1234",
            "f600=01,02",
            "--dump",
            "f100",
            "10",
            "--can-rx",
            "100:01:10",
        ])
        .unwrap();
        assert_eq!(o.dumps, vec![(0xf000, 0x20), (0xf100, 0x10)]);
        assert_eq!(o.pokes, vec![(0x1234, 0xf600, vec![1, 2])]);
        assert!(o.can_ack);
    }
}
