//! ecu8051: run the Coupe engine ECU ROM (8051 / 80C517 class) offline and
//! drive a simulated tester against its serial channel 1.
//!
//! Offline analysis only. Nothing here talks to a car.
//!
//! ```text
//! ecu8051 ROM [-m MS] [--fosc HZ] [--init-at MS] [--addr HEX]
//!             [--req "HEX ..."]... [--acks N] [--raw | --kwp] [--fast]
//!             [--adc CH=VAL]... [--watch-sfr 9B,9C,...] [--xlog LO-HI]
//!             [--trace-from MS] [--trace-n N] [--stop-at PC] [--hot]
//!             [--new-pcs] [--show-rx] [--no-echo] [--gap MS]
//! ```
//!
//! Tester modes, all after `--init-at MS` of emulated time:
//! - default: 5-baud address (`--addr`, default 10), answer the key bytes
//!   with the inverted KB2, then a KW1281-style block link: every byte except
//!   a block's final 03 is acknowledged with its complement. `--req` gives a
//!   block body (title and data); length, counter and 03 are added. Requests
//!   go out once the ECU idles with an ACK block (title 09).
//! - `--raw`: 5-baud init, then each `--req` is sent as raw bytes (for the
//!   ISO 9141-2 OBD path, address 33).
//! - `--fast --kwp`: KWP2000 fast init, `81 11 F1 81 04`, then each `--req`
//!   wrapped in an ISO 14230 header.
//!
//! `--fosc` defaults to 16 MHz: the ROM's 5-baud decoder only syncs for
//! 15-16 MHz in this model (12-14.75 and 17-20 MHz all fail).

mod cpu;

use cpu::{Cpu, Event};
use std::collections::BTreeMap;
use std::process::ExitCode;

struct Opts {
    rom: String,
    max_ms: f64,
    fosc: u64,
    chan: usize,
    init_at_ms: Option<f64>,
    reqs: Vec<Vec<u8>>,
    watch: Vec<u8>,
    xlog: Option<(u16, u16)>,
    trace_from_ms: Option<f64>,
    trace_n: u64,
    new_pcs: bool,
    hot: bool,
    adc: Vec<(usize, u16)>,
    pin: Option<(u8, u8)>,
    echo: bool,
    gap_ms: f64,
    baud: f64,
    stop_at_pc: Option<u16>,
    five_baud: bool,
    wake06: bool,
    addr: u8,
    kwp: bool,
    show_rx: bool,
    kw1281: bool,
    acks: u32,
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

fn parse() -> Result<Opts, String> {
    let mut a = std::env::args().skip(1);
    let mut o = Opts {
        rom: String::new(),
        max_ms: 2000.0,
        fosc: 16_000_000,
        chan: 1,
        init_at_ms: None,
        reqs: Vec::new(),
        watch: Vec::new(),
        xlog: None,
        trace_from_ms: None,
        trace_n: 2000,
        new_pcs: false,
        hot: false,
        adc: Vec::new(),
        pin: None,
        echo: true,
        gap_ms: 1.0,
        baud: 10400.0,
        stop_at_pc: None,
        five_baud: true,
        wake06: false,
        addr: 0x10,
        kwp: false,
        show_rx: false,
        kw1281: true,
        acks: 6,
    };
    while let Some(x) = a.next() {
        let mut v = || a.next().ok_or(format!("{x} needs a value"));
        match x.as_str() {
            "-m" => o.max_ms = v()?.parse().map_err(|e| format!("{e}"))?,
            "--fosc" => o.fosc = num(&v()?)?,
            "--chan" => o.chan = num(&v()?)? as usize,
            "--init-at" => o.init_at_ms = Some(v()?.parse().map_err(|e| format!("{e}"))?),
            "--req" => o.reqs.push(hex_bytes(&v()?)?),
            "--watch-sfr" => {
                for t in v()?.split(',') {
                    o.watch
                        .push(u8::from_str_radix(t, 16).map_err(|e| format!("{e}"))?);
                }
            }
            "--xlog" => {
                let s = v()?;
                let (l, h) = s.split_once('-').ok_or("--xlog LO-HI")?;
                o.xlog = Some((
                    u16::from_str_radix(l, 16).map_err(|e| format!("{e}"))?,
                    u16::from_str_radix(h, 16).map_err(|e| format!("{e}"))?,
                ));
            }
            "--trace-from" => o.trace_from_ms = Some(v()?.parse().map_err(|e| format!("{e}"))?),
            "--trace-n" => o.trace_n = num(&v()?)?,
            "--new-pcs" => o.new_pcs = true,
            "--hot" => o.hot = true,
            "--no-echo" => o.echo = false,
            "--fast" => o.five_baud = false,
            "--wake06" => o.wake06 = true,
            "--kwp" => {
                o.kwp = true;
                o.kw1281 = false;
            }
            "--raw" => o.kw1281 = false,
            "--acks" => o.acks = num(&v()?)? as u32,
            "--show-rx" => o.show_rx = true,
            "--addr" => o.addr = u8::from_str_radix(&v()?, 16).map_err(|e| format!("{e}"))?,
            "--gap" => o.gap_ms = v()?.parse().map_err(|e| format!("{e}"))?,
            "--baud" => o.baud = v()?.parse().map_err(|e| format!("{e}"))?,
            "--stop-at" => {
                o.stop_at_pc = Some(u16::from_str_radix(&v()?, 16).map_err(|e| format!("{e}"))?)
            }
            "--adc" => {
                let s = v()?;
                let (c, val) = s.split_once('=').ok_or("--adc CH=VAL")?;
                o.adc.push((num(c)? as usize, num(val)? as u16));
            }
            "--pin" => {
                let s = v()?;
                let (r, b) = s.split_once('.').ok_or("--pin SFR.BIT")?;
                o.pin = Some((
                    u8::from_str_radix(r, 16).map_err(|e| format!("{e}"))?,
                    b.parse().map_err(|e| format!("{e}"))?,
                ));
            }
            s if s.starts_with('-') => return Err(format!("unknown option {s}")),
            s => o.rom = s.to_string(),
        }
    }
    if o.rom.is_empty() {
        return Err("usage: ecu8051 ROM [options] (see source header)".into());
    }
    Ok(o)
}

/// ISO 14230 frame: format byte with length, target 0x11, source 0xF1.
fn frame(data: &[u8]) -> Vec<u8> {
    let mut f = if data.len() < 64 {
        vec![0x80 | data.len() as u8, 0x11, 0xF1]
    } else {
        vec![0x80, 0x11, 0xF1, data.len() as u8]
    };
    f.extend_from_slice(data);
    let sum = f.iter().fold(0u8, |s, &b| s.wrapping_add(b));
    f.push(sum);
    f
}

/// Split ECU output into ISO 14230 frames (best effort).
fn frames(bytes: &[u8]) -> Vec<Vec<u8>> {
    let mut out = Vec::new();
    let mut i = 0;
    while i < bytes.len() {
        let fmt = bytes[i];
        let (hdr, len) = match fmt & 0xC0 {
            0x00 => {
                if fmt & 0x3F == 0 {
                    (2, bytes.get(i + 1).copied().unwrap_or(0) as usize)
                } else {
                    (1, (fmt & 0x3F) as usize)
                }
            }
            _ => {
                if fmt & 0x3F == 0 {
                    (4, bytes.get(i + 3).copied().unwrap_or(0) as usize)
                } else {
                    (3, (fmt & 0x3F) as usize)
                }
            }
        };
        let end = (i + hdr + len + 1).min(bytes.len());
        out.push(bytes[i..end].to_vec());
        i = end;
    }
    out
}

fn hx(b: &[u8]) -> String {
    b.iter()
        .map(|x| format!("{x:02X}"))
        .collect::<Vec<_>>()
        .join(" ")
}

/// KW1281-style block link: every byte except a block's final 0x03 is
/// acknowledged by the receiver with its complement.
struct Kw {
    rx: Vec<u8>,
    tx: Vec<u8>,
    tx_i: usize,
    await_ack: Option<u8>,
    reqs: Vec<Vec<u8>>,
    acks_left: u32,
    counter: u8,
    pub log: Vec<(f64, bool, Vec<u8>)>,
}

impl Kw {
    fn send_next(&mut self, c: &mut Cpu, at: u64, bit: u64, chan: usize) {
        let b = self.tx[self.tx_i];
        c.line_rx(chan, at + 10 * bit, b, true);
        self.tx_i += 1;
        self.await_ack = if self.tx_i < self.tx.len() {
            Some(!b)
        } else {
            None
        };
    }

    /// One ECU byte seen on the line. Returns true when the session is over.
    #[allow(clippy::too_many_arguments)]
    fn on_ecu_byte(
        &mut self,
        c: &mut Cpu,
        now: u64,
        b: u8,
        cpm: f64,
        bit: u64,
        chan: usize,
    ) -> bool {
        let ms = now as f64 / cpm;
        if let Some(exp) = self.await_ack {
            if b != exp {
                println!("{ms:9.3} ms  kw: expected ack {exp:02X}, got {b:02X}");
            }
            self.send_next(c, now + (1.0 * cpm) as u64, bit, chan);
            if self.await_ack.is_none() {
                println!("{ms:9.3} ms  tester block {}", hx(&self.tx));
                self.log.push((ms, true, self.tx.clone()));
            }
            return false;
        }
        self.rx.push(b);
        let len = self.rx[0] as usize;
        if self.rx.len() < len + 1 {
            c.line_rx(chan, now + (2.0 * cpm) as u64 + 10 * bit, !b, true);
            return false;
        }
        let blk = std::mem::take(&mut self.rx);
        println!("{ms:9.3} ms  ECU block    {}", hx(&blk));
        self.log.push((ms, false, blk.clone()));
        self.counter = blk.get(1).copied().unwrap_or(0).wrapping_add(1);
        // Requests only once the ECU idles with an ACK block (it NAKs
        // requests while it is still sending its identification).
        let ecu_idle = blk.get(2) == Some(&0x09);
        let next = if ecu_idle { self.reqs.pop() } else { None };
        let body = if let Some(r) = next {
            r
        } else if !ecu_idle || self.acks_left > 0 {
            if ecu_idle {
                self.acks_left -= 1;
            }
            vec![0x09]
        } else {
            return true;
        };
        let mut t = vec![body.len() as u8 + 2, self.counter];
        t.extend_from_slice(&body);
        t.push(0x03);
        self.tx = t;
        self.tx_i = 0;
        self.send_next(c, now + (10.0 * cpm) as u64, bit, chan);
        false
    }
}

enum Phase {
    /// Sent the idle-baud wake byte 06; waiting for the ECU's F9.
    Wake,
    Kw,
    Idle,
    /// Line levels still to apply: (machine cycle, level).
    Levels(Vec<(u64, bool)>),
    /// 5-baud init done; waiting for 0x55, KB1, KB2.
    Sync,
    /// Waiting for the inverted address after our inverted KB2.
    InvAddr,
    Sending,
    Waiting {
        since: u64,
    },
    Done,
}

fn main() -> ExitCode {
    let o = match parse() {
        Ok(o) => o,
        Err(e) => {
            eprintln!("{e}");
            return ExitCode::from(2);
        }
    };
    let rom = match std::fs::read(&o.rom) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("{}: {e}", o.rom);
            return ExitCode::from(2);
        }
    };
    let mut c = Cpu::new(rom, o.fosc);
    c.echo = o.echo;
    c.xram_log = o.xlog;
    for &a in &o.watch {
        if a >= 0x80 {
            c.watch_sfr[(a - 0x80) as usize] = true;
        }
    }
    for &(ch, v) in &o.adc {
        c.adc[ch & 15] = v;
    }
    let cpm = o.fosc as f64 / 12.0 / 1000.0; // machine cycles per ms
    let ms = |cy: u64| cy as f64 / cpm;
    let max = (o.max_ms * cpm) as u64;
    let bit = (o.fosc as f64 / 12.0 / o.baud).round() as u64;
    let trace_from = o.trace_from_ms.map(|m| (m * cpm) as u64);
    let mut traced = 0u64;

    // coverage: [bank][pc] -> first cycle seen
    let mut seen = vec![u64::MAX; 0x20000];
    let mut hot: BTreeMap<u32, u64> = BTreeMap::new();
    let init_at = o.init_at_ms.map(|m| (m * cpm) as u64);
    let mut phase = Phase::Idle;
    let mut reqs: Vec<Vec<u8>> = Vec::new();
    if !o.five_baud {
        reqs.push(frame(&[0x81]));
    }
    reqs.extend(
        o.reqs
            .iter()
            .map(|r| if o.kwp { frame(r) } else { r.clone() }),
    );
    reqs.reverse();
    let mut kw = Kw {
        rx: Vec::new(),
        tx: Vec::new(),
        tx_i: 0,
        await_ack: None,
        reqs: if o.kw1281 {
            std::mem::take(&mut reqs)
        } else {
            Vec::new()
        },
        acks_left: o.acks,
        counter: 0,
        log: Vec::new(),
    };
    let mut ecu_bytes: Vec<u8> = Vec::new();
    let mut new_tx: Vec<u8> = Vec::new();
    let mut last_ecu_at = 0u64;
    let mut sent_req: Vec<u8> = Vec::new();
    let mut status = 0u8;

    while c.cycles < max {
        let pc = c.pc;
        let key = (c.bank() as u32) << 16 | pc as u32;
        if seen[key as usize] == u64::MAX {
            seen[key as usize] = c.cycles;
        }
        if o.hot && c.cycles > max / 2 {
            *hot.entry(key).or_default() += 1;
        }
        if let Some(t) = trace_from
            && c.cycles >= t
            && traced < o.trace_n
        {
            traced += 1;
            println!(
                "T {:9.3} b{} {:04X} {:02X} A={:02X} DP={:04X} SP={:02X}",
                ms(c.cycles),
                c.bank(),
                pc,
                c.code(pc),
                c.acc(),
                c.dp(),
                c.sfr_get(cpu::SP)
            );
        }
        if o.stop_at_pc == Some(pc) {
            println!(
                "stop at {:04X} bank {} t={:.3} ms",
                pc,
                c.bank(),
                ms(c.cycles)
            );
            break;
        }
        if let Err(e) = c.step() {
            println!("stop: {e:?} at t={:.3} ms", ms(c.cycles));
            status = 1;
            break;
        }
        for ev in c.events.drain(..) {
            match ev {
                Event::Sfr { pc, addr, val, acc } => println!(
                    "{:9.3} ms  {:04X} {:?} {}({:02X}) = {:02X}",
                    ms(c.cycles),
                    pc,
                    acc,
                    cpu::sfr_name(addr),
                    addr,
                    val
                ),
                Event::Tx { ch, byte, bit9 } => {
                    println!(
                        "{:9.3} ms  TX{ch} {byte:02X}{}",
                        ms(c.cycles),
                        if bit9 { "+" } else { "" }
                    );
                    if ch as usize == o.chan {
                        new_tx.push(byte);
                        ecu_bytes.push(byte);
                        last_ecu_at = c.cycles;
                    }
                }
                Event::Rx { ch, byte } => {
                    if o.show_rx {
                        println!("{:9.3} ms  RX{ch} {byte:02X}", ms(c.cycles));
                    }
                }
                Event::Irq { vector } => {
                    if o.watch.contains(&0) {
                        println!("{:9.3} ms  IRQ {vector:02X}", ms(c.cycles));
                    }
                }
            }
        }
        let now = c.cycles;
        let set_level = |c: &mut Cpu, lv: bool| {
            c.kline_level = lv;
            if let Some((r, b)) = o.pin {
                let p = &mut c.port_in[(r - 0x80) as usize];
                *p = if lv { *p | 1 << b } else { *p & !(1 << b) };
            }
        };
        if let Phase::Kw = phase {
            for b in new_tx.drain(..) {
                if kw.on_ecu_byte(&mut c, now, b, cpm, bit, o.chan) {
                    println!("{:9.3} ms  tester: done, going quiet", ms(now));
                    phase = Phase::Done;
                    break;
                }
            }
        }
        new_tx.clear();
        match &mut phase {
            Phase::Kw => {}
            Phase::Idle if o.wake06 => {
                if let Some(t) = init_at
                    && now >= t
                {
                    println!("{:9.3} ms  tester -> 06 (idle-baud wake)", ms(now));
                    c.line_rx(o.chan, now + 10 * bit, 0x06, true);
                    ecu_bytes.clear();
                    phase = Phase::Wake;
                }
            }
            Phase::Wake => {
                if let Some(&b) = ecu_bytes.first() {
                    println!("{:9.3} ms  ECU -> {b:02X}", ms(now));
                    ecu_bytes.clear();
                    // tester speaks first on this path
                    let body = kw.reqs.pop().unwrap_or_else(|| vec![0x09]);
                    let mut t = vec![body.len() as u8 + 2, 0x01];
                    t.extend_from_slice(&body);
                    t.push(0x03);
                    kw.tx = t;
                    kw.tx_i = 0;
                    kw.send_next(&mut c, now + (10.0 * cpm) as u64, bit, o.chan);
                    phase = Phase::Kw;
                }
            }
            Phase::Idle => {
                if let Some(t) = init_at
                    && now >= t
                {
                    let mut lv = Vec::new();
                    if o.five_baud {
                        println!("{:9.3} ms  tester: 5-baud address {:02X}", ms(now), o.addr);
                        let b = (200.0 * cpm) as u64;
                        lv.push((now, false)); // start bit
                        for i in 0..8 {
                            lv.push((now + (i + 1) * b, o.addr >> i & 1 != 0));
                        }
                        lv.push((now + 9 * b, true)); // stop bit
                    } else {
                        println!("{:9.3} ms  tester: fast init, K low 25 ms", ms(now));
                        // A 25 ms low also reads as 0x00 with a missing stop
                        // bit after one character time.
                        c.line_rx(o.chan, now + 10 * bit, 0x00, false);
                        lv.push((now, false));
                        lv.push((now + (25.0 * cpm) as u64, true));
                        lv.push((now + (50.0 * cpm) as u64, true));
                    }
                    lv.reverse();
                    phase = Phase::Levels(lv);
                }
            }
            Phase::Levels(lv) => {
                while let Some(&(t, l)) = lv.last() {
                    if now < t {
                        break;
                    }
                    lv.pop();
                    set_level(&mut c, l);
                }
                if lv.is_empty() {
                    ecu_bytes.clear();
                    phase = if o.five_baud {
                        Phase::Sync
                    } else {
                        Phase::Sending
                    };
                    last_ecu_at = now;
                }
            }
            Phase::Sync => {
                if ecu_bytes.len() >= 3 {
                    let kb2 = ecu_bytes[2];
                    println!("{:9.3} ms  ECU sync/key bytes {}", ms(now), hx(&ecu_bytes));
                    let at = now + (30.0 * cpm) as u64;
                    println!("{:9.3} ms  tester -> {:02X} (inverted KB2)", ms(at), !kb2);
                    c.line_rx(o.chan, at + 10 * bit, !kb2, true);
                    ecu_bytes.clear();
                    phase = if o.kw1281 { Phase::Kw } else { Phase::InvAddr };
                } else if now > last_ecu_at + (2000.0 * cpm) as u64 {
                    println!(
                        "{:9.3} ms  tester: no sync, got {}",
                        ms(now),
                        hx(&ecu_bytes)
                    );
                    phase = Phase::Done;
                }
            }
            Phase::InvAddr => {
                if !ecu_bytes.is_empty() && now > last_ecu_at + (60.0 * cpm) as u64 {
                    println!(
                        "{:9.3} ms  ECU -> {} (expect inverted address {:02X})",
                        ms(now),
                        hx(&ecu_bytes),
                        !o.addr
                    );
                    ecu_bytes.clear();
                    phase = Phase::Sending;
                } else if now > last_ecu_at + (2000.0 * cpm) as u64 {
                    println!("{:9.3} ms  tester: no inverted address", ms(now));
                    phase = Phase::Sending;
                }
            }
            Phase::Sending => {
                if let Some(f) = reqs.pop() {
                    println!("{:9.3} ms  tester -> {}", ms(now), hx(&f));
                    let step = 10 * bit + (o.gap_ms * cpm) as u64;
                    for (i, &b) in f.iter().enumerate() {
                        c.line_rx(o.chan, now + (i as u64 + 1) * step, b, true);
                    }
                    sent_req = f;
                    ecu_bytes.clear();
                    phase = Phase::Waiting {
                        since: now + sent_req.len() as u64 * step,
                    };
                } else {
                    phase = Phase::Done;
                }
            }
            Phase::Waiting { since } => {
                let since = *since;
                let quiet = (100.0 * cpm) as u64;
                let start = since.max(last_ecu_at);
                if now > start + quiet {
                    if ecu_bytes.is_empty() {
                        if now > since + (1500.0 * cpm) as u64 {
                            println!("{:9.3} ms  tester: no answer to {}", ms(now), hx(&sent_req));
                            phase = Phase::Sending;
                        }
                    } else {
                        if o.kwp {
                            for f in frames(&ecu_bytes) {
                                println!("{:9.3} ms  ECU -> {}", ms(now), hx(&f));
                            }
                        } else {
                            println!("{:9.3} ms  ECU -> {}", ms(now), hx(&ecu_bytes));
                        }
                        ecu_bytes.clear();
                        phase = Phase::Sending;
                    }
                }
            }
            Phase::Done => {}
        }
    }
    let _ = sent_req;
    println!(
        "end t={:.3} ms pc={:04X} bank={} S1 baud={:.0} S0 baud={:.0} overruns={:?}",
        ms(c.cycles),
        c.pc,
        c.bank(),
        c.baud(1),
        c.baud(0),
        [c.uart[0].rx_overruns, c.uart[1].rx_overruns]
    );
    if o.hot {
        let mut v: Vec<_> = hot.into_iter().collect();
        v.sort_by_key(|e| std::cmp::Reverse(e.1));
        for (k, n) in v.iter().take(40) {
            println!("hot b{} {:04X} {}", k >> 16, k & 0xFFFF, n);
        }
    }
    if o.new_pcs
        && let Some(t) = init_at
    {
        let mut ranges: Vec<(u32, u32, u64)> = Vec::new();
        for (k, &first) in seen.iter().enumerate() {
            if first != u64::MAX && first >= t {
                let k = k as u32;
                match ranges.last_mut() {
                    Some(r) if k <= r.1 + 4 => {
                        r.1 = k;
                        r.2 = r.2.min(first);
                    }
                    _ => ranges.push((k, k, first)),
                }
            }
        }
        ranges.sort_by_key(|r| r.2);
        for (a, b, first) in ranges {
            println!(
                "new b{} {:04X}-{:04X} first at {:.3} ms",
                a >> 16,
                a & 0xFFFF,
                b & 0xFFFF,
                ms(first)
            );
        }
    }
    ExitCode::from(status)
}
