//! C167-class CPU core with the peripherals the SIMK41 code needs to reach
//! its main loop and serial handler: CAPCOM T0/T1 (+ compare interrupts),
//! GPT1/GPT2 timers, ADC (single, continuous and auto-scan), ASC0 with a
//! line model, SSC as a dumb loopback, PEC transfers and the interrupt
//! controller.
//!
//! Memory map, from the boot code's ADDRSEL/BUSCON setup [verified by
//! reading it] and assumptions where noted:
//! - 0x000000-0x07FFFF: CS0, the 256 KB flash, mirrored (A18 ignored, so
//!   the DPP0/1/2 pages 0x12/0x14/0x13 at 0x48000.. read flash 0x8000..).
//! - 0x00C000-0x00FFFF: RAM (CS2 external RAM at 0xC000-0xDFFF, XRAM and
//!   CAN at 0xE000-0xEFFF, internal RAM 0xF600-0xFDFF, SFRs) [assumed].
//! - everything else: RAM (CS1 at 0x80000 is 512 KB, contents unknown).

use crate::isa::{self, Alu, BitOp, Div, Ext, Insn, Jb, Op, Shift, bitoff_addr, decode, reg_addr};

pub const PSW: u32 = 0xFF10;
pub const P2: u32 = 0xFFC0;
pub const SP: u32 = 0xFE12;
pub const CP: u32 = 0xFE10;
pub const CSP: u32 = 0xFE08;
pub const MDH: u32 = 0xFE0C;
pub const MDL: u32 = 0xFE0E;
pub const SYSCON: u32 = 0xFF12;
pub const S0CON: u32 = 0xFFB0;
pub const S0TBUF: u32 = 0xFEB0;
pub const S0RBUF: u32 = 0xFEB2;
pub const S0BG: u32 = 0xFEB4;
pub const S0TIC: u32 = 0xFF6C;
pub const S0RIC: u32 = 0xFF6E;
pub const S0EIC: u32 = 0xFF70;
pub const S0TBIC: u32 = 0xF19C;
pub const P3: u32 = 0xFFC4;
pub const ADCON: u32 = 0xFFA0;
pub const ADDAT: u32 = 0xFEA0;
pub const ADCIC: u32 = 0xFF98;
pub const SSCTB: u32 = 0xF0B0;
pub const SSCRB: u32 = 0xF0B2;
pub const SSCTIC: u32 = 0xFF72;
pub const SSCRIC: u32 = 0xFF74;

const N: u16 = 1;
const C: u16 = 2;
const V: u16 = 4;
const Z: u16 = 8;
const E: u16 = 16;

/// Interrupt control registers and their trap numbers (C167CR).
fn irq_table() -> Vec<(u32, u8)> {
    let mut t = Vec::new();
    for k in 0..16 {
        t.push((0xFF78 + 2 * k, 0x10 + k as u8));
    }
    t.push((0xFF9C, 0x20));
    t.push((0xFF9E, 0x21));
    for k in 0..6 {
        t.push((0xFF60 + 2 * k, 0x22 + k as u8));
    }
    t.push((0xFF98, 0x28));
    t.push((0xFF9A, 0x29));
    for k in 0..6 {
        t.push((0xFF6C + 2 * k, 0x2A + k as u8));
    }
    for k in 0..13 {
        t.push((0xF160 + 2 * k, 0x30 + k as u8));
    }
    t.extend([
        (0xF17A, 0x3D),
        (0xF17C, 0x3E),
        (0xF17E, 0x3F),
        (0xF186, 0x40),
        (0xF18E, 0x41),
        (0xF196, 0x42),
        (0xF19E, 0x43),
        (0xF184, 0x44),
        (0xF18C, 0x45),
        (0xF194, 0x46),
        (0xF19C, 0x47),
    ]);
    t
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ExtMode {
    None,
    Page(u16),
    Seg(u8),
}

/// Something the caller may want to log or react to.
#[derive(Clone, Debug)]
pub enum Event {
    /// A CAPCOM compare match (channel in compare mode 1-3 or with its
    /// interrupt) at this time: unit 0/1, channel 0-15, mode bits.
    Compare { at: u64, unit: u8, ch: u8, mode: u8 },
    /// The ECU finished sending a byte on ASC0.
    Tx { at: u64, byte: u8 },
    /// S0BG/S0CON changed.
    Baud { at: u64, s0bg: u16, s0con: u16 },
    /// The ECU's receiver completed a frame.
    Rx {
        at: u64,
        byte: u8,
        fe: bool,
        enabled: bool,
    },
}

#[derive(Debug)]
pub enum Stop {
    Bad { pc: u32, bytes: [u8; 4] },
    Halt(&'static str, u32),
}

impl std::fmt::Display for Stop {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Stop::Bad { pc, bytes } => write!(f, "undefined opcode {bytes:02X?} at {pc:06X}"),
            Stop::Halt(what, pc) => write!(f, "{what} at {pc:06X}"),
        }
    }
}

pub struct Cpu {
    pub mem: Vec<u8>,
    rom: Vec<u8>,
    pub ip: u16,
    pub cycles: u64,
    pub insns: u64,
    pub fcpu: u64,
    ext: ExtMode,
    ext_cnt: u8,
    extr: bool,
    extr_cnt: u8,
    atomic: u8,
    /// Set by the EXT*/ATOMIC instruction itself: no interrupt before the
    /// next instruction.
    irq_block: bool,
    irq_check: bool,
    irqs: Vec<(u32, u8)>,
    pub events: Vec<Event>,
    // Timers: prescaler accumulators.
    acc_t: [u64; 11],
    // ADC
    pub adc: [u16; 16],
    adc_busy_until: Option<u64>,
    adc_ch: u8,
    // ASC0
    pub line: crate::line::Line,
    rx_ready: u64,
    rx_ver: u64,
    rx_edge: Option<u64>,
    rx_busy: Option<(u64, u64)>,
    tx_shift: Option<(u64, u8)>,
    tx_buf: Option<u8>,
    pub echo: bool,
    // SSC
    ssc_done: Option<(u64, u8)>,
    pub ssc_log: Vec<(u64, u8)>,
    /// (cycle, P6 latch, SSCCON, word) per SSC transmit, when enabled.
    pub ssc_trace: Option<Vec<(u64, u16, u16, u16)>>,
    /// Reads of flash-mapped addresses that look like stores (ignored).
    pub flash_writes: u64,
    pub trace: bool,
    pub prof: Option<std::collections::HashMap<u32, u64>>,
    pub watch: Vec<(u32, u32)>,
    pub watch_log: Vec<(u64, u32, u32, u16, bool)>,
    pub cur_pc: u32,
    pub watch_writes_only: bool,
    /// Toothed wheels driving capture inputs (crank, cam).
    pub wheels: Vec<Wheel>,
    /// Report CAPCOM compare matches as events.
    pub log_compares: bool,
    /// Compare matches also when software writes a CAPCOM timer (default on).
    pub compare_on_write: bool,
    /// Levels driven onto P2 input pins (wheels); reads merge them with the
    /// output latch by DP2.
    pub p2_in: u16,
}

/// A toothed wheel on a CAPCOM capture input: `teeth` positions per turn,
/// the last `missing` of them without a tooth. Each tooth gives a falling
/// edge at its leading flank and a rising edge half a pitch later.
#[derive(Clone, Debug)]
pub struct Wheel {
    /// Where the wheel's edges go.
    pub targets: Vec<Pin>,
    pub teeth: u32,
    pub missing: u32,
    /// Revolutions per minute of this wheel; 0 stops it.
    pub rpm: f64,
    /// Next tooth position and edge (false = falling) and when.
    pos: u32,
    rising: bool,
    next_at: u64,
    /// Edges delivered so far.
    pub edges: u64,
    /// A cam signal locked to this (crank) wheel over two turns.
    pub cam: Option<Cam>,
    /// Position over two turns (0..2 * teeth).
    pos720: u32,
}

/// A cam level over a 720° cycle, in crank tooth positions (0..2 * teeth,
/// position 0 is the first tooth after the gap): low from `fall` to `rise`.
#[derive(Clone, Debug)]
pub struct Cam {
    pub targets: Vec<Pin>,
    pub fall: u32,
    pub rise: u32,
}

/// Timer inputs a wheel can drive.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Pin {
    /// A CAPCOM channel's pin (unit 0 = CAPCOM1, 1 = CAPCOM2; channel 0-15).
    Cc { unit: u8, ch: u8 },
    /// T0IN (CAPCOM1 T0 counter input).
    T0In,
    /// T7IN (CAPCOM2 T7 counter input).
    T7In,
    /// GPT2 CAPIN (T5 capture into CAPREL).
    CapIn,
}

impl std::str::FromStr for Pin {
    type Err = String;
    fn from_str(s: &str) -> Result<Self, String> {
        Ok(match s.to_ascii_lowercase().as_str() {
            "t0in" => Pin::T0In,
            "t7in" => Pin::T7In,
            "capin" => Pin::CapIn,
            cc => {
                let n: u8 = cc
                    .strip_prefix("cc")
                    .and_then(|n| n.parse().ok())
                    .filter(|&n| n < 32)
                    .ok_or_else(|| format!("unknown pin {s:?} (t0in, t7in, capin, cc0..cc31)"))?;
                Pin::Cc {
                    unit: n / 16,
                    ch: n % 16,
                }
            }
        })
    }
}

impl Wheel {
    pub fn new(targets: Vec<Pin>, teeth: u32, missing: u32) -> Self {
        Self {
            targets,
            teeth,
            missing,
            rpm: 0.0,
            pos: 0,
            rising: false,
            next_at: 0,
            edges: 0,
            cam: None,
            pos720: 0,
        }
    }

    /// Half a tooth pitch in CPU cycles at the current speed.
    fn half_pitch(&self, fcpu: u64) -> Option<u64> {
        (self.rpm > 0.0)
            .then(|| (fcpu as f64 * 60.0 / (self.rpm * self.teeth as f64) / 2.0).max(1.0) as u64)
    }
}

#[derive(Clone, Copy)]
struct Flags {
    n: bool,
    c: bool,
    v: bool,
    z: bool,
    e: bool,
}

impl Cpu {
    pub fn new(rom: Vec<u8>, fcpu: u64) -> Self {
        let mut c = Cpu {
            mem: vec![0; 1 << 24],
            rom,
            ip: 0,
            cycles: 0,
            insns: 0,
            fcpu,
            ext: ExtMode::None,
            ext_cnt: 0,
            extr: false,
            extr_cnt: 0,
            atomic: 0,
            irq_block: false,
            irq_check: true,
            irqs: irq_table(),
            events: Vec::new(),
            acc_t: [0; 11],
            adc: [0x200; 16],
            adc_busy_until: None,
            adc_ch: 0,
            line: Default::default(),
            rx_ready: 0,
            rx_ver: u64::MAX,
            rx_edge: None,
            rx_busy: None,
            tx_shift: None,
            tx_buf: None,
            echo: true,
            ssc_done: None,
            ssc_log: Vec::new(),
            ssc_trace: None,
            flash_writes: 0,
            trace: false,
            prof: None,
            watch: Vec::new(),
            watch_log: Vec::new(),
            cur_pc: 0,
            watch_writes_only: false,
            wheels: Vec::new(),
            log_compares: false,
            p2_in: 0,
            compare_on_write: true,
        };
        // Reset values.
        c.put16(0xFE06, 3); // DPP3
        c.put16(0xFE04, 2);
        c.put16(0xFE02, 1);
        c.put16(SP, 0xFC00);
        c.put16(0xFE14, 0xFA00); // STKOV
        c.put16(0xFE16, 0xFC00); // STKUN
        c.put16(0xFF1E, 0xFFFF); // ONES
        c.put16(P3, 0xFFFF);
        c
    }

    // ---------------------------------------------------------------- memory

    fn is_flash(a: u32) -> bool {
        a < 0x80000 && !(0xC000..0x10000).contains(&a)
    }

    pub fn peek8(&self, a: u32) -> u8 {
        let a = a & 0xFF_FFFF;
        if Self::is_flash(a) {
            self.rom[(a as usize) & (self.rom.len() - 1)]
        } else {
            self.mem[a as usize]
        }
    }

    pub fn peek16(&self, a: u32) -> u16 {
        let a = a & !1;
        u16::from_le_bytes([self.peek8(a), self.peek8(a + 1)])
    }

    /// Store without side effects (used for SFR state the model owns).
    pub fn put16(&mut self, a: u32, v: u16) {
        let a = (a & 0xFF_FFFE) as usize;
        self.mem[a..a + 2].copy_from_slice(&v.to_le_bytes());
    }

    fn rd16(&mut self, a: u32) -> u16 {
        let a = a & 0xFF_FFFE;
        let v = match a {
            P3 => {
                let mut v = self.peek16(P3);
                let lvl = self.line.level(self.cycles);
                v = (v & !(1 << 11)) | ((lvl as u16) << 11);
                v
            }
            // input pins read their level, output pins the latch
            P2 => {
                let dp = self.peek16(0xFFC2);
                (self.peek16(P2) & dp) | (self.p2_in & !dp)
            }
            _ => self.peek16(a),
        };
        self.watch_hit(a, v, false);
        v
    }

    fn rd8(&mut self, a: u32) -> u8 {
        let w = self.rd16(a & !1);
        if a & 1 == 1 { (w >> 8) as u8 } else { w as u8 }
    }

    fn watch_hit(&mut self, a: u32, v: u16, w: bool) {
        if self.watch_writes_only && !w {
            return;
        }
        if !self.watch.is_empty() && self.watch.iter().any(|&(l, h)| a >= l && a <= h) {
            let pc = self.cur_pc;
            self.watch_log.push((self.cycles, pc, a, v, w));
        }
    }

    fn wr16(&mut self, a: u32, v: u16) {
        let a = a & 0xFF_FFFE;
        self.watch_hit(a, v, true);
        if Self::is_flash(a) {
            self.flash_writes += 1;
            return;
        }
        let old = self.peek16(a);
        self.put16(a, v);
        self.on_write(a, old, v);
    }

    fn wr8(&mut self, a: u32, v: u8) {
        let w = self.peek16(a & !1);
        let nw = if a & 1 == 1 {
            (w & 0x00FF) | ((v as u16) << 8)
        } else {
            (w & 0xFF00) | v as u16
        };
        self.wr16(a & !1, nw);
    }

    fn on_write(&mut self, a: u32, old: u16, v: u16) {
        // a CAPCOM timer written to a compare value matches like a count
        // would [assumed]. The SIMK41 code steps T7 by software across the
        // crank gap and needs CC24 to fire then; without this it drops sync
        // every few revolutions.
        if self.compare_on_write {
            match a {
                0xFE50 => self.compare_equal(0, 0, v),
                0xFE52 => self.compare_equal(0, 1, v),
                0xF050 => self.compare_equal(1, 0, v),
                0xF052 => self.compare_equal(1, 1, v),
                _ => {}
            }
        }
        match a {
            0xFF60..=0xFF9E | 0xF160..=0xF19E | PSW => self.irq_check = true,
            S0TBUF => self.asc_tx_write(v as u8),
            S0BG | S0CON => {
                if a == S0CON && (old ^ v) & 0x8010 != 0 || a == S0BG {
                    let s0bg = self.peek16(S0BG);
                    let s0con = self.peek16(S0CON);
                    self.events.push(Event::Baud {
                        at: self.cycles,
                        s0bg,
                        s0con,
                    });
                }
            }
            ADCON => {
                if v & 0x80 != 0 && self.adc_busy_until.is_none() {
                    self.adc_ch = (v & 15) as u8;
                    self.adc_busy_until = Some(self.cycles + self.fcpu / 100_000);
                    self.put16(ADCON, v | 0x100);
                }
            }
            SSCTB => {
                // Loopback-free dumb slave: answers 0xFF (no EEPROM fitted
                // in the model) after 8 bit times at the SSC rate.
                self.ssc_log.push((self.cycles, v as u8));
                let p6 = self.peek16(0xFFCC);
                let con = self.peek16(0xFFB2);
                if let Some(t) = &mut self.ssc_trace {
                    t.push((self.cycles, p6, con, v));
                }
                self.ssc_done = Some((self.cycles + 200, 0xFF));
            }
            _ => {}
        }
    }

    fn dpp(&self, i: u16) -> u32 {
        (self.peek16(0xFE00 + 2 * i as u32) & 0x3FF) as u32
    }

    /// Physical address of a 16-bit data address (mem or indirect).
    pub fn phys(&self, a: u16) -> u32 {
        match self.ext {
            ExtMode::Seg(s) => ((s as u32) << 16) | a as u32,
            ExtMode::Page(p) => ((p as u32) << 14) | (a & 0x3FFF) as u32,
            ExtMode::None => (self.dpp(a >> 14) << 14) | (a & 0x3FFF) as u32,
        }
    }

    pub fn cp(&self) -> u32 {
        self.peek16(CP) as u32
    }

    pub fn gpr(&self, n: u8) -> u16 {
        self.peek16(self.cp() + 2 * n as u32)
    }

    fn set_gpr(&mut self, n: u8, v: u16) {
        let a = self.cp() + 2 * n as u32;
        self.wr16(a, v);
    }

    fn gprb(&self, n: u8) -> u8 {
        self.peek8(self.cp() + n as u32)
    }

    fn set_gprb(&mut self, n: u8, v: u8) {
        let a = self.cp() + n as u32;
        self.wr8(a, v);
    }

    pub fn csp(&self) -> u8 {
        self.peek16(CSP) as u8
    }

    pub fn pc(&self) -> u32 {
        ((self.csp() as u32) << 16) | self.ip as u32
    }

    fn psw(&self) -> u16 {
        self.peek16(PSW)
    }

    fn set_flags(&mut self, f: Flags) {
        let p = self.psw() & !0x1F;
        let p = p
            | (f.n as u16)
            | ((f.c as u16) << 1)
            | ((f.v as u16) << 2)
            | ((f.z as u16) << 3)
            | ((f.e as u16) << 4);
        self.put16(PSW, p);
    }

    // ------------------------------------------------------------ operands

    fn reg_rd(&mut self, r: u8, b: bool) -> u16 {
        if r >= 0xF0 {
            if b {
                self.gprb(r & 15) as u16
            } else {
                self.gpr(r & 15)
            }
        } else {
            let a = reg_addr(r, self.extr) as u32;
            if b { self.rd8(a) as u16 } else { self.rd16(a) }
        }
    }

    fn reg_wr(&mut self, r: u8, b: bool, v: u16) {
        if r >= 0xF0 {
            if b {
                self.set_gprb(r & 15, v as u8)
            } else {
                self.set_gpr(r & 15, v)
            }
        } else {
            let a = reg_addr(r, self.extr) as u32;
            if b {
                self.wr8(a, v as u8)
            } else {
                self.wr16(a, v)
            }
        }
    }

    fn mrd(&mut self, a: u16, b: bool) -> u16 {
        let p = self.phys(a);
        if b { self.rd8(p) as u16 } else { self.rd16(p) }
    }

    fn mwr(&mut self, a: u16, b: bool, v: u16) {
        let p = self.phys(a);
        if b {
            self.wr8(p, v as u8)
        } else {
            self.wr16(p, v)
        }
    }

    fn rd_op(&mut self, o: Op, b: bool) -> u16 {
        let step = if b { 1 } else { 2 };
        match o {
            Op::Rw(n) => self.gpr(n),
            Op::Rb(n) => self.gprb(n) as u16,
            Op::Reg(r) => self.reg_rd(r, b),
            Op::Mem(a) => self.mrd(a, b),
            Op::Ind(n) => {
                let a = self.gpr(n);
                self.mrd(a, b)
            }
            Op::PostInc(n) => {
                let a = self.gpr(n);
                let v = self.mrd(a, b);
                self.set_gpr(n, a.wrapping_add(step));
                v
            }
            Op::PreDec(n) => {
                let a = self.gpr(n).wrapping_sub(step);
                self.set_gpr(n, a);
                self.mrd(a, b)
            }
            Op::Off(n, d) => {
                let a = self.gpr(n).wrapping_add(d);
                self.mrd(a, b)
            }
            Op::Imm(v) => v,
        }
    }

    fn wr_op(&mut self, o: Op, b: bool, v: u16) {
        let step = if b { 1 } else { 2 };
        match o {
            Op::Rw(n) => self.set_gpr(n, v),
            Op::Rb(n) => self.set_gprb(n, v as u8),
            Op::Reg(r) => self.reg_wr(r, b, v),
            Op::Mem(a) => self.mwr(a, b, v),
            Op::Ind(n) => {
                let a = self.gpr(n);
                self.mwr(a, b, v)
            }
            Op::PostInc(n) => {
                let a = self.gpr(n);
                self.mwr(a, b, v);
                self.set_gpr(n, a.wrapping_add(step));
            }
            Op::PreDec(n) => {
                let a = self.gpr(n).wrapping_sub(step);
                self.set_gpr(n, a);
                self.mwr(a, b, v)
            }
            Op::Off(n, d) => {
                let a = self.gpr(n).wrapping_add(d);
                self.mwr(a, b, v)
            }
            Op::Imm(_) => {}
        }
    }

    fn bit_addr(&self, q: u8) -> Option<u32> {
        if q >= 0xF0 {
            None
        } else {
            Some(bitoff_addr(q, self.extr) as u32)
        }
    }

    fn bitword_rd(&mut self, q: u8) -> u16 {
        match self.bit_addr(q) {
            None => self.gpr(q & 15),
            Some(a) => self.rd16(a),
        }
    }

    fn bitword_wr(&mut self, q: u8, v: u16) {
        match self.bit_addr(q) {
            None => self.set_gpr(q & 15, v),
            Some(a) => self.wr16(a, v),
        }
    }

    fn push(&mut self, v: u16) {
        let sp = self.peek16(SP).wrapping_sub(2);
        self.put16(SP, sp);
        self.wr16(sp as u32, v);
    }

    fn pop(&mut self) -> u16 {
        let sp = self.peek16(SP);
        let v = self.rd16(sp as u32);
        self.put16(SP, sp.wrapping_add(2));
        v
    }

    fn segmented(&self) -> bool {
        self.peek16(SYSCON) & (1 << 11) == 0
    }

    fn cond(&self, cc: u8) -> bool {
        let p = self.psw();
        let (n, c, v, z, e) = (p & N != 0, p & C != 0, p & V != 0, p & Z != 0, p & E != 0);
        match cc {
            0 => true,
            1 => !z && !e,
            2 => z,
            3 => !z,
            4 => v,
            5 => !v,
            6 => n,
            7 => !n,
            8 => c,
            9 => !c,
            0xA => !(z || (n ^ v)),
            0xB => z || (n ^ v),
            0xC => n ^ v,
            0xD => !(n ^ v),
            0xE => !(c || z),
            _ => c || z,
        }
    }

    // ------------------------------------------------------------ execution

    pub fn fetch4(&self, pc: u32) -> [u8; 4] {
        [
            self.peek8(pc),
            self.peek8(pc + 1),
            self.peek8(pc + 2),
            self.peek8(pc + 3),
        ]
    }

    pub fn disasm_at(&self, pc: u32) -> (u8, String) {
        let d = decode(pc as u16, self.fetch4(pc));
        (d.len, isa::Fmt { d: &d, ext: false }.to_string())
    }

    /// Execute one instruction (or take one interrupt). Returns cycles used.
    pub fn step(&mut self) -> Result<u32, Stop> {
        if self.irq_check
            && !self.irq_block
            && self.ext_cnt == 0
            && self.atomic == 0
            && self.extr_cnt == 0
            && self.take_irq()
        {
            self.tick(4);
            return Ok(4);
        }
        self.irq_block = false;
        let pc = self.pc();
        self.cur_pc = pc;
        let bytes = self.fetch4(pc);
        let d = decode(self.ip, bytes);
        if self.trace {
            let s = isa::Fmt {
                d: &d,
                ext: self.extr,
            }
            .to_string();
            println!(
                "{:10} {:06X}  {:<11} {s}",
                self.cycles,
                pc,
                bytes[..d.len as usize]
                    .iter()
                    .map(|b| format!("{b:02X}"))
                    .collect::<Vec<_>>()
                    .join(" ")
            );
        }
        if let Some(p) = &mut self.prof {
            *p.entry(pc).or_default() += 1;
        }
        self.ip = self.ip.wrapping_add(d.len as u16);
        let in_ext = self.ext_cnt > 0;
        let in_extr = self.extr_cnt > 0;
        let in_atomic = self.atomic > 0;
        let mut cyc = d.len as u32;
        match d.insn {
            Insn::Bad => {
                return Err(Stop::Bad { pc, bytes });
            }
            Insn::Ext { kind, cnt, arg } => {
                let v = match arg {
                    Op::Rw(n) => self.gpr(n),
                    Op::Imm(v) => v,
                    _ => 0,
                };
                self.ext = match kind {
                    Ext::Exts | Ext::Extsr => ExtMode::Seg(v as u8),
                    _ => ExtMode::Page(v & 0x3FF),
                };
                self.ext_cnt = cnt;
                if matches!(kind, Ext::Extsr | Ext::Extpr) {
                    self.extr = true;
                    self.extr_cnt = cnt;
                }
                self.irq_block = true;
                self.tick(cyc);
                return Ok(cyc);
            }
            Insn::Extr(cnt) => {
                self.extr = true;
                self.extr_cnt = cnt;
                self.irq_block = true;
                self.tick(cyc);
                return Ok(cyc);
            }
            Insn::Atomic(cnt) => {
                self.atomic = cnt;
                self.irq_block = true;
                self.tick(cyc);
                return Ok(cyc);
            }
            _ => cyc += self.exec(d.insn)?,
        }
        if in_ext {
            self.ext_cnt -= 1;
            if self.ext_cnt == 0 {
                self.ext = ExtMode::None;
            }
        }
        if in_extr {
            self.extr_cnt -= 1;
            if self.extr_cnt == 0 {
                self.extr = false;
            }
        }
        if in_atomic {
            self.atomic -= 1;
        }
        self.insns += 1;
        self.tick(cyc);
        Ok(cyc)
    }

    fn jump_far(&mut self, seg: u8, t: u16) {
        self.put16(CSP, seg as u16);
        self.ip = t;
    }

    /// Returns extra cycles.
    fn exec(&mut self, insn: Insn) -> Result<u32, Stop> {
        match insn {
            Insn::Alu { op, b, d, s } => {
                let sv = self.rd_op(s, b);
                let dv = self.rd_op(d, b);
                let p = self.psw();
                let (r, mut f) = alu(op, b, dv, sv, p & C != 0);
                if matches!(op, Alu::Addc | Alu::Subc) {
                    f.z = f.z && p & Z != 0;
                }
                self.set_flags(f);
                if op != Alu::Cmp {
                    self.wr_op(d, b, r);
                }
            }
            Insn::Mov { b, d, s } => {
                let v = self.rd_op(s, b);
                self.wr_op(d, b, v);
                let (msb, mask) = if b { (0x80, 0xFF) } else { (0x8000, 0xFFFF) };
                let v = v & mask;
                let p = self.psw();
                self.set_flags(Flags {
                    n: v & msb != 0,
                    c: p & C != 0,
                    v: p & V != 0,
                    z: v == 0,
                    e: v == msb,
                });
            }
            Insn::Movx { signed, d, s } => {
                let v = self.rd_op(s, true) as u8;
                let w = if signed {
                    v as i8 as i16 as u16
                } else {
                    v as u16
                };
                self.wr_op(d, false, w);
                let p = self.psw();
                self.set_flags(Flags {
                    n: w & 0x8000 != 0,
                    c: p & C != 0,
                    v: p & V != 0,
                    z: w == 0,
                    e: v == 0x80,
                });
            }
            Insn::Cmpid { delta, n, s } => {
                let sv = self.rd_op(s, false);
                let a = self.gpr(n);
                let (_, f) = alu(Alu::Cmp, false, a, sv, false);
                self.set_flags(f);
                self.set_gpr(n, a.wrapping_add(delta as i16 as u16));
            }
            Insn::Neg { b, n } => {
                let (a, op) = if b {
                    (self.gprb(n) as u16, Op::Rb(n))
                } else {
                    (self.gpr(n), Op::Rw(n))
                };
                let (r, f) = alu(Alu::Sub, b, 0, a, false);
                self.set_flags(f);
                self.wr_op(op, b, r);
            }
            Insn::Cpl { b, n } => {
                let (a, op, msb) = if b {
                    (self.gprb(n) as u16, Op::Rb(n), 0x80)
                } else {
                    (self.gpr(n), Op::Rw(n), 0x8000)
                };
                let r = if b { !a & 0xFF } else { !a };
                self.wr_op(op, b, r);
                let p = self.psw();
                self.set_flags(Flags {
                    n: r & msb != 0,
                    c: p & C != 0,
                    v: p & V != 0,
                    z: r == 0,
                    e: a == msb,
                });
            }
            Insn::Mul { signed, n, m } => {
                let (a, b) = (self.gpr(n), self.gpr(m));
                let r: u32 = if signed {
                    (a as i16 as i32 * b as i16 as i32) as u32
                } else {
                    a as u32 * b as u32
                };
                self.put16(MDH, (r >> 16) as u16);
                self.put16(MDL, r as u16);
                let v = if signed {
                    (r as i32) != (r as i16 as i32)
                } else {
                    r > 0xFFFF
                };
                self.set_flags(Flags {
                    n: r & 0x8000_0000 != 0,
                    c: false,
                    v,
                    z: r == 0,
                    e: false,
                });
                return Ok(8);
            }
            Insn::Div { kind, n } => {
                let dv = self.gpr(n);
                let md = ((self.peek16(MDH) as u32) << 16) | self.peek16(MDL) as u32;
                let mut ovf = dv == 0;
                let (q, r): (u16, u16) = if ovf {
                    (0, 0)
                } else {
                    match kind {
                        Div::Div => {
                            let a = self.peek16(MDL) as i16 as i32;
                            let b = dv as i16 as i32;
                            ((a / b) as u16, (a % b) as u16)
                        }
                        Div::Divu => {
                            let a = self.peek16(MDL);
                            (a / dv, a % dv)
                        }
                        Div::Divl => {
                            let a = md as i32 as i64;
                            let b = dv as i16 as i64;
                            let q = a / b;
                            ovf = q != q as i16 as i64;
                            (q as u16, (a % b) as u16)
                        }
                        Div::Divlu => {
                            let q = md / dv as u32;
                            ovf = q > 0xFFFF;
                            (q as u16, (md % dv as u32) as u16)
                        }
                    }
                };
                if dv != 0 {
                    self.put16(MDL, q);
                    self.put16(MDH, r);
                }
                self.set_flags(Flags {
                    n: q & 0x8000 != 0,
                    c: false,
                    v: ovf,
                    z: q == 0,
                    e: false,
                });
                return Ok(18);
            }
            Insn::Shift { kind, n, s } => {
                let cnt = (self.rd_op(s, false) & 15) as u32;
                let a = self.gpr(n);
                let (r, c, v) = if cnt == 0 {
                    (a, false, false)
                } else {
                    match kind {
                        Shift::Shl => (a << cnt, (a >> (16 - cnt)) & 1 != 0, false),
                        Shift::Shr => (
                            a >> cnt,
                            (a >> (cnt - 1)) & 1 != 0,
                            cnt > 1 && a & ((1 << (cnt - 1)) - 1) != 0,
                        ),
                        Shift::Ashr => (
                            ((a as i16) >> cnt) as u16,
                            (a >> (cnt - 1)) & 1 != 0,
                            cnt > 1 && a & ((1 << (cnt - 1)) - 1) != 0,
                        ),
                        Shift::Rol => {
                            let r = a.rotate_left(cnt);
                            (r, r & 1 != 0, false)
                        }
                        Shift::Ror => {
                            let r = a.rotate_right(cnt);
                            (
                                r,
                                r & 0x8000 != 0,
                                cnt > 1 && a & ((1 << (cnt - 1)) - 1) != 0,
                            )
                        }
                    }
                };
                self.set_gpr(n, r);
                self.set_flags(Flags {
                    n: r & 0x8000 != 0,
                    c,
                    v,
                    z: r == 0,
                    e: false,
                });
            }
            Insn::Prior { n, m } => {
                let a = self.gpr(m);
                let r = if a == 0 { 0 } else { a.leading_zeros() as u16 };
                self.set_gpr(n, r);
                self.set_flags(Flags {
                    n: false,
                    c: false,
                    v: false,
                    z: a == 0,
                    e: false,
                });
            }
            Insn::Bclr { q, bit } | Insn::Bset { q, bit } => {
                let set = matches!(insn, Insn::Bset { .. });
                let w = self.bitword_rd(q);
                let old = w & (1 << bit) != 0;
                let nw = if set { w | (1 << bit) } else { w & !(1 << bit) };
                self.bitword_wr(q, nw);
                self.set_flags(Flags {
                    n: old,
                    c: false,
                    v: false,
                    z: !old,
                    e: false,
                });
            }
            Insn::Bit2 { op, zq, z, qq, q } => {
                let src = self.bitword_rd(qq) & (1 << q) != 0;
                let dw = self.bitword_rd(zq);
                let dst = dw & (1 << z) != 0;
                let r = match op {
                    BitOp::Bmov => Some(src),
                    BitOp::Bmovn => Some(!src),
                    BitOp::Band => Some(dst & src),
                    BitOp::Bor => Some(dst | src),
                    BitOp::Bxor => Some(dst ^ src),
                    BitOp::Bcmp => None,
                };
                if let Some(r) = r {
                    let nw = if r { dw | (1 << z) } else { dw & !(1 << z) };
                    self.bitword_wr(zq, nw);
                }
                let f = match op {
                    BitOp::Bmov | BitOp::Bmovn => {
                        let s = if op == BitOp::Bmov { src } else { !src };
                        Flags {
                            n: s,
                            c: false,
                            v: false,
                            z: !s,
                            e: false,
                        }
                    }
                    _ => Flags {
                        n: dst ^ src,
                        c: dst & src,
                        v: dst | src,
                        z: !(dst | src),
                        e: false,
                    },
                };
                self.set_flags(f);
            }
            Insn::Bfld {
                high,
                q,
                mask,
                data,
            } => {
                let w = self.bitword_rd(q);
                let (m, d) = if high {
                    ((mask as u16) << 8, (data as u16) << 8)
                } else {
                    (mask as u16, data as u16)
                };
                let nw = (w & !m) | (d & m);
                self.bitword_wr(q, nw);
                let p = self.psw();
                self.set_flags(Flags {
                    n: w & 0x8000 != 0,
                    c: p & C != 0,
                    v: false,
                    z: w & m == 0,
                    e: false,
                });
            }
            Insn::Jmpr { cc, t } | Insn::Jmpa { cc, t } => {
                if self.cond(cc) {
                    self.ip = t;
                    return Ok(2);
                }
            }
            Insn::Jmps { seg, t } => {
                self.jump_far(seg, t);
                return Ok(2);
            }
            Insn::Jmpi { cc, n } => {
                if self.cond(cc) {
                    self.ip = self.gpr(n);
                    return Ok(2);
                }
            }
            Insn::Jbit { kind, q, bit, t } => {
                let w = self.bitword_rd(q);
                let set = w & (1 << bit) != 0;
                let take = match kind {
                    Jb::Jb => set,
                    Jb::Jnb => !set,
                    Jb::Jbc => {
                        if set {
                            self.bitword_wr(q, w & !(1 << bit));
                        }
                        set
                    }
                    Jb::Jnbs => {
                        if !set {
                            self.bitword_wr(q, w | (1 << bit));
                        }
                        !set
                    }
                };
                if take {
                    self.ip = t;
                    return Ok(2);
                }
            }
            Insn::Calla { cc, t } => {
                if self.cond(cc) {
                    let ip = self.ip;
                    self.push(ip);
                    self.ip = t;
                    return Ok(2);
                }
            }
            Insn::Calls { seg, t } => {
                let csp = self.csp() as u16;
                self.push(csp);
                let ip = self.ip;
                self.push(ip);
                self.jump_far(seg, t);
                return Ok(2);
            }
            Insn::Calli { cc, n } => {
                if self.cond(cc) {
                    let ip = self.ip;
                    self.push(ip);
                    self.ip = self.gpr(n);
                    return Ok(2);
                }
            }
            Insn::Callr { t } => {
                let ip = self.ip;
                self.push(ip);
                self.ip = t;
                return Ok(2);
            }
            Insn::Pcall { r, t } => {
                let v = self.reg_rd(r, false);
                self.push(v);
                let ip = self.ip;
                self.push(ip);
                self.ip = t;
                return Ok(2);
            }
            Insn::Ret => {
                self.ip = self.pop();
                return Ok(2);
            }
            Insn::Rets => {
                self.ip = self.pop();
                let s = self.pop();
                self.put16(CSP, s & 0xFF);
                return Ok(2);
            }
            Insn::Retp(r) => {
                self.ip = self.pop();
                let v = self.pop();
                self.reg_wr(r, false, v);
                return Ok(2);
            }
            Insn::Reti => {
                self.ip = self.pop();
                if self.segmented() {
                    let s = self.pop();
                    self.put16(CSP, s & 0xFF);
                }
                let p = self.pop();
                self.put16(PSW, p);
                self.irq_check = true;
                return Ok(2);
            }
            Insn::Trap(n) => {
                let p = self.psw();
                self.push(p);
                if self.segmented() {
                    let s = self.csp() as u16;
                    self.push(s);
                }
                let ip = self.ip;
                self.push(ip);
                self.jump_far(0, n as u16 * 4);
                return Ok(2);
            }
            Insn::Push(r) => {
                let v = self.reg_rd(r, false);
                self.push(v);
            }
            Insn::Pop(r) => {
                let v = self.pop();
                self.reg_wr(r, false, v);
                if r == 0x88 {
                    self.irq_check = true;
                }
            }
            Insn::Scxt { r, s } => {
                let nv = self.rd_op(s, false);
                let v = self.reg_rd(r, false);
                self.push(v);
                self.reg_wr(r, false, nv);
            }
            Insn::Nop => {}
            Insn::Sys(s) => match s {
                "SRST" => return Err(Stop::Halt("SRST", self.cur_pc)),
                "IDLE" | "PWRDN" => return Err(Stop::Halt(s, self.cur_pc)),
                _ => {}
            },
            Insn::Ext { .. } | Insn::Extr(_) | Insn::Atomic(_) | Insn::Bad => unreachable!(),
        }
        Ok(0)
    }

    fn take_irq(&mut self) -> bool {
        let psw = self.psw();
        let cur = (psw >> 12) as u8;
        let ien = psw & (1 << 11) != 0;
        let mut best: Option<(u8, u8, u32, u8)> = None;
        let mut any = false;
        for &(ic, trap) in &self.irqs {
            let v = self.peek16(ic);
            if v & 0xC0 != 0xC0 {
                continue;
            }
            any = true;
            let lvl = ((v >> 2) & 15) as u8;
            let g = (v & 3) as u8;
            if best.is_none_or(|(bl, bg, _, _)| (lvl, g) > (bl, bg)) {
                best = Some((lvl, g, ic, trap));
            }
        }
        // PEC requests (ILVL 14/15) are serviced regardless of IEN.
        if let Some((lvl, g, ic, trap)) = best {
            if lvl >= 14 {
                let ch = g as u32 + if lvl == 15 { 4 } else { 0 };
                let pecc = self.peek16(0xFEC0 + 2 * ch);
                if pecc & 0xFF != 0 && self.pec(ch, pecc) {
                    // IR stays set when the count runs out.
                    return true;
                }
            }
            if ien && lvl > cur {
                let v = self.peek16(ic);
                self.put16(ic, v & !0x80);
                self.push(psw);
                if self.segmented() {
                    let s = self.csp() as u16;
                    self.push(s);
                }
                let ip = self.ip;
                self.push(ip);
                let np = (psw & 0x0FFF & !(1 << 6)) | ((lvl as u16) << 12);
                self.put16(PSW, np);
                self.jump_far(0, trap as u16 * 4);
                return true;
            }
        }
        if !any {
            self.irq_check = false;
        }
        false
    }

    /// One PEC transfer. Returns true if it consumed the request.
    fn pec(&mut self, ch: u32, pecc: u16) -> bool {
        let src_p = 0xFCE0 + 4 * ch;
        let dst_p = src_p + 2;
        let src = self.peek16(src_p);
        let dst = self.peek16(dst_p);
        let byte = pecc & 0x100 != 0;
        if byte {
            let v = self.rd8(src as u32);
            self.wr8(dst as u32, v);
        } else {
            let v = self.rd16(src as u32);
            self.wr16(dst as u32, v);
        }
        let step = if byte { 1 } else { 2 };
        match (pecc >> 9) & 3 {
            1 => self.put16(dst_p, dst.wrapping_add(step)),
            2 => self.put16(src_p, src.wrapping_add(step)),
            _ => {}
        }
        let cnt = pecc & 0xFF;
        if cnt != 0xFF {
            let nc = cnt - 1;
            self.put16(0xFEC0 + 2 * ch, (pecc & 0xFF00) | nc);
            if nc == 0 {
                // Leave IR set: the CPU takes the interrupt next.
                return true;
            }
        }
        // Clear the request that triggered the transfer.
        for i in 0..self.irqs.len() {
            let ic = self.irqs[i].0;
            let v = self.peek16(ic);
            if v & 0xC0 == 0xC0
                && ((v >> 2) & 15) >= 14
                && (v & 3) as u32 + if (v >> 2) & 15 == 15 { 4 } else { 0 } == ch
            {
                self.put16(ic, v & !0x80);
                break;
            }
        }
        true
    }

    fn set_ir(&mut self, ic: u32) {
        let v = self.peek16(ic);
        self.put16(ic, v | 0x80);
        self.irq_check = true;
    }

    // ---------------------------------------------------------- peripherals

    /// One bit time in CPU clocks: fCPU / (32 or 48 * (S0BG + 1)) baud.
    pub fn bit_cycles(&self) -> u64 {
        let bg = self.peek16(S0BG) as u64 + 1;
        let div = if self.peek16(S0CON) & (1 << 13) != 0 {
            48
        } else {
            32
        };
        bg * div
    }

    fn asc_tx_write(&mut self, b: u8) {
        if self.tx_shift.is_none() {
            self.tx_start(b);
        } else {
            self.tx_buf = Some(b);
        }
    }

    fn tx_start(&mut self, b: u8) {
        let bit = self.bit_cycles();
        let stops = if self.peek16(S0CON) & 8 != 0 { 2 } else { 1 };
        let t = self.cycles;
        if self.echo {
            self.line.frame(t, b, bit);
        }
        self.tx_shift = Some((t + (9 + stops) * bit, b));
        self.set_ir(S0TBIC);
    }

    fn tick(&mut self, cyc: u32) {
        let cyc = cyc as u64;
        self.cycles += cyc;
        self.tick_capcom(cyc);
        self.tick_gpt(cyc);
        self.tick_wheels();
        // ADC
        if let Some(t) = self.adc_busy_until
            && self.cycles >= t
        {
            let con = self.peek16(ADCON);
            let ch = self.adc_ch;
            let val = self.adc[ch as usize] & 0x3FF;
            self.put16(ADDAT, ((ch as u16) << 12) | val);
            self.set_ir(ADCIC);
            let mode = (con >> 4) & 3;
            match mode {
                0 => {
                    self.adc_busy_until = None;
                    self.put16(ADCON, con & !0x180);
                }
                1 => self.adc_busy_until = Some(self.cycles + self.fcpu / 100_000),
                _ => {
                    // auto scan: ADCH down to 0
                    if ch == 0 {
                        if mode == 3 {
                            self.adc_ch = (con & 15) as u8;
                            self.adc_busy_until = Some(self.cycles + self.fcpu / 100_000);
                        } else {
                            self.adc_busy_until = None;
                            self.put16(ADCON, con & !0x180);
                        }
                    } else {
                        self.adc_ch = ch - 1;
                        self.adc_busy_until = Some(self.cycles + self.fcpu / 100_000);
                    }
                }
            }
        }
        // ASC0 transmit
        if let Some((end, b)) = self.tx_shift
            && self.cycles >= end
        {
            self.events.push(Event::Tx { at: end, byte: b });
            self.set_ir(S0TIC);
            self.tx_shift = None;
            if let Some(nb) = self.tx_buf.take() {
                self.tx_start(nb);
            }
        }
        // ASC0 receive: sample the line like the UART does.
        self.tick_rx();
        // SSC
        if let Some((t, b)) = self.ssc_done
            && self.cycles >= t
        {
            self.put16(SSCRB, b as u16);
            self.set_ir(SSCRIC);
            self.set_ir(SSCTIC);
            self.ssc_done = None;
        }
    }

    fn tick_rx(&mut self) {
        let con = self.peek16(S0CON);
        let on = con & 0x8000 != 0;
        if self.rx_busy.is_none() {
            if self.rx_ver != self.line.version {
                self.rx_ver = self.line.version;
                self.rx_edge = self.line.next_fall(self.rx_ready);
            }
            if let Some(e) = self.rx_edge
                && self.cycles >= e
                && on
            {
                self.rx_busy = Some((e, self.bit_cycles()));
                self.rx_edge = None;
                self.rx_ver = u64::MAX;
            } else if let Some(e) = self.rx_edge
                && self.cycles >= e
            {
                // Receiver off: skip this edge.
                self.rx_ready = e + 1;
                self.rx_ver = u64::MAX;
            }
        }
        let Some((s, bit)) = self.rx_busy else {
            return;
        };
        let done = s + 9 * bit + bit / 2;
        if self.cycles < done {
            return;
        }
        self.rx_busy = None;
        self.rx_ready = done;
        self.rx_ver = u64::MAX;
        self.line.forget_before(s.saturating_sub(1_000_000));
        let mut v = 0u8;
        for k in 0..8 {
            if self.line.level(s + (k + 1) * bit + bit / 2) {
                v |= 1 << k;
            }
        }
        let fe = !self.line.level(done);
        let enabled = on && con & 0x10 != 0;
        self.events.push(Event::Rx {
            at: done,
            byte: v,
            fe,
            enabled,
        });
        if !enabled {
            return;
        }
        self.put16(S0RBUF, v as u16);
        let mut ncon = con;
        if fe && con & 0x40 != 0 {
            ncon |= 1 << 9;
            self.set_ir(S0EIC);
        }
        if self.peek16(S0RIC) & 0x80 != 0 && con & 0x80 != 0 {
            ncon |= 1 << 10;
            self.set_ir(S0EIC);
        }
        self.put16(S0CON, ncon);
        self.set_ir(S0RIC);
    }

    fn tick_wheels(&mut self) {
        for i in 0..self.wheels.len() {
            loop {
                let w = &self.wheels[i];
                let Some(half) = w.half_pitch(self.fcpu) else {
                    // stopped: start timing afresh when it turns again
                    self.wheels[i].next_at = 0;
                    break;
                };
                if w.next_at == 0 {
                    self.wheels[i].next_at = self.cycles + half;
                    break;
                }
                if w.next_at > self.cycles {
                    break;
                }
                let (rising, pos) = (w.rising, w.pos);
                let tooth = pos < w.teeth - w.missing;
                if tooth {
                    for p in self.wheels[i].targets.clone() {
                        self.pin_edge(p, rising);
                    }
                    self.wheels[i].edges += 1;
                }
                let w = &mut self.wheels[i];
                if rising {
                    w.pos = (w.pos + 1) % w.teeth;
                    w.pos720 = (w.pos720 + 1) % (2 * w.teeth);
                }
                w.rising = !rising;
                w.next_at += half;
                // cam edges at whole tooth positions (on the leading flank)
                if !rising && let Some(cam) = &w.cam {
                    let p = w.pos720;
                    let edge = if p == cam.fall {
                        Some(false)
                    } else if p == cam.rise {
                        Some(true)
                    } else {
                        None
                    };
                    if let Some(level) = edge {
                        for t in cam.targets.clone() {
                            self.pin_edge(t, level);
                        }
                    }
                }
            }
        }
    }

    /// An edge on a timer input pin.
    pub fn pin_edge(&mut self, pin: Pin, rising: bool) {
        let src = if rising { 1 } else { 2 };
        // CAPCOM1 pins are P2.0-P2.15 (T7IN shares P2.15): the code also
        // reads their levels
        let p2_bit = match pin {
            Pin::Cc { unit: 0, ch } => Some(ch),
            Pin::T7In => Some(15),
            _ => None,
        };
        if let Some(b) = p2_bit {
            if rising {
                self.p2_in |= 1 << b;
            } else {
                self.p2_in &= !(1 << b);
            }
        }
        match pin {
            Pin::Cc { unit, ch } => self.capture_edge(unit, ch, rising),
            Pin::T0In => self.capcom_count(0, src),
            Pin::T7In => self.capcom_count(1, src),
            Pin::CapIn => self.capin_edge(rising),
        }
    }

    /// An edge on a CAPCOM pin: a channel in capture mode for that edge
    /// latches its timer into CCx and requests its interrupt.
    pub fn capture_edge(&mut self, unit: u8, ch: u8, rising: bool) {
        const CC2_IC: [u32; 16] = [
            0xF160, 0xF162, 0xF164, 0xF166, 0xF168, 0xF16A, 0xF16C, 0xF16E, 0xF170, 0xF172, 0xF174,
            0xF176, 0xF178, 0xF184, 0xF18C, 0xF194,
        ];
        let ch = (ch & 15) as u32;
        let (t_a, ccm_a, cc_a) = if unit == 0 {
            (0xFE50, 0xFF52, 0xFE80)
        } else {
            (0xF050, 0xFF22, 0xFE60)
        };
        let f = (self.peek16(ccm_a + 2 * (ch / 4)) >> (4 * (ch % 4))) & 15;
        let hit = match f & 7 {
            1 => rising,
            2 => !rising,
            3 => true,
            _ => false,
        };
        if !hit {
            return;
        }
        let t = self.peek16(t_a + 2 * ((f >> 3) as u32));
        self.put16(cc_a + 2 * ch, t);
        let ic = if unit == 0 {
            0xFF78 + 2 * ch
        } else {
            CC2_IC[ch as usize]
        };
        self.set_ir(ic);
    }

    fn capcom_regs(unit: usize) -> (u32, u32, u32, u32, u32, [u32; 2]) {
        if unit == 0 {
            (0xFF50, 0xFE50, 0xFE54, 0xFF52, 0xFE80, [0xFF9C, 0xFF9E])
        } else {
            (0xFF20, 0xF050, 0xF054, 0xFF22, 0xFE60, [0xF17A, 0xF17C])
        }
    }

    fn tick_capcom(&mut self, cyc: u64) {
        // CAPCOM1: T01CON, T0/T1, T0REL/T1REL, CCM0-3, CC0-15.
        // CAPCOM2: T78CON, T7/T8 (ESFR), T7REL/T8REL, CCM4-7, CC16-31.
        // Timers in counter mode (TxM) count input events instead
        // (`capcom_count`).
        for unit in 0..2 {
            let con = self.peek16(Self::capcom_regs(unit).0);
            for t in 0..2 {
                let c = con >> (8 * t);
                if c & 0x40 == 0 || c & 0x08 != 0 {
                    continue;
                }
                let pre = 8u64 << (c & 7);
                let ai = 7 + unit * 2 + t;
                self.acc_t[ai] += cyc;
                let inc = self.acc_t[ai] / pre;
                if inc == 0 {
                    continue;
                }
                self.acc_t[ai] %= pre;
                self.capcom_inc(unit, t, inc);
            }
        }
    }

    /// A count event for CAPCOM timers in counter mode. `source`: 0 = GPT2
    /// T6 over/underflow, 1 = rising and 2 = falling edge on the timer's
    /// input pin (T0IN, T7IN). Only T0 and T7 have a counter input.
    fn capcom_count(&mut self, unit: usize, source: u8) {
        let c = self.peek16(Self::capcom_regs(unit).0) & 0xFF;
        if c & 0x40 == 0 || c & 0x08 == 0 {
            return;
        }
        let hit = matches!((c & 3, source), (0, 0) | (1, 1) | (2, 2) | (3, 1) | (3, 2));
        if hit {
            self.capcom_inc(unit, 0, 1);
        }
    }

    /// Compare channels of timer `t` in `unit` whose CC register equals `v`.
    fn compare_equal(&mut self, unit: usize, t: usize, v: u16) {
        const CC2_IC: [u32; 16] = [
            0xF160, 0xF162, 0xF164, 0xF166, 0xF168, 0xF16A, 0xF16C, 0xF16E, 0xF170, 0xF172, 0xF174,
            0xF176, 0xF178, 0xF184, 0xF18C, 0xF194,
        ];
        let (_, _, _, ccm_a, cc_a, _) = Self::capcom_regs(unit);
        for ch in 0..16u32 {
            let f = (self.peek16(ccm_a + 2 * (ch / 4)) >> (4 * (ch % 4))) & 15;
            if f & 7 < 4 || ((f >> 3) as usize) != t || self.peek16(cc_a + 2 * ch) != v {
                continue;
            }
            let ic = if unit == 0 {
                0xFF78 + 2 * ch
            } else {
                CC2_IC[ch as usize]
            };
            self.set_ir(ic);
        }
    }

    fn capcom_inc(&mut self, unit: usize, t: usize, inc: u64) {
        const CC2_IC: [u32; 16] = [
            0xF160, 0xF162, 0xF164, 0xF166, 0xF168, 0xF16A, 0xF16C, 0xF16E, 0xF170, 0xF172, 0xF174,
            0xF176, 0xF178, 0xF184, 0xF18C, 0xF194,
        ];
        let (_, t_a, rel_a, ccm_a, cc_a, tic) = Self::capcom_regs(unit);
        let ta = t_a + 2 * t as u32;
        let old = self.peek16(ta);
        let mut new = old as u64 + inc;
        let mut ovf = false;
        if new > 0xFFFF {
            ovf = true;
            let rel = self.peek16(rel_a + 2 * t as u32) as u64;
            new = rel + (new - 0x10000) % (0x10000 - rel).max(1);
        }
        self.put16(ta, new as u16);
        if ovf {
            self.set_ir(tic[t]);
        }
        for ch in 0..16u32 {
            let ccm = self.peek16(ccm_a + 2 * (ch / 4));
            let f = (ccm >> (4 * (ch % 4))) & 15;
            if f & 7 < 4 || ((f >> 3) as usize) != t {
                continue;
            }
            let cv = self.peek16(cc_a + 2 * ch);
            let hit = if !ovf {
                cv > old && cv as u64 <= new
            } else {
                cv > old || cv as u64 <= new
            };
            if hit {
                let ic = if unit == 0 {
                    0xFF78 + 2 * ch
                } else {
                    CC2_IC[ch as usize]
                };
                self.set_ir(ic);
                if self.log_compares {
                    self.events.push(Event::Compare {
                        at: self.cycles,
                        unit: unit as u8,
                        ch: ch as u8,
                        mode: (f & 7) as u8,
                    });
                }
            }
        }
    }

    /// An edge on GPT2's CAPIN pin: with T5SC set and a matching CI edge,
    /// CAPREL latches T5 (T5 is cleared if T5CLR) and CRIR is set.
    pub fn capin_edge(&mut self, rising: bool) {
        let con = self.peek16(0xFF46);
        if con & 0x8000 == 0 {
            return;
        }
        let hit = match (con >> 12) & 3 {
            1 => rising,
            2 => !rising,
            3 => true,
            _ => false,
        };
        if !hit {
            return;
        }
        let t5 = self.peek16(0xFE46);
        self.put16(0xFE4A, t5);
        if con & 0x4000 != 0 {
            self.put16(0xFE46, 0);
        }
        self.set_ir(0xFF6A);
    }

    fn tick_gpt(&mut self, cyc: u64) {
        // (con, timer, ic, base prescaler, acc idx)
        let cfg: [(u32, u32, u32, u64, usize); 5] = [
            (0xFF40, 0xFE40, 0xFF60, 8, 2),
            (0xFF42, 0xFE42, 0xFF62, 8, 3),
            (0xFF44, 0xFE44, 0xFF64, 8, 4),
            (0xFF46, 0xFE46, 0xFF66, 4, 5),
            (0xFF48, 0xFE48, 0xFF68, 4, 6),
        ];
        let t3_mode_reload = |c: u16| (c >> 3) & 7 == 4;
        for (i, &(ca, ta, ic, base, ai)) in cfg.iter().enumerate() {
            let con = self.peek16(ca);
            if con & 0x40 == 0 || (con >> 3) & 7 != 0 {
                continue;
            }
            let pre = base << (con & 7);
            self.acc_t[ai] += cyc;
            let inc = self.acc_t[ai] / pre;
            if inc == 0 {
                continue;
            }
            self.acc_t[ai] %= pre;
            let old = self.peek16(ta) as i64;
            let down = con & 0x80 != 0;
            let mut new = if down {
                old - inc as i64
            } else {
                old + inc as i64
            };
            let wrapped = !(0..=0xFFFF).contains(&new);
            if wrapped {
                // Reload: T3 from T2/T4 in reload mode, T6 from CAPREL (T6SR).
                let reload = if i == 1 {
                    let t4 = self.peek16(0xFF44);
                    let t2 = self.peek16(0xFF40);
                    if t3_mode_reload(t4) {
                        self.set_ir(0xFF64);
                        Some(self.peek16(0xFE44) as i64)
                    } else if t3_mode_reload(t2) {
                        self.set_ir(0xFF60);
                        Some(self.peek16(0xFE40) as i64)
                    } else {
                        None
                    }
                } else if i == 4 && con & 0x8000 != 0 {
                    Some(self.peek16(0xFE4A) as i64)
                } else {
                    None
                };
                new = match reload {
                    Some(r) => r,
                    None => new.rem_euclid(0x10000),
                };
                self.set_ir(ic);
                if i == 4 {
                    // T6 over/underflow: count input of CAPCOM T0/T7 (TxI = X00)
                    self.capcom_count(0, 0);
                    self.capcom_count(1, 0);
                }
            }
            self.put16(ta, new as u16);
        }
    }
}

fn alu(op: Alu, b: bool, a: u16, s: u16, carry: bool) -> (u16, Flags) {
    let (mask, msb): (u32, u32) = if b { (0xFF, 0x80) } else { (0xFFFF, 0x8000) };
    let a32 = a as u32 & mask;
    let s32 = s as u32 & mask;
    let e = s32 == msb;
    let (r, c, v) = match op {
        Alu::Add | Alu::Addc => {
            let ci = (op == Alu::Addc && carry) as u32;
            let r = a32 + s32 + ci;
            let v = (!(a32 ^ s32) & (a32 ^ r) & msb) != 0;
            (r & mask, r > mask, v)
        }
        Alu::Sub | Alu::Subc | Alu::Cmp => {
            let bi = (op == Alu::Subc && carry) as u32;
            let r = a32.wrapping_sub(s32).wrapping_sub(bi);
            let v = ((a32 ^ s32) & (a32 ^ r) & msb) != 0;
            (r & mask, s32 + bi > a32, v)
        }
        Alu::And => (a32 & s32, false, false),
        Alu::Or => (a32 | s32, false, false),
        Alu::Xor => (a32 ^ s32, false, false),
    };
    // ADDC/SUBC: the caller ANDs Z with the previous Z.
    let z = r == 0;
    (
        r as u16,
        Flags {
            n: r & msb != 0,
            c,
            v,
            z,
            e,
        },
    )
}
