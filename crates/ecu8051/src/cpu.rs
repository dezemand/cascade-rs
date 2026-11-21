//! 8051 core with the Siemens 80C517(A) extras this ROM uses: eight data
//! pointers (DPSEL), the multiplication/division unit (MD0-MD5, ARCON),
//! Timer 2 with compare, the compare timer, two serial channels, the A/D
//! converter and the extended interrupt system.
//!
//! The SFR map follows the SAB 80C517A data sheet as remembered **[assumed]**;
//! what the ROM actually does with each register is what the analysis checks.
//!
//! Banking (from the ROM's trampolines at 0x1E1D/0x1E38/0x1E43): the whole 64 KB code
//! space is switched by P5.4 (SFR F8h bit 4). P5.4 = 1 selects file half 0,
//! P5.4 = 0 selects file half 1.

use std::collections::VecDeque;

/// External I/O register the ROM reads the K-line level from.
pub const KLINE_IN: u16 = 0xBFFE;
pub const SP: u8 = 0x81;
pub const DPL: u8 = 0x82;
pub const DPH: u8 = 0x83;
pub const TCON: u8 = 0x88;
pub const TMOD: u8 = 0x89;
pub const TL0: u8 = 0x8A;
pub const TL1: u8 = 0x8B;
pub const TH0: u8 = 0x8C;
pub const TH1: u8 = 0x8D;
pub const DPSEL: u8 = 0x92;
pub const S0CON: u8 = 0x98;
pub const S0BUF: u8 = 0x99;
pub const IEN2: u8 = 0x9A;
pub const S1CON: u8 = 0x9B;
pub const S1BUF: u8 = 0x9C;
pub const S1RELL: u8 = 0x9D;
pub const IEN0: u8 = 0xA8;
pub const IP0: u8 = 0xA9;
pub const S0RELL: u8 = 0xAA;
pub const IEN1: u8 = 0xB8;
pub const IP1: u8 = 0xB9;
pub const S0RELH: u8 = 0xBA;
pub const S1RELH: u8 = 0xBB;
pub const IRCON: u8 = 0xC0;
pub const CCEN: u8 = 0xC1;
pub const T2CON: u8 = 0xC8;
pub const CRCL: u8 = 0xCA;
pub const CRCH: u8 = 0xCB;
pub const TL2: u8 = 0xCC;
pub const TH2: u8 = 0xCD;
pub const PSW: u8 = 0xD0;
pub const IRCON1: u8 = 0xD1;
pub const ADCON0: u8 = 0xD8;
pub const ADDATH: u8 = 0xD9;
pub const ADDATL: u8 = 0xDA;
pub const ADCON1: u8 = 0xDC;
pub const CTRELL: u8 = 0xDE;
pub const CTRELH: u8 = 0xDF;
pub const ACC: u8 = 0xE0;
pub const CTCON: u8 = 0xE1;
pub const MD0: u8 = 0xE9;
pub const MD5: u8 = 0xEE;
pub const ARCON: u8 = 0xEF;
pub const B: u8 = 0xF0;
pub const P5: u8 = 0xF8;

pub fn sfr_name(a: u8) -> &'static str {
    match a {
        0x80 => "P0",
        0x81 => "SP",
        0x82 => "DPL",
        0x83 => "DPH",
        0x86 => "WDTREL",
        0x87 => "PCON",
        0x88 => "TCON",
        0x89 => "TMOD",
        0x8A => "TL0",
        0x8B => "TL1",
        0x8C => "TH0",
        0x8D => "TH1",
        0x90 => "P1",
        0x92 => "DPSEL",
        0x98 => "S0CON",
        0x99 => "S0BUF",
        0x9A => "IEN2",
        0x9B => "S1CON",
        0x9C => "S1BUF",
        0x9D => "S1RELL",
        0xA0 => "P2",
        0xA8 => "IEN0",
        0xA9 => "IP0",
        0xAA => "S0RELL",
        0xB0 => "P3",
        0xB1 => "SYSCON",
        0xB8 => "IEN1",
        0xB9 => "IP1",
        0xBA => "S0RELH",
        0xBB => "S1RELH",
        0xC0 => "IRCON",
        0xC1 => "CCEN",
        0xC2 => "CCL1",
        0xC3 => "CCH1",
        0xC4 => "CCL2",
        0xC5 => "CCH2",
        0xC6 => "CCL3",
        0xC7 => "CCH3",
        0xC8 => "T2CON",
        0xC9 => "CC4EN",
        0xCA => "CRCL",
        0xCB => "CRCH",
        0xCC => "TL2",
        0xCD => "TH2",
        0xCE => "CCL4",
        0xCF => "CCH4",
        0xD0 => "PSW",
        0xD1 => "IRCON1",
        0xD8 => "ADCON0",
        0xD9 => "ADDATH",
        0xDA => "ADDATL",
        0xDB => "P7",
        0xDC => "ADCON1",
        0xDD => "P8",
        0xDE => "CTRELL",
        0xDF => "CTRELH",
        0xE0 => "ACC",
        0xE1 => "CTCON",
        0xE8 => "P4",
        0xE9..=0xEE => "MDx",
        0xEF => "ARCON",
        0xF0 => "B",
        0xF6 => "CMEN",
        0xF7 => "CMSEL",
        0xF8 => "P5",
        0xFA => "P6",
        _ => "",
    }
}

/// Interrupt sources: (vector, priority group). Groups per the 80C517A data
/// sheet as remembered **[assumed]**.
const IRQS: [(u16, u8); 17] = [
    (0x03, 0),
    (0x43, 0),
    (0x83, 0),
    (0x0B, 1),
    (0x4B, 1),
    (0x9B, 1),
    (0x13, 2),
    (0x53, 2),
    (0x93, 2),
    (0x1B, 3),
    (0x5B, 3),
    (0xA3, 3),
    (0x23, 4),
    (0x63, 4),
    (0xAB, 4),
    (0x2B, 5),
    (0x6B, 5),
];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Access {
    Read,
    Write,
}

/// An observable peripheral event, for tracing.
#[derive(Debug, Clone)]
pub enum Event {
    Sfr {
        pc: u16,
        addr: u8,
        val: u8,
        acc: Access,
    },
    Tx {
        ch: u8,
        byte: u8,
        bit9: bool,
    },
    Rx {
        ch: u8,
        byte: u8,
    },
    Irq {
        vector: u16,
    },
}

pub struct Uart {
    /// Machine cycles until the byte in the shifter is out.
    pub tx_left: u64,
    pub tx_byte: u8,
    pub tx_bit9: bool,
    /// Bytes waiting on the line: (machine cycle when the stop bit ends,
    /// byte, stop/ninth bit). A break reads as 0x00 with that bit clear.
    pub rx_queue: VecDeque<(u64, u8, bool)>,
    pub rx_overruns: u32,
}

impl Uart {
    fn new() -> Self {
        Uart {
            tx_left: 0,
            tx_byte: 0,
            tx_bit9: false,
            rx_queue: VecDeque::new(),
            rx_overruns: 0,
        }
    }
}

pub struct Cpu {
    pub rom: Vec<u8>,
    pub iram: [u8; 256],
    pub sfr: [u8; 128],
    pub xram: Vec<u8>,
    pub pc: u16,
    pub cycles: u64,
    pub fosc: u64,
    pub dptr: [u16; 8],
    /// Interrupt priority levels currently in service (stack of levels).
    pub in_service: Vec<u8>,
    pub uart: [Uart; 2],
    /// Echo every transmitted byte back into the same channel's receiver
    /// (K-line is a single wire).
    pub echo: bool,
    pub events: Vec<Event>,
    /// SFRs whose accesses are logged as events.
    pub watch_sfr: [bool; 128],
    /// A/D results per channel (10 bit).
    pub adc: [u16; 16],
    adc_left: u32,
    /// External input levels for port pins (ANDed with the latch on reads).
    pub port_in: [u8; 128],
    md_seq: Vec<u8>,
    md_out: [u8; 6],
    ct: u16,
    ct_run: bool,
    ct_frac: u32,
    t2_frac: u32,
    pub unknown_ops: u32,
    pub xram_log: Option<(u16, u16)>,
    pub kline_level: bool,
}

#[derive(Debug)]
pub enum Stop {
    #[allow(dead_code)] // read through Debug
    BadOpcode(u16, u8),
}

impl Cpu {
    pub fn new(rom: Vec<u8>, fosc: u64) -> Self {
        let mut c = Cpu {
            rom,
            iram: [0; 256],
            sfr: [0; 128],
            xram: vec![0; 0x10000],
            pc: 0,
            cycles: 0,
            fosc,
            dptr: [0; 8],
            in_service: Vec::new(),
            uart: [Uart::new(), Uart::new()],
            echo: true,
            events: Vec::new(),
            watch_sfr: [false; 128],
            adc: [0x200; 16],
            adc_left: 0,
            port_in: [0xFF; 128],
            md_seq: Vec::new(),
            md_out: [0; 6],
            ct: 0,
            ct_run: false,
            ct_frac: 0,
            t2_frac: 0,
            unknown_ops: 0,
            xram_log: None,
            kline_level: true,
        };
        for p in [0x80, 0x90, 0xA0, 0xB0, 0xE8, 0xF8, 0xFA, 0xDB, 0xDD] {
            c.sfr[(p - 0x80) as usize] = 0xFF;
        }
        c.sfr[(SP - 0x80) as usize] = 0x07;
        c
    }

    // ---- memory -------------------------------------------------------

    pub fn bank(&self) -> usize {
        if self.sfr[(P5 - 0x80) as usize] & 0x10 != 0 {
            0
        } else {
            1
        }
    }

    pub fn code(&self, a: u16) -> u8 {
        let off = self.bank() * 0x10000 + a as usize;
        self.rom.get(off).copied().unwrap_or(0xFF)
    }

    fn fetch(&mut self) -> u8 {
        let b = self.code(self.pc);
        self.pc = self.pc.wrapping_add(1);
        b
    }

    fn dpsel(&self) -> usize {
        (self.sfr[(DPSEL - 0x80) as usize] & 7) as usize
    }

    pub fn dp(&self) -> u16 {
        self.dptr[self.dpsel()]
    }

    fn set_dp(&mut self, v: u16) {
        let i = self.dpsel();
        self.dptr[i] = v;
    }

    pub fn acc(&self) -> u8 {
        self.sfr[(ACC - 0x80) as usize]
    }

    fn set_acc(&mut self, v: u8) {
        self.sfr[(ACC - 0x80) as usize] = v;
    }

    fn psw(&self) -> u8 {
        self.sfr[(PSW - 0x80) as usize]
    }

    fn cy(&self) -> bool {
        self.psw() & 0x80 != 0
    }

    fn set_cy(&mut self, v: bool) {
        let p = &mut self.sfr[(PSW - 0x80) as usize];
        *p = (*p & !0x80) | if v { 0x80 } else { 0 };
    }

    fn set_flags(&mut self, cy: bool, ac: bool, ov: bool) {
        let p = &mut self.sfr[(PSW - 0x80) as usize];
        *p &= !0xC4;
        *p |= (cy as u8) << 7 | (ac as u8) << 6 | (ov as u8) << 2;
    }

    fn reg_addr(&self, r: u8) -> u8 {
        (self.psw() & 0x18) + r
    }

    fn reg(&self, r: u8) -> u8 {
        self.iram[self.reg_addr(r) as usize]
    }

    fn set_reg(&mut self, r: u8, v: u8) {
        let a = self.reg_addr(r);
        self.iram[a as usize] = v;
    }

    pub fn sfr_get(&self, a: u8) -> u8 {
        match a {
            DPL => self.dp() as u8,
            DPH => (self.dp() >> 8) as u8,
            PSW => {
                let p = self.sfr[(PSW - 0x80) as usize] & 0xFE;
                p | (self.acc().count_ones() as u8 & 1)
            }
            _ => self.sfr[(a - 0x80) as usize],
        }
    }

    fn sfr_set_raw(&mut self, a: u8, v: u8) {
        self.sfr[(a - 0x80) as usize] = v;
    }

    /// Direct read. `rmw` reads port latches instead of pins.
    fn rd(&mut self, a: u8, rmw: bool) -> u8 {
        if a < 0x80 {
            return self.iram[a as usize];
        }
        let mut v = self.sfr_get(a);
        let is_port = matches!(
            a,
            0x80 | 0x90 | 0xA0 | 0xB0 | 0xE8 | 0xF8 | 0xFA | 0xDB | 0xDD
        );
        if is_port && !rmw {
            v &= self.port_in[(a - 0x80) as usize];
        }
        if (MD0..=ARCON).contains(&a) && !rmw {
            if a <= MD5 {
                v = self.md_out[(a - MD0) as usize];
            }
            self.md_seq.clear();
        }
        if self.watch_sfr[(a - 0x80) as usize] {
            self.events.push(Event::Sfr {
                pc: self.pc,
                addr: a,
                val: v,
                acc: Access::Read,
            });
        }
        v
    }

    fn wr(&mut self, a: u8, v: u8) {
        if a < 0x80 {
            self.iram[a as usize] = v;
            return;
        }
        if self.watch_sfr[(a - 0x80) as usize] {
            self.events.push(Event::Sfr {
                pc: self.pc,
                addr: a,
                val: v,
                acc: Access::Write,
            });
        }
        match a {
            DPL => {
                let d = self.dp();
                self.set_dp((d & 0xFF00) | v as u16);
            }
            DPH => {
                let d = self.dp();
                self.set_dp((d & 0x00FF) | (v as u16) << 8);
            }
            S0BUF => {
                self.sfr_set_raw(a, v);
                self.start_tx(0, v);
            }
            S1BUF => {
                self.sfr_set_raw(a, v);
                self.start_tx(1, v);
            }
            ADDATL => {
                // Writing ADDATL / DAPR starts a conversion [assumed].
                self.sfr_set_raw(a, v);
                let c0 = self.sfr_get(ADCON0) | 0x10;
                self.sfr_set_raw(ADCON0, c0);
                self.adc_left = 13;
            }
            CTRELL => {
                self.sfr_set_raw(a, v);
                self.ct = (self.sfr_get(CTRELH) as u16) << 8 | v as u16;
                self.ct_run = true;
            }
            MD0..=MD5 => {
                self.sfr_set_raw(a, v);
                self.md_out[(a - MD0) as usize] = v;
                if a == MD0 {
                    self.md_seq.clear();
                }
                self.md_seq.push(a - MD0);
                if a == MD5 {
                    self.mdu_exec();
                }
            }
            ARCON => {
                self.sfr_set_raw(a, v);
                self.mdu_shift(v);
            }
            _ => self.sfr_set_raw(a, v),
        }
    }

    fn mdu_exec(&mut self) {
        let m = self.md_out;
        let seq = std::mem::take(&mut self.md_seq);
        let mut arcon = self.sfr_get(ARCON) & !0xC0;
        match seq.as_slice() {
            [0, 1, 2, 3, 4, 5] => {
                let n = u32::from_le_bytes([m[0], m[1], m[2], m[3]]);
                let d = u16::from_le_bytes([m[4], m[5]]) as u32;
                match (n.checked_div(d), n.checked_rem(d)) {
                    (Some(q), Some(r)) => {
                        let (q, r) = (q.to_le_bytes(), (r as u16).to_le_bytes());
                        self.md_out = [q[0], q[1], q[2], q[3], r[0], r[1]];
                    }
                    _ => arcon |= 0x40,
                }
            }
            [0, 1, 4, 5] => {
                let n = u16::from_le_bytes([m[0], m[1]]) as u32;
                let d = u16::from_le_bytes([m[4], m[5]]) as u32;
                match (n.checked_div(d), n.checked_rem(d)) {
                    (Some(q), Some(r)) => {
                        let (q, r) = ((q as u16).to_le_bytes(), (r as u16).to_le_bytes());
                        self.md_out = [q[0], q[1], m[2], m[3], r[0], r[1]];
                    }
                    _ => arcon |= 0x40,
                }
            }
            [0, 4, 1, 5] => {
                let a = u16::from_le_bytes([m[0], m[1]]) as u32;
                let b = u16::from_le_bytes([m[4], m[5]]) as u32;
                let p = (a * b).to_le_bytes();
                if a * b > 0xFFFF {
                    arcon |= 0x40;
                }
                self.md_out = [p[0], p[1], p[2], p[3], m[4], m[5]];
            }
            _ => {
                arcon |= 0x80; // MDEF: unrecognised sequence
            }
        }
        let base = (MD0 - 0x80) as usize;
        self.sfr[base..base + 6].copy_from_slice(&self.md_out);
        self.sfr_set_raw(ARCON, arcon);
    }

    fn mdu_shift(&mut self, v: u8) {
        let m = self.md_out;
        let mut x = u32::from_le_bytes([m[0], m[1], m[2], m[3]]);
        let mut arcon = v & 0x3F;
        let sc = v & 0x1F;
        if sc == 0 {
            // normalise
            if x == 0 {
                arcon |= 0x40;
            } else {
                let n = x.leading_zeros() as u8;
                x <<= n;
                arcon = (arcon & !0x1F) | n;
            }
        } else if v & 0x20 != 0 {
            x >>= sc;
        } else {
            x <<= sc;
        }
        let b = x.to_le_bytes();
        self.md_out[..4].copy_from_slice(&b);
        let base = (MD0 - 0x80) as usize;
        self.sfr[base..base + 4].copy_from_slice(&b);
        self.sfr_set_raw(ARCON, arcon);
        self.md_seq.clear();
    }

    fn bit_addr(b: u8) -> (u8, u8) {
        if b < 0x80 {
            (0x20 + b / 8, b % 8)
        } else {
            (b & 0xF8, b & 7)
        }
    }

    fn rd_bit(&mut self, b: u8, rmw: bool) -> bool {
        let (a, n) = Self::bit_addr(b);
        self.rd(a, rmw) >> n & 1 != 0
    }

    fn wr_bit(&mut self, b: u8, v: bool) {
        let (a, n) = Self::bit_addr(b);
        let old = self.rd(a, true);
        let nv = (old & !(1 << n)) | ((v as u8) << n);
        self.wr(a, nv);
    }

    fn ind(&self, i: u8) -> u8 {
        self.reg(i)
    }

    pub fn xread(&mut self, a: u16) -> u8 {
        if a == KLINE_IN {
            // bit 0: K-line level as the ROM samples it at 0x3B0C (it XORs
            // with 0xEC, which leaves bit 0 alone) [assumed: 1 = high]
            return (self.xram[a as usize] & 0xFE) | self.kline_level as u8;
        }
        self.xram[a as usize]
    }

    pub fn xwrite(&mut self, a: u16, v: u8) {
        if let Some((lo, hi)) = self.xram_log
            && (lo..=hi).contains(&a)
        {
            eprintln!(
                "xw {:04X}={:02X} pc={:04X} cyc={}",
                a, v, self.pc, self.cycles
            );
        }
        self.xram[a as usize] = v;
    }

    fn push(&mut self, v: u8) {
        let sp = self.sfr_get(SP).wrapping_add(1);
        self.sfr_set_raw(SP, sp);
        self.iram[sp as usize] = v;
    }

    fn pop(&mut self) -> u8 {
        let sp = self.sfr_get(SP);
        let v = self.iram[sp as usize];
        self.sfr_set_raw(SP, sp.wrapping_sub(1));
        v
    }

    fn rel(&mut self) -> u16 {
        let r = self.fetch() as i8;
        self.pc.wrapping_add(r as u16)
    }

    // ---- ALU ----------------------------------------------------------

    fn add(&mut self, v: u8, carry: bool) {
        let a = self.acc();
        let c = carry as u16;
        let r = a as u16 + v as u16 + c;
        let ac = (a & 0xF) + (v & 0xF) + c as u8 > 0xF;
        let ov = ((a ^ r as u8) & (v ^ r as u8) & 0x80) != 0;
        self.set_flags(r > 0xFF, ac, ov);
        self.set_acc(r as u8);
    }

    fn subb(&mut self, v: u8) {
        let a = self.acc();
        let c = self.cy() as i16;
        let r = a as i16 - v as i16 - c;
        let ac = ((a & 0xF) as i16 - (v & 0xF) as i16 - c) < 0;
        let res = r as u8;
        let ov = ((a ^ v) & (a ^ res) & 0x80) != 0;
        self.set_flags(r < 0, ac, ov);
        self.set_acc(res);
    }

    // ---- execution ----------------------------------------------------

    /// Execute one instruction (or take an interrupt). Returns machine cycles.
    pub fn step(&mut self) -> Result<u32, Stop> {
        if let Some(v) = self.pending_irq() {
            let pc = self.pc;
            self.push(pc as u8);
            self.push((pc >> 8) as u8);
            self.pc = v;
            self.events.push(Event::Irq { vector: v });
            self.tick(2);
            return Ok(2);
        }
        let pc0 = self.pc;
        let op = self.fetch();
        let cyc = self.exec(op, pc0)?;
        self.tick(cyc);
        Ok(cyc)
    }

    fn exec(&mut self, op: u8, pc0: u16) -> Result<u32, Stop> {
        let lo = op & 0x0F;
        // Rn / @Ri operand forms
        let operand_rn = lo >= 8;
        let operand_ri = lo == 6 || lo == 7;
        let ri = op & 1;
        let rn = op & 7;
        macro_rules! src {
            () => {{
                if operand_rn {
                    self.reg(rn)
                } else {
                    let a = self.ind(ri);
                    self.iram[a as usize]
                }
            }};
        }
        macro_rules! dst {
            ($v:expr) => {{
                let v = $v;
                if operand_rn {
                    self.set_reg(rn, v);
                } else {
                    let a = self.ind(ri);
                    self.iram[a as usize] = v;
                }
            }};
        }
        let hi = op >> 4;
        if operand_rn || operand_ri {
            match hi {
                0x0 => {
                    let v = src!().wrapping_add(1);
                    dst!(v);
                    return Ok(1);
                }
                0x1 => {
                    let v = src!().wrapping_sub(1);
                    dst!(v);
                    return Ok(1);
                }
                0x2 => {
                    let v = src!();
                    self.add(v, false);
                    return Ok(1);
                }
                0x3 => {
                    let v = src!();
                    let c = self.cy();
                    self.add(v, c);
                    return Ok(1);
                }
                0x4 => {
                    let v = src!() | self.acc();
                    self.set_acc(v);
                    return Ok(1);
                }
                0x5 => {
                    let v = src!() & self.acc();
                    self.set_acc(v);
                    return Ok(1);
                }
                0x6 => {
                    let v = src!() ^ self.acc();
                    self.set_acc(v);
                    return Ok(1);
                }
                0x7 => {
                    let v = self.fetch();
                    dst!(v);
                    return Ok(1);
                }
                0x8 => {
                    let d = self.fetch();
                    let v = src!();
                    self.wr(d, v);
                    return Ok(2);
                }
                0x9 => {
                    let v = src!();
                    self.subb(v);
                    return Ok(1);
                }
                0xA => {
                    let d = self.fetch();
                    let v = self.rd(d, false);
                    dst!(v);
                    return Ok(2);
                }
                0xB => {
                    let imm = self.fetch();
                    let t = self.rel();
                    let v = src!();
                    self.set_cy(v < imm);
                    if v != imm {
                        self.pc = t;
                    }
                    return Ok(2);
                }
                0xC => {
                    let v = src!();
                    let a = self.acc();
                    dst!(a);
                    self.set_acc(v);
                    return Ok(1);
                }
                0xD => {
                    if operand_rn {
                        let t = self.rel();
                        let v = self.reg(rn).wrapping_sub(1);
                        self.set_reg(rn, v);
                        if v != 0 {
                            self.pc = t;
                        }
                        return Ok(2);
                    }
                    // XCHD A,@Ri
                    let a = self.ind(ri);
                    let m = self.iram[a as usize];
                    let acc = self.acc();
                    self.iram[a as usize] = (m & 0xF0) | (acc & 0x0F);
                    self.set_acc((acc & 0xF0) | (m & 0x0F));
                    return Ok(1);
                }
                0xE => {
                    let v = src!();
                    self.set_acc(v);
                    return Ok(1);
                }
                0xF => {
                    let a = self.acc();
                    dst!(a);
                    return Ok(1);
                }
                _ => unreachable!(),
            }
        }
        if lo == 1 {
            let lo8 = self.fetch();
            let t = (self.pc & 0xF800) | ((op as u16 >> 5) << 8) | lo8 as u16;
            if hi & 1 == 1 {
                let pc = self.pc;
                self.push(pc as u8);
                self.push((pc >> 8) as u8);
            }
            self.pc = t;
            return Ok(2);
        }
        match op {
            0x00 => Ok(1),
            0x02 => {
                let h = self.fetch();
                let l = self.fetch();
                self.pc = (h as u16) << 8 | l as u16;
                Ok(2)
            }
            0x03 => {
                let a = self.acc();
                self.set_acc(a.rotate_right(1));
                Ok(1)
            }
            0x04 => {
                let a = self.acc().wrapping_add(1);
                self.set_acc(a);
                Ok(1)
            }
            0x05 => {
                let d = self.fetch();
                let v = self.rd(d, true).wrapping_add(1);
                self.wr(d, v);
                Ok(1)
            }
            0x10 | 0x20 | 0x30 => {
                let b = self.fetch();
                let t = self.rel();
                let v = self.rd_bit(b, op == 0x10);
                let take = if op == 0x30 { !v } else { v };
                if op == 0x10 && v {
                    self.wr_bit(b, false);
                }
                if take {
                    self.pc = t;
                }
                Ok(2)
            }
            0x12 => {
                let h = self.fetch();
                let l = self.fetch();
                let pc = self.pc;
                self.push(pc as u8);
                self.push((pc >> 8) as u8);
                self.pc = (h as u16) << 8 | l as u16;
                Ok(2)
            }
            0x13 => {
                let a = self.acc();
                let c = self.cy();
                self.set_cy(a & 1 != 0);
                self.set_acc(a >> 1 | (c as u8) << 7);
                Ok(1)
            }
            0x14 => {
                let a = self.acc().wrapping_sub(1);
                self.set_acc(a);
                Ok(1)
            }
            0x15 => {
                let d = self.fetch();
                let v = self.rd(d, true).wrapping_sub(1);
                self.wr(d, v);
                Ok(1)
            }
            0x22 | 0x32 => {
                let h = self.pop();
                let l = self.pop();
                self.pc = (h as u16) << 8 | l as u16;
                if op == 0x32 {
                    self.in_service.pop();
                }
                Ok(2)
            }
            0x23 => {
                let a = self.acc();
                self.set_acc(a.rotate_left(1));
                Ok(1)
            }
            0x24 | 0x34 => {
                let v = self.fetch();
                let c = op == 0x34 && self.cy();
                self.add(v, c);
                Ok(1)
            }
            0x25 | 0x35 => {
                let d = self.fetch();
                let v = self.rd(d, false);
                let c = op == 0x35 && self.cy();
                self.add(v, c);
                Ok(1)
            }
            0x33 => {
                let a = self.acc();
                let c = self.cy();
                self.set_cy(a & 0x80 != 0);
                self.set_acc(a << 1 | c as u8);
                Ok(1)
            }
            0x40 | 0x50 | 0x60 | 0x70 | 0x80 => {
                let t = self.rel();
                let take = match op {
                    0x40 => self.cy(),
                    0x50 => !self.cy(),
                    0x60 => self.acc() == 0,
                    0x70 => self.acc() != 0,
                    _ => true,
                };
                if take {
                    if op == 0x80 && t == pc0 {
                        // SJMP $: idle forever; let time pass
                    }
                    self.pc = t;
                }
                Ok(2)
            }
            // ORL/ANL/XRL direct,A ; direct,#imm ; A,#imm ; A,direct
            0x42 | 0x52 | 0x62 | 0x43 | 0x53 | 0x63 | 0x44 | 0x54 | 0x64 | 0x45 | 0x55 | 0x65 => {
                let f = |o: u8, a: u8, b: u8| match o >> 4 {
                    4 => a | b,
                    5 => a & b,
                    _ => a ^ b,
                };
                match lo {
                    2 => {
                        let d = self.fetch();
                        let v = f(op, self.rd(d, true), self.acc());
                        self.wr(d, v);
                        Ok(1)
                    }
                    3 => {
                        let d = self.fetch();
                        let i = self.fetch();
                        let v = f(op, self.rd(d, true), i);
                        self.wr(d, v);
                        Ok(2)
                    }
                    4 => {
                        let i = self.fetch();
                        let v = f(op, self.acc(), i);
                        self.set_acc(v);
                        Ok(1)
                    }
                    _ => {
                        let d = self.fetch();
                        let v = self.rd(d, false);
                        let v = f(op, self.acc(), v);
                        self.set_acc(v);
                        Ok(1)
                    }
                }
            }
            0x72 | 0x82 | 0xA0 | 0xB0 => {
                let b = self.fetch();
                let v = self.rd_bit(b, false);
                let c = self.cy();
                let r = match op {
                    0x72 => c | v,
                    0x82 => c & v,
                    0xA0 => c | !v,
                    _ => c & !v,
                };
                self.set_cy(r);
                Ok(2)
            }
            0x73 => {
                self.pc = self.dp().wrapping_add(self.acc() as u16);
                Ok(2)
            }
            0x74 => {
                let v = self.fetch();
                self.set_acc(v);
                Ok(1)
            }
            0x75 => {
                let d = self.fetch();
                let v = self.fetch();
                self.wr(d, v);
                Ok(2)
            }
            0x83 => {
                let a = self.pc.wrapping_add(self.acc() as u16);
                let v = self.code(a);
                self.set_acc(v);
                Ok(2)
            }
            0x84 => {
                let a = self.acc();
                let b = self.sfr_get(B);
                let ac = self.psw() & 0x40 != 0;
                match (a.checked_div(b), a.checked_rem(b)) {
                    (Some(q), Some(r)) => {
                        self.set_acc(q);
                        self.sfr_set_raw(B, r);
                        self.set_flags(false, ac, false);
                    }
                    _ => self.set_flags(false, ac, true),
                }
                Ok(4)
            }
            0x85 => {
                let s = self.fetch();
                let d = self.fetch();
                let v = self.rd(s, false);
                self.wr(d, v);
                Ok(2)
            }
            0x90 => {
                let h = self.fetch();
                let l = self.fetch();
                self.set_dp((h as u16) << 8 | l as u16);
                Ok(2)
            }
            0x92 => {
                let b = self.fetch();
                let c = self.cy();
                self.wr_bit(b, c);
                Ok(2)
            }
            0x93 => {
                let a = self.dp().wrapping_add(self.acc() as u16);
                let v = self.code(a);
                self.set_acc(v);
                Ok(2)
            }
            0x94 => {
                let v = self.fetch();
                self.subb(v);
                Ok(1)
            }
            0x95 => {
                let d = self.fetch();
                let v = self.rd(d, false);
                self.subb(v);
                Ok(1)
            }
            0xA2 => {
                let b = self.fetch();
                let v = self.rd_bit(b, false);
                self.set_cy(v);
                Ok(1)
            }
            0xA3 => {
                let d = self.dp().wrapping_add(1);
                self.set_dp(d);
                Ok(2)
            }
            0xA4 => {
                let r = self.acc() as u16 * self.sfr_get(B) as u16;
                self.set_acc(r as u8);
                self.sfr_set_raw(B, (r >> 8) as u8);
                let ac = self.psw() & 0x40 != 0;
                self.set_flags(false, ac, r > 0xFF);
                Ok(4)
            }
            0xB2 => {
                let b = self.fetch();
                let v = self.rd_bit(b, true);
                self.wr_bit(b, !v);
                Ok(1)
            }
            0xB3 => {
                let c = self.cy();
                self.set_cy(!c);
                Ok(1)
            }
            0xB4 | 0xB5 => {
                let x = self.fetch();
                let t = self.rel();
                let v = if op == 0xB4 { x } else { self.rd(x, false) };
                let a = self.acc();
                self.set_cy(a < v);
                if a != v {
                    self.pc = t;
                }
                Ok(2)
            }
            0xC0 => {
                let d = self.fetch();
                let v = self.rd(d, false);
                self.push(v);
                Ok(2)
            }
            0xC2 | 0xD2 => {
                let b = self.fetch();
                self.wr_bit(b, op == 0xD2);
                Ok(1)
            }
            0xC3 | 0xD3 => {
                self.set_cy(op == 0xD3);
                Ok(1)
            }
            0xC4 => {
                let a = self.acc();
                self.set_acc(a.rotate_left(4));
                Ok(1)
            }
            0xC5 => {
                let d = self.fetch();
                let v = self.rd(d, true);
                let a = self.acc();
                self.wr(d, a);
                self.set_acc(v);
                Ok(1)
            }
            0xD0 => {
                let d = self.fetch();
                let v = self.pop();
                self.wr(d, v);
                Ok(2)
            }
            0xD4 => {
                let mut a = self.acc() as u16;
                let mut c = self.cy();
                if a & 0xF > 9 || self.psw() & 0x40 != 0 {
                    a += 6;
                }
                if a > 0xFF {
                    c = true;
                }
                if (a & 0x1F0) > 0x90 || c {
                    a += 0x60;
                }
                if a > 0xFF {
                    c = true;
                }
                self.set_cy(c);
                self.set_acc(a as u8);
                Ok(1)
            }
            0xD5 => {
                let d = self.fetch();
                let t = self.rel();
                let v = self.rd(d, true).wrapping_sub(1);
                self.wr(d, v);
                if v != 0 {
                    self.pc = t;
                }
                Ok(2)
            }
            0xE0 => {
                let a = self.dp();
                let v = self.xread(a);
                self.set_acc(v);
                Ok(2)
            }
            0xE2 | 0xE3 => {
                let a = (self.sfr_get(0xA0) as u16) << 8 | self.reg(op & 1) as u16;
                let v = self.xread(a);
                self.set_acc(v);
                Ok(2)
            }
            0xE4 => {
                self.set_acc(0);
                Ok(1)
            }
            0xE5 => {
                let d = self.fetch();
                let v = self.rd(d, false);
                self.set_acc(v);
                Ok(1)
            }
            0xF0 => {
                let a = self.dp();
                let v = self.acc();
                self.xwrite(a, v);
                Ok(2)
            }
            0xF2 | 0xF3 => {
                let a = (self.sfr_get(0xA0) as u16) << 8 | self.reg(op & 1) as u16;
                let v = self.acc();
                self.xwrite(a, v);
                Ok(2)
            }
            0xF4 => {
                let a = self.acc();
                self.set_acc(!a);
                Ok(1)
            }
            0xF5 => {
                let d = self.fetch();
                let a = self.acc();
                self.wr(d, a);
                Ok(1)
            }
            _ => {
                self.unknown_ops += 1;
                Err(Stop::BadOpcode(pc0, op))
            }
        }
    }

    // ---- interrupts ---------------------------------------------------

    fn flag(&self, a: u8, bit: u8) -> bool {
        self.sfr[(a - 0x80) as usize] >> bit & 1 != 0
    }

    fn clr_flag(&mut self, a: u8, bit: u8) {
        self.sfr[(a - 0x80) as usize] &= !(1 << bit);
    }

    fn set_flag(&mut self, a: u8, bit: u8) {
        self.sfr[(a - 0x80) as usize] |= 1 << bit;
    }

    fn irq_requested(&self, v: u16) -> bool {
        match v {
            0x03 => self.flag(IEN0, 0) && self.flag(TCON, 1),
            0x0B => self.flag(IEN0, 1) && self.flag(TCON, 5),
            0x13 => self.flag(IEN0, 2) && self.flag(TCON, 3),
            0x1B => self.flag(IEN0, 3) && self.flag(TCON, 7),
            0x23 => self.flag(IEN0, 4) && self.sfr_get(S0CON) & 3 != 0,
            0x2B => self.flag(IEN0, 5) && self.sfr_get(IRCON) & 0xC0 != 0,
            0x43 => self.flag(IEN1, 0) && self.flag(IRCON, 0),
            0x4B => self.flag(IEN1, 1) && self.flag(IRCON, 1),
            0x53 => self.flag(IEN1, 2) && self.flag(IRCON, 2),
            0x5B => self.flag(IEN1, 3) && self.flag(IRCON, 3),
            0x63 => self.flag(IEN1, 4) && self.flag(IRCON, 4),
            0x6B => self.flag(IEN1, 5) && self.flag(IRCON, 5),
            0x83 => self.flag(IEN2, 0) && self.sfr_get(S1CON) & 3 != 0,
            0x93 => self.flag(IEN2, 2) && self.sfr_get(IRCON1) != 0,
            0x9B => self.flag(IEN2, 3) && self.flag(CTCON, 3),
            _ => false,
        }
    }

    fn pending_irq(&mut self) -> Option<u16> {
        if !self.flag(IEN0, 7) {
            return None;
        }
        let cur = self.in_service.last().map(|&l| l as i8).unwrap_or(-1);
        let (ip0, ip1) = (self.sfr_get(IP0), self.sfr_get(IP1));
        let mut best: Option<(u8, u16)> = None;
        for &(v, g) in &IRQS {
            if !self.irq_requested(v) {
                continue;
            }
            let lvl = (ip1 >> g & 1) << 1 | (ip0 >> g & 1);
            if (lvl as i8) <= cur {
                continue;
            }
            if best.is_none_or(|(bl, _)| lvl > bl) {
                best = Some((lvl, v));
            }
        }
        let (lvl, v) = best?;
        // hardware-cleared flags
        match v {
            0x03 => self.clr_flag(TCON, 1),
            0x0B => self.clr_flag(TCON, 5),
            0x13 => self.clr_flag(TCON, 3),
            0x1B => self.clr_flag(TCON, 7),
            0x43 => self.clr_flag(IRCON, 0),
            0x4B => self.clr_flag(IRCON, 1),
            0x53 => self.clr_flag(IRCON, 2),
            0x5B => self.clr_flag(IRCON, 3),
            0x63 => self.clr_flag(IRCON, 4),
            0x6B => self.clr_flag(IRCON, 5),
            0x9B => self.clr_flag(CTCON, 3),
            _ => {}
        }
        self.in_service.push(lvl);
        Some(v)
    }

    // ---- peripherals --------------------------------------------------

    /// Machine cycles per bit on serial channel `ch`.
    pub fn bit_cycles(&self, ch: usize) -> u64 {
        let osc_per_bit = if ch == 1 {
            let rel = ((self.sfr_get(S1RELH) as u64 & 3) << 8) | self.sfr_get(S1RELL) as u64;
            32 * (1024 - rel)
        } else {
            let smod = self.sfr_get(0x87) & 0x80 != 0;
            if self.sfr_get(ADCON0) & 0x80 != 0 {
                let rel = ((self.sfr_get(S0RELH) as u64 & 3) << 8) | self.sfr_get(S0RELL) as u64;
                (if smod { 32 } else { 64 }) * (1024 - rel)
            } else {
                // Timer 1 mode 2 [assumed]
                let th1 = self.sfr_get(TH1) as u64;
                (if smod { 16 } else { 32 }) * 12 * (256 - th1)
            }
        };
        (osc_per_bit / 12).max(1)
    }

    pub fn baud(&self, ch: usize) -> f64 {
        self.fosc as f64 / 12.0 / self.bit_cycles(ch) as f64
    }

    fn start_tx(&mut self, ch: usize, v: u8) {
        let con = if ch == 0 { S0CON } else { S1CON };
        let c = self.sfr_get(con);
        let nine = if ch == 0 {
            c & 0x80 != 0
        } else {
            c & 0x80 == 0
        };
        let bit9 = c & 0x08 != 0;
        self.uart[ch].tx_byte = v;
        self.uart[ch].tx_bit9 = bit9;
        self.uart[ch].tx_left = self.bit_cycles(ch) * if nine { 11 } else { 10 };
        self.events.push(Event::Tx {
            ch: ch as u8,
            byte: v,
            bit9: nine && bit9,
        });
    }

    /// Queue a byte on the line so its stop bit ends at machine cycle `at`.
    pub fn line_rx(&mut self, ch: usize, at: u64, byte: u8, stop: bool) {
        self.uart[ch].rx_queue.push_back((at, byte, stop));
    }

    fn tick(&mut self, c: u32) {
        let c64 = c as u64;
        self.cycles += c64;
        // Timers 0/1
        let tcon = self.sfr_get(TCON);
        let tmod = self.sfr_get(TMOD);
        for t in 0..2u8 {
            let tr = tcon >> (4 + 2 * t) & 1 != 0;
            let mode = tmod >> (4 * t) & 3;
            if !tr || mode == 3 && t == 1 {
                continue;
            }
            let (tl, th) = if t == 0 { (TL0, TH0) } else { (TL1, TH1) };
            let tf = 5 + 2 * t;
            let mut ovf = false;
            match mode {
                0 => {
                    let v = ((self.sfr_get(th) as u32) << 5 | (self.sfr_get(tl) & 0x1F) as u32) + c;
                    ovf = v > 0x1FFF;
                    self.sfr_set_raw(th, (v >> 5) as u8);
                    self.sfr_set_raw(tl, (v & 0x1F) as u8);
                }
                1 => {
                    let v = ((self.sfr_get(th) as u32) << 8 | self.sfr_get(tl) as u32) + c;
                    ovf = v > 0xFFFF;
                    self.sfr_set_raw(th, (v >> 8) as u8);
                    self.sfr_set_raw(tl, v as u8);
                }
                2 => {
                    let v = self.sfr_get(tl) as u32 + c;
                    if v > 0xFF {
                        ovf = true;
                        let r = self.sfr_get(th) as u32;
                        self.sfr_set_raw(tl, (r + (v - 0x100)) as u8);
                    } else {
                        self.sfr_set_raw(tl, v as u8);
                    }
                }
                _ => {
                    let v = self.sfr_get(TL0) as u32 + c;
                    ovf = v > 0xFF;
                    self.sfr_set_raw(TL0, v as u8);
                }
            }
            if ovf {
                self.set_flag(TCON, tf);
            }
        }
        self.tick_t2(c);
        self.tick_ct(c);
        // ADC
        if self.adc_left > 0 {
            self.adc_left = self.adc_left.saturating_sub(c);
            if self.adc_left == 0 {
                // channel select MX3-MX0 in ADCON1 (the ROM writes the channel there)
                let ch = self.sfr_get(ADCON1) & 0x0F;
                let v = self.adc[ch as usize & 15];
                self.sfr_set_raw(ADDATH, (v >> 2) as u8);
                self.sfr_set_raw(ADDATL, ((v & 3) << 6) as u8);
                let c0 = self.sfr_get(ADCON0) & !0x10;
                self.sfr_set_raw(ADCON0, c0);
                self.set_flag(IRCON, 0);
            }
        }
        // UARTs
        for ch in 0..2usize {
            let con = if ch == 0 { S0CON } else { S1CON };
            if self.uart[ch].tx_left > 0 {
                self.uart[ch].tx_left = self.uart[ch].tx_left.saturating_sub(c64);
                if self.uart[ch].tx_left == 0 {
                    let c = self.sfr_get(con) | 0x02;
                    self.sfr_set_raw(con, c);
                    if self.echo {
                        let b = self.uart[ch].tx_byte;
                        let at = self.cycles;
                        let bit9 = self.uart[ch].tx_bit9;
                        self.uart[ch].rx_queue.push_front((at, b, bit9));
                    }
                }
            }
            if let Some(&(at, b, stop)) = self.uart[ch].rx_queue.front()
                && at <= self.cycles
            {
                self.uart[ch].rx_queue.pop_front();
                let cv = self.sfr_get(con);
                if cv & 0x10 != 0 {
                    if cv & 1 != 0 {
                        // RI still set: the byte is lost, as on the chip
                        self.uart[ch].rx_overruns += 1;
                    } else {
                        let buf = if ch == 0 { S0BUF } else { S1BUF };
                        self.sfr_set_raw(buf, b);
                        let rb8 = if stop { 0x04 } else { 0 };
                        self.sfr_set_raw(con, (cv & !0x04) | rb8 | 1);
                    }
                    self.events.push(Event::Rx {
                        ch: ch as u8,
                        byte: b,
                    });
                }
            }
        }
    }

    fn tick_t2(&mut self, c: u32) {
        let t2con = self.sfr_get(T2CON);
        if t2con & 3 != 1 {
            return; // only the internal-clock timer function is modelled
        }
        let div = if t2con & 0x80 != 0 { 2 } else { 1 };
        self.t2_frac += c;
        let inc = self.t2_frac / div;
        self.t2_frac %= div;
        if inc == 0 {
            return;
        }
        let old = (self.sfr_get(TH2) as u32) << 8 | self.sfr_get(TL2) as u32;
        let mut v = old + inc;
        if v > 0xFFFF {
            self.set_flag(IRCON, 6);
            v = if t2con & 0x18 == 0x10 {
                let crc = (self.sfr_get(CRCH) as u32) << 8 | self.sfr_get(CRCL) as u32;
                crc + (v - 0x10000)
            } else {
                v & 0xFFFF
            };
        }
        self.sfr_set_raw(TH2, (v >> 8) as u8);
        self.sfr_set_raw(TL2, v as u8);
        // compare matches: CRC -> IEX3, CC1..CC3 -> IEX4..IEX6 [assumed]
        let ccen = self.sfr_get(CCEN);
        let regs = [
            (CRCL, CRCH, 2u8),
            (0xC2, 0xC3, 3),
            (0xC4, 0xC5, 4),
            (0xC6, 0xC7, 5),
        ];
        for (i, &(l, h, bit)) in regs.iter().enumerate() {
            if ccen >> (2 * i) & 3 != 2 {
                continue;
            }
            let cmp = (self.sfr_get(h) as u32) << 8 | self.sfr_get(l) as u32;
            let hit = if v >= old {
                cmp > old && cmp <= v
            } else {
                cmp > old || cmp <= v
            };
            if hit {
                self.set_flag(IRCON, bit);
            }
        }
    }

    fn tick_ct(&mut self, c: u32) {
        if !self.ct_run {
            return;
        }
        // CT clock = fosc/2 / 2^n [assumed]; per machine cycle 6 >> n.
        let n = self.sfr_get(CTCON) & 7;
        self.ct_frac += c * 6;
        let inc = self.ct_frac >> n;
        self.ct_frac -= inc << n;
        let mut v = self.ct as u32 + inc;
        if v > 0xFFFF {
            let rel = (self.sfr_get(CTRELH) as u32) << 8 | self.sfr_get(CTRELL) as u32;
            v = rel + (v - 0x10000) % (0x10000 - rel).max(1);
            self.set_flag(CTCON, 3);
        }
        self.ct = v as u16;
    }
}
