//! C166/C167 instruction decoder, shared by the disassembler and the
//! emulator so the two cannot disagree.
//!
//! Encodings follow the Infineon C166 Family Instruction Set Manual. Notes on
//! the less obvious ones (each checked against code in the SIMK41 dump):
//! - `BFLDL bitoff,#mask,#data` is `0A QQ @@ ##`, but `BFLDH` is `1A QQ ## @@`
//!   (data before mask). The boot code's `BUSCON` setup only has data within
//!   mask under that reading.
//! - `MOVBZ/MOVBS Rw,Rb` (`C0`/`D0`) put the byte source in the high nibble.
//! - The short ALU forms (`x8`/`x9`) address `[Rw]`/`[Rw+]` with R0-R3 only.

use std::fmt;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Op {
    /// General-purpose word register R0-R15.
    Rw(u8),
    /// General-purpose byte register (RL0, RH0, RL1, ... as 0-15).
    Rb(u8),
    /// An 8-bit `reg` field: 0x00-0xEF an SFR (or ESFR under EXTR), 0xF0-0xFF
    /// a GPR. Width comes from the instruction.
    Reg(u8),
    /// 16-bit data address (paged through DPPx, EXTP or EXTS).
    Mem(u16),
    Ind(u8),
    PostInc(u8),
    PreDec(u8),
    Off(u8, u16),
    Imm(u16),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Alu {
    Add,
    Addc,
    Sub,
    Subc,
    Cmp,
    Xor,
    And,
    Or,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Shift {
    Rol,
    Ror,
    Shl,
    Shr,
    Ashr,
}

/// Variant names are the mnemonics.
#[allow(clippy::enum_variant_names)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Div {
    Div,
    Divu,
    Divl,
    Divlu,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BitOp {
    Bcmp,
    Bmovn,
    Bmov,
    Bor,
    Band,
    Bxor,
}

/// Variant names are the mnemonics.
#[allow(clippy::enum_variant_names)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Jb {
    Jb,
    Jnb,
    Jbc,
    Jnbs,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Ext {
    Exts,
    Extp,
    Extsr,
    Extpr,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Insn {
    Alu {
        op: Alu,
        b: bool,
        d: Op,
        s: Op,
    },
    Mov {
        b: bool,
        d: Op,
        s: Op,
    },
    /// MOVBZ (signed = false) / MOVBS: word destination, byte source.
    Movx {
        signed: bool,
        d: Op,
        s: Op,
    },
    /// CMPI1/CMPI2/CMPD1/CMPD2: compare Rw with s, then add `delta` to Rw.
    Cmpid {
        delta: i8,
        n: u8,
        s: Op,
    },
    Neg {
        b: bool,
        n: u8,
    },
    Cpl {
        b: bool,
        n: u8,
    },
    Mul {
        signed: bool,
        n: u8,
        m: u8,
    },
    Div {
        kind: Div,
        n: u8,
    },
    Shift {
        kind: Shift,
        n: u8,
        s: Op,
    },
    Prior {
        n: u8,
        m: u8,
    },
    Bclr {
        q: u8,
        bit: u8,
    },
    Bset {
        q: u8,
        bit: u8,
    },
    /// Two-bit operations: destination (zq.z), source (qq.q).
    Bit2 {
        op: BitOp,
        zq: u8,
        z: u8,
        qq: u8,
        q: u8,
    },
    Bfld {
        high: bool,
        q: u8,
        mask: u8,
        data: u8,
    },
    Jmpr {
        cc: u8,
        t: u16,
    },
    Jmpa {
        cc: u8,
        t: u16,
    },
    Jmps {
        seg: u8,
        t: u16,
    },
    Jmpi {
        cc: u8,
        n: u8,
    },
    Jbit {
        kind: Jb,
        q: u8,
        bit: u8,
        t: u16,
    },
    Calla {
        cc: u8,
        t: u16,
    },
    Calls {
        seg: u8,
        t: u16,
    },
    Calli {
        cc: u8,
        n: u8,
    },
    Callr {
        t: u16,
    },
    Pcall {
        r: u8,
        t: u16,
    },
    Ret,
    Rets,
    Retp(u8),
    Reti,
    Trap(u8),
    Push(u8),
    Pop(u8),
    Scxt {
        r: u8,
        s: Op,
    },
    /// EXTS/EXTP/EXTSR/EXTPR; `arg` is `Rw(n)` or `Imm(seg or page)`.
    Ext {
        kind: Ext,
        cnt: u8,
        arg: Op,
    },
    Atomic(u8),
    Extr(u8),
    Nop,
    Sys(&'static str),
    Bad,
}

#[derive(Clone, Copy, Debug)]
pub struct Decoded {
    pub len: u8,
    pub insn: Insn,
}

fn alu_of(hi: u8) -> Alu {
    match hi {
        0 => Alu::Add,
        1 => Alu::Addc,
        2 => Alu::Sub,
        3 => Alu::Subc,
        4 => Alu::Cmp,
        5 => Alu::Xor,
        6 => Alu::And,
        _ => Alu::Or,
    }
}

/// Decode one instruction at `ip` from its first four bytes.
pub fn decode(ip: u16, c: [u8; 4]) -> Decoded {
    let [o, b1, b2, b3] = c;
    let w23 = u16::from_le_bytes([b2, b3]);
    let hi = o >> 4;
    let lo = o & 15;
    let n = b1 >> 4;
    let m = b1 & 15;
    let rel = |next: u16, r: u8| next.wrapping_add((r as i8 as i16 as u16).wrapping_mul(2));
    let d2 = |insn| Decoded { len: 2, insn };
    let d4 = |insn| Decoded { len: 4, insn };

    // Bit set/clear and relative jumps take whole columns.
    match lo {
        0xD => {
            return d2(Insn::Jmpr {
                cc: hi,
                t: rel(ip.wrapping_add(2), b1),
            });
        }
        0xE => return d2(Insn::Bclr { q: b1, bit: hi }),
        0xF => return d2(Insn::Bset { q: b1, bit: hi }),
        _ => {}
    }
    if hi < 8 && lo <= 9 {
        let op = alu_of(hi);
        let b = lo & 1 == 1;
        let gpr = |x: u8| if b { Op::Rb(x) } else { Op::Rw(x) };
        return match lo {
            0 | 1 => d2(Insn::Alu {
                op,
                b,
                d: gpr(n),
                s: gpr(m),
            }),
            2 | 3 => d4(Insn::Alu {
                op,
                b,
                d: Op::Reg(b1),
                s: Op::Mem(w23),
            }),
            4 | 5 => d4(Insn::Alu {
                op,
                b,
                d: Op::Mem(w23),
                s: Op::Reg(b1),
            }),
            6 => d4(Insn::Alu {
                op,
                b,
                d: Op::Reg(b1),
                s: Op::Imm(w23),
            }),
            7 => d4(Insn::Alu {
                op,
                b,
                d: Op::Reg(b1),
                s: Op::Imm(b2 as u16),
            }),
            _ => {
                let s = if m & 8 == 0 {
                    Op::Imm((m & 7) as u16)
                } else if m & 4 == 0 {
                    Op::Ind(m & 3)
                } else {
                    Op::PostInc(m & 3)
                };
                d2(Insn::Alu {
                    op,
                    b,
                    d: gpr(n),
                    s,
                })
            }
        };
    }
    match o {
        0x0A | 0x1A => {
            let (mask, data) = if o == 0x0A { (b2, b3) } else { (b3, b2) };
            d4(Insn::Bfld {
                high: o == 0x1A,
                q: b1,
                mask,
                data,
            })
        }
        0x2A | 0x3A | 0x4A | 0x5A | 0x6A | 0x7A => {
            let op = match o {
                0x2A => BitOp::Bcmp,
                0x3A => BitOp::Bmovn,
                0x4A => BitOp::Bmov,
                0x5A => BitOp::Bor,
                0x6A => BitOp::Band,
                _ => BitOp::Bxor,
            };
            d4(Insn::Bit2 {
                op,
                qq: b1,
                zq: b2,
                q: b3 >> 4,
                z: b3 & 15,
            })
        }
        0x8A | 0x9A | 0xAA | 0xBA => {
            let kind = match o {
                0x8A => Jb::Jb,
                0x9A => Jb::Jnb,
                0xAA => Jb::Jbc,
                _ => Jb::Jnbs,
            };
            d4(Insn::Jbit {
                kind,
                q: b1,
                bit: b3 >> 4,
                t: rel(ip.wrapping_add(4), b2),
            })
        }
        0xCA => d4(Insn::Calla { cc: n, t: w23 }),
        0xDA => d4(Insn::Calls { seg: b1, t: w23 }),
        0xEA => d4(Insn::Jmpa { cc: n, t: w23 }),
        0xFA => d4(Insn::Jmps { seg: b1, t: w23 }),
        0x0B | 0x1B => d2(Insn::Mul {
            signed: o == 0x0B,
            n,
            m,
        }),
        0x2B => d2(Insn::Prior { n, m }),
        0x4B | 0x5B | 0x6B | 0x7B => d2(Insn::Div {
            kind: match o {
                0x4B => Div::Div,
                0x5B => Div::Divu,
                0x6B => Div::Divl,
                _ => Div::Divlu,
            },
            n,
        }),
        0x9B => d2(Insn::Trap(b1 >> 1)),
        0xAB => d2(Insn::Calli { cc: n, n: m }),
        0xBB => d2(Insn::Callr {
            t: rel(ip.wrapping_add(2), b1),
        }),
        0xCB => d2(Insn::Ret),
        0xDB => d2(Insn::Rets),
        0xEB => d2(Insn::Retp(b1)),
        0xFB => d2(Insn::Reti),
        0x0C | 0x1C | 0x2C | 0x3C | 0x4C | 0x5C | 0x6C | 0x7C | 0xAC | 0xBC => {
            let kind = match o & 0xE0 {
                0x00 => Shift::Rol,
                0x20 => Shift::Ror,
                0x40 => Shift::Shl,
                0x60 => Shift::Shr,
                _ => Shift::Ashr,
            };
            let s = if o & 0x10 != 0 {
                Op::Imm(n as u16)
            } else {
                Op::Rw(m)
            };
            let dst = if o & 0x10 != 0 { m } else { n };
            d2(Insn::Shift { kind, n: dst, s })
        }
        0x9C => d2(Insn::Jmpi { cc: n, n: m }),
        0xCC => d2(Insn::Nop),
        0xDC => {
            let kind = match b1 >> 6 {
                0 => Ext::Exts,
                1 => Ext::Extp,
                2 => Ext::Extsr,
                _ => Ext::Extpr,
            };
            d2(Insn::Ext {
                kind,
                cnt: ((b1 >> 4) & 3) + 1,
                arg: Op::Rw(m),
            })
        }
        0xD7 => {
            let kind = match b1 >> 6 {
                0 => Ext::Exts,
                1 => Ext::Extp,
                2 => Ext::Extsr,
                _ => Ext::Extpr,
            };
            let arg = if matches!(kind, Ext::Exts | Ext::Extsr) {
                Op::Imm(b2 as u16)
            } else {
                Op::Imm(w23 & 0x3FF)
            };
            d4(Insn::Ext {
                kind,
                cnt: ((b1 >> 4) & 3) + 1,
                arg,
            })
        }
        0xD1 => {
            let cnt = ((b1 >> 4) & 3) + 1;
            if b1 & 0x80 != 0 {
                d2(Insn::Extr(cnt))
            } else {
                d2(Insn::Atomic(cnt))
            }
        }
        0xEC => d2(Insn::Push(b1)),
        0xFC => d2(Insn::Pop(b1)),
        0xC6 => d4(Insn::Scxt {
            r: b1,
            s: Op::Imm(w23),
        }),
        0xD6 => d4(Insn::Scxt {
            r: b1,
            s: Op::Mem(w23),
        }),
        0xE2 => d4(Insn::Pcall { r: b1, t: w23 }),
        // CMPI/CMPD
        0x80 | 0x90 | 0xA0 | 0xB0 => d2(Insn::Cmpid {
            delta: cmpid_delta(o),
            n: m,
            s: Op::Imm(n as u16),
        }),
        0x82 | 0x92 | 0xA2 | 0xB2 => d4(Insn::Cmpid {
            delta: cmpid_delta(o),
            n: m,
            s: Op::Mem(w23),
        }),
        0x86 | 0x96 | 0xA6 | 0xB6 => d4(Insn::Cmpid {
            delta: cmpid_delta(o),
            n: m,
            s: Op::Imm(w23),
        }),
        0x81 => d2(Insn::Neg { b: false, n }),
        0xA1 => d2(Insn::Neg { b: true, n }),
        0x91 => d2(Insn::Cpl { b: false, n }),
        0xB1 => d2(Insn::Cpl { b: true, n }),
        0xC0 | 0xD0 => d2(Insn::Movx {
            signed: o == 0xD0,
            d: Op::Rw(m),
            s: Op::Rb(n),
        }),
        0xC2 | 0xD2 => d4(Insn::Movx {
            signed: o == 0xD2,
            d: Op::Reg(b1),
            s: Op::Mem(w23),
        }),
        0xC5 | 0xD5 => d4(Insn::Movx {
            signed: o == 0xD5,
            d: Op::Mem(w23),
            s: Op::Reg(b1),
        }),
        0xE0 => d2(Insn::Mov {
            b: false,
            d: Op::Rw(m),
            s: Op::Imm(n as u16),
        }),
        0xE1 => d2(Insn::Mov {
            b: true,
            d: Op::Rb(m),
            s: Op::Imm(n as u16),
        }),
        0xF0 | 0xF1 => {
            let b = o == 0xF1;
            let g = |x| if b { Op::Rb(x) } else { Op::Rw(x) };
            d2(Insn::Mov {
                b,
                d: g(n),
                s: g(m),
            })
        }
        0xE6 => d4(Insn::Mov {
            b: false,
            d: Op::Reg(b1),
            s: Op::Imm(w23),
        }),
        0xE7 => d4(Insn::Mov {
            b: true,
            d: Op::Reg(b1),
            s: Op::Imm(b2 as u16),
        }),
        0xF2 | 0xF3 => d4(Insn::Mov {
            b: o == 0xF3,
            d: Op::Reg(b1),
            s: Op::Mem(w23),
        }),
        0xF6 | 0xF7 => d4(Insn::Mov {
            b: o == 0xF7,
            d: Op::Mem(w23),
            s: Op::Reg(b1),
        }),
        0x84 | 0xA4 => d4(Insn::Mov {
            b: o == 0xA4,
            d: Op::Ind(m),
            s: Op::Mem(w23),
        }),
        0x94 | 0xB4 => d4(Insn::Mov {
            b: o == 0xB4,
            d: Op::Mem(w23),
            s: Op::Ind(m),
        }),
        0xC4 | 0xE4 => {
            let b = o == 0xE4;
            d4(Insn::Mov {
                b,
                d: Op::Off(m, w23),
                s: if b { Op::Rb(n) } else { Op::Rw(n) },
            })
        }
        0xD4 | 0xF4 => {
            let b = o == 0xF4;
            d4(Insn::Mov {
                b,
                d: if b { Op::Rb(n) } else { Op::Rw(n) },
                s: Op::Off(m, w23),
            })
        }
        0x88..=0xE9 if lo == 8 || lo == 9 => {
            let b = lo == 9;
            let g = |x| if b { Op::Rb(x) } else { Op::Rw(x) };
            let (d, s) = match hi {
                0x8 => (Op::PreDec(m), g(n)),
                0x9 => (g(n), Op::PostInc(m)),
                0xA => (g(n), Op::Ind(m)),
                0xB => (Op::Ind(m), g(n)),
                0xC => (Op::Ind(n), Op::Ind(m)),
                0xD => (Op::PostInc(n), Op::Ind(m)),
                _ => (Op::Ind(n), Op::PostInc(m)),
            };
            d2(Insn::Mov { b, d, s })
        }
        _ => {
            let sys = match (o, b1, b2, b3) {
                (0xB7, 0x48, 0xB7, 0xB7) => Some("SRST"),
                (0x87, 0x78, 0x87, 0x87) => Some("IDLE"),
                (0x97, 0x68, 0x97, 0x97) => Some("PWRDN"),
                (0xA7, 0x58, 0xA7, 0xA7) => Some("SRVWDT"),
                (0xA5, 0x5A, 0xA5, 0xA5) => Some("DISWDT"),
                (0xB5, 0x4A, 0xB5, 0xB5) => Some("EINIT"),
                _ => None,
            };
            match sys {
                Some(s) => d4(Insn::Sys(s)),
                None => d2(Insn::Bad),
            }
        }
    }
}

fn cmpid_delta(o: u8) -> i8 {
    match o >> 4 {
        8 => 1,
        9 => 2,
        0xA => -1,
        _ => -2,
    }
}

pub const CC: [&str; 16] = [
    "cc_UC", "cc_NET", "cc_Z", "cc_NZ", "cc_V", "cc_NV", "cc_N", "cc_NN", "cc_C", "cc_NC",
    "cc_SGT", "cc_SLE", "cc_SLT", "cc_SGE", "cc_UGT", "cc_ULE",
];

/// Name of the SFR at a 16-bit address (FE00-FFFF, or ESFR F000-F1FF).
pub fn sfr_name(a: u16) -> Option<&'static str> {
    Some(match a {
        0xFE00 => "DPP0",
        0xFE02 => "DPP1",
        0xFE04 => "DPP2",
        0xFE06 => "DPP3",
        0xFE08 => "CSP",
        0xFE0C => "MDH",
        0xFE0E => "MDL",
        0xFE10 => "CP",
        0xFE12 => "SP",
        0xFE14 => "STKOV",
        0xFE16 => "STKUN",
        0xFE18 => "ADDRSEL1",
        0xFE1A => "ADDRSEL2",
        0xFE1C => "ADDRSEL3",
        0xFE1E => "ADDRSEL4",
        0xFE30 => "PW0",
        0xFE32 => "PW1",
        0xFE34 => "PW2",
        0xFE36 => "PW3",
        0xFE40 => "T2",
        0xFE42 => "T3",
        0xFE44 => "T4",
        0xFE46 => "T5",
        0xFE48 => "T6",
        0xFE4A => "CAPREL",
        0xFE50 => "T0",
        0xFE52 => "T1",
        0xFE54 => "T0REL",
        0xFE56 => "T1REL",
        0xFEA0 => "ADDAT",
        0xFEB0 => "S0TBUF",
        0xFEB2 => "S0RBUF",
        0xFEB4 => "S0BG",
        0xFEC0 => "PECC0",
        0xFEC2 => "PECC1",
        0xFEC4 => "PECC2",
        0xFEC6 => "PECC3",
        0xFEC8 => "PECC4",
        0xFECA => "PECC5",
        0xFECC => "PECC6",
        0xFECE => "PECC7",
        0xFF00 => "P0L",
        0xFF02 => "P0H",
        0xFF04 => "P1L",
        0xFF06 => "P1H",
        0xFF0C => "BUSCON0",
        0xFF0E => "MDC",
        0xFF10 => "PSW",
        0xFF12 => "SYSCON",
        0xFF14 => "BUSCON1",
        0xFF16 => "BUSCON2",
        0xFF18 => "BUSCON3",
        0xFF1A => "BUSCON4",
        0xFF1C => "ZEROS",
        0xFF1E => "ONES",
        0xFF30 => "PWMCON0",
        0xFF32 => "PWMCON1",
        0xFF40 => "T2CON",
        0xFF42 => "T3CON",
        0xFF44 => "T4CON",
        0xFF46 => "T5CON",
        0xFF48 => "T6CON",
        0xFF50 => "T01CON",
        0xFF52 => "CCM0",
        0xFF54 => "CCM1",
        0xFF56 => "CCM2",
        0xFF58 => "CCM3",
        0xFF60 => "T2IC",
        0xFF62 => "T3IC",
        0xFF64 => "T4IC",
        0xFF66 => "T5IC",
        0xFF68 => "T6IC",
        0xFF6A => "CRIC",
        0xFF6C => "S0TIC",
        0xFF6E => "S0RIC",
        0xFF70 => "S0EIC",
        0xFF72 => "SSCTIC",
        0xFF74 => "SSCRIC",
        0xFF76 => "SSCEIC",
        0xFF98 => "ADCIC",
        0xFF9A => "ADEIC",
        0xFF9C => "T0IC",
        0xFF9E => "T1IC",
        0xFFA0 => "ADCON",
        0xFFA2 => "P5",
        0xFFA4 => "P5DIDIS",
        0xFFAC => "TFR",
        0xFFAE => "WDTCON",
        0xFFB0 => "S0CON",
        0xFFB2 => "SSCCON",
        0xFFC0 => "P2",
        0xFFC2 => "DP2",
        0xFFC4 => "P3",
        0xFFC6 => "DP3",
        0xFFC8 => "P4",
        0xFFCA => "DP4",
        0xFFCC => "P6",
        0xFFCE => "DP6",
        0xFFD0 => "P7",
        0xFFD2 => "DP7",
        0xFFD4 => "P8",
        0xFFD6 => "DP8",
        0xF0B0 => "SSCTB",
        0xF0B2 => "SSCRB",
        0xF0B4 => "SSCBR",
        0xF100 => "DP0L",
        0xF102 => "DP0H",
        0xF104 => "DP1L",
        0xF106 => "DP1H",
        0xF186 => "XP0IC",
        0xF18E => "XP1IC",
        0xF196 => "XP2IC",
        0xF19C => "S0TBIC",
        0xF19E => "XP3IC",
        0xF1C0 => "EXICON",
        0xF1C2 => "ODP2",
        0xF1C6 => "ODP3",
        0xF1D2 => "ODP7",
        0xF1D6 => "ODP8",
        0xF1DC => "EXISEL",
        _ => {
            return match a {
                0xFE80..=0xFE9E => Some(CCN[((a - 0xFE80) / 2) as usize]),
                0xFE60..=0xFE7E => Some(CCN[16 + ((a - 0xFE60) / 2) as usize]),
                0xFF78..=0xFF96 => Some(CCIC[((a - 0xFF78) / 2) as usize]),
                0xF160..=0xF17E => Some(CCIC[16 + ((a - 0xF160) / 2) as usize]),
                _ => None,
            };
        }
    })
}

const CCN: [&str; 32] = [
    "CC0", "CC1", "CC2", "CC3", "CC4", "CC5", "CC6", "CC7", "CC8", "CC9", "CC10", "CC11", "CC12",
    "CC13", "CC14", "CC15", "CC16", "CC17", "CC18", "CC19", "CC20", "CC21", "CC22", "CC23", "CC24",
    "CC25", "CC26", "CC27", "CC28", "CC29", "CC30", "CC31",
];
const CCIC: [&str; 32] = [
    "CC0IC", "CC1IC", "CC2IC", "CC3IC", "CC4IC", "CC5IC", "CC6IC", "CC7IC", "CC8IC", "CC9IC",
    "CC10IC", "CC11IC", "CC12IC", "CC13IC", "CC14IC", "CC15IC", "CC16IC", "CC17IC", "CC18IC",
    "CC19IC", "CC20IC", "CC21IC", "CC22IC", "CC23IC", "CC24IC", "CC25IC", "CC26IC", "CC27IC",
    "CC28IC", "CC29IC", "CC30IC", "CC31IC",
];

const RB: [&str; 16] = [
    "RL0", "RH0", "RL1", "RH1", "RL2", "RH2", "RL3", "RH3", "RL4", "RH4", "RL5", "RH5", "RL6",
    "RH6", "RL7", "RH7",
];

/// Address of an 8-bit `reg` field (< 0xF0). `ext` = under EXTR.
pub fn reg_addr(r: u8, ext: bool) -> u16 {
    (if ext { 0xF000 } else { 0xFE00 }) + 2 * r as u16
}

/// Address of a bit-offset field (< 0xF0). `ext` = under EXTR.
pub fn bitoff_addr(q: u8, ext: bool) -> u16 {
    if q < 0x80 {
        0xFD00 + 2 * q as u16
    } else {
        (if ext { 0xF100 } else { 0xFF00 }) + 2 * (q as u16 - 0x80)
    }
}

/// Formatting context: whether EXTR is active (changes reg/bitoff names).
pub struct Fmt<'a> {
    pub d: &'a Decoded,
    pub ext: bool,
}

fn reg_s(r: u8, b: bool, ext: bool) -> String {
    if r >= 0xF0 {
        if b {
            RB[(r & 15) as usize].into()
        } else {
            format!("R{}", r & 15)
        }
    } else {
        let a = reg_addr(r, ext);
        sfr_name(a).map_or_else(|| format!("{a:04X}h"), str::to_string)
    }
}

fn bit_s(q: u8, bit: u8, ext: bool) -> String {
    if q >= 0xF0 {
        format!("R{}.{bit}", q & 15)
    } else {
        let a = bitoff_addr(q, ext);
        match sfr_name(a) {
            Some(nm) => format!("{nm}.{bit}"),
            None => format!("{a:04X}h.{bit}"),
        }
    }
}

fn op_s(o: Op, b: bool, ext: bool) -> String {
    match o {
        Op::Rw(n) => format!("R{n}"),
        Op::Rb(n) => RB[n as usize].into(),
        Op::Reg(r) => reg_s(r, b, ext),
        Op::Mem(a) => match sfr_name(a) {
            Some(nm) if a >= 0xFE00 => nm.to_string(),
            _ => format!("{a:04X}h"),
        },
        Op::Ind(n) => format!("[R{n}]"),
        Op::PostInc(n) => format!("[R{n}+]"),
        Op::PreDec(n) => format!("[-R{n}]"),
        Op::Off(n, d) => format!("[R{n}+#{d:04X}h]"),
        Op::Imm(v) => format!("#{v:X}h"),
    }
}

impl fmt::Display for Fmt<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let ext = self.ext;
        let cc = |c: u8| CC[c as usize];
        match self.d.insn {
            Insn::Alu { op, b, d, s } => {
                let nm = format!("{op:?}").to_uppercase();
                write!(
                    f,
                    "{nm}{} {}, {}",
                    if b { "B" } else { "" },
                    op_s(d, b, ext),
                    op_s(s, b, ext)
                )
            }
            Insn::Mov { b, d, s } => write!(
                f,
                "MOV{} {}, {}",
                if b { "B" } else { "" },
                op_s(d, b, ext),
                op_s(s, b, ext)
            ),
            Insn::Movx { signed, d, s } => write!(
                f,
                "MOVB{} {}, {}",
                if signed { "S" } else { "Z" },
                op_s(d, false, ext),
                op_s(s, true, ext)
            ),
            Insn::Cmpid { delta, n, s } => {
                let nm = match delta {
                    1 => "CMPI1",
                    2 => "CMPI2",
                    -1 => "CMPD1",
                    _ => "CMPD2",
                };
                write!(f, "{nm} R{n}, {}", op_s(s, false, ext))
            }
            Insn::Neg { b, n } => {
                if b {
                    write!(f, "NEGB {}", RB[n as usize])
                } else {
                    write!(f, "NEG R{n}")
                }
            }
            Insn::Cpl { b, n } => {
                if b {
                    write!(f, "CPLB {}", RB[n as usize])
                } else {
                    write!(f, "CPL R{n}")
                }
            }
            Insn::Mul { signed, n, m } => {
                write!(f, "MUL{} R{n}, R{m}", if signed { "" } else { "U" })
            }
            Insn::Div { kind, n } => write!(f, "{} R{n}", format!("{kind:?}").to_uppercase()),
            Insn::Shift { kind, n, s } => write!(
                f,
                "{} R{n}, {}",
                format!("{kind:?}").to_uppercase(),
                op_s(s, false, ext)
            ),
            Insn::Prior { n, m } => write!(f, "PRIOR R{n}, R{m}"),
            Insn::Bclr { q, bit } => write!(f, "BCLR {}", bit_s(q, bit, ext)),
            Insn::Bset { q, bit } => write!(f, "BSET {}", bit_s(q, bit, ext)),
            Insn::Bit2 { op, zq, z, qq, q } => write!(
                f,
                "{} {}, {}",
                format!("{op:?}").to_uppercase(),
                bit_s(zq, z, ext),
                bit_s(qq, q, ext)
            ),
            Insn::Bfld {
                high,
                q,
                mask,
                data,
            } => write!(
                f,
                "BFLD{} {}, #{mask:02X}h, #{data:02X}h",
                if high { "H" } else { "L" },
                if q >= 0xF0 {
                    format!("R{}", q & 15)
                } else {
                    let a = bitoff_addr(q, ext);
                    sfr_name(a).map_or_else(|| format!("{a:04X}h"), str::to_string)
                }
            ),
            Insn::Jmpr { cc: c, t } => write!(f, "JMPR {}, {t:04X}h", cc(c)),
            Insn::Jmpa { cc: c, t } => write!(f, "JMPA {}, {t:04X}h", cc(c)),
            Insn::Jmps { seg, t } => write!(f, "JMPS {seg:02X}h, {t:04X}h"),
            Insn::Jmpi { cc: c, n } => write!(f, "JMPI {}, [R{n}]", cc(c)),
            Insn::Jbit { kind, q, bit, t } => write!(
                f,
                "{} {}, {t:04X}h",
                format!("{kind:?}").to_uppercase(),
                bit_s(q, bit, ext)
            ),
            Insn::Calla { cc: c, t } => write!(f, "CALLA {}, {t:04X}h", cc(c)),
            Insn::Calls { seg, t } => write!(f, "CALLS {seg:02X}h, {t:04X}h"),
            Insn::Calli { cc: c, n } => write!(f, "CALLI {}, [R{n}]", cc(c)),
            Insn::Callr { t } => write!(f, "CALLR {t:04X}h"),
            Insn::Pcall { r, t } => write!(f, "PCALL {}, {t:04X}h", reg_s(r, false, ext)),
            Insn::Ret => write!(f, "RET"),
            Insn::Rets => write!(f, "RETS"),
            Insn::Retp(r) => write!(f, "RETP {}", reg_s(r, false, ext)),
            Insn::Reti => write!(f, "RETI"),
            Insn::Trap(n) => write!(f, "TRAP #{n:02X}h"),
            Insn::Push(r) => write!(f, "PUSH {}", reg_s(r, false, ext)),
            Insn::Pop(r) => write!(f, "POP {}", reg_s(r, false, ext)),
            Insn::Scxt { r, s } => {
                write!(f, "SCXT {}, {}", reg_s(r, false, ext), op_s(s, false, ext))
            }
            Insn::Ext { kind, cnt, arg } => {
                let nm = format!("{kind:?}").to_uppercase();
                match arg {
                    Op::Rw(n) => write!(f, "{nm} R{n}, #{cnt}"),
                    Op::Imm(v) => write!(f, "{nm} #{v:X}h, #{cnt}"),
                    _ => write!(f, "{nm} ?"),
                }
            }
            Insn::Atomic(c) => write!(f, "ATOMIC #{c}"),
            Insn::Extr(c) => write!(f, "EXTR #{c}"),
            Insn::Nop => write!(f, "NOP"),
            Insn::Sys(s) => write!(f, "{s}"),
            Insn::Bad => write!(f, "???"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn s(ip: u16, b: &[u8]) -> String {
        let mut c = [0u8; 4];
        c[..b.len()].copy_from_slice(b);
        let d = decode(ip, c);
        assert_eq!(d.len as usize, b.len(), "length of {b:02X?}");
        Fmt { d: &d, ext: false }.to_string()
    }

    #[test]
    fn boot_sequence() {
        // Reset code of the GK-20 dump at 0x4C6.
        assert_eq!(s(0x4C6, &[0xE6, 0x08, 0x06, 0xF6]), "MOV CP, #F606h");
        assert_eq!(s(0, &[0xFA, 0x05, 0x14, 0x02]), "JMPS 05h, 0214h");
        assert_eq!(s(0, &[0x1A, 0x86, 0x44, 0xD6]), "BFLDH BUSCON0, #D6h, #44h");
        assert_eq!(s(0, &[0x0A, 0x86, 0xFF, 0xBF]), "BFLDL BUSCON0, #FFh, #BFh");
        assert_eq!(s(0, &[0xB5, 0x4A, 0xB5, 0xB5]), "EINIT");
        assert_eq!(s(0, &[0xD1, 0x80]), "EXTR #1");
        assert_eq!(s(0x100, &[0x3D, 0xFE]), "JMPR cc_NZ, 00FEh");
        assert_eq!(s(0, &[0x49, 0x81]), "CMPB RL4, #1h");
        assert_eq!(s(0, &[0xF3, 0xF0, 0xB2, 0xFE]), "MOVB RL0, S0RBUF");
        assert_eq!(s(0, &[0xC0, 0x84]), "MOVBZ R4, RL4");
        assert_eq!(s(0, &[0x5C, 0x24]), "SHL R4, #2h");
        assert_eq!(s(0, &[0xD4, 0x45, 0x10, 0x00]), "MOV R4, [R5+#0010h]");
    }
}
