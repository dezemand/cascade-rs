//! 8XC196 (MCS-96) disassembler.
//!
//! Text follows upstream's `Cpu::disassemble()` style (`LD 1Ch, #0012h`,
//! destination first, hex with an `h` suffix), but the decoding is written
//! for correct instruction lengths, which static analysis needs. Upstream's
//! printer has quirks that do not matter for a trace but would derail a
//! linear sweep, for example 16-bit immediates on byte operations
//! (`LDB C7h, #5200h`) and no `FE` prefix decoding. The core's executor is
//! the reference: the ROM-gated test `disasm_lengths_match_execution`
//! checks every executed instruction's length and branch target.

use std::fmt::Write;

/// How an instruction affects control flow. Targets are 16-bit virtual
/// addresses (the code window maps 0xC000-0xFFFF to a ROM bank).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Flow {
    /// Falls through to the next instruction.
    Next,
    /// Unconditional jump.
    Jump(u16),
    /// Conditional branch: target or fall through.
    Branch(u16),
    /// Subroutine call; returns to the next instruction.
    Call(u16),
    Ret,
    /// Jump through a register or table (`BR [x]`, `TIJMP`).
    Indirect,
    /// Reset, trap, reserved or undecodable opcode.
    Stop,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Insn {
    pub len: u8,
    pub text: String,
    pub flow: Flow,
}

/// Operand addressing mode bits (`aa`, the low two opcode bits).
#[derive(Clone, Copy)]
enum Aa {
    Direct,
    Imm,
    Indirect,
    Indexed,
}

impl Aa {
    fn of(op: u8) -> Self {
        match op & 3 {
            0 => Aa::Direct,
            1 => Aa::Imm,
            2 => Aa::Indirect,
            _ => Aa::Indexed,
        }
    }
}

struct Rd<'a> {
    b: &'a [u8],
    i: usize,
}

impl Rd<'_> {
    fn u8(&mut self) -> Option<u8> {
        let v = *self.b.get(self.i)?;
        self.i += 1;
        Some(v)
    }
    fn u16(&mut self) -> Option<u16> {
        Some(self.u8()? as u16 | (self.u8()? as u16) << 8)
    }
    /// A generic operand (`aop`): returns its text.
    fn aop(&mut self, aa: Aa, byte: bool) -> Option<String> {
        Some(match aa {
            Aa::Direct => format!("{:02X}h", self.u8()?),
            Aa::Imm if byte => format!("#{:02X}h", self.u8()?),
            Aa::Imm => format!("#{:04X}h", self.u16()?),
            Aa::Indirect => {
                let r = self.u8()?;
                format!("[{:02X}h]{}", r & 0xfe, if r & 1 != 0 { "+" } else { "" })
            }
            Aa::Indexed => {
                let r = self.u8()?;
                if r & 1 != 0 {
                    format!("{:04X}h[{:02X}h]", self.u16()?, r & 0xfe)
                } else {
                    format!("{:02X}h[{:02X}h]", self.u8()?, r)
                }
            }
        })
    }
}

/// 3-operand arithmetic, opcodes 0x40-0x5F (`dest = src1 op src2`).
fn three_op(op: u8) -> (&'static str, bool) {
    match op & 0x1c {
        0x00 => ("AND", false),
        0x04 => ("ADD", false),
        0x08 => ("SUB", false),
        0x0c => ("MULU", false),
        0x10 => ("ANDB", true),
        0x14 => ("ADDB", true),
        0x18 => ("SUBB", true),
        _ => ("MULUB", true),
    }
}

/// 2-operand opcodes 0x60-0xBF: mnemonic and whether the *source* is a byte.
fn two_op(op: u8) -> (&'static str, bool) {
    match op & 0xfc {
        0x60 => ("AND", false),
        0x64 => ("ADD", false),
        0x68 => ("SUB", false),
        0x6c => ("MULU", false),
        0x70 => ("ANDB", true),
        0x74 => ("ADDB", true),
        0x78 => ("SUBB", true),
        0x7c => ("MULUB", true),
        0x80 => ("OR", false),
        0x84 => ("XOR", false),
        0x88 => ("CMP", false),
        0x8c => ("DIVU", false),
        0x90 => ("ORB", true),
        0x94 => ("XORB", true),
        0x98 => ("CMPB", true),
        0x9c => ("DIVUB", true),
        0xa0 => ("LD", false),
        0xa4 => ("ADDC", false),
        0xa8 => ("SUBC", false),
        0xac => ("LDBZE", true),
        0xb0 => ("LDB", true),
        0xb4 => ("ADDCB", true),
        0xb8 => ("SUBCB", true),
        _ => ("LDBSE", true),
    }
}

fn sext11(op: u8, lo: u8) -> u16 {
    let v = (((op & 7) as u16) << 8) | lo as u16;
    if v & 0x400 != 0 { v | 0xf800 } else { v }
}

fn rel8(next: u16, d: u8) -> u16 {
    next.wrapping_add(d as i8 as u16)
}

/// Decode one instruction at virtual address `pc` from `bytes` (which start
/// at `pc`). `None` if `bytes` ends inside the instruction.
pub fn decode(bytes: &[u8], pc: u16) -> Option<Insn> {
    let mut r = Rd { b: bytes, i: 0 };
    let op = r.u8()?;
    let mut text = String::new();
    let mut flow = Flow::Next;
    let mut prefix = "";
    let op = if op == 0xfe {
        // signed multiply/divide prefix
        let e = r.u8()?;
        match e {
            0x4c..=0x4f | 0x5c..=0x5f | 0x6c..=0x6f | 0x7c..=0x7f | 0x8c..=0x8f | 0x9c..=0x9f => {
                prefix = "S";
                e
            }
            _ => {
                return Some(Insn {
                    len: 2,
                    text: format!("(FE {e:02X} undecoded)"),
                    flow: Flow::Stop,
                });
            }
        }
    } else {
        op
    };
    let signed = |m: &str| -> String {
        // MULU/MULUB/DIVU/DIVUB become MUL/MULB/DIV/DIVB with the prefix
        if prefix.is_empty() {
            m.to_string()
        } else {
            // MULU -> MUL, MULUB -> MULB, DIVU -> DIV, DIVUB -> DIVB
            m.replacen("LU", "L", 1).replacen("VU", "V", 1)
        }
    };
    match op {
        0x00..=0x03 | 0x05..=0x07 | 0x11..=0x13 | 0x15..=0x17 => {
            let m = [
                "SKIP", "CLR", "NOT", "NEG", "", "DEC", "EXT", "INC", "", "CLRB", "NOTB", "NEGB",
                "", "DECB", "EXTB", "INCB",
            ][(op & 7) as usize + if op >= 0x10 { 8 } else { 0 }];
            write!(text, "{m} {:02X}h", r.u8()?).ok();
        }
        0x04 | 0x14 => {
            let a = r.u8()?;
            let b = r.u8()?;
            let m = if op == 0x04 { "XCH" } else { "XCHB" };
            write!(text, "{m} {b:02X}h, {a:02X}h").ok();
        }
        0x0b | 0x1b => {
            let a = r.aop(Aa::Indexed, false)?;
            let b = r.u8()?;
            let m = if op == 0x0b { "XCH" } else { "XCHB" };
            write!(text, "{m} {b:02X}h, {a}").ok();
        }
        0x08..=0x0a | 0x0c..=0x0f | 0x18..=0x1a => {
            let m = match op {
                0x08 => "SHR",
                0x09 => "SHL",
                0x0a => "SHRA",
                0x0c => "SHRL",
                0x0d => "SHLL",
                0x0e => "SHRAL",
                0x0f => "NORML",
                0x18 => "SHRB",
                0x19 => "SHLB",
                _ => "SHRAB",
            };
            let c = r.u8()?;
            let reg = r.u8()?;
            if c > 15 || op == 0x0f {
                write!(text, "{m} {reg:02X}h, {c:02X}h").ok();
            } else {
                write!(text, "{m} {reg:02X}h, #{c:02X}h").ok();
            }
        }
        0x20..=0x2f => {
            let lo = r.u8()?;
            let t = pc.wrapping_add(2).wrapping_add(sext11(op, lo));
            if op < 0x28 {
                write!(text, "SJMP {t:04X}h").ok();
                flow = Flow::Jump(t);
            } else {
                write!(text, "SCALL {t:04X}h").ok();
                flow = Flow::Call(t);
            }
        }
        0x30..=0x3f => {
            let reg = r.u8()?;
            let t = rel8(pc.wrapping_add(3), r.u8()?);
            let m = if op < 0x38 { "JBC" } else { "JBS" };
            write!(text, "{m} {reg:02X}h, {}, {t:04X}h", op & 7).ok();
            flow = Flow::Branch(t);
        }
        0x40..=0x5f => {
            let (m, byte) = three_op(op);
            let src2 = r.aop(Aa::of(op), byte)?;
            let src1 = r.u8()?;
            let dest = r.u8()?;
            write!(text, "{} {dest:02X}h, {src1:02X}h, {src2}", signed(m)).ok();
        }
        0x60..=0xbf => {
            let (m, byte) = two_op(op);
            let src = r.aop(Aa::of(op), byte)?;
            let dest = r.u8()?;
            write!(text, "{} {dest:02X}h, {src}", signed(m)).ok();
        }
        0xc0 | 0xc2 | 0xc3 | 0xc4 | 0xc6 | 0xc7 => {
            let byte = op >= 0xc4;
            let dest = r.aop(Aa::of(op), byte)?;
            let src = r.u8()?;
            write!(
                text,
                "{} {src:02X}h, {dest}",
                if byte { "STB" } else { "ST" }
            )
            .ok();
        }
        0xc1 | 0xc5 | 0xcd => {
            let a = r.u8()?;
            let b = r.u8()?;
            let m = match op {
                0xc1 => "BMOV",
                0xc5 => "CMPL",
                _ => "BMOVI",
            };
            write!(text, "{m} {b:02X}h, {a:02X}h").ok();
        }
        0xc8..=0xcb => {
            let a = r.aop(Aa::of(op), false)?;
            write!(text, "PUSH {a}").ok();
        }
        0xcc | 0xce | 0xcf => {
            let a = r.aop(Aa::of(op), false)?;
            write!(text, "POP {a}").ok();
        }
        0xd0..=0xdf => {
            let m = [
                "JNST", "JNH", "JGT", "JNC", "JNVT", "JNV", "JGE", "JNE", "JST", "JH", "JLE", "JC",
                "JVT", "JV", "JLT", "JE",
            ][(op & 0xf) as usize];
            let t = rel8(pc.wrapping_add(2), r.u8()?);
            write!(text, "{m} {t:04X}h").ok();
            flow = Flow::Branch(t);
        }
        0xe0 | 0xe1 => {
            let reg = r.u8()?;
            let t = rel8(pc.wrapping_add(3), r.u8()?);
            let m = if op == 0xe0 { "DJNZ" } else { "DJNZW" };
            write!(text, "{m} {reg:02X}h, {t:04X}h").ok();
            flow = Flow::Branch(t);
        }
        0xe2 => {
            let index = r.u8()?;
            let mask = r.u8()?;
            let tbase = r.u8()?;
            write!(text, "TIJMP {tbase:02X}h, [{index:02X}h], #{mask:02X}h").ok();
            flow = Flow::Indirect;
        }
        0xe3 => {
            write!(text, "BR [{:02X}h]", r.u8()?).ok();
            flow = Flow::Indirect;
        }
        0xe7 | 0xef => {
            let t = pc.wrapping_add(3).wrapping_add(r.u16()?);
            if op == 0xe7 {
                write!(text, "LJMP {t:04X}h").ok();
                flow = Flow::Jump(t);
            } else {
                write!(text, "LCALL {t:04X}h").ok();
                flow = Flow::Call(t);
            }
        }
        0xf6 => {
            write!(text, "IDLPD #{:02X}h", r.u8()?).ok();
        }
        0xf0 => {
            text.push_str("RET");
            flow = Flow::Ret;
        }
        0xec | 0xed | 0xf2..=0xf5 | 0xf8..=0xfd => {
            text.push_str(match op {
                0xec => "DPTS",
                0xed => "EPTS",
                0xf2 => "PUSHF",
                0xf3 => "POPF",
                0xf4 => "PUSHA",
                0xf5 => "POPA",
                0xf8 => "CLRC",
                0xf9 => "SETC",
                0xfa => "DI",
                0xfb => "EI",
                0xfc => "CLRVT",
                _ => "NOP",
            });
        }
        0xf7 => {
            text.push_str("TRAP");
            flow = Flow::Stop;
        }
        0xff => {
            text.push_str("RST");
            flow = Flow::Stop;
        }
        _ => {
            // 0x10, 0x1c-0x1f, 0xe4-0xe6, 0xe8-0xeb, 0xee, 0xf1, 0xfe
            write!(text, "(reserved {op:02X})").ok();
            flow = Flow::Stop;
        }
    }
    Some(Insn {
        len: r.i as u8,
        text,
        flow,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn d(b: &[u8], pc: u16) -> (u8, String, Flow) {
        let i = decode(b, pc).unwrap();
        (i.len, i.text, i.flow)
    }

    #[test]
    fn operand_modes_and_lengths() {
        assert_eq!(d(&[0xa1, 0x34, 0x12, 0x1c], 0).0, 4); // LD 1C, #1234
        assert_eq!(d(&[0xa1, 0x34, 0x12, 0x1c], 0).1, "LD 1Ch, #1234h");
        assert_eq!(d(&[0xb1, 0x52, 0xc7], 0).1, "LDB C7h, #52h");
        assert_eq!(d(&[0xad, 0x52, 0x1c], 0).1, "LDBZE 1Ch, #52h");
        assert_eq!(d(&[0xa2, 0x21, 0x1c], 0).1, "LD 1Ch, [20h]+");
        assert_eq!(d(&[0xa3, 0x20, 0x05, 0x1c], 0).1, "LD 1Ch, 05h[20h]");
        assert_eq!(
            d(&[0xa3, 0x21, 0x00, 0x40, 0x1c], 0).1,
            "LD 1Ch, 4000h[20h]"
        );
        assert_eq!(
            d(&[0x45, 0x10, 0x00, 0x1e, 0x1c], 0).1,
            "ADD 1Ch, 1Eh, #0010h"
        );
        assert_eq!(
            d(&[0xc3, 0x21, 0x00, 0x40, 0x1c], 0).1,
            "ST 1Ch, 4000h[20h]"
        );
        assert_eq!(d(&[0xfe, 0x6c, 0x1e, 0x1c], 0).1, "MUL 1Ch, 1Eh");
        assert_eq!(d(&[0x0f, 0x1e, 0x1c], 0).0, 3);
    }

    #[test]
    fn control_flow_targets() {
        assert_eq!(d(&[0x27, 0xfe], 0x2100).2, Flow::Jump(0x2100)); // SJMP $
        assert_eq!(
            d(&[0x2f, 0xff], 0x2100),
            (2, "SCALL 2101h".into(), Flow::Call(0x2101))
        );
        assert_eq!(d(&[0xd7, 0x10], 0x2100).2, Flow::Branch(0x2112));
        assert_eq!(d(&[0x37, 0x20, 0xfd], 0x2100).2, Flow::Branch(0x2100));
        assert_eq!(d(&[0xef, 0x00, 0x10], 0xfff0).2, Flow::Call(0x0ff3));
        assert_eq!(d(&[0xf0], 0).2, Flow::Ret);
    }

    #[test]
    fn truncated_input() {
        assert!(decode(&[0xa1, 0x34], 0).is_none());
        assert!(decode(&[], 0).is_none());
    }
}
