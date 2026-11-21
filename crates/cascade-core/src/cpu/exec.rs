//! Opcode interpreter (port of the `switch` in `cpu_emu.cpp`).
//!
//! Kept in upstream's case order so the two can be read side by side.
//! Integer behaviour follows what the C++ actually computes, including
//! integer promotion (e.g. `val16 - 1` is an `int`) and shift counts beyond
//! the operand width (masked, as the arm64 oracle binary does).

use super::{Cpu, PSW_C, PSW_INTE, PSW_N, PSW_PTSE, PSW_ST, PSW_V, PSW_VT, PSW_Z, StepEvent};
use crate::stop::{R, Stop};

/// Sign-extend the 11-bit SJMP/SCALL displacement.
fn rel11(op: u8, lo: u8) -> u16 {
    let v = ((lo as u16 | (((op & 7) as u16) << 8)) << 5) as i16 >> 5;
    v as u16
}

impl Cpu {
    // Flag helpers shared by several opcodes (upstream's goto labels).

    /// `set_psw_add16`: V is never set upstream (`(res & 0x8000) == 1`).
    fn psw_add16(&mut self, val: u16, res: u16) {
        self.psw &= !(PSW_Z | PSW_N | PSW_C | PSW_V);
        if res < val {
            self.psw |= PSW_C;
        }
        if res == 0 {
            self.psw |= PSW_Z;
        }
        if res >= 0x8000 {
            self.psw |= PSW_N;
        }
    }

    fn psw_add8(&mut self, val: u8, res: u8) {
        self.psw &= !(PSW_Z | PSW_N | PSW_C | PSW_V);
        if res < val {
            self.psw |= PSW_C;
        }
        if res == 0 {
            self.psw |= PSW_Z;
        }
        if res >= 0x80 {
            self.psw |= PSW_N;
        }
    }

    /// `set_psw_sub16`: C means "no borrow"; V never set upstream.
    fn psw_sub16(&mut self, val: u16, imm: u16, res: u16) {
        self.psw &= !(PSW_Z | PSW_N | PSW_C | PSW_V);
        if val >= imm {
            self.psw |= PSW_C;
        }
        if res == 0 {
            self.psw |= PSW_Z;
        }
        if res >= 0x8000 {
            self.psw |= PSW_N;
        }
    }

    fn psw_sub8(&mut self, val: u8, imm: u8, res: u8) {
        self.psw &= !(PSW_Z | PSW_N | PSW_C | PSW_V);
        if val >= imm {
            self.psw |= PSW_C;
        }
        if res == 0 {
            self.psw |= PSW_Z;
        }
        if res >= 0x80 {
            self.psw |= PSW_N;
        }
    }

    fn psw_logical16(&mut self, res: u16) {
        self.psw &= !(PSW_C | PSW_V | PSW_N | PSW_Z);
        if res & 0x8000 != 0 {
            self.psw |= PSW_N;
        }
        if res == 0 {
            self.psw |= PSW_Z;
        }
    }

    fn psw_logical8(&mut self, res: u8) {
        self.psw &= !(PSW_C | PSW_V | PSW_N | PSW_Z);
        if res & 0x80 != 0 {
            self.psw |= PSW_N;
        }
        if res == 0 {
            self.psw |= PSW_Z;
        }
    }

    /// Shift count operand: immediate if <= 15, else a register holding it.
    fn shift_count(&mut self) -> R<u8> {
        let imm8 = self.fetch()?;
        if imm8 > 15 {
            self.rd8(imm8 as u16)
        } else {
            Ok(imm8)
        }
    }

    /// `[a8]` / `[a8]+` operand: returns (pointer register, address).
    fn indirect(&mut self) -> R<(u8, u16)> {
        let a = self.fetch()?;
        let p = self.rd16((a & 0xfe) as u16)?;
        Ok((a, p))
    }

    /// Auto-increment for `[a8]+` (bit 0 of the register byte).
    fn autoinc(&mut self, a: u8, p: u16, by: u16) -> R<()> {
        if a & 1 != 0 {
            self.wr16((a & 0xfe) as u16, p.wrapping_add(by))?;
            self.cycle(1);
        }
        Ok(())
    }

    fn cond_jump(&mut self, taken: bool) -> R<()> {
        let rel = self.fetch()? as i8;
        if taken {
            self.pc = self.pc.wrapping_add(rel as u16);
            self.cycle(4);
        }
        self.cycle(4);
        Ok(())
    }

    fn illegal(&self, opcode: u8, ext: Option<u8>, opc: u16) -> Stop {
        Stop::IllegalOpcode {
            opcode,
            ext,
            pc: opc,
        }
    }

    pub(super) fn execute(&mut self) -> R<StepEvent> {
        let opc = self.pc;
        let opcode = self.fetch()?;
        self.stats.opcodes[opcode as usize] += 1;
        let psw_c = |s: &Self| s.psw & PSW_C != 0;
        match opcode {
            0x00 => {
                // skip
                self.fetch()?;
                self.cycle(3);
            }
            0x01 => {
                // clr
                let a = self.fetch()? as u16;
                self.wr16(a, 0)?;
                self.psw &= !(PSW_N | PSW_C | PSW_V);
                self.psw |= PSW_Z;
                self.cycle(3);
            }
            0x02 => {
                // not
                let a = self.fetch()? as u16;
                let res = !self.rd16(a)?;
                self.wr16(a, res)?;
                self.psw &= !(PSW_Z | PSW_N | PSW_C | PSW_V);
                if res == 0 {
                    self.psw |= PSW_Z;
                }
                if res & 0x8000 != 0 {
                    self.psw |= PSW_N;
                }
                self.cycle(3);
            }
            0x03 => {
                // neg
                let a = self.fetch()? as u16;
                let res = (self.rd16(a)? as i16).wrapping_neg();
                self.wr16(a, res as u16)?;
                self.psw &= !(PSW_Z | PSW_N | PSW_C | PSW_V);
                if res == 0 {
                    self.psw |= PSW_Z | PSW_C;
                }
                if res < 0 {
                    self.psw |= PSW_N | PSW_V | PSW_VT;
                }
                self.cycle(3);
            }
            0x05 => {
                // dec: res32 = val16 - 1 as int
                let a = self.fetch()? as u16;
                let val = self.rd16(a)?;
                let res = (val as i32 - 1) as u32;
                self.wr16(a, res as u16)?;
                self.psw &= !(PSW_Z | PSW_N | PSW_C | PSW_V);
                if val != 0 {
                    self.psw |= PSW_C;
                }
                if res == 0 {
                    self.psw |= PSW_Z;
                }
                if res & 0x8000 != 0 {
                    self.psw |= PSW_N;
                }
                self.cycle(3);
            }
            0x06 => {
                // ext
                let a = self.fetch()? as u16;
                let res = self.rd16(a)? as i16 as i32;
                self.wr32(a, res as u32)?;
                self.psw &= !(PSW_Z | PSW_N | PSW_C | PSW_V);
                if res == 0 {
                    self.psw |= PSW_Z;
                }
                if res < 0 {
                    self.psw |= PSW_N;
                }
                self.cycle(4);
            }
            0x07 => {
                // inc: res32 = val16 + 1 never wraps, so Z is never set
                let a = self.fetch()? as u16;
                let val = self.rd16(a)?;
                let res = val as u32 + 1;
                self.wr16(a, res as u16)?;
                self.psw &= !(PSW_Z | PSW_N | PSW_C | PSW_V | PSW_ST);
                if val == 0xffff {
                    self.psw |= PSW_C;
                }
                if res == 0 {
                    self.psw |= PSW_Z;
                }
                if res & 0x8000 != 0 {
                    self.psw |= PSW_N;
                }
                self.cycle(3);
            }
            0x08 => {
                // shr
                let n = self.shift_count()?;
                let a = self.fetch()? as u16;
                let val = self.rd16(a)?;
                let res = ((val as u32) << 16).wrapping_shr(n as u32);
                self.wr16(a, (res >> 16) as u16)?;
                self.cycle_shift(6, n);
                self.psw &= !(PSW_Z | PSW_N | PSW_C | PSW_V | PSW_ST);
                if res >> 16 == 0 {
                    self.psw |= PSW_Z;
                }
                if res & 0x8000 != 0 {
                    self.psw |= PSW_C;
                }
                if res & 0x7fff != 0 {
                    self.psw |= PSW_ST;
                }
            }
            0x09 => {
                // shl
                let n = self.shift_count()?;
                let a = self.fetch()? as u16;
                let val = self.rd16(a)?;
                let res = (val as u32).wrapping_shl(n as u32);
                self.wr16(a, res as u16)?;
                self.cycle_shift(6, n);
                self.psw &= !(PSW_Z | PSW_N | PSW_C | PSW_V);
                if res & 0xffff == 0 {
                    self.psw |= PSW_Z;
                }
                if res & 0x8000 != 0 {
                    self.psw |= PSW_N;
                }
                if res & 0x10000 != 0 {
                    self.psw |= PSW_C;
                }
                if (res & 0x8000) != (val as u32 & 0x8000) {
                    self.psw |= PSW_V | PSW_VT;
                }
            }
            0x0a => {
                // shra
                let n = self.shift_count()?;
                let a = self.fetch()? as u16;
                let val = self.rd16(a)?;
                let sres = ((val as i16 as i32) << 16).wrapping_shr(n as u32);
                self.wr16(a, (sres >> 16) as u16)?;
                let res = sres as u32;
                self.cycle_shift(6, n);
                self.psw &= !(PSW_Z | PSW_N | PSW_C | PSW_V | PSW_ST);
                if res >> 16 == 0 {
                    self.psw |= PSW_Z;
                }
                if res & 0x8000 != 0 {
                    self.psw |= PSW_C;
                }
                if res & 0x8000_0000 != 0 {
                    self.psw |= PSW_N;
                }
                if res & 0x7fff != 0 {
                    self.psw |= PSW_ST;
                }
            }
            0x0c => {
                // shrl
                let n = self.shift_count()?;
                let a = self.fetch()? as u16;
                let val = self.rd32(a)?;
                let res = ((val as u64) << 32).wrapping_shr(n as u32);
                self.wr32(a, (res >> 32) as u32)?;
                self.cycle_shift(7, n);
                self.psw &= !(PSW_Z | PSW_N | PSW_C | PSW_V | PSW_ST);
                if res >> 32 == 0 {
                    self.psw |= PSW_Z;
                }
                if res & 0x8000_0000 != 0 {
                    self.psw |= PSW_C;
                }
                if res & 0x7fff_ffff != 0 {
                    self.psw |= PSW_ST;
                }
            }
            0x0d => {
                // shll
                let n = self.shift_count()?;
                let a = self.fetch()? as u16;
                let val = self.rd32(a)?;
                let res = (val as u64).wrapping_shl(n as u32);
                self.wr32(a, res as u32)?;
                self.cycle_shift(7, n);
                self.psw &= !(PSW_Z | PSW_N | PSW_C | PSW_V);
                if res & 0xffff_ffff == 0 {
                    self.psw |= PSW_Z;
                }
                if res & 0x8000_0000 != 0 {
                    self.psw |= PSW_N;
                }
                if res & 0x1_0000_0000 != 0 {
                    self.psw |= PSW_C;
                }
            }
            0x0e => {
                // shral
                let n = self.shift_count()?;
                let a = self.fetch()? as u16;
                let val = self.rd32(a)?;
                let sres = ((val as i32 as i64) << 32).wrapping_shr(n as u32);
                self.wr32(a, (sres >> 32) as u32)?;
                let res = sres as u64;
                self.cycle_shift(7, n);
                self.psw &= !(PSW_Z | PSW_N | PSW_C | PSW_V | PSW_ST);
                if res >> 32 == 0 {
                    self.psw |= PSW_Z;
                }
                if res & 0x8000_0000 != 0 {
                    self.psw |= PSW_C;
                }
                if res & 0x8000_0000_0000_0000 != 0 {
                    self.psw |= PSW_N;
                }
                if res & 0x7fff_ffff != 0 {
                    self.psw |= PSW_ST;
                }
            }
            0x0f => {
                // norml
                let a = self.fetch()? as u16;
                let b = self.fetch()? as u16;
                let mut val = self.rd32(b)?;
                let mut n: u8 = 0;
                while n < 31 && val & 0x8000_0000 == 0 {
                    val <<= 1;
                    n += 1;
                }
                self.wr32(b, val)?;
                self.wr8(a, n)?;
                self.psw &= !(PSW_Z | PSW_C);
                if n == 31 && val & 0x8000_0000 == 0 {
                    self.psw |= PSW_Z;
                }
                self.cycle_shift(8, n);
            }
            0x11 => {
                // clrb
                let a = self.fetch()? as u16;
                self.wr8(a, 0)?;
                self.psw &= !(PSW_N | PSW_C | PSW_V);
                self.psw |= PSW_Z;
                self.cycle(3);
            }
            0x12 => {
                // notb
                let a = self.fetch()? as u16;
                let res = !self.rd8(a)?;
                self.wr8(a, res)?;
                self.psw &= !(PSW_Z | PSW_N | PSW_C | PSW_V);
                if res == 0 {
                    self.psw |= PSW_Z;
                }
                if res & 0x80 != 0 {
                    self.psw |= PSW_N;
                }
                self.cycle(3);
            }
            0x13 => {
                // negb
                let a = self.fetch()? as u16;
                let res = (self.rd8(a)? as i8).wrapping_neg();
                self.wr8(a, res as u8)?;
                self.psw &= !(PSW_Z | PSW_N | PSW_C | PSW_V);
                if res == 0 {
                    self.psw |= PSW_Z | PSW_C;
                }
                if res < 0 {
                    self.psw |= PSW_N | PSW_V | PSW_VT;
                }
                self.cycle(3);
            }
            0x14 => {
                // xchb: upstream reads 16 bits and keeps the low byte
                let a = self.fetch()? as u16;
                let v1 = self.rd16(a)? as u8;
                let b = self.fetch()? as u16;
                let v2 = self.rd16(b)? as u8;
                self.wr8(a, v2)?;
                self.wr8(b, v1)?;
                self.cycle(5);
            }
            0x15 => {
                // decb: res16 = val8 - 1 as int, stored in uint16
                let a = self.fetch()? as u16;
                let val = self.rd8(a)?;
                let res = (val as i32 - 1) as u16;
                self.wr8(a, res as u8)?;
                self.psw &= !(PSW_Z | PSW_N | PSW_C | PSW_V);
                if val != 0 {
                    self.psw |= PSW_C;
                }
                if res == 0 {
                    self.psw |= PSW_Z;
                }
                if res & 0x80 != 0 {
                    self.psw |= PSW_N;
                }
                self.cycle(3);
            }
            0x17 => {
                // incb
                let a = self.fetch()? as u16;
                let val = self.rd8(a)?;
                let res = val.wrapping_add(1);
                self.wr8(a, res)?;
                self.cycle(3);
                self.psw_add8(val, res);
            }
            0x18 => {
                // shrb
                let n = self.shift_count()?;
                let a = self.fetch()? as u16;
                let val = self.rd8(a)?;
                let res = ((val as u32) << 8).wrapping_shr(n as u32) as u16;
                self.wr8(a, (res >> 8) as u8)?;
                self.cycle_shift(6, n);
                self.psw &= !(PSW_Z | PSW_N | PSW_C | PSW_V | PSW_ST);
                if res >> 8 == 0 {
                    self.psw |= PSW_Z;
                }
                if res & 0x80 != 0 {
                    self.psw |= PSW_C;
                }
                if res & 0x7f != 0 {
                    self.psw |= PSW_ST;
                }
            }
            0x19 => {
                // shlb
                let n = self.shift_count()?;
                let a = self.fetch()? as u16;
                let val = self.rd8(a)?;
                let res = (val as u32).wrapping_shl(n as u32) as u16;
                self.wr8(a, res as u8)?;
                self.cycle_shift(6, n);
                self.psw &= !(PSW_Z | PSW_N | PSW_C | PSW_V);
                if res & 0xff == 0 {
                    self.psw |= PSW_Z;
                }
                if res & 0x80 != 0 {
                    self.psw |= PSW_N;
                }
                if res & 0x100 != 0 {
                    self.psw |= PSW_C;
                }
                if (res & 0x80) != (val as u16 & 0x80) {
                    self.psw |= PSW_V | PSW_VT;
                }
            }
            0x20..=0x27 => {
                // sjmp
                let lo = self.fetch()?;
                let target = self.pc.wrapping_add(rel11(opcode, lo));
                if self.pc >= 2 && target == self.pc - 2 {
                    let phys = self.virt_to_phys(opc, true);
                    let cycles = self.cycles;
                    self.reset();
                    return Ok(StepEvent::EndlessLoopReset {
                        pc: opc,
                        phys,
                        cycles,
                    });
                }
                self.pc = target;
                self.cycle(7);
            }
            0x28..=0x2f => {
                // scall
                let lo = self.fetch()?;
                let target = self.pc.wrapping_add(rel11(opcode, lo));
                self.push16(self.pc)?;
                self.pc = target;
                self.cycle(11);
            }
            0x30..=0x3f => {
                // jbc / jbs
                let mask = 1u8 << (opcode & 7);
                let a = self.fetch()? as u16;
                let rel = self.fetch()? as i8;
                let target = self.pc.wrapping_add(rel as u16);
                let set = mask & self.rd8(a)? != 0;
                if set == (opcode >= 0x38) {
                    self.pc = target;
                    self.cycle(4);
                }
                self.cycle(5);
            }
            0x40..=0x43 => {
                // and c, b, x
                let val = match opcode {
                    0x40 => {
                        let a = self.fetch()? as u16;
                        let v = self.rd16(a)?;
                        self.cycle(5);
                        v
                    }
                    0x41 => {
                        let v = self.fetch16()?;
                        self.cycle(6);
                        v
                    }
                    0x42 => {
                        let (a, p) = self.indirect()?;
                        let v = self.rd16(p)?;
                        self.autoinc(a, p, 2)?;
                        self.cycle_rm3(7, p);
                        v
                    }
                    _ => {
                        let p = self.indexed_addr()?;
                        let v = self.rd16(p)?;
                        self.cycle_rm3(7, p);
                        v
                    }
                };
                let b = self.fetch()? as u16;
                let res = val & self.rd16(b)?;
                let c = self.fetch()? as u16;
                self.wr16(c, res)?;
                self.psw_logical16(res);
            }
            0x44 | 0x45 | 0x47 => {
                // add c, b, x
                let val = match opcode {
                    0x44 => {
                        let a = self.fetch()? as u16;
                        let v = self.rd16(a)?;
                        self.cycle(5);
                        v
                    }
                    0x45 => {
                        let v = self.fetch16()?;
                        self.cycle(6);
                        v
                    }
                    _ => {
                        let p = self.indexed_addr()?;
                        let v = self.rd16(p)?;
                        self.cycle_rm3(7, p);
                        v
                    }
                };
                let b = self.fetch()? as u16;
                let res = self.rd16(b)?.wrapping_add(val);
                let c = self.fetch()? as u16;
                self.wr16(c, res)?;
                self.psw_add16(val, res);
            }
            0x48 | 0x4b => {
                // sub c, b, x
                let imm = if opcode == 0x48 {
                    let a = self.fetch()? as u16;
                    let v = self.rd16(a)?;
                    self.cycle(5);
                    v
                } else {
                    let p = self.indexed_addr()?;
                    let v = self.rd16(p)?;
                    self.cycle_rm3(7, p);
                    v
                };
                let b = self.fetch()? as u16;
                let val = self.rd16(b)?;
                let res = val.wrapping_sub(imm);
                let c = self.fetch()? as u16;
                self.wr16(c, res)?;
                self.psw_sub16(val, imm, res);
            }
            0x4c | 0x4d | 0x4f => {
                // mulu c, b, x
                let imm = match opcode {
                    0x4c => {
                        let a = self.fetch()? as u16;
                        let v = self.rd16(a)?;
                        self.cycle(14);
                        v
                    }
                    0x4d => {
                        let v = self.fetch16()?;
                        self.cycle(15);
                        v
                    }
                    _ => {
                        let p = self.indexed_addr()?;
                        let v = self.rd16(p)?;
                        self.cycle_rm3(17, p);
                        v
                    }
                };
                let b = self.fetch()? as u16;
                let c = self.fetch()? as u16;
                let res = (self.rd16(b)? as u32).wrapping_mul(imm as u32);
                self.wr32(c, res)?;
            }
            0x50 | 0x51 => {
                // andb c, b, x
                let imm = if opcode == 0x50 {
                    let a = self.fetch()? as u16;
                    let v = self.rd8(a)?;
                    self.cycle(5);
                    v
                } else {
                    let v = self.fetch()?;
                    self.cycle(6);
                    v
                };
                let b = self.fetch()? as u16;
                let res = self.rd8(b)? & imm;
                let c = self.fetch()? as u16;
                self.wr8(c, res)?;
                self.psw_logical8(res);
            }
            0x54 | 0x55 | 0x57 => {
                // addb c, b, x
                let v2 = match opcode {
                    0x54 => {
                        let a = self.fetch()? as u16;
                        let v = self.rd8(a)?;
                        self.cycle(5);
                        v
                    }
                    0x55 => {
                        let v = self.fetch()?;
                        self.cycle(6);
                        v
                    }
                    _ => {
                        let p = self.indexed_addr()?;
                        let v = self.rd8(p)?;
                        self.cycle_rm3(7, p);
                        v
                    }
                };
                let b = self.fetch()? as u16;
                let val = self.rd8(b)?;
                let c = self.fetch()? as u16;
                let res = val.wrapping_add(v2);
                self.wr8(c, res)?;
                self.psw_add8(val, res);
            }
            0x58 | 0x5b => {
                // subb c, b, x
                let imm = if opcode == 0x58 {
                    let a = self.fetch()? as u16;
                    let v = self.rd8(a)?;
                    self.cycle(5);
                    v
                } else {
                    let p = self.indexed_addr()?;
                    let v = self.rd8(p)?;
                    self.cycle_rm3(7, p);
                    v
                };
                let b = self.fetch()? as u16;
                let val = self.rd8(b)?;
                let res = val.wrapping_sub(imm);
                let c = self.fetch()? as u16;
                self.wr8(c, res)?;
                self.psw_sub8(val, imm, res);
            }
            0x5c | 0x5d => {
                // mulub c, b, x
                let imm = if opcode == 0x5c {
                    let a = self.fetch()? as u16;
                    self.rd8(a)?
                } else {
                    self.fetch()?
                };
                self.cycle(10);
                let b = self.fetch()? as u16;
                let c = self.fetch()? as u16;
                let res = self.rd8(b)? as u16 * imm as u16;
                self.wr16(c, res)?;
            }
            0x60 | 0x61 | 0x63 => {
                // and b, x
                let imm = match opcode & 3 {
                    0 => {
                        let a = self.fetch()? as u16;
                        let v = self.rd16(a)?;
                        self.cycle(4);
                        v
                    }
                    1 => {
                        let v = self.fetch16()?;
                        self.cycle(5);
                        v
                    }
                    _ => {
                        let (a, p) = self.indirect()?;
                        let v = self.rd16(p)?;
                        self.autoinc(a, p, 2)?;
                        self.cycle(6);
                        v
                    }
                };
                let a = self.fetch()? as u16;
                let res = imm & self.rd16(a)?;
                self.wr16(a, res)?;
                self.psw_logical16(res);
            }
            0x64..=0x66 => {
                // add b, x
                let val = match opcode {
                    0x64 => {
                        let a = self.fetch()? as u16;
                        let v = self.rd16(a)?;
                        self.cycle(4);
                        v
                    }
                    0x65 => {
                        let v = self.fetch16()?;
                        self.cycle(5);
                        v
                    }
                    _ => {
                        let (a, p) = self.indirect()?;
                        let v = self.rd16(p)?;
                        self.autoinc(a, p, 2)?;
                        self.cycle(6);
                        v
                    }
                };
                let a = self.fetch()? as u16;
                let res = self.rd16(a)?.wrapping_add(val);
                self.wr16(a, res)?;
                self.psw_add16(val, res);
            }
            0x67 => {
                // add b, indexed
                let t = self.indexed_addr()?;
                let val = self.rd16(t)?;
                let a = self.fetch()? as u16;
                let res = self.rd16(a)?.wrapping_add(val);
                self.wr16(a, res)?;
                self.cycle_rm2(6, t);
                self.psw_add16(val, res);
            }
            0x68 | 0x69 => {
                // sub b, x
                let imm = if opcode == 0x68 {
                    let a = self.fetch()? as u16;
                    let v = self.rd16(a)?;
                    self.cycle(4);
                    v
                } else {
                    let v = self.fetch16()?;
                    self.cycle(5);
                    v
                };
                let a = self.fetch()? as u16;
                let val = self.rd16(a)?;
                let res = val.wrapping_sub(imm);
                self.wr16(a, res)?;
                self.psw_sub16(val, imm, res);
            }
            0x6a | 0x8a => {
                // sub / cmp b, [a]+
                let (a, p) = self.indirect()?;
                let imm = self.rd16(p)?;
                let t = self.fetch()? as u16;
                let val = self.rd16(t)?;
                let res = val.wrapping_sub(imm);
                if opcode == 0x6a {
                    self.wr16(t, res)?;
                }
                self.autoinc(a, p, 2)?;
                self.cycle_rm2(6, p);
                self.psw_sub16(val, imm, res);
            }
            0x6b => {
                // sub b, indexed
                let p = self.indexed_addr()?;
                let imm = self.rd16(p)?;
                let t = self.fetch()? as u16;
                let val = self.rd16(t)?;
                let res = val.wrapping_sub(imm);
                self.wr16(t, res)?;
                self.cycle_rm2(6, p);
                self.psw_sub16(val, imm, res);
            }
            0x6c..=0x6f => {
                // mulu b, x
                let imm = match opcode {
                    0x6c => {
                        let a = self.fetch()? as u16;
                        let v = self.rd16(a)?;
                        self.cycle(14);
                        v
                    }
                    0x6d => {
                        let v = self.fetch16()?;
                        self.cycle(15);
                        v
                    }
                    0x6e => {
                        let (a, p) = self.indirect()?;
                        let v = self.rd16(p)?;
                        self.autoinc(a, p, 2)?;
                        self.cycle_rm3(16, p);
                        v
                    }
                    _ => {
                        let p = self.indexed_addr()?;
                        let v = self.rd16(p)?;
                        self.cycle_rm3(17, p);
                        v
                    }
                };
                let a = self.fetch()? as u16;
                let res = (imm as u32).wrapping_mul(self.rd16(a)? as u32);
                self.wr32(a, res)?;
            }
            0x70..=0x73 | 0x90..=0x93 | 0x94..=0x97 => {
                // andb / orb / xorb b, x
                let kind = opcode & 0xfc;
                let imm = match opcode & 3 {
                    3 => {
                        let p = self.indexed_addr()?;
                        let v = self.rd8(p)?;
                        self.cycle_rm2(6, p);
                        v
                    }
                    2 => {
                        let (a, p) = self.indirect()?;
                        let v = self.rd8(p)?;
                        self.autoinc(a, p, 1)?;
                        self.cycle_rm2(6, p);
                        v
                    }
                    1 => {
                        let v = self.fetch()?;
                        self.cycle(5);
                        v
                    }
                    _ => {
                        let a = self.fetch()? as u16;
                        let v = self.rd8(a)?;
                        self.cycle(4);
                        v
                    }
                };
                let t = self.fetch()? as u16;
                let cur = self.rd8(t)?;
                let res = match kind {
                    0x70 => cur & imm,
                    0x90 => cur | imm,
                    _ => cur ^ imm,
                };
                self.wr8(t, res)?;
                self.psw_logical8(res);
            }
            0x74..=0x7b => {
                // addb / subb b, x
                let imm = match opcode & 3 {
                    0 => {
                        let a = self.fetch()? as u16;
                        let v = self.rd8(a)?;
                        self.cycle(4);
                        v
                    }
                    1 => {
                        let v = self.fetch()?;
                        self.cycle(5);
                        v
                    }
                    2 => {
                        let (a, p) = self.indirect()?;
                        let v = self.rd8(p)?;
                        self.autoinc(a, p, 1)?;
                        self.cycle_rm2(6, p);
                        v
                    }
                    _ => {
                        let p = self.indexed_addr()?;
                        let v = self.rd8(p)?;
                        self.cycle_rm2(6, p);
                        v
                    }
                };
                let a = self.fetch()? as u16;
                let val = self.rd8(a)?;
                if opcode < 0x78 {
                    let res = val.wrapping_add(imm);
                    self.wr8(a, res)?;
                    self.psw_add8(val, res);
                } else {
                    let res = val.wrapping_sub(imm);
                    self.wr8(a, res)?;
                    self.psw_sub8(val, imm, res);
                }
            }
            0x7d => {
                // mulub b, #imm
                let imm = self.fetch()?;
                let t = self.fetch()? as u16;
                let res = self.rd8(t)? as u16 * imm as u16;
                self.wr16(t, res)?;
                self.cycle(10);
            }
            0x80 | 0x81 | 0x83 | 0x84 | 0x87 => {
                // or / xor b, x
                let val = match opcode {
                    0x80 | 0x84 => {
                        let a = self.fetch()? as u16;
                        let v = self.rd16(a)?;
                        self.cycle(4);
                        v
                    }
                    0x81 => {
                        let v = self.fetch16()?;
                        self.cycle(5);
                        v
                    }
                    _ => {
                        let p = self.indexed_addr()?;
                        let v = self.rd16(p)?;
                        self.cycle_rm2(6, p);
                        v
                    }
                };
                let a = self.fetch()? as u16;
                let cur = self.rd16(a)?;
                let res = if opcode < 0x84 { val | cur } else { val ^ cur };
                self.wr16(a, res)?;
                self.psw_logical16(res);
            }
            0x88 | 0x89 => {
                // cmp b, x
                let imm = if opcode == 0x88 {
                    let a = self.fetch()? as u16;
                    self.rd16(a)?
                } else {
                    self.fetch16()?
                };
                let b = self.fetch()? as u16;
                let val = self.rd16(b)?;
                let res = val.wrapping_sub(imm);
                self.cycle(if opcode == 0x88 { 4 } else { 5 });
                self.psw_sub16(val, imm, res);
            }
            0x8b => {
                // cmp b, indexed
                let t = self.indexed_addr()?;
                let imm = self.rd16(t)?;
                let b = self.fetch()? as u16;
                let val = self.rd16(b)?;
                let res = val.wrapping_sub(imm);
                self.cycle_rm2(6, t);
                self.psw_sub16(val, imm, res);
            }
            0x8c | 0x8d | 0x8f => {
                // divu
                let mut div = match opcode {
                    0x8c => {
                        let a = self.fetch()? as u16;
                        let v = self.rd16(a)?;
                        self.cycle(24);
                        v
                    }
                    0x8d => {
                        let v = self.fetch16()?;
                        self.cycle(25);
                        v
                    }
                    _ => {
                        let p = self.indexed_addr()?;
                        let v = self.rd16(p)?;
                        self.cycle_rm3(26, p);
                        v
                    }
                };
                let t = self.fetch()? as u16;
                let val = self.rd32(t)?;
                if div == 0 {
                    div = 1;
                }
                self.wr16(t, (val / div as u32) as u16)?;
                self.wr16(t.wrapping_add(2), (val % div as u32) as u16)?;
            }
            0x98 | 0x99 => {
                // cmpb b, x
                let imm = if opcode == 0x98 {
                    let a = self.fetch()? as u16;
                    let v = self.rd8(a)?;
                    self.cycle(4);
                    v
                } else {
                    let v = self.fetch()?;
                    self.cycle(5);
                    v
                };
                let b = self.fetch()? as u16;
                let val = self.rd8(b)?;
                self.psw_sub8(val, imm, val.wrapping_sub(imm));
            }
            0x9a => {
                // cmpb b, [a]+
                let (a, p) = self.indirect()?;
                let imm = self.rd8(p)?;
                let b = self.fetch()? as u16;
                let val = self.rd8(b)?;
                self.autoinc(a, p, 1)?;
                self.cycle_rm2(6, p);
                self.psw_sub8(val, imm, val.wrapping_sub(imm));
            }
            0x9b => {
                // cmpb b, indexed
                let t = self.indexed_addr()?;
                let imm = self.rd8(t)?;
                let b = self.fetch()? as u16;
                let val = self.rd8(b)?;
                self.cycle_rm2(6, t);
                self.psw_sub8(val, imm, val.wrapping_sub(imm));
            }
            0x9d | 0x9f => {
                // divub
                let mut div = if opcode == 0x9d {
                    let v = self.fetch()?;
                    self.cycle(16);
                    v
                } else {
                    let t = self.indexed_addr()?;
                    let v = self.rd8(t)?;
                    self.cycle_rm3(19, t);
                    v
                };
                let a = self.fetch()? as u16;
                let val = self.rd16(a)?;
                if div == 0 {
                    div = 1;
                }
                self.wr8(a, (val / div as u16) as u8)?;
                self.wr8(a + 1, (val % div as u16) as u8)?;
            }
            0xa0 => {
                // ld b, a
                let a = self.fetch()? as u16;
                let b = self.fetch()? as u16;
                let v = self.rd16(a)?;
                self.wr16(b, v)?;
                self.cycle(4);
            }
            0xa1 => {
                // ld b, #imm16
                let v = self.fetch16()?;
                let b = self.fetch()? as u16;
                self.wr16(b, v)?;
                self.cycle(5);
            }
            0xa2 => {
                // ld b, [a]+
                let (a, p) = self.indirect()?;
                let v = self.rd16(p)?;
                let b = self.fetch()? as u16;
                self.wr16(b, v)?;
                self.autoinc(a, p, 2)?;
                self.cycle_rm3(5, p);
            }
            0xa3 => {
                // ld b, indexed
                let p = self.indexed_addr()?;
                let b = self.fetch()? as u16;
                let v = self.rd16(p)?;
                self.wr16(b, v)?;
                self.cycle_rm3(6, p);
            }
            0xa4 | 0xa5 | 0xa7 => {
                // addc
                let val = match opcode {
                    0xa4 => {
                        let a = self.fetch()? as u16;
                        let v = self.rd16(a)?;
                        self.cycle(4);
                        v
                    }
                    0xa5 => {
                        let v = self.fetch16()?;
                        self.cycle(5);
                        v
                    }
                    _ => {
                        let p = self.indexed_addr()?;
                        let v = self.rd16(p)?;
                        self.cycle_rm2(6, p);
                        v
                    }
                };
                let a = self.fetch()? as u16;
                let mut res = self.rd16(a)?.wrapping_add(val);
                if psw_c(self) {
                    res = res.wrapping_add(1);
                }
                self.wr16(a, res)?;
                self.psw &= !(PSW_N | PSW_V);
                if res < val || (res == val && psw_c(self)) {
                    self.psw |= PSW_C;
                } else {
                    self.psw &= !PSW_C;
                }
                if res != 0 {
                    self.psw &= !PSW_Z;
                }
                if res >= 0x8000 {
                    self.psw |= PSW_N;
                }
            }
            0xa8..=0xaa => {
                // subc
                let imm = match opcode {
                    0xa8 => {
                        let a = self.fetch()? as u16;
                        let v = self.rd16(a)?;
                        self.cycle(4);
                        v
                    }
                    0xa9 => {
                        let v = self.fetch16()?;
                        self.cycle(5);
                        v
                    }
                    _ => {
                        let (a, p) = self.indirect()?;
                        let v = self.rd16(p)?;
                        self.autoinc(a, p, 2)?;
                        self.cycle_rm2(6, p);
                        v
                    }
                };
                let a = self.fetch()? as u16;
                let val = self.rd16(a)?;
                let mut res = val.wrapping_sub(imm);
                if !psw_c(self) {
                    res = res.wrapping_sub(1);
                }
                self.wr16(a, res)?;
                if val > imm || (val == imm && psw_c(self)) {
                    self.psw |= PSW_C;
                } else {
                    self.psw &= !PSW_C;
                }
                self.psw &= !(PSW_N | PSW_V);
                if res != 0 {
                    self.psw &= !PSW_Z;
                }
                if res >= 0x8000 {
                    self.psw |= PSW_N;
                }
            }
            0xac..=0xaf => {
                // ldbze
                let v = match opcode {
                    0xac => {
                        let a = self.fetch()? as u16;
                        let v = self.rd8(a)?;
                        self.cycle(4);
                        v
                    }
                    0xad => {
                        let v = self.fetch()?;
                        self.cycle(4);
                        v
                    }
                    0xae => {
                        let (a, p) = self.indirect()?;
                        let v = self.rd8(p)?;
                        self.autoinc(a, p, 1)?;
                        self.cycle_rm3(5, p);
                        v
                    }
                    _ => {
                        let p = self.indexed_addr()?;
                        let v = self.rd8(p)?;
                        self.cycle_rm3(6, p);
                        v
                    }
                };
                let b = self.fetch()? as u16;
                self.wr16(b, v as u16)?;
            }
            0xb0 => {
                // ldb b, a
                let a = self.fetch()? as u16;
                let b = self.fetch()? as u16;
                let v = self.rd8(a)?;
                self.wr8(b, v)?;
                self.cycle(4);
            }
            0xb1 => {
                // ldb b, #imm8
                let v = self.fetch()?;
                let b = self.fetch()? as u16;
                self.wr8(b, v)?;
                self.cycle(5);
            }
            0xb2 => {
                // ldb b, [a]+
                let (a, p) = self.indirect()?;
                let b = self.fetch()? as u16;
                let v = self.rd8(p)?;
                self.wr8(b, v)?;
                self.autoinc(a, p, 1)?;
                self.cycle_rm3(5, p);
            }
            0xb3 => {
                // ldb b, indexed
                let t = self.indexed_addr()?;
                let v = self.rd8(t)?;
                let b = self.fetch()? as u16;
                self.wr8(b, v)?;
                self.cycle_rm3(6, t);
            }
            0xb4 | 0xb5 => {
                // addcb
                let imm = if opcode == 0xb4 {
                    let a = self.fetch()? as u16;
                    let v = self.rd8(a)?;
                    self.cycle(4);
                    v
                } else {
                    let v = self.fetch()?;
                    self.cycle(5);
                    v
                };
                let a = self.fetch()? as u16;
                let val = self.rd8(a)?;
                let mut res = val.wrapping_add(imm);
                if psw_c(self) {
                    res = res.wrapping_add(1);
                }
                self.wr8(a, res)?;
                self.psw &= !(PSW_N | PSW_V);
                if res < val || (res == val && psw_c(self)) {
                    self.psw |= PSW_C;
                } else {
                    self.psw &= !PSW_C;
                }
                if res != 0 {
                    self.psw &= !PSW_Z;
                }
                if res >= 0x80 {
                    self.psw |= PSW_N;
                }
            }
            0xb8..=0xba => {
                // subcb
                let imm = match opcode {
                    0xb8 => {
                        let a = self.fetch()? as u16;
                        let v = self.rd8(a)?;
                        self.cycle(4);
                        v
                    }
                    0xb9 => {
                        let v = self.fetch()?;
                        self.cycle(5);
                        v
                    }
                    _ => {
                        let (a, p) = self.indirect()?;
                        let v = self.rd8(p)?;
                        self.autoinc(a, p, 1)?;
                        self.cycle_rm2(6, p);
                        v
                    }
                };
                let a = self.fetch()? as u16;
                let val = self.rd8(a)?;
                let mut res = val.wrapping_sub(imm);
                if !psw_c(self) {
                    res = res.wrapping_sub(1);
                }
                self.wr8(a, res)?;
                if val > imm || (val == imm && psw_c(self)) {
                    self.psw |= PSW_C;
                } else {
                    self.psw &= !PSW_C;
                }
                self.psw &= !(PSW_N | PSW_V);
                if res != 0 {
                    self.psw &= !PSW_Z;
                }
                if res >= 0x80 {
                    self.psw |= PSW_N;
                }
            }
            0xbc | 0xbd => {
                // ldbse
                let v = if opcode == 0xbc {
                    let a = self.fetch()? as u16;
                    self.rd8(a)?
                } else {
                    self.fetch()?
                };
                let b = self.fetch()? as u16;
                self.wr16(b, v as i8 as i16 as u16)?;
                self.cycle(4);
            }
            0xc0 => {
                // st b, a
                let a = self.fetch()? as u16;
                let b = self.fetch()? as u16;
                let v = self.rd16(b)?;
                self.wr16(a, v)?;
                self.cycle(4);
            }
            0xc2 => {
                // st b, [a]+
                let a = self.fetch()?;
                let b = self.fetch()? as u16;
                let v = self.rd16(b)?;
                let p = self.rd16((a & 0xfe) as u16)?;
                self.wr16(p, v)?;
                self.autoinc(a, p, 2)?;
                self.cycle_rm3(5, p);
            }
            0xc3 => {
                // st b, indexed
                let t = self.indexed_addr()?;
                let b = self.fetch()? as u16;
                let v = self.rd16(b)?;
                self.wr16(t, v)?;
                self.cycle_rm3(6, t);
            }
            0xc4 => {
                // stb b, a
                let a = self.fetch()? as u16;
                let b = self.fetch()? as u16;
                let v = self.rd8(b)?;
                self.wr8(a, v)?;
                self.cycle(4);
            }
            0xc5 => {
                // cmpl: register 0 reads as zero
                let a = self.fetch()? as u16;
                let imm = if a == 0 { 0 } else { self.rd32(a)? };
                let b = self.fetch()? as u16;
                let val = if b == 0 { 0 } else { self.rd32(b)? };
                let res = val.wrapping_sub(imm);
                self.cycle(7);
                self.psw &= !(PSW_Z | PSW_N | PSW_C | PSW_V | PSW_VT);
                if val >= imm {
                    self.psw |= PSW_C;
                }
                if res == 0 {
                    self.psw |= PSW_Z;
                }
                if res >= 0x8000_0000 {
                    self.psw |= PSW_N;
                }
            }
            0xc6 => {
                // stb b, [a]+
                let a = self.fetch()?;
                let b = self.fetch()? as u16;
                let p = self.rd16((a & 0xfe) as u16)?;
                let v = self.rd8(b)?;
                self.wr8(p, v)?;
                self.autoinc(a, p, 1)?;
                self.cycle_rm3(5, p);
            }
            0xc7 => {
                // stb b, indexed
                let t = self.indexed_addr()?;
                let b = self.fetch()? as u16;
                let v = self.rd8(b)?;
                self.wr8(t, v)?;
                self.cycle_rm3(6, t);
            }
            0xc8 => {
                // push a
                let a = self.fetch()? as u16;
                let v = self.rd16(a)?;
                self.push16(v)?;
                self.cycle(8);
            }
            0xc9 => {
                // push #imm16
                let v = self.fetch16()?;
                self.push16(v)?;
                self.cycle(9);
            }
            0xca => {
                // push [a]+
                let (a, p) = self.indirect()?;
                let v = self.rd16(p)?;
                self.push16(v)?;
                self.autoinc(a, p, 2)?;
                self.cycle_rm3(11, p);
            }
            0xcb => {
                // push indexed
                let t = self.indexed_addr()?;
                let v = self.rd16(t)?;
                self.push16(v)?;
                self.cycle_rm3(12, t);
            }
            0xcc => {
                // pop a
                let a = self.fetch()? as u16;
                let v = self.pop16()?;
                self.wr16(a, v)?;
                self.cycle(11);
            }
            0xcd => {
                // bmovi
                let c = self.fetch()? as u16;
                let mut count = self.rd16(c)?;
                let ptrs = self.fetch()? as u16;
                let mut src = self.rd16(ptrs)?;
                let mut dest = self.rd16(ptrs + 2)?;
                let mut inc = 8;
                if src > 0x200 {
                    inc += 3;
                }
                if dest > 0x200 {
                    inc += 3;
                }
                while count != 0 {
                    let v = self.rd16(src)?;
                    self.wr16(dest, v)?;
                    self.cycle(inc);
                    src = src.wrapping_add(2);
                    dest = dest.wrapping_add(2);
                    count -= 1;
                }
                self.wr16(ptrs, src)?;
                self.wr16(ptrs + 2, dest)?;
                self.cycle(7);
            }
            0xd0 => self.cond_jump(self.psw & PSW_ST == 0)?, // jnst
            0xd1 => self.cond_jump(self.psw & PSW_Z != 0 || self.psw & PSW_C == 0)?, // jnh
            0xd2 => self.cond_jump(self.psw & PSW_N == 0 && self.psw & PSW_Z == 0)?, // jgt
            0xd3 => self.cond_jump(self.psw & PSW_C == 0)?,  // jnc
            0xd6 => self.cond_jump(self.psw & PSW_N == 0)?,  // jge
            0xd7 => self.cond_jump(self.psw & PSW_Z == 0)?,  // jne
            0xd9 => self.cond_jump(self.psw & PSW_Z == 0 && self.psw & PSW_C != 0)?, // jh
            0xda => self.cond_jump(self.psw & PSW_N != 0 || self.psw & PSW_Z != 0)?, // jle
            0xdb => self.cond_jump(self.psw & PSW_C != 0)?,  // jc
            0xde => self.cond_jump(self.psw & PSW_N != 0)?,  // jlt
            0xdf => self.cond_jump(self.psw & PSW_Z != 0)?,  // je
            0xe0 => {
                // djnz
                let a = self.fetch()? as u16;
                let v = self.rd8(a)?.wrapping_sub(1);
                self.wr8(a, v)?;
                let rel = self.fetch()? as i8;
                if v != 0 {
                    self.pc = self.pc.wrapping_add(rel as u16);
                    self.cycle(4);
                }
                self.cycle(5);
            }
            0xe1 => {
                // djnzw
                let a = self.fetch()? as u16;
                let v = self.rd16(a)?.wrapping_sub(1);
                self.wr16(a, v)?;
                let rel = self.fetch()? as i8;
                if v != 0 {
                    self.pc = self.pc.wrapping_add(rel as u16);
                    self.cycle(4);
                }
                self.cycle(6);
            }
            0xe3 => {
                // br [a]
                let a = self.fetch()? as u16;
                self.pc = self.rd16(a)?;
                self.cycle(7);
            }
            0xe7 => {
                // ljmp
                let rel = self.fetch16()?;
                self.pc = self.pc.wrapping_add(rel);
                self.cycle(7);
            }
            0xec => {
                self.psw &= !PSW_PTSE;
                self.cycle(2);
            }
            0xed => {
                self.psw |= PSW_PTSE;
                self.cycle(2);
            }
            0xef => {
                // lcall
                let rel = self.fetch16()?;
                let target = self.pc.wrapping_add(rel);
                self.push16(self.pc)?;
                self.pc = target;
                self.cycle(13);
            }
            0xf0 => {
                // ret
                self.pc = self.pop16()?;
                self.cycle(14);
            }
            0xf2 => {
                // pushf
                self.push16(((self.psw as u16) << 8) | self.int_mask as u16)?;
                self.psw = 0;
                self.int_mask = 0;
                self.cycle(8);
            }
            0xf4 => {
                // pusha
                self.push16(((self.psw as u16) << 8) | self.int_mask as u16)?;
                self.push16(((self.int_mask1 as u16) << 8) | self.wsr as u16)?;
                self.psw = 0;
                self.int_mask = 0;
                self.int_mask1 = 0;
                self.cycle(18);
            }
            0xf5 => {
                // popa
                let v = self.pop16()?;
                self.int_mask1 = (v >> 8) as u8;
                self.wsr = v as u8;
                let v = self.pop16()?;
                self.psw = (v >> 8) as u8;
                self.int_mask = v as u8;
                self.cycle(18);
            }
            0xf8 => {
                self.psw &= !PSW_C;
                self.cycle(2);
            }
            0xf9 => {
                self.psw |= PSW_C;
                self.cycle(2);
            }
            0xfa => {
                self.psw &= !PSW_INTE;
                self.cycle(2);
            }
            0xfb => {
                self.psw |= PSW_INTE;
                self.cycle(2);
            }
            0xfc => {
                self.psw &= !PSW_VT;
                self.cycle(2);
            }
            0xfe => {
                let e = self.fetch()?;
                self.stats.fe_opcodes[e as usize] += 1;
                match e {
                    0x8c | 0x8d | 0x8f => {
                        // div (signed)
                        let mut div = match e {
                            0x8c => {
                                let a = self.fetch()? as u16;
                                let v = self.rd16(a)? as i16;
                                self.cycle(26);
                                v
                            }
                            0x8d => {
                                let v = self.fetch16()? as i16;
                                self.cycle(27);
                                v
                            }
                            _ => {
                                let p = self.indexed_addr()?;
                                let v = self.rd16(p)? as i16;
                                self.cycle_rm3(29, p);
                                v
                            }
                        };
                        let a = self.fetch()? as u16;
                        let val = self.rd32(a)? as i32;
                        if div == 0 {
                            div = 1;
                        }
                        // arm64 SDIV semantics on overflow (INT_MIN / -1)
                        self.wr16(a, val.wrapping_div(div as i32) as u16)?;
                        self.wr16(a.wrapping_add(2), val.wrapping_rem(div as i32) as u16)?;
                        self.psw &= !PSW_V;
                    }
                    _ => return Err(self.illegal(opcode, Some(e), opc)),
                }
            }
            _ => return Err(self.illegal(opcode, None, opc)),
        }
        Ok(StepEvent::Executed)
    }
}

#[cfg(test)]
mod tests {
    use super::rel11;

    #[test]
    fn rel11_sign_extends() {
        assert_eq!(rel11(0x20, 0x00), 0);
        assert_eq!(rel11(0x27, 0xfe), (-2i16) as u16);
        assert_eq!(rel11(0x23, 0xff), 0x3ff);
        assert_eq!(rel11(0x24, 0x00), (-1024i16) as u16);
    }
}
