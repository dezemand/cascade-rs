//! The instruction trace format of the C++ oracle (`-d trace,-abridged`).
//!
//! Each executed instruction produces two lines, printed *before* it executes:
//!
//! ```text
//! PC 2080 (00002080) AX FFFF BX FFFF CX FFFF DX FFFF A0 FFFF A2 FFFF A6 FFFF E0 FFFF E2 FFFF E4 FFFF E8 FFFF PSW 00 SP 504D 25E FF
//!    DI
//! ```
//!
//! The register names are the oracle's: `AX`..`DX` are words at 0x50..0x56,
//! `A0`..`E8` are words at those RAM addresses, `SP` is the word at 0x18 and
//! `25E` is the byte at 0x25E (see `cpu_emu.cpp`). [`Record`]'s `Display`
//! reproduces the two lines byte for byte, so the Rust core can emit the same
//! format and be diffed against the oracle directly.

use std::fmt;

/// Names of the 16-bit values between the physical PC and `PSW`, in order.
pub const WORD_NAMES: [&str; 11] = [
    "AX", "BX", "CX", "DX", "A0", "A2", "A6", "E0", "E2", "E4", "E8",
];

/// RAM addresses of [`WORD_NAMES`], in the same order.
pub const WORD_ADDRS: [u16; 11] = [
    0x50, 0x52, 0x54, 0x56, 0xa0, 0xa2, 0xa6, 0xe0, 0xe2, 0xe4, 0xe8,
];

/// Machine state before one instruction, plus the oracle's disassembly of it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Record {
    /// Logical program counter.
    pub pc: u16,
    /// Physical address of `pc` under the current code mapping.
    pub phys: u32,
    /// Values of [`WORD_NAMES`].
    pub words: [u16; 11],
    pub psw: u8,
    pub sp: u16,
    /// Byte at 0x25E (beeper / reset control).
    pub r25e: u8,
    /// Disassembly text without the three-space indent. Upstream's
    /// disassembler has quirks; compare it for information, not correctness.
    pub disasm: String,
}

impl Record {
    /// Compare machine state only (everything except the disassembly text).
    pub fn same_state(&self, other: &Record) -> bool {
        self.pc == other.pc
            && self.phys == other.phys
            && self.words == other.words
            && self.psw == other.psw
            && self.sp == other.sp
            && self.r25e == other.r25e
    }
}

impl fmt::Display for Record {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "PC {:04X} ({:08X})", self.pc, self.phys)?;
        for (name, v) in WORD_NAMES.iter().zip(self.words) {
            write!(f, " {name} {v:04X}")?;
        }
        write!(
            f,
            " PSW {:02X} SP {:04X} 25E {:02X}\n   {}",
            self.psw, self.sp, self.r25e, self.disasm
        )
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ParseError {
    pub line: usize,
    pub msg: String,
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "line {}: {}", self.line, self.msg)
    }
}

impl std::error::Error for ParseError {}

/// Parse a `PC ...` state line (without the disassembly line).
pub fn parse_state_line(line: &str) -> Result<Record, String> {
    let toks: Vec<&str> = line.split_ascii_whitespace().collect();
    // PC v (phys) + 11 * (name v) + PSW v SP v 25E v
    const N: usize = 3 + 2 * 11 + 6;
    if toks.len() != N || toks[0] != "PC" {
        return Err(format!(
            "expected {N} tokens starting with PC, got {}",
            toks.len()
        ));
    }
    let hex16 = |s: &str| u16::from_str_radix(s, 16).map_err(|e| format!("{s:?}: {e}"));
    let hex8 = |s: &str| u8::from_str_radix(s, 16).map_err(|e| format!("{s:?}: {e}"));
    let pc = hex16(toks[1])?;
    let phys_tok = toks[2]
        .strip_prefix('(')
        .and_then(|s| s.strip_suffix(')'))
        .ok_or_else(|| format!("bad physical address {:?}", toks[2]))?;
    let phys = u32::from_str_radix(phys_tok, 16).map_err(|e| format!("{phys_tok:?}: {e}"))?;
    let mut words = [0u16; 11];
    for (i, name) in WORD_NAMES.iter().enumerate() {
        let k = 3 + 2 * i;
        if toks[k] != *name {
            return Err(format!("expected {name:?}, got {:?}", toks[k]));
        }
        words[i] = hex16(toks[k + 1])?;
    }
    let tail = &toks[3 + 22..];
    for (k, name) in [(0, "PSW"), (2, "SP"), (4, "25E")] {
        if tail[k] != name {
            return Err(format!("expected {name:?}, got {:?}", tail[k]));
        }
    }
    Ok(Record {
        pc,
        phys,
        words,
        psw: hex8(tail[1])?,
        sp: hex16(tail[3])?,
        r25e: hex8(tail[5])?,
        disasm: String::new(),
    })
}

/// Iterator over [`Record`]s in an oracle log.
///
/// Lines that are neither state lines nor disassembly lines (warnings,
/// wall-clock messages, UI-thread output) are skipped, because they are not
/// deterministic.
pub struct Reader<I> {
    lines: I,
    lineno: usize,
}

impl<I: Iterator<Item = String>> Reader<I> {
    pub fn new(lines: I) -> Self {
        Self { lines, lineno: 0 }
    }
}

impl<I: Iterator<Item = String>> Iterator for Reader<I> {
    type Item = Result<Record, ParseError>;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            let line = self.lines.next()?;
            self.lineno += 1;
            if !line.starts_with("PC ") {
                continue;
            }
            let at = self.lineno;
            let mut rec = match parse_state_line(&line) {
                Ok(r) => r,
                Err(msg) => return Some(Err(ParseError { line: at, msg })),
            };
            // The disassembly line normally follows directly. A missing one
            // (log truncated mid-record) yields a record with empty disasm.
            if let Some(next) = self.lines.next() {
                self.lineno += 1;
                match next.strip_prefix("   ") {
                    Some(d) => rec.disasm = d.to_owned(),
                    None => {
                        return Some(Err(ParseError {
                            line: self.lineno,
                            msg: format!("expected disassembly line, got {next:?}"),
                        }));
                    }
                }
            }
            return Some(Ok(rec));
        }
    }
}

/// Where two traces first disagree.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Divergence {
    /// Record `index` (0-based) differs in machine state.
    State {
        index: u64,
        left: Record,
        right: Record,
    },
    /// One trace ended after `len` records while the other continued.
    Length { len: u64, left_longer: bool },
}

/// Compare two record streams by machine state until the first divergence.
///
/// Returns `Ok(None)` and the number of records compared if they agree.
pub fn first_divergence<L, R>(left: L, right: R) -> Result<(u64, Option<Divergence>), ParseError>
where
    L: IntoIterator<Item = Result<Record, ParseError>>,
    R: IntoIterator<Item = Result<Record, ParseError>>,
{
    let (mut l, mut r) = (left.into_iter(), right.into_iter());
    let mut index = 0u64;
    loop {
        match (l.next().transpose()?, r.next().transpose()?) {
            (None, None) => return Ok((index, None)),
            (Some(_), None) => {
                return Ok((
                    index,
                    Some(Divergence::Length {
                        len: index,
                        left_longer: true,
                    }),
                ));
            }
            (None, Some(_)) => {
                return Ok((
                    index,
                    Some(Divergence::Length {
                        len: index,
                        left_longer: false,
                    }),
                ));
            }
            (Some(a), Some(b)) => {
                if !a.same_state(&b) {
                    return Ok((
                        index,
                        Some(Divergence::State {
                            index,
                            left: a,
                            right: b,
                        }),
                    ));
                }
            }
        }
        index += 1;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE: &str = "\
PC 2080 (00002080) AX FFFF BX FFFF CX FFFF DX FFFF A0 FFFF A2 FFFF A6 FFFF E0 FFFF E2 FFFF E4 FFFF E8 FFFF PSW 00 SP 504D 25E FF
   DI
running Cpu::emulate()
PC 2081 (00002081) AX FFFF BX FFFF CX FFFF DX FFFF A0 FFFF A2 FFFF A6 FFFF E0 FFFF E2 FFFF E4 FFFF E8 FFFF PSW 00 SP 504D 25E FF
   LDB 14h, 00h
PC EB4F (0012AB4F) AX 000C BX 0001 CX EB24 DX 0047 A0 FF4D A2 FF06 A6 FFFF E0 0100 E2 0080 E4 000E E8 001A PSW 50 SP 19B4 25E 6D
   JGT EB55h (04h)
";

    fn records(s: &str) -> Vec<Record> {
        Reader::new(s.lines().map(str::to_owned))
            .collect::<Result<_, _>>()
            .unwrap()
    }

    #[test]
    fn parses_and_skips_noise() {
        let r = records(SAMPLE);
        assert_eq!(r.len(), 3);
        assert_eq!(r[2].pc, 0xeb4f);
        assert_eq!(r[2].phys, 0x12ab4f);
        assert_eq!(r[2].words[2], 0xeb24);
        assert_eq!(r[2].psw, 0x50);
        assert_eq!(r[2].sp, 0x19b4);
        assert_eq!(r[2].r25e, 0x6d);
        assert_eq!(r[2].disasm, "JGT EB55h (04h)");
    }

    #[test]
    fn display_roundtrips_exactly() {
        let lines: Vec<&str> = SAMPLE
            .lines()
            .filter(|l| !l.starts_with("running"))
            .collect();
        let out: Vec<String> = records(SAMPLE).iter().map(|r| r.to_string()).collect();
        assert_eq!(out.join("\n"), lines.join("\n"));
    }

    #[test]
    fn finds_first_divergence() {
        let a = records(SAMPLE);
        let mut b = a.clone();
        b[1].psw = 0x02;
        b[2].disasm = "different text is not a divergence".into();
        let (_, d) =
            first_divergence(a.iter().cloned().map(Ok), b.iter().cloned().map(Ok)).unwrap();
        assert!(matches!(d, Some(Divergence::State { index: 1, .. })));

        let (n, d) =
            first_divergence(a.iter().cloned().map(Ok), a[..2].iter().cloned().map(Ok)).unwrap();
        assert_eq!(n, 2);
        assert_eq!(
            d,
            Some(Divergence::Length {
                len: 2,
                left_longer: true
            })
        );
    }

    #[test]
    fn rejects_malformed_state_line() {
        assert!(parse_state_line("PC 2080 (00002080) AX FFFF").is_err());
        assert!(parse_state_line(&SAMPLE.lines().next().unwrap().replace("PSW", "PSX")).is_err());
    }
}
