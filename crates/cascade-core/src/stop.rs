//! Conditions under which emulation cannot continue.

use std::fmt;

/// Why `Cpu::step` stopped.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Stop {
    /// `-m` cap reached (`cycles > max_cycles`), same check as upstream.
    MaxCycles,
    /// Opcode upstream does not implement. `ext` is the second byte of FE xx.
    IllegalOpcode {
        opcode: u8,
        ext: Option<u8>,
        pc: u16,
    },
    /// A path where upstream calls `exit(1)` (unimplemented I/O, LCD command, ...).
    Unimplemented(String),
    /// A path where upstream has undefined behaviour (out-of-bounds pointer).
    /// The port refuses to guess what the C++ binary would have read.
    OutOfBounds(String),
}

impl fmt::Display for Stop {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Stop::MaxCycles => write!(f, "maximum cycles exceeded"),
            Stop::IllegalOpcode {
                opcode,
                ext: None,
                pc,
            } => {
                write!(f, "illegal opcode {opcode:02X} at {pc:04X}")
            }
            Stop::IllegalOpcode {
                opcode,
                ext: Some(e),
                pc,
            } => {
                write!(f, "illegal opcode {opcode:02X} {e:02X} at {pc:04X}")
            }
            Stop::Unimplemented(s) => write!(f, "unimplemented: {s}"),
            Stop::OutOfBounds(s) => write!(f, "out of bounds (UB upstream): {s}"),
        }
    }
}

impl std::error::Error for Stop {}

pub type R<T> = Result<T, Stop>;

pub(crate) fn unimp<T>(msg: impl Into<String>) -> R<T> {
    Err(Stop::Unimplemented(msg.into()))
}

pub(crate) fn oob<T>(msg: impl Into<String>) -> R<T> {
    Err(Stop::OutOfBounds(msg.into()))
}
