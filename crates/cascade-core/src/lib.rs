//! Emulation core for the Hi-Scan Pro: Intel 8XC196 CPU, I/O, HSIO, UART,
//! LCD, keypad and EEPROM. Deterministic and cycle-counted; no OS, threads
//! or wall-clock time. Port of CASCADE by Ulrich Hecht (CasPL 1.0).

#![forbid(unsafe_code)]

pub mod cpu;
pub mod disasm;
pub mod eeprom;
pub mod glibc_rand;
pub mod hsio;
pub mod keypad;
pub mod lcd;
pub mod ring;
pub mod serial;
pub mod stop;
pub mod trace;

pub use cpu::{Cpu, KeyEvent, Stats, StepEvent};
pub use keypad::Key;
pub use stop::Stop;
