//! Emulation core for the Hi-Scan Pro: Intel 8XC196 CPU, I/O, HSIO, UART,
//! LCD, keypad and EEPROM. Deterministic and cycle-counted; no OS, threads
//! or wall-clock time. Port of CASCADE by Ulrich Hecht (CasPL 1.0).
//!
//! Status: only the oracle trace format exists so far. The CPU port follows.

#![forbid(unsafe_code)]

pub mod trace;
