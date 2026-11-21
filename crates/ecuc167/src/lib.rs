//! ecuc167: an Infineon C167-class (C16x) disassembler and emulator, built
//! to run a GK SIMK41 engine ECU dump offline. The dump is proprietary and
//! not part of this repository.
//!
//! [`iface::Simk41`] puts the emulated ECU behind `cascade_iface::Interface`,
//! so the Hi-Scan emulator and kdiag's `SimLine` can talk to the ECU's own
//! code instead of the hand-written `KwpEcu` model.
//!
//! Offline analysis only. Nothing here talks to a car. The ROM is never part
//! of this repository.

#![forbid(unsafe_code)]

pub mod cpu;
pub mod iface;
pub mod isa;
pub mod line;
