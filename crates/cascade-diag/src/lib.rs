//! Native KWP2000 diagnostics for the 2002 Hyundai Coupe (Tiburon GK),
//! built from protocol data learned by running the Hi-Scan firmware in
//! cascade-rs. No emulator or Hyundai software is needed at run time.

#![forbid(unsafe_code)]

pub mod defs;
pub mod kwp;
pub mod line;
#[cfg(feature = "serial")]
pub mod serial;
pub mod sim;
pub mod trace;
