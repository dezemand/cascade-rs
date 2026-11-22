//! Native KWP2000 diagnostics for the 2002 Hyundai Coupe (Tiburon GK),
//! built from protocol data learned by running the Hi-Scan firmware in
//! cascade-rs. No emulator or Hyundai software is needed at run time.

#![forbid(unsafe_code)]

pub mod can;
pub mod defs;
#[cfg(all(feature = "serial", unix))]
pub mod ecusim_line;
pub mod elm;
pub mod guard;
pub mod iso9141;
pub mod j2534;
#[cfg(all(feature = "serial", unix))]
pub mod kline_iface;
pub mod kwp;
pub mod line;
#[cfg(all(feature = "serial", unix))]
pub mod serial;
pub mod sim;
pub mod trace;
