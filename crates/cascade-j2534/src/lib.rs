//! Minimal SAE J2534 (PassThru) access for KWP2000 on the K-line (ISO 14230).
//!
//! - [`PassThru`]: the few operations kdiag needs, as a trait, so the
//!   message-level link in `cascade-diag` can be tested against a mock on any
//!   platform.
//! - [`dll::DllPassThru`] (Windows only): loads a vendor J2534 04.04 DLL
//!   (for example the Chipsoft's) and calls it. Vendor DLLs are usually
//!   32-bit, so build for `i686-pc-windows-msvc` to load them.
//! - [`drivers`] (Windows only): installed drivers from the registry.
//!
//! In J2534 the adapter does the fast init (`FAST_INIT`), framing,
//! checksums and inter-byte timing; messages carry the ISO 14230 header and
//! data but no checksum.

#![deny(unsafe_code)]

use std::fmt;
use std::time::Duration;

#[cfg(windows)]
pub mod dll;

/// Protocol IDs (J2534-1).
pub mod protocol {
    pub const ISO9141: u32 = 0x03;
    pub const ISO14230: u32 = 0x04;
}

/// Configuration parameter IDs (J2534-1, SET_CONFIG). Units as noted.
pub mod config {
    pub const DATA_RATE: u32 = 0x01;
    pub const LOOPBACK: u32 = 0x03;
    /// Max gap between ECU response bytes, 0.5 ms units.
    pub const P1_MAX: u32 = 0x07;
    /// Min time between ECU response and next request, 0.5 ms units.
    pub const P3_MIN: u32 = 0x0A;
    /// Min gap between tester request bytes, 0.5 ms units.
    pub const P4_MIN: u32 = 0x0C;
    /// Bus idle before a fast init, ms.
    pub const TIDLE: u32 = 0x13;
    /// Fast init low time, ms.
    pub const TINIL: u32 = 0x14;
    /// Fast init total wake-up time (low + high), ms.
    pub const TWUP: u32 = 0x15;
    pub const PARITY: u32 = 0x16;
    pub const DATA_BITS: u32 = 0x20;
}

/// A J2534 call failed: the status code and the driver's description.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Error {
    pub code: u32,
    pub message: String,
}

impl Error {
    pub fn new(code: u32, message: impl Into<String>) -> Self {
        Self {
            code,
            message: message.into(),
        }
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "J2534 error {:#04x}: {}", self.code, self.message)
    }
}

impl std::error::Error for Error {}

/// J2534 status codes used here.
pub mod status {
    pub const NOERROR: u32 = 0x00;
    pub const ERR_TIMEOUT: u32 = 0x09;
    pub const ERR_BUFFER_EMPTY: u32 = 0x10;
}

/// What kdiag needs from a J2534 device, on one ISO 14230 channel.
/// Messages are ISO 14230 frames **without** the checksum byte.
pub trait PassThru {
    /// Open an ISO 14230 channel at `baud` and apply `config` (id, value).
    fn connect_iso14230(&mut self, baud: u32, config: &[(u32, u32)]) -> Result<(), Error>;

    /// FAST_INIT: wake-up pattern, send `request` (StartCommunication),
    /// return the ECU's response.
    fn fast_init(&mut self, request: &[u8]) -> Result<Vec<u8>, Error>;

    fn write(&mut self, msg: &[u8], timeout: Duration) -> Result<(), Error>;

    /// The next received ECU message, or `None` after `timeout`.
    /// Transmit echoes and start-of-message indications are skipped.
    fn read(&mut self, timeout: Duration) -> Result<Option<Vec<u8>>, Error>;

    fn clear_rx(&mut self) -> Result<(), Error>;

    /// Battery voltage at OBD pin 16, mV.
    fn battery_mv(&mut self) -> Result<u32, Error>;

    /// Time since the device was opened (virtual for mocks).
    fn elapsed(&self) -> Duration;

    /// Wait (virtual for mocks).
    fn idle(&mut self, d: Duration);
}

/// An installed J2534 driver, from the Windows registry.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Driver {
    pub name: String,
    pub library: String,
}

/// Installed J2534 04.04 drivers (HKLM `PassThruSupport.04.04`, both the
/// 64-bit and the WOW6432Node views). Uses `reg query`, no extra crates.
#[cfg(windows)]
pub fn drivers() -> std::io::Result<Vec<Driver>> {
    let mut out = Vec::new();
    for key in [
        r"HKLM\SOFTWARE\PassThruSupport.04.04",
        r"HKLM\SOFTWARE\WOW6432Node\PassThruSupport.04.04",
    ] {
        let o = std::process::Command::new("reg")
            .args(["query", key, "/s", "/v", "FunctionLibrary"])
            .output()?;
        if o.status.success() {
            out.extend(parse_reg_query(&String::from_utf8_lossy(&o.stdout)));
        }
    }
    out.dedup();
    Ok(out)
}

/// Parse `reg query … /s /v FunctionLibrary` output into drivers.
pub fn parse_reg_query(text: &str) -> Vec<Driver> {
    let mut out = Vec::new();
    let mut name = String::new();
    for line in text.lines() {
        let t = line.trim();
        if t.starts_with("HKEY_") {
            name = t.rsplit('\\').next().unwrap_or(t).to_string();
        } else if let Some(rest) = t.strip_prefix("FunctionLibrary")
            && let Some(i) = rest.find("REG_SZ")
        {
            out.push(Driver {
                name: name.clone(),
                library: rest[i + "REG_SZ".len()..].trim().to_string(),
            });
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reg_query_output() {
        let text = "\r\nHKEY_LOCAL_MACHINE\\SOFTWARE\\WOW6432Node\\PassThruSupport.04.04\\CHIPSOFT J2534 Pro\r\n    FunctionLibrary    REG_SZ    C:\\Program Files (x86)\\Chipsoft\\J2534.dll\r\n\r\nEnd of search: 1 match(es) found.\r\n";
        assert_eq!(
            parse_reg_query(text),
            vec![Driver {
                name: "CHIPSOFT J2534 Pro".into(),
                library: r"C:\Program Files (x86)\Chipsoft\J2534.dll".into(),
            }]
        );
    }
}
