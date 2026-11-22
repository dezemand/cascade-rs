//! ecusim: a SIMK41 engine ECU dump running in real time as its own
//! process, with an open-loop engine model and a local socket.
//!
//! - [`server`] owns the emulated ECU (`ecuc167`), paces it to wall time,
//!   powers it with the ignition and drives its inputs from [`plant`].
//! - [`proto`] is the line protocol on the socket: K-line events (bytes,
//!   line low/high with the client's own timestamps) and controls.
//! - [`client`] connects to a server: [`client::SocketKLine`] is a
//!   `cascade_iface::Interface` for the Hi-Scan emulator; kdiag wraps
//!   [`client::Conn`] as its `Line`.
//!
//! The engine model is open loop **[assumed]**: engine speed, MAP and the
//! like follow the controls (ignition, starter, throttle), not the ECU's
//! injection and ignition outputs, which are not decoded yet. The sensor
//! curves (ADC counts per °C, V, degree, kPa) were measured by sweeping the
//! ROM's own inputs in the emulator. Unix only (Unix-domain socket).
//!
//! The ECU is powered while the battery is connected: the ignition only
//! starts the engine and lets it answer on the K-line, so its fault memory
//! survives key cycles [assumed: stands in for its non-volatile store].
//! Injected wiring faults ([`plant::Fault`]) make its own code set codes.
//!
//! Use a ROM without an immobiliser: with one, the ECU stops crank
//! processing when no immobiliser answers.

#![forbid(unsafe_code)]

pub mod client;
pub mod ecu_view;
pub mod plant;
pub mod probe;
pub mod proto;
pub mod server;
pub mod settings;
pub mod vehicle;

/// Default socket path.
pub fn default_socket() -> std::path::PathBuf {
    std::env::temp_dir().join("ecusim.sock")
}

/// Where an ecusim server listens, as people type it:
/// - starting with `/` (or `unix:`, `~/`, `$TMPDIR/`): a Unix socket;
/// - `host:port` (or `tcp:host:port`), host starting with a letter or
///   digit: TCP. Parsed here; the TCP transport itself is not built yet.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Endpoint {
    Unix(std::path::PathBuf),
    Tcp { host: String, port: u16 },
}

impl Endpoint {
    pub fn parse(text: &str) -> Result<Self, String> {
        let s = text.trim();
        if s.is_empty() {
            return Err("enter a socket path or host:port".into());
        }
        if let Some(rest) = s.strip_prefix("unix:") {
            return Self::parse(rest).and_then(|e| match e {
                Endpoint::Unix(p) => Ok(Endpoint::Unix(p)),
                _ => Err(format!("{rest:?} is not a socket path")),
            });
        }
        if s.starts_with('/') {
            return Ok(Endpoint::Unix(s.into()));
        }
        if let Some(rest) = s.strip_prefix("$TMPDIR/") {
            return Ok(Endpoint::Unix(std::env::temp_dir().join(rest)));
        }
        if let Some(rest) = s.strip_prefix("~/")
            && let Some(home) = std::env::var_os("HOME")
        {
            return Ok(Endpoint::Unix(std::path::PathBuf::from(home).join(rest)));
        }
        let hp = s.strip_prefix("tcp:").unwrap_or(s);
        let (host, port) = hp
            .rsplit_once(':')
            .ok_or_else(|| format!("{s:?}: expected /path/to/socket or host:port"))?;
        let host_ok = host
            .chars()
            .next()
            .is_some_and(|c| c.is_ascii_alphanumeric())
            && host
                .chars()
                .all(|c| c.is_ascii_alphanumeric() || c == '.' || c == '-');
        if !host_ok {
            return Err(format!("{host:?} is not a host name or address"));
        }
        let port: u16 = port
            .parse()
            .ok()
            .filter(|&p| p != 0)
            .ok_or_else(|| format!("{port:?} is not a port (1-65535)"))?;
        Ok(Endpoint::Tcp {
            host: host.into(),
            port,
        })
    }

    /// For people: `unix:…` (shortened) or `tcp:host:port`.
    pub fn label(&self) -> String {
        match self {
            Endpoint::Unix(p) => endpoint_label(p),
            Endpoint::Tcp { host, port } => format!("tcp:{host}:{port}"),
        }
    }
}

/// Where a server listens, for people: `unix:` and the socket path, the
/// temporary folder shortened to `$TMPDIR` and the home folder to `~`.
/// (Written as an endpoint so other transports can be shown the same way.)
pub fn endpoint_label(socket: &std::path::Path) -> String {
    let tmp = std::env::temp_dir();
    let short = if let Ok(rest) = socket.strip_prefix(&tmp) {
        format!("$TMPDIR/{}", rest.display())
    } else if let Some(home) = std::env::var_os("HOME")
        && let Ok(rest) = socket.strip_prefix(&home)
    {
        format!("~/{}", rest.display())
    } else {
        socket.display().to_string()
    };
    format!("unix:{short}")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn endpoints() {
        assert_eq!(
            Endpoint::parse(" /tmp/e.sock "),
            Ok(Endpoint::Unix("/tmp/e.sock".into()))
        );
        assert_eq!(
            Endpoint::parse("unix:/tmp/e.sock"),
            Ok(Endpoint::Unix("/tmp/e.sock".into()))
        );
        assert_eq!(
            Endpoint::parse("$TMPDIR/ecusim.sock"),
            Ok(Endpoint::Unix(default_socket()))
        );
        let tcp = |h: &str, p| {
            Ok(Endpoint::Tcp {
                host: h.into(),
                port: p,
            })
        };
        assert_eq!(Endpoint::parse("localhost:35000"), tcp("localhost", 35000));
        assert_eq!(Endpoint::parse("tcp:10.0.0.2:7"), tcp("10.0.0.2", 7));
        assert!(Endpoint::parse("localhost").is_err());
        assert!(Endpoint::parse("host:0").is_err());
        assert!(Endpoint::parse("-x:80").is_err());
        assert!(Endpoint::parse("").is_err());
        assert_eq!(
            Endpoint::parse("tcp:pi.local:35000").unwrap().label(),
            "tcp:pi.local:35000"
        );
    }
}
