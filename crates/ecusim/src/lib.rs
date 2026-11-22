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
