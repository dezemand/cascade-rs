//! The physical K-line, as seen by the diagnostic client.

use std::io;
use std::time::Duration;

/// A K-line connection. Implementations: [`crate::sim::SimLine`] (virtual
/// time against an ECU model) and, with the `serial` feature,
/// [`crate::serial::SerialLine`] (a real adapter).
pub trait Line {
    /// Time since the line was opened (virtual for simulated lines).
    fn elapsed(&self) -> Duration;

    fn set_baud(&mut self, baud: u32) -> io::Result<()>;

    /// ISO 14230 fast-init wake-up pattern: K low for `low`, then idle
    /// (high) for `high`.
    fn wake_up(&mut self, low: Duration, high: Duration) -> io::Result<()>;

    /// ISO 9141 5-baud initialisation: send `addr` at 5 baud (start bit,
    /// eight data bits LSB first, stop bit; 2 s in all) by driving K
    /// directly. Lines that cannot do this return `Unsupported`.
    fn five_baud(&mut self, _addr: u8) -> io::Result<()> {
        Err(io::Error::new(
            io::ErrorKind::Unsupported,
            "this line cannot send a 5-baud address",
        ))
    }

    /// Send bytes. On a real K-line the adapter hears its own bytes; an
    /// implementation consumes that echo before returning.
    fn write(&mut self, data: &[u8]) -> io::Result<()>;

    /// Next received byte, or `None` after `timeout` without one.
    fn read_byte(&mut self, timeout: Duration) -> io::Result<Option<u8>>;

    /// Drop anything already received.
    fn flush_input(&mut self) -> io::Result<()>;

    /// Wait without traffic (virtual on simulated lines).
    fn idle(&mut self, d: Duration);
}

/// Time one byte takes on the wire at `baud` (start + 8 data + stop bits).
pub fn byte_time(baud: u32) -> Duration {
    Duration::from_nanos(10 * 1_000_000_000 / baud as u64)
}
