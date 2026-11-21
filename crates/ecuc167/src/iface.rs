//! The emulated SIMK41 as a `cascade_iface::Interface`: a K-line with the
//! ECU's own code on the other end.
//!
//! Time: callers count in Hi-Scan cycles (10,000 per ms); the ECU runs at
//! `fcpu` (16 MHz assumed, see `docs/simk41-notes.md`). The ECU is powered
//! on lazily: at the tester's first wake-up it first runs `boot_ms` of its
//! own time (past the immobiliser phase, which holds the shared UART for
//! about 2.6 s when no immobiliser answers), then follows the caller's clock.
//!
//! The tester's bytes go onto the modelled line as UART frames; only bytes
//! the ECU transmits come back (no echo of the tester's own bytes, like
//! `KwpEcu`). A fast init (bitwise "slow init" low/high from the caller) is a
//! low interval on the line; the ECU is held while the line is low, because
//! the interval's end is only known when the caller raises the line.

use std::collections::VecDeque;

use cascade_iface::{Cycles, Interface, RxBuf};

use crate::cpu::{Cpu, Event};

/// Caller cycles per millisecond (the Hi-Scan core's bookkeeping).
const HOST_PER_MS: u64 = 10_000;

pub struct Simk41 {
    cpu: Cpu,
    fcpu: u64,
    /// ECU time to run before following the caller (power-on to K-line).
    pub boot_ms: u64,
    /// (caller time, ECU cycles) at the moment the clocks were tied.
    sync: Option<(Cycles, u64)>,
    baud: u32,
    /// ECU time at which the tester pulled the line low (fast init).
    low_since: Option<u64>,
    /// ECU transmissions not yet delivered: (ECU cycles at end of frame, byte).
    out: VecDeque<(u64, u8)>,
    /// Why the emulation stopped, if it did.
    pub stopped: Option<String>,
}

impl Simk41 {
    pub fn new(rom: Vec<u8>, fcpu: u64) -> Self {
        Self {
            cpu: Cpu::new(rom, fcpu),
            fcpu,
            boot_ms: 3000,
            sync: None,
            baud: 10_400,
            low_since: None,
            out: VecDeque::new(),
            stopped: None,
        }
    }

    /// Set a 10-bit ADC input (sensor voltage × 1023 / 5 V). Mid-scale (0x200)
    /// by default. Takes effect at the ECU's next conversion.
    pub fn set_adc(&mut self, ch: usize, value: u16) {
        self.cpu.adc[ch & 15] = value & 0x3ff;
    }

    /// Read ECU memory (physical address), e.g. a RAM variable.
    pub fn peek8(&self, addr: u32) -> u8 {
        self.cpu.peek8(addr)
    }

    pub fn ecu_ms(&self) -> f64 {
        self.cpu.cycles as f64 * 1000.0 / self.fcpu as f64
    }

    fn ecu_at(&self, host: Cycles) -> u64 {
        match self.sync {
            Some((h0, e0)) => {
                e0 + (host.saturating_sub(h0) as u128 * self.fcpu as u128
                    / (HOST_PER_MS as u128 * 1000)) as u64
            }
            None => 0,
        }
    }

    fn host_at(&self, ecu: u64) -> Cycles {
        match self.sync {
            Some((h0, e0)) => {
                h0 + (ecu.saturating_sub(e0) as u128 * (HOST_PER_MS as u128 * 1000)
                    / self.fcpu as u128) as u64
            }
            None => 0,
        }
    }

    /// Power the ECU on (once) and tie its clock to the caller's `now`.
    fn ensure_started(&mut self, now: Cycles) {
        if self.sync.is_some() {
            return;
        }
        let boot = self.boot_ms * self.fcpu / 1000;
        self.run_ecu_to(boot);
        // whatever it sent while booting (immobiliser frames) is not K-line traffic
        self.out.clear();
        self.sync = Some((now, self.cpu.cycles));
    }

    fn run_ecu_to(&mut self, target: u64) {
        while self.stopped.is_none() && self.cpu.cycles < target {
            if let Err(e) = self.cpu.step() {
                self.stopped = Some(e.to_string());
            }
            for ev in self.cpu.events.drain(..) {
                if let Event::Tx { at, byte } = ev {
                    self.out.push_back((at, byte));
                }
            }
        }
        self.cpu
            .line
            .forget_before(self.cpu.cycles.saturating_sub(1_000_000));
    }

    /// Run the ECU up to caller time `now`, but not into an open low phase.
    fn advance(&mut self, now: Cycles) {
        let mut t = self.ecu_at(now);
        if let Some(s) = self.low_since {
            t = t.min(s);
        }
        self.run_ecu_to(t);
    }

    fn bit(&self) -> u64 {
        self.fcpu / self.baud as u64
    }
}

/// The Hi-Scan's UART divisor as a baud rate (as in `kline_iface`).
fn divisor_to_baud(d: u32) -> u32 {
    match d {
        0x77 | 0x76 | 0 => 10_400,
        d => (1_250_000 + d / 2) / d,
    }
}

impl Interface for Simk41 {
    fn set_baud_divisor(&mut self, _now: Cycles, divisor: u32, _rx: &mut dyn RxBuf) {
        self.baud = divisor_to_baud(divisor);
    }

    fn send_byte(&mut self, now: Cycles, byte: u8, _rx: &mut dyn RxBuf) {
        self.ensure_started(now);
        self.advance(now);
        let t = self.ecu_at(now).max(self.cpu.cycles);
        let bit = self.bit();
        self.cpu.line.frame(t, byte, bit);
    }

    fn poll_input(&mut self, now: Cycles, rx: &mut dyn RxBuf) {
        if self.sync.is_none() {
            return;
        }
        self.advance(now);
        while let Some(&(at, b)) = self.out.front() {
            if self.host_at(at) > now {
                break;
            }
            self.out.pop_front();
            rx.push(b);
        }
    }

    fn send_slow_init_bitwise(&mut self, now: Cycles, bit: bool, _rx: &mut dyn RxBuf) -> bool {
        self.ensure_started(now);
        if !bit {
            self.advance(now);
            self.low_since
                .get_or_insert(self.ecu_at(now).max(self.cpu.cycles));
        } else if let Some(s) = self.low_since.take() {
            let e = self.ecu_at(now);
            self.cpu.line.pull_low(s, e);
        }
        true
    }
}
