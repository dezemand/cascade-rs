//! A K-line in virtual time, attached to an emulated ECU (`cascade-iface`
//! backend such as `KwpEcu`). Used for tests and `kdiag --sim`.

use std::collections::VecDeque;
use std::io;
use std::time::Duration;

use cascade_iface::{Cycles, Interface, RxBuf};

use crate::line::{Line, byte_time};

/// Emulated cycles per millisecond, as in `cascade_iface::kwp`.
const CYCLES_PER_MS: u64 = 10_000;

#[derive(Default)]
struct Rx(VecDeque<u8>);

impl RxBuf for Rx {
    fn push(&mut self, byte: u8) {
        self.0.push_back(byte);
    }
    fn flush(&mut self) {
        self.0.clear();
    }
}

pub struct SimLine<I: Interface> {
    pub ecu: I,
    now: Cycles,
    baud: u32,
    rx: Rx,
}

impl<I: Interface> SimLine<I> {
    pub fn new(ecu: I) -> Self {
        Self {
            ecu,
            now: 0,
            baud: 10_400,
            rx: Rx::default(),
        }
    }

    fn advance(&mut self, d: Duration) {
        self.now += (d.as_nanos() as u64 * CYCLES_PER_MS) / 1_000_000;
    }
}

impl<I: Interface> Line for SimLine<I> {
    fn elapsed(&self) -> Duration {
        Duration::from_nanos(self.now * 1_000_000 / CYCLES_PER_MS)
    }

    fn set_baud(&mut self, baud: u32) -> io::Result<()> {
        self.baud = baud;
        Ok(())
    }

    fn wake_up(&mut self, low: Duration, high: Duration) -> io::Result<()> {
        self.ecu
            .send_slow_init_bitwise(self.now, false, &mut self.rx);
        self.advance(low);
        self.ecu
            .send_slow_init_bitwise(self.now, true, &mut self.rx);
        self.advance(high);
        Ok(())
    }

    fn five_baud(&mut self, addr: u8) -> io::Result<()> {
        // 10 bits of 200 ms; the ECU model reacts once the address is complete
        self.advance(Duration::from_millis(2000));
        self.ecu.send_slow_init(self.now, addr, &mut self.rx);
        Ok(())
    }

    fn write(&mut self, data: &[u8]) -> io::Result<()> {
        for &b in data {
            self.advance(byte_time(self.baud));
            self.ecu.send_byte(self.now, b, &mut self.rx);
        }
        Ok(())
    }

    fn read_byte(&mut self, timeout: Duration) -> io::Result<Option<u8>> {
        let deadline = self.now + (timeout.as_nanos() as u64 * CYCLES_PER_MS) / 1_000_000;
        loop {
            self.ecu.poll_input(self.now, &mut self.rx);
            if let Some(b) = self.rx.0.pop_front() {
                self.advance(byte_time(self.baud));
                return Ok(Some(b));
            }
            if self.now >= deadline {
                return Ok(None);
            }
            self.now = (self.now + CYCLES_PER_MS / 10).min(deadline);
        }
    }

    fn flush_input(&mut self) -> io::Result<()> {
        self.rx.0.clear();
        Ok(())
    }

    fn idle(&mut self, d: Duration) {
        self.advance(d);
    }
}

/// A plausible warm idle for the engine simulator: 800 rpm, 88 degC, 14.1 V.
pub fn engine_idle_block() -> Vec<u8> {
    let mut b = vec![0u8; 80];
    b[0] = 139; // 14.1 V
    b[3] = 181; // 87.75 degC
    b[5] = 91; // 20.25 degC
    b[7] = 2; // 0.9 deg
    b[10] = 77; // 1504 mV
    b[13..15].copy_from_slice(&4800u16.to_le_bytes()); // 5.8 psi
    b[19..21].copy_from_slice(&800i16.to_le_bytes());
    b[21..23].copy_from_slice(&800i16.to_le_bytes());
    b[26..28].copy_from_slice(&92u16.to_le_bytes()); // 449 mV
    b[30] = 168; // 9 deg BTDC
    b[42..44].copy_from_slice(&700u16.to_le_bytes()); // 2.8 ms
    b[73] = 0b0100; // closed throttle
    b[74..76].copy_from_slice(&800u16.to_le_bytes()); // 3.2 ms
    b
}

/// A moving engine for demos (`kdiag --sim` stays static): `21 01` answers
/// rotate through 120 blocks, a slow blip from idle to about 3000 rpm with
/// the coolant warming up and the O2 sensor switching.
pub fn engine_demo_config() -> cascade_iface::KwpConfig {
    let mut cfg = cascade_iface::KwpConfig {
        dtcs: vec![(0x0130, 0x20)],
        ..Default::default()
    };
    let base = engine_idle_block();
    cfg.local_data.insert(0x01, base.clone());
    let blocks = (0..120)
        .map(|i| {
            let mut b = base.clone();
            let phase = i as f64 / 120.0 * std::f64::consts::TAU;
            let rpm = 800.0 + 1100.0 * (1.0 - phase.cos());
            b[19..21].copy_from_slice(&(rpm as i16).to_le_bytes());
            b[3] = (150 + i / 4) as u8; // coolant rises
            let o2: u16 = if (i / 3) % 2 == 0 { 20 } else { 180 };
            b[26..28].copy_from_slice(&o2.to_le_bytes());
            b[7] = (2.0 + 60.0 * (1.0 - phase.cos()) / 2.0) as u8; // throttle
            let mut r = vec![0x61, 0x01];
            r.extend_from_slice(&b);
            r
        })
        .collect();
    cfg.sequences.insert(vec![0x21, 0x01], blocks);
    cfg
}
