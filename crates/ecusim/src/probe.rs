//! An internal tester: reads the engine ECU over its own K-line while no
//! outside tester uses the line, so the panel can show the ECU's stored
//! codes and readings for any dump (KWP2000 `18 00 00 00` and OBD-II mode 01
//! PIDs, which every dump tried answers), not only for the one whose RAM
//! layout is mapped (`ecu_view`).
//!
//! It runs a complete session each round (fast init, StartCommunication,
//! the reads, StopCommunication) and backs off as soon as an outside tester
//! talks on the engine line.

use std::time::{Duration, Instant};

use cascade_iface::iso14230;
use ecuc167::cpu::Cpu;

/// Requests of one round, after StartCommunication.
const READS: [&[u8]; 4] = [
    &[0x18, 0x00, 0x00, 0x00],
    &[0x01, 0x0c],
    &[0x01, 0x05],
    &[0x01, 0x0d],
];

#[derive(Clone, Debug, Default)]
pub struct Readings {
    /// Stored codes with the status the ECU reports.
    pub codes: Vec<(u16, u8)>,
    pub rpm: Option<f64>,
    pub coolant_c: Option<f64>,
    pub speed_kmh: Option<f64>,
    pub at: Option<Instant>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum State {
    Idle,
    /// Fast-init pattern on the line until this ECU cycle.
    Wake(u64),
    /// Request `step` sent; answer expected before the deadline (cycles).
    Await {
        step: usize,
        deadline: u64,
    },
    /// Pause after an answer (P3) until this cycle, then send `step`.
    Pause {
        step: usize,
        until: u64,
    },
}

pub struct Probe {
    state: State,
    rx: Vec<u8>,
    next_round: Instant,
    last_outside: Option<Instant>,
    pub readings: Readings,
}

impl Default for Probe {
    fn default() -> Self {
        Self {
            state: State::Idle,
            rx: Vec::new(),
            next_round: Instant::now() + Duration::from_secs(4),
            last_outside: None,
            readings: Readings::default(),
        }
    }
}

/// Steps: 0 = StartCommunication, 1..=4 = READS, 5 = StopCommunication.
fn request(step: usize) -> Vec<u8> {
    match step {
        0 => vec![0x81],
        5 => vec![0x82],
        s => READS[s - 1].to_vec(),
    }
}

impl Probe {
    /// An outside tester used the engine line: stop and stay away a while.
    pub fn outside_activity(&mut self) {
        self.last_outside = Some(Instant::now());
        self.state = State::Idle;
        self.rx.clear();
    }

    /// Whether the probe owns the line now (its ECU answers are not
    /// forwarded to outside testers).
    pub fn active(&self) -> bool {
        self.state != State::Idle
    }

    fn send(cpu: &mut Cpu, fcpu: u64, at: u64, data: &[u8]) {
        let bit = fcpu / 10_400;
        for (i, &b) in iso14230::encode(0x11, 0xf1, data).iter().enumerate() {
            cpu.line.frame(at + i as u64 * 10 * bit, b, bit);
        }
    }

    /// Advance with the ECU's latest K-line bytes; `fcpu` is its clock.
    pub fn tick(&mut self, cpu: &mut Cpu, fcpu: u64, ecu_rx: &[u8]) {
        let now = cpu.cycles;
        let ms = fcpu / 1000;
        let quiet = self
            .last_outside
            .is_none_or(|t| t.elapsed() > Duration::from_secs(8));
        match self.state {
            State::Idle => {
                if quiet && Instant::now() >= self.next_round {
                    // fast init: 25 ms low, 25 ms high
                    cpu.line.pull_low(now, now + 25 * ms);
                    self.state = State::Wake(now + 50 * ms);
                    self.rx.clear();
                }
            }
            State::Wake(until) if now >= until => {
                Self::send(cpu, fcpu, now, &request(0));
                self.state = State::Await {
                    step: 0,
                    deadline: now + 500 * ms,
                };
            }
            State::Wake(_) => {}
            State::Await { step, deadline } => {
                self.rx.extend_from_slice(ecu_rx);
                if let Some(n) = iso14230::expected_len(&self.rx)
                    && self.rx.len() >= n
                {
                    let frame: Vec<u8> = self.rx.drain(..n).collect();
                    let Some(f) = iso14230::parse(&frame) else {
                        self.finish(false);
                        return;
                    };
                    if f.data.len() == 3 && f.data[0] == 0x7f && f.data[2] == 0x78 {
                        return; // response pending
                    }
                    self.take(step, &f.data);
                    self.state = if step == 5 {
                        self.finish(true);
                        State::Idle
                    } else {
                        State::Pause {
                            step: step + 1,
                            until: now + 60 * ms,
                        }
                    };
                } else if now > deadline {
                    self.finish(false);
                }
            }
            State::Pause { step, until } if now >= until => {
                Self::send(cpu, fcpu, now, &request(step));
                self.state = State::Await {
                    step,
                    deadline: now + 500 * ms,
                };
            }
            State::Pause { .. } => {}
        }
    }

    fn finish(&mut self, ok: bool) {
        self.state = State::Idle;
        self.rx.clear();
        self.next_round = Instant::now() + Duration::from_secs(if ok { 2 } else { 5 });
    }

    fn take(&mut self, step: usize, d: &[u8]) {
        let r = &mut self.readings;
        match (step, d) {
            (1, [0x58, n, rest @ ..]) => {
                r.codes = rest
                    .as_chunks::<3>()
                    .0
                    .iter()
                    .take(*n as usize)
                    .map(|c| (u16::from_be_bytes([c[0], c[1]]), c[2]))
                    .collect();
                r.at = Some(Instant::now());
            }
            (2, [0x41, 0x0c, a, b, ..]) => r.rpm = Some((*a as f64 * 256.0 + *b as f64) / 4.0),
            (3, [0x41, 0x05, a, ..]) => r.coolant_c = Some(*a as f64 - 40.0),
            (4, [0x41, 0x0d, a, ..]) => r.speed_kmh = Some(*a as f64),
            _ => {}
        }
    }
}
