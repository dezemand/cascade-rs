//! The Hi-Scan Pro's CAN Interface Module (the box between the Hi-Scan and
//! the main cable that the 2006+ menus ask for), with a KWP2000-on-CAN
//! engine ECU behind it. **Experimental:** the module's protocol is read
//! from what the firmware sends; the reply layout is a hypothesis this
//! backend exists to test.
//!
//! What the firmware sends [verified in the emulator, Sonata NF engine]:
//! every message is `F1 mode len payload`, on the line the firmware selects
//! as CAN (I/O 0x250 = 0xDF):
//! - `FA 00`: ping. Answered `42 F2 F6 00` (as upstream CASCADE's fake).
//! - `FB 14 ..`: configuration, 20 bytes; the second one carries `FD 00`
//!   (0x7E8 << 5, the answer identifier) [assumed: acceptance filter].
//!   Answered as the fake does.
//! - `FC 0A id_hi id_lo d0..d7`: one CAN frame, identifier << 5 in two
//!   bytes (`FC 00` = 0x7E0), eight data bytes: ISO-TP frames, KWP2000
//!   services (`10 81`, `18 00 FF 00`).
//! - `F8 00`: close. Answered `42 F2 FF 00`.
//!
//! Hypothesis tested here: the module answers each frame from the car as
//! `42 F2 FC 0A id_hi id_lo d0..d7`, the same layout.

use std::collections::VecDeque;

use crate::{Cycles, Interface, KwpConfig, RxBuf};

/// The engine ECU's identifiers (ISO 15765-4).
const REQUEST_ID: u16 = 0x7e0;
const RESPONSE_ID: u16 = 0x7e8;

/// Reply delay after a complete message (about 1 ms at 10,000 cycles/ms).
const DELAY: Cycles = 10_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Idle,
    Mode,
    Len,
    Payload,
}

pub struct CanModule {
    pub config: KwpConfig,
    state: State,
    mode: u8,
    len: usize,
    payload: Vec<u8>,
    /// (due, bytes) replies waiting to be delivered.
    out: VecDeque<(Cycles, Vec<u8>)>,
    /// Consecutive frames of a long answer, sent after the tester's flow
    /// control.
    pending_cf: VecDeque<[u8; 8]>,
    /// Every CAN frame both ways, for logs: (to the ECU?, id, data).
    pub frames: Vec<(bool, u16, [u8; 8])>,
}

impl CanModule {
    pub fn new(config: KwpConfig) -> Self {
        Self {
            config,
            state: State::Idle,
            mode: 0,
            len: 0,
            payload: Vec::new(),
            out: VecDeque::new(),
            pending_cf: VecDeque::new(),
            frames: Vec::new(),
        }
    }

    fn reply(&mut self, now: Cycles, mode: u8, payload: &[u8]) {
        let mut m = vec![0x42, 0xf2, mode, payload.len() as u8];
        m.extend_from_slice(payload);
        let due = self.out.back().map_or(now, |(t, _)| *t).max(now) + DELAY;
        self.out.push_back((due, m));
    }

    fn frame_to_tester(&mut self, now: Cycles, data: [u8; 8]) {
        self.frames.push((false, RESPONSE_ID, data));
        let id = RESPONSE_ID << 5;
        let mut p = vec![(id >> 8) as u8, id as u8];
        p.extend_from_slice(&data);
        self.reply(now, 0xfc, &p);
    }

    /// The ECU's KWP2000 answer to `req` (service data, SID first).
    fn answer(&self, req: &[u8]) -> Option<Vec<u8>> {
        let c = &self.config;
        if let Some(r) = c.responses.get(req) {
            return Some(r.clone());
        }
        let &sid = req.first()?;
        Some(match req {
            [0x10, sub] => vec![0x50, *sub],
            [0x3e, ..] => vec![0x7e],
            [0x1a, opt] => {
                let mut r = vec![0x5a, *opt];
                r.extend(c.ecu_ident.get(opt)?);
                r
            }
            [0x21, id] => {
                let mut r = vec![0x61, *id];
                r.extend(c.local_data.get(id)?);
                r
            }
            [0x18, ..] => {
                let mut r = vec![0x58, c.dtcs.len() as u8];
                for (code, st) in &c.dtcs {
                    r.extend(code.to_be_bytes());
                    r.push(*st);
                }
                r
            }
            [0x14, ..] => vec![0x54, 0xff, 0x00],
            _ => vec![0x7f, sid, 0x11],
        })
    }

    /// One CAN frame from the tester to the ECU.
    fn tester_frame(&mut self, now: Cycles, id: u16, d: [u8; 8]) {
        self.frames.push((true, id, d));
        if id != REQUEST_ID && id != 0x7df {
            return;
        }
        match d[0] >> 4 {
            0 => {
                let n = (d[0] & 0x0f) as usize;
                if n == 0 || n > 7 {
                    return;
                }
                let Some(r) = self.answer(&d[1..=n]) else {
                    return;
                };
                self.send_message(now, &r);
            }
            // flow control: the rest of a long answer
            3 if d[0] == 0x30 => {
                while let Some(cf) = self.pending_cf.pop_front() {
                    self.frame_to_tester(now, cf);
                }
            }
            _ => {}
        }
    }

    fn send_message(&mut self, now: Cycles, msg: &[u8]) {
        let pad = 0x00;
        if msg.len() <= 7 {
            let mut f = [pad; 8];
            f[0] = msg.len() as u8;
            f[1..=msg.len()].copy_from_slice(msg);
            self.frame_to_tester(now, f);
            return;
        }
        let mut f = [pad; 8];
        f[0] = 0x10 | (msg.len() >> 8) as u8;
        f[1] = msg.len() as u8;
        f[2..8].copy_from_slice(&msg[..6]);
        self.frame_to_tester(now, f);
        self.pending_cf.clear();
        for (i, chunk) in msg[6..].chunks(7).enumerate() {
            let mut cf = [pad; 8];
            cf[0] = 0x20 | ((i + 1) & 0x0f) as u8;
            cf[1..=chunk.len()].copy_from_slice(chunk);
            self.pending_cf.push_back(cf);
        }
    }

    fn message(&mut self, now: Cycles) {
        let p = std::mem::take(&mut self.payload);
        match self.mode {
            0xfa => self.reply(now, 0xf6, &[]),
            // as upstream's fake answers the configuration
            0xfb => self.reply(now, 0xfc, &[0xfc, 0xe0, 0x70, 0x01, 0, 0, 0, 0, 0, 0]),
            0xfc if p.len() == 10 => {
                let id = u16::from_be_bytes([p[0], p[1]]) >> 5;
                let mut d = [0u8; 8];
                d.copy_from_slice(&p[2..10]);
                self.tester_frame(now, id, d);
            }
            _ => self.reply(now, 0xff, &[]),
        }
    }
}

impl Interface for CanModule {
    fn set_baud_divisor(&mut self, _now: Cycles, _divisor: u32, _rx: &mut dyn RxBuf) {}

    fn send_byte(&mut self, now: Cycles, byte: u8, rx: &mut dyn RxBuf) {
        match self.state {
            State::Idle if byte == 0xf1 => {
                rx.flush();
                self.state = State::Mode;
            }
            State::Idle => {}
            State::Mode => {
                self.mode = byte;
                self.state = State::Len;
            }
            State::Len => {
                self.len = byte as usize;
                self.payload.clear();
                if self.len == 0 {
                    self.state = State::Idle;
                    self.message(now);
                } else {
                    self.state = State::Payload;
                }
            }
            State::Payload => {
                self.payload.push(byte);
                if self.payload.len() == self.len {
                    self.state = State::Idle;
                    self.message(now);
                }
            }
        }
    }

    fn poll_input(&mut self, now: Cycles, rx: &mut dyn RxBuf) {
        while self.out.front().is_some_and(|(t, _)| *t <= now) {
            let (_, m) = self.out.pop_front().unwrap();
            for b in m {
                rx.push(b);
            }
        }
    }

    fn set_can(&mut self, _now: Cycles, on: bool, _rx: &mut dyn RxBuf) {
        if !on {
            self.state = State::Idle;
            self.out.clear();
            self.pending_cf.clear();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn send(m: &mut CanModule, t: Cycles, bytes: &[u8], rx: &mut Vec<u8>) {
        for &b in bytes {
            m.send_byte(t, b, rx);
        }
    }

    #[test]
    fn handshake_and_a_kwp_request_over_can() {
        let mut m = CanModule::new(KwpConfig {
            dtcs: vec![(0x0117, 0x60)],
            ..Default::default()
        });
        let mut rx = Vec::new();
        send(&mut m, 0, &[0xf1, 0xfa, 0x00], &mut rx);
        m.poll_input(DELAY, &mut rx);
        assert_eq!(rx, [0x42, 0xf2, 0xf6, 0x00]);
        rx.clear();
        // 18 00 FF 00 to 7E0 (FC 00 = 0x7E0 << 5)
        send(
            &mut m,
            100_000,
            &[
                0xf1, 0xfc, 0x0a, 0xfc, 0x00, 0x04, 0x18, 0x00, 0xff, 0x00, 0, 0, 0,
            ],
            &mut rx,
        );
        m.poll_input(200_000, &mut rx);
        assert_eq!(
            rx,
            [
                0x42, 0xf2, 0xfc, 0x0a, 0xfd, 0x00, 0x05, 0x58, 0x01, 0x01, 0x17, 0x60, 0, 0
            ]
        );
    }

    #[test]
    fn long_answers_wait_for_flow_control() {
        let mut cfg = KwpConfig::default();
        cfg.local_data.insert(0x01, (0..20).collect());
        let mut m = CanModule::new(cfg);
        let mut rx = Vec::new();
        send(
            &mut m,
            0,
            &[
                0xf1, 0xfc, 0x0a, 0xfc, 0x00, 0x02, 0x21, 0x01, 0, 0, 0, 0, 0,
            ],
            &mut rx,
        );
        m.poll_input(100_000, &mut rx);
        assert_eq!(
            &rx[4..8],
            &[0xfd, 0x00, 0x10, 22],
            "first frame of 22 bytes"
        );
        rx.clear();
        send(
            &mut m,
            200_000,
            &[0xf1, 0xfc, 0x0a, 0xfc, 0x00, 0x30, 0, 0, 0, 0, 0, 0, 0],
            &mut rx,
        );
        m.poll_input(400_000, &mut rx);
        assert_eq!(rx.len(), 3 * 14, "three consecutive frames");
        assert_eq!(rx[6], 0x21);
    }
}
