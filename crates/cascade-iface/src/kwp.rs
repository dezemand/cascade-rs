//! A KWP2000 (ISO 14230) ECU on the K-line, for driving the Hi-Scan firmware
//! past the point where upstream's fake interface goes silent.
//!
//! This is **not** a port of anything upstream: it is a model written for
//! this project, and its replies are invented placeholders unless a comment
//! says otherwise. What it is for:
//! - letting the firmware establish a session so its later screens and
//!   requests can be observed;
//! - logging every request the firmware makes, which is the raw material for
//!   the protocol definitions (Phase 4).
//!
//! Framing follows ISO 14230-2 as also implemented by gkbus (GKFlasher's bus
//! library, which talks to real SIMK41/43 ECUs): format byte `0x80 | len`
//! (or `0x80` plus a length byte), target, source, data, 8-bit sum checksum.
//!
//! Timing is in emulated cycles; 10,000 cycles are 1 ms (upstream's clock
//! bookkeeping: `cycles * 2 * 1000 / 20 MHz` ms).

use std::cell::RefCell;
use std::collections::BTreeMap;
use std::rc::Rc;

use crate::iso14230;
use crate::{Cycles, Interface, RxBuf};

/// Emulated cycles per millisecond.
pub const CYCLES_PER_MS: Cycles = 10_000;

/// One request and what the ECU answered.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Exchange {
    /// Cycle at which the request's last byte was sent.
    pub at: Cycles,
    pub target: u8,
    pub source: u8,
    /// Service data (SID and parameters), without header or checksum.
    pub request: Vec<u8>,
    /// Response service data, or `None` if the ECU stayed silent.
    pub response: Option<Vec<u8>>,
}

pub type ExchangeLog = Rc<RefCell<Vec<Exchange>>>;

/// Configuration of the simulated ECU.
#[derive(Clone, Debug)]
pub struct KwpConfig {
    /// Physical address the ECU answers to (engine: 0x11).
    pub address: u8,
    /// Key bytes returned to StartCommunication.
    pub key_bytes: [u8; 2],
    /// Delay from end of request to start of response (P2).
    pub p2: Cycles,
    /// Session ends after this long without a request (P3max).
    pub p3_max: Cycles,
    /// ReadEcuIdentification (0x1A) answers by option byte.
    pub ecu_ident: BTreeMap<u8, Vec<u8>>,
    /// ReadDataByLocalIdentifier (0x21) answers by local identifier.
    pub local_data: BTreeMap<u8, Vec<u8>>,
    /// Stored DTCs as (code, status) for ReadDTCsByStatus (0x18).
    pub dtcs: Vec<(u16, u8)>,
    /// Fixed responses for exact requests (service data, SID first),
    /// checked before the built-in services once a session is active.
    pub responses: BTreeMap<Vec<u8>, Vec<u8>>,
    /// Always use the separate length byte (`80 tgt src len ...`), even
    /// for short responses.
    pub length_byte: bool,
}

impl Default for KwpConfig {
    fn default() -> Self {
        Self {
            address: 0x11,
            // EF: length in format byte or extra byte, 1-byte header and
            // target/source addressing supported, normal timing. 8F: ISO 14230.
            key_bytes: [0xef, 0x8f],
            p2: 25 * CYCLES_PER_MS,
            p3_max: 5000 * CYCLES_PER_MS,
            ecu_ident: BTreeMap::new(),
            local_data: BTreeMap::new(),
            dtcs: Vec::new(),
            responses: BTreeMap::new(),
            length_byte: false,
        }
    }
}

/// Negative response codes (ISO 14230-3).
pub mod nrc {
    pub const SERVICE_NOT_SUPPORTED: u8 = 0x11;
    pub const SUBFUNCTION_NOT_SUPPORTED: u8 = 0x12;
    pub const REQUEST_OUT_OF_RANGE: u8 = 0x31;
}

pub struct KwpEcu {
    pub config: KwpConfig,
    rx_frame: Vec<u8>,
    last_byte_at: Cycles,
    active: bool,
    last_request_at: Cycles,
    /// Encoded response frame waiting for its P2 time.
    pending: Option<(Cycles, Vec<u8>)>,
    log: ExchangeLog,
}

impl KwpEcu {
    pub fn new(config: KwpConfig) -> Self {
        Self {
            config,
            rx_frame: Vec::new(),
            last_byte_at: 0,
            active: false,
            last_request_at: 0,
            pending: None,
            log: ExchangeLog::default(),
        }
    }

    pub fn log_handle(&self) -> ExchangeLog {
        self.log.clone()
    }

    fn encode(&self, target: u8, data: &[u8]) -> Vec<u8> {
        if self.config.length_byte {
            let mut f = vec![0x80, target, self.config.address, data.len() as u8];
            f.extend_from_slice(data);
            f.push(iso14230::checksum(&f));
            f
        } else {
            iso14230::encode(target, self.config.address, data)
        }
    }

    fn negative(sid: u8, code: u8) -> Option<Vec<u8>> {
        Some(vec![0x7f, sid, code])
    }

    /// Service dispatch. `None` means no response.
    fn respond(&mut self, req: &[u8]) -> Option<Vec<u8>> {
        let sid = *req.first()?;
        if sid == 0x81 {
            self.active = true;
            let [kb1, kb2] = self.config.key_bytes;
            return Some(vec![0xc1, kb1, kb2]);
        }
        if !self.active {
            return None; // a real ECU ignores requests before StartCommunication
        }
        if let Some(r) = self.config.responses.get(req) {
            return Some(r.clone());
        }
        match sid {
            0x82 => {
                self.active = false;
                Some(vec![0xc2])
            }
            0x3e => {
                // TesterPresent; responseRequired = 0x01 or absent
                if req.get(1) == Some(&0x02) {
                    None
                } else {
                    Some(vec![0x7e])
                }
            }
            0x10 => match req.get(1) {
                Some(&mode) => Some(vec![0x50, mode]),
                None => Self::negative(sid, nrc::SUBFUNCTION_NOT_SUPPORTED),
            },
            0x1a => {
                let opt = *req.get(1)?;
                match self.config.ecu_ident.get(&opt) {
                    Some(v) => {
                        let mut r = vec![0x5a, opt];
                        r.extend_from_slice(v);
                        Some(r)
                    }
                    None => Self::negative(sid, nrc::REQUEST_OUT_OF_RANGE),
                }
            }
            0x21 => {
                let id = *req.get(1)?;
                match self.config.local_data.get(&id) {
                    Some(v) => {
                        let mut r = vec![0x61, id];
                        r.extend_from_slice(v);
                        Some(r)
                    }
                    None => Self::negative(sid, nrc::REQUEST_OUT_OF_RANGE),
                }
            }
            0x18 => {
                let mut r = vec![0x58, self.config.dtcs.len() as u8];
                for &(code, status) in &self.config.dtcs {
                    r.extend_from_slice(&[(code >> 8) as u8, code as u8, status]);
                }
                Some(r)
            }
            0x14 => {
                self.config.dtcs.clear();
                let group = req.get(1..3).unwrap_or(&[0xff, 0x00]);
                Some(vec![0x54, group[0], group[1]])
            }
            _ => Self::negative(sid, nrc::SERVICE_NOT_SUPPORTED),
        }
    }

    fn frame_complete(&mut self, now: Cycles) {
        let frame = std::mem::take(&mut self.rx_frame);
        let Some(iso14230::Frame {
            target,
            source,
            data,
        }) = iso14230::parse(&frame)
        else {
            return;
        };
        if target != self.config.address {
            return;
        }
        if self.active && now.saturating_sub(self.last_request_at) > self.config.p3_max {
            self.active = false; // P3 expired: session is gone
        }
        self.last_request_at = now;
        let response = self.respond(&data);
        if let Some(r) = &response {
            let encoded = self.encode(source, r);
            self.pending = Some((now + self.config.p2, encoded));
        }
        self.log.borrow_mut().push(Exchange {
            at: now,
            target,
            source,
            request: data,
            response,
        });
    }
}

impl Interface for KwpEcu {
    fn set_baud_divisor(&mut self, _now: Cycles, _divisor: u32, _rx: &mut dyn RxBuf) {
        // the emulated UART and this ECU always agree on speed
    }

    fn send_byte(&mut self, now: Cycles, byte: u8, _rx: &mut dyn RxBuf) {
        // a gap longer than P4max-ish (20 ms) starts a new frame
        if !self.rx_frame.is_empty() && now - self.last_byte_at > 20 * CYCLES_PER_MS {
            self.rx_frame.clear();
        }
        self.last_byte_at = now;
        self.rx_frame.push(byte);
        if iso14230::expected_len(&self.rx_frame) == Some(self.rx_frame.len()) {
            self.frame_complete(now);
        }
    }

    fn poll_input(&mut self, now: Cycles, rx: &mut dyn RxBuf) {
        if let Some((at, _)) = &self.pending
            && now >= *at
        {
            let (_, frame) = self.pending.take().expect("checked above");
            for b in frame {
                rx.push(b);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn send(ecu: &mut KwpEcu, now: &mut Cycles, frame: &[u8], rx: &mut Vec<u8>) {
        for &b in frame {
            ecu.send_byte(*now, b, rx);
            *now += 55_000; // about one byte time at 10400 baud
        }
    }

    fn reply_after(ecu: &mut KwpEcu, now: &mut Cycles, rx: &mut Vec<u8>) -> Vec<u8> {
        *now += ecu.config.p2;
        ecu.poll_input(*now, rx);
        std::mem::take(rx)
    }

    #[test]
    fn start_communication_and_tester_present() {
        let mut ecu = KwpEcu::new(KwpConfig::default());
        let (mut now, mut rx) = (0, Vec::new());
        // exactly what the Hi-Scan firmware sends (verified against the oracle)
        send(&mut ecu, &mut now, &[0x81, 0x11, 0xf1, 0x81, 0x04], &mut rx);
        assert!(rx.is_empty(), "nothing before P2");
        assert_eq!(
            reply_after(&mut ecu, &mut now, &mut rx),
            [0x83, 0xf1, 0x11, 0xc1, 0xef, 0x8f, 0xc4]
        );
        send(&mut ecu, &mut now, &[0x81, 0x11, 0xf1, 0x3e, 0xc1], &mut rx);
        assert_eq!(
            reply_after(&mut ecu, &mut now, &mut rx),
            [0x81, 0xf1, 0x11, 0x7e, 0x01]
        );
        let log = ecu.log_handle();
        assert_eq!(log.borrow().len(), 2);
        assert_eq!(log.borrow()[1].request, [0x3e]);
    }

    #[test]
    fn silent_before_start_and_on_bad_checksum() {
        let mut ecu = KwpEcu::new(KwpConfig::default());
        let (mut now, mut rx) = (0, Vec::new());
        send(&mut ecu, &mut now, &[0x81, 0x11, 0xf1, 0x3e, 0xc1], &mut rx);
        assert!(reply_after(&mut ecu, &mut now, &mut rx).is_empty());
        send(&mut ecu, &mut now, &[0x81, 0x11, 0xf1, 0x81, 0x05], &mut rx);
        assert!(reply_after(&mut ecu, &mut now, &mut rx).is_empty());
    }

    #[test]
    fn unknown_service_is_refused_and_p3_ends_session() {
        let mut ecu = KwpEcu::new(KwpConfig::default());
        let (mut now, mut rx) = (0, Vec::new());
        send(&mut ecu, &mut now, &[0x81, 0x11, 0xf1, 0x81, 0x04], &mut rx);
        reply_after(&mut ecu, &mut now, &mut rx);
        send(&mut ecu, &mut now, &[0x81, 0x11, 0xf1, 0x30, 0xb3], &mut rx);
        assert_eq!(
            reply_after(&mut ecu, &mut now, &mut rx),
            [0x83, 0xf1, 0x11, 0x7f, 0x30, 0x11, 0x45]
        );
        now += 6000 * CYCLES_PER_MS;
        send(&mut ecu, &mut now, &[0x81, 0x11, 0xf1, 0x3e, 0xc1], &mut rx);
        assert!(
            reply_after(&mut ecu, &mut now, &mut rx).is_empty(),
            "session timed out"
        );
    }
}
