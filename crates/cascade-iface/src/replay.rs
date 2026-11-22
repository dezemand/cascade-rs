//! Replay of a recorded K-line session (a `kdiag --trace` JSONL file) as a
//! KWP2000 ECU, so the Hi-Scan firmware can show data captured from the car.
//!
//! Replay is by message, not by byte timing: the Hi-Scan asks in its own
//! order and at its own pace, which never matches the recording. Each
//! request in the trace is paired with the ECU's final answer (after any
//! "response pending"); [`KwpEcu`](crate::KwpEcu) then serves the answers to
//! each exact request in recorded order, wrapping around. Requests the
//! recording never made fall back to the ECU model's built-in replies.

use std::collections::BTreeMap;

use crate::iso14230;
use crate::kwp::KwpConfig;

/// What a trace recorded, ready to drive [`KwpEcu`](crate::KwpEcu).
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Recording {
    /// Address the tester talked to (engine 0x11, ABS 0x28, SRS 0x58).
    pub address: Option<u8>,
    /// Key bytes from the recorded StartCommunication answer.
    pub key_bytes: Option<[u8; 2]>,
    /// The ECU answered with the separate length byte (`80 tgt src len`).
    pub length_byte: bool,
    /// Final responses per request, in recorded order.
    pub sequences: BTreeMap<Vec<u8>, Vec<Vec<u8>>>,
    /// Requests that got no (parsable) answer.
    pub unanswered: usize,
    /// The session was an ISO 9141-2 one (5-baud init, `68 6A F1` frames).
    pub iso9141: bool,
    /// Key bytes the ECU sent after `55` in a 5-baud init.
    pub slow_init_key_bytes: Option<[u8; 2]>,
}

impl Recording {
    /// Number of recorded request/response pairs.
    pub fn pairs(&self) -> usize {
        self.sequences.values().map(Vec::len).sum()
    }

    /// An ECU configuration that replays this recording.
    pub fn config(&self) -> KwpConfig {
        let mut c = KwpConfig::default();
        if let Some(a) = self.address {
            c.address = a;
        }
        if let Some(kb) = self.key_bytes {
            c.key_bytes = kb;
        }
        c.length_byte = self.length_byte;
        c.iso9141 = self.iso9141;
        if let Some(kb) = self.slow_init_key_bytes {
            c.slow_init_key_bytes = kb;
        }
        c.sequences = self.sequences.clone();
        c
    }
}

/// Value of `"key":` in one JSONL object written by kdiag's tracer (numbers
/// and plain strings only, no escapes).
fn field<'a>(line: &'a str, key: &str) -> Option<&'a str> {
    let pat = format!("\"{key}\":");
    let rest = &line[line.find(&pat)? + pat.len()..];
    if let Some(s) = rest.strip_prefix('"') {
        Some(&s[..s.find('"')?])
    } else {
        let end = rest.find([',', '}']).unwrap_or(rest.len());
        Some(rest[..end].trim())
    }
}

fn unhex(s: &str) -> Option<Vec<u8>> {
    let s: String = s.chars().filter(|c| !c.is_whitespace()).collect();
    if !s.len().is_multiple_of(2) {
        return None;
    }
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).ok())
        .collect()
}

/// Split received bytes into frames; trailing garbage is dropped.
fn frames(mut bytes: &[u8]) -> Vec<iso14230::Frame> {
    let mut out = Vec::new();
    while !bytes.is_empty() {
        let Some(n) = (1..=bytes.len()).find(|&n| iso14230::expected_len(&bytes[..n]) == Some(n))
        else {
            break;
        };
        match iso14230::parse(&bytes[..n]) {
            Some(f) => {
                out.push(f);
                bytes = &bytes[n..];
            }
            // resynchronise one byte later (noise, a lost byte)
            None => bytes = &bytes[1..],
        }
    }
    out
}

/// Split received bytes into ISO 9141-2 response frames (`48 6B addr data
/// sum`, no length): a frame ends where its checksum matches and the next
/// header (or the end) follows. Returns each frame's data.
fn frames_9141(bytes: &[u8]) -> Vec<Vec<u8>> {
    let mut out = Vec::new();
    let mut i = 0;
    while i + 4 < bytes.len() {
        if bytes[i..i + 2] != [0x48, 0x6b] {
            i += 1;
            continue;
        }
        let end = (i + 4..bytes.len()).find(|&j| {
            iso14230::checksum(&bytes[i..j]) == bytes[j]
                && (j + 1 == bytes.len() || bytes[j + 1..].starts_with(&[0x48, 0x6b]))
        });
        match end {
            Some(j) => {
                out.push(bytes[i + 3..j].to_vec());
                i = j + 1;
            }
            None => i += 1,
        }
    }
    out
}

/// An ISO 9141-2 request frame `68 6A F1 data sum`, as a frame to `6A`.
fn parse_9141_request(tx: &[u8]) -> Option<iso14230::Frame> {
    let (body, sum) = tx.split_at(tx.len().checked_sub(1)?);
    (tx.len() >= 5 && tx[..3] == [0x68, 0x6a, 0xf1] && iso14230::checksum(body) == sum[0]).then(
        || iso14230::Frame {
            target: 0x6a,
            source: 0xf1,
            data: body[3..].to_vec(),
        },
    )
}

struct Pending {
    tx: Vec<u8>,
    req: iso14230::Frame,
    rx: Vec<u8>,
}

fn finish(p: Pending, rec: &mut Recording) {
    // drop the adapter's echo of the request, if the trace kept it
    let rx = p.rx.strip_prefix(p.tx.as_slice()).unwrap_or(&p.rx);
    // a separate length byte on a short frame is the ECU's framing choice;
    // long frames (64+ bytes) always need it
    if rx.first() == Some(&0x80) && rx.get(3).is_some_and(|&n| n < 0x40) {
        rec.length_byte = true;
    }
    if rec.iso9141 {
        // one answer per response frame (mode 03 can send several)
        let answers = frames_9141(rx);
        if answers.is_empty() {
            rec.unanswered += 1;
        }
        for a in answers {
            rec.sequences.entry(p.req.data.clone()).or_default().push(a);
        }
        return;
    }
    let sid = p.req.data.first().copied();
    let answer = frames(rx)
        .into_iter()
        .filter(|f| f.source == p.req.target && f.target == p.req.source)
        .map(|f| f.data)
        .rfind(|d| !(d.len() == 3 && d[0] == 0x7f && Some(d[1]) == sid && d[2] == 0x78));
    let Some(answer) = answer else {
        rec.unanswered += 1;
        return;
    };
    if let ([0x81], [0xc1, a, b]) = (p.req.data.as_slice(), answer.as_slice()) {
        rec.key_bytes = Some([*a, *b]);
    }
    rec.address.get_or_insert(p.req.target);
    rec.sequences.entry(p.req.data).or_default().push(answer);
}

/// Parse a `kdiag --trace` JSONL file. Lines that are not trace events are
/// an error; unknown event kinds are ignored.
pub fn from_jsonl(text: &str) -> Result<Recording, String> {
    let mut rec = Recording::default();
    let mut cur: Option<Pending> = None;
    // bytes received after a 5-baud address: 55, key bytes, inverted address
    let mut slow: Option<Vec<u8>> = None;
    for (n, line) in text.lines().enumerate() {
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        let ev = field(line, "ev").ok_or_else(|| format!("line {}: no \"ev\"", n + 1))?;
        match ev {
            "tx" => {
                if let Some(p) = cur.take() {
                    finish(p, &mut rec);
                }
                let tx = field(line, "data")
                    .and_then(unhex)
                    .ok_or_else(|| format!("line {}: bad tx data", n + 1))?;
                if let Some([0x55, a, b, ..]) = slow.take().as_deref() {
                    rec.slow_init_key_bytes = Some([*a, *b]);
                }
                // a tx that is not a whole frame (a wake byte) starts nothing
                let req = if rec.iso9141 {
                    parse_9141_request(&tx)
                } else {
                    iso14230::parse(&tx)
                };
                cur = req.map(|req| Pending {
                    tx,
                    req,
                    rx: Vec::new(),
                });
            }
            "rx" => {
                let b = field(line, "byte")
                    .and_then(unhex)
                    .filter(|b| b.len() == 1)
                    .ok_or_else(|| format!("line {}: bad rx byte", n + 1))?;
                if let Some(p) = &mut cur {
                    p.rx.push(b[0]);
                } else if let Some(v) = &mut slow {
                    v.push(b[0]);
                }
            }
            "five_baud" => {
                if let Some(p) = cur.take() {
                    finish(p, &mut rec);
                }
                let addr = field(line, "addr")
                    .and_then(|a| a.parse::<u8>().ok())
                    .ok_or_else(|| format!("line {}: bad five_baud addr", n + 1))?;
                rec.address = Some(addr);
                rec.iso9141 = true;
                slow = Some(Vec::new());
            }
            "wake" | "baud" => {
                if let Some(p) = cur.take() {
                    finish(p, &mut rec);
                }
            }
            _ => {} // timeouts and future event kinds
        }
    }
    if let Some(p) = cur.take() {
        finish(p, &mut rec);
    }
    if rec.sequences.is_empty() {
        return Err("no answered requests in the trace".into());
    }
    Ok(rec)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kwp::KwpEcu;
    use crate::{Cycles, Interface};

    fn tx(t: u32, f: &[u8]) -> String {
        let h: String = f.iter().map(|b| format!("{b:02X}")).collect();
        format!("{{\"t_us\":{t},\"ev\":\"tx\",\"data\":\"{h}\"}}\n")
    }
    fn rx(t: u32, f: &[u8]) -> String {
        f.iter()
            .map(|b| format!("{{\"t_us\":{t},\"ev\":\"rx\",\"byte\":\"{b:02X}\"}}\n"))
            .collect()
    }

    fn trace() -> String {
        let mut s = String::from("{\"t_us\":0,\"ev\":\"baud\",\"baud\":10400}\n");
        s += "{\"t_us\":1,\"ev\":\"wake\",\"low_us\":25000,\"high_us\":25000}\n";
        s += &tx(2, &[0x81, 0x11, 0xf1, 0x81, 0x04]);
        s += &rx(3, &[0x83, 0xf1, 0x11, 0xc1, 0xea, 0x8f, 0xbf]);
        for v in [0x10u8, 0x20] {
            s += &tx(4, &iso14230::encode(0x11, 0xf1, &[0x21, 0x01]));
            s += &rx(5, &iso14230::encode(0xf1, 0x11, &[0x7f, 0x21, 0x78]));
            s += &rx(6, &iso14230::encode(0xf1, 0x11, &[0x61, 0x01, v]));
        }
        s += &tx(7, &iso14230::encode(0x11, 0xf1, &[0x1a, 0x8e]));
        s += "{\"t_us\":8,\"ev\":\"timeout\",\"after_us\":50000}\n";
        s
    }

    #[test]
    fn iso9141_session_replays() {
        let req = |d: &[u8]| {
            let mut f = vec![0x68, 0x6a, 0xf1];
            f.extend_from_slice(d);
            f.push(iso14230::checksum(&f));
            f
        };
        let resp = |d: &[u8]| {
            let mut f = vec![0x48, 0x6b, 0x32];
            f.extend_from_slice(d);
            f.push(iso14230::checksum(&f));
            f
        };
        let mut s = String::from("{\"t_us\":0,\"ev\":\"five_baud\",\"addr\":50}\n");
        s += &rx(1, &[0x55, 0x08, 0x08]);
        s += &tx(2, &[0xf7]);
        s += &rx(3, &[0xcd]);
        s += &tx(4, &req(&[0x03]));
        let mut two = resp(&[0x43, 0x52, 0x00, 0x52, 0x03, 0x52, 0x06]);
        two.extend(resp(&[0x43, 0x52, 0x09, 0x00, 0x00, 0x00, 0x00]));
        s += &rx(5, &two);
        s += &tx(6, &req(&[0xb1, 0x0e]));
        s += &rx(7, &resp(&[0xf1, 0x0e, 10, 20, 30, 40]));
        let r = from_jsonl(&s).unwrap();
        assert!(r.iso9141);
        assert_eq!(r.address, Some(0x32));
        assert_eq!(r.slow_init_key_bytes, Some([0x08, 0x08]));
        assert_eq!(r.sequences[&vec![0x03]].len(), 2, "one answer per frame");
        assert_eq!(
            r.sequences[&vec![0xb1, 0x0e]],
            [vec![0xf1, 0x0e, 10, 20, 30, 40]]
        );
        let c = r.config();
        assert!(c.iso9141 && c.address == 0x32);
    }

    #[test]
    fn pairs_requests_with_final_answers() {
        let r = from_jsonl(&trace()).unwrap();
        assert_eq!(r.address, Some(0x11));
        assert_eq!(r.key_bytes, Some([0xea, 0x8f]));
        assert!(!r.length_byte);
        let mut long = trace();
        long += &tx(9, &iso14230::encode(0x11, 0xf1, &[0x21, 0x02]));
        long += &rx(10, &iso14230::encode(0xf1, 0x11, &[0x61; 70]));
        assert!(
            !from_jsonl(&long).unwrap().length_byte,
            "a long frame says nothing about framing"
        );
        assert_eq!(r.unanswered, 1);
        assert_eq!(
            r.sequences[&vec![0x21, 0x01]],
            [vec![0x61, 0x01, 0x10], vec![0x61, 0x01, 0x20]],
            "response pending dropped, order kept"
        );
    }

    #[test]
    fn echo_is_stripped_and_length_byte_detected() {
        let req = [0x80, 0x28, 0xf1, 0x01, 0x82, 0x1c];
        let resp = [0x80, 0xf1, 0x28, 0x01, 0xc2, 0x5c];
        let mut s = tx(0, &req);
        s += &rx(1, &req);
        s += &rx(2, &resp);
        let r = from_jsonl(&s).unwrap();
        assert!(r.length_byte);
        assert_eq!(r.address, Some(0x28));
        assert_eq!(r.sequences[&vec![0x82]], [vec![0xc2]]);
    }

    #[test]
    fn ecu_serves_recorded_answers_in_rotation() {
        let mut ecu = KwpEcu::new(from_jsonl(&trace()).unwrap().config());
        let (mut now, mut out): (Cycles, Vec<u8>) = (0, Vec::new());
        let mut ask = |ecu: &mut KwpEcu, data: &[u8]| {
            for b in iso14230::encode(0x11, 0xf1, data) {
                ecu.send_byte(now, b, &mut out);
                now += 10_000;
            }
            now += ecu.config.p2;
            ecu.poll_input(now, &mut out);
            iso14230::parse(&std::mem::take(&mut out)).unwrap().data
        };
        assert_eq!(ask(&mut ecu, &[0x81]), [0xc1, 0xea, 0x8f]);
        assert_eq!(ask(&mut ecu, &[0x21, 0x01]), [0x61, 0x01, 0x10]);
        assert_eq!(ask(&mut ecu, &[0x21, 0x01]), [0x61, 0x01, 0x20]);
        assert_eq!(ask(&mut ecu, &[0x21, 0x01]), [0x61, 0x01, 0x10]);
        assert_eq!(ask(&mut ecu, &[0x3e]), [0x7e], "built-in fallback");
    }

    #[test]
    fn rejects_garbage() {
        assert!(from_jsonl("hello\n").is_err());
        assert!(from_jsonl("").is_err());
    }
}
