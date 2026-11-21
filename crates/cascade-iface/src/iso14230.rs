//! ISO 14230-2 (KWP2000 on K-line) framing, shared by the ECU model and the
//! native diagnostic client.
//!
//! Frame: format byte (`0x80 | len`, or `0x80` with a separate length byte
//! after the addresses), target, source, data, 8-bit sum checksum. Only the
//! physically addressed form with target and source is produced; parsing
//! accepts any addressed form.

/// A decoded frame.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Frame {
    pub target: u8,
    pub source: u8,
    pub data: Vec<u8>,
}

/// 8-bit sum of `bytes`.
pub fn checksum(bytes: &[u8]) -> u8 {
    bytes.iter().fold(0u8, |a, &b| a.wrapping_add(b))
}

/// Total frame length implied by the bytes received so far, if known yet.
pub fn expected_len(frame: &[u8]) -> Option<usize> {
    let fmt = *frame.first()?;
    let hdr = if fmt & 0xc0 != 0 { 3 } else { 1 };
    match (fmt & 0x3f) as usize {
        0 => Some(hdr + 1 + *frame.get(hdr)? as usize + 1),
        len => Some(hdr + len + 1),
    }
}

/// Decode a complete frame. `None` if the checksum is wrong, the length does
/// not match the header, or the frame carries no addresses.
pub fn parse(frame: &[u8]) -> Option<Frame> {
    if frame.len() < 4 || expected_len(frame) != Some(frame.len()) {
        return None;
    }
    let (body, cs) = frame.split_at(frame.len() - 1);
    if checksum(body) != cs[0] || body[0] & 0xc0 == 0 {
        return None;
    }
    let data = if body[0] & 0x3f != 0 {
        &body[3..]
    } else {
        &body[4..]
    };
    Some(Frame {
        target: body[1],
        source: body[2],
        data: data.to_vec(),
    })
}

/// Encode `data` from `source` to `target`. Uses the separate length byte
/// for 64 bytes and more. Panics above 255 bytes, which ISO 14230 cannot carry.
pub fn encode(target: u8, source: u8, data: &[u8]) -> Vec<u8> {
    assert!(
        data.len() <= 255,
        "KWP2000 frame data longer than 255 bytes"
    );
    let mut f = if data.len() < 64 {
        vec![0x80 | data.len() as u8, target, source]
    } else {
        vec![0x80, target, source, data.len() as u8]
    };
    f.extend_from_slice(data);
    f.push(checksum(&f));
    f
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_short_and_long() {
        for len in [1usize, 63, 64, 200] {
            let data: Vec<u8> = (0..len as u8).collect();
            let f = encode(0x11, 0xf1, &data);
            assert_eq!(expected_len(&f[..4]), Some(f.len()));
            assert_eq!(
                parse(&f),
                Some(Frame {
                    target: 0x11,
                    source: 0xf1,
                    data
                })
            );
        }
    }

    #[test]
    fn known_frame_and_bad_checksum() {
        // StartCommunication as sent by the Hi-Scan (verified against the oracle)
        assert_eq!(encode(0x11, 0xf1, &[0x81]), [0x81, 0x11, 0xf1, 0x81, 0x04]);
        assert_eq!(parse(&[0x81, 0x11, 0xf1, 0x81, 0x05]), None);
    }
}
