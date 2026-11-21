//! Minimal PNG writer (1-bit greyscale, stored deflate blocks), so the CLI
//! can save the LCD without image dependencies.

fn crc32(data: &[u8]) -> u32 {
    let mut c = 0xffff_ffffu32;
    for &b in data {
        c ^= b as u32;
        for _ in 0..8 {
            c = if c & 1 != 0 {
                0xedb8_8320 ^ (c >> 1)
            } else {
                c >> 1
            };
        }
    }
    !c
}

fn adler32(data: &[u8]) -> u32 {
    let (mut a, mut b) = (1u32, 0u32);
    for &d in data {
        a = (a + d as u32) % 65521;
        b = (b + a) % 65521;
    }
    (b << 16) | a
}

fn chunk(out: &mut Vec<u8>, kind: &[u8; 4], data: &[u8]) {
    out.extend_from_slice(&(data.len() as u32).to_be_bytes());
    let start = out.len();
    out.extend_from_slice(kind);
    out.extend_from_slice(data);
    let crc = crc32(&out[start..]);
    out.extend_from_slice(&crc.to_be_bytes());
}

/// Encode a 1-bit image; `pixel(x, y)` true = dark. Scaled by `scale`.
pub fn encode(
    width: usize,
    height: usize,
    scale: usize,
    pixel: impl Fn(usize, usize) -> bool,
) -> Vec<u8> {
    let (w, h) = (width * scale, height * scale);
    let row_bytes = w.div_ceil(8);
    let mut raw = Vec::with_capacity((row_bytes + 1) * h);
    for y in 0..h {
        raw.push(0); // filter: none
        let mut byte = 0u8;
        for x in 0..w {
            // greyscale 1-bit: 1 = white
            if !pixel(x / scale, y / scale) {
                byte |= 0x80 >> (x % 8);
            }
            if x % 8 == 7 || x == w - 1 {
                raw.push(byte);
                byte = 0;
            }
        }
    }
    let mut z = vec![0x78, 0x01];
    let mut chunks = raw.chunks(65535).peekable();
    while let Some(c) = chunks.next() {
        z.push(if chunks.peek().is_none() { 1 } else { 0 });
        z.extend_from_slice(&(c.len() as u16).to_le_bytes());
        z.extend_from_slice(&(!(c.len() as u16)).to_le_bytes());
        z.extend_from_slice(c);
    }
    z.extend_from_slice(&adler32(&raw).to_be_bytes());

    let mut out = b"\x89PNG\r\n\x1a\n".to_vec();
    let mut ihdr = Vec::new();
    ihdr.extend_from_slice(&(w as u32).to_be_bytes());
    ihdr.extend_from_slice(&(h as u32).to_be_bytes());
    ihdr.extend_from_slice(&[1, 0, 0, 0, 0]); // 1-bit greyscale
    chunk(&mut out, b"IHDR", &ihdr);
    chunk(&mut out, b"IDAT", &z);
    chunk(&mut out, b"IEND", &[]);
    out
}

#[cfg(test)]
mod tests {
    #[test]
    fn crc_of_iend_matches_spec() {
        assert_eq!(super::crc32(b"IEND"), 0xae42_6082);
    }
}
