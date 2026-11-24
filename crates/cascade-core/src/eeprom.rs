//! 93Cxx-style serial EEPROM bit-banged through IOPORT1 (port of `eeprom.cpp`).

use crate::stop::{R, oob, unimp};

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Mode {
    Cmd,
    AddrWrite,
    DataWrite,
    AddrRead,
    DataRead,
    Unknown,
}

#[derive(Clone)]
pub struct Eeprom {
    enable: bool,
    clock: bool,
    bit_count: i32,
    cmd: u8,
    data: u32,
    addr: u16,
    mode: Mode,
    /// 128 16-bit words, stored as upstream's `.eep` file (host byte order).
    pub mem: [u16; 128],
    /// Transactions started (chip selects), for activity displays.
    pub accesses: u64,
}

impl Default for Eeprom {
    fn default() -> Self {
        Self {
            enable: false,
            clock: true,
            bit_count: 0,
            cmd: 0,
            data: 0,
            addr: 0,
            mode: Mode::Cmd,
            mem: [0; 128],
            accesses: 0,
        }
    }
}

/// Size of a saved EEPROM image: 128 words, little-endian.
pub const IMAGE_BYTES: usize = 256;

impl Eeprom {
    /// The contents as a file image: 128 words, little-endian (upstream's
    /// `.eep` format on the little-endian hosts it ran on).
    pub fn to_image(&self) -> Vec<u8> {
        self.mem.iter().flat_map(|w| w.to_le_bytes()).collect()
    }

    /// Load contents from a file image. Upstream writes only the first 128
    /// bytes (64 words) of its `.eep` files; such files load into the first
    /// half and leave the rest as it was. Longer input is truncated.
    pub fn load_image(&mut self, image: &[u8]) {
        for (w, c) in self.mem.iter_mut().zip(image.chunks(2)) {
            *w = u16::from_le_bytes([c[0], c.get(1).copied().unwrap_or((*w >> 8) as u8)]);
        }
    }

    pub fn toggle_inputs(&mut self, ena: bool, clk: bool, bit: bool) -> R<()> {
        if !self.enable && ena {
            self.accesses += 1;
            self.mode = Mode::Cmd;
            self.cmd = 0;
            self.bit_count = 3;
        } else if self.enable && !ena {
            self.enable = false;
        } else if self.enable && self.clock && !clk {
            match self.mode {
                Mode::Cmd => {
                    self.bit_count -= 1;
                    if bit {
                        self.cmd |= 1 << self.bit_count;
                    }
                    if self.bit_count == 0 {
                        self.mode = match self.cmd {
                            6 => Mode::AddrRead,
                            5 => Mode::AddrWrite,
                            4 => Mode::Unknown,
                            c => return unimp(format!("EEPROM command {c:#x}")),
                        };
                        self.bit_count = 8;
                        self.addr = 0;
                    }
                }
                Mode::AddrRead | Mode::AddrWrite => {
                    self.bit_count -= 1;
                    if bit {
                        self.addr |= 1 << self.bit_count;
                    }
                    if self.bit_count == 0 {
                        self.bit_count = 16;
                        if self.mode == Mode::AddrRead {
                            self.mode = Mode::DataRead;
                            self.data = self.word(self.addr)? as u32;
                        } else {
                            self.mode = Mode::DataWrite;
                            self.data = 0;
                        }
                    }
                }
                Mode::DataRead | Mode::Unknown => {
                    if self.bit_count != 0 {
                        self.bit_count -= 1;
                    }
                }
                Mode::DataWrite => {
                    if self.bit_count != 0 {
                        self.bit_count -= 1;
                        if bit {
                            self.data |= 1 << self.bit_count;
                        }
                    }
                    if self.bit_count == 0 {
                        let a = self.addr as usize;
                        if a >= self.mem.len() {
                            return oob(format!("EEPROM write to word {a:#x}"));
                        }
                        self.mem[a] = self.data as u16;
                    }
                }
            }
        }
        self.enable = ena;
        self.clock = clk;
        Ok(())
    }

    fn word(&self, addr: u16) -> R<u16> {
        match self.mem.get(addr as usize) {
            Some(w) => Ok(*w),
            None => oob(format!("EEPROM read of word {addr:#x}")),
        }
    }

    pub fn read_data(&self) -> bool {
        // bit_count is 0..=16 here; data is u32
        self.data & (1u32 << self.bit_count) != 0
    }

    pub fn erase(&mut self) {
        self.mem = [0; 128];
    }
}

#[cfg(test)]
mod image_tests {
    use super::*;

    #[test]
    fn image_round_trip_and_upstream_half_files() {
        let mut e = Eeprom::default();
        for (i, w) in e.mem.iter_mut().enumerate() {
            *w = 0x1000 + i as u16;
        }
        let img = e.to_image();
        assert_eq!(img.len(), IMAGE_BYTES);
        assert_eq!(&img[..4], &[0x00, 0x10, 0x01, 0x10], "little-endian words");
        let mut f = Eeprom::default();
        f.load_image(&img);
        assert_eq!(f.mem, e.mem);
        // an upstream .eep file: only the first 128 bytes
        let mut g = Eeprom::default();
        g.mem[100] = 0xbeef;
        g.load_image(&img[..128]);
        assert_eq!(g.mem[63], 0x1000 + 63);
        assert_eq!(g.mem[64], 0, "second half untouched");
        assert_eq!(g.mem[100], 0xbeef);
    }
}
