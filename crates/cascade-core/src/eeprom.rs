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
        }
    }
}

impl Eeprom {
    pub fn toggle_inputs(&mut self, ena: bool, clk: bool, bit: bool) -> R<()> {
        if !self.enable && ena {
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
