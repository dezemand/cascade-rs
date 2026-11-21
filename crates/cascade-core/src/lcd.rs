//! SED1335 LCD controller (port of `lcd.cpp`), 320x240, two layers XORed.

use crate::stop::{R, unimp};

pub const LCD_WIDTH: usize = 320;
pub const LCD_HEIGHT: usize = 240;
const LAYER2_OFFSET: usize = 0x2580;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum State {
    Idle,
    MWrite,
    MRead,
    CsrW,
    Ovlay,
    DispOnOff,
}

#[derive(Clone)]
pub struct Lcd {
    mem: Vec<u8>,
    state: State,
    next_param: i32,
    cursor: u16,
    pub mx: u8,
    pub dm: u8,
    pub ov: u8,
    pub display_on: bool,
    pub fc: u8,
    pub fp1: u8,
    pub fp2_4: u8,
    pub fp3: u8,
    /// Set when display memory changed since the last `take_dirty`.
    dirty: bool,
}

impl Default for Lcd {
    fn default() -> Self {
        Self {
            mem: vec![0; 65536],
            state: State::Idle,
            next_param: 0,
            cursor: 0,
            mx: 0,
            dm: 0,
            ov: 0,
            display_on: false,
            fc: 0,
            fp1: 0,
            fp2_4: 0,
            fp3: 0,
            dirty: true,
        }
    }
}

impl Lcd {
    /// Upstream `Lcd::reset`: clears memory, state and cursor, keeps mode bits.
    pub fn reset(&mut self) {
        self.mem.fill(0);
        self.state = State::Idle;
        self.next_param = 0;
        self.cursor = 0;
        self.dirty = true;
    }

    /// `a0 == true` is the data port (0x212), false the status port (0x210).
    pub fn read(&mut self, data_port: bool) -> R<u8> {
        if !data_port {
            return unimp("LCD status read");
        }
        match self.state {
            State::MRead => {
                let v = self.mem[self.cursor as usize];
                self.cursor = self.cursor.wrapping_add(1);
                Ok(v)
            }
            s => unimp(format!("LCD read in state {s:?}")),
        }
    }

    /// `command == true` is 0x212 (command), false is 0x210 (data).
    pub fn write(&mut self, command: bool, val: u8) -> R<()> {
        if command {
            match val {
                0x42 => self.state = State::MWrite,
                0x43 => self.state = State::MRead,
                0x46 => {
                    self.state = State::CsrW;
                    self.next_param = 0;
                }
                0x58 | 0x59 => {
                    self.display_on = val & 1 != 0;
                    self.state = State::DispOnOff;
                    self.next_param = 0;
                }
                0x5b => {
                    self.state = State::Ovlay;
                    self.next_param = 0;
                }
                _ => return unimp(format!("LCD command {val:02X}")),
            }
            return Ok(());
        }
        match self.state {
            State::MWrite => {
                self.mem[self.cursor as usize] = val;
                self.dirty = true;
                self.cursor = self.cursor.wrapping_add(1);
            }
            State::CsrW => {
                match self.next_param {
                    0 => self.cursor = val as u16,
                    1 => self.cursor |= (val as u16) << 8,
                    _ => return unimp("LCD error in CSRW"),
                }
                self.next_param += 1;
            }
            State::Ovlay => {
                if self.next_param != 0 {
                    return unimp("LCD error in OVLAY");
                }
                self.mx = val & 3;
                self.dm = (val >> 2) & 1;
                self.ov = (val >> 4) & 1;
                self.next_param += 1;
            }
            State::DispOnOff => {
                // upstream never increments next_param here
                if self.next_param != 0 {
                    return unimp("LCD error in DISP ON/OFF");
                }
                self.fc = val & 3;
                self.fp1 = (val >> 2) & 3;
                self.fp2_4 = (val >> 4) & 3;
                self.fp3 = (val >> 6) & 3;
            }
            s => return unimp(format!("LCD write {val:02X} in state {s:?}")),
        }
        Ok(())
    }

    /// Display memory (64 KB).
    pub fn memory(&self) -> &[u8] {
        &self.mem
    }

    pub fn take_dirty(&mut self) -> bool {
        std::mem::replace(&mut self.dirty, false)
    }

    /// Pixel at (x, y), as upstream draws it: layer 1 XOR layer 2.
    pub fn pixel(&self, x: usize, y: usize) -> bool {
        let i = (y * LCD_WIDTH + x) / 8;
        let byte = self.mem[i] ^ self.mem[LAYER2_OFFSET + i];
        byte & (0x80 >> (x % 8)) != 0
    }

    /// Screen as text, `#` for set pixels, for debugging and tests.
    pub fn render_ascii(&self, step_x: usize, step_y: usize) -> String {
        let mut s = String::new();
        for y in (0..LCD_HEIGHT).step_by(step_y) {
            for x in (0..LCD_WIDTH).step_by(step_x) {
                s.push(if self.pixel(x, y) { '#' } else { ' ' });
            }
            s.push('\n');
        }
        s
    }
}
