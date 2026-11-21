//! HSI/HSO unit (port of `hsio.cpp`). Only the software timers (SWT0..3)
//! are modelled; they drive the timer interrupt at vector 0x200A.

use crate::stop::{R, oob, unimp};

pub const HSI_TIME_LO: u8 = 0;
pub const HSI_TIME_HI: u8 = 1;
pub const HSO_TIME_LO: u8 = 2;
pub const HSO_TIME_HI: u8 = 3;

#[derive(Clone)]
pub struct Hsio {
    hsi_mode: u8,
    hso_swt_time: [u16; 4],
    hso_command: u8,
    hso_swt_command: [u8; 4],
}

impl Default for Hsio {
    fn default() -> Self {
        Self {
            hsi_mode: 0xff,
            hso_swt_time: [0; 4],
            hso_command: 0,
            hso_swt_command: [0; 4],
        }
    }
}

impl Hsio {
    pub fn mode(&self) -> u8 {
        self.hsi_mode
    }

    pub fn set_mode(&mut self, m: u8) {
        self.hsi_mode = m;
    }

    /// All HSI/HSO time reads are faked as 0xFF upstream.
    pub fn time(&self, _which: u8) -> u8 {
        0xff
    }

    pub fn set_time(&mut self, which: u8, value: u8) -> R<()> {
        match which {
            HSO_TIME_LO | HSO_TIME_HI => {
                if self.hso_command & 0xf0 != 0x30 {
                    return Ok(()); // "HSO unimp1", ignored upstream
                }
                let swt = (self.hso_command & 0xf) as i32 - 8;
                if !(0..4).contains(&swt) {
                    return oob(format!(
                        "HSO time write with command {:02X}",
                        self.hso_command
                    ));
                }
                let t = &mut self.hso_swt_time[swt as usize];
                *t = if which == HSO_TIME_LO {
                    (*t & 0xff00) | value as u16
                } else {
                    (*t & 0xff) | ((value as u16) << 8)
                };
                Ok(())
            }
            _ => unimp("HSI time write"),
        }
    }

    pub fn set_command(&mut self, cmd: u8) -> R<()> {
        self.hso_command = cmd;
        if cmd & 0xf0 != 0x30 {
            return Ok(()); // "HSO unimp5", ignored upstream
        }
        match cmd & 0xf {
            0x8..=0xb => {
                self.hso_swt_command[(cmd & 0xf) as usize - 8] = cmd;
                Ok(())
            }
            _ => unimp(format!("HSIO command {cmd:02X}")),
        }
    }

    /// Fires SWT `which` if its time lies in (old, new], with wrap-around.
    pub fn swt_interrupt(&mut self, old: u16, new: u16, which: usize) -> bool {
        let t = self.hso_swt_time[which];
        if self.hso_swt_command[which] != 0
            && ((new > old && t > old && t <= new) || (new < old && (t > old || t <= new)))
        {
            self.hso_swt_command[which] = 0;
            true
        } else {
            false
        }
    }
}
