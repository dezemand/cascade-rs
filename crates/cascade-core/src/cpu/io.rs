//! SFR and board I/O (port of `cpu_io.cpp`).

use super::{Cpu, EXROM_BASE, MAPPED_RAM_BASE, Window};
use crate::hsio::{HSI_TIME_HI, HSI_TIME_LO, HSO_TIME_HI, HSO_TIME_LO};
use crate::stop::{R, unimp};

const EXTINT_QUEUE: [u8; 8] = [0, 0xf1, 0xf7, 0x01, 0x42, 0, 0, 0];

impl Cpu {
    fn io_fail<T>(&self, what: &str, addr: u16) -> R<T> {
        unimp(format!(
            "I/O {what} at {addr:#x} (WSR {:02X}, PC {:04X})",
            self.wsr, self.pc
        ))
    }

    pub(super) fn io_read8(&mut self, addr: u16) -> R<u8> {
        let wsr = self.wsr;
        let now = self.cycles;
        let ret = match addr {
            0x02 => match wsr {
                0 => (self.ad_command & 0x7) | (self.ad_command & 0x10),
                15 => self.ad_command,
                _ => return self.io_fail("read", addr),
            },
            0x03 => match wsr {
                0 => (self.ad_result >> 8) as u8,
                1 => 0xff,
                15 => self.hsio.mode(),
                _ => return self.io_fail("read", addr),
            },
            0x04 => match wsr {
                0 => self.hsio.time(HSI_TIME_LO),
                1 => self.ptssel as u8,
                15 => self.hsio.time(HSO_TIME_LO),
                _ => return self.io_fail("read", addr),
            },
            0x05 => match wsr {
                0 => self.hsio.time(HSI_TIME_HI),
                1 => (self.ptssel >> 8) as u8,
                15 => self.hsio.time(HSO_TIME_HI),
                _ => return self.io_fail("read", addr),
            },
            0x07 => match wsr {
                0 => {
                    if self.ioc1 & 0x20 != 0 {
                        self.serial.read_data()
                    } else {
                        0xff
                    }
                }
                15 => 0xff,
                _ => return self.io_fail("read", addr),
            },
            0x08 => self.int_mask,
            0x0a => match wsr {
                0 => self.timer1() as u8,
                15 => 0xff,
                _ => return self.io_fail("read", addr),
            },
            0x0b => match wsr {
                0 => (self.timer1() >> 8) as u8,
                15 => 0xff,
                _ => return self.io_fail("read", addr),
            },
            0x0c => match wsr {
                0 => (self.timer2 / 8) as u8,
                1 => 0xff,
                _ => return self.io_fail("read", addr),
            },
            0x0d => match wsr {
                0 => ((self.timer2 / 8) >> 8) as u8,
                _ => return self.io_fail("read", addr),
            },
            0x0e => 0xff,
            0x0f => {
                self.ioport1 &= !(1 << 4);
                if self.eeprom.read_data() {
                    self.ioport1 |= 1 << 4;
                }
                self.ioport1
            }
            0x10 => {
                let rx = self.serial.rx_state(now)?;
                let r = (self.ioport2 & 0xfd) | (rx << 1);
                self.ioport2 = r;
                r
            }
            0x11 => match wsr {
                0 => {
                    if self.ioc1 & 0x20 != 0 {
                        self.serial.read_stat(now)
                    } else {
                        0
                    }
                }
                15 => self.serial.read_control(),
                _ => return self.io_fail("read", addr),
            },
            0x12 => 0xff,
            0x13 => self.int_mask1,
            0x15 => match wsr {
                0 => self.ios0,
                15 => self.ioc0 | 2,
                _ => return self.io_fail("read", addr),
            },
            0x16 => match wsr {
                0 => {
                    let t1 = self.timer1();
                    if self.last_ios1_read > t1 {
                        self.ios1 |= 1 << 5;
                    }
                    let r = self.ios1;
                    self.ios1 &= 0xc0;
                    self.last_ios1_read = t1;
                    r
                }
                1 => 0xff,
                15 => self.ioc1,
                _ => return self.io_fail("read", addr),
            },
            0x17 => match wsr {
                0 | 1 | 15 => 0xff,
                _ => return self.io_fail("read", addr),
            },
            0x210 => self.lcd.read(false)?,
            0x212 => self.lcd.read(true)?,
            0x200 => match self.ram[0x202] {
                0xe0 => self.keypad.line(0),
                0xe5 => self.keypad.line(1),
                0xe6 => self.keypad.line(2),
                0xec => self.keypad.line(3),
                _ => 0x7f,
            },
            0x236 => {
                self.stats.rand_reads += 1;
                ((self.rand.rand() % 0xff) & 0xef) as u8
            }
            0x201 => {
                self.stats.rand_reads += 1;
                ((self.rand.rand() % 0xff) | 0x08) as u8
            }
            0x240 => {
                let r = EXTINT_QUEUE[self.extint_pos];
                self.extint_pos = (self.extint_pos + 1) % 8;
                r
            }
            0x270 => self.code_lo,
            0x271 => self.code_hi,
            0x272 => self.data_lo,
            0x273 => self.data_hi,
            0x203..=0x20f
            | 0x211
            | 0x213..=0x235
            | 0x237..=0x23f
            | 0x241..=0x26f
            | 0x274..=0x2ff => {
                if addr == 0x274 {
                    0
                } else {
                    self.ram[addr as usize]
                }
            }
            _ => return self.io_fail("byte read", addr),
        };
        Ok(ret)
    }

    pub(super) fn io_write8(&mut self, addr: u16, value: u8) -> R<()> {
        let wsr = self.wsr;
        let now = self.cycles;
        match addr {
            0x00 | 0x01 => {}
            0x02 => match wsr {
                0 => self.ad_command = value,
                15 => {}
                _ => return self.io_fail("write", addr),
            },
            0x03 => match wsr {
                0 => self.hsio.set_mode(value),
                1 | 15 => {}
                _ => return self.io_fail("write", addr),
            },
            0x04 => match wsr {
                0 => self.hsio.set_time(HSO_TIME_LO, value)?,
                1 => self.ptssel = (self.ptssel & 0xff00) | value as u16,
                15 => self.hsio.set_time(HSI_TIME_LO, value)?,
                _ => return self.io_fail("write", addr),
            },
            0x05 => match wsr {
                0 => self.hsio.set_time(HSO_TIME_HI, value)?,
                1 => self.ptssel = (self.ptssel & 0xff) | ((value as u16) << 8),
                15 => self.hsio.set_time(HSI_TIME_HI, value)?,
                _ => return self.io_fail("write", addr),
            },
            0x06 => match wsr {
                0 => self.hsio.set_command(value)?,
                1 => self.ptssrv = (self.ptssrv & 0xff00) | value as u16,
                15 => {} // HSI status, unimplemented and ignored upstream
                _ => return self.io_fail("write", addr),
            },
            0x07 => match wsr {
                0 => {
                    self.stats.uart_tx += 1;
                    self.serial.write_data(now, value)
                }
                1 => self.ptssrv = (self.ptssrv & 0xff) | ((value as u16) << 8),
                15 => {}
                _ => return self.io_fail("write", addr),
            },
            0x08 => self.int_mask = value,
            0x09 => {}
            0x0a => match wsr {
                0 => {}
                15 => {
                    self.timer1_offset &= 0xff00;
                    self.timer1_offset |=
                        ((value as u64).wrapping_sub(self.cycles / 8) & 0xff) as u16;
                }
                _ => return self.io_fail("write", addr),
            },
            0x0b => match wsr {
                0 => {
                    self.timer2_inc_factor = if value & 1 != 0 { 8 } else { 1 };
                    if value & 2 != 0 {
                        self.timer2_inc_factor = -self.timer2_inc_factor;
                    }
                }
                15 => {
                    self.timer1_offset &= 0xff;
                    let hi = (value as u64).wrapping_sub((self.cycles / 8) >> 8) << 8;
                    self.timer1_offset |= hi as u16;
                }
                _ => return self.io_fail("write", addr),
            },
            0x0c => match wsr {
                0 => self.timer2 = (((self.timer2 / 8) & 0xff00) | value as u32).wrapping_mul(8),
                1 => {}
                _ => return self.io_fail("write", addr),
            },
            0x0d => match wsr {
                0 => {
                    self.timer2 =
                        (((self.timer2 / 8) & 0xff) | ((value as u32) << 8)).wrapping_mul(8)
                }
                15 => {}
                _ => return self.io_fail("write", addr),
            },
            0x0e => self.serial.set_baudrate(now, value, self.clock),
            0x0f => {
                self.eeprom.toggle_inputs(
                    value & 0x80 != 0,
                    value & 0x40 != 0,
                    value & 0x20 != 0,
                )?;
                self.ioport1 = value;
            }
            0x10 => {
                if (value & 0xfe) == (self.ioport2 & 0xfe) {
                    self.serial.slow_init(now, value & 1);
                }
                self.ioport2 = value;
            }
            0x11 => match wsr {
                0 => self.serial.set_control(now, value),
                15 => self.serial.set_stat(value),
                _ => return self.io_fail("write", addr),
            },
            0x12 => {}
            0x13 => self.int_mask1 = value,
            0x14 => self.wsr = value,
            0x15 => self.wsr1 = value,
            0x16 => match wsr {
                0 => self.ioc1 = value,
                1 => {}
                15 => self.ios1 = value,
                _ => return self.io_fail("write", addr),
            },
            0x17 => match wsr {
                0 | 1 | 15 => {}
                _ => return self.io_fail("write", addr),
            },
            0x210 => self.lcd.write(false, value)?,
            0x212 => self.lcd.write(true, value)?,
            0x202 => self.ram[addr as usize] = value,
            0x250 => {
                let line = if self.is_hiscan {
                    match value {
                        0xbf => 7,
                        0xf7 => 8,
                        0xfb => 9,
                        0xef => 12,
                        0xfe => 12,
                        0xfd => 10,
                        0xdf => 42,
                        _ => -((!value) as i32),
                    }
                } else {
                    match value {
                        0xff | 0x7f => 7,
                        0xf7 => 8,
                        0xfb => 9,
                        0xbf => 10,
                        0xfe => 12,
                        0xdf => 42,
                        _ => -((!value) as i32),
                    }
                };
                self.serial.set_comm_line(now, line);
            }
            0x254 => {
                let old = self.ram[0x254];
                if value == 0x23 && old != 0x23 {
                    self.serial.set_l(now, 0);
                } else if old == 0x23 && value != 0x23 {
                    self.serial.set_l(now, 1);
                } else if value == 0x33 && old != 0x33 {
                    self.serial.set_l(now, 0);
                } else if old == 0x33 && value != 0x33 {
                    self.serial.set_l(now, 1);
                } else if (value & 0x02) != (old & 0x02) {
                    match value >> 4 {
                        0 | 3 => self.serial.slow_init(now, (value >> 1) & 1),
                        7 => self.serial.set_l(now, (value >> 1) & 1),
                        _ => return unimp(format!("port 254h value {value:02X}")),
                    }
                }
                self.ram[0x254] = value;
            }
            0x25e => self.ram[0x25e] = value,
            0x270 => {
                self.code_lo = value;
                self.code_win = self.virt_to_phys_slow(0xc000, true)?;
            }
            0x271 => {
                self.code_hi = value;
                self.code_win = self.virt_to_phys_slow(0xc000, true)?;
            }
            0x272 => {
                self.data_lo = value;
                let phys = self.virt_to_phys_slow(0xc000, false)?;
                self.data_win = if phys >= MAPPED_RAM_BASE {
                    Window::MappedRam(phys - MAPPED_RAM_BASE)
                } else if phys >= EXROM_BASE {
                    Window::ExRom(phys - EXROM_BASE)
                } else if (phys as u64) + 0x4000 <= self.rom.len() as u64 {
                    // oracle patch 0002: upstream checks rom_size - 0xc000
                    Window::Rom(phys)
                } else {
                    // "invalid ROM data mapping": upstream falls back to offset 0
                    Window::Rom(0)
                };
            }
            0x273 => {
                self.data_hi = value;
                let phys = self.virt_to_phys_slow(0xc000, false)?;
                // no bounds check upstream; window_read reports misuse
                self.data_win = if phys >= MAPPED_RAM_BASE {
                    Window::MappedRam(phys - MAPPED_RAM_BASE)
                } else if phys >= EXROM_BASE {
                    Window::ExRom(phys - EXROM_BASE)
                } else {
                    Window::Rom(phys)
                };
            }
            0x200
            | 0x201
            | 0x203..=0x20f
            | 0x211
            | 0x213..=0x24f
            | 0x251..=0x253
            | 0x255..=0x25d
            | 0x25f..=0x26f
            | 0x274..=0x2ff => self.ram[addr as usize] = value,
            _ => return self.io_fail("byte write", addr),
        }
        Ok(())
    }
}
