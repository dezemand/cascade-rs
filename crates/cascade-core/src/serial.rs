//! Emulated 8XC196 serial port (port of `serial.cpp`).
//!
//! Upstream's record/replay paths (`isReplaying`) are not ported: in this
//! port, determinism comes from cycle-stamped input instead. UI and hint
//! calls are dropped; wall-clock `sync()` calls are dropped.

use cascade_iface::Interface;

use crate::ring::Ring;
use crate::stop::{R, unimp};

const SERSTAT_OE: u8 = 1 << 2;
const SERSTAT_TXE: u8 = 1 << 3;
const SERSTAT_FE: u8 = 1 << 4;
const SERSTAT_TI: u8 = 1 << 5;
const SERSTAT_RI: u8 = 1 << 6;
const SERSTAT_RPERB8: u8 = 1 << 7;

const FORCED_BAUD_RATES: [u16; 3] = [0x8081, 0x8077, 0x8102];

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum SlowInit {
    Normal,
    Iso,
    #[allow(dead_code)] // never entered upstream either; read in read_stat
    Spam,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum BaudrateMethod {
    Auto,
    AutoPlus,
    Force,
}

pub struct Serial {
    pub iface: Box<dyn Interface>,
    baudrate: u16,
    specified_baudrate: u16,
    pub baudrate_method: BaudrateMethod,
    pub fixed_baudrate: i32,
    comm_line: i32,
    control: u8,
    stat: u8,
    baudrate_pos: i32,
    slow_init_target: u8,
    slow_init_count: i32,
    si_state: SlowInit,
    ti_set_time: u64,
    ri_set_time: u64,
    read_after_write: bool,
    enable_echo: bool,
    echo_back_on_time: u64,
    ti_set_byte: u8,
    force_baud_rate: bool,
    bitbang_reads_after_slow_init: i32,
    forced_baud_rate_no: usize,
    last_slow_init: u64,
    serial_bitbang_enabled: bool,
    serial_bitbang_bit_pos: i32,
    serial_bitbang_last: u64,
    serial_bitbang_last_bit_sent: u8,
    rx_buf: Ring<u8, 256>,
}

impl Serial {
    /// `Serial::Serial` followed by `setEcho(expect_echo)` as in `Cpu::setSerial`.
    pub fn new(iface: Box<dyn Interface>, expect_echo: bool) -> Self {
        let mut s = Self {
            iface,
            baudrate: 0,
            specified_baudrate: 0,
            baudrate_method: BaudrateMethod::Auto,
            fixed_baudrate: 0,
            comm_line: 7,
            control: 0,
            stat: SERSTAT_RI,
            baudrate_pos: 0,
            slow_init_target: 0,
            slow_init_count: 0,
            si_state: SlowInit::Normal,
            ti_set_time: 0,
            ri_set_time: 0,
            read_after_write: true,
            enable_echo: true,
            echo_back_on_time: 0,
            ti_set_byte: 0,
            force_baud_rate: false,
            bitbang_reads_after_slow_init: 0,
            forced_baud_rate_no: 0,
            last_slow_init: 0,
            serial_bitbang_enabled: false,
            serial_bitbang_bit_pos: 0,
            serial_bitbang_last: 0,
            serial_bitbang_last_bit_sent: 0,
            rx_buf: Ring::default(),
        };
        s.set_echo(expect_echo);
        s
    }

    pub fn reset(&mut self) {
        self.ti_set_time = 0;
        self.ti_set_byte = 0;
        self.ri_set_time = 0;
        self.last_slow_init = 0;
    }

    pub fn echo(&self) -> bool {
        self.enable_echo
    }

    pub fn set_echo(&mut self, echo: bool) {
        self.enable_echo = echo;
    }

    pub fn baud_divisor(&self) -> u16 {
        self.baudrate & 0x7fff
    }

    pub fn set_baudrate(&mut self, now: u64, divisor: u8, clock: u32) {
        self.force_baud_rate = false;
        self.bitbang_reads_after_slow_init = 0;
        if self.baudrate_pos == 0 {
            self.baudrate = (self.baudrate & 0xff00) | divisor as u16;
            self.baudrate_pos += 1;
        } else {
            self.baudrate = ((divisor as u16) << 8) | (self.baudrate & 0xff);
            self.specified_baudrate = self.baudrate;
            self.baudrate_pos -= 1;
            if self.baudrate_method == BaudrateMethod::Force {
                // Cpu::serBaudToDiv
                self.baudrate = ((clock / 2 / self.fixed_baudrate as u32 / 8) | 0x8000) as u16;
            }
            self.iface
                .set_baud_divisor(now, (self.baudrate & 0x7fff) as u32);
        }
    }

    pub fn set_control(&mut self, now: u64, val: u8) {
        self.control = val;
        self.si_state = SlowInit::Normal;
        if val & 8 == 0 {
            if val == 0x01 {
                self.set_echo(false);
                self.echo_back_on_time = now + 10_000_000;
            } else {
                self.serial_bitbang_bit_pos = 0;
                self.serial_bitbang_enabled = true;
            }
        } else {
            self.serial_bitbang_enabled = false;
            if now >= self.echo_back_on_time {
                self.set_echo(true);
            }
        }
        self.iface.set_rx_bitbang(now, self.serial_bitbang_enabled);
        self.read_after_write = true;
    }

    pub fn set_stat(&mut self, val: u8) {
        self.stat = val;
    }

    fn check_input(&mut self, now: u64) {
        let buf = &mut self.rx_buf;
        self.iface.poll_input(now, &mut |b| buf.add(b));
    }

    pub fn read_stat(&mut self, now: u64) -> u8 {
        if self.ti_set_time != 0 && now >= self.ti_set_time {
            self.ti_set_time = 0;
            self.stat |= SERSTAT_TI;
        }
        let mut ret = self.stat;
        self.stat &= !(SERSTAT_OE | SERSTAT_FE | SERSTAT_TI | SERSTAT_RI | SERSTAT_RPERB8);
        if self.si_state == SlowInit::Spam {
            ret |= SERSTAT_RI;
        }
        if self.ti_set_time == 0 {
            self.stat |= SERSTAT_TXE;
            self.check_input(now);
        }
        if !self.rx_buf.is_empty() && now >= self.ri_set_time {
            ret |= SERSTAT_RI;
        }
        ret
    }

    pub fn read_control(&self) -> u8 {
        self.control
    }

    /// Upstream returns `int` -1 for an empty buffer, truncated to 0xFF.
    pub fn read_data(&mut self) -> u8 {
        let ret = if self.rx_buf.is_empty() {
            0xff
        } else {
            self.rx_buf.consume()
        };
        self.read_after_write = true;
        ret
    }

    pub fn write_data(&mut self, now: u64, data: u8) {
        if !self.read_after_write && self.enable_echo {
            self.set_echo(false);
            if self.ti_set_time == 0
                && !self.rx_buf.is_empty()
                && self.rx_buf.snoop() == self.ti_set_byte
            {
                self.rx_buf.consume();
            }
        }
        self.read_after_write = false;
        self.ti_set_time = now + 8 * (self.baudrate & 0x7fff) as u64 * 10;
        self.ri_set_time = if self.enable_echo {
            self.ti_set_time
        } else {
            self.ti_set_time + 7000
        };
        self.ti_set_byte = data;
        if !self.rx_buf.is_empty() {
            self.rx_buf.flush();
        }
        self.stat &= !(SERSTAT_TXE | SERSTAT_TI);
        self.iface.send_byte(now, data);
        if self.enable_echo {
            self.rx_buf.prepend(self.ti_set_byte);
        }
    }

    pub fn slow_init(&mut self, now: u64, bit: u8) {
        self.set_echo(true);
        self.rx_buf.flush();
        let since = now.wrapping_sub(self.last_slow_init);
        if self.baudrate_method == BaudrateMethod::AutoPlus
            && (self.force_baud_rate || self.bitbang_reads_after_slow_init > 100_000)
            && since > 1000 * 10000
        {
            self.force_baud_rate = true;
            self.baudrate = FORCED_BAUD_RATES[self.forced_baud_rate_no];
            self.iface
                .set_baud_divisor(now, (self.baudrate & 0x7fff) as u32);
            self.forced_baud_rate_no = (self.forced_baud_rate_no + 1) % FORCED_BAUD_RATES.len();
        }
        self.bitbang_reads_after_slow_init = 0;
        self.last_slow_init = now;
        if self.iface.send_slow_init_bitwise(now, bit != 0) {
            return;
        }
        if self.si_state != SlowInit::Iso {
            if bit != 0 {
                return;
            }
            self.si_state = SlowInit::Iso;
            self.slow_init_count = 0;
            self.slow_init_target = 0;
            self.iface.slow_init_imminent(now);
        } else if self.slow_init_count < 8 {
            self.slow_init_target = (self.slow_init_target >> 1) | if bit != 0 { 0x80 } else { 0 };
            self.slow_init_count += 1;
        } else {
            if bit == 0 {
                self.si_state = SlowInit::Normal;
                return;
            }
            self.rx_buf.flush();
            self.iface.send_slow_init(now, self.slow_init_target);
            self.si_state = SlowInit::Normal;
        }
    }

    pub fn rx_state(&mut self, now: u64) -> R<u8> {
        self.bitbang_reads_after_slow_init += 1;
        if let Some(bit) = self.iface.rx_state(now) {
            return Ok(bit as u8);
        }
        self.check_input(now);
        if !self.serial_bitbang_enabled || self.rx_buf.is_empty() {
            return Ok(1);
        }
        if now.wrapping_sub(self.serial_bitbang_last) > self.baud_divisor() as u64 * 8 {
            match self.serial_bitbang_bit_pos {
                0 => self.serial_bitbang_last_bit_sent = 0,
                1..=8 => {
                    let byte = self.rx_buf.snoop();
                    self.serial_bitbang_last_bit_sent =
                        (byte >> (self.serial_bitbang_bit_pos - 1)) & 1;
                }
                9 => self.serial_bitbang_last_bit_sent = 1,
                _ => return unimp("serial bitbang internal error"),
            }
            self.serial_bitbang_bit_pos += 1;
            self.serial_bitbang_last = now;
        }
        if self.serial_bitbang_bit_pos == 10 {
            self.serial_bitbang_bit_pos = 0;
            if self.rx_buf.is_empty() {
                return unimp("skipByte: none in buffer");
            }
            self.rx_buf.consume();
        }
        Ok(self.serial_bitbang_last_bit_sent)
    }

    pub fn set_comm_line(&mut self, now: u64, line: i32) {
        if line != self.comm_line {
            self.check_input(now);
            self.rx_buf.flush();
        }
        if line != self.comm_line && (line == 42 || self.comm_line == 42) {
            self.iface.set_can(now, line == 42);
        }
        self.comm_line = line;
    }

    pub fn comm_line(&self) -> i32 {
        self.comm_line
    }

    pub fn set_l(&mut self, now: u64, bit: u8) {
        self.iface.set_l(now, bit != 0);
    }
}
