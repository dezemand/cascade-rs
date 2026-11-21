//! Intel 8XC196 core as emulated by upstream CASCADE (`cpu.h`, `cpu.cpp`,
//! `cpu_emu.cpp`, `cpu_io.cpp`).
//!
//! The port follows the oracle's **debug** build, which is what the
//! reference traces come from. The visible difference from a release build is
//! that interrupts and the event pump are checked before every instruction
//! instead of every tenth.
//!
//! Upstream quirks are kept deliberately (the oracle defines correctness),
//! for example: the V flag is never set by add/sub (`(x & 0x8000) == 1`),
//! INC never sets Z, short-indexed offsets are not sign-extended, and shift
//! counts taken from a register are masked to the host operand width.

mod exec;
mod io;

use std::collections::VecDeque;

use cascade_iface::{Interface, Null};

use crate::eeprom::Eeprom;
use crate::glibc_rand::GlibcRand;
use crate::hsio::Hsio;
use crate::keypad::{Key, Keypad};
use crate::lcd::Lcd;
use crate::serial::Serial;
use crate::stop::{R, Stop, oob, unimp};
use crate::trace::Record;

pub const PSW_ST: u8 = 1 << 0;
pub const PSW_INTE: u8 = 1 << 1;
pub const PSW_PTSE: u8 = 1 << 2;
pub const PSW_C: u8 = 1 << 3;
pub const PSW_VT: u8 = 1 << 4;
pub const PSW_V: u8 = 1 << 5;
pub const PSW_N: u8 = 1 << 6;
pub const PSW_Z: u8 = 1 << 7;

const RAM_SIZE: usize = 0xc000;
const MAPPED_RAM_SIZE: usize = 524_288;
const MAPPED_RAM_BASE: u32 = 0xcaf0_0000;
const EXROM_BASE: u32 = 0xbab0_0000;

/// Where the 16 KB window at 0xC000 currently points.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Window {
    /// Offset into the ROM image.
    Rom(u32),
    /// Offset into the 512 KB mapped RAM.
    MappedRam(u32),
}

/// Something that happened during a step that the caller may care about.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StepEvent {
    /// Instruction executed normally.
    Executed,
    /// `SJMP $` detected; upstream shows "Endless loop, resetting." and
    /// resets the machine (including the cycle counter). `pc` is the loop.
    EndlessLoopReset { pc: u16, phys: u32, cycles: u64 },
}

/// A key press or release scheduled at an emulated cycle. It takes effect at
/// the first event pump (every 131072 cycles) at or after `at`, as upstream
/// applies UI and replayed key events there.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct KeyEvent {
    pub at: u64,
    pub key: Key,
    pub down: bool,
}

/// Counters for coverage reporting; they do not affect emulation.
#[derive(Clone, Debug)]
pub struct Stats {
    /// Executions per first opcode byte.
    pub opcodes: [u64; 256],
    /// Executions per FE-prefixed second byte.
    pub fe_opcodes: [u64; 256],
    /// HSO software-timer interrupts taken.
    pub swt_interrupts: u64,
    /// Reads of the `rand()`-backed ports 0x201 and 0x236.
    pub rand_reads: u64,
    /// Bytes written to the UART (SBUF TX).
    pub uart_tx: u64,
}

impl Default for Stats {
    fn default() -> Self {
        Self {
            opcodes: [0; 256],
            fe_opcodes: [0; 256],
            swt_interrupts: 0,
            rand_reads: 0,
            uart_tx: 0,
        }
    }
}

pub struct Cpu {
    pub stats: Stats,
    rom: Vec<u8>,
    ram: Vec<u8>,
    mapped_ram: Vec<u8>,
    code_win: u32,
    data_win: Window,
    pub is_hiscan: bool,

    pc: u16,
    psw: u8,
    code_hi: u8,
    code_lo: u8,
    data_hi: u8,
    data_lo: u8,
    wsr: u8,
    wsr1: u8,
    int_mask: u8,
    int_mask1: u8,
    ptssel: u16,
    ptssrv: u16,
    ad_command: u8,
    ad_result: u16,

    cycles: u64,
    oldcycles: u64,
    pub max_cycles: u64,
    clock: u32,

    ioc0: u8,
    ioc1: u8,
    ios0: u8,
    ios1: u8,
    last_ios1_read: u16,
    ioport1: u8,
    ioport2: u8,
    timer1_offset: u16,
    timer2: u32,
    timer2_inc_factor: i32,

    next_event_pumping: u64,

    pub hsio: Hsio,
    pub lcd: Lcd,
    pub eeprom: Eeprom,
    pub keypad: Keypad,
    pub serial: Serial,
    rand: GlibcRand,
    /// `extint_pos` is a file-static upstream and survives `reset()`.
    extint_pos: usize,
    keys: VecDeque<KeyEvent>,
}

impl Cpu {
    /// Build a machine for `rom` with the given interface, as upstream's
    /// `Cpu::Cpu`, `loadRom` and `setSerial` do in `main()`.
    pub fn new(rom: Vec<u8>, iface: Box<dyn Interface>, expect_echo: bool) -> Self {
        let mut ram = vec![0u8; RAM_SIZE];
        let n = rom.len().min(RAM_SIZE);
        ram[..n].copy_from_slice(&rom[..n]);
        let is_hiscan = !upstream_contains(&rom, b"CARMAN");
        let mut cpu = Self {
            stats: Stats::default(),
            rom,
            ram,
            mapped_ram: vec![0; MAPPED_RAM_SIZE],
            code_win: 0,
            data_win: Window::Rom(0),
            is_hiscan,
            pc: 0,
            psw: 0,
            code_hi: 0,
            code_lo: 0,
            data_hi: 0,
            data_lo: 0,
            wsr: 0,
            wsr1: 0,
            int_mask: 0,
            int_mask1: 0,
            ptssel: 0,
            ptssrv: 0,
            ad_command: 0,
            ad_result: 0,
            cycles: 0,
            oldcycles: 0,
            max_cycles: u64::MAX,
            clock: 20_000_000,
            ioc0: 0,
            ioc1: 0,
            ios0: 0,
            ios1: 0,
            last_ios1_read: 0,
            ioport1: 0,
            ioport2: 0,
            timer1_offset: 0,
            timer2: 0,
            timer2_inc_factor: 1,
            next_event_pumping: 0,
            hsio: Hsio::default(),
            lcd: Lcd::default(),
            eeprom: Eeprom::default(),
            keypad: Keypad::default(),
            serial: Serial::new(iface, expect_echo),
            rand: GlibcRand::default(),
            extint_pos: 0,
            keys: VecDeque::new(),
        };
        cpu.reset();
        cpu
    }

    /// A machine with nothing on the diagnostic line.
    pub fn with_null_interface(rom: Vec<u8>) -> Self {
        Self::new(rom, Box::new(Null), false)
    }

    /// Upstream `Cpu::reset`. RAM, mapped RAM, keypad, EEPROM and HSIO keep
    /// their contents, as upstream.
    pub fn reset(&mut self) {
        self.pc = 0x2080;
        self.psw = 0;
        self.cycles = 0;
        self.int_mask = 0;
        self.int_mask1 = 0;
        self.ios0 = 0;
        self.ios1 = 0;
        self.ioc0 = 0;
        self.ioc1 = 0;
        self.last_ios1_read = 0;
        self.clock = 20_000_000;
        self.timer1_offset = 0;
        self.timer2_inc_factor = 1;
        self.timer2 = 0;
        self.ad_command = 0;
        self.ad_result = 0xff00;
        self.code_lo = 0;
        self.code_hi = 0;
        self.data_lo = 0;
        self.data_hi = 0;
        self.code_win = 0;
        self.data_win = Window::Rom(0);
        self.wsr = 0;
        self.wsr1 = 0;
        self.ioport1 = 0;
        self.ioport2 = 0;
        self.ptssrv = 0;
        self.ptssel = 0;
        self.oldcycles = self.cycles;
        self.next_event_pumping = 0;
        self.lcd.reset();
        self.serial.reset();
    }

    pub fn cycles(&self) -> u64 {
        self.cycles
    }

    pub fn pc(&self) -> u16 {
        self.pc
    }

    pub fn psw(&self) -> u8 {
        self.psw
    }

    /// Upstream's `dump` file: the 48 KB of internal address space.
    pub fn ram(&self) -> &[u8] {
        &self.ram
    }

    /// Upstream's `dump.ram` file: the 512 KB of banked RAM.
    pub fn mapped_ram(&self) -> &[u8] {
        &self.mapped_ram
    }

    /// Queue a key press or release (see [`KeyEvent`]).
    pub fn queue_key(&mut self, ev: KeyEvent) {
        let pos = self
            .keys
            .iter()
            .position(|k| k.at > ev.at)
            .unwrap_or(self.keys.len());
        self.keys.insert(pos, ev);
    }

    // ---- address translation -------------------------------------------

    /// Upstream `virtToPhysSlow` for an address in 0xC000..=0xFFFF.
    fn virt_to_phys_slow(&self, addr: u16, fetch: bool) -> R<u32> {
        let (lo, hi) = if fetch {
            (self.code_lo, self.code_hi)
        } else {
            (self.data_lo, self.data_hi)
        };
        let off = addr as i64 - 0xc000;
        let lo_i = lo as i64;
        let p: i64 = match hi {
            9 => 0xc000 + (lo_i - 6) * 0x4000 + off,
            0 => lo_i * 0x4000 + off,
            1..=6 => 0x40_0000 * hi as i64 + lo_i * 0x4000 + off,
            8 if lo <= 0x1f => MAPPED_RAM_BASE as i64 + lo_i * 0x4000 + off,
            // no extended ROM is loaded, so map_hi 0x10 falls back to plain ROM
            0x10 => lo_i * 0x4000 + off,
            7 | 0x1e | 0x1f => MAPPED_RAM_BASE as i64 + (lo_i % 0x20) * 0x4000 + off,
            _ => return unimp(format!("memory mapping {hi:02X}/{lo:02X}")),
        };
        Ok(p as u32)
    }

    fn virt_to_phys(&self, addr: u16, fetch: bool) -> u32 {
        if addr < 0xc000 {
            addr as u32
        } else {
            // A mapping that cannot be translated would already have stopped
            // the machine when it was written.
            self.virt_to_phys_slow(addr, fetch).unwrap_or(0)
        }
    }

    // ---- memory ------------------------------------------------------------

    fn window_read(&self, addr: u16) -> R<u8> {
        let off = (addr - 0xc000) as usize;
        match self.data_win {
            Window::Rom(base) => match self.rom.get(base as usize + off) {
                Some(b) => Ok(*b),
                None => oob(format!(
                    "data window read past ROM at {:#x}",
                    base as usize + off
                )),
            },
            Window::MappedRam(base) => match self.mapped_ram.get(base as usize + off) {
                Some(b) => Ok(*b),
                None => oob(format!(
                    "data window read past mapped RAM at {:#x}",
                    base as usize + off
                )),
            },
        }
    }

    fn window_write(&mut self, addr: u16, v: u8) -> R<()> {
        let off = (addr - 0xc000) as usize;
        // Upstream writes through data_ptr even when it points at the ROM image.
        let (buf, i) = match self.data_win {
            Window::Rom(base) => (&mut self.rom, base as usize + off),
            Window::MappedRam(base) => (&mut self.mapped_ram, base as usize + off),
        };
        match buf.get_mut(i) {
            Some(b) => {
                *b = v;
                Ok(())
            }
            None => oob(format!("data window write past buffer at {i:#x}")),
        }
    }

    /// Upstream `memRead8Bus` (debug build: extra I/O ranges included).
    fn read8_bus(&mut self, addr: u16, fetch: bool) -> R<u8> {
        if addr >= 0xc000 {
            return self.mapped_read8(addr, fetch);
        }
        if addr < 0x18
            || (0x1f00..0x2000).contains(&addr)
            || (0x2014..0x2030).contains(&addr)
            || (0x205e..0x2080).contains(&addr)
            || (0x200..0x2ff).contains(&addr)
        {
            self.io_read8(addr)
        } else {
            Ok(self.ram[addr as usize])
        }
    }

    /// Upstream `mappedRead8` (only reachable from the disassembler there).
    fn mapped_read8(&self, addr: u16, fetch: bool) -> R<u8> {
        let p = self.virt_to_phys_slow(addr, fetch)?;
        let got = if p >= MAPPED_RAM_BASE {
            self.mapped_ram.get((p - MAPPED_RAM_BASE) as usize)
        } else if p >= EXROM_BASE {
            None
        } else {
            self.rom.get(p as usize)
        };
        match got {
            Some(b) => Ok(*b),
            None => oob(format!("mapped read at phys {p:#x}")),
        }
    }

    /// Upstream `memRead8`.
    fn rd8(&mut self, addr: u16) -> R<u8> {
        match addr {
            0..=1 => Ok(0),
            0x18..=0xff | 0x2000..=0xbfff => Ok(self.ram[addr as usize]),
            0xc000..=0xffff => self.window_read(addr),
            _ => self.read8_bus(addr, false),
        }
    }

    fn rd16(&mut self, addr: u16) -> R<u16> {
        let lo = self.rd8(addr)? as u16;
        let hi = self.rd8(addr.wrapping_add(1))? as u16;
        Ok(lo | (hi << 8))
    }

    fn rd32(&mut self, addr: u16) -> R<u32> {
        let lo = self.rd16(addr)? as u32;
        let hi = self.rd16(addr.wrapping_add(2))? as u32;
        Ok(lo | (hi << 16))
    }

    /// Upstream `memWrite8`.
    fn wr8(&mut self, addr: u16, v: u8) -> R<()> {
        match addr {
            0..=0x17 | 0x200..=0x2ff => self.io_write8(addr, v),
            0x18..=0xff | 0x2000..=0xbfff => {
                self.ram[addr as usize] = v;
                Ok(())
            }
            0xc000..=0xffff => self.window_write(addr, v),
            // memWrite8Slow: virtToPhys(addr) == addr below 0xC000
            _ => {
                self.ram[addr as usize] = v;
                Ok(())
            }
        }
    }

    fn wr16(&mut self, addr: u16, v: u16) -> R<()> {
        self.wr8(addr, v as u8)?;
        self.wr8(addr.wrapping_add(1), (v >> 8) as u8)
    }

    fn wr32(&mut self, addr: u16, v: u32) -> R<()> {
        self.wr16(addr, v as u16)?;
        self.wr16(addr.wrapping_add(2), (v >> 16) as u16)
    }

    fn fetch(&mut self) -> R<u8> {
        let pc = self.pc;
        self.pc = pc.wrapping_add(1);
        if pc >= 0xc000 {
            let i = self.code_win as usize + (pc - 0xc000) as usize;
            match self.rom.get(i) {
                Some(b) => Ok(*b),
                None => oob(format!("code fetch past ROM at {i:#x}")),
            }
        } else {
            self.read8_bus(pc, true)
        }
    }

    fn fetch16(&mut self) -> R<u16> {
        let lo = self.fetch()? as u16;
        let hi = self.fetch()? as u16;
        Ok(lo | (hi << 8))
    }

    fn push16(&mut self, w: u16) -> R<()> {
        let sp = self.rd16(0x18)?.wrapping_sub(2);
        self.wr16(0x18, sp)?;
        self.wr16(sp, w)
    }

    fn pop16(&mut self) -> R<u16> {
        let sp = self.rd16(0x18)?;
        let v = self.rd16(sp)?;
        self.wr16(0x18, sp.wrapping_add(2))?;
        Ok(v)
    }

    /// Upstream `indexedAddr`. The short-index offset is *not* sign-extended.
    fn indexed_addr(&mut self) -> R<u16> {
        let imm8 = self.fetch()?;
        if imm8 & 1 != 0 {
            let disp = self.fetch16()?;
            let base = self.rd16((imm8 & 0xfe) as u16)?;
            self.cycle(1);
            Ok(disp.wrapping_add(base))
        } else {
            let disp = self.fetch()? as u16;
            let base = self.rd16(imm8 as u16)?;
            Ok(disp.wrapping_add(base))
        }
    }

    // ---- timing ------------------------------------------------------------

    fn cycle(&mut self, c: u64) {
        self.cycles += c;
    }

    fn cycle_shift(&mut self, c: u64, shift: u8) {
        self.cycles += c + if shift != 0 { shift as u64 } else { 1 };
    }

    fn cycle_rm3(&mut self, c: u64, addr: u16) {
        self.cycles += c + if addr >= 0x200 { 3 } else { 0 };
    }

    fn cycle_rm2(&mut self, c: u64, addr: u16) {
        self.cycles += c + if addr >= 0x200 { 2 } else { 0 };
    }

    fn timer1(&self) -> u16 {
        ((self.cycles / 8) as u16).wrapping_add(self.timer1_offset)
    }

    // ---- main loop ---------------------------------------------------------

    /// Machine state as the oracle prints it before an instruction.
    pub fn trace_record(&self) -> Record {
        let w = |a: usize| self.ram[a] as u16 | ((self.ram[a + 1] as u16) << 8);
        let mut words = [0u16; 11];
        for (dst, a) in words.iter_mut().zip(crate::trace::WORD_ADDRS) {
            *dst = w(a as usize);
        }
        Record {
            pc: self.pc,
            phys: self.virt_to_phys(self.pc, true),
            words,
            psw: self.psw,
            sp: w(0x18),
            r25e: self.ram[0x25e],
            disasm: String::new(),
        }
    }

    /// One iteration of upstream's `emulate()` loop (debug build): timers,
    /// cycle cap, SWT interrupts, event pump, then one instruction. If
    /// `trace` is given it receives the pre-instruction state, exactly where
    /// the oracle prints its trace lines.
    pub fn step(&mut self, trace: Option<&mut dyn FnMut(&Record)>) -> R<StepEvent> {
        let passed = self.cycles.wrapping_sub(self.oldcycles) as u32;
        self.timer2 = self
            .timer2
            .wrapping_add(passed.wrapping_mul(self.timer2_inc_factor as u32));
        self.oldcycles = self.cycles;

        if self.cycles > self.max_cycles {
            return Err(Stop::MaxCycles);
        }

        if self.int_mask & (1 << 5) != 0 && self.psw & PSW_INTE != 0 {
            for i in 0..4 {
                let old = ((self.cycles - passed as u64) / 8) as u16;
                let old = old.wrapping_add(self.timer1_offset);
                let new = self.timer1();
                if self.hsio.swt_interrupt(old, new, i) {
                    self.stats.swt_interrupts += 1;
                    self.ios1 |= 1 << i;
                    self.push16(self.pc)?;
                    self.pc = self.rd16(0x200a)?;
                }
            }
        }

        if self.cycles >= self.next_event_pumping {
            while let Some(k) = self.keys.front().copied() {
                if k.at > self.cycles {
                    break;
                }
                self.keys.pop_front();
                self.keypad.set(k.key, k.down);
            }
            self.next_event_pumping += 131_072;
        }

        if let Some(t) = trace {
            t(&self.trace_record());
        }

        self.execute()
    }

    /// Run until a [`Stop`]; returns it. Endless-loop resets are reported to
    /// `on_reset` and emulation continues, as upstream does once its warning
    /// dialog is dismissed.
    pub fn run(
        &mut self,
        mut trace: Option<&mut dyn FnMut(&Record)>,
        mut on_reset: impl FnMut(&StepEvent),
    ) -> Stop {
        loop {
            let t = trace.as_mut().map(|t| &mut **t as &mut dyn FnMut(&Record));
            match self.step(t) {
                Ok(StepEvent::Executed) => {}
                Ok(ev) => on_reset(&ev),
                Err(stop) => return stop,
            }
        }
    }
}

/// Upstream's naive "CARMAN" search in `loadRom` (no backtracking on a
/// partial match), reproduced exactly.
fn upstream_contains(hay: &[u8], needle: &[u8]) -> bool {
    let mut i = 0;
    for &b in hay {
        if b == needle[i] {
            i += 1;
            if i == needle.len() {
                return true;
            }
        } else {
            i = 0;
        }
    }
    false
}
