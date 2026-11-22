//! The C167CR's on-chip CAN controller (at 0xEF00-0xEFFF, an Intel
//! 82527-style "full CAN" module with 15 message objects), enough for the
//! SIMK4x engine code: it sets up receive and transmit objects, polls them
//! and the status register, and requests transmissions. No CAN interrupts
//! (no dump tried enables XP0IC).
//!
//! Registers (word offsets from 0xEF00):
//! - `00` control (low byte: INIT, IE, SIE, EIE, CCE) and status (high
//!   byte: LEC 2..0, TXOK 3, RXOK 4, EWRN 6, BOFF 7; EWRN/BOFF read-only)
//! - `02` interrupt/port control, `04` bit timing, `06` global mask short,
//!   `08`/`0A` global mask long, `0C`/`0E` mask of the last message
//! - object n (1..=15) at `n*0x10`: `+0` message control, `+2`/`+4`
//!   arbitration, `+6` configuration (DLC << 4, DIR bit 3, XTD bit 2),
//!   `+7..=+E` data
//!
//! Message control is eight two-bit fields; a field reads `10` (set) or
//! `01` (reset). Writing `10` sets it, `01` resets it, `11`/`00` leave it.
//! Fields, from bit 0: INTPND, RXIE, TXIE, MSGVAL, NEWDAT, MSGLST (receive)
//! or CPUUPD (transmit), TXRQ, RMTPND.
//!
//! The bus is outside: transmitted frames collect in [`Can::tx`], received
//! ones come in through [`Can::rx`]. Whether anyone acknowledges frames is
//! [`Can::acked`] (another node on the bus). Unacknowledged transmissions
//! stay pending with LEC 3 (acknowledge error) and raise the transmit error
//! counter to error-passive, which by the CAN rules does not lead to bus-off
//! [assumed: the chip follows ISO 11898 here].

use cascade_can::{EXTENDED, Frame};

pub const BASE: u32 = 0xEF00;

// message control fields (bit position of each pair)
const NEWDAT: u32 = 8;
const MSGVAL: u32 = 6;
const MSGLST_CPUUPD: u32 = 10;
const TXRQ: u32 = 12;
const INTPND: u32 = 0;
const RXIE: u32 = 2;
const TXIE: u32 = 4;

// status bits
const TXOK: u8 = 1 << 3;
const RXOK: u8 = 1 << 4;
const EWRN: u8 = 1 << 6;
const BOFF: u8 = 1 << 7;
const LEC_ACK: u8 = 3;

#[derive(Clone)]
pub struct Can {
    regs: [u8; 256],
    /// Frames sent, with the CPU cycle they went out; the bus owner drains
    /// them.
    pub tx: Vec<(u64, Frame)>,
    /// Another node acknowledges frames.
    pub acked: bool,
    tec: u16,
}

impl Default for Can {
    fn default() -> Self {
        let mut regs = [0u8; 256];
        regs[0] = 0x01; // INIT after reset
        // message control after reset: all fields read "reset" (01)
        for n in 1..=15 {
            regs[n * 0x10] = 0x55;
            regs[n * 0x10 + 1] = 0x55;
        }
        Self {
            regs,
            tx: Vec::new(),
            acked: false,
            tec: 0,
        }
    }
}

fn get(mcr: u16, field: u32) -> bool {
    (mcr >> field) & 3 == 2
}

fn put(mcr: &mut u16, field: u32, on: bool) {
    *mcr = (*mcr & !(3 << field)) | ((if on { 2 } else { 1 }) << field);
}

impl Can {
    pub fn contains(a: u32) -> bool {
        (BASE..BASE + 0x100).contains(&a)
    }

    fn mcr(&self, n: usize) -> u16 {
        u16::from_le_bytes([self.regs[n * 0x10], self.regs[n * 0x10 + 1]])
    }

    fn set_mcr(&mut self, n: usize, v: u16) {
        self.regs[n * 0x10..n * 0x10 + 2].copy_from_slice(&v.to_le_bytes());
    }

    fn word(&self, off: usize) -> u16 {
        u16::from_le_bytes([self.regs[off], self.regs[off + 1]])
    }

    fn initialising(&self) -> bool {
        self.regs[0] & 1 != 0
    }

    pub fn read16(&self, a: u32) -> u16 {
        self.word((a - BASE) as usize & 0xfe)
    }

    pub fn write16(&mut self, now: u64, a: u32, v: u16) {
        self.write8(now, a & !1, v as u8);
        self.write8(now, a | 1, (v >> 8) as u8);
    }

    pub fn write8(&mut self, now: u64, a: u32, v: u8) {
        let off = (a - BASE) as usize;
        match off {
            // status: EWRN and BOFF belong to the controller
            0x01 => self.regs[1] = (self.regs[1] & (EWRN | BOFF)) | (v & !(EWRN | BOFF)),
            0x00 => {
                let was_init = self.initialising();
                self.regs[0] = v;
                if was_init && !self.initialising() {
                    // leaving INIT: the controller rejoins the bus
                    self.regs[1] &= !BOFF;
                    for n in 1..=15 {
                        self.try_send(now, n);
                    }
                }
            }
            // message control: set/reset pairs
            o if o >= 0x10 && o & 0x0e == 0 => {
                let n = o >> 4;
                let shift = (o & 1) * 8;
                let mut mcr = self.mcr(n);
                for i in 0..4 {
                    let bits = (v >> (2 * i)) & 3;
                    let field = (shift + 2 * i) as u32;
                    if bits == 2 || bits == 1 {
                        mcr = (mcr & !(3 << field)) | ((bits as u16) << field);
                    }
                }
                self.set_mcr(n, mcr);
                self.try_send(now, n);
            }
            o => self.regs[o] = v,
        }
    }

    /// The identifier an object's arbitration registers hold.
    fn object_id(&self, n: usize) -> u32 {
        let (uar, lar) = (self.word(n * 0x10 + 2), self.word(n * 0x10 + 4));
        if self.regs[n * 0x10 + 6] & 0x04 != 0 {
            let id = ((uar as u32 & 0xff) << 21)
                | ((uar as u32 >> 8) << 13)
                | ((lar as u32 & 0xff) << 5)
                | (lar as u32 >> 11);
            id | EXTENDED
        } else {
            ((uar as u32 & 0xff) << 3) | (uar as u32 >> 13)
        }
    }

    fn mask(&self, n: usize, ext: bool) -> u32 {
        let (hi, lo) = if ext {
            (self.word(0x08), self.word(0x0a))
        } else {
            (self.word(0x06), 0)
        };
        let m = if ext {
            ((hi as u32 & 0xff) << 21)
                | ((hi as u32 >> 8) << 13)
                | ((lo as u32 & 0xff) << 5)
                | (lo as u32 >> 11)
        } else {
            ((hi as u32 & 0xff) << 3) | (hi as u32 >> 13)
        };
        if n == 15 {
            // object 15: global mask and the mask of the last message
            let (h, l) = (self.word(0x0c), self.word(0x0e));
            let m15 = ((h as u32 & 0xff) << 21)
                | ((h as u32 >> 8) << 13)
                | ((l as u32 & 0xff) << 5)
                | (l as u32 >> 11);
            m & if ext { m15 } else { m15 >> 18 }
        } else {
            m
        }
    }

    /// Send object `n` if it asks to and the controller is on the bus.
    fn try_send(&mut self, now: u64, n: usize) {
        let mut mcr = self.mcr(n);
        let cfg = self.regs[n * 0x10 + 6];
        if self.initialising()
            || !get(mcr, MSGVAL)
            || !get(mcr, TXRQ)
            || get(mcr, MSGLST_CPUUPD)
            || cfg & 0x08 == 0
        {
            return;
        }
        if !self.acked {
            // nobody acknowledges: the frame stays pending
            self.regs[1] = (self.regs[1] & !7) | LEC_ACK;
            if self.regs[1] & EWRN == 0 {
                self.tec = (self.tec + 8).min(128);
                if self.tec >= 96 {
                    self.regs[1] |= EWRN;
                }
            }
            return;
        }
        let dlc = (cfg >> 4).min(8) as usize;
        let data = self.regs[n * 0x10 + 7..n * 0x10 + 7 + dlc].to_vec();
        self.tx.push((
            now,
            Frame {
                id: self.object_id(n),
                data,
            },
        ));
        put(&mut mcr, TXRQ, false);
        if get(mcr, TXIE) {
            put(&mut mcr, INTPND, true);
        }
        self.set_mcr(n, mcr);
        self.tec = self.tec.saturating_sub(1);
        if self.tec < 96 {
            self.regs[1] &= !EWRN;
        }
        self.regs[1] = (self.regs[1] & !7) | TXOK;
    }

    /// Another node acknowledges frames from now on (or no longer):
    /// pending transmissions go out.
    pub fn set_acked(&mut self, now: u64, acked: bool) {
        self.acked = acked;
        for n in 1..=15 {
            self.try_send(now, n);
        }
    }

    /// A frame from the bus. Returns whether an object took it.
    pub fn rx(&mut self, f: &Frame) -> bool {
        if self.initialising() {
            return false;
        }
        let ext = f.id & EXTENDED != 0;
        for n in 1..=15 {
            let mcr = self.mcr(n);
            let cfg = self.regs[n * 0x10 + 6];
            if !get(mcr, MSGVAL) || cfg & 0x08 != 0 || (cfg & 0x04 != 0) != ext {
                continue;
            }
            let m = self.mask(n, ext);
            if (self.object_id(n) ^ f.id) & m & !EXTENDED != 0 {
                continue;
            }
            let mut mcr = mcr;
            let dlc = f.data.len().min(8);
            self.regs[n * 0x10 + 6] = (dlc as u8) << 4 | (cfg & 0x0f);
            self.regs[n * 0x10 + 7..n * 0x10 + 7 + dlc].copy_from_slice(&f.data[..dlc]);
            let lost = get(mcr, NEWDAT);
            put(&mut mcr, MSGLST_CPUUPD, lost);
            put(&mut mcr, NEWDAT, true);
            if get(mcr, RXIE) {
                put(&mut mcr, INTPND, true);
            }
            self.set_mcr(n, mcr);
            self.regs[1] = (self.regs[1] & !7) | RXOK;
            return true;
        }
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Object `n` set up as the SIMK4x code does it.
    fn object(c: &mut Can, n: u32, uar: u16, cfg: u8, mcr: u16) {
        let b = BASE + n * 0x10;
        c.write16(0, b, 0x5595); // valid, everything else reset
        c.write16(0, b + 2, uar);
        c.write16(0, b + 4, 0xffff);
        c.write8(0, b + 6, cfg);
        c.write16(0, b, mcr);
    }

    fn start(c: &mut Can) {
        c.write16(0, BASE, 0x0741); // INIT, CCE
        c.write16(0, BASE + 6, 0xffff); // exact match
        object(c, 1, 0xe087, 0x80, 0x5595); // receive 43F, 8 bytes
        object(c, 14, 0xc062, 0x88, 0x5595); // transmit 316, 8 bytes
        c.write16(0, BASE, 0x0700); // leave INIT
    }

    #[test]
    fn message_control_pairs() {
        let mut c = Can::default();
        start(&mut c);
        let a = BASE + 0x10;
        assert_eq!(c.read16(a), 0x5595);
        c.write16(0, a, 0xfeff); // NEWDAT set (10), others unchanged (11)
        assert_eq!(c.read16(a) & 0x300, 0x200);
        assert_eq!(c.read16(a) & 0xc0, 0x80, "still valid");
        c.write8(0, a + 1, 0xfd); // byte write: NEWDAT reset
        assert_eq!(c.read16(a) & 0x300, 0x100);
    }

    #[test]
    fn transmit_when_acknowledged() {
        let mut c = Can::default();
        start(&mut c);
        let o = BASE + 0xe0;
        c.write16(0, o + 6, 0x0588); // cfg 88, data0 05
        c.write16(0, o + 8, 0x845c);
        c.write16(0, o, 0x6595); // TXRQ
        assert!(c.tx.is_empty(), "no acknowledgement: pending");
        assert_eq!(c.read16(BASE) >> 8 & 7, 3, "LEC acknowledge error");
        assert_eq!(c.read16(o) & 0x3000, 0x2000);
        c.set_acked(10, true);
        assert_eq!(c.tx.len(), 1);
        let (at, f) = &c.tx[0];
        assert_eq!((*at, f.id), (10, 0x316));
        assert_eq!(&f.data[..4], &[0x05, 0x5c, 0x84, 0x00]);
        assert_eq!(f.data.len(), 8);
        assert_eq!(c.read16(o) & 0x3000, 0x1000, "TXRQ reset");
        assert_ne!(c.read16(BASE) & (TXOK as u16) << 8, 0);
    }

    #[test]
    fn receive_by_exact_id() {
        let mut c = Can::default();
        start(&mut c);
        assert!(!c.rx(&Frame::new(0x43e, &[1])));
        assert!(c.rx(&Frame::new(0x43f, &[1, 2, 3])));
        let o = BASE + 0x10;
        assert_eq!(c.read16(o) & 0x300, 0x200, "NEWDAT");
        assert_eq!(c.read16(o + 6), 0x0130, "DLC 3, first byte 01");
        assert!(c.rx(&Frame::new(0x43f, &[9])));
        assert_eq!(c.read16(o) & 0xc00, 0x800, "MSGLST: not read in time");
        assert_ne!(c.read16(BASE) & (RXOK as u16) << 8, 0);
    }

    #[test]
    fn nothing_while_initialising() {
        let mut c = Can {
            acked: true,
            ..Default::default()
        };
        c.write16(0, BASE, 0x0741);
        object(&mut c, 14, 0xc062, 0x88, 0x6595);
        assert!(c.tx.is_empty());
        assert!(!c.rx(&Frame::new(0x316, &[0])));
        c.write16(5, BASE, 0x0700);
        assert_eq!(c.tx.len(), 1, "sent on leaving INIT");
    }
}
