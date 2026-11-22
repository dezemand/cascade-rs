//! What the ECU itself holds, read from its RAM: its stored fault codes and
//! a few of its own measurements. Addresses are from the non-immobiliser
//! GK 2.0 dump, found by following its `18` (ReadDTCsByStatus) handler
//! (emulator-derived); with another dump the values may be wrong.
//!
//! Fault memory: `C438` holds the number of stored codes and `CA7A..` the
//! fault path of each. Path `p` has a 14-byte record at `C43A + 14 p`; its
//! first byte is the status the ECU reports over KWP (bit 6 set while the
//! fault is present) and its low nibble picks the code variant: 1, 2 or
//! other select the word at offset 0, 2 or 4 of the path's row in a ROM
//! table at data address 0x46C4 (flash 0x506C4), rows 0x16 bytes apart.

use ecuc167::cpu::Cpu;

const COUNT: u32 = 0xC438;
const PATHS: u32 = 0xCA7A;
const RECORDS: u32 = 0xC43A;
const RECORD_LEN: u32 = 14;
const CODE_TABLE: u32 = 0x506C4;
const CODE_ROW: u32 = 0x16;

/// One stored fault code.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Stored {
    /// "P0118" etc.
    pub code: String,
    /// The status byte the ECU reports with it.
    pub status: u8,
}

impl Stored {
    /// The fault is present now (not only remembered).
    pub fn present(&self) -> bool {
        self.status & 0x40 != 0
    }
}

/// The ECU's stored fault codes, in its own order.
pub fn stored_codes(cpu: &Cpu) -> Vec<Stored> {
    let n = cpu.peek8(COUNT) as u32;
    if n > 32 {
        return Vec::new(); // not this dump's layout
    }
    (0..n)
        .map(|i| {
            let path = cpu.peek8(PATHS + i) as u32;
            let status = cpu.peek8(RECORDS + RECORD_LEN * path);
            let offset = match status & 0x0f {
                1 => 0,
                2 => 2,
                _ => 4,
            };
            let raw = cpu.peek16(CODE_TABLE + CODE_ROW * path + offset);
            Stored {
                code: dtc_name(raw),
                status,
            }
        })
        .collect()
}

/// SAE code text for a two-byte DTC (top two bits are the letter).
pub fn dtc_name(raw: u16) -> String {
    let letter = ['P', 'C', 'B', 'U'][(raw >> 14) as usize];
    format!("{letter}{:04X}", raw & 0x3fff)
}

/// The ECU's own measurements (the values behind its `21 01` block):
/// (engine speed rpm, road speed km/h, coolant °C, battery V).
pub fn readings(cpu: &Cpu) -> (u16, u8, f64, f64) {
    (
        cpu.peek16(0xE354),
        cpu.peek8(0xCBB6),
        cpu.peek8(0xCB9C) as f64 * 0.75 - 48.0,
        cpu.peek8(0xF8A4) as f64 * 0.101_562_5,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn code_names() {
        assert_eq!(dtc_name(0x0118), "P0118");
        assert_eq!(dtc_name(0x4200), "C0200");
        assert_eq!(dtc_name(0x9346), "B1346");
    }
}
