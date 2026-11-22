//! Parameter definitions (decoded from the protocol CSV files in `protocol/`).

/// The Tiburon 2003 Engine L4 Current Data block (`21 01`), embedded.
pub const TIBURON_2003_ENGINE_L4: &str =
    include_str!("../../../protocol/tiburon-2003-engine-l4-current-data.csv");

/// The Tiburon ABS Current Data block (`21 01` to 0x28), embedded.
pub const TIBURON_2003_ABS: &str =
    include_str!("../../../protocol/tiburon-2003-abs-current-data.csv");

/// The Tiburon traction control wheel speeds (`B1 0E` to 0x32), embedded.
pub const TIBURON_2003_TCS_WHEEL_SPEEDS: &str =
    include_str!("../../../protocol/tiburon-2003-tcs-wheel-speeds.csv");

/// The Tiburon traction control valve outputs (`B1 2C` to 0x32), embedded.
pub const TIBURON_2003_TCS_VALVES: &str =
    include_str!("../../../protocol/tiburon-2003-tcs-valves.csv");

/// Traction control (C) fault code texts, read off the Hi-Scan's Tiburon TCS
/// screen. Not the same table as ABS.
pub const TCS_DTC_NAMES: &str = include_str!("../../../protocol/dtc-names-tcs.csv");

/// SRS fault code texts as the Hi-Scan shows them on the Tiburon's airbag
/// screen (read off its display in the emulator), embedded.
pub const SRS_DTC_NAMES: &str = include_str!("../../../protocol/dtc-names-srs.csv");

/// Engine (P) fault code texts, read off the Hi-Scan's Tiburon engine screen.
pub const ENGINE_DTC_NAMES: &str = include_str!("../../../protocol/dtc-names-engine.csv");

/// ABS (C) fault code texts, read off the Hi-Scan's Tiburon ABS screen.
pub const ABS_DTC_NAMES: &str = include_str!("../../../protocol/dtc-names-abs.csv");

/// `code,text` CSV (`#` comments) into a lookup table ("B1346" -> text).
pub fn dtc_texts(csv: &str) -> std::collections::BTreeMap<String, String> {
    csv.lines()
        .filter(|l| !l.starts_with('#') && !l.starts_with("code,"))
        .filter_map(|l| {
            let (code, text) = l.split_once(',')?;
            Some((code.to_string(), text.trim_matches('"').to_string()))
        })
        .collect()
}

/// The SRS module's identification (`1A 80`), laid out as the Hi-Scan shows
/// it. Byte positions count the response from its SID (`5A` = 0); found by
/// answering the Hi-Scan with a pattern in the emulator.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SrsIdent {
    /// Bytes 1-6. The first is the option byte `80` itself.
    pub vehicle: String,
    /// Bytes 7-17.
    pub part_no: String,
    /// Bytes 18-37.
    pub part_name: String,
    /// Bytes 68 and 69, shown as "a.b".
    pub acu_version: Option<String>,
}

pub fn parse_srs_ident(resp: &[u8]) -> SrsIdent {
    let text = |a: usize, b: usize| -> String {
        resp.get(a..b.min(resp.len()))
            .unwrap_or(&[])
            .iter()
            .map(|&c| {
                if (0x20..0x7f).contains(&c) {
                    c as char
                } else {
                    '.'
                }
            })
            .collect::<String>()
            .trim_end()
            .to_string()
    };
    SrsIdent {
        vehicle: text(1, 7),
        part_no: text(7, 18),
        part_name: text(18, 38),
        acu_version: match (resp.get(68), resp.get(69)) {
            (Some(&a), Some(&b)) => Some(format!("{}.{}", a as char, b as char)),
            _ => None,
        },
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Kind {
    U8,
    U16Le,
    S16Le,
    Bit(u8),
}

#[derive(Clone, Debug, PartialEq)]
pub struct Param {
    pub name: String,
    pub offset: usize,
    pub kind: Kind,
    pub scale: f64,
    pub add: f64,
    pub unit: String,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Value {
    Number(f64),
    Flag(bool),
}

impl Param {
    /// Decode from a block (bytes after `61 id`); `None` if too short.
    pub fn decode(&self, block: &[u8]) -> Option<Value> {
        let b = |i: usize| block.get(self.offset + i).copied();
        Some(match self.kind {
            Kind::U8 => Value::Number(b(0)? as f64 * self.scale + self.add),
            Kind::U16Le => {
                Value::Number(u16::from_le_bytes([b(0)?, b(1)?]) as f64 * self.scale + self.add)
            }
            Kind::S16Le => {
                Value::Number(i16::from_le_bytes([b(0)?, b(1)?]) as f64 * self.scale + self.add)
            }
            Kind::Bit(n) => Value::Flag(b(0)? & (1 << n) != 0),
        })
    }
}

/// Parse the CSV format used in `protocol/`:
/// `name,offset,size,signed,scale,add,unit,metric`, `#` comments,
/// `size` is 1, 2 or `bitN`, a leading `~` on scale marks an estimate.
pub fn parse_csv(text: &str) -> Result<Vec<Param>, String> {
    let mut out = Vec::new();
    for (n, line) in text.lines().enumerate() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') || line.starts_with("name,") {
            continue;
        }
        let f: Vec<&str> = line.split(',').collect();
        if f.len() < 7 {
            return Err(format!("line {}: expected at least 7 fields", n + 1));
        }
        let num = |s: &str| -> Result<f64, String> {
            s.trim_start_matches('~')
                .parse::<f64>()
                .map_err(|e| format!("line {}: {s:?}: {e}", n + 1))
        };
        let offset = f[1]
            .parse::<usize>()
            .map_err(|e| format!("line {}: offset: {e}", n + 1))?;
        let kind = match (f[2], f[3]) {
            ("1", _) => Kind::U8,
            ("2", "yes") => Kind::S16Le,
            ("2", _) => Kind::U16Le,
            (b, _) if b.starts_with("bit") => Kind::Bit(
                b[3..]
                    .parse()
                    .map_err(|e| format!("line {}: {b}: {e}", n + 1))?,
            ),
            (s, _) => return Err(format!("line {}: unknown size {s:?}", n + 1)),
        };
        let (scale, add) = match kind {
            Kind::Bit(_) => (1.0, 0.0),
            _ => (num(f[4])?, num(f[5])?),
        };
        out.push(Param {
            name: f[0].to_string(),
            offset,
            kind,
            scale,
            add,
            unit: f[6].to_string(),
        });
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn embedded_definitions_parse_and_decode() {
        let p = parse_csv(TIBURON_2003_ENGINE_L4).unwrap();
        assert_eq!(p.len(), 27);
        assert_eq!(parse_csv(TIBURON_2003_ABS).unwrap().len(), 21);
        let get = |n: &str| p.iter().find(|x| x.name == n).unwrap().clone();
        let mut block = vec![0u8; 80];
        block[19] = 0x20; // 0x0320 = 800 rpm, little-endian
        block[20] = 0x03;
        block[3] = 112; // 0.75 * 112 - 48 = 36 degC = 96.8 degF
        block[73] = 0b0100;
        assert_eq!(
            get("engine_speed").decode(&block),
            Some(Value::Number(800.0))
        );
        let Some(Value::Number(t)) = get("coolant_temp").decode(&block) else {
            panic!()
        };
        assert!((t - 96.8).abs() < 1e-9);
        assert_eq!(
            get("closed_throttle").decode(&block),
            Some(Value::Flag(true))
        );
        assert_eq!(
            get("cranking_signal").decode(&block),
            Some(Value::Flag(false))
        );
        assert_eq!(get("ign_dwell").decode(&block[..70]), None);
        block[51..53].copy_from_slice(&(-250i16).to_le_bytes());
        assert_eq!(
            get("lt_fuel_trim_idle").decode(&block),
            Some(Value::Number(-1.0))
        );
    }
}

#[cfg(test)]
mod srs_tests {
    use super::*;

    #[test]
    fn srs_names_are_embedded() {
        let t = dtc_texts(SRS_DTC_NAMES);
        assert!(t.len() > 400);
        assert_eq!(t["B1102"], "BATTERY VOLTAGE LOW");
        assert_eq!(t["B1346"], "DAB RESISTANCE-HIGH(1ST)");
    }

    #[test]
    fn srs_ident_layout() {
        // the pattern the Hi-Scan was probed with: A-Z, 0-9, then each
        // position's own index digit
        let mut r = vec![0x5a, 0x80];
        r.extend(b"ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789");
        while r.len() < 80 {
            let i = r.len() - 1;
            r.push(b'0' + (i % 10) as u8);
        }
        let id = parse_srs_ident(&r);
        assert_eq!(id.vehicle, ".ABCDE");
        assert_eq!(id.part_no, "FGHIJKLMNOP");
        assert_eq!(id.part_name, "QRSTUVWXYZ0123456789");
        assert_eq!(id.acu_version.as_deref(), Some("7.8"));
    }
}
