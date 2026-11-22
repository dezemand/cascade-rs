//! Socket protocol: one message per line, ASCII.
//!
//! Client to server:
//! - `TX <us> <hex>`: the tester sends these bytes, starting at its time `us`
//!   (microseconds on the client's own clock; only differences matter).
//! - `LOW <us>` / `HIGH <us>`: the tester pulls K low / releases it (fast
//!   init, 5-baud address bits).
//! - `BAUD <n>`: the tester's bit rate (default 10,400).
//! - `LINE <n>`: the diagnostic line the tester switched to (the Hi-Scan's
//!   numbering: 7 engine K-line, 8 ABS/TCS, 12 or 10 SRS); default 7.
//! - `SLOW <us> <addr>`: a complete 5-baud address (also sent as LOW/HIGH).
//! - `SET <name> <value>`: a control (see [`crate::plant::Controls::set`]).
//! - `WATCH`: send `STATUS` lines to this connection (every 100 ms).
//! - `KLINE`: send `RX` lines to this connection.
//!
//! Server to client:
//! - `RX <hex>`: bytes the ECU sent.
//! - `STATUS key=value ...`: controls, engine and ECU state.
//! - `ERR <text>`: a rejected message.

#[derive(Clone, Debug, PartialEq)]
pub enum ToServer {
    Tx {
        us: u64,
        bytes: Vec<u8>,
    },
    Low {
        us: u64,
    },
    High {
        us: u64,
    },
    Baud(u32),
    Line(i32),
    Slow {
        us: u64,
        addr: u8,
    },
    Set {
        name: String,
        value: String,
    },
    Watch,
    KLine,
    /// Send a frame on the engine CAN bus (and receive its traffic):
    /// `CAN id hex`, id in hex (8 digits: 29-bit).
    Can {
        id: u32,
        data: Vec<u8>,
    },
    /// Receive the engine CAN bus traffic.
    CanSub,
}

#[derive(Clone, Debug, PartialEq)]
pub enum ToClient {
    Rx(Vec<u8>),
    /// A frame on the engine CAN bus: `CAN id hex`.
    Can {
        id: u32,
        data: Vec<u8>,
    },
    Status(Vec<(String, String)>),
    Err(String),
}

pub fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02X}")).collect()
}

pub fn unhex(s: &str) -> Option<Vec<u8>> {
    if !s.len().is_multiple_of(2) {
        return None;
    }
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(s.get(i..i + 2)?, 16).ok())
        .collect()
}

/// A CAN identifier as text: 3 hex digits, or 8 for a 29-bit one.
fn can_id(id: u32) -> String {
    if id & cascade_can::EXTENDED != 0 {
        format!("{:08X}", id & !cascade_can::EXTENDED)
    } else {
        format!("{id:03X}")
    }
}

fn can_args(id: Option<&&str>, data: Option<&&str>) -> Result<(u32, Vec<u8>), String> {
    let id = cascade_can::parse_id(id.ok_or("missing CAN id")?)?;
    // no data field: an empty frame
    let data = match data {
        Some(h) => unhex(h).ok_or("bad hex")?,
        None => Vec::new(),
    };
    if data.len() > 8 {
        return Err("a CAN frame carries at most 8 bytes".into());
    }
    Ok((id, data))
}

impl ToServer {
    pub fn parse(line: &str) -> Result<Self, String> {
        let f: Vec<&str> = line.split_whitespace().collect();
        let num = |s: Option<&&str>| -> Result<u64, String> {
            s.ok_or("missing number")?
                .parse()
                .map_err(|e| format!("{e}"))
        };
        Ok(match f.first().copied() {
            Some("TX") => ToServer::Tx {
                us: num(f.get(1))?,
                bytes: f.get(2).and_then(|h| unhex(h)).ok_or("bad hex")?,
            },
            Some("LOW") => ToServer::Low { us: num(f.get(1))? },
            Some("HIGH") => ToServer::High { us: num(f.get(1))? },
            Some("BAUD") => ToServer::Baud(num(f.get(1))? as u32),
            Some("LINE") => ToServer::Line(
                f.get(1)
                    .ok_or("missing line")?
                    .parse()
                    .map_err(|e| format!("{e}"))?,
            ),
            Some("SLOW") => ToServer::Slow {
                us: num(f.get(1))?,
                addr: num(f.get(2))? as u8,
            },
            Some("SET") if f.len() == 3 => ToServer::Set {
                name: f[1].into(),
                value: f[2].into(),
            },
            Some("WATCH") => ToServer::Watch,
            Some("KLINE") => ToServer::KLine,
            Some("CANSUB") => ToServer::CanSub,
            Some("CAN") => {
                let (id, data) = can_args(f.get(1), f.get(2))?;
                ToServer::Can { id, data }
            }
            _ => return Err(format!("unknown message {line:?}")),
        })
    }

    pub fn line(&self) -> String {
        match self {
            ToServer::Tx { us, bytes } => format!("TX {us} {}", hex(bytes)),
            ToServer::Low { us } => format!("LOW {us}"),
            ToServer::High { us } => format!("HIGH {us}"),
            ToServer::Baud(b) => format!("BAUD {b}"),
            ToServer::Line(l) => format!("LINE {l}"),
            ToServer::Slow { us, addr } => format!("SLOW {us} {addr}"),
            ToServer::Set { name, value } => format!("SET {name} {value}"),
            ToServer::Watch => "WATCH".into(),
            ToServer::KLine => "KLINE".into(),
            ToServer::CanSub => "CANSUB".into(),
            ToServer::Can { id, data } => format!("CAN {} {}", can_id(*id), hex(data)),
        }
    }
}

impl ToClient {
    pub fn parse(line: &str) -> Result<Self, String> {
        let (head, rest) = line.split_once(' ').unwrap_or((line, ""));
        Ok(match head {
            "RX" => ToClient::Rx(unhex(rest.trim()).ok_or("bad hex")?),
            "STATUS" => ToClient::Status(
                rest.split_whitespace()
                    .filter_map(|kv| kv.split_once('='))
                    .map(|(k, v)| (k.to_string(), v.to_string()))
                    .collect(),
            ),
            "ERR" => ToClient::Err(rest.into()),
            "CAN" => {
                let f: Vec<&str> = rest.split_whitespace().collect();
                let (id, data) = can_args(f.first(), f.get(1))?;
                ToClient::Can { id, data }
            }
            _ => return Err(format!("unknown message {line:?}")),
        })
    }

    pub fn line(&self) -> String {
        match self {
            ToClient::Rx(b) => format!("RX {}", hex(b)),
            ToClient::Status(kv) => {
                let mut s = String::from("STATUS");
                for (k, v) in kv {
                    s.push(' ');
                    s.push_str(k);
                    s.push('=');
                    s.push_str(v);
                }
                s
            }
            ToClient::Err(e) => format!("ERR {e}"),
            ToClient::Can { id, data } => format!("CAN {} {}", can_id(*id), hex(data)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip() {
        for m in [
            ToServer::Tx {
                us: 12,
                bytes: vec![0x81, 0x11, 0xf1],
            },
            ToServer::Low { us: 5 },
            ToServer::High { us: 25_005 },
            ToServer::Baud(10_400),
            ToServer::Line(12),
            ToServer::Slow { us: 9, addr: 0x32 },
            ToServer::Set {
                name: "ignition".into(),
                value: "on".into(),
            },
            ToServer::Watch,
            ToServer::KLine,
            ToServer::CanSub,
            ToServer::Can {
                id: 0x43f,
                data: vec![0, 1],
            },
            ToServer::Can {
                id: 0x18da_10f1 | cascade_can::EXTENDED,
                data: vec![],
            },
        ] {
            assert_eq!(ToServer::parse(&m.line()).unwrap(), m);
        }
        for m in [
            ToClient::Rx(vec![0x83, 0xf1]),
            ToClient::Status(vec![("rpm".into(), "800".into())]),
            ToClient::Err("x y".into()),
            ToClient::Can {
                id: 0x316,
                data: vec![5, 0x5a],
            },
        ] {
            assert_eq!(ToClient::parse(&m.line()).unwrap(), m);
        }
        assert!(ToServer::parse("TX 1 ABC").is_err());
    }
}
