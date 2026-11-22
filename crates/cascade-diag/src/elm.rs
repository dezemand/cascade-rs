//! An ELM327 front: OBD apps that talk to a WiFi ELM327 adapter (TCP, AT
//! commands, hex requests) can read the car through kdiag's client. Every
//! request still passes the read-only guard in [`crate::kwp::Client`]:
//! clearing (mode 04) only with `--allow-clear`, everything that writes,
//! actuates or programs is refused ("?" to the app).
//!
//! Enough of the ELM327 command set for common apps: ATZ, ATI, AT@1, ATE,
//! ATL, ATS, ATH, ATSP/ATTP, ATDP/ATDPN, ATRV, plus harmless settings
//! answered OK. Requests go to the engine ECU over KWP2000 (ISO 14230-4)
//! or, with kdiag's `--can`, OBD-II on CAN (ISO 15765-4, 11-bit, 500 kbit/s
//! reported [assumed: the bus rate is whatever the interface is set to]).

use std::io::{self, BufRead, BufReader, Write};
use std::net::{TcpListener, TcpStream};

use crate::kwp::{Client, Error, Link, hex};

/// What the client's link is, for ATDP and headers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Proto {
    Kwp,
    /// CAN, with the ECU's response identifier.
    Can {
        rx_id: u32,
    },
}

#[derive(Clone, Copy)]
struct Settings {
    echo: bool,
    linefeed: bool,
    spaces: bool,
    headers: bool,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            echo: true,
            linefeed: false,
            spaces: true,
            headers: false,
        }
    }
}

/// Answer one command line; `None` closes nothing, it is the text to send.
fn answer<L: Link>(
    c: &mut Client<L>,
    session: &mut bool,
    st: &mut Settings,
    battery: &dyn Fn() -> Option<f64>,
    proto: Proto,
    cmd: &str,
) -> String {
    let cmd: String = cmd.chars().filter(|c| !c.is_whitespace()).collect();
    let up = cmd.to_ascii_uppercase();
    if let Some(at) = up.strip_prefix("AT") {
        return match at {
            "Z" | "WS" => {
                *st = Settings::default();
                *session = false;
                "ELM327 v1.5".into()
            }
            "I" => "ELM327 v1.5".into(),
            "@1" => "cascade-rs kdiag".into(),
            "E0" | "E1" => {
                st.echo = at == "E1";
                "OK".into()
            }
            "L0" | "L1" => {
                st.linefeed = at == "L1";
                "OK".into()
            }
            "S0" | "S1" => {
                st.spaces = at == "S1";
                "OK".into()
            }
            "H0" | "H1" => {
                st.headers = at == "H1";
                "OK".into()
            }
            "DP" => match proto {
                Proto::Kwp => "ISO 14230-4 (KWP FAST)".into(),
                Proto::Can { .. } => "ISO 15765-4 (CAN 11/500)".into(),
            },
            "DPN" => match proto {
                Proto::Kwp => "A5".into(),
                Proto::Can { .. } => "A6".into(),
            },
            "RV" => battery().map_or("?".into(), |v| format!("{v:.1}V")),
            a if a.starts_with("SP") || a.starts_with("TP") => {
                *session = false;
                "OK".into()
            }
            // timeouts, adaptive timing, CAN options, memory, headers...
            _ => "OK".into(),
        };
    }
    let Some(req) = (0..up.len())
        .step_by(2)
        .map(|i| {
            up.get(i..i + 2)
                .and_then(|b| u8::from_str_radix(b, 16).ok())
        })
        .collect::<Option<Vec<u8>>>()
        .filter(|r| !r.is_empty())
    else {
        return "?".into();
    };
    let attempt = |c: &mut Client<L>, session: &mut bool| -> Result<Vec<u8>, Error> {
        if !*session {
            c.start()?;
            *session = true;
        }
        c.request(&req)
    };
    let r = match attempt(c, session) {
        // the session may have timed out: start again once
        Err(Error::Timeout(_)) | Err(Error::Io(_)) => {
            *session = false;
            attempt(c, session)
        }
        other => other,
    };
    match r {
        Ok(data) => {
            let h = match (st.headers, proto) {
                (false, _) => hex(&data),
                (true, Proto::Kwp) => {
                    let mut f = vec![0x80 | data.len() as u8, c.source, c.target];
                    f.extend_from_slice(&data);
                    f.push(f.iter().fold(0u8, |a, &b| a.wrapping_add(b)));
                    hex(&f)
                }
                // identifier, then the single-frame length byte as an ELM
                // shows it (longer answers: identifier and data only)
                (true, Proto::Can { rx_id }) if data.len() <= 7 => {
                    format!("{rx_id:03X} {:02X} {}", data.len(), hex(&data))
                }
                (true, Proto::Can { rx_id }) => format!("{rx_id:03X} {}", hex(&data)),
            };
            if st.spaces { h } else { h.replace(' ', "") }
        }
        Err(Error::Blocked(b)) => {
            eprintln!("elm: refused {}: {b}", hex(&req));
            "?".into()
        }
        Err(Error::Negative { .. }) => "NO DATA".into(),
        Err(Error::Timeout(_)) => {
            *session = false;
            "NO DATA".into()
        }
        Err(e) => {
            eprintln!("elm: {}: {e}", hex(&req));
            *session = false;
            "CAN ERROR".into()
        }
    }
}

fn serve_one<L: Link>(
    c: &mut Client<L>,
    battery: &dyn Fn() -> Option<f64>,
    proto: Proto,
    stream: TcpStream,
) -> io::Result<()> {
    let mut w = stream.try_clone()?;
    let mut st = Settings::default();
    let mut session = false;
    let mut reader = BufReader::new(stream);
    let mut buf = Vec::new();
    w.write_all(b"\r\n>")?;
    loop {
        buf.clear();
        if reader.read_until(b'\r', &mut buf)? == 0 {
            return Ok(());
        }
        let line = String::from_utf8_lossy(&buf).trim().to_string();
        if line.is_empty() {
            w.write_all(b">")?;
            continue;
        }
        let out = answer(c, &mut session, &mut st, battery, proto, &line);
        let nl = if st.linefeed { "\r\n" } else { "\r" };
        let mut text = String::new();
        if st.echo {
            text.push_str(&line);
            text.push_str(nl);
        }
        text.push_str(&out);
        text.push_str(nl);
        text.push_str(nl);
        text.push('>');
        w.write_all(text.as_bytes())?;
    }
}

/// Serve ELM327 clients on `addr` (one at a time), forever.
pub fn serve<L: Link>(
    c: &mut Client<L>,
    battery: &dyn Fn() -> Option<f64>,
    proto: Proto,
    addr: &str,
) -> io::Result<()> {
    let listener = TcpListener::bind(addr)?;
    eprintln!("elm: ELM327 on {addr} (read-only guard on)");
    for stream in listener.incoming() {
        let stream = stream?;
        eprintln!("elm: client {}", stream.peer_addr()?);
        if let Err(e) = serve_one(c, battery, proto, stream) {
            eprintln!("elm: {e}");
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::SimLine;
    use cascade_iface::{KwpConfig, KwpEcu};

    #[test]
    fn at_commands_and_obd_requests() {
        let mut cfg = KwpConfig::default();
        cfg.responses
            .insert(vec![0x01, 0x0c], vec![0x41, 0x0c, 0x0c, 0x80]);
        let mut c = Client::new(SimLine::new(KwpEcu::new(cfg)));
        let (mut session, mut st) = (false, Settings::default());
        let bat = || Some(13.9);
        let mut ask = |cmd: &str| answer(&mut c, &mut session, &mut st, &bat, Proto::Kwp, cmd);
        assert_eq!(ask("ATZ"), "ELM327 v1.5");
        assert_eq!(ask("ATRV"), "13.9V");
        assert_eq!(ask("010C"), "41 0C 0C 80");
        assert_eq!(ask("ATS0"), "OK");
        assert_eq!(ask("01 0C"), "410C0C80");
        // the read-only guard: clearing and writes are refused
        assert_eq!(ask("04"), "?");
        assert_eq!(ask("2E0101"), "?");
        assert_eq!(ask("zz"), "?");
    }

    #[test]
    fn can_protocol_and_headers() {
        let mut cfg = KwpConfig::default();
        cfg.responses
            .insert(vec![0x01, 0x0c], vec![0x41, 0x0c, 0x0c, 0x80]);
        let mut c = Client::new(SimLine::new(KwpEcu::new(cfg)));
        let (mut session, mut st) = (false, Settings::default());
        let bat = || None;
        let can = Proto::Can { rx_id: 0x7e8 };
        let mut ask = |cmd: &str| answer(&mut c, &mut session, &mut st, &bat, can, cmd);
        assert_eq!(ask("ATDP"), "ISO 15765-4 (CAN 11/500)");
        assert_eq!(ask("ATDPN"), "A6");
        assert_eq!(ask("ATRV"), "?");
        assert_eq!(ask("ATH1"), "OK");
        assert_eq!(ask("010C"), "7E8 04 41 0C 0C 80");
    }
}
