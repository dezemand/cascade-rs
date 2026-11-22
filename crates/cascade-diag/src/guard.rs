//! Read-only guard: which requests kdiag may send to a real car.
//!
//! Every request passes through [`check`] before it reaches the line. The
//! allowlist is deliberately narrow: session handling and reads. Clearing
//! fault codes is the one opt-in exception. Everything else (resets,
//! writes, actuation, routines, security access, memory up/download,
//! programming sessions) is refused, whatever the caller asks for.

use std::fmt;

/// Why a request was refused.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Blocked {
    pub sid: u8,
    pub reason: &'static str,
}

impl fmt::Display for Blocked {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "service {:02X} refused: {}", self.sid, self.reason)
    }
}

/// Session handling and reads, by service ID. KWP2000 SIDs unless noted.
fn read_only(sid: u8) -> Option<&'static str> {
    Some(match sid {
        0x81 => "StartCommunication",
        0x82 => "StopCommunication",
        0x3e => "TesterPresent",
        0x1a => "ReadEcuIdentification",
        0x21 => "ReadDataByLocalIdentifier",
        0x22 => "ReadDataByCommonIdentifier",
        0x12 => "ReadFreezeFrameData",
        0x13 => "ReadDiagnosticTroubleCodes",
        0x17 => "ReadStatusOfDiagnosticTroubleCodes",
        0x18 => "ReadDiagnosticTroubleCodesByStatus",
        0x01 => "OBD mode 01 (current data)",
        0x02 => "OBD mode 02 (freeze frame)",
        0x03 => "OBD mode 03 (stored DTCs)",
        0x07 => "OBD mode 07 (pending DTCs)",
        0x09 => "OBD mode 09 (vehicle information)",
        _ => return None,
    })
}

/// Check a request (service data, SID first).
pub fn check(req: &[u8], allow_clear: bool) -> Result<(), Blocked> {
    let Some(&sid) = req.first() else {
        return Err(Blocked {
            sid: 0,
            reason: "empty request",
        });
    };
    if read_only(sid).is_some() {
        return Ok(());
    }
    match sid {
        // only the default session; others are programming/development
        0x10 if req.get(1) == Some(&0x81) => Ok(()),
        0x10 => Err(Blocked {
            sid,
            reason: "only the default diagnostic session (10 81) is allowed",
        }),
        0x14 | 0x04 if allow_clear => Ok(()),
        0x14 | 0x04 => Err(Blocked {
            sid,
            reason: "clearing fault codes needs an explicit opt-in",
        }),
        0x11 => Err(Blocked {
            sid,
            reason: "ECU reset",
        }),
        0x27 => Err(Blocked {
            sid,
            reason: "security access",
        }),
        0x2e | 0x3b | 0x3d => Err(Blocked {
            sid,
            reason: "writes to the ECU",
        }),
        0x2f | 0x30 => Err(Blocked {
            sid,
            reason: "actuator / input-output control",
        }),
        0x31..=0x33 | 0x38..=0x3a => Err(Blocked {
            sid,
            reason: "ECU routines",
        }),
        0x34..=0x37 => Err(Blocked {
            sid,
            reason: "memory download/upload (flashing)",
        }),
        0x23 => Err(Blocked {
            sid,
            reason: "memory reads by address are not needed and not allowed",
        }),
        0x28 | 0x29 | 0x83 | 0x84 | 0x85 => Err(Blocked {
            sid,
            reason: "communication/timing control",
        }),
        _ => Err(Blocked {
            sid,
            reason: "not on the read-only allowlist",
        }),
    }
}

/// Check a request to the traction control module (ISO 9141-2, address
/// 0x32). Its manufacturer service B1 carries both reads and the actuation
/// tests, so only the exact reads the Hi-Scan sends are let through.
pub fn check_tcs(req: &[u8]) -> Result<(), Blocked> {
    match req {
        [0x03] => Ok(()),
        // B1 01 keep-alive, B1 0E wheel speeds, B1 2C valve outputs
        [0xb1, 0x01 | 0x0e | 0x2c] => Ok(()),
        [0x04, ..] => Err(Blocked {
            sid: 0x04,
            reason: "clearing traction control codes is not supported",
        }),
        [sid, ..] => Err(Blocked {
            sid: *sid,
            reason: "not one of the known traction control reads",
        }),
        [] => Err(Blocked {
            sid: 0,
            reason: "empty request",
        }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tcs_allows_only_known_reads() {
        for req in [&[0x03][..], &[0xb1, 0x01], &[0xb1, 0x0e], &[0xb1, 0x2c]] {
            assert_eq!(check_tcs(req), Ok(()), "{req:02X?}");
        }
        for req in [
            &[0x04][..],
            &[0xb1, 0x02],
            &[0xb1, 0x0e, 0x00],
            &[0xb1],
            &[0x30, 0x01],
            &[],
        ] {
            assert!(check_tcs(req).is_err(), "{req:02X?}");
        }
    }

    #[test]
    fn reads_and_session_pass() {
        for req in [
            &[0x81][..],
            &[0x3e, 0x01],
            &[0x21, 0x01],
            &[0x18, 0x00, 0x00, 0x00],
            &[0x13, 0x40, 0x00],
            &[0x17, 0x03, 0x01],
            &[0x12, 0x00, 0x00],
            &[0x01, 0x01],
            &[0x1a, 0x8e],
            &[0x10, 0x81],
            &[0x82],
        ] {
            assert_eq!(check(req, false), Ok(()), "{req:02X?}");
        }
    }

    #[test]
    fn clear_needs_opt_in() {
        assert!(check(&[0x14, 0xff, 0x00], false).is_err());
        assert_eq!(check(&[0x14, 0xff, 0x00], true), Ok(()));
        assert!(check(&[0x04], false).is_err());
    }

    #[test]
    fn dangerous_services_are_always_refused() {
        for req in [
            &[0x11, 0x01][..],
            &[0x27, 0x01],
            &[0x3b, 0x01, 0x00],
            &[0x30, 0x10, 0x07],
            &[0x31, 0x01],
            &[0x34, 0x00],
            &[0x10, 0x85],
            &[0x23, 0x00, 0x00, 0x00, 0x10],
            &[0x99],
            &[],
        ] {
            assert!(
                check(req, true).is_err(),
                "{req:02X?} should be refused even with clear allowed"
            );
        }
    }
}
