//! Linux SocketCAN: a raw CAN socket bound to one interface (`can0`,
//! `vcan0`, `slcan0`, ...). The interface must already be up with its
//! bitrate set, for example:
//!
//! ```text
//! sudo ip link set can0 up type can bitrate 500000
//! sudo ip link add dev vcan0 type vcan && sudo ip link set vcan0 up
//! ```
//!
//! This file is the crate's only `unsafe` code: the socket, bind, poll,
//! read and write calls into libc.

#![allow(unsafe_code)]

use std::ffi::CString;
use std::io;
use std::mem;
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
use std::time::{Duration, Instant};

use crate::{CanBus, EXTENDED, Frame};

pub struct SocketCan {
    fd: OwnedFd,
    pub interface: String,
}

impl SocketCan {
    /// Open a raw socket on `interface`. Frames this socket sends are not
    /// received back (the kernel's default for CAN_RAW).
    pub fn open(interface: &str) -> io::Result<Self> {
        let name = CString::new(interface)
            .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "interface name"))?;
        // SAFETY: plain libc calls with valid arguments; the fd is owned
        // by `OwnedFd` from here on, and `addr` is a zeroed, fully
        // initialised sockaddr_can of the size passed.
        unsafe {
            let index = libc::if_nametoindex(name.as_ptr());
            if index == 0 {
                return Err(io::Error::new(
                    io::ErrorKind::NotFound,
                    format!("no CAN interface {interface:?} (is it up? see `ip link`)"),
                ));
            }
            let raw = libc::socket(
                libc::PF_CAN,
                libc::SOCK_RAW | libc::SOCK_CLOEXEC,
                libc::CAN_RAW,
            );
            if raw < 0 {
                return Err(io::Error::last_os_error());
            }
            let fd = OwnedFd::from_raw_fd(raw);
            let mut addr: libc::sockaddr_can = mem::zeroed();
            addr.can_family = libc::AF_CAN as libc::sa_family_t;
            addr.can_ifindex = index as libc::c_int;
            let r = libc::bind(
                fd.as_raw_fd(),
                &addr as *const libc::sockaddr_can as *const libc::sockaddr,
                mem::size_of::<libc::sockaddr_can>() as libc::socklen_t,
            );
            if r < 0 {
                return Err(io::Error::last_os_error());
            }
            Ok(Self {
                fd,
                interface: interface.into(),
            })
        }
    }

    /// Wait until a frame can be read; false on timeout.
    fn readable(&self, timeout: Duration) -> io::Result<bool> {
        let deadline = Instant::now() + timeout;
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            let mut p = libc::pollfd {
                fd: self.fd.as_raw_fd(),
                events: libc::POLLIN,
                revents: 0,
            };
            let ms = left.as_millis().min(i32::MAX as u128) as libc::c_int;
            // round sub-millisecond waits up so a short timeout still waits
            let ms = if ms == 0 && !left.is_zero() { 1 } else { ms };
            // SAFETY: one valid pollfd, count 1.
            let r = unsafe { libc::poll(&mut p, 1, ms) };
            if r < 0 {
                let e = io::Error::last_os_error();
                if e.kind() == io::ErrorKind::Interrupted {
                    continue;
                }
                return Err(e);
            }
            return Ok(r > 0);
        }
    }
}

impl CanBus for SocketCan {
    fn send(&mut self, frame: &Frame) -> io::Result<()> {
        if frame.data.len() > 8 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "more than 8 bytes",
            ));
        }
        // SAFETY: a zeroed can_frame is valid (padding fields included);
        // write gets its exact size.
        unsafe {
            let mut f: libc::can_frame = mem::zeroed();
            f.can_id = if frame.id & EXTENDED != 0 {
                (frame.id & libc::CAN_EFF_MASK) | libc::CAN_EFF_FLAG
            } else {
                frame.id & libc::CAN_SFF_MASK
            };
            f.can_dlc = frame.data.len() as u8;
            f.data[..frame.data.len()].copy_from_slice(&frame.data);
            let n = mem::size_of::<libc::can_frame>();
            let w = libc::write(
                self.fd.as_raw_fd(),
                &f as *const libc::can_frame as *const libc::c_void,
                n,
            );
            if w < 0 {
                return Err(io::Error::last_os_error());
            }
            if w as usize != n {
                return Err(io::Error::other("short write to CAN socket"));
            }
        }
        Ok(())
    }

    fn recv(&mut self, timeout: Duration) -> io::Result<Option<Frame>> {
        let deadline = Instant::now() + timeout;
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            if !self.readable(left)? {
                return Ok(None);
            }
            // SAFETY: read into a zeroed can_frame of its exact size.
            let f = unsafe {
                let mut f: libc::can_frame = mem::zeroed();
                let n = mem::size_of::<libc::can_frame>();
                let r = libc::read(
                    self.fd.as_raw_fd(),
                    &mut f as *mut libc::can_frame as *mut libc::c_void,
                    n,
                );
                if r < 0 {
                    let e = io::Error::last_os_error();
                    if e.kind() == io::ErrorKind::Interrupted {
                        continue;
                    }
                    return Err(e);
                }
                if (r as usize) < n {
                    continue; // not a classic CAN frame
                }
                f
            };
            // error and remote frames carry no diagnostic data
            if f.can_id & (libc::CAN_ERR_FLAG | libc::CAN_RTR_FLAG) != 0 {
                continue;
            }
            let id = if f.can_id & libc::CAN_EFF_FLAG != 0 {
                (f.can_id & libc::CAN_EFF_MASK) | EXTENDED
            } else {
                f.can_id & libc::CAN_SFF_MASK
            };
            let len = (f.can_dlc as usize).min(8);
            return Ok(Some(Frame {
                id,
                data: f.data[..len].to_vec(),
            }));
        }
    }
}
