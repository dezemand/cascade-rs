//! A vendor J2534 04.04 DLL, loaded at run time (Windows only).
//!
//! This is the only `unsafe` code in the workspace: calling C functions in a
//! DLL cannot be checked by the compiler. Each call passes pointers to
//! locals that outlive the call, with the struct layouts from J2534-1.
//! Untested against a real adapter.

#![allow(unsafe_code)]

use std::ffi::{CStr, c_char, c_void};
use std::time::{Duration, Instant};

use libloading::Library;

use crate::{Error, PassThru, protocol, status};

const MSG_DATA: usize = 4128;

/// PASSTHRU_MSG (J2534-1). All fields are 32-bit, so there is no padding.
#[repr(C)]
struct Msg {
    protocol_id: u32,
    rx_status: u32,
    tx_flags: u32,
    timestamp: u32,
    data_size: u32,
    extra_data_index: u32,
    data: [u8; MSG_DATA],
}

impl Msg {
    fn new(protocol_id: u32, data: &[u8]) -> Result<Box<Msg>, Error> {
        if data.len() > MSG_DATA {
            return Err(Error::new(0, "message too long"));
        }
        let mut m = Box::new(Msg {
            protocol_id,
            rx_status: 0,
            tx_flags: 0,
            timestamp: 0,
            data_size: data.len() as u32,
            extra_data_index: 0,
            data: [0; MSG_DATA],
        });
        m.data[..data.len()].copy_from_slice(data);
        Ok(m)
    }

    fn bytes(&self) -> Vec<u8> {
        self.data[..(self.data_size as usize).min(MSG_DATA)].to_vec()
    }
}

#[repr(C)]
struct SConfig {
    parameter: u32,
    value: u32,
}

#[repr(C)]
struct SConfigList {
    num_of_params: u32,
    config: *mut SConfig,
}

type OpenFn = unsafe extern "system" fn(name: *const c_void, device: *mut u32) -> i32;
type CloseFn = unsafe extern "system" fn(device: u32) -> i32;
type ConnectFn = unsafe extern "system" fn(
    device: u32,
    protocol: u32,
    flags: u32,
    baud: u32,
    channel: *mut u32,
) -> i32;
type DisconnectFn = unsafe extern "system" fn(channel: u32) -> i32;
type MsgsFn =
    unsafe extern "system" fn(channel: u32, msgs: *mut Msg, num: *mut u32, timeout: u32) -> i32;
type IoctlFn =
    unsafe extern "system" fn(handle: u32, id: u32, input: *mut c_void, output: *mut c_void) -> i32;
type LastErrorFn = unsafe extern "system" fn(description: *mut c_char) -> i32;

mod ioctl {
    pub const SET_CONFIG: u32 = 0x02;
    pub const READ_VBATT: u32 = 0x03;
    pub const FAST_INIT: u32 = 0x05;
    pub const CLEAR_RX_BUFFER: u32 = 0x08;
}

/// RxStatus bits that mark messages that are not ECU data.
const TX_MSG_TYPE: u32 = 0x01;
const START_OF_MESSAGE: u32 = 0x02;
const TX_INDICATION: u32 = 0x08;

pub struct DllPassThru {
    open_fn: OpenFn,
    close_fn: CloseFn,
    connect_fn: ConnectFn,
    disconnect_fn: DisconnectFn,
    read_fn: MsgsFn,
    write_fn: MsgsFn,
    ioctl_fn: IoctlFn,
    last_error_fn: LastErrorFn,
    device: Option<u32>,
    channel: Option<u32>,
    t0: Instant,
    // dropped last: the function pointers above point into it
    _lib: Library,
}

impl DllPassThru {
    /// Load the DLL at `path` and open the first device it offers.
    pub fn open(path: &str) -> Result<Self, Error> {
        // SAFETY: loading a vendor DLL runs its initialisation code; that is
        // the purpose here, and the path comes from the user or the registry.
        let lib =
            unsafe { Library::new(path) }.map_err(|e| Error::new(0, format!("{path}: {e}")))?;
        // SAFETY: the symbol types follow the J2534-1 prototypes.
        let sym = |name: &[u8]| -> Result<*const c_void, Error> {
            unsafe { lib.get::<*const c_void>(name) }
                .map(|s| *s)
                .map_err(|e| Error::new(0, format!("{}: {e}", String::from_utf8_lossy(name))))
        };
        // SAFETY: each pointer is a function with the matching J2534 signature.
        let mut pt = unsafe {
            DllPassThru {
                open_fn: std::mem::transmute::<*const c_void, OpenFn>(sym(b"PassThruOpen\0")?),
                close_fn: std::mem::transmute::<*const c_void, CloseFn>(sym(b"PassThruClose\0")?),
                connect_fn: std::mem::transmute::<*const c_void, ConnectFn>(sym(
                    b"PassThruConnect\0",
                )?),
                disconnect_fn: std::mem::transmute::<*const c_void, DisconnectFn>(sym(
                    b"PassThruDisconnect\0",
                )?),
                read_fn: std::mem::transmute::<*const c_void, MsgsFn>(sym(b"PassThruReadMsgs\0")?),
                write_fn: std::mem::transmute::<*const c_void, MsgsFn>(sym(
                    b"PassThruWriteMsgs\0",
                )?),
                ioctl_fn: std::mem::transmute::<*const c_void, IoctlFn>(sym(b"PassThruIoctl\0")?),
                last_error_fn: std::mem::transmute::<*const c_void, LastErrorFn>(sym(
                    b"PassThruGetLastError\0",
                )?),
                device: None,
                channel: None,
                t0: Instant::now(),
                _lib: lib,
            }
        };
        let mut dev = 0u32;
        // SAFETY: a null name means "first device"; `dev` outlives the call.
        let r = unsafe { (pt.open_fn)(std::ptr::null(), &mut dev) };
        pt.check(r)?;
        pt.device = Some(dev);
        Ok(pt)
    }

    fn check(&self, code: i32) -> Result<(), Error> {
        if code as u32 == status::NOERROR {
            return Ok(());
        }
        let mut buf = [0 as c_char; 256];
        // SAFETY: J2534 writes at most 80 characters plus NUL into the buffer.
        let msg = unsafe {
            (self.last_error_fn)(buf.as_mut_ptr());
            CStr::from_ptr(buf.as_ptr()).to_string_lossy().into_owned()
        };
        Err(Error::new(code as u32, msg))
    }

    fn channel(&self) -> Result<u32, Error> {
        self.channel
            .ok_or_else(|| Error::new(0, "no channel connected"))
    }

    fn ms(d: Duration) -> u32 {
        d.as_millis().min(u32::MAX as u128) as u32
    }
}

impl PassThru for DllPassThru {
    fn connect_iso14230(&mut self, baud: u32, config: &[(u32, u32)]) -> Result<(), Error> {
        let dev = self
            .device
            .ok_or_else(|| Error::new(0, "device not open"))?;
        let mut ch = 0u32;
        // SAFETY: `ch` outlives the call.
        let r = unsafe { (self.connect_fn)(dev, protocol::ISO14230, 0, baud, &mut ch) };
        self.check(r)?;
        self.channel = Some(ch);
        if !config.is_empty() {
            let mut items: Vec<SConfig> = config
                .iter()
                .map(|&(parameter, value)| SConfig { parameter, value })
                .collect();
            let mut list = SConfigList {
                num_of_params: items.len() as u32,
                config: items.as_mut_ptr(),
            };
            // SAFETY: `list` and `items` outlive the call.
            let r = unsafe {
                (self.ioctl_fn)(
                    ch,
                    ioctl::SET_CONFIG,
                    (&mut list as *mut SConfigList).cast(),
                    std::ptr::null_mut(),
                )
            };
            self.check(r)?;
        }
        Ok(())
    }

    fn fast_init(&mut self, request: &[u8]) -> Result<Vec<u8>, Error> {
        let ch = self.channel()?;
        let mut input = Msg::new(protocol::ISO14230, request)?;
        let mut output = Msg::new(protocol::ISO14230, &[])?;
        // SAFETY: both messages are boxed and outlive the call.
        let r = unsafe {
            (self.ioctl_fn)(
                ch,
                ioctl::FAST_INIT,
                (&mut *input as *mut Msg).cast(),
                (&mut *output as *mut Msg).cast(),
            )
        };
        self.check(r)?;
        Ok(output.bytes())
    }

    fn write(&mut self, msg: &[u8], timeout: Duration) -> Result<(), Error> {
        let ch = self.channel()?;
        let mut m = Msg::new(protocol::ISO14230, msg)?;
        let mut n = 1u32;
        // SAFETY: `m` and `n` outlive the call.
        let r = unsafe { (self.write_fn)(ch, &mut *m, &mut n, Self::ms(timeout)) };
        self.check(r)
    }

    fn read(&mut self, timeout: Duration) -> Result<Option<Vec<u8>>, Error> {
        let ch = self.channel()?;
        let deadline = Instant::now() + timeout;
        let mut m = Msg::new(protocol::ISO14230, &[])?;
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            let mut n = 1u32;
            // SAFETY: `m` and `n` outlive the call.
            let r = unsafe { (self.read_fn)(ch, &mut *m, &mut n, Self::ms(left)) };
            let code = r as u32;
            if n == 0 && (code == status::ERR_BUFFER_EMPTY || code == status::ERR_TIMEOUT) {
                return Ok(None);
            }
            self.check(r)?;
            if n == 1
                && m.data_size > 0
                && m.rx_status & (TX_MSG_TYPE | START_OF_MESSAGE | TX_INDICATION) == 0
            {
                return Ok(Some(m.bytes()));
            }
            if left.is_zero() {
                return Ok(None);
            }
        }
    }

    fn clear_rx(&mut self) -> Result<(), Error> {
        let ch = self.channel()?;
        // SAFETY: CLEAR_RX_BUFFER takes no input or output.
        let r = unsafe {
            (self.ioctl_fn)(
                ch,
                ioctl::CLEAR_RX_BUFFER,
                std::ptr::null_mut(),
                std::ptr::null_mut(),
            )
        };
        self.check(r)
    }

    fn battery_mv(&mut self) -> Result<u32, Error> {
        let dev = self
            .device
            .ok_or_else(|| Error::new(0, "device not open"))?;
        let mut mv = 0u32;
        // SAFETY: READ_VBATT writes one u32 to `mv`, which outlives the call.
        let r = unsafe {
            (self.ioctl_fn)(
                dev,
                ioctl::READ_VBATT,
                std::ptr::null_mut(),
                (&mut mv as *mut u32).cast(),
            )
        };
        self.check(r)?;
        Ok(mv)
    }

    fn elapsed(&self) -> Duration {
        self.t0.elapsed()
    }

    fn idle(&mut self, d: Duration) {
        std::thread::sleep(d);
    }
}

impl Drop for DllPassThru {
    fn drop(&mut self) {
        // SAFETY: handles came from this DLL and are released once.
        unsafe {
            if let Some(ch) = self.channel.take() {
                (self.disconnect_fn)(ch);
            }
            if let Some(dev) = self.device.take() {
                (self.close_fn)(dev);
            }
        }
    }
}
