//! Runs the emulator on a dedicated thread, paced to real time, for GUI
//! front ends. Front ends send [`Command`]s and read [`Snapshot`]s; they
//! never touch the `Cpu` (it is not `Send`: its interface and event log use
//! `Rc`, so it is built and lives entirely inside the thread).
//!
//! Real time is 10,000 emulated cycles per millisecond (the oracle ran
//! 20M cycles in 1992 ms).

#![forbid(unsafe_code)]

use std::collections::VecDeque;
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use cascade_core::lcd::{LCD_HEIGHT, LCD_WIDTH};
use cascade_core::{Cpu, Key, KeyEvent, StepEvent};
use cascade_iface::{Event, EventLog, Fake, Interface, KwpConfig, KwpEcu, Null, Traced};

/// Emulated cycles per real millisecond.
pub const CYCLES_PER_MS: u64 = 10_000;
/// Bytes in one frame: 1 bit per pixel, MSB first, rows of 40 bytes.
pub const FRAME_BYTES: usize = LCD_WIDTH * LCD_HEIGHT / 8;
/// Shortest key hold in emulated cycles; shorter presses can fall between
/// two keypad scans and be missed (1.5M is what the oracle runs used).
pub const MIN_HOLD_CYCLES: u64 = 1_572_864;
/// Gap between a stretched release and the next press of the same key.
/// Two keypad scans (26 ms) lose presses on the Hi-Scan's model list;
/// 157 ms (the same as the minimum hold) delivers all of them.
pub const KEY_GAP_CYCLES: u64 = MIN_HOLD_CYCLES;
/// Line events kept for display.
const LOG_KEEP: usize = 500;

/// Builds an interface inside the emulator thread (interfaces need not be
/// `Send`). Used for real adapters, which may fail to open.
pub type IfaceFactory = Arc<dyn Fn() -> Result<Box<dyn Interface>, String> + Send + Sync>;

/// What is on the diagnostic line.
#[derive(Clone)]
pub enum IfaceKind {
    /// Nothing attached.
    Null,
    /// Upstream's fake ECU (ISO 9141 / Hyundai slow init / OBD table).
    Fake,
    /// The KWP2000 ECU model with the given configuration.
    Kwp(KwpConfig),
    /// Anything else, e.g. a serial K-line adapter; `label` is shown.
    Custom { label: String, make: IfaceFactory },
}

impl std::fmt::Debug for IfaceKind {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.label())
    }
}

impl IfaceKind {
    pub fn label(&self) -> String {
        match self {
            IfaceKind::Null => "nothing".into(),
            IfaceKind::Fake => "upstream fake".into(),
            IfaceKind::Kwp(c) => format!("KWP2000 ECU @{:02X}", c.address),
            IfaceKind::Custom { label, .. } => label.clone(),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Speed {
    /// Paced: emulated time runs at this multiple of wall time.
    Real(f64),
    /// As fast as the host allows.
    Max,
}

#[derive(Clone, Debug)]
pub enum Command {
    Key {
        key: Key,
        down: bool,
    },
    /// Release every key that is down (focus lost, mouse left a button).
    ReleaseAll,
    Pause(bool),
    Speed(Speed),
    /// Reset the machine as upstream's reset does (RAM is kept).
    Reset,
    /// Power-cycle: a fresh machine with the given interface.
    Restart(IfaceKind),
    Quit,
}

/// State published by the emulator thread.
#[derive(Clone, Debug)]
pub struct Snapshot {
    /// LCD as shown (layer 1 XOR layer 2), 1 bit per pixel.
    pub frame: Vec<u8>,
    /// Increments whenever `frame` changes.
    pub frame_seq: u64,
    /// Cycle counter of the machine (upstream resets it on reset).
    pub cycles: u64,
    /// Cycles executed since start, across resets.
    pub total_cycles: u64,
    pub pc: u16,
    pub paused: bool,
    pub speed: Speed,
    /// Emulated seconds per wall second over the last second or so.
    pub ratio: f64,
    /// Why the machine stopped, if it did. It stays stopped until reset.
    pub stopped: Option<String>,
    /// Firmware endless-loop resets seen.
    pub resets: u32,
    pub iface: String,
    /// Recent line events, oldest first, already formatted.
    pub log: VecDeque<String>,
    /// Increments whenever `log` changes.
    pub log_seq: u64,
    pub uart_tx: u64,
}

impl Default for Snapshot {
    fn default() -> Self {
        Self {
            frame: vec![0; FRAME_BYTES],
            frame_seq: 0,
            cycles: 0,
            total_cycles: 0,
            pc: 0,
            paused: false,
            speed: Speed::Real(1.0),
            ratio: 0.0,
            stopped: None,
            resets: 0,
            iface: String::new(),
            log: VecDeque::new(),
            log_seq: 0,
            uart_tx: 0,
        }
    }
}

/// Handle to the emulator thread. Dropping it stops the thread.
pub struct Runner {
    tx: Sender<Command>,
    shared: Arc<Mutex<Snapshot>>,
    handle: Option<JoinHandle<()>>,
}

impl Runner {
    pub fn spawn(rom: Vec<u8>, iface: IfaceKind) -> Self {
        Self::spawn_with(rom, iface, Speed::Real(1.0), None)
    }

    /// `max_cycles`: stop (as upstream's `-m`) once the machine's cycle
    /// counter passes it; for tests.
    pub fn spawn_with(
        rom: Vec<u8>,
        iface: IfaceKind,
        speed: Speed,
        max_cycles: Option<u64>,
    ) -> Self {
        Self::spawn_full(rom, None, None, iface, speed, max_cycles)
    }

    /// As [`Runner::spawn_with`], plus an extended ROM (upstream `-x`), such
    /// as a Hyundai "TROUBLE SHOOTING PACK" behind the main pack.
    ///
    /// `eeprom`: a file that holds the 128-word EEPROM between runs, as
    /// upstream's `ROM.eep`: loaded at start if it exists, written whenever
    /// the contents change. The EEPROM also survives power cycles
    /// ([`Command::Restart`]), as on the real tool.
    pub fn spawn_full(
        rom: Vec<u8>,
        exrom: Option<Vec<u8>>,
        eeprom: Option<std::path::PathBuf>,
        iface: IfaceKind,
        speed: Speed,
        max_cycles: Option<u64>,
    ) -> Self {
        let (tx, rx) = mpsc::channel();
        let shared = Arc::new(Mutex::new(Snapshot::default()));
        let s2 = shared.clone();
        let handle = std::thread::Builder::new()
            .name("cascade-emu".into())
            .spawn(move || Emu::new(rom, exrom, eeprom, iface, speed, max_cycles, s2).run(rx))
            .expect("spawn emulator thread");
        Self {
            tx,
            shared,
            handle: Some(handle),
        }
    }

    pub fn send(&self, cmd: Command) {
        // the thread only goes away on Quit; nothing useful to do on error
        let _ = self.tx.send(cmd);
    }

    pub fn snapshot(&self) -> Snapshot {
        self.shared.lock().map(|s| s.clone()).unwrap_or_default()
    }

    /// Read without cloning the frame and log when nothing changed.
    pub fn with_snapshot<R>(&self, f: impl FnOnce(&Snapshot) -> R) -> Option<R> {
        self.shared.lock().ok().map(|s| f(&s))
    }
}

impl Drop for Runner {
    fn drop(&mut self) {
        let _ = self.tx.send(Command::Quit);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

struct Emu {
    rom: Vec<u8>,
    exrom: Option<Vec<u8>>,
    eeprom_path: Option<std::path::PathBuf>,
    /// EEPROM contents last written to `eeprom_path`.
    eeprom_saved: [u16; 128],
    cpu: Cpu,
    log: EventLog,
    log_read: usize,
    iface_label: String,
    speed: Speed,
    paused: bool,
    stopped: Option<String>,
    max_cycles: Option<u64>,
    total_cycles: u64,
    resets: u32,
    /// When each currently pressed key went down (machine cycles).
    held: Vec<(Key, u64)>,
    /// Releases queued for the future, per key (machine cycles).
    pending_release: Vec<(Key, u64)>,
    shared: Arc<Mutex<Snapshot>>,
    frame: Vec<u8>,
    frame_seq: u64,
    lines: VecDeque<String>,
    log_seq: u64,
    // pacing
    anchor_wall: Instant,
    anchor_cycles: u64,
    ratio_wall: Instant,
    ratio_cycles: u64,
    ratio: f64,
}

/// Returns the machine, its line log, and an error if the interface could
/// not be built (the machine then has nothing on the line).
fn build_cpu(
    rom: &[u8],
    exrom: Option<&[u8]>,
    iface: &IfaceKind,
    max_cycles: Option<u64>,
) -> (Cpu, EventLog, Option<String>) {
    let mut error = None;
    let inner: Box<dyn Interface> = match iface {
        IfaceKind::Null => Box::new(Null),
        IfaceKind::Fake => Box::new(Fake::default()),
        IfaceKind::Kwp(cfg) => Box::new(KwpEcu::new(cfg.clone())),
        IfaceKind::Custom { label, make } => match make() {
            Ok(i) => i,
            Err(e) => {
                error = Some(format!("{label}: {e}"));
                Box::new(Null)
            }
        },
    };
    let traced = Traced::new(inner);
    let log = traced.log_handle();
    let mut cpu = Cpu::new(rom.to_vec(), Box::new(traced), false);
    if let Some(x) = exrom {
        cpu.load_extended_rom(x.to_vec());
    }
    if let Some(m) = max_cycles {
        cpu.max_cycles = m;
    }
    (cpu, log, error)
}

fn format_event(e: &cascade_iface::Stamped) -> String {
    let ms = e.at as f64 / CYCLES_PER_MS as f64;
    let what = match e.event {
        Event::Tx(b) => format!("TX {b:02X}"),
        Event::Rx(b) => format!("RX {b:02X}"),
        Event::RxFlush => "RX flush".into(),
        Event::BaudDivisor(d) => format!("baud divisor {d:#x}"),
        Event::SlowInitImminent => "slow init".into(),
        Event::SlowInit(t) => format!("slow init to {t:02X}"),
        Event::SlowInitBit(b) => format!("K {}", if b { "high" } else { "low" }),
        Event::L(b) => format!("L {}", if b { "high" } else { "low" }),
        Event::Can(b) => format!("CAN {}", if b { "on" } else { "off" }),
        Event::Line(l) => format!("line {l}"),
    };
    format!("{ms:10.1} ms  {what}")
}

impl Emu {
    fn new(
        rom: Vec<u8>,
        exrom: Option<Vec<u8>>,
        eeprom_path: Option<std::path::PathBuf>,
        iface: IfaceKind,
        speed: Speed,
        max_cycles: Option<u64>,
        shared: Arc<Mutex<Snapshot>>,
    ) -> Self {
        let (mut cpu, log, error) = build_cpu(&rom, exrom.as_deref(), &iface, max_cycles);
        let mut eeprom_note = None;
        if let Some(p) = &eeprom_path {
            match std::fs::read(p) {
                Ok(img) => {
                    cpu.eeprom.load_image(&img);
                    eeprom_note = Some(format!("EEPROM loaded from {}", p.display()));
                }
                Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
                    eeprom_note = Some(format!("EEPROM starts blank; saving to {}", p.display()));
                }
                Err(e) => eeprom_note = Some(format!("EEPROM {}: {e}", p.display())),
            }
        }
        let eeprom_saved = cpu.eeprom.mem;
        let now = Instant::now();
        let mut emu = Self {
            rom,
            exrom,
            eeprom_path,
            eeprom_saved,
            cpu,
            log,
            log_read: 0,
            iface_label: match &error {
                Some(_) => format!("{} (failed to open)", iface.label()),
                None => iface.label(),
            },
            speed,
            paused: false,
            stopped: None,
            max_cycles,
            total_cycles: 0,
            resets: 0,
            held: Vec::new(),
            pending_release: Vec::new(),
            shared,
            frame: vec![0; FRAME_BYTES],
            frame_seq: 0,
            lines: VecDeque::new(),
            log_seq: 0,
            anchor_wall: now,
            anchor_cycles: 0,
            ratio_wall: now,
            ratio_cycles: 0,
            ratio: 0.0,
        };
        if let Some(e) = error {
            emu.note(e);
        }
        if let Some(n) = eeprom_note {
            emu.note(n);
        }
        emu
    }

    fn reanchor(&mut self) {
        self.anchor_wall = Instant::now();
        self.anchor_cycles = self.total_cycles;
    }

    /// Returns false on Quit.
    fn handle(&mut self, cmd: Command) -> bool {
        match cmd {
            Command::Key { key, down } => {
                let now = self.cpu.cycles();
                if down {
                    if self.held.iter().any(|(k, _)| *k == key) {
                        return true; // auto-repeat or double press
                    }
                    // A release of this key may still be pending (holds are
                    // stretched to MIN_HOLD_CYCLES). Press again only after it,
                    // with one keypad-scan gap, or the two presses merge.
                    let after = self
                        .pending_release
                        .iter()
                        .filter(|(k, at)| *k == key && *at > now)
                        .map(|(_, at)| at + KEY_GAP_CYCLES)
                        .max();
                    self.pending_release.retain(|(_, at)| *at > now);
                    // at 0: applied at the next keypad scan, even after a reset
                    let at = after.unwrap_or(0);
                    self.held.push((key, at.max(now)));
                    self.note(format!("key {:03X} down", key.0));
                    self.cpu.queue_key(KeyEvent {
                        at,
                        key,
                        down: true,
                    });
                } else if let Some(i) = self.held.iter().position(|(k, _)| *k == key) {
                    let (_, pressed) = self.held.remove(i);
                    let at = now.max(pressed + MIN_HOLD_CYCLES);
                    self.note(format!(
                        "key {:03X} up after {:.0} ms",
                        key.0,
                        (now - pressed.min(now)) as f64 / CYCLES_PER_MS as f64
                    ));
                    self.pending_release.push((key, at));
                    self.cpu.queue_key(KeyEvent {
                        at,
                        key,
                        down: false,
                    });
                }
            }
            Command::ReleaseAll => {
                for (key, pressed) in std::mem::take(&mut self.held) {
                    let at = self.cpu.cycles().max(pressed + MIN_HOLD_CYCLES);
                    self.pending_release.push((key, at));
                    self.cpu.queue_key(KeyEvent {
                        at,
                        key,
                        down: false,
                    });
                }
            }
            Command::Pause(p) => {
                self.paused = p;
                self.reanchor();
            }
            Command::Speed(s) => {
                self.speed = s;
                self.reanchor();
            }
            Command::Reset => {
                self.cpu.reset();
                self.cpu.release_all_keys();
                self.stopped = None;
                self.held.clear();
                self.pending_release.clear();
                self.reanchor();
            }
            Command::Restart(iface) => {
                let (cpu, log, error) =
                    build_cpu(&self.rom, self.exrom.as_deref(), &iface, self.max_cycles);
                // a power cycle keeps the EEPROM, as on the real tool
                let eeprom = self.cpu.eeprom.mem;
                self.cpu = cpu;
                self.cpu.eeprom.mem = eeprom;
                self.log = log;
                self.total_cycles = 0;
                self.log_read = 0;
                self.iface_label = match &error {
                    Some(_) => format!("{} (failed to open)", iface.label()),
                    None => iface.label(),
                };
                self.stopped = None;
                self.held.clear();
                self.pending_release.clear();
                self.lines.clear();
                self.log_seq += 1;
                self.resets = 0;
                if let Some(e) = error {
                    self.note(e);
                }
                self.reanchor();
            }
            Command::Quit => {
                self.save_eeprom();
                return false;
            }
        }
        true
    }

    /// Write the EEPROM file if the contents changed since the last write.
    fn save_eeprom(&mut self) {
        let Some(p) = &self.eeprom_path else { return };
        if self.cpu.eeprom.mem == self.eeprom_saved {
            return;
        }
        match std::fs::write(p, self.cpu.eeprom.to_image()) {
            Ok(()) => self.eeprom_saved = self.cpu.eeprom.mem,
            Err(e) => {
                let msg = format!("EEPROM not saved to {}: {e}", p.display());
                // do not retry every slice
                self.eeprom_saved = self.cpu.eeprom.mem;
                self.note(msg);
            }
        }
    }

    /// Add a line to the displayed event log.
    fn note(&mut self, line: String) {
        let ms = self.cpu.cycles() as f64 / CYCLES_PER_MS as f64;
        self.lines.push_back(format!("{ms:10.1} ms  {line}"));
        if self.lines.len() > LOG_KEEP {
            self.lines.pop_front();
        }
        self.log_seq += 1;
    }

    /// Run about `budget` cycles; stops early on a reset or a stop.
    fn run_slice(&mut self, budget: u64) {
        let start = self.cpu.cycles();
        loop {
            let before = self.cpu.cycles();
            match self.cpu.step(None) {
                Ok(StepEvent::Executed) => {
                    self.total_cycles += self.cpu.cycles() - before;
                    if self.cpu.cycles() - start >= budget {
                        return;
                    }
                }
                Ok(StepEvent::EndlessLoopReset { .. }) => {
                    // upstream shows a warning and resets; cycles restart at 0,
                    // so cycle-stamped key events no longer make sense
                    self.resets += 1;
                    self.cpu.release_all_keys();
                    self.held.clear();
                    self.pending_release.clear();
                    return;
                }
                Err(stop) => {
                    self.stopped = Some(format!(
                        "{stop} at {} cycles, PC {:04X}",
                        self.cpu.cycles(),
                        self.cpu.pc()
                    ));
                    return;
                }
            }
        }
    }

    fn publish(&mut self) {
        let mut changed = false;
        if self.cpu.lcd.take_dirty() {
            let m = self.cpu.lcd.memory();
            for (i, b) in self.frame.iter_mut().enumerate() {
                *b = m[i] ^ m[0x2580 + i];
            }
            self.frame_seq += 1;
            changed = true;
        }
        {
            let log = self.log.borrow();
            if log.len() > self.log_read {
                for e in &log[self.log_read..] {
                    self.lines.push_back(format_event(e));
                    if self.lines.len() > LOG_KEEP {
                        self.lines.pop_front();
                    }
                }
                self.log_read = log.len();
                self.log_seq += 1;
            }
        }
        // keep the shared log from growing without bound
        if self.log_read > 10_000 {
            self.log.borrow_mut().clear();
            self.log_read = 0;
        }
        let el = self.ratio_wall.elapsed();
        if el >= Duration::from_millis(500) {
            let emu_ms = (self.total_cycles - self.ratio_cycles) as f64 / CYCLES_PER_MS as f64;
            self.ratio = emu_ms / el.as_secs_f64() / 1000.0;
            self.ratio_wall = Instant::now();
            self.ratio_cycles = self.total_cycles;
        }
        if let Ok(mut s) = self.shared.lock() {
            if changed {
                s.frame.copy_from_slice(&self.frame);
                s.frame_seq = self.frame_seq;
            }
            if s.log_seq != self.log_seq {
                s.log = self.lines.clone();
                s.log_seq = self.log_seq;
            }
            s.cycles = self.cpu.cycles();
            s.total_cycles = self.total_cycles;
            s.pc = self.cpu.pc();
            s.paused = self.paused;
            s.speed = self.speed;
            s.ratio = if self.paused || self.stopped.is_some() {
                0.0
            } else {
                self.ratio
            };
            s.stopped = self.stopped.clone();
            s.resets = self.resets;
            s.iface = self.iface_label.clone();
            s.uart_tx = self.cpu.stats.uart_tx;
        }
    }

    fn run(mut self, rx: Receiver<Command>) {
        self.run_loop(rx);
        // however the loop ended (Quit, or the front end went away)
        self.save_eeprom();
    }

    fn run_loop(&mut self, rx: Receiver<Command>) {
        // 5 ms of emulated time per slice at 1x
        const SLICE_MS: u64 = 5;
        self.publish();
        loop {
            // commands
            loop {
                let idle = self.paused || self.stopped.is_some();
                let msg = if idle {
                    match rx.recv_timeout(Duration::from_millis(50)) {
                        Ok(m) => Some(m),
                        Err(RecvTimeoutError::Timeout) => None,
                        Err(RecvTimeoutError::Disconnected) => return,
                    }
                } else {
                    match rx.try_recv() {
                        Ok(m) => Some(m),
                        Err(mpsc::TryRecvError::Empty) => None,
                        Err(mpsc::TryRecvError::Disconnected) => return,
                    }
                };
                match msg {
                    Some(m) => {
                        if !self.handle(m) {
                            return;
                        }
                    }
                    None => break,
                }
            }
            if self.paused || self.stopped.is_some() {
                self.publish();
                continue;
            }
            match self.speed {
                Speed::Max => self.run_slice(SLICE_MS * CYCLES_PER_MS * 20),
                Speed::Real(f) => {
                    // how far emulated time may be ahead of wall time
                    let allowed = self.anchor_cycles
                        + (self.anchor_wall.elapsed().as_secs_f64()
                            * 1000.0
                            * f
                            * CYCLES_PER_MS as f64) as u64;
                    if self.total_cycles >= allowed {
                        let ahead_ms =
                            (self.total_cycles - allowed) as f64 / (CYCLES_PER_MS as f64 * f);
                        std::thread::sleep(Duration::from_secs_f64(
                            (ahead_ms / 1000.0).clamp(0.0005, 0.02),
                        ));
                    } else {
                        // after a long stall (debugger, sleep) don't try to catch up
                        if allowed - self.total_cycles > 200 * CYCLES_PER_MS {
                            self.reanchor();
                        }
                        self.run_slice((SLICE_MS as f64 * f * CYCLES_PER_MS as f64) as u64);
                    }
                }
            }
            self.publish();
            self.save_eeprom();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rom() -> Option<Vec<u8>> {
        std::fs::read(std::env::var("CASCADE_ROM").ok()?).ok()
    }

    fn wait_for(r: &Runner, what: impl Fn(&Snapshot) -> bool) -> Snapshot {
        let t = Instant::now();
        loop {
            let s = r.snapshot();
            if what(&s) {
                return s;
            }
            assert!(
                t.elapsed() < Duration::from_secs(60),
                "timed out; last state {:?}",
                s.stopped
            );
            std::thread::sleep(Duration::from_millis(20));
        }
    }

    /// The EEPROM file is written while running and when the runner ends,
    /// and its contents survive a power cycle.
    #[test]
    fn eeprom_persists_to_file_and_across_power_cycles() {
        let Some(rom) = rom() else {
            eprintln!("CASCADE_ROM not set; skipping");
            return;
        };
        let path =
            std::env::temp_dir().join(format!("cascade-eeprom-test-{}.eep", std::process::id()));
        let _ = std::fs::remove_file(&path);
        let r = Runner::spawn_full(
            rom.clone(),
            None,
            Some(path.clone()),
            IfaceKind::Null,
            Speed::Max,
            None,
        );
        // the Hyundai pack writes the EEPROM during boot
        wait_for(&r, |s| s.total_cycles > 120_000_000);
        r.send(Command::Restart(IfaceKind::Null));
        wait_for(&r, |s| s.total_cycles > 1_000_000);
        drop(r);
        let img = std::fs::read(&path).expect("EEPROM file written");
        assert_eq!(img.len(), cascade_core::eeprom::IMAGE_BYTES);
        assert!(img.iter().any(|&b| b != 0), "firmware wrote something");

        // a second runner loads the file; stopping at once leaves it intact
        let r = Runner::spawn_full(
            rom,
            None,
            Some(path.clone()),
            IfaceKind::Null,
            Speed::Max,
            Some(1_000),
        );
        wait_for(&r, |s| s.stopped.is_some());
        drop(r);
        assert_eq!(
            std::fs::read(&path).unwrap(),
            img,
            "load and save round trip"
        );
        let _ = std::fs::remove_file(&path);
    }

    /// Unpaced run to a cycle cap: the published frame equals the LCD of a
    /// direct Cpu run to the same cap. Needs CASCADE_ROM.
    #[test]
    fn frame_matches_direct_run() {
        let Some(rom) = rom() else {
            eprintln!("CASCADE_ROM not set; skipping");
            return;
        };
        let cap = 200_000_000;
        let r = Runner::spawn_with(rom.clone(), IfaceKind::Fake, Speed::Max, Some(cap));
        let s = wait_for(&r, |s| s.stopped.is_some());
        assert!(s.stopped.as_deref().unwrap().starts_with("maximum cycles"));

        let mut cpu = Cpu::new(rom, Box::new(Fake::default()), false);
        cpu.max_cycles = cap;
        while cpu.step(None).is_ok() {}
        let m = cpu.lcd.memory();
        let direct: Vec<u8> = (0..FRAME_BYTES).map(|i| m[i] ^ m[0x2580 + i]).collect();
        assert_eq!(s.frame, direct);
        assert!(s.frame.iter().any(|&b| b != 0), "screen not blank");
    }

    /// Real-time pacing keeps emulated time close to wall time.
    #[test]
    fn real_time_pacing() {
        let Some(rom) = rom() else {
            eprintln!("CASCADE_ROM not set; skipping");
            return;
        };
        let r = Runner::spawn(rom, IfaceKind::Fake);
        std::thread::sleep(Duration::from_millis(1500));
        let s = r.snapshot();
        let emu_ms = s.total_cycles as f64 / CYCLES_PER_MS as f64;
        assert!(
            (1300.0..1700.0).contains(&emu_ms),
            "emulated {emu_ms} ms in 1500 ms"
        );
    }

    /// Rapid repeated presses of one key are all delivered: eight quick DOWN
    /// presses on the 26-entry model list move the cursor eight places, the
    /// same screen as eight presses spaced out by the oracle runs.
    #[test]
    fn rapid_repeats_are_not_merged() {
        let Some(rom) = rom() else {
            eprintln!("CASCADE_ROM not set; skipping");
            return;
        };
        let press = |r: &Runner, k: Key| {
            r.send(Command::Key { key: k, down: true });
            r.send(Command::Key {
                key: k,
                down: false,
            });
        };
        let settle = |r: &Runner| {
            let t0 = r.snapshot().total_cycles;
            wait_for(r, |s| s.total_cycles > t0 + 40_000_000);
        };
        let run = |rapid: bool| {
            let r = Runner::spawn_with(rom.clone(), IfaceKind::Fake, Speed::Max, None);
            wait_for(&r, |s| s.total_cycles > 100_000_000);
            press(&r, Key::ENTER); // info screen -> main menu
            settle(&r);
            press(&r, Key::ENTER); // -> model list
            settle(&r);
            for _ in 0..8 {
                press(&r, Key::DOWN);
                if !rapid {
                    settle(&r);
                }
            }
            settle(&r);
            settle(&r);
            r.snapshot().frame
        };
        let (fast, slow) = (run(true), run(false));
        if let Ok(dir) = std::env::var("CASCADE_DEBUG_DIR") {
            for (name, f) in [("rapid", &fast), ("spaced", &slow)] {
                let mut mem = vec![0u8; 65536];
                mem[..FRAME_BYTES].copy_from_slice(f);
                std::fs::write(format!("{dir}/{name}.lcd"), mem).unwrap();
            }
        }
        assert!(
            fast == slow,
            "rapid and spaced presses ended on different screens"
        );
    }

    /// Key presses reach the firmware: ENTER on the info screen opens the
    /// main menu, whose frame differs from the info screen's.
    #[test]
    fn keys_reach_the_firmware() {
        let Some(rom) = rom() else {
            eprintln!("CASCADE_ROM not set; skipping");
            return;
        };
        let r = Runner::spawn_with(rom, IfaceKind::Fake, Speed::Max, None);
        wait_for(&r, |s| s.total_cycles > 200_000_000);
        r.send(Command::Pause(true));
        let before = wait_for(&r, |s| s.paused).frame;
        r.send(Command::Key {
            key: Key::ENTER,
            down: true,
        });
        r.send(Command::Key {
            key: Key::ENTER,
            down: false,
        });
        r.send(Command::Pause(false));
        let t0 = r.snapshot().total_cycles;
        let s = wait_for(&r, |s| s.total_cycles > t0 + 60_000_000);
        assert_ne!(s.frame, before, "screen did not change after ENTER");
    }
}
