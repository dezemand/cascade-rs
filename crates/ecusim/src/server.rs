//! The ECU process: the dump running in real time behind a Unix socket.
//!
//! One thread owns the emulated C167 (`ecuc167::cpu::Cpu`) and the engine
//! model; a listener thread accepts connections and a reader thread per
//! connection forwards its messages. The ECU is powered while the ignition
//! is on (a fresh CPU at every key-on) and paced to wall time.
//!
//! K-line timing: a client stamps its events with its own clock. The first
//! event maps that clock onto ECU time "now"; later events keep their
//! spacing, so a 25 ms fast-init pulse stays 25 ms even if the messages
//! arrive in bursts. While the tester holds K low the ECU waits at the start
//! of the pulse, because the pulse's end is only known when `HIGH` arrives.

use std::collections::BTreeMap;
use std::io::{self, BufRead, BufReader, Write};
use std::os::unix::net::{UnixListener, UnixStream};
use std::path::{Path, PathBuf};
use std::sync::mpsc::{self, Receiver, Sender};
use std::time::{Duration, Instant};

use ecuc167::cpu::{Cam, Cpu, Event, Pin, Wheel};

use crate::ecu_view;
use crate::plant::{Controls, Fault, Plant};
use crate::probe;
use crate::proto::{ToClient, ToServer};
use crate::vehicle;
use cascade_iface::Interface;

/// CPU clock of the running dump: 16 MHz for a 256 KB SIMK41 (its UART
/// divisors and 1 ms tick fit 16 MHz), 24 MHz for a 512 KB SIMK43 (its
/// 200-baud divisor 0xEA5 fits 24 MHz) [emulator-derived]. Set once by
/// [`Server::new`]; one ECU per process.
static FCPU_HZ: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(16_000_000);

fn fcpu() -> u64 {
    FCPU_HZ.load(std::sync::atomic::Ordering::Relaxed)
}

/// The clock a dump of this size runs at.
pub fn fcpu_for(rom: &[u8]) -> u64 {
    if rom.len() > 256 * 1024 {
        24_000_000
    } else {
        16_000_000
    }
}

/// Crank wheel: 60-2 on P2.15 (CC15 capture and T7IN), T0IN (the angle
/// count that schedules injection and spark on CC0-CC3/CC12) and GPT2 CAPIN; cam
/// level on P2.14 (CC14). Found by running the dump: with these the ECU's
/// engine speed follows the wheel (it needs `Cpu::compare_on_write`, on by
/// default, to keep sync across the gap). Road speed: one pulse wheel per known input
/// (CC30 on the GK-20 dump, CC13 on the non-immobiliser one); only the one
/// the running software sets up turns.
fn wheels() -> Vec<Wheel> {
    let mut crank = Wheel::new(
        vec![
            Pin::Cc { unit: 0, ch: 15 },
            Pin::T7In,
            Pin::T0In,
            Pin::CapIn,
        ],
        60,
        2,
    );
    crank.cam = Some(Cam {
        targets: vec![Pin::Cc { unit: 0, ch: 14 }],
        // low from tooth 50 to 110 of the cycle: every dump tried syncs
        // and sets no cam code with it (SIMK41 and the CVVT SIMK43s; most
        // other phases set P0340 on the SIMK43s, and 0..60 stops them from
        // seeing the engine at all)
        lows: vec![(50, 110)],
    });
    let vss_cc30 = Wheel::new(vec![Pin::Cc { unit: 1, ch: 14 }], 1, 0);
    let vss_cc13 = Wheel::new(vec![Pin::Cc { unit: 0, ch: 13 }], 1, 0);
    vec![crank, vss_cc30, vss_cc13]
}

/// Road-speed pulses per second per km/h, emulator-derived from each dump's
/// speed byte: GK-20 on CC30 about 6.96, the non-immobiliser dump on CC13
/// 6.37 (linear from 8 to 251 km/h).
const VSS_HZ_PER_KMH_CC30: f64 = 6.96;
const VSS_HZ_PER_KMH_CC13: f64 = 6.37;

/// Which road-speed input the running software has set up for capture.
fn speed_input(cpu: &Cpu) -> Option<&'static str> {
    let capture = |ccm: u32, n: u32| (1..=3).contains(&((cpu.peek16(ccm) >> (4 * n)) & 7));
    if capture(0xFF28, 2) {
        Some("cc30")
    } else if capture(0xFF58, 1) {
        Some("cc13")
    } else {
        None
    }
}

enum Msg {
    Connect(u64, UnixStream),
    Disconnect(u64),
    Cmd(u64, ToServer),
}

#[derive(Default)]
struct Client {
    stream: Option<UnixStream>,
    watch: bool,
    kline: bool,
    baud: u32,
    /// (client us, ECU cycles) tying the client's clock to ECU time.
    clock: Option<(u64, u64)>,
    /// The diagnostic line the tester has selected (7 = engine).
    line: i32,
    /// (client us, model cycles) for the ABS/TCS/SRS models.
    mclock: Option<(u64, u64)>,
    /// Receives the engine CAN bus traffic.
    can: bool,
}

impl Client {
    fn send(&mut self, m: &ToClient) {
        if let Some(s) = &mut self.stream
            && writeln!(s, "{}", m.line()).is_err()
        {
            self.stream = None;
        }
    }

    /// Model time (10,000 cycles per ms) for the client's `us`, never in
    /// the models' past.
    fn model_time(&mut self, us: u64, now: u64) -> u64 {
        let t = match self.mclock {
            Some((u0, t0)) if us >= u0 => t0 + (us - u0) * 10,
            _ => now,
        };
        let t = if t < now || t > now + 10_000_000 {
            now
        } else {
            t
        };
        self.mclock = Some((us, t));
        t
    }

    /// ECU time for the client's `us`, never in the ECU's past.
    fn ecu_time(&mut self, us: u64, now: u64) -> u64 {
        let t = match self.clock {
            Some((u0, t0)) if us >= u0 => t0 + (us - u0) * (fcpu() / 1_000_000),
            _ => now,
        };
        // more than a second off: the client paused; start again from now
        let t = if t < now || t > now + fcpu() { now } else { t };
        self.clock = Some((us, t));
        t
    }
}

#[derive(Clone)]
struct Ecu {
    cpu: Cpu,
    /// Virtual time (s) at power-on; the ECU follows `Server::virt`.
    on_virt: f64,
    /// Tester pulled K low at this ECU time; the ECU waits there.
    hold: Option<u64>,
    stopped: Option<String>,
    /// Compare matches in the last status period: CC12 (ignition events)
    /// and CC0-CC3 (injection events) [assumed roles].
    spark: u64,
    inj: u64,
    rx_bytes: u64,
    /// Virtual time of the last injection event (CC0-CC3).
    inj_at: f64,
    /// The other nodes on the engine CAN bus.
    bus: Bus,
    /// Frames on the bus since the last pass (the ECU's and the nodes'),
    /// for subscribed clients.
    can_out: Vec<cascade_can::Frame>,
    /// Last frame per identifier the ECU sent, and how many.
    can_seen: BTreeMap<u32, (Vec<u8>, u64)>,
}

/// The engine CAN bus's other nodes (500 kbit/s; identifiers and layouts
/// from the OpenGK CAN database, periods [assumed]). Their frames are
/// put on the bus on the ECU's own clock.
#[derive(Clone, Default)]
struct Bus {
    esp: bool,
    gearbox: bool,
    /// Wheel speed the ESP reports.
    kmh: f64,
    /// Frames from socket clients, sent with the next pass.
    inject: Vec<cascade_can::Frame>,
    /// ECU cycle each node frame is next due: 153, 1F0, 43F.
    next: [u64; 3],
}

/// ESP and gearbox frames: (identifier, period ms).
const NODE_FRAMES: [(u32, u64); 3] = [(0x153, 10), (0x1f0, 20), (0x43f, 10)];

impl Bus {
    fn frame(&self, i: usize) -> Option<cascade_can::Frame> {
        // ESP wheel speeds: 0.95439 km/h per bit (database)
        let v = (self.kmh / 0.95439).round().clamp(0.0, 255.0) as u8;
        let (id, _) = NODE_FRAMES[i];
        let data = match id {
            0x153 if self.esp => vec![0, 0, v, 0, 0, 0, 0],
            0x1f0 if self.esp => vec![v, v, v, v, 0, 0, 0],
            // gearbox frame layout unknown: present, all zero [assumed]
            0x43f if self.gearbox => vec![0; 8],
            _ => return None,
        };
        Some(cascade_can::Frame { id, data })
    }
}

impl Ecu {
    fn new(rom: &[u8], virt: f64) -> Self {
        let mut cpu = Cpu::new(rom.to_vec(), fcpu());
        cpu.wheels = wheels();
        cpu.log_compares = true;
        Self {
            cpu,
            on_virt: virt,
            hold: None,
            stopped: None,
            spark: 0,
            inj: 0,
            rx_bytes: 0,
            inj_at: -1.0,
            bus: Bus::default(),
            can_out: Vec::new(),
            can_seen: BTreeMap::new(),
        }
    }

    /// Run up to wall time (or the hold point), at most `max` cycles.
    /// Returns bytes the ECU sent at K-line speed.
    fn run(&mut self, virt: f64, max: u64) -> Vec<u8> {
        let wall = ((virt - self.on_virt).max(0.0) * fcpu() as f64) as u64;
        let mut target = wall.min(self.cpu.cycles + max);
        if let Some(h) = self.hold {
            target = target.min(h);
        }
        let mut out = Vec::new();
        for f in std::mem::take(&mut self.bus.inject) {
            self.cpu.can.rx(&f);
            self.can_out.push(f);
        }
        let ms = fcpu() / 1000;
        while self.stopped.is_none() && self.cpu.cycles < target {
            for (i, &(_, period)) in NODE_FRAMES.iter().enumerate() {
                if self.cpu.cycles >= self.bus.next[i] {
                    self.bus.next[i] = self.cpu.cycles + period * ms;
                    if let Some(f) = self.bus.frame(i) {
                        self.cpu.can.rx(&f);
                        self.can_out.push(f);
                    }
                }
            }
            if let Err(e) = self.cpu.step() {
                self.stopped = Some(e.to_string());
            }
            let fast = self.cpu.bit_cycles() < fcpu() / 5_000; // above 5 kbaud: K-line speed
            for ev in self.cpu.events.drain(..) {
                match ev {
                    // only K-line-speed bytes: the boot code also talks at
                    // 200/4800 baud on the same UART
                    Event::Tx { byte, .. } if fast => out.push(byte),
                    Event::Compare {
                        unit: 0, ch: 12, ..
                    } => self.spark += 1,
                    Event::Compare {
                        unit: 0, ch: 0..=3, ..
                    } => {
                        self.inj += 1;
                        self.inj_at = virt;
                    }
                    _ => {}
                }
            }
        }
        for (_, f) in self.cpu.can.tx.drain(..) {
            let e = self.can_seen.entry(f.id).or_default();
            e.0 = f.data.clone();
            e.1 += 1;
            self.can_out.push(f);
        }
        self.cpu
            .line
            .forget_before(self.cpu.cycles.saturating_sub(fcpu()));
        // far behind wall time (a paused process): skip ahead
        if wall > self.cpu.cycles + fcpu() / 5 && self.hold.is_none() {
            self.on_virt += (wall - self.cpu.cycles) as f64 / fcpu() as f64;
        }
        self.rx_bytes += out.len() as u64;
        out
    }
}

pub struct Server {
    rom: Vec<u8>,
    controls: Controls,
    plant: Plant,
    ecu: Option<Ecu>,
    clients: BTreeMap<u64, Client>,
    /// ABS, traction control and airbag models on their own lines.
    mods: Vec<vehicle::Module>,
    /// Internal tester for dumps whose RAM layout is not mapped.
    probe: probe::Probe,
    /// The codes this dump stores per (ADC channel, 0 or 0x3FF), from a
    /// `tools/fault_sweep.py --csv` profile beside the dump, if there is one.
    profile: BTreeMap<(usize, u16), String>,
    /// Virtual time in seconds: wall time times the time factor (1x while
    /// a tester is connected, whose clock must match).
    virt: f64,
    /// Saved states by name (in memory only: ECU RAM is dump-derived).
    snapshots: BTreeMap<String, Snapshot>,
    /// Intermittent faults: whether each is on now, and until when (virt).
    flicker: BTreeMap<Fault, (bool, f64)>,
    rng: u64,
}

#[derive(Clone)]
struct Snapshot {
    ecu: Option<Ecu>,
    controls: Controls,
    plant: Plant,
    mods: Vec<vehicle::Module>,
    virt: f64,
}

impl Server {
    /// Load a fault profile (`adc,sensor,fault,codes` CSV from
    /// `tools/fault_sweep.py --csv`).
    pub fn with_profile(mut self, csv: &str) -> Self {
        for line in csv.lines().skip(1) {
            let f: Vec<&str> = line.split(',').collect();
            if let [adc, _, kind, codes] = f[..]
                && let Ok(ch) = adc.parse::<usize>()
            {
                let v = if kind.starts_with("short") { 0 } else { 0x3ff };
                self.profile.insert((ch, v), codes.to_string());
            }
        }
        self
    }

    pub fn new(rom: Vec<u8>) -> Self {
        FCPU_HZ.store(fcpu_for(&rom), std::sync::atomic::Ordering::Relaxed);
        let family = crate::plant::Family::for_rom(&rom);
        Self {
            rom,
            controls: Controls {
                family,
                ..Default::default()
            },
            plant: Plant::default(),
            ecu: None,
            clients: BTreeMap::new(),
            mods: vehicle::modules(),
            probe: probe::Probe::default(),
            profile: BTreeMap::new(),
            virt: 0.0,
            snapshots: BTreeMap::new(),
            flicker: BTreeMap::new(),
            rng: 0x2545_f491_4f6c_dd1d,
        }
    }

    /// Model time now (10,000 cycles per ms since the server started).
    fn model_now(&self) -> u64 {
        vehicle::now(self.virt * 1000.0)
    }

    fn handle(&mut self, id: u64, m: ToServer) {
        let now = self.ecu.as_ref().map_or(0, |e| e.cpu.cycles);
        let mnow = self.model_now();
        let Some(c) = self.clients.get_mut(&id) else {
            return;
        };
        if std::env::var_os("ECUSIM_DEBUG").is_some() {
            eprintln!(
                "client {id} line {} ign {}: {}",
                c.line,
                self.controls.ignition,
                m.line()
            );
        }
        // a tester on another line talks to the models, not the engine ECU
        if c.line != 7 && self.controls.ignition {
            match m {
                ToServer::Tx { us, bytes } => {
                    let t = c.model_time(us, mnow);
                    let byte = 10 * 10_000_000 / c.baud.max(1) as u64; // 10 bits
                    let mut rx = Vec::new();
                    for m in self.mods.iter_mut().filter(|m| m.on(c.line)) {
                        for (i, &b) in bytes.iter().enumerate() {
                            m.ecu.send_byte(t + i as u64 * byte, b, &mut rx);
                        }
                    }
                    return;
                }
                ToServer::Slow { us, addr } => {
                    let t = c.model_time(us, mnow);
                    vehicle::slow_init(&mut self.mods, c.line, addr, t);
                    return;
                }
                ToServer::Low { .. } | ToServer::High { .. } => return,
                _ => {}
            }
        }
        if c.line == 7
            && matches!(
                m,
                ToServer::Tx { .. } | ToServer::Low { .. } | ToServer::High { .. }
            )
        {
            self.probe.outside_activity();
        }
        match m {
            ToServer::Line(l) => c.line = l,
            ToServer::Slow { .. } => {}
            ToServer::Set { name, value } if name == "snapshot" => {
                let snap = Snapshot {
                    ecu: self.ecu.clone(),
                    controls: self.controls.clone(),
                    plant: self.plant.clone(),
                    mods: self.mods.clone(),
                    virt: self.virt,
                };
                self.snapshots.insert(value, snap);
            }
            ToServer::Set { name, value } if name == "restore" => {
                match self.snapshots.get(&value) {
                    Some(snap) => {
                        let snap = snap.clone();
                        self.ecu = snap.ecu;
                        self.controls = snap.controls;
                        self.plant = snap.plant;
                        self.mods = snap.mods;
                        self.virt = snap.virt;
                        self.probe = probe::Probe::default();
                        for c in self.clients.values_mut() {
                            c.clock = None;
                            c.mclock = None;
                        }
                    }
                    None => c.send(&ToClient::Err(format!("no snapshot {value:?}"))),
                }
            }
            ToServer::Set { name, value } => {
                if let Err(e) = self.controls.set(&name, &value) {
                    c.send(&ToClient::Err(e));
                }
            }
            ToServer::Watch => c.watch = true,
            ToServer::KLine => c.kline = true,
            ToServer::CanSub => c.can = true,
            ToServer::Can { id, data } => {
                c.can = true;
                if let Some(e) = &mut self.ecu
                    && !self.controls.faults.contains(&Fault::CanBusDead)
                {
                    e.bus.inject.push(cascade_can::Frame { id, data });
                }
            }
            ToServer::Baud(b) => c.baud = b.max(1),
            ToServer::Tx { .. } | ToServer::Low { .. } | ToServer::High { .. }
                if !self.controls.ignition =>
            {
                // key off: the ECU's K-line interface is unpowered
                if let Some(e) = &mut self.ecu {
                    e.hold = None;
                }
            }
            ToServer::Tx { us, bytes } => {
                let t = c.ecu_time(us, now);
                let bit = fcpu() / c.baud.max(1) as u64;
                if let Some(e) = &mut self.ecu {
                    for (i, &b) in bytes.iter().enumerate() {
                        e.cpu.line.frame(t + i as u64 * 10 * bit, b, bit);
                    }
                }
            }
            ToServer::Low { us } => {
                let t = c.ecu_time(us, now);
                if let Some(e) = &mut self.ecu {
                    e.hold = Some(t);
                }
            }
            ToServer::High { us } => {
                let t = c.ecu_time(us, now);
                if let Some(e) = &mut self.ecu
                    && let Some(s) = e.hold.take()
                {
                    e.cpu.line.pull_low(s, t);
                }
            }
        }
    }

    /// One pass of the ECU loop, `dt` seconds after the previous one.
    /// The controls with intermittent faults applied: each is on for 4-10 s
    /// at random moments 5-20 s apart (shorter bursts are healed by the ECU
    /// before it stores a code).
    fn effective_controls(&mut self) -> Controls {
        let mut c = self.controls.clone();
        self.flicker.retain(|f, _| c.intermittent.contains(f));
        for &f in &c.intermittent.clone() {
            // xorshift: reproducible, no dependency
            let mut rand = |lo: f64, hi: f64| {
                self.rng ^= self.rng << 13;
                self.rng ^= self.rng >> 7;
                self.rng ^= self.rng << 17;
                lo + (self.rng % 10_000) as f64 / 10_000.0 * (hi - lo)
            };
            let (on, until) = *self.flicker.entry(f).or_insert((false, self.virt + 2.0));
            if self.virt >= until {
                let next = if on { rand(5.0, 20.0) } else { rand(4.0, 10.0) };
                self.flicker.insert(f, (!on, self.virt + next));
            }
            if self.flicker[&f].0 {
                c.faults.insert(f);
            }
        }
        c
    }

    fn tick(&mut self, dt: f64) {
        // closed loop on the ECU's own outputs when the dump is mapped
        let outputs = match &self.ecu {
            Some(e) if !self.controls.open_loop_only && self.controls.battery_connected => {
                ecu_view::outputs(&e.cpu).map(|o| (o, self.virt - e.inj_at < 0.3))
            }
            _ => None,
        };
        match outputs {
            Some((o, fueled)) => self.plant.step_closed(&self.controls, dt, &o, fueled),
            None => {
                self.plant.closed_loop = false;
                self.plant.step(&self.controls, dt);
            }
        }
        self.controls.stop_request = false;
        // the ECU lives on the battery (KL30): it keeps its memory, fault
        // codes included, across key cycles [assumed: stands in for the
        // dump's non-volatile store, which is not emulated]
        match (self.controls.battery_connected, self.ecu.is_some()) {
            (true, false) => self.ecu = Some(Ecu::new(&self.rom, self.virt)),
            (false, true) => self.ecu = None,
            _ => {}
        }
        let eff = self.effective_controls();
        let s = self.plant.sensors(&eff);
        let Some(e) = &mut self.ecu else {
            return;
        };
        for &(ch, v) in &s.adc {
            e.cpu.adc[ch] = v;
        }
        e.cpu.wheels[0].rpm = self.plant.rpm;
        let cam_lost = eff.faults.contains(&Fault::CamMissing);
        if cam_lost != e.cpu.wheels[0].cam.is_none() {
            e.cpu.wheels[0].cam = if cam_lost {
                None
            } else {
                wheels().remove(0).cam
            };
        }
        let input = speed_input(&e.cpu);
        let kmh = self.controls.vehicle_kmh;
        e.cpu.wheels[1].rpm = if input == Some("cc30") {
            kmh * VSS_HZ_PER_KMH_CC30 * 60.0
        } else {
            0.0
        };
        e.cpu.wheels[2].rpm = if input == Some("cc13") {
            kmh * VSS_HZ_PER_KMH_CC13 * 60.0
        } else {
            0.0
        };
        // the other CAN nodes follow the car; a subscribed client is a
        // node too (it acknowledges frames)
        let dead = eff.faults.contains(&Fault::CanBusDead);
        e.bus.esp = !dead && !eff.faults.contains(&Fault::CanEspSilent);
        e.bus.gearbox = !dead && !eff.faults.contains(&Fault::CanGearboxSilent);
        e.bus.kmh = kmh;
        let listeners = self.clients.values().any(|c| c.can);
        let acked = e.bus.esp || e.bus.gearbox || (listeners && !dead);
        if acked != e.cpu.can.acked {
            let now = e.cpu.cycles;
            e.cpu.can.set_acked(now, acked);
        }
        // a held line (a lost HIGH) must not stall the ECU for long
        let wall = ((self.virt - e.on_virt).max(0.0) * fcpu() as f64) as u64;
        if e.hold.is_some_and(|h| wall > h + fcpu()) {
            e.hold = None;
        }
        let rx = e.run(self.virt, fcpu() / 50);
        let frames = std::mem::take(&mut e.can_out);
        if !frames.is_empty() {
            for c in self.clients.values_mut().filter(|c| c.can) {
                for f in &frames {
                    c.send(&ToClient::Can {
                        id: f.id,
                        data: f.data.clone(),
                    });
                }
            }
        }
        // dumps without a mapped RAM layout are read over the K-line
        if self.controls.ignition && !ecu_view::layout_known(&e.cpu) {
            self.probe.tick(&mut e.cpu, fcpu(), &rx);
        }
        // with the key off the ECU does not answer on the K-line; the
        // internal tester's session is not shown to outside testers
        if !rx.is_empty() && self.controls.ignition && !self.probe.active() {
            let m = ToClient::Rx(rx);
            for c in self.clients.values_mut().filter(|c| c.kline && c.line == 7) {
                c.send(&m);
            }
        }
        self.tick_modules(&eff);
    }

    /// The ABS, TCS and SRS models: follow the car, answer their lines.
    fn tick_modules(&mut self, eff: &Controls) {
        vehicle::update(&mut self.mods, &self.plant, eff);
        let now = self.model_now();
        for m in self.mods.iter_mut() {
            let mut rx = Vec::new();
            m.ecu.poll_input(now, &mut rx);
            if rx.is_empty() || !self.controls.ignition {
                continue;
            }
            let msg = ToClient::Rx(rx);
            for c in self
                .clients
                .values_mut()
                .filter(|c| c.kline && m.on(c.line))
            {
                c.send(&msg);
            }
        }
    }

    fn status(&mut self, period_s: f64) -> ToClient {
        let c = &self.controls;
        let s = self.plant.sensors(c);
        let on = |b: bool| if b { "on" } else { "off" }.to_string();
        let mut kv: Vec<(String, String)> = vec![
            ("ignition".into(), on(c.ignition)),
            ("abs_dtcs".into(), vehicle::codes_text(&self.mods[0])),
            ("tcs_dtcs".into(), vehicle::codes_text(&self.mods[1])),
            ("srs_dtcs".into(), vehicle::codes_text(&self.mods[2])),
            ("battery_connected".into(), on(c.battery_connected)),
            ("time".into(), format!("{:.0}", c.time_factor)),
            (
                "intermittent".into(),
                if c.intermittent.is_empty() {
                    "none".into()
                } else {
                    c.intermittent
                        .iter()
                        .map(|f| f.name())
                        .collect::<Vec<_>>()
                        .join(",")
                },
            ),
            (
                "snapshots".into(),
                if self.snapshots.is_empty() {
                    "none".into()
                } else {
                    self.snapshots.keys().cloned().collect::<Vec<_>>().join(",")
                },
            ),
            ("family".into(), format!("{:?}", c.family).to_lowercase()),
            (
                // this dump's codes per fault, when a profile was loaded
                "fault_codes".into(),
                if self.profile.is_empty() {
                    "default".into()
                } else {
                    Fault::ALL
                        .iter()
                        .filter_map(|f| {
                            let k = f.adc(c.family)?;
                            Some(format!(
                                "{}:{}",
                                f.name(),
                                self.profile
                                    .get(&k)
                                    .map_or("none", String::as_str)
                                    .replace(' ', "+")
                            ))
                        })
                        .collect::<Vec<_>>()
                        .join(",")
                },
            ),
            (
                "faults".into(),
                if c.faults.is_empty() {
                    "none".into()
                } else {
                    c.faults
                        .iter()
                        .map(|f| f.name())
                        .collect::<Vec<_>>()
                        .join(",")
                },
            ),
            ("starter".into(), on(c.starter)),
            ("engine".into(), self.plant.engine.name().into()),
            (
                "model".into(),
                if self.plant.closed_loop {
                    "closed"
                } else {
                    "open"
                }
                .into(),
            ),
            (
                "lambda".into(),
                format!("{:.2}", self.plant.lambda.min(9.9)),
            ),
            ("rpm".into(), format!("{:.0}", self.plant.rpm)),
            ("throttle".into(), format!("{:.0}", c.throttle)),
            ("coolant".into(), format!("{:.1}", c.coolant_c)),
            ("intake".into(), format!("{:.1}", c.intake_c)),
            ("battery_set".into(), format!("{:.1}", c.battery_v)),
            ("battery".into(), format!("{:.2}", s.battery_v)),
            ("map".into(), format!("{:.0}", s.map_kpa)),
            ("o2_front".into(), format!("{:.0}", s.o2_front_mv)),
            ("speed".into(), format!("{:.0}", c.vehicle_kmh)),
            ("idle".into(), format!("{:.0}", c.idle_rpm)),
            (
                "adc".into(),
                s.adc
                    .iter()
                    .map(|(ch, v)| format!("{ch}:{v}"))
                    .collect::<Vec<_>>()
                    .join(","),
            ),
            (
                "testers".into(),
                self.clients
                    .values()
                    .filter(|c| c.kline)
                    .count()
                    .to_string(),
            ),
        ];
        match &mut self.ecu {
            None => kv.push(("ecu".into(), "off".into())),
            Some(e) => {
                let state = match &e.stopped {
                    Some(why) => format!("stopped:{}", why.replace(' ', "_")),
                    None => "on".into(),
                };
                kv.push(("ecu".into(), state));
                kv.push((
                    "ecu_s".into(),
                    format!("{:.1}", e.cpu.cycles as f64 / fcpu() as f64),
                ));
                kv.push((
                    "spark_hz".into(),
                    format!("{:.0}", e.spark as f64 / period_s),
                ));
                kv.push(("inj_hz".into(), format!("{:.0}", e.inj as f64 / period_s)));
                kv.push(("rx_bytes".into(), e.rx_bytes.to_string()));
                // the engine CAN bus: the ECU's frames (id:count:data)
                let nodes: Vec<&str> = [(e.bus.esp, "esp"), (e.bus.gearbox, "gearbox")]
                    .iter()
                    .filter(|(on, _)| *on)
                    .map(|(_, n)| *n)
                    .collect();
                kv.push((
                    "can_nodes".into(),
                    if nodes.is_empty() {
                        "none".into()
                    } else {
                        nodes.join(",")
                    },
                ));
                kv.push((
                    "can".into(),
                    if e.can_seen.is_empty() {
                        "none".into()
                    } else {
                        e.can_seen
                            .iter()
                            .map(|(id, (d, n))| format!("{id:03X}:{n}:{}", crate::proto::hex(d)))
                            .collect::<Vec<_>>()
                            .join(",")
                    },
                ));
                // decoded broadcasts: engine speed 0.25 rpm/bit [verified
                // against the crank wheel in the emulator; the database
                // says 0.15625], coolant 0.75 x - 48 and battery 0.1 V
                // [database]
                if let Some((d, _)) = e.can_seen.get(&0x316)
                    && d.len() >= 4
                {
                    let rpm = u16::from_le_bytes([d[2], d[3]]) as f64 * 0.25;
                    kv.push(("can_rpm".into(), format!("{rpm:.0}")));
                }
                if let Some((d, _)) = e.can_seen.get(&0x329)
                    && d.len() >= 2
                {
                    kv.push((
                        "can_coolant".into(),
                        format!("{:.1}", d[1] as f64 * 0.75 - 48.0),
                    ));
                }
                if let Some((d, _)) = e.can_seen.get(&0x545)
                    && d.len() >= 4
                {
                    kv.push(("can_battery".into(), format!("{:.1}", d[3] as f64 * 0.1)));
                }
                // what the ECU itself holds (addresses known for one dump)
                if ecu_view::layout_known(&e.cpu) {
                    let (rpm, kmh, coolant, battery) = ecu_view::readings(&e.cpu);
                    kv.push(("ecu_rpm".into(), rpm.to_string()));
                    kv.push(("ecu_speed".into(), kmh.to_string()));
                    kv.push(("ecu_coolant".into(), format!("{coolant:.1}")));
                    kv.push(("ecu_battery".into(), format!("{battery:.1}")));
                    let codes = ecu_view::stored_codes(&e.cpu);
                    kv.push((
                        "dtcs".into(),
                        if codes.is_empty() {
                            "none".into()
                        } else {
                            codes
                                .iter()
                                .map(|c| format!("{}:{:02X}", c.code, c.status))
                                .collect::<Vec<_>>()
                                .join(",")
                        },
                    ));
                } else {
                    // read over the K-line by the internal tester
                    let r = &self.probe.readings;
                    let opt =
                        |v: Option<f64>, d: usize| v.map_or("–".into(), |x| format!("{x:.d$}"));
                    kv.push(("ecu_rpm".into(), opt(r.rpm, 0)));
                    kv.push(("ecu_speed".into(), opt(r.speed_kmh, 0)));
                    kv.push(("ecu_coolant".into(), opt(r.coolant_c, 0)));
                    kv.push(("ecu_battery".into(), "–".into()));
                    kv.push((
                        "dtcs".into(),
                        match r.at {
                            None => "reading".into(),
                            Some(_) if r.codes.is_empty() => "none".into(),
                            Some(_) => r
                                .codes
                                .iter()
                                .map(|&(c, st)| format!("{}:{st:02X}", ecu_view::dtc_name(c)))
                                .collect::<Vec<_>>()
                                .join(","),
                        },
                    ));
                    kv.push(("ecu_source".into(), "kline".into()));
                }
                kv.push((
                    "speed_input".into(),
                    speed_input(&e.cpu).unwrap_or("none").into(),
                ));
                e.spark = 0;
                e.inj = 0;
            }
        }
        ToClient::Status(kv)
    }

    /// Serve forever on `socket` (an existing socket file is replaced).
    pub fn serve(mut self, socket: &Path) -> io::Result<()> {
        let _ = std::fs::remove_file(socket);
        let listener = UnixListener::bind(socket)?;
        let (tx, rx) = mpsc::channel();
        std::thread::spawn(move || accept(listener, tx));
        self.run_loop(rx);
        Ok(())
    }

    fn run_loop(&mut self, rx: Receiver<Msg>) {
        let mut last = Instant::now();
        let mut last_status = Instant::now();
        loop {
            while let Ok(m) = rx.try_recv() {
                match m {
                    Msg::Connect(id, s) => {
                        self.clients.insert(
                            id,
                            Client {
                                stream: Some(s),
                                baud: 10_400,
                                line: 7,
                                ..Default::default()
                            },
                        );
                    }
                    Msg::Disconnect(id) => {
                        self.clients.remove(&id);
                    }
                    Msg::Cmd(id, m) => self.handle(id, m),
                }
            }
            // fast-forward only while no tester is connected: a tester's own
            // clock runs at 1x
            let factor = if self.clients.values().any(|c| c.kline) {
                1.0
            } else {
                self.controls.time_factor
            };
            let dt = last.elapsed().as_secs_f64() * factor;
            last = Instant::now();
            self.virt += dt;
            self.tick(dt);
            let period = last_status.elapsed().as_secs_f64();
            if period >= 0.1 {
                last_status = Instant::now();
                let st = self.status(period);
                for c in self.clients.values_mut().filter(|c| c.watch) {
                    c.send(&st);
                }
            }
            std::thread::sleep(Duration::from_micros(500));
        }
    }
}

fn accept(listener: UnixListener, tx: Sender<Msg>) {
    for (id, stream) in listener.incoming().enumerate() {
        let Ok(stream) = stream else {
            continue;
        };
        let id = id as u64;
        let Ok(w) = stream.try_clone() else {
            continue;
        };
        if tx.send(Msg::Connect(id, w)).is_err() {
            return;
        }
        let tx = tx.clone();
        std::thread::spawn(move || {
            let mut err = stream.try_clone().ok();
            for line in BufReader::new(stream).lines() {
                let Ok(line) = line else {
                    break;
                };
                if line.trim().is_empty() {
                    continue;
                }
                match ToServer::parse(&line) {
                    Ok(m) => {
                        if tx.send(Msg::Cmd(id, m)).is_err() {
                            return;
                        }
                    }
                    Err(e) => {
                        if let Some(s) = &mut err {
                            let _ = writeln!(s, "{}", ToClient::Err(e).line());
                        }
                    }
                }
            }
            let _ = tx.send(Msg::Disconnect(id));
        });
    }
}

/// Read a ROM file (256 KB SIMK41 dump expected).
pub fn load_rom(path: &PathBuf) -> io::Result<Vec<u8>> {
    let rom = std::fs::read(path)?;
    if rom.len() != 256 * 1024 && rom.len() != 512 * 1024 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!(
                "{}: {} bytes, expected a 256 KB SIMK41 or 512 KB SIMK43 dump",
                path.display(),
                rom.len()
            ),
        ));
    }
    Ok(rom)
}

#[cfg(test)]
mod tests {
    #[test]
    fn clock_follows_the_dump_size() {
        assert_eq!(super::fcpu_for(&vec![0; 256 * 1024]), 16_000_000);
        assert_eq!(super::fcpu_for(&vec![0; 512 * 1024]), 24_000_000);
    }
}
