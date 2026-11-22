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
use crate::proto::{ToClient, ToServer};

/// CPU clock (the dump's UART and 1 ms tick fit 16 MHz).
pub const FCPU: u64 = 16_000_000;

/// Crank wheel: 60-2 on P2.15 (CC15 capture and T7IN) and GPT2 CAPIN; cam
/// level on P2.14 (CC14). Found by running the dump: with these the ECU's
/// engine speed follows the wheel (it needs `Cpu::compare_on_write`, on by
/// default, to keep sync across the gap). Road speed: one pulse wheel per known input
/// (CC30 on the GK-20 dump, CC13 on the non-immobiliser one); only the one
/// the running software sets up turns.
fn wheels() -> Vec<Wheel> {
    let mut crank = Wheel::new(
        vec![Pin::Cc { unit: 0, ch: 15 }, Pin::T7In, Pin::CapIn],
        60,
        2,
    );
    crank.cam = Some(Cam {
        targets: vec![Pin::Cc { unit: 0, ch: 14 }],
        // low for half the cycle: the pattern checked to set no cam code
        // (low only around one gap sets P0340)
        fall: 30,
        rise: 90,
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
}

impl Client {
    fn send(&mut self, m: &ToClient) {
        if let Some(s) = &mut self.stream
            && writeln!(s, "{}", m.line()).is_err()
        {
            self.stream = None;
        }
    }

    /// ECU time for the client's `us`, never in the ECU's past.
    fn ecu_time(&mut self, us: u64, now: u64) -> u64 {
        let t = match self.clock {
            Some((u0, t0)) if us >= u0 => t0 + (us - u0) * (FCPU / 1_000_000),
            _ => now,
        };
        // more than a second off: the client paused; start again from now
        let t = if t < now || t > now + FCPU { now } else { t };
        self.clock = Some((us, t));
        t
    }
}

struct Ecu {
    cpu: Cpu,
    on_at: Instant,
    /// Tester pulled K low at this ECU time; the ECU waits there.
    hold: Option<u64>,
    stopped: Option<String>,
    /// Compare matches in the last status period: CC12 (ignition events)
    /// and CC0-CC3 (injection events) [assumed roles].
    spark: u64,
    inj: u64,
    rx_bytes: u64,
}

impl Ecu {
    fn new(rom: &[u8]) -> Self {
        let mut cpu = Cpu::new(rom.to_vec(), FCPU);
        cpu.wheels = wheels();
        cpu.log_compares = true;
        Self {
            cpu,
            on_at: Instant::now(),
            hold: None,
            stopped: None,
            spark: 0,
            inj: 0,
            rx_bytes: 0,
        }
    }

    /// Run up to wall time (or the hold point), at most `max` cycles.
    /// Returns bytes the ECU sent at K-line speed.
    fn run(&mut self, max: u64) -> Vec<u8> {
        let wall = (self.on_at.elapsed().as_nanos() * FCPU as u128 / 1_000_000_000) as u64;
        let mut target = wall.min(self.cpu.cycles + max);
        if let Some(h) = self.hold {
            target = target.min(h);
        }
        let mut out = Vec::new();
        while self.stopped.is_none() && self.cpu.cycles < target {
            if let Err(e) = self.cpu.step() {
                self.stopped = Some(e.to_string());
            }
            let fast = self.cpu.bit_cycles() < 2_000;
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
                    } => self.inj += 1,
                    _ => {}
                }
            }
        }
        self.cpu
            .line
            .forget_before(self.cpu.cycles.saturating_sub(FCPU));
        // far behind wall time (a paused process): skip ahead
        if wall > self.cpu.cycles + FCPU / 5 && self.hold.is_none() {
            self.on_at += Duration::from_nanos((wall - self.cpu.cycles) * 1_000_000_000 / FCPU);
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
}

impl Server {
    pub fn new(rom: Vec<u8>) -> Self {
        Self {
            rom,
            controls: Controls::default(),
            plant: Plant::default(),
            ecu: None,
            clients: BTreeMap::new(),
        }
    }

    fn handle(&mut self, id: u64, m: ToServer) {
        let now = self.ecu.as_ref().map_or(0, |e| e.cpu.cycles);
        let Some(c) = self.clients.get_mut(&id) else {
            return;
        };
        match m {
            ToServer::Set { name, value } => {
                if let Err(e) = self.controls.set(&name, &value) {
                    c.send(&ToClient::Err(e));
                }
            }
            ToServer::Watch => c.watch = true,
            ToServer::KLine => c.kline = true,
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
                let bit = FCPU / c.baud.max(1) as u64;
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
    fn tick(&mut self, dt: f64) {
        self.plant.step(&self.controls, dt);
        self.controls.stop_request = false;
        // the ECU lives on the battery (KL30): it keeps its memory, fault
        // codes included, across key cycles [assumed: stands in for the
        // dump's non-volatile store, which is not emulated]
        match (self.controls.battery_connected, self.ecu.is_some()) {
            (true, false) => self.ecu = Some(Ecu::new(&self.rom)),
            (false, true) => self.ecu = None,
            _ => {}
        }
        let s = self.plant.sensors(&self.controls);
        let Some(e) = &mut self.ecu else {
            return;
        };
        for &(ch, v) in &s.adc {
            e.cpu.adc[ch] = v;
        }
        e.cpu.wheels[0].rpm = self.plant.rpm;
        let cam_lost = self.controls.faults.contains(&Fault::CamMissing);
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
        // a held line (a lost HIGH) must not stall the ECU for long
        let wall = (e.on_at.elapsed().as_nanos() * FCPU as u128 / 1_000_000_000) as u64;
        if e.hold.is_some_and(|h| wall > h + FCPU) {
            e.hold = None;
        }
        let rx = e.run(FCPU / 50);
        // with the key off the ECU does not answer on the K-line
        if !rx.is_empty() && self.controls.ignition {
            let m = ToClient::Rx(rx);
            for c in self.clients.values_mut().filter(|c| c.kline) {
                c.send(&m);
            }
        }
    }

    fn status(&mut self, period_s: f64) -> ToClient {
        let c = &self.controls;
        let s = self.plant.sensors(c);
        let on = |b: bool| if b { "on" } else { "off" }.to_string();
        let mut kv: Vec<(String, String)> = vec![
            ("ignition".into(), on(c.ignition)),
            ("battery_connected".into(), on(c.battery_connected)),
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
                    format!("{:.1}", e.cpu.cycles as f64 / FCPU as f64),
                ));
                kv.push((
                    "spark_hz".into(),
                    format!("{:.0}", e.spark as f64 / period_s),
                ));
                kv.push(("inj_hz".into(), format!("{:.0}", e.inj as f64 / period_s)));
                kv.push(("rx_bytes".into(), e.rx_bytes.to_string()));
                // what the ECU itself holds
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
            let dt = last.elapsed().as_secs_f64();
            last = Instant::now();
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
    if rom.len() != 256 * 1024 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!(
                "{}: {} bytes, expected a 256 KB SIMK41 dump",
                path.display(),
                rom.len()
            ),
        ));
    }
    Ok(rom)
}
