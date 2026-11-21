//! Native live data (kdiag's client) for the GUI: a worker thread polls
//! `21 01` from the engine ECU and decodes it with the protocol CSV. No
//! emulator involved. Every request passes kdiag's read-only guard.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::Duration;

use cascade_diag::defs::{self, Value};
use cascade_diag::kwp::Client;
use cascade_diag::line::Line;
use cascade_diag::sim::{SimLine, engine_demo_config};
use cascade_iface::KwpEcu;

#[derive(Clone, Debug, PartialEq)]
pub enum Source {
    /// The ECU model in virtual time, with a moving demo engine.
    Sim,
    /// A serial K-line adapter.
    Port { path: String, echo: bool },
}

#[derive(Clone, Debug, Default)]
pub struct LiveSnap {
    pub running: bool,
    pub status: String,
    /// (name, value, unit)
    pub rows: Vec<(String, String, String)>,
    pub reads: u64,
    pub errors: u64,
    pub seq: u64,
}

pub struct Live {
    state: Arc<Mutex<LiveSnap>>,
    stop: Arc<AtomicBool>,
    handle: Option<JoinHandle<()>>,
}

fn fmt_value(v: Option<Value>) -> String {
    match v {
        Some(Value::Number(x)) if x.abs() >= 100.0 => format!("{x:.0}"),
        Some(Value::Number(x)) if x.abs() >= 10.0 => format!("{x:.1}"),
        Some(Value::Number(x)) => format!("{x:.2}"),
        Some(Value::Flag(f)) => if f { "ON" } else { "OFF" }.into(),
        None => "-".into(),
    }
}

impl Live {
    pub fn start(source: Source) -> Self {
        let state = Arc::new(Mutex::new(LiveSnap {
            running: true,
            status: "connecting…".into(),
            ..Default::default()
        }));
        let stop = Arc::new(AtomicBool::new(false));
        let (st, sp) = (state.clone(), stop.clone());
        let handle = std::thread::Builder::new()
            .name("live-data".into())
            .spawn(move || run(source, st, sp))
            .ok();
        Self {
            state,
            stop,
            handle,
        }
    }

    pub fn snapshot(&self) -> LiveSnap {
        self.state.lock().map(|s| s.clone()).unwrap_or_default()
    }

    pub fn seq(&self) -> u64 {
        self.state.lock().map(|s| s.seq).unwrap_or(0)
    }
}

impl Drop for Live {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

fn run(source: Source, state: Arc<Mutex<LiveSnap>>, stop: Arc<AtomicBool>) {
    let set = |f: &dyn Fn(&mut LiveSnap)| {
        if let Ok(mut s) = state.lock() {
            f(&mut s);
            s.seq += 1;
        }
    };
    let params = match defs::parse_csv(defs::TIBURON_2003_ENGINE_L4) {
        Ok(p) => p,
        Err(e) => {
            set(&|s| {
                s.running = false;
                s.status = format!("definitions: {e}");
            });
            return;
        }
    };
    let line: Box<dyn Line> = match &source {
        Source::Sim => Box::new(SimLine::new(KwpEcu::new(engine_demo_config()))),
        Source::Port { path, echo } => match cascade_diag::serial::SerialLine::open(path, *echo) {
            Ok(l) => Box::new(l),
            Err(e) => {
                let msg = format!("{path}: {e}");
                set(&|s| {
                    s.running = false;
                    s.status = msg.clone();
                });
                return;
            }
        },
    };
    let mut c = Client::new(line);
    let mut session = false;
    let mut errors_in_row = 0;
    while !stop.load(Ordering::Relaxed) {
        if !session {
            match c.start() {
                Ok(kb) => {
                    session = true;
                    let msg = format!("session up, key bytes {:02X} {:02X}", kb[0], kb[1]);
                    set(&|s| s.status = msg.clone());
                }
                Err(e) => {
                    let msg = format!("no session: {e}; retrying");
                    set(&|s| {
                        s.status = msg.clone();
                        s.errors += 1;
                    });
                    std::thread::sleep(Duration::from_secs(1));
                    continue;
                }
            }
        }
        match c.read_local(0x01) {
            Ok(block) => {
                errors_in_row = 0;
                // read_local returns the block after `61 01`
                let rows: Vec<_> = params
                    .iter()
                    .map(|p| (p.name.clone(), fmt_value(p.decode(&block)), p.unit.clone()))
                    .collect();
                set(&|s| {
                    s.rows = rows.clone();
                    s.reads += 1;
                });
            }
            Err(e) => {
                errors_in_row += 1;
                let msg = format!("read failed: {e}");
                set(&|s| {
                    s.status = msg.clone();
                    s.errors += 1;
                });
                if errors_in_row >= 3 {
                    session = false;
                    errors_in_row = 0;
                }
            }
        }
        std::thread::sleep(Duration::from_millis(150));
    }
    if session {
        let _ = c.stop();
    }
    set(&|s| {
        s.running = false;
        s.status = "stopped".into();
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sim_source_produces_decoded_rows() {
        let live = Live::start(Source::Sim);
        let t = std::time::Instant::now();
        let mut snap = live.snapshot();
        while snap.reads < 3 && t.elapsed() < Duration::from_secs(5) {
            std::thread::sleep(Duration::from_millis(50));
            snap = live.snapshot();
        }
        assert!(snap.reads >= 3, "{snap:?}");
        assert!(snap.status.starts_with("session up"), "{}", snap.status);
        let rpm = snap
            .rows
            .iter()
            .find(|r| r.0 == "engine_speed")
            .expect("engine_speed row");
        assert!(rpm.1.parse::<f64>().unwrap() >= 800.0, "{rpm:?}");
        drop(live);
    }
}
