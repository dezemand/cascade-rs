//! ecusim-gui: the car's controls for an `ecusim` ECU process.
//!
//! ```text
//! ecusim-gui [--rom SIMK41.bin] [--socket PATH]
//! ```
//!
//! Connects to the ECU process on its socket (default `$TMPDIR/ecusim.sock`).
//! With `--rom`, and no server answering, it starts `ecusim serve` itself
//! (the `ecusim` binary next to this one) and stops it on exit.
//!
//! Ignition, a hold-to-crank START button, throttle, temperatures, battery,
//! road speed and idle speed go to the engine model; the panel shows what
//! the model does and what the ECU reports about itself. Testers (the
//! Hi-Scan GUI's "Process" line, `kdiag --ecusim auto`) connect to the same
//! socket.

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::process::Command;
use std::time::Duration;

use ecusim::client::Conn;
use ecusim::plant::Fault;
use ecusim::proto::{ToClient, ToServer};
use gpui_kit::component::slider::{Slider, SliderEvent, SliderState};
use gpui_kit::prelude::FluentBuilder as _;
use gpui_kit::*;

const BG: u32 = 0x0e0f11;
const CARD: u32 = 0x17181b;
const EDGE: u32 = 0x25272c;
const TEXT: u32 = 0xe7e9ec;
const DIM: u32 = 0x8a8f98;
const ACCENT: u32 = 0xf5b82e;
const ACCENT_TEXT: u32 = 0x1d1606;
const OK: u32 = 0x4ade80;
const WARN: u32 = 0xf87171;

/// A slider bound to one control.
struct Control {
    name: &'static str,
    label: &'static str,
    unit: &'static str,
    state: Entity<SliderState>,
    decimals: usize,
}

struct Panel {
    socket: PathBuf,
    conn: Option<Conn>,
    status: BTreeMap<String, String>,
    last_error: Option<String>,
    controls: Vec<Control>,
    cranking: bool,
    _subs: Vec<Subscription>,
}

impl Panel {
    fn new(socket: PathBuf, cx: &mut Context<Self>) -> Self {
        // name, label, unit, min, max, step, default, decimals
        #[allow(clippy::type_complexity)]
        let specs: [(&str, &str, &str, f32, f32, f32, f32, usize); 6] = [
            ("throttle", "Throttle", "%", 0., 100., 1., 0., 0),
            ("coolant", "Coolant", "°C", -15., 130., 1., 20., 0),
            ("intake", "Intake air", "°C", -15., 80., 1., 20., 0),
            ("battery", "Battery (rest)", "V", 9., 15., 0.1, 12.6, 1),
            ("speed", "Road speed", "km/h", 0., 200., 1., 0., 0),
            ("idle", "Idle speed", "rpm", 600., 1200., 10., 800., 0),
        ];
        let mut controls = Vec::new();
        let mut subs = Vec::new();
        for (name, label, unit, min, max, step, val, decimals) in specs {
            let state = cx.new(|_| {
                SliderState::new()
                    .min(min)
                    .max(max)
                    .step(step)
                    .default_value(val)
            });
            subs.push(
                cx.subscribe(&state, move |this: &mut Self, _, ev: &SliderEvent, cx| {
                    if let SliderEvent::Change(v) = ev {
                        this.set(name, &format!("{:.*}", decimals, v.start()));
                        cx.notify();
                    }
                }),
            );
            controls.push(Control {
                name,
                label,
                unit,
                state,
                decimals,
            });
        }
        // poll the socket 20 times a second; reconnect when it is gone
        cx.spawn(async move |this, cx| {
            loop {
                cx.background_executor()
                    .timer(Duration::from_millis(50))
                    .await;
                if this.update(cx, |v, cx| v.poll(cx)).is_err() {
                    break;
                }
            }
        })
        .detach();
        Self {
            socket,
            conn: None,
            status: BTreeMap::new(),
            last_error: None,
            controls,
            cranking: false,
            _subs: subs,
        }
    }

    fn connect(&mut self, cx: &mut Context<Self>) {
        match Conn::connect(&self.socket) {
            Ok(mut c) => {
                if c.send(&ToServer::Watch).is_ok() {
                    self.conn = Some(c);
                    self.last_error = None;
                    // the panel's slider values win, so both sides agree
                    let vals: Vec<(&'static str, String)> = self
                        .controls
                        .iter()
                        .map(|c| {
                            (
                                c.name,
                                format!("{:.*}", c.decimals, c.state.read(cx).value().start()),
                            )
                        })
                        .collect();
                    for (n, v) in vals {
                        self.set(n, &v);
                    }
                }
            }
            Err(e) => self.last_error = Some(format!("{}: {e}", self.socket.display())),
        }
    }

    fn poll(&mut self, cx: &mut Context<Self>) {
        if self.conn.is_none() {
            self.connect(cx);
            cx.notify();
            return;
        }
        let mut changed = false;
        loop {
            let m = match self.conn.as_ref().map(|c| c.recv_timeout(Duration::ZERO)) {
                Some(Ok(Some(m))) => m,
                Some(Ok(None)) | None => break,
                Some(Err(e)) => {
                    self.conn = None;
                    self.status.clear();
                    self.last_error = Some(e.to_string());
                    changed = true;
                    break;
                }
            };
            match m {
                ToClient::Status(kv) => {
                    self.status = kv.into_iter().collect();
                    changed = true;
                }
                ToClient::Err(e) => {
                    self.last_error = Some(e);
                    changed = true;
                }
                ToClient::Rx(_) => {}
            }
        }
        if changed {
            cx.notify();
        }
    }

    fn set(&mut self, name: &str, value: &str) {
        if let Some(c) = &mut self.conn
            && c.send(&ToServer::Set {
                name: name.into(),
                value: value.into(),
            })
            .is_err()
        {
            self.conn = None;
        }
    }

    fn get(&self, k: &str) -> &str {
        self.status.get(k).map_or("–", String::as_str)
    }

    fn starter(&mut self, on: bool, cx: &mut Context<Self>) {
        if self.cranking != on {
            self.cranking = on;
            self.set("starter", if on { "on" } else { "off" });
            cx.notify();
        }
    }

    fn stat(&self, label: &'static str, value: String, unit: &'static str) -> impl IntoElement {
        div()
            .flex()
            .flex_col()
            .gap(px(2.))
            .p(px(10.))
            .rounded(px(8.))
            .bg(rgb(CARD))
            .border_1()
            .border_color(rgb(EDGE))
            .min_w(px(110.))
            .child(div().text_xs().text_color(rgb(DIM)).child(label))
            .child(
                div()
                    .flex()
                    .flex_row()
                    .items_baseline()
                    .gap(px(4.))
                    .child(div().text_lg().text_color(rgb(TEXT)).child(value))
                    .child(div().text_xs().text_color(rgb(DIM)).child(unit)),
            )
    }
}

impl Render for Panel {
    fn render(&mut self, _window: &mut Window, cx: &mut Context<Self>) -> impl IntoElement {
        let connected = self.conn.is_some() && !self.status.is_empty();
        let ign = self.get("ignition") == "on";
        let engine = self.get("engine").to_string();
        let ecu = self.get("ecu").to_string();
        let rpm = self.get("rpm").to_string();

        let header = div()
            .flex()
            .flex_row()
            .items_center()
            .justify_between()
            .child(
                div()
                    .flex()
                    .flex_col()
                    .child(div().text_lg().text_color(rgb(TEXT)).child("GK 2.0 engine"))
                    .child(
                        div()
                            .text_xs()
                            .text_color(rgb(DIM))
                            .child("SIMK41 dump in ecusim · open-loop engine model"),
                    ),
            )
            .child(
                div()
                    .flex()
                    .flex_row()
                    .items_center()
                    .gap(px(6.))
                    .child(div().size(px(8.)).rounded_full().bg(rgb(if connected {
                        OK
                    } else {
                        WARN
                    })))
                    .child(div().text_xs().text_color(rgb(DIM)).child(if connected {
                        format!("ECU {ecu}")
                    } else {
                        "not connected".into()
                    })),
            );

        // ignition and start
        let key_btn = div()
            .id("ignition")
            .flex()
            .items_center()
            .justify_center()
            .w(px(150.))
            .h(px(56.))
            .rounded(px(10.))
            .border_1()
            .border_color(rgb(if ign { ACCENT } else { EDGE }))
            .bg(rgb(CARD))
            .text_color(rgb(if ign { ACCENT } else { TEXT }))
            .cursor_pointer()
            .hover(|s| s.bg(rgb(0x1f2024)))
            .child(if ign { "IGNITION ON" } else { "IGNITION OFF" })
            .on_click(cx.listener(move |v, _, _, cx| {
                v.set("ignition", if ign { "off" } else { "on" });
                if ign {
                    v.starter(false, cx);
                }
                cx.notify();
            }));
        let running = matches!(self.get("engine"), "running" | "cranking");
        let stop_btn = div()
            .id("stop")
            .flex()
            .items_center()
            .justify_center()
            .w(px(120.))
            .h(px(56.))
            .rounded(px(10.))
            .border_1()
            .border_color(rgb(EDGE))
            .bg(rgb(CARD))
            .text_color(rgb(TEXT))
            .when(!running, |d| d.opacity(0.4))
            .cursor_pointer()
            .hover(|s| s.bg(rgb(0x1f2024)))
            .child("STOP ENGINE")
            .on_click(cx.listener(move |v, _, _, cx| {
                if running {
                    v.set("engine", "stop");
                }
                cx.notify();
            }));
        let start_btn = div()
            .id("start")
            .flex()
            .flex_col()
            .items_center()
            .justify_center()
            .w(px(150.))
            .h(px(56.))
            .rounded(px(10.))
            .bg(rgb(if self.cranking { 0xd99c17 } else { ACCENT }))
            .text_color(rgb(ACCENT_TEXT))
            .font_weight(FontWeight::SEMIBOLD)
            .when(!ign, |d| d.opacity(0.4))
            .cursor_pointer()
            .child("START")
            .child(div().text_xs().child("hold to crank"))
            .on_mouse_down(
                MouseButton::Left,
                cx.listener(move |v, _, _, cx| {
                    if ign {
                        v.starter(true, cx);
                    }
                }),
            )
            .on_mouse_up(
                MouseButton::Left,
                cx.listener(|v, _, _, cx| v.starter(false, cx)),
            );

        let engine_color = match engine.as_str() {
            "running" => OK,
            "cranking" => ACCENT,
            _ => DIM,
        };
        let big = div()
            .flex()
            .flex_row()
            .items_end()
            .justify_between()
            .p(px(16.))
            .rounded(px(10.))
            .bg(rgb(CARD))
            .border_1()
            .border_color(rgb(EDGE))
            .child(
                div()
                    .flex()
                    .flex_col()
                    .child(
                        div()
                            .text_xs()
                            .text_color(rgb(DIM))
                            .child("ENGINE SPEED (model)"),
                    )
                    .child(
                        div()
                            .flex()
                            .flex_row()
                            .items_baseline()
                            .gap(px(6.))
                            .child(div().text_3xl().text_color(rgb(TEXT)).child(rpm))
                            .child(div().text_sm().text_color(rgb(DIM)).child("rpm")),
                    ),
            )
            .child(
                div()
                    .text_sm()
                    .text_color(rgb(engine_color))
                    .child(engine.to_uppercase()),
            );

        let stats = div()
            .flex()
            .flex_row()
            .flex_wrap()
            .gap(px(8.))
            .child(self.stat("Battery", self.get("battery").into(), "V"))
            .child(self.stat("MAP", self.get("map").into(), "kPa"))
            .child(self.stat("Front O2", self.get("o2_front").into(), "mV"))
            .child(self.stat("Testers", self.get("testers").into(), ""));

        // what the ECU itself holds, read from its memory
        let ecu_on = self.get("ecu") == "on";
        let ecu_val = |k: &str| {
            if ecu_on {
                self.get(k).to_string()
            } else {
                "–".into()
            }
        };
        let ecu_stats = div()
            .flex()
            .flex_row()
            .flex_wrap()
            .gap(px(8.))
            .child(self.stat("Engine speed", ecu_val("ecu_rpm"), "rpm"))
            .child(self.stat("Road speed", ecu_val("ecu_speed"), "km/h"))
            .child(self.stat("Coolant", ecu_val("ecu_coolant"), "°C"))
            .child(self.stat("Battery", ecu_val("ecu_battery"), "V"))
            .child(self.stat("Running for", ecu_val("ecu_s"), "s"));
        let codes: Vec<(String, bool)> = self
            .get("dtcs")
            .split(',')
            .filter_map(|e| e.split_once(':'))
            .map(|(code, st)| {
                let present = u8::from_str_radix(st, 16).is_ok_and(|b| b & 0x40 != 0);
                (code.to_string(), present)
            })
            .collect();
        let code_list = if !ecu_on {
            div()
                .text_sm()
                .text_color(rgb(DIM))
                .child("ECU off (battery disconnected)")
        } else if codes.is_empty() {
            div()
                .text_sm()
                .text_color(rgb(OK))
                .child("No stored fault codes")
        } else {
            codes
                .iter()
                .fold(div().flex().flex_col().gap(px(4.)), |d, (code, present)| {
                    d.child(
                        div()
                            .flex()
                            .flex_row()
                            .justify_between()
                            .child(
                                div()
                                    .font_family("Menlo")
                                    .text_sm()
                                    .text_color(rgb(if *present { WARN } else { TEXT }))
                                    .child(code.clone()),
                            )
                            .child(div().text_xs().text_color(rgb(DIM)).child(if *present {
                                "present now"
                            } else {
                                "stored"
                            })),
                    )
                })
        };
        let ecu_card = div()
            .flex()
            .flex_col()
            .gap(px(10.))
            .p(px(12.))
            .rounded(px(10.))
            .bg(rgb(CARD))
            .border_1()
            .border_color(rgb(EDGE))
            .child(
                div()
                    .text_xs()
                    .text_color(rgb(DIM))
                    .child("ECU (its own readings and fault memory)"),
            )
            .child(ecu_stats)
            .child(
                div()
                    .text_xs()
                    .text_color(rgb(DIM))
                    .child("Stored fault codes"),
            )
            .child(code_list);

        let sliders = self
            .controls
            .iter()
            .fold(div().flex().flex_col().gap(px(10.)), |d, c| {
                let v = c.state.read(cx).value().start();
                // road speed only matters when the ECU reads the speed input
                let unused = c.name == "speed" && self.get("speed_input") == "none";
                d.child(
                    div()
                        .flex()
                        .flex_col()
                        .gap(px(4.))
                        .child(
                            div()
                                .flex()
                                .flex_row()
                                .justify_between()
                                .child(div().text_sm().text_color(rgb(TEXT)).child(if unused {
                                    format!("{} (not read by this ECU)", c.label)
                                } else {
                                    c.label.to_string()
                                }))
                                .child(
                                    div()
                                        .text_sm()
                                        .text_color(rgb(DIM))
                                        .child(format!("{:.*} {}", c.decimals, v, c.unit)),
                                ),
                        )
                        .child(Slider::new(&c.state).disabled(unused)),
                )
            });

        let footer = div()
            .text_xs()
            .text_color(rgb(DIM))
            .child(match &self.last_error {
                Some(e) => e.clone(),
                None => format!(
                    "Testers: Hi-Scan GUI line \"Process\", or kdiag --ecusim auto. ADC {}",
                    self.get("adc")
                ),
            });

        // faults: wiring faults the ECU detects by itself
        let active: Vec<String> = self
            .get("faults")
            .split(',')
            .filter(|f| !f.is_empty() && *f != "none" && *f != "–")
            .map(String::from)
            .collect();
        let faults = Fault::ALL
            .into_iter()
            .fold(div().flex().flex_col().gap(px(6.)), |d, f| {
                let on = active.iter().any(|a| a == f.name());
                d.child(
                    div()
                        .id(SharedString::from(format!("fault-{}", f.name())))
                        .flex()
                        .flex_row()
                        .items_center()
                        .justify_between()
                        .gap(px(8.))
                        .px(px(10.))
                        .py(px(7.))
                        .rounded(px(8.))
                        .border_1()
                        .border_color(rgb(if on { ACCENT } else { EDGE }))
                        .bg(rgb(CARD))
                        .cursor_pointer()
                        .hover(|s| s.bg(rgb(0x1f2024)))
                        .child(
                            div()
                                .text_sm()
                                .text_color(rgb(if on { ACCENT } else { TEXT }))
                                .child(f.label()),
                        )
                        .child(
                            div()
                                .text_xs()
                                .font_family("Menlo")
                                .text_color(rgb(DIM))
                                .child(f.code()),
                        )
                        .on_click(cx.listener(move |v, _, _, cx| {
                            v.set(
                                &format!("fault.{}", f.name()),
                                if on { "off" } else { "on" },
                            );
                            cx.notify();
                        })),
                )
            });
        let battery_on = self.get("battery_connected") != "off";
        let battery_btn = div()
            .id("battery")
            .flex()
            .items_center()
            .justify_center()
            .h(px(40.))
            .rounded(px(8.))
            .border_1()
            .border_color(rgb(if battery_on { EDGE } else { WARN }))
            .bg(rgb(CARD))
            .text_sm()
            .text_color(rgb(if battery_on { TEXT } else { WARN }))
            .cursor_pointer()
            .hover(|s| s.bg(rgb(0x1f2024)))
            .child(if battery_on {
                "Disconnect battery (wipes the ECU's memory)"
            } else {
                "Battery disconnected: reconnect"
            })
            .on_click(cx.listener(move |v, _, _, cx| {
                v.set("battery_connected", if battery_on { "off" } else { "on" });
                cx.notify();
            }));
        let right = div()
            .flex()
            .flex_col()
            .gap(px(10.))
            .w(px(330.))
            .flex_none()
            .child(div().text_sm().text_color(rgb(TEXT)).child("Faults"))
            .child(div().text_xs().text_color(rgb(DIM)).child(
                "Wiring faults the ECU detects itself; it stores the code shown (takes \
                 a few seconds, some need the engine running). Fault codes stay \
                 through key cycles. Clear them with the Hi-Scan (ERAS) or kdiag.",
            ))
            .child(faults)
            .child(battery_btn);

        let left = div()
            .flex()
            .flex_col()
            .gap(px(14.))
            .flex_1()
            .min_w(px(0.))
            .overflow_hidden()
            .child(header)
            .child(
                div()
                    .flex()
                    .flex_row()
                    .gap(px(10.))
                    .child(key_btn)
                    .child(start_btn)
                    .child(stop_btn),
            )
            .child(big)
            .child(ecu_card)
            .child(stats)
            .child(sliders)
            .child(footer);

        div()
            .id("root")
            .size_full()
            .flex()
            .flex_row()
            .gap(px(18.))
            .p(px(18.))
            .bg(rgb(BG))
            .text_color(rgb(TEXT))
            // a release anywhere stops the starter
            .on_mouse_up(
                MouseButton::Left,
                cx.listener(|v, _, _, cx| v.starter(false, cx)),
            )
            .child(left)
            .child(right)
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let arg = |name: &str| {
        args.iter()
            .position(|a| a == name)
            .and_then(|i| args.get(i + 1))
            .cloned()
    };
    let socket = arg("--socket").map_or_else(ecusim::default_socket, PathBuf::from);
    // start a server if none answers and a ROM was given
    let server = match (Conn::connect(&socket), arg("--rom")) {
        (Ok(_), _) | (Err(_), None) => None,
        (Err(_), Some(rom)) => {
            let exe = std::env::current_exe()
                .ok()
                .and_then(|p| p.parent().map(|d| d.join("ecusim")))
                .unwrap_or_else(|| PathBuf::from("ecusim"));
            match Command::new(&exe)
                .args(["serve", "--rom", &rom, "--socket"])
                .arg(&socket)
                .spawn()
            {
                Ok(c) => {
                    // give it a moment to bind
                    std::thread::sleep(Duration::from_millis(300));
                    Some(c)
                }
                Err(e) => {
                    eprintln!("ecusim-gui: starting {}: {e}", exe.display());
                    None
                }
            }
        }
    };
    gpui_kit::application()
        .with_assets(gpui_kit::assets::AllAssets)
        .run(move |cx| {
            gpui_kit::init(cx);
            gpui_kit::component::Theme::change(gpui_kit::component::ThemeMode::Dark, None, cx);
            let bounds = Bounds::centered(None, size(px(940.), px(1040.)), cx);
            let opts = WindowOptions {
                window_bounds: Some(WindowBounds::Windowed(bounds)),
                titlebar: Some(TitlebarOptions {
                    title: Some("GK 2.0 engine".into()),
                    ..Default::default()
                }),
                ..Default::default()
            };
            let socket = socket.clone();
            gpui_kit::open_window(opts, cx, move |_, cx| {
                cx.new(|cx| Panel::new(socket.clone(), cx))
            })
            .expect("failed to open window");
            // a server this panel started goes with it, however the app ends
            let server = std::rc::Rc::new(std::cell::RefCell::new(server));
            let stop = move || {
                if let Some(mut c) = server.borrow_mut().take() {
                    let _ = c.kill();
                    let _ = c.wait();
                }
            };
            let stop2 = stop.clone();
            cx.on_app_quit(move |_| {
                stop2();
                async {}
            })
            .detach();
            cx.on_window_closed(move |cx, _| {
                stop();
                cx.quit()
            })
            .detach();
            cx.activate(true);
        });
}
