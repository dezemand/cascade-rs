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
    tab: Tab,
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
            tab: Tab::Codes,
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
}

impl Panel {
    /// Time factor buttons and snapshots (save, restore by name).
    fn time_and_snapshots(&self, cx: &mut Context<Self>) -> impl IntoElement {
        let chip = |id: String, label: String, active: bool| {
            div()
                .id(SharedString::from(id))
                .px(px(10.))
                .py(px(5.))
                .rounded(px(6.))
                .border_1()
                .border_color(rgb(if active { ACCENT } else { EDGE }))
                .bg(rgb(CARD))
                .text_xs()
                .text_color(rgb(if active { ACCENT } else { TEXT }))
                .cursor_pointer()
                .hover(|s| s.bg(rgb(0x1f2024)))
                .child(label)
        };
        let time = self.get("time").to_string();
        let times = ["1", "2", "4", "8"].into_iter().fold(
            div().flex().flex_row().gap(px(6.)).items_center().child(
                div()
                    .text_xs()
                    .text_color(rgb(DIM))
                    .w(px(70.))
                    .child("Time"),
            ),
            |d, t| {
                d.child(
                    chip(format!("time-{t}"), format!("{t}×"), time == t).on_click(cx.listener(
                        move |v, _, _, cx| {
                            v.set("time", t);
                            cx.notify();
                        },
                    )),
                )
            },
        );
        let snaps: Vec<String> = self
            .get("snapshots")
            .split(',')
            .filter(|s| !s.is_empty() && *s != "none" && *s != "–")
            .map(String::from)
            .collect();
        let next = format!("S{}", snaps.len() + 1);
        let snap_row = snaps.iter().fold(
            div()
                .flex()
                .flex_row()
                .flex_wrap()
                .gap(px(6.))
                .items_center()
                .child(
                    div()
                        .text_xs()
                        .text_color(rgb(DIM))
                        .w(px(70.))
                        .child("Snapshots"),
                )
                .child(
                    chip("snap-save".into(), "Save".into(), false).on_click(cx.listener(
                        move |v, _, _, cx| {
                            v.set("snapshot", &next);
                            cx.notify();
                        },
                    )),
                ),
            |d, name| {
                let n = name.clone();
                d.child(
                    chip(format!("snap-{name}"), format!("↺ {name}"), false).on_click(cx.listener(
                        move |v, _, _, cx| {
                            v.set("restore", &n);
                            cx.notify();
                        },
                    )),
                )
            },
        );
        div()
            .flex()
            .flex_col()
            .gap(px(8.))
            .child(times)
            .child(snap_row)
            .child(
                div()
                    .text_xs()
                    .text_color(rgb(DIM))
                    .child("Fast-forward drops to 1× while a tester is connected."),
            )
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Tab {
    Codes,
    Faults,
    Power,
}

/// A small bordered card.
fn card() -> Div {
    div()
        .flex()
        .flex_col()
        .p(px(12.))
        .rounded(px(10.))
        .bg(rgb(CARD))
        .border_1()
        .border_color(rgb(EDGE))
}

fn caption(text: impl Into<SharedString>) -> Div {
    div().text_xs().text_color(rgb(DIM)).child(text.into())
}

/// Entries of a comma list in the status ("none" and "–" are empty).
fn list(v: &str) -> Vec<String> {
    v.split(',')
        .filter(|f| !f.is_empty() && *f != "none" && *f != "–")
        .map(String::from)
        .collect()
}

impl Panel {
    /// A compact reading: caption over value and unit.
    fn gauge(&self, label: &'static str, value: String, unit: &'static str) -> Div {
        card()
            .gap(px(2.))
            .py(px(8.))
            .min_w(px(96.))
            .flex_1()
            .child(caption(label))
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

    fn top_bar(&self, cx: &mut Context<Self>) -> Div {
        let connected = self.conn.is_some() && !self.status.is_empty();
        let ign = self.get("ignition") == "on";
        let running = matches!(self.get("engine"), "running" | "cranking");
        let btn = |id: &'static str| {
            div()
                .id(id)
                .flex()
                .flex_col()
                .items_center()
                .justify_center()
                .h(px(44.))
                .px(px(16.))
                .rounded(px(8.))
                .text_sm()
                .cursor_pointer()
        };
        let key = btn("ignition")
            .border_1()
            .border_color(rgb(if ign { ACCENT } else { EDGE }))
            .bg(rgb(CARD))
            .text_color(rgb(if ign { ACCENT } else { TEXT }))
            .hover(|s| s.bg(rgb(0x1f2024)))
            .child(if ign { "IGNITION ON" } else { "IGNITION OFF" })
            .on_click(cx.listener(move |v, _, _, cx| {
                v.set("ignition", if ign { "off" } else { "on" });
                if ign {
                    v.starter(false, cx);
                }
                cx.notify();
            }));
        let start = btn("start")
            .min_w(px(120.))
            .bg(rgb(if self.cranking { 0xd99c17 } else { ACCENT }))
            .text_color(rgb(ACCENT_TEXT))
            .font_weight(FontWeight::SEMIBOLD)
            .when(!ign, |d| d.opacity(0.4))
            .child("START")
            .child(
                div()
                    .text_xs()
                    .font_weight(FontWeight::NORMAL)
                    .child("hold to crank"),
            )
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
        let stop = btn("stop")
            .border_1()
            .border_color(rgb(EDGE))
            .bg(rgb(CARD))
            .text_color(rgb(TEXT))
            .when(!running, |d| d.opacity(0.4))
            .hover(|s| s.bg(rgb(0x1f2024)))
            .child("STOP")
            .on_click(cx.listener(move |v, _, _, cx| {
                if running {
                    v.set("engine", "stop");
                }
                cx.notify();
            }));
        let model = if self.get("model") == "closed" {
            "runs on the ECU's fuel and spark"
        } else {
            "open-loop engine model"
        };
        div()
            .flex()
            .flex_row()
            .items_center()
            .justify_between()
            .child(
                div()
                    .flex()
                    .flex_col()
                    .gap(px(2.))
                    .child(div().text_lg().text_color(rgb(TEXT)).child("GK 2.0 engine"))
                    .child(
                        div()
                            .flex()
                            .flex_row()
                            .items_center()
                            .gap(px(6.))
                            .child(div().size(px(7.)).rounded_full().bg(rgb(if connected {
                                OK
                            } else {
                                WARN
                            })))
                            .child(caption(if connected {
                                format!(
                                    "{} · ECU {} · {model}",
                                    self.get("family").to_uppercase(),
                                    self.get("ecu")
                                )
                            } else {
                                "not connected to ecusim".to_string()
                            })),
                    ),
            )
            .child(
                div()
                    .flex()
                    .flex_row()
                    .gap(px(8.))
                    .child(key)
                    .child(start)
                    .child(stop),
            )
    }

    fn gauges(&self) -> Div {
        let engine = self.get("engine").to_string();
        let color = match engine.as_str() {
            "running" => OK,
            "cranking" => ACCENT,
            _ => DIM,
        };
        let ecu_on = self.get("ecu") == "on";
        let ecu = |k: &str| {
            if ecu_on {
                self.get(k).to_string()
            } else {
                "–".into()
            }
        };
        let speed = card()
            .gap(px(2.))
            .py(px(8.))
            .min_w(px(170.))
            .child(
                div()
                    .flex()
                    .flex_row()
                    .justify_between()
                    .child(caption("ENGINE SPEED"))
                    .child(
                        div()
                            .text_xs()
                            .text_color(rgb(color))
                            .child(engine.to_uppercase()),
                    ),
            )
            .child(
                div()
                    .flex()
                    .flex_row()
                    .items_baseline()
                    .gap(px(6.))
                    .child(
                        div()
                            .text_2xl()
                            .text_color(rgb(TEXT))
                            .child(self.get("rpm").to_string()),
                    )
                    .child(div().text_xs().text_color(rgb(DIM)).child("rpm")),
            );
        div()
            .flex()
            .flex_row()
            .gap(px(8.))
            .child(speed)
            .child(self.gauge("Battery", self.get("battery").into(), "V"))
            .child(self.gauge("MAP", self.get("map").into(), "kPa"))
            .child(self.gauge("Front O2", self.get("o2_front").into(), "mV"))
            .when(self.get("model") == "closed" && engine == "running", |d| {
                d.child(self.gauge("Mixture λ", self.get("lambda").into(), ""))
            })
            .child(self.gauge("ECU coolant", ecu("ecu_coolant"), "°C"))
            .child(self.gauge("ECU road speed", ecu("ecu_speed"), "km/h"))
    }

    fn drive(&self, cx: &mut Context<Self>) -> Div {
        let sliders = self
            .controls
            .iter()
            .fold(div().flex().flex_col().gap(px(10.)), |d, c| {
                let v = c.state.read(cx).value().start();
                let unused = c.name == "speed" && self.get("speed_input") == "none";
                d.child(
                    div()
                        .flex()
                        .flex_col()
                        .gap(px(3.))
                        .child(
                            div()
                                .flex()
                                .flex_row()
                                .justify_between()
                                .child(
                                    div()
                                        .text_sm()
                                        .text_color(rgb(if unused { DIM } else { TEXT }))
                                        .child(c.label),
                                )
                                .child(div().text_sm().text_color(rgb(DIM)).child(if unused {
                                    "not read".to_string()
                                } else {
                                    format!("{:.*} {}", c.decimals, v, c.unit)
                                })),
                        )
                        .child(Slider::new(&c.state).disabled(unused)),
                )
            });
        card()
            .gap(px(12.))
            .w(px(340.))
            .flex_none()
            .child(caption("DRIVE"))
            .child(sliders)
    }

    fn tabs(&self, cx: &mut Context<Self>) -> Div {
        let codes = list(self.get("dtcs"));
        let module_codes: usize = ["abs_dtcs", "tcs_dtcs", "srs_dtcs"]
            .iter()
            .map(|k| list(self.get(k)).len())
            .sum();
        let n_faults = list(self.get("faults")).len() + list(self.get("intermittent")).len();
        let tab = |id: &'static str, label: String, t: Tab, active: Tab| {
            let on = t == active;
            div()
                .id(id)
                .px(px(12.))
                .py(px(6.))
                .rounded(px(6.))
                .text_sm()
                .text_color(rgb(if on { ACCENT } else { DIM }))
                .when(on, |d| d.bg(rgb(0x1f2024)))
                .cursor_pointer()
                .hover(|s| s.text_color(rgb(TEXT)))
                .child(label)
        };
        let count = |n: usize| {
            if n > 0 {
                format!(" ({n})")
            } else {
                String::new()
            }
        };
        let bar = div()
            .flex()
            .flex_row()
            .gap(px(4.))
            .child(
                tab(
                    "tab-codes",
                    format!("Fault codes{}", count(codes.len() + module_codes)),
                    Tab::Codes,
                    self.tab,
                )
                .on_click(cx.listener(|v, _, _, cx| {
                    v.tab = Tab::Codes;
                    cx.notify();
                })),
            )
            .child(
                tab(
                    "tab-faults",
                    format!("Inject faults{}", count(n_faults)),
                    Tab::Faults,
                    self.tab,
                )
                .on_click(cx.listener(|v, _, _, cx| {
                    v.tab = Tab::Faults;
                    cx.notify();
                })),
            )
            .child(
                tab("tab-power", "Power & time".into(), Tab::Power, self.tab).on_click(
                    cx.listener(|v, _, _, cx| {
                        v.tab = Tab::Power;
                        cx.notify();
                    }),
                ),
            );
        let body = match self.tab {
            Tab::Codes => self.codes_view(),
            Tab::Faults => self.faults_view(cx),
            Tab::Power => self.power_view(cx),
        };
        card().gap(px(12.)).flex_1().min_w(px(0.)).child(bar).child(
            div()
                .id("tab-body")
                .flex_1()
                .min_h(px(0.))
                .overflow_y_scroll()
                .child(body),
        )
    }

    fn codes_view(&self) -> Div {
        let ecu_on = self.get("ecu") == "on";
        let dtcs = self.get("dtcs").to_string();
        let row = |code: String, note: &'static str, warn: bool| {
            div()
                .flex()
                .flex_row()
                .justify_between()
                .py(px(3.))
                .child(
                    div()
                        .font_family("Menlo")
                        .text_sm()
                        .text_color(rgb(if warn { WARN } else { TEXT }))
                        .child(code),
                )
                .child(caption(note))
        };
        let engine = if !ecu_on {
            div().child(caption("ECU off (battery disconnected)"))
        } else if dtcs == "reading" {
            div().child(caption("Reading the ECU over the K-line…"))
        } else {
            let codes: Vec<(String, bool)> = dtcs
                .split(',')
                .filter_map(|e| e.split_once(':'))
                .map(|(c, st)| {
                    (
                        c.to_string(),
                        u8::from_str_radix(st, 16).is_ok_and(|b| b & 0x40 != 0),
                    )
                })
                .collect();
            if codes.is_empty() {
                div()
                    .text_sm()
                    .text_color(rgb(OK))
                    .child("No stored fault codes")
            } else {
                codes
                    .into_iter()
                    .fold(div().flex().flex_col(), |d, (c, present)| {
                        d.child(row(
                            c,
                            if present { "present now" } else { "stored" },
                            present,
                        ))
                    })
            }
        };
        let source = if self.get("ecu_source") == "kline" {
            "ENGINE ECU · read over its K-line while no tester is connected"
        } else {
            "ENGINE ECU · from its fault memory"
        };
        let modules = [
            ("abs_dtcs", "ABS"),
            ("tcs_dtcs", "Traction control"),
            ("srs_dtcs", "Airbag"),
        ]
        .into_iter()
        .fold(div().flex().flex_col().gap(px(2.)), |d, (key, name)| {
            let codes = list(self.get(key));
            d.child(
                div()
                    .flex()
                    .flex_row()
                    .justify_between()
                    .py(px(3.))
                    .child(div().text_sm().text_color(rgb(TEXT)).child(name))
                    .child(if codes.is_empty() {
                        div().text_sm().text_color(rgb(OK)).child("no codes")
                    } else {
                        div()
                            .font_family("Menlo")
                            .text_sm()
                            .text_color(rgb(TEXT))
                            .child(codes.join("  ").replace(':', " "))
                    }),
            )
        });
        div()
            .flex()
            .flex_col()
            .gap(px(14.))
            .child(
                div()
                    .flex()
                    .flex_col()
                    .gap(px(4.))
                    .child(caption(source))
                    .child(engine),
            )
            .child(
                div()
                    .flex()
                    .flex_col()
                    .gap(px(4.))
                    .child(caption("OTHER MODULES (models)"))
                    .child(modules),
            )
            .child(caption(
                "Read and erase them with the Hi-Scan (line \"Process\"; ERAS with key on, \
                 engine off) or kdiag --ecusim auto. Codes stay through key cycles.",
            ))
    }

    fn faults_view(&self, cx: &mut Context<Self>) -> Div {
        let active = list(self.get("faults"));
        let flickering = list(self.get("intermittent"));
        let profile: BTreeMap<String, String> = self
            .get("fault_codes")
            .split(',')
            .filter_map(|e| e.split_once(':'))
            .map(|(k, v)| (k.to_string(), v.replace('+', " ")))
            .collect();
        let grid =
            Fault::ALL
                .into_iter()
                .fold(div().flex().flex_row().flex_wrap().gap(px(6.)), |d, f| {
                    let on = active.iter().any(|a| a == f.name());
                    let flicker = flickering.iter().any(|a| a == f.name());
                    let code = match profile.get(f.name()) {
                        Some(c) if c == "none" => "–".to_string(),
                        Some(c) => c.clone(),
                        None => f.code().to_string(),
                    };
                    d.child(
                        div()
                            .id(SharedString::from(format!("fault-{}", f.name())))
                            .w(relative(0.493))
                            .flex()
                            .flex_row()
                            .items_center()
                            .justify_between()
                            .gap(px(8.))
                            .px(px(10.))
                            .py(px(6.))
                            .rounded(px(6.))
                            .border_1()
                            .border_color(rgb(if on {
                                ACCENT
                            } else if flicker {
                                0x8a6d1f
                            } else {
                                EDGE
                            }))
                            .cursor_pointer()
                            .hover(|s| s.bg(rgb(0x1f2024)))
                            .child(
                                div()
                                    .text_xs()
                                    .text_color(rgb(if on || flicker { ACCENT } else { TEXT }))
                                    .child(if flicker {
                                        format!("{} · intermittent", f.label())
                                    } else {
                                        f.label().to_string()
                                    }),
                            )
                            .child(
                                div()
                                    .text_xs()
                                    .font_family("Menlo")
                                    .text_color(rgb(DIM))
                                    .child(code),
                            )
                            .on_click(cx.listener(move |v, _, _, cx| {
                                // off -> on -> intermittent -> off
                                v.set(
                                    &format!("fault.{}", f.name()),
                                    if on {
                                        "intermittent"
                                    } else if flicker {
                                        "off"
                                    } else {
                                        "on"
                                    },
                                );
                                cx.notify();
                            })),
                    )
                });
        div()
            .flex()
            .flex_col()
            .gap(px(10.))
            .child(caption(
                "Click: on → intermittent → off. The ECU detects the fault itself and stores \
                 the code shown (this dump's profile, if there is one); some need the engine \
                 running.",
            ))
            .child(grid)
    }

    fn power_view(&self, cx: &mut Context<Self>) -> Div {
        let battery_on = self.get("battery_connected") != "off";
        let battery = div()
            .id("battery")
            .flex()
            .items_center()
            .justify_center()
            .h(px(36.))
            .px(px(14.))
            .rounded(px(8.))
            .border_1()
            .border_color(rgb(if battery_on { EDGE } else { WARN }))
            .text_sm()
            .text_color(rgb(if battery_on { TEXT } else { WARN }))
            .cursor_pointer()
            .hover(|s| s.bg(rgb(0x1f2024)))
            .child(if battery_on {
                "Disconnect battery"
            } else {
                "Battery disconnected · reconnect"
            })
            .on_click(cx.listener(move |v, _, _, cx| {
                v.set("battery_connected", if battery_on { "off" } else { "on" });
                cx.notify();
            }));
        div()
            .flex()
            .flex_col()
            .gap(px(14.))
            .child(
                div()
                    .flex()
                    .flex_col()
                    .gap(px(6.))
                    .child(caption("BATTERY"))
                    .child(div().flex().flex_row().child(battery))
                    .child(caption(
                        "A disconnect power-cycles the ECU and wipes its memory, fault codes \
                         included.",
                    )),
            )
            .child(
                div()
                    .flex()
                    .flex_col()
                    .gap(px(6.))
                    .child(caption("TIME AND SNAPSHOTS"))
                    .child(self.time_and_snapshots(cx)),
            )
            .child(
                div()
                    .flex()
                    .flex_col()
                    .gap(px(4.))
                    .child(caption("ECU"))
                    .child(caption(format!(
                        "running for {} s · testers connected: {} · ADC {}",
                        self.get("ecu_s"),
                        self.get("testers"),
                        self.get("adc")
                    ))),
            )
    }
}

impl Render for Panel {
    fn render(&mut self, _window: &mut Window, cx: &mut Context<Self>) -> impl IntoElement {
        let footer = caption(match &self.last_error {
            Some(e) => e.clone(),
            None => "Testers: Hi-Scan GUI line \"Process\" · kdiag --ecusim auto · \
                     kdiag elm --ecusim auto"
                .to_string(),
        });
        div()
            .id("root")
            .size_full()
            .flex()
            .flex_col()
            .gap(px(12.))
            .p(px(16.))
            .bg(rgb(BG))
            .text_color(rgb(TEXT))
            // a release anywhere stops the starter
            .on_mouse_up(
                MouseButton::Left,
                cx.listener(|v, _, _, cx| v.starter(false, cx)),
            )
            .child(self.top_bar(cx))
            .child(self.gauges())
            .child(
                div()
                    .flex()
                    .flex_row()
                    .gap(px(12.))
                    .flex_1()
                    .min_h(px(0.))
                    .child(
                        div()
                            .id("drive-scroll")
                            .flex_none()
                            .h_full()
                            .overflow_y_scroll()
                            .child(self.drive(cx)),
                    )
                    .child(self.tabs(cx)),
            )
            .child(footer)
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
            let bounds = Bounds::centered(None, size(px(1080.), px(680.)), cx);
            let opts = WindowOptions {
                window_bounds: Some(WindowBounds::Windowed(bounds)),
                window_min_size: Some(size(px(900.), px(560.))),
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
