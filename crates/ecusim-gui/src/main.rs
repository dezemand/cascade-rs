//! ecusim-gui: the car's controls for an `ecusim` ECU process.
//!
//! ```text
//! ecusim-gui [--rom SIMK41.bin] [--socket PATH]
//! ```
//!
//! Connects to the ECU process on its socket (default `$TMPDIR/ecusim.sock`).
//! With `--rom`, and no server answering, it starts `ecusim serve` itself
//! (the `ecusim` binary next to this one) and stops it on exit. With
//! neither, a setup screen asks for the dump (remembering recent ones) or
//! connects to a server started elsewhere.
//!
//! Ignition, a hold-to-crank START button, throttle, temperatures, battery,
//! road speed and idle speed go to the engine model; the panel shows what
//! the model does and what the ECU reports about itself. Testers (the
//! Hi-Scan GUI's "Simulated car" line, `kdiag --ecusim auto`) connect to the same
//! socket.

mod cli;

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::process::Command;
use std::time::Duration;

use ecusim::client::Conn;
use ecusim::plant::Fault;
use ecusim::proto::{ToClient, ToServer};
use gpui_kit::assets::IconName;
use gpui_kit::component::slider::{Slider, SliderEvent, SliderState};
use gpui_kit::prelude::FluentBuilder as _;
use gpui_kit::*;

use cascade_theme::{self as ui, Palette};

thread_local! {
    /// The platform palette for the window being drawn (set at the start
    /// of each `render`).
    static PALETTE: std::cell::Cell<Palette> =
        std::cell::Cell::new(Palette::of(WindowAppearance::Dark));
}

fn pal() -> Palette {
    PALETTE.with(|p| p.get())
}

/// Use the palette for `window`'s appearance; gpui-kit's sliders follow it.
fn use_palette(window: &Window, cx: &mut App) {
    let p = Palette::of(window.appearance());
    PALETTE.with(|c| c.set(p));
    let t = gpui_kit::component::Theme::global_mut(cx);
    let accent: Hsla = rgb(p.accent).into();
    t.colors.slider_bar = accent;
    t.colors.slider_thumb = rgb(0xffffff).into();
    t.tokens.slider_bar = accent.into();
    t.tokens.slider_thumb = Hsla::from(rgb(0xffffff)).into();
}

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
    /// The side panel (codes, faults, CAN, power) is shown.
    sidebar: bool,
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
            sidebar: true,
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
                ToClient::Rx(_) | ToClient::Can { .. } => {}
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
                .rounded(px(ui::RADIUS))
                .border_1()
                .border_color(rgb(if active {
                    pal().accent
                } else {
                    pal().separator
                }))
                .bg(rgb(pal().panel))
                .text_xs()
                .text_color(rgb(if active { pal().accent } else { pal().label }))
                .cursor_pointer()
                .hover(|s| s.bg(rgb(pal().fill_hover)))
                .child(label)
        };
        let time = self.get("time").to_string();
        let times = ["1", "2", "4", "8"].into_iter().fold(
            div().flex().flex_row().gap(px(6.)).items_center().child(
                div()
                    .text_xs()
                    .text_color(rgb(pal().secondary))
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
                        .text_color(rgb(pal().secondary))
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
                    .text_color(rgb(pal().secondary))
                    .child("Fast-forward drops to 1× while a tester is connected."),
            )
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Tab {
    Codes,
    Faults,
    Can,
    Power,
}

/// A small bordered card.
fn card() -> Div {
    div()
        .flex()
        .flex_col()
        .p(px(12.))
        .rounded(px(7.))
        .bg(rgb(pal().panel))
        .border_1()
        .border_color(rgb(pal().separator))
}

fn caption(text: impl Into<SharedString>) -> Div {
    div()
        .text_xs()
        .text_color(rgb(pal().secondary))
        .child(text.into())
}

/// Entries of a comma list in the status ("none" and "–" are empty).
fn list(v: &str) -> Vec<String> {
    v.split(',')
        .filter(|f| !f.is_empty() && *f != "none" && *f != "–")
        .map(String::from)
        .collect()
}

impl Panel {
    /// One reading: label over value and unit, no box.
    fn reading(&self, label: &'static str, value: String, unit: &'static str) -> Div {
        let p = pal();
        div()
            .flex()
            .flex_col()
            .gap(px(2.))
            .w(relative(1. / 3.))
            .child(ui::note(p, label))
            .child(
                div()
                    .flex()
                    .flex_row()
                    .items_baseline()
                    .gap(px(4.))
                    .child(
                        div()
                            .text_size(px(20.))
                            .font_family(ui::mono())
                            .text_color(rgb(p.label))
                            .child(value),
                    )
                    .child(ui::note(p, unit)),
            )
    }

    /// The car's controls and the side panel toggle, across the top.
    fn toolbar(&self, cx: &mut Context<Self>) -> Div {
        let p = pal();
        let connected = self.conn.is_some() && !self.status.is_empty();
        let ign = self.get("ignition") == "on";
        let running = matches!(self.get("engine"), "running" | "cranking");
        let key = ui::button(
            p,
            "ignition",
            None,
            if ign { "Ignition On" } else { "Ignition Off" },
        )
        .when(ign, |d| {
            d.bg(rgb(p.selected))
                .text_color(rgb(p.accent))
                .border_color(rgb(p.accent))
        })
        .on_click(cx.listener(move |v, _, _, cx| {
            v.set("ignition", if ign { "off" } else { "on" });
            if ign {
                v.starter(false, cx);
            }
            cx.notify();
        }));
        let start = ui::default_button(
            p,
            "start",
            None,
            if self.cranking {
                "Cranking…"
            } else {
                "Start"
            },
        )
        .min_w(px(90.))
        .when(!ign, |d| d.opacity(0.4))
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
        let stop = ui::button(p, "stop", None, "Stop")
            .when(!running, |d| d.opacity(0.4))
            .on_click(cx.listener(move |v, _, _, cx| {
                if running {
                    v.set("engine", "stop");
                }
                cx.notify();
            }));
        div()
            .flex()
            .flex_row()
            .items_center()
            .justify_between()
            .gap(px(12.))
            .h(px(52.))
            .px(px(16.))
            .bg(rgb(p.panel))
            .border_b_1()
            .border_color(rgb(p.separator))
            .child(
                div()
                    .flex()
                    .flex_col()
                    .min_w(px(0.))
                    .child(
                        div()
                            .text_size(px(ui::BODY))
                            .font_weight(FontWeight::SEMIBOLD)
                            .child("GK 2.0 Engine"),
                    )
                    .child(
                        div()
                            .flex()
                            .flex_row()
                            .items_center()
                            .gap(px(5.))
                            .child(div().size(px(6.)).rounded_full().bg(rgb(if connected {
                                p.green
                            } else {
                                p.red
                            })))
                            .child(ui::note(
                                p,
                                if connected {
                                    if self.get("model") == "closed" {
                                        "Runs on the ECU's own fuel and spark"
                                    } else {
                                        "Open-loop engine model"
                                    }
                                } else {
                                    "Not connected to ecusim"
                                },
                            )),
                    ),
            )
            .child(
                div()
                    .flex()
                    .flex_row()
                    .items_center()
                    .gap(px(6.))
                    .child(key)
                    .child(start)
                    .child(stop)
                    .child(div().w(px(8.)))
                    .child(
                        ui::button(p, "sidebar", Some(IconName::PanelRight), "")
                            .when(self.sidebar, |d| d.bg(rgb(p.selected)))
                            .on_click(cx.listener(|v, _, _, cx| {
                                v.sidebar = !v.sidebar;
                                cx.notify();
                            })),
                    ),
            )
    }

    /// Engine speed large, the other readings below it.
    fn readings(&self) -> Div {
        let p = pal();
        let engine = self.get("engine").to_string();
        let (state, color) = match engine.as_str() {
            "running" => ("Running", p.green),
            "cranking" => ("Cranking", p.accent),
            "stalled" => ("Stalled", p.red),
            _ => ("Off", p.tertiary),
        };
        let ecu_on = self.get("ecu") == "on";
        let ecu = |k: &str| {
            if ecu_on {
                self.get(k).to_string()
            } else {
                "–".into()
            }
        };
        let rule = || div().h(px(1.)).bg(rgb(p.separator));
        let lambda = self.get("model") == "closed" && engine == "running";
        div()
            .flex()
            .flex_col()
            .gap(px(12.))
            .child(
                div()
                    .flex()
                    .flex_row()
                    .items_end()
                    .justify_between()
                    .child(
                        div()
                            .flex()
                            .flex_col()
                            .child(ui::header(p, "Engine Speed"))
                            .child(
                                div()
                                    .flex()
                                    .flex_row()
                                    .items_baseline()
                                    .gap(px(6.))
                                    .child(
                                        div()
                                            .text_size(px(44.))
                                            .font_family(ui::mono())
                                            .child(self.get("rpm").to_string()),
                                    )
                                    .child(ui::note(p, "rpm")),
                            ),
                    )
                    .child(
                        div()
                            .flex()
                            .flex_row()
                            .items_center()
                            .gap(px(6.))
                            .pb(px(10.))
                            .child(div().size(px(8.)).rounded_full().bg(rgb(color)))
                            .child(div().text_size(px(ui::BODY)).child(state)),
                    ),
            )
            .child(rule())
            .child(
                div()
                    .flex()
                    .flex_row()
                    .flex_wrap()
                    .gap_y(px(12.))
                    .child(self.reading("Battery", self.get("battery").into(), "V"))
                    .child(self.reading("Manifold pressure", self.get("map").into(), "kPa"))
                    .child(self.reading("Front O2", self.get("o2_front").into(), "mV"))
                    .when(lambda, |d| {
                        d.child(self.reading("Mixture λ", self.get("lambda").into(), ""))
                    })
                    .child(self.reading("ECU coolant", ecu("ecu_coolant"), "°C"))
                    .child(self.reading("ECU road speed", ecu("ecu_speed"), "km/h")),
            )
    }

    /// The driver's and the environment's inputs, one row each.
    fn controls_form(&self, cx: &mut Context<Self>) -> Div {
        let p = pal();
        self.controls.iter().fold(
            div()
                .flex()
                .flex_col()
                .gap(px(4.))
                .child(ui::header(p, "Controls")),
            |d, c| {
                let v = c.state.read(cx).value().start();
                let unused = c.name == "speed" && self.get("speed_input") == "none";
                d.child(
                    div()
                        .flex()
                        .flex_row()
                        .items_center()
                        .gap(px(14.))
                        .h(px(32.))
                        .child(
                            div()
                                .w(px(110.))
                                .text_size(px(ui::BODY))
                                .text_color(rgb(if unused { p.tertiary } else { p.label }))
                                .child(c.label),
                        )
                        .child(div().flex_1().child(Slider::new(&c.state).disabled(unused)))
                        .child(
                            div()
                                .w(px(80.))
                                .text_right()
                                .font_family(ui::mono())
                                .text_size(px(ui::BODY))
                                .text_color(rgb(p.secondary))
                                .child(if unused {
                                    "not read".to_string()
                                } else {
                                    format!("{:.*} {}", c.decimals, v, c.unit)
                                }),
                        ),
                )
            },
        )
    }

    /// The side panel: tabs, their content, and a status footer.
    fn side(&self, cx: &mut Context<Self>) -> Div {
        let p = pal();
        let codes = list(self.get("dtcs"));
        let module_codes: usize = ["abs_dtcs", "tcs_dtcs", "srs_dtcs"]
            .iter()
            .map(|k| list(self.get(k)).len())
            .sum();
        let n_faults = list(self.get("faults")).len() + list(self.get("intermittent")).len();
        let count = |name: &str, n: usize| {
            if n > 0 {
                format!("{name} ({n})")
            } else {
                name.to_string()
            }
        };
        let tabs = [
            (
                Tab::Codes,
                "tab-codes",
                count("Codes", codes.len() + module_codes),
            ),
            (Tab::Faults, "tab-faults", count("Faults", n_faults)),
            (Tab::Can, "tab-can", "CAN".to_string()),
            (Tab::Power, "tab-power", "Power".to_string()),
        ]
        .into_iter()
        .fold(ui::segmented(p), |d, (t, id, label)| {
            d.child(
                ui::segment(p, id, label, self.tab == t).on_click(cx.listener(
                    move |v, _, _, cx| {
                        v.tab = t;
                        cx.notify();
                    },
                )),
            )
        });
        let body = match self.tab {
            Tab::Codes => self.codes_view(),
            Tab::Faults => self.faults_view(cx),
            Tab::Can => self.can_view(),
            Tab::Power => self.power_view(cx),
        };
        let ecu_on = self.get("ecu") == "on";
        let status = div()
            .flex()
            .flex_col()
            .gap(px(2.))
            .px(px(14.))
            .py(px(8.))
            .border_t_1()
            .border_color(rgb(p.separator))
            .text_size(px(ui::SMALL))
            .child(
                div()
                    .flex()
                    .flex_row()
                    .items_center()
                    .gap(px(6.))
                    .child(div().size(px(7.)).rounded_full().bg(rgb(if ecu_on {
                        p.green
                    } else {
                        p.tertiary
                    })))
                    .child(div().text_color(rgb(p.label)).child(if ecu_on {
                        "ECU on"
                    } else {
                        "ECU off"
                    }))
                    .child(div().flex_1())
                    .child(
                        div()
                            .font_family(ui::mono())
                            .text_color(rgb(p.secondary))
                            .child(format!(
                                "{} · {} s",
                                self.get("family").to_uppercase(),
                                self.get("ecu_s")
                            )),
                    ),
            )
            .child(div().text_color(rgb(p.secondary)).child(format!(
                "Testers connected: {} · time ×{}",
                self.get("testers"),
                self.get("time")
            )))
            .when_some(self.last_error.clone(), |d, e| {
                d.child(div().truncate().text_color(rgb(p.red)).child(e))
            });
        div()
            .flex()
            .flex_col()
            .w(px(380.))
            .h_full()
            .bg(rgb(p.panel))
            .border_l_1()
            .border_color(rgb(p.separator))
            .text_size(px(ui::BODY))
            .child(div().p(px(10.)).child(tabs))
            .child(
                div()
                    .id("side-body")
                    .flex_1()
                    .min_h(px(0.))
                    .overflow_y_scroll()
                    .px(px(14.))
                    .pb(px(14.))
                    .child(body),
            )
            .child(status)
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
                        .font_family(ui::mono())
                        .text_sm()
                        .text_color(rgb(if warn { pal().red } else { pal().label }))
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
                    .text_color(rgb(pal().green))
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
            "Engine ECU · read over its K-line while no tester is connected"
        } else {
            "Engine ECU · from its fault memory"
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
                    .child(div().text_sm().text_color(rgb(pal().label)).child(name))
                    .child(if codes.is_empty() {
                        div()
                            .text_sm()
                            .text_color(rgb(pal().green))
                            .child("no codes")
                    } else {
                        div()
                            .font_family(ui::mono())
                            .text_sm()
                            .text_color(rgb(pal().label))
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
                    .child(caption("Other modules (models)"))
                    .child(modules),
            )
            .child(caption(
                "Read and erase them with the Hi-Scan (line \"Simulated car\"; ERAS with key on, \
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
                            .w_full()
                            .flex()
                            .flex_row()
                            .items_center()
                            .justify_between()
                            .gap(px(8.))
                            .px(px(10.))
                            .py(px(6.))
                            .rounded(px(ui::RADIUS))
                            .border_1()
                            .border_color(rgb(if on {
                                pal().accent
                            } else if flicker {
                                pal().tertiary
                            } else {
                                pal().separator
                            }))
                            .cursor_pointer()
                            .hover(|s| s.bg(rgb(pal().fill_hover)))
                            .child(
                                div()
                                    .text_xs()
                                    .text_color(rgb(if on || flicker {
                                        pal().accent
                                    } else {
                                        pal().label
                                    }))
                                    .child(if flicker {
                                        format!("{} · intermittent", f.label())
                                    } else {
                                        f.label().to_string()
                                    }),
                            )
                            .child(
                                div()
                                    .text_xs()
                                    .font_family(ui::mono())
                                    .text_color(rgb(pal().secondary))
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

    fn can_view(&self) -> Div {
        let name = |id: &str| match id {
            "316" => "DME1 · engine speed, torque",
            "329" => "DME2 · coolant, throttle",
            "545" => "DME4 · status, battery",
            "2A0" => "DME5",
            _ => "",
        };
        let frames: Vec<(String, String, String)> = self
            .get("can")
            .split(',')
            .filter_map(|e| {
                let mut p = e.splitn(3, ':');
                Some((p.next()?.into(), p.next()?.into(), p.next()?.into()))
            })
            .collect();
        // identifier and data, what the frame is underneath
        let row = |id: String, label: &'static str, data: String| {
            div()
                .flex()
                .flex_col()
                .py(px(3.))
                .child(
                    div()
                        .flex()
                        .flex_row()
                        .gap(px(12.))
                        .font_family(ui::mono())
                        .text_sm()
                        .child(div().w(px(36.)).text_color(rgb(pal().accent)).child(id))
                        .child(div().text_color(rgb(pal().label)).child(data)),
                )
                .when(!label.is_empty(), |d| d.child(caption(label).pl(px(48.))))
        };
        let spaced = |h: &str| {
            h.as_bytes()
                .chunks(2)
                .map(|c| String::from_utf8_lossy(c).into_owned())
                .collect::<Vec<_>>()
                .join(" ")
        };
        let list = if frames.is_empty() {
            div().child(caption(
                "No frames: ECU off, or the bus has no other node yet.",
            ))
        } else {
            frames
                .into_iter()
                .fold(div().flex().flex_col(), |d, (id, _, data)| {
                    let label = name(&id);
                    d.child(row(id, label, spaced(&data)))
                })
        };
        let nodes = self.get("can_nodes");
        let decoded = format!(
            "{} rpm · {} °C coolant · {} V battery",
            self.get("can_rpm"),
            self.get("can_coolant"),
            self.get("can_battery")
        );
        div()
            .flex()
            .flex_col()
            .gap(px(14.))
            .child(
                div()
                    .flex()
                    .flex_col()
                    .gap(px(4.))
                    .child(caption("Engine ECU sends (500 kbit/s, every 10 ms)"))
                    .child(list)
                    .child(caption(decoded)),
            )
            .child(
                div()
                    .flex()
                    .flex_col()
                    .gap(px(4.))
                    .child(caption("Other nodes"))
                    .child(
                        div()
                            .text_sm()
                            .text_color(rgb(pal().label))
                            .child(match nodes {
                                "esp,gearbox" => {
                                    "ESP (153, 1F0) and gearbox computer (43F) sending"
                                }
                                "esp" => "ESP sending; gearbox computer silent",
                                "gearbox" => "Gearbox computer sending; ESP silent",
                                _ => "None: nothing acknowledges the ECU's frames",
                            }),
                    )
                    .child(caption(
                        "Silence them in Inject faults (CAN: ...). Automatic-gearbox dumps \
                         then store P1602 or P0600 with the engine running. Watch the bus \
                         with `ecusim can`.",
                    )),
            )
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
            .rounded(px(ui::RADIUS))
            .border_1()
            .border_color(rgb(if battery_on {
                pal().separator
            } else {
                pal().red
            }))
            .text_sm()
            .text_color(rgb(if battery_on { pal().label } else { pal().red }))
            .cursor_pointer()
            .hover(|s| s.bg(rgb(pal().fill_hover)))
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
                    .child(caption("Battery"))
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
                    .child(caption("Time and snapshots"))
                    .child(self.time_and_snapshots(cx)),
            )
            .child(
                div()
                    .flex()
                    .flex_col()
                    .gap(px(4.))
                    .child(caption("Testers"))
                    .child(caption(
                        "Connect with the Hi-Scan GUI (line \"Simulated car\"), \
                         kdiag --ecusim auto, or kdiag elm --ecusim auto.",
                    ))
                    .child(caption(format!("ADC {}", self.get("adc")))),
            )
    }
}

impl Render for Panel {
    fn render(&mut self, window: &mut Window, cx: &mut Context<Self>) -> impl IntoElement {
        use_palette(window, cx);
        let p = pal();
        div()
            .id("root")
            .size_full()
            .flex()
            .flex_row()
            .bg(rgb(p.canvas))
            .text_color(rgb(p.label))
            // a release anywhere stops the starter
            .on_mouse_up(
                MouseButton::Left,
                cx.listener(|v, _, _, cx| v.starter(false, cx)),
            )
            .child(
                div()
                    .flex_1()
                    .min_w(px(0.))
                    .h_full()
                    .flex()
                    .flex_col()
                    .child(self.toolbar(cx))
                    .child(
                        div()
                            .id("main")
                            .flex_1()
                            .min_h(px(0.))
                            .overflow_y_scroll()
                            .flex()
                            .flex_col()
                            .gap(px(22.))
                            .p(px(20.))
                            .child(self.readings())
                            .child(div().h(px(1.)).bg(rgb(p.separator)))
                            .child(self.controls_form(cx)),
                    ),
            )
            .when(self.sidebar, |d| d.child(self.side(cx)))
    }
}

type Server = std::rc::Rc<std::cell::RefCell<Option<std::process::Child>>>;

/// Start `ecusim serve` (the binary next to this one) on `socket` and wait
/// until it answers.
fn start_server(
    rom: &std::path::Path,
    socket: &std::path::Path,
) -> Result<std::process::Child, String> {
    let exe = std::env::current_exe()
        .ok()
        .and_then(|p| p.parent().map(|d| d.join("ecusim")))
        .unwrap_or_else(|| PathBuf::from("ecusim"));
    let mut child = Command::new(&exe)
        .args(["serve", "--rom"])
        .arg(rom)
        .arg("--socket")
        .arg(socket)
        .spawn()
        .map_err(|e| format!("starting {}: {e}", exe.display()))?;
    for _ in 0..50 {
        if Conn::connect(socket).is_ok() {
            return Ok(child);
        }
        if let Ok(Some(st)) = child.try_wait() {
            return Err(format!(
                "ecusim stopped ({st}); is this an engine ECU dump?"
            ));
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    let _ = child.kill();
    Err("ecusim did not answer on its socket".into())
}

/// The car's controls.
fn open_panel(socket: PathBuf, cx: &mut App) {
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
    gpui_kit::open_window(opts, cx, move |_, cx| {
        cx.new(|cx| Panel::new(socket.clone(), cx))
    })
    .expect("failed to open window");
}

/// The first screen when nothing says what to run: pick an engine ECU
/// dump to start the simulated car, or connect to an `ecusim` already
/// running.
struct Setup {
    settings: ecusim::settings::Settings,
    dump: Option<PathBuf>,
    socket: PathBuf,
    server: Server,
    error: Option<String>,
    busy: bool,
}

impl Setup {
    fn new(socket: PathBuf, server: Server) -> Self {
        let settings = ecusim::settings::Settings::load("ecusim-gui");
        let dump = settings.recent("rom").into_iter().next();
        Self {
            settings,
            dump,
            socket,
            server,
            error: None,
            busy: false,
        }
    }

    fn choose(&mut self, cx: &mut Context<Self>) {
        let rx = cx.prompt_for_paths(PathPromptOptions {
            files: true,
            directories: false,
            multiple: false,
            prompt: Some("Use this dump".into()),
        });
        cx.spawn(async move |this, cx| {
            if let Ok(Ok(Some(paths))) = rx.await
                && let Some(p) = paths.into_iter().next()
            {
                let _ = this.update(cx, |v, cx| {
                    v.dump = Some(p);
                    v.error = None;
                    cx.notify();
                });
            }
        })
        .detach();
    }

    fn start(&mut self, window: &mut Window, cx: &mut Context<Self>) {
        let Some(dump) = self.dump.clone() else {
            return;
        };
        match std::fs::metadata(&dump).map(|m| m.len()) {
            Ok(0x40000 | 0x80000) => {}
            Ok(n) => {
                self.error = Some(format!(
                    "{n} bytes: an engine ECU dump is 256 KB (SIMK41) or 512 KB (SIMK43)"
                ));
                return;
            }
            Err(e) => {
                self.error = Some(e.to_string());
                return;
            }
        }
        self.busy = true;
        self.error = None;
        cx.notify();
        // the server needs up to a few seconds to answer: wait off the UI
        let socket = self.socket.clone();
        let setup_window = window.window_handle();
        cx.spawn(async move |this, cx| {
            let (d, s) = (dump.clone(), socket.clone());
            let started = cx
                .background_executor()
                .spawn(async move { start_server(&d, &s) })
                .await;
            let _ = this.update(cx, |v, cx| match started {
                Ok(child) => {
                    *v.server.borrow_mut() = Some(child);
                    v.settings.push_recent("rom", &dump);
                    v.settings.save();
                    open_panel(socket, cx);
                    let _ = cx.update_window(setup_window, |_, w, _| w.remove_window());
                }
                Err(e) => {
                    v.busy = false;
                    v.error = Some(e);
                    cx.notify();
                }
            });
        })
        .detach();
    }

    fn connect(&mut self, window: &mut Window, cx: &mut Context<Self>) {
        match Conn::connect(&self.socket) {
            Ok(_) => {
                open_panel(self.socket.clone(), cx);
                window.remove_window();
            }
            Err(_) => {
                self.error = Some(format!(
                    "nothing answers on {}: start one with `ecusim serve --rom DUMP`",
                    self.socket.display()
                ))
            }
        }
    }
}

fn file_label(p: &std::path::Path) -> (String, String) {
    let name = p
        .file_name()
        .map_or_else(|| p.display().to_string(), |n| n.to_string_lossy().into());
    let dir = p
        .parent()
        .map_or(String::new(), |d| d.display().to_string());
    (name, dir)
}

/// A flat secondary button.
fn ghost(id: impl Into<ElementId>, label: impl Into<SharedString>) -> Stateful<Div> {
    div()
        .id(id)
        .flex()
        .items_center()
        .justify_center()
        .h(px(34.))
        .px(px(14.))
        .rounded(px(ui::RADIUS))
        .border_1()
        .border_color(rgb(pal().separator))
        .text_sm()
        .text_color(rgb(pal().label))
        .cursor_pointer()
        .hover(|s| s.bg(rgb(pal().fill_hover)))
        .child(label.into())
}

impl Render for Setup {
    fn render(&mut self, window: &mut Window, cx: &mut Context<Self>) -> impl IntoElement {
        use_palette(window, cx);
        let recent: Vec<PathBuf> = self
            .settings
            .recent("rom")
            .into_iter()
            .filter(|p| Some(p) != self.dump.as_ref())
            .collect();
        let chosen = match &self.dump {
            Some(p) => {
                let (name, dir) = file_label(p);
                div()
                    .flex()
                    .flex_col()
                    .gap(px(2.))
                    .flex_1()
                    .min_w(px(0.))
                    .child(
                        div()
                            .truncate()
                            .text_sm()
                            .text_color(rgb(pal().label))
                            .child(name),
                    )
                    .child(caption(dir).truncate())
            }
            None => div().flex_1().child(caption("No dump chosen yet")),
        };
        let recent_list = recent.into_iter().enumerate().fold(
            div().flex().flex_col().gap(px(2.)),
            |d, (i, p)| {
                let (name, dir) = file_label(&p);
                d.child(
                    div()
                        .id(("recent", i))
                        .flex()
                        .flex_row()
                        .justify_between()
                        .gap(px(12.))
                        .px(px(10.))
                        .py(px(6.))
                        .rounded(px(ui::RADIUS))
                        .cursor_pointer()
                        .hover(|s| s.bg(rgb(pal().fill_hover)))
                        .child(
                            div()
                                .flex_1()
                                .min_w(px(0.))
                                .truncate()
                                .text_sm()
                                .text_color(rgb(pal().label))
                                .child(name),
                        )
                        .child(caption(dir).flex_none().max_w(px(160.)).truncate())
                        .on_click(cx.listener(move |v, _, _, cx| {
                            v.dump = Some(p.clone());
                            v.error = None;
                            cx.notify();
                        })),
                )
            },
        );
        let can_start = self.dump.is_some() && !self.busy;
        let start = div()
            .id("start")
            .flex()
            .items_center()
            .justify_center()
            .h(px(40.))
            .px(px(22.))
            .rounded(px(ui::RADIUS))
            .bg(rgb(pal().accent))
            .text_color(rgb(pal().on_accent))
            .font_weight(FontWeight::SEMIBOLD)
            .when(!can_start, |d| d.opacity(0.4))
            .cursor_pointer()
            .child(if self.busy {
                "Starting…"
            } else {
                "Start the car"
            })
            .on_click(cx.listener(move |v, _, w, cx| {
                if can_start {
                    v.start(w, cx);
                }
            }));
        div()
            .size_full()
            .flex()
            .flex_col()
            .gap(px(16.))
            .p(px(24.))
            .bg(rgb(pal().canvas))
            .text_color(rgb(pal().label))
            .child(
                div()
                    .flex()
                    .flex_col()
                    .gap(px(4.))
                    .child(div().text_xl().child("EcuSim"))
                    .child(caption(
                        "Runs a real engine ECU dump as a simulated car that the Hi-Scan GUI \
                         and kdiag can diagnose.",
                    )),
            )
            .child(
                card()
                    .gap(px(12.))
                    .child(caption("Engine ECU dump"))
                    .child(
                        div()
                            .flex()
                            .flex_row()
                            .items_center()
                            .gap(px(12.))
                            .child(chosen)
                            .child(
                                ghost("choose", "Choose…")
                                    .on_click(cx.listener(|v, _, _, cx| v.choose(cx))),
                            ),
                    )
                    .when(!recent_list_is_empty(&self.settings, &self.dump), |d| {
                        d.child(caption("Recent")).child(recent_list)
                    })
                    .child(caption(
                        "A 256 KB (SIMK41) or 512 KB (SIMK43) dump without immobiliser. \
                         Dumps stay on this computer.",
                    )),
            )
            .child(
                div()
                    .flex()
                    .flex_row()
                    .items_center()
                    .justify_between()
                    .child(
                        ghost("connect", "Connect to a running ecusim")
                            .on_click(cx.listener(|v, _, w, cx| v.connect(w, cx))),
                    )
                    .child(start),
            )
            .when_some(self.error.clone(), |d, e| {
                d.child(div().text_sm().text_color(rgb(pal().red)).child(e))
            })
    }
}

fn recent_list_is_empty(s: &ecusim::settings::Settings, chosen: &Option<PathBuf>) -> bool {
    s.recent("rom").iter().all(|p| Some(p) == chosen.as_ref())
}

fn main() {
    let cli = <cli::Cli as clap::Parser>::parse();
    let socket = cli.socket.unwrap_or_else(ecusim::default_socket);
    let running = Conn::connect(&socket).is_ok();
    // with --rom and no server answering: start one; with neither, the
    // setup screen asks
    let mut server = None;
    let mut setup = false;
    match (running, cli.rom) {
        (true, _) => {}
        (false, Some(rom)) => match start_server(std::path::Path::new(&rom), &socket) {
            Ok(c) => server = Some(c),
            Err(e) => {
                eprintln!("ecusim-gui: {e}");
                setup = true;
            }
        },
        (false, None) => setup = true,
    }
    let server: Server = std::rc::Rc::new(std::cell::RefCell::new(server));
    gpui_kit::application()
        .with_assets(gpui_kit::assets::AllAssets)
        .run(move |cx| {
            gpui_kit::init(cx);
            gpui_kit::component::Theme::sync_system_appearance(None, cx);
            if setup {
                let bounds = Bounds::centered(None, size(px(640.), px(480.)), cx);
                let opts = WindowOptions {
                    window_bounds: Some(WindowBounds::Windowed(bounds)),
                    titlebar: Some(TitlebarOptions {
                        title: Some("EcuSim".into()),
                        ..Default::default()
                    }),
                    ..Default::default()
                };
                let (socket, server) = (socket.clone(), server.clone());
                gpui_kit::open_window(opts, cx, move |_, cx| {
                    cx.new(|_| Setup::new(socket.clone(), server.clone()))
                })
                .expect("failed to open window");
            } else {
                open_panel(socket.clone(), cx);
            }
            // a server this app started goes with it, however the app ends
            let stop = {
                let server = server.clone();
                move || {
                    if let Some(mut c) = server.borrow_mut().take() {
                        let _ = c.kill();
                        let _ = c.wait();
                    }
                }
            };
            let stop2 = stop.clone();
            cx.on_app_quit(move |_| {
                stop2();
                async {}
            })
            .detach();
            // the setup window closes itself when the panel opens
            cx.on_window_closed(move |cx, _| {
                if cx.windows().is_empty() {
                    stop();
                    cx.quit()
                }
            })
            .detach();
            cx.activate(true);
        });
}
