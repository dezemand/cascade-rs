//! cascade-gui: desktop front end for the cascade-rs Hi-Scan emulator.
//!
//! Usage: cascade-gui ROM   (or set CASCADE_ROM)
//!
//! A flat, modern take on the Hi-Scan Pro handset: dark card, LCD on top,
//! keys on one aligned six-column grid, the Hi-Scan yellow as the only
//! accent (ENTER, active states). The device scales with the window; the
//! LCD uses whole device pixels per LCD pixel.
//!
//! The emulator runs on its own thread (`cascade-runner`), paced to real
//! time; this view sends commands and draws snapshots. Keyboard: Enter,
//! Esc, arrows, Backspace (UNDO), 0-9, F1-F6, Y/N, H (HELP), B (backlight).
//! A key is held as long as the mouse button or keyboard key is down.

use std::time::Duration;

use cascade_core::Key;
use cascade_core::lcd::{LCD_HEIGHT, LCD_WIDTH};
use cascade_iface::KwpConfig;
use cascade_runner::{Command, IfaceKind, Runner, Snapshot, Speed};
use gpui_kit::assets::IconName;
use gpui_kit::component::Icon;
use gpui_kit::component::button::{Button, ButtonVariants};
use gpui_kit::prelude::FluentBuilder as _;
use gpui_kit::*;

// Palette: neutral darks, one accent.
const DESK: u32 = 0x0e0f11;
const CARD: u32 = 0x17181b;
const CARD_EDGE: u32 = 0x25272c;
const BEZEL: u32 = 0x0a0b0c;
const LCD_BG: u32 = 0xc6cfb6;
const LCD_FG: u32 = 0x1b2318;
const KEY: u32 = 0x222428;
const KEY_HOVER: u32 = 0x2a2d32;
const KEY_DOWN: u32 = 0x363a41;
const KEY_EDGE: u32 = 0x2e3136;
const TEXT: u32 = 0xe7e9ec;
const TEXT_DIM: u32 = 0x8a8f98;
const ACCENT: u32 = 0xf5b82e;
const ACCENT_HOVER: u32 = 0xffc84f;
const ACCENT_DOWN: u32 = 0xd99c17;
const ACCENT_TEXT: u32 = 0x1d1606;
const OK: u32 = 0x4ade80;
const WARN: u32 = 0xf87171;
const PANEL: u32 = 0x131416;
const LOG_BG: u32 = 0x0b0c0d;

/// Side panel width in points.
const PANEL_W: f32 = 300.0;
/// Device size per point of LCD scale (measured from the drawn layout, with
/// a little margin), in points.
const DEVICE_W_PER_SCALE: f32 = 352.0;
const DEVICE_H_PER_SCALE: f32 = 492.0;

/// Map a GPUI key name to a Hi-Scan key.
fn map_key(name: &str) -> Option<Key> {
    Some(match name {
        "enter" => Key::ENTER,
        "escape" => Key::ESCAPE,
        "up" => Key::UP,
        "down" => Key::DOWN,
        "left" => Key::LEFT,
        "right" => Key::RIGHT,
        "backspace" => Key::UNDO,
        "y" => Key::YES,
        "n" => Key::NO,
        "h" => Key::HELP,
        "b" => Key::BACKLIGHT,
        "f1" => Key::F1,
        "f2" => Key::F2,
        "f3" => Key::F3,
        "f4" => Key::F4,
        "f5" => Key::F5,
        "f6" => Key::F6,
        d if d.len() == 1 && d.as_bytes()[0].is_ascii_digit() => Key::from_name(d)?,
        _ => return None,
    })
}

/// The KWP2000 engine ECU with plausible warm-idle values, so menus past
/// the year selection work without a car.
fn kwp_engine() -> KwpConfig {
    let mut b = vec![0u8; 80];
    b[0] = 139; // 14.1 V
    b[3] = 181; // 87.75 degC
    b[5] = 91; // 20.25 degC
    b[19..21].copy_from_slice(&800i16.to_le_bytes());
    b[21..23].copy_from_slice(&800i16.to_le_bytes());
    b[26..28].copy_from_slice(&92u16.to_le_bytes());
    b[30] = 168;
    b[42..44].copy_from_slice(&700u16.to_le_bytes());
    b[73] = 0b0100;
    let mut cfg = KwpConfig {
        dtcs: vec![(0x0130, 0x20)],
        ..Default::default()
    };
    cfg.local_data.insert(0x01, b);
    cfg
}

const IFACES: [&str; 4] = ["Fake", "KWP", "None", "Adapter"];
/// Index of "Adapter" in IFACES.
const ADAPTER: usize = 3;

/// A real K-line adapter on `path` (opened inside the emulator thread).
fn adapter_iface(path: String, echo: bool) -> IfaceKind {
    let label = format!("adapter {path}{}", if echo { "" } else { " (no echo)" });
    let make: cascade_runner::IfaceFactory = std::sync::Arc::new(move || {
        cascade_diag::kline_iface::SerialKLine::open(&path, echo)
            .map(|k| Box::new(k) as Box<dyn cascade_iface::Interface>)
            .map_err(|e| e.to_string())
    });
    IfaceKind::Custom { label, make }
}

fn iface_kind(i: usize) -> IfaceKind {
    match i {
        0 => IfaceKind::Fake,
        1 => IfaceKind::Kwp(kwp_engine()),
        _ => IfaceKind::Null,
    }
}

/// What a key does.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Cap {
    Key(Key),
    /// Power-cycle the emulated handset.
    Power,
}

/// What a key shows.
#[derive(Clone)]
enum Face {
    Text(&'static str),
    Icon(IconName),
    IconText(IconName, &'static str),
}

/// Key styles.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Tone {
    Plain,
    Accent,
}

/// Grid geometry derived from the LCD scale.
#[derive(Clone, Copy)]
struct Geo {
    /// Design unit: 1.0 at LCD scale 2.
    u: f32,
    /// Points per LCD pixel.
    scale: f32,
    /// Width of one grid column and gap between columns.
    col: f32,
    gap: f32,
    /// Height of one key row.
    row: f32,
}

impl Geo {
    fn new(scale: f32) -> Self {
        let u = scale / 2.0;
        let gap = 10.0 * u;
        let mid = 18.0 * u; // extra space between the two key blocks
        let inner = LCD_WIDTH as f32 * scale + 2.0 * Self::BEZEL * u;
        let col = (inner - 5.0 * gap - mid) / 6.0;
        Self {
            u,
            scale,
            col,
            gap,
            row: 46.0 * u,
        }
    }

    /// Width of `n` adjacent columns.
    fn span(&self, n: f32) -> f32 {
        n * self.col + (n - 1.0) * self.gap
    }

    const BEZEL: f32 = 12.0;
}

struct CascadeView {
    runner: Runner,
    snap: Snapshot,
    focus: FocusHandle,
    /// Keys currently held, by mouse or keyboard (for drawing).
    pressed: Vec<Key>,
    /// The key held with the mouse, so a release outside it still lands.
    mouse_key: Option<Cap>,
    iface_choice: usize,
    /// Adapter ports found by the last scan, the chosen one, and whether
    /// the adapter hears its own bytes.
    ports: Vec<String>,
    port: Option<String>,
    adapter_echo: bool,
    _refresh: Task<()>,
}

impl CascadeView {
    fn new(rom: Vec<u8>, window: &mut Window, cx: &mut Context<Self>) -> Self {
        let runner = Runner::spawn(rom, iface_kind(0));
        let focus = cx.focus_handle();
        focus.focus(window, cx);
        // poll the emulator about 60 times a second; redraw only on change
        let refresh = cx.spawn(async move |this, cx| {
            loop {
                cx.background_executor()
                    .timer(Duration::from_millis(16))
                    .await;
                let ok = this.update(cx, |v, cx| {
                    let changed = v
                        .runner
                        .with_snapshot(|s| {
                            s.frame_seq != v.snap.frame_seq
                                || s.log_seq != v.snap.log_seq
                                || s.total_cycles / 2_000_000 != v.snap.total_cycles / 2_000_000
                                || s.paused != v.snap.paused
                                || s.stopped != v.snap.stopped
                                || s.iface != v.snap.iface
                        })
                        .unwrap_or(false);
                    if changed {
                        v.snap = v.runner.snapshot();
                        cx.notify();
                    }
                });
                if ok.is_err() {
                    break; // view gone
                }
            }
        });
        let snap = runner.snapshot();
        Self {
            runner,
            snap,
            focus,
            pressed: Vec::new(),
            mouse_key: None,
            iface_choice: 0,
            ports: Vec::new(),
            port: None,
            adapter_echo: true,
            _refresh: refresh,
        }
    }

    fn cap_down(&mut self, cap: Cap) {
        match cap {
            Cap::Key(k) => {
                if !self.pressed.contains(&k) {
                    self.pressed.push(k);
                }
                self.runner.send(Command::Key { key: k, down: true });
            }
            Cap::Power => self.runner.send(Command::Restart(self.current_iface())),
        }
    }

    /// What the diagnostic-line selection currently means.
    fn current_iface(&self) -> IfaceKind {
        match (self.iface_choice, &self.port) {
            (ADAPTER, Some(p)) => adapter_iface(p.clone(), self.adapter_echo),
            (ADAPTER, None) => IfaceKind::Null,
            (i, _) => iface_kind(i),
        }
    }

    /// Switch to an adapter port: power-cycle onto it at real time (a real
    /// line has real-time partners).
    fn use_port(&mut self, port: String) {
        self.port = Some(port);
        self.runner.send(Command::Speed(Speed::Real(1.0)));
        self.runner.send(Command::Restart(self.current_iface()));
    }

    fn cap_up(&mut self, cap: Cap) {
        if let Cap::Key(k) = cap {
            self.pressed.retain(|p| *p != k);
            self.runner.send(Command::Key {
                key: k,
                down: false,
            });
        }
    }

    /// LCD scale for the current window: a multiple of 0.5 points per LCD
    /// pixel, so on a 2x display every LCD pixel is whole device pixels.
    fn lcd_scale(window: &Window) -> f32 {
        let vp = window.viewport_size();
        let fit = ((f32::from(vp.width) - PANEL_W - 24.0) / DEVICE_W_PER_SCALE)
            .min((f32::from(vp.height) - 24.0) / DEVICE_H_PER_SCALE);
        ((fit * 2.0).floor() / 2.0).clamp(1.0, 6.0)
    }

    fn lcd(&self, g: Geo) -> impl IntoElement {
        let frame = self.snap.frame.clone();
        let scale = g.scale;
        div()
            .p(px(Geo::BEZEL * g.u))
            .rounded(px(14.0 * g.u))
            .bg(rgb(BEZEL))
            .child(
                div().rounded(px(4.0 * g.u)).overflow_hidden().child(
                    canvas(
                        |_, _, _| (),
                        move |bounds, _, window, _| {
                            window.paint_quad(fill(bounds, rgb(LCD_BG)));
                            let o = bounds.origin;
                            for y in 0..LCD_HEIGHT {
                                let row = &frame[y * LCD_WIDTH / 8..(y + 1) * LCD_WIDTH / 8];
                                let on = |x: usize| row[x / 8] & (0x80 >> (x % 8)) != 0;
                                let mut x = 0;
                                while x < LCD_WIDTH {
                                    if !on(x) {
                                        x += 1;
                                        continue;
                                    }
                                    let start = x;
                                    while x < LCD_WIDTH && on(x) {
                                        x += 1;
                                    }
                                    let b = Bounds::new(
                                        point(
                                            o.x + px(start as f32 * scale),
                                            o.y + px(y as f32 * scale),
                                        ),
                                        size(px((x - start) as f32 * scale), px(scale)),
                                    );
                                    window.paint_quad(fill(b, rgb(LCD_FG)));
                                }
                            }
                        },
                    )
                    .w(px(LCD_WIDTH as f32 * scale))
                    .h(px(LCD_HEIGHT as f32 * scale)),
                ),
            )
    }

    /// A key `w` columns wide and `h` rows tall.
    #[allow(clippy::too_many_arguments)]
    fn key(
        &self,
        id: &str,
        face: Face,
        cap: Cap,
        w: f32,
        h: f32,
        tone: Tone,
        g: Geo,
        cx: &mut Context<Self>,
    ) -> Stateful<Div> {
        let down = match cap {
            Cap::Key(k) => self.pressed.contains(&k),
            Cap::Power => self.mouse_key == Some(Cap::Power),
        };
        let (bg, hover, pressed, fg, edge) = match tone {
            Tone::Plain => (KEY, KEY_HOVER, KEY_DOWN, TEXT, KEY_EDGE),
            Tone::Accent => (ACCENT, ACCENT_HOVER, ACCENT_DOWN, ACCENT_TEXT, ACCENT_DOWN),
        };
        let height = h * g.row + (h - 1.0) * g.gap;
        let content: AnyElement = match face {
            Face::Text(t) => div().child(t).into_any_element(),
            Face::Icon(i) => Icon::new(i).size(px(20.0 * g.u)).into_any_element(),
            Face::IconText(i, t) => div()
                .flex()
                .flex_row()
                .items_center()
                .gap(px(8.0 * g.u))
                .child(Icon::new(i).size(px(18.0 * g.u)))
                .child(t)
                .into_any_element(),
        };
        div()
            .id(SharedString::from(id.to_string()))
            .w(px(g.span(w)))
            .h(px(height))
            .flex()
            .items_center()
            .justify_center()
            .rounded(px(10.0 * g.u))
            .bg(rgb(if down { pressed } else { bg }))
            .border_1()
            .border_color(rgb(edge))
            .text_color(rgb(fg))
            .text_size(px(14.0 * g.u))
            .font_weight(FontWeight::MEDIUM)
            .cursor_pointer()
            .when(!down, |d| d.hover(move |s| s.bg(rgb(hover))))
            .child(content)
            .on_mouse_down(
                MouseButton::Left,
                cx.listener(move |v, _, window, cx| {
                    v.mouse_key = Some(cap);
                    v.cap_down(cap);
                    v.focus.focus(window, cx);
                    cx.notify();
                }),
            )
            .on_mouse_up(
                MouseButton::Left,
                cx.listener(move |v, _, _, cx| {
                    if v.mouse_key.take().is_some() {
                        v.cap_up(cap);
                        cx.notify();
                    }
                }),
            )
    }

    fn device(&self, g: Geo, cx: &mut Context<Self>) -> impl IntoElement {
        let u = g.u;
        let row = || div().flex().flex_row().gap(px(g.gap));
        let col = || div().flex().flex_col().gap(px(g.gap));
        let txt = |s: &'static str| Face::Text(s);
        let icon = Face::Icon;
        let k = |s: &Self, id: &str, face: Face, key: Key, w: f32, cx: &mut Context<Self>| {
            s.key(id, face, Cap::Key(key), w, 1.0, Tone::Plain, g, cx)
        };
        let blank = || div().w(px(g.col)).h(px(g.row));

        let running = self.snap.stopped.is_none() && !self.snap.paused;
        let header = div()
            .flex()
            .flex_row()
            .items_center()
            .justify_between()
            .h(px(32.0 * u))
            .child(
                div()
                    .flex()
                    .flex_row()
                    .items_center()
                    .gap(px(8.0 * u))
                    .child(
                        div()
                            .w(px(8.0 * u))
                            .h(px(8.0 * u))
                            .rounded_full()
                            .bg(rgb(ACCENT)),
                    )
                    .child(
                        div()
                            .text_size(px(14.0 * u))
                            .font_weight(FontWeight::SEMIBOLD)
                            .text_color(rgb(TEXT))
                            .child("Hi-SCAN Pro"),
                    ),
            )
            .child(
                div()
                    .flex()
                    .flex_row()
                    .items_center()
                    .gap(px(10.0 * u))
                    .child(
                        div()
                            .flex()
                            .flex_row()
                            .items_center()
                            .gap(px(6.0 * u))
                            .text_size(px(12.0 * u))
                            .text_color(rgb(TEXT_DIM))
                            .child(
                                div()
                                    .w(px(7.0 * u))
                                    .h(px(7.0 * u))
                                    .rounded_full()
                                    .bg(rgb(if running { OK } else { WARN })),
                            )
                            .child(if running {
                                "running"
                            } else if self.snap.paused {
                                "paused"
                            } else {
                                "stopped"
                            }),
                    )
                    .child(
                        self.key(
                            "power",
                            icon(IconName::Power),
                            Cap::Power,
                            1.0,
                            1.0,
                            Tone::Plain,
                            g,
                            cx,
                        )
                        .w(px(32.0 * u))
                        .h(px(32.0 * u))
                        .rounded_full(),
                    ),
            );

        let fkeys = row()
            .child(k(self, "f1", txt("F1"), Key::F1, 1.0, cx))
            .child(k(self, "f2", txt("F2"), Key::F2, 1.0, cx))
            .child(k(self, "f3", txt("F3"), Key::F3, 1.0, cx))
            .child(k(self, "f4", txt("F4"), Key::F4, 1.0, cx))
            .child(k(self, "f5", txt("F5"), Key::F5, 1.0, cx))
            .child(k(self, "f6", txt("F6"), Key::F6, 1.0, cx));

        let r3 = |s: &Self,
                  a: (&'static str, Key),
                  b: (&'static str, Key),
                  c: (&'static str, Key),
                  cx: &mut Context<Self>| {
            row()
                .child(k(s, &format!("k{}", a.0), Face::Text(a.0), a.1, 1.0, cx))
                .child(k(s, &format!("k{}", b.0), Face::Text(b.0), b.1, 1.0, cx))
                .child(k(s, &format!("k{}", c.0), Face::Text(c.0), c.1, 1.0, cx))
        };
        let left = col()
            .child(
                row()
                    .child(k(self, "shift", txt("SHIFT"), Key::SHIFT, 1.0, cx))
                    .child(k(self, "help", txt("HELP"), Key::HELP, 1.0, cx))
                    .child(k(
                        self,
                        "light",
                        icon(IconName::Sun),
                        Key::BACKLIGHT,
                        1.0,
                        cx,
                    )),
            )
            .child(r3(self, ("1", Key::N1), ("2", Key::N2), ("3", Key::N3), cx))
            .child(r3(self, ("4", Key::N4), ("5", Key::N5), ("6", Key::N6), cx))
            .child(r3(self, ("7", Key::N7), ("8", Key::N8), ("9", Key::N9), cx))
            .child(r3(
                self,
                ("NO", Key::NO),
                ("0", Key::N0),
                ("YES", Key::YES),
                cx,
            ));

        let right = col()
            .child(
                row()
                    .child(blank())
                    .child(k(self, "up", icon(IconName::ChevronUp), Key::UP, 1.0, cx))
                    .child(blank()),
            )
            .child(
                row()
                    .child(k(
                        self,
                        "left",
                        icon(IconName::ChevronLeft),
                        Key::LEFT,
                        1.0,
                        cx,
                    ))
                    .child(k(
                        self,
                        "down",
                        icon(IconName::ChevronDown),
                        Key::DOWN,
                        1.0,
                        cx,
                    ))
                    .child(k(
                        self,
                        "right",
                        icon(IconName::ChevronRight),
                        Key::RIGHT,
                        1.0,
                        cx,
                    )),
            )
            .child(
                row()
                    .child(
                        self.key(
                            "esc",
                            txt("ESC"),
                            Cap::Key(Key::ESCAPE),
                            1.5,
                            1.0,
                            Tone::Plain,
                            g,
                            cx,
                        )
                        .w(px((g.span(3.0) - g.gap) / 2.0)),
                    )
                    .child(
                        self.key(
                            "undo",
                            Face::IconText(IconName::Undo2, "UNDO"),
                            Cap::Key(Key::UNDO),
                            1.5,
                            1.0,
                            Tone::Plain,
                            g,
                            cx,
                        )
                        .w(px((g.span(3.0) - g.gap) / 2.0)),
                    ),
            )
            .child(self.key(
                "enter",
                Face::IconText(IconName::CornerDownLeft, "ENTER"),
                Cap::Key(Key::ENTER),
                3.0,
                2.0,
                Tone::Accent,
                g,
                cx,
            ));

        div()
            .flex()
            .flex_col()
            .gap(px(16.0 * u))
            .p(px(20.0 * u))
            .rounded(px(28.0 * u))
            .bg(rgb(CARD))
            .border_1()
            .border_color(rgb(CARD_EDGE))
            .shadow_lg()
            .child(header)
            .child(self.lcd(g))
            .child(div().h(px(2.0 * u)))
            .child(fkeys)
            .child(
                div()
                    .flex()
                    .flex_row()
                    .justify_between()
                    .child(left)
                    .child(right),
            )
    }

    /// Port list and options shown when "Adapter" is selected.
    fn adapter_panel(&self, cx: &mut Context<Self>) -> impl IntoElement {
        let ports =
            self.ports
                .iter()
                .enumerate()
                .fold(div().flex().flex_col().gap(px(4.)), |d, (i, p)| {
                    let selected = self.port.as_deref() == Some(p.as_str());
                    let name = p.trim_start_matches("/dev/cu.").to_string();
                    let path = p.clone();
                    let b = Button::new(SharedString::from(format!("port-{i}")))
                        .label(name)
                        .on_click(cx.listener(move |v, _, _, cx| {
                            v.use_port(path.clone());
                            cx.notify();
                        }));
                    d.child(if selected { b.primary() } else { b.ghost() })
                });
        let echo = self.adapter_echo;
        div()
            .flex()
            .flex_col()
            .gap(px(6.))
            .p(px(8.))
            .rounded(px(8.))
            .bg(rgb(CARD))
            .text_sm()
            .when(self.ports.is_empty(), |d| {
                d.child(
                    div()
                        .text_color(rgb(TEXT_DIM))
                        .child("No adapter found. Plug it in and rescan."),
                )
            })
            .child(ports)
            .child(
                div()
                    .flex()
                    .flex_row()
                    .gap(px(6.))
                    .child(
                        Button::new("rescan")
                            .icon(IconName::RefreshCw)
                            .label("Rescan")
                            .ghost()
                            .on_click(cx.listener(|v, _, _, cx| {
                                v.ports = cascade_diag::serial::list_ports();
                                cx.notify();
                            })),
                    )
                    .child(
                        Button::new("echo")
                            .label(if echo { "Echo: on" } else { "Echo: off" })
                            .ghost()
                            .on_click(cx.listener(move |v, _, _, cx| {
                                v.adapter_echo = !echo;
                                if let Some(p) = v.port.clone() {
                                    v.use_port(p);
                                }
                                cx.notify();
                            })),
                    ),
            )
            .child(div().text_xs().text_color(rgb(TEXT_DIM)).child(
                "Runs the original Hi-Scan software on the car. Its menus include actuator \
                 tests and resets: use them only on purpose.",
            ))
    }

    fn side(&self, cx: &mut Context<Self>) -> impl IntoElement {
        let s = &self.snap;
        let paused = s.paused;
        let section = |t: &'static str| {
            div()
                .text_xs()
                .font_weight(FontWeight::SEMIBOLD)
                .text_color(rgb(TEXT_DIM))
                .child(t)
        };
        let seg = |id: String, label: &'static str, selected: bool| {
            let b = Button::new(SharedString::from(id)).label(label).flex_1();
            if selected { b.primary() } else { b.ghost() }
        };
        let speeds = [
            ("1×", Speed::Real(1.0)),
            ("4×", Speed::Real(4.0)),
            ("Max", Speed::Max),
        ];
        let speed_row = speeds.iter().fold(
            div()
                .flex()
                .flex_row()
                .gap(px(4.))
                .p(px(3.))
                .rounded(px(8.))
                .bg(rgb(CARD)),
            |d, &(label, speed)| {
                d.child(
                    seg(format!("speed-{label}"), label, s.speed == speed).on_click(
                        cx.listener(move |v, _, _, _| v.runner.send(Command::Speed(speed))),
                    ),
                )
            },
        );
        let line_row = IFACES.iter().enumerate().fold(
            div()
                .flex()
                .flex_row()
                .gap(px(4.))
                .p(px(3.))
                .rounded(px(8.))
                .bg(rgb(CARD)),
            |d, (i, name)| {
                d.child(
                    seg(format!("iface-{i}"), name, self.iface_choice == i).on_click(cx.listener(
                        move |v, _, _, cx| {
                            v.iface_choice = i;
                            if i == ADAPTER {
                                // pick a port first; nothing changes until then
                                v.ports = cascade_diag::serial::list_ports();
                            } else {
                                v.runner.send(Command::Restart(iface_kind(i)));
                            }
                            cx.notify();
                        },
                    )),
                )
            },
        );
        let kv = |k: &'static str, v: String| {
            div()
                .flex()
                .flex_row()
                .justify_between()
                .text_sm()
                .child(div().text_color(rgb(TEXT_DIM)).child(k))
                .child(div().font_family("Menlo").child(v))
        };
        let secs = s.total_cycles as f64 / 10_000_000.0;
        let log_lines: Vec<String> = s.log.iter().rev().take(200).cloned().collect();
        div()
            .flex()
            .flex_col()
            .gap(px(10.))
            .w(px(PANEL_W))
            .h_full()
            .p(px(16.))
            .bg(rgb(PANEL))
            .border_l_1()
            .border_color(rgb(CARD_EDGE))
            .text_color(rgb(TEXT))
            .child(section("EMULATOR"))
            .child(
                div()
                    .flex()
                    .flex_row()
                    .gap(px(6.))
                    .child(
                        Button::new("run")
                            .icon(if paused {
                                IconName::Play
                            } else {
                                IconName::Pause
                            })
                            .label(if paused { "Run" } else { "Pause" })
                            .primary()
                            .flex_1()
                            .on_click(cx.listener(move |v, _, _, _| {
                                v.runner.send(Command::Pause(!paused))
                            })),
                    )
                    .child(
                        Button::new("reset")
                            .icon(IconName::RotateCcw)
                            .label("Reset")
                            .flex_1()
                            .on_click(cx.listener(|v, _, _, _| v.runner.send(Command::Reset))),
                    ),
            )
            .child(speed_row)
            .child(div().h(px(4.)))
            .child(section("DIAGNOSTIC LINE"))
            .child(line_row)
            .when(self.iface_choice == ADAPTER, |d| {
                d.child(self.adapter_panel(cx))
            })
            .child(div().h(px(4.)))
            .child(section("STATUS"))
            .child(
                div()
                    .flex()
                    .flex_col()
                    .gap(px(4.))
                    .child(kv("Emulated", format!("{secs:.1} s")))
                    .child(kv("Speed", format!("×{:.2}", s.ratio)))
                    .child(kv("PC", format!("{:04X}", s.pc)))
                    .child(kv("Firmware resets", s.resets.to_string()))
                    .child(kv("UART TX", s.uart_tx.to_string()))
                    .child(kv("Line", s.iface.clone())),
            )
            .when_some(s.stopped.clone(), |d, why| {
                d.child(
                    div()
                        .p(px(8.))
                        .rounded(px(6.))
                        .bg(rgb(0x3a1d1d))
                        .text_color(rgb(WARN))
                        .text_sm()
                        .child(format!("Stopped: {why}")),
                )
            })
            .child(div().h(px(4.)))
            .child(section("LINE EVENTS"))
            .child(
                div()
                    .id("log")
                    .flex_1()
                    .overflow_y_scroll()
                    .rounded(px(8.))
                    .bg(rgb(LOG_BG))
                    .border_1()
                    .border_color(rgb(CARD_EDGE))
                    .p(px(8.))
                    .font_family("Menlo")
                    .text_xs()
                    .text_color(rgb(0xb5bac2))
                    .children(log_lines.into_iter().map(|l| div().child(l))),
            )
    }
}

impl Render for CascadeView {
    fn render(&mut self, window: &mut Window, cx: &mut Context<Self>) -> impl IntoElement {
        let g = Geo::new(Self::lcd_scale(window));
        div()
            .track_focus(&self.focus)
            .size_full()
            .flex()
            .flex_row()
            .bg(rgb(DESK))
            .on_key_down(cx.listener(|v, e: &KeyDownEvent, _, cx| {
                let k = &e.keystroke;
                if e.is_held || k.modifiers.platform || k.modifiers.control {
                    return;
                }
                if let Some(key) = map_key(&k.key) {
                    v.cap_down(Cap::Key(key));
                    cx.stop_propagation();
                    cx.notify();
                }
            }))
            .on_key_up(cx.listener(|v, e: &KeyUpEvent, _, cx| {
                if let Some(key) = map_key(&e.keystroke.key) {
                    v.cap_up(Cap::Key(key));
                    cx.notify();
                }
            }))
            .on_mouse_up_out(
                MouseButton::Left,
                cx.listener(|v, _, _, cx| {
                    if let Some(c) = v.mouse_key.take() {
                        v.cap_up(c);
                        cx.notify();
                    }
                }),
            )
            .child(
                div()
                    .flex_1()
                    .h_full()
                    .flex()
                    .items_center()
                    .justify_center()
                    .child(self.device(g, cx)),
            )
            .child(self.side(cx))
    }
}

fn main() {
    let Some(path) = std::env::args()
        .nth(1)
        .or_else(|| std::env::var("CASCADE_ROM").ok())
    else {
        eprintln!("usage: cascade-gui ROM   (or set CASCADE_ROM)");
        std::process::exit(2);
    };
    let rom = match std::fs::read(&path) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("cascade-gui: {path}: {e}");
            std::process::exit(2);
        }
    };
    gpui_kit::application()
        .with_assets(gpui_kit::assets::AllAssets)
        .run(move |cx| {
            gpui_kit::init(cx);
            // the panel is dark; gpui-component's buttons must match
            gpui_kit::component::Theme::change(gpui_kit::component::ThemeMode::Dark, None, cx);
            // as tall as the display allows; the device scales to the window
            let display_h = cx
                .primary_display()
                .map(|d| f32::from(d.bounds().size.height))
                .unwrap_or(1000.0);
            let h = (display_h - 80.0).clamp(700.0, 1400.0);
            let w = h * DEVICE_W_PER_SCALE / DEVICE_H_PER_SCALE + PANEL_W + 40.0;
            let bounds = Bounds::centered(None, size(px(w), px(h)), cx);
            let opts = WindowOptions {
                window_bounds: Some(WindowBounds::Windowed(bounds)),
                titlebar: Some(TitlebarOptions {
                    title: Some("Cascade".into()),
                    ..Default::default()
                }),
                ..Default::default()
            };
            gpui_kit::open_window(opts, cx, |window, cx| {
                cx.new(|cx| CascadeView::new(rom, window, cx))
            })
            .expect("failed to open window");
            cx.on_window_closed(|cx, _| cx.quit()).detach();
            cx.activate(true);
        });
}
