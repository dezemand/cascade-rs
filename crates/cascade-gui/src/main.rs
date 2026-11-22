//! cascade-gui: desktop front end for the cascade-rs Hi-Scan emulator.
//!
//! Usage: cascade-gui [ROM [--exrom INFO.dat] [--eeprom FILE | --no-eeprom]
//!        [--replay KDIAG.jsonl] [--ecu-rom SIMK41.bin]]
//! (or set CASCADE_ROM, CASCADE_EXROM, CASCADE_ECU_ROM). Without a ROM a
//! setup screen asks for the pack, the optional troubleshooting pack and
//! engine ECU dump, and remembers them.
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

mod cli;
mod live;

use cascade_core::Key;
use cascade_core::lcd::{LCD_HEIGHT, LCD_WIDTH};
use cascade_iface::KwpConfig;
use cascade_runner::{Command, IfaceKind, Runner, Snapshot, Speed};
use gpui_kit::assets::IconName;
use gpui_kit::component::Icon;
use gpui_kit::prelude::FluentBuilder as _;
use gpui_kit::*;

// The handset's own colours (neutral darks, the Hi-Scan yellow on ENTER).
const CARD: u32 = 0x1c1f23;
/// The darker face around the brand line and the screen.
const FACE: u32 = 0x131518;
const CARD_EDGE: u32 = 0x2b2f34;
const BEZEL: u32 = 0x08090a;
const LCD_BG: u32 = 0xc6cfb6;
const LCD_FG: u32 = 0x1b2318;
const KEY: u32 = 0x2b2f35;
/// Function keys: quieter than the rest.
const FKEY: u32 = 0x22262b;
const FKEY_TEXT: u32 = 0xa9b0b8;
const KEY_HOVER: u32 = 0x343940;
const KEY_DOWN: u32 = 0x3e444c;
const TEXT: u32 = 0xe7e9ec;
const TEXT_DIM: u32 = 0x8a8f98;
const ACCENT: u32 = 0xf2c14e;
const ACCENT_HOVER: u32 = 0xf7cd68;
const ACCENT_DOWN: u32 = 0xd9a738;
const ACCENT_TEXT: u32 = 0x1d1606;
const OK: u32 = 0x4ade80;
const WARN: u32 = 0xf87171;

/// The app's chrome (side panel, live data) follows the platform; the
/// handset keeps its own colours above.
use cascade_theme::{self as ui, Palette};

/// What each diagnostic line is, for people: name and one line.
const LINE_INFO: [(&str, &str); 7] = [
    ("Canned responder", "Upstream CASCADE's fixed replies"),
    ("Engine ECU model", "KWP2000 model with warm-idle values"),
    ("Engine ECU dump", "The ECU's own code, emulated"),
    ("Nothing", "An open line: nothing answers"),
    ("Car, via adapter", "A serial K-line adapter on a real car"),
    ("Recorded session", "Answers from a kdiag trace"),
    ("Simulated car", "The ecusim process (EcuSim)"),
];

/// The side panel's tabs.
#[derive(Clone, Copy, PartialEq, Eq)]
enum SideTab {
    Connection,
    Emulator,
    Developer,
}

/// Toolbar height in points.
const TOOLBAR_H: f32 = 40.0;

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

/// Diagnostic lines by index (names in LINE_INFO): 0 canned responder
/// (upstream fake), 1 KWP2000 model, 2 ECU dump, 3 none, 4 adapter,
/// 5 replay, 6 ecusim process.
/// Index of "ECU" (the engine ECU's own code, a SIMK41 dump in ecuc167).
const ECU: usize = 2;
/// Index of the serial adapter ("Port") in LINE_INFO.
const ADAPTER: usize = 4;
/// Index of "Replay" (a `kdiag --trace` recording) in LINE_INFO.
const REPLAY: usize = 5;
/// Index of "Process": an `ecusim serve` process (the ECU dump running in
/// real time with an engine model) on its default socket.
const PROCESS: usize = 6;

/// The line to an `ecusim` server (connected inside the emulator thread).
fn process_iface() -> IfaceKind {
    let sock = ecusim::default_socket();
    let label = "ECU process (ecusim)".to_string();
    // waits for (and survives restarts of) the server, like a cable that
    // gets plugged in later
    let make: cascade_runner::IfaceFactory = std::sync::Arc::new(move || {
        Ok(Box::new(ecusim::client::SocketKLine::waiting(&sock))
            as Box<dyn cascade_iface::Interface>)
    });
    IfaceKind::Custom { label, make }
}

/// An engine ECU dump for the "ECU" line option. Proprietary; loaded from
/// outside the repository at run time.
#[derive(Clone)]
struct EcuRom {
    name: String,
    data: std::sync::Arc<Vec<u8>>,
}

fn load_ecu(path: &std::path::Path) -> Result<EcuRom, String> {
    let data = std::fs::read(path).map_err(|e| e.to_string())?;
    if data.is_empty() || !data.len().is_power_of_two() {
        return Err(format!("{} bytes: not a ROM dump", data.len()));
    }
    Ok(EcuRom {
        name: path.file_name().map_or_else(
            || path.display().to_string(),
            |n| n.to_string_lossy().into(),
        ),
        data: std::sync::Arc::new(data),
    })
}

/// The SIMK41 code behind the line (built inside the emulator thread).
fn ecu_iface(r: &EcuRom) -> IfaceKind {
    let short: String = r.name.chars().take(8).collect();
    let data = r.data.clone();
    let make: cascade_runner::IfaceFactory = std::sync::Arc::new(move || {
        Ok(
            Box::new(ecuc167::iface::Simk41::new((*data).clone(), 16_000_000))
                as Box<dyn cascade_iface::Interface>,
        )
    });
    IfaceKind::Custom {
        label: format!("SIMK41 ECU {short}"),
        make,
    }
}

/// A loaded recording: file name, ECU configuration, one-line summary.
struct Loaded {
    name: String,
    config: KwpConfig,
    summary: String,
}

fn load_trace(path: &std::path::Path) -> Result<Loaded, String> {
    let text = std::fs::read_to_string(path).map_err(|e| e.to_string())?;
    let rec = cascade_iface::replay::from_jsonl(&text)?;
    let summary = format!(
        "{} answers to {} requests, ECU {}{}",
        rec.pairs(),
        rec.sequences.len(),
        rec.address.map_or("?".into(), |a| format!("{a:02X}")),
        if rec.unanswered > 0 {
            format!(", {} unanswered", rec.unanswered)
        } else {
            String::new()
        }
    );
    Ok(Loaded {
        name: path.file_name().map_or_else(
            || path.display().to_string(),
            |n| n.to_string_lossy().into(),
        ),
        config: rec.config(),
        summary,
    })
}

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
    /// F1-F6: a quieter row above the keypad.
    Function,
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
    /// The recording for "Replay", or why the last one failed to load.
    replay: Option<Loaded>,
    replay_err: Option<String>,
    /// The engine ECU dump for "ECU", or why the last one failed to load.
    ecu: Option<EcuRom>,
    ecu_err: Option<String>,
    /// Show the native live-data table instead of the handset.
    live_view: bool,
    live: Option<live::Live>,
    live_snap: live::LiveSnap,
    /// Port for live data (None = simulator).
    live_port: Option<String>,
    /// Extended ROM (troubleshooting pack) behind the main pack, if any.
    exrom_name: Option<String>,
    /// The side panel is shown, and which of its tabs.
    sidebar: bool,
    tab: SideTab,
    _appearance: Subscription,
    _refresh: Task<()>,
}

impl CascadeView {
    fn new(
        rom: Vec<u8>,
        replay: Option<Result<Loaded, String>>,
        ecu: (Option<Result<EcuRom, String>>, bool),
        exrom: Option<(String, Vec<u8>)>,
        eeprom: Option<std::path::PathBuf>,
        window: &mut Window,
        cx: &mut Context<Self>,
    ) -> Self {
        let (replay, replay_err) = match replay {
            Some(Ok(l)) => (Some(l), None),
            Some(Err(e)) => (None, Some(e)),
            None => (None, None),
        };
        let (ecu, ecu_first) = ecu;
        let (ecu, ecu_err) = match ecu {
            Some(Ok(r)) => (Some(r), None),
            Some(Err(e)) => (None, Some(e)),
            None => (None, None),
        };
        let first = match (&replay, &ecu) {
            (Some(l), _) => IfaceKind::Kwp(l.config.clone()),
            (None, Some(r)) if ecu_first => ecu_iface(r),
            _ => iface_kind(0),
        };
        let exrom_name = exrom.as_ref().map(|(n, _)| n.clone());
        let runner = Runner::spawn_full(
            rom,
            exrom.map(|(_, x)| x),
            eeprom,
            first,
            cascade_runner::Speed::Real(1.0),
            None,
        );
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
                    let live_changed = v.live.as_ref().is_some_and(|l| l.seq() != v.live_snap.seq);
                    if live_changed && let Some(l) = &v.live {
                        v.live_snap = l.snapshot();
                        cx.notify();
                    }
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
            iface_choice: if replay.is_some() || replay_err.is_some() {
                REPLAY
            } else if ecu_first {
                ECU
            } else {
                0
            },
            ports: Vec::new(),
            port: None,
            adapter_echo: true,
            replay,
            replay_err,
            ecu,
            ecu_err,
            live_view: false,
            live: None,
            live_snap: live::LiveSnap::default(),
            live_port: None,
            exrom_name,
            sidebar: true,
            tab: SideTab::Connection,
            _appearance: cx.observe_window_appearance(window, |_, window, cx| {
                gpui_kit::component::Theme::sync_system_appearance(Some(window), cx);
                cx.notify();
            }),
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
            (ECU, _) => match &self.ecu {
                Some(r) => ecu_iface(r),
                None => IfaceKind::Null,
            },
            (REPLAY, _) => match &self.replay {
                Some(l) => IfaceKind::Kwp(l.config.clone()),
                None => IfaceKind::Null,
            },
            (PROCESS, _) => process_iface(),
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

    /// Ask for a `kdiag --trace` file and power-cycle onto it.
    fn open_trace(&mut self, cx: &mut Context<Self>) {
        let rx = cx.prompt_for_paths(PathPromptOptions {
            files: true,
            directories: false,
            multiple: false,
            prompt: Some("Replay".into()),
        });
        cx.spawn(async move |this, cx| {
            if let Ok(Ok(Some(paths))) = rx.await
                && let Some(p) = paths.first()
            {
                let loaded = load_trace(p);
                let _ = this.update(cx, |v, cx| {
                    match loaded {
                        Ok(l) => {
                            v.replay = Some(l);
                            v.replay_err = None;
                            v.runner.send(Command::Restart(v.current_iface()));
                        }
                        Err(e) => v.replay_err = Some(format!("{}: {e}", p.display())),
                    }
                    cx.notify();
                });
            }
        })
        .detach();
    }

    /// Ask for an engine ECU dump and power-cycle onto it.
    fn open_ecu(&mut self, cx: &mut Context<Self>) {
        let rx = cx.prompt_for_paths(PathPromptOptions {
            files: true,
            directories: false,
            multiple: false,
            prompt: Some("Use as ECU".into()),
        });
        cx.spawn(async move |this, cx| {
            if let Ok(Ok(Some(paths))) = rx.await
                && let Some(p) = paths.first()
            {
                let loaded = load_ecu(p);
                let _ = this.update(cx, |v, cx| {
                    match loaded {
                        Ok(r) => {
                            v.ecu = Some(r);
                            v.ecu_err = None;
                            v.runner.send(Command::Restart(v.current_iface()));
                        }
                        Err(e) => v.ecu_err = Some(format!("{}: {e}", p.display())),
                    }
                    cx.notify();
                });
            }
        })
        .detach();
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

    /// LCD scale for the current window: points per LCD pixel, as large as
    /// the window allows (see `lcd` for how it stays sharp).
    fn lcd_scale(&self, window: &Window) -> f32 {
        let vp = window.viewport_size();
        let side = if self.sidebar { PANEL_W } else { 0.0 };
        let fit = ((f32::from(vp.width) - side - 24.0) / DEVICE_W_PER_SCALE)
            .min((f32::from(vp.height) - TOOLBAR_H - 24.0) / DEVICE_H_PER_SCALE);
        // continuous, in 1/16 steps so small resizes do not relayout; the
        // LCD stays sharp because its pixels are snapped to the screen's
        ((fit * 16.0).floor() / 16.0).clamp(1.0, 6.0)
    }

    fn lcd(&self, g: Geo) -> impl IntoElement {
        let frame = self.snap.frame.clone();
        let scale = g.scale;
        div()
            .p(px(4.0 * g.u))
            .rounded(px(6.0 * g.u))
            .bg(rgb(BEZEL))
            .child(
                div().rounded(px(4.0 * g.u)).overflow_hidden().child(
                    canvas(
                        |_, _, _| (),
                        move |bounds, _, window, _| {
                            window.paint_quad(fill(bounds, rgb(LCD_BG)));
                            let o = bounds.origin;
                            // LCD pixel edges on whole screen pixels: neighbours
                            // share an edge, nothing is blurred, sizes differ by
                            // at most one screen pixel
                            let dpr = window.scale_factor();
                            let snap = |v: f32| (v * dpr).round() / dpr;
                            let ex = |i: usize| snap(f32::from(o.x) + i as f32 * scale);
                            let ey = |i: usize| snap(f32::from(o.y) + i as f32 * scale);
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
                                        point(px(ex(start)), px(ey(y))),
                                        size(px(ex(x) - ex(start)), px(ey(y + 1) - ey(y))),
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
            // filled keys, no visible outline
            Tone::Plain => (KEY, KEY_HOVER, KEY_DOWN, TEXT, KEY),
            Tone::Function => (FKEY, KEY_HOVER, KEY_DOWN, FKEY_TEXT, FKEY),
            Tone::Accent => (ACCENT, ACCENT_HOVER, ACCENT_DOWN, ACCENT_TEXT, ACCENT),
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
            .rounded(px(7.0 * g.u))
            .bg(rgb(if down { pressed } else { bg }))
            .border_1()
            .border_color(rgb(edge))
            .text_color(rgb(fg))
            .text_size(px(13.0 * g.u))
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
            .h(px(26.0 * u))
            .px(px(4.0 * u))
            .child(
                div()
                    .flex()
                    .flex_row()
                    .items_center()
                    .gap(px(5.0 * u))
                    .child(
                        div()
                            .text_size(px(13.0 * u))
                            .font_weight(FontWeight::BOLD)
                            .text_color(rgb(TEXT))
                            .child("HI-SCAN"),
                    )
                    .child(
                        div()
                            .text_size(px(13.0 * u))
                            .text_color(rgb(TEXT_DIM))
                            .child("PRO"),
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
                        .w(px(26.0 * u))
                        .h(px(26.0 * u))
                        .bg(rgb(FACE))
                        .rounded_full(),
                    ),
            );

        let f = |s: &Self, id: &str, t: &'static str, key: Key, cx: &mut Context<Self>| {
            s.key(id, txt(t), Cap::Key(key), 1.0, 0.72, Tone::Function, g, cx)
        };
        // as wide as the keypad below (which has a gap between its blocks)
        let fkeys = row()
            .justify_between()
            .child(f(self, "f1", "F1", Key::F1, cx))
            .child(f(self, "f2", "F2", Key::F2, cx))
            .child(f(self, "f3", "F3", Key::F3, cx))
            .child(f(self, "f4", "F4", Key::F4, cx))
            .child(f(self, "f5", "F5", Key::F5, cx))
            .child(f(self, "f6", "F6", Key::F6, cx));

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
            .p(px(16.0 * u))
            .rounded(px(20.0 * u))
            .bg(rgb(CARD))
            .border_1()
            .border_color(rgb(CARD_EDGE))
            .shadow_md()
            .child(
                div()
                    .flex()
                    .flex_col()
                    .gap(px(8.0 * u))
                    .p(px(8.0 * u))
                    .rounded(px(12.0 * u))
                    .bg(rgb(FACE))
                    .child(header)
                    .child(self.lcd(g)),
            )
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

    /// Adapter ports and options (line "Car, via adapter").
    fn adapter_panel(&self, p: Palette, cx: &mut Context<Self>) -> Div {
        let ports = self.ports.iter().enumerate().fold(
            div().flex().flex_col().gap(px(2.)),
            |d, (i, port)| {
                let selected = self.port.as_deref() == Some(port.as_str());
                let name = port.trim_start_matches("/dev/cu.").to_string();
                let path = port.clone();
                d.child(
                    div()
                        .id(("port", i))
                        .px(px(8.))
                        .py(px(3.))
                        .rounded(px(ui::RADIUS))
                        .text_size(px(ui::BODY))
                        .cursor_pointer()
                        .map(|d| {
                            if selected {
                                d.bg(rgb(p.accent)).text_color(rgb(p.on_accent))
                            } else {
                                d.text_color(rgb(p.label))
                                    .hover(move |s| s.bg(rgb(p.fill_hover)))
                            }
                        })
                        .child(name)
                        .on_click(cx.listener(move |v, _, _, cx| {
                            v.use_port(path.clone());
                            cx.notify();
                        })),
                )
            },
        );
        let echo = self.adapter_echo;
        div()
            .flex()
            .flex_col()
            .gap(px(8.))
            .when(self.ports.is_empty(), |d| {
                d.child(ui::note(p, "No adapter found. Plug it in and rescan."))
            })
            .child(ports)
            .child(
                div()
                    .flex()
                    .flex_row()
                    .gap(px(6.))
                    .child(
                        ui::button(p, "rescan", Some(IconName::RefreshCw), "Rescan").on_click(
                            cx.listener(|v, _, _, cx| {
                                v.ports = cascade_diag::serial::list_ports();
                                cx.notify();
                            }),
                        ),
                    )
                    .child(
                        ui::button(p, "echo", None, if echo { "Echo on" } else { "Echo off" })
                            .on_click(cx.listener(move |v, _, _, cx| {
                                v.adapter_echo = !echo;
                                if let Some(port) = v.port.clone() {
                                    v.use_port(port);
                                }
                                cx.notify();
                            })),
                    ),
            )
            .child(ui::note(
                p,
                "Runs the original Hi-Scan software on the car. Its menus include \
                 actuator tests and resets: use them only on purpose.",
            ))
    }

    /// The loaded ECU dump (line "Engine ECU dump").
    fn ecu_panel(&self, p: Palette, cx: &mut Context<Self>) -> Div {
        div()
            .flex()
            .flex_col()
            .gap(px(6.))
            .child(match &self.ecu {
                Some(r) => div()
                    .flex()
                    .flex_col()
                    .gap(px(2.))
                    .child(
                        div()
                            .truncate()
                            .text_size(px(ui::BODY))
                            .text_color(rgb(p.label))
                            .child(r.name.clone()),
                    )
                    .child(ui::note(
                        p,
                        format!("{} KB C167 engine ECU, address 0x11", r.data.len() / 1024),
                    )),
                None => ui::note(p, "No ECU dump loaded."),
            })
            .when_some(self.ecu_err.clone(), |d, e| {
                d.child(ui::note(p, e).text_color(rgb(p.red)))
            })
            .child(
                div().flex().flex_row().child(
                    ui::button(p, "open-ecu", None, "Open ECU Dump…")
                        .on_click(cx.listener(|v, _, _, cx| v.open_ecu(cx))),
                ),
            )
            .child(ui::note(
                p,
                "Use Tiburon / Engine L4 / 2003. It boots at the first wake-up \
                 (about 3 s of ECU time).",
            ))
    }

    /// The loaded recording (line "Recorded session").
    fn replay_panel(&self, p: Palette, cx: &mut Context<Self>) -> Div {
        div()
            .flex()
            .flex_col()
            .gap(px(6.))
            .child(match &self.replay {
                Some(l) => div()
                    .flex()
                    .flex_col()
                    .gap(px(2.))
                    .child(
                        div()
                            .truncate()
                            .text_size(px(ui::BODY))
                            .text_color(rgb(p.label))
                            .child(l.name.clone()),
                    )
                    .child(ui::note(p, l.summary.clone())),
                None => ui::note(p, "No recording loaded."),
            })
            .when_some(self.replay_err.clone(), |d, e| {
                d.child(ui::note(p, e).text_color(rgb(p.red)))
            })
            .child(
                div().flex().flex_row().child(
                    ui::button(p, "open-trace", None, "Open kdiag Trace…")
                        .on_click(cx.listener(|v, _, _, cx| v.open_trace(cx))),
                ),
            )
    }

    /// Native live data: engine `21 01` through kdiag's client, decoded with
    /// the protocol CSV. Independent of the emulator.
    fn live_panel(&self, p: Palette, cx: &mut Context<Self>) -> impl IntoElement {
        let s = &self.live_snap;
        let running = self.live.is_some() && s.running;
        let ports = cascade_diag::serial::list_ports();
        let sources = std::iter::once(None)
            .chain(ports.into_iter().map(Some))
            .enumerate()
            .fold(ui::segmented(p), |d, (i, port)| {
                let label = port.as_deref().map_or("Simulator".to_string(), |port| {
                    port.trim_start_matches("/dev/cu.").to_string()
                });
                let selected = self.live_port == port;
                d.child(
                    ui::segment(p, ("live-src", i), label, selected).on_click(cx.listener(
                        move |v, _, _, cx| {
                            v.live_port = port.clone();
                            v.live = None;
                            cx.notify();
                        },
                    )),
                )
            });
        let start = if running {
            ui::button(p, "live-start", Some(IconName::Pause), "Stop")
        } else {
            ui::default_button(p, "live-start", Some(IconName::Play), "Start")
        }
        .on_click(cx.listener(move |v, _, _, cx| {
            if running {
                v.live = None;
                v.live_snap.running = false;
                v.live_snap.status = "stopped".into();
            } else {
                let src = match &v.live_port {
                    None => live::Source::Sim,
                    Some(port) => live::Source::Port {
                        path: port.clone(),
                        echo: v.adapter_echo,
                    },
                };
                v.live = Some(live::Live::start(src));
            }
            cx.notify();
        }));
        let rows = s.rows.iter().enumerate().fold(
            div().flex().flex_col().w_full(),
            |d, (i, (name, value, unit))| {
                d.child(
                    div()
                        .flex()
                        .flex_row()
                        .items_center()
                        .px(px(10.))
                        .py(px(3.))
                        .when(i % 2 == 1, |d| {
                            d.bg(rgb(if p.dark { 0x252525 } else { 0xf4f5f5 }))
                        })
                        .child(
                            div()
                                .flex_1()
                                .text_color(rgb(p.label))
                                .child(name.replace('_', " ")),
                        )
                        .child(
                            div()
                                .w(px(110.))
                                .text_right()
                                .font_family(ui::mono())
                                .text_color(rgb(p.label))
                                .child(value.clone()),
                        )
                        .child(
                            div()
                                .w(px(56.))
                                .pl(px(8.))
                                .text_color(rgb(p.secondary))
                                .child(unit.clone()),
                        ),
                )
            },
        );
        div()
            .flex()
            .flex_col()
            .gap(px(10.))
            .w(px(560.))
            .max_h_full()
            .text_size(px(ui::BODY))
            .text_color(rgb(p.label))
            .child(
                div()
                    .flex()
                    .flex_row()
                    .justify_between()
                    .items_center()
                    .child(
                        div()
                            .flex()
                            .flex_col()
                            .gap(px(2.))
                            .child(
                                div()
                                    .font_weight(FontWeight::SEMIBOLD)
                                    .child("Engine Live Data"),
                            )
                            .child(ui::note(p, "kdiag client, read-only; 21 01 every 150 ms")),
                    )
                    .child(start),
            )
            .child(sources)
            .child(ui::note(
                p,
                format!(
                    "{} · {} reads · {} errors",
                    if s.status.is_empty() {
                        "idle"
                    } else {
                        &s.status
                    },
                    s.reads,
                    s.errors
                ),
            ))
            .child(
                div()
                    .id("live-rows")
                    .flex_1()
                    .overflow_y_scroll()
                    .rounded(px(ui::RADIUS))
                    .border_1()
                    .border_color(rgb(p.separator))
                    .bg(rgb(p.panel))
                    .child(rows),
            )
    }

    /// Choose diagnostic line `i`.
    fn choose_line(&mut self, i: usize, cx: &mut Context<Self>) {
        self.iface_choice = i;
        if i == ADAPTER {
            // pick a port first; nothing changes until then
            self.ports = cascade_diag::serial::list_ports();
        } else if i == ECU {
            if self.ecu.is_some() {
                self.runner.send(Command::Restart(self.current_iface()));
            } else {
                self.open_ecu(cx);
            }
        } else if i == PROCESS {
            // a real-time partner: run at real time
            self.runner.send(Command::Speed(Speed::Real(1.0)));
            self.runner.send(Command::Restart(self.current_iface()));
        } else if i == REPLAY {
            if self.replay.is_some() {
                self.runner.send(Command::Restart(self.current_iface()));
            } else {
                self.open_trace(cx);
            }
        } else {
            self.runner.send(Command::Restart(iface_kind(i)));
        }
        cx.notify();
    }

    /// Connection tab: the lines to choose from, and the chosen line's own
    /// controls.
    fn connection(&self, p: Palette, cx: &mut Context<Self>) -> Div {
        let list = LINE_INFO.iter().enumerate().fold(
            div().flex().flex_col().gap(px(1.)),
            |d, (i, (n, w))| {
                let on = i == self.iface_choice;
                d.child(
                    div()
                        .id(("line", i))
                        .flex()
                        .flex_col()
                        .px(px(8.))
                        .py(px(4.))
                        .rounded(px(ui::RADIUS))
                        .cursor_pointer()
                        .map(|d| {
                            if on {
                                d.bg(rgb(p.accent))
                            } else {
                                d.hover(move |s| s.bg(rgb(p.fill_hover)))
                            }
                        })
                        .child(
                            div()
                                .text_size(px(ui::BODY))
                                .text_color(rgb(if on { p.on_accent } else { p.label }))
                                .child(*n),
                        )
                        .child(
                            div()
                                .text_size(px(ui::SMALL))
                                .text_color(rgb(if on { p.on_accent } else { p.secondary }))
                                .child(*w),
                        )
                        .on_click(cx.listener(move |v, _, _, cx| v.choose_line(i, cx))),
                )
            },
        );
        let detail: Option<Div> = match self.iface_choice {
            ADAPTER => Some(self.adapter_panel(p, cx)),
            ECU => Some(self.ecu_panel(p, cx)),
            REPLAY => Some(self.replay_panel(p, cx)),
            _ => None,
        };
        let snap = &self.snap;
        let pins = (1..=16).fold(div().flex().flex_row().flex_wrap().gap(px(3.)), |d, pin| {
            let on = snap.line == pin;
            d.child(
                div()
                    .w(px(22.))
                    .h(px(18.))
                    .flex()
                    .items_center()
                    .justify_center()
                    .rounded(px(3.))
                    .text_size(px(ui::SMALL))
                    .font_family(ui::mono())
                    .map(|d| {
                        if on {
                            d.bg(rgb(p.accent)).text_color(rgb(p.on_accent))
                        } else {
                            d.border_1()
                                .border_color(rgb(p.separator))
                                .text_color(rgb(p.secondary))
                        }
                    })
                    .child(pin.to_string()),
            )
        });
        let can = snap.line == 42;
        let baud = |b: u32| {
            if b == 0 {
                "–".to_string()
            } else {
                b.to_string()
            }
        };
        let kv = |k: &'static str, v: String| {
            div()
                .flex()
                .flex_row()
                .justify_between()
                .child(div().text_color(rgb(p.secondary)).child(k))
                .child(div().font_family(ui::mono()).child(v))
        };
        let status = div()
            .flex()
            .flex_col()
            .gap(px(6.))
            .child(ui::header(p, "Line Status"))
            .child(ui::note(p, "OBD pin the Hi-Scan selected"))
            .child(
                pins.child(
                    div()
                        .h(px(18.))
                        .px(px(5.))
                        .flex()
                        .items_center()
                        .rounded(px(3.))
                        .text_size(px(ui::SMALL))
                        .map(|d| {
                            if can {
                                d.bg(rgb(p.accent)).text_color(rgb(p.on_accent))
                            } else {
                                d.border_1()
                                    .border_color(rgb(p.separator))
                                    .text_color(rgb(p.secondary))
                            }
                        })
                        .child("CAN"),
                ),
            )
            .child(kv("Baud rate", baud(snap.baud)))
            .child(kv("Requested", baud(snap.baud_requested)))
            .child(kv("Echo", if snap.echo { "on" } else { "off" }.into()));
        div()
            .flex()
            .flex_col()
            .gap(px(10.))
            .child(ui::header(p, "Diagnostic Line"))
            .child(list)
            .child(div().h(px(1.)).bg(rgb(p.separator)))
            .child(status)
            .when_some(detail, |d, x| {
                d.child(div().h(px(1.)).bg(rgb(p.separator)))
                    .child(ui::header(p, LINE_INFO[self.iface_choice].0))
                    .child(x)
            })
    }

    /// Emulator tab: run, reset, speed.
    fn emulator(&self, p: Palette, cx: &mut Context<Self>) -> Div {
        let s = &self.snap;
        let paused = s.paused;
        let speeds = [
            ("1×", Speed::Real(1.0)),
            ("4×", Speed::Real(4.0)),
            ("Max", Speed::Max),
        ];
        let speed_row = speeds.iter().fold(ui::segmented(p), |d, &(text, speed)| {
            d.child(
                ui::segment(
                    p,
                    SharedString::from(format!("speed-{text}")),
                    text,
                    s.speed == speed,
                )
                .on_click(cx.listener(move |v, _, _, _| v.runner.send(Command::Speed(speed)))),
            )
        });
        div()
            .flex()
            .flex_col()
            .gap(px(10.))
            .child(ui::header(p, "Handset"))
            .child(
                div()
                    .flex()
                    .flex_row()
                    .gap(px(6.))
                    .child(
                        ui::button(
                            p,
                            "run",
                            Some(if paused {
                                IconName::Play
                            } else {
                                IconName::Pause
                            }),
                            if paused { "Run" } else { "Pause" },
                        )
                        .on_click(
                            cx.listener(move |v, _, _, _| v.runner.send(Command::Pause(!paused))),
                        ),
                    )
                    .child(
                        ui::button(p, "reset", Some(IconName::RotateCcw), "Reset")
                            .on_click(cx.listener(|v, _, _, _| v.runner.send(Command::Reset))),
                    ),
            )
            .child(ui::note(
                p,
                "Reset restarts the Hi-Scan's firmware; the EEPROM is kept.",
            ))
            .child(div().h(px(1.)).bg(rgb(p.separator)))
            .child(ui::header(p, "Speed"))
            .child(speed_row)
            .child(ui::note(
                p,
                "Real time is needed with a car or the simulated car; Max runs the \
                 firmware as fast as this computer can.",
            ))
            .when_some(self.exrom_name.clone(), |d, n| {
                d.child(div().h(px(1.)).bg(rgb(p.separator)))
                    .child(ui::header(p, "Troubleshooting Pack"))
                    .child(div().truncate().child(n))
            })
    }

    /// Developer tab: CPU state and the line events.
    fn developer(&self, p: Palette) -> Div {
        let s = &self.snap;
        let kv = |k: &'static str, v: String| {
            div()
                .flex()
                .flex_row()
                .justify_between()
                .text_size(px(ui::SMALL))
                .child(div().text_color(rgb(p.secondary)).child(k))
                .child(
                    div()
                        .font_family(ui::mono())
                        .text_color(rgb(p.label))
                        .child(v),
                )
        };
        let log_lines: Vec<String> = s.log.iter().rev().take(200).cloned().collect();
        div()
            .flex()
            .flex_col()
            .gap(px(10.))
            .flex_1()
            .min_h(px(0.))
            .child(ui::header(p, "Firmware"))
            .child(
                div()
                    .flex()
                    .flex_col()
                    .gap(px(3.))
                    .child(kv("PC", format!("{:04X}", s.pc)))
                    .child(kv("Firmware resets", s.resets.to_string()))
                    .child(kv("Line", s.iface.clone())),
            )
            .child(ui::header(p, "Counters"))
            .child(
                div()
                    .flex()
                    .flex_col()
                    .gap(px(3.))
                    .child(kv("Bytes sent (UART TX)", s.uart_tx.to_string()))
                    .child(kv("Bytes received", s.act.rx_bytes.to_string()))
                    .child(kv("EEPROM accesses", s.act.eeprom_accesses.to_string())),
            )
            .child(ui::header(p, "Line Events"))
            .child(
                div()
                    .id("log")
                    .flex_1()
                    .min_h(px(80.))
                    .overflow_y_scroll()
                    .rounded(px(ui::RADIUS))
                    .bg(rgb(p.canvas))
                    .border_1()
                    .border_color(rgb(p.separator))
                    .p(px(6.))
                    .font_family(ui::mono())
                    .text_size(px(ui::SMALL))
                    .text_color(rgb(p.secondary))
                    .children(log_lines.into_iter().map(|l| div().child(l))),
            )
    }

    /// Always at the foot of the side panel: is it running, how fast, on
    /// which line.
    fn status(&self, p: Palette) -> Div {
        let s = &self.snap;
        let secs = s.total_cycles as f64 / 10_000_000.0;
        let (state, dot) = if s.stopped.is_some() {
            ("Stopped", p.red)
        } else if s.paused {
            ("Paused", p.tertiary)
        } else {
            ("Running", p.green)
        };
        div()
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
                    .child(div().size(px(7.)).rounded_full().bg(rgb(dot)))
                    .child(div().text_color(rgb(p.label)).child(state))
                    .child(div().flex_1())
                    .child(
                        div()
                            .font_family(ui::mono())
                            .text_color(rgb(p.secondary))
                            .child(format!("{secs:.1} s  ×{:.2}", s.ratio)),
                    ),
            )
            .child({
                // lit while seen in the last 150 ms of emulated time
                let recent = |t: u64| t != 0 && s.total_cycles.saturating_sub(t) < 1_500_000;
                let a = &s.act;
                [
                    ("RX", recent(a.last_rx)),
                    ("TX", recent(a.last_tx)),
                    ("Break", recent(a.last_break)),
                    ("EEPROM", recent(a.last_eeprom)),
                    ("CAN", s.line == 42),
                ]
                .into_iter()
                .fold(
                    div().flex().flex_row().gap(px(10.)).py(px(2.)),
                    |d, (name, on)| {
                        d.child(
                            div()
                                .flex()
                                .flex_row()
                                .items_center()
                                .gap(px(4.))
                                .child(div().size(px(6.)).rounded_full().bg(rgb(if on {
                                    p.accent
                                } else {
                                    p.separator
                                })))
                                .child(
                                    div()
                                        .text_color(rgb(if on { p.label } else { p.tertiary }))
                                        .child(name),
                                ),
                        )
                    },
                )
            })
            .child(
                div()
                    .truncate()
                    .text_color(rgb(p.secondary))
                    .child(match &s.stopped {
                        Some(why) => format!("Stopped: {why}"),
                        None => LINE_INFO[self.iface_choice].0.to_string(),
                    }),
            )
    }

    fn side(&self, p: Palette, cx: &mut Context<Self>) -> impl IntoElement {
        let tabs = [
            (SideTab::Connection, "Connection"),
            (SideTab::Emulator, "Emulator"),
            (SideTab::Developer, "Developer"),
        ]
        .into_iter()
        .fold(ui::segmented(p), |d, (t, name)| {
            d.child(
                ui::segment(p, name, name, self.tab == t).on_click(cx.listener(
                    move |v, _, _, cx| {
                        v.tab = t;
                        cx.notify();
                    },
                )),
            )
        });
        let body = match self.tab {
            SideTab::Connection => self.connection(p, cx),
            SideTab::Emulator => self.emulator(p, cx),
            SideTab::Developer => self.developer(p),
        };
        div()
            .flex()
            .flex_col()
            .w(px(PANEL_W))
            .h_full()
            .bg(rgb(p.panel))
            .border_l_1()
            .border_color(rgb(p.separator))
            .text_size(px(ui::BODY))
            .text_color(rgb(p.label))
            .child(div().p(px(10.)).child(tabs))
            .child(
                div()
                    .id("side-body")
                    .flex_1()
                    .min_h(px(0.))
                    .flex()
                    .flex_col()
                    .overflow_y_scroll()
                    .px(px(14.))
                    .pb(px(14.))
                    .child(body),
            )
            .child(self.status(p))
    }

    /// Above the handset: which view, and the side panel toggle.
    fn toolbar(&self, p: Palette, cx: &mut Context<Self>) -> Div {
        div()
            .flex()
            .flex_row()
            .items_center()
            .justify_between()
            .h(px(TOOLBAR_H))
            .px(px(10.))
            .child(
                ui::segmented(p)
                    .w(px(200.))
                    .child(
                        ui::segment(p, "view-dev", "Hi-Scan", !self.live_view).on_click(
                            cx.listener(|v, _, _, cx| {
                                v.live_view = false;
                                cx.notify();
                            }),
                        ),
                    )
                    .child(
                        ui::segment(p, "view-live", "Live Data", self.live_view).on_click(
                            cx.listener(|v, _, _, cx| {
                                v.live_view = true;
                                cx.notify();
                            }),
                        ),
                    ),
            )
            .child(
                ui::button(p, "sidebar", Some(IconName::PanelRight), "")
                    .when(self.sidebar, |d| d.bg(rgb(p.selected)))
                    .on_click(cx.listener(|v, _, _, cx| {
                        v.sidebar = !v.sidebar;
                        cx.notify();
                    })),
            )
    }
}

impl Render for CascadeView {
    fn render(&mut self, window: &mut Window, cx: &mut Context<Self>) -> impl IntoElement {
        let g = Geo::new(self.lcd_scale(window));
        let p = Palette::of(window.appearance());
        div()
            .track_focus(&self.focus)
            .size_full()
            .flex()
            .flex_row()
            .bg(rgb(p.canvas))
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
                    .flex_col()
                    .child(self.toolbar(p, cx))
                    .child(
                        div()
                            .flex_1()
                            .min_h(px(0.))
                            .flex()
                            .items_center()
                            .justify_center()
                            .p(px(12.))
                            .map(|d| {
                                if self.live_view {
                                    d.child(self.live_panel(p, cx))
                                } else {
                                    d.child(self.device(g, cx))
                                }
                            }),
                    ),
            )
            .when(self.sidebar, |d| d.child(self.side(p, cx)))
    }
}

/// What the device window starts with.
struct Start {
    rom: Vec<u8>,
    replay: Option<Result<Loaded, String>>,
    ecu: (Option<Result<EcuRom, String>>, bool),
    exrom: Option<(String, Vec<u8>)>,
    eeprom: Option<std::path::PathBuf>,
}

fn read_exrom(p: &std::path::Path) -> Option<(String, Vec<u8>)> {
    match std::fs::read(p) {
        Ok(x) => {
            let name = p
                .file_name()
                .map_or_else(|| p.display().to_string(), |n| n.to_string_lossy().into());
            Some((name, x))
        }
        Err(e) => {
            eprintln!("cascade-gui: {}: {e}", p.display());
            None
        }
    }
}

/// The Hi-Scan window, as tall as the display allows.
fn open_device(start: Start, cx: &mut App) {
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
    let mut start = Some(start);
    gpui_kit::open_window(opts, cx, move |window, cx| {
        let s = start.take().expect("one window");
        cx.new(|cx| CascadeView::new(s.rom, s.replay, s.ecu, s.exrom, s.eeprom, window, cx))
    })
    .expect("failed to open window");
}

/// The first screen when no pack was given: which Hi-Scan pack to run,
/// and the optional troubleshooting pack, engine ECU dump and EEPROM file.
struct Setup {
    settings: ecusim::settings::Settings,
    pack: Option<std::path::PathBuf>,
    exrom: Option<std::path::PathBuf>,
    ecu: Option<std::path::PathBuf>,
    eeprom: bool,
    error: Option<String>,
}

#[derive(Clone, Copy)]
enum Pick {
    Pack,
    Exrom,
    Ecu,
}

impl Setup {
    fn new() -> Self {
        let settings = ecusim::settings::Settings::load("cascade-gui");
        let first = |k: &str| settings.recent(k).into_iter().next();
        Self {
            pack: first("pack"),
            exrom: first("exrom").filter(|_| settings.flag("use_exrom", false)),
            ecu: first("ecu").filter(|_| settings.flag("use_ecu", false)),
            eeprom: settings.flag("eeprom", true),
            settings,
            error: None,
        }
    }

    fn choose(&mut self, what: Pick, cx: &mut Context<Self>) {
        let rx = cx.prompt_for_paths(PathPromptOptions {
            files: true,
            directories: false,
            multiple: false,
            prompt: Some("Choose".into()),
        });
        cx.spawn(async move |this, cx| {
            if let Ok(Ok(Some(paths))) = rx.await
                && let Some(p) = paths.into_iter().next()
            {
                let _ = this.update(cx, |v, cx| {
                    match what {
                        Pick::Pack => v.pack = Some(p),
                        Pick::Exrom => v.exrom = Some(p),
                        Pick::Ecu => v.ecu = Some(p),
                    }
                    v.error = None;
                    cx.notify();
                });
            }
        })
        .detach();
    }

    fn start(&mut self, window: &mut Window, cx: &mut Context<Self>) {
        let Some(pack) = self.pack.clone() else {
            return;
        };
        let rom = match std::fs::read(&pack) {
            Ok(r) if r.len() >= 0x10000 => r,
            Ok(r) => {
                self.error = Some(format!(
                    "{} bytes is too small for a Hi-Scan Pro pack",
                    r.len()
                ));
                return;
            }
            Err(e) => {
                self.error = Some(format!("{}: {e}", pack.display()));
                return;
            }
        };
        let ecu = self
            .ecu
            .as_deref()
            .map(|p| load_ecu(p).map_err(|e| format!("{}: {e}", p.display())));
        let start = Start {
            rom,
            replay: None,
            ecu: (ecu, false),
            exrom: self.exrom.as_deref().and_then(read_exrom),
            eeprom: self
                .eeprom
                .then(|| std::path::PathBuf::from(format!("{}.eep", pack.display()))),
        };
        let s = &mut self.settings;
        s.push_recent("pack", &pack);
        s.set_flag("eeprom", self.eeprom);
        s.set_flag("use_exrom", self.exrom.is_some());
        s.set_flag("use_ecu", self.ecu.is_some());
        if let Some(p) = &self.exrom {
            s.push_recent("exrom", p);
        }
        if let Some(p) = &self.ecu {
            s.push_recent("ecu", p);
        }
        s.save();
        open_device(start, cx);
        window.remove_window();
    }

    /// One file choice: header, chosen file (or what is missing), Choose…
    /// and, for optional ones, Remove.
    fn file_row(
        &self,
        p: Palette,
        what: Pick,
        label: &'static str,
        none: &'static str,
        cx: &mut Context<Self>,
    ) -> Div {
        let chosen = match what {
            Pick::Pack => &self.pack,
            Pick::Exrom => &self.exrom,
            Pick::Ecu => &self.ecu,
        };
        let (name, dir) = match chosen {
            Some(f) => (
                f.file_name()
                    .map_or_else(|| f.display().to_string(), |n| n.to_string_lossy().into()),
                f.parent()
                    .map_or(String::new(), |d| d.display().to_string()),
            ),
            None => (none.to_string(), String::new()),
        };
        let optional = !matches!(what, Pick::Pack);
        let ids = match what {
            Pick::Pack => ("choose-pack", "clear-pack"),
            Pick::Exrom => ("choose-exrom", "clear-exrom"),
            Pick::Ecu => ("choose-ecu", "clear-ecu"),
        };
        div()
            .flex()
            .flex_col()
            .gap(px(4.))
            .child(ui::header(p, label))
            .child(
                div()
                    .flex()
                    .flex_row()
                    .items_center()
                    .gap(px(8.))
                    .child(
                        div()
                            .flex()
                            .flex_col()
                            .flex_1()
                            .min_w(px(0.))
                            .child(
                                div()
                                    .truncate()
                                    .text_color(rgb(if chosen.is_some() {
                                        p.label
                                    } else {
                                        p.secondary
                                    }))
                                    .child(name),
                            )
                            .when(!dir.is_empty(), |d| d.child(ui::note(p, dir).truncate())),
                    )
                    .when(optional && chosen.is_some(), |d| {
                        d.child(ui::button(p, ids.1, None, "Remove").on_click(cx.listener(
                            move |v, _, _, cx| {
                                match what {
                                    Pick::Pack => {}
                                    Pick::Exrom => v.exrom = None,
                                    Pick::Ecu => v.ecu = None,
                                }
                                cx.notify();
                            },
                        )))
                    })
                    .child(
                        ui::button(p, ids.0, None, "Choose…")
                            .on_click(cx.listener(move |v, _, _, cx| v.choose(what, cx))),
                    ),
            )
    }
}

impl Render for Setup {
    fn render(&mut self, window: &mut Window, cx: &mut Context<Self>) -> impl IntoElement {
        let p = Palette::of(window.appearance());
        let recent: Vec<std::path::PathBuf> = self
            .settings
            .recent("pack")
            .into_iter()
            .filter(|f| Some(f) != self.pack.as_ref())
            .collect();
        let has_recent = !recent.is_empty();
        let recent_list =
            recent
                .into_iter()
                .enumerate()
                .fold(div().flex().flex_col(), |d, (i, f)| {
                    let name = f
                        .file_name()
                        .map_or_else(|| f.display().to_string(), |n| n.to_string_lossy().into());
                    d.child(
                        div()
                            .id(("recent", i))
                            .px(px(6.))
                            .py(px(2.))
                            .rounded(px(ui::RADIUS))
                            .truncate()
                            .text_color(rgb(p.label))
                            .cursor_pointer()
                            .hover(move |s| s.bg(rgb(p.fill_hover)))
                            .child(name)
                            .on_click(cx.listener(move |v, _, _, cx| {
                                v.pack = Some(f.clone());
                                cx.notify();
                            })),
                    )
                });
        let can_start = self.pack.is_some();
        let rule = || div().h(px(1.)).bg(rgb(p.separator));
        div()
            .id("setup")
            .size_full()
            .flex()
            .flex_col()
            .gap(px(14.))
            .p(px(20.))
            .bg(rgb(p.panel))
            .text_size(px(ui::BODY))
            .text_color(rgb(p.label))
            .overflow_y_scroll()
            .child(
                div()
                    .flex()
                    .flex_col()
                    .gap(px(2.))
                    .child(
                        div()
                            .text_size(px(17.))
                            .font_weight(FontWeight::SEMIBOLD)
                            .child("Cascade"),
                    )
                    .child(ui::note(
                        p,
                        "The Hi-Scan Pro, emulated. It runs your own pack; nothing is bundled.",
                    )),
            )
            .child(rule())
            .child(self.file_row(
                p,
                Pick::Pack,
                "Hi-Scan Pack",
                "No pack chosen (for example hyundai.his.dat)",
                cx,
            ))
            .when(has_recent, |d| {
                d.child(
                    div()
                        .flex()
                        .flex_col()
                        .gap(px(2.))
                        .child(ui::note(p, "Recent"))
                        .child(recent_list),
                )
            })
            .child(rule())
            .child(self.file_row(
                p,
                Pick::Exrom,
                "Troubleshooting Pack (optional)",
                "None: the DTC HELP flow screens stay empty",
                cx,
            ))
            .child(self.file_row(
                p,
                Pick::Ecu,
                "Engine ECU Dump (optional, for the ECU line)",
                "None: the ECU line can load one later",
                cx,
            ))
            .child(
                ui::checkbox(
                    p,
                    "eeprom",
                    "Keep the Hi-Scan's settings (EEPROM) in a file next to the pack",
                    self.eeprom,
                )
                .on_click(cx.listener(|v, _, _, cx| {
                    v.eeprom = !v.eeprom;
                    cx.notify();
                })),
            )
            .child(div().flex_1())
            .when_some(self.error.clone(), |d, e| {
                d.child(ui::note(p, e).text_color(rgb(p.red)))
            })
            .child(
                div().flex().flex_row().justify_end().child(
                    ui::default_button(p, "start", None, "Power On")
                        .when(!can_start, |d| d.opacity(0.4))
                        .on_click(cx.listener(move |v, _, w, cx| {
                            if can_start {
                                v.start(w, cx);
                            }
                        })),
                ),
            )
    }
}

fn main() {
    let cli = <cli::Cli as clap::Parser>::parse();
    let path = cli.rom.or_else(|| std::env::var("CASCADE_ROM").ok());
    let options =
        cli.exrom.is_some() || cli.eeprom.is_some() || cli.no_eeprom || cli.replay.is_some();
    if path.is_none() && (options || cli.ecu_rom.is_some()) {
        eprintln!(
            "cascade-gui: no pack given; options without one are ignored and the setup \
             screen opens"
        );
    }
    let start = path.map(|path| {
        let rom = match std::fs::read(&path) {
            Ok(r) => r,
            Err(e) => {
                eprintln!("cascade-gui: {path}: {e}");
                std::process::exit(2);
            }
        };
        let replay = cli.replay.map(|p| {
            let p = std::path::Path::new(&p);
            load_trace(p).map_err(|e| format!("{}: {e}", p.display()))
        });
        // --ecu-rom selects the ECU line at start; CASCADE_ECU_ROM only preloads it
        let ecu_arg = cli.ecu_rom;
        let ecu_first = ecu_arg.is_some();
        // EEPROM file, as upstream: ROM.eep next to the ROM unless --eeprom
        // PATH names another or --no-eeprom keeps it in memory only
        let eeprom = if cli.no_eeprom {
            None
        } else {
            Some(
                cli.eeprom
                    .map(std::path::PathBuf::from)
                    .unwrap_or_else(|| std::path::PathBuf::from(format!("{path}.eep"))),
            )
        };
        // extended ROM (troubleshooting pack): --exrom PATH or CASCADE_EXROM
        let exrom = cli
            .exrom
            .or_else(|| std::env::var("CASCADE_EXROM").ok())
            .and_then(|p| read_exrom(std::path::Path::new(&p)));
        let ecu = ecu_arg
            .or_else(|| std::env::var("CASCADE_ECU_ROM").ok())
            .map(|p| {
                let p = std::path::Path::new(&p);
                load_ecu(p).map_err(|e| format!("{}: {e}", p.display()))
            });
        Start {
            rom,
            replay,
            ecu: (ecu, ecu_first),
            exrom,
            eeprom,
        }
    });
    gpui_kit::application()
        .with_assets(gpui_kit::assets::AllAssets)
        .run(move |cx| {
            gpui_kit::init(cx);
            // gpui-component's widgets follow the system light/dark setting
            gpui_kit::component::Theme::sync_system_appearance(None, cx);
            match start {
                Some(s) => open_device(s, cx),
                None => {
                    let bounds = Bounds::centered(None, size(px(640.), px(560.)), cx);
                    let opts = WindowOptions {
                        window_bounds: Some(WindowBounds::Windowed(bounds)),
                        titlebar: Some(TitlebarOptions {
                            title: Some("Cascade".into()),
                            ..Default::default()
                        }),
                        ..Default::default()
                    };
                    gpui_kit::open_window(opts, cx, |_, cx| cx.new(|_| Setup::new()))
                        .expect("failed to open window");
                }
            }
            // the setup window closes itself when the device opens
            cx.on_window_closed(|cx, _| {
                if cx.windows().is_empty() {
                    cx.quit()
                }
            })
            .detach();
            cx.activate(true);
        });
}
