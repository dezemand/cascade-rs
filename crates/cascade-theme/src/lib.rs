//! The cascade-rs desktop apps (Cascade, EcuSim) look like ordinary apps of
//! the platform they run on: system font and sizes, the system's light or
//! dark appearance, neutral greys, and the platform accent only on
//! selection and the default button.
//!
//! GPUI draws every control itself, so this follows platform conventions;
//! it is not AppKit, WinUI or GTK.
//!
//! Colours per platform and appearance live in [`Palette::of`]:
//! - macOS: read from AppKit's semantic colours on macOS 26 [verified,
//!   composited over the window background where they are translucent];
//! - Windows: Fluent (WinUI 3) defaults [assumed, from Microsoft's docs];
//! - Linux: GNOME Adwaita defaults [assumed, from the libadwaita docs].

use gpui_kit::assets::IconName;
use gpui_kit::prelude::FluentBuilder as _;
use gpui_kit::*;

/// Colours for one platform and appearance (`0xRRGGBB`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Palette {
    pub dark: bool,
    /// Behind content that is not a panel (where the handset sits).
    pub canvas: u32,
    /// Side panels and windows.
    pub panel: u32,
    pub separator: u32,
    /// Text: primary, secondary, tertiary (hints, disabled).
    pub label: u32,
    pub secondary: u32,
    pub tertiary: u32,
    /// Plain control fill, its hover, and the selected segment.
    pub fill: u32,
    pub fill_hover: u32,
    pub selected: u32,
    /// The platform accent and text on it.
    pub accent: u32,
    pub on_accent: u32,
    pub red: u32,
    pub green: u32,
}

impl Palette {
    /// The palette for a window's appearance on this platform.
    pub fn of(appearance: WindowAppearance) -> Self {
        let dark = matches!(
            appearance,
            WindowAppearance::Dark | WindowAppearance::VibrantDark
        );
        if cfg!(target_os = "windows") {
            if dark {
                Self {
                    dark,
                    canvas: 0x202020,
                    panel: 0x2b2b2b,
                    separator: 0x3b3b3b,
                    label: 0xffffff,
                    secondary: 0xc5c5c5,
                    tertiary: 0x7a7a7a,
                    fill: 0x373737,
                    fill_hover: 0x3d3d3d,
                    selected: 0x454545,
                    accent: 0x60cdff,
                    on_accent: 0x000000,
                    red: 0xff99a4,
                    green: 0x6ccb5f,
                }
            } else {
                Self {
                    dark,
                    canvas: 0xf3f3f3,
                    panel: 0xfbfbfb,
                    separator: 0xe5e5e5,
                    label: 0x1a1a1a,
                    secondary: 0x5d5d5d,
                    tertiary: 0x9d9d9d,
                    fill: 0xfdfdfd,
                    fill_hover: 0xf5f5f5,
                    selected: 0xffffff,
                    accent: 0x005fb8,
                    on_accent: 0xffffff,
                    red: 0xc42b1c,
                    green: 0x0f7b0f,
                }
            }
        } else if cfg!(target_os = "linux") {
            if dark {
                Self {
                    dark,
                    canvas: 0x1e1e1e,
                    panel: 0x242424,
                    separator: 0x383838,
                    label: 0xffffff,
                    secondary: 0xb5b5b5,
                    tertiary: 0x7c7c7c,
                    fill: 0x383838,
                    fill_hover: 0x424242,
                    selected: 0x4a4a4a,
                    accent: 0x78aeed,
                    on_accent: 0xffffff,
                    red: 0xff7b63,
                    green: 0x8ff0a4,
                }
            } else {
                Self {
                    dark,
                    canvas: 0xfafafa,
                    panel: 0xffffff,
                    separator: 0xdedede,
                    label: 0x2e2e2e,
                    secondary: 0x6e6e6e,
                    tertiary: 0xa8a8a8,
                    fill: 0xebebeb,
                    fill_hover: 0xe0e0e0,
                    selected: 0xffffff,
                    accent: 0x3584e4,
                    on_accent: 0xffffff,
                    red: 0xe01b24,
                    green: 0x2ec27e,
                }
            }
        } else if dark {
            Self {
                dark,
                canvas: 0x282828,
                panel: 0x1e1e1e,
                separator: 0x343434,
                label: 0xdddddd,
                secondary: 0x9a9a9a,
                tertiary: 0x565656,
                fill: 0x2c2c2c,
                fill_hover: 0x363636,
                selected: 0x464646,
                accent: 0x007aff,
                on_accent: 0xffffff,
                red: 0xff4245,
                green: 0x30d158,
            }
        } else {
            Self {
                dark,
                canvas: 0xf4f5f5,
                panel: 0xffffff,
                separator: 0xe6e6e6,
                label: 0x272727,
                secondary: 0x808080,
                tertiary: 0xbdbdbd,
                fill: 0xeeeeee,
                fill_hover: 0xe4e4e4,
                selected: 0xffffff,
                accent: 0x007aff,
                on_accent: 0xffffff,
                red: 0xff383c,
                green: 0x34c759,
            }
        }
    }
}

/// Body text size, and the small size for secondary text and headers
/// (macOS: 13 and 11 pt).
pub const BODY: f32 = 13.0;
pub const SMALL: f32 = 11.0;
/// Control corner radius and height.
pub const RADIUS: f32 = 5.0;
pub const CONTROL_H: f32 = 22.0;
/// Numbers and codes.
pub fn mono() -> &'static str {
    if cfg!(target_os = "windows") {
        "Consolas"
    } else if cfg!(target_os = "linux") {
        "monospace"
    } else {
        "Menlo"
    }
}

/// A section header in a side panel ("Connection").
pub fn header(p: Palette, text: impl Into<SharedString>) -> Div {
    div()
        .text_size(px(SMALL))
        .font_weight(FontWeight::SEMIBOLD)
        .text_color(rgb(p.secondary))
        .child(text.into())
}

/// Secondary text.
pub fn note(p: Palette, text: impl Into<SharedString>) -> Div {
    div()
        .text_size(px(SMALL))
        .text_color(rgb(p.secondary))
        .child(text.into())
}

/// The shape and content shared by both button variants, without a hover
/// style (GPUI allows one per element).
fn base_button(
    p: Palette,
    id: impl Into<ElementId>,
    icon: Option<IconName>,
    text: impl Into<SharedString>,
) -> Stateful<Div> {
    let text = text.into();
    let icon_only = text.is_empty();
    div()
        .id(id)
        .flex()
        .flex_row()
        .items_center()
        .justify_center()
        .gap(px(5.))
        .h(px(CONTROL_H))
        .when(icon_only, |d| d.w(px(CONTROL_H + 6.)))
        .when(!icon_only, |d| d.px(px(10.)))
        .rounded(px(RADIUS))
        .bg(rgb(p.fill))
        .border_1()
        .border_color(rgb(p.separator))
        .text_size(px(BODY))
        .text_color(rgb(p.label))
        .cursor_pointer()
        .when_some(icon, |d, i| {
            d.child(gpui_kit::component::Icon::new(i).size(px(14.)))
        })
        .when(!icon_only, |d| d.child(text))
}

/// A plain push button; `icon` alone makes a square icon button.
pub fn button(
    p: Palette,
    id: impl Into<ElementId>,
    icon: Option<IconName>,
    text: impl Into<SharedString>,
) -> Stateful<Div> {
    base_button(p, id, icon, text).hover(move |s| s.bg(rgb(p.fill_hover)))
}

/// The default button of a view: the platform accent.
pub fn default_button(
    p: Palette,
    id: impl Into<ElementId>,
    icon: Option<IconName>,
    text: impl Into<SharedString>,
) -> Stateful<Div> {
    base_button(p, id, icon, text)
        .bg(rgb(p.accent))
        .border_color(rgb(p.accent))
        .text_color(rgb(p.on_accent))
        .hover(|s| s.opacity(0.9))
}

/// The track of a segmented control.
pub fn segmented(p: Palette) -> Div {
    div()
        .flex()
        .flex_row()
        .p(px(2.))
        .gap(px(2.))
        .rounded(px(RADIUS + 1.))
        .bg(rgb(if p.dark { p.fill } else { 0xe6e6e6 }))
}

/// One segment; the selected one is raised.
pub fn segment(
    p: Palette,
    id: impl Into<ElementId>,
    text: impl Into<SharedString>,
    selected: bool,
) -> Stateful<Div> {
    div()
        .id(id)
        .flex_1()
        .flex()
        .items_center()
        .justify_center()
        .h(px(CONTROL_H - 2.))
        .px(px(8.))
        .rounded(px(RADIUS - 1.))
        .text_size(px(BODY))
        .text_color(rgb(p.label))
        .cursor_pointer()
        .when(selected, |d| d.bg(rgb(p.selected)).shadow_sm())
        .child(text.into())
}

/// A disclosure row ("▸ Developer").
pub fn disclosure(
    p: Palette,
    id: impl Into<ElementId>,
    text: impl Into<SharedString>,
    open: bool,
) -> Stateful<Div> {
    div()
        .id(id)
        .flex()
        .flex_row()
        .items_center()
        .gap(px(4.))
        .cursor_pointer()
        .text_size(px(SMALL))
        .font_weight(FontWeight::SEMIBOLD)
        .text_color(rgb(p.secondary))
        .child(
            gpui_kit::component::Icon::new(if open {
                IconName::ChevronDown
            } else {
                IconName::ChevronRight
            })
            .size(px(12.)),
        )
        .child(text.into())
}

/// A checkbox with its label.
pub fn checkbox(
    p: Palette,
    id: impl Into<ElementId>,
    text: impl Into<SharedString>,
    checked: bool,
) -> Stateful<Div> {
    div()
        .id(id)
        .flex()
        .flex_row()
        .items_center()
        .gap(px(6.))
        .cursor_pointer()
        .text_size(px(BODY))
        .text_color(rgb(p.label))
        .child(
            div()
                .flex()
                .items_center()
                .justify_center()
                .size(px(14.))
                .rounded(px(3.))
                .map(|d| {
                    if checked {
                        d.bg(rgb(p.accent))
                            .text_color(rgb(p.on_accent))
                            .child(gpui_kit::component::Icon::new(IconName::Check).size(px(11.)))
                    } else {
                        d.bg(rgb(p.fill)).border_1().border_color(rgb(p.tertiary))
                    }
                }),
        )
        .child(text.into())
}

/// A radio button with a label and an optional line of detail under it.
pub fn radio(
    p: Palette,
    id: impl Into<ElementId>,
    text: impl Into<SharedString>,
    detail: Option<SharedString>,
    selected: bool,
) -> Stateful<Div> {
    div()
        .id(id)
        .flex()
        .flex_row()
        .items_start()
        .gap(px(7.))
        .py(px(2.))
        .cursor_pointer()
        .child(
            div()
                .mt(px(2.))
                .flex()
                .flex_none()
                .items_center()
                .justify_center()
                .size(px(14.))
                .rounded_full()
                .map(|d| {
                    if selected {
                        d.bg(rgb(p.accent))
                            .child(div().size(px(5.)).rounded_full().bg(rgb(p.on_accent)))
                    } else {
                        d.bg(rgb(p.fill)).border_1().border_color(rgb(p.tertiary))
                    }
                }),
        )
        .child(
            div()
                .flex()
                .flex_col()
                .min_w(px(0.))
                .child(
                    div()
                        .text_size(px(BODY))
                        .text_color(rgb(p.label))
                        .child(text.into()),
                )
                .when_some(detail, |d, t| d.child(note(p, t))),
        )
}
