//! Visual theme + bundled fonts.
//!
//! IBM Plex Sans (Regular + Medium) and IBM Plex Mono are embedded as static
//! bytes so the app looks identical regardless of what the host system has
//! installed. egui's default emoji fonts are kept as fallbacks so glyphs like
//! ☀ / ☾ in the theme toggle still render.

use std::path::PathBuf;

use egui::{
    Color32, FontData, FontDefinitions, FontFamily, FontId, Rounding, Stroke, Style, TextStyle,
    Visuals,
};

const SANS_REGULAR: &[u8] = include_bytes!("../assets/fonts/IBMPlexSans-Regular.ttf");
const SANS_MEDIUM: &[u8] = include_bytes!("../assets/fonts/IBMPlexSans-Medium.ttf");
const MONO_REGULAR: &[u8] = include_bytes!("../assets/fonts/IBMPlexMono-Regular.ttf");

/// Shared accent across both variants. Picked to harmonise with the warm
/// brown ink in the FastViz logo sketch.
const ACCENT: Color32 = Color32::from_rgb(0xC7, 0x7D, 0x2A);

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Mode {
    Dark,
    Light,
}

impl Mode {
    pub fn toggled(self) -> Self {
        match self {
            Mode::Dark => Mode::Light,
            Mode::Light => Mode::Dark,
        }
    }

    fn as_str(self) -> &'static str {
        match self {
            Mode::Dark => "dark",
            Mode::Light => "light",
        }
    }

    fn from_str(s: &str) -> Option<Self> {
        match s.trim() {
            "dark" => Some(Mode::Dark),
            "light" => Some(Mode::Light),
            _ => None,
        }
    }
}

/// Register IBM Plex with egui. Call once after the context is created.
pub fn install_fonts(ctx: &egui::Context) {
    let mut fonts = FontDefinitions::default();

    fonts
        .font_data
        .insert("plex_sans".to_owned(), FontData::from_static(SANS_REGULAR));
    fonts.font_data.insert(
        "plex_sans_medium".to_owned(),
        FontData::from_static(SANS_MEDIUM),
    );
    fonts
        .font_data
        .insert("plex_mono".to_owned(), FontData::from_static(MONO_REGULAR));

    // Primary Proportional / Monospace families: prepend Plex, keep egui's
    // built-in fallbacks (NotoEmoji etc.) for missing glyphs.
    fonts
        .families
        .entry(FontFamily::Proportional)
        .or_default()
        .insert(0, "plex_sans".to_owned());
    fonts
        .families
        .entry(FontFamily::Monospace)
        .or_default()
        .insert(0, "plex_mono".to_owned());

    // Dedicated family for headings/branding so we can use Plex Sans Medium
    // without affecting body text. Falls back to regular Sans, then default.
    fonts.families.insert(
        FontFamily::Name("plex_sans_medium".into()),
        vec![
            "plex_sans_medium".to_owned(),
            "plex_sans".to_owned(),
            "Ubuntu-Light".to_owned(),
        ],
    );

    ctx.set_fonts(fonts);
}

/// Push a refined Style (visuals + spacing + text sizes) for the chosen mode.
pub fn apply(ctx: &egui::Context, mode: Mode) {
    let mut style: Style = (*ctx.style()).clone();
    style.visuals = visuals(mode);
    tune_spacing(&mut style);
    tune_text_styles(&mut style);
    ctx.set_style(style);
}

fn visuals(mode: Mode) -> Visuals {
    match mode {
        Mode::Dark => dark_visuals(),
        Mode::Light => light_visuals(),
    }
}

fn dark_visuals() -> Visuals {
    let mut v = Visuals::dark();
    // Deep neutral background with a touch of warmth.
    let panel = Color32::from_rgb(0x14, 0x16, 0x1A);
    let window = Color32::from_rgb(0x1B, 0x1E, 0x23);
    let raised = Color32::from_rgb(0x22, 0x26, 0x2C);
    let hover = Color32::from_rgb(0x2A, 0x2E, 0x36);
    let active = Color32::from_rgb(0x33, 0x38, 0x42);
    let separator = Color32::from_rgb(0x2A, 0x2E, 0x35);
    let text = Color32::from_rgb(0xE6, 0xE1, 0xD7);
    let muted = Color32::from_rgb(0x9A, 0x96, 0x8C);

    v.dark_mode = true;
    v.override_text_color = Some(text);
    v.panel_fill = panel;
    v.window_fill = window;
    v.window_stroke = Stroke::new(1.0, separator);
    v.faint_bg_color = raised;
    v.extreme_bg_color = Color32::from_rgb(0x0E, 0x10, 0x13);
    v.code_bg_color = Color32::from_rgb(0x10, 0x12, 0x16);
    v.hyperlink_color = ACCENT;
    v.warn_fg_color = Color32::from_rgb(0xE0, 0xB3, 0x4D);
    v.error_fg_color = Color32::from_rgb(0xE0, 0x6C, 0x5A);

    v.window_rounding = Rounding::same(8.0);
    v.menu_rounding = Rounding::same(6.0);

    v.selection.bg_fill = ACCENT.linear_multiply(0.55);
    v.selection.stroke = Stroke::new(1.0, text);

    v.widgets.noninteractive.bg_fill = panel;
    v.widgets.noninteractive.weak_bg_fill = panel;
    v.widgets.noninteractive.bg_stroke = Stroke::new(1.0, separator);
    v.widgets.noninteractive.fg_stroke = Stroke::new(1.0, muted);
    v.widgets.noninteractive.rounding = Rounding::same(6.0);

    v.widgets.inactive.bg_fill = raised;
    v.widgets.inactive.weak_bg_fill = raised;
    v.widgets.inactive.bg_stroke = Stroke::NONE;
    v.widgets.inactive.fg_stroke = Stroke::new(1.0, text);
    v.widgets.inactive.rounding = Rounding::same(6.0);

    v.widgets.hovered.bg_fill = hover;
    v.widgets.hovered.weak_bg_fill = hover;
    v.widgets.hovered.bg_stroke = Stroke::new(1.0, ACCENT.linear_multiply(0.45));
    v.widgets.hovered.fg_stroke = Stroke::new(1.0, text);
    v.widgets.hovered.rounding = Rounding::same(6.0);

    v.widgets.active.bg_fill = active;
    v.widgets.active.weak_bg_fill = active;
    v.widgets.active.bg_stroke = Stroke::new(1.0, ACCENT);
    v.widgets.active.fg_stroke = Stroke::new(1.0, text);
    v.widgets.active.rounding = Rounding::same(6.0);

    v.widgets.open.bg_fill = raised;
    v.widgets.open.weak_bg_fill = raised;
    v.widgets.open.bg_stroke = Stroke::new(1.0, separator);
    v.widgets.open.fg_stroke = Stroke::new(1.0, text);
    v.widgets.open.rounding = Rounding::same(6.0);

    v
}

fn light_visuals() -> Visuals {
    let mut v = Visuals::light();
    // Warm off-white "paper" surface.
    let panel = Color32::from_rgb(0xFA, 0xF7, 0xF1);
    let window = Color32::from_rgb(0xFF, 0xFE, 0xFA);
    let raised = Color32::from_rgb(0xF1, 0xEC, 0xE0);
    let hover = Color32::from_rgb(0xE8, 0xE1, 0xD0);
    let active = Color32::from_rgb(0xDC, 0xD3, 0xBC);
    let separator = Color32::from_rgb(0xE2, 0xDB, 0xCB);
    let text = Color32::from_rgb(0x20, 0x22, 0x2A);
    let muted = Color32::from_rgb(0x6B, 0x67, 0x5C);

    v.dark_mode = false;
    v.override_text_color = Some(text);
    v.panel_fill = panel;
    v.window_fill = window;
    v.window_stroke = Stroke::new(1.0, separator);
    v.faint_bg_color = raised;
    v.extreme_bg_color = Color32::from_rgb(0xFF, 0xFF, 0xFF);
    v.code_bg_color = Color32::from_rgb(0xF4, 0xEF, 0xE2);
    v.hyperlink_color = ACCENT;
    v.warn_fg_color = Color32::from_rgb(0xB8, 0x7A, 0x1C);
    v.error_fg_color = Color32::from_rgb(0xC0, 0x40, 0x30);

    v.window_rounding = Rounding::same(8.0);
    v.menu_rounding = Rounding::same(6.0);

    v.selection.bg_fill = ACCENT.linear_multiply(0.35);
    v.selection.stroke = Stroke::new(1.0, text);

    v.widgets.noninteractive.bg_fill = panel;
    v.widgets.noninteractive.weak_bg_fill = panel;
    v.widgets.noninteractive.bg_stroke = Stroke::new(1.0, separator);
    v.widgets.noninteractive.fg_stroke = Stroke::new(1.0, muted);
    v.widgets.noninteractive.rounding = Rounding::same(6.0);

    v.widgets.inactive.bg_fill = raised;
    v.widgets.inactive.weak_bg_fill = raised;
    v.widgets.inactive.bg_stroke = Stroke::NONE;
    v.widgets.inactive.fg_stroke = Stroke::new(1.0, text);
    v.widgets.inactive.rounding = Rounding::same(6.0);

    v.widgets.hovered.bg_fill = hover;
    v.widgets.hovered.weak_bg_fill = hover;
    v.widgets.hovered.bg_stroke = Stroke::new(1.0, ACCENT.linear_multiply(0.6));
    v.widgets.hovered.fg_stroke = Stroke::new(1.0, text);
    v.widgets.hovered.rounding = Rounding::same(6.0);

    v.widgets.active.bg_fill = active;
    v.widgets.active.weak_bg_fill = active;
    v.widgets.active.bg_stroke = Stroke::new(1.0, ACCENT);
    v.widgets.active.fg_stroke = Stroke::new(1.0, text);
    v.widgets.active.rounding = Rounding::same(6.0);

    v.widgets.open.bg_fill = raised;
    v.widgets.open.weak_bg_fill = raised;
    v.widgets.open.bg_stroke = Stroke::new(1.0, separator);
    v.widgets.open.fg_stroke = Stroke::new(1.0, text);
    v.widgets.open.rounding = Rounding::same(6.0);

    v
}

fn tune_spacing(style: &mut Style) {
    let s = &mut style.spacing;
    s.item_spacing = egui::vec2(8.0, 6.0);
    s.button_padding = egui::vec2(10.0, 4.0);
    s.menu_margin = egui::Margin::same(6.0);
    s.window_margin = egui::Margin::same(10.0);
    s.indent = 16.0;
    s.interact_size.y = 22.0;
    s.icon_width = 16.0;
    s.icon_width_inner = 10.0;
    s.icon_spacing = 6.0;
    s.scroll.bar_width = 8.0;
    s.scroll.floating = true;
}

fn tune_text_styles(style: &mut Style) {
    use FontFamily::{Monospace, Proportional};
    let medium = FontFamily::Name("plex_sans_medium".into());
    style.text_styles.insert(TextStyle::Small, FontId::new(11.0, Proportional.clone()));
    style.text_styles.insert(TextStyle::Body, FontId::new(13.0, Proportional.clone()));
    style.text_styles.insert(TextStyle::Button, FontId::new(13.0, Proportional.clone()));
    style.text_styles.insert(TextStyle::Heading, FontId::new(17.0, medium));
    style.text_styles.insert(TextStyle::Monospace, FontId::new(12.5, Monospace));
}

/// Accent color exposed for the rare site (e.g. brand mark) that wants to
/// match the theme directly rather than going through Style.
pub fn accent() -> Color32 {
    ACCENT
}

// --- persistence ------------------------------------------------------------

fn config_path() -> Option<PathBuf> {
    let base = std::env::var_os("XDG_CONFIG_HOME")
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config")))?;
    Some(base.join("fastviz").join("theme"))
}

/// Read the saved preference. Defaults to dark when nothing is saved.
pub fn load() -> Mode {
    config_path()
        .and_then(|p| std::fs::read_to_string(p).ok())
        .and_then(|s| Mode::from_str(&s))
        .unwrap_or(Mode::Dark)
}

/// Persist the user's choice. Best-effort: errors are logged and dropped so a
/// read-only home directory doesn't break the toggle.
pub fn save(mode: Mode) {
    let Some(path) = config_path() else {
        return;
    };
    if let Some(parent) = path.parent() {
        if let Err(e) = std::fs::create_dir_all(parent) {
            log::warn!("theme: create_dir_all({}) failed: {e}", parent.display());
            return;
        }
    }
    if let Err(e) = std::fs::write(&path, mode.as_str()) {
        log::warn!("theme: write({}) failed: {e}", path.display());
    }
}
