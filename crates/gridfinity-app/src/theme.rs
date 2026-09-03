//! The one place the app's colours, type scale and control metrics are decided.
//!
//! This is the egui counterpart of `web/src/theme.ts`, and it holds the same
//! contract: nothing else in the app names a colour or a font size, so
//! restyling the whole window -- a different accent, rounder corners, denser
//! text -- is an edit to this file and nothing else. The palette is Mantine's
//! own dark scale, read out of `@mantine/core`'s stylesheet, so a bin, a
//! divider, a warning banner or a panel edge is literally the same colour in
//! both front ends.
//!
//! `apply` installs the style on a context and `install_fonts` gives it the
//! system UI face the web app asks for; both run once, at startup.

use eframe::egui::{
    self, Color32, CornerRadius, FontData, FontDefinitions, FontFamily, FontId, Margin, Stroke,
    TextStyle,
};

/// One hex literal as Mantine writes it, as the colour egui paints with.
const fn hex(rgb: u32) -> Color32 {
    Color32::from_rgb((rgb >> 16) as u8, (rgb >> 8) as u8, rgb as u8)
}

// Mantine's dark scale: 0 is the lightest (body text) and 9 the darkest.
pub const DARK_0: Color32 = hex(0xc9c9c9);
pub const DARK_1: Color32 = hex(0xb8b8b8);
pub const DARK_2: Color32 = hex(0x828282);
pub const DARK_3: Color32 = hex(0x696969);
pub const DARK_4: Color32 = hex(0x424242);
pub const DARK_5: Color32 = hex(0x3b3b3b);
pub const DARK_6: Color32 = hex(0x2e2e2e);
pub const DARK_7: Color32 = hex(0x242424);
pub const DARK_8: Color32 = hex(0x1f1f1f);

pub const BLUE: Color32 = hex(0x228be6);
pub const BLUE_LIGHT: Color32 = hex(0x339af0);
pub const GREEN: Color32 = hex(0x40c057);
pub const YELLOW: Color32 = hex(0xfcc419);
pub const RED: Color32 = hex(0xfa5252);
pub const RED_PALE: Color32 = hex(0xff8787);
pub const TEAL: Color32 = hex(0x20c997);
pub const TEAL_LIGHT: Color32 = hex(0x38d9a9);
pub const TEAL_PALE: Color32 = hex(0x63e6be);
pub const GRAY_3: Color32 = hex(0xdee2e6);
pub const GRAY_5: Color32 = hex(0xadb5bd);

/// `--mantine-color-body`: the ground every panel is painted on.
pub const BODY: Color32 = DARK_7;
/// The fill of a `<Paper bg="dark.6">` row -- a list entry inside a panel.
pub const SURFACE: Color32 = DARK_6;
/// Panel edges and control borders: Mantine's default dark border.
pub const BORDER: Color32 = DARK_4;
/// Ordinary copy.
pub const TEXT: Color32 = DARK_0;
/// `<Text c="dimmed">`: hints, captions, units.
pub const TEXT_DIMMED: Color32 = DARK_2;
/// `<Text c="bright">`: a value the eye should land on.
pub const TEXT_BRIGHT: Color32 = Color32::WHITE;

/// Mantine's `md` radius, which `theme.ts` sets as the app-wide default.
const RADIUS: u8 = 6;

/// The header strip's height, matching `AppShell`'s `header={{ height: 48 }}`.
pub const HEADER_HEIGHT: f32 = 48.0;

/// The two side panels' starting widths, as `store.ts` seeds `panelWidths`.
pub const SIDEBAR_WIDTH: f32 = 340.0;
pub const SETTINGS_WIDTH: f32 = 320.0;

/// Mantine's `xs`/`sm` font sizes, which is the whole scale this app uses: the
/// web theme defaults every control one step below Mantine's own, and `Text`
/// to `xs`, because both front ends are dense tool UIs.
pub const FONT_XS: f32 = 12.0;
pub const FONT_SM: f32 = 14.0;

/// The context styled as the web app is: Mantine's dark palette, its type
/// scale, and its control metrics.
///
/// Called once at startup. Everything it sets is a default the widgets inherit,
/// so a panel that paints nothing of its own already looks right.
pub fn apply(ctx: &egui::Context) {
    install_fonts(ctx);

    ctx.set_theme(egui::ThemePreference::Dark);
    ctx.all_styles_mut(style);
}

/// One `Style` restated as the web app's: Mantine's dark palette, its type
/// scale and its control metrics.
///
/// Applied to every theme egui keeps, because the app has one look -- the web
/// app pins `forceColorScheme="dark"` for the same reason.
fn style(style: &mut egui::Style) {
    let v = &mut style.visuals;

    v.dark_mode = true;
    v.panel_fill = BODY;
    v.window_fill = SURFACE;
    v.window_stroke = Stroke::new(1.0, BORDER);
    v.window_corner_radius = CornerRadius::same(RADIUS);
    v.menu_corner_radius = CornerRadius::same(RADIUS);
    v.extreme_bg_color = DARK_8;
    v.faint_bg_color = SURFACE;
    v.code_bg_color = DARK_8;
    v.warn_fg_color = YELLOW;
    v.error_fg_color = RED;
    v.hyperlink_color = BLUE_LIGHT;
    v.selection.bg_fill = BLUE;
    v.selection.stroke = Stroke::new(1.0, TEXT_BRIGHT);
    v.weak_text_color = Some(TEXT_DIMMED);
    v.slider_trailing_fill = true;
    v.button_frame = true;
    v.collapsing_header_frame = false;
    v.indent_has_left_vline = false;
    v.interact_cursor = Some(egui::CursorIcon::PointingHand);

    let radius = CornerRadius::same(RADIUS);
    // A Mantine `variant="default"` control: dark.6 ground, dark.4 border, and
    // a step lighter under the pointer.
    let w = &mut v.widgets;
    w.noninteractive.bg_fill = BODY;
    w.noninteractive.weak_bg_fill = BODY;
    w.noninteractive.bg_stroke = Stroke::new(1.0, BORDER);
    w.noninteractive.fg_stroke = Stroke::new(1.0, TEXT);
    w.noninteractive.corner_radius = radius;
    w.inactive.bg_fill = DARK_5;
    w.inactive.weak_bg_fill = SURFACE;
    w.inactive.bg_stroke = Stroke::new(1.0, BORDER);
    w.inactive.fg_stroke = Stroke::new(1.0, TEXT);
    w.inactive.corner_radius = radius;
    w.hovered.bg_fill = DARK_4;
    w.hovered.weak_bg_fill = DARK_5;
    w.hovered.bg_stroke = Stroke::new(1.0, DARK_3);
    w.hovered.fg_stroke = Stroke::new(1.0, TEXT_BRIGHT);
    w.hovered.corner_radius = radius;
    w.active.bg_fill = BLUE;
    w.active.weak_bg_fill = DARK_4;
    w.active.bg_stroke = Stroke::new(1.0, BLUE);
    w.active.fg_stroke = Stroke::new(1.0, TEXT_BRIGHT);
    w.active.corner_radius = radius;
    w.open.bg_fill = DARK_5;
    w.open.weak_bg_fill = SURFACE;
    w.open.bg_stroke = Stroke::new(1.0, BORDER);
    w.open.fg_stroke = Stroke::new(1.0, TEXT);
    w.open.corner_radius = radius;

    style.text_styles = [
        (TextStyle::Small, FontId::proportional(11.0)),
        (TextStyle::Body, FontId::proportional(FONT_XS)),
        (TextStyle::Button, FontId::proportional(FONT_XS)),
        (TextStyle::Heading, FontId::proportional(FONT_SM)),
        (TextStyle::Monospace, FontId::monospace(11.0)),
    ]
    .into();

    let s = &mut style.spacing;
    s.item_spacing = egui::vec2(8.0, 6.0);
    s.button_padding = egui::vec2(8.0, 3.0);
    s.window_margin = Margin::same(8);
    s.menu_margin = Margin::same(4);
    s.indent = 12.0;
    s.interact_size = egui::vec2(36.0, 20.0);
    s.slider_width = 100.0;
    s.combo_width = 140.0;
}

/// The system UI face registered as the proportional family, where the platform
/// has one this app knows how to find.
///
/// The web app's `fontFamily` is `system-ui, "Segoe UI", ...`, which on the
/// platform this desktop app is developed on resolves to Segoe UI. Loading it
/// is what makes the two windows read as the same product rather than as a
/// native app beside a web page. A platform without that file keeps egui's own
/// bundled face: the typeface is a nicety, and failing to start over it would
/// not be.
fn install_fonts(ctx: &egui::Context) {
    #[cfg(target_arch = "wasm32")]
    let _ = ctx;
    #[cfg(not(target_arch = "wasm32"))]
    {
    const SEGOE_UI: &str = r"C:\Windows\Fonts\segoeui.ttf";
    let Ok(bytes) = std::fs::read(SEGOE_UI) else {
        return;
    };
    let mut fonts = FontDefinitions::default();
    fonts
        .font_data
        .insert("system-ui".to_owned(), FontData::from_owned(bytes).into());
    fonts
        .families
        .entry(FontFamily::Proportional)
        .or_default()
        .insert(0, "system-ui".to_owned());
    ctx.set_fonts(fonts);
    }
}
