//! The shared control vocabulary of the panels: the egui counterpart of
//! `web/src/components/ui/`.
//!
//! Each function here is one of the web app's small composite controls --
//! `Field`'s uppercase `Label` and muted `Hint`, `SliderField`'s slider with a
//! right-hand readout, `StatusBanner`'s green/amber alert, the settings
//! accordion's section, and the bin buttons' colour swatch -- so a panel is
//! written by naming controls rather than by placing pixels, and the two apps
//! cannot drift apart one widget at a time. `dashed_line` is the one addition
//! the web side gets from CSS for free.
//!
//! Everything paints in `theme`'s colours and takes no colour argument that is
//! not data (a bin's own colour).

use eframe::egui::{self, Color32, CornerRadius, Frame, Margin, Pos2, Response, RichText, Stroke};

use crate::theme;

/// A section or field name, as `Field.tsx`'s `Label`: uppercase, bold, and
/// letterspaced so it reads as a heading at caption size.
pub fn label_text(text: &str) -> RichText {
    RichText::new(text.to_uppercase())
        .strong()
        .size(11.0)
        .extra_letter_spacing(0.6)
        .color(theme::TEXT)
}

/// That label, written into `ui`.
pub fn label(ui: &mut egui::Ui, text: &str) {
    ui.label(label_text(text));
}

/// Muted helper copy under a control, as `Field.tsx`'s `Hint`. Wraps, because
/// every hint in the web app is a sentence rather than a phrase.
pub fn hint(ui: &mut egui::Ui, text: &str) {
    ui.label(RichText::new(text).size(11.0).color(theme::TEXT_DIMMED));
}

/// A value the eye should land on: `<Text c="bright">`.
pub fn value_text(text: &str) -> RichText {
    RichText::new(text)
        .size(theme::FONT_SM)
        .color(theme::TEXT_BRIGHT)
}

/// The standard numeric control -- `SliderField` -- as a labelled row: the
/// label above, the slider filling the width beside a right-aligned readout of
/// `display(value)` followed by a dimmed `unit`, and `hint` underneath.
///
/// Returns whether the drag changed the value, so a caller accumulates
/// `changed |= slider_field(..)` exactly as it accumulates a bare `Slider`.
pub fn slider_field<T: egui::emath::Numeric>(
    ui: &mut egui::Ui,
    name: &str,
    value: &mut T,
    range: std::ops::RangeInclusive<T>,
    display: impl Fn(&T) -> String,
    unit: &str,
    hint_text: Option<&str>,
) -> bool {
    const READOUT_WIDTH: f32 = 88.0;
    let mut changed = false;
    ui.scope(|ui| {
        ui.spacing_mut().item_spacing.y = 2.0;
        label(ui, name);
        ui.horizontal(|ui| {
            let width = (ui.available_width() - READOUT_WIDTH).max(60.0);
            ui.spacing_mut().slider_width = width;
            changed = ui
                .add(egui::Slider::new(value, range).show_value(false))
                .changed();
            ui.label(value_text(&display(value)));
            if !unit.is_empty() {
                ui.label(RichText::new(unit).color(theme::TEXT_DIMMED));
            }
        });
        if let Some(text) = hint_text {
            hint(ui, text);
        }
    });
    changed
}

/// `StatusBanner`: a green tick when `ok`, an amber warning when not, with the
/// message beside it in a tinted frame.
///
/// The tint is the state colour at low alpha over the panel's own ground, which
/// is what Mantine's `variant="light"` Alert paints.
pub fn status_banner(ui: &mut egui::Ui, ok: bool, text: &str) {
    let color = if ok { theme::GREEN } else { theme::YELLOW };
    Frame::new()
        .fill(color.gamma_multiply(0.14))
        .stroke(Stroke::new(1.0, color.gamma_multiply(0.5)))
        .corner_radius(CornerRadius::same(6))
        .inner_margin(Margin::symmetric(8, 6))
        .show(ui, |ui| {
            ui.horizontal_wrapped(|ui| {
                ui.spacing_mut().item_spacing.x = 6.0;
                ui.label(RichText::new(if ok { "✓" } else { "⚠" }).color(color));
                ui.label(RichText::new(text).color(theme::TEXT));
            });
        });
}

/// One section of the settings accordion: the uppercase name, a disclosure
/// triangle, and `body` when it is open.
pub fn section<R>(
    ui: &mut egui::Ui,
    name: &str,
    default_open: bool,
    body: impl FnOnce(&mut egui::Ui) -> R,
) -> Option<R> {
    let out = egui::CollapsingHeader::new(label_text(name))
        .default_open(default_open)
        .show_unindented(ui, |ui| {
            ui.add_space(2.0);
            let out = body(ui);
            ui.add_space(6.0);
            out
        });
    out.body_returned
}

/// A bin button: the bin's own colour as a dot, then its name, tinted and
/// outlined in that colour while it is the active bin. The counterpart of
/// `ShapeTab`'s bin buttons and their `leftSection` swatch.
///
/// Painted rather than composed, because a `Button`'s label is one colour and
/// the dot must be the bin's while the text stays the panel's.
pub fn swatch_button(ui: &mut egui::Ui, selected: bool, color: Color32, text: &str) -> Response {
    const DOT_R: f32 = 4.0;
    let font = egui::TextStyle::Button.resolve(ui.style());
    let ink = if selected {
        theme::TEXT_BRIGHT
    } else {
        theme::TEXT
    };
    let galley = ui.painter().layout_no_wrap(text.to_owned(), font, ink);
    let padding = ui.spacing().button_padding;
    let lead = 3.0 * DOT_R + padding.x;
    let size = egui::vec2(
        galley.size().x + lead + padding.x,
        galley.size().y.max(ui.spacing().interact_size.y) + 2.0 * padding.y,
    );
    let (rect, response) = ui.allocate_exact_size(size, egui::Sense::click());
    let visuals = ui.style().interact(&response);
    let painter = ui.painter();
    painter.rect(
        rect,
        visuals.corner_radius,
        if selected {
            color.gamma_multiply(0.30)
        } else {
            theme::SURFACE
        },
        Stroke::new(1.0, if selected { color } else { theme::BORDER }),
        egui::StrokeKind::Inside,
    );
    painter.circle_filled(
        Pos2::new(rect.left() + padding.x + DOT_R, rect.center().y),
        DOT_R,
        color,
    );
    painter.galley(
        Pos2::new(rect.left() + lead, rect.center().y - galley.size().y / 2.0),
        galley,
        ink,
    );
    response
}

/// A segmented row of mutually exclusive choices, as Mantine's
/// `SegmentedControl`: equal-width cells, the chosen one filled.
///
/// Returns true when the click changed `current`, so the caller can mark itself
/// dirty on the same expression it draws with.
pub fn segmented<T: PartialEq + Copy>(
    ui: &mut egui::Ui,
    current: &mut T,
    options: &[(T, &str)],
    full_width: bool,
) -> bool {
    let mut changed = false;
    ui.horizontal(|ui| {
        ui.spacing_mut().item_spacing.x = 2.0;
        let each = if full_width {
            (ui.available_width() - 2.0 * (options.len() as f32 - 1.0)) / options.len() as f32
        } else {
            0.0
        };
        for (value, text) in options {
            let selected = *current == *value;
            let mut button = egui::Button::new(RichText::new(*text).color(if selected {
                theme::TEXT_BRIGHT
            } else {
                theme::TEXT
            }))
            .fill(if selected {
                theme::BLUE
            } else {
                theme::SURFACE
            })
            .stroke(Stroke::new(
                1.0,
                if selected { theme::BLUE } else { theme::BORDER },
            ));
            if full_width {
                button = button.min_size(egui::vec2(each, 0.0));
            }
            if ui.add(button).clicked() && !selected {
                *current = *value;
                changed = true;
            }
        }
    });
    changed
}

/// A line drawn in `on`-long dashes separated by `off`-long gaps, which is
/// `stroke-dasharray` and the one thing `editor.css` expresses that egui's
/// painter does not: an opening and a candidate cut are both dashed there, and
/// solid they would read as a wall.
pub fn dashed_line(painter: &egui::Painter, points: [Pos2; 2], stroke: Stroke, on: f32, off: f32) {
    assert!(
        on > 0.0 && off > 0.0,
        "a dash pattern advances, but {on} on / {off} off does not"
    );
    let [a, b] = points;
    let span = b - a;
    let length = span.length();
    if length <= f32::EPSILON {
        return;
    }
    let step = span / length;
    let mut travelled = 0.0;
    while travelled < length {
        let end = (travelled + on).min(length);
        painter.line_segment([a + step * travelled, a + step * end], stroke);
        travelled = end + off;
    }
}
