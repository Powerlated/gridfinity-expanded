//! The left panel: the tab strip and the three spatial editors under it.
//!
//! The counterpart of `web/src/components/sidebar/`. `Sidebar.tsx` splits the
//! app's controls the same way this file does -- the tabs hold what is edited
//! *on the grid* (which cells a bin owns, where its walls and openings are,
//! where it is cut) while every form-shaped parameter lives in the other panel
//! -- and the three tabs are named, ordered and worded as its are.
//!
//! Each tab function draws its own controls around the one `Editor::canvas`,
//! and returns whether it changed the model, so the shell marks itself dirty on
//! the same expression it drew with.

use eframe::egui::{self, CornerRadius, Frame, Margin, RichText, Stroke};
use gridfinity_model::gridfinity::{LogicalBin, Params};
use gridfinity_model::layout::GridFootprint;
use gridfinity_model::printers::{BedFitResult, PrinterProfile, check_bed_fit};

use crate::App;
use crate::editor::{Tab, bin_color};
use crate::explode::Explosion;
use crate::theme;
use crate::widgets;

/// The whole left panel drawn, returning whether the frame's input changed the
/// bin.
///
/// The tab strip sits above the scroll view rather than inside it, as the web
/// sidebar's `Tabs.List` sits above its `ScrollArea`: the tabs are how the
/// panel is navigated and scrolling away from them would hide that.
pub fn panel(app: &mut App, ui: &mut egui::Ui) -> bool {
    let mut changed = false;
    widgets::segmented(
        ui,
        &mut app.editor.tab,
        &[(Tab::Shape, "Shape"), (Tab::Walls, "Walls"), (Tab::Cuts, "Cuts")],
        true,
    );
    ui.add_space(4.0);
    egui::ScrollArea::vertical().show(ui, |ui| {
        changed = match app.editor.tab {
            Tab::Shape => shape(app, ui),
            Tab::Walls => walls(app, ui),
            Tab::Cuts => cuts(app, ui),
        };
    });
    changed
}

/// The Shape tab: which bin is being painted, the cell canvas, and what the
/// painted cells add up to. `ShapeTab.tsx`.
fn shape(app: &mut App, ui: &mut egui::Ui) -> bool {
    let mut changed = false;

    ui.horizontal_wrapped(|ui| {
        let mut pick: Option<usize> = None;
        for bi in 0..app.params.bins.len() {
            let selected = bi == app.editor.active_bin;
            if widgets::swatch_button(ui, selected, bin_color(bi), &format!("Bin {}", bi + 1))
                .clicked()
            {
                pick = Some(bi);
            }
        }
        if let Some(bi) = pick {
            app.editor.active_bin = bi;
        }
        if ui.button("+ New").clicked() {
            app.params.bins.push(LogicalBin::default());
            app.editor.active_bin = app.params.bins.len() - 1;
        }
        if app.params.bins.len() > 1 && ui.button("− Bin").clicked() {
            app.params.bins.remove(app.editor.active_bin);
            app.editor.active_bin = 0;
            changed = true;
        }
    });

    widgets::hint(
        ui,
        "Click a cell to paint it into the active bin. A cell another bin owns \
         moves to the active one; the last cell of the only bin stays put.",
    );
    changed |= app.editor.canvas(ui, &mut app.params);

    let p = &app.params;
    let cells: Vec<_> = p.bins.iter().flat_map(|b| b.cells.iter().copied()).collect();
    ui.label(
        RichText::new(format!(
            "{} cell{}{}",
            cells.len(),
            if cells.len() == 1 { "" } else { "s" },
            if p.bins.len() > 1 { format!(" in {} bins", p.bins.len()) } else { String::new() }
        ))
        .color(theme::TEXT_DIMMED),
    );
    if let Some((w, d)) = GridFootprint::from_cells(&cells).map(|f| f.mm(p.pitch)) {
        ui.label(
            RichText::new(format!("{w:.0} × {d:.0} mm layout footprint"))
                .color(theme::TEXT_DIMMED),
        );
    }
    changed
}

/// The Walls tab: the canvas both openings and free-form walls are drawn on,
/// the legend that says which stroke is which, and the walls already drawn.
/// `WallsTab.tsx`.
fn walls(app: &mut App, ui: &mut egui::Ui) -> bool {
    let mut changed = false;
    widgets::hint(
        ui,
        "Click a perimeter edge to toggle an opening, or an internal edge to toggle a \
         divider. Drag anywhere else to draw a straight inner wall.",
    );
    changed |= app.editor.canvas(ui, &mut app.params);

    ui.horizontal_wrapped(|ui| {
        for (color, name) in [
            (theme::GRAY_5, "perimeter"),
            (theme::DARK_2, "opening"),
            (theme::TEAL, "internal wall"),
        ] {
            let (rect, _) = ui.allocate_exact_size(egui::vec2(16.0, 10.0), egui::Sense::hover());
            ui.painter().rect_filled(
                egui::Rect::from_center_size(rect.center(), egui::vec2(16.0, 3.0)),
                CornerRadius::same(2),
                color,
            );
            ui.label(RichText::new(name).color(theme::TEXT_DIMMED));
        }
    });

    ui.horizontal(|ui| {
        widgets::label(ui, "Width");
        ui.add(egui::DragValue::new(&mut app.editor.wall_width).range(0.4..=8.0).speed(0.1));
        ui.checkbox(&mut app.editor.wall_full, "Full height");
        if !app.editor.wall_full {
            ui.add(egui::DragValue::new(&mut app.editor.wall_height).range(0.5..=60.0).speed(0.5));
        }
    });

    if !app.params.inner_walls.is_empty() {
        widgets::label(ui, "Internal walls");
        let mut remove: Option<usize> = None;
        for (i, w) in app.params.inner_walls.iter().enumerate() {
            row(ui, |ui| {
                let height = w.height.map_or("full".to_string(), |h| format!("{h:.1} mm"));
                ui.label(widgets::value_text(&format!(
                    "#{} · ({:.0},{:.0})→({:.0},{:.0})",
                    i + 1,
                    w.x1,
                    w.y1,
                    w.x2,
                    w.y2
                )));
                ui.label(
                    RichText::new(format!("w {:.1} · {height}", w.width)).color(theme::TEXT_DIMMED),
                );
                if ui.small_button("✕").clicked() {
                    remove = Some(i);
                }
            });
        }
        if let Some(i) = remove {
            app.params.inner_walls.remove(i);
            changed = true;
        }
        widgets::hint(ui, "Each wall reaches the rim unless it was drawn at a set height.");
    }
    changed
}

/// The Cuts tab: how many pieces each bin is in, the canvas the split lines are
/// picked on, and whether those pieces reach the bed. `CutsTab.tsx`.
fn cuts(app: &mut App, ui: &mut egui::Ui) -> bool {
    let mut changed = false;
    widgets::hint(
        ui,
        "Cuts are grid lines the bin is split on. Click a faint candidate to add one, \
         or an active cut to remove it. Only the active bin is cut.",
    );

    let pitch = app.params.pitch;
    for (bi, bin) in app.params.bins.iter().enumerate() {
        if bin.cells.is_empty() {
            continue;
        }
        let parts = Explosion::of(bin, pitch).pieces().len();
        ui.horizontal(|ui| {
            let (rect, _) = ui.allocate_exact_size(egui::vec2(12.0, 12.0), egui::Sense::hover());
            ui.painter().circle_filled(rect.center(), 5.0, bin_color(bi));
            ui.label(RichText::new(format!("Bin {}", bi + 1)).color(theme::TEXT));
            ui.label(
                RichText::new(format!("{parts} part{}", if parts == 1 { "" } else { "s" }))
                    .color(theme::TEXT_DIMMED),
            );
        });
    }

    changed |= app.editor.canvas(ui, &mut app.params);

    let printer = app.printer;
    if let Some(worst) = worst_piece(&app.params, printer) {
        widgets::status_banner(
            ui,
            worst.fits,
            &if worst.fits {
                format!(
                    "Every part fits the {} bed ({} × {} mm).",
                    printer.name, printer.bed_width, printer.bed_depth
                )
            } else {
                format!(
                    "The largest part ({} × {} mm) exceeds the {} bed ({} × {} mm).",
                    worst.bin_width,
                    worst.bin_depth,
                    printer.name,
                    printer.bed_width,
                    printer.bed_depth
                )
            },
        );
    }

    if let Some(bin) = app.params.bins.get_mut(app.editor.active_bin) {
        if !bin.split_lines.is_empty() && ui.button("Clear cuts").clicked() {
            bin.split_lines.clear();
            changed = true;
        }
    }
    changed
}

/// The bed fit of the largest piece of any bin, or `None` when no bin has a
/// cell yet: `fits` is whether *every* piece reaches the bed, and the
/// measurements are the biggest piece's.
///
/// Measured per *piece*, not per bin, because a bin the bed cannot take whole is
/// exactly the bin the Cuts tab exists to divide -- `checkDesignFit` measures
/// the web app's parts the same way.
fn worst_piece(p: &Params, printer: PrinterProfile) -> Option<BedFitResult> {
    let mut worst: Option<BedFitResult> = None;
    for bin in &p.bins {
        if bin.cells.is_empty() {
            continue;
        }
        for piece in Explosion::of(bin, p.pitch).pieces() {
            let fit = check_bed_fit(&piece.cells, printer, p.pitch);
            let bigger = worst.is_none_or(|w| {
                (w.fits && !fit.fits)
                    || (w.fits == fit.fits
                        && w.bin_width * w.bin_depth < fit.bin_width * fit.bin_depth)
            });
            if bigger {
                worst = Some(fit);
            }
        }
    }
    worst
}

/// One list row, as the web app's `<Paper p={6} bg="dark.6">`: a tinted strip
/// holding a wall or a cut and the control that removes it.
fn row(ui: &mut egui::Ui, body: impl FnOnce(&mut egui::Ui)) {
    Frame::new()
        .fill(theme::SURFACE)
        .stroke(Stroke::new(1.0, theme::BORDER))
        .corner_radius(CornerRadius::same(6))
        .inner_margin(Margin::symmetric(6, 4))
        .show(ui, |ui| {
            ui.horizontal(|ui| body(ui));
        });
}
