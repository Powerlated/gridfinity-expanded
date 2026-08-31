//! The right panel: every form-shaped parameter, in four collapsible sections.
//!
//! The counterpart of `web/src/components/sidebar/SettingsPanel.tsx`, section
//! for section and in its order -- Dimensions, Features, Printer fit, Display.
//! Nothing spatial lives here: what is edited by pointing at the grid is in
//! `sidebar`, and what is edited by naming a number is here, which is the split
//! the web app makes and the reason both panels stay legible.
//!
//! `panel` returns whether the frame's input changed the bin, so the shell marks
//! itself dirty on the expression it drew with. Sections that change only how
//! the preview is drawn (render quality, what is shown) set `App::dirty`
//! themselves and are not part of that answer.

use eframe::egui::{self, RichText};
use gridfinity_cad::gridfinity::{self, BinSlope, Mode, SlopeDir};
use gridfinity_cad::printers::{PRINTER_PROFILES, check_bed_fit};
use gridfinity_cad::layout::GridFootprint;

use crate::App;
use crate::theme;
use crate::viewport::Quality;
use crate::widgets;

/// The whole right panel drawn in Bin-editor mode.
pub fn panel(app: &mut App, ui: &mut egui::Ui) -> bool {
    let mut changed = false;
    egui::ScrollArea::vertical().show(ui, |ui| {
        changed |= widgets::section(ui, "Dimensions", true, |ui| dimensions(app, ui)).unwrap_or(false);
        changed |= widgets::section(ui, "Features", false, |ui| features(app, ui)).unwrap_or(false);
        changed |= widgets::section(ui, "Printer fit", false, |ui| printer(app, ui)).unwrap_or(false);
        widgets::section(ui, "Display", false, |ui| display(app, ui));
    });
    changed
}

/// Height, wall, cavity corner and floor fillet, then the active bin's sloped
/// floor. `DimensionsTab.tsx` plus the slope the web app has no control for.
fn dimensions(app: &mut App, ui: &mut egui::Ui) -> bool {
    let mut changed = false;
    let p = &mut app.params;

    let millimetres = format!("({:.0} mm)", p.height_units as f64 * gridfinity::HEIGHT_PER_UNIT);
    changed |= widgets::slider_field(
        ui,
        "Height",
        &mut p.height_units,
        1..=30,
        |u| format!("{u}u"),
        &millimetres,
        None,
    );
    changed |= widgets::slider_field(
        ui,
        "Perimeter thickness",
        &mut p.wall_thickness,
        0.8..=4.0,
        |v| format!("{v:.1}"),
        "mm",
        Some("The cavity floor stays at its own thickness."),
    );
    changed |= widgets::slider_field(
        ui,
        "Cavity corner",
        &mut p.cavity_corner_radius,
        0.0..=8.0,
        |v| format!("{v:.1}"),
        "mm",
        None,
    );
    changed |= widgets::slider_field(
        ui,
        "Floor fillet",
        &mut p.floor_fillet,
        0.0..=6.0,
        |v| format!("{v:.1}"),
        "mm",
        Some("Rounds every floor-to-wall transition. A sloped floor takes none."),
    );

    ui.add_space(6.0);
    widgets::label(ui, &format!("Sloped floor · bin {}", app.editor.active_bin + 1));
    if let Some(bin) = p.bins.get_mut(app.editor.active_bin) {
        let mut on = bin.slope.is_some();
        changed |= ui.checkbox(&mut on, "Enable (disables floor fillet)").changed();
        if on {
            let slope = bin.slope.get_or_insert(BinSlope { angle_deg: 20.0, dir: SlopeDir::MinusX });
            changed |= widgets::slider_field(
                ui,
                "Angle",
                &mut slope.angle_deg,
                5.0..=45.0,
                |v| format!("{v:.0}"),
                "°",
                None,
            );
            changed |= widgets::segmented(
                ui,
                &mut slope.dir,
                &[
                    (SlopeDir::MinusX, "−X"),
                    (SlopeDir::PlusX, "+X"),
                    (SlopeDir::MinusY, "−Y"),
                    (SlopeDir::PlusY, "+Y"),
                ],
                true,
            );
        } else if bin.slope.take().is_some() {
            changed = true;
        }
    }
    changed
}

/// The fasteners under the base, and which body the layout is built as.
/// `FeaturesTab.tsx`.
fn features(app: &mut App, ui: &mut egui::Ui) -> bool {
    let mut changed = false;
    let p = &mut app.params;
    widgets::label(ui, "Base attachment");
    changed |= ui.checkbox(&mut p.magnet_holes, "Magnet recesses").changed();
    widgets::hint(ui, "⌀6.5 × 2.4 mm, four per cell.");
    changed |= ui.checkbox(&mut p.screw_holes, "M3 recesses").changed();
    widgets::hint(ui, "M3 × 6 mm, inside the same four base positions.");

    ui.add_space(6.0);
    widgets::label(ui, "Body");
    changed |= widgets::segmented(
        ui,
        &mut p.mode,
        &[(Mode::Bin, "Bin"), (Mode::Baseplate, "Baseplate")],
        true,
    );
    widgets::hint(ui, "A baseplate is the grid the bins drop into. It carries no fasteners.");
    changed
}

/// The bed the parts are printed on, whether the active bin reaches it, and the
/// one action that fixes it. `PrinterTab.tsx`.
fn printer(app: &mut App, ui: &mut egui::Ui) -> bool {
    let mut changed = false;
    egui::ComboBox::from_id_salt("printer")
        .selected_text(app.printer.name)
        .show_ui(ui, |ui| {
            for prof in PRINTER_PROFILES {
                ui.selectable_value(&mut app.printer, *prof, prof.name);
            }
        });
    ui.label(
        RichText::new(format!("{} × {} mm bed", app.printer.bed_width, app.printer.bed_depth))
            .color(theme::TEXT_DIMMED),
    );

    let printer = app.printer;
    let pitch = app.params.pitch;
    let active = app.editor.active_bin;
    let Some(bin) = app.params.bins.get_mut(active) else { return changed };
    if bin.cells.is_empty() {
        widgets::hint(ui, "Paint cells in the Shape tab first.");
        return changed;
    }
    let fit = check_bed_fit(&bin.cells, printer, pitch);
    let (w, d) = GridFootprint::from_cells(&bin.cells).map(|f| f.mm(pitch)).unwrap_or((0.0, 0.0));
    widgets::status_banner(
        ui,
        fit.fits,
        &if fit.fits {
            format!(
                "Bin {} fits whole: {w:.0} × {d:.0} mm{}.",
                active + 1,
                match (fit.rotated, fit.tilt_deg) {
                    (true, _) => ", rotated".to_string(),
                    (_, Some(t)) => format!(", laid at {t:.1}°"),
                    _ => String::new(),
                }
            )
        } else {
            format!("Bin {} ({w:.0} × {d:.0} mm) exceeds the bed. Cut it.", active + 1)
        },
    );
    if !fit.fits && ui.button("Auto-cut to fit").clicked() {
        bin.split_lines =
            gridfinity_cad::printers::compute_auto_split_lines(&bin.cells, printer, pitch);
        changed = true;
    }
    changed
}

/// How the preview is drawn and what is in it -- render quality, the fitted
/// objects and the baseplate, and what the scene came to. `DisplayTab.tsx`,
/// plus the two `optimize` overlays a `--view` run brings with it.
fn display(app: &mut App, ui: &mut egui::Ui) {
    let before = app.quality;
    widgets::segmented(
        ui,
        &mut app.quality,
        &[(Quality::Low, "Low"), (Quality::Medium, "Medium"), (Quality::High, "High")],
        true,
    );
    if app.quality != before {
        app.renderer.lock().unwrap().set_quality(app.quality);
    }
    widgets::hint(
        ui,
        "High adds the floor reflection, contact shadow, ambient occlusion, bloom \
         and anti-aliasing; Low drops all of them. Preview only.",
    );

    if !app.object_boxes.is_empty() {
        ui.add_space(6.0);
        let too_tall = app.object_boxes.iter().filter(|b| !b.fits).count();
        let mut show = app.show_object_boxes;
        if ui
            .checkbox(&mut show, format!("Object boxes ({too_tall} too tall)"))
            .on_hover_text(
                "The boxes the packer reserved for the fitted objects, standing on the \
                 cavity floor. Red is an object taller than the compartment it was \
                 packed into.",
            )
            .changed()
        {
            app.show_object_boxes = show;
            app.dirty = true;
        }
    }
    if app.plate.is_some() {
        let mut show = app.show_plate;
        if ui
            .checkbox(&mut show, "Baseplate")
            .on_hover_text(
                "The grid the fitted bin drops into, cut on its own seams rather than \
                 the bin's and exploded along them. A plate piece spanning a bin seam \
                 is what holds the bin's pieces together, and the other way round.",
            )
            .changed()
        {
            app.show_plate = show;
            app.dirty = true;
        }
    }

    ui.add_space(6.0);
    ui.label(RichText::new(format!("{} triangles", app.tri_count)).color(theme::TEXT_DIMMED));
    if !app.status.is_empty() {
        widgets::hint(ui, &app.status);
    }
}
