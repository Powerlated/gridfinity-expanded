//! egui front-end for the analytic B-rep Gridfinity engine: a 2D layout
//! editor (polyomino bins, open/divider edges, split lines, inner walls), a
//! live glow-rendered 3D preview, and binary-STL export (assembled or split
//! pieces).

#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod debugger;
mod editor;
mod viewport;
mod wireframe;

use eframe::egui;
use debugger::Debugger;
use editor::{BIN_COLORS, Editor, Tool};
use gridfinity_cad::gridfinity::{self, BinSlope, LogicalBin, Mode, Params, SlopeDir};
use gridfinity_cad::layout::GridFootprint;
use gridfinity_cad::printers::{DEFAULT_PRINTER, PRINTER_PROFILES, PrinterProfile, check_bed_fit, compute_auto_split_lines};
use gridfinity_cad::tessellate;
use std::sync::{Arc, Mutex};
use viewport::{Camera, Renderer};

/// Curve resolution (segments per 90° arc) for the live preview vs. export.
const PREVIEW_RES: usize = 5;
const EXPORT_RES: usize = 48;

fn main() -> eframe::Result<()> {
    let options = eframe::NativeOptions {
        depth_buffer: 24,
        renderer: eframe::Renderer::Glow,
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1200.0, 760.0])
            .with_title("Gridfinity Parametric — analytic B-rep CAD"),
        ..Default::default()
    };
    eframe::run_native(
        "gridfinity-gui",
        options,
        Box::new(|cc| Ok(Box::new(App::new(cc)))),
    )
}

struct App {
    params: Params,
    editor: Editor,
    debugger: Debugger,
    printer: PrinterProfile,
    gl: Arc<eframe::glow::Context>,
    renderer: Arc<Mutex<Renderer>>,
    camera: Camera,
    /// Type tags for the debugger overlay, rebuilt with the geometry and
    /// projected to screen space each frame.
    labels: Vec<wireframe::Label>,
    dirty: bool,
    /// Program cache is stale (params changed) — refresh before next regenerate.
    program_dirty: bool,
    tri_count: usize,
    status: String,
}

impl App {
    fn new(cc: &eframe::CreationContext<'_>) -> App {
        let gl = cc.gl.clone().expect("this build requires the glow backend");
        let renderer = Arc::new(Mutex::new(Renderer::new(&gl)));
        let mut app = App {
            params: Params::default(),
            editor: Editor::default(),
            debugger: Debugger::default(),
            printer: DEFAULT_PRINTER,
            gl,
            renderer,
            camera: Camera::default(),
            labels: Vec::new(),
            dirty: true,
            program_dirty: true,
            tri_count: 0,
            status: String::new(),
        };
        app.regenerate(true);
        app
    }

    /// Rebuild the solid, tessellate, and upload; optionally reframe the camera.
    /// When the debugger is active, the solid is built from its enabled subset
    /// of the model's program; otherwise the full `gridfinity::build` is used.
    fn regenerate(&mut self, reframe: bool) {
        if self.program_dirty {
            self.debugger.refresh(&self.params);
            self.program_dirty = false;
        }
        let solid_opt = self.debugger.build_solid();
        let solid = match solid_opt {
            Some(s) => s,
            None => gridfinity::build(&self.params),
        };
        let tess = tessellate(&solid, PREVIEW_RES);
        let (min, max) = tess.bounds();
        self.camera.target = (min + max) * 0.5;
        if reframe {
            self.camera.frame(min, max);
        }
        self.tri_count = tess.tris.len();

        // Build the debugger's wireframe here, while `solid` is still alive —
        // it is dropped at the end of this function.
        let mut wf = wireframe::Wireframe::default();
        if self.debugger.is_shown() {
            // Sketches first: labels are thinned in insertion order, and there
            // are far fewer sketch tags than B-rep ones, so letting the B-rep
            // edges go first would starve them of screen cells entirely.
            for (profile, plane) in self.debugger.sketch_planes() {
                wf.add_sketch(profile, plane, PREVIEW_RES, wireframe::SKETCH_BLACK);
            }
            wf.add_brep_edges(&solid, PREVIEW_RES, wireframe::EDGE_ORANGE);
        }
        self.labels = wf.labels;

        let mut r = self.renderer.lock().unwrap();
        r.upload(&self.gl, &tess.render_buffer());
        r.upload_lines(&self.gl, &wf.lines);
        drop(r);
        self.dirty = false;
    }

    fn export_stl(&mut self) {
        if let Some(path) = rfd::FileDialog::new()
            .set_file_name("gridfinity-bin.stl")
            .add_filter("STL", &["stl"])
            .save_file()
        {
            let solid = gridfinity::build(&self.params);
            let mesh = tessellate(&solid, EXPORT_RES).to_mesh();
            match std::fs::write(&path, mesh.to_stl_binary()) {
                Ok(()) => self.status = format!("Exported {} ({} triangles)", path.display(), mesh.tri_count()),
                Err(e) => self.status = format!("Export failed: {e}"),
            }
        }
    }

    /// Split-aware export: one STL per printable piece, into a folder.
    fn export_pieces(&mut self) {
        let Some(dir) = rfd::FileDialog::new().pick_folder() else { return };
        let pieces = gridfinity::build_pieces(&self.params);
        let mut n = 0usize;
        for piece in &pieces {
            let mesh = tessellate(&piece.solid, EXPORT_RES).to_mesh();
            let path = dir.join(&piece.name);
            match std::fs::write(&path, mesh.to_stl_binary()) {
                Ok(()) => n += 1,
                Err(e) => {
                    self.status = format!("Export failed at {}: {e}", piece.name);
                    return;
                }
            }
        }
        self.status = format!("Exported {n} piece(s) to {}", dir.display());
    }
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        egui::Panel::left("params")
            .resizable(true)
            .default_size(340.0)
            .show(ui, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| self.params_panel(ui));
            });

        let dbg_changed = if self.debugger.is_shown() {
            let mut out = false;
            egui::Panel::right("debug")
                .resizable(true)
                .default_size(320.0)
                .show(ui, |ui| {
                    egui::ScrollArea::neither().show(ui, |ui| {
                        if self.debugger.panel(ui) {
                            out = true;
                        }
                    });
                });
            out
        } else {
            false
        };

        egui::CentralPanel::default().show(ui, |ui| self.viewport(ui));

        if self.dirty || dbg_changed {
            self.dirty = true;
            self.regenerate(false);
        }
    }

    fn on_exit(&mut self, gl: Option<&eframe::glow::Context>) {
        if let Some(gl) = gl {
            self.renderer.lock().unwrap().destroy(gl);
        }
    }
}

impl App {
    fn params_panel(&mut self, ui: &mut egui::Ui) {
        let mut changed = false;
        let p = &mut self.params;

        ui.heading("Gridfinity");
        ui.add_space(4.0);

        // ── Debugger toggle ───────────────────────────────────────────────
        ui.horizontal(|ui| {
            let mut shown = self.debugger.is_shown();
            if ui.checkbox(&mut shown, "Construction debugger").changed() {
                self.debugger.set_shown(shown);
                changed = true;
            }
        });

        // ── Layout editor ────────────────────────────────────────────────
        ui.horizontal(|ui| {
            for (tool, label) in [
                (Tool::Cells, "Cells"),
                (Tool::Edges, "Edges"),
                (Tool::Split, "Split"),
                (Tool::Walls, "Walls"),
            ] {
                ui.selectable_value(&mut self.editor.tool, tool, label);
            }
        });
        ui.horizontal_wrapped(|ui| {
            for bi in 0..p.bins.len() {
                let col = BIN_COLORS[bi % BIN_COLORS.len()];
                let label = egui::RichText::new(format!("Bin {}", bi + 1)).color(col);
                ui.selectable_value(&mut self.editor.active_bin, bi, label);
            }
            if ui.button("＋ bin").clicked() {
                p.bins.push(LogicalBin::default());
                self.editor.active_bin = p.bins.len() - 1;
            }
            if p.bins.len() > 1 && ui.button("− bin").clicked() {
                p.bins.remove(self.editor.active_bin);
                self.editor.active_bin = 0;
                changed = true;
            }
        });
        match self.editor.tool {
            Tool::Cells => ui.label("Click cells to paint the active bin."),
            Tool::Edges => ui.label("Click a perimeter edge → open; internal edge → divider."),
            Tool::Split => ui.label("Click a grid line inside the active bin to split it."),
            Tool::Walls => ui.label("Drag to draw a free-form inner wall."),
        };
        changed |= self.editor.canvas(ui, p);

        if self.editor.tool == Tool::Walls {
            ui.horizontal(|ui| {
                ui.label("width");
                ui.add(egui::DragValue::new(&mut self.editor.wall_width).range(0.4..=8.0).speed(0.1));
                ui.checkbox(&mut self.editor.wall_full, "full height");
                if !self.editor.wall_full {
                    ui.add(egui::DragValue::new(&mut self.editor.wall_height).range(0.5..=60.0).speed(0.5));
                }
            });
            let mut remove: Option<usize> = None;
            for (i, w) in p.inner_walls.iter().enumerate() {
                ui.horizontal(|ui| {
                    let h = w.height.map_or("full".into(), |h| format!("{h:.1} mm"));
                    ui.label(format!(
                        "#{}: ({:.0},{:.0})→({:.0},{:.0}) w{:.1} {h}",
                        i + 1, w.x1, w.y1, w.x2, w.y2, w.width
                    ));
                    if ui.small_button("✕").clicked() {
                        remove = Some(i);
                    }
                });
            }
            if let Some(i) = remove {
                p.inner_walls.remove(i);
                changed = true;
            }
        }

        ui.separator();
        egui::Grid::new("dims").num_columns(2).spacing([8.0, 6.0]).show(ui, |ui| {
            ui.label("Height (7 mm units)");
            changed |= ui.add(egui::DragValue::new(&mut p.height_units).range(1..=30)).changed();
            ui.end_row();
        });

        ui.label("Walls & floor");
        changed |= ui.add(egui::Slider::new(&mut p.wall_thickness, 0.8..=4.0).text("wall")).changed();
        changed |= ui.add(egui::Slider::new(&mut p.cavity_corner_radius, 0.0..=8.0).text("corner r")).changed();
        changed |= ui.add(egui::Slider::new(&mut p.floor_fillet, 0.0..=6.0).text("floor fillet")).changed();

        // ── Per-bin sloped floor ─────────────────────────────────────────
        ui.separator();
        ui.label(format!("Sloped floor (bin {})", self.editor.active_bin + 1));
        if let Some(bin) = p.bins.get_mut(self.editor.active_bin) {
            let mut on = bin.slope.is_some();
            changed |= ui.checkbox(&mut on, "Enable (disables floor fillet)").changed();
            if on {
                let slope = bin.slope.get_or_insert(BinSlope { angle_deg: 20.0, dir: SlopeDir::MinusX });
                changed |= ui
                    .add(egui::Slider::new(&mut slope.angle_deg, 5.0..=45.0).text("angle °"))
                    .changed();
                ui.horizontal(|ui| {
                    for (dir, label) in [
                        (SlopeDir::MinusX, "−X"),
                        (SlopeDir::PlusX, "+X"),
                        (SlopeDir::MinusY, "−Y"),
                        (SlopeDir::PlusY, "+Y"),
                    ] {
                        changed |= ui.selectable_value(&mut slope.dir, dir, label).changed();
                    }
                });
            } else if bin.slope.take().is_some() {
                changed = true;
            }
        }

        ui.separator();
        ui.label("Fasteners");
        changed |= ui.checkbox(&mut p.magnet_holes, "Magnet holes (⌀6.5 × 2.4)").changed();
        changed |= ui.checkbox(&mut p.screw_holes, "Screw holes (M3 × 6)").changed();

        ui.separator();
        ui.horizontal(|ui| {
            changed |= ui.selectable_value(&mut p.mode, Mode::Bin, "Bin").changed();
            changed |= ui.selectable_value(&mut p.mode, Mode::Baseplate, "Baseplate").changed();
        });

        // ── Printer / bed fit ────────────────────────────────────────────
        ui.separator();
        ui.label("Printer");
        egui::ComboBox::from_id_salt("printer")
            .selected_text(self.printer.name)
            .show_ui(ui, |ui| {
                for prof in PRINTER_PROFILES {
                    ui.selectable_value(&mut self.printer, *prof, prof.name);
                }
            });
        if let Some(bin) = p.bins.get_mut(self.editor.active_bin) {
            if !bin.cells.is_empty() {
                let fit = check_bed_fit(&bin.cells, self.printer);
                let (w, d) = GridFootprint::from_cells(&bin.cells)
                    .map(|f| f.mm())
                    .unwrap_or((0.0, 0.0));
                if fit.fits {
                    ui.label(format!("Bin {} fits: {w:.0} × {d:.0} mm{}",
                        self.editor.active_bin + 1,
                        if fit.rotated { " (rotated)" } else { "" }));
                } else {
                    ui.colored_label(
                        egui::Color32::from_rgb(0xd6, 0x63, 0x33),
                        format!("Bin {} exceeds bed ({w:.0} × {d:.0} mm)", self.editor.active_bin + 1),
                    );
                    if ui.button("Auto-split to fit").clicked() {
                        bin.split_lines = compute_auto_split_lines(&bin.cells, self.printer);
                        changed = true;
                    }
                }
                if !bin.split_lines.is_empty() && ui.button("Clear splits").clicked() {
                    bin.split_lines.clear();
                    changed = true;
                }
            }
        }

        // ── Export ───────────────────────────────────────────────────────
        ui.separator();
        ui.horizontal(|ui| {
            if ui.button("Export STL…").clicked() {
                self.export_stl();
            }
            if ui.button("Export pieces…").clicked() {
                self.export_pieces();
            }
            if ui.button("Fit view").clicked() {
                self.regenerate(true);
            }
        });
        ui.label(format!("{} triangles", self.tri_count));
        if !self.status.is_empty() {
            ui.add_space(4.0);
            ui.label(&self.status);
        }

        if changed {
            self.dirty = true;
            self.program_dirty = true;
        }
    }

    fn viewport(&mut self, ui: &mut egui::Ui) {
        let (rect, response) =
            ui.allocate_exact_size(ui.available_size(), egui::Sense::click_and_drag());

        if response.dragged() {
            self.camera.handle_input(response.drag_delta(), 0.0);
        }
        let scroll = ui.input(|i| i.smooth_scroll_delta.y);
        if response.hovered() && scroll != 0.0 {
            self.camera.handle_input(egui::Vec2::ZERO, scroll);
        }

        let cam = self.camera;
        let renderer = self.renderer.clone();
        ui.painter().add(viewport::callback(rect, renderer, cam));
        self.paint_labels(ui, rect);
    }

    /// Paint the overlay's type tags as 2D text tracking their 3D anchors.
    ///
    /// Text goes through egui rather than GL: it rides on top of the paint
    /// callback, so it needs no font atlas of its own and stays crisp.
    ///
    /// A default bin has hundreds of edges, so labels are thinned by claiming a
    /// coarse screen-space cell per label and dropping any that land in a taken
    /// cell. Without that the tags overlap into an unreadable smear at anything
    /// but extreme zoom; with it, density self-adjusts as you zoom in.
    fn paint_labels(&self, ui: &egui::Ui, rect: egui::Rect) {
        if self.labels.is_empty() {
            return;
        }
        const CELL: f32 = 34.0;
        let painter = ui.painter_at(rect);
        let font = egui::FontId::monospace(9.0);
        let mut taken: std::collections::HashSet<(i32, i32)> = std::collections::HashSet::new();
        for label in &self.labels {
            let Some(p) = self.camera.project(label.at, rect) else {
                continue;
            };
            if !rect.contains(p) {
                continue;
            }
            let cell = ((p.x / CELL) as i32, (p.y / CELL) as i32);
            if !taken.insert(cell) {
                continue;
            }
            let [r, g, b] = label.color;
            let color = egui::Color32::from_rgb(
                (r * 255.0) as u8,
                (g * 255.0) as u8,
                (b * 255.0) as u8,
            );
            painter.text(p, egui::Align2::CENTER_CENTER, label.text, font.clone(), color);
        }
    }
}
