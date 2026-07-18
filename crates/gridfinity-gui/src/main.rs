//! egui front-end for the analytic B-rep Gridfinity engine: a parameter panel,
//! a live glow-rendered 3D preview, and binary-STL export.

#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod viewport;

use eframe::egui;
use gridfinity_cad::gridfinity::{self, BinSlope, Mode, Params, SlopeDir};
use gridfinity_cad::tessellate;
use std::sync::{Arc, Mutex};
use viewport::{Camera, Renderer};

/// Curve resolution (segments per 90° arc) for the live preview vs. export.
const PREVIEW_RES: usize = 20;
const EXPORT_RES: usize = 48;

fn main() -> eframe::Result<()> {
    let options = eframe::NativeOptions {
        depth_buffer: 24,
        renderer: eframe::Renderer::Glow,
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1100.0, 720.0])
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
    // Single rectangular logical bin (params.bins[0]); the engine supports
    // arbitrary polyominoes, the GUI exposes the rectangular case.
    grid_x: u32,
    grid_y: u32,
    // Even-division counts (compartments per axis); expanded to divider edges.
    comp_x: u32,
    comp_y: u32,
    // Slope widget state (params.slope is Some only while enabled).
    slope_on: bool,
    slope_angle: f32,
    slope_dir: SlopeDir,
    gl: Arc<eframe::glow::Context>,
    renderer: Arc<Mutex<Renderer>>,
    camera: Camera,
    dirty: bool,
    tri_count: usize,
    status: String,
}

impl App {
    fn new(cc: &eframe::CreationContext<'_>) -> App {
        let gl = cc.gl.clone().expect("this build requires the glow backend");
        let renderer = Arc::new(Mutex::new(Renderer::new(&gl)));
        let mut app = App {
            params: Params::default(),
            grid_x: 2,
            grid_y: 2,
            comp_x: 1,
            comp_y: 1,
            slope_on: false,
            slope_angle: 20.0,
            slope_dir: SlopeDir::MinusX,
            gl,
            renderer,
            camera: Camera::default(),
            dirty: true,
            tri_count: 0,
            status: String::new(),
        };
        app.regenerate(true);
        app
    }

    /// Rebuild the solid, tessellate, and upload; optionally reframe the camera.
    fn regenerate(&mut self, reframe: bool) {
        let solid = gridfinity::build(&self.params);
        let tess = tessellate(&solid, PREVIEW_RES);
        let (min, max) = tess.bounds();
        self.camera.target = (min + max) * 0.5;
        if reframe {
            self.camera.frame(min, max);
        }
        self.tri_count = tess.tris.len();
        self.renderer.lock().unwrap().upload(&self.gl, &tess.render_buffer());
        self.dirty = false;
    }

    fn export_stl(&mut self) {
        let default_name = format!(
            "gridfinity-{}x{}x{}u.stl",
            self.grid_x, self.grid_y, self.params.height_units
        );
        if let Some(path) = rfd::FileDialog::new()
            .set_file_name(default_name)
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
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        egui::Panel::left("params")
            .resizable(true)
            .default_size(280.0)
            .show(ui, |ui| self.params_panel(ui));

        egui::CentralPanel::default().show(ui, |ui| self.viewport(ui));

        if self.dirty {
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

        egui::Grid::new("dims").num_columns(2).spacing([8.0, 6.0]).show(ui, |ui| {
            ui.label("Grid X");
            let gx = ui.add(egui::DragValue::new(&mut self.grid_x).range(1..=12)).changed();
            ui.end_row();
            ui.label("Grid Y");
            let gy = ui.add(egui::DragValue::new(&mut self.grid_y).range(1..=12)).changed();
            ui.end_row();
            if gx || gy {
                p.bins[0].cells = gridfinity::rect_cells(self.grid_x, self.grid_y);
                changed = true;
            }
            ui.label("Height (7 mm units)");
            changed |= ui.add(egui::DragValue::new(&mut p.height_units).range(1..=30)).changed();
            ui.end_row();
        });

        ui.separator();
        ui.label("Walls & floor");
        changed |= ui.add(egui::Slider::new(&mut p.wall_thickness, 0.8..=4.0).text("wall")).changed();
        changed |= ui.add(egui::Slider::new(&mut p.cavity_corner_radius, 0.0..=8.0).text("corner r")).changed();
        changed |= ui.add(egui::Slider::new(&mut p.floor_fillet, 0.0..=6.0).text("floor fillet")).changed();

        ui.separator();
        ui.label("Compartments");
        let mut divs_changed = false;
        egui::Grid::new("divs").num_columns(2).spacing([8.0, 6.0]).show(ui, |ui| {
            ui.label("Across X");
            divs_changed |= ui.add(egui::DragValue::new(&mut self.comp_x).range(1..=10)).changed();
            ui.end_row();
            ui.label("Across Y");
            divs_changed |= ui.add(egui::DragValue::new(&mut self.comp_y).range(1..=10)).changed();
            ui.end_row();
        });
        if divs_changed || changed {
            // Grid size affects the expansion too, so refresh on any dims change.
            p.divider_edges = gridfinity::divisions_to_edges(
                self.grid_x,
                self.grid_y,
                self.comp_x.saturating_sub(1),
                self.comp_y.saturating_sub(1),
            );
            changed = true;
        }

        ui.separator();
        ui.label("Sloped floor");
        changed |= ui.checkbox(&mut self.slope_on, "Enable (disables floor fillet)").changed();
        if self.slope_on {
            changed |= ui
                .add(egui::Slider::new(&mut self.slope_angle, 5.0..=45.0).text("angle °"))
                .changed();
            ui.horizontal(|ui| {
                for (dir, label) in [
                    (SlopeDir::MinusX, "−X"),
                    (SlopeDir::PlusX, "+X"),
                    (SlopeDir::MinusY, "−Y"),
                    (SlopeDir::PlusY, "+Y"),
                ] {
                    changed |= ui.selectable_value(&mut self.slope_dir, dir, label).changed();
                }
            });
        }
        p.bins[0].slope =
            self.slope_on.then_some(BinSlope { angle_deg: self.slope_angle, dir: self.slope_dir });

        ui.separator();
        ui.label("Fasteners");
        changed |= ui.checkbox(&mut p.magnet_holes, "Magnet holes (⌀6.5 × 2.4)").changed();
        changed |= ui.checkbox(&mut p.screw_holes, "Screw holes (M3 × 6)").changed();

        ui.separator();
        ui.horizontal(|ui| {
            changed |= ui.selectable_value(&mut p.mode, Mode::Bin, "Bin").changed();
            changed |= ui.selectable_value(&mut p.mode, Mode::Baseplate, "Baseplate").changed();
        });

        ui.separator();
        ui.horizontal(|ui| {
            if ui.button("Export STL…").clicked() {
                self.export_stl();
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
    }
}
