//! egui front-end for the analytic B-rep Gridfinity engine: a 2D layout
//! editor (polyomino bins, open/divider edges, split lines, inner walls), a
//! live glow-rendered 3D preview, and binary-STL export (assembled or split
//! pieces).

#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod badapple;
mod debugger;
mod editor;
mod viewport;
mod wireframe;

use eframe::egui;

/// Counts allocations for the debugger's Profile panel. A library must not pick
/// the allocator for its dependents, so the wrapper lives in the kernel and the
/// binary installs it. It only counts while `perf` is enabled, which the
/// debugger switches on around a single rebuild.
#[global_allocator]
static ALLOC: gridfinity_cad::kernel::perf::CountingAlloc<std::alloc::System> =
    gridfinity_cad::kernel::perf::CountingAlloc::new(std::alloc::System);
use debugger::Debugger;
use editor::{BIN_COLORS, Editor, Tool};
use gridfinity_cad::gridfinity::{self, BinSlope, LogicalBin, Mode, Params, SlopeDir};
use gridfinity_cad::layout::GridFootprint;
use gridfinity_cad::printers::{DEFAULT_PRINTER, PRINTER_PROFILES, PrinterProfile, check_bed_fit, compute_auto_split_lines};
use gridfinity_cad::kernel::build::extrude;
use gridfinity_cad::kernel::math::Vec3;
use gridfinity_cad::kernel::sketch::Sketch;
use gridfinity_cad::kernel::topo::Solid;
use gridfinity_cad::tessellate;
use std::sync::{Arc, Mutex};
use viewport::{Camera, Renderer};

/// Curve resolution (segments per 90° arc) for the live preview vs. export.
const PREVIEW_RES: usize = 5;
const EXPORT_RES: usize = 48;

/// Floats per shaded vertex: the kernel's `[pos(3), normal(3)]` plus a
/// "this bin failed to build" flag the GUI appends itself.
pub const MESH_STRIDE: usize = 7;

/// A logical bin the model refused to build, and why.
struct BinError {
    /// Index into `Params::bins`.
    bin: usize,
    msg: String,
}

/// Tessellation → shaded vertex buffer, with every vertex tagged `bad`.
fn flagged(tess: &gridfinity_cad::Tessellation, bad: bool) -> Vec<f32> {
    let src = tess.render_buffer();
    let flag = if bad { 1.0 } else { 0.0 };
    let mut out = Vec::with_capacity(src.len() / 6 * MESH_STRIDE);
    for v in src.chunks_exact(6) {
        out.extend_from_slice(v);
        out.push(flag);
    }
    out
}

fn vert_bounds(verts: &[f32]) -> (Vec3, Vec3) {
    let mut min = Vec3::splat(f32::INFINITY);
    let mut max = Vec3::splat(f32::NEG_INFINITY);
    for v in verts.chunks_exact(MESH_STRIDE) {
        let p = Vec3::new(v[0], v[1], v[2]);
        min = min.min(p);
        max = max.max(p);
    }
    if min.x > max.x { (Vec3::ZERO, Vec3::ZERO) } else { (min, max) }
}

/// Build one logical bin, converting both a returned error and a panic into a
/// message.
///
/// The panic arm is not belt-and-braces. `run_all` reports what it can as an
/// `Err`, but the model layer upstream of it still indexes and asserts its way
/// through geometry that a bad parameter combination can make degenerate, and
/// in a GUI an unwind out of `regenerate` takes the whole window with it. The
/// build is pure — it borrows `Params` and returns a fresh `Solid`, touching no
/// shared state — so catching the unwind here cannot leave anything torn.
fn build_bin(p: &Params, bin: &LogicalBin) -> Result<Solid, String> {
    catch(|| gridfinity::build_piece(p, &bin.cells, &bin.cells, bin.slope))
}

/// Stand-in geometry for a bin that would not build: one plain box per cell, at
/// the bin's real footprint and height. It is deliberately featureless — no
/// pegs, no cavity — so it cannot be mistaken for a successful build, while
/// still showing *where* the bad bin is and how big it is.
fn placeholder(p: &Params, bin: &LogicalBin) -> Vec<f32> {
    let h = (p.height_units as f32 * gridfinity::HEIGHT_PER_UNIT).max(1.0);
    let side = gridfinity::GRID_PITCH - 2.0 * gridfinity::HALF_TOL;
    let mut out = Vec::new();
    for c in &bin.cells {
        let cx = c.x as f32 * gridfinity::GRID_PITCH + gridfinity::GRID_PITCH / 2.0;
        let cy = c.y as f32 * gridfinity::GRID_PITCH + gridfinity::GRID_PITCH / 2.0;
        let sk = Sketch::rounded_rect(cx, cy, side, side, gridfinity::OUTER_R);
        out.extend(flagged(&tessellate(&extrude(&sk, 0.0, h), PREVIEW_RES), true));
    }
    out
}

/// Run a fallible build, turning a panic into the same `Err(String)` a clean
/// failure would produce. See [`build_bin`] for why the panic arm is needed.
fn catch<T>(f: impl FnOnce() -> Result<T, String>) -> Result<T, String> {
    // Silence the default hook for the duration of the build. Its output is
    // redundant — the message is caught below and shown in the viewport — and a
    // slider dragged through a bad parameter range would otherwise spew a
    // backtrace per frame. The swap is global, but only the UI thread builds,
    // and the window it covers is exactly this call.
    let prev = std::panic::take_hook();
    std::panic::set_hook(Box::new(|_| {}));
    let caught = std::panic::catch_unwind(std::panic::AssertUnwindSafe(f));
    std::panic::set_hook(prev);
    match caught {
        Ok(r) => r,
        Err(e) => {
            let what = e
                .downcast_ref::<&str>()
                .map(|s| s.to_string())
                .or_else(|| e.downcast_ref::<String>().cloned())
                .unwrap_or_else(|| "panicked".to_string());
            Err(format!("panicked: {what}"))
        }
    }
}

/// The whole layout as one solid, without the panic.
fn try_whole(p: &Params) -> Result<Solid, String> {
    catch(|| gridfinity::try_build(p))
}

/// Build the whole scene bin by bin, so a bin the model cannot build is
/// isolated: the others still render normally and it gets placeholder geometry
/// flagged for the shader's red glow.
fn build_scene(p: &Params) -> (Vec<f32>, Vec<BinError>) {
    let mut verts = Vec::new();
    let mut errors = Vec::new();
    if p.mode != Mode::Bin {
        // The baseplate is one solid over every cell — there is no per-bin
        // split to isolate, so it succeeds or fails whole.
        match try_whole(p) {
            Ok(s) => verts = flagged(&tessellate(&s, PREVIEW_RES), false),
            Err(msg) => errors.push(BinError { bin: 0, msg }),
        }
        if !errors.is_empty() {
            for bin in &p.bins {
                verts.extend(placeholder(p, bin));
            }
        }
        return (verts, errors);
    }
    for (i, bin) in p.bins.iter().enumerate() {
        if bin.cells.is_empty() {
            continue;
        }
        match build_bin(p, bin) {
            Ok(solid) => verts.extend(flagged(&tessellate(&solid, PREVIEW_RES), false)),
            Err(msg) => {
                errors.push(BinError { bin: i, msg });
                verts.extend(placeholder(p, bin));
            }
        }
    }
    (verts, errors)
}

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
    /// Bins the model refused to build this regenerate; empty when all is well.
    errors: Vec<BinError>,
    /// The Bad Apple stress-test player; `None` unless the demo is running.
    badapple: Option<BadApple>,
}

/// Live "Bad Apple!!" playback: which frame we are on, when playback started,
/// and a smoothed measure of how fast the kernel is turning frames into
/// triangles.
struct BadApple {
    /// Builds frames off the UI thread.
    worker: badapple::Worker,
    /// Frame currently uploaded to the GPU (`usize::MAX` until the first).
    frame: usize,
    /// Frame currently being built by the worker, if any.
    requested: Option<usize>,
    /// `ui.input().time` at which frame 0 should have been shown.
    epoch: f64,
    /// Loop back to the start at the end instead of stopping.
    looping: bool,
    /// Exponentially-smoothed generation rate, triangles per second of kernel
    /// wall time (build only, excluding upload and paint).
    tri_rate: f64,
    /// Triangles in the frame currently shown.
    tris: usize,
    /// Build wall time of the frame currently shown, seconds.
    build_secs: f64,
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
            errors: Vec::new(),
            badapple: None,
        };
        app.regenerate(true);
        app
    }

    /// Rebuild the solid, tessellate, and upload; optionally reframe the camera.
    /// When the debugger is active, the solid is built from its enabled subset
    /// of the model's program; otherwise every logical bin is built on its own
    /// so one bin that fails cannot take the preview down with it.
    fn regenerate(&mut self, reframe: bool) {
        if self.program_dirty {
            self.debugger.refresh(&self.params);
            self.program_dirty = false;
        }
        // The debugger runs an arbitrary op subset, which is *expected* to be
        // non-manifold, so it reports its own status and nothing it produces is
        // flagged. Only the normal preview path builds per bin and can fail.
        let dbg_solid = self.debugger.build_solid();
        let (verts, errors) = match &dbg_solid {
            Some(s) => (flagged(&tessellate(s, PREVIEW_RES), false), Vec::new()),
            None => build_scene(&self.params),
        };
        self.errors = errors;
        let (min, max) = vert_bounds(&verts);
        self.camera.target = (min + max) * 0.5;
        if reframe {
            self.camera.frame(min, max);
        }
        self.tri_count = verts.len() / (3 * MESH_STRIDE);

        // The debugger's wireframe needs the B-rep itself, not the mesh.
        let mut wf = wireframe::Wireframe::default();
        if self.debugger.is_shown() {
            // Sketches first: labels are thinned in insertion order, and there
            // are far fewer sketch tags than B-rep ones, so letting the B-rep
            // edges go first would starve them of screen cells entirely.
            for (profile, plane) in self.debugger.sketch_planes() {
                wf.add_sketch(profile, plane, PREVIEW_RES, wireframe::SKETCH_BLACK);
            }
            if let Some(s) = &dbg_solid {
                wf.add_brep_edges(s, PREVIEW_RES, wireframe::EDGE_ORANGE);
            }
        }
        self.labels = wf.labels;

        let mut r = self.renderer.lock().unwrap();
        r.upload(&self.gl, &verts);
        r.upload_lines(&self.gl, &wf.lines);
        drop(r);
        self.dirty = false;
    }

    /// Start the Bad Apple demo: frame the camera on the plate once and reset
    /// the clock so playback begins at frame 0.
    fn badapple_start(&mut self, time: f64) {
        let (min, max) = badapple::bounds();
        self.camera.frame(Vec3::from_array(min), Vec3::from_array(max));
        // A gentle three-quarter view reads better than dead-on.
        self.camera.yaw = 1.05;
        self.camera.pitch = 0.35;
        self.badapple = Some(BadApple {
            worker: badapple::Worker::spawn(),
            frame: usize::MAX, // force the first build
            requested: None,
            epoch: time,
            looping: true,
            tri_rate: 0.0,
            tris: 0,
            build_secs: 0.0,
        });
        self.errors.clear();
        self.labels.clear(); // no debugger tags floating over the demo
    }

    /// Advance the demo. The worker builds off-thread: we upload whatever frame
    /// it has finished, then queue the frame the wall clock is now on (dropping
    /// any it was too slow to reach). Returns `true` while playback continues.
    fn badapple_tick(&mut self, time: f64) -> bool {
        if self.badapple.is_none() {
            return false;
        }
        let n = badapple::frame_count();

        // End-of-clip: loop or stop.
        {
            let ba = self.badapple.as_mut().unwrap();
            let elapsed = (time - ba.epoch).max(0.0);
            if (elapsed * badapple::FPS) as usize >= n {
                if ba.looping {
                    ba.epoch = time;
                } else {
                    self.badapple = None;
                    self.dirty = true; // fall back to the normal model next frame
                    return false;
                }
            }
        }

        // Upload the most recent finished frame, if the worker produced one.
        if let Some(r) = self.badapple.as_ref().unwrap().worker.try_recv() {
            let ba = self.badapple.as_mut().unwrap();
            ba.requested = None;
            ba.frame = r.frame;
            ba.tris = r.tris;
            ba.build_secs = r.build_secs;
            let inst = if r.build_secs > 0.0 { r.tris as f64 / r.build_secs } else { 0.0 };
            // EMA so the readout is legible instead of flickering frame to frame.
            ba.tri_rate = if ba.tri_rate == 0.0 { inst } else { 0.85 * ba.tri_rate + 0.15 * inst };
            self.tri_count = r.tris;
            let mut rr = self.renderer.lock().unwrap();
            rr.upload(&self.gl, &r.verts);
            rr.upload_lines(&self.gl, &[]);
        }

        // Queue the current target frame if the worker is idle and it's new.
        let ba = self.badapple.as_mut().unwrap();
        let elapsed = (time - ba.epoch).max(0.0);
        let target = ((elapsed * badapple::FPS) as usize).min(n - 1);
        if ba.requested.is_none() && target != ba.frame {
            ba.worker.request(target);
            ba.requested = Some(target);
        }
        true
    }

    fn export_stl(&mut self) {
        // A model that will not build has nothing to write. Refusing here is
        // what stops the export from being the one path that still crashes.
        if !self.errors.is_empty() {
            self.status = "Cannot export: fix the failed bin first".into();
            return;
        }
        if let Some(path) = rfd::FileDialog::new()
            .set_file_name("gridfinity-bin.stl")
            .add_filter("STL", &["stl"])
            .save_file()
        {
            let solid = match try_whole(&self.params) {
                Ok(s) => s,
                Err(msg) => {
                    self.status = format!("Export failed: {msg}");
                    return;
                }
            };
            let mesh = tessellate(&solid, EXPORT_RES).to_mesh();
            match std::fs::write(&path, mesh.to_stl_binary()) {
                Ok(()) => self.status = format!("Exported {} ({} triangles)", path.display(), mesh.tri_count()),
                Err(e) => self.status = format!("Export failed: {e}"),
            }
        }
    }

    /// Split-aware export: one STL per printable piece, into a folder.
    fn export_pieces(&mut self) {
        if !self.errors.is_empty() {
            self.status = "Cannot export: fix the failed bin first".into();
            return;
        }
        let Some(dir) = rfd::FileDialog::new().pick_folder() else { return };
        let pieces = match catch(|| gridfinity::try_build_pieces(&self.params)) {
            Ok(p) => p,
            Err(msg) => {
                self.status = format!("Export failed: {msg}");
                return;
            }
        };
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

        // The demo owns the geometry while it runs: drive it from the wall
        // clock, keep repainting, and skip the normal model rebuild entirely.
        if self.badapple.is_some() {
            let time = ui.input(|i| i.time);
            self.badapple_tick(time);
            ui.ctx().request_repaint();
        } else if self.dirty || dbg_changed {
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

        ui.heading("Gridfinity");
        ui.add_space(4.0);

        // ── Bad Apple!! kernel stress test ────────────────────────────────
        ui.horizontal(|ui| {
            let playing = self.badapple.is_some();
            let label = if playing { "■ Stop Bad Apple" } else { "▶ Bad Apple!!" };
            if ui.button(label).clicked() {
                if playing {
                    self.badapple = None;
                    self.dirty = true;
                } else {
                    let time = ui.input(|i| i.time);
                    self.badapple_start(time);
                }
            }
        });
        if let Some(ba) = &self.badapple {
            let n = badapple::frame_count();
            ui.label(format!("frame {}/{}  ·  {} tris", ba.frame + 1, n, ba.tris));
            ui.label(
                egui::RichText::new(format!("{:.2} M triangles/sec", ba.tri_rate / 1e6))
                    .strong()
                    .color(egui::Color32::from_rgb(120, 220, 140)),
            );
            ui.label(
                egui::RichText::new(format!(
                    "kernel build: {:.2} ms/frame",
                    ba.build_secs * 1e3
                ))
                .small()
                .weak(),
            );
            ui.separator();
            // The rest of the panel drives the model, which is paused; the
            // player owns the viewport until it is stopped.
            return;
        }

        let p = &mut self.params;

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
        // The failed-bin glow pulses, so while anything is flagged the viewport
        // has to keep animating; otherwise egui only repaints on input.
        let time = ui.input(|i| i.time) as f32;
        if !self.errors.is_empty() {
            ui.ctx().request_repaint();
        }
        ui.painter().add(viewport::callback(rect, renderer, cam, time));
        self.paint_labels(ui, rect);
        self.paint_error_banner(ui, rect);
    }

    /// Report failed bins over the viewport: what broke, and which bin the red
    /// glow belongs to. Anchored bottom-left so it never covers the model.
    fn paint_error_banner(&self, ui: &mut egui::Ui, rect: egui::Rect) {
        if self.errors.is_empty() {
            return;
        }
        let red = egui::Color32::from_rgb(255, 90, 70);
        let mut child = ui.new_child(
            egui::UiBuilder::new()
                .max_rect(rect.shrink(12.0))
                .layout(egui::Layout::bottom_up(egui::Align::LEFT)),
        );
        egui::Frame::popup(ui.style())
            .fill(egui::Color32::from_rgba_unmultiplied(40, 6, 8, 235))
            .stroke(egui::Stroke::new(1.0, red))
            .show(&mut child, |ui| {
                ui.set_max_width(rect.width() * 0.6);
                for e in &self.errors {
                    let who = if self.params.mode == Mode::Bin {
                        format!("Bin {}", e.bin + 1)
                    } else {
                        "Baseplate".to_string()
                    };
                    ui.label(
                        egui::RichText::new(format!("{who} could not be built"))
                            .color(red)
                            .strong(),
                    );
                    ui.label(egui::RichText::new(&e.msg).color(egui::Color32::from_rgb(255, 190, 180)));
                }
                ui.label(
                    egui::RichText::new("Shown as a plain block; the rest of the layout is unaffected.")
                        .small()
                        .color(egui::Color32::from_rgb(200, 150, 145)),
                );
            });
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

#[cfg(test)]
mod tests {
    use super::*;
    use gridfinity_cad::layout::GridCell;

    /// A parameter set the model cannot build. `wall_thickness` this small at
    /// one height unit leaves the cavity degenerate, and the model panics
    /// rather than returning an `Err` — which is exactly the case the preview
    /// has to survive.
    fn broken() -> Params {
        Params { height_units: 1, wall_thickness: 0.4, floor_fillet: 0.0,
                 cavity_corner_radius: 0.0, ..Params::default() }
    }

    fn flags(verts: &[f32]) -> (usize, usize) {
        let mut good = 0;
        let mut bad = 0;
        for v in verts.chunks_exact(MESH_STRIDE) {
            if v[6] > 0.5 { bad += 1 } else { good += 1 }
        }
        (good, bad)
    }

    /// The whole point: invalid geometry must not take the process down.
    #[test]
    fn a_bin_that_cannot_be_built_is_reported_not_fatal() {
        let (verts, errors) = build_scene(&broken());
        assert_eq!(errors.len(), 1, "the one bad bin should be reported once");
        assert_eq!(errors[0].bin, 0);
        assert!(!errors[0].msg.is_empty(), "the failure needs a message to show");
        let (good, bad) = flags(&verts);
        assert!(bad > 0, "the failed bin needs placeholder geometry to glow");
        assert_eq!(good, 0, "nothing else built, so nothing should be unflagged");
    }

    /// A good layout stays completely unflagged — the glow must not bleed into
    /// the normal case.
    #[test]
    fn a_valid_layout_reports_nothing_and_flags_nothing() {
        let (verts, errors) = build_scene(&Params::default());
        assert!(errors.is_empty(), "default bin should build: {:?}", errors[0].msg);
        let (good, bad) = flags(&verts);
        assert!(good > 0);
        assert_eq!(bad, 0, "a healthy bin must not be flagged");
    }

    /// One bad bin must not cost the others their geometry — that is what
    /// building per bin buys, versus one program over the whole layout.
    #[test]
    fn a_failed_bin_does_not_take_its_neighbours_with_it() {
        let mut p = broken();
        p.bins = vec![
            LogicalBin { cells: vec![GridCell { x: 0, y: 0 }], ..Default::default() },
            LogicalBin { cells: vec![GridCell { x: 4, y: 0 }], ..Default::default() },
        ];
        let (verts, errors) = build_scene(&p);
        assert_eq!(errors.len(), 2, "both bins share the bad parameters");
        assert_eq!(errors.iter().map(|e| e.bin).collect::<Vec<_>>(), vec![0, 1]);

        // Now make only the second bin's parameters workable.
        let mut ok = Params::default();
        ok.bins = p.bins.clone();
        let (ok_verts, ok_errors) = build_scene(&ok);
        assert!(ok_errors.is_empty());
        assert!(ok_verts.len() > verts.len(), "real geometry beats placeholders");
    }

    /// The placeholder has to land where the bin is, or the glow points at the
    /// wrong part of the layout.
    #[test]
    fn the_placeholder_sits_on_the_failed_bin_footprint() {
        let mut p = broken();
        p.bins = vec![LogicalBin { cells: vec![GridCell { x: 2, y: 1 }], ..Default::default() }];
        let (verts, _) = build_scene(&p);
        let (min, max) = vert_bounds(&verts);
        let pitch = gridfinity::GRID_PITCH;
        assert!(min.x > 2.0 * pitch - 1.0 && max.x < 3.0 * pitch + 1.0, "x {min:?}..{max:?}");
        assert!(min.y > 1.0 * pitch - 1.0 && max.y < 2.0 * pitch + 1.0, "y {min:?}..{max:?}");
        assert!((max.z - gridfinity::HEIGHT_PER_UNIT).abs() < 1e-3, "one height unit tall");
    }
}
