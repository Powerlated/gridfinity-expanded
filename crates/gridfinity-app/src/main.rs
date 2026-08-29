
//! The Gridfinity desktop app, and the one command it also answers to.
//!
//! With no arguments it is the egui construction debugger: `App` owns the
//! `Params` being edited, the `Editor` that edits them, the `Debugger` that
//! steps the kernel program, and the viewport the result is drawn in. With
//! `optimize` it is instead the headless drawer fitter in `optimize`/`input`/
//! `export`/`report`, which packs a TOML of objects into a drawer, writes the
//! geometry, and prints what it did. `--view` runs the fitter and then hands the
//! `Params` it produced straight to the same window -- one process, one build,
//! nothing written between them, and with no `-o` beside it, nothing written at
//! all. `Cli` is the whole command line, declared for `clap`.
//!
//! Deliberately **not** `windows_subsystem = "windows"`: the same binary writes
//! the fitting report to stdout, and a windows-subsystem process inherits no
//! console, so the report would go nowhere in a release build.

mod badapple;
mod debugger;
mod editor;
mod export;
mod input;
mod optimize;
mod report;
mod viewport;
mod wireframe;

use clap::{Parser, Subcommand};
use eframe::egui;

#[global_allocator]
static ALLOC: gridfinity_cad::kernel::perf::CountingAlloc<mimalloc::MiMalloc> =
    gridfinity_cad::kernel::perf::CountingAlloc::new(mimalloc::MiMalloc);
use debugger::Debugger;
use editor::{BIN_COLORS, Editor, Tool};
use gridfinity_cad::gridfinity::{self, BinSlope, LogicalBin, Mode, Params, SlopeDir};
use gridfinity_cad::layout::GridFootprint;
use gridfinity_cad::printers::{DEFAULT_PRINTER, PRINTER_PROFILES, PrinterProfile, check_bed_fit, compute_auto_split_lines};
use gridfinity_cad::kernel::build::extrude;
use glam::Vec3;
use gridfinity_cad::kernel::sketch::Sketch;
use gridfinity_cad::kernel::topo::Solid;
use gridfinity_cad::{tessellate, tessellate_shell};
use std::sync::{Arc, Mutex};
use viewport::{Camera, CameraExt, Gpu, Quality, Renderer};

const PREVIEW_RES: usize = 5;
const EXPORT_RES: usize = 48;

pub const MESH_STRIDE: usize = gridfinity_render::VERTEX_STRIDE;
pub const BAD_FLAG_OFFSET: usize = MESH_STRIDE - 1;

pub const DEBUG_BASE_COLOR: u32 = 0x4c8cd9;

struct BinError {
    bin: usize,
    msg: String,
}

fn flagged(tess: &gridfinity_cad::Tessellation, bad: bool) -> Vec<f32> {
    let mut out = Vec::new();
    gridfinity_render::append_smooth_shaded(
        &mut out,
        &tess.render_buffer(),
        Vec3::ZERO,
        gridfinity_render::color_of(DEBUG_BASE_COLOR),
        bad,
    );
    out
}

/// A point the kernel measured in millimetres, as the camera's own `f32`
/// vector. The kernel models in `f64` and the renderer draws in `f32`, so this
/// is where a coordinate crosses.
fn render_point(p: [f64; 3]) -> Vec3 {
    Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32)
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

fn build_bin(p: &Params, bin: &LogicalBin) -> Result<Solid, String> {
    catch(|| gridfinity::build_piece(p, &bin.cells, &bin.cells, bin.slope, &bin.pockets))
}

fn placeholder(p: &Params, bin: &LogicalBin) -> Vec<f32> {
    let h = (p.height_units as f64 * gridfinity::HEIGHT_PER_UNIT).max(1.0);
    let side = gridfinity::GRID_PITCH - 2.0 * gridfinity::HALF_TOL;
    let mut out = Vec::new();
    for c in &bin.cells {
        let cx = c.x as f64 * gridfinity::GRID_PITCH + gridfinity::GRID_PITCH / 2.0;
        let cy = c.y as f64 * gridfinity::GRID_PITCH + gridfinity::GRID_PITCH / 2.0;
        let sk = Sketch::rounded_rect(cx, cy, side, side, gridfinity::OUTER_R);
        out.extend(flagged(&tessellate(&extrude(&sk, 0.0, h), PREVIEW_RES), true));
    }
    out
}

pub(crate) fn catch<T>(f: impl FnOnce() -> Result<T, String>) -> Result<T, String> {
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

fn try_whole(p: &Params) -> Result<Solid, String> {
    catch(|| gridfinity::try_build(p))
}

fn build_scene(p: &Params) -> (Vec<f32>, Vec<BinError>) {
    build_scene_with(p, build_bin)
}

/// `build_scene` with the per-bin builder supplied.
///
/// The tests below drive the failure path through this, rather than through a
/// bin the kernel happens to choke on. A fixture of the second kind is only a
/// fixture until the kernel is fixed -- the "broken" bin these tests used to
/// pass in builds cleanly now -- and what they are actually about is that one
/// bin's failure is reported, is confined to that bin, and leaves a placeholder
/// behind. None of that is a statement about geometry.
fn build_scene_with(
    p: &Params,
    build: impl Fn(&Params, &LogicalBin) -> Result<Solid, String>,
) -> (Vec<f32>, Vec<BinError>) {
    let mut verts = Vec::new();
    let mut errors = Vec::new();
    if p.mode != Mode::Bin {
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
        match build(p, bin) {
            Ok(solid) => verts.extend(flagged(&tessellate(&solid, PREVIEW_RES), false)),
            Err(msg) => {
                errors.push(BinError { bin: i, msg });
                verts.extend(placeholder(p, bin));
            }
        }
    }
    (verts, errors)
}

/// Everything the viewer uploads for one construction-debugger subset.
///
/// A subset is an open shell whenever the steps that would close it are rolled
/// back -- which is the whole point of stepping through a construction -- so it
/// goes through `tessellate_shell`. `tessellate` states a watertight
/// postcondition it cannot hold here, and asserting it turned every rollback
/// into an unwind out of `regenerate`.
fn debug_view(debugger: &Debugger, solid: &Solid) -> (Vec<f32>, wireframe::Wireframe) {
    let verts = flagged(&tessellate_shell(solid, PREVIEW_RES), false);
    let mut wf = wireframe::Wireframe::default();
    for (profile, plane) in debugger.sketch_planes() {
        wf.add_sketch(profile, plane, PREVIEW_RES, wireframe::SKETCH_BLACK);
    }
    wf.add_brep_edges(solid, PREVIEW_RES, wireframe::EDGE_ORANGE);
    (verts, wf)
}

/// The command line: no subcommand opens the construction debugger, `optimize`
/// runs the headless drawer fitter.
///
/// `clap` owns the parsing, the spellings and the help text, so `--help` and
/// every "that is not an option" message come from the declaration rather than
/// from a hand-written usage string that had to be kept in step with it.
#[derive(Parser, Debug)]
#[command(
    name = "gridfinity-app",
    about = "The Gridfinity parametric CAD app",
    long_about = "With no arguments, opens the construction debugger on a default bin. With `optimize`, fits a drawer full of objects headlessly and writes the geometry.",
    disable_help_subcommand = true
)]
struct Cli {
    #[command(subcommand)]
    command: Option<Command>,
}

#[derive(Subcommand, Debug)]
enum Command {
    Optimize(optimize::Args),
}

/// Dispatches the command line: `optimize` runs the fitter and opens a window
/// only if it asked to, no subcommand opens the debugger on a default bin.
fn main() -> eframe::Result<()> {
    let initial = match Cli::parse().command {
        Some(Command::Optimize(args)) => match optimize::run(&args) {
            Ok(Some(view)) => Some(view),
            Ok(None) => return Ok(()),
            Err(message) => {
                eprintln!("error: {message}");
                std::process::exit(1);
            }
        },
        None => None,
    };
    window(initial)
}

/// Opens the construction debugger, on the given fit when there is one.
fn window(initial: Option<optimize::View>) -> eframe::Result<()> {
    let options = eframe::NativeOptions {
        renderer: eframe::Renderer::Wgpu,
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1200.0, 760.0])
            .with_title("Gridfinity Parametric — analytic B-rep CAD"),
        ..Default::default()
    };
    eframe::run_native(
        "gridfinity-app",
        options,
        Box::new(move |cc| Ok(Box::new(App::new(cc, initial)))),
    )
}

struct App {
    params: Params,
    editor: Editor,
    debugger: Debugger,
    printer: PrinterProfile,
    gpu: Gpu,
    renderer: Arc<Mutex<Renderer>>,
    camera: Camera,
    quality: Quality,
    labels: Vec<wireframe::Label>,
    object_boxes: Vec<optimize::ObjectBox>,
    show_object_boxes: bool,
    dirty: bool,
    program_dirty: bool,
    tri_count: usize,
    status: String,
    errors: Vec<BinError>,
    badapple: Option<BadApple>,
}

struct BadApple {
    worker: badapple::Worker,
    frame: usize,
    inflight: usize,
    last_requested: Option<usize>,
    epoch: f64,
    looping: bool,
    tri_rate: f64,
    tris: usize,
    build_secs: f64,
}

impl App {
    fn new(cc: &eframe::CreationContext<'_>, initial: Option<optimize::View>) -> App {
        let state =
            cc.wgpu_render_state.as_ref().expect("this build requires the wgpu backend");
        let gpu = Gpu {
            device: state.device.clone(),
            queue: state.queue.clone(),
            format: state.target_format,
        };
        let renderer = Arc::new(Mutex::new(
            Renderer::new(&gpu.device, &state.adapter)
                .expect("the wgpu backend must build the viewport pipelines"),
        ));
        let (params, object_boxes) = match initial {
            Some(view) => (view.params, view.boxes),
            None => (Params::default(), Vec::new()),
        };
        let mut app = App {
            show_object_boxes: !object_boxes.is_empty(),
            object_boxes,
            params,
            editor: Editor::default(),
            debugger: Debugger::default(),
            printer: DEFAULT_PRINTER,
            gpu,
            renderer,
            camera: Camera::default(),
            quality: Quality::default(),
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

    fn regenerate(&mut self, reframe: bool) {
        if self.program_dirty {
            self.debugger.refresh(&self.params);
            self.program_dirty = false;
        }
        let dbg_solid = self.debugger.build_solid();
        let mut wf = wireframe::Wireframe::default();
        let (verts, errors) = match &dbg_solid {
            Some(s) => match catch(|| Ok(debug_view(&self.debugger, s))) {
                Ok((v, w)) => {
                    wf = w;
                    (v, Vec::new())
                }
                Err(msg) => (Vec::new(), vec![BinError { bin: 0, msg }]),
            },
            None => build_scene(&self.params),
        };
        self.errors = errors;
        let (min, max) = vert_bounds(&verts);
        self.camera.target = (min + max) * 0.5;
        if reframe {
            self.camera.frame(min, max);
        }
        self.tri_count = verts.len() / (3 * MESH_STRIDE);

        if dbg_solid.is_none() && self.debugger.is_shown() {
            for (profile, plane) in self.debugger.sketch_planes() {
                wf.add_sketch(profile, plane, PREVIEW_RES, wireframe::SKETCH_BLACK);
            }
        }
        if self.show_object_boxes {
            for b in &self.object_boxes {
                let color = if b.fits { wireframe::OBJECT_BLUE } else { wireframe::OBJECT_RED };
                wf.add_box(b.min, b.max, color);
            }
        }
        self.labels = wf.labels;

        let mut r = self.renderer.lock().unwrap();
        r.upload(&self.gpu.device, &self.gpu.queue, &verts);
        r.upload_lines(&self.gpu.device, &self.gpu.queue, &wf.lines);
        drop(r);
        self.dirty = false;
    }

    fn badapple_start(&mut self, time: f64) {
        let (min, max) = badapple::bounds();
        self.camera.frame(render_point(min), render_point(max));
        self.camera.yaw = 1.05;
        self.camera.pitch = 0.35;
        self.badapple = Some(BadApple {
            worker: badapple::Worker::spawn(),
            frame: usize::MAX,
            inflight: 0,
            last_requested: None,
            epoch: time,
            looping: true,
            tri_rate: 0.0,
            tris: 0,
            build_secs: 0.0,
        });
        self.errors.clear();
        self.labels.clear();
    }

    fn badapple_tick(&mut self, time: f64) -> bool {
        if self.badapple.is_none() {
            return false;
        }
        let n = badapple::frame_count();

        {
            let ba = self.badapple.as_mut().unwrap();
            let elapsed = (time - ba.epoch).max(0.0);
            if (elapsed * badapple::FPS) as usize >= n {
                if ba.looping {
                    ba.epoch = time;
                } else {
                    self.badapple = None;
                    self.dirty = true;
                    return false;
                }
            }
        }

        if let Some((r, seen)) = self.badapple.as_ref().unwrap().worker.try_recv() {
            let ba = self.badapple.as_mut().unwrap();
            ba.inflight = ba.inflight.saturating_sub(seen);
            ba.frame = r.frame;
            ba.tris = r.tris;
            ba.build_secs = r.build_secs;
            let inst = if r.build_secs > 0.0 { r.tris as f64 / r.build_secs } else { 0.0 };
            ba.tri_rate = if ba.tri_rate == 0.0 { inst } else { 0.85 * ba.tri_rate + 0.15 * inst };
            self.tri_count = r.tris;
            let mut rr = self.renderer.lock().unwrap();
            rr.upload(&self.gpu.device, &self.gpu.queue, &r.verts);
            rr.upload_lines(&self.gpu.device, &self.gpu.queue, &[]);
        }

        let ba = self.badapple.as_mut().unwrap();
        let elapsed = (time - ba.epoch).max(0.0);
        let target = ((elapsed * badapple::FPS) as usize).min(n - 1);
        if ba.inflight < badapple::PIPELINE_DEPTH
            && target != ba.frame
            && ba.last_requested != Some(target)
        {
            ba.worker.request(target);
            ba.last_requested = Some(target);
            ba.inflight += 1;
        }
        true
    }

    /// The bin as a Rust literal, with what the fillets actually did to it.
    ///
    /// The config alone says what was asked for; the `BlendReport` says which
    /// blends landed and, when one did not, the message the kernel refused it
    /// with. That message is the part that names the defect -- a count of
    /// dropped edges does not -- and it is otherwise only visible from inside
    /// `fillet_best_effort`.
    fn config_report(&self) -> String {
        let mut out = self.params.rust_literal();
        out.push('\n');
        match catch(|| gridfinity::try_build_reporting(&self.params)) {
            Ok((_, r)) => {
                out.push_str(&format!(
                    "// blends: {} requested, {} made, {} matched no edge, {} refused\n",
                    r.requested,
                    r.made(),
                    r.unresolved,
                    r.dropped.len()
                ));
                if let Some(why) = &r.refusal {
                    out.push_str(&format!("// refused because: {why}\n"));
                }
            }
            // `catch` turns a panic into this too, which is the case worth
            // exporting most: a bin the kernel cannot build at all.
            Err(msg) => out.push_str(&format!("// build failed: {msg}\n")),
        }
        out
    }

    fn export_config(&mut self, ctx: &egui::Context) {
        let text = self.config_report();
        ctx.copy_text(text.clone());
        let line = text.lines().count();
        self.status = match rfd::FileDialog::new()
            .set_file_name("bin-config.rs")
            .add_filter("Rust", &["rs"])
            .save_file()
        {
            Some(path) => match std::fs::write(&path, &text) {
                Ok(()) => format!("Config copied to clipboard and written to {}", path.display()),
                Err(e) => format!("Config copied to clipboard; writing failed: {e}"),
            },
            None => format!("Config copied to clipboard ({line} line(s))"),
        };
    }

    fn export_stl(&mut self) {
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

        if self.badapple.is_some() {
            let time = ui.input(|i| i.time);
            self.badapple_tick(time);
            ui.ctx().request_repaint();
        } else if self.dirty || dbg_changed {
            self.dirty = true;
            self.regenerate(false);
        }

        egui::CentralPanel::default().show(ui, |ui| self.viewport(ui));
    }

    fn on_exit(&mut self) {
        self.renderer.lock().unwrap().destroy();
    }
}

impl App {
    fn params_panel(&mut self, ui: &mut egui::Ui) {
        let mut changed = false;

        ui.heading("Gridfinity");
        ui.add_space(4.0);

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
            let shown = if ba.frame == usize::MAX { 0 } else { ba.frame + 1 };
            ui.label(format!("frame {shown}/{n}  ·  {} tris", ba.tris));
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
            return;
        }

        let p = &mut self.params;

        ui.horizontal(|ui| {
            let mut shown = self.debugger.is_shown();
            if ui.checkbox(&mut shown, "Construction debugger").changed() {
                self.debugger.set_shown(shown);
                changed = true;
            }
        });

        ui.horizontal(|ui| {
            ui.label("Render");
            let before = self.quality;
            for (level, label) in [
                (Quality::Low, "Low"),
                (Quality::Medium, "Medium"),
                (Quality::High, "High"),
            ] {
                ui.selectable_value(&mut self.quality, level, label);
            }
            if self.quality != before {
                self.renderer.lock().unwrap().set_quality(self.quality);
            }
        });

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
        ui.horizontal(|ui| {
            if ui
                .button("Copy config")
                .on_hover_text(
                    "The bin as a Rust `Params` literal, plus what the fillets did to it. \
                     Goes to the clipboard, and optionally to a file. Paste it into a test \
                     to reproduce this exact bin.",
                )
                .clicked()
            {
                let ctx = ui.ctx().clone();
                self.export_config(&ctx);
            }
        });
        if !self.object_boxes.is_empty() {
            let too_tall = self.object_boxes.iter().filter(|b| !b.fits).count();
            let mut show = self.show_object_boxes;
            if ui
                .checkbox(&mut show, format!("Object boxes ({too_tall} too tall)"))
                .on_hover_text(
                    "The boxes the packer reserved for the fitted objects, standing on the \
                     cavity floor. Red is an object taller than the compartment it was \
                     packed into.",
                )
                .changed()
            {
                self.show_object_boxes = show;
                self.dirty = true;
            }
        }
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
        let time = ui.input(|i| i.time) as f32;
        if !self.errors.is_empty() || self.renderer.lock().unwrap().is_accumulating() {
            ui.ctx().request_repaint();
        }
        ui.painter().add(viewport::callback(rect, self.gpu.clone(), renderer, cam, time));
        self.paint_labels(ui, rect);
        self.paint_error_banner(ui, rect);
    }

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

    fn paint_labels(&self, ui: &egui::Ui, rect: egui::Rect) {
        if self.labels.is_empty() {
            return;
        }
        const CELL: f32 = 34.0;
        let painter = ui.painter_at(rect);
        let font = egui::FontId::monospace(9.0);
        let mut taken: std::collections::HashSet<(i32, i32)> = std::collections::HashSet::new();
        for label in &self.labels {
            let Some(p) = self.camera.project(label.at.as_vec3(), rect) else {
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

    /// A builder that refuses everything, so the failure path is exercised
    /// without asking the kernel for a bin it cannot make.
    fn always_fails(_: &Params, _: &LogicalBin) -> Result<Solid, String> {
        Err("refused by the test builder".to_string())
    }

    fn bins_at(cells: &[GridCell]) -> Params {
        Params {
            bins: cells
                .iter()
                .map(|&c| LogicalBin { cells: vec![c], ..Default::default() })
                .collect(),
            height_units: 1,
            ..Params::default()
        }
    }

    fn flags(verts: &[f32]) -> (usize, usize) {
        let mut good = 0;
        let mut bad = 0;
        for v in verts.chunks_exact(MESH_STRIDE) {
            if v[BAD_FLAG_OFFSET] > 0.5 { bad += 1 } else { good += 1 }
        }
        (good, bad)
    }

    #[test]
    fn a_bin_that_cannot_be_built_is_reported_not_fatal() {
        let p = bins_at(&[GridCell { x: 0, y: 0 }]);
        let (verts, errors) = build_scene_with(&p, always_fails);
        assert_eq!(errors.len(), 1, "the one bad bin should be reported once");
        assert_eq!(errors[0].bin, 0);
        assert!(!errors[0].msg.is_empty(), "the failure needs a message to show");
        let (good, bad) = flags(&verts);
        assert!(bad > 0, "the failed bin needs placeholder geometry to glow");
        assert_eq!(good, 0, "nothing else built, so nothing should be unflagged");
    }

    #[test]
    fn a_valid_layout_reports_nothing_and_flags_nothing() {
        let (verts, errors) = build_scene(&Params::default());
        assert!(errors.is_empty(), "default bin should build: {:?}", errors[0].msg);
        let (good, bad) = flags(&verts);
        assert!(good > 0);
        assert_eq!(bad, 0, "a healthy bin must not be flagged");
    }

    #[test]
    fn a_failed_bin_does_not_take_its_neighbours_with_it() {
        let p = bins_at(&[GridCell { x: 0, y: 0 }, GridCell { x: 4, y: 0 }]);
        let (verts, errors) = build_scene_with(&p, always_fails);
        assert_eq!(errors.len(), 2, "both bins were refused");
        assert_eq!(errors.iter().map(|e| e.bin).collect::<Vec<_>>(), vec![0, 1]);

        let (ok_verts, ok_errors) = build_scene(&p);
        assert!(ok_errors.is_empty());
        assert!(ok_verts.len() > verts.len(), "real geometry beats placeholders");
    }

    #[test]
    fn the_placeholder_sits_on_the_failed_bin_footprint() {
        let p = bins_at(&[GridCell { x: 2, y: 1 }]);
        let (verts, _) = build_scene_with(&p, always_fails);
        let (min, max) = vert_bounds(&verts);
        let pitch = gridfinity::GRID_PITCH as f32;
        assert!(min.x > 2.0 * pitch - 1.0 && max.x < 3.0 * pitch + 1.0, "x {min:?}..{max:?}");
        assert!(min.y > 1.0 * pitch - 1.0 && max.y < 2.0 * pitch + 1.0, "y {min:?}..{max:?}");
        assert!(
            (max.z - gridfinity::HEIGHT_PER_UNIT as f32).abs() < 1e-3,
            "one height unit tall"
        );
    }
}
