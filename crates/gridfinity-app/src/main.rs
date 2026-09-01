
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

mod debugger;
mod editor;
mod explode;
mod export;
mod grouping;
mod input;
mod optimize;
mod report;
mod settings;
mod sidebar;
mod theme;
mod viewport;
mod widgets;
mod wireframe;

use clap::{Parser, Subcommand};
use eframe::egui;

#[global_allocator]
static ALLOC: gridfinity_brep::perf::CountingAlloc<mimalloc::MiMalloc> =
    gridfinity_brep::perf::CountingAlloc::new(mimalloc::MiMalloc);
use debugger::Debugger;
use explode::Explosion;
use editor::Editor;
use gridfinity_model::gridfinity::{self, LogicalBin, Mode, Params};
use gridfinity_brep::math::Vec3 as KernelVec3;
use gridfinity_model::layout::{GridCell, SplitLine, partition_cells};
use gridfinity_model::printers::{DEFAULT_PRINTER, PrinterProfile};
use gridfinity_brep::build::extrude;
use glam::Vec3;
use gridfinity_brep::sketch::Sketch;
use gridfinity_brep::topo::Solid;
use gridfinity_model::subbin::build_subbin;
use gridfinity_model::{tessellate, tessellate_shell};
use std::sync::{Arc, Mutex};
use viewport::{Camera, CameraExt, Gpu, Quality, Renderer};

const PREVIEW_RES: usize = 5;
const EXPORT_RES: usize = 48;

pub const MESH_STRIDE: usize = gridfinity_render::VERTEX_STRIDE;
pub const BAD_FLAG_OFFSET: usize = MESH_STRIDE - 1;

pub const DEBUG_BASE_COLOR: u32 = 0x4c8cd9;

/// The colour of a packed object's box: white, so an object reads as a thing
/// placed in the bin rather than as part of it.
pub const OBJECT_WHITE: u32 = 0xffffff;

/// The colour a label is drawn in: near black, which reads over the bin's blue,
/// the plate's grey and an object's white alike.
pub const LABEL_INK: [f32; 3] = [0.05, 0.05, 0.06];

/// The colour of a label on an object that does not clear the cavity it was
/// packed into -- the same thing the box's own red rim says, in words.
pub const LABEL_BAD: [f32; 3] = [0.75, 0.1, 0.05];

/// The colour of the baseplate under a fitted bin: a neutral grey, so the two
/// bodies read apart where they interleave and a plate piece lapping a bin seam
/// is visible for what it is.
pub const PLATE_GREY: u32 = 0x8c8f94;

/// The colour of an insert standing in a compartment: a warm amber, so a body
/// that is neither the bin nor the plate nor an object reads as its own part.
pub const SUBBIN_AMBER: u32 = 0xd9a441;

struct BinError {
    bin: usize,
    msg: String,
}

fn shaded(tess: &gridfinity_model::Tessellation, rgb: u32, bad: bool) -> Vec<f32> {
    let mut out = Vec::new();
    gridfinity_render::append_smooth_shaded(
        &mut out,
        &tess.render_buffer(),
        Vec3::ZERO,
        gridfinity_render::color_of(rgb),
        bad,
    );
    out
}

fn flagged(tess: &gridfinity_model::Tessellation, bad: bool) -> Vec<f32> {
    shaded(tess, DEBUG_BASE_COLOR, bad)
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

/// The same vertices, every position moved by `shift` and every normal, colour
/// and flag left as it was.
fn displace(verts: &mut [f32], shift: Vec3) {
    for v in verts.chunks_exact_mut(MESH_STRIDE) {
        v[0] += shift.x;
        v[1] += shift.y;
        v[2] += shift.z;
    }
}

/// The explosion as asked for: itself when the gaps are shown, collapsed when
/// they are not.
///
/// One function so the bin, the baseplate and the object boxes cannot disagree
/// about whether the scene is open -- a body standing apart from boxes that
/// stayed put is a picture of nothing.
fn explode(explosion: Explosion, gaps: bool) -> Explosion {
    if gaps { explosion } else { explosion.collapsed() }
}

/// One bin's preview vertices, given the whole solid the kernel built for it:
/// that solid tessellated when the bin is not split, and otherwise its carved
/// pieces, each displaced by its band's `Explosion::shift` so every cut the
/// printer will make opens by one gap.
///
/// The pieces are the same ones the export writes -- the bin's own split lines,
/// carved out of the one solid -- so what the window shows apart is what the
/// files hold separately, and a carve the kernel refuses here is the error it
/// would be there.
fn bin_vertices(
    bin: &LogicalBin,
    pitch: f64,
    solid: &Solid,
    gaps: bool,
) -> Result<Vec<f32>, String> {
    let explosion = explode(Explosion::of(bin, pitch), gaps);
    if !explosion.is_split() {
        return Ok(flagged(&tessellate(solid, PREVIEW_RES), false));
    }
    let mut out = Vec::new();
    for part in explosion.pieces() {
        let piece = catch(|| gridfinity::carve_to_cells(solid, pitch, &bin.cells, &part.cells))?;
        let mut verts = flagged(&tessellate(&piece, PREVIEW_RES), false);
        displace(&mut verts, explosion.shift(part.col, part.row));
        out.extend(verts);
    }
    Ok(out)
}

/// The whole baseplate's preview vertices, given the one solid the kernel built
/// for it: that solid tessellated when the plate is not split, and otherwise its
/// carved pieces, each displaced by its band's `Explosion::shift`.
///
/// The plate is cut on **its own** lines -- the union of every bin's
/// `split_lines`, which `optimize` staggers off the bin's -- so it explodes
/// along bands of its own and a plate piece visibly laps a seam of the bin
/// above it. That is the interlock made visible: neither body can be lifted out
/// without the pieces of the other that span its seams. Carving is
/// `carve_baseplate_to_cells`, the same call the export makes, so what the
/// window shows apart is what the files hold separately.
fn plate_vertices(p: &Params, solid: &Solid, gaps: bool) -> Result<Vec<f32>, String> {
    let cells = p.all_cells();
    let mut splits: Vec<SplitLine> = Vec::new();
    for bin in &p.bins {
        for line in &bin.split_lines {
            if !splits.contains(line) {
                splits.push(*line);
            }
        }
    }
    let explosion = explode(Explosion::new(&cells, &splits, p.pitch), gaps);
    if !explosion.is_split() {
        return Ok(shaded(&tessellate(solid, PREVIEW_RES), PLATE_GREY, false));
    }
    let mut out = Vec::new();
    for part in explosion.pieces() {
        let piece = catch(|| {
            gridfinity::carve_baseplate_to_cells(
                solid,
                p.pitch,
                &cells,
                &part.cells,
                p.plate_margin_x,
                p.plate_margin_y,
            )
        })?;
        let mut verts = shaded(&tessellate(&piece, PREVIEW_RES), PLATE_GREY, false);
        displace(&mut verts, explosion.shift(part.col, part.row));
        out.extend(verts);
    }
    Ok(out)
}

/// The packed objects as solid boxes in the drawer's own millimetre coordinates,
/// each cut on the split lines of the bin it stands in and each part displaced
/// with the piece it lies in, so an object crosses a cut exactly the way the
/// body holding it does.
///
/// White, and flagged bad -- the renderer's pulsing red rim -- for an object
/// that does not clear the cavity, which is the report's `is N mm tall, but a
/// compartment is only M mm deep` warning made visible. The box stands its full
/// height either way: clipping it to the cavity would hide the thing worth
/// seeing.
fn object_box_vertices(
    boxes: &[optimize::ObjectBox],
    params: &Params,
    gaps: bool,
) -> Vec<f32> {
    let explosions: Vec<Explosion> = params
        .bins
        .iter()
        .map(|bin| explode(Explosion::of(bin, params.pitch), gaps))
        .collect();
    let mut out = Vec::new();
    for b in boxes {
        assert!(b.max.z > b.min.z, "an object box stands some height, but {} is not under {}", b.min, b.max);
        let explosion = &explosions[b.bin];
        for part in explosion.pieces() {
            let Some((min, max)) = explosion.clip(part.col, part.row, b.min, b.max) else {
                continue;
            };
            let sketch = Sketch::rectangle(
                (min.x + max.x) / 2.0,
                (min.y + max.y) / 2.0,
                max.x - min.x,
                max.y - min.y,
            );
            let solid = extrude(&sketch, min.z, max.z);
            let mut verts = shaded(&tessellate(&solid, PREVIEW_RES), OBJECT_WHITE, !b.fits);
            displace(&mut verts, explosion.shift(part.col, part.row));
            out.extend(verts);
        }
    }
    out
}

/// The centre of a set of cells in millimetres, on the x-y plane: where the
/// label naming the body those cells make up is drawn.
///
/// The centre of the *cells* and not of the built solid, because the two differ
/// by an inset a label does not care about and cells are what every caller here
/// already has.
fn cells_centre(cells: &[GridCell], pitch: f64) -> (f64, f64) {
    let (mut lo_x, mut hi_x, mut lo_y, mut hi_y) = (i32::MAX, i32::MIN, i32::MAX, i32::MIN);
    for c in cells {
        lo_x = lo_x.min(c.x);
        hi_x = hi_x.max(c.x);
        lo_y = lo_y.min(c.y);
        hi_y = hi_y.max(c.y);
    }
    assert!(
        lo_x <= hi_x && lo_y <= hi_y,
        "a body to label has at least one cell, but its cells span {lo_x}..{hi_x} x {lo_y}..{hi_y}"
    );
    (
        (lo_x + hi_x + 1) as f64 * pitch / 2.0,
        (lo_y + hi_y + 1) as f64 * pitch / 2.0,
    )
}

/// One body named once: `text` at the centre of `cells`, on top of the body at
/// height `top`, moved with the band that centre stands in.
///
/// **A label names an item, not a piece of one.** Cutting a bin for the bed does
/// not make it two bins, so a split body carries the one name its unsplit self
/// carries; the label rides the band its centre falls in so it stands on the
/// body rather than in the gap a cut opened.
fn body_label(
    explosion: &Explosion,
    cells: &[GridCell],
    pitch: f64,
    top: f64,
    text: String,
    color: [f32; 3],
) -> wireframe::Label {
    let (x, y) = cells_centre(cells, pitch);
    let shift = explosion.shift_at(x, y);
    wireframe::Label {
        at: KernelVec3::new(x + f64::from(shift.x), y + f64::from(shift.y), top),
        text,
        color,
    }
}

/// One body named as many times as it becomes files: a label over each piece the
/// cells and `splits` partition into, reading `name` above that piece's own
/// file, or a single label over the whole body where `files` is empty.
///
/// `partition_cells` here is the same call the exporter and `bin_vertices` make,
/// so piece `i` is one body across all three and `files[i]` is that body's own
/// name. The count is asserted rather than zipped over, because a short list
/// would otherwise silently leave the last pieces unnamed.
fn body_labels(
    explosion: &Explosion,
    cells: &[GridCell],
    splits: &[SplitLine],
    pitch: f64,
    top: f64,
    name: &str,
    files: &[String],
) -> Vec<wireframe::Label> {
    if files.is_empty() {
        return vec![body_label(explosion, cells, pitch, top, name.to_string(), LABEL_INK)];
    }
    let parts = partition_cells(cells, splits);
    assert_eq!(
        parts.len(),
        files.len(),
        "{name} is cut into {} piece(s) and was given {} file name(s), so the two partitioned it \
         differently",
        parts.len(),
        files.len()
    );
    parts
        .iter()
        .zip(files)
        .map(|(part, file)| {
            body_label(explosion, &part.cells, pitch, top, format!("{name}\n{file}"), LABEL_INK)
        })
        .collect()
}

/// The inserts as solid bodies in the drawer's own millimetre coordinates, each
/// displaced with the piece of its bin that it stands on.
///
/// Built from the same `SubbinSpec` the export built from, so the window shows
/// the body the file holds. An insert is never cut, so it moves whole: it takes
/// the shift of the band its own centre falls in, which is the band of the piece
/// it would be lifted out with.
fn subbin_vertices(
    subbins: &[optimize::PlacedSubbin],
    params: &Params,
    gaps: bool,
) -> (Vec<f32>, Vec<BinError>) {
    let mut out = Vec::new();
    let mut errors = Vec::new();
    for insert in subbins {
        match catch(|| build_subbin(&insert.spec)) {
            Ok(solid) => {
                let mut verts = shaded(&tessellate(&solid, PREVIEW_RES), SUBBIN_AMBER, false);
                displace(&mut verts, subbin_shift(insert, params, gaps));
                out.extend(verts);
            }
            Err(msg) => errors.push(BinError {
                bin: insert.bin,
                msg,
            }),
        }
    }
    (out, errors)
}

/// Where an insert is drawn relative to where it is built: nowhere at all with
/// the gaps closed, and lifted clear of the bin's rim with them open.
///
/// **An insert takes no band displacement**, which is the one thing separating
/// it from an object box. A compartment may straddle a cut -- the AAA batteries
/// of `examples/ikea-alex-drawer-1.toml` span the seam of their own bin -- and
/// there the two halves of the compartment open by `SPLIT_APART_MM` while the
/// insert, being one printed body, cannot. Giving it the band its centre falls
/// in put it half a gap into one half of a compartment that had just been pulled
/// apart around it, which reads as a part that does not fit. Lifting it out
/// instead is what the gaps mean for a body that is *assembled* rather than cut:
/// with them closed it sits exactly in its compartment, which is the view that
/// answers whether it fits, and with them open it stands above the compartment
/// it belongs to.
fn subbin_shift(insert: &optimize::PlacedSubbin, params: &Params, gaps: bool) -> Vec3 {
    if !gaps {
        return Vec3::ZERO;
    }
    let clear = params.total_height() - insert.spec.z + f64::from(explode::SPLIT_APART_MM);
    Vec3::new(0.0, 0.0, clear.max(0.0) as f32)
}

/// Every item in the scene named where it stands: each bin, the baseplate under
/// it, each insert standing in a compartment, and each object the packer placed.
///
/// An insert is one body and one file, so it is named once, over its own top,
/// with its file name under it -- the two-line form every exported body carries.
///
/// **A body is named once per file it becomes.** A piece is what gets written,
/// so a bin cut into six for the bed carries six labels, each over its own
/// piece and each reading the body's name above the file that piece would be
/// exported as -- which is the question the exploded view exists to answer, the
/// pieces being laid out in front of a reader about to send one to a slicer.
/// A body that becomes **no** file is named once over the whole of it, with no
/// second line: that is the bin editor, which exports through a dialog and has
/// no names to show, and it must not start reading "bin" six times.
///
/// An **object** is not exported and stays named once whatever it crosses: an
/// object spanning a cut is one object, and one made of several boxes is named
/// once over all of them.
///
/// Each label rides the band its own point falls in, so it stands on the piece
/// rather than in a gap the explosion opened.
///
/// A bin is named by `bin_names` where the fit gave it a name -- in `bins` mode
/// a bin *is* an object, and "bin 3" says nothing the viewer wants -- and
/// falls back to "bin" or "bin N" for a run that named none. That name is not
/// the file's stem: the same body reads "AAA batteries + caliper box" and
/// writes `gridfinity-bin-1`, which is why `bin_files` is carried from the
/// exporter rather than rebuilt here.
///
/// An object is named in red where it does not clear the cavity -- the same
/// thing the box's red rim says, in words. Object labels come first because
/// `paint_labels` keeps the first label in each cell of its grid: where an
/// object's name and the body it sits in would collide, the object wins, being
/// the thing the viewer cannot identify by looking.
fn scene_labels(
    params: &Params,
    gaps: bool,
    boxes: &[optimize::ObjectBox],
    subbins: &[optimize::PlacedSubbin],
    plate: Option<&Params>,
    bin_names: &[String],
    bin_files: &[Vec<String>],
    plate_files: &[String],
) -> Vec<wireframe::Label> {
    let pitch = params.pitch;
    let bins: Vec<(usize, &LogicalBin)> = params
        .bins
        .iter()
        .enumerate()
        .filter(|(_, b)| !b.cells.is_empty())
        .collect();
    let mut out = Vec::new();
    let mut instances: Vec<(usize, &optimize::ObjectBox, KernelVec3, KernelVec3)> = Vec::new();
    for b in boxes {
        match instances.iter_mut().find(|(i, ..)| *i == b.instance) {
            Some((_, _, min, max)) => {
                *min = min.min(b.min);
                *max = max.max(b.max);
            }
            None => instances.push((b.instance, b, b.min, b.max)),
        }
    }
    for (_, b, min, max) in &instances {
        let Some(bin) = params.bins.get(b.bin) else {
            continue;
        };
        let (x, y) = ((min.x + max.x) / 2.0, (min.y + max.y) / 2.0);
        let shift = explode(Explosion::of(bin, pitch), gaps).shift_at(x, y);
        out.push(wireframe::Label {
            at: KernelVec3::new(x + f64::from(shift.x), y + f64::from(shift.y), max.z),
            text: b.name.clone(),
            color: if b.fits { LABEL_INK } else { LABEL_BAD },
        });
    }
    for insert in subbins {
        let (x, y) = (
            insert.spec.x + insert.spec.outer_width / 2.0,
            insert.spec.y + insert.spec.outer_depth / 2.0,
        );
        let shift = subbin_shift(insert, params, gaps);
        out.push(wireframe::Label {
            at: KernelVec3::new(x, y, insert.spec.top() + f64::from(shift.z)),
            text: format!("{}
{}", insert.label, insert.file),
            color: LABEL_INK,
        });
    }
    for (ord, (index, bin)) in bins.iter().enumerate() {
        let name = match bin_names.get(*index) {
            Some(name) => name.clone(),
            None if bins.len() == 1 => "bin".to_string(),
            None => format!("bin {}", ord + 1),
        };
        out.extend(body_labels(
            &explode(Explosion::of(bin, pitch), gaps),
            &bin.cells,
            &bin.split_lines,
            pitch,
            params.total_height(),
            &name,
            bin_files.get(*index).map_or(&[][..], Vec::as_slice),
        ));
    }
    if let Some(plate) = plate {
        let cells = plate.all_cells();
        if !cells.is_empty() {
            let mut splits: Vec<SplitLine> = Vec::new();
            for bin in &plate.bins {
                for line in &bin.split_lines {
                    if !splits.contains(line) {
                        splits.push(*line);
                    }
                }
            }
            out.extend(body_labels(
                &explode(Explosion::new(&cells, &splits, plate.pitch), gaps),
                &cells,
                &splits,
                plate.pitch,
                gridfinity::PEG_HEIGHT,
                "baseplate",
                plate_files,
            ));
        }
    }
    out
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

/// Every bin of `p` as preview vertices, with the split bodies standing apart
/// when `gaps` and abutting as printed when not -- the viewport's *Show gaps*
/// toggle, and the only thing the window varies about the scene.
fn build_scene(p: &Params, gaps: bool) -> (Vec<f32>, Vec<BinError>) {
    build_scene_with(p, build_bin, gaps)
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
    gaps: bool,
) -> (Vec<f32>, Vec<BinError>) {
    let mut verts = Vec::new();
    let mut errors = Vec::new();
    if p.mode != Mode::Bin {
        match try_whole(p).and_then(|s| plate_vertices(p, &s, gaps)) {
            Ok(v) => verts = v,
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
        match build(p, bin).and_then(|solid| bin_vertices(bin, p.pitch, &solid, gaps)) {
            Ok(piece_verts) => verts.extend(piece_verts),
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
    bin_names: Vec<String>,
    bin_files: Vec<Vec<String>>,
    plate_files: Vec<String>,
    object_boxes: Vec<optimize::ObjectBox>,
    show_object_boxes: bool,
    subbins: Vec<optimize::PlacedSubbin>,
    show_subbins: bool,
    plate: Option<Params>,
    show_plate: bool,
    show_gaps: bool,
    dirty: bool,
    program_dirty: bool,
    tri_count: usize,
    status: String,
    errors: Vec<BinError>,
}


impl App {
    fn new(cc: &eframe::CreationContext<'_>, initial: Option<optimize::View>) -> App {
        theme::apply(&cc.egui_ctx);
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
        let (params, object_boxes, subbins, plate, bin_names, bin_files, plate_files) =
            match initial {
                Some(view) => (
                    view.params,
                    view.boxes,
                    view.subbins,
                    view.plate,
                    view.bin_names,
                    view.bin_files,
                    view.plate_files,
                ),
                None => (
                    Params::default(),
                    Vec::new(),
                    Vec::new(),
                    None,
                    Vec::new(),
                    Vec::new(),
                    Vec::new(),
                ),
            };
        let mut app = App {
            show_object_boxes: !object_boxes.is_empty(),
            show_subbins: !subbins.is_empty(),
            subbins,
            bin_names,
            bin_files,
            plate_files,
            object_boxes,
            show_plate: plate.is_some(),
            show_gaps: true,
            plate,
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
        let (mut verts, errors) = match &dbg_solid {
            Some(s) => match catch(|| Ok(debug_view(&self.debugger, s))) {
                Ok((v, w)) => {
                    wf = w;
                    (v, Vec::new())
                }
                Err(msg) => (Vec::new(), vec![BinError { bin: 0, msg }]),
            },
            None => build_scene(&self.params, self.show_gaps),
        };
        self.errors = errors;
        self.tri_count = verts.len() / (3 * MESH_STRIDE);

        if dbg_solid.is_none() && self.debugger.is_shown() {
            for (profile, plane) in self.debugger.sketch_planes() {
                wf.add_sketch(profile, plane, PREVIEW_RES, wireframe::SKETCH_BLACK);
            }
        }
        if self.show_object_boxes && !self.params.bins.is_empty() {
            verts.extend(object_box_vertices(&self.object_boxes, &self.params, self.show_gaps));
        }
        if dbg_solid.is_none() && self.show_subbins {
            let (insert_verts, insert_errors) =
                subbin_vertices(&self.subbins, &self.params, self.show_gaps);
            verts.extend(insert_verts);
            self.errors.extend(insert_errors);
        }
        if dbg_solid.is_none() && self.show_plate {
            if let Some(plate) = &self.plate {
                let (plate_verts, plate_errors) = build_scene(plate, self.show_gaps);
                verts.extend(plate_verts);
                self.errors.extend(plate_errors);
            }
        }
        let (min, max) = vert_bounds(&verts);
        self.camera.target = (min + max) * 0.5;
        if reframe {
            self.camera.frame(min, max);
        }
        self.labels = wf.labels;
        if dbg_solid.is_none() {
            let boxes: &[optimize::ObjectBox] =
                if self.show_object_boxes { &self.object_boxes } else { &[] };
            let plate = if self.show_plate { self.plate.as_ref() } else { None };
            self.labels.extend(scene_labels(
                &self.params,
                self.show_gaps,
                boxes,
                if self.show_subbins { &self.subbins } else { &[] },
                plate,
                &self.bin_names,
                &self.bin_files,
                &self.plate_files,
            ));
        }

        let mut r = self.renderer.lock().unwrap();
        r.upload(&self.gpu.device, &self.gpu.queue, &verts);
        r.upload_lines(&self.gpu.device, &self.gpu.queue, &wf.lines);
        drop(r);
        self.dirty = false;
    }



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

/// Which workspace the window is showing, and so what the right panel holds.
///
/// The web app's header switches between the bin editor and Project mode and
/// swaps both panels' contents with it; this binary's second workspace is the
/// construction debugger, so the switch is in the same place and does the same
/// thing to the same panel.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Workspace {
    Editor,
    Debugger,
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let mut changed = false;
        let mut dbg_changed = false;

        egui::Panel::top("header")
            .exact_size(theme::HEADER_HEIGHT)
            .show(ui, |ui| self.header(ui));

        egui::Panel::left("sidebar")
            .resizable(true)
            .default_size(theme::SIDEBAR_WIDTH)
            .show(ui, |ui| {
                changed |= sidebar::panel(self, ui);
            });

        egui::Panel::right("settings")
            .resizable(true)
            .default_size(theme::SETTINGS_WIDTH)
            .show(ui, |ui| {
                if self.debugger.is_shown() {
                    egui::ScrollArea::neither().show(ui, |ui| {
                        dbg_changed = self.debugger.panel(ui);
                    });
                } else {
                    changed |= settings::panel(self, ui);
                }
            });

        if changed {
            self.dirty = true;
            self.program_dirty = true;
        }
        if self.dirty || dbg_changed {
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
    /// The header strip: what the app is, which workspace is showing, and the
    /// two exports. `App.tsx`'s `AppShell.Header`, item for item.
    fn header(&mut self, ui: &mut egui::Ui) {
        ui.horizontal_centered(|ui| {
            ui.label(
                egui::RichText::new("gridfinity-expanded")
                    .strong()
                    .size(theme::FONT_SM)
                    .extra_letter_spacing(0.5)
                    .color(theme::TEXT_BRIGHT),
            );
            ui.add_space(12.0);
            let mut workspace = if self.debugger.is_shown() {
                Workspace::Debugger
            } else {
                Workspace::Editor
            };
            if widgets::segmented(
                ui,
                &mut workspace,
                &[(Workspace::Editor, "Bin editor"), (Workspace::Debugger, "Debugger")],
                false,
            ) {
                self.debugger.set_shown(workspace == Workspace::Debugger);
                self.dirty = true;
                self.program_dirty = true;
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
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
                if ui.button("Export pieces…").clicked() {
                    self.export_pieces();
                }
                if ui.button("Export STL…").clicked() {
                    self.export_stl();
                }
            });
        });
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
        self.viewport_tools(ui, rect);
        self.paint_error_banner(ui, rect);
    }

    /// The two buttons over the top-right corner of the 3D view: whether a
    /// split body is shown open, and reframing the camera.
    ///
    /// `ModelViewer`'s `.viewer-tools`, in the same corner and with the same
    /// words. *Show gaps* rebuilds rather than easing a displacement per frame
    /// the way the web viewer's uniform does -- here the pieces are carved on
    /// the CPU, so the toggle is a rebuild and eating one is what it costs.
    fn viewport_tools(&mut self, ui: &mut egui::Ui, rect: egui::Rect) {
        let mut child = ui.new_child(
            egui::UiBuilder::new()
                .max_rect(rect.shrink(8.0))
                .layout(egui::Layout::right_to_left(egui::Align::TOP)),
        );
        if child.button("Reset view").clicked() {
            self.regenerate(true);
        }
        let label = if self.show_gaps { "Close up" } else { "Show gaps" };
        if child
            .add(egui::Button::new(label).selected(self.show_gaps))
            .on_hover_text("Stand every cut piece off its neighbours, or abut them as printed.")
            .clicked()
        {
            self.show_gaps = !self.show_gaps;
            self.dirty = true;
        }
    }

    fn paint_error_banner(&self, ui: &mut egui::Ui, rect: egui::Rect) {
        if self.errors.is_empty() {
            return;
        }
        let red = theme::RED;
        let mut child = ui.new_child(
            egui::UiBuilder::new()
                .max_rect(rect.shrink(12.0))
                .layout(egui::Layout::bottom_up(egui::Align::LEFT)),
        );
        egui::Frame::popup(ui.style())
            .fill(theme::RED.gamma_multiply(0.12))
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
                    ui.label(egui::RichText::new(&e.msg).color(theme::RED_PALE));
                }
                ui.label(
                    egui::RichText::new("Shown as a plain block; the rest of the layout is unaffected.")
                        .small()
                        .color(theme::TEXT_DIMMED),
                );
            });
    }

    /// Every label of the scene painted over the viewport at the point it names,
    /// one per `CELL`-sized square of the screen and the first to claim one
    /// keeping it -- which is why `scene_labels` pushes objects before bodies.
    ///
    /// A label's text may carry a newline and a body's does: `Painter::text`
    /// lays the lines out itself, centred on the same point. Two lines of the
    /// 9 pt face stand about 22 px, inside one `CELL`, so the collision grid
    /// still admits one label per square and needs no measuring of its own.
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
            painter.text(p, egui::Align2::CENTER_CENTER, label.text.clone(), font.clone(), color);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::explode::SPLIT_APART_MM;
    use gridfinity_brep::math::Vec3 as KernelVec3;
    use gridfinity_model::layout::{Axis, GridCell, SplitLine};

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
        let (verts, errors) = build_scene_with(&p, always_fails, true);
        assert_eq!(errors.len(), 1, "the one bad bin should be reported once");
        assert_eq!(errors[0].bin, 0);
        assert!(!errors[0].msg.is_empty(), "the failure needs a message to show");
        let (good, bad) = flags(&verts);
        assert!(bad > 0, "the failed bin needs placeholder geometry to glow");
        assert_eq!(good, 0, "nothing else built, so nothing should be unflagged");
    }

    #[test]
    fn a_valid_layout_reports_nothing_and_flags_nothing() {
        let (verts, errors) = build_scene(&Params::default(), true);
        assert!(errors.is_empty(), "default bin should build: {:?}", errors[0].msg);
        let (good, bad) = flags(&verts);
        assert!(good > 0);
        assert_eq!(bad, 0, "a healthy bin must not be flagged");
    }

    #[test]
    fn a_failed_bin_does_not_take_its_neighbours_with_it() {
        let p = bins_at(&[GridCell { x: 0, y: 0 }, GridCell { x: 4, y: 0 }]);
        let (verts, errors) = build_scene_with(&p, always_fails, true);
        assert_eq!(errors.len(), 2, "both bins were refused");
        assert_eq!(errors.iter().map(|e| e.bin).collect::<Vec<_>>(), vec![0, 1]);

        let (ok_verts, ok_errors) = build_scene(&p, true);
        assert!(ok_errors.is_empty());
        assert!(ok_verts.len() > verts.len(), "real geometry beats placeholders");
    }

    /// A bin of `cells` in one piece, and the same bin cut on `splits`.
    fn split_bin(cells: &[GridCell], splits: &[SplitLine]) -> Params {
        Params {
            bins: vec![LogicalBin {
                cells: cells.to_vec(),
                split_lines: splits.to_vec(),
                ..Default::default()
            }],
            height_units: 1,
            ..Params::default()
        }
    }

    /// An insert is one printed body, so the gaps lift it out of its compartment
    /// rather than moving it with a band.
    ///
    /// The regression this pins is a compartment that **straddles a cut**, which
    /// the AAA batteries of `examples/ikea-alex-drawer-1.toml` did: the two
    /// halves of the compartment open by `SPLIT_APART_MM` while the insert,
    /// being uncut, can only take one band's shift -- so it read as a part
    /// standing half a gap into a compartment that had been pulled apart around
    /// it. The displacement is along +Z and nothing else, and it is zero with
    /// the gaps closed, which is the view that answers whether the insert fits.
    #[test]
    fn an_insert_is_lifted_out_rather_than_moved_with_a_band() {
        let cells: Vec<GridCell> = (0..4)
            .flat_map(|x| (0..4).map(move |y| GridCell { x, y }))
            .collect();
        let params = split_bin(&cells, &[SplitLine { axis: Axis::Y, index: 2 }]);
        let insert = optimize::PlacedSubbin {
            label: "battery".to_string(),
            file: "gridfinity-subbin.stl".to_string(),
            bin: 0,
            spec: gridfinity_model::subbin::SubbinSpec {
                x: 20.0,
                y: 40.0,
                z: 8.2,
                outer_width: 40.0,
                outer_depth: 80.0,
                interior_width: 37.6,
                interior_depth: 77.6,
                interior_height: 10.0,
                floor: 2.5,
                corner_r: 2.6,
                interior_corner_r: 2.5,
                chamfer: 2.5,
            },
        };
        assert!(
            insert.spec.y < 84.0 && insert.spec.y + insert.spec.outer_depth > 84.0,
            "the fixture insert must straddle the cut at y = 2 cells, or it cannot see the defect"
        );

        let closed = subbin_shift(&insert, &params, false);
        assert_eq!(
            closed,
            Vec3::ZERO,
            "with the gaps closed an insert sits exactly in its compartment"
        );
        let open = subbin_shift(&insert, &params, true);
        assert!(
            open.x == 0.0 && open.y == 0.0,
            "an insert takes no band displacement, but it moved ({}, {}) across the bin",
            open.x,
            open.y
        );
        assert!(
            (open.z - (params.total_height() - insert.spec.z + f64::from(SPLIT_APART_MM)) as f32)
                .abs()
                < 1e-6,
            "an insert is lifted clear of the bin's rim, but it rose {} mm",
            open.z
        );

        let labels = scene_labels(&params, true, &[], &[insert], None, &[], &[], &[]);
        let label = labels
            .iter()
            .find(|l| l.text.starts_with("battery"))
            .expect("the insert is named");
        assert!(
            (label.at.z - (8.2 + 12.5 + f64::from(open.z))).abs() < 1e-6,
            "the insert's label rides with it, but it sits at z = {}",
            label.at.z
        );
    }

    #[test]
    fn a_split_bin_previews_as_pieces_a_gap_apart() {
        let cells = [GridCell { x: 0, y: 0 }, GridCell { x: 1, y: 0 }];
        let (whole, errors) = build_scene(&split_bin(&cells, &[]), true);
        assert!(errors.is_empty(), "the uncut bin must build");
        let (cut, errors) =
            build_scene(&split_bin(&cells, &[SplitLine { axis: Axis::X, index: 1 }]), true);
        assert!(errors.is_empty(), "the cut bin must build");

        let (whole_min, whole_max) = vert_bounds(&whole);
        let (cut_min, cut_max) = vert_bounds(&cut);
        assert!(
            ((cut_max.x - cut_min.x) - (whole_max.x - whole_min.x) - SPLIT_APART_MM).abs() < 1e-3,
            "one cut opens by one {SPLIT_APART_MM} mm gap, so the bin does not go from {} to {}",
            whole_max.x - whole_min.x,
            cut_max.x - cut_min.x
        );
        assert!(
            (cut_min.y - whole_min.y).abs() < 1e-3 && (cut_max.y - whole_max.y).abs() < 1e-3,
            "a bin cut across x moves nothing in y"
        );
    }

    fn boxed(
        name: &str,
        instance: usize,
        min: KernelVec3,
        max: KernelVec3,
        fits: bool,
    ) -> optimize::ObjectBox {
        optimize::ObjectBox { name: name.to_string(), instance, bin: 0, min, max, fits }
    }

    /// Everything `--view` puts on screen says what it is: both bodies, and
    /// every object packed into them. A white box and a grey plate are
    /// otherwise unidentifiable, and the point of the view is matching what is
    /// on screen to what the report names.
    #[test]
    fn every_item_in_the_view_is_labelled() {
        let cells = [GridCell { x: 0, y: 0 }, GridCell { x: 1, y: 0 }];
        let params = split_bin(&cells, &[SplitLine { axis: Axis::X, index: 1 }]);
        let plate = Params { mode: Mode::Baseplate, ..split_bin(&cells, &[]) };
        let boxes = [
            boxed("socket set", 0, KernelVec3::new(2.0, 2.0, 0.0), KernelVec3::new(30.0, 30.0, 5.0), true),
            boxed("tape measure", 1, KernelVec3::new(50.0, 2.0, 0.0), KernelVec3::new(80.0, 30.0, 400.0), false),
        ];
        let labels = scene_labels(&params, true, &boxes, &[], Some(&plate), &[], &[], &[]);
        let text: Vec<&str> = labels.iter().map(|l| l.text.as_str()).collect();
        assert_eq!(
            text,
            vec!["socket set", "tape measure", "bin", "baseplate"],
            "one label per item, objects first so they win the collision grid"
        );

        let too_tall = labels
            .iter()
            .find(|l| l.text == "tape measure")
            .expect("the object that does not fit is labelled too");
        assert_eq!(
            too_tall.color, LABEL_BAD,
            "an object that does not clear the cavity says so in its label as well as its rim"
        );
        assert!(
            labels.iter().filter(|l| l.text == "socket set").all(|l| l.color == LABEL_INK),
            "an object that fits is labelled plainly"
        );
        assert!(
            (too_tall.at.z - 400.0).abs() < 1e-9,
            "a box's label sits on top of it, not inside it: {}",
            too_tall.at.z
        );
        let bin = labels.iter().find(|l| l.text == "bin").expect("the bin is labelled");
        assert!(
            (bin.at.z - params.total_height()).abs() < 1e-9,
            "the bin is labelled on its rim, not at {}",
            bin.at.z
        );
        let plate_label = labels.iter().find(|l| l.text == "baseplate").expect("so is the plate");
        assert!(
            (plate_label.at.z - gridfinity::PEG_HEIGHT).abs() < 1e-9,
            "the plate is labelled at its own height, under the bin's"
        );
    }

    /// A body that becomes no file is named once however many pieces it is cut
    /// into -- the bin editor, which exports through a dialog and has no file
    /// names to show. And an **object** is named once whatever it crosses, in
    /// this view and in every other: a cut is not a new object, and an object of
    /// several boxes is one object.
    #[test]
    fn a_body_that_becomes_no_file_is_labelled_once_however_it_is_cut() {
        let cells: Vec<GridCell> = (0..4)
            .flat_map(|x| (0..2).map(move |y| GridCell { x, y }))
            .collect();
        let splits = [
            SplitLine { axis: Axis::X, index: 1 },
            SplitLine { axis: Axis::X, index: 3 },
            SplitLine { axis: Axis::Y, index: 1 },
        ];
        let params = split_bin(&cells, &splits);
        let pitch = gridfinity::GRID_PITCH;
        assert_eq!(
            Explosion::of(&params.bins[0], pitch).pieces().len(),
            6,
            "the fixture is a bin in six pieces"
        );
        let l_shaped = [
            boxed("bracket", 0, KernelVec3::new(10.0, 10.0, 0.0), KernelVec3::new(140.0, 30.0, 5.0), true),
            boxed("bracket", 0, KernelVec3::new(10.0, 30.0, 0.0), KernelVec3::new(40.0, 60.0, 5.0), true),
        ];
        let labels = scene_labels(&params, true, &l_shaped, &[], None, &[], &[], &[]);
        let text: Vec<&str> = labels.iter().map(|l| l.text.as_str()).collect();
        assert_eq!(
            text,
            vec!["bracket", "bin"],
            "six pieces that are written nowhere are one bin, and a two-box object crossing \
             three cuts is one object"
        );
        assert!(
            (labels[0].at.x - 75.0).abs() < f64::from(SPLIT_APART_MM) + 1e-9,
            "the object is named over the whole of it, near the centre of its own boxes, not {}",
            labels[0].at.x
        );
    }

    /// A piece is what gets written, so a body carries one label per file it
    /// becomes: the body's name over that piece's own file name, each label on
    /// its own piece rather than all six at the body's centre.
    ///
    /// This is what the exploded view is for -- the pieces are laid out and the
    /// reader is about to send one of them to a slicer -- so the label has to
    /// say which file each grey body is.
    #[test]
    fn a_body_is_named_once_per_file_it_becomes() {
        let cells: Vec<GridCell> = (0..4).map(|x| GridCell { x, y: 0 }).collect();
        let splits = [SplitLine { axis: Axis::X, index: 2 }];
        let params = split_bin(&cells, &splits);
        let files = vec![vec![
            "gridfinity-bin-2-piece-1-of-2.stl".to_string(),
            "gridfinity-bin-2-piece-2-of-2.stl".to_string(),
        ]];
        let labels = scene_labels(&params, true, &[], &[], None, &["bin 2".to_string()], &files, &[]);
        let text: Vec<&str> = labels.iter().map(|l| l.text.as_str()).collect();
        assert_eq!(
            text,
            vec![
                "bin 2\ngridfinity-bin-2-piece-1-of-2.stl",
                "bin 2\ngridfinity-bin-2-piece-2-of-2.stl"
            ],
            "each piece says what body it is and what file it becomes"
        );
        assert!(
            labels[0].at.x < labels[1].at.x,
            "each label stands on its own piece, so the two are apart along the cut: {} and {}",
            labels[0].at.x,
            labels[1].at.x
        );
        assert!(
            (labels[1].at.x - labels[0].at.x - 2.0 * gridfinity::GRID_PITCH
                - f64::from(SPLIT_APART_MM))
            .abs()
                < 1e-9,
            "and each rides its own exploded band, so they stand one gap further apart than the \
             pieces themselves"
        );

        let whole = split_bin(&cells, &[]);
        let one = scene_labels(
            &whole,
            true,
            &[],
            &[],
            None,
            &["bin 2".to_string()],
            &[vec!["gridfinity-bin-2.stl".to_string()]],
            &[],
        );
        assert_eq!(
            one.iter().map(|l| l.text.as_str()).collect::<Vec<_>>(),
            vec!["bin 2\ngridfinity-bin-2.stl"],
            "an uncut body becomes one file and is named once"
        );
    }

    /// The baseplate is written per piece like the bin, and cut on lines
    /// staggered off the bin's, so each of its pieces stands somewhere the bin's
    /// do not and is worth naming separately.
    #[test]
    fn the_baseplate_names_each_of_its_own_pieces() {
        let cells: Vec<GridCell> = (0..3).map(|x| GridCell { x, y: 0 }).collect();
        let params = split_bin(&cells, &[SplitLine { axis: Axis::X, index: 2 }]);
        let plate = Params {
            mode: Mode::Baseplate,
            ..split_bin(&cells, &[SplitLine { axis: Axis::X, index: 1 }])
        };
        let plate_files = vec![
            "gridfinity-baseplate-piece-1-of-2.stl".to_string(),
            "gridfinity-baseplate-piece-2-of-2.stl".to_string(),
        ];
        let labels = scene_labels(&params, true, &[], &[], Some(&plate), &[], &[], &plate_files);
        let plate_text: Vec<&str> = labels
            .iter()
            .map(|l| l.text.as_str())
            .filter(|t| t.starts_with("baseplate"))
            .collect();
        assert_eq!(
            plate_text,
            vec![
                "baseplate\ngridfinity-baseplate-piece-1-of-2.stl",
                "baseplate\ngridfinity-baseplate-piece-2-of-2.stl"
            ]
        );
    }

    /// A whole item's label rides the band its own centre stands in, so it is
    /// drawn on the item rather than in the gap a cut opened beside it.
    #[test]
    fn a_labels_position_follows_the_piece_its_centre_stands_on() {
        let cells: Vec<GridCell> = (0..3).map(|x| GridCell { x, y: 0 }).collect();
        let pitch = gridfinity::GRID_PITCH;
        let centred = split_bin(&cells, &[SplitLine { axis: Axis::X, index: 1 }]);
        let label = scene_labels(&centred, true, &[], &[], None, &[], &[], &[]);
        assert_eq!(label.len(), 1);
        assert!(
            (label[0].at.x - (1.5 * pitch + f64::from(SPLIT_APART_MM) / 2.0)).abs() < 1e-9,
            "the bin's centre stands on the second band, so its name moves with it, not to {}",
            label[0].at.x
        );

        let other = split_bin(&cells, &[SplitLine { axis: Axis::X, index: 2 }]);
        let label = scene_labels(&other, true, &[], &[], None, &[], &[], &[]);
        assert!(
            (label[0].at.x - (1.5 * pitch - f64::from(SPLIT_APART_MM) / 2.0)).abs() < 1e-9,
            "cut later and the same centre stands on the first band instead, not {}",
            label[0].at.x
        );
    }

    /// The plate the bin drops into is cut and exploded exactly as the bin is,
    /// on its own lines. Without it a split baseplate previewed as one solid,
    /// which is the picture of an assembly that does not come apart.
    #[test]
    fn a_split_baseplate_previews_as_pieces_a_gap_apart() {
        let cells = [GridCell { x: 0, y: 0 }, GridCell { x: 1, y: 0 }];
        let plate = |splits: &[SplitLine]| Params {
            mode: Mode::Baseplate,
            ..split_bin(&cells, splits)
        };
        let (whole, errors) = build_scene(&plate(&[]), true);
        assert!(errors.is_empty(), "the uncut plate must build");
        let (cut, errors) = build_scene(&plate(&[SplitLine { axis: Axis::X, index: 1 }]), true);
        assert!(errors.is_empty(), "the cut plate must build");

        let (whole_min, whole_max) = vert_bounds(&whole);
        let (cut_min, cut_max) = vert_bounds(&cut);
        assert!(
            ((cut_max.x - cut_min.x) - (whole_max.x - whole_min.x) - SPLIT_APART_MM).abs() < 1e-3,
            "one cut opens the plate by one {SPLIT_APART_MM} mm gap, not from {} to {}",
            whole_max.x - whole_min.x,
            cut_max.x - cut_min.x
        );
        assert!(
            (cut_min.y - whole_min.y).abs() < 1e-3 && (cut_max.y - whole_max.y).abs() < 1e-3,
            "a plate cut across x moves nothing in y"
        );
    }

    /// The two bodies explode along their **own** bands, which is the whole
    /// point of staggering the seams: a plate cut where the bin is not opens its
    /// gap somewhere else, so a piece of each spans a seam of the other.
    #[test]
    fn the_plate_and_the_bin_open_their_gaps_in_different_places() {
        let cells: Vec<GridCell> = (0..4).map(|x| GridCell { x, y: 0 }).collect();
        let bin_line = SplitLine { axis: Axis::X, index: 2 };
        let plate_line = SplitLine { axis: Axis::X, index: 1 };
        let bin = Explosion::new(&cells, &[bin_line], gridfinity::GRID_PITCH);
        let plate = Explosion::new(&cells, &[plate_line], gridfinity::GRID_PITCH);
        for (body, line) in [(&bin, bin_line), (&plate, plate_line)] {
            let moved: Vec<f32> = body.pieces().iter().map(|p| body.shift(p.col, p.row).x).collect();
            assert_eq!(
                moved,
                vec![-SPLIT_APART_MM / 2.0, SPLIT_APART_MM / 2.0],
                "a body cut at {line:?} opens into two bands"
            );
        }
        let cell_two = |body: &Explosion| {
            let piece = body
                .pieces()
                .iter()
                .find(|p| p.cells.iter().any(|c| c.x == 2))
                .expect("cell 2 lies in some piece");
            (piece.cells.len(), body.shift(piece.col, piece.row).x)
        };
        assert_eq!(cell_two(&bin), (2, SPLIT_APART_MM / 2.0));
        assert_eq!(
            cell_two(&plate),
            (3, SPLIT_APART_MM / 2.0),
            "the plate piece holding cell 2 also holds cells 1 and 3, so it spans the bin's seam"
        );
    }

    #[test]
    fn an_object_box_is_cut_and_moved_with_the_piece_it_lies_in() {
        let cells = [GridCell { x: 0, y: 0 }, GridCell { x: 1, y: 0 }];
        let params = split_bin(&cells, &[SplitLine { axis: Axis::X, index: 1 }]);
        let pitch = gridfinity::GRID_PITCH;
        let across = optimize::ObjectBox {
            name: "across the cut".to_string(),
            instance: 0,
            bin: 0,
            min: KernelVec3::new(0.25 * pitch, 0.25 * pitch, 0.0),
            max: KernelVec3::new(1.75 * pitch, 0.75 * pitch, 5.0),
            fits: true,
        };
        let verts = object_box_vertices(std::slice::from_ref(&across), &params, true);
        let (min, max) = vert_bounds(&verts);
        assert!(
            ((max.x - min.x) - (across.max.x - across.min.x) as f32 - SPLIT_APART_MM).abs() < 1e-3,
            "the two halves of the box open by the same gap the bin does, not {} from {}",
            max.x - min.x,
            across.max.x - across.min.x
        );
        assert!(
            (max.z - across.max.z as f32).abs() < 1e-3 && (min.z - across.min.z as f32).abs() < 1e-3,
            "a cut is vertical, so it takes nothing off the box's height"
        );
        let (good, bad) = flags(&verts);
        assert!(good > 0 && bad == 0, "an object that fits is drawn plain");
    }

    #[test]
    fn an_object_that_does_not_fit_is_flagged_rather_than_hidden() {
        let params = split_bin(&[GridCell { x: 0, y: 0 }], &[]);
        let tall = optimize::ObjectBox {
            name: "too tall".to_string(),
            instance: 0,
            bin: 0,
            min: KernelVec3::new(5.0, 5.0, 0.0),
            max: KernelVec3::new(30.0, 30.0, 400.0),
            fits: false,
        };
        let verts = object_box_vertices(std::slice::from_ref(&tall), &params, true);
        let (_, max) = vert_bounds(&verts);
        assert!((max.z - 400.0).abs() < 1e-3, "the box stands its full height, not {}", max.z);
        let (good, bad) = flags(&verts);
        assert!(bad > 0 && good == 0, "every vertex of a box that does not fit is flagged");
    }

    #[test]
    fn the_placeholder_sits_on_the_failed_bin_footprint() {
        let p = bins_at(&[GridCell { x: 2, y: 1 }]);
        let (verts, _) = build_scene_with(&p, always_fails, true);
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
