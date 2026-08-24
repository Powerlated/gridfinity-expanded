//! One fuzz path, driven by an `Options` object.
//!
//! Every `#[test]` below is the same generate -> check -> group -> shrink ->
//! report pipeline aimed at a different corner of the model by the options it
//! passes. There is exactly one generator, one checker, one shrinker and one
//! repro printer, so an invariant added here is immediately enforced by every
//! profile that enables the feature it covers.

use gridfinity_cad::gridfinity::{
    self, BinSlope, GRID_PITCH, InnerWall, LogicalBin, Mode, Params, SlopeDir, rect_cells,
};
use gridfinity_cad::kernel::geom::Surface;
use gridfinity_cad::kernel::mesh::Mesh;
use gridfinity_cad::kernel::program::BlendReport;
use gridfinity_cad::kernel::tess::tessellate;
use gridfinity_cad::layout::{
    Axis, GridCell, GridEdge, Orientation, SplitLine, effective_walls, internal_edges,
    partition_cells, perimeter_edges,
};
use gridfinity_cad::{Solid, audit, tessellation_leaks};
use rayon::prelude::*;
use std::collections::{BTreeMap, HashSet};
use std::panic::{AssertUnwindSafe, PanicHookInfo, catch_unwind};
use std::sync::Mutex;

struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Rng {
        Rng(seed | 1)
    }
    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    fn below(&mut self, n: u32) -> u32 {
        (self.next_u64() >> 32) as u32 % n.max(1)
    }
    fn range(&mut self, lo: f32, hi: f32) -> f32 {
        let t = (self.next_u64() >> 40) as f32 / (1u64 << 24) as f32;
        lo + (hi - lo) * t
    }
    fn quantised(&mut self, lo: f32, hi: f32, step: f32) -> f32 {
        (self.range(lo, hi) / step).round() * step
    }
    fn chance(&mut self, num: u32, den: u32) -> bool {
        self.below(den) < num
    }
    fn pick<T: Copy>(&mut self, xs: &[T]) -> T {
        xs[self.below(xs.len() as u32) as usize]
    }
}

// ---------------------------------------------------------------------------
// Options: what a profile generates, and what it insists on.
// ---------------------------------------------------------------------------

/// Where a case's cells come from.
#[derive(Clone, Copy, PartialEq)]
enum Shape {
    /// Always this rectangle -- the shape stays out of the way so the feature
    /// under test is what varies.
    Fixed(u32, u32),
    /// A random small rectangle. No reentrant corner, so every perimeter run
    /// between two convex corners is straight.
    Rect,
    /// A small rectangle, occasionally with one cell knocked out -- which is
    /// what puts a reentrant corner in the outline.
    SmallRect,
    /// A random connected polyomino within `SHAPE_EXTENT`.
    Polyomino,
}

/// What kind of free-form inner walls a case gets.
///
/// `Tidy` is the shape the editor and the Projects packer emit -- axis
/// aligned, spanning the bin, a printable thickness -- and the model rounds
/// every one of those cleanly. `Freeform` puts a wall at any angle and any
/// offset, which routinely leaves a sliver too thin to carry the floor fillet.
///
/// That difference used to decide whether a refused blend counted as a defect,
/// through a `require_blends` opt-in. It no longer does: a fillet that does not
/// land is an error on every profile (`FILLET_FAILED`), so what the split now
/// decides is only how *often* the model is asked for something it currently
/// cannot do.
#[derive(Clone, Copy, PartialEq)]
enum Walls {
    None,
    /// Inclusive count range of axis-aligned spanning walls.
    Tidy(u32, u32),
    /// Inclusive count range of arbitrary walls.
    Freeform(u32, u32),
}

impl Walls {
    fn count(self, rng: &mut Rng) -> u32 {
        let (lo, hi) = match self {
            Walls::None => return 0,
            Walls::Tidy(lo, hi) | Walls::Freeform(lo, hi) => (lo, hi),
        };
        lo + rng.below(hi.saturating_sub(lo) + 1)
    }
}

/// How a case's printable pieces are derived.
#[derive(Clone, Copy, PartialEq)]
enum Split {
    /// No split: the whole bin is the only thing checked.
    Whole,
    /// The web app's model -- sever random adjacencies and flood fill, so a
    /// piece is any connected polyomino.
    Flood,
    /// The product's own model -- random `SplitLine`s through `partition_cells`.
    Lines,
}

/// How much of a bin's perimeter wall is taken away.
///
/// The share matters on its own, not just whether there is an opening at all.
/// One opening in a rectangle is a single pinch against a straight run; half a
/// complex polyomino's perimeter puts openings either side of a reentrant
/// corner, back to back along one run, and wrapped around a convex corner, all
/// in the same bin.
#[derive(Clone, Copy, PartialEq)]
enum Openings {
    None,
    /// Numerator and denominator of the share of perimeter edges opened.
    Share(u32, u32),
}

#[derive(Clone, Copy)]
struct Options {
    shape: Shape,
    inner_walls: Walls,
    /// How much of the perimeter wall is opened.
    openings: Openings,
    /// Wall off some internal edges (a divider).
    dividers: bool,
    /// Vary height, thicknesses, radii and holes rather than taking defaults.
    vary_params: bool,
    /// Sometimes give the bin a sloped floor.
    slope: bool,
    /// Sometimes build a baseplate instead of a bin.
    baseplate: bool,
    split: Split,
}

impl Default for Options {
    fn default() -> Options {
        Options {
            shape: Shape::Fixed(2, 2),
            inner_walls: Walls::None,
            openings: Openings::None,
            dividers: false,
            vary_params: false,
            slope: false,
            baseplate: false,
            split: Split::Whole,
        }
    }
}

/// One fuzz case. `opts` rides along because the checker and the shrinker have
/// to know which invariants this case was generated to satisfy.
#[derive(Clone)]
struct Case {
    opts: Options,
    params: Params,
    /// The flood-fill pieces, for `Split::Flood` only. The other modes derive
    /// their pieces from the params.
    pieces: Vec<Vec<GridCell>>,
}

// ---------------------------------------------------------------------------
// Generation
// ---------------------------------------------------------------------------

const SHAPE_EXTENT: i32 = 4;
const STEPS: [(i32, i32); 4] = [(1, 0), (-1, 0), (0, 1), (0, -1)];

fn edge_connected(cells: &[GridCell]) -> bool {
    !cells.is_empty() && flood_parts(cells, &[]).len() == 1
}

fn sort_cells(cells: &mut [GridCell]) {
    cells.sort_by_key(|c| (c.y, c.x));
}

fn gen_rect(rng: &mut Rng) -> Vec<GridCell> {
    let (gx, gy) = (rng.below(3) + 1, rng.below(3) + 1);
    rect_cells(gx, gy)
}

fn gen_small_rect(rng: &mut Rng) -> Vec<GridCell> {
    let mut cells = gen_rect(rng);
    if cells.len() > 2 && rng.chance(1, 3) {
        let victim = rng.below(cells.len() as u32) as usize;
        let kept: Vec<GridCell> = cells
            .iter()
            .copied()
            .enumerate()
            .filter(|(i, _)| *i != victim)
            .map(|(_, c)| c)
            .collect();
        if edge_connected(&kept) {
            cells = kept;
        }
    }
    cells
}

fn gen_polyomino(rng: &mut Rng, target: usize) -> Vec<GridCell> {
    let mut cells = vec![GridCell { x: 0, y: 0 }];
    for _ in 0..target * 12 {
        if cells.len() >= target {
            break;
        }
        let from = cells[rng.below(cells.len() as u32) as usize];
        let (dx, dy) = rng.pick(&STEPS);
        let next = GridCell {
            x: from.x + dx,
            y: from.y + dy,
        };
        let inside = (0..SHAPE_EXTENT).contains(&next.x) && (0..SHAPE_EXTENT).contains(&next.y);
        if inside && !cells.contains(&next) {
            cells.push(next);
        }
    }
    sort_cells(&mut cells);
    cells
}

fn gen_cells(rng: &mut Rng, shape: Shape) -> Vec<GridCell> {
    match shape {
        Shape::Fixed(gx, gy) => rect_cells(gx, gy),
        Shape::Rect => gen_rect(rng),
        Shape::SmallRect => gen_small_rect(rng),
        Shape::Polyomino => {
            let target = rng.below(8) as usize + 2;
            gen_polyomino(rng, target)
        }
    }
}

fn bin_span(cells: &[GridCell]) -> (f32, f32) {
    let span = |sel: fn(&GridCell) -> i32| {
        let hi = cells.iter().map(sel).max().unwrap_or(0);
        (hi + 1) as f32 * GRID_PITCH
    };
    (span(|c| c.x), span(|c| c.y))
}

/// A wall the way the product makes one: axis aligned, on a cell boundary or a
/// cell centre, spanning the whole bin, and a thickness something could print.
fn gen_tidy_wall(rng: &mut Rng, cells: &[GridCell]) -> InnerWall {
    let (w, h) = bin_span(cells);
    let m = 10.0;
    let half = GRID_PITCH * 0.5;
    let at = |rng: &mut Rng, extent: f32| rng.quantised(half, extent - half, half);
    let (x1, y1, x2, y2) = if rng.chance(1, 2) {
        let y = at(rng, h);
        (-m, y, w + m, y)
    } else {
        let x = at(rng, w);
        (x, -m, x, h + m)
    };
    InnerWall {
        x1,
        y1,
        x2,
        y2,
        width: rng.quantised(0.8, 3.0, 0.2),
        height: if rng.chance(1, 3) {
            Some(rng.quantised(4.0, 12.0, 0.5))
        } else {
            None
        },
    }
}

/// A wall at any angle, any offset, any width -- including ones that clip a
/// corner or leave a sliver the floor fillet cannot round.
fn gen_freeform_wall(rng: &mut Rng, cells: &[GridCell]) -> InnerWall {
    let (w, h) = bin_span(cells);
    let m = 12.0;
    InnerWall {
        x1: rng.quantised(-m, w + m, 0.5),
        y1: rng.quantised(-m, h + m, 0.5),
        x2: rng.quantised(-m, w + m, 0.5),
        y2: rng.quantised(-m, h + m, 0.5),
        width: rng.quantised(0.8, 6.0, 0.2),
        height: if rng.chance(1, 3) {
            Some(rng.quantised(2.0, 16.0, 0.5))
        } else {
            None
        },
    }
}

/// Keep each of `edges` with probability `num`/`den`, in a fixed order, so the
/// case stream stays a function of the seed alone.
fn subset(rng: &mut Rng, edges: &[GridEdge], num: u32, den: u32) -> Vec<GridEdge> {
    edges
        .iter()
        .copied()
        .filter(|_| rng.chance(num, den))
        .collect()
}

fn adjacent_pairs(cells: &[GridCell]) -> Vec<(GridCell, GridCell)> {
    let mut out = Vec::new();
    for &c in cells {
        for (dx, dy) in [(1, 0), (0, 1)] {
            let n = GridCell {
                x: c.x + dx,
                y: c.y + dy,
            };
            if cells.contains(&n) {
                out.push((c, n));
            }
        }
    }
    out
}

fn flood_parts(cells: &[GridCell], severed: &[(GridCell, GridCell)]) -> Vec<Vec<GridCell>> {
    let joined = |a: GridCell, b: GridCell| {
        !severed
            .iter()
            .any(|&(p, q)| (p == a && q == b) || (p == b && q == a))
    };
    let mut unseen = cells.to_vec();
    let mut parts: Vec<Vec<GridCell>> = Vec::new();
    while let Some(first) = unseen.pop() {
        let mut queue = vec![first];
        let mut part = Vec::new();
        while let Some(c) = queue.pop() {
            part.push(c);
            for (dx, dy) in STEPS {
                let n = GridCell {
                    x: c.x + dx,
                    y: c.y + dy,
                };
                if !joined(c, n) {
                    continue;
                }
                if let Some(i) = unseen.iter().position(|&u| u == n) {
                    unseen.remove(i);
                    queue.push(n);
                }
            }
        }
        sort_cells(&mut part);
        parts.push(part);
    }
    parts.sort_by_key(|p| (p[0].y, p[0].x));
    parts
}

fn gen_case(rng: &mut Rng, opts: Options) -> Case {
    let cells = gen_cells(rng, opts.shape);

    let mut params = Params {
        bins: vec![LogicalBin {
            cells: cells.clone(),
            ..Default::default()
        }],
        ..Params::default()
    };

    let n_walls = opts.inner_walls.count(rng);
    params.inner_walls = (0..n_walls)
        .map(|_| match opts.inner_walls {
            Walls::None => unreachable!("count() returned 0"),
            Walls::Tidy(..) => gen_tidy_wall(rng, &cells),
            Walls::Freeform(..) => gen_freeform_wall(rng, &cells),
        })
        .collect();

    if let Openings::Share(num, den) = opts.openings {
        params.open_edges = subset(rng, &perimeter_edges(&cells), num, den);
    }
    if opts.dividers {
        params.divider_edges = subset(rng, &internal_edges(&cells), 1, 3);
    }

    if opts.vary_params {
        params.height_units = rng.below(6) + 1;
        params.wall_thickness = rng.quantised(0.4, 3.0, 0.1);
        params.cavity_corner_radius = rng.quantised(0.0, 5.0, 0.5);
        params.floor_fillet = rng.quantised(0.0, 5.6, 0.2);
        params.magnet_holes = rng.chance(1, 3);
        params.screw_holes = params.magnet_holes && rng.chance(1, 2);
    }
    if opts.slope && rng.chance(1, 6) {
        params.bins[0].slope = Some(BinSlope {
            angle_deg: rng.quantised(2.0, 20.0, 1.0),
            dir: rng.pick(&[
                SlopeDir::PlusX,
                SlopeDir::MinusX,
                SlopeDir::PlusY,
                SlopeDir::MinusY,
            ]),
        });
    }
    if opts.baseplate && rng.chance(1, 8) {
        params.mode = Mode::Baseplate;
    }

    let mut pieces = Vec::new();
    match opts.split {
        Split::Whole => {}
        Split::Flood => {
            let severed: Vec<(GridCell, GridCell)> = adjacent_pairs(&cells)
                .into_iter()
                .filter(|_| rng.chance(1, 3))
                .collect();
            pieces = flood_parts(&cells, &severed);
        }
        Split::Lines => {
            for axis in [Axis::X, Axis::Y] {
                for index in 1..SHAPE_EXTENT {
                    if rng.chance(1, 3) {
                        params.bins[0].split_lines.push(SplitLine { axis, index });
                    }
                }
            }
        }
    }

    Case {
        opts,
        params,
        pieces,
    }
}

// ---------------------------------------------------------------------------
// Checking
// ---------------------------------------------------------------------------

const TESS_SEGS: usize = 6;
const ENCLOSED_PIECE: &str = "surrounded on every side";
const VOLUME_DRIFT: f64 = 0.002;
const SPLIT_TOL: f32 = 0.15;
const QUANTUM: f32 = 20.0;
const OVERHANG_REACH: f32 = 8.0;

type Hook = Box<dyn Fn(&PanicHookInfo<'_>) + Sync + Send + 'static>;

static QUIET_DEPTH: Mutex<usize> = Mutex::new(0);
static LOUD_HOOK: Mutex<Option<Hook>> = Mutex::new(None);

struct Quiet;

fn quiet_panics() -> Quiet {
    let mut depth = QUIET_DEPTH.lock().unwrap();
    if *depth == 0 {
        *LOUD_HOOK.lock().unwrap() = Some(std::panic::take_hook());
        std::panic::set_hook(Box::new(|_| {}));
    }
    *depth += 1;
    Quiet
}

impl Drop for Quiet {
    fn drop(&mut self) {
        let mut depth = QUIET_DEPTH.lock().unwrap();
        *depth -= 1;
        if *depth == 0 {
            if let Some(loud) = LOUD_HOOK.lock().unwrap().take() {
                std::panic::set_hook(loud);
            }
        }
    }
}

fn catching(f: impl FnOnce() -> Result<(), String>) -> Result<(), String> {
    match catch_unwind(AssertUnwindSafe(f)) {
        Ok(r) => r,
        Err(e) => {
            let msg = e
                .downcast_ref::<&str>()
                .map(|s| s.to_string())
                .or_else(|| e.downcast_ref::<String>().cloned())
                .unwrap_or_else(|| "<non-string payload>".into());
            Err(format!("panic: {msg}"))
        }
    }
}

fn sound(solid: &Solid, what: &str) -> Result<(), String> {
    solid
        .validate()
        .map_err(|e| format!("{what}validate: {e}"))?;
    let report = audit(solid);
    if !report.is_ok() {
        return Err(format!("{what}audit: {report}"));
    }
    let leaks = tessellation_leaks(&tessellate(solid, TESS_SEGS));
    if !leaks.is_empty() {
        let named = leaks
            .iter()
            .map(|l| format!("{l:?}"))
            .min()
            .unwrap_or_default();
        return Err(format!(
            "{what}tessellation: {} leak(s), first {named}",
            leaks.len()
        ));
    }
    Ok(())
}

fn mesh_volume(solid: &Solid) -> f64 {
    let mesh = tessellate(solid, TESS_SEGS).to_mesh();
    let mut v = 0.0f64;
    for [a, b, c] in mesh.triangles() {
        v += a.dot(b.cross(c)) as f64;
    }
    v / 6.0
}

fn mesh_parts(mesh: &Mesh) -> usize {
    let mut owner: Vec<usize> = (0..mesh.positions.len()).collect();
    fn root(owner: &mut [usize], mut i: usize) -> usize {
        while owner[i] != i {
            owner[i] = owner[owner[i]];
            i = owner[i];
        }
        i
    }
    for t in mesh.indices.chunks_exact(3) {
        for pair in [(t[0], t[1]), (t[1], t[2])] {
            let (a, b) = (
                root(&mut owner, pair.0 as usize),
                root(&mut owner, pair.1 as usize),
            );
            owner[a] = b;
        }
    }
    let mut seen: HashSet<usize> = HashSet::new();
    let welded: Vec<usize> = mesh.indices.iter().map(|&i| i as usize).collect();
    for i in welded {
        let r = root(&mut owner, i);
        seen.insert(r);
    }
    seen.len()
}

fn well_inside(cells: &[GridCell], x: f32, y: f32) -> bool {
    cells.iter().any(|c| {
        let (x0, y0) = (c.x as f32 * GRID_PITCH, c.y as f32 * GRID_PITCH);
        x > x0 + SPLIT_TOL
            && x < x0 + GRID_PITCH - SPLIT_TOL
            && y > y0 + SPLIT_TOL
            && y < y0 + GRID_PITCH - SPLIT_TOL
    })
}

fn grid_gap(a: &[GridCell], b: &[GridCell]) -> f32 {
    let mut best = f32::INFINITY;
    for p in a {
        for q in b {
            let dx = ((p.x - q.x).abs() - 1).max(0) as f32 * GRID_PITCH;
            let dy = ((p.y - q.y).abs() - 1).max(0) as f32 * GRID_PITCH;
            best = best.min((dx * dx + dy * dy).sqrt());
        }
    }
    best
}

fn footprint(mesh: &Mesh) -> Vec<(f32, f32)> {
    let mut seen: HashSet<(i32, i32)> = HashSet::new();
    for p in &mesh.positions {
        seen.insert((
            (p.x * QUANTUM).round() as i32,
            (p.y * QUANTUM).round() as i32,
        ));
    }
    seen.into_iter()
        .map(|(x, y)| (x as f32 / QUANTUM, y as f32 / QUANTUM))
        .collect()
}

fn closest(a: &[(f32, f32)], b: &[(f32, f32)]) -> f32 {
    let mut best = f32::INFINITY;
    for &(ax, ay) in a {
        for &(bx, by) in b {
            best = best.min(((ax - bx).powi(2) + (ay - by).powi(2)).sqrt());
        }
    }
    best
}

fn as_pairs(cells: &[GridCell]) -> Vec<(i32, i32)> {
    let mut v: Vec<(i32, i32)> = cells.iter().map(|c| (c.x, c.y)).collect();
    v.sort_unstable();
    v
}

/// An independently derived chunking of the cells, to hold `partition_cells` to.
fn chunk_cells(cells: &[GridCell], lines: &[SplitLine]) -> Vec<Vec<(i32, i32)>> {
    let key = |c: GridCell| {
        let across = |axis, at| {
            lines
                .iter()
                .filter(|l| l.axis == axis && l.index <= at)
                .count()
        };
        (across(Axis::Y, c.y), across(Axis::X, c.x))
    };
    let mut groups: BTreeMap<(usize, usize), Vec<(i32, i32)>> = BTreeMap::new();
    for &c in cells {
        groups.entry(key(c)).or_default().push((c.x, c.y));
    }
    let mut out: Vec<Vec<(i32, i32)>> = groups.into_values().collect();
    for g in &mut out {
        g.sort_unstable();
    }
    out.sort();
    out
}

/// The pieces this case is to be carved into: empty for a whole-bin profile.
fn pieces_of(c: &Case) -> Vec<Vec<GridCell>> {
    match c.opts.split {
        Split::Whole => Vec::new(),
        Split::Flood => c.pieces.clone(),
        Split::Lines => partition_cells(&c.params.bins[0].cells, &c.params.bins[0].split_lines)
            .into_iter()
            .map(|p| p.cells)
            .collect(),
    }
}

/// The height of every bin's cavity floor. `plan_piece` takes it from these two
/// constants unconditionally, so a face at that z is a floor and nothing else
/// in the model is.
const FLOOR_Z: f32 = gridfinity::BASE_TOTAL_HEIGHT + gridfinity::FLOOR_THICKNESS;

/// How far a normal may sit off `+-Z` and still count as parallel to it. The
/// two normals are equal in exact arithmetic -- tangency is what a rolling-ball
/// blend *is* -- so this bounds accumulated f32 error in the blend solve and
/// the surface evaluation, nothing about the model.
const NORMAL_TOL: f32 = 1e-3;

/// `(cavity floors, of those, floors that meet every one of their walls sharp)`.
///
/// The floor fillet is the one thing about a bin whose loss is invisible to
/// every other invariant here: with every corner left sharp the solid is still
/// manifold, still audits clean and still tessellates without leaks. It is
/// visible in the finished B-rep as **tangency**. A rolling-ball blend meets
/// the floor along their shared edge with the floor's own normal -- that is the
/// definition of the blend, not a property of how it was requested -- while an
/// unblended wall meets the floor at a right angle. So an edge of a floor face
/// is rounded exactly when the face on its far side has `|n . Z| = 1` there.
///
/// Reading the solid rather than the `BlendReport` is what makes two builds of
/// two *different* bins comparable at all: `EdgeId`s do not survive a change to
/// the input, and the counts in the report do not say which compartment lost
/// what.
fn floor_fillet_coverage(solid: &Solid) -> (usize, usize) {
    let ef = solid.edge_faces();
    let (mut floors, mut sharp) = (0, 0);
    for fid in 0..solid.faces.len() {
        let Surface::Plane { origin, normal, .. } = solid.faces[fid].surface else {
            continue;
        };
        if normal.z.abs() < 1.0 - NORMAL_TOL || (origin.z - FLOOR_Z).abs() > 1e-3 {
            continue;
        }
        floors += 1;
        let rounded = solid.face_loops(fid).flatten().any(|&(e, _)| {
            let edge = &solid.edges[e];
            let p = edge.curve.point(0.5 * (edge.t0 + edge.t1));
            ef[e].iter().filter(|&&g| g != fid).any(|&g| {
                let s = &solid.faces[g].surface;
                s.normal(s.project(p)).z.abs() >= 1.0 - NORMAL_TOL
            })
        });
        if !rounded {
            sharp += 1;
        }
    }
    (floors, sharp)
}

/// An opening must not cost a compartment the floor fillet it would have had
/// without it.
///
/// `FILLET_FAILED` alone cannot see this. It holds the model to the blends it
/// *asked for*, and the loss happens one step earlier: whatever leaves the
/// cavity loop with sharp corners also stops `plan_piece` asking, and a report
/// of 0 requested / 0 refused is perfectly clean. So the same bin is built
/// again with `open_edges` cleared and the two are compared by
/// `floor_fillet_coverage`, **per floor face**: a compartment the closed bin
/// rounds and the opened bin leaves sharp is the defect, however many other
/// compartments kept their blends. Comparing against the closed build is also
/// what lets the check run on shapes where the fillet legitimately degrades --
/// a reentrant corner the model declines to round is declined in both builds
/// and cancels.
///
/// A bin with no wall left standing is exempt: there is nothing for a floor
/// fillet to roll against.
///
/// The report is compared as well as the solid, because `is_clean()` is
/// **vacuously true at zero requested**: a change that stops `plan_piece`
/// asking for the fillet altogether scores better on `FILLET_FAILED` than one
/// that asks and is refused, so tuning the model against that gate alone
/// rewards deleting the blend request. `made()` is what a user would see, and
/// it may not fall to nothing on a bin whose closed build rounds anything.
fn opening_keeps_the_fillet(
    c: &Case,
    opened: &Solid,
    opened_blends: &BlendReport,
) -> Result<(), String> {
    if c.params.open_edges.is_empty() {
        return Ok(());
    }
    let walls = effective_walls(
        &c.params.bins[0].cells,
        &c.params.bins[0].cells,
        &c.params.open_edges,
        &c.params.divider_edges,
    );
    if walls.walled.is_empty() {
        return Ok(());
    }
    let mut closed = c.params.clone();
    closed.open_edges.clear();
    let Ok((before, before_blends)) = gridfinity::try_build_reporting(&closed) else {
        return Ok(());
    };
    if before_blends.made() > 0 && opened_blends.made() == 0 {
        return Err(format!(
            "{OPENING_LOSES_FILLET}: {} opening(s) left the bin asking for {} blend(s) where \
             the same bin closed asks for {} and lands {}, though {} wall(s) still stand",
            c.params.open_edges.len(),
            opened_blends.requested,
            before_blends.requested,
            before_blends.made(),
            walls.walled.len()
        ));
    }
    let (was_floors, was_sharp) = floor_fillet_coverage(&before);
    let (now_floors, now_sharp) = floor_fillet_coverage(opened);
    if now_sharp <= was_sharp {
        return Ok(());
    }
    Err(format!(
        "{OPENING_LOSES_FILLET}: {} opening(s) took the bin from {was_sharp} of {was_floors} cavity floor(s) unrounded to {now_sharp} of {now_floors}, though {} wall(s) still stand",
        c.params.open_edges.len(),
        walls.walled.len()
    ))
}

fn check(c: &Case) -> Result<(), String> {
    catching(|| {
        let bin = &c.params.bins[0];

        if c.opts.split == Split::Lines {
            let parts = partition_cells(&bin.cells, &bin.split_lines);
            let mut got: Vec<Vec<(i32, i32)>> = parts.iter().map(|p| as_pairs(&p.cells)).collect();
            got.sort();
            let want = chunk_cells(&bin.cells, &bin.split_lines);
            if got != want {
                return Err(format!(
                    "partition: {} split line(s) should cut {} cell(s) into {} piece(s) {want:?}, got {} piece(s) {got:?}",
                    bin.split_lines.len(),
                    bin.cells.len(),
                    want.len(),
                    got.len()
                ));
            }
        }

        let pieces = pieces_of(c);

        // A whole-bin profile goes through the product's own entry point, which
        // also covers Baseplate mode; a split profile needs the one bin solid
        // every piece is carved off.
        let (whole, blends) = if pieces.is_empty() {
            gridfinity::try_build_reporting(&c.params).map_err(|e| format!("build error: {e}"))?
        } else {
            gridfinity::build_bin_solid_reporting(&c.params, &bin.cells, bin.slope)
                .map_err(|e| format!("build error: {e}"))?
        };
        // On a split profile the whole solid's failures are prefixed `whole `,
        // so a pre-existing model defect never reads as a split defect.
        sound(&whole, if pieces.is_empty() { "" } else { "whole " })?;

        // A fillet that does not land is a failure, not a degradation. The
        // model's own policy is the opposite -- `fillet_best_effort` would
        // rather leave a corner sharp than fail the build, so the user gets an
        // unrounded part and no error -- and that policy is exactly what a
        // fuzzer must not adopt: every profile here holds the model to every
        // blend it asked for.
        if !blends.is_clean() {
            return Err(format!(
                "{FILLET_FAILED}: of {} blend(s) the model asked for, {} matched no edge \
                 and {} were refused -- {}",
                blends.requested,
                blends.unresolved,
                blends.dropped.len(),
                blends.refusal.as_deref().unwrap_or("no reason recorded")
            ));
        }

        opening_keeps_the_fillet(c, &whole, &blends)?;

        if pieces.is_empty() {
            return Ok(());
        }

        let expected = mesh_volume(&whole);
        let mut meshes: Vec<Mesh> = Vec::new();
        let mut carved = 0.0;
        for (i, piece) in pieces.iter().enumerate() {
            let solid = match gridfinity::carve_to_cells(&whole, &bin.cells, piece) {
                Ok(s) => s,
                Err(e) if e.contains(ENCLOSED_PIECE) => return Ok(()),
                Err(e) => return Err(format!("carve piece {i}: {e}")),
            };
            sound(&solid, &format!("piece {i} "))?;
            let mesh = tessellate(&solid, TESS_SEGS).to_mesh();

            let islands = flood_parts(piece, &[]).len();
            let shells = mesh_parts(&mesh);
            if shells != islands {
                return Err(format!(
                    "piece {i} covers {} cell(s) in {islands} island(s) but is {shells} separate geometry(s)",
                    piece.len()
                ));
            }
            let trespass =
                pieces
                    .iter()
                    .enumerate()
                    .filter(|(j, _)| *j != i)
                    .find_map(|(j, other)| {
                        mesh.positions
                            .iter()
                            .find(|p| well_inside(other, p.x, p.y))
                            .map(|p| (j, *p))
                    });
            if let Some((j, p)) = trespass {
                return Err(format!(
                    "piece {i} keeps material at ({:.3}, {:.3}), which stands over piece {j}",
                    p.x, p.y
                ));
            }
            carved += mesh_volume(&solid);
            meshes.push(mesh);
        }

        let drift = (carved - expected).abs() / expected.abs().max(1.0);
        if drift > VOLUME_DRIFT {
            return Err(format!(
                "volume: {} piece(s) sum to {carved:.4} mm^3, whole is {expected:.4} mm^3 ({:.3}% off)",
                pieces.len(),
                drift * 100.0
            ));
        }

        // Whether two pieces touch is measured between their meshes' *vertices*,
        // which is only a proxy for "the surfaces meet". It holds when every
        // piece is a grid slab, because then two abutting pieces' cut faces have
        // the same outline and so land vertices at the same places. It does not
        // hold for a ragged flood-fill piece: a three-cell row cut against a
        // single cell genuinely abuts on the cut plane, yet the row's cut face
        // is subdivided differently and the nearest vertex pair stands 0.5 mm
        // apart. So this pair of checks is for `Split::Lines` only -- volume
        // conservation is what holds the flood pieces to meeting exactly.
        if c.opts.split != Split::Lines {
            return Ok(());
        }

        let prints: Vec<Vec<(f32, f32)>> = meshes.iter().map(footprint).collect();
        for i in 0..pieces.len() {
            for j in (i + 1)..pieces.len() {
                let want = grid_gap(&pieces[i], &pieces[j]);
                let got = closest(&prints[i], &prints[j]);
                if want == 0.0 && got > SPLIT_TOL {
                    return Err(format!(
                        "gap: pieces {i} and {j} were cut apart on a shared edge but stand {got:.3} mm apart"
                    ));
                }
                if want > 0.0 && got <= SPLIT_TOL {
                    return Err(format!(
                        "gap: pieces {i} and {j} touch, but no cell of one adjoins a cell of the other"
                    ));
                }
                if got < want - 2.0 * OVERHANG_REACH {
                    return Err(format!(
                        "gap: pieces {i} and {j} stand {got:.3} mm apart, the grid leaves {want:.3} mm between them"
                    ));
                }
            }
        }
        Ok(())
    })
}

// ---------------------------------------------------------------------------
// Shrinking
// ---------------------------------------------------------------------------

fn signature(err: &str) -> String {
    let mut out = String::new();
    let mut in_num = false;
    for c in err.chars() {
        if c.is_ascii_digit() || (c == '.' && in_num) {
            if !in_num {
                out.push('#');
                in_num = true;
            }
        } else {
            in_num = false;
            out.push(c);
        }
    }
    out.chars().take(140).collect()
}

fn keep_if(best: &mut Case, edit: impl FnOnce(&mut Case), accept: impl Fn(&Case) -> bool) {
    let mut q = best.clone();
    edit(&mut q);
    if accept(&q) {
        *best = q;
    }
}

/// Drop edges naming a cell the bin no longer has, so a shrunk repro carries no
/// entry the model would ignore anyway.
fn prune_edges(c: &mut Case) {
    let cells = &c.params.bins[0].cells;
    let perimeter: HashSet<GridEdge> = perimeter_edges(cells).into_iter().collect();
    let internal: HashSet<GridEdge> = internal_edges(cells).into_iter().collect();
    c.params.open_edges.retain(|e| perimeter.contains(e));
    c.params.divider_edges.retain(|e| internal.contains(e));
}

fn shrink(c: &Case, sig: &str) -> Case {
    let same = |q: &Case| check(q).is_err_and(|e| signature(&e) == sig);
    let mut best = c.clone();

    // Cells first: dropping one usually retires several walls and edges at once.
    //
    // Every candidate is an independent `check`, and a `check` is the most
    // expensive thing in the file, so the sweep is parallel. `find_map_first`
    // and not `find_map_any`: it returns the match earliest in *iteration*
    // order whatever order the threads finish in, which is what keeps a shrunk
    // repro a function of the seed alone.
    loop {
        let cells = best.params.bins[0].cells.clone();
        if cells.len() <= 1 {
            break;
        }
        let Some(smaller) = cells.par_iter().find_map_first(|&victim| {
            let kept: Vec<GridCell> = cells.iter().copied().filter(|&c| c != victim).collect();
            if !edge_connected(&kept) {
                return None;
            }
            let pieces: Vec<Vec<GridCell>> = best
                .pieces
                .iter()
                .map(|p| p.iter().copied().filter(|&c| c != victim).collect())
                .filter(|p: &Vec<GridCell>| !p.is_empty())
                .collect();
            if pieces.iter().any(|p| flood_parts(p, &[]).len() != 1) {
                return None;
            }
            let mut q = Case {
                pieces,
                ..best.clone()
            };
            q.params.bins[0].cells = kept;
            prune_edges(&mut q);
            same(&q).then_some(q)
        }) else {
            break;
        };
        best = smaller;
    }

    for i in (0..best.params.inner_walls.len()).rev() {
        keep_if(
            &mut best,
            |q| {
                q.params.inner_walls.remove(i);
            },
            &same,
        );
    }
    for i in (0..best.params.open_edges.len()).rev() {
        keep_if(
            &mut best,
            |q| {
                q.params.open_edges.remove(i);
            },
            &same,
        );
    }
    for i in (0..best.params.divider_edges.len()).rev() {
        keep_if(
            &mut best,
            |q| {
                q.params.divider_edges.remove(i);
            },
            &same,
        );
    }
    for i in (0..best.params.bins[0].split_lines.len()).rev() {
        keep_if(
            &mut best,
            |q| {
                q.params.bins[0].split_lines.remove(i);
            },
            &same,
        );
    }
    keep_if(&mut best, |q| q.params.bins[0].slope = None, &same);

    let d = Params::default();
    for (get, set) in [
        (
            (|p: &Params| p.floor_fillet) as fn(&Params) -> f32,
            (|p: &mut Params, v: f32| p.floor_fillet = v) as fn(&mut Params, f32),
        ),
        (
            |p| p.cavity_corner_radius,
            |p, v| p.cavity_corner_radius = v,
        ),
        (|p| p.wall_thickness, |p, v| p.wall_thickness = v),
    ] {
        keep_if(&mut best, |q| set(&mut q.params, get(&d)), &same);
    }
    keep_if(
        &mut best,
        |q| {
            q.params.magnet_holes = false;
            q.params.screw_holes = false;
        },
        &same,
    );
    best
}

// ---------------------------------------------------------------------------
// Repro printing
// ---------------------------------------------------------------------------

fn cell_list(cells: &[GridCell]) -> String {
    let cs: Vec<String> = cells
        .iter()
        .map(|c| format!("GridCell {{ x: {}, y: {} }}", c.x, c.y))
        .collect();
    format!("vec![{}]", cs.join(", "))
}

/// A failing case as the Rust literal that rebuilds it.
///
/// The `Params` half is `Params::rust_literal`, so a case the shrinker found and
/// a bin exported from the egui debugger print in exactly one format and either
/// pastes straight into a `#[test]`. Only the flood-fill piece list, which lives
/// on the `Case` rather than on `Params`, is added here.
fn repro(c: &Case) -> String {
    let head = c.params.rust_literal();
    if c.opts.split == Split::Flood {
        let pieces: Vec<String> = c.pieces.iter().map(|p| cell_list(p)).collect();
        return format!(
            "{head}
     pieces: vec![{}]",
            pieces.join(", ")
        );
    }
    head
}

// ---------------------------------------------------------------------------
// The sweep
// ---------------------------------------------------------------------------

struct Finding {
    count: usize,
    first: usize,
    detail: String,
}

/// A sweep's outcome: the whole report, and how many cases produced it.
///
/// There is no notion of an expected, known or tolerated failure here, and
/// there must not be one. A mechanism for forgiving a named signature is a
/// mechanism for a defect to sit in the suite indefinitely while the profile
/// around it reads green, and every use of it eventually outlives the diagnosis
/// that justified it. A profile either passes or it is telling you about a bug.
struct Report {
    text: String,
    failures: usize,
}

impl Report {
    /// Print the findings, then fail if there were any.
    fn gate(&self) {
        if !self.text.is_empty() {
            println!("{}", self.text);
        }
        assert!(self.failures == 0, "{}", self.text);
    }
}

fn sweep(opts: Options, cases: u32, seed: u64) -> Report {
    let mut rng = Rng::new(seed);
    let pool: Vec<Case> = (0..cases).map(|_| gen_case(&mut rng, opts)).collect();

    let quiet = quiet_panics();
    let errors: Vec<Option<String>> = pool.par_iter().map(|c| check(c).err()).collect();

    let mut found: BTreeMap<String, Finding> = BTreeMap::new();
    let mut failures = 0usize;
    for (i, err) in errors.iter().enumerate() {
        let Some(err) = err else { continue };
        failures += 1;
        match found.get_mut(&signature(err)) {
            Some(f) => f.count += 1,
            None => {
                found.insert(
                    signature(err),
                    Finding {
                        count: 1,
                        first: i,
                        detail: err.clone(),
                    },
                );
            }
        }
    }
    if found.is_empty() {
        return Report {
            text: String::new(),
            failures: 0,
        };
    }

    let entries: Vec<(&String, &Finding)> = found.iter().collect();
    let smallest: Vec<Case> = entries
        .par_iter()
        .map(|(sig, f)| shrink(&pool[f.first], sig.as_str()))
        .collect();
    drop(quiet);

    let mut out = format!(
        "{failures}/{cases} cases failed, {} distinct defect(s) (seed {seed}):\n",
        found.len()
    );
    for (i, ((_, f), small)) in entries.iter().zip(smallest.iter()).enumerate() {
        out.push_str(&format!(
            "\n[{}] x{}  {}\n     {}\n",
            i + 1,
            f.count,
            f.detail.lines().next().unwrap_or(""),
            repro(small)
        ));
    }
    Report {
        text: out,
        failures,
    }
}

fn env_u64(key: &str, default: u64) -> u64 {
    std::env::var(key)
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(default)
}

fn run(opts: Options, default_cases: u32) -> Report {
    let cases = env_u64("FUZZ_CASES", default_cases as u64) as u32;
    let seed = env_u64("FUZZ_SEED", 0x9E37_79B9_7F4A_7C15);
    sweep(opts, cases, seed)
}

// ---------------------------------------------------------------------------
// The floor-fillet predicate itself
// ---------------------------------------------------------------------------

/// `floor_fillet_coverage` has to be pinned against a bin whose answer is known
/// by construction, because both of its failure modes are silent: a predicate
/// that finds no floor at all, and one that calls every floor rounded, each
/// make `opening_keeps_the_fillet` pass unconditionally.
///
/// A divider is per *cell* edge, so it takes both internal H edges of a 2x2 to
/// cut the cavity in two -- one of them alone leaves the floor a connected U
/// around the far column, which is a single face and correctly counted as one.
/// `floor_fillet: 0.0` is the model declining to round any of them.
#[test]
fn a_cavity_floor_is_rounded_exactly_when_the_model_filleted_it() {
    let bin = |dividers: Vec<GridEdge>, fr: f32| {
        let mut p = Params {
            divider_edges: dividers,
            ..Params::rect(2, 2)
        };
        p.floor_fillet = fr;
        floor_fillet_coverage(&gridfinity::try_build(&p).expect("2x2 bin builds"))
    };
    let d = Params::default();
    assert!(d.floor_fillet > 0.0, "the default bin must want a fillet");

    assert_eq!(bin(Vec::new(), d.floor_fillet), (1, 0), "one round floor");
    assert_eq!(bin(Vec::new(), 0.0), (1, 1), "one sharp floor");

    let split: Vec<GridEdge> = (0..2)
        .map(|x| GridEdge {
            x,
            y: 1,
            orientation: Orientation::H,
        })
        .collect();
    assert_eq!(bin(split.clone(), d.floor_fillet), (2, 0), "both rounded");
    assert_eq!(bin(split, 0.0), (2, 2), "both sharp");
}

// ---------------------------------------------------------------------------
// Profiles
// ---------------------------------------------------------------------------

/// An opening still costs a compartment its whole floor fillet.
const OPENING_LOSES_FILLET: &str = "opening cost the floor fillet";

/// A blend the model asked for did not land. Every profile fails on this now:
/// a fillet that does not build is an error, the way it is in a commercial
/// modeller, not a corner quietly left sharp.
const FILLET_FAILED: &str = "fillet failed";

/// Free-form inner walls on a fixed 2x2 -- the manifoldness gate, and now the
/// sharpest measure of what `fillet_best_effort` cannot do. A wall at an
/// arbitrary angle routinely leaves a sliver narrower than the floor fillet and
/// the blend is refused; that used to be exempted here as documented policy and
/// is a `FILLET_FAILED` failure now.
#[test]
fn fuzz_inner_walls() {
    run(
        Options {
            inner_walls: Walls::Freeform(1, 3),
            ..Options::default()
        },
        150,
    )
    .gate();
}

/// Inner walls the product actually makes, up to three of them so they cross
/// one another. **Reports rather than gates**: two crossing walls at certain
/// positions hand the triangulator a face whose loops do not tile, which is a
/// live undiagnosed defect (see `crates/CLAUDE.md`).
#[test]
fn fuzz_tidy_inner_walls() {
    run(
        Options {
            shape: Shape::Rect,
            inner_walls: Walls::Tidy(1, 3),
            ..Options::default()
        },
        150,
    )
    .gate();
}

/// Wall openings and grid dividers -- an opening removes a wall the floor
/// fillet was blending against, which is exactly where a runout has to hold up.
/// `Shape::Rect` keeps the outline free of reentrant corners;
/// `fuzz_stripped_polyominoes` and `fuzz_params_broad` cover the ones that are
/// not. Together with `fuzz_openings_and_inner_walls` this is the pair that
/// gates "adding a wall opening does not break filleting", and it holds three
/// separate things: every blend the opened bin asks for lands
/// (`FILLET_FAILED`), no compartment it rounded when closed comes back sharp
/// (`OPENING_LOSES_FILLET`), and the solid is still sound.
#[test]
fn fuzz_wall_openings() {
    run(
        Options {
            shape: Shape::Rect,
            openings: Openings::Share(1, 4),
            dividers: true,
            ..Options::default()
        },
        150,
    )
    .gate();
}

/// Openings, dividers and inner walls together: the combination the fillet has
/// the least room to work in. This is the gate for "adding a wall opening or an
/// internal wall never breaks filleting".
#[test]
fn fuzz_openings_and_inner_walls() {
    run(
        Options {
            shape: Shape::Rect,
            inner_walls: Walls::Tidy(1, 2),
            openings: Openings::Share(1, 4),
            dividers: true,
            ..Options::default()
        },
        150,
    )
    .gate();
}

/// Half a complex polyomino's perimeter wall taken away.
///
/// The other opening profiles use `Shape::Rect` deliberately, so every run an
/// opening pinches against is straight. This one gives up that guarantee on
/// purpose: a polyomino has reentrant corners, and at a quarter of the
/// perimeter most of them are never reached. At a half they are, along with the
/// cases a single opening cannot produce -- two openings either side of one
/// corner, a run of them meeting end to end, and an opening wrapped around a
/// convex corner so the cavity leaves the outline on two sides at once.
///
/// A reentrant corner is where the floor fillet used to be allowed to degrade,
/// and this profile is where that shows up in quantity: it also holds that the
/// bin *builds*, stays manifold, audits clean and tessellates without leaks,
/// all of which it does.
#[test]
fn fuzz_stripped_polyominoes() {
    run(
        Options {
            shape: Shape::Polyomino,
            openings: Openings::Share(1, 2),
            dividers: true,
            vary_params: true,
            ..Options::default()
        },
        150,
    )
    .gate();
}

/// The split path through the web app's partition model: arbitrary connected
/// polyomino pieces carved off one bin.
#[test]
fn fuzz_bin_shapes() {
    run(
        Options {
            shape: Shape::Polyomino,
            vary_params: true,
            split: Split::Flood,
            ..Options::default()
        },
        120,
    )
    .gate();
}

/// The split path through the product's own model: `SplitLine`s and
/// `partition_cells`, plus everything a split is supposed to mean.
#[test]
fn fuzz_split_pieces() {
    run(
        Options {
            shape: Shape::Polyomino,
            vary_params: true,
            split: Split::Lines,
            ..Options::default()
        },
        120,
    )
    .gate();
}

/// Everything at once, on shapes that include a reentrant corner. Reports
/// rather than gates: it covers two undiagnosed defects -- an opening whose run
/// abuts a reentrant fillet panics the open-run planner, and a free-form wall
/// makes the model over-ask for floor blends.
#[test]
fn fuzz_params_broad() {
    run(
        Options {
            shape: Shape::SmallRect,
            inner_walls: Walls::Freeform(0, 2),
            openings: Openings::Share(1, 4),
            dividers: true,
            vary_params: true,
            slope: true,
            baseplate: true,
            ..Options::default()
        },
        400,
    )
    .gate();
}
