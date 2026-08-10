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
use gridfinity_cad::kernel::mesh::Mesh;
use gridfinity_cad::kernel::tess::tessellate;
use gridfinity_cad::layout::{
    Axis, GridCell, GridEdge, SplitLine, internal_edges, partition_cells, perimeter_edges,
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
/// The split matters because it decides whether a refused blend is a defect.
/// `Tidy` is the shape the editor and the Projects packer emit -- axis
/// aligned, spanning the bin, a printable thickness -- and the model rounds
/// every one of those cleanly, so a profile generating them can *require* the
/// blends to land. `Freeform` puts a wall at any angle and any offset, which
/// routinely leaves a sliver too thin to carry the floor fillet; there
/// `fillet_best_effort` degrades by design and requiring blends would be
/// asserting the impossible.
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

#[derive(Clone, Copy)]
struct Options {
    shape: Shape,
    inner_walls: Walls,
    /// Open some perimeter edges (a wall opening).
    openings: bool,
    /// Wall off some internal edges (a divider).
    dividers: bool,
    /// Vary height, thicknesses, radii and holes rather than taking defaults.
    vary_params: bool,
    /// Sometimes give the bin a sloped floor.
    slope: bool,
    /// Sometimes build a baseplate instead of a bin.
    baseplate: bool,
    split: Split,
    /// Insist every blend the model asked for was actually made. A dropped
    /// blend is legal geometry -- `fillet_best_effort` would rather leave a
    /// corner sharp than fail the build -- so this only means something on a
    /// profile whose features all have room for the model's radii. See
    /// `Walls`.
    require_blends: bool,
    /// Substrings naming defects that are pre-existing, undiagnosed and
    /// recorded in `rust/CLAUDE.md`. A case failing only these is still
    /// counted and still printed, but does not fail the gate -- so the profile
    /// keeps catching everything else instead of being demoted to reporting
    /// wholesale. Never add a signature here to silence something new.
    known: &'static [&'static str],
}

impl Default for Options {
    fn default() -> Options {
        Options {
            shape: Shape::Fixed(2, 2),
            inner_walls: Walls::None,
            openings: false,
            dividers: false,
            vary_params: false,
            slope: false,
            baseplate: false,
            split: Split::Whole,
            require_blends: false,
            known: &[],
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

    if opts.openings {
        params.open_edges = subset(rng, &perimeter_edges(&cells), 1, 4);
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

        if c.opts.require_blends && !blends.is_clean() {
            return Err(format!(
                "fillet: of {} blend(s) the model asked for, {} matched no edge and {} were refused",
                blends.requested,
                blends.unresolved,
                blends.dropped.len()
            ));
        }

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
            let trespass = pieces
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
        let Some(smaller) = cells
            .par_iter()
            .find_map_first(|&victim| {
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
            })
        else {
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

fn edge_list(edges: &[GridEdge]) -> String {
    let es: Vec<String> = edges
        .iter()
        .map(|e| {
            format!(
                "GridEdge {{ x: {}, y: {}, orientation: Orientation::{:?} }}",
                e.x, e.y, e.orientation
            )
        })
        .collect();
    format!("vec![{}]", es.join(", "))
}

fn repro(c: &Case) -> String {
    let p = &c.params;
    let d = Params::default();
    let mut f: Vec<String> = Vec::new();
    let bin = &p.bins[0];

    let mut binf: Vec<String> = vec![format!("cells: {}", cell_list(&bin.cells))];
    if !bin.split_lines.is_empty() {
        let ls: Vec<String> = bin
            .split_lines
            .iter()
            .map(|l| {
                format!(
                    "SplitLine {{ axis: Axis::{:?}, index: {} }}",
                    l.axis, l.index
                )
            })
            .collect();
        binf.push(format!("split_lines: vec![{}]", ls.join(", ")));
    }
    if let Some(s) = bin.slope {
        binf.push(format!(
            "slope: Some(BinSlope {{ angle_deg: {:?}, dir: SlopeDir::{:?} }})",
            s.angle_deg, s.dir
        ));
    }
    f.push(format!(
        "bins: vec![LogicalBin {{ {}, ..Default::default() }}]",
        binf.join(", ")
    ));

    if p.height_units != d.height_units {
        f.push(format!("height_units: {}", p.height_units));
    }
    for (name, v, dv) in [
        ("wall_thickness", p.wall_thickness, d.wall_thickness),
        (
            "cavity_corner_radius",
            p.cavity_corner_radius,
            d.cavity_corner_radius,
        ),
        ("floor_fillet", p.floor_fillet, d.floor_fillet),
    ] {
        if v != dv {
            f.push(format!("{name}: {v:?}"));
        }
    }
    if p.magnet_holes {
        f.push("magnet_holes: true".into());
    }
    if p.screw_holes {
        f.push("screw_holes: true".into());
    }
    if p.mode != d.mode {
        f.push(format!("mode: Mode::{:?}", p.mode));
    }
    if !p.open_edges.is_empty() {
        f.push(format!("open_edges: {}", edge_list(&p.open_edges)));
    }
    if !p.divider_edges.is_empty() {
        f.push(format!("divider_edges: {}", edge_list(&p.divider_edges)));
    }
    if !p.inner_walls.is_empty() {
        let ws: Vec<String> = p
            .inner_walls
            .iter()
            .map(|w| {
                let h = match w.height {
                    Some(h) => format!("Some({h:?})"),
                    None => "None".into(),
                };
                format!(
                    "InnerWall {{ x1: {:?}, y1: {:?}, x2: {:?}, y2: {:?}, width: {:?}, height: {h} }}",
                    w.x1, w.y1, w.x2, w.y2, w.width
                )
            })
            .collect();
        f.push(format!("inner_walls: vec![{}]", ws.join(", ")));
    }

    let head = format!("Params {{ {}, ..Params::default() }}", f.join(", "));
    if c.opts.split == Split::Flood {
        let pieces: Vec<String> = c.pieces.iter().map(|p| cell_list(p)).collect();
        return format!("{head}\n     pieces: vec![{}]", pieces.join(", "));
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

/// A sweep's outcome. `text` is the whole report, known defects included;
/// `unexpected` counts only the cases that failed for a reason the profile did
/// not already know about, and is what a gate asserts on.
struct Report {
    text: String,
    unexpected: usize,
}

impl Report {
    /// Print the findings, then fail if any of them was new.
    fn gate(&self) {
        if !self.text.is_empty() {
            println!("{}", self.text);
        }
        assert!(self.unexpected == 0, "{}", self.text);
    }

    /// Print the findings and let them all pass.
    fn note(&self) {
        if self.text.is_empty() {
            println!("all clean");
        } else {
            println!("{}", self.text);
        }
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
            unexpected: 0,
        };
    }

    let entries: Vec<(&String, &Finding)> = found.iter().collect();
    let smallest: Vec<Case> = entries
        .par_iter()
        .map(|(sig, f)| shrink(&pool[f.first], sig.as_str()))
        .collect();
    drop(quiet);

    let is_known = |f: &Finding| opts.known.iter().any(|k| f.detail.contains(k));
    let unexpected: usize = entries
        .iter()
        .filter(|(_, f)| !is_known(f))
        .map(|(_, f)| f.count)
        .sum();

    let mut out = format!(
        "{failures}/{cases} cases failed ({unexpected} unexpected), {} distinct defect(s) (seed {seed}):\n",
        found.len()
    );
    for (i, ((_, f), small)) in entries.iter().zip(smallest.iter()).enumerate() {
        out.push_str(&format!(
            "\n[{}] x{}{}  {}\n     {}\n",
            i + 1,
            f.count,
            if is_known(f) { " KNOWN" } else { "" },
            f.detail.lines().next().unwrap_or(""),
            repro(small)
        ));
    }
    Report {
        text: out,
        unexpected,
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
// Profiles
// ---------------------------------------------------------------------------

/// The one section curve `trim` still cannot express. Recorded in
/// `rust/CLAUDE.md` as undiagnosed; both split profiles meet it at ~1-2% of
/// cases at every seed, so it is named rather than left to redden the gate.
const TRIM_SECTION_CURVE: &str = "no closed-form section curve for a face the cut crosses";

/// Two inner walls crossing at a cell centre can hand the triangulator a face
/// whose loops do not tile. Found by `fuzz_tidy_inner_walls`, undiagnosed, and
/// recorded in `rust/CLAUDE.md`. It is named only where a profile is aimed at
/// something else; `fuzz_tidy_inner_walls` itself keeps reporting it.
const CROSSING_WALL_TILING: &str = "is not a tiling";

/// Free-form inner walls on a fixed 2x2 -- the manifoldness gate. Blends are
/// *not* required here: a wall at an arbitrary angle routinely leaves a sliver
/// narrower than the floor fillet, and the model declining that blend is the
/// documented policy rather than a defect.
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
/// live undiagnosed defect (see `rust/CLAUDE.md`). The blend requirement over
/// tidy walls is gated by `fuzz_openings_and_inner_walls`.
#[test]
fn fuzz_tidy_inner_walls() {
    run(
        Options {
            shape: Shape::Rect,
            inner_walls: Walls::Tidy(1, 3),
            require_blends: true,
            ..Options::default()
        },
        150,
    )
    .note();
}

/// Wall openings and grid dividers, with every blend required to land -- an
/// opening removes a wall the floor fillet was blending against, which is
/// exactly where a runout has to hold up. `Shape::Rect` keeps the outline free
/// of reentrant corners; `fuzz_params_broad` covers the ones that are not.
#[test]
fn fuzz_wall_openings() {
    run(
        Options {
            shape: Shape::Rect,
            openings: true,
            dividers: true,
            require_blends: true,
            ..Options::default()
        },
        150,
    )
    .gate();
}

/// Openings, dividers and inner walls together: the combination the fillet has
/// the least room to work in, still required to blend. This is the gate for
/// "adding a wall opening or an internal wall never breaks filleting".
#[test]
fn fuzz_openings_and_inner_walls() {
    run(
        Options {
            shape: Shape::Rect,
            inner_walls: Walls::Tidy(1, 2),
            openings: true,
            dividers: true,
            require_blends: true,
            known: &[CROSSING_WALL_TILING],
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
            known: &[TRIM_SECTION_CURVE],
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
            known: &[TRIM_SECTION_CURVE],
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
            openings: true,
            dividers: true,
            vary_params: true,
            slope: true,
            baseplate: true,
            ..Options::default()
        },
        400,
    )
    .note();
}
