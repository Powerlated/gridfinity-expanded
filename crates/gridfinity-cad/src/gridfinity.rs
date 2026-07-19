//! The parametric Gridfinity model, built on the analytic B-rep kernel.
//!
//! Mirrors the TypeScript reference's `BinConfig` model: a layout holds
//! **logical bins**, each an arbitrary polyomino of grid cells with optional
//! split lines and a floor slope; global parameters add walls, cavity corner
//! rounding, the concave floor fillet, fastener pockets, and per-edge
//! open/divider exceptions.
//!
//! Everything is assembled constructively into [`Builder`]s (no booleans):
//! - The **base** is one spec connector peg per cell (35.6 → 37.2 → 41.5 mm
//!   chamfered lofts). Where a peg's top runs along the bin perimeter it is
//!   flush with (and shares edges with) the outer wall; between cells the
//!   "bridge" underside is authored as planar faces stitched from the free
//!   peg-top and outer-profile segments.
//! - The **outer wall** follows the traced polyomino boundary, inset 0.25 mm
//!   from the pitch lines with convex corners rounded `OUTER_R` (concave
//!   corners sharp) — the spec profile.
//! - The **cavity** is planned as axis-aligned rectangles exactly like the
//!   reference (`cells − walled strips − divider strips − concave patches`),
//!   resolved by the [`rectregion`](crate::kernel::rectregion) engine, with convex
//!   corners rounded by `cavity_corner_radius` and concave corners rounded by
//!   the fillet radius so the floor-wall blend chain stays tangent-continuous.
//! - The **floor fillet** is a true rolling-ball blend
//!   ([`fillet::blend_edges`](crate::kernel::fillet::blend_edges)) over each cavity
//!   loop's floor-wall edges.

use crate::kernel::build::{RingEdges, loop_of, ring, wall_between};
use crate::kernel::fillet::blend_edges;
use crate::kernel::geom::Surface;
use crate::layout::{
    EdgeClass, EffectiveWalls, GridCell, GridEdge, Orientation, SplitLine, cell_edges,
    classify_edge, effective_walls, partition_cells,
};
use crate::kernel::math::{Vec2, Vec3, vec3_of};
use crate::kernel::rectregion::{LoopStyle, RectF, TracedLoop, shape_loop, trace_rects};
use crate::kernel::build::{seg_edge, wall_seg};
use crate::kernel::segdiff::{chain_loops, split_region_by_quad, subtract_convex_quad};
use crate::kernel::slab::{Op, Slab, SlabOpts, emit_slabs};
use crate::kernel::sketch::{Seg, Sketch, ccw_segs, loop_area, point_in_segs, reverse_loop};
use crate::kernel::topo::{Builder, EdgeId, Loop, Solid, VertexId};
use std::collections::{HashMap, HashSet};
use std::f32::consts::PI;

// ── Gridfinity spec constants (mm) ───────────────────────────────────────────
pub const GRID_PITCH: f32 = 42.0;
pub const HEIGHT_PER_UNIT: f32 = 7.0;
pub const BASE_TOTAL_HEIGHT: f32 = 7.0;
pub const PEG_HEIGHT: f32 = 4.75;
pub const PEG_Z1: f32 = 0.8;
pub const PEG_Z2: f32 = 2.6;
pub const OUTER_R: f32 = 3.75;
pub const FLOOR_THICKNESS: f32 = 1.2;
pub const HALF_TOL: f32 = 0.25; // (GRID_PITCH − 41.5)/2 clearance per side
const MAGNET_RADIUS: f32 = 3.25;
const MAGNET_DEPTH: f32 = 2.4;
const SCREW_RADIUS: f32 = 1.5;
const SCREW_DEPTH: f32 = 6.0;
const FASTENER_INSET: f32 = 13.0;

// Spec connector-peg profile (mm).
const PEG_W_BOTTOM: f32 = 35.6;
const PEG_W_MID: f32 = 37.2;
const PEG_W_TOP: f32 = 41.5;
const PEG_R_BOTTOM: f32 = 0.8;
// The mid radius keeps every profile's corner-arc CENTER on the same vertical
// axis (inset + radius = 4.0 for all three sections: 3.2+0.8, 2.4+1.6,
// 0.25+3.75), so the chamfer walls are true coaxial cones. The reference uses
// 1.5 here, but it lofts with mesh hulls and doesn't need coaxiality; the
// visual difference on a hidden chamfer is 0.1 mm.
const PEG_R_MID: f32 = 1.6;
/// Distance from a pitch corner to a peg-top arc tangent point along the side.
const PEG_TANGENT: f32 = HALF_TOL + OUTER_R; // = 4.0

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    Bin,
    Baseplate,
}

/// Side of the bin whose floor is lowest (floor rises away from this side).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SlopeDir {
    PlusX,
    MinusX,
    PlusY,
    MinusY,
}

/// A sloped compartment floor: `angle_deg` from horizontal, lowest at `dir`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BinSlope {
    pub angle_deg: f32,
    pub dir: SlopeDir,
}

/// A complete logical bin: its polyomino cell set, the split lines that cut it
/// into printable pieces, and an optional floor slope.
#[derive(Clone, Debug, Default)]
pub struct LogicalBin {
    pub cells: Vec<GridCell>,
    pub split_lines: Vec<SplitLine>,
    pub slope: Option<BinSlope>,
}

impl LogicalBin {
    pub fn rect(gx: u32, gy: u32) -> LogicalBin {
        LogicalBin { cells: rect_cells(gx, gy), ..Default::default() }
    }
}

/// Rectangular cell block at the origin.
pub fn rect_cells(gx: u32, gy: u32) -> Vec<GridCell> {
    let mut out = Vec::new();
    for x in 0..gx.max(1) as i32 {
        for y in 0..gy.max(1) as i32 {
            out.push(GridCell { x, y });
        }
    }
    out
}

/// Free-form inner wall: a straight segment in whole-layout mm coordinates,
/// not grid-aligned. Clipped to the cavity interior; where it is lower than
/// the rim, a concave quarter-cylinder ramp blends its top into any taller
/// structure it touches (mirrors the reference `InnerWall`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InnerWall {
    pub x1: f32,
    pub y1: f32,
    pub x2: f32,
    pub y2: f32,
    /// mm; clamped to ≥ 0.4 like the reference.
    pub width: f32,
    /// mm above the cavity floor; `None` = full height.
    pub height: Option<f32>,
}

/// Everything the UI can tune — mirrors the reference `BinConfig`.
#[derive(Clone, Debug)]
pub struct Params {
    pub bins: Vec<LogicalBin>,
    pub height_units: u32,
    pub wall_thickness: f32,
    pub cavity_corner_radius: f32,
    pub floor_fillet: f32,
    pub magnet_holes: bool,
    pub screw_holes: bool,
    pub open_edges: Vec<GridEdge>,
    pub divider_edges: Vec<GridEdge>,
    pub inner_walls: Vec<InnerWall>,
    pub mode: Mode,
}

impl Default for Params {
    fn default() -> Params {
        Params {
            bins: vec![LogicalBin::rect(2, 2)],
            height_units: 3,
            wall_thickness: 1.2,
            cavity_corner_radius: 2.5,
            floor_fillet: 3.0,
            magnet_holes: false,
            screw_holes: false,
            open_edges: Vec::new(),
            divider_edges: Vec::new(),
            inner_walls: Vec::new(),
            mode: Mode::Bin,
        }
    }
}

impl Params {
    /// A single rectangular bin (the historical `grid_x`/`grid_y` form).
    pub fn rect(gx: u32, gy: u32) -> Params {
        Params { bins: vec![LogicalBin::rect(gx, gy)], ..Default::default() }
    }

    /// Convenience: evenly-spaced divider edges over the first bin's bounding
    /// grid (n cuts ⇒ n+1 compartments per axis).
    pub fn divisions(mut self, div_x: u32, div_y: u32) -> Params {
        let (gx, gy) = self.grid_extent();
        self.divider_edges = divisions_to_edges(gx, gy, div_x, div_y);
        self
    }

    /// Bounding grid extent over every bin's cells.
    pub fn grid_extent(&self) -> (u32, u32) {
        let mut mx = 0;
        let mut my = 0;
        for b in &self.bins {
            for c in &b.cells {
                mx = mx.max(c.x + 1);
                my = my.max(c.y + 1);
            }
        }
        (mx.max(1) as u32, my.max(1) as u32)
    }

    pub fn all_cells(&self) -> Vec<GridCell> {
        let mut out = Vec::new();
        for b in &self.bins {
            out.extend(b.cells.iter().copied());
        }
        out
    }

    pub fn total_height(&self) -> f32 {
        BASE_TOTAL_HEIGHT + HEIGHT_PER_UNIT * self.height_units.max(1) as f32
    }
}

/// Build evenly spaced divider edges for a `gx × gy` grid: `dx` vertical cuts
/// and `dy` horizontal cuts, each spanning the full grid.
pub fn divisions_to_edges(gx: u32, gy: u32, dx: u32, dy: u32) -> Vec<GridEdge> {
    let mut out = Vec::new();
    if gx >= 2 {
        let n = dx.min(gx - 1) as i32;
        let span = gx as i32;
        for i in 0..n {
            let idx = ((i + 1) as f32 * span as f32 / (n + 1) as f32).round() as i32;
            for y in 0..gy as i32 {
                out.push(GridEdge { x: idx, y, orientation: Orientation::V });
            }
        }
    }
    if gy >= 2 {
        let n = dy.min(gy - 1) as i32;
        let span = gy as i32;
        for i in 0..n {
            let idx = ((i + 1) as f32 * span as f32 / (n + 1) as f32).round() as i32;
            for x in 0..gx as i32 {
                out.push(GridEdge { x, y: idx, orientation: Orientation::H });
            }
        }
    }
    out
}

/// Horizontal planar face at height `z` (`up` → +Z normal, else −Z).
fn planar(b: &mut Builder, z: f32, up: bool, outer: Loop, inners: Vec<Loop>) {
    let surface = if up {
        Surface::plane_z(z)
    } else {
        Surface::plane(vec3_of(0.0, 0.0, z), -Vec3::Z)
    };
    b.face(surface, true, outer, inners);
}

/// Build the whole layout (every logical bin, splits ignored, open edges
/// applied) as one solid — the assembled preview.
pub fn build(p: &Params) -> Solid {
    match p.mode {
        Mode::Bin => {
            let mut b = Builder::new();
            let mut blends: Vec<(EdgeId, f32)> = Vec::new();
            for bin in &p.bins {
                if bin.cells.is_empty() {
                    continue;
                }
                let walls =
                    effective_walls(&bin.cells, &bin.cells, &p.open_edges, &p.divider_edges);
                blends.extend(build_piece(&mut b, p, &bin.cells, &bin.cells, walls, bin.slope));
            }
            let solid = b.build();
            if blends.is_empty() {
                solid
            } else {
                blend_edges(&solid, &blends).expect("floor fillet blend")
            }
        }
        Mode::Baseplate => build_baseplate(p),
    }
}

/// One printable piece of the split-aware build.
pub struct BinPiece {
    pub name: String,
    pub col: i32,
    pub row: i32,
    pub solid: Solid,
}

/// Split-aware build: partitions each logical bin by its split lines and
/// builds every piece as an independent watertight solid (in layout
/// coordinates). Seam faces are square on the pitch plane and open — walled
/// only where a divider sits on the split line — so glued pieces form one
/// continuous bin. Every piece keeps its own base pegs.
pub fn build_pieces(p: &Params) -> Vec<BinPiece> {
    if p.mode == Mode::Baseplate {
        return vec![BinPiece {
            name: "gridfinity-baseplate.stl".into(),
            col: 0,
            row: 0,
            solid: build_baseplate(p),
        }];
    }
    let bins: Vec<&LogicalBin> = p.bins.iter().filter(|b| !b.cells.is_empty()).collect();
    let mut out = Vec::new();
    for (bi, bin) in bins.iter().enumerate() {
        let parts = partition_cells(&bin.cells, &bin.split_lines);
        let stem = if bins.len() == 1 {
            "gridfinity-bin".to_string()
        } else {
            format!("gridfinity-bin-{}", bi + 1)
        };
        for (i, part) in parts.iter().enumerate() {
            let mut b = Builder::new();
            let walls =
                effective_walls(&part.cells, &bin.cells, &p.open_edges, &p.divider_edges);
            let blends = build_piece(&mut b, p, &part.cells, &bin.cells, walls, bin.slope);
            let solid = b.build();
            let solid = if blends.is_empty() {
                solid
            } else {
                blend_edges(&solid, &blends).expect("floor fillet blend")
            };
            let name = if parts.len() == 1 {
                format!("{stem}.stl")
            } else {
                format!("{stem}-piece-{}-of-{}.stl", i + 1, parts.len())
            };
            out.push(BinPiece { name, col: part.col, row: part.row, solid });
        }
    }
    out
}

// ── Boundary walk ────────────────────────────────────────────────────────────

/// One unit step of a region boundary (material on the left), on pitch-corner
/// lattice coordinates.
#[derive(Clone, Copy, Debug)]
struct Step {
    from: (i32, i32),
    to: (i32, i32),
    edge: GridEdge,
}

impl Step {
    fn dir(&self) -> (i32, i32) {
        (self.to.0 - self.from.0, self.to.1 - self.from.1)
    }
}

/// Trace the boundary of a cell set into unit-step loops (outer loops CCW,
/// hole loops CW; material on the left throughout).
fn boundary_steps(cells: &[GridCell]) -> Vec<Vec<Step>> {
    let set: HashSet<GridCell> = cells.iter().copied().collect();
    let present = |x: i32, y: i32| set.contains(&GridCell { x, y });
    let mut adj: HashMap<(i32, i32), Vec<Step>> = HashMap::new();
    for &c in cells {
        let (x, y) = (c.x, c.y);
        if !present(x, y - 1) {
            let s = Step { from: (x, y), to: (x + 1, y), edge: GridEdge { x, y, orientation: Orientation::H } };
            adj.entry(s.from).or_default().push(s);
        }
        if !present(x + 1, y) {
            let s = Step { from: (x + 1, y), to: (x + 1, y + 1), edge: GridEdge { x: x + 1, y, orientation: Orientation::V } };
            adj.entry(s.from).or_default().push(s);
        }
        if !present(x, y + 1) {
            let s = Step { from: (x + 1, y + 1), to: (x, y + 1), edge: GridEdge { x, y: y + 1, orientation: Orientation::H } };
            adj.entry(s.from).or_default().push(s);
        }
        if !present(x - 1, y) {
            let s = Step { from: (x, y + 1), to: (x, y), edge: GridEdge { x, y, orientation: Orientation::V } };
            adj.entry(s.from).or_default().push(s);
        }
    }

    let mut used: HashSet<((i32, i32), (i32, i32))> = HashSet::new();
    let mut starts: Vec<(i32, i32)> = adj.keys().copied().collect();
    starts.sort_unstable();
    let mut loops = Vec::new();
    for &start in &starts {
        loop {
            let Some(&first) = adj[&start].iter().find(|s| !used.contains(&(s.from, s.to))) else {
                break;
            };
            used.insert((first.from, first.to));
            let mut steps = vec![first];
            let mut cur = first;
            while cur.to != start {
                let din = cur.dir();
                // Left-most-turn preference keeps diagonally-touching regions
                // as separate simple loops.
                let prefs = [(-din.1, din.0), din, (din.1, -din.0)];
                let mut next: Option<Step> = None;
                'outer: for d in prefs {
                    if let Some(cands) = adj.get(&cur.to) {
                        for &s in cands {
                            if s.dir() == d && !used.contains(&(s.from, s.to)) {
                                next = Some(s);
                                break 'outer;
                            }
                        }
                    }
                }
                let Some(nxt) = next else { break };
                used.insert((nxt.from, nxt.to));
                steps.push(nxt);
                cur = nxt;
            }
            loops.push(steps);
        }
    }
    loops
}

fn mm(p: (i32, i32)) -> Vec2 {
    Vec2::new(p.0 as f32 * GRID_PITCH, p.1 as f32 * GRID_PITCH)
}

/// Left normal of a unit direction (interior side, since material is on the left).
fn left_of(d: (i32, i32)) -> Vec2 {
    Vec2::new(-d.1 as f32, d.0 as f32)
}

fn dirv(d: (i32, i32)) -> Vec2 {
    Vec2::new(d.0 as f32, d.1 as f32)
}

// ── Outer profile authoring ──────────────────────────────────────────────────

/// One authored piece of the outer profile: the segment plus which cell's
/// peg-top it coincides with (`shared` ⇒ the edge welds to that peg ring).
/// `edge` is the source grid edge for body pieces (used to mirror pinch splits
/// into the peg profiles); corner/connector pieces carry `None`.
#[derive(Clone, Copy)]
struct OuterPiece {
    seg: Seg,
    shared: bool,
    edge: Option<GridEdge>,
}

/// Peg-top segments a cell shares with the outer profile: sides (by GridEdge)
/// and corner arcs (by lattice corner).
#[derive(Default)]
struct SharedWithPegs {
    sides: HashSet<GridEdge>,
    corners: HashSet<(i32, i32)>,
}

/// Author the outer wall profile for one boundary loop: pitch lines inset by
/// `inset(edge)`, convex corners rounded `OUTER_R` (only when both adjacent
/// insets equal `HALF_TOL` **and** both adjacent edges agree on walled-ness —
/// seam faces and mixed walled/open corners stay square, so open-face pinch
/// points always land on straight pieces), straight runs split at the peg
/// tangent points so peg-top edges weld with the wall's bottom ring.
fn author_outer_loop(
    steps: &[Step],
    inset: &dyn Fn(&GridEdge) -> f32,
    walled: &dyn Fn(&GridEdge) -> bool,
    shared: &mut SharedWithPegs,
) -> Vec<OuterPiece> {
    let n = steps.len();
    let mut pieces: Vec<OuterPiece> = Vec::new();
    for k in 0..n {
        let s = &steps[k];
        let s_next = &steps[(k + 1) % n];
        let d = dirv(s.dir());
        let nrm = left_of(s.dir());
        let ins = inset(&s.edge);
        let ins_next = inset(&s_next.edge);
        let from = mm(s.from);
        let to = mm(s.to);
        let is_std = (ins - HALF_TOL).abs() < 1e-6;

        // Body: the peg-tangent-to-peg-tangent span of this cell side.
        let a = from + d * PEG_TANGENT + nrm * ins;
        let b = to - d * PEG_TANGENT + nrm * ins;
        pieces.push(OuterPiece { seg: Seg::Line { a, b }, shared: is_std, edge: Some(s.edge) });
        if is_std {
            shared.sides.insert(s.edge);
        }

        // Corner piece between this step and the next, at `to`.
        let d1 = dirv(s_next.dir());
        let n1 = left_of(s_next.dir());
        let cross = d.x * d1.y - d.y * d1.x;
        let start = to - d * PEG_TANGENT + nrm * ins; // == b
        let end = to + d1 * PEG_TANGENT + n1 * ins_next;
        let both_std = is_std && (ins_next - HALF_TOL).abs() < 1e-6;
        let same_side = walled(&s.edge) == walled(&s_next.edge);
        if cross.abs() < 0.5 {
            // Straight run: single connector over the inter-cell gap.
            pieces.push(OuterPiece { seg: Seg::Line { a: start, b: end }, shared: false, edge: None });
        } else if cross > 0.0 && both_std && same_side {
            // Convex spec corner: the OUTER_R arc — identical to the corner
            // cell's peg-top arc, so it welds shared.
            let c = mm(s.to);
            let center = c + nrm * (ins + OUTER_R) + n1 * (ins_next + OUTER_R);
            let a0 = f32::atan2(start.y - center.y, start.x - center.x);
            let a1 = f32::atan2(end.y - center.y, end.x - center.x);
            let (a0, a1) = short_arc(a0, a1);
            pieces.push(OuterPiece {
                seg: Seg::Arc { a: start, b: end, center, radius: OUTER_R, a0, a1 },
                shared: true,
                edge: None,
            });
            shared.corners.insert(s.to);
        } else {
            // Concave corner, non-spec inset, or a mixed walled/open corner:
            // sharp — two lines meeting at the inset-line intersection.
            let q = mm(s.to) + nrm * ins + n1 * ins_next;
            pieces.push(OuterPiece { seg: Seg::Line { a: start, b: q }, shared: false, edge: None });
            pieces.push(OuterPiece { seg: Seg::Line { a: q, b: end }, shared: false, edge: None });
        }
    }
    pieces
}

fn short_arc(a0: f32, a1: f32) -> (f32, f32) {
    let mut d = a1 - a0;
    while d > PI {
        d -= 2.0 * PI;
    }
    while d < -PI {
        d += 2.0 * PI;
    }
    (a0, a0 + d)
}

// ── Open / seam faces ────────────────────────────────────────────────────────
//
// An OPEN perimeter edge keeps the spec outer profile but loses its wall above
// the floor; a split SEAM sits square on the pitch plane (inset 0) and is open
// unless a divider turns it into a wall. In both cases the effective cavity
// boundary along that face COINCIDES with the outer profile, so the cavity
// loop splices in the very same authored outer pieces (interned edges then
// weld automatically) and the wall above the floor becomes prism sectors over
// the region left between outer profile and cavity.

/// Geometric weld tolerance for the open-face bookkeeping (mm).
const W_EPS: f32 = 1e-3;

fn v2_eq(a: Vec2, b: Vec2) -> bool {
    (a - b).length() < W_EPS
}

/// One open/seam edge's stretch of grid line, in mm.
#[derive(Clone, Copy, Debug)]
struct OpenSpan {
    /// true → on line `y = coord` spanning `x ∈ [lo, hi]`; false → on `x = coord`.
    horiz: bool,
    coord: f32,
    lo: f32,
    hi: f32,
}

fn open_spans(cells: &[GridCell], walls: &EffectiveWalls) -> Vec<OpenSpan> {
    let set: HashSet<GridCell> = cells.iter().copied().collect();
    walls
        .open
        .iter()
        .filter(|e| edge_inside_cell(&set, e).is_some())
        .map(|e| {
            let p = GRID_PITCH;
            match e.orientation {
                Orientation::H => OpenSpan {
                    horiz: true,
                    coord: e.y as f32 * p,
                    lo: e.x as f32 * p,
                    hi: (e.x + 1) as f32 * p,
                },
                Orientation::V => OpenSpan {
                    horiz: false,
                    coord: e.x as f32 * p,
                    lo: e.y as f32 * p,
                    hi: (e.y + 1) as f32 * p,
                },
            }
        })
        .collect()
}

fn point_on_spans(spans: &[OpenSpan], pt: Vec2) -> bool {
    spans.iter().any(|s| {
        let (c, a) = if s.horiz { (pt.y, pt.x) } else { (pt.x, pt.y) };
        (c - s.coord).abs() < W_EPS && a > s.lo - W_EPS && a < s.hi + W_EPS
    })
}

/// A straight cavity-trace segment lying end-to-end on open/seam pitch lines.
fn seg_on_open(spans: &[OpenSpan], seg: &Seg) -> bool {
    let Seg::Line { a, b } = *seg else { return false };
    let horiz = (a.y - b.y).abs() < W_EPS;
    let vert = (a.x - b.x).abs() < W_EPS;
    if horiz == vert {
        return false;
    }
    let coord = if horiz { a.y } else { a.x };
    let (mut lo, mut hi) = if horiz { (a.x, b.x) } else { (a.y, b.y) };
    if lo > hi {
        std::mem::swap(&mut lo, &mut hi);
    }
    // The span must be covered by the union of open spans on this grid line.
    let mut covers: Vec<(f32, f32)> = spans
        .iter()
        .filter(|s| s.horiz == horiz && (s.coord - coord).abs() < W_EPS)
        .map(|s| (s.lo, s.hi))
        .collect();
    if covers.is_empty() {
        return false;
    }
    covers.sort_by(|x, y| x.0.total_cmp(&y.0));
    let mut cur = lo;
    for (l, h) in covers {
        if l <= cur + W_EPS && h > cur {
            cur = h;
        }
    }
    cur >= hi - W_EPS
}

/// The authored outer-profile loops of one piece, with per-piece consumption
/// marks (a consumed piece has been spliced into a cavity loop and no longer
/// carries wall above the floor).
struct OuterLoops {
    loops: Vec<Vec<OuterPiece>>,
    consumed: Vec<Vec<bool>>,
}

impl OuterLoops {
    fn new(loops: Vec<Vec<OuterPiece>>) -> OuterLoops {
        let consumed = loops.iter().map(|l| vec![false; l.len()]).collect();
        OuterLoops { loops, consumed }
    }

    /// Intersect the infinite line through `s` with direction `d` (unit) with
    /// the nearest straight outer piece; returns `(loop, point)`.
    fn pinch(&self, s: Vec2, d: Vec2) -> Option<(usize, Vec2)> {
        let mut best: Option<(usize, Vec2, f32)> = None;
        for (li, pieces) in self.loops.iter().enumerate() {
            for pc in pieces {
                let Seg::Line { a, b } = pc.seg else { continue };
                let e = b - a;
                let den = d.x * e.y - d.y * e.x;
                if den.abs() < 1e-6 {
                    continue; // parallel
                }
                // s + t·d = a + u·e
                let t = ((a.x - s.x) * e.y - (a.y - s.y) * e.x) / den;
                let u = ((a.x - s.x) * d.y - (a.y - s.y) * d.x) / den;
                let len = e.length();
                if !(-W_EPS..=1.0 + W_EPS / len).contains(&(u))
                    || !(-0.5..=PEG_TANGENT + 0.6).contains(&t)
                {
                    continue;
                }
                if best.map_or(true, |(_, _, bt)| t.abs() < bt.abs()) {
                    best = Some((li, s + d * t, t));
                }
            }
        }
        best.map(|(li, p, _)| (li, p))
    }

    /// Split the straight piece containing `p` (interior) at `p`, recording the
    /// station on shared body pieces so the peg profiles split identically.
    fn split_at(&mut self, li: usize, p: Vec2, peg_splits: &mut HashMap<GridEdge, Vec<f32>>) {
        let pieces = &mut self.loops[li];
        for i in 0..pieces.len() {
            let Seg::Line { a, b } = pieces[i].seg else { continue };
            if v2_eq(a, p) || v2_eq(b, p) {
                // Already a piece endpoint — check it is on THIS piece first.
                let d = b - a;
                let t = (p - a).dot(d) / d.length_squared();
                if (-0.1..=1.1).contains(&t) && (a + d * t - p).length() < W_EPS {
                    return;
                }
                continue;
            }
            let d = b - a;
            let t = (p - a).dot(d) / d.length_squared();
            if !(0.0..=1.0).contains(&t) || (a + d * t - p).length() > W_EPS {
                continue;
            }
            let pc = pieces[i];
            if pc.shared {
                if let Some(e) = pc.edge {
                    let station = match e.orientation {
                        Orientation::H => p.x,
                        Orientation::V => p.y,
                    };
                    peg_splits.entry(e).or_default().push(station);
                }
            }
            pieces[i] = OuterPiece { seg: Seg::Line { a, b: p }, ..pc };
            pieces.insert(i + 1, OuterPiece { seg: Seg::Line { a: p, b }, ..pc });
            let was = self.consumed[li][i];
            self.consumed[li].insert(i + 1, was);
            return;
        }
        panic!("open-face pinch point {p:?} not on any straight outer piece");
    }

    /// Walk pieces forward from the piece starting at `from` until the piece
    /// ending at `to`, marking them consumed; returns their segments.
    fn consume_walk(&mut self, li: usize, from: Vec2, to: Vec2) -> Vec<Seg> {
        let pieces = &self.loops[li];
        let n = pieces.len();
        let start = (0..n)
            .find(|&i| v2_eq(pieces[i].seg.start(), from))
            .unwrap_or_else(|| panic!("no outer piece starts at {from:?}"));
        let mut out = Vec::new();
        for k in 0..n {
            let i = (start + k) % n;
            out.push(self.loops[li][i].seg);
            self.consumed[li][i] = true;
            if v2_eq(self.loops[li][i].seg.end(), to) {
                return out;
            }
        }
        panic!("outer walk from {from:?} never reached {to:?}");
    }

    /// Consume an entire loop (a cavity that coincides with the whole profile).
    fn consume_all_near(&mut self, probe: Vec2) -> Vec<Seg> {
        for (li, pieces) in self.loops.iter().enumerate() {
            let hit = pieces.iter().any(|pc| match pc.seg {
                Seg::Line { a, b } => {
                    let d = b - a;
                    let t = ((probe - a).dot(d) / d.length_squared()).clamp(0.0, 1.0);
                    (a + d * t - probe).length() < W_EPS
                }
                _ => false,
            });
            if hit {
                for c in &mut self.consumed[li] {
                    *c = true;
                }
                return self.loops[li].iter().map(|pc| pc.seg).collect();
            }
        }
        panic!("no outer loop passes near {probe:?}");
    }
}

/// One resolved cavity loop: its final segments plus which of them ARE outer
/// profile pieces (coincident open/seam runs).
struct CavityLoop {
    segs: Vec<Seg>,
    coincident: Vec<bool>,
}

impl CavityLoop {
    fn untouched(segs: Vec<Seg>) -> CavityLoop {
        let n = segs.len();
        CavityLoop { segs, coincident: vec![false; n] }
    }
    fn touched(&self) -> bool {
        self.coincident.iter().any(|&c| c)
    }
}

/// Replace every maximal run of on-open-line segments in a shaped cavity loop
/// with the outer-profile path between its pinch points, trimming the
/// neighbouring segments onto the profile.
fn resolve_open_runs(
    shaped: Vec<Seg>,
    spans: &[OpenSpan],
    o: &mut OuterLoops,
    peg_splits: &mut HashMap<GridEdge, Vec<f32>>,
) -> CavityLoop {
    let on: Vec<bool> = shaped.iter().map(|s| seg_on_open(spans, s)).collect();
    if !on.iter().any(|&b| b) {
        return CavityLoop::untouched(shaped);
    }
    if on.iter().all(|&b| b) {
        // The cavity coincides with an entire outer loop (all faces open).
        // Probe with a point of the matching outer boundary: on a seam line the
        // profile passes through the trace point itself; on an open line it is
        // inset — pinch along the line's normal finds it either way.
        let s0 = shaped[0].start();
        let s1 = shaped[0].end();
        let d = s1 - s0;
        let nrm = Vec2::new(-d.y, d.x).normalize();
        let mid = (s0 + s1) * 0.5;
        let probe = [mid, mid + nrm * HALF_TOL, mid - nrm * HALF_TOL]
            .into_iter()
            .find(|&q| {
                o.loops.iter().any(|pieces| {
                    pieces.iter().any(|pc| match pc.seg {
                        Seg::Line { a, b } => {
                            let e = b - a;
                            let t = ((q - a).dot(e) / e.length_squared()).clamp(0.0, 1.0);
                            (a + e * t - q).length() < W_EPS
                        }
                        _ => false,
                    })
                })
            })
            .expect("fully-open cavity: no outer loop found near its boundary");
        let segs = o.consume_all_near(probe);
        let n = segs.len();
        return CavityLoop { segs, coincident: vec![true; n] };
    }

    // Rotate so index 0 is NOT on an open line (runs never wrap).
    let start = on.iter().position(|&b| !b).unwrap();
    let n = shaped.len();
    let mut segs: Vec<Seg> = (0..n).map(|k| shaped[(start + k) % n]).collect();
    let on: Vec<bool> = (0..n).map(|k| on[(start + k) % n]).collect();

    let mut out: Vec<(Seg, bool)> = Vec::new();
    let mut i = 0;
    while i < n {
        if !on[i] {
            out.push((segs[i], false));
            i += 1;
            continue;
        }
        let mut j = i;
        while j < n && on[j] {
            j += 1;
        }
        // Pinch the run's neighbours onto the outer profile. `prev` is the last
        // pushed segment; `next` is segs[j] (or the already-pushed out[0] when
        // the run ends the rotated list).
        let (prev_seg, _) = *out.last().expect("run preceded by a segment");
        let Seg::Line { a: pa, b: pb } = prev_seg else {
            panic!("open-run neighbour must be straight (got an arc before the run)");
        };
        let d_prev = (pa - pb).normalize();
        let (li_s, p_s) = o
            .pinch(pb, d_prev)
            .unwrap_or_else(|| panic!("no pinch for run start at {pb:?}"));
        o.split_at(li_s, p_s, peg_splits);

        let next_seg = if j < n { segs[j] } else { out[0].0 };
        let Seg::Line { a: na, b: nb } = next_seg else {
            panic!("open-run neighbour must be straight (got an arc after the run)");
        };
        let d_next = (nb - na).normalize();
        let (li_e, p_e) = o
            .pinch(na, d_next)
            .unwrap_or_else(|| panic!("no pinch for run end at {na:?}"));
        assert_eq!(li_s, li_e, "open run spans two outer loops");
        o.split_at(li_e, p_e, peg_splits);

        // Trim the neighbours onto the profile and splice the outer path in.
        if let Some((Seg::Line { b, .. }, _)) = out.last_mut() {
            *b = p_s;
        }
        if j < n {
            if let Seg::Line { a, .. } = &mut segs[j] {
                *a = p_e;
            }
        } else if let (Seg::Line { a, .. }, _) = &mut out[0] {
            *a = p_e;
        }
        for pc in o.consume_walk(li_s, p_s, p_e) {
            out.push((pc, true));
        }
        i = j;
    }
    let (segs, coincident) = out.into_iter().unzip();
    CavityLoop { segs, coincident }
}

/// Chain fragments (already-directed seg runs) into closed loops by endpoint.
fn chain_fragments(mut frags: Vec<Vec<Seg>>) -> Vec<Vec<Seg>> {
    let mut out = Vec::new();
    while let Some(mut cur) = frags.pop() {
        loop {
            let end = cur.last().unwrap().end();
            if v2_eq(end, cur[0].start()) {
                break;
            }
            let next = frags
                .iter()
                .position(|f| v2_eq(f[0].start(), end))
                .unwrap_or_else(|| panic!("wall-sector chain stuck at {end:?}"));
            cur.extend(frags.swap_remove(next));
        }
        out.push(cur);
    }
    out
}


/// One island of a cavity loop: a tower of material rising from the floor to
/// `top` (`None` = the rim, capped by the total_h face assembly; `Some(z)` =
/// a partial-height inner wall, capped immediately at that z).
#[derive(Clone, Debug)]
struct Island {
    segs: Vec<Seg>,
    top: Option<f32>,
}

/// One boundary-contact partial-height inner wall's pieces on a cavity loop.
#[derive(Clone, Debug)]
struct Notch {
    /// Region boundary pieces inside the quad — the taller face is absent
    /// below the wall top there (span top → total_h).
    contact: Vec<Seg>,
    /// Chained CCW cap loops at `top` (sides forward + contact forward).
    caps: Vec<Vec<Seg>>,
    top: f32,
}

/// A cavity loop notched by boundary-contact partial-height inner walls,
/// requiring the z-banded prism instead of the single-span flat builder.
#[derive(Clone, Debug)]
struct Banded {
    /// Notched floor outers (CCW), i.e. the original loop minus all wall
    /// quads. Each seg carries its provenance: `Some(k)` = a side of notch k
    /// (spans floor → that notch's top), `None` = original boundary (full
    /// span). Tags come from `split_region_by_quad`, never float matching.
    outline_a: Vec<Vec<(Seg, Option<usize>)>>,
    /// The original loop re-split at every contact point (rim + upper prism).
    outline_b: Vec<Seg>,
    notches: Vec<Notch>,
}

/// CCW footprint quad of an inner-wall segment (`None` when degenerate).
/// Widths clamp to ≥ 0.4 mm like the reference.
fn inner_wall_quad(w: &InnerWall) -> Option<Vec<Seg>> {
    let a = Vec2::new(w.x1, w.y1);
    let b = Vec2::new(w.x2, w.y2);
    let d = b - a;
    let len = d.length();
    if len < 0.1 {
        return None;
    }
    let u = d / len;
    let n = Vec2::new(-u.y, u.x);
    let hw = w.width.max(0.4) / 2.0;
    let (p0, p1, p2, p3) = (a - n * hw, b - n * hw, b + n * hw, a + n * hw);
    let segs = vec![
        Seg::Line { a: p0, b: p1 },
        Seg::Line { a: p1, b: p2 },
        Seg::Line { a: p2, b: p3 },
        Seg::Line { a: p3, b: p0 },
    ];
    Some(if loop_area(&segs) < 0.0 { reverse_loop(&segs) } else { segs })
}

// ── Peg authoring ────────────────────────────────────────────────────────────

/// Rounded-rect profile for a peg section, centred on cell `c`.
fn peg_profile(c: GridCell, w: f32, r: f32) -> Vec<Seg> {
    let cx = (c.x as f32 + 0.5) * GRID_PITCH;
    let cy = (c.y as f32 + 0.5) * GRID_PITCH;
    ccw_segs(&Sketch::rounded_rect(cx, cy, w, w, r))
}

/// Whether one peg-top ring segment is FREE (bounds the bridge underside)
/// rather than welding with the outer wall's bottom ring. Decided
/// geometrically so it stays correct after profile splitting.
fn peg_seg_free(s: &Seg, c: GridCell, shared: &SharedWithPegs) -> bool {
    let cx = (c.x as f32 + 0.5) * GRID_PITCH;
    let cy = (c.y as f32 + 0.5) * GRID_PITCH;
    match *s {
        Seg::Line { a, b } => {
            let m = (a + b) * 0.5;
            let horiz = (a.y - b.y).abs() < W_EPS;
            let e = if horiz {
                let y = if m.y < cy { c.y } else { c.y + 1 };
                GridEdge { x: c.x, y, orientation: Orientation::H }
            } else {
                let x = if m.x < cx { c.x } else { c.x + 1 };
                GridEdge { x, y: c.y, orientation: Orientation::V }
            };
            !shared.sides.contains(&e)
        }
        Seg::Arc { center, .. } => {
            // The arc centre sits PEG_TANGENT diagonally inside its lattice corner.
            let lx = if center.x > cx { c.x + 1 } else { c.x };
            let ly = if center.y > cy { c.y + 1 } else { c.y };
            !shared.corners.contains(&(lx, ly))
        }
    }
}

/// Split a peg profile's straight sides at the recorded stations of the cell's
/// side edges (mirroring pinch splits of shared outer body pieces, so peg-top
/// edges keep welding 1:1 with the wall's bottom ring). All three peg profiles
/// are split at the same stations, keeping the lofts' segment structures
/// aligned.
fn split_peg_profile(
    segs: Vec<Seg>,
    c: GridCell,
    splits: &HashMap<GridEdge, Vec<f32>>,
) -> Vec<Seg> {
    let [west, east, south, north] = cell_edges(c);
    let cx = (c.x as f32 + 0.5) * GRID_PITCH;
    let cy = (c.y as f32 + 0.5) * GRID_PITCH;
    let mut out = Vec::with_capacity(segs.len());
    for s in segs {
        let Seg::Line { a, b } = s else {
            out.push(s);
            continue;
        };
        let horiz = (a.y - b.y).abs() < W_EPS;
        let e = if horiz {
            if a.y < cy { south } else { north }
        } else if a.x < cx {
            west
        } else {
            east
        };
        let Some(stations) = splits.get(&e) else {
            out.push(s);
            continue;
        };
        // Stations are coordinates along the edge (x for H, y for V).
        let coord = |p: Vec2| if horiz { p.x } else { p.y };
        let (c0, c1) = (coord(a), coord(b));
        let mut cuts: Vec<f32> = stations
            .iter()
            .copied()
            .filter(|&t| (t - c0.min(c1)) > W_EPS && (c0.max(c1) - t) > W_EPS)
            .collect();
        cuts.sort_by(|x, y| {
            if c1 > c0 { x.total_cmp(y) } else { y.total_cmp(x) }
        });
        cuts.dedup_by(|x, y| (*x - *y).abs() < W_EPS);
        let mut prev = a;
        for t in cuts {
            let p = if horiz { Vec2::new(t, a.y) } else { Vec2::new(a.x, t) };
            out.push(Seg::Line { a: prev, b: p });
            prev = p;
        }
        out.push(Seg::Line { a: prev, b });
    }
    out
}

/// Magnet/screw pockets for one cell, drilled up from z = 0. Returns pocket rim
/// rings (holes in the peg's bottom cap).
fn cell_fasteners(b: &mut Builder, p: &Params, c: GridCell) -> Vec<RingEdges> {
    let mut out = Vec::new();
    if !p.magnet_holes && !p.screw_holes {
        return out;
    }
    let ccx = (c.x as f32 + 0.5) * GRID_PITCH;
    let ccy = (c.y as f32 + 0.5) * GRID_PITCH;
    for (dx, dy) in [(-1.0, -1.0), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0)] {
        let hx = ccx + dx * FASTENER_INSET;
        let hy = ccy + dy * FASTENER_INSET;
        out.push(match (p.magnet_holes, p.screw_holes) {
            (true, true) => drill_stepped(b, hx, hy, MAGNET_RADIUS, MAGNET_DEPTH, SCREW_RADIUS, SCREW_DEPTH),
            (true, false) => drill_blind(b, hx, hy, MAGNET_RADIUS, MAGNET_DEPTH),
            (false, true) => drill_blind(b, hx, hy, SCREW_RADIUS, SCREW_DEPTH),
            (false, false) => unreachable!(),
        });
    }
    out
}

fn drill_blind(b: &mut Builder, x: f32, y: f32, radius: f32, depth: f32) -> RingEdges {
    let profile = ccw_segs(&Sketch::circle(x, y, radius));
    let bottom = ring(b, &profile, 0.0);
    let top = ring(b, &profile, depth);
    wall_between(b, &profile, &profile, &bottom, &top, 0.0, depth, false);
    planar(b, depth, false, loop_of(&top, true), vec![]);
    bottom
}

fn drill_stepped(b: &mut Builder, x: f32, y: f32, r0: f32, d0: f32, r1: f32, d1: f32) -> RingEdges {
    let outer = ccw_segs(&Sketch::circle(x, y, r0));
    let inner = ccw_segs(&Sketch::circle(x, y, r1));
    let o_bot = ring(b, &outer, 0.0);
    let o_top = ring(b, &outer, d0);
    let i_top0 = ring(b, &inner, d0);
    let i_top1 = ring(b, &inner, d1);
    wall_between(b, &outer, &outer, &o_bot, &o_top, 0.0, d0, false);
    planar(b, d0, false, loop_of(&o_top, true), vec![loop_of(&i_top0, false)]);
    wall_between(b, &inner, &inner, &i_top0, &i_top1, d0, d1, false);
    planar(b, d1, false, loop_of(&i_top1, true), vec![]);
    o_bot
}

// ── Cavity plan (port of the reference `planCavity`) ─────────────────────────

const STRIP_OUT: f32 = 1.0; // harmless outward slop past the pitch line

/// The cell the given edge borders from inside the region.
fn edge_inside_cell(set: &HashSet<GridCell>, e: &GridEdge) -> Option<GridCell> {
    let (a, b) = match e.orientation {
        Orientation::V => (GridCell { x: e.x - 1, y: e.y }, GridCell { x: e.x, y: e.y }),
        Orientation::H => (GridCell { x: e.x, y: e.y - 1 }, GridCell { x: e.x, y: e.y }),
    };
    if set.contains(&a) { Some(a) } else if set.contains(&b) { Some(b) } else { None }
}

/// Wall layout → rectangles (positive cavity squares, negative material
/// strips), matching the reference semantics exactly.
fn plan_cavity(cells: &[GridCell], walls: &EffectiveWalls, wall_thickness: f32) -> (Vec<RectF>, Vec<RectF>) {
    let p = GRID_PITCH;
    let t = HALF_TOL + wall_thickness;
    let set: HashSet<GridCell> = cells.iter().copied().collect();

    let pos: Vec<RectF> = cells
        .iter()
        .map(|c| RectF::new(c.x as f32 * p, c.y as f32 * p, p, p))
        .collect();
    let mut neg: Vec<RectF> = Vec::new();

    for e in &walls.walled {
        let Some(inside) = edge_inside_cell(&set, e) else { continue };
        match e.orientation {
            Orientation::H => {
                let below = inside.y == e.y - 1;
                let y0 = e.y as f32 * p - if below { t } else { STRIP_OUT };
                neg.push(RectF::new(e.x as f32 * p, y0, p, t + STRIP_OUT));
            }
            Orientation::V => {
                let left = inside.x == e.x - 1;
                let x0 = e.x as f32 * p - if left { t } else { STRIP_OUT };
                neg.push(RectF::new(x0, e.y as f32 * p, t + STRIP_OUT, p));
            }
        }
    }

    for e in &walls.dividers {
        match e.orientation {
            Orientation::H => neg.push(RectF::new(
                e.x as f32 * p,
                e.y as f32 * p - wall_thickness / 2.0,
                p,
                wall_thickness,
            )),
            Orientation::V => neg.push(RectF::new(
                e.x as f32 * p - wall_thickness / 2.0,
                e.y as f32 * p,
                wall_thickness,
                p,
            )),
        }
    }

    // Concave-corner patches: where exactly one of a lattice point's four
    // quadrant cells is absent and BOTH perimeter edges bordering the absent
    // cell are walled, patch the diagonally opposite t×t quadrant.
    let mut lattice: HashSet<(i32, i32)> = HashSet::new();
    for c in cells {
        for l in [(c.x, c.y), (c.x + 1, c.y), (c.x, c.y + 1), (c.x + 1, c.y + 1)] {
            if !lattice.insert(l) {
                continue;
            }
            let quads = [(-1, -1), (0, -1), (-1, 0), (0, 0)];
            let absent: Vec<(i32, i32)> = quads
                .iter()
                .filter(|(qx, qy)| !set.contains(&GridCell { x: l.0 + qx, y: l.1 + qy }))
                .copied()
                .collect();
            if absent.len() != 1 {
                continue;
            }
            let (qx, qy) = absent[0];
            let v_edge = GridEdge { x: l.0, y: l.1 + qy, orientation: Orientation::V };
            let h_edge = GridEdge { x: l.0 + qx, y: l.1, orientation: Orientation::H };
            if !walls.walled.contains(&v_edge) || !walls.walled.contains(&h_edge) {
                continue;
            }
            neg.push(RectF::new(
                l.0 as f32 * p + if qx == 0 { -t } else { 0.0 },
                l.1 as f32 * p + if qy == 0 { -t } else { 0.0 },
                t,
                t,
            ));
        }
    }

    (pos, neg)
}

// ── Piece build ──────────────────────────────────────────────────────────────

/// Directed reference to an authored ring segment, for stitching bridge faces.
#[derive(Clone, Copy)]
struct DirEdge {
    start: VertexId,
    end: VertexId,
    entry: (EdgeId, bool),
}

/// Build one bin/piece into `b`; returns the floor-wall edges to blend.
/// `cells` is the piece's cell set; `bin_cells` the whole logical bin's (for
/// seam classification and the shared sloped-floor plane).
fn build_piece(
    b: &mut Builder,
    p: &Params,
    cells: &[GridCell],
    bin_cells: &[GridCell],
    walls: EffectiveWalls,
    slope: Option<BinSlope>,
) -> Vec<(EdgeId, f32)> {
    let total_h = p.total_height();
    let floor_z = BASE_TOTAL_HEIGHT + FLOOR_THICKNESS;
    let openish = !walls.open.is_empty();
    // A sloped floor is not combined with open/seam faces (its tilted floor
    // would need extra band geometry where it meets an opening); open pieces
    // build flat.
    let slope = if openish { None } else { slope };

    // 1) Outer profile from the boundary walk. Open perimeter edges keep the
    //    spec inset; split seams sit square on the pitch plane (inset 0).
    let seam = |e: &GridEdge| classify_edge(bin_cells, *e) == EdgeClass::Internal;
    let inset = |e: &GridEdge| -> f32 { if seam(e) { 0.0 } else { HALF_TOL } };
    let walled = |e: &GridEdge| walls.walled.contains(e);
    let loops = boundary_steps(cells);
    let mut shared = SharedWithPegs::default();
    let outer_loops: Vec<Vec<OuterPiece>> = loops
        .iter()
        .map(|steps| author_outer_loop(steps, &inset, &walled, &mut shared))
        .collect();
    let mut o = OuterLoops::new(outer_loops);
    let spans = open_spans(cells, &walls);
    let mut peg_splits: HashMap<GridEdge, Vec<f32>> = HashMap::new();

    // 2) Cavity plan → trace → shape → resolve against the outer profile.
    //    This runs BEFORE the pegs: pinch splits on shared outer pieces must
    //    be mirrored into the peg profiles so their edges keep welding 1:1.
    //    (A wall thicker than PEG_TANGENT would push pinch points into the
    //    peg-welded body pieces; clamp on open pieces.)
    let wt = if openish {
        p.wall_thickness.max(0.4).min(PEG_TANGENT - 0.6)
    } else {
        p.wall_thickness.max(0.4)
    };
    let (pos, neg) = plan_cavity(cells, &walls, wt);
    let traced = trace_rects(&pos, &neg);
    let cavity_depth = total_h - floor_z;
    let rc = p.cavity_corner_radius.max(0.0);
    // Fillet radius: bounded by the cavity depth; convex corner arcs must stay
    // strictly larger than the fillet (torus major radius > 0); zero under a
    // slope (the tilted floor keeps sharp corners).
    let mut fr = p.floor_fillet.min(cavity_depth - 0.05).max(0.0);
    if slope.is_some() {
        fr = 0.0;
    }
    if rc > 0.05 {
        fr = fr.min(rc - 0.02);
    } else {
        fr = 0.0;
    }

    // Assign hole loops (divider islands) to their containing cavity loop.
    let outers_traced: Vec<&TracedLoop> = traced.iter().filter(|l| !l.is_hole()).collect();
    let holes_of = |ol: &TracedLoop| -> Vec<&TracedLoop> {
        traced
            .iter()
            .filter(|l| l.is_hole() && point_in_rect_loop(l.pts[0], ol))
            .collect()
    };

    let mut planned: Vec<(CavityLoop, Vec<Island>, f32, Option<Banded>)> = Vec::new();
    for ol in &outers_traced {
        let (convex_r, concave_r) = if slope.is_some() { (0.0, 0.0) } else { (rc, fr) };
        let shape = if openish {
            shape_cavity_loop_open(ol, convex_r, concave_r, &spans)
        } else {
            shape_cavity_loop(ol, convex_r, concave_r)
        };
        let cl = if openish {
            resolve_open_runs(shape, &spans, &mut o, &mut peg_splits)
        } else {
            CavityLoop::untouched(shape)
        };
        let islands: Vec<Island> = holes_of(ol)
            .iter()
            .map(|il| Island { segs: shape_cavity_loop(il, rc, fr), top: None })
            .collect();
        // Free-form full-height inner walls: subtract each footprint quad
        // from this loop's region (outer + island holes) — a free-standing
        // wall comes back as a new hole (island tower), a wall reaching the
        // boundary notches the loop, one crossing it splits the compartment.
        // Open/seam loops don't combine with inner walls (skipped there).
        let full_walls: Vec<Vec<Seg>> = p
            .inner_walls
            .iter()
            .filter(|w| w.height.is_none_or(|h| h >= cavity_depth))
            .filter_map(inner_wall_quad)
            .collect();
        let mut entries: Vec<(CavityLoop, Vec<Island>, Option<Banded>)> = Vec::new();
        if cl.touched() || full_walls.is_empty() {
            entries.push((cl, islands, None));
        } else {
            let mut region: Vec<Vec<Seg>> = vec![cl.segs.clone()];
            region.extend(islands.iter().map(|il| reverse_loop(&il.segs)));
            for q in &full_walls {
                region = subtract_convex_quad(&region, q);
            }
            let mut outs: Vec<(Vec<Seg>, Vec<Island>)> = Vec::new();
            let mut hole_loops: Vec<Vec<Seg>> = Vec::new();
            for lp in region {
                if loop_area(&lp) > 0.0 {
                    outs.push((lp, Vec::new()));
                } else {
                    hole_loops.push(lp);
                }
            }
            // Attach each hole to its innermost containing outer.
            for h in hole_loops {
                let pt = h[0].start();
                let mut best: Option<usize> = None;
                for (i, (o, _)) in outs.iter().enumerate() {
                    if point_in_segs(pt, o)
                        && best
                            .is_none_or(|bi| loop_area(o).abs() < loop_area(&outs[bi].0).abs())
                    {
                        best = Some(i);
                    }
                }
                if let Some(bi) = best {
                    outs[bi].1.push(Island { segs: reverse_loop(&h), top: None });
                }
            }
            for (o, isls) in outs {
                entries.push((CavityLoop::untouched(o), isls, None));
            }
        }
        // Partial-height inner walls. Fully inside a loop → island towers
        // capped below the rim. Reaching the loop boundary → the z-banded
        // prism (`Banded`): the notched outline builds the floor band, the
        // contact runs build only above the wall top, and the wall is capped
        // at its own height. Restrictions: partial walls must not touch
        // islands or each other, and sloped bins skip boundary-contact ones.
        let partial_walls: Vec<(Vec<Seg>, f32)> = p
            .inner_walls
            .iter()
            .filter_map(|w| {
                let h = w.height?;
                if h >= cavity_depth {
                    return None;
                }
                Some((inner_wall_quad(w)?, floor_z + h.max(0.5)))
            })
            .collect();
        'walls: for (q, t) in &partial_walls {
            for (ecl, eisl, band) in &mut entries {
                if ecl.touched() {
                    continue;
                }
                let corners: Vec<Vec2> = q.iter().map(|s| s.start()).collect();
                let n_in = corners.iter().filter(|&&c| point_in_segs(c, &ecl.segs)).count();
                if n_in == 0 {
                    continue;
                }
                let clear = corners
                    .iter()
                    .all(|&c| eisl.iter().all(|il| !point_in_segs(c, &il.segs)));
                if !clear {
                    continue 'walls; // touches an island: unsupported
                }
                if n_in == 4 {
                    eisl.push(Island { segs: q.clone(), top: Some(*t) });
                    continue 'walls;
                }
                if slope.is_some() {
                    continue 'walls; // no banded prism under a tilted floor
                }
                let bd = band.get_or_insert_with(|| Banded {
                    outline_a: vec![ecl.segs.iter().map(|&s| (s, None)).collect()],
                    outline_b: ecl.segs.clone(),
                    notches: Vec::new(),
                });
                let ni = bd.notches.len();
                let sa = split_region_by_quad(&bd.outline_a, q, Some(ni));
                if sa.quad_inside.is_empty() || sa.inside.is_empty() {
                    continue 'walls;
                }
                let caps = chain_loops(
                    sa.quad_inside.iter().chain(sa.inside.iter()).copied().collect(),
                )
                .into_iter()
                .map(|lp| lp.into_iter().map(|(s, _)| s).collect())
                .collect();
                let mut kept = sa.outside.clone();
                kept.extend(sa.quad_inside.iter().map(|&(s, t)| (s.reversed(), t)));
                bd.outline_a = chain_loops(kept);
                let ob: Vec<Vec<(Seg, ())>> = vec![
                    std::mem::take(&mut bd.outline_b).into_iter().map(|s| (s, ())).collect(),
                ];
                let sb = split_region_by_quad(&ob, q, ());
                let mut b_all = sb.outside;
                b_all.extend(sb.inside);
                bd.outline_b = chain_loops(b_all)
                    .pop()
                    .unwrap_or_else(|| ob[0].clone())
                    .into_iter()
                    .map(|(s, _)| s)
                    .collect();
                bd.notches.push(Notch {
                    contact: sa.inside.into_iter().map(|(s, _)| s).collect(),
                    caps,
                    top: *t,
                });
                continue 'walls;
            }
        }
        // The fillet requires every corner arc the ball rolls inside to have
        // survived clamping (a sharp corner would break the tangent chain);
        // coincident runs always carry sharp pinch corners, so touched loops
        // build sharp. On the cavity loop the ball rolls inside at convex
        // arcs; on an island loop (material inside, ball outside) it rolls
        // inside at the non-convex arcs, so the guard inverts there.
        let clamp = |segs: &[Seg], ball_inside_convex: bool, loop_fr: &mut f32| {
            for s in segs {
                if let Seg::Arc { radius, .. } = s {
                    if *radius < *loop_fr + 0.02
                        && is_convex_arc(segs, s) == ball_inside_convex
                    {
                        *loop_fr = (*radius - 0.02).max(0.0);
                    }
                }
            }
            if fr > 0.0 && has_sharp_corner(segs) {
                *loop_fr = 0.0;
            }
        };
        for (cl, islands, banded) in entries {
            let mut loop_fr = fr;
            clamp(&cl.segs, true, &mut loop_fr);
            for isl in &islands {
                clamp(&isl.segs, false, &mut loop_fr);
            }
            // Banded loops always carry sharp notch corners in outline_a.
            if cl.touched() || banded.is_some() {
                loop_fr = 0.0;
            }
            planned.push((cl, islands, loop_fr, banded));
        }
    }

    // 3) Pegs per cell + their bottom caps with fastener pockets.
    let mut peg_tops: Vec<(GridCell, RingEdges, Vec<Seg>)> = Vec::new();
    for &c in cells {
        let s_bot = split_peg_profile(peg_profile(c, PEG_W_BOTTOM, PEG_R_BOTTOM), c, &peg_splits);
        let s_mid = split_peg_profile(peg_profile(c, PEG_W_MID, PEG_R_MID), c, &peg_splits);
        let s_top = split_peg_profile(peg_profile(c, PEG_W_TOP, OUTER_R), c, &peg_splits);
        let r0 = ring(b, &s_bot, 0.0);
        let r1 = ring(b, &s_mid, PEG_Z1);
        let r2 = ring(b, &s_mid, PEG_Z2);
        let r3 = ring(b, &s_top, PEG_HEIGHT);
        wall_between(b, &s_bot, &s_mid, &r0, &r1, 0.0, PEG_Z1, true);
        wall_between(b, &s_mid, &s_mid, &r1, &r2, PEG_Z1, PEG_Z2, true);
        wall_between(b, &s_mid, &s_top, &r2, &r3, PEG_Z2, PEG_HEIGHT, true);
        let pockets = cell_fasteners(b, p, c);
        let pocket_loops: Vec<Loop> = pockets.iter().map(|h| loop_of(h, false)).collect();
        planar(b, 0.0, false, loop_of(&r0, false), pocket_loops);
        peg_tops.push((c, r3, s_top));
    }

    // 4) Outer wall from the composite profile. The bottom ring's shared
    //    pieces intern onto the peg-top edges. Fully-walled pieces build the
    //    wall in one span; open/seam pieces build only the floor band here
    //    (PEG_HEIGHT → floor_z) — above the floor the wall exists only over
    //    the sectors left between profile and cavity, built in step 7.
    let mut bridge_rings: Vec<(RingEdges, Vec<bool>)> = Vec::new();
    let mut full_hi_rings: Vec<(Vec<Seg>, RingEdges)> = Vec::new();
    for pieces in &o.loops {
        let segs: Vec<Seg> = pieces.iter().map(|p| p.seg).collect();
        let shared_flags: Vec<bool> = pieces.iter().map(|p| p.shared).collect();
        let lo = ring(b, &segs, PEG_HEIGHT);
        // Outward for the outer boundary loop (CCW, area > 0); a hole loop of a
        // ring-shaped bin (CW) is an interior perimeter — its wall also faces
        // away from the material, which `wall_between` derives from the segs'
        // winding, so `outward = true` is correct for both.
        if openish {
            let mid = ring(b, &segs, floor_z);
            wall_between(b, &segs, &segs, &lo, &mid, PEG_HEIGHT, floor_z, true);
        } else {
            let hi = ring(b, &segs, total_h);
            wall_between(b, &segs, &segs, &lo, &hi, PEG_HEIGHT, total_h, true);
            full_hi_rings.push((segs, hi));
        }
        bridge_rings.push((lo, shared_flags));
    }

    // 5) Bridge underside faces at PEG_HEIGHT: stitch the free (non-welded)
    //    peg-top segments (forward) with the free outer-profile pieces
    //    (reversed) into loops.
    let mut free: Vec<DirEdge> = Vec::new();
    for (c, r3, s_top) in &peg_tops {
        for (k, &(e, d)) in r3.edges.iter().enumerate() {
            if peg_seg_free(&s_top[k], *c, &shared) {
                let k1 = (k + 1) % r3.verts.len();
                free.push(DirEdge { start: r3.verts[k], end: r3.verts[k1], entry: (e, d) });
            }
        }
    }
    for (lo, shared_flags) in &bridge_rings {
        for (k, &(e, d)) in lo.edges.iter().enumerate() {
            if !shared_flags[k] {
                let k1 = (k + 1) % lo.verts.len();
                free.push(DirEdge { start: lo.verts[k1], end: lo.verts[k], entry: (e, !d) });
            }
        }
    }
    for (outer, holes) in stitch_loops(&free, b) {
        planar(b, PEG_HEIGHT, false, outer, holes);
    }

    // 6) Cavities: prisms + floors (+ fillet chains). Touched loops (with
    //    coincident open/seam runs) build only their floor cap and island
    //    towers here — their wall faces belong to the sector loops of step 7.
    let mut blends: Vec<(EdgeId, f32)> = Vec::new();
    let mut rim_holes: Vec<(Vec<Seg>, Loop)> = Vec::new();
    let mut island_tops: Vec<(Vec<Seg>, RingEdges)> = Vec::new();
    let mut touched: Vec<CavityLoop> = Vec::new();

    for (cl, island_shapes, loop_fr, banded) in planned {
        if !cl.touched() {
            if let Some(bd) = banded {
                let (bw, tops) = build_cavity_banded(b, &bd, &island_shapes, floor_z, total_h);
                island_tops.extend(tops);
                blends.extend(bw);
                // Rim hole from the re-split outline so its edges pair with
                // the contact walls' top edges.
                let top_ring = ring(b, &bd.outline_b, total_h);
                rim_holes.push((bd.outline_b.clone(), loop_of(&top_ring, true)));
                continue;
            }
            match slope {
                Some(sl) => {
                    island_tops.extend(build_cavity_sloped(
                        b, bin_cells, &cl.segs, &island_shapes, floor_z, total_h, sl,
                    ));
                }
                None => {
                    let (fwe, tops) =
                        build_cavity_flat(b, &cl.segs, &island_shapes, floor_z, total_h);
                    island_tops.extend(tops);
                    if loop_fr > 0.01 {
                        blends.extend(fwe.into_iter().map(|e| (e, loop_fr)));
                    }
                }
            }
            // Rim hole for this cavity's opening.
            let top_ring = ring(b, &cl.segs, total_h);
            rim_holes.push((cl.segs.clone(), loop_of(&top_ring, true)));
        } else {
            let r_lo = ring(b, &cl.segs, floor_z);
            let mut floor_holes: Vec<Loop> = Vec::new();
            for isl in &island_shapes {
                let i_lo = ring(b, &isl.segs, floor_z);
                let i_hi = ring(b, &isl.segs, total_h);
                wall_between(b, &isl.segs, &isl.segs, &i_lo, &i_hi, floor_z, total_h, true);
                island_tops.push((isl.segs.clone(), i_hi));
                // Reversed: the tower wall traverses its bottom ring forward.
                floor_holes.push(loop_of(&i_lo, false));
            }
            // Forward traversal: the floor band's top edges and the sector
            // walls' bottom edges both take the reversed direction here (the
            // cavity has no wall of its own above the floor on this loop).
            planar(b, floor_z, true, loop_of(&r_lo, true), floor_holes);
            touched.push(cl);
        }
    }

    // 7) Faces at total_h. On open/seam pieces the wall exists only over the
    //    sectors left between outer profile and cavities.
    let mut top_walls: Vec<(Vec<Seg>, RingEdges)> = full_hi_rings;
    if openish {
        top_walls = build_wall_sectors(b, &o, &touched, floor_z, total_h);
    }

    // Assemble total_h faces: wall tops and island caps are face outers;
    // compartment openings (and a ring bin's inner-perimeter wall top) are
    // holes of the INNERMOST containing face — a compartment inside an island
    // belongs to the island's cap, not the rim.
    let mut cap_outers: Vec<(f32, Vec<Seg>, Loop, Vec<Loop>)> = Vec::new();
    let mut cap_holes: Vec<(Vec2, Loop)> = Vec::new();
    for (segs, hi) in top_walls {
        let a = loop_area(&segs);
        let lp = loop_of(&hi, true);
        if a > 0.0 {
            cap_outers.push((a, segs, lp, Vec::new()));
        } else {
            cap_holes.push((segs[0].start(), lp));
        }
    }
    for (segs, hi) in island_tops {
        let a = loop_area(&segs).abs();
        let lp = loop_of(&hi, true);
        cap_outers.push((a, segs, lp, Vec::new()));
    }
    for (segs, lp) in rim_holes {
        cap_holes.push((segs[0].start(), lp));
    }
    for (pt, lp) in cap_holes {
        let mut best: Option<usize> = None;
        for (i, (a, segs, _, _)) in cap_outers.iter().enumerate() {
            if point_in_segs(pt, segs)
                && best.is_none_or(|bi| *a < cap_outers[bi].0)
            {
                best = Some(i);
            }
        }
        let bi = best.expect("total_h hole without a containing face");
        cap_outers[bi].3.push(lp);
    }
    for (_, _, outer, holes) in cap_outers {
        planar(b, total_h, true, outer, holes);
    }

    blends
}

/// Wall sectors of an open/seam piece: the region between outer profile and
/// cavities, bounded by the unconsumed outer fragments plus the reversed
/// non-coincident cavity fragments, chained into closed loops. Each sector
/// gets prism walls (floor_z → total_h); its boundary edges weld with the
/// floor band top, the cavity floor caps, and the rim by interning. Returns
/// `(loop segs, top ring)` per sector for the rim assembly.
fn build_wall_sectors(
    b: &mut Builder,
    o: &OuterLoops,
    touched: &[CavityLoop],
    floor_z: f32,
    total_h: f32,
) -> Vec<(Vec<Seg>, RingEdges)> {
    let mut frags: Vec<Vec<Seg>> = Vec::new();
    for (li, pieces) in o.loops.iter().enumerate() {
        let cons = &o.consumed[li];
        let n = pieces.len();
        if cons.iter().all(|&c| c) {
            continue;
        }
        if !cons.iter().any(|&c| c) {
            frags.push(pieces.iter().map(|p| p.seg).collect());
            continue;
        }
        let start = cons.iter().position(|&c| c).unwrap();
        let mut run: Vec<Seg> = Vec::new();
        for k in 1..=n {
            let idx = (start + k) % n;
            if !cons[idx] {
                run.push(pieces[idx].seg);
            } else if !run.is_empty() {
                frags.push(std::mem::take(&mut run));
            }
        }
        if !run.is_empty() {
            frags.push(run);
        }
    }
    for cl in touched {
        let n = cl.segs.len();
        if cl.coincident.iter().all(|&c| c) {
            continue; // cavity fills the whole profile: no wall left at all
        }
        let start = cl.coincident.iter().position(|&c| c).unwrap();
        let mut run: Vec<Seg> = Vec::new();
        let flush = |run: &mut Vec<Seg>, frags: &mut Vec<Vec<Seg>>| {
            if !run.is_empty() {
                frags.push(run.iter().rev().map(|s| s.reversed()).collect());
                run.clear();
            }
        };
        for k in 1..=n {
            let idx = (start + k) % n;
            if !cl.coincident[idx] {
                run.push(cl.segs[idx]);
            } else {
                flush(&mut run, &mut frags);
            }
        }
        flush(&mut run, &mut frags);
    }

    let sectors = chain_fragments(frags);
    let mut sector_rings: Vec<(Vec<Seg>, RingEdges)> = Vec::new();
    for sl in &sectors {
        let lo = ring(b, sl, floor_z);
        let hi = ring(b, sl, total_h);
        wall_between(b, sl, sl, &lo, &hi, floor_z, total_h, true);
        sector_rings.push((sl.clone(), hi));
    }
    sector_rings
}

/// Shape a traced cavity loop for an open/seam piece: corner rounding is
/// suppressed at every corner sitting on an open pitch line — the outer
/// profile supplies the shape there (spliced in by `resolve_open_runs`), and
/// pinch corners must stay square for the reveal planes to land on straight
/// pieces.
fn shape_cavity_loop_open(lp: &TracedLoop, rc: f32, rf: f32, spans: &[OpenSpan]) -> Vec<Seg> {
    let n = lp.pts.len();
    let suppressed: Vec<bool> = lp.pts.iter().map(|&p| point_on_spans(spans, p)).collect();
    let inset = |_: usize, _: Vec2, _: Vec2| 0.0f32;
    let radius = move |i: usize, convex: bool| {
        if suppressed[i] {
            return 0.0;
        }
        let mut r = if convex { rc } else { rf };
        // A corner next to a suppressed one must leave a stub of straight line
        // on the shared edge: the run-replacement pinches that line onto the
        // outer profile, and pinch neighbours must be straight.
        let prev = (i + n - 1) % n;
        let next = (i + 1) % n;
        // 0.35 = the HALF_TOL pinch trim plus margin.
        if suppressed[prev] {
            r = r.min(((lp.pts[i] - lp.pts[prev]).length() - 0.35).max(0.0));
        }
        if suppressed[next] {
            r = r.min(((lp.pts[next] - lp.pts[i]).length() - 0.35).max(0.0));
        }
        r
    };
    shape_loop(lp, &LoopStyle { inset: &inset, radius: &radius })
}

/// Shape a traced cavity loop: convex corners rounded `rc`, concave rounded
/// `rf` (both clamped by `shape_loop` against edge lengths).
fn shape_cavity_loop(lp: &TracedLoop, rc: f32, rf: f32) -> Vec<Seg> {
    let inset = |_: usize, _: Vec2, _: Vec2| 0.0f32;
    let radius = move |_: usize, convex: bool| if convex { rc } else { rf };
    let segs = shape_loop(lp, &LoopStyle { inset: &inset, radius: &radius });
    // Cavity faces expect CCW authoring; hole (island) loops trace CW.
    if loop_area(&segs) < 0.0 { reverse_loop(&segs) } else { segs }
}

fn has_sharp_corner(shape: &[Seg]) -> bool {
    // Consecutive Line/Line pairs are sharp corners.
    let n = shape.len();
    (0..n).any(|i| {
        matches!(shape[i], Seg::Line { .. }) && matches!(shape[(i + 1) % n], Seg::Line { .. })
    })
}

/// Whether an arc bulges away from the loop interior (a convex cavity corner:
/// material outside the arc). For CCW loops that is a CCW (a1 > a0) arc.
fn is_convex_arc(shape: &[Seg], s: &Seg) -> bool {
    let ccw = loop_area(shape) > 0.0;
    match s {
        Seg::Arc { a0, a1, .. } => (a1 > a0) == ccw,
        _ => false,
    }
}

/// Flat cavity: vertical walls, planar floor (with island holes), island
/// towers. Returns the floor-wall edges (cavity ring + island rings) and the
/// island top rings (capped by the total_h face assembly, which knows about
/// compartments nested inside islands).
fn build_cavity_flat(
    b: &mut Builder,
    shape: &[Seg],
    islands: &[Island],
    floor_z: f32,
    total_h: f32,
) -> (Vec<EdgeId>, Vec<(Vec<Seg>, RingEdges)>) {
    // The cavity is a void: the compartment profile, minus each island tower
    // over its own height. Partial-height islands need no separate capping
    // code — the band machinery caps them where their slab ends. The rim is
    // declared open because step 7 supplies that face.
    let mut ops = vec![(Op::Union, Slab::new(vec![shape.to_vec()], floor_z, total_h))];
    for isl in islands {
        ops.push((
            Op::Difference,
            Slab::new(vec![isl.segs.clone()], floor_z, isl.top.unwrap_or(total_h)),
        ));
    }
    emit_slabs(b, &ops, &SlabOpts { cavity: true, open_at: vec![total_h] })
        .expect("flat cavity slab stack");

    // Re-derive what callers need. `seg_edge`/`ring` are interned, so these
    // are the very edges the stack just emitted.
    let mut fwe: Vec<EdgeId> = shape.iter().map(|s| seg_edge(b, s, floor_z).0).collect();
    let mut tops: Vec<(Vec<Seg>, RingEdges)> = Vec::new();
    for isl in islands {
        if isl.top.is_none() {
            fwe.extend(isl.segs.iter().map(|s| seg_edge(b, s, floor_z).0));
            // The stack emitted this island as a CW *hole* of the void, so
            // hand step 7 a ring wound that way — `loop_of(.., true)` there
            // must oppose the wall top the stack just built.
            tops.push((isl.segs.clone(), ring(b, &reverse_loop(&isl.segs), total_h)));
        }
    }
    (fwe, tops)
}

/// Z-banded cavity for a loop notched by boundary-contact partial-height
/// inner walls. The notched outers (`outline_a`) carry the floor band and the
/// full-height walls; the notch sides rise only to their wall's top; the
/// contact runs exist only from the wall top up; each wall is capped at its
/// own height; the rim opening uses the re-split original outline. Vertical
/// edges at corners where spans differ are split at every breakpoint so the
/// shared edges pair 1:1. No floor-wall fillet edges are returned (banded
/// loops always build sharp).
fn build_cavity_banded(
    b: &mut Builder,
    bd: &Banded,
    islands: &[Island],
    floor_z: f32,
    total_h: f32,
) -> (Vec<(EdgeId, f32)>, Vec<(Vec<Seg>, RingEdges)>) {
    let key = |p: Vec2| ((p.x * 1024.0).round() as i64, (p.y * 1024.0).round() as i64);

    // The wall top runs out into the taller face with a rolling-ball blend of
    // radius min(TRANSITION_R, headroom). It is NOT built here: the prism is
    // built sharp and the cap↔contact edges are handed to `blend_edges`,
    // which trims the blend against the wall's side planes itself (the exact
    // cylinder/plane runout ellipse).
    const TRANSITION_R: f32 = 4.0;

    // 1) Wall spans per segment. Sides run floor → wall top, contacts run
    //    wall top → rim; the blend eats into both afterwards.
    enum Kind {
        Plain { za: f32, zb: f32 },
        Side { top: f32 }, // notch side: floor → wall top
    }
    let mut faces: Vec<(Seg, Kind)> = Vec::new();
    for lp in &bd.outline_a {
        for &(s, tag) in lp {
            match tag {
                Some(ni) => faces.push((s, Kind::Side { top: bd.notches[ni].top })),
                Option::None => faces.push((s, Kind::Plain { za: floor_z, zb: total_h })),
            }
        }
    }
    for n in &bd.notches {
        for s in &n.contact {
            faces.push((*s, Kind::Plain { za: n.top, zb: total_h }));
        }
    }
    let span = |kind: &Kind| -> (f32, f32) {
        match kind {
            Kind::Plain { za, zb } => (*za, *zb),
            Kind::Side { top } => (floor_z, *top),
        }
    };
    let mut breaks: HashMap<(i64, i64), Vec<f32>> = HashMap::new();
    for (s, kind) in &faces {
        let (za, zb) = span(kind);
        for at_start in [true, false] {
            let pt = if at_start { s.start() } else { s.end() };
            let e = breaks.entry(key(pt)).or_default();
            e.push(za);
            e.push(zb);
        }
    }
    let get_breaks = |breaks: &HashMap<(i64, i64), Vec<f32>>, p: Vec2| -> Vec<f32> {
        breaks.get(&key(p)).cloned().unwrap_or_default()
    };

    // Every face is a plain span now — the blend is the kernel's job.
    for (s, kind) in &faces {
        let (za, zb) = span(kind);
        let bl = get_breaks(&breaks, s.start());
        let br = get_breaks(&breaks, s.end());
        wall_seg(b, s, za, zb, &bl, &br, false);
    }

    // 2) Island towers + floor caps per notched outer.
    let mut tops: Vec<(Vec<Seg>, RingEdges)> = Vec::new();
    let mut floor_holes: Vec<Vec<Loop>> = vec![Vec::new(); bd.outline_a.len()];
    for isl in islands {
        let t = isl.top.unwrap_or(total_h);
        let i_lo = ring(b, &isl.segs, floor_z);
        let i_hi = ring(b, &isl.segs, t);
        wall_between(b, &isl.segs, &isl.segs, &i_lo, &i_hi, floor_z, t, true);
        if isl.top.is_some() {
            planar(b, t, true, loop_of(&i_hi, true), Vec::new());
        } else {
            tops.push((isl.segs.clone(), i_hi));
        }
        let pt = isl.segs[0].start();
        let owner = bd
            .outline_a
            .iter()
            .position(|lp| {
                let bare: Vec<Seg> = lp.iter().map(|&(s, _)| s).collect();
                point_in_segs(pt, &bare)
            })
            .unwrap_or(0);
        floor_holes[owner].push(loop_of(&i_lo, false));
    }
    for (li, lp) in bd.outline_a.iter().enumerate() {
        let bare: Vec<Seg> = lp.iter().map(|&(s, _)| s).collect();
        let r_lo = ring(b, &bare, floor_z);
        planar(b, floor_z, true, loop_of(&r_lo, false), std::mem::take(&mut floor_holes[li]));
    }

    // 3) Wall-top caps (reversed traversal, opposing the side walls' top
    //    edges and the contact walls' bottom edges — same pattern as the
    //    floor cap).
    for n in &bd.notches {
        for cap_lp in &n.caps {
            let r = ring(b, cap_lp, n.top);
            planar(b, n.top, true, loop_of(&r, false), Vec::new());
        }
    }

    // 4) Hand the cap↔contact edges to the kernel to blend. `seg_edge` is
    //    interned, so this re-derives the very edges the cap and the contact
    //    wall already share — no float matching. `blend_edges` runs each
    //    chain out against the wall's side planes on its own.
    let mut blends: Vec<(EdgeId, f32)> = Vec::new();
    for n in &bd.notches {
        let r = (total_h - n.top).min(TRANSITION_R);
        if r < 0.05 {
            continue;
        }
        for s in &n.contact {
            blends.push((seg_edge(b, s, n.top).0, r));
        }
    }
    (blends, tops)
}

/// Sloped cavity: single tilted plane floor over the bin's bounding box, sharp
/// (line-only) walls rising to the rim.
fn build_cavity_sloped(
    b: &mut Builder,
    bin_cells: &[GridCell],
    shape: &[Seg],
    islands: &[Island],
    floor_z: f32,
    total_h: f32,
    slope: BinSlope,
) -> Vec<(Vec<Seg>, RingEdges)> {
    let (ux, uy) = uphill_unit(slope.dir);
    let (min_a, span) = slope_span(bin_cells, ux, uy);
    let m = slope.angle_deg.to_radians().tan().clamp(0.0, 3.0);
    let cavity_depth = total_h - floor_z;
    let h_max = (m * span).min(cavity_depth - 0.5).max(0.0);
    let eff_m = if span > 1e-6 { h_max / span } else { 0.0 };
    let z_of = move |pt: Vec2| floor_z + eff_m * (ux * pt.x + uy * pt.y - min_a);

    let bottom = ring_z(b, shape, &z_of);
    let top = ring(b, shape, total_h);
    wall_between(b, shape, shape, &bottom, &top, floor_z, total_h, false);

    let mut tops: Vec<(Vec<Seg>, RingEdges)> = Vec::new();
    let mut floor_holes: Vec<Loop> = Vec::new();
    for isl in islands {
        // A partial top submerged under the tilted floor builds full height.
        let slope_max = isl
            .segs
            .iter()
            .map(|s| z_of(s.start()))
            .fold(floor_z, f32::max);
        let t = isl.top.filter(|&t| t > slope_max + 0.2).unwrap_or(total_h);
        let i_lo = ring_z(b, &isl.segs, &z_of);
        let i_hi = ring(b, &isl.segs, t);
        wall_between(b, &isl.segs, &isl.segs, &i_lo, &i_hi, floor_z, t, true);
        if t < total_h {
            planar(b, t, true, loop_of(&i_hi, true), Vec::new());
        } else {
            tops.push((isl.segs.clone(), i_hi));
        }
        floor_holes.push(loop_of(&i_lo, false));
    }

    let origin = b.point(bottom.verts[0]);
    let normal = Vec3::new(-eff_m * ux, -eff_m * uy, 1.0).normalize();
    let surface = Surface::plane(origin, normal);
    b.face(surface, true, loop_of(&bottom, false), floor_holes);
    tops
}

fn slope_span(cells: &[GridCell], ux: f32, uy: f32) -> (f32, f32) {
    let mut min_a = f32::INFINITY;
    let mut max_a = f32::NEG_INFINITY;
    for c in cells {
        for (dx, dy) in [(0, 0), (1, 0), (0, 1), (1, 1)] {
            let x = (c.x + dx) as f32 * GRID_PITCH;
            let y = (c.y + dy) as f32 * GRID_PITCH;
            let a = ux * x + uy * y;
            min_a = min_a.min(a);
            max_a = max_a.max(a);
        }
    }
    (min_a, (max_a - min_a).max(1e-6))
}

/// Uphill unit vector (XY) for a slope whose low side is `dir`.
fn uphill_unit(dir: SlopeDir) -> (f32, f32) {
    match dir {
        SlopeDir::PlusX => (-1.0, 0.0),
        SlopeDir::MinusX => (1.0, 0.0),
        SlopeDir::PlusY => (0.0, -1.0),
        SlopeDir::MinusY => (0.0, 1.0),
    }
}

/// A profile ring realised at a per-vertex height (tilted bottoms). Lines only.
fn ring_z(b: &mut Builder, segs: &[Seg], z_of: &dyn Fn(Vec2) -> f32) -> RingEdges {
    let n = segs.len();
    let verts: Vec<VertexId> = segs
        .iter()
        .map(|s| {
            let p = s.start();
            b.vertex(vec3_of(p.x, p.y, z_of(p)))
        })
        .collect();
    let mut edges = Vec::with_capacity(n);
    for k in 0..n {
        let k1 = (k + 1) % n;
        edges.push(match segs[k] {
            Seg::Line { .. } => b.line(verts[k], verts[k1]),
            Seg::Arc { center, radius, a0, a1, .. } => {
                let cz = z_of(Vec2::new(center.x, center.y));
                b.arc(verts[k], verts[k1], vec3_of(center.x, center.y, cz), Vec3::Z, radius, Vec3::X, a0, a1)
            }
        });
    }
    RingEdges { verts, edges }
}

/// Ray-cast point-in-polygon on a traced rectilinear loop.
fn point_in_rect_loop(pt: Vec2, lp: &TracedLoop) -> bool {
    let n = lp.pts.len();
    let mut inside = false;
    for i in 0..n {
        let a = lp.pts[i];
        let b = lp.pts[(i + 1) % n];
        if (a.y > pt.y) != (b.y > pt.y) {
            let x = a.x + (pt.y - a.y) / (b.y - a.y) * (b.x - a.x);
            if x > pt.x {
                inside = !inside;
            }
        }
    }
    inside
}

// ── Bridge stitching ─────────────────────────────────────────────────────────

/// Stitch directed edges into closed loops by chaining end → start vertices,
/// then group hole loops under the outer loop containing them. Returns
/// `(outer, holes)` pairs ready for planar faces.
fn stitch_loops(free: &[DirEdge], b: &Builder) -> Vec<(Loop, Vec<Loop>)> {
    let mut by_start: HashMap<VertexId, Vec<usize>> = HashMap::new();
    for (i, de) in free.iter().enumerate() {
        by_start.entry(de.start).or_default().push(i);
    }
    let mut used = vec![false; free.len()];
    let mut loops: Vec<Vec<usize>> = Vec::new();
    for i in 0..free.len() {
        if used[i] {
            continue;
        }
        let mut seq = vec![i];
        used[i] = true;
        let mut cur = free[i].end;
        while cur != free[i].start {
            let Some(cands) = by_start.get(&cur) else { break };
            let Some(&j) = cands.iter().find(|&&j| !used[j]) else { break };
            used[j] = true;
            seq.push(j);
            cur = free[j].end;
        }
        if cur == free[i].start && seq.len() >= 2 {
            loops.push(seq);
        }
    }
    // Group loops geometrically: a loop strictly contained in another (an
    // interior peg's top ring inside the surrounding bridge region, or the
    // inner perimeter of a ring-shaped bin) is a hole of its innermost
    // container, not a face of its own — emitting it standalone would cap the
    // peg with a disk that overlaps the real bridge face.
    let polys: Vec<Vec<(f32, f32)>> = loops
        .iter()
        .map(|seq| {
            seq.iter()
                .map(|&j| {
                    let p = b.point(free[j].start);
                    (p.x, p.y)
                })
                .collect()
        })
        .collect();
    let inside = |pt: (f32, f32), poly: &[(f32, f32)]| -> bool {
        let mut c = false;
        let n = poly.len();
        for i in 0..n {
            let (x0, y0) = poly[i];
            let (x1, y1) = poly[(i + 1) % n];
            if (y0 > pt.1) != (y1 > pt.1)
                && pt.0 < (x1 - x0) * (pt.1 - y0) / (y1 - y0) + x0
            {
                c = !c;
            }
        }
        c
    };
    // Containers of each loop, and nesting depth = number of containers.
    let containers: Vec<Vec<usize>> = (0..loops.len())
        .map(|i| {
            (0..loops.len())
                .filter(|&j| j != i && inside(polys[i][0], &polys[j]))
                .collect()
        })
        .collect();
    let mk_loop = |seq: &[usize]| {
        Loop::new(seq.iter().map(|&j| free[j].entry).collect::<Vec<_>>())
    };
    let mut out: Vec<(usize, Loop, Vec<Loop>)> = Vec::new();
    for (i, seq) in loops.iter().enumerate() {
        if containers[i].len() % 2 == 0 {
            out.push((i, mk_loop(seq), Vec::new()));
        }
    }
    for (i, seq) in loops.iter().enumerate() {
        if containers[i].len() % 2 == 1 {
            // Innermost containing outer = the container with the most
            // containers itself.
            let owner = *containers[i]
                .iter()
                .filter(|&&j| containers[j].len() % 2 == 0)
                .max_by_key(|&&j| containers[j].len())
                .expect("hole loop without containing outer");
            let slot = out.iter_mut().find(|(o, _, _)| *o == owner).unwrap();
            slot.2.push(mk_loop(seq));
        }
    }
    out.into_iter().map(|(_, o, h)| (o, h)).collect()
}

// ── Baseplate ────────────────────────────────────────────────────────────────

/// A "lite" baseplate over the union of every bin's cells: a slab
/// (0 → PEG_HEIGHT) with a peg-shaped through-socket per cell. Full-pitch
/// outline (no 0.25 clearance), convex corners rounded `OUTER_R`.
fn build_baseplate(p: &Params) -> Solid {
    let cells = p.all_cells();
    if cells.is_empty() {
        return Builder::new().build();
    }
    let mut b = Builder::new();

    let traced = trace_rects(
        &cells
            .iter()
            .map(|c| RectF::new(c.x as f32 * GRID_PITCH, c.y as f32 * GRID_PITCH, GRID_PITCH, GRID_PITCH))
            .collect::<Vec<_>>(),
        &[],
    );
    let inset = |_: usize, _: Vec2, _: Vec2| 0.0f32;
    let radius = |_: usize, convex: bool| if convex { OUTER_R } else { 0.0 };
    let mut outer_top: Vec<Loop> = Vec::new();
    let mut outer_bot: Vec<Loop> = Vec::new();
    let mut first_outer_top: Option<Loop> = None;
    let mut first_outer_bot: Option<Loop> = None;
    for lp in &traced {
        let segs = {
            let s = shape_loop(lp, &LoopStyle { inset: &inset, radius: &radius });
            if loop_area(&s) < 0.0 && !lp.is_hole() { reverse_loop(&s) } else { s }
        };
        let r_bot = ring(&mut b, &segs, 0.0);
        let r_top = ring(&mut b, &segs, PEG_HEIGHT);
        wall_between(&mut b, &segs, &segs, &r_bot, &r_top, 0.0, PEG_HEIGHT, true);
        if lp.is_hole() {
            outer_top.push(loop_of(&r_top, true));
            outer_bot.push(loop_of(&r_bot, false));
        } else if first_outer_top.is_none() {
            first_outer_top = Some(loop_of(&r_top, true));
            first_outer_bot = Some(loop_of(&r_bot, false));
        }
    }

    for c in &cells {
        let s_bot = peg_profile(*c, PEG_W_BOTTOM, PEG_R_BOTTOM);
        let s_mid = peg_profile(*c, PEG_W_MID, PEG_R_MID);
        let s_top = peg_profile(*c, PEG_W_TOP, OUTER_R);
        let r0 = ring(&mut b, &s_bot, 0.0);
        let r1 = ring(&mut b, &s_mid, PEG_Z1);
        let r2 = ring(&mut b, &s_mid, PEG_Z2);
        let r3 = ring(&mut b, &s_top, PEG_HEIGHT);
        wall_between(&mut b, &s_bot, &s_mid, &r0, &r1, 0.0, PEG_Z1, false);
        wall_between(&mut b, &s_mid, &s_mid, &r1, &r2, PEG_Z1, PEG_Z2, false);
        wall_between(&mut b, &s_mid, &s_top, &r2, &r3, PEG_Z2, PEG_HEIGHT, false);
        outer_top.push(loop_of(&r3, true));
        outer_bot.push(loop_of(&r0, false));
    }

    if let Some(t) = first_outer_top {
        planar(&mut b, PEG_HEIGHT, true, t, outer_top);
    }
    if let Some(bt) = first_outer_bot {
        planar(&mut b, 0.0, false, bt, outer_bot);
    }
    b.build()
}
