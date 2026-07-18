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
//!   resolved by the [`rectregion`](crate::rectregion) engine, with convex
//!   corners rounded by `cavity_corner_radius` and concave corners rounded by
//!   the fillet radius so the floor-wall blend chain stays tangent-continuous.
//! - The **floor fillet** is a true rolling-ball blend
//!   ([`fillet::blend_edges`](crate::fillet::blend_edges)) over each cavity
//!   loop's floor-wall edges.

use crate::build::{RingEdges, loop_of, ring, wall_between};
use crate::fillet::blend_edges;
use crate::geom::Surface;
use crate::layout::{EffectiveWalls, GridCell, GridEdge, Orientation, SplitLine, effective_walls};
use crate::math::{Vec2, Vec3, vec3_of};
use crate::rectregion::{LoopStyle, RectF, TracedLoop, shape_loop, trace_rects};
use crate::sketch::{Seg, Sketch, ccw_segs, loop_area, reverse_loop};
use crate::topo::{Builder, EdgeId, Loop, Solid, VertexId};
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

/// Build the whole layout (every logical bin, splits ignored) as one solid.
pub fn build(p: &Params) -> Solid {
    match p.mode {
        Mode::Bin => {
            let mut b = Builder::new();
            let mut blends: Vec<(EdgeId, f32)> = Vec::new();
            for bin in &p.bins {
                if bin.cells.is_empty() {
                    continue;
                }
                // Open edges are resolved by the split/open-face path (in
                // progress); the assembled preview keeps every perimeter walled.
                let walls = effective_walls(&bin.cells, &bin.cells, &[], &p.divider_edges);
                blends.extend(build_piece(&mut b, p, &bin.cells, walls, bin.slope));
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
struct OuterPiece {
    seg: Seg,
    shared: bool,
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
/// insets equal `HALF_TOL` — seam faces stay square), straight runs split at
/// the peg tangent points so peg-top edges weld with the wall's bottom ring.
fn author_outer_loop(
    steps: &[Step],
    inset: &dyn Fn(&GridEdge) -> f32,
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
        pieces.push(OuterPiece { seg: Seg::Line { a, b }, shared: is_std });
        if is_std {
            shared.sides.insert(s.edge);
        }

        // Corner piece between this step and the next, at `to`.
        let d1 = dirv(s_next.dir());
        let n1 = left_of(s_next.dir());
        let cross = d.x * d1.y - d.y * d1.x;
        let start = to - d * PEG_TANGENT + nrm * ins; // == b
        let end = to + d1 * PEG_TANGENT + n1 * ins_next;
        if cross.abs() < 0.5 {
            // Straight run: single connector over the inter-cell gap.
            pieces.push(OuterPiece { seg: Seg::Line { a: start, b: end }, shared: false });
        } else if cross > 0.0 && is_std && (ins_next - HALF_TOL).abs() < 1e-6 {
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
            });
            shared.corners.insert(s.to);
        } else {
            // Concave corner (or a non-spec inset): sharp — two lines meeting
            // at the inset-line intersection.
            let q = mm(s.to) + nrm * ins + n1 * ins_next;
            pieces.push(OuterPiece { seg: Seg::Line { a: start, b: q }, shared: false });
            pieces.push(OuterPiece { seg: Seg::Line { a: q, b: end }, shared: false });
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

// ── Peg authoring ────────────────────────────────────────────────────────────

/// Rounded-rect profile for a peg section, centred on cell `c`.
fn peg_profile(c: GridCell, w: f32, r: f32) -> Vec<Seg> {
    let cx = (c.x as f32 + 0.5) * GRID_PITCH;
    let cy = (c.y as f32 + 0.5) * GRID_PITCH;
    ccw_segs(&Sketch::rounded_rect(cx, cy, w, w, r))
}

/// Which of a peg-top ring's 8 segments (line/arc alternating, starting with
/// the bottom line) are FREE (bound the bridge underside rather than welding
/// with the outer wall).
fn peg_free_segments(c: GridCell, shared: &SharedWithPegs) -> [bool; 8] {
    let side_edges = [
        GridEdge { x: c.x, y: c.y, orientation: Orientation::H }, // bottom
        GridEdge { x: c.x + 1, y: c.y, orientation: Orientation::V }, // right
        GridEdge { x: c.x, y: c.y + 1, orientation: Orientation::H }, // top
        GridEdge { x: c.x, y: c.y, orientation: Orientation::V }, // left
    ];
    let corners = [
        (c.x + 1, c.y),     // BR
        (c.x + 1, c.y + 1), // TR
        (c.x, c.y + 1),     // TL
        (c.x, c.y),         // BL
    ];
    // Ring order from Sketch::rounded_rect: bottom, BR, right, TR, top, TL,
    // left, BL.
    [
        !shared.sides.contains(&side_edges[0]),
        !shared.corners.contains(&corners[0]),
        !shared.sides.contains(&side_edges[1]),
        !shared.corners.contains(&corners[1]),
        !shared.sides.contains(&side_edges[2]),
        !shared.corners.contains(&corners[2]),
        !shared.sides.contains(&side_edges[3]),
        !shared.corners.contains(&corners[3]),
    ]
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
fn build_piece(
    b: &mut Builder,
    p: &Params,
    cells: &[GridCell],
    walls: EffectiveWalls,
    slope: Option<BinSlope>,
) -> Vec<(EdgeId, f32)> {
    let total_h = p.total_height();
    let floor_z = BASE_TOTAL_HEIGHT + FLOOR_THICKNESS;

    // 1) Outer profile from the boundary walk (all perimeter edges walled at
    //    the spec inset for now; open/seam faces arrive with the split work).
    let inset = |_e: &GridEdge| -> f32 { HALF_TOL };
    let loops = boundary_steps(cells);
    let mut shared = SharedWithPegs::default();
    let outer_loops: Vec<Vec<OuterPiece>> = loops
        .iter()
        .map(|steps| author_outer_loop(steps, &inset, &mut shared))
        .collect();

    // 2) Pegs per cell + their bottom caps with fastener pockets.
    let mut peg_tops: Vec<(GridCell, RingEdges, Vec<Seg>)> = Vec::new();
    for &c in cells {
        let s_bot = peg_profile(c, PEG_W_BOTTOM, PEG_R_BOTTOM);
        let s_mid = peg_profile(c, PEG_W_MID, PEG_R_MID);
        let s_top = peg_profile(c, PEG_W_TOP, OUTER_R);
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

    // 3) Outer wall (PEG_HEIGHT → total_h) from the composite profile. The
    //    bottom ring's shared pieces intern onto the peg-top edges.
    let mut outer_rings: Vec<(Vec<Seg>, RingEdges, RingEdges, Vec<bool>)> = Vec::new();
    for pieces in &outer_loops {
        let segs: Vec<Seg> = pieces.iter().map(|p| p.seg).collect();
        let shared_flags: Vec<bool> = pieces.iter().map(|p| p.shared).collect();
        let lo = ring(b, &segs, PEG_HEIGHT);
        let hi = ring(b, &segs, total_h);
        // Outward for the outer boundary loop (CCW, area > 0); a hole loop of a
        // ring-shaped bin (CW) is an interior perimeter — its wall also faces
        // away from the material, which `wall_between` derives from the segs'
        // winding, so `outward = true` is correct for both.
        wall_between(b, &segs, &segs, &lo, &hi, PEG_HEIGHT, total_h, true);
        outer_rings.push((segs, lo, hi, shared_flags));
    }

    // 4) Bridge underside faces at PEG_HEIGHT: stitch the free (non-welded)
    //    peg-top segments (forward) with the free outer-profile pieces
    //    (reversed) into loops.
    let mut free: Vec<DirEdge> = Vec::new();
    for (c, r3, _) in &peg_tops {
        let free_flags = peg_free_segments(*c, &shared);
        for (k, &(e, d)) in r3.edges.iter().enumerate() {
            if free_flags[k] {
                let k1 = (k + 1) % r3.verts.len();
                free.push(DirEdge { start: r3.verts[k], end: r3.verts[k1], entry: (e, d) });
            }
        }
    }
    for (_, lo, _, shared_flags) in &outer_rings {
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

    // 5) Cavity: plan → trace → shape → prisms + floors (+ fillet chains).
    let (pos, neg) = plan_cavity(cells, &walls, p.wall_thickness.max(0.4));
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
    let outers: Vec<&TracedLoop> = traced.iter().filter(|l| !l.is_hole()).collect();
    let holes_of = |ol: &TracedLoop| -> Vec<&TracedLoop> {
        traced
            .iter()
            .filter(|l| l.is_hole() && point_in_rect_loop(l.pts[0], ol))
            .collect()
    };

    let mut blends: Vec<(EdgeId, f32)> = Vec::new();
    let mut rim_holes: Vec<Loop> = Vec::new();

    for ol in &outers {
        let (convex_r, concave_r) = if slope.is_some() { (0.0, 0.0) } else { (rc, fr) };
        let shape = shape_cavity_loop(ol, convex_r, concave_r);
        // The fillet requires every corner arc to have survived clamping (a
        // sharp corner would break the tangent chain).
        let mut loop_fr = fr;
        for s in &shape {
            if let Seg::Arc { radius, .. } = s {
                if *radius < loop_fr + 0.02 && is_convex_arc(&shape, s) {
                    loop_fr = (*radius - 0.02).max(0.0);
                }
            }
        }
        if fr > 0.0 && has_sharp_corner(&shape) {
            loop_fr = 0.0;
        }

        let islands = holes_of(ol);
        let island_shapes: Vec<Vec<Seg>> = islands
            .iter()
            .map(|il| shape_cavity_loop(il, rc, fr.max(loop_fr)))
            .collect();

        match slope {
            Some(sl) => {
                build_cavity_sloped(b, cells, &shape, &island_shapes, floor_z, total_h, sl);
            }
            None => {
                let fwe = build_cavity_flat(b, &shape, &island_shapes, floor_z, total_h);
                if loop_fr > 0.01 {
                    blends.extend(fwe.into_iter().map(|e| (e, loop_fr)));
                }
            }
        }

        // Rim hole for this cavity (island tops are their own caps).
        let top_ring = ring(b, &shape, total_h);
        rim_holes.push(loop_of(&top_ring, true));
    }

    // 6) Rim at total_h: outer loops + cavity holes.
    //    (Multiple outer loops cannot happen for a connected piece; holes of a
    //    ring-shaped bin become extra "hole" loops of the rim face.)
    let mut rim_outer: Option<Loop> = None;
    let mut rim_inners: Vec<Loop> = rim_holes;
    for (segs, _, hi, _) in &outer_rings {
        let lp = loop_of(hi, true);
        if loop_area(segs) > 0.0 && rim_outer.is_none() {
            rim_outer = Some(lp);
        } else {
            rim_inners.push(lp);
        }
    }
    if let Some(outer) = rim_outer {
        planar(b, total_h, true, outer, rim_inners);
    }

    blends
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
/// towers. Returns the floor-wall edges (cavity ring + island rings).
fn build_cavity_flat(
    b: &mut Builder,
    shape: &[Seg],
    islands: &[Vec<Seg>],
    floor_z: f32,
    total_h: f32,
) -> Vec<EdgeId> {
    let r_lo = ring(b, shape, floor_z);
    let r_hi = ring(b, shape, total_h);
    wall_between(b, shape, shape, &r_lo, &r_hi, floor_z, total_h, false);

    let mut fwe: Vec<EdgeId> = r_lo.edges.iter().map(|&(e, _)| e).collect();
    let mut floor_holes: Vec<Loop> = Vec::new();
    for isl in islands {
        let i_lo = ring(b, isl, floor_z);
        let i_hi = ring(b, isl, total_h);
        wall_between(b, isl, isl, &i_lo, &i_hi, floor_z, total_h, true);
        planar(b, total_h, true, loop_of(&i_hi, true), vec![]);
        floor_holes.push(loop_of(&i_lo, true));
        fwe.extend(i_lo.edges.iter().map(|&(e, _)| e));
    }
    planar(b, floor_z, true, loop_of(&r_lo, false), floor_holes);
    fwe
}

/// Sloped cavity: single tilted plane floor over the bin's bounding box, sharp
/// (line-only) walls rising to the rim.
fn build_cavity_sloped(
    b: &mut Builder,
    bin_cells: &[GridCell],
    shape: &[Seg],
    islands: &[Vec<Seg>],
    floor_z: f32,
    total_h: f32,
    slope: BinSlope,
) {
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

    let mut floor_holes: Vec<Loop> = Vec::new();
    for isl in islands {
        let i_lo = ring_z(b, isl, &z_of);
        let i_hi = ring(b, isl, total_h);
        wall_between(b, isl, isl, &i_lo, &i_hi, floor_z, total_h, true);
        planar(b, total_h, true, loop_of(&i_hi, true), vec![]);
        floor_holes.push(loop_of(&i_lo, true));
    }

    let origin = b.point(bottom.verts[0]);
    let normal = Vec3::new(-eff_m * ux, -eff_m * uy, 1.0).normalize();
    let surface = Surface::plane(origin, normal);
    b.face(surface, true, loop_of(&bottom, false), floor_holes);
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
