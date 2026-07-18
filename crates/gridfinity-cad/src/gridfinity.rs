//! The parametric Gridfinity model, built on the analytic B-rep kernel.
//!
//! The whole bin is assembled into one [`Builder`] so every interface edge is
//! shared automatically (no booleans needed for the vertical structure). Faces
//! carry exact `Plane` / `Cylinder` / `Cone` surfaces; the concave floor fillet
//! is a stack of constructed blend rings (the documented `loft`-based fillet);
//! a sloped floor is a single tilted `Plane` with walls rising from the tilted
//! bottom ring to the flat rim.
//!
//! Simplifications vs. the reference (documented):
//! - The base is a single analytic chamfered perimeter foot spanning the whole
//!   footprint (not one foot per cell). Watertight and printable for any N×M.
//! - `divider_edges` are interpreted as full split lines (a divider on a grid
//!   line walls the whole line), so compartments are always rectangular.
//! - `open_edges` are accepted by `Params` but not yet applied geometrically
//!   (faithful open-perimeter cavities need a notched rim, which requires the
//!   polygon work the kernel deliberately omits).

use crate::build::{RingEdges, loop_of, ring, wall_between};
use crate::fillet::blend_edges;
use crate::geom::Surface;
use crate::layout::{Axis, GridEdge, Orientation};
use crate::math::{Vec2, Vec3, vec3_of};
use crate::sketch::{Seg, Sketch, ccw_segs};
use crate::topo::{Builder, EdgeId, Loop, Solid, VertexId};

// ── Gridfinity spec constants (mm) ───────────────────────────────────────────
pub const GRID_PITCH: f32 = 42.0;
pub const HEIGHT_PER_UNIT: f32 = 7.0;
pub const BASE_TOTAL_HEIGHT: f32 = 7.0;
pub const PEG_HEIGHT: f32 = 4.75;
pub const PEG_Z1: f32 = 0.8;
pub const PEG_Z2: f32 = 2.6;
pub const OUTER_R: f32 = 3.75;
pub const FLOOR_THICKNESS: f32 = 1.2;
const HALF_TOL: f32 = 0.25; // (GRID_PITCH − 41.5)/2 clearance per side
const MAGNET_RADIUS: f32 = 3.25;
const MAGNET_DEPTH: f32 = 2.4;
const SCREW_RADIUS: f32 = 1.5;
const SCREW_DEPTH: f32 = 6.0;
const FASTENER_INSET: f32 = 13.0;

// Foot chamfer insets from the outer profile (give corner radii 0.8 / 1.6 / 3.75).
const FOOT_BOTTOM_INSET: f32 = 2.95;
const FOOT_MID_INSET: f32 = 2.15;

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

/// Everything the UI can tune. Carries the reference `BinConfig` surface:
/// footprint grid, height, walls/fillet, fasteners, divider edges, open edges,
/// and an optional per-bin floor slope. (`divisions_x/y` are replaced by
/// `divider_edges`; see [`Params::divisions`] for the convenience form.)
#[derive(Clone, Debug)]
pub struct Params {
    pub grid_x: u32,
    pub grid_y: u32,
    pub height_units: u32,
    pub wall_thickness: f32,
    pub cavity_corner_radius: f32,
    pub floor_fillet: f32,
    pub magnet_holes: bool,
    pub screw_holes: bool,
    pub mode: Mode,
    pub divider_edges: Vec<GridEdge>,
    pub open_edges: Vec<GridEdge>,
    pub slope: Option<BinSlope>,
}

impl Default for Params {
    fn default() -> Params {
        Params {
            grid_x: 2,
            grid_y: 2,
            height_units: 3,
            wall_thickness: 1.2,
            cavity_corner_radius: 2.5,
            floor_fillet: 3.0,
            magnet_holes: false,
            screw_holes: false,
            mode: Mode::Bin,
            divider_edges: Vec::new(),
            open_edges: Vec::new(),
            slope: None,
        }
    }
}

impl Params {
    /// Convenience: set evenly-spaced divider edges equivalent to the old
    /// `divisions_x`/`divisions_y` (n cuts ⇒ n+1 compartments per axis).
    pub fn divisions(mut self, div_x: u32, div_y: u32) -> Params {
        self.divider_edges = divisions_to_edges(self.grid_x, self.grid_y, div_x, div_y);
        self
    }

    pub fn total_height(&self) -> f32 {
        BASE_TOTAL_HEIGHT + HEIGHT_PER_UNIT * self.height_units.max(1) as f32
    }
    fn footprint(&self) -> (f32, f32) {
        (self.grid_x.max(1) as f32 * GRID_PITCH, self.grid_y.max(1) as f32 * GRID_PITCH)
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
    let _ = Axis::X; // keep the Axis import meaningful for future per-edge work
    out
}

/// One rectangular compartment in mm (centre + size).
struct Compartment {
    cx: f32,
    cy: f32,
    w: f32,
    h: f32,
}

/// Compartments derived from the divider edges: each divider grid line is a full
/// split line, so the footprint tiles into rectangular compartments (one per
/// cell-range product). Compartment widths are proportional to their cell count.
fn compartments(p: &Params) -> Vec<Compartment> {
    let (fw, fh) = p.footprint();
    let (gx, gy) = (p.grid_x.max(1) as i32, p.grid_y.max(1) as i32);
    let t = HALF_TOL + p.wall_thickness;
    let wt = p.wall_thickness;

    let mut xs: Vec<i32> = p
        .divider_edges
        .iter()
        .filter(|e| e.orientation == Orientation::V)
        .map(|e| e.x.clamp(1, gx - 1))
        .collect();
    let mut ys: Vec<i32> = p
        .divider_edges
        .iter()
        .filter(|e| e.orientation == Orientation::H)
        .map(|e| e.y.clamp(1, gy - 1))
        .collect();
    xs.sort_unstable();
    xs.dedup();
    ys.sort_unstable();
    ys.dedup();

    let mut xb: Vec<i32> = vec![0];
    xb.extend(xs);
    xb.push(gx);
    let mut yb: Vec<i32> = vec![0];
    yb.extend(ys);
    yb.push(gy);

    let inner_w = fw - 2.0 * t;
    let inner_h = fh - 2.0 * t;
    let n_x = xb.len() - 1;
    let n_y = yb.len() - 1;
    let avail_x = inner_w - (n_x - 1) as f32 * wt;
    let avail_y = inner_h - (n_y - 1) as f32 * wt;

    let mut out = Vec::new();
    let mut x_pos = t;
    for i in 0..n_x {
        let cells_x = (xb[i + 1] - xb[i]) as f32;
        let cw = avail_x * cells_x / gx as f32;
        let mut y_pos = t;
        for j in 0..n_y {
            let cells_y = (yb[j + 1] - yb[j]) as f32;
            let ch = avail_y * cells_y / gy as f32;
            out.push(Compartment {
                cx: x_pos + cw * 0.5,
                cy: y_pos + ch * 0.5,
                w: cw,
                h: ch,
            });
            y_pos += ch + wt;
        }
        x_pos += cw + wt;
    }
    out
}

/// A compartment profile inset inward by `inset`. The corner arc CENTER stays
/// fixed (only its radius shrinks as `rc - inset`), so the corner faces lofted
/// across fillet levels are valid coaxial cones — never non-coaxial frustums.
fn comp_profile(c: &Compartment, rc: f32, inset: f32) -> Sketch {
    let w = (c.w - 2.0 * inset).max(0.1);
    let h = (c.h - 2.0 * inset).max(0.1);
    if rc <= 1e-4 {
        Sketch::rectangle(c.cx, c.cy, w, h)
    } else {
        Sketch::rounded_rect(c.cx, c.cy, w, h, (rc - inset).max(0.02))
    }
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

/// Build the bin (or baseplate) as an analytic B-rep solid.
pub fn build(p: &Params) -> Solid {
    match p.mode {
        Mode::Bin => build_bin(p),
        Mode::Baseplate => build_baseplate(p),
    }
}

fn build_bin(p: &Params) -> Solid {
    let (fw, fh) = p.footprint();
    let (cx, cy) = (fw / 2.0, fh / 2.0);
    let ow = fw - 2.0 * HALF_TOL;
    let oh = fh - 2.0 * HALF_TOL;
    let total_h = p.total_height();
    let floor_z = BASE_TOTAL_HEIGHT + FLOOR_THICKNESS;

    let outer = ccw_segs(&Sketch::rounded_rect(cx, cy, ow, oh, OUTER_R));
    let foot_ring = |inset: f32| ccw_segs(&Sketch::rounded_rect(cx, cy, ow - 2.0 * inset, oh - 2.0 * inset, OUTER_R - inset));

    let mut b = Builder::new();

    // ── Base: single chamfered perimeter foot (0 → PEG_HEIGHT). ──────────────
    let f_bottom = foot_ring(FOOT_BOTTOM_INSET);
    let f_mid = foot_ring(FOOT_MID_INSET);
    let r_f0 = ring(&mut b, &f_bottom, 0.0);
    let r_f1 = ring(&mut b, &f_mid, PEG_Z1);
    let r_f2 = ring(&mut b, &f_mid, PEG_Z2);
    let r_otop_lo = ring(&mut b, &outer, PEG_HEIGHT); // == foot top == outer-wall bottom
    wall_between(&mut b, &f_bottom, &f_mid, &r_f0, &r_f1, 0.0, PEG_Z1, true);
    wall_between(&mut b, &f_mid, &f_mid, &r_f1, &r_f2, PEG_Z1, PEG_Z2, true);
    wall_between(&mut b, &f_mid, &outer, &r_f2, &r_otop_lo, PEG_Z2, PEG_HEIGHT, true);

    // Foot bottom cap, with magnet/screw pockets punched through it.
    let holes = fastener_holes(&mut b, p);
    let hole_loops: Vec<Loop> = holes.iter().map(|h| loop_of(h, false)).collect();
    planar(&mut b, 0.0, false, loop_of(&r_f0, false), hole_loops);

    // ── Outer wall (PEG_HEIGHT → total_h). ───────────────────────────────────
    let r_otop_hi = ring(&mut b, &outer, total_h);
    wall_between(&mut b, &outer, &outer, &r_otop_lo, &r_otop_hi, PEG_HEIGHT, total_h, true);

    // ── Cavities (one per compartment) + rim. ────────────────────────────────
    let rc = p.cavity_corner_radius.max(0.0);
    let comps = compartments(p);
    let mut rim_holes: Vec<Loop> = Vec::new();
    let mut floor_wall: Vec<(EdgeId, f32)> = Vec::new();
    for c in &comps {
        let (cavity_top, fwe) = build_cavity(&mut b, p, c, rc, floor_z, total_h);
        rim_holes.push(loop_of(&cavity_top, true));
        floor_wall.extend(fwe);
    }
    planar(&mut b, total_h, true, loop_of(&r_otop_hi, true), rim_holes);

    let mut solid = b.build();
    // Apply the concave floor fillet as a true rolling-ball blend on every
    // compartment's floor-wall loop (replaces the old cone-loft approximation).
    if !floor_wall.is_empty() {
        solid = blend_edges(&solid, &floor_wall).expect("floor fillet blend");
    }
    solid
}

/// Build one compartment's inner walls + floor, returning the top ring (shared
/// with the rim as a hole) and the floor-wall loop edges (with the fillet radius
/// to apply). Dispatches to the sloped path (tilted plane floor, no fillet) when
/// a slope is set, else the flat sharp path (fillet applied later by the caller).
fn build_cavity(
    b: &mut Builder,
    p: &Params,
    c: &Compartment,
    rc: f32,
    floor_z: f32,
    total_h: f32,
) -> (RingEdges, Vec<(EdgeId, f32)>) {
    match p.slope {
        Some(slope) => (build_cavity_sloped(b, p, c, floor_z, total_h, slope), Vec::new()),
        None => build_cavity_flat(b, p, c, rc, floor_z, total_h),
    }
}

/// Sharp flat compartment floor + walls. The concave floor fillet is applied
/// later (by `blend_edges` in `build_bin`) over the returned floor-wall edges.
fn build_cavity_flat(
    b: &mut Builder,
    p: &Params,
    c: &Compartment,
    rc: f32,
    floor_z: f32,
    total_h: f32,
) -> (RingEdges, Vec<(EdgeId, f32)>) {
    let cavity_depth = total_h - floor_z;
    let fr = p
        .floor_fillet
        .min(cavity_depth)
        .min(c.w / 2.0 - 0.2)
        .min(c.h / 2.0 - 0.2)
        .max(0.0);
    let fr = if rc > 0.02 { fr.min(rc - 0.02) } else { 0.0 };

    let bot = ccw_segs(&comp_profile(c, rc, 0.0));
    let top = bot.clone();
    let r_lo = ring(b, &bot, floor_z);
    let r_hi = ring(b, &top, total_h);
    wall_between(b, &bot, &top, &r_lo, &r_hi, floor_z, total_h, false);
    planar(b, floor_z, true, loop_of(&r_lo, false), vec![]);

    let fwe: Vec<(EdgeId, f32)> = if fr > 1e-4 {
        r_lo.edges.iter().map(|&(e, _)| (e, fr)).collect()
    } else {
        Vec::new()
    };
    (r_hi, fwe)
}

/// Sloped compartment: the floor is a single tilted plane (lowest at the
/// configured side, rising away from it across the whole bin footprint), and the
/// four walls rise from that tilted bottom ring to the flat rim. Corner rounding
/// and floor fillet are intentionally skipped on sloped floors.
fn build_cavity_sloped(
    b: &mut Builder,
    p: &Params,
    c: &Compartment,
    floor_z: f32,
    total_h: f32,
    slope: BinSlope,
) -> RingEdges {
    let (fw, fh) = p.footprint();
    let (ux, uy) = uphill_unit(slope.dir);
    let corners = [(0.0, 0.0), (fw, 0.0), (0.0, fh), (fw, fh)];
    let dots = corners.map(|(x, y)| ux * x + uy * y);
    let min_a = dots.iter().copied().fold(f32::INFINITY, f32::min);
    let max_a = dots.iter().copied().fold(f32::NEG_INFINITY, f32::max);
    let span = (max_a - min_a).max(1e-6);
    let m = slope.angle_deg.to_radians().tan().clamp(0.0, 3.0);
    let cavity_depth = total_h - floor_z;
    let h_max = (m * span).min(cavity_depth - 0.5).max(0.0);
    let eff_m = h_max / span;
    let z_of = |pt: Vec2| floor_z + eff_m * (ux * pt.x + uy * pt.y - min_a);

    let bottom_segs = ccw_segs(&Sketch::rectangle(c.cx, c.cy, c.w, c.h));
    let top_segs = bottom_segs.clone();
    let bottom = ring_z(b, &bottom_segs, z_of);
    let top = ring(b, &top_segs, total_h);
    wall_between(b, &bottom_segs, &top_segs, &bottom, &top, floor_z, total_h, false);

    // Tilted floor plane (upward normal), paired against the walls' bottom ring.
    let origin = b.point(bottom.verts[0]);
    let normal = Vec3::new(-eff_m * ux, -eff_m * uy, 1.0).normalize();
    let surface = Surface::plane(origin, normal);
    b.face(surface, true, loop_of(&bottom, false), vec![]);
    top
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

/// A profile ring realised at a per-vertex height (used for the tilted bottom of
/// a sloped compartment). Lines only — sloped cavities use sharp corners.
fn ring_z(b: &mut Builder, segs: &[Seg], z_of: impl Fn(Vec2) -> f32) -> RingEdges {
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

/// Magnet and/or screw pockets at the four ±FASTENER_INSET corners of each
/// cell, drilled up from z = 0. Returns each pocket's rim ring in the base
/// underside (for the foot bottom cap's holes).
fn fastener_holes(b: &mut Builder, p: &Params) -> Vec<RingEdges> {
    let mut out = Vec::new();
    if !p.magnet_holes && !p.screw_holes {
        return out;
    }
    let (gx, gy) = (p.grid_x.max(1), p.grid_y.max(1));
    for i in 0..gx {
        for j in 0..gy {
            let ccx = (i as f32 + 0.5) * GRID_PITCH;
            let ccy = (j as f32 + 0.5) * GRID_PITCH;
            for (dx, dy) in [(-1.0, -1.0), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0)] {
                let hx = ccx + dx * FASTENER_INSET;
                let hy = ccy + dy * FASTENER_INSET;
                out.push(match (p.magnet_holes, p.screw_holes) {
                    // Concentric: a stepped counterbore (wide magnet recess, then
                    // a narrow screw pilot through its ceiling).
                    (true, true) => drill_stepped(b, hx, hy, MAGNET_RADIUS, MAGNET_DEPTH, SCREW_RADIUS, SCREW_DEPTH),
                    (true, false) => drill_blind(b, hx, hy, MAGNET_RADIUS, MAGNET_DEPTH),
                    (false, true) => drill_blind(b, hx, hy, SCREW_RADIUS, SCREW_DEPTH),
                    (false, false) => unreachable!(),
                });
            }
        }
    }
    out
}

/// A single blind cylindrical pocket drilled up from z = 0 to `depth`. Returns
/// its rim ring at z = 0.
fn drill_blind(b: &mut Builder, x: f32, y: f32, radius: f32, depth: f32) -> RingEdges {
    let profile = ccw_segs(&Sketch::circle(x, y, radius));
    let bottom = ring(b, &profile, 0.0);
    let top = ring(b, &profile, depth);
    wall_between(b, &profile, &profile, &bottom, &top, 0.0, depth, false);
    planar(b, depth, false, loop_of(&top, true), vec![]); // blind end (−Z)
    bottom
}

/// A stepped counterbore: a wide pocket (`r0`,`d0`) whose ceiling is pierced by
/// a narrow concentric pocket (`r1`,`d1`). Returns the wide rim ring at z = 0.
fn drill_stepped(b: &mut Builder, x: f32, y: f32, r0: f32, d0: f32, r1: f32, d1: f32) -> RingEdges {
    let outer = ccw_segs(&Sketch::circle(x, y, r0));
    let inner = ccw_segs(&Sketch::circle(x, y, r1));
    let o_bot = ring(b, &outer, 0.0);
    let o_top = ring(b, &outer, d0);
    let i_top0 = ring(b, &inner, d0);
    let i_top1 = ring(b, &inner, d1);
    // Wide wall 0→d0, its annular ceiling at d0, narrow wall d0→d1, blind end.
    wall_between(b, &outer, &outer, &o_bot, &o_top, 0.0, d0, false);
    planar(b, d0, false, loop_of(&o_top, true), vec![loop_of(&i_top0, false)]);
    wall_between(b, &inner, &inner, &i_top0, &i_top1, d0, d1, false);
    planar(b, d1, false, loop_of(&i_top1, true), vec![]);
    o_bot
}

// Spec connector-peg profile (mm), used for baseplate sockets.
const PEG_W_BOTTOM: f32 = 35.6;
const PEG_W_MID: f32 = 37.2;
const PEG_W_TOP: f32 = 41.5;
const PEG_R_BOTTOM: f32 = 0.8;
const PEG_R_MID: f32 = 1.5;

/// A standard "lite" baseplate: a slab (0 → PEG_HEIGHT) with a chamfered,
/// through-socket per cell shaped exactly like the Gridfinity connector peg, so
/// standard bins nest into it. Built entirely by construction — the sockets are
/// peg-shaped holes whose rims live in the slab's top and bottom faces.
fn build_baseplate(p: &Params) -> Solid {
    // Baseplates are full grid pitch (42·n), unlike bins (42·n − 0.5), so the
    // 41.5 sockets sit inside a thin perimeter frame instead of flush.
    let (fw, fh) = p.footprint();
    let (cx, cy) = (fw / 2.0, fh / 2.0);
    let outer = ccw_segs(&Sketch::rounded_rect(cx, cy, fw, fh, OUTER_R));

    let mut b = Builder::new();
    let r_bot = ring(&mut b, &outer, 0.0);
    let r_top = ring(&mut b, &outer, PEG_HEIGHT);
    wall_between(&mut b, &outer, &outer, &r_bot, &r_top, 0.0, PEG_HEIGHT, true);

    // One peg-shaped socket per cell; collect the rim rings that become holes in
    // the top (z = PEG_HEIGHT) and bottom (z = 0) faces.
    let (gx, gy) = (p.grid_x.max(1), p.grid_y.max(1));
    let mut top_holes: Vec<Loop> = Vec::new();
    let mut bot_holes: Vec<Loop> = Vec::new();
    for i in 0..gx {
        for j in 0..gy {
            let (scx, scy) = ((i as f32 + 0.5) * GRID_PITCH, (j as f32 + 0.5) * GRID_PITCH);
            let s_bot = ccw_segs(&Sketch::rounded_rect(scx, scy, PEG_W_BOTTOM, PEG_W_BOTTOM, PEG_R_BOTTOM));
            let s_mid = ccw_segs(&Sketch::rounded_rect(scx, scy, PEG_W_MID, PEG_W_MID, PEG_R_MID));
            let s_top = ccw_segs(&Sketch::rounded_rect(scx, scy, PEG_W_TOP, PEG_W_TOP, OUTER_R));
            let r0 = ring(&mut b, &s_bot, 0.0);
            let r1 = ring(&mut b, &s_mid, PEG_Z1);
            let r2 = ring(&mut b, &s_mid, PEG_Z2);
            let r3 = ring(&mut b, &s_top, PEG_HEIGHT);
            // Socket walls face into the socket (a hole → outward = false).
            wall_between(&mut b, &s_bot, &s_mid, &r0, &r1, 0.0, PEG_Z1, false);
            wall_between(&mut b, &s_mid, &s_mid, &r1, &r2, PEG_Z1, PEG_Z2, false);
            wall_between(&mut b, &s_mid, &s_top, &r2, &r3, PEG_Z2, PEG_HEIGHT, false);
            top_holes.push(loop_of(&r3, true));
            bot_holes.push(loop_of(&r0, false));
        }
    }

    planar(&mut b, PEG_HEIGHT, true, loop_of(&r_top, true), top_holes);
    planar(&mut b, 0.0, false, loop_of(&r_bot, false), bot_holes);
    b.build()
}
