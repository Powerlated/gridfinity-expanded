//! The parametric Gridfinity model, built on the analytic B-rep kernel.
//!
//! The whole bin is assembled into one [`Builder`] so every interface edge is
//! shared automatically (no booleans needed for the vertical structure). Faces
//! carry exact `Plane` / `Cylinder` / `Cone` surfaces; the concave floor fillet
//! is a stack of constructed blend rings (the documented `loft`-based fillet).
//!
//! Simplification vs. the reference: the base is a single analytic chamfered
//! perimeter foot spanning the whole footprint (not one foot per cell). This is
//! watertight and printable for any N×M and mates with baseplates at the
//! perimeter; per-cell underside feet are intentionally omitted.

use crate::build::{RingEdges, loop_of, ring, wall_between};
use crate::geom::Surface;
use crate::math::{Vec3, vec3_of};
use crate::sketch::{Sketch, ccw_segs};
use crate::topo::{Builder, Loop, Solid};

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

const FILLET_STEPS: usize = 6;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    Bin,
    Baseplate,
}

/// Everything the UI can tune.
#[derive(Clone, Copy, Debug)]
pub struct Params {
    pub grid_x: u32,
    pub grid_y: u32,
    pub height_units: u32,
    pub wall_thickness: f32,
    pub cavity_corner_radius: f32,
    pub floor_fillet: f32,
    pub magnet_holes: bool,
    pub screw_holes: bool,
    pub divisions_x: u32,
    pub divisions_y: u32,
    pub mode: Mode,
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
            divisions_x: 1,
            divisions_y: 1,
            mode: Mode::Bin,
        }
    }
}

impl Params {
    pub fn total_height(&self) -> f32 {
        BASE_TOTAL_HEIGHT + HEIGHT_PER_UNIT * self.height_units.max(1) as f32
    }
    fn footprint(&self) -> (f32, f32) {
        (self.grid_x.max(1) as f32 * GRID_PITCH, self.grid_y.max(1) as f32 * GRID_PITCH)
    }
}

/// One rectangular compartment in mm (centre + size).
struct Compartment {
    cx: f32,
    cy: f32,
    w: f32,
    h: f32,
}

fn compartments(p: &Params) -> Vec<Compartment> {
    let (fw, fh) = p.footprint();
    let t = HALF_TOL + p.wall_thickness;
    let wt = p.wall_thickness;
    let (dx, dy) = (p.divisions_x.max(1), p.divisions_y.max(1));
    let inner_w = fw - 2.0 * t;
    let inner_h = fh - 2.0 * t;
    let cw = (inner_w - (dx - 1) as f32 * wt) / dx as f32;
    let ch = (inner_h - (dy - 1) as f32 * wt) / dy as f32;
    let mut out = Vec::new();
    for a in 0..dx {
        for b in 0..dy {
            let cx = t + a as f32 * (cw + wt) + cw / 2.0;
            let cy = t + b as f32 * (ch + wt) + ch / 2.0;
            out.push(Compartment { cx, cy, w: cw, h: ch });
        }
    }
    out
}

/// A compartment profile inset inward by `inset`, preserving segment structure
/// so fillet rings loft cleanly.
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
    for c in &comps {
        let cavity_top = build_cavity(&mut b, p, c, rc, floor_z, total_h);
        rim_holes.push(loop_of(&cavity_top, true));
    }
    planar(&mut b, total_h, true, loop_of(&r_otop_hi, true), rim_holes);

    b.build()
}

/// Build one compartment's inner walls + concave floor fillet + floor, returning
/// the top ring (shared with the rim as a hole).
fn build_cavity(
    b: &mut Builder,
    p: &Params,
    c: &Compartment,
    rc: f32,
    floor_z: f32,
    total_h: f32,
) -> RingEdges {
    let cavity_depth = total_h - floor_z;
    let fr = p
        .floor_fillet
        .min(cavity_depth)
        .min(c.w / 2.0 - 0.2)
        .min(c.h / 2.0 - 0.2)
        .max(0.0);

    // Ring stack bottom → top: concave fillet arc, then the straight wall.
    let mut levels: Vec<(f32, Vec<crate::sketch::Seg>)> = Vec::new();
    if fr > 1e-3 {
        for i in 0..=FILLET_STEPS {
            let h = fr * i as f32 / FILLET_STEPS as f32;
            let inset = fr - (fr * fr - (fr - h) * (fr - h)).max(0.0).sqrt();
            levels.push((floor_z + h, ccw_segs(&comp_profile(c, rc, inset))));
        }
    } else {
        levels.push((floor_z, ccw_segs(&comp_profile(c, rc, 0.0))));
    }
    levels.push((total_h, ccw_segs(&comp_profile(c, rc, 0.0))));

    let rings: Vec<RingEdges> = levels.iter().map(|(z, segs)| ring(b, segs, *z)).collect();
    for i in 0..levels.len() - 1 {
        wall_between(b, &levels[i].1, &levels[i + 1].1, &rings[i], &rings[i + 1], levels[i].0, levels[i + 1].0, false);
    }
    // Cavity floor (bottom ring, +Z, reversed to oppose the wall above it).
    planar(b, levels[0].0, true, loop_of(&rings[0], false), vec![]);
    rings.into_iter().last().unwrap()
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
