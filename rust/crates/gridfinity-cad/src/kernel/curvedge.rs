//! A curve carrying the parameter range an edge occupies of it, and the pieces
//! of edge emission both blending operators need. `emit_curv` interns an edge by
//! its two endpoints and orients the stored range from them, so two faces
//! emitting the same curve from opposite sides land on one edge rather than two.
//! `as_plane` and `loop_edge_dir` are the two questions those operators ask of a
//! solid often enough to be worth naming. `fillet` and `chamfer` each carried a
//! byte-identical copy of all four until they were lifted here.

use crate::kernel::geom::{Curve, Surface};
use crate::kernel::math::Vec3;
use crate::kernel::topo::{Builder, EdgeId, Solid};

#[derive(Clone, Copy)]
pub struct CurvEdge {
    pub curve: Curve,
    pub t0: f32,
    pub t1: f32,
}

pub fn as_plane(s: &Surface) -> Option<(Vec3, Vec3)> {
    if let Surface::Plane { origin, normal, .. } = s {
        Some((*origin, *normal))
    } else {
        None
    }
}

pub fn loop_edge_dir(solid: &Solid, fid: usize, e: EdgeId) -> bool {
    for lp in solid.face_loops(fid) {
        for &(ee, f) in lp {
            if ee == e {
                return f;
            }
        }
    }
    true
}

pub fn emit_curv(b: &mut Builder, start: Vec3, end: Vec3, ce: CurvEdge) -> (EdgeId, bool) {
    let vs = b.vertex(start);
    let ve = b.vertex(end);
    let forward = || {
        let at_start = ce.curve.point(ce.t0);
        (at_start - start).length() < (ce.curve.point(ce.t1) - start).length()
    };
    match ce.curve {
        Curve::Line { .. } => b.line(vs, ve),
        Curve::Circle {
            center,
            axis,
            radius,
            ref_dir,
        } => {
            let (t0, t1) = if forward() {
                (ce.t0, ce.t1)
            } else {
                (ce.t1, ce.t0)
            };
            b.arc(vs, ve, center, axis, radius, ref_dir, t0, t1)
        }
        Curve::Ellipse {
            center,
            a: ea,
            b: eb,
        } => {
            let (t0, t1) = if forward() {
                (ce.t0, ce.t1)
            } else {
                (ce.t1, ce.t0)
            };
            b.ellipse(vs, ve, center, ea, eb, t0, t1)
        }
        Curve::TorusSection { .. } => {
            let (t0, t1) = if forward() {
                (ce.t0, ce.t1)
            } else {
                (ce.t1, ce.t0)
            };
            b.torus_section(vs, ve, ce.curve, t0, t1)
        }
    }
}
