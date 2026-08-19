//! A curve carrying the parameter range an edge occupies of it, and the pieces
//! of edge emission every operator that rewrites a loop needs. `emit_edge` is
//! the primitive -- the one place that knows which `Builder` constructor each
//! `Curve` variant is built by -- and `emit_curv` is it plus the endpoint
//! bookkeeping a blend needs, interning an edge by its two endpoints and
//! orienting the stored range from them, so two faces emitting the same curve
//! from opposite sides land on one edge rather than two. `as_plane` and
//! `loop_edge_dir` are the two questions those operators ask of a solid often
//! enough to be worth naming. `fillet` and `chamfer` each carried a
//! byte-identical copy of all four until they were lifted here, and `fillet`,
//! `chamfer` and `split` each open-coded `emit_edge`'s match until it followed
//! them.

use crate::kernel::geom::{Curve, Surface};
use crate::kernel::math::Vec3;
use crate::kernel::topo::{Builder, EdgeId, Solid, VertexId};

#[derive(Clone, Copy)]
pub struct CurvEdge {
    pub curve: Curve,
    pub t0: f32,
    pub t1: f32,
}

pub fn as_plane(s: &Surface) -> Option<(Vec3, Vec3)> {
    if let Surface::Plane { origin, normal, .. } = s {
        Some((*origin, (**normal)))
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

/// The edge running along `curve` from `vs` to `ve`, interned in `b` and built
/// by whichever constructor that curve variant needs, with `(t0, t1)` the
/// parameter range in the direction `vs -> ve`.
///
/// The caller has already interned both vertices and already ordered the range
/// to match them -- `Builder::arc` trusts that its first vertex sits at the
/// first angle, so a range running the other way names the complementary arc.
/// A `Curve::Line` ignores the range entirely, its extent being its two
/// endpoints. Returns the edge and whether the loop traverses it forwards, which
/// is what interning answers when another face has already emitted it.
pub fn emit_edge(
    b: &mut Builder,
    vs: VertexId,
    ve: VertexId,
    curve: Curve,
    t0: f32,
    t1: f32,
) -> (EdgeId, bool) {
    match curve {
        Curve::Line { .. } => b.line(vs, ve),
        Curve::Circle {
            center,
            axis,
            radius,
            ref_dir,
        } => b.arc(vs, ve, center, *axis, radius, *ref_dir, t0, t1),
        Curve::Ellipse { center, a, b: eb } => b.ellipse(vs, ve, center, a, eb, t0, t1),
        Curve::TorusSection { .. } => b.torus_section(vs, ve, curve, t0, t1),
    }
}

/// The edge running along `ce`'s curve between the points `start` and `end`,
/// with `ce`'s stored range turned to face the way those two points do.
///
/// `emit_edge` needs a range already ordered `vs -> ve`, and a blend holds its
/// trim curves in whatever direction the surface was swept in; whichever of the
/// range's two ends sits nearer `start` is the one the edge leaves from. Both
/// endpoints are interned, so a face emitting this curve from the other side
/// lands on the same edge with the opposite traversal flag.
pub fn emit_curv(b: &mut Builder, start: Vec3, end: Vec3, ce: CurvEdge) -> (EdgeId, bool) {
    let vs = b.vertex(start);
    let ve = b.vertex(end);
    let forward = (ce.curve.point(ce.t0) - start).length_squared()
        < (ce.curve.point(ce.t1) - start).length_squared();
    let (t0, t1) = if forward {
        (ce.t0, ce.t1)
    } else {
        (ce.t1, ce.t0)
    };
    emit_edge(b, vs, ve, ce.curve, t0, t1)
}
