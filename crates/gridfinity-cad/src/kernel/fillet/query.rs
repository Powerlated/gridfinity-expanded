//! Read-only questions about a solid that several blend phases ask. Nothing here
//! mutates the solid or decides anything; every answer is a function of the solid
//! and the arguments alone, which is the property that matters, since two faces
//! sharing an edge ask these independently and must get the same answer.

use crate::kernel::curvedge::as_plane;
use crate::kernel::geom::{Curve, Surface};
use crate::kernel::math::Vec3;
use crate::kernel::topo::{Edge, EdgeFaces, EdgeId, Solid};

/// Maps a cylindrical surface to its `(base, axis, radius)` and every other
/// surface kind to `None`. The axis is returned as stored, not renormalized.
pub(super) fn as_cyl(s: &Surface) -> Option<(Vec3, Vec3, f64)> {
    if let Surface::Cylinder {
        base, axis, radius, ..
    } = s
    {
        Some((*base, (**axis), *radius))
    } else {
        None
    }
}

/// True when both surfaces are planes, their unit normals are parallel to within
/// 1e-5 of sine, and one's origin lies within 1e-4 mm of the other's plane;
/// false for any non-planar surface, whatever its shape. Normal sign is
/// ignored, so two planes facing opposite ways are coplanar. The answer is a
/// pairwise test and nothing more: it is not transitive, and a caller wanting a
/// coplanar *set* must compare every member against one chosen representative
/// rather than chaining.
pub(super) fn coplanar(x: &Surface, y: &Surface) -> bool {
    match (as_plane(x), as_plane(y)) {
        (Some((o0, n0)), Some((o1, n1))) => {
            let (n0, n1) = (n0.normalize_or_zero(), n1.normalize_or_zero());
            n0.cross(n1).length() < 1e-5 && (o1 - o0).dot(n0).abs() < 1e-4
        }
        _ => false,
    }
}

/// Maps a vertex index to the faces touching it: every face at least one of
/// whose loops -- outer or inner -- names an edge ending at `v`. Each face
/// appears once, in ascending face id, so the result is a function of the solid
/// and `v` alone and does not depend on traversal order.
pub(super) fn faces_at_vertex(solid: &Solid, v: usize) -> Vec<usize> {
    let mut out = Vec::new();
    for fi in 0..solid.faces.len() {
        let mut hit = false;
        for lp in solid.face_loops(fi) {
            for &(e, _) in lp {
                let ed = solid.edges[e];
                if ed.v0 == v || ed.v1 == v {
                    hit = true;
                }
            }
        }
        if hit {
            out.push(fi);
        }
    }
    out
}

/// Maps `(vertex v, face side)` to the faces reached by stepping over `side`'s
/// own edges at `v`: for every edge of `side` ending at `v` other than `skip`,
/// the other face `ef` gives that edge. `side` itself never appears, each
/// neighbour appears once, and the order is `side`'s loop order. With `skip` set
/// to the blended edge this is "what lies beyond this chain end on `side`".
pub(super) fn across_at(
    solid: &Solid,
    v: usize,
    side: usize,
    skip: EdgeId,
    ef: &EdgeFaces,
) -> Vec<usize> {
    let mut out = Vec::new();
    for &(e, _) in solid.face_loops(side).flatten() {
        let ed = solid.edges[e];
        if e == skip || (ed.v0 != v && ed.v1 != v) {
            continue;
        }
        for &f in &ef[e] {
            if f != side && !out.contains(&f) {
                out.push(f);
            }
        }
    }
    out
}

/// Maps a point to its distance from the edge's supporting curve **extended**
/// past `ed`'s own parameter range: the perpendicular distance to the infinite
/// line for `Curve::Line`, the distance to the full circle for `Curve::Circle`,
/// and the distance to the infinite chord line through the edge's two vertices
/// for every other curve kind. The range is deliberately not clamped -- a
/// retreating corner lands beyond the stored ends as often as inside them, and a
/// clamped distance would rank two tangent points by how far past the end they
/// fall rather than by which line they are on. Every branch is a function of the
/// edge alone, so the two faces sharing it agree.
pub(super) fn dist_to_curve(p: Vec3, ed: &Edge, solid: &Solid) -> f64 {
    match ed.curve {
        Curve::Line { p0, dir } => {
            let rel = p - p0;
            (rel - *dir * rel.dot(*dir)).length()
        }
        Curve::Circle {
            center,
            axis,
            radius,
            ..
        } => {
            let rel = p - center;
            let along = rel.dot(*axis);
            let radial = (rel - *axis * along).length();
            ((radial - radius).powi(2) + along * along).sqrt()
        }
        _ => {
            let (a, b) = (solid.verts[ed.v0].point, solid.verts[ed.v1].point);
            let d = (b - a).normalize_or_zero();
            let rel = p - a;
            (rel - d * rel.dot(d)).length()
        }
    }
}
