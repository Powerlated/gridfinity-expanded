//! Edge chamfering: a true B-rep chamfer operator, parallel to [`fillet`].
//!
//! [`chamfer_edges`] consumes a [`Solid`] and a list of `(EdgeId, d_a, d_b)`
//! chamfers and returns a new `Solid` in which each chamfered edge is replaced
//! by a flat bevel — an analytic `Plane` through two offset curves, one on
//! each adjacent face. Adjacent chamfers in a chain share a corner point on
//! each face (the intersection of the two offset lines), so the chain welds
//! into one shared connect edge by interning.
//!
//! ## Geometry
//!
//! For an edge between planar faces with outward normals `m_a`, `m_b` and
//! direction `d_hat`, the in-face tangent on face A pointing into the face
//! (away from the edge) is `t_a`, similarly `t_b` on face B. The chamfer
//! offsets the edge by `d_a` along `t_a` on face A and `d_b` along `t_b` on
//! face B; the chamfer surface is the `Plane` through the four offset points.
//!
//! Asymmetric distances (`d_a != d_b`) tilt the chamfer plane; symmetric
//! (`d_a == d_b`) is the classic 45° bevel when the faces meet at 90°.
//!
//! ## Scope (vs [`fillet`])
//!
//! - Plane/plane edges only (a plane/cylinder chamfer would need a cone,
//!   which the project's no-approximation rule forbids as a polyline stand-in;
//!   defer until a true cone-blend primitive exists).
//! - Line edges only (chamfer of an arc edge is also a cone).
//! - Closed smooth chains: every blended vertex must be shared by exactly two
//!   blended edges. Runout (one blended edge at a vertex) and spherical
//!   corners (three or more) are not yet supported → `Err`.
//!
//! No runout means the chamfer must form a closed loop on the solid (a pocket
//! rim, a box edge loop, etc.). That matches every current Gridfinity use
//! case; the floor fillet's pocket-wall loop is exactly this shape.

use crate::kernel::geom::{Curve, Surface};
use crate::kernel::math::Vec3;
use crate::kernel::topo::{Builder, EdgeId, Loop, Solid};
use std::collections::HashMap;

/// One chamfer between two planar faces: a flat `Plane` through four offset
/// points. `ta`/`tb` are the offset *lines* on faces a/b; `ca0`/`ca1` are the
/// straight connect lines at the chain corners. Compare [`fillet::Blend`],
/// where these are circles/arcs and the surface is a Cylinder/Torus.
#[derive(Clone)]
struct Chamfer {
    ta: CurvEdge, // offset line on face a, p0→p1
    tb: CurvEdge, // offset line on face b, p0→p1
    ca0: CurvEdge, // connect line at p0: ta_p0 → tb_p0
    ca1: CurvEdge, // connect line at p1: ta_p1 → tb_p1
    ta_p0: Vec3,
    ta_p1: Vec3,
    tb_p0: Vec3,
    tb_p1: Vec3,
    surface: Surface,
    sense: bool,
    /// How face a traverses the original edge (v0→v1 = true). The chamfer loop
    /// must oppose it on both shared offset lines (manifold invariant).
    fwd_a: bool,
}

/// A curve with its parameter range, as emitted between two endpoints.
/// Identical to the one in [`fillet`] but kept local so the modules don't
/// cross-reference each other's internals.
#[derive(Clone, Copy)]
struct CurvEdge {
    curve: Curve,
    t0: f32,
    t1: f32,
}

/// Chamfer a set of edges by `(d_a, d_b)` distance pairs. `d_a` is the
/// offset distance on the first face of each edge (per `edge_faces` order),
/// `d_b` on the second.
pub fn chamfer_edges(solid: &Solid, chamfers: &[(EdgeId, f32, f32)]) -> Result<Solid, String> {
    let want: HashMap<EdgeId, (f32, f32)> =
        chamfers.iter().copied().map(|(e, da, db)| (e, (da, db))).collect();
    let edge_faces = solid.edge_faces();

    for &(e, d_a, d_b) in chamfers {
        if e >= solid.edges.len() {
            return Err(format!("chamfer: edge {e} out of range"));
        }
        if edge_faces[e].len() != 2 {
            return Err(format!("chamfer: edge {e} has {} faces (want 2)", edge_faces[e].len()));
        }
        if d_a <= 0.0 || d_b <= 0.0 {
            return Err(format!("chamfer: edge {e} distances must be positive (got {d_a}, {d_b})"));
        }
    }

    // Chamfer requires a closed smooth chain (two chamfered edges per
    // vertex). One or 3+ needs runout/spherical-corner support not yet here.
    let mut vertex_chamfers: HashMap<usize, Vec<EdgeId>> = HashMap::new();
    for &e in want.keys() {
        let ed = solid.edges[e];
        vertex_chamfers.entry(ed.v0).or_default().push(e);
        vertex_chamfers.entry(ed.v1).or_default().push(e);
    }
    for (v, es) in &vertex_chamfers {
        if es.len() != 2 {
            return Err(format!(
                "chamfer: vertex {v} has {} chamfered edges (want 2; runout and spherical corners unsupported)",
                es.len()
            ));
        }
    }

    let face_outward = |fid: usize, p: Vec3| -> Vec3 {
        let f = &solid.faces[fid];
        let n = f.surface.normal(f.surface.project(p));
        if f.sense { n } else { -n }
    };

    let mut cm: HashMap<EdgeId, Chamfer> = HashMap::new();
    let mut want_sorted: Vec<EdgeId> = want.keys().copied().collect();
    want_sorted.sort_unstable();
    for &e in &want_sorted {
        let ed = solid.edges[e];
        let (fa, fb) = (edge_faces[e][0], edge_faces[e][1]);
        let (p0, p1) = (solid.verts[ed.v0].point, solid.verts[ed.v1].point);
        let mid = (p0 + p1) * 0.5;
        let (d_a, d_b) = want[&e];

        // Both faces must be planar (a cone-blend primitive would be needed
        // for plane/cylinder).
        let plane_a = as_plane(&solid.faces[fa].surface)
            .ok_or_else(|| format!("chamfer: edge {e} face {fa} not planar"))?;
        let plane_b = as_plane(&solid.faces[fb].surface)
            .ok_or_else(|| format!("chamfer: edge {e} face {fb} not planar"))?;
        let _ = (plane_a, plane_b);

        let edge_dir = match ed.curve {
            Curve::Line { dir, .. } => dir.normalize_or(Vec3::X),
            _ => return Err(format!("chamfer: edge {e} not a line (arc/cone chamfer unsupported)")),
        };

        let na_mid = face_outward(fa, mid);
        let nb_mid = face_outward(fb, mid);

        // In-face tangents pointing into each face's interior (away from the
        // edge). For a convex edge the cross product edge_dir × normal points
        // into one of the two faces; we pick the sign via centroid voting on
        // face a, then derive b's tangent to be consistent (the chamfer
        // surface between them must be a real Plane, so the two tangents and
        // edge_dir must be coplanar — which holds when both faces are planar
        // and meet at the edge).
        let centroid_a = face_centroid(solid, fa);
        let to_centroid = centroid_a - mid;
        let t_a_plus = edge_dir.cross(na_mid).normalize_or(Vec3::X);
        let t_a = if to_centroid.dot(t_a_plus) > 0.0 { t_a_plus } else { -t_a_plus };
        // Face b's inward tangent: by symmetry of the convex setup, the
        // inward tangent on b is the negation of (edge_dir × nb_mid) when
        // t_a picked the +sign — equivalently, pick the direction whose
        // dot with -t_a_align is positive. The robust way: vote against
        // face b's centroid.
        let centroid_b = face_centroid(solid, fb);
        let to_centroid_b = centroid_b - mid;
        let t_b_plus = edge_dir.cross(nb_mid).normalize_or(Vec3::X);
        let t_b = if to_centroid_b.dot(t_b_plus) > 0.0 { t_b_plus } else { -t_b_plus };

        // Offset endpoints on each face.
        let ta_p0 = p0 + t_a * d_a;
        let ta_p1 = p1 + t_a * d_a;
        let tb_p0 = p0 + t_b * d_b;
        let tb_p1 = p1 + t_b * d_b;

        // Chamfer surface: the Plane through ta_p0, ta_p1, tb_p0. The two
        // ta-points and the edge direction are parallel (ta_p1 - ta_p0 is
        // along edge_dir), so any 3 of the 4 offset points define the plane
        // as long as we don't pick three collinear ones.
        let v1 = ta_p1 - ta_p0; // along edge_dir
        let v2 = tb_p0 - ta_p0; // across the chamfer
        let normal = v1.cross(v2).normalize_or(Vec3::Z);
        // Orient outward: the chamfer's outward normal should sit between
        // faces a and b's outward normals (it replaces the convex edge).
        // If our computed normal points into the solid, flip it.
        let outward_test = (na_mid + nb_mid).normalize_or(Vec3::Z);
        let normal = if normal.dot(outward_test) > 0.0 { normal } else { -normal };
        let surface = Surface::plane(ta_p0, normal);

        // Sense: the chamfer meets face a along `ta`, so its outward normal
        // at ta_p0 must equal face a's outward normal `na_mid`.
        let sense = surface.normal(surface.project(ta_p0)).dot(na_mid) > 0.0;

        let fwd_a = loop_edge_dir(solid, fa, e);

        let ta = CurvEdge {
            curve: Curve::Line { p0: ta_p0, dir: edge_dir },
            t0: 0.0,
            t1: (ta_p1 - ta_p0).length(),
        };
        let tb = CurvEdge {
            curve: Curve::Line { p0: tb_p0, dir: edge_dir },
            t0: 0.0,
            t1: (tb_p1 - tb_p0).length(),
        };
        // Connect lines at the endpoints (preliminary; chain vertices will
        // have these updated to the corner intersection below).
        let ca0 = line_curve(ta_p0, tb_p0);
        let ca1 = line_curve(ta_p1, tb_p1);

        cm.insert(
            e,
            Chamfer { ta, tb, ca0, ca1, ta_p0, ta_p1, tb_p0, tb_p1, surface, sense, fwd_a },
        );
    }

    // Per consumed vertex: (corner on face a, corner on face b). For a chain
    // vertex shared by two chamfered edges, the corner on face a is the
    // intersection of the two edges' offset lines on face a — a single point
    // both edges' ta lines end at. Same for face b.
    let mut vinfo: HashMap<usize, (Vec3, Vec3)> = HashMap::new();
    for (v, es) in &vertex_chamfers {
        let (e1, e2) = (es[0], es[1]);
        // For each chamfered edge, pick its (ta, tb) offset point AT vertex v.
        // Edges store ta_p0 at v0 and ta_p1 at v1; choose by which endpoint v is.
        let pick_at_v = |e: EdgeId, c: &Chamfer| -> (Vec3, Vec3) {
            let ed = solid.edges[e];
            if ed.v0 == *v { (c.ta_p0, c.tb_p0) } else { (c.ta_p1, c.tb_p1) }
        };
        let (off_a1, off_b1) = pick_at_v(e1, &cm[&e1]);
        let (off_a2, off_b2) = pick_at_v(e2, &cm[&e2]);
        // Direction of each edge leaving v: +dir if v is v0, -dir if v is v1.
        let edge_dir_at = |e: EdgeId| -> Vec3 {
            let ed = solid.edges[e];
            let Curve::Line { dir, .. } = ed.curve else { return Vec3::X };
            if ed.v0 == *v { dir.normalize_or(Vec3::X) } else { -dir.normalize_or(Vec3::X) }
        };
        let d1 = edge_dir_at(e1);
        let d2 = edge_dir_at(e2);
        let corner_a = intersect_lines(off_a1, d1, off_a2, d2)
            .ok_or_else(|| format!("chamfer: chain vertex {v} offset lines parallel on face a"))?;
        let corner_b = intersect_lines(off_b1, d1, off_b2, d2)
            .ok_or_else(|| format!("chamfer: chain vertex {v} offset lines parallel on face b"))?;
        vinfo.insert(*v, (corner_a, corner_b));

        // Update both chamfers' endpoints + connect lines to share the corner.
        // The ca at this vertex runs corner_a → corner_b on both edges, so
        // they weld by interning when re-emitted.
        for e in [e1, e2] {
            let ed_e = solid.edges[e];
            let at_v0 = ed_e.v0 == *v;
            let c = cm.get_mut(&e).unwrap();
            let dir = edge_dir_of(solid, e);
            if at_v0 {
                c.ta_p0 = corner_a;
                c.tb_p0 = corner_b;
                c.ca0 = line_curve(corner_a, corner_b);
                c.ta = CurvEdge { curve: Curve::Line { p0: corner_a, dir }, t0: 0.0, t1: (c.ta_p1 - corner_a).length() };
                c.tb = CurvEdge { curve: Curve::Line { p0: corner_b, dir }, t0: 0.0, t1: (c.tb_p1 - corner_b).length() };
            } else {
                c.ta_p1 = corner_a;
                c.tb_p1 = corner_b;
                c.ca1 = line_curve(corner_a, corner_b);
                c.ta = CurvEdge { curve: Curve::Line { p0: c.ta_p0, dir }, t0: 0.0, t1: (corner_a - c.ta_p0).length() };
                c.tb = CurvEdge { curve: Curve::Line { p0: c.tb_p0, dir }, t0: 0.0, t1: (corner_b - c.tb_p0).length() };
            }
        }
    }

    // Rebuild the solid through a fresh builder, mirroring fillet's
    // rebuild_loop but simpler (no runout splice).
    let mut b = Builder::new();
    for fi in 0..solid.faces.len() {
        let outer = rebuild_loop(solid, &cm, &vinfo, &want, fi, solid.outer_edges(fi), &mut b)?;
        let mut inners = Vec::with_capacity(solid.n_inners(fi));
        for lp in solid.inner_loops(fi) {
            inners.push(rebuild_loop(solid, &cm, &vinfo, &want, fi, lp, &mut b)?);
        }
        let (surface, sense) = (solid.faces[fi].surface, solid.faces[fi].sense);
        b.face(surface, sense, outer, inners);
    }

    // Emit one chamfer face per chamfered edge.
    let mut chamfer_keys: Vec<EdgeId> = cm.keys().copied().collect();
    chamfer_keys.sort_unstable();
    for k in chamfer_keys {
        let c = &cm[&k];
        let e_ta = emit_curv(&mut b, c.ta_p0, c.ta_p1, c.ta);
        let e_tb = emit_curv(&mut b, c.tb_p0, c.tb_p1, c.tb);
        let e_ca0 = emit_curv(&mut b, c.ta_p0, c.tb_p0, c.ca0);
        let e_ca1 = emit_curv(&mut b, c.ta_p1, c.tb_p1, c.ca1);
        // Orient the chamfer loop so ta opposes face a's traversal and tb
        // opposes face b's. Matches fillet's loop orientation logic.
        let lp = if c.fwd_a {
            Loop::new(vec![(e_ta.0, !e_ta.1), e_ca0, e_tb, (e_ca1.0, !e_ca1.1)])
        } else {
            Loop::new(vec![e_ta, e_ca1, (e_tb.0, !e_tb.1), (e_ca0.0, !e_ca0.1)])
        };
        b.face(c.surface, c.sense, lp, vec![]);
    }

    let s = b.build();
    if let Err(e) = s.validate() {
        return Err(format!("chamfer: rebuilt solid invalid: {e}"));
    }
    Ok(s)
}

/// Helper: extract the edge's line direction as a normalized Vec3.
fn edge_dir_of(solid: &Solid, e: EdgeId) -> Vec3 {
    match solid.edges[e].curve {
        Curve::Line { dir, .. } => dir.normalize_or(Vec3::X),
        _ => Vec3::X,
    }
}

/// How a face traverses `e` in its loops (true = v0→v1).
fn loop_edge_dir(solid: &Solid, fid: usize, e: EdgeId) -> bool {
    for lp in solid.face_loops(fid) {
        for &(ee, f) in lp {
            if ee == e {
                return f;
            }
        }
    }
    true
}

/// Average position of a face's outer-loop vertices (a robust interior hint).
fn face_centroid(solid: &Solid, fid: usize) -> Vec3 {
    let mut sum = Vec3::ZERO;
    let mut n = 0;
    for &(e, fwd) in solid.outer_edges(fid) {
        let ed = solid.edges[e];
        let v = if fwd { ed.v0 } else { ed.v1 };
        sum += solid.verts[v].point;
        n += 1;
    }
    if n > 0 { sum / n as f32 } else { Vec3::ZERO }
}

fn as_plane(s: &Surface) -> Option<(Vec3, Vec3)> {
    if let Surface::Plane { origin, normal, .. } = s {
        Some((*origin, *normal))
    } else {
        None
    }
}

/// Intersection of two 3D lines (p0 + t*d0) and (p1 + s*d1), if any. Returns
/// None if they're parallel or skew beyond tolerance.
fn intersect_lines(p0: Vec3, d0: Vec3, p1: Vec3, d1: Vec3) -> Option<Vec3> {
    let d0 = d0.normalize_or(Vec3::X);
    let d1 = d1.normalize_or(Vec3::X);
    let cross = d0.cross(d1);
    let denom = cross.length_squared();
    if denom < 1e-9 {
        return None; // parallel
    }
    // Closest-point-on-each-line parametric solve.
    let w = p1 - p0;
    let t = (w.cross(d1)).dot(cross) / denom;
    let s = (w.cross(d0)).dot(cross) / denom;
    let a = p0 + d0 * t;
    let b = p1 + d1 * s;
    if (a - b).length() > 1e-3 {
        return None; // skew
    }
    Some((a + b) * 0.5)
}

fn line_curve(a: Vec3, b: Vec3) -> CurvEdge {
    let dir = (b - a).normalize_or(Vec3::X);
    CurvEdge { curve: Curve::Line { p0: a, dir }, t0: 0.0, t1: (b - a).length() }
}

fn rebuild_loop(
    solid: &Solid,
    cm: &HashMap<EdgeId, Chamfer>,
    vinfo: &HashMap<usize, (Vec3, Vec3)>,
    want: &HashMap<EdgeId, (f32, f32)>,
    fi: usize,
    lp: &[(EdgeId, bool)],
    b: &mut Builder,
) -> Result<Loop, String> {
    let ef = solid.edge_faces();
    let mut out: Vec<(EdgeId, bool)> = Vec::with_capacity(lp.len());
    for &(e, fwd) in lp {
        let ed = solid.edges[e];
        if want.contains_key(&e) {
            let c = &cm[&e];
            let side_a = ef[e][0] == fi;
            let ce = if side_a { c.ta } else { c.tb };
            let (tp0, tp1) = if side_a { (c.ta_p0, c.ta_p1) } else { (c.tb_p0, c.tb_p1) };
            let (start, end) = if fwd { (tp0, tp1) } else { (tp1, tp0) };
            out.push(emit_curv(b, start, end, ce));
        } else {
            // Plain edge: emit unchanged, but move each endpoint that is a
            // chain vertex to the corner on *this* face. The chain vertex's
            // two corners (pa on face a, pb on face b) are on different faces;
            // pick by whether `fi` is face a or face b of the chamfered pair.
            let pos0 = solid.verts[ed.v0].point;
            let pos1 = solid.verts[ed.v1].point;
            let new0 = moved_vertex(solid, vinfo, ed.v0, pos0, fi, &ef);
            let new1 = moved_vertex(solid, vinfo, ed.v1, pos1, fi, &ef);
            let (start, end) = if fwd { (new0, new1) } else { (new1, new0) };
            let vs = b.vertex(start);
            let ve = b.vertex(end);
            let eid = match ed.curve {
                Curve::Line { .. } => b.line(vs, ve),
                Curve::Circle { center, axis, radius, ref_dir } => {
                    let (a0, a1) = if fwd { (ed.t0, ed.t1) } else { (ed.t1, ed.t0) };
                    b.arc(vs, ve, center, axis, radius, ref_dir, a0, a1)
                }
                Curve::Ellipse { center, a: ea, b: eb } => {
                    let (t0, t1) = if fwd { (ed.t0, ed.t1) } else { (ed.t1, ed.t0) };
                    b.ellipse(vs, ve, center, ea, eb, t0, t1)
                }
            };
            out.push(eid);
        }
    }
    Ok(Loop::new(out))
}

/// Move a vertex that is a chamfer chain corner to the corner position on
/// face `fi`. The chain vertex's `vinfo` carries two corners: `(pa, pb)`
/// where pa is on the face-a side of the chamfered pair and pb on face-b.
///
/// `fi` is one of three: face a (use pa), face b (use pb), or a third face
/// passing through v that is not part of the chamfered pair (the vertex
/// stays put — chamfer doesn't move it). For a closed convex chain, every
/// face at v is one of the chamfer's two faces, so the third case doesn't
/// arise; kept as a fallback for safety.
fn moved_vertex(
    solid: &Solid,
    vinfo: &HashMap<usize, (Vec3, Vec3)>,
    v: usize,
    fallback: Vec3,
    fi: usize,
    ef: &[Vec<usize>],
) -> Vec3 {
    let Some(&(pa, pb)) = vinfo.get(&v) else {
        return fallback;
    };
    // Find the chamfered edge at v (any edge in ef[e] containing fi that has
    // a vinfo entry). For that edge, decide if fi is its face-a or face-b.
    for e in 0..solid.edges.len() {
        if ef[e].contains(&fi) && vinfo.contains_key(&v) {
            // Edge e has v as one endpoint and fi as one of its two faces.
            let ed = solid.edges[e];
            if ed.v0 != v && ed.v1 != v {
                continue;
            }
            return if ef[e][0] == fi { pa } else { pb };
        }
    }
    // No chamfered edge connects fi and v: vertex stays at fallback.
    fallback
}

/// Emit a CurvEdge between two points in start→end direction. Returns the
/// edge id and whether the stored edge runs start→end.
fn emit_curv(b: &mut Builder, start: Vec3, end: Vec3, ce: CurvEdge) -> (EdgeId, bool) {
    let vs = b.vertex(start);
    let ve = b.vertex(end);
    let forward = || {
        let at_start = ce.curve.point(ce.t0);
        (at_start - start).length() < (ce.curve.point(ce.t1) - start).length()
    };
    match ce.curve {
        Curve::Line { .. } => b.line(vs, ve),
        Curve::Circle { center, axis, radius, ref_dir } => {
            let (t0, t1) = if forward() { (ce.t0, ce.t1) } else { (ce.t1, ce.t0) };
            b.arc(vs, ve, center, axis, radius, ref_dir, t0, t1)
        }
        Curve::Ellipse { center, a: ea, b: eb } => {
            let (t0, t1) = if forward() { (ce.t0, ce.t1) } else { (ce.t1, ce.t0) };
            b.ellipse(vs, ve, center, ea, eb, t0, t1)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kernel::build::extrude;
    use crate::kernel::sketch::Sketch;

    /// Box from 0..10 in x and y, 0..5 in z. The top rim (4 edges at z=5) is
    /// a clean target for chamfering.
    fn box_solid() -> Solid {
        let s = Sketch::rectangle(5.0, 5.0, 10.0, 10.0);
        extrude(&s, 0.0, 5.0)
    }

    #[test]
    fn intersect_lines_perpendicular() {
        let p = intersect_lines(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(5.0, -2.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        )
        .expect("perpendicular lines intersect");
        assert!((p - Vec3::new(5.0, 0.0, 0.0)).length() < 1e-4, "got {p}");
    }

    #[test]
    fn intersect_parallel_returns_none() {
        let p = intersect_lines(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
        );
        assert!(p.is_none(), "parallel lines don't intersect");
    }

    #[test]
    fn chamfer_top_rim_of_a_box_is_manifold() {
        let s = box_solid();
        // Pick the 4 top-rim edges by scanning for edges at z=5.
        let top_edges: Vec<EdgeId> = (0..s.edges.len())
            .filter(|&e| {
                let ed = s.edges[e];
                let z0 = s.verts[ed.v0].point.z;
                let z1 = s.verts[ed.v1].point.z;
                (z0 - 5.0).abs() < 1e-3 && (z1 - 5.0).abs() < 1e-3
            })
            .collect();
        assert_eq!(top_edges.len(), 4, "box top rim has 4 edges");
        let chamfers: Vec<(EdgeId, f32, f32)> =
            top_edges.into_iter().map(|e| (e, 1.0, 1.0)).collect();
        let c = chamfer_edges(&s, &chamfers).expect("chamfer");
        c.validate().expect("chamfered box is manifold");
        // Each chamfer adds one new face; the original top face is replaced
        // by a smaller face; the 4 side faces are trimmed. Net face count
        // rises by 4 (the chamfer bevels).
        assert!(c.faces.len() >= s.faces.len() + 4, "chamfer added bevel faces");
    }

    #[test]
    fn asymmetric_chamfer_tilts_the_bevel() {
        // d_a != d_b: the chamfer plane is tilted relative to 45°. Test by
        // building a box, chamfering with (1, 2), and verifying the chamfer
        // surface is a Plane whose normal is neither face's normal.
        let s = box_solid();
        let top_edges: Vec<EdgeId> = (0..s.edges.len())
            .filter(|&e| {
                let ed = s.edges[e];
                (s.verts[ed.v0].point.z - 5.0).abs() < 1e-3
                    && (s.verts[ed.v1].point.z - 5.0).abs() < 1e-3
            })
            .collect();
        let chamfers: Vec<(EdgeId, f32, f32)> =
            top_edges.into_iter().map(|e| (e, 1.0, 2.0)).collect();
        let c = chamfer_edges(&s, &chamfers).expect("chamfer");
        c.validate().expect("asymmetric chamfer box is manifold");
        // Find the chamfer face: it's a Plane that isn't axis-aligned.
        let has_tilted_plane = c.faces.iter().any(|f| match f.surface {
            Surface::Plane { normal, .. } => {
                normal.x.abs() > 0.1 && normal.z.abs() > 0.1
            }
            _ => false,
        });
        assert!(has_tilted_plane, "asymmetric chamfer should produce a tilted plane");
    }

    #[test]
    fn chamfer_rejects_open_chain() {
        // Chamfering 3 of the 4 top-rim edges leaves one vertex with only one
        // chamfered edge — an open chain, which is unsupported.
        let s = box_solid();
        let top_edges: Vec<EdgeId> = (0..s.edges.len())
            .filter(|&e| {
                let ed = s.edges[e];
                (s.verts[ed.v0].point.z - 5.0).abs() < 1e-3
                    && (s.verts[ed.v1].point.z - 5.0).abs() < 1e-3
            })
            .take(3)
            .collect();
        let chamfers: Vec<(EdgeId, f32, f32)> =
            top_edges.into_iter().map(|e| (e, 1.0, 1.0)).collect();
        let err = chamfer_edges(&s, &chamfers).unwrap_err();
        assert!(err.contains("want 2"), "open chain should error cleanly; got: {err}");
    }
}
