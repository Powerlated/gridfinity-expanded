//! Rolling-ball edge blending: a true B-rep fillet operator.
//!
//! `blend_edges` consumes a `Solid` and a list of `(EdgeId, radius)` blends and
//! returns a new `Solid` in which each blended edge is replaced by a smooth
//! blend face — an analytic `Cylinder` for a plane/plane edge, a `Torus` for a
//! plane/cylinder coaxial edge — while the two adjacent faces are trimmed back
//! to the exact tangent curves. Adjacent blends in a smooth chain share a
//! connect arc at each common vertex (the quarter-circle cross-section of the
//! rolling ball), which the rebuild welds into one shared edge automatically.
//!
//! The blend geometry is exact rolling-ball. For an edge between faces with
//! effective outward normals `m_a`, `m_b` (which already point into the blend
//! region — into the cavity for an internal edge, into the exterior for a
//! convex edge), the ball centre at a point `P` of the edge is
//!
//! ```text
//!     C = P + r · (m_a + m_b) / |m_a × m_b|
//! ```
//!
//! and the tangent point on each face is `C − r · m_face`. Uniform over
//! concave and convex edges.
//!
//! Scope: every blended vertex must be shared by exactly two blended edges (a
//! closed smooth chain, e.g. a pocket floor-wall loop). Partial / non-smooth
//! vertices needing a spherical corner patch are rejected for now.

use crate::geom::{Curve, Surface};
use crate::math::Vec3;
use crate::topo::{Builder, EdgeId, Loop, Solid};
use std::collections::HashMap;

/// A curve with its parameter range, as emitted between two endpoints.
#[derive(Clone, Copy)]
struct CurvEdge {
    curve: Curve,
    t0: f32,
    t1: f32,
}

#[derive(Clone)]
struct Blend {
    ta: CurvEdge, // tangent on face a, p0→p1
    tb: CurvEdge, // tangent on face b, p0→p1
    ca0: CurvEdge, // connect arc at p0, ta_p0→tb_p0
    ca1: CurvEdge, // connect arc at p1, ta_p1→tb_p1
    ta_p0: Vec3,
    ta_p1: Vec3,
    tb_p0: Vec3,
    tb_p1: Vec3,
    surface: Surface,
    sense: bool,
    // How face a traverses the original edge (v0→v1 = true). The blend loop must
    // oppose it on both shared tangents (manifold invariant).
    fwd_a: bool,
}

/// Blend a set of edges of `solid` by the given radii.
pub fn blend_edges(solid: &Solid, blends: &[(EdgeId, f32)]) -> Result<Solid, String> {
    let want: HashMap<EdgeId, f32> = blends.iter().copied().collect();
    let edge_faces = solid.edge_faces();

    for &e in want.keys() {
        if e >= solid.edges.len() {
            return Err(format!("blend: edge {e} out of range"));
        }
        if edge_faces[e].len() != 2 {
            return Err(format!("blend: edge {e} has {} faces (want 2)", edge_faces[e].len()));
        }
    }

    // Every blended vertex must be shared by exactly two blended edges.
    let mut vertex_blends: HashMap<usize, Vec<EdgeId>> = HashMap::new();
    for &e in want.keys() {
        let ed = solid.edges[e];
        vertex_blends.entry(ed.v0).or_default().push(e);
        vertex_blends.entry(ed.v1).or_default().push(e);
    }
    for (v, es) in &vertex_blends {
        if es.len() != 2 {
            return Err(format!(
                "blend: vertex {v} has {} blended edges (want 2; spherical corners unsupported)",
                es.len()
            ));
        }
    }

    let face_outward = |fid: usize, p: Vec3| -> Vec3 {
        let f = &solid.faces[fid];
        let n = f.surface.normal(f.surface.project(p));
        if f.sense { n } else { -n }
    };

    let mut bm: HashMap<EdgeId, Blend> = HashMap::new();
    let mut want_sorted: Vec<EdgeId> = want.keys().copied().collect();
    want_sorted.sort_unstable();
    for &e in &want_sorted {
        let ed = solid.edges[e];
        let (fa, fb) = (edge_faces[e][0], edge_faces[e][1]);
        let (p0, p1) = (solid.verts[ed.v0].point, solid.verts[ed.v1].point);
        let mid = (p0 + p1) * 0.5;
        let r = want[&e];
        let na_mid = face_outward(fa, mid);
        let nb_mid = face_outward(fb, mid);
        let sin_mid = na_mid.cross(nb_mid).length();
        if sin_mid < 1e-6 || r <= 0.0 {
            return Err(format!("blend: edge {e} degenerate (parallel faces or r≤0)"));
        }
        // Fillet removes material: pick the sign that pulls the midpoint
        // tangent toward face a's interior (centroid). Convexity is constant
        // along an edge, so one sign decision suffices.
        let centroid_a = face_centroid(solid, fa);
        let to_centroid = centroid_a - mid;
        let ta_plus = mid + r * (na_mid + nb_mid) / sin_mid - r * na_mid;
        let ta_minus = mid - r * (na_mid + nb_mid) / sin_mid + r * na_mid;
        let s = if to_centroid.dot(ta_minus - mid) > to_centroid.dot(ta_plus - mid) {
            -1.0
        } else {
            1.0
        };
        // Face normals vary along a curved edge: evaluate locally at each end.
        let (na0, nb0) = (face_outward(fa, p0), face_outward(fb, p0));
        let (na1, nb1) = (face_outward(fa, p1), face_outward(fb, p1));
        let (ma0, mb0) = (s * na0, s * nb0);
        let (ma1, mb1) = (s * na1, s * nb1);
        let sin0 = ma0.cross(mb0).length().max(1e-9);
        let sin1 = ma1.cross(mb1).length().max(1e-9);
        let cv0 = p0 + r * (ma0 + mb0) / sin0;
        let cv1 = p1 + r * (ma1 + mb1) / sin1;
        let ta_p0 = cv0 - r * ma0;
        let ta_p1 = cv1 - r * ma1;
        let tb_p0 = cv0 - r * mb0;
        let tb_p1 = cv1 - r * mb1;
        let ma = ma0;

        let plane_a = as_plane(&solid.faces[fa].surface);
        let plane_b = as_plane(&solid.faces[fb].surface);
        let cyl = as_cyl(&solid.faces[fa].surface).or_else(|| as_cyl(&solid.faces[fb].surface));
        let is_circle = matches!(ed.curve, Curve::Circle { .. });

        // How face a traverses edge e in its loop (determines blend loop orientation).
        let fwd_a = loop_edge_dir(&solid.faces[fa], e);

        let blend = if plane_a.is_some() && plane_b.is_some() {
            build_cyl_blend(ed, cv0, cv1, ma, na0, ta_p0, ta_p1, tb_p0, tb_p1, r, fwd_a)?
        } else if cyl.is_some() && is_circle && (plane_a.is_some() || plane_b.is_some()) {
            // ta/tb stay per-face (ta on fa, tb on fb); the torus is symmetric.
            build_torus_blend(ed, cv0, cv1, na0, ta_p0, ta_p1, tb_p0, tb_p1, r, cyl.unwrap(), fwd_a)?
        } else {
            return Err(format!(
                "blend: edge {e} pair not supported (only plane/plane or plane/coaxial-cylinder)"
            ));
        };
        bm.insert(e, blend);
    }

    // Per consumed vertex: (point on face-a side, point on face-b side).
    let mut vinfo: HashMap<usize, (Vec3, Vec3)> = HashMap::new();
    for (e, bld) in &bm {
        let ed = solid.edges[*e];
        vinfo.insert(ed.v0, (bld.ta_p0, bld.tb_p0));
        vinfo.insert(ed.v1, (bld.ta_p1, bld.tb_p1));
    }

    let mut b = Builder::new();

    for (fi, face) in solid.faces.iter().enumerate() {
        let outer = rebuild_loop(solid, &bm, &vinfo, &want, fi, &face.outer, &mut b)?;
        let mut inners = Vec::with_capacity(face.inners.len());
        for lp in &face.inners {
            inners.push(rebuild_loop(solid, &bm, &vinfo, &want, fi, lp, &mut b)?);
        }
        b.face(face.surface, face.sense, outer, inners);
    }

    let mut blend_keys: Vec<EdgeId> = bm.keys().copied().collect();
    blend_keys.sort_unstable();
    for k in blend_keys {
        let bld = &bm[&k];
        let e_ta = emit_curv(&mut b, bld.ta_p0, bld.ta_p1, bld.ta);
        let e_tb = emit_curv(&mut b, bld.tb_p0, bld.tb_p1, bld.tb);
        let e_ca0 = emit_curv(&mut b, bld.ta_p0, bld.tb_p0, bld.ca0);
        let e_ca1 = emit_curv(&mut b, bld.ta_p1, bld.tb_p1, bld.ca1);
        // Orient the blend loop so ta opposes face a's traversal of the original
        // edge and tb opposes face b's (fwd_b = !fwd_a for a manifold edge). This
        // is what makes the rebuilt solid pass the 1:1 edge invariant.
        let lp = if bld.fwd_a {
            // ta: p1→p0, ca0: p0→p0b, tb: p0b→p1b, ca1: p1b→p1.
            Loop::new(vec![
                (e_ta.0, !e_ta.1),
                e_ca0,
                e_tb,
                (e_ca1.0, !e_ca1.1),
            ])
        } else {
            Loop::new(vec![e_ta, e_ca1, (e_tb.0, !e_tb.1), (e_ca0.0, !e_ca0.1)])
        };
        b.face(bld.surface, bld.sense, lp, vec![]);
    }

    let s = b.build();
    if let Err(e) = s.validate() {
        return Err(format!("blend: rebuilt solid invalid: {e}"));
    }
    Ok(s)
}

/// How a face traverses `e` in its loops (true = v0→v1).
fn loop_edge_dir(face: &crate::topo::Face, e: EdgeId) -> bool {
    for lp in face.loops() {
        for &(ee, f) in &lp.edges {
            if ee == e {
                return f;
            }
        }
    }
    true
}

/// Average position of a face's outer-loop vertices (a robust interior hint).
fn face_centroid(solid: &Solid, fid: usize) -> Vec3 {
    let face = &solid.faces[fid];
    let mut sum = Vec3::ZERO;
    let mut n = 0;
    for &(e, fwd) in &face.outer.edges {
        let ed = solid.edges[e];
        let v = if fwd { ed.v0 } else { ed.v1 };
        sum += solid.verts[v].point;
        n += 1;
    }
    if n > 0 { sum / n as f32 } else { Vec3::ZERO }
}

fn as_plane(s: &Surface) -> Option<(Vec3, Vec3)> {    if let Surface::Plane { origin, normal, .. } = s {
        Some((*origin, *normal))
    } else {
        None
    }
}

fn as_cyl(s: &Surface) -> Option<(Vec3, Vec3, f32)> {
    if let Surface::Cylinder { base, axis, radius, .. } = s {
        Some((*base, *axis, *radius))
    } else {
        None
    }
}

#[allow(clippy::too_many_arguments)]
fn build_cyl_blend(
    ed: crate::topo::Edge,
    cv0: Vec3,
    cv1: Vec3,
    ma: Vec3,
    na0: Vec3,
    ta_p0: Vec3,
    ta_p1: Vec3,
    tb_p0: Vec3,
    tb_p1: Vec3,
    r: f32,
    fwd_a: bool,
) -> Result<Blend, String> {
    let dir = match ed.curve {
        Curve::Line { dir, .. } => dir,
        _ => return Err("cyl blend: edge not a line".into()),
    };
    let ref_dir = (-ma).normalize_or(Vec3::X);

    let ta = CurvEdge { curve: Curve::Line { p0: ta_p0, dir }, t0: 0.0, t1: (ta_p1 - ta_p0).length() };
    let tb = CurvEdge { curve: Curve::Line { p0: tb_p0, dir }, t0: 0.0, t1: (tb_p1 - tb_p0).length() };

    let ca0 = connect_arc(cv0, dir, ta_p0, tb_p0)?;
    let ca1 = connect_arc(cv1, dir, ta_p1, tb_p1)?;

    let surface = Surface::Cylinder { base: cv0, axis: dir, radius: r, ref_dir };
    // Sense: the blend must meet face a tangentially, so its outward normal at
    // the ta tangent equals face a's outward normal `na0` (NOT the blend-side
    // normal `ma`, which is sign-flipped on convex edges).
    let sense = surface.normal(surface.project(ta_p0)).dot(na0) > 0.0;

    Ok(Blend { ta, tb, ca0, ca1, ta_p0, ta_p1, tb_p0, tb_p1, surface, sense, fwd_a })
}

#[allow(clippy::too_many_arguments)]
fn build_torus_blend(
    ed: crate::topo::Edge,
    cv0: Vec3,
    cv1: Vec3,
    na0: Vec3,
    ta_p0: Vec3,
    ta_p1: Vec3,
    tb_p0: Vec3,
    tb_p1: Vec3,
    r: f32,
    cyl: (Vec3, Vec3, f32),
    fwd_a: bool,
) -> Result<Blend, String> {
    let (cyl_base, cyl_axis, _cyl_radius) = cyl;
    let (edge_center, edge_axis, edge_radius, edge_ref_dir) = match ed.curve {
        Curve::Circle { center, axis, radius, ref_dir } => (center, axis, radius, ref_dir),
        _ => return Err("torus blend: edge not a circle".into()),
    };
    let (a0, a1) = (ed.t0, ed.t1);
    let cv0_on = cyl_base + cyl_axis * (cv0 - cyl_base).dot(cyl_axis);
    let major = (cv0 - cv0_on).length();
    let torus_center = cv0_on;
    let torus_axis = edge_axis;
    // Reuse the edge's own ref_dir so the original angles a0,a1 map correctly.
    let ref_dir = edge_ref_dir;
    let surface = Surface::Torus {
        center: torus_center,
        axis: torus_axis,
        major_r: major,
        minor_r: r,
        ref_dir,
    };
    let _ = (edge_center, edge_radius);

    // Tangent circles coaxial with the torus, at the plane/cyl contact radii.
    let ta_center = torus_center + torus_axis * (ta_p0 - torus_center).dot(torus_axis);
    let ta_r = (ta_p0 - ta_center).length();
    let tb_center = torus_center + torus_axis * (tb_p0 - torus_center).dot(torus_axis);
    let tb_r = (tb_p0 - tb_center).length();
    let ta = CurvEdge {
        curve: Curve::Circle { center: ta_center, axis: torus_axis, radius: ta_r, ref_dir },
        t0: a0,
        t1: a1,
    };
    let tb = CurvEdge {
        curve: Curve::Circle { center: tb_center, axis: torus_axis, radius: tb_r, ref_dir },
        t0: a0,
        t1: a1,
    };

    // Connect arcs: axis = circle tangent at the endpoint (G1 with neighbours).
    let p0 = ed.curve.point(a0);
    let tan_at = |p: Vec3| {
        let v = p - torus_center;
        let perp = v - torus_axis * v.dot(torus_axis);
        torus_axis.cross(perp.normalize_or(Vec3::X))
    };
    let ca0 = connect_arc(cv0, tan_at(p0), ta_p0, tb_p0)?;
    let p1 = ed.curve.point(a1);
    let ca1 = connect_arc(cv1, tan_at(p1), ta_p1, tb_p1)?;

    // Sense: blend outward at the ta tangent equals face a's outward normal.
    let sense = surface.normal(surface.project(ta_p0)).dot(na0) > 0.0;

    Ok(Blend { ta, tb, ca0, ca1, ta_p0, ta_p1, tb_p0, tb_p1, surface, sense, fwd_a })
}

/// Quarter-circle connect arc about `center`, axis `axis`, from `from_pt` to
/// `to_pt`. ref_dir = (from_pt − center); sweep = short signed angle to to_pt.
fn connect_arc(center: Vec3, axis: Vec3, from_pt: Vec3, to_pt: Vec3) -> Result<CurvEdge, String> {
    let ref_dir = (from_pt - center).normalize_or(Vec3::X);
    let d1 = axis.cross(ref_dir);
    let sweep = {
        let v = to_pt - center;
        let mut a = v.dot(d1).atan2(v.dot(ref_dir));
        while a > std::f32::consts::PI {
            a -= 2.0 * std::f32::consts::PI;
        }
        while a < -std::f32::consts::PI {
            a += 2.0 * std::f32::consts::PI;
        }
        a
    };
    Ok(CurvEdge {
        curve: Curve::Circle {
            center,
            axis,
            radius: (from_pt - center).length(),
            ref_dir,
        },
        t0: 0.0,
        t1: sweep,
    })
}

fn rebuild_loop(
    solid: &Solid,
    bm: &HashMap<EdgeId, Blend>,
    vinfo: &HashMap<usize, (Vec3, Vec3)>,
    want: &HashMap<EdgeId, f32>,
    fi: usize,
    lp: &Loop,
    b: &mut Builder,
) -> Result<Loop, String> {
    let face_surface = solid.faces[fi].surface;
    let ef = solid.edge_faces();
    let mut out = Vec::with_capacity(lp.edges.len());
    for &(e, fwd) in &lp.edges {
        if want.contains_key(&e) {
            let bld = &bm[&e];
            let side_a = ef[e][0] == fi;
            let ce = if side_a { bld.ta } else { bld.tb };
            let (tp0, tp1) = if side_a { (bld.ta_p0, bld.ta_p1) } else { (bld.tb_p0, bld.tb_p1) };
            let (start, end) = if fwd { (tp0, tp1) } else { (tp1, tp0) };
            out.push(emit_curv(b, start, end, ce));
        } else {
            let ed = solid.edges[e];
            let pos0 = solid.verts[ed.v0].point;
            let pos1 = solid.verts[ed.v1].point;
            let new0 = move_vertex(vinfo, ed.v0, pos0, face_surface);
            let new1 = move_vertex(vinfo, ed.v1, pos1, face_surface);
            let (start, end) = if fwd { (new0, new1) } else { (new1, new0) };
            let vs = b.vertex(start);
            let ve = b.vertex(end);
            let eid = match ed.curve {
                Curve::Line { .. } => b.line(vs, ve),
                Curve::Circle { center, axis, radius, ref_dir } => {
                    // The builder trusts that `vs` sits at the first angle, so
                    // swap the angle range when emitting the arc reversed.
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

fn move_vertex(vinfo: &HashMap<usize, (Vec3, Vec3)>, v: usize, fallback: Vec3, surface: Surface) -> Vec3 {
    if let Some((pa, pb)) = vinfo.get(&v) {
        if dist_to_surface(*pa, surface) < dist_to_surface(*pb, surface) { *pa } else { *pb }
    } else {
        fallback
    }
}

fn dist_to_surface(p: Vec3, s: Surface) -> f32 {
    match s {
        Surface::Plane { origin, normal, .. } => (p - origin).dot(normal).abs(),
        Surface::Cylinder { base, axis, radius, .. } => {
            let rel = p - base;
            (rel - axis * rel.dot(axis)).length() - radius
        }
        Surface::Sphere { center, radius, .. } => (p - center).length() - radius,
        _ => (s.point(s.project(p)) - p).length(),
    }
    .abs()
}

/// Emit a CurvEdge between two points in start→end direction. Returns the edge
/// id and whether the stored edge runs start→end.
fn emit_curv(b: &mut Builder, start: Vec3, end: Vec3, ce: CurvEdge) -> (EdgeId, bool) {
    let vs = b.vertex(start);
    let ve = b.vertex(end);
    match ce.curve {
        Curve::Line { .. } => b.line(vs, ve),
        Curve::Ellipse { .. } => unreachable!("blend construction emits only lines and circles"),
        Curve::Circle { center, axis, radius, ref_dir } => {
            // Walk the stored t0→t1 if it already maps start→end; else reverse.
            let at_start = ce.curve.point(ce.t0);
            let forward = (at_start - start).length() < (ce.curve.point(ce.t1) - start).length();
            if forward {
                b.arc(vs, ve, center, axis, radius, ref_dir, ce.t0, ce.t1)
            } else {
                b.arc(vs, ve, center, axis, radius, ref_dir, ce.t1, ce.t0)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Vec3;

    fn approx(a: Vec3, b: Vec3) -> bool {
        (a - b).length() < 1e-4
    }

    /// Pure rolling-ball math: a 90° concave corner (floor + wall) must put the
    /// ball centre, floor-tangent and wall-tangent at the textbook positions.
    #[test]
    fn rolling_ball_corner_math() {
        // Wall plane x=10 (cavity on -x side), floor plane z=0 (cavity above).
        // Outward normals already point into the cavity: m_a = +Z (floor up),
        // m_b = -X (wall into cavity). Edge point P = (10, 0, 0).
        let p = Vec3::new(10.0, 0.0, 0.0);
        let ma = Vec3::new(0.0, 0.0, 1.0);
        let mb = Vec3::new(-1.0, 0.0, 0.0);
        let r = 2.0_f32;
        let sin_theta = ma.cross(mb).length(); // = 1
        let c = p + r * (ma + mb) / sin_theta;
        assert!(approx(c, Vec3::new(8.0, 0.0, 2.0)), "ball centre {c}");
        let ta = c - r * ma; // on floor
        let tb = c - r * mb; // on wall
        assert!(approx(ta, Vec3::new(8.0, 0.0, 0.0)), "floor tangent {ta}");
        assert!(approx(tb, Vec3::new(10.0, 0.0, 2.0)), "wall tangent {tb}");
    }

    /// A connect arc from `from_pt` to `to_pt` must actually end at those points.
    #[test]
    fn connect_arc_endpoints() {
        let center = Vec3::new(8.0, 0.0, 2.0);
        let axis = Vec3::new(0.0, 1.0, 0.0);
        let from = Vec3::new(8.0, 0.0, 0.0); // floor tangent
        let to = Vec3::new(10.0, 0.0, 2.0); // wall tangent
        let ce = connect_arc(center, axis, from, to).unwrap();
        assert!(approx(ce.curve.point(ce.t0), from), "arc start");
        assert!(approx(ce.curve.point(ce.t1), to), "arc end");
        // And it must be a quarter circle (sweep magnitude π/2).
        assert!(((ce.t1 - ce.t0).abs() - std::f32::consts::FRAC_PI_2).abs() < 1e-4);
    }
}

