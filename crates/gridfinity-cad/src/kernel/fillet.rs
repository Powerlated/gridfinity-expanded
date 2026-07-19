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
//! A blended vertex shared by **two** blended edges continues the chain (the
//! two blends share a connect arc). A vertex with **one** blended edge is a
//! *runout*: the chain terminates against a third face, and the blend surface
//! is trimmed by that face instead of closed with a quarter arc. For a
//! cylindrical blend cut by an oblique plane the exact trim curve is an
//! ellipse arc (`Curve::Ellipse`), and the two tangent curves are extended to
//! meet the plane. The runout face gets that ellipse spliced into its loop
//! where its sharp corner used to be, so the arc is used exactly twice.
//!
//! Scope: vertices with three or more blended edges (spherical corner patches)
//! are still rejected, and runout is implemented for cylindrical blends only.

use crate::kernel::geom::{Curve, Surface};
use crate::kernel::math::Vec3;
use crate::kernel::topo::{Builder, EdgeId, Loop, Solid};
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

/// Blend as many of `blends` as this blender can actually close, instead of
/// refusing the lot.
///
/// [`blend_edges`] is all-or-nothing: one corner it cannot resolve loses the
/// fillet on every edge in the call, and the model layer's only recourse is to
/// give up on that region entirely. There are real configurations — a divider
/// crossing a compartment, a runout onto a non-planar face — where that costs a
/// part its whole floor fillet over a couple of corners. A partial fillet with
/// some sharp corners left in is worse-looking than a complete one but far
/// better than none, so this degrades instead of failing.
///
/// Two tiers, both driven by *trying* the blender rather than predicting it:
///
/// 1. **Per chain.** Edges are grouped into connected chains (a compartment
///    boundary, an island). Chains are added one at a time and kept only while
///    the result still blends, so one bad compartment cannot cost the others.
/// 2. **Within a chain.** A chain that fails whole is bisected, depth-limited,
///    and each half retried. A partial run simply terminates in runouts, which
///    the blender already supports, so the salvaged part is ordinary geometry.
///
/// Returns the blended solid and the edges it had to leave sharp.
///
/// **Errs only when the input is at fault** — a blended edge that is missing or
/// not shared by exactly two faces means `solid` was already non-manifold,
/// which no amount of dropping blends can fix. Degrading there would swap a
/// loud error for a silently unsound part, so that case still propagates
/// exactly as [`blend_edges`] reports it. Everything the blender merely cannot
/// *close* degrades instead.
///
/// Cost is one `blend_edges` attempt per chain when things go well, and up to
/// `2^MAX_SPLIT` more per failing chain. Nothing is attempted at all when the
/// whole set succeeds first time, which is the common path.
pub fn blend_best_effort(
    solid: &Solid,
    blends: &[(EdgeId, f32)],
) -> Result<(Solid, Vec<EdgeId>), String> {
    /// Bisection depth. 3 → a chain is probed at worst in eighths; beyond that
    /// the salvaged runs are too short to be worth the rebuilds.
    const MAX_SPLIT: u32 = 3;

    if blends.is_empty() {
        return Ok((solid.clone(), Vec::new()));
    }
    // The input-side preconditions, checked up front so an unsound solid is
    // reported rather than quietly blended around.
    let edge_faces = solid.edge_faces();
    for &(e, _) in blends {
        if e >= solid.edges.len() {
            return Err(format!("blend: edge {e} out of range"));
        }
        if edge_faces[e].len() != 2 {
            return Err(format!("blend: edge {e} has {} faces (want 2)", edge_faces[e].len()));
        }
    }
    if let Ok(s) = blend_edges(solid, blends) {
        return Ok((s, Vec::new()));
    }

    let mut kept: Vec<(EdgeId, f32)> = Vec::new();
    for chain in chains(solid, blends) {
        let salvaged = salvage(solid, &kept, &chain, MAX_SPLIT);
        kept.extend(salvaged);
    }

    let dropped: Vec<EdgeId> = blends
        .iter()
        .map(|&(e, _)| e)
        .filter(|e| !kept.iter().any(|&(k, _)| k == *e))
        .collect();

    match blend_edges(solid, &kept) {
        Ok(s) => Ok((s, dropped)),
        // Only reachable if a set that blended during probing stops doing so,
        // which would be a blender inconsistency; fall back to no fillet at all
        // rather than fail a build over a fillet.
        Err(_) => Ok((solid.clone(), blends.iter().map(|&(e, _)| e).collect())),
    }
}

/// The longest prefix-closed subset of `run` that still blends on top of
/// `base`, found by bisection. Input order is preserved so a half is a
/// *contiguous* run of the chain, whose ends become runouts.
fn salvage(
    solid: &Solid,
    base: &[(EdgeId, f32)],
    run: &[(EdgeId, f32)],
    depth: u32,
) -> Vec<(EdgeId, f32)> {
    if run.is_empty() {
        return Vec::new();
    }
    let mut trial = base.to_vec();
    trial.extend_from_slice(run);
    if blend_edges(solid, &trial).is_ok() {
        return run.to_vec();
    }
    if depth == 0 || run.len() < 2 {
        return Vec::new();
    }
    let mid = run.len() / 2;
    let head = salvage(solid, base, &run[..mid], depth - 1);
    let mut base2 = base.to_vec();
    base2.extend_from_slice(&head);
    let tail = salvage(solid, &base2, &run[mid..], depth - 1);
    let mut out = head;
    out.extend(tail);
    out
}

/// Group blended edges into connected chains by shared vertices, preserving
/// input order within each chain (the model emits a loop's edges in traversal
/// order, and [`salvage`]'s bisection relies on that to cut contiguous runs).
fn chains(solid: &Solid, blends: &[(EdgeId, f32)]) -> Vec<Vec<(EdgeId, f32)>> {
    let mut parent: Vec<usize> = (0..blends.len()).collect();
    fn find(parent: &mut Vec<usize>, i: usize) -> usize {
        if parent[i] != i {
            let r = find(parent, parent[i]);
            parent[i] = r;
        }
        parent[i]
    }
    let mut by_vertex: HashMap<usize, usize> = HashMap::new();
    for (i, &(e, _)) in blends.iter().enumerate() {
        if e >= solid.edges.len() {
            continue;
        }
        let ed = solid.edges[e];
        for v in [ed.v0, ed.v1] {
            match by_vertex.get(&v) {
                Some(&j) => {
                    let (a, b) = (find(&mut parent, i), find(&mut parent, j));
                    parent[a] = b;
                }
                None => {
                    by_vertex.insert(v, i);
                }
            }
        }
    }
    let mut groups: Vec<(usize, Vec<(EdgeId, f32)>)> = Vec::new();
    for (i, &b) in blends.iter().enumerate() {
        let root = find(&mut parent, i);
        match groups.iter_mut().find(|(r, _)| *r == root) {
            Some((_, v)) => v.push(b),
            None => groups.push((root, vec![b])),
        }
    }
    groups.into_iter().map(|(_, v)| v).collect()
}

/// Blend a set of edges of `solid` by the given radii.
pub fn blend_edges(solid: &Solid, blends: &[(EdgeId, f32)]) -> Result<Solid, String> {
    let _perf = crate::kernel::perf::scope(crate::kernel::perf::Metric::BlendEdges);
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

    // Two blended edges continue a chain; one is a runout (the chain
    // terminates against a third face). Three or more needs a spherical
    // corner patch, still unsupported.
    let mut vertex_blends: HashMap<usize, Vec<EdgeId>> = HashMap::new();
    for &e in want.keys() {
        let ed = solid.edges[e];
        vertex_blends.entry(ed.v0).or_default().push(e);
        vertex_blends.entry(ed.v1).or_default().push(e);
    }
    let mut terminating: HashMap<usize, EdgeId> = HashMap::new();
    for (v, es) in &vertex_blends {
        match es.len() {
            2 => {}
            1 => {
                terminating.insert(*v, es[0]);
            }
            n => {
                return Err(format!(
                    "blend: vertex {v} has {n} blended edges (want 1 or 2; \
                     spherical corners unsupported)"
                ));
            }
        }
    }

    let face_outward = |fid: usize, p: Vec3| -> Vec3 {
        let f = &solid.faces[fid];
        let n = f.surface.normal(f.surface.project(p));
        if f.sense { n } else { -n }
    };

    let mut bm: HashMap<EdgeId, Blend> = HashMap::new();
    let mut runouts: HashMap<usize, Runout> = HashMap::new();
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
        let fwd_a = loop_edge_dir(solid, fa, e);

        let mut blend = if plane_a.is_some() && plane_b.is_some() {
            build_cyl_blend(ed, cv0, cv1, ma, na0, ta_p0, ta_p1, tb_p0, tb_p1, r, fwd_a)?
        } else if cyl.is_some() && is_circle && (plane_a.is_some() || plane_b.is_some()) {
            // ta/tb stay per-face (ta on fa, tb on fb); the torus is symmetric.
            build_torus_blend(ed, cv0, cv1, na0, ta_p0, ta_p1, tb_p0, tb_p1, r, cyl.unwrap(), fwd_a)?
        } else {
            return Err(format!(
                "blend: edge {e} pair not supported (only plane/plane or plane/coaxial-cylinder)"
            ));
        };

        // Runout: where this chain terminates, trim the blend against the
        // face it dies into instead of closing it with a quarter arc.
        for (at_v0, v) in [(true, ed.v0), (false, ed.v1)] {
            if terminating.get(&v) != Some(&e) {
                continue;
            }
            if plane_a.is_none() || plane_b.is_none() {
                return Err(format!(
                    "blend: runout at vertex {v} is only supported for cylindrical blends"
                ));
            }
            let ft = find_runout_face(solid, v, fa, fb)?;
            let plane = as_plane(&solid.faces[ft].surface).ok_or_else(|| {
                format!("blend: runout face {ft} at vertex {v} is not planar")
            })?;
            let dir = match ed.curve {
                Curve::Line { dir, .. } => dir,
                _ => return Err(format!("blend: runout at vertex {v} needs a straight edge")),
            };
            let (cv, tap, tbp) =
                if at_v0 { (cv0, ta_p0, tb_p0) } else { (cv1, ta_p1, tb_p1) };
            let (ta_new, tb_new, arc) = runout_cyl(cv, dir, r, tap, tbp, plane)?;
            if at_v0 {
                blend.ta_p0 = ta_new;
                blend.tb_p0 = tb_new;
                blend.ca0 = arc;
            } else {
                blend.ta_p1 = ta_new;
                blend.tb_p1 = tb_new;
                blend.ca1 = arc;
            }
            runouts.insert(
                v,
                Runout { face: ft, arc, ta_p: ta_new, tb_p: tb_new, fa, fb },
            );
        }
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

    for fi in 0..solid.faces.len() {
        let outer = rebuild_loop(solid, &bm, &vinfo, &runouts, &want, fi, solid.outer_edges(fi), &mut b)?;
        let mut inners = Vec::with_capacity(solid.n_inners(fi));
        for lp in solid.inner_loops(fi) {
            inners.push(rebuild_loop(solid, &bm, &vinfo, &runouts, &want, fi, lp, &mut b)?);
        }
        let (surface, sense) = (solid.faces[fi].surface, solid.faces[fi].sense);
        b.face(surface, sense, outer, inners);
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

/// Where a blend chain dies into a third face, and the trim curve there.
#[derive(Clone, Copy)]
struct Runout {
    face: usize,
    arc: CurvEdge,
    /// Tangent points extended onto the runout face.
    ta_p: Vec3,
    tb_p: Vec3,
    fa: usize,
    fb: usize,
}

fn faces_at_vertex(solid: &Solid, v: usize) -> Vec<usize> {
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

fn coplanar(x: &Surface, y: &Surface) -> bool {
    match (as_plane(x), as_plane(y)) {
        (Some((o0, n0)), Some((o1, n1))) => {
            let (n0, n1) = (n0.normalize_or_zero(), n1.normalize_or_zero());
            n0.cross(n1).length() < 1e-5 && (o1 - o0).dot(n0).abs() < 1e-4
        }
        _ => false,
    }
}

/// The face a blend chain runs out onto at `v`: it touches `v`, is neither of
/// the blended pair, and is not coplanar with either — a coplanar neighbour
/// continues the same surface rather than terminating the blend.
fn find_runout_face(solid: &Solid, v: usize, fa: usize, fb: usize) -> Result<usize, String> {
    let mut cands = Vec::new();
    for fi in faces_at_vertex(solid, v) {
        if fi == fa
            || fi == fb
            || coplanar(&solid.faces[fi].surface, &solid.faces[fa].surface)
            || coplanar(&solid.faces[fi].surface, &solid.faces[fb].surface)
        {
            continue;
        }
        cands.push(fi);
    }
    match cands.len() {
        1 => Ok(cands[0]),
        0 => Err(format!("blend runout: no terminating face at vertex {v}")),
        n => Err(format!("blend runout: {n} candidate terminating faces at vertex {v}")),
    }
}

/// Trim one end of a cylindrical blend against the plane it runs out onto.
///
/// Sliding a cylinder point onto the plane along the axis is affine in
/// `(cos t, sin t)`, so the cut really is `p(t) = C + cos t·A + sin t·B` with
/// `A`/`B` conjugate semi-diameters — an exact ellipse, no approximation. The
/// frame is chosen with `e1` aimed at face a's tangent, so the arc starts
/// there at `t = 0` and sweeps to face b's tangent.
fn runout_cyl(
    cv: Vec3,
    axis: Vec3,
    r: f32,
    ta_p: Vec3,
    tb_p: Vec3,
    plane: (Vec3, Vec3),
) -> Result<(Vec3, Vec3, CurvEdge), String> {
    let (q, n) = plane;
    let n = n.normalize_or_zero();
    let d = axis.normalize_or_zero();
    let dn = d.dot(n);
    if dn.abs() < 1e-6 {
        return Err("blend runout: terminating face is parallel to the blend axis".into());
    }
    let onto = |p: Vec3| p + d * ((q - p).dot(n) / dn);
    let e1 = (ta_p - cv).normalize_or_zero();
    let e2 = d.cross(e1);
    let a_vec = d * (-r * e1.dot(n) / dn) + e1 * r;
    let b_vec = d * (-r * e2.dot(n) / dn) + e2 * r;
    let u = (tb_p - cv).normalize_or_zero();
    let t1 = u.dot(e2).atan2(u.dot(e1));
    let arc = CurvEdge {
        curve: Curve::Ellipse { center: onto(cv), a: a_vec, b: b_vec },
        t0: 0.0,
        t1,
    };
    Ok((onto(ta_p), onto(tb_p), arc))
}

#[allow(clippy::too_many_arguments)]
fn build_cyl_blend(
    ed: crate::kernel::topo::Edge,
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
    ed: crate::kernel::topo::Edge,
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

/// One rebuilt loop entry, with the endpoints it was actually emitted between
/// (needed to spot the gap a runout corner opens up).
struct Emitted {
    edge: (EdgeId, bool),
    start: Vec3,
    end_v: usize,
    end: Vec3,
}

#[allow(clippy::too_many_arguments)]
fn rebuild_loop(
    solid: &Solid,
    bm: &HashMap<EdgeId, Blend>,
    vinfo: &HashMap<usize, (Vec3, Vec3)>,
    runouts: &HashMap<usize, Runout>,
    want: &HashMap<EdgeId, f32>,
    fi: usize,
    lp: &[(EdgeId, bool)],
    b: &mut Builder,
) -> Result<Loop, String> {
    let face_surface = solid.faces[fi].surface;
    let ef = solid.edge_faces();
    // On the runout face the corner vertex splits in two: the edge it shares
    // with face a ends at the extended face-a tangent, the one it shares with
    // face b at the face-b tangent. `move_vertex` cannot choose between them
    // — both tangents lie exactly in this face — so decide by adjacency.
    let split_at = |v: usize, e: EdgeId| -> Option<Vec3> {
        let ro = runouts.get(&v)?;
        if ro.face != fi {
            return None;
        }
        if ef[e].contains(&ro.fa) {
            Some(ro.ta_p)
        } else if ef[e].contains(&ro.fb) {
            Some(ro.tb_p)
        } else {
            None
        }
    };

    let mut items: Vec<Emitted> = Vec::with_capacity(lp.len());
    for &(e, fwd) in lp {
        let ed = solid.edges[e];
        let end_v = if fwd { ed.v1 } else { ed.v0 };
        if want.contains_key(&e) {
            let bld = &bm[&e];
            let side_a = ef[e][0] == fi;
            let ce = if side_a { bld.ta } else { bld.tb };
            let (tp0, tp1) = if side_a { (bld.ta_p0, bld.ta_p1) } else { (bld.tb_p0, bld.tb_p1) };
            let (start, end) = if fwd { (tp0, tp1) } else { (tp1, tp0) };
            items.push(Emitted { edge: emit_curv(b, start, end, ce), start, end_v, end });
        } else {
            let pos0 = solid.verts[ed.v0].point;
            let pos1 = solid.verts[ed.v1].point;
            let new0 = split_at(ed.v0, e)
                .unwrap_or_else(|| move_vertex(vinfo, ed.v0, pos0, face_surface));
            let new1 = split_at(ed.v1, e)
                .unwrap_or_else(|| move_vertex(vinfo, ed.v1, pos1, face_surface));
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
            items.push(Emitted { edge: eid, start, end_v, end });
        }
    }

    // Splice the trim arc into the gap the split corner opened.
    let n = items.len();
    let mut out = Vec::with_capacity(n + 2);
    for i in 0..n {
        out.push(items[i].edge);
        let next_start = items[(i + 1) % n].start;
        if let Some(ro) = runouts.get(&items[i].end_v) {
            if ro.face == fi && (next_start - items[i].end).length() > 1e-6 {
                out.push(emit_curv(b, items[i].end, next_start, ro.arc));
            }
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
    // Walk the stored t0→t1 if it already maps start→end; else reverse.
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
    use crate::kernel::math::Vec3;

    fn approx(a: Vec3, b: Vec3) -> bool {
        (a - b).length() < 1e-4
    }

    /// A box whose top rim blends cleanly. `blend_best_effort` must be
    /// *transparent* on the happy path: same result as `blend_edges`, nothing
    /// reported dropped, and no extra probing.
    #[test]
    fn best_effort_matches_blend_edges_when_nothing_fails() {
        use crate::kernel::build::extrude;
        use crate::kernel::sketch::Sketch;

        let sk = Sketch::rounded_rect(0.0, 0.0, 20.0, 20.0, 4.0);
        let solid = extrude(&sk, 0.0, 5.0);
        // Every edge of the top cap's loop.
        let top: Vec<(EdgeId, f32)> = (0..solid.edges.len())
            .filter(|&e| {
                let ed = solid.edges[e];
                let (a, b) = (solid.vertex(ed.v0), solid.vertex(ed.v1));
                (a.z - 5.0).abs() < 1e-5 && (b.z - 5.0).abs() < 1e-5
            })
            .map(|e| (e, 1.0))
            .collect();
        assert!(!top.is_empty(), "expected a top rim to blend");

        let direct = blend_edges(&solid, &top).expect("rim blends");
        let (best, dropped) = blend_best_effort(&solid, &top).expect("sound input");
        assert!(dropped.is_empty(), "nothing should be dropped, got {dropped:?}");
        assert_eq!(best.faces.len(), direct.faces.len());
        best.validate().expect("best-effort result is manifold");
    }

    /// Degrading must never hide an unsound *input*. An edge that is not shared
    /// by exactly two faces means the solid was already non-manifold, and no
    /// choice of blend subset fixes that — so it still errs rather than
    /// returning a quietly broken part.
    #[test]
    fn best_effort_still_reports_a_non_manifold_input() {
        use crate::kernel::build::extrude;
        use crate::kernel::sketch::Sketch;

        let solid = extrude(&Sketch::rectangle(0.0, 0.0, 10.0, 10.0), 0.0, 5.0);
        let err = blend_best_effort(&solid, &[(solid.edges.len() + 7, 1.0)])
            .expect_err("out-of-range edge must be reported");
        assert!(err.contains("out of range"), "unexpected error: {err}");
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

