use crate::kernel::geom::{Curve, Surface};
use crate::kernel::math::Vec3;
use crate::kernel::topo::{Builder, EdgeId, Solid};
use std::collections::HashMap;

/// A face whose analytic normal evaluates to NaN at one of a blended edge's
/// endpoints. Pre-existing and undiagnosed; `fuzz_stripped_polyominoes` is what
/// reaches it. See `rust/CLAUDE.md`.
pub const NON_FINITE_NORMAL: &str = "blend: face normal is not finite";

#[derive(Clone, Copy)]
struct CurvEdge {
    curve: Curve,
    t0: f32,
    t1: f32,
}

#[derive(Clone)]
struct Fillet {
    ta: CurvEdge,
    tb: CurvEdge,
    ca0: CurvEdge,
    ca1: CurvEdge,
    ta_p0: Vec3,
    ta_p1: Vec3,
    tb_p0: Vec3,
    tb_p1: Vec3,
    surface: Surface,
    sense: bool,
    fwd_a: bool,
}

pub fn fillet_best_effort(
    solid: &Solid,
    blends: &[(EdgeId, f32)],
) -> Result<(Solid, Vec<EdgeId>), String> {
    const MAX_SPLIT: u32 = 3;

    if blends.is_empty() {
        return Ok((solid.clone(), Vec::new()));
    }
    let edge_faces = solid.edge_faces();
    for &(e, _) in blends {
        if e >= solid.edges.len() {
            return Err(format!("blend: edge {e} out of range"));
        }
        if edge_faces[e].len() != 2 {
            return Err(format!(
                "blend: edge {e} has {} faces (want 2)",
                edge_faces[e].len()
            ));
        }
    }
    if let Ok(s) = fillet_edges_with(solid, blends, &edge_faces) {
        return Ok((s, Vec::new()));
    }

    let mut kept: Vec<(EdgeId, f32)> = Vec::new();
    for chain in chains(solid, blends) {
        let salvaged = salvage(solid, &edge_faces, &kept, &chain, MAX_SPLIT);
        kept.extend(salvaged);
    }

    let dropped: Vec<EdgeId> = blends
        .iter()
        .map(|&(e, _)| e)
        .filter(|e| !kept.iter().any(|&(k, _)| k == *e))
        .collect();

    match fillet_edges_with(solid, &kept, &edge_faces) {
        Ok(s) => Ok((s, dropped)),
        Err(_) => Ok((solid.clone(), blends.iter().map(|&(e, _)| e).collect())),
    }
}

fn salvage(
    solid: &Solid,
    ef: &crate::kernel::topo::EdgeFaces,
    base: &[(EdgeId, f32)],
    run: &[(EdgeId, f32)],
    depth: u32,
) -> Vec<(EdgeId, f32)> {
    if run.is_empty() {
        return Vec::new();
    }
    let mut trial = base.to_vec();
    trial.extend_from_slice(run);
    if fillet_edges_with(solid, &trial, ef).is_ok() {
        return run.to_vec();
    }
    if depth == 0 || run.len() < 2 {
        return Vec::new();
    }
    let mid = run.len() / 2;
    let head = salvage(solid, ef, base, &run[..mid], depth - 1);
    let mut base2 = base.to_vec();
    base2.extend_from_slice(&head);
    let tail = salvage(solid, ef, &base2, &run[mid..], depth - 1);
    let mut out = head;
    out.extend(tail);
    out
}

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

pub fn fillet_edges(solid: &Solid, blends: &[(EdgeId, f32)]) -> Result<Solid, String> {
    let edge_faces = solid.edge_faces();
    fillet_edges_with(solid, blends, &edge_faces)
}

fn fillet_edges_with(
    solid: &Solid,
    blends: &[(EdgeId, f32)],
    edge_faces: &crate::kernel::topo::EdgeFaces,
) -> Result<Solid, String> {
    let _perf = crate::kernel::perf::scope(crate::kernel::perf::Metric::FilletEdges);
    let mut want: HashMap<EdgeId, f32> = HashMap::with_capacity(blends.len());
    want.extend(blends.iter().copied());

    for &e in want.keys() {
        if e >= solid.edges.len() {
            return Err(format!("blend: edge {e} out of range"));
        }
        if edge_faces[e].len() != 2 {
            return Err(format!(
                "blend: edge {e} has {} faces (want 2)",
                edge_faces[e].len()
            ));
        }
    }

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

    let mut bm: HashMap<EdgeId, Fillet> = HashMap::with_capacity(want.len());
    let mut runouts: HashMap<usize, Runout> = HashMap::new();
    let mut want_sorted: Vec<EdgeId> = want.keys().copied().collect();
    want_sorted.sort_unstable();
    for &e in &want_sorted {
        let ed = solid.edges[e];
        let (fa, fb) = (edge_faces[e][0], edge_faces[e][1]);
        let (p0, p1) = (solid.verts[ed.v0].point, solid.verts[ed.v1].point);
        // The *curve's* midpoint, not the chord's. They agree on a line, and on
        // a semicircle the chord midpoint is the circle's centre -- nowhere near
        // either surface, so every normal taken there is meaningless.
        let t_mid = (ed.t0 + ed.t1) * 0.5;
        let mid = ed.curve.point(t_mid);
        let r = want[&e];
        let na_mid = face_outward(fa, mid);
        let nb_mid = face_outward(fb, mid);
        let sin_mid = na_mid.cross(nb_mid).length();
        if sin_mid < 1e-6 || r <= 0.0 {
            return Err(format!(
                "blend: edge {e} degenerate (parallel faces or r≤0)"
            ));
        }
        // Which side of the edge the blend sits on is decided *locally*, from
        // the direction fa's material lies in at this edge. The orientation
        // invariant says a loop keeps its face's material on the left, so that
        // direction is `outward normal x edge tangent`. Reading it off
        // `face_centroid` instead only works for a face whose centroid is
        // inside it: an L-shaped cavity floor, and any floor with an opening's
        // mouth in it, pull the centroid far enough to flip the choice on one
        // edge of a chain, which lands that edge's tangent points 2r away from
        // its neighbour's and tears the loop open.
        let fwd_a = loop_edge_dir(solid, fa, e);
        // `tangent` differentiates in increasing t, and an edge whose stored
        // range runs backwards traverses v0 -> v1 the other way.
        let rising = if ed.t1 >= ed.t0 { 1.0 } else { -1.0 };
        let tan_mid = (ed.curve.tangent(t_mid) * rising).normalize_or_zero();
        let along_a = if fwd_a { tan_mid } else { -tan_mid };
        let into_a = na_mid.cross(along_a);
        let ta_plus = mid + r * (na_mid + nb_mid) / sin_mid - r * na_mid;
        let ta_minus = mid - r * (na_mid + nb_mid) / sin_mid + r * na_mid;
        let s = if into_a.dot(ta_minus - mid) > into_a.dot(ta_plus - mid) {
            -1.0
        } else {
            1.0
        };
        let (na0, nb0) = (face_outward(fa, p0), face_outward(fb, p0));
        let (na1, nb1) = (face_outward(fa, p1), face_outward(fb, p1));
        // A non-finite normal poisons everything downstream -- `sin.max(1e-9)`
        // keeps a NaN cross product from being caught, the ball centre comes out
        // NaN, and the failure only surfaces much later as a non-finite vertex
        // in the builder. Name the surface that produced it instead.
        for (which, fid, n) in [(0, fa, na0), (0, fb, nb0), (1, fa, na1), (1, fb, nb1)] {
            assert!(
                n.is_finite(),
                "{NON_FINITE_NORMAL}: face {fid} at edge {e}'s v{which} ({n:?}), surface {:?}",
                solid.faces[fid].surface
            );
        }
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

        let mut blend = if plane_a.is_some() && plane_b.is_some() {
            build_cyl_blend(ed, cv0, cv1, ma, na0, ta_p0, ta_p1, tb_p0, tb_p1, r, fwd_a)?
        } else if cyl.is_some() && is_circle && (plane_a.is_some() || plane_b.is_some()) {
            build_torus_blend(
                ed,
                cv0,
                cv1,
                na0,
                ta_p0,
                ta_p1,
                tb_p0,
                tb_p1,
                r,
                cyl.expect("this arm is guarded by cyl.is_some()"),
                fwd_a,
            )?
        } else {
            return Err(format!(
                "blend: edge {e} pair not supported (only plane/plane or plane/coaxial-cylinder)"
            ));
        };

        for (at_v0, v) in [(true, ed.v0), (false, ed.v1)] {
            if terminating.get(&v) != Some(&e) {
                continue;
            }
            if plane_a.is_none() || plane_b.is_none() {
                return Err(format!(
                    "blend: runout at vertex {v} is only supported for cylindrical blends"
                ));
            }
            let dir = match ed.curve {
                Curve::Line { dir, .. } => dir,
                _ => return Err(format!("blend: runout at vertex {v} needs a straight edge")),
            };
            let (cv, tap, tbp) = if at_v0 {
                (cv0, ta_p0, tb_p0)
            } else {
                (cv1, ta_p1, tb_p1)
            };
            let land =
                |plane| runout_cyl(cv, dir, r, tap, tbp, plane).map(|(a, b, _)| (a + b) * 0.5);
            let end = plan_runout_end(solid, v, e, fa, fb, edge_faces, land)?;
            let ft = match end {
                RunoutEnd::Absorb { face } => face,
                RunoutEnd::Cap { fa_side, .. } => fa_side,
            };
            let plane = as_plane(&solid.faces[ft].surface)
                .ok_or_else(|| format!("blend: runout face {ft} at vertex {v} is not planar"))?;
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
                Runout {
                    end,
                    corner: solid.verts[v].point,
                    arc,
                    ta_p: ta_new,
                    tb_p: tb_new,
                    fa,
                    fb,
                },
            );
        }
        bm.insert(e, blend);
    }

    let mut vinfo: HashMap<usize, (Vec3, Vec3)> = HashMap::with_capacity(bm.len() * 2);
    let mut vinfo_order: Vec<EdgeId> = bm.keys().copied().collect();
    vinfo_order.sort_unstable();
    for e in vinfo_order {
        let bld = &bm[&e];
        let ed = solid.edges[e];
        vinfo.insert(ed.v0, (bld.ta_p0, bld.tb_p0));
        vinfo.insert(ed.v1, (bld.ta_p1, bld.tb_p1));
    }

    let mut edge_moved = vec![false; solid.edges.len()];
    for (e, moved) in edge_moved.iter_mut().enumerate() {
        let ed = solid.edges[e];
        *moved = want.contains_key(&e) || vinfo.contains_key(&ed.v0) || vinfo.contains_key(&ed.v1);
    }
    let mut touched = vec![false; solid.faces.len()];
    for (fi, t) in touched.iter_mut().enumerate() {
        *t = solid
            .face_loops(fi)
            .any(|lp| lp.iter().any(|&(e, _)| edge_moved[e]));
    }

    let mut b = Builder::resume(solid, &touched);

    let mut loop_scratch: Vec<(EdgeId, bool)> = Vec::new();
    let mut items_scratch: Vec<Emitted> = Vec::new();
    let mut inner_ranges: Vec<usize> = Vec::new();

    for (fi, &is_touched) in touched.iter().enumerate() {
        if !is_touched {
            b.copy_face(solid, fi);
            continue;
        }
        loop_scratch.clear();
        inner_ranges.clear();
        rebuild_loop(
            solid,
            &bm,
            &vinfo,
            &runouts,
            &want,
            fi,
            solid.outer_edges(fi),
            edge_faces,
            &mut b,
            &mut items_scratch,
            &mut loop_scratch,
        )?;
        let outer_len = loop_scratch.len();
        for lp in solid.inner_loops(fi) {
            let before = loop_scratch.len();
            rebuild_loop(
                solid,
                &bm,
                &vinfo,
                &runouts,
                &want,
                fi,
                lp,
                edge_faces,
                &mut b,
                &mut items_scratch,
                &mut loop_scratch,
            )?;
            inner_ranges.push(loop_scratch.len() - before);
        }
        let outer = &loop_scratch[..outer_len];
        let mut inners: Vec<&[(EdgeId, bool)]> = Vec::with_capacity(inner_ranges.len());
        let mut off = outer_len;
        for &len in &inner_ranges {
            inners.push(&loop_scratch[off..off + len]);
            off += len;
        }
        let (surface, sense) = (solid.faces[fi].surface, solid.faces[fi].sense);
        b.face_from(surface, sense, outer, &inners);
    }

    let mut blend_faces: Vec<usize> = Vec::with_capacity(bm.len());
    let mut blend_keys: Vec<EdgeId> = bm.keys().copied().collect();
    blend_keys.sort_unstable();
    // How the blend face traversed each capped end's trim curve. The cap shares
    // that edge and has to run the other way, which is what fixes its winding
    // against all three of its neighbours at once.
    let mut arc_used: HashMap<usize, (EdgeId, bool)> = HashMap::new();
    for k in blend_keys {
        let bld = &bm[&k];
        let e_ta = emit_curv(&mut b, bld.ta_p0, bld.ta_p1, bld.ta);
        let e_tb = emit_curv(&mut b, bld.tb_p0, bld.tb_p1, bld.tb);
        let e_ca0 = emit_curv(&mut b, bld.ta_p0, bld.tb_p0, bld.ca0);
        let e_ca1 = emit_curv(&mut b, bld.ta_p1, bld.tb_p1, bld.ca1);
        let lp: [(EdgeId, bool); 4] = if bld.fwd_a {
            [(e_ta.0, !e_ta.1), e_ca0, e_tb, (e_ca1.0, !e_ca1.1)]
        } else {
            [e_ta, e_ca1, (e_tb.0, !e_tb.1), (e_ca0.0, !e_ca0.1)]
        };
        let ed = solid.edges[k];
        let (used0, used1) = if bld.fwd_a {
            (e_ca0, (e_ca1.0, !e_ca1.1))
        } else {
            ((e_ca0.0, !e_ca0.1), e_ca1)
        };
        arc_used.insert(ed.v0, used0);
        arc_used.insert(ed.v1, used1);
        blend_faces.push(b.face_from(bld.surface, bld.sense, &lp, &[]));
    }

    // Where the chain died on an opening's mouth there was no face to take the
    // trim curve, so the cap is emitted here. Every one of its three edges is
    // interned by its endpoints -- the ellipse is the edge the blend face just
    // built, and the two stubs are the lines the neighbours emitted when their
    // edge was split at the tangent point -- so the cap pairs up without being
    // told who its neighbours are.
    let mut cap_vs: Vec<usize> = runouts.keys().copied().collect();
    cap_vs.sort_unstable();
    for v in cap_vs {
        let ro = runouts[&v];
        let RunoutEnd::Cap { fa_side, .. } = ro.end else {
            continue;
        };
        let (va, vc, vb) = (b.vertex(ro.ta_p), b.vertex(ro.corner), b.vertex(ro.tb_p));
        let arc = emit_curv(&mut b, ro.tb_p, ro.ta_p, ro.arc);
        let mut lp = vec![b.line(va, vc), b.line(vc, vb), arc];
        if arc_used.get(&v) == Some(&arc) {
            lp.reverse();
            for x in &mut lp {
                x.1 = !x.1;
            }
        }
        let f = &solid.faces[fa_side];
        b.face_from(f.surface, f.sense, &lp, &[]);
    }

    let s = b.build_compact_unvalidated();
    if let Err(e) = s.validate() {
        return Err(format!("blend: rebuilt solid invalid: {e}"));
    }
    let _ = &blend_faces;
    for fid in 0..s.faces.len() {
        if crate::kernel::audit::face_loops_self_intersect(&s, fid) {
            return Err(format!("blend: face {fid}'s boundary crosses itself"));
        }
    }
    Ok(s)
}

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

fn as_plane(s: &Surface) -> Option<(Vec3, Vec3)> {
    if let Surface::Plane { origin, normal, .. } = s {
        Some((*origin, *normal))
    } else {
        None
    }
}

fn as_cyl(s: &Surface) -> Option<(Vec3, Vec3, f32)> {
    if let Surface::Cylinder {
        base, axis, radius, ..
    } = s
    {
        Some((*base, *axis, *radius))
    } else {
        None
    }
}

/// How a blend chain's end is closed off.
///
/// `Absorb` is the original case: one face beyond the chain owns the corner at
/// the vertex, so trimming its two edges back to the tangent points and
/// splicing the trim curve between them closes the gap.
///
/// `Cap` is what an opening's mouth needs. There the chain dies on a boundary
/// where the corner is *void* rather than material -- the wall stops and the
/// blend's quarter-round cross-section is left standing in mid-air -- so no
/// existing face can absorb the curve and one has to be emitted. The two
/// neighbours either side of the corner keep their vertex; each has its edge
/// split at the tangent point instead, and the piece from there to the corner
/// pairs with the new cap.
#[derive(Clone, Copy, PartialEq)]
enum RunoutEnd {
    Absorb {
        face: usize,
    },
    Cap {
        /// The neighbour across the fa-side edge, whose edge splits at `ta_p`.
        fa_side: usize,
        /// The neighbour across the fb-side edge, whose edge splits at `tb_p`.
        fb_side: usize,
    },
}

#[derive(Clone, Copy)]
struct Runout {
    end: RunoutEnd,
    /// The corner the chain dies on. `Cap` keeps it; `Absorb` trims it away.
    corner: Vec3,
    arc: CurvEdge,
    ta_p: Vec3,
    tb_p: Vec3,
    fa: usize,
    fb: usize,
}

impl Runout {
    /// The face whose loop the trim curve is spliced into, if any. A capped
    /// runout has none -- the curve bounds the cap instead.
    fn absorbing(&self) -> Option<usize> {
        match self.end {
            RunoutEnd::Absorb { face } => Some(face),
            RunoutEnd::Cap { .. } => None,
        }
    }

    /// Where face `fi`'s edge into the corner has to be split, for a capped
    /// runout. `None` when `fi` is not one of the two neighbours, or when the
    /// edge in hand is not the one that reaches the blend.
    fn cap_split(&self, fi: usize, edge_faces: &[usize]) -> Option<Vec3> {
        match self.end {
            RunoutEnd::Cap { fa_side, fb_side } => {
                if fi == fa_side && edge_faces.contains(&self.fa) {
                    Some(self.ta_p)
                } else if fi == fb_side && edge_faces.contains(&self.fb) {
                    Some(self.tb_p)
                } else {
                    None
                }
            }
            RunoutEnd::Absorb { .. } => None,
        }
    }

    /// Whether face `fi` keeps the corner vertex where it is. Only the two
    /// blended faces retreat to their tangent points; a capped runout leaves
    /// the corner standing for every other face that meets it, including the
    /// two the cap pairs with.
    fn keeps_corner(&self, fi: usize) -> bool {
        matches!(self.end, RunoutEnd::Cap { .. }) && fi != self.fa && fi != self.fb
    }
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

/// The face across `side`'s edge at `v` -- the neighbour the blend meets when
/// it runs off the end of `side` at that corner. `skip` is the blended edge
/// itself, which is not a way out.
fn across_at(
    solid: &Solid,
    v: usize,
    side: usize,
    skip: EdgeId,
    ef: &crate::kernel::topo::EdgeFaces,
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

/// Decide how the chain's end at `v` is closed. `land` reports where the runout
/// would come to rest on a given plane, which is what tells a face the blend
/// actually runs into from one that merely touches the vertex.
fn plan_runout_end(
    solid: &Solid,
    v: usize,
    e: EdgeId,
    fa: usize,
    fb: usize,
    ef: &crate::kernel::topo::EdgeFaces,
    land: impl Fn((Vec3, Vec3)) -> Result<Vec3, String>,
) -> Result<RunoutEnd, String> {
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
    if cands.len() == 1 {
        return Ok(RunoutEnd::Absorb { face: cands[0] });
    }
    if cands.is_empty() {
        return Err(format!("blend runout: no terminating face at vertex {v}"));
    }
    // Several candidates in one plane are not an ambiguity about *where* the
    // blend ends. The bin's outer wall is cut into bands by the peg profile and
    // three of them meet at the pinch a wall opening leaves, so the plane is
    // settled and only the owner of the trim curve is open. Whichever band the
    // runout actually lands in absorbs it; if it lands in none of them the
    // corner is void -- the mouth of the opening -- and the blend needs a cap.
    let plane = as_plane(&solid.faces[cands[0]].surface);
    let one_plane = cands[1..]
        .iter()
        .all(|&c| coplanar(&solid.faces[c].surface, &solid.faces[cands[0]].surface));
    if let (true, Some(plane)) = (one_plane, plane) {
        let at = land(plane)?;
        let inside: Vec<usize> = cands
            .iter()
            .copied()
            .filter(|&c| planar_face_contains(solid, c, at))
            .collect();
        if inside.len() == 1 {
            return Ok(RunoutEnd::Absorb { face: inside[0] });
        }
        if inside.is_empty() {
            let fa_side = across_at(solid, v, fa, e, ef);
            let fb_side = across_at(solid, v, fb, e, ef);
            let pick = |xs: &[usize]| -> Option<usize> {
                let hit: Vec<usize> = xs.iter().copied().filter(|c| cands.contains(c)).collect();
                (hit.len() == 1).then(|| hit[0])
            };
            if let (Some(a), Some(b)) = (pick(&fa_side), pick(&fb_side)) {
                if a != b {
                    return Ok(RunoutEnd::Cap {
                        fa_side: a,
                        fb_side: b,
                    });
                }
            }
        }
    }
    Err(format!(
        "blend runout: {} candidate terminating faces at vertex {v}",
        cands.len()
    ))
}

/// Whether `p` lies inside planar face `f`, by even-odd parity against every
/// loop projected into the surface's own uv. A curved edge contributes its
/// midpoint as well as its endpoints, so a mostly-curved loop does not collapse
/// onto its chords.
fn planar_face_contains(solid: &Solid, f: usize, p: Vec3) -> bool {
    let surf = &solid.faces[f].surface;
    let Some((origin, n)) = as_plane(surf) else {
        return false;
    };
    if (p - origin).dot(n.normalize_or_zero()).abs() > 1e-3 {
        return false;
    }
    let uv = surf.project(p);
    let mut crossings = 0u32;
    let mut poly: Vec<(f32, f32)> = Vec::new();
    for lp in solid.face_loops(f) {
        poly.clear();
        for &(e, fwd) in lp {
            let ed = solid.edges[e];
            let a = if fwd { ed.v0 } else { ed.v1 };
            poly.push(surf.project(solid.verts[a].point));
            if !matches!(ed.curve, Curve::Line { .. }) {
                poly.push(surf.project(ed.curve.point((ed.t0 + ed.t1) * 0.5)));
            }
        }
        let m = poly.len();
        for i in 0..m {
            let (q0, q1) = (poly[i], poly[(i + 1) % m]);
            if (q0.1 > uv.1) != (q1.1 > uv.1)
                && q0.0 + (uv.1 - q0.1) / (q1.1 - q0.1) * (q1.0 - q0.0) > uv.0
            {
                crossings += 1;
            }
        }
    }
    crossings % 2 == 1
}

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
        curve: Curve::Ellipse {
            center: onto(cv),
            a: a_vec,
            b: b_vec,
        },
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
) -> Result<Fillet, String> {
    let dir = match ed.curve {
        Curve::Line { dir, .. } => dir,
        _ => return Err("cyl blend: edge not a line".into()),
    };
    let ref_dir = (-ma).normalize_or(Vec3::X);

    let ta = CurvEdge {
        curve: Curve::Line { p0: ta_p0, dir },
        t0: 0.0,
        t1: (ta_p1 - ta_p0).length(),
    };
    let tb = CurvEdge {
        curve: Curve::Line { p0: tb_p0, dir },
        t0: 0.0,
        t1: (tb_p1 - tb_p0).length(),
    };

    let ca0 = connect_arc(cv0, dir, ta_p0, tb_p0)?;
    let ca1 = connect_arc(cv1, dir, ta_p1, tb_p1)?;

    let surface = Surface::Cylinder {
        base: cv0,
        axis: dir,
        radius: r,
        ref_dir,
    };
    let sense = surface.normal(surface.project(ta_p0)).dot(na0) > 0.0;

    Ok(Fillet {
        ta,
        tb,
        ca0,
        ca1,
        ta_p0,
        ta_p1,
        tb_p0,
        tb_p1,
        surface,
        sense,
        fwd_a,
    })
}

fn circle_span(
    center: Vec3,
    axis: Vec3,
    ref_dir: Vec3,
    p0: Vec3,
    p1: Vec3,
    src: (f32, f32),
) -> (f32, f32) {
    let (d0, d1) = crate::kernel::geom::radial_frame(axis, ref_dir);
    let angle = |p: Vec3| {
        let v = p - center;
        v.dot(d1).atan2(v.dot(d0))
    };
    let span = src.1 - src.0;
    let t0 = angle(p0);
    if span.abs() >= std::f32::consts::TAU - 1e-3 {
        return (t0, t0 + span);
    }
    let wrapped = |mut a: f32| {
        while a > std::f32::consts::PI {
            a -= std::f32::consts::TAU;
        }
        while a < -std::f32::consts::PI {
            a += std::f32::consts::TAU;
        }
        a
    };
    let want = angle(p1);
    let miss = |s: f32| wrapped(t0 + s - want).abs();
    let sweep = if miss(span.abs()) <= miss(-span.abs()) {
        span.abs()
    } else {
        -span.abs()
    };
    (t0, t0 + sweep)
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
) -> Result<Fillet, String> {
    let (cyl_base, cyl_axis, _cyl_radius) = cyl;
    let (edge_center, edge_axis, edge_radius, edge_ref_dir) = match ed.curve {
        Curve::Circle {
            center,
            axis,
            radius,
            ref_dir,
        } => (center, axis, radius, ref_dir),
        _ => return Err("torus blend: edge not a circle".into()),
    };
    let (a0, a1) = (ed.t0, ed.t1);
    let cv0_on = cyl_base + cyl_axis * (cv0 - cyl_base).dot(cyl_axis);
    let major = (cv0 - cv0_on).length();
    let torus_center = cv0_on;
    let torus_axis = edge_axis;
    let ref_dir = edge_ref_dir;
    assert!(
        major > 0.05,
        "blend torus degenerates to a ring: major {major} minor {r}"
    );
    let surface = Surface::Torus {
        center: torus_center,
        axis: torus_axis,
        major_r: major,
        minor_r: r,
        ref_dir,
    };
    let _ = (edge_center, edge_radius);

    let ta_center = torus_center + torus_axis * (ta_p0 - torus_center).dot(torus_axis);
    let ta_r = (ta_p0 - ta_center).length();
    let tb_center = torus_center + torus_axis * (tb_p0 - torus_center).dot(torus_axis);
    let tb_r = (tb_p0 - tb_center).length();
    let (ta_t0, ta_t1) = circle_span(ta_center, torus_axis, ref_dir, ta_p0, ta_p1, (a0, a1));
    let (tb_t0, tb_t1) = circle_span(tb_center, torus_axis, ref_dir, tb_p0, tb_p1, (a0, a1));
    let ta = CurvEdge {
        curve: Curve::Circle {
            center: ta_center,
            axis: torus_axis,
            radius: ta_r,
            ref_dir,
        },
        t0: ta_t0,
        t1: ta_t1,
    };
    let tb = CurvEdge {
        curve: Curve::Circle {
            center: tb_center,
            axis: torus_axis,
            radius: tb_r,
            ref_dir,
        },
        t0: tb_t0,
        t1: tb_t1,
    };

    let p0 = ed.curve.point(a0);
    let tan_at = |p: Vec3| {
        let v = p - torus_center;
        let perp = v - torus_axis * v.dot(torus_axis);
        torus_axis.cross(perp.normalize_or(Vec3::X))
    };
    let ca0 = connect_arc(cv0, tan_at(p0), ta_p0, tb_p0)?;
    let p1 = ed.curve.point(a1);
    let ca1 = connect_arc(cv1, tan_at(p1), ta_p1, tb_p1)?;

    let sense = surface.normal(surface.project(ta_p0)).dot(na0) > 0.0;

    Ok(Fillet {
        ta,
        tb,
        ca0,
        ca1,
        ta_p0,
        ta_p1,
        tb_p0,
        tb_p1,
        surface,
        sense,
        fwd_a,
    })
}

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

struct Emitted {
    edge: (EdgeId, bool),
    start: Vec3,
    end_v: usize,
    end: Vec3,
}

#[allow(clippy::too_many_arguments)]
fn rebuild_loop(
    solid: &Solid,
    bm: &HashMap<EdgeId, Fillet>,
    vinfo: &HashMap<usize, (Vec3, Vec3)>,
    runouts: &HashMap<usize, Runout>,
    want: &HashMap<EdgeId, f32>,
    fi: usize,
    lp: &[(EdgeId, bool)],
    ef: &crate::kernel::topo::EdgeFaces,
    b: &mut Builder,
    items: &mut Vec<Emitted>,
    out: &mut Vec<(EdgeId, bool)>,
) -> Result<(), String> {
    let face_surface = solid.faces[fi].surface;
    let split_at = |v: usize, e: EdgeId| -> Option<Vec3> {
        let ro = runouts.get(&v)?;
        if ro.absorbing() != Some(fi) {
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
    // A capped runout leaves the corner standing, so a face that only touches
    // it must not follow the blend back to a tangent point.
    let point_at = |v: usize, e: EdgeId, fallback: Vec3| -> Vec3 {
        match runouts.get(&v) {
            Some(ro) if ro.keeps_corner(fi) => fallback,
            _ => split_at(v, e).unwrap_or_else(|| move_vertex(vinfo, v, fallback, face_surface)),
        }
    };

    items.clear();
    items.reserve(lp.len());
    for &(e, fwd) in lp {
        let ed = solid.edges[e];
        let end_v = if fwd { ed.v1 } else { ed.v0 };
        if want.contains_key(&e) {
            let bld = &bm[&e];
            let side_a = ef[e][0] == fi;
            let ce = if side_a { bld.ta } else { bld.tb };
            let (tp0, tp1) = if side_a {
                (bld.ta_p0, bld.ta_p1)
            } else {
                (bld.tb_p0, bld.tb_p1)
            };
            let (start, end) = if fwd { (tp0, tp1) } else { (tp1, tp0) };
            items.push(Emitted {
                edge: emit_curv(b, start, end, ce),
                start,
                end_v,
                end,
            });
        } else {
            let pos0 = solid.verts[ed.v0].point;
            let pos1 = solid.verts[ed.v1].point;
            let new0 = point_at(ed.v0, e, pos0);
            let new1 = point_at(ed.v1, e, pos1);
            let (start, end) = if fwd { (new0, new1) } else { (new1, new0) };
            // The cap pairs with the stub between the tangent point and the
            // corner, so the edge running into the corner is cut in two there.
            let cut = [ed.v0, ed.v1]
                .into_iter()
                .find_map(|v| runouts.get(&v).and_then(|ro| ro.cap_split(fi, &ef[e])));
            if let Some(p) = cut {
                if !matches!(ed.curve, Curve::Line { .. }) {
                    return Err(format!(
                        "blend runout: capped end needs a straight edge into the corner (face {fi})"
                    ));
                }
                // The tangent point has to land *inside* the edge it splits. A
                // blend wider than the run leaves it beyond the far end, and
                // splitting there emits a stub that doubles back over the rest
                // of the edge -- a loop no triangulation can tile. Refusing
                // costs the chain a blend, which `fillet_best_effort` already
                // treats as a corner left sharp.
                let d = end - start;
                let t = (p - start).dot(d) / d.length_squared();
                if !(1e-4..=1.0 - 1e-4).contains(&t) {
                    return Err(format!(
                        "blend runout: capped end at {p:?} is not inside the edge it splits \
                         ({start:?} -> {end:?})"
                    ));
                }
                for (s, t) in [(start, p), (p, end)] {
                    let (vs, ve) = (b.vertex(s), b.vertex(t));
                    items.push(Emitted {
                        edge: b.line(vs, ve),
                        start: s,
                        end_v: end_v,
                        end: t,
                    });
                }
                continue;
            }
            let vs = b.vertex(start);
            let ve = b.vertex(end);
            let eid = match ed.curve {
                Curve::Line { .. } => b.line(vs, ve),
                Curve::Circle {
                    center,
                    axis,
                    radius,
                    ref_dir,
                } => {
                    let (a0, a1) = if fwd { (ed.t0, ed.t1) } else { (ed.t1, ed.t0) };
                    b.arc(vs, ve, center, axis, radius, ref_dir, a0, a1)
                }
                Curve::Ellipse {
                    center,
                    a: ea,
                    b: eb,
                } => {
                    let (t0, t1) = if fwd { (ed.t0, ed.t1) } else { (ed.t1, ed.t0) };
                    b.ellipse(vs, ve, center, ea, eb, t0, t1)
                }
                Curve::TorusSection { .. } => {
                    let (t0, t1) = if fwd { (ed.t0, ed.t1) } else { (ed.t1, ed.t0) };
                    b.torus_section(vs, ve, ed.curve, t0, t1)
                }
            };
            items.push(Emitted {
                edge: eid,
                start,
                end_v,
                end,
            });
        }
    }

    let n = items.len();
    out.reserve(n + 2);
    for i in 0..n {
        out.push(items[i].edge);
        let next_start = items[(i + 1) % n].start;
        if let Some(ro) = runouts.get(&items[i].end_v) {
            if ro.absorbing() == Some(fi) && (next_start - items[i].end).length() > 1e-6 {
                out.push(emit_curv(b, items[i].end, next_start, ro.arc));
            }
        }
    }
    Ok(())
}

fn move_vertex(
    vinfo: &HashMap<usize, (Vec3, Vec3)>,
    v: usize,
    fallback: Vec3,
    surface: Surface,
) -> Vec3 {
    if let Some((pa, pb)) = vinfo.get(&v) {
        if dist_to_surface(*pa, surface) < dist_to_surface(*pb, surface) {
            *pa
        } else {
            *pb
        }
    } else {
        fallback
    }
}

fn dist_to_surface(p: Vec3, s: Surface) -> f32 {
    match s {
        Surface::Plane { origin, normal, .. } => (p - origin).dot(normal).abs(),
        Surface::Cylinder {
            base, axis, radius, ..
        } => {
            let rel = p - base;
            (rel - axis * rel.dot(axis)).length() - radius
        }
        Surface::Sphere { center, radius, .. } => (p - center).length() - radius,
        _ => (s.point(s.project(p)) - p).length(),
    }
    .abs()
}

fn emit_curv(b: &mut Builder, start: Vec3, end: Vec3, ce: CurvEdge) -> (EdgeId, bool) {
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kernel::math::Vec3;

    fn approx(a: Vec3, b: Vec3) -> bool {
        (a - b).length() < 1e-4
    }

    #[test]
    fn best_effort_matches_fillet_edges_when_nothing_fails() {
        use crate::kernel::build::extrude;
        use crate::kernel::sketch::Sketch;

        let sk = Sketch::rounded_rect(0.0, 0.0, 20.0, 20.0, 4.0);
        let solid = extrude(&sk, 0.0, 5.0);
        let top: Vec<(EdgeId, f32)> = (0..solid.edges.len())
            .filter(|&e| {
                let ed = solid.edges[e];
                let (a, b) = (solid.vertex(ed.v0), solid.vertex(ed.v1));
                (a.z - 5.0).abs() < 1e-5 && (b.z - 5.0).abs() < 1e-5
            })
            .map(|e| (e, 1.0))
            .collect();
        assert!(!top.is_empty(), "expected a top rim to blend");

        let direct = fillet_edges(&solid, &top).expect("rim blends");
        let (best, dropped) = fillet_best_effort(&solid, &top).expect("sound input");
        assert!(
            dropped.is_empty(),
            "nothing should be dropped, got {dropped:?}"
        );
        assert_eq!(best.faces.len(), direct.faces.len());
        best.validate().expect("best-effort result is manifold");
    }

    #[test]
    fn best_effort_still_reports_a_non_manifold_input() {
        use crate::kernel::build::extrude;
        use crate::kernel::sketch::Sketch;

        let solid = extrude(&Sketch::rectangle(0.0, 0.0, 10.0, 10.0), 0.0, 5.0);
        let err = fillet_best_effort(&solid, &[(solid.edges.len() + 7, 1.0)])
            .expect_err("out-of-range edge must be reported");
        assert!(err.contains("out of range"), "unexpected error: {err}");
    }

    #[test]
    fn rolling_ball_corner_math() {
        let p = Vec3::new(10.0, 0.0, 0.0);
        let ma = Vec3::new(0.0, 0.0, 1.0);
        let mb = Vec3::new(-1.0, 0.0, 0.0);
        let r = 2.0_f32;
        let sin_theta = ma.cross(mb).length();
        let c = p + r * (ma + mb) / sin_theta;
        assert!(approx(c, Vec3::new(8.0, 0.0, 2.0)), "ball centre {c}");
        let ta = c - r * ma;
        let tb = c - r * mb;
        assert!(approx(ta, Vec3::new(8.0, 0.0, 0.0)), "floor tangent {ta}");
        assert!(approx(tb, Vec3::new(10.0, 0.0, 2.0)), "wall tangent {tb}");
    }

    #[test]
    fn connect_arc_endpoints() {
        let center = Vec3::new(8.0, 0.0, 2.0);
        let axis = Vec3::new(0.0, 1.0, 0.0);
        let from = Vec3::new(8.0, 0.0, 0.0);
        let to = Vec3::new(10.0, 0.0, 2.0);
        let ce = connect_arc(center, axis, from, to).unwrap();
        assert!(approx(ce.curve.point(ce.t0), from), "arc start");
        assert!(approx(ce.curve.point(ce.t1), to), "arc end");
        assert!(((ce.t1 - ce.t0).abs() - std::f32::consts::FRAC_PI_2).abs() < 1e-4);
    }

    #[test]
    fn circle_span_lands_on_its_own_endpoints_not_the_source_edges() {
        let center = Vec3::new(80.0, 4.0, 23.7);
        let axis = Vec3::Z;
        let ref_dir = Vec3::X;
        let p0 = Vec3::new(80.0, 5.45, 23.7);
        let p1 = Vec3::new(78.55, 4.0, 23.7);
        let src = (0.0, -std::f32::consts::FRAC_PI_2);
        let (t0, t1) = circle_span(center, axis, ref_dir, p0, p1, src);
        let c = Curve::Circle {
            center,
            axis,
            radius: 1.45,
            ref_dir,
        };
        assert!(approx(c.point(t0), p0), "span start {:?}", c.point(t0));
        assert!(approx(c.point(t1), p1), "span end {:?}", c.point(t1));
    }

    #[test]
    fn circle_span_keeps_a_full_turn_a_full_turn() {
        let center = Vec3::ZERO;
        let p = Vec3::new(3.0, 0.0, 0.0);
        let src = (0.0, std::f32::consts::TAU);
        let (t0, t1) = circle_span(center, Vec3::Z, Vec3::X, p, p, src);
        assert!(
            (t1 - t0 - std::f32::consts::TAU).abs() < 1e-4,
            "sweep {}",
            t1 - t0
        );
    }
}
