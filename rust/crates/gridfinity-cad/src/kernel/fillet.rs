
use crate::kernel::geom::{Curve, Surface};
use crate::kernel::math::Vec3;
use crate::kernel::topo::{Builder, EdgeId, Loop, Solid};
use std::collections::HashMap;

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
            return Err(format!("blend: edge {e} has {} faces (want 2)", edge_faces[e].len()));
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
            return Err(format!("blend: edge {e} has {} faces (want 2)", edge_faces[e].len()));
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
        let mid = (p0 + p1) * 0.5;
        let r = want[&e];
        let na_mid = face_outward(fa, mid);
        let nb_mid = face_outward(fb, mid);
        let sin_mid = na_mid.cross(nb_mid).length();
        if sin_mid < 1e-6 || r <= 0.0 {
            return Err(format!("blend: edge {e} degenerate (parallel faces or r≤0)"));
        }
        let centroid_a = face_centroid(solid, fa);
        let to_centroid = centroid_a - mid;
        let ta_plus = mid + r * (na_mid + nb_mid) / sin_mid - r * na_mid;
        let ta_minus = mid - r * (na_mid + nb_mid) / sin_mid + r * na_mid;
        let s = if to_centroid.dot(ta_minus - mid) > to_centroid.dot(ta_plus - mid) {
            -1.0
        } else {
            1.0
        };
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

        let fwd_a = loop_edge_dir(solid, fa, e);

        let mut blend = if plane_a.is_some() && plane_b.is_some() {
            build_cyl_blend(ed, cv0, cv1, ma, na0, ta_p0, ta_p1, tb_p0, tb_p1, r, fwd_a)?
        } else if cyl.is_some() && is_circle && (plane_a.is_some() || plane_b.is_some()) {
            build_torus_blend(ed, cv0, cv1, na0, ta_p0, ta_p1, tb_p0, tb_p1, r, cyl.unwrap(), fwd_a)?
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

    let mut vinfo: HashMap<usize, (Vec3, Vec3)> = HashMap::with_capacity(bm.len() * 2);
    for (e, bld) in &bm {
        let ed = solid.edges[*e];
        vinfo.insert(ed.v0, (bld.ta_p0, bld.tb_p0));
        vinfo.insert(ed.v1, (bld.ta_p1, bld.tb_p1));
    }

    let nb = bm.len();
    let mut b = Builder::with_capacity(
        solid.verts.len() + 4 * nb,
        solid.edges.len() + 4 * nb,
        solid.faces.len() + nb,
        solid.loop_edges_len() + 4 * nb,
        solid.faces.len() + nb,
    );

    let mut loop_scratch: Vec<(EdgeId, bool)> = Vec::new();
    let mut items_scratch: Vec<Emitted> = Vec::new();
    let mut inner_ranges: Vec<usize> = Vec::new();

    for fi in 0..solid.faces.len() {
        loop_scratch.clear();
        inner_ranges.clear();
        rebuild_loop(solid, &bm, &vinfo, &runouts, &want, fi, solid.outer_edges(fi), edge_faces, &mut b, &mut items_scratch, &mut loop_scratch)?;
        let outer_len = loop_scratch.len();
        for lp in solid.inner_loops(fi) {
            let before = loop_scratch.len();
            rebuild_loop(solid, &bm, &vinfo, &runouts, &want, fi, lp, edge_faces, &mut b, &mut items_scratch, &mut loop_scratch)?;
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

    let mut blend_keys: Vec<EdgeId> = bm.keys().copied().collect();
    blend_keys.sort_unstable();
    for k in blend_keys {
        let bld = &bm[&k];
        let e_ta = emit_curv(&mut b, bld.ta_p0, bld.ta_p1, bld.ta);
        let e_tb = emit_curv(&mut b, bld.tb_p0, bld.tb_p1, bld.tb);
        let e_ca0 = emit_curv(&mut b, bld.ta_p0, bld.tb_p0, bld.ca0);
        let e_ca1 = emit_curv(&mut b, bld.ta_p1, bld.tb_p1, bld.ca1);
        let lp = if bld.fwd_a {
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

#[derive(Clone, Copy)]
struct Runout {
    face: usize,
    arc: CurvEdge,
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
) -> Result<Fillet, String> {
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
    let sense = surface.normal(surface.project(ta_p0)).dot(na0) > 0.0;

    Ok(Fillet { ta, tb, ca0, ca1, ta_p0, ta_p1, tb_p0, tb_p1, surface, sense, fwd_a })
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
        Curve::Circle { center, axis, radius, ref_dir } => (center, axis, radius, ref_dir),
        _ => return Err("torus blend: edge not a circle".into()),
    };
    let (a0, a1) = (ed.t0, ed.t1);
    let cv0_on = cyl_base + cyl_axis * (cv0 - cyl_base).dot(cyl_axis);
    let major = (cv0 - cv0_on).length();
    let torus_center = cv0_on;
    let torus_axis = edge_axis;
    let ref_dir = edge_ref_dir;
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

    Ok(Fillet { ta, tb, ca0, ca1, ta_p0, ta_p1, tb_p0, tb_p1, surface, sense, fwd_a })
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

    items.clear();
    items.reserve(lp.len());
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

    let n = items.len();
    out.reserve(n + 2);
    for i in 0..n {
        out.push(items[i].edge);
        let next_start = items[(i + 1) % n].start;
        if let Some(ro) = runouts.get(&items[i].end_v) {
            if ro.face == fi && (next_start - items[i].end).length() > 1e-6 {
                out.push(emit_curv(b, items[i].end, next_start, ro.arc));
            }
        }
    }
    Ok(())
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
        assert!(dropped.is_empty(), "nothing should be dropped, got {dropped:?}");
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
}
