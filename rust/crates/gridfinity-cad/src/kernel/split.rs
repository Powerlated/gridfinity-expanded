use crate::kernel::geom::{Curve, Surface, radial_frame};
use crate::kernel::isect::{Intersection, intersect_surfaces};
use crate::kernel::math::Vec3;
use crate::kernel::topo::{Builder, EdgeId, Solid, VertexId};

pub const ON_PLANE: f32 = 1e-4;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Side {
    Negative,
    On,
    Positive,
}

pub fn side_of(plane: &Surface, p: Vec3) -> Side {
    let d = plane.signed_distance(p);
    if d > ON_PLANE {
        Side::Positive
    } else if d < -ON_PLANE {
        Side::Negative
    } else {
        Side::On
    }
}

pub fn curve_plane_params(curve: &Curve, t0: f32, t1: f32, plane: &Surface) -> Vec<f32> {
    let (lo, hi) = (t0.min(t1), t0.max(t1));
    let within = |t: f32| t > lo + ON_PLANE && t < hi - ON_PLANE;
    let Surface::Plane { origin, normal, .. } = *plane else {
        return Vec::new();
    };
    let c = origin.dot(normal);
    match *curve {
        Curve::Line { p0, dir } => {
            let denom = dir.dot(normal);
            if denom.abs() < ON_PLANE {
                return Vec::new();
            }
            let t = (c - p0.dot(normal)) / denom;
            if within(t) { vec![t] } else { Vec::new() }
        }
        Curve::Circle { center, axis, radius, ref_dir } => {
            let (d0, d1) = radial_frame(axis, ref_dir);
            harmonic_roots(
                radius * d0.dot(normal),
                radius * d1.dot(normal),
                c - center.dot(normal),
                lo,
                hi,
            )
            .into_iter()
            .filter(|&t| within(t))
            .collect()
        }
        Curve::Ellipse { center, a, b } => harmonic_roots(
            a.dot(normal),
            b.dot(normal),
            c - center.dot(normal),
            lo,
            hi,
        )
        .into_iter()
        .filter(|&t| within(t))
        .collect(),
        Curve::TorusSection { .. } => Vec::new(),
    }
}

fn harmonic_roots(a: f32, b: f32, rhs: f32, lo: f32, hi: f32) -> Vec<f32> {
    let amp = (a * a + b * b).sqrt();
    if amp < ON_PLANE {
        return Vec::new();
    }
    let ratio = rhs / amp;
    if ratio.abs() > 1.0 {
        return Vec::new();
    }
    let phase = b.atan2(a);
    let base = ratio.acos();
    let mut out = Vec::new();
    for root in [phase + base, phase - base] {
        let mut t = root;
        let two_pi = std::f32::consts::TAU;
        while t < lo {
            t += two_pi;
        }
        while t > hi {
            t -= two_pi;
        }
        if t >= lo {
            out.push(t);
        }
    }
    out.sort_by(f32::total_cmp);
    out.dedup_by(|x, y| (*x - *y).abs() < ON_PLANE);
    out
}

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
        Curve::Circle { center, axis, radius, ref_dir } => {
            b.arc(vs, ve, center, axis, radius, ref_dir, t0, t1)
        }
        Curve::Ellipse { center, a, b: eb } => b.ellipse(vs, ve, center, a, eb, t0, t1),
        Curve::TorusSection { .. } => b.torus_section(vs, ve, curve, t0, t1),
    }
}

pub fn param_of(curve: &Curve, p: Vec3) -> f32 {
    match *curve {
        Curve::Line { p0, dir } => (p - p0).dot(dir),
        Curve::Circle { center, axis, ref_dir, .. } => {
            let (d0, d1) = radial_frame(axis, ref_dir);
            let v = p - center;
            v.dot(d1).atan2(v.dot(d0))
        }
        Curve::Ellipse { center, a, b } => {
            let v = p - center;
            let (la, lb) = (a.length_squared().max(1e-12), b.length_squared().max(1e-12));
            (v.dot(b) / lb).atan2(v.dot(a) / la)
        }
        Curve::TorusSection { center, axis, major, minor, .. } => {
            let v = p - center;
            let along = v.dot(axis);
            let radial = (v - axis * along).length();
            (along / minor.max(1e-12)).atan2((radial - major) / minor.max(1e-12))
        }
    }
}

pub fn connector_curve(surface: &Surface, plane: &Surface, from: Vec3, to: Vec3) -> Option<Curve> {
    match intersect_surfaces(surface, plane) {
        Intersection::Curves(cs) => {
            let mid = (from + to) * 0.5;
            cs.into_iter()
                .filter(|c| (c.point(param_of(c, from)) - from).length() < 1e-2)
                .min_by(|a, b| {
                    let da = (a.point(param_of(a, mid)) - mid).length();
                    let db = (b.point(param_of(b, mid)) - mid).length();
                    da.total_cmp(&db)
                })
        }
        _ => None,
    }
}

struct Chain {
    edges: Vec<(EdgeId, bool)>,
    start: Vec3,
    end: Vec3,
}

fn winding_normal(surface: &Surface, sense: bool, p: Vec3) -> Vec3 {
    let n = surface.normal(surface.project(p));
    if sense { n } else { -n }
}

fn trim_loop(
    b: &mut Builder,
    solid: &Solid,
    lp: &[(EdgeId, bool)],
    plane: &Surface,
    discard: Side,
) -> Result<Option<Vec<Chain>>, String> {
    let mut pieces: Vec<(Vec3, Vec3, Curve, f32, f32, bool)> = Vec::new();
    for &(e, fwd) in lp {
        let ed = solid.edges[e];
        let (ta, tb) = if fwd { (ed.t0, ed.t1) } else { (ed.t1, ed.t0) };
        let mut cuts = curve_plane_params(&ed.curve, ed.t0, ed.t1, plane);
        cuts.sort_by(|x, y| {
            let dx = (x - ta).abs();
            let dy = (y - ta).abs();
            dx.total_cmp(&dy)
        });
        let mut bounds = vec![ta];
        bounds.extend(cuts);
        bounds.push(tb);
        let (vstart, vend) = solid.directed(e, fwd);
        let last = bounds.len() - 1;
        let point_at = |i: usize| -> Vec3 {
            if i == 0 {
                solid.vertex(vstart)
            } else if i == last {
                solid.vertex(vend)
            } else {
                ed.curve.point(bounds[i])
            }
        };
        for i in 0..last {
            let (u0, u1) = (bounds[i], bounds[i + 1]);
            if (u1 - u0).abs() < ON_PLANE {
                continue;
            }
            let mid = ed.curve.point((u0 + u1) * 0.5);
            let keep = side_of(plane, mid) != discard;
            pieces.push((point_at(i), point_at(i + 1), ed.curve, u0, u1, keep));
        }
    }
    if pieces.iter().all(|p| p.5) {
        return Ok(None);
    }
    if pieces.iter().all(|p| !p.5) {
        return Ok(Some(Vec::new()));
    }
    let n = pieces.len();
    let first_kept = (0..n)
        .find(|&i| pieces[i].5 && !pieces[(i + n - 1) % n].5)
        .expect("a run of kept pieces starts somewhere");
    let mut chains: Vec<Chain> = Vec::new();
    let mut current: Option<Chain> = None;
    for k in 0..n {
        let piece = &pieces[(first_kept + k) % n];
        if !piece.5 {
            if let Some(c) = current.take() {
                chains.push(c);
            }
            continue;
        }
        let vs = b.vertex(piece.0);
        let ve = b.vertex(piece.1);
        if vs == ve {
            continue;
        }
        let edge = emit_edge(b, vs, ve, piece.2, piece.3, piece.4);
        match &mut current {
            Some(c) => {
                c.edges.push(edge);
                c.end = piece.1;
            }
            None => {
                current = Some(Chain { edges: vec![edge], start: piece.0, end: piece.1 });
            }
        }
    }
    if let Some(c) = current.take() {
        chains.push(c);
    }
    if chains.is_empty() {
        return Err("trimmed loop kept nothing after welding".into());
    }
    Ok(Some(chains))
}

fn advance_along(curve: &Curve, from: Vec3, to: Vec3, dir: Vec3) -> Option<(f32, f32)> {
    let t_from = param_of(curve, from);
    let t_to = param_of(curve, to);
    if (curve.point(t_to) - to).length() > 1e-2 {
        return None;
    }
    let h = 1e-3;
    let tangent = curve.point(t_from + h) - curve.point(t_from - h);
    if tangent.length() < 1e-9 {
        return None;
    }
    let sign = if tangent.dot(dir) > 0.0 { 1.0 } else { -1.0 };
    let mut delta = (t_to - t_from) * sign;
    if matches!(curve, Curve::Line { .. }) {
        return (delta > ON_PLANE).then_some((delta, delta * sign));
    }
    while delta <= ON_PLANE {
        delta += std::f32::consts::TAU;
    }
    Some((delta, delta * sign))
}

fn close_chains(
    b: &mut Builder,
    surface: &Surface,
    sense: bool,
    plane: &Surface,
    discard_normal: Vec3,
    mut chains: Vec<Chain>,
    connectors: &mut Vec<(EdgeId, bool)>,
) -> Result<Vec<Vec<(EdgeId, bool)>>, String> {
    let mut loops: Vec<Vec<(EdgeId, bool)>> = Vec::new();
    while let Some(mut chain) = chains.pop() {
        loop {
            let normal = winding_normal(surface, sense, chain.end);
            let dir = normal.cross(discard_normal);
            if dir.length() < ON_PLANE {
                return Err("cut is tangent to a face; no connector direction".into());
            }
            let dir = dir.normalize();

            let mut best: Option<(Option<usize>, f32, Curve, f32)> = None;
            let consider = |target: Vec3,
                            idx: Option<usize>,
                            best: &mut Option<(Option<usize>, f32, Curve, f32)>| {
                if let Some(curve) = connector_curve(surface, plane, chain.end, target)
                    && let Some((advance, signed)) = advance_along(&curve, chain.end, target, dir)
                    && best.as_ref().is_none_or(|(_, d, _, _)| advance < *d)
                {
                    *best = Some((idx, advance, curve, signed));
                }
            };
            consider(chain.start, None, &mut best);
            for i in 0..chains.len() {
                consider(chains[i].start, Some(i), &mut best);
            }

            let Some((idx, _, curve, signed)) = best else {
                return Err("no closed-form section curve for a face the cut crosses".into());
            };
            let target = match idx {
                None => chain.start,
                Some(i) => chains[i].start,
            };
            let vs = b.vertex(chain.end);
            let ve = b.vertex(target);
            if vs != ve {
                let t0 = param_of(&curve, chain.end);
                let edge = emit_edge(b, vs, ve, curve, t0, t0 + signed);
                chain.edges.push(edge);
                connectors.push(edge);
            }
            match idx {
                None => {
                    chain.end = chain.start;
                    break;
                }
                Some(i) => {
                    let next = chains.remove(i);
                    chain.edges.extend(next.edges);
                    chain.end = next.end;
                }
            }
        }
        loops.push(chain.edges);
    }
    Ok(loops)
}

pub fn trim_half_space(solid: &Solid, plane: &Surface, keep: Side) -> Result<Solid, String> {
    let discard = match keep {
        Side::Negative => Side::Positive,
        Side::Positive => Side::Negative,
        Side::On => return Err("cannot keep only the material on the plane".into()),
    };
    let Surface::Plane { origin, normal, .. } = *plane else {
        return Err("half-space trim needs a planar cut".into());
    };
    let discard_normal = if discard == Side::Positive { normal } else { -normal };

    let mut b = Builder::new();
    let mut connectors: Vec<(EdgeId, bool)> = Vec::new();

    for fid in 0..solid.faces.len() {
        let face = solid.faces[fid].clone();
        let mut intact: Vec<Vec<(EdgeId, bool)>> = Vec::new();
        let mut cut_chains: Vec<Chain> = Vec::new();
        let mut dropped_any = false;
        for lp in solid.face_loops(fid) {
            match trim_loop(&mut b, solid, lp, plane, discard)? {
                None => intact.push(rebuild_loop(&mut b, solid, lp)),
                Some(chains) if chains.is_empty() => dropped_any = true,
                Some(chains) => cut_chains.extend(chains),
            }
        }
        if intact.is_empty() && cut_chains.is_empty() {
            continue;
        }
        if cut_chains.is_empty() {
            if dropped_any && intact.is_empty() {
                continue;
            }
            let outer = intact.remove(0);
            let inners: Vec<&[(EdgeId, bool)]> = intact.iter().map(|l| l.as_slice()).collect();
            b.face_from(face.surface, face.sense, &outer, &inners);
            continue;
        }
        let closed = close_chains(
            &mut b,
            &face.surface,
            face.sense,
            plane,
            discard_normal,
            cut_chains,
            &mut connectors,
        )?;
        let mut all = closed;
        all.extend(intact);
        let outer = all.remove(0);
        let inners: Vec<&[(EdgeId, bool)]> = all.iter().map(|l| l.as_slice()).collect();
        b.face_from(face.surface, face.sense, &outer, &inners);
    }

    if connectors.is_empty() {
        return Err("the cut plane misses the solid entirely".into());
    }
    emit_caps(&mut b, &connectors, origin, discard_normal)?;
    let solid = b.build();
    solid.validate().map_err(|e| format!("split: {e}"))?;
    Ok(solid)
}

fn rebuild_loop(b: &mut Builder, solid: &Solid, lp: &[(EdgeId, bool)]) -> Vec<(EdgeId, bool)> {
    lp.iter()
        .map(|&(e, fwd)| {
            let ed = solid.edges[e];
            let (ta, tb) = if fwd { (ed.t0, ed.t1) } else { (ed.t1, ed.t0) };
            let (v0, v1) = solid.directed(e, fwd);
            let vs = b.vertex(solid.vertex(v0));
            let ve = b.vertex(solid.vertex(v1));
            emit_edge(b, vs, ve, ed.curve, ta, tb)
        })
        .collect()
}

fn emit_caps(
    b: &mut Builder,
    connectors: &[(EdgeId, bool)],
    origin: Vec3,
    outward: Vec3,
) -> Result<(), String> {
    let mut remaining: Vec<(EdgeId, bool)> =
        connectors.iter().map(|&(e, fwd)| (e, !fwd)).collect();
    let mut loops: Vec<Vec<(EdgeId, bool)>> = Vec::new();
    while let Some(seed) = remaining.pop() {
        let mut lp = vec![seed];
        loop {
            let (_, tail) = b.directed_ends(lp[lp.len() - 1]);
            let (head, _) = b.directed_ends(lp[0]);
            if tail == head {
                break;
            }
            let Some(k) = remaining.iter().position(|&d| b.directed_ends(d).0 == tail) else {
                return Err("cut section does not close into a loop".into());
            };
            lp.push(remaining.remove(k));
        }
        loops.push(lp);
    }

    let surface = Surface::plane(origin, outward);
    let (u_dir, v_dir) = match surface {
        Surface::Plane { u_dir, v_dir, .. } => (u_dir, v_dir),
        _ => unreachable!(),
    };
    let to_2d = |p: Vec3| (p.dot(u_dir), p.dot(v_dir));
    let poly = |lp: &[(EdgeId, bool)]| -> Vec<(f32, f32)> {
        lp.iter().map(|&d| to_2d(b.point(b.directed_ends(d).0))).collect()
    };
    let area = |pts: &[(f32, f32)]| -> f32 {
        let mut a = 0.0;
        for i in 0..pts.len() {
            let (x0, y0) = pts[i];
            let (x1, y1) = pts[(i + 1) % pts.len()];
            a += x0 * y1 - x1 * y0;
        }
        a * 0.5
    };
    let inside = |pts: &[(f32, f32)], p: (f32, f32)| -> bool {
        let mut hit = false;
        for i in 0..pts.len() {
            let (x0, y0) = pts[i];
            let (x1, y1) = pts[(i + 1) % pts.len()];
            if (y0 > p.1) != (y1 > p.1) {
                let x = x0 + (p.1 - y0) / (y1 - y0) * (x1 - x0);
                if x > p.0 {
                    hit = !hit;
                }
            }
        }
        hit
    };

    let polys: Vec<Vec<(f32, f32)>> = loops.iter().map(|l| poly(l)).collect();
    let areas: Vec<f32> = polys.iter().map(|p| area(p)).collect();
    let mut used = vec![false; loops.len()];
    for i in 0..loops.len() {
        if areas[i] <= 0.0 {
            continue;
        }
        let mut inners: Vec<usize> = Vec::new();
        for j in 0..loops.len() {
            if i != j && areas[j] < 0.0 && inside(&polys[i], polys[j][0]) {
                inners.push(j);
                used[j] = true;
            }
        }
        used[i] = true;
        let inner_slices: Vec<&[(EdgeId, bool)]> =
            inners.iter().map(|&j| loops[j].as_slice()).collect();
        b.face_from(surface, true, &loops[i], &inner_slices);
    }
    if used.iter().any(|u| !u) {
        return Err("a cut section loop was neither an outer nor inside one".into());
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::PI;

    fn plane_x(c: f32) -> Surface {
        Surface::plane(Vec3::new(c, 0.0, 0.0), Vec3::X)
    }

    fn assert_lands_on_plane(curve: &Curve, params: &[f32], plane: &Surface) {
        for &t in params {
            let d = plane.signed_distance(curve.point(t));
            assert!(d.abs() < 1e-3, "t={t} is {d} off the plane");
        }
    }

    #[test]
    fn a_line_crossing_the_plane_yields_its_single_parameter() {
        let curve = Curve::line(Vec3::new(-5.0, 0.0, 0.0), Vec3::new(5.0, 0.0, 0.0));
        let params = curve_plane_params(&curve, 0.0, 10.0, &plane_x(-2.0));
        assert_eq!(params.len(), 1);
        assert_lands_on_plane(&curve, &params, &plane_x(-2.0));
    }

    #[test]
    fn a_line_parallel_to_the_plane_never_crosses() {
        let curve = Curve::line(Vec3::new(0.0, -5.0, 0.0), Vec3::new(0.0, 5.0, 0.0));
        assert!(curve_plane_params(&curve, 0.0, 10.0, &plane_x(3.0)).is_empty());
    }

    #[test]
    fn a_full_circle_crosses_a_secant_plane_twice() {
        let curve = Curve::circle_z(Vec3::ZERO, 4.0);
        let plane = plane_x(1.5);
        let params = curve_plane_params(&curve, -PI, PI, &plane);
        assert_eq!(params.len(), 2, "a secant plane cuts a circle twice: {params:?}");
        assert_lands_on_plane(&curve, &params, &plane);
    }

    #[test]
    fn a_plane_clear_of_the_circle_yields_nothing() {
        let curve = Curve::circle_z(Vec3::ZERO, 4.0);
        assert!(curve_plane_params(&curve, -PI, PI, &plane_x(9.0)).is_empty());
    }

    #[test]
    fn only_crossings_inside_the_edge_range_are_reported() {
        let curve = Curve::circle_z(Vec3::ZERO, 4.0);
        let plane = plane_x(1.5);
        let quarter = curve_plane_params(&curve, 0.0, PI / 2.0, &plane);
        assert_eq!(quarter.len(), 1, "one crossing in the first quadrant: {quarter:?}");
        assert_lands_on_plane(&curve, &quarter, &plane);
    }

    #[test]
    fn an_ellipse_crosses_a_secant_plane_twice() {
        let curve = Curve::Ellipse {
            center: Vec3::ZERO,
            a: Vec3::new(6.0, 0.0, 0.0),
            b: Vec3::new(0.0, 3.0, 0.0),
        };
        let plane = plane_x(2.0);
        let params = curve_plane_params(&curve, -PI, PI, &plane);
        assert_eq!(params.len(), 2, "{params:?}");
        assert_lands_on_plane(&curve, &params, &plane);
    }

    #[test]
    fn a_torus_section_already_lies_in_the_plane_so_it_never_crosses() {
        let curve = Curve::torus_section(Vec3::ZERO, Vec3::Z, Vec3::X, 3.0, 10.0, 2.0, 1.0);
        assert!(curve_plane_params(&curve, -PI, PI, &plane_x(3.0)).is_empty());
    }

    #[test]
    fn param_of_inverts_every_curve_type_in_closed_form() {
        let curves = [
            Curve::line(Vec3::new(-3.0, 1.0, 2.0), Vec3::new(5.0, 1.0, 2.0)),
            Curve::circle_z(Vec3::new(1.0, -2.0, 3.0), 4.0),
            Curve::Ellipse {
                center: Vec3::new(0.5, 0.0, -1.0),
                a: Vec3::new(6.0, 0.0, 0.0),
                b: Vec3::new(0.0, 3.0, 0.0),
            },
            Curve::torus_section(Vec3::new(2.0, 1.0, 0.0), Vec3::Z, Vec3::X, 1.0, 10.0, 2.0, 1.0),
        ];
        for curve in &curves {
            for i in 1..12 {
                let t = -1.2 + i as f32 * 0.2;
                let p = curve.point(t);
                let back = curve.point(param_of(curve, p));
                assert!(
                    (back - p).length() < 1e-3,
                    "{curve:?} at t={t}: round trip moved {} mm",
                    (back - p).length()
                );
            }
        }
    }

    use crate::kernel::build::extrude;
    use crate::kernel::sketch::Sketch;
    use crate::kernel::tess::tessellate;

    fn volume(solid: &crate::kernel::topo::Solid) -> f64 {
        volume_at(solid, 12)
    }

    fn volume_at(solid: &crate::kernel::topo::Solid, segs: usize) -> f64 {
        let mesh = tessellate(solid, segs).to_mesh();
        let mut v = 0.0f64;
        for [a, b, c] in mesh.triangles() {
            v += a.dot(b.cross(c)) as f64;
        }
        v / 6.0
    }

    fn assert_mesh_closed(solid: &crate::kernel::topo::Solid) {
        use std::collections::HashMap;
        let mesh = tessellate(solid, 12).to_mesh();
        let mut dir: HashMap<(u32, u32), i32> = HashMap::new();
        for t in mesh.indices.chunks_exact(3) {
            for &(a, b) in &[(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                *dir.entry((a, b)).or_default() += 1;
            }
        }
        for (&(a, b), &f) in dir.iter() {
            let r = dir.get(&(b, a)).copied().unwrap_or(0);
            assert_eq!(f, r, "edge ({a},{b}) unpaired: {f} vs {r}");
        }
    }

    #[test]
    fn cutting_a_box_gives_two_valid_halves_that_conserve_volume() {
        let solid = extrude(&Sketch::rectangle(0.0, 0.0, 10.0, 20.0), 0.0, 5.0);
        let plane = plane_x(1.5);
        let lo = trim_half_space(&solid, &plane, Side::Negative).expect("negative half");
        let hi = trim_half_space(&solid, &plane, Side::Positive).expect("positive half");
        lo.validate().expect("low half manifold");
        hi.validate().expect("high half manifold");
        assert_mesh_closed(&lo);
        assert_mesh_closed(&hi);
        let (vw, vl, vh) = (volume(&solid), volume(&lo), volume(&hi));
        assert!(vl > 0.0 && vh > 0.0, "halves must have positive volume: {vl} {vh}");
        assert!((vl + vh - vw).abs() < 1e-2, "{vl} + {vh} != {vw}");
    }

    #[test]
    fn cutting_a_rounded_prism_keeps_its_corner_cylinders_watertight() {
        let solid = extrude(&Sketch::rounded_rect(0.0, 0.0, 40.0, 30.0, 5.0), 0.0, 7.0);
        let plane = plane_x(3.0);
        let lo = trim_half_space(&solid, &plane, Side::Negative).expect("negative half");
        let hi = trim_half_space(&solid, &plane, Side::Positive).expect("positive half");
        lo.validate().expect("low half manifold");
        hi.validate().expect("high half manifold");
        assert_mesh_closed(&lo);
        assert_mesh_closed(&hi);
        let (vw, vl, vh) = (volume(&solid), volume(&lo), volume(&hi));
        assert!((vl + vh - vw).abs() < 1e-1, "{vl} + {vh} != {vw}");
    }

    #[test]
    fn cutting_a_bin_gives_two_watertight_halves_that_conserve_volume() {
        for cut in [21.0, 42.0, 50.0] {
            let p = crate::gridfinity::Params::rect(2, 1);
            let solid = crate::gridfinity::build(&p);
            let plane = plane_x(cut);
            let lo = trim_half_space(&solid, &plane, Side::Negative)
                .unwrap_or_else(|e| panic!("x={cut} negative half: {e}"));
            let hi = trim_half_space(&solid, &plane, Side::Positive)
                .unwrap_or_else(|e| panic!("x={cut} positive half: {e}"));
            lo.validate().expect("low half manifold");
            hi.validate().expect("high half manifold");
            assert_mesh_closed(&lo);
            assert_mesh_closed(&hi);
            let (vw, vl, vh) = (volume(&solid), volume(&lo), volume(&hi));
            assert!(vl > 0.0 && vh > 0.0, "x={cut}: halves must have volume: {vl} {vh}");
            assert!((vl + vh - vw).abs() < 0.05, "x={cut}: {vl} + {vh} != {vw}");
        }
    }

    #[test]
    fn a_cut_through_a_floor_fillet_keeps_the_blend_on_its_own_surface() {
        let p = crate::gridfinity::Params::rect(2, 1);
        let solid = crate::gridfinity::build(&p);
        let plane = plane_x(21.0);
        for keep in [Side::Negative, Side::Positive] {
            let half = trim_half_space(&solid, &plane, keep).expect("half");
            let tess = tessellate(&half, 24);
            for (ti, tri) in tess.tris.iter().enumerate() {
                let face = &half.faces[tess.face_of_tri[ti]];
                let Surface::Cylinder { base, axis, radius, .. } = face.surface else { continue };
                let c = (tri.pos[0] + tri.pos[1] + tri.pos[2]) / 3.0;
                let v = c - base;
                let d = (v - axis * v.dot(axis)).length();
                assert!(
                    (d - radius).abs() < 0.05,
                    "a cylinder triangle sits {d} from the axis, not {radius}"
                );
            }
        }
    }

    #[test]
    fn sides_are_classified_with_an_on_plane_band() {
        let plane = plane_x(2.0);
        assert_eq!(side_of(&plane, Vec3::new(5.0, 0.0, 0.0)), Side::Positive);
        assert_eq!(side_of(&plane, Vec3::new(-5.0, 0.0, 0.0)), Side::Negative);
        assert_eq!(side_of(&plane, Vec3::new(2.0, 7.0, -3.0)), Side::On);
    }
}
