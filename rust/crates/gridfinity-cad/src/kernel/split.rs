use crate::kernel::curvedge::emit_edge;
use crate::kernel::geom::{Curve, Surface, radial_frame};
use crate::kernel::isect::{Intersection, intersect_surfaces};
use crate::kernel::math::{Dir, Vec2, Vec3};
use crate::kernel::sketch::{point_in_polygon, polygon_area};
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
    let c = origin.dot(*normal);
    match *curve {
        Curve::Line { p0, dir } => {
            let denom = dir.dot(*normal);
            if denom.abs() < ON_PLANE {
                return Vec::new();
            }
            let t = (c - p0.dot(*normal)) / denom;
            if within(t) { vec![t] } else { Vec::new() }
        }
        Curve::Circle {
            center,
            axis,
            radius,
            ref_dir,
        } => {
            let (d0, d1) = radial_frame(axis, ref_dir);
            harmonic_roots(
                radius * d0.dot(*normal),
                radius * d1.dot(*normal),
                c - center.dot(*normal),
                lo,
                hi,
            )
            .into_iter()
            .filter(|&t| within(t))
            .collect()
        }
        Curve::Ellipse { center, a, b } => {
            harmonic_roots(a.dot(*normal), b.dot(*normal), c - center.dot(*normal), lo, hi)
                .into_iter()
                .filter(|&t| within(t))
                .collect()
        }
        Curve::TorusSection {
            center,
            axis,
            ref_dir,
            major,
            minor,
            offset,
            branch,
        } => torus_section_roots(
            center, axis, ref_dir, major, minor, offset, branch, *normal, c, lo, hi,
        )
        .into_iter()
        .filter(|&t| within(t))
        .collect(),
    }
}

/// `root` shifted by whole turns into `[lo, hi]`, and `None` when no whole
/// number of turns lands it there.
fn wrap_into(root: f32, lo: f32, hi: f32) -> Option<f32> {
    let mut t = root;
    let two_pi = std::f32::consts::TAU;
    while t < lo {
        t += two_pi;
    }
    while t > hi {
        t -= two_pi;
    }
    (t >= lo).then_some(t)
}

/// Minor angles at which a torus section crosses the plane `x . n = c`, before
/// the caller's range filter.
///
/// A `TorusSection` never crosses **the plane that cut it out** -- it lies in
/// that plane -- and this used to answer every plane with that one fact. It is
/// false for any other plane of a multi-plane cut, and the case is ordinary: a
/// piece carved out of an L turns a corner while a connector is running along a
/// reentrant floor fillet's section, so the section has to hand over to the next
/// plane of the prism and nothing reported where.
///
/// Writing `point(t)` out with `rad . cos u = offset` substituted, a point of the
/// section is `center + offset . d0 + branch . sqrt(rad^2 - offset^2) . d1 +
/// minor . sin t . axis` for `rad = major + minor . cos t` and `(d0, d1)` the
/// section plane's own frame. Dotting that with `n` gives
///
/// ```text
/// A . sqrt(rad^2 - offset^2) + B . sin t = K
/// A = branch . (d1 . n)   B = minor . (axis . n)   K = c - center . n - offset . (d0 . n)
/// ```
///
/// which is closed form in each of the two families the kernel produces, and a
/// quartic in `cos t` in between:
///
/// - **`n` perpendicular to the axis** (`B = 0`), every plane of a `Cut::prism`
///   swept along the torus's own axis: `sqrt(rad^2 - offset^2) = K / A` forces
///   `rad^2 = offset^2 + (K/A)^2`, so `cos t = (R - major) / minor` for that one
///   `R >= 0`. Only `+R` is a root: `torus_section_exists` bounds the curve to
///   `rad > 0`, and the caller must not sample it outside that.
/// - **`n` along the axis** (`A = 0`): `sin t = K / B` directly.
///
/// The square root is non-negative, so `K / A < 0` in the first family means the
/// plane is crossed by the *other* branch of the section and not by this one --
/// no root, rather than a root of the squared equation that does not satisfy the
/// original.
#[allow(clippy::too_many_arguments)]
fn torus_section_roots(
    center: Vec3,
    axis: Dir,
    ref_dir: Dir,
    major: f32,
    minor: f32,
    offset: f32,
    branch: f32,
    n: Vec3,
    c: f32,
    lo: f32,
    hi: f32,
) -> Vec<f32> {
    assert!(
        minor > 0.0,
        "a torus section's minor radius is the blend radius it was rolled with and is positive, \
         got {minor}"
    );
    let (d0, d1) = radial_frame(axis, ref_dir);
    let a_coef = branch * d1.dot(n);
    let b_coef = minor * axis.dot(n);
    let k = c - center.dot(n) - offset * d0.dot(n);

    let roots: Vec<f32> = if b_coef.abs() < ON_PLANE {
        if a_coef.abs() < ON_PLANE {
            return Vec::new();
        }
        let s = k / a_coef;
        if s < 0.0 {
            return Vec::new();
        }
        let r = (offset * offset + s * s).sqrt();
        let cos_t = (r - major) / minor;
        if cos_t.abs() > 1.0 {
            return Vec::new();
        }
        let base = cos_t.acos();
        vec![base, -base]
    } else if a_coef.abs() < ON_PLANE {
        let s = k / b_coef;
        if s.abs() > 1.0 {
            return Vec::new();
        }
        let base = s.asin();
        vec![base, std::f32::consts::PI - base]
    } else {
        panic!(
            "a torus section against a plane oblique to its axis is a quartic in cos t and is \
             not solved: axis {axis:?} against plane normal {n:?}, which meet at a cosine of {}",
            axis.dot(n)
        )
    };

    let mut out: Vec<f32> = roots
        .into_iter()
        .filter_map(|r| wrap_into(r, lo, hi))
        .collect();
    out.sort_by(f32::total_cmp);
    out.dedup_by(|x, y| (*x - *y).abs() < ON_PLANE);
    out
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
    let mut out: Vec<f32> = [phase + base, phase - base]
        .into_iter()
        .filter_map(|root| wrap_into(root, lo, hi))
        .collect();
    out.sort_by(f32::total_cmp);
    out.dedup_by(|x, y| (*x - *y).abs() < ON_PLANE);
    out
}

pub fn param_of(curve: &Curve, p: Vec3) -> f32 {
    match *curve {
        Curve::Line { p0, dir } => (p - p0).dot(*dir),
        Curve::Circle {
            center,
            axis,
            ref_dir,
            ..
        } => {
            let (d0, d1) = radial_frame(axis, ref_dir);
            let v = p - center;
            v.dot(d1).atan2(v.dot(d0))
        }
        Curve::Ellipse { center, a, b } => {
            let v = p - center;
            let (la, lb) = (a.length_squared().max(1e-12), b.length_squared().max(1e-12));
            (v.dot(b) / lb).atan2(v.dot(a) / la)
        }
        Curve::TorusSection {
            center,
            axis,
            major,
            minor,
            ..
        } => {
            let v = p - center;
            let along = v.dot(*axis);
            let radial = (v - *axis * along).length();
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

/// The cut surface: one or more oriented planes whose kept side is the negative
/// side of each `discard_normal`. A half-space is the one-plane case; a
/// rectilinear prism is the many-plane case, where a point is discarded only
/// when it is on the discard side of the plane whose window contains it.
pub struct Cut {
    planes: Vec<CutPlane>,
    region: Option<Prism>,
}

struct CutPlane {
    surface: Surface,
    origin: Vec3,
    discard_normal: Vec3,
    /// The extent of this plane that is actually part of the cut surface. `None`
    /// is an unbounded window, which is what makes a half-space a plain plane.
    span: Option<(Vec3, Vec3)>,
}

/// A vertical prism over a 2D region, traced material-on-the-left.
struct Prism {
    loops: Vec<Vec<Vec2>>,
    axis: Vec3,
}

impl CutPlane {
    /// Whether a point lies on this plane *and* within its window.
    fn holds(&self, p: Vec3) -> bool {
        if side_of(&self.surface, p) != Side::On {
            return false;
        }
        let Some((a, b)) = self.span else {
            return true;
        };
        let along = b - a;
        let len2 = along.length_squared();
        if len2 < 1e-12 {
            return false;
        }
        let t = (p - a).dot(along) / len2;
        let slack = ON_PLANE / len2.sqrt();
        t >= -slack && t <= 1.0 + slack
    }
}

impl Cut {
    pub fn half_space(plane: &Surface, keep: Side) -> Result<Cut, String> {
        let Surface::Plane { origin, normal, .. } = *plane else {
            return Err("half-space trim needs a planar cut".into());
        };
        let discard_normal = match keep {
            Side::Negative => normal,
            Side::Positive => -normal,
            Side::On => return Err("cannot keep only the material on the plane".into()),
        };
        Ok(Cut {
            planes: vec![CutPlane {
                surface: *plane,
                origin,
                discard_normal: *discard_normal,
                span: None,
            }],
            region: None,
        })
    }

    /// Keep the material inside a vertical prism over `loops`, each traced with
    /// the kept region on its left (outers CCW, holes CW).
    pub fn prism(loops: &[Vec<Vec2>], axis: Vec3) -> Result<Cut, String> {
        let mut planes = Vec::new();
        for lp in loops {
            if lp.len() < 3 {
                return Err("a prism loop needs at least three points".into());
            }
            let (u, v) = axis_frame(axis);
            for i in 0..lp.len() {
                let p0 = lp[i];
                let p1 = lp[(i + 1) % lp.len()];
                let a = u * p0.x + v * p0.y;
                let b = u * p1.x + v * p1.y;
                let along = b - a;
                if along.length() < ON_PLANE {
                    continue;
                }
                let outward = along.normalize().cross(axis);
                planes.push(CutPlane {
                    surface: Surface::plane(a, outward),
                    origin: a,
                    discard_normal: outward,
                    span: Some((a, b)),
                });
            }
        }
        if planes.is_empty() {
            return Err("a prism cut needs at least one boundary segment".into());
        }
        Ok(Cut {
            planes,
            region: Some(Prism {
                loops: loops.to_vec(),
                axis,
            }),
        })
    }

    pub fn side_of_point(&self, p: Vec3) -> Side {
        self.side_of(p)
    }

    fn side_of(&self, p: Vec3) -> Side {
        if self.planes.iter().any(|cp| cp.holds(p)) {
            return Side::On;
        }
        match &self.region {
            Some(prism) => {
                if prism.contains(p) {
                    Side::Positive
                } else {
                    Side::Negative
                }
            }
            None => {
                for cp in &self.planes {
                    let s = side_of(&cp.surface, p);
                    let discarded = (s == Side::Positive)
                        == (cp.discard_normal.dot(plane_normal(&cp.surface)) > 0.0);
                    if discarded {
                        return Side::Negative;
                    }
                }
                Side::Positive
            }
        }
    }

    /// Parameters at which an edge crosses the cut surface, each tagged with the
    /// plane it crosses.
    fn crossings(&self, curve: &Curve, t0: f32, t1: f32) -> Vec<(f32, usize)> {
        let mut out: Vec<(f32, usize)> = Vec::new();
        for (i, cp) in self.planes.iter().enumerate() {
            for t in curve_plane_params(curve, t0, t1, &cp.surface) {
                if self.side_of(curve.point(t)) == Side::On {
                    out.push((t, i));
                }
            }
        }
        out
    }

    /// Every plane a point on the cut surface lies on. More than one means the
    /// point sits on an edge of the cut surface, where one plane's window ends
    /// and the next begins.
    fn planes_at(&self, p: Vec3) -> Vec<usize> {
        (0..self.planes.len())
            .filter(|&i| self.planes[i].holds(p))
            .collect()
    }
}

impl Prism {
    /// Even-odd containment, with the region's loops projected off the axis.
    /// Parity is taken over *all* the loops together, so a point inside an outer
    /// but also inside one of its holes crosses an even number of times and is
    /// correctly outside the region.
    fn contains(&self, p: Vec3) -> bool {
        let (u, v) = axis_frame(self.axis);
        let q = Vec2::new(p.dot(u), p.dot(v));
        self.loops
            .iter()
            .filter(|lp| point_in_polygon(lp, q))
            .count()
            % 2
            == 1
    }
}

fn axis_frame(axis: Vec3) -> (Vec3, Vec3) {
    let a = if axis.dot(Vec3::Z).abs() > 0.9 {
        (Vec3::X, Vec3::Y)
    } else {
        (Vec3::Y, Vec3::Z)
    };
    a
}

/// Section curves of a face that pass through a point on the cut surface.
fn section_curves_through(surface: &Surface, plane: &Surface, from: Vec3) -> Vec<Curve> {
    match intersect_surfaces(surface, plane) {
        Intersection::Curves(cs) => cs
            .into_iter()
            .filter(|c| (c.point(param_of(c, from)) - from).length() < 1e-2)
            .collect(),
        _ => Vec::new(),
    }
}

/// Why no connector left `from` along the cut, as one line per plane of the cut
/// that passes through `from`: what `surface` and that plane meet in, and how
/// many of those curves ran through `from` itself.
///
/// The refusal it feeds used to say "no closed-form section curve", which is
/// only one of the two things that reach it -- a pair the curve set cannot name
/// and a section that exists but led nowhere are the same `None` at the call
/// site and want opposite fixes, one a new `Curve` variant and one a bug in the
/// walk. Naming the surface pair and the outcome separates them at the point of
/// failure instead of leaving it to be guessed downstream.
fn no_connector_reason(surface: &Surface, cut: &Cut, from: Vec3) -> String {
    let mut out = String::new();
    for pi in cut.planes_at(from) {
        let isect = intersect_surfaces(surface, &cut.planes[pi].surface);
        let through = section_curves_through(surface, &cut.planes[pi].surface, from).len();
        let what = match &isect {
            Intersection::Curves(cs) => {
                format!("{} curve(s), {through} of them through the point", cs.len())
            }
            Intersection::Unsupported(why) => format!("unsupported: {why}"),
            Intersection::Empty => "empty".to_string(),
            Intersection::Coincident => "coincident".to_string(),
            Intersection::Tangent(p) => format!("tangent at {p:?}"),
        };
        out.push_str(&format!(
            "\n  plane {pi} {:?} meets the face: {what}",
            cut.planes[pi].surface
        ));
    }
    out
}

/// Directed advance from `from` to `to` along `curve` travelling towards `dir`.
fn advance_to(curve: &Curve, t_from: f32, t_to: f32, sign: f32) -> Option<f32> {
    let mut delta = (t_to - t_from) * sign;
    if matches!(curve, Curve::Line { .. }) {
        return (delta > ON_PLANE).then_some(delta);
    }
    while delta <= ON_PLANE {
        delta += std::f32::consts::TAU;
    }
    Some(delta)
}

fn travel_sign(curve: &Curve, t_from: f32, dir: Vec3) -> Option<f32> {
    let h = 1e-3;
    let tangent = curve.point(t_from + h) - curve.point(t_from - h);
    if tangent.length() < 1e-9 {
        return None;
    }
    Some(if tangent.dot(dir) > 0.0 { 1.0 } else { -1.0 })
}

/// Points ahead along `curve` where the plane being followed hands over to
/// another plane of the cut — the edges of the cut surface.
fn window_exits(cut: &Cut, curve: &Curve, from: Vec3, dir: Vec3) -> Vec<(f32, f32, Vec3)> {
    let t_from = param_of(curve, from);
    let Some(sign) = travel_sign(curve, t_from, dir) else {
        return Vec::new();
    };
    let (lo, hi) = match curve {
        Curve::Line { .. } => (t_from - 1e6, t_from + 1e6),
        _ => (
            t_from - std::f32::consts::TAU,
            t_from + std::f32::consts::TAU,
        ),
    };
    let mut out = Vec::new();
    for cp in &cut.planes {
        for t in curve_plane_params(curve, lo, hi, &cp.surface) {
            let p = curve.point(t);
            if cut.side_of(p) != Side::On {
                continue;
            }
            if let Some(advance) = advance_to(curve, t_from, t, sign) {
                out.push((advance, advance * sign, p));
            }
        }
    }
    out
}

/// A connector may only run along the cut surface, never across material the
/// cut removes.
fn runs_along_cut(cut: &Cut, curve: &Curve, from: Vec3, signed: f32) -> bool {
    let t0 = param_of(curve, from);
    for k in 1..4 {
        let t = t0 + signed * (k as f32 / 4.0);
        if cut.side_of(curve.point(t)) != Side::On {
            return false;
        }
    }
    true
}

fn plane_normal(surface: &Surface) -> Vec3 {
    match *surface {
        Surface::Plane { normal, .. } => *normal,
        _ => Vec3::ZERO,
    }
}

fn winding_normal(surface: &Surface, sense: bool, p: Vec3) -> Vec3 {
    let n = surface.normal(surface.project(p));
    if sense { n } else { -n }
}

fn trim_loop(
    b: &mut Builder,
    solid: &Solid,
    lp: &[(EdgeId, bool)],
    cut: &Cut,
) -> Result<Option<Vec<Chain>>, String> {
    let mut pieces: Vec<(Vec3, Vec3, Curve, f32, f32, bool)> = Vec::new();
    for &(e, fwd) in lp {
        let ed = solid.edges[e];
        let (ta, tb) = if fwd { (ed.t0, ed.t1) } else { (ed.t1, ed.t0) };
        let mut cuts: Vec<f32> = cut
            .crossings(&ed.curve, ed.t0, ed.t1)
            .into_iter()
            .map(|(t, _)| t)
            .collect();
        cuts.sort_by(f32::total_cmp);
        cuts.dedup_by(|x, y| (*x - *y).abs() < ON_PLANE);
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
            let keep = cut.side_of(mid) != Side::Negative;
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
                current = Some(Chain {
                    edges: vec![edge],
                    start: piece.0,
                    end: piece.1,
                });
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
    cut: &Cut,
    mut chains: Vec<Chain>,
    connectors: &mut Vec<(EdgeId, bool, usize)>,
) -> Result<Vec<Vec<(EdgeId, bool)>>, String> {
    let mut loops: Vec<Vec<(EdgeId, bool)>> = Vec::new();
    while let Some(mut chain) = chains.pop() {
        let mut hops = 0;
        loop {
            hops += 1;
            if hops > MAX_CONNECTOR_HOPS {
                return Err("a connector failed to close along the cut surface".into());
            }
            let at = cut.planes_at(chain.end);
            if at.is_empty() {
                return Err("a trimmed chain ends off the cut surface".into());
            }

            let mut best: Option<Connector> = None;
            let consider = |c: Connector, best: &mut Option<Connector>| {
                if best.as_ref().is_none_or(|prev| c.advance < prev.advance) {
                    *best = Some(c);
                }
            };
            for pi in at {
                let normal = winding_normal(surface, sense, chain.end);
                let dir = normal.cross(cut.planes[pi].discard_normal);
                if dir.length() < ON_PLANE {
                    continue;
                }
                let dir = dir.normalize();
                for curve in section_curves_through(surface, &cut.planes[pi].surface, chain.end) {
                    let targets = std::iter::once((None, chain.start))
                        .chain(chains.iter().enumerate().map(|(i, c)| (Some(i), c.start)));
                    for (idx, target) in targets {
                        if let Some((advance, signed)) =
                            advance_along(&curve, chain.end, target, dir)
                            && runs_along_cut(cut, &curve, chain.end, signed)
                        {
                            consider(
                                Connector {
                                    stop: Stop::Chain(idx),
                                    advance,
                                    curve,
                                    signed,
                                    plane: pi,
                                },
                                &mut best,
                            );
                        }
                    }
                    for (advance, signed, point) in window_exits(cut, &curve, chain.end, dir) {
                        if runs_along_cut(cut, &curve, chain.end, signed) {
                            consider(
                                Connector {
                                    stop: Stop::Edge(point),
                                    advance,
                                    curve,
                                    signed,
                                    plane: pi,
                                },
                                &mut best,
                            );
                        }
                    }
                }
            }

            let Some(Connector {
                stop,
                curve,
                signed,
                plane: pi,
                ..
            }) = best
            else {
                return Err(format!(
                    "no connector along the cut from {:?} on face surface {surface:?}: the chain \
                     reached the cut surface but nothing carried it onward{}",
                    chain.end,
                    no_connector_reason(surface, cut, chain.end)
                ));
            };
            let target = match stop {
                Stop::Edge(p) => p,
                Stop::Chain(None) => chain.start,
                Stop::Chain(Some(i)) => chains[i].start,
            };
            let vs = b.vertex(chain.end);
            let ve = b.vertex(target);
            if vs != ve {
                let t0 = param_of(&curve, chain.end);
                let edge = emit_edge(b, vs, ve, curve, t0, t0 + signed);
                chain.edges.push(edge);
                connectors.push((edge.0, edge.1, pi));
            }
            match stop {
                Stop::Edge(p) => {
                    if vs == ve {
                        return Err("a connector stalled on a cut-surface edge".into());
                    }
                    chain.end = p;
                }
                Stop::Chain(None) => {
                    chain.end = chain.start;
                    break;
                }
                Stop::Chain(Some(i)) => {
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

const MAX_CONNECTOR_HOPS: usize = 256;

enum Stop {
    /// The connector reaches the start of a trimmed chain and the loop continues there.
    Chain(Option<usize>),
    /// The plane's window ends here; the connector continues on the next plane.
    Edge(Vec3),
}

struct Connector {
    stop: Stop,
    advance: f32,
    curve: Curve,
    signed: f32,
    plane: usize,
}

pub fn trim_half_space(solid: &Solid, plane: &Surface, keep: Side) -> Result<Solid, String> {
    trim(solid, &Cut::half_space(plane, keep)?)
}

pub fn trim(solid: &Solid, cut: &Cut) -> Result<Solid, String> {
    let mut b = Builder::new();
    let mut connectors: Vec<(EdgeId, bool, usize)> = Vec::new();

    for fid in 0..solid.faces.len() {
        let face = solid.faces[fid].clone();
        let mut intact: Vec<Vec<(EdgeId, bool)>> = Vec::new();
        let mut cut_chains: Vec<Chain> = Vec::new();
        let mut dropped_any = false;
        for lp in solid.face_loops(fid) {
            match trim_loop(&mut b, solid, lp, cut)? {
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
            cut,
            cut_chains,
            &mut connectors,
        )?;
        let mut all = closed;
        all.extend(intact);
        emit_trimmed_faces(&mut b, face.surface, face.sense, &all)?;
    }

    if connectors.is_empty() {
        return Err("the cut plane misses the solid entirely".into());
    }
    for (i, cp) in cut.planes.iter().enumerate() {
        let on_plane: Vec<(EdgeId, bool)> = connectors
            .iter()
            .filter(|&&(_, _, pi)| pi == i)
            .map(|&(e, fwd, _)| (e, fwd))
            .collect();
        if on_plane.is_empty() {
            continue;
        }
        emit_caps(&mut b, &on_plane, cp.origin, cp.discard_normal, cut, i)?;
    }
    let solid = b.build();
    solid.validate().map_err(|e| format!("split: {e}"))?;
    Ok(solid)
}

/// Where a cap loop continues when it runs off this plane's window: along the
/// edge shared with a neighbouring plane, to the nearest point on that edge that
/// is either another cap piece's start (`Some(index)`) or the loop's own head
/// (`None`). `None` overall means no such edge, which is a malformed cut.
fn nearest_along_shared_edge(
    b: &Builder,
    cut: &Cut,
    plane: usize,
    tail: VertexId,
    head: VertexId,
    remaining: &[(EdgeId, bool)],
) -> Option<Option<usize>> {
    let tp = b.point(tail);
    let shared: Vec<usize> = cut
        .planes_at(tp)
        .into_iter()
        .filter(|&j| j != plane)
        .collect();
    if shared.is_empty() {
        return None;
    }
    let on_shared =
        |p: Vec3| (p - tp).length() > ON_PLANE && shared.iter().any(|&j| cut.planes[j].holds(p));
    let mut best: Option<(Option<usize>, f32)> = None;
    let mut consider = |idx: Option<usize>, p: Vec3| {
        if on_shared(p) {
            let d = (p - tp).length();
            if best.is_none_or(|(_, bd)| d < bd) {
                best = Some((idx, d));
            }
        }
    };
    consider(None, b.point(head));
    for (i, &d) in remaining.iter().enumerate() {
        consider(Some(i), b.point(b.directed_ends(d).0));
    }
    best.map(|(idx, _)| idx)
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

fn unwrap_u(pts: &mut [Vec2]) {
    use std::f32::consts::{PI, TAU};
    for i in 1..pts.len() {
        while pts[i].x - pts[i - 1].x > PI {
            pts[i].x -= TAU;
        }
        while pts[i].x - pts[i - 1].x < -PI {
            pts[i].x += TAU;
        }
    }
    let min = pts.iter().map(|p| p.x).fold(f32::INFINITY, f32::min);
    let shift = (min / TAU).floor() * TAU;
    if shift != 0.0 {
        for p in pts.iter_mut() {
            p.x -= shift;
        }
    }
}

fn loop_encloses(outer: &[Vec2], inner: &[Vec2], angular: bool) -> bool {
    let p = inner[0];
    if point_in_polygon(outer, p) {
        return true;
    }
    if !angular {
        return false;
    }
    let tau = std::f32::consts::TAU;
    point_in_polygon(outer, Vec2::new(p.x + tau, p.y))
        || point_in_polygon(outer, Vec2::new(p.x - tau, p.y))
}

fn emit_trimmed_faces(
    b: &mut Builder,
    surface: Surface,
    sense: bool,
    loops: &[Vec<(EdgeId, bool)>],
) -> Result<(), String> {
    let emit_flat = |b: &mut Builder| {
        let inners: Vec<&[(EdgeId, bool)]> = loops[1..].iter().map(|l| l.as_slice()).collect();
        b.face_from(surface, sense, &loops[0], &inners);
    };
    if loops.len() < 2 {
        emit_flat(b);
        return Ok(());
    }

    let angular = !matches!(surface, Surface::Plane { .. });
    let mut polys: Vec<Vec<Vec2>> = loops
        .iter()
        .map(|lp| {
            lp.iter()
                .map(|&d| {
                    let uv = surface.project(b.point(b.directed_ends(d).0));
                    Vec2::new(uv.0, uv.1)
                })
                .collect()
        })
        .collect();
    if angular {
        for poly in &mut polys {
            unwrap_u(poly);
        }
    }
    let winding = if sense { 1.0 } else { -1.0 };
    let areas: Vec<f32> = polys.iter().map(|p| polygon_area(p) * winding).collect();

    let mut owner: Vec<Option<usize>> = vec![None; loops.len()];
    let outers: Vec<usize> = (0..loops.len())
        .filter(|&i| polys[i].len() >= 3 && areas[i] > 0.0)
        .collect();
    if outers.len() < 2 {
        emit_flat(b);
        return Ok(());
    }
    for j in 0..loops.len() {
        if areas[j] >= 0.0 || polys[j].is_empty() {
            continue;
        }
        owner[j] = outers
            .iter()
            .copied()
            .find(|&i| loop_encloses(&polys[i], &polys[j], angular));
        if owner[j].is_none() {
            emit_flat(b);
            return Ok(());
        }
    }

    for &i in &outers {
        let inners: Vec<&[(EdgeId, bool)]> = (0..loops.len())
            .filter(|&j| owner[j] == Some(i))
            .map(|j| loops[j].as_slice())
            .collect();
        b.face_from(surface, sense, &loops[i], &inners);
    }
    Ok(())
}

fn emit_caps(
    b: &mut Builder,
    connectors: &[(EdgeId, bool)],
    origin: Vec3,
    outward: Vec3,
    cut: &Cut,
    plane: usize,
) -> Result<(), String> {
    let mut remaining: Vec<(EdgeId, bool)> = connectors.iter().map(|&(e, fwd)| (e, !fwd)).collect();
    let mut loops: Vec<Vec<(EdgeId, bool)>> = Vec::new();
    while let Some(seed) = remaining.pop() {
        let mut lp = vec![seed];
        loop {
            let (_, tail) = b.directed_ends(lp[lp.len() - 1]);
            let (head, _) = b.directed_ends(lp[0]);
            if tail == head {
                break;
            }
            if let Some(k) = remaining.iter().position(|&d| b.directed_ends(d).0 == tail) {
                lp.push(remaining.remove(k));
                continue;
            }
            // The cap runs off this plane's window. It continues along the edge
            // the window shares with the neighbouring plane, which is interior
            // to the solid, so no face section produced it -- synthesise it.
            match nearest_along_shared_edge(b, cut, plane, tail, head, &remaining) {
                Some(Some(k)) => {
                    let start = b.directed_ends(remaining[k]).0;
                    lp.push(b.line(tail, start));
                    lp.push(remaining.remove(k));
                }
                Some(None) => lp.push(b.line(tail, head)),
                None => return Err("cut section does not close into a loop".into()),
            }
        }
        loops.push(lp);
    }

    let surface = Surface::plane(origin, outward);
    let (u_dir, v_dir) = surface.plane_axes();
    let to_2d = |p: Vec3| Vec2::new(p.dot(u_dir), p.dot(v_dir));
    let poly = |lp: &[(EdgeId, bool)]| -> Vec<Vec2> {
        lp.iter()
            .map(|&d| to_2d(b.point(b.directed_ends(d).0)))
            .collect()
    };
    let polys: Vec<Vec<Vec2>> = loops.iter().map(|l| poly(l)).collect();
    let areas: Vec<f32> = polys.iter().map(|p| polygon_area(p)).collect();
    let mut used = vec![false; loops.len()];
    for i in 0..loops.len() {
        if areas[i] <= 0.0 {
            continue;
        }
        let mut inners: Vec<usize> = Vec::new();
        for j in 0..loops.len() {
            if i != j && areas[j] < 0.0 && point_in_polygon(&polys[i], polys[j][0]) {
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

    fn pts(xy: &[(f32, f32)]) -> Vec<Vec2> {
        xy.iter().map(|&(x, y)| Vec2::new(x, y)).collect()
    }

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
        assert_eq!(
            params.len(),
            2,
            "a secant plane cuts a circle twice: {params:?}"
        );
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
        assert_eq!(
            quarter.len(),
            1,
            "one crossing in the first quadrant: {quarter:?}"
        );
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

    /// The section a torus was cut into by one plane, against a *second* plane.
    ///
    /// Every root has to satisfy both surfaces at once, so each is checked
    /// against the plane and against the torus's own spine -- landing on the
    /// plane alone would be satisfied by a root of the squared equation that the
    /// original does not have.
    fn assert_on_plane_and_torus(
        curve: &Curve,
        params: &[f32],
        plane: &Surface,
        center: Vec3,
        axis: Vec3,
        major: f32,
        minor: f32,
    ) {
        for &t in params {
            assert!(
                Curve::torus_section_exists(major, minor, 3.0, t),
                "t={t} is outside the section's own domain"
            );
            let p = curve.point(t);
            let d = plane.signed_distance(p);
            assert!(d.abs() < 1e-3, "t={t} lands {d} off the plane");
            let rel = p - center;
            let h = rel.dot(axis);
            let spine = ((rel - axis * h).length() - major).hypot(h);
            assert!(
                (spine - minor).abs() < 1e-3,
                "t={t} lands {spine} from the spine, not the minor radius {minor}"
            );
        }
    }

    #[test]
    fn a_torus_section_crosses_the_next_plane_of_a_prism_twice() {
        let curve = Curve::torus_section(Vec3::ZERO, Vec3::Z, Vec3::X, 3.0, 10.0, 2.0, 1.0);
        let plane = Surface::plane(Vec3::new(0.0, 9.0, 0.0), Vec3::Y);
        let params = curve_plane_params(&curve, -PI, PI, &plane);
        assert_eq!(
            params.len(),
            2,
            "the section reaches out to y=11.6 and back, so a plane at y=9 cuts it \
             twice: {params:?}"
        );
        assert_on_plane_and_torus(&curve, &params, &plane, Vec3::ZERO, Vec3::Z, 10.0, 2.0);
    }

    #[test]
    fn a_plane_past_the_reach_of_a_torus_section_is_not_crossed() {
        let curve = Curve::torus_section(Vec3::ZERO, Vec3::Z, Vec3::X, 3.0, 10.0, 2.0, 1.0);
        let plane = Surface::plane(Vec3::new(0.0, 20.0, 0.0), Vec3::Y);
        assert!(curve_plane_params(&curve, -PI, PI, &plane).is_empty());
    }

    #[test]
    fn a_torus_section_crosses_a_plane_square_on_to_its_axis_where_its_height_says() {
        let curve = Curve::torus_section(Vec3::ZERO, Vec3::Z, Vec3::X, 3.0, 10.0, 2.0, 1.0);
        let plane = Surface::plane_z(1.0);
        let params = curve_plane_params(&curve, -PI, PI, &plane);
        assert_eq!(
            params.len(),
            2,
            "the section rises to z=2 and falls back, so z=1 is met on the way up \
             and on the way down: {params:?}"
        );
        assert_on_plane_and_torus(&curve, &params, &plane, Vec3::ZERO, Vec3::Z, 10.0, 2.0);
    }

    #[test]
    #[should_panic(expected = "quartic in cos t")]
    fn a_plane_oblique_to_a_torus_sections_axis_is_refused_rather_than_missed() {
        let curve = Curve::torus_section(Vec3::ZERO, Vec3::Z, Vec3::X, 3.0, 10.0, 2.0, 1.0);
        let plane = Surface::plane(Vec3::ZERO, Vec3::new(0.0, 1.0, 1.0).normalize());
        curve_plane_params(&curve, -PI, PI, &plane);
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
            Curve::torus_section(
                Vec3::new(2.0, 1.0, 0.0),
                Vec3::Z,
                Vec3::X,
                1.0,
                10.0,
                2.0,
                1.0,
            ),
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
    fn a_prism_cut_keeps_the_material_inside_a_convex_window() {
        let solid = extrude(&Sketch::rectangle(0.0, 0.0, 12.0, 12.0), 0.0, 5.0);
        let strip = vec![pts(&[(-2.0, -9.0), (2.0, -9.0), (2.0, 9.0), (-2.0, 9.0)])];
        let cut = Cut::prism(&strip, Vec3::Z).unwrap();
        let kept = trim(&solid, &cut).unwrap();
        kept.validate().expect("manifold");
        assert_mesh_closed(&kept);
        assert!((volume(&kept) - 240.0).abs() < 1e-2, "{}", volume(&kept));
    }

    #[test]
    fn a_prism_cut_turns_the_corner_at_a_reentrant_window_edge() {
        let solid = extrude(&Sketch::rectangle(0.0, 0.0, 12.0, 12.0), 0.0, 5.0);
        let l = vec![pts(&[
            (-9.0, -9.0),
            (0.0, -9.0),
            (0.0, 0.0),
            (9.0, 0.0),
            (9.0, 9.0),
            (-9.0, 9.0),
        ])];
        let cut = Cut::prism(&l, Vec3::Z).unwrap();
        let kept = trim(&solid, &cut).unwrap();
        kept.validate().expect("manifold");
        assert_mesh_closed(&kept);
        assert!((volume(&kept) - 540.0).abs() < 1e-2, "{}", volume(&kept));
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
        assert!(
            vl > 0.0 && vh > 0.0,
            "halves must have positive volume: {vl} {vh}"
        );
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
            assert!(
                vl > 0.0 && vh > 0.0,
                "x={cut}: halves must have volume: {vl} {vh}"
            );
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
                let Surface::Cylinder {
                    base, axis, radius, ..
                } = face.surface
                else {
                    continue;
                };
                let c = (tri.pos[0] + tri.pos[1] + tri.pos[2]) / 3.0;
                let v = c - base;
                let d = (v - *axis * v.dot(*axis)).length();
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
