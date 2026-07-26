use crate::kernel::geom::{Curve, Surface, radial_frame};
use crate::kernel::isect::{Intersection, intersect_surfaces};
use crate::kernel::math::Vec3;
use crate::kernel::topo::{Builder, EdgeId, VertexId};

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
        Intersection::Coincident => None,
        _ => None,
    }
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

    #[test]
    fn sides_are_classified_with_an_on_plane_band() {
        let plane = plane_x(2.0);
        assert_eq!(side_of(&plane, Vec3::new(5.0, 0.0, 0.0)), Side::Positive);
        assert_eq!(side_of(&plane, Vec3::new(-5.0, 0.0, 0.0)), Side::Negative);
        assert_eq!(side_of(&plane, Vec3::new(2.0, 7.0, -3.0)), Side::On);
    }
}
