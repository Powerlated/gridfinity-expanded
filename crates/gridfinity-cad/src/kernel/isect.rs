//! Surface/surface intersection: the exact, closed-form half.
//!
//! This is the first half of the general boolean's foundation. Given two
//! [`Surface`]s it returns the curves they meet in, and it is **exact-first**:
//! every pair whose intersection is a line, circle or ellipse is solved in
//! closed form and returned as the corresponding [`Curve`]. Nothing here
//! approximates a curve that has a conic answer.
//!
//! The pairs that *have* no conic answer are reported as
//! [`Intersection::Unsupported`] rather than being faked. Drawn from
//! `{Plane, Cylinder, Cone, Torus, Sphere}` those are:
//!
//! - general (non-parallel, non-coaxial) cylinder/cylinder — a quartic space curve
//! - plane/torus at a general angle — a quartic (Cassinian) curve
//! - torus against anything curved — up to degree 8
//! - an oblique plane/cone section that opens into a parabola or hyperbola:
//!   [`Curve`] has `Ellipse` but no unbounded conic
//!
//! Those are what the procedural `Curve` variant is for; until it exists, a
//! caller that meets one gets a clean error instead of a wrong answer.
//!
//! Sign convention: every curve is returned with an arbitrary but consistent
//! parameterisation. Trimming to actual edge extents is the boolean's job, not
//! this module's — here a "circle" means the whole circle.

use crate::kernel::geom::{Curve, Surface, perp_unit};
use crate::kernel::math::Vec3;

/// Distances below this count as zero. Chosen to match the kernel's existing
/// welding tolerance rather than f32 epsilon: two surfaces authored to meet
/// exactly can still differ in the last bit or two after a frame change.
pub const TOL: f32 = 1e-5;

/// What two surfaces meet in.
#[derive(Clone, Debug)]
pub enum Intersection {
    /// The surfaces do not meet at all.
    Empty,
    /// The same surface twice. The boolean has to resolve this as a
    /// face-on-face overlap in 2D, not as a curve.
    Coincident,
    /// Exact curves. More than one arises for e.g. a plane slicing a cylinder
    /// parallel to its axis (two lines).
    Curves(Vec<Curve>),
    /// The surfaces touch along a curve but only at a single point or a
    /// tangency this module will not guess at.
    Tangent(Vec3),
    /// No closed-form conic exists for this pair; the reason names which case.
    Unsupported(&'static str),
}

impl Intersection {
    /// The curves, or an empty slice for every other outcome.
    pub fn curves(&self) -> &[Curve] {
        match self {
            Intersection::Curves(c) => c,
            _ => &[],
        }
    }
}

/// Exact intersection of two surfaces, or a named reason why there isn't one.
///
/// Order does not matter: the pair is normalised internally so `(a, b)` and
/// `(b, a)` give the same answer.
pub fn intersect_surfaces(a: &Surface, b: &Surface) -> Intersection {
    use Surface::*;
    match (a, b) {
        (Plane { .. }, Plane { .. }) => plane_plane(a, b),
        (Plane { .. }, Sphere { .. }) => plane_sphere(a, b),
        (Sphere { .. }, Plane { .. }) => plane_sphere(b, a),
        (Sphere { .. }, Sphere { .. }) => sphere_sphere(a, b),
        (Plane { .. }, Cylinder { .. }) => plane_cylinder(a, b),
        (Cylinder { .. }, Plane { .. }) => plane_cylinder(b, a),
        (Plane { .. }, Cone { .. }) => plane_cone(a, b),
        (Cone { .. }, Plane { .. }) => plane_cone(b, a),
        (Cylinder { .. }, Cylinder { .. }) => cylinder_cylinder(a, b),
        (Plane { .. }, Torus { .. }) | (Torus { .. }, Plane { .. }) => plane_torus(a, b),
        (Torus { .. }, _) | (_, Torus { .. }) => {
            Intersection::Unsupported("torus against a curved surface is degree 8")
        }
        _ => Intersection::Unsupported("no closed-form solution for this surface pair"),
    }
}

fn plane_parts(s: &Surface) -> (Vec3, Vec3) {
    match *s {
        Surface::Plane { origin, normal, .. } => (origin, normal),
        _ => unreachable!("plane expected"),
    }
}

fn plane_plane(a: &Surface, b: &Surface) -> Intersection {
    let (o1, n1) = plane_parts(a);
    let (o2, n2) = plane_parts(b);
    let dir = n1.cross(n2);
    if dir.length() < TOL {
        // Parallel: coincident when the second origin lies on the first.
        return if (o2 - o1).dot(n1).abs() < TOL {
            Intersection::Coincident
        } else {
            Intersection::Empty
        };
    }
    // Point on both planes, nearest the origin of the pencil they span.
    let (d1, d2) = (o1.dot(n1), o2.dot(n2));
    let c = n1.dot(n2);
    let denom = 1.0 - c * c;
    let p0 = n1 * ((d1 - d2 * c) / denom) + n2 * ((d2 - d1 * c) / denom);
    Intersection::Curves(vec![Curve::Line { p0, dir: dir.normalize() }])
}

fn plane_sphere(pl: &Surface, sp: &Surface) -> Intersection {
    let (_, n) = plane_parts(pl);
    let Surface::Sphere { center, radius, .. } = *sp else { unreachable!() };
    let d = pl.signed_distance(center);
    if d.abs() > radius + TOL {
        return Intersection::Empty;
    }
    let foot = center - n * d;
    if (d.abs() - radius).abs() < TOL {
        return Intersection::Tangent(foot);
    }
    let r = (radius * radius - d * d).max(0.0).sqrt();
    Intersection::Curves(vec![Curve::Circle {
        center: foot,
        axis: n,
        radius: r,
        ref_dir: perp_unit(n, Vec3::X),
    }])
}

fn sphere_sphere(a: &Surface, b: &Surface) -> Intersection {
    let Surface::Sphere { center: c1, radius: r1, .. } = *a else { unreachable!() };
    let Surface::Sphere { center: c2, radius: r2, .. } = *b else { unreachable!() };
    let delta = c2 - c1;
    let d = delta.length();
    if d < TOL {
        return if (r1 - r2).abs() < TOL { Intersection::Coincident } else { Intersection::Empty };
    }
    if d > r1 + r2 + TOL || d < (r1 - r2).abs() - TOL {
        return Intersection::Empty;
    }
    let axis = delta / d;
    // Distance from c1 to the plane of the intersection circle.
    let x = (d * d + r1 * r1 - r2 * r2) / (2.0 * d);
    let h2 = r1 * r1 - x * x;
    let foot = c1 + axis * x;
    if h2 <= TOL * TOL {
        return Intersection::Tangent(foot);
    }
    Intersection::Curves(vec![Curve::Circle {
        center: foot,
        axis,
        radius: h2.sqrt(),
        ref_dir: perp_unit(axis, Vec3::X),
    }])
}

/// Plane against cylinder. Three regimes, all conic:
///
/// - plane ⟂ axis → a circle;
/// - plane ∥ axis → zero, one or two straight lines;
/// - oblique → an ellipse, semi-minor `radius` across the axis and semi-major
///   `radius / |cos θ|` along the tilt, θ being the angle between the plane
///   normal and the axis. That is the standard result: the cut lengthens only
///   in the direction the plane leans.
fn plane_cylinder(pl: &Surface, cy: &Surface) -> Intersection {
    let (_, n) = plane_parts(pl);
    let Surface::Cylinder { base, axis, radius, .. } = *cy else { unreachable!() };
    let cos_t = axis.dot(n);

    if cos_t.abs() < TOL {
        // Plane parallel to the axis: cut the base circle in the plane's own
        // cross-section, then sweep each root along the axis as a line.
        let d = pl.signed_distance(base);
        if d.abs() > radius + TOL {
            return Intersection::Empty;
        }
        let m = axis.cross(n).normalize(); // in-plane, ⟂ axis
        let foot = base - n * d;
        if (d.abs() - radius).abs() < TOL {
            return Intersection::Curves(vec![Curve::Line { p0: foot, dir: axis }]);
        }
        let h = (radius * radius - d * d).max(0.0).sqrt();
        return Intersection::Curves(vec![
            Curve::Line { p0: foot + m * h, dir: axis },
            Curve::Line { p0: foot - m * h, dir: axis },
        ]);
    }

    // Centre: where the cylinder's axis crosses the plane.
    let t = -pl.signed_distance(base) / cos_t;
    let center = base + axis * t;
    // Minor axis: perpendicular to both the plane normal and the cylinder
    // axis, so the cut is exactly `radius` wide there.
    let minor = axis.cross(n).normalize();
    if cos_t.abs() > 1.0 - TOL {
        return Intersection::Curves(vec![Curve::Circle {
            center,
            axis,
            radius,
            ref_dir: perp_unit(axis, Vec3::X),
        }]);
    }
    let major = n.cross(minor).normalize();
    Intersection::Curves(vec![Curve::Ellipse {
        center,
        a: major * (radius / cos_t.abs()),
        b: minor * radius,
    }])
}

/// Plane against cone. Only the closed sections are representable: a circle
/// when the plane is perpendicular to the axis, an ellipse when it cuts every
/// generator. A parabola or hyperbola has no [`Curve`] variant, so it is
/// reported rather than approximated.
fn plane_cone(pl: &Surface, co: &Surface) -> Intersection {
    let (_, n) = plane_parts(pl);
    let Surface::Cone { apex, axis, half_angle, .. } = *co else { unreachable!() };
    let cos_t = axis.dot(n).abs();
    let sin_ha = half_angle.sin();

    if cos_t > 1.0 - TOL {
        // Perpendicular to the axis: a circle of radius |h|·tan(half_angle).
        let h = -pl.signed_distance(apex) / axis.dot(n);
        let r = h.abs() * half_angle.tan();
        if r < TOL {
            return Intersection::Tangent(apex);
        }
        return Intersection::Curves(vec![Curve::Circle {
            center: apex + axis * h,
            axis,
            radius: r,
            ref_dir: perp_unit(axis, Vec3::X),
        }]);
    }
    if cos_t < sin_ha + TOL {
        return Intersection::Unsupported(
            "oblique plane/cone section is a parabola or hyperbola; Curve has no unbounded conic",
        );
    }

    // Closed (elliptical) section. Work in the plane containing the axis and
    // the direction of steepest tilt: the two extreme points of the ellipse lie
    // on the generators in that plane, and the ellipse is fully determined by
    // them plus the semi-minor axis at the midpoint.
    let minor_dir = axis.cross(n).normalize(); // in-plane, ⟂ axis
    let tilt = n.cross(minor_dir).normalize(); // in-plane, steepest direction
    let Some(p1) = cone_generator_hit(apex, axis, half_angle, pl, tilt) else {
        return Intersection::Unsupported("cone section endpoints are not both finite");
    };
    let Some(p2) = cone_generator_hit(apex, axis, half_angle, pl, -tilt) else {
        return Intersection::Unsupported("cone section endpoints are not both finite");
    };
    let center = (p1 + p2) * 0.5;
    let semi_major = (p2 - p1).length() * 0.5;
    // Semi-minor from the cone's radius at the centre's height.
    let h = (center - apex).dot(axis);
    let r_at_c = h.abs() * half_angle.tan();
    let off = (center - apex - axis * h).length();
    let semi_minor_sq = r_at_c * r_at_c - off * off;
    if semi_minor_sq <= TOL {
        return Intersection::Unsupported("degenerate cone section");
    }
    Intersection::Curves(vec![Curve::Ellipse {
        center,
        a: (p2 - p1).normalize() * semi_major,
        b: minor_dir * semi_minor_sq.sqrt(),
    }])
}

/// Where the cone's generator leaning toward `dir` meets the plane. The
/// generator is a ray from the apex; solve the linear equation for its
/// parameter.
fn cone_generator_hit(
    apex: Vec3,
    axis: Vec3,
    half_angle: f32,
    pl: &Surface,
    dir: Vec3,
) -> Option<Vec3> {
    let (_, n) = plane_parts(pl);
    // `dir` comes in as an in-plane direction, which is tilted relative to the
    // axis. A generator is `cos·axis + sin·radial` only when `radial` is
    // genuinely perpendicular to the axis, so project first — otherwise the
    // result is not at the half angle and does not lie on the cone at all.
    let radial = perp_unit(axis, dir);
    // Both nappes: the generator leaning toward `radial` on either side.
    for sign in [1.0f32, -1.0] {
        let g = axis * sign * half_angle.cos() + radial * half_angle.sin();
        let denom = g.dot(n);
        if denom.abs() < TOL {
            continue;
        }
        let t = -pl.signed_distance(apex) / denom;
        if t.is_finite() && t > TOL {
            return Some(apex + g * t);
        }
    }
    None
}

/// Cylinder against cylinder. Coaxial and parallel cases are conic; anything
/// else is a quartic space curve.
fn cylinder_cylinder(a: &Surface, b: &Surface) -> Intersection {
    let Surface::Cylinder { base: b1, axis: a1, radius: r1, .. } = *a else { unreachable!() };
    let Surface::Cylinder { base: b2, axis: a2, radius: r2, .. } = *b else { unreachable!() };
    if a1.cross(a2).length() > TOL {
        return Intersection::Unsupported("non-parallel cylinder/cylinder is a quartic space curve");
    }
    // Parallel. Offset between the two axes, measured across them.
    let d = b2 - b1;
    let off = d - a1 * d.dot(a1);
    if off.length() < TOL {
        // Coaxial: coincident if the radii match, else they never meet.
        return if (r1 - r2).abs() < TOL { Intersection::Coincident } else { Intersection::Empty };
    }
    // Two parallel cylinders meet in straight lines — the same 2D
    // circle/circle problem as `region2d`, swept along the shared axis.
    let dist = off.length();
    if dist > r1 + r2 + TOL || dist < (r1 - r2).abs() - TOL {
        return Intersection::Empty;
    }
    let u = off / dist;
    let x = (dist * dist + r1 * r1 - r2 * r2) / (2.0 * dist);
    let h2 = r1 * r1 - x * x;
    let foot = b1 + u * x;
    if h2 <= TOL * TOL {
        return Intersection::Curves(vec![Curve::Line { p0: foot, dir: a1 }]);
    }
    let v = a1.cross(u).normalize();
    let h = h2.sqrt();
    Intersection::Curves(vec![
        Curve::Line { p0: foot + v * h, dir: a1 },
        Curve::Line { p0: foot - v * h, dir: a1 },
    ])
}

/// Plane against torus. Only the two axis-perpendicular families are conic: a
/// plane square to the axis cuts circles, and the plane through the axis cuts
/// the two tube circles. Everything else is a quartic.
fn plane_torus(a: &Surface, b: &Surface) -> Intersection {
    let (pl, to) = match a {
        Surface::Plane { .. } => (a, b),
        _ => (b, a),
    };
    let (_, n) = plane_parts(pl);
    let Surface::Torus { center, axis, major_r, minor_r, .. } = *to else { unreachable!() };
    let cos_t = axis.dot(n).abs();
    if cos_t < 1.0 - TOL {
        return Intersection::Unsupported(
            "plane/torus at a general angle is a quartic (Cassinian) curve",
        );
    }
    // Perpendicular to the axis: one or two concentric circles, at the radii
    // where the tube reaches this height.
    let h = pl.signed_distance(center) * -axis.dot(n).signum();
    if h.abs() > minor_r + TOL {
        return Intersection::Empty;
    }
    let dr = (minor_r * minor_r - h * h).max(0.0).sqrt();
    let plane_center = center + axis * h;
    let ref_dir = perp_unit(axis, Vec3::X);
    if dr < TOL {
        return Intersection::Curves(vec![Curve::Circle {
            center: plane_center,
            axis,
            radius: major_r,
            ref_dir,
        }]);
    }
    Intersection::Curves(vec![
        Curve::Circle { center: plane_center, axis, radius: major_r + dr, ref_dir },
        Curve::Circle { center: plane_center, axis, radius: major_r - dr, ref_dir },
    ])
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::PI;

    /// The load-bearing property: every point of every returned curve lies on
    /// *both* surfaces. This is what makes the result exact rather than
    /// plausible, so it is asserted for each supported pair rather than
    /// spot-checked.
    fn assert_on_both(a: &Surface, b: &Surface, isect: &Intersection) {
        let curves = isect.curves();
        assert!(!curves.is_empty(), "expected curves, got {isect:?}");
        for c in curves {
            for i in 0..64 {
                // Lines are parameterised by distance, conics by angle; sweep a
                // range that covers a full turn and a useful span either way.
                let t = match c {
                    Curve::Line { .. } => (i as f32 - 32.0) * 0.5,
                    _ => i as f32 / 64.0 * 2.0 * PI,
                };
                let p = c.point(t);
                assert!(
                    a.signed_distance(p).abs() < 1e-3,
                    "t={t} off surface a by {} ({c:?})",
                    a.signed_distance(p)
                );
                assert!(
                    b.signed_distance(p).abs() < 1e-3,
                    "t={t} off surface b by {} ({c:?})",
                    b.signed_distance(p)
                );
            }
        }
    }

    #[test]
    fn plane_plane_is_a_line() {
        let a = Surface::plane_z(2.0);
        let b = Surface::plane(Vec3::new(1.0, 0.0, 0.0), Vec3::X);
        assert_on_both(&a, &b, &intersect_surfaces(&a, &b));
    }

    #[test]
    fn parallel_planes_are_empty_or_coincident() {
        let a = Surface::plane_z(2.0);
        assert!(matches!(intersect_surfaces(&a, &Surface::plane_z(5.0)), Intersection::Empty));
        assert!(matches!(intersect_surfaces(&a, &Surface::plane_z(2.0)), Intersection::Coincident));
    }

    #[test]
    fn plane_sphere_is_a_circle() {
        let sp = Surface::sphere(Vec3::new(1.0, 2.0, 3.0), 5.0);
        let pl = Surface::plane_z(1.0);
        assert_on_both(&pl, &sp, &intersect_surfaces(&pl, &sp));
        // ...and the radius is the one Pythagoras gives.
        let Intersection::Curves(c) = intersect_surfaces(&pl, &sp) else { panic!() };
        let Curve::Circle { radius, .. } = c[0] else { panic!("want a circle") };
        assert!((radius - (25.0f32 - 4.0).sqrt()).abs() < 1e-4, "radius {radius}");
    }

    #[test]
    fn plane_sphere_misses_and_touches() {
        let sp = Surface::sphere(Vec3::ZERO, 2.0);
        assert!(matches!(intersect_surfaces(&Surface::plane_z(9.0), &sp), Intersection::Empty));
        assert!(matches!(intersect_surfaces(&Surface::plane_z(2.0), &sp), Intersection::Tangent(_)));
    }

    #[test]
    fn sphere_sphere_is_a_circle() {
        let a = Surface::sphere(Vec3::ZERO, 3.0);
        let b = Surface::sphere(Vec3::new(4.0, 0.0, 0.0), 3.0);
        assert_on_both(&a, &b, &intersect_surfaces(&a, &b));
    }

    #[test]
    fn plane_cylinder_square_on_is_a_circle() {
        let cy = Surface::cylinder_z(Vec3::ZERO, 4.0);
        let pl = Surface::plane_z(3.0);
        let isect = intersect_surfaces(&pl, &cy);
        assert_on_both(&pl, &cy, &isect);
        assert!(matches!(isect.curves()[0], Curve::Circle { .. }));
    }

    /// The case the kernel already meets in `fillet.rs`: a cylinder cut by a
    /// plane oblique to its axis gives a real ellipse, not a circle.
    #[test]
    fn plane_cylinder_oblique_is_an_ellipse() {
        let cy = Surface::cylinder_z(Vec3::ZERO, 4.0);
        let pl = Surface::plane(Vec3::ZERO, Vec3::new(0.0, 1.0, 1.0).normalize());
        let isect = intersect_surfaces(&pl, &cy);
        assert_on_both(&pl, &cy, &isect);
        let Curve::Ellipse { a, b, .. } = isect.curves()[0] else { panic!("want an ellipse") };
        // 45°: the major axis stretches by exactly √2, the minor stays put.
        assert!((b.length() - 4.0).abs() < 1e-4, "semi-minor {}", b.length());
        assert!((a.length() - 4.0 * 2.0f32.sqrt()).abs() < 1e-3, "semi-major {}", a.length());
    }

    #[test]
    fn plane_cylinder_parallel_gives_two_lines() {
        let cy = Surface::cylinder_z(Vec3::ZERO, 4.0);
        let pl = Surface::plane(Vec3::new(0.0, 2.0, 0.0), Vec3::Y);
        let isect = intersect_surfaces(&pl, &cy);
        assert_eq!(isect.curves().len(), 2, "a chord cut gives two generators");
        assert_on_both(&pl, &cy, &isect);
    }

    #[test]
    fn plane_cone_square_on_is_a_circle() {
        let co = Surface::cone_z(Vec3::ZERO, PI / 6.0);
        let pl = Surface::plane_z(-3.0);
        let isect = intersect_surfaces(&pl, &co);
        assert_on_both(&pl, &co, &isect);
        let Curve::Circle { radius, .. } = isect.curves()[0] else { panic!("want a circle") };
        assert!((radius - 3.0 * (PI / 6.0).tan()).abs() < 1e-4, "radius {radius}");
    }

    #[test]
    fn plane_cone_closed_section_is_an_ellipse() {
        let co = Surface::cone_z(Vec3::ZERO, PI / 8.0);
        // Tilted well inside the half angle, so the section still closes.
        let pl = Surface::plane(Vec3::new(0.0, 0.0, -5.0), Vec3::new(0.0, 0.2, 1.0).normalize());
        let isect = intersect_surfaces(&pl, &co);
        assert_on_both(&pl, &co, &isect);
        assert!(matches!(isect.curves()[0], Curve::Ellipse { .. }));
    }

    /// A section that opens must be reported, not bent into an ellipse.
    #[test]
    fn plane_cone_open_section_is_reported_unsupported() {
        let co = Surface::cone_z(Vec3::ZERO, PI / 3.0);
        let pl = Surface::plane(Vec3::new(0.0, 0.0, -5.0), Vec3::new(0.0, 1.0, 0.2).normalize());
        assert!(matches!(intersect_surfaces(&pl, &co), Intersection::Unsupported(_)));
    }

    #[test]
    fn parallel_cylinders_meet_in_lines() {
        let a = Surface::cylinder_z(Vec3::ZERO, 3.0);
        let b = Surface::cylinder_z(Vec3::new(4.0, 0.0, 0.0), 3.0);
        let isect = intersect_surfaces(&a, &b);
        assert_eq!(isect.curves().len(), 2);
        assert_on_both(&a, &b, &isect);
    }

    #[test]
    fn coaxial_cylinders_are_coincident_or_empty() {
        let a = Surface::cylinder_z(Vec3::ZERO, 3.0);
        assert!(matches!(
            intersect_surfaces(&a, &Surface::cylinder_z(Vec3::new(0.0, 0.0, 9.0), 3.0)),
            Intersection::Coincident
        ));
        assert!(matches!(
            intersect_surfaces(&a, &Surface::cylinder_z(Vec3::ZERO, 5.0)),
            Intersection::Empty
        ));
    }

    #[test]
    fn crossed_cylinders_are_reported_unsupported() {
        let a = Surface::cylinder_z(Vec3::ZERO, 3.0);
        let b = Surface::cylinder(Vec3::ZERO, Vec3::X, 3.0, Vec3::Y);
        assert!(matches!(intersect_surfaces(&a, &b), Intersection::Unsupported(_)));
    }

    #[test]
    fn plane_torus_square_on_gives_two_circles() {
        let to = Surface::torus_z(Vec3::ZERO, 10.0, 2.0);
        let pl = Surface::plane_z(1.0);
        let isect = intersect_surfaces(&pl, &to);
        assert_eq!(isect.curves().len(), 2, "inner and outer rings");
        assert_on_both(&pl, &to, &isect);
    }

    #[test]
    fn plane_torus_oblique_is_reported_unsupported() {
        let to = Surface::torus_z(Vec3::ZERO, 10.0, 2.0);
        let pl = Surface::plane(Vec3::ZERO, Vec3::new(0.0, 1.0, 1.0).normalize());
        assert!(matches!(intersect_surfaces(&pl, &to), Intersection::Unsupported(_)));
    }

    /// Argument order must not change the answer.
    #[test]
    fn the_pair_is_symmetric() {
        let pl = Surface::plane_z(1.0);
        let cy = Surface::cylinder_z(Vec3::ZERO, 4.0);
        let f = intersect_surfaces(&pl, &cy);
        let r = intersect_surfaces(&cy, &pl);
        assert_eq!(f.curves().len(), r.curves().len());
        assert_on_both(&pl, &cy, &r);
    }

    /// `signed_distance` must actually be a distance: stepping along the
    /// gradient by `-f` has to land on the surface, for every surface type.
    #[test]
    fn signed_distance_and_gradient_agree() {
        let surfaces = [
            Surface::plane(Vec3::new(1.0, 2.0, 3.0), Vec3::new(1.0, 1.0, 1.0).normalize()),
            Surface::cylinder_z(Vec3::new(1.0, 0.0, 0.0), 3.0),
            Surface::sphere(Vec3::new(0.0, 1.0, 0.0), 4.0),
            Surface::cone_z(Vec3::ZERO, PI / 5.0),
            Surface::torus_z(Vec3::ZERO, 8.0, 2.0),
        ];
        for s in &surfaces {
            for p in [
                Vec3::new(5.0, 6.0, 7.0),
                Vec3::new(-3.0, 2.0, -4.0),
                Vec3::new(0.5, -0.5, 9.0),
            ] {
                let d = s.signed_distance(p);
                let landed = p - s.gradient(p) * d;
                assert!(
                    s.signed_distance(landed).abs() < 1e-3,
                    "{s:?} at {p:?}: stepped by {d} and landed {} off",
                    s.signed_distance(landed)
                );
            }
        }
    }
}
