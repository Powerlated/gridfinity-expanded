//! Closed-form intersection of two analytic surfaces, in world millimetres.
//!
//! `intersect_surfaces` is the whole surface: it dispatches an unordered pair to
//! the one routine that solves it and answers with exact `Curve`s, never with a
//! polyline or a numerical trace -- the kernel's rule that a curve is solved or
//! not produced at all. Everything else here is one such routine, plus
//! `cone_generator_hit`, which the cone's ellipse solve needs to find the two
//! ends of its section along the plane's steepest direction.
//!
//! What a pair yields is decided by which of the results the geometry actually
//! is, not by which one is convenient: a pair that meets in curves the set
//! cannot name -- a torus against anything curved, two crossed cylinders, a
//! plane cutting a cone open into a parabola or hyperbola -- is
//! `Unsupported(why)` and stays the caller's problem. Refusing is what keeps a
//! quartic from being approximated by something that merely looks like it. The
//! solvable degeneracies are not refusals: two surfaces on top of one another
//! are `Coincident`, two that miss are `Empty`, and two that touch at a point
//! are `Tangent(p)`, all three distinguished at `TOL`.

use crate::kernel::geom::{Curve, Surface, perp_unit};
use crate::kernel::math::Vec3;

/// How near two quantities in world millimetres must be to count as equal
/// here -- a plane's distance from a sphere's surface, a cross product's length
/// against zero, a cosine against 1. 1e-5 mm is four orders below the smallest
/// feature the model builds and two orders above f32 noise at bin scale, so the
/// degenerate cases it names are the geometric ones and not float slop.
pub const TOL: f32 = 1e-5;

/// What two surfaces meet in.
///
/// `Empty` and `Coincident` are the two ends of the degenerate range -- no
/// common point at all, and every point in common. `Tangent` is a single point
/// of contact, carried explicitly because it is not a curve of zero length. The
/// intersection proper is `Curves`, each an exact analytic curve lying on both
/// surfaces. `Unsupported` carries why no closed-form answer was produced, and
/// is a statement about the curve set rather than about the surfaces: the
/// intersection exists, this module cannot name it.
#[derive(Clone, Debug)]
pub enum Intersection {
    Empty,
    Coincident,
    Curves(Vec<Curve>),
    Tangent(Vec3),
    Unsupported(&'static str),
}

impl Intersection {
    /// The curves of a `Curves` result, and an empty slice for every other
    /// variant -- for a caller that wants the curves and treats "there are
    /// none" and "there are none it can have" alike.
    pub fn curves(&self) -> &[Curve] {
        match self {
            Intersection::Curves(c) => c,
            _ => &[],
        }
    }
}

/// What `a` and `b` meet in, as exact curves on both of them.
///
/// Both surfaces are unbounded analytic surfaces in world millimetres -- the
/// answer is the full intersection of the surfaces, not of any faces trimmed out
/// of them, so a caller holding faces must still clip what comes back to its own
/// parameter ranges. Symmetric in its arguments up to the order and direction of
/// the curves returned: each pair is dispatched to a single routine with the
/// plane first, so `intersect_surfaces(a, b)` and `(b, a)` describe the same
/// point set. A pair whose intersection falls outside `Curve` -- degree 8 for a
/// torus against a curved surface, quartic for crossed cylinders -- is
/// `Unsupported`, never approximated.
pub fn intersect_surfaces(a: &Surface, b: &Surface) -> Intersection {
    let _perf = crate::kernel::perf::scope(crate::kernel::perf::Metric::IntersectSurfaces);
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

/// A plane's `(origin, unit normal)`. Panics on any other surface, so a caller
/// reaching for it has already established by dispatch that `s` is a `Plane`.
fn plane_parts(s: &Surface) -> (Vec3, Vec3) {
    match *s {
        Surface::Plane { origin, normal, .. } => (origin, normal),
        _ => unreachable!("plane expected"),
    }
}

/// Two planes, meeting in the one `Curve::Line` along both of them.
///
/// Direction is `n1 x n2`, normalized. The line's `p0` is the point of it
/// nearest the origin, solved from the two plane equations in the basis the
/// normals span, which is well conditioned exactly when the normals are not
/// parallel. Parallel normals give no line: same plane within `TOL` is
/// `Coincident`, otherwise `Empty`.
fn plane_plane(a: &Surface, b: &Surface) -> Intersection {
    let (o1, n1) = plane_parts(a);
    let (o2, n2) = plane_parts(b);
    let dir = n1.cross(n2);
    if dir.length() < TOL {
        return if (o2 - o1).dot(n1).abs() < TOL {
            Intersection::Coincident
        } else {
            Intersection::Empty
        };
    }
    let (d1, d2) = (o1.dot(n1), o2.dot(n2));
    let c = n1.dot(n2);
    let denom = 1.0 - c * c;
    let p0 = n1 * ((d1 - d2 * c) / denom) + n2 * ((d2 - d1 * c) / denom);
    Intersection::Curves(vec![Curve::Line {
        p0,
        dir: dir.normalize(),
    }])
}

/// A plane against a sphere: the one `Curve::Circle` cut out of the sphere.
///
/// The circle is centred on the sphere's centre projected onto the plane, has
/// the plane's normal as its axis, and radius `sqrt(r^2 - d^2)` for `d` the
/// signed distance from centre to plane. A plane grazing the sphere within `TOL`
/// is `Tangent` at that projected point rather than a zero-radius circle;
/// further off than the radius is `Empty`.
fn plane_sphere(pl: &Surface, sp: &Surface) -> Intersection {
    let (_, n) = plane_parts(pl);
    let Surface::Sphere { center, radius, .. } = *sp else {
        unreachable!()
    };
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

/// Two spheres, meeting in the one `Curve::Circle` on their radical plane.
///
/// The circle's axis is the unit vector from the first centre to the second and
/// its centre sits `(d^2 + r1^2 - r2^2) / 2d` along that axis, which is where
/// the two sphere equations agree. Concentric within `TOL` is `Coincident` when
/// the radii match and `Empty` when they do not; separated further than
/// `r1 + r2` or nested closer than `|r1 - r2|` is `Empty`; touching at one point
/// is `Tangent` there.
fn sphere_sphere(a: &Surface, b: &Surface) -> Intersection {
    let Surface::Sphere {
        center: c1,
        radius: r1,
        ..
    } = *a
    else {
        unreachable!()
    };
    let Surface::Sphere {
        center: c2,
        radius: r2,
        ..
    } = *b
    else {
        unreachable!()
    };
    let delta = c2 - c1;
    let d = delta.length();
    if d < TOL {
        return if (r1 - r2).abs() < TOL {
            Intersection::Coincident
        } else {
            Intersection::Empty
        };
    }
    if d > r1 + r2 + TOL || d < (r1 - r2).abs() - TOL {
        return Intersection::Empty;
    }
    let axis = delta / d;
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

/// A plane against a cylinder: one to two curves, whichever conic section the
/// angle between plane normal and cylinder axis makes it.
///
/// A plane parallel to the axis (`|axis . n| < TOL`) cuts generators: two
/// `Curve::Line`s a half-chord either side of the axis' projection, one when it
/// is tangent within `TOL`, none when it clears the radius. A plane square on
/// (`|axis . n| > 1 - TOL`) cuts a `Curve::Circle` of the cylinder's own radius.
/// Every angle between gives one `Curve::Ellipse` centred where the axis crosses
/// the plane, semi-minor `radius` across the tilt and semi-major
/// `radius / |axis . n|` along it.
fn plane_cylinder(pl: &Surface, cy: &Surface) -> Intersection {
    let (_, n) = plane_parts(pl);
    let Surface::Cylinder {
        base, axis, radius, ..
    } = *cy
    else {
        unreachable!()
    };
    let cos_t = axis.dot(n);

    if cos_t.abs() < TOL {
        let d = pl.signed_distance(base);
        if d.abs() > radius + TOL {
            return Intersection::Empty;
        }
        let m = axis.cross(n).normalize();
        let foot = base - n * d;
        if (d.abs() - radius).abs() < TOL {
            return Intersection::Curves(vec![Curve::Line {
                p0: foot,
                dir: axis,
            }]);
        }
        let h = (radius * radius - d * d).max(0.0).sqrt();
        return Intersection::Curves(vec![
            Curve::Line {
                p0: foot + m * h,
                dir: axis,
            },
            Curve::Line {
                p0: foot - m * h,
                dir: axis,
            },
        ]);
    }

    let t = -pl.signed_distance(base) / cos_t;
    let center = base + axis * t;
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

/// A plane against a cone: the section, but only where the section is closed.
///
/// A plane square on the axis gives a `Curve::Circle` of radius
/// `|h| tan(half_angle)` at the height it crosses, or `Tangent(apex)` when it
/// passes through the apex. A plane tilted less than the half angle from square
/// on gives a `Curve::Ellipse`, spanned between the two points where the section
/// crosses the cone's steepest generators and with the semi-minor axis solved
/// from the cone's radius at the ellipse's own centre. Tilt the plane past that
/// and the section opens into a parabola or hyperbola, which `Curve` has no
/// unbounded conic for: `Unsupported`, as is a section whose ends do not both
/// come back finite or whose semi-minor collapses.
fn plane_cone(pl: &Surface, co: &Surface) -> Intersection {
    let (_, n) = plane_parts(pl);
    let Surface::Cone {
        apex,
        axis,
        half_angle,
        ..
    } = *co
    else {
        unreachable!()
    };
    let cos_t = axis.dot(n).abs();
    let sin_ha = half_angle.sin();

    if cos_t > 1.0 - TOL {
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

    let minor_dir = axis.cross(n).normalize();
    let tilt = n.cross(minor_dir).normalize();
    let Some(p1) = cone_generator_hit(apex, axis, half_angle, pl, tilt) else {
        return Intersection::Unsupported("cone section endpoints are not both finite");
    };
    let Some(p2) = cone_generator_hit(apex, axis, half_angle, pl, -tilt) else {
        return Intersection::Unsupported("cone section endpoints are not both finite");
    };
    let center = (p1 + p2) * 0.5;
    let semi_major = (p2 - p1).length() * 0.5;
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

/// Where the cone's generator leaning towards `dir` meets the plane, or `None`
/// when neither nappe's generator crosses it ahead of the apex.
///
/// `dir` need only have a component across the axis -- the generator is built in
/// the plane of `axis` and `perp_unit(axis, dir)`, which is the direction the
/// section reaches furthest in, so the two calls at `+/-tilt` bracket an
/// elliptical section's major axis. Both nappes are tried and the first
/// crossing at parameter `> TOL` wins, so the point returned always lies on the
/// cone ahead of the apex rather than behind it on the mirrored nappe.
fn cone_generator_hit(
    apex: Vec3,
    axis: Vec3,
    half_angle: f32,
    pl: &Surface,
    dir: Vec3,
) -> Option<Vec3> {
    let (_, n) = plane_parts(pl);
    let radial = perp_unit(axis, dir);
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

/// Two cylinders, and only while their axes are parallel.
///
/// Parallel axes reduce the problem to two circles in the plane across them, so
/// the answer is generators: two `Curve::Line`s along the shared axis direction,
/// through the two points where those circles cross, or one where they are
/// tangent. Coaxial within `TOL` is `Coincident` at equal radii and `Empty`
/// otherwise, as is a pair too far apart or too deeply nested to cross.
/// Non-parallel axes meet in a quartic space curve outside `Curve`:
/// `Unsupported`.
fn cylinder_cylinder(a: &Surface, b: &Surface) -> Intersection {
    let Surface::Cylinder {
        base: b1,
        axis: a1,
        radius: r1,
        ..
    } = *a
    else {
        unreachable!()
    };
    let Surface::Cylinder {
        base: b2,
        axis: a2,
        radius: r2,
        ..
    } = *b
    else {
        unreachable!()
    };
    if a1.cross(a2).length() > TOL {
        return Intersection::Unsupported(
            "non-parallel cylinder/cylinder is a quartic space curve",
        );
    }
    let d = b2 - b1;
    let off = d - a1 * d.dot(a1);
    if off.length() < TOL {
        return if (r1 - r2).abs() < TOL {
            Intersection::Coincident
        } else {
            Intersection::Empty
        };
    }
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
        Curve::Line {
            p0: foot + v * h,
            dir: a1,
        },
        Curve::Line {
            p0: foot - v * h,
            dir: a1,
        },
    ])
}

/// A plane against a torus, in either argument order, and only at the two angles
/// whose section `Curve` can name.
///
/// A plane parallel to the torus axis cuts the two `Curve::TorusSection`
/// branches either side of the plane's normal -- the spiric curve, exact because
/// fixing the minor angle fixes a ring radius and the plane meets that ring in
/// closed form -- or `Empty` when it clears `major_r + minor_r`. A plane square
/// on the axis cuts circles concentric with the torus: two of radius
/// `major_r +/- sqrt(minor_r^2 - h^2)` for `h` the plane's height above the
/// torus centre, one of `major_r` where the plane grazes the tube's silhouette,
/// and `Empty` past `minor_r`. Any angle between is a quartic Cassinian curve:
/// `Unsupported`.
fn plane_torus(a: &Surface, b: &Surface) -> Intersection {
    let (pl, to) = match a {
        Surface::Plane { .. } => (a, b),
        _ => (b, a),
    };
    let (_, n) = plane_parts(pl);
    let Surface::Torus {
        center,
        axis,
        major_r,
        minor_r,
        ..
    } = *to
    else {
        unreachable!()
    };
    let cos_t = axis.dot(n).abs();
    if cos_t < TOL {
        let offset = -pl.signed_distance(center);
        if offset.abs() > major_r + minor_r + TOL {
            return Intersection::Empty;
        }
        return Intersection::Curves(
            [1.0f32, -1.0]
                .iter()
                .map(|&branch| {
                    Curve::torus_section(center, axis, n, offset, major_r, minor_r, branch)
                })
                .collect(),
        );
    }
    if cos_t < 1.0 - TOL {
        return Intersection::Unsupported(
            "plane/torus at a general angle is a quartic (Cassinian) curve",
        );
    }
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
        Curve::Circle {
            center: plane_center,
            axis,
            radius: major_r + dr,
            ref_dir,
        },
        Curve::Circle {
            center: plane_center,
            axis,
            radius: major_r - dr,
            ref_dir,
        },
    ])
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::PI;

    fn assert_on_both(a: &Surface, b: &Surface, isect: &Intersection) {
        let curves = isect.curves();
        assert!(!curves.is_empty(), "expected curves, got {isect:?}");
        for c in curves {
            for i in 0..64 {
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
        assert!(matches!(
            intersect_surfaces(&a, &Surface::plane_z(5.0)),
            Intersection::Empty
        ));
        assert!(matches!(
            intersect_surfaces(&a, &Surface::plane_z(2.0)),
            Intersection::Coincident
        ));
    }

    #[test]
    fn plane_sphere_is_a_circle() {
        let sp = Surface::sphere(Vec3::new(1.0, 2.0, 3.0), 5.0);
        let pl = Surface::plane_z(1.0);
        assert_on_both(&pl, &sp, &intersect_surfaces(&pl, &sp));
        let Intersection::Curves(c) = intersect_surfaces(&pl, &sp) else {
            panic!()
        };
        let Curve::Circle { radius, .. } = c[0] else {
            panic!("want a circle")
        };
        assert!(
            (radius - (25.0f32 - 4.0).sqrt()).abs() < 1e-4,
            "radius {radius}"
        );
    }

    #[test]
    fn plane_sphere_misses_and_touches() {
        let sp = Surface::sphere(Vec3::ZERO, 2.0);
        assert!(matches!(
            intersect_surfaces(&Surface::plane_z(9.0), &sp),
            Intersection::Empty
        ));
        assert!(matches!(
            intersect_surfaces(&Surface::plane_z(2.0), &sp),
            Intersection::Tangent(_)
        ));
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

    #[test]
    fn plane_cylinder_oblique_is_an_ellipse() {
        let cy = Surface::cylinder_z(Vec3::ZERO, 4.0);
        let pl = Surface::plane(Vec3::ZERO, Vec3::new(0.0, 1.0, 1.0).normalize());
        let isect = intersect_surfaces(&pl, &cy);
        assert_on_both(&pl, &cy, &isect);
        let Curve::Ellipse { a, b, .. } = isect.curves()[0] else {
            panic!("want an ellipse")
        };
        assert!((b.length() - 4.0).abs() < 1e-4, "semi-minor {}", b.length());
        assert!(
            (a.length() - 4.0 * 2.0f32.sqrt()).abs() < 1e-3,
            "semi-major {}",
            a.length()
        );
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
        let Curve::Circle { radius, .. } = isect.curves()[0] else {
            panic!("want a circle")
        };
        assert!(
            (radius - 3.0 * (PI / 6.0).tan()).abs() < 1e-4,
            "radius {radius}"
        );
    }

    #[test]
    fn plane_cone_closed_section_is_an_ellipse() {
        let co = Surface::cone_z(Vec3::ZERO, PI / 8.0);
        let pl = Surface::plane(
            Vec3::new(0.0, 0.0, -5.0),
            Vec3::new(0.0, 0.2, 1.0).normalize(),
        );
        let isect = intersect_surfaces(&pl, &co);
        assert_on_both(&pl, &co, &isect);
        assert!(matches!(isect.curves()[0], Curve::Ellipse { .. }));
    }

    #[test]
    fn plane_cone_open_section_is_reported_unsupported() {
        let co = Surface::cone_z(Vec3::ZERO, PI / 3.0);
        let pl = Surface::plane(
            Vec3::new(0.0, 0.0, -5.0),
            Vec3::new(0.0, 1.0, 0.2).normalize(),
        );
        assert!(matches!(
            intersect_surfaces(&pl, &co),
            Intersection::Unsupported(_)
        ));
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
        assert!(matches!(
            intersect_surfaces(&a, &b),
            Intersection::Unsupported(_)
        ));
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
    fn plane_torus_parallel_to_the_axis_gives_two_spiric_branches() {
        let to = Surface::torus_z(Vec3::ZERO, 10.0, 2.0);
        let pl = Surface::plane(Vec3::new(3.0, 0.0, 0.0), Vec3::X);
        let isect = intersect_surfaces(&pl, &to);
        assert_eq!(
            isect.curves().len(),
            2,
            "the two halves either side of the plane normal"
        );
        assert_on_both(&pl, &to, &isect);
    }

    #[test]
    fn a_plane_clear_of_the_torus_meets_it_nowhere() {
        let to = Surface::torus_z(Vec3::ZERO, 10.0, 2.0);
        let pl = Surface::plane(Vec3::new(12.5, 0.0, 0.0), Vec3::X);
        assert!(matches!(intersect_surfaces(&pl, &to), Intersection::Empty));
    }

    #[test]
    fn plane_torus_oblique_is_reported_unsupported() {
        let to = Surface::torus_z(Vec3::ZERO, 10.0, 2.0);
        let pl = Surface::plane(Vec3::ZERO, Vec3::new(0.0, 1.0, 1.0).normalize());
        assert!(matches!(
            intersect_surfaces(&pl, &to),
            Intersection::Unsupported(_)
        ));
    }

    #[test]
    fn the_pair_is_symmetric() {
        let pl = Surface::plane_z(1.0);
        let cy = Surface::cylinder_z(Vec3::ZERO, 4.0);
        let f = intersect_surfaces(&pl, &cy);
        let r = intersect_surfaces(&cy, &pl);
        assert_eq!(f.curves().len(), r.curves().len());
        assert_on_both(&pl, &cy, &r);
    }

    #[test]
    fn signed_distance_and_gradient_agree() {
        let surfaces = [
            Surface::plane(
                Vec3::new(1.0, 2.0, 3.0),
                Vec3::new(1.0, 1.0, 1.0).normalize(),
            ),
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
