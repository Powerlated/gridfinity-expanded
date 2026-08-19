//! The kernel's analytic surfaces and curves, and the frame arithmetic they
//! share.
//!
//! Every surface and curve here is exact: a closed-form `point`, `normal` and
//! `project` over its own parameters, with no sampling and no approximation
//! anywhere below the tessellator. The set is deliberately the set a Parasolid
//! transmit file can name -- plane, cylinder, cone, sphere, torus; line, circle,
//! ellipse -- plus `TorusSection`, the one curve that has no analytic node and
//! is written as an intersection instead.
//!
//! **Directions are `Dir`, not `Vec3`**: unit in `f64`, established once at
//! construction. A rotational surface additionally stores its `ref_dir` already
//! perpendicular to its axis, which is what makes `radial_frame` a read rather
//! than a solve and what lets a writer emit the reference direction verbatim.

use crate::kernel::math::{Dir, Vec3};
use std::f32::consts::PI;

pub type Uv = (f32, f32);

/// The unit direction perpendicular to `axis` that lies nearest `hint`, or an
/// arbitrary perpendicular where `hint` is parallel to the axis and names none.
pub fn perp_unit(axis: Dir, hint: Vec3) -> Dir {
    let a = axis.vec();
    let d = hint - a * hint.dot(a);
    if d.length_squared() < 1e-12 {
        let fallback = if a.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
        Dir::new(fallback - a * fallback.dot(a)).perp_to(axis)
    } else {
        Dir::new(d).perp_to(axis)
    }
}

/// The two radial basis vectors of a rotational surface or curve about `axis`,
/// with `u = 0` along `ref_dir`.
///
/// `ref_dir` is already perpendicular to `axis` -- every constructor here stores
/// it that way -- so this is a read and a cross product, not an
/// orthogonalisation. The pair is returned in `f32` because it feeds the
/// kernel's own arithmetic; a caller needing the exact reference direction takes
/// it off the surface.
pub fn radial_frame(axis: Dir, ref_dir: Dir) -> (Vec3, Vec3) {
    (ref_dir.vec(), axis.cross_exact(ref_dir).vec())
}

#[derive(Clone, Copy, Debug)]
pub enum Surface {
    Plane {
        origin: Vec3,
        normal: Dir,
        x_axis: Dir,
    },
    Cylinder {
        base: Vec3,
        axis: Dir,
        radius: f32,
        ref_dir: Dir,
    },
    Cone {
        apex: Vec3,
        axis: Dir,
        half_angle: f32,
        ref_dir: Dir,
    },
    Torus {
        center: Vec3,
        axis: Dir,
        major_r: f32,
        minor_r: f32,
        ref_dir: Dir,
    },
    Sphere {
        center: Vec3,
        axis: Dir,
        radius: f32,
        ref_dir: Dir,
    },
}

impl Surface {
    /// The plane through `origin` with the given normal, taking an arbitrary
    /// reference direction in it.
    pub fn plane(origin: Vec3, normal: Vec3) -> Surface {
        let normal = Dir::new(normal);
        let hint = if normal.vec().z.abs() < 0.9 { Vec3::Z } else { Vec3::X };
        Surface::Plane {
            origin,
            normal,
            x_axis: perp_unit(normal, hint.cross(normal.vec())),
        }
    }

    /// The plane through `origin` whose in-plane `u` runs along `x_hint`,
    /// for a caller whose 2D work is stated in a frame it chose.
    pub fn plane_with_x(origin: Vec3, normal: Vec3, x_hint: Vec3) -> Surface {
        let normal = Dir::new(normal);
        Surface::Plane {
            origin,
            normal,
            x_axis: perp_unit(normal, x_hint),
        }
    }

    pub fn plane_z(z: f32) -> Surface {
        Surface::Plane {
            origin: Vec3::new(0.0, 0.0, z),
            normal: Dir::from_f64([0.0, 0.0, 1.0]),
            x_axis: Dir::from_f64([1.0, 0.0, 0.0]),
        }
    }

    /// The in-plane basis a plane's `(u, v)` are measured along, `v` being the
    /// normal crossed into `u`. Not stored: one cross product of two exact
    /// directions, so keeping it would be a second copy of the same fact.
    pub fn plane_axes(&self) -> (Vec3, Vec3) {
        let Surface::Plane { normal, x_axis, .. } = *self else {
            panic!("only a plane has an in-plane basis, got {self:?}");
        };
        (x_axis.vec(), normal.cross_exact(x_axis).vec())
    }

    pub fn cylinder_z(base: Vec3, radius: f32) -> Surface {
        Surface::cylinder(base, Vec3::Z, radius, Vec3::X)
    }

    pub fn cylinder(base: Vec3, axis: Vec3, radius: f32, ref_dir: Vec3) -> Surface {
        let axis = Dir::new(axis);
        Surface::Cylinder {
            base,
            axis,
            radius,
            ref_dir: perp_unit(axis, ref_dir),
        }
    }

    pub fn cone_z(apex: Vec3, half_angle: f32) -> Surface {
        Surface::cone(apex, Vec3::Z, half_angle, Vec3::X)
    }

    pub fn cone(apex: Vec3, axis: Vec3, half_angle: f32, ref_dir: Vec3) -> Surface {
        let axis = Dir::new(axis);
        Surface::Cone {
            apex,
            axis,
            half_angle,
            ref_dir: perp_unit(axis, ref_dir),
        }
    }

    pub fn torus_z(center: Vec3, major_r: f32, minor_r: f32) -> Surface {
        Surface::torus(center, Vec3::Z, major_r, minor_r, Vec3::X)
    }

    pub fn torus(center: Vec3, axis: Vec3, major_r: f32, minor_r: f32, ref_dir: Vec3) -> Surface {
        let axis = Dir::new(axis);
        Surface::Torus {
            center,
            axis,
            major_r,
            minor_r,
            ref_dir: perp_unit(axis, ref_dir),
        }
    }

    pub fn sphere(center: Vec3, radius: f32) -> Surface {
        Surface::Sphere {
            center,
            axis: Dir::from_f64([0.0, 0.0, 1.0]),
            radius,
            ref_dir: Dir::from_f64([1.0, 0.0, 0.0]),
        }
    }

    #[inline]
    fn radial_at(u: f32, d0: Vec3, d1: Vec3) -> Vec3 {
        u.cos() * d0 + u.sin() * d1
    }

    fn point_r(&self, radial: Vec3, uv: Uv, basis: (Vec3, Vec3)) -> Vec3 {
        let (u, v) = uv;
        match *self {
            Surface::Plane { origin, .. } => origin + u * basis.0 + v * basis.1,
            Surface::Cylinder {
                base, axis, radius, ..
            } => base + radius * radial + v * axis.vec(),
            Surface::Cone {
                apex,
                axis,
                half_angle,
                ..
            } => {
                let r = v.abs() * half_angle.tan();
                apex + v * axis.vec() + r * radial
            }
            Surface::Torus {
                center,
                axis,
                major_r,
                minor_r,
                ..
            } => {
                center + (major_r + minor_r * v.cos()) * radial + minor_r * v.sin() * axis.vec()
            }
            Surface::Sphere {
                center,
                axis,
                radius,
                ..
            } => center + radius * (v.sin() * radial + v.cos() * axis.vec()),
        }
    }

    fn normal_r(&self, radial: Vec3, v: f32) -> Vec3 {
        match *self {
            Surface::Plane { normal, .. } => normal.vec(),
            Surface::Cylinder { .. } => radial.normalize(),
            Surface::Cone {
                axis, half_angle, ..
            } => {
                let sgn = if v >= 0.0 { -1.0 } else { 1.0 };
                (half_angle.cos() * radial + sgn * half_angle.sin() * axis.vec()).normalize()
            }
            Surface::Torus { axis, .. } => {
                (v.cos() * radial + v.sin() * axis.vec()).normalize()
            }
            Surface::Sphere { axis, .. } => {
                (v.sin() * radial + v.cos() * axis.vec()).normalize()
            }
        }
    }

    fn normal_ignores_v(&self, v0: f32, v1: f32) -> bool {
        match *self {
            Surface::Plane { .. } | Surface::Cylinder { .. } => true,
            Surface::Cone { .. } => (v0 >= 0.0) == (v1 >= 0.0),
            Surface::Torus { .. } | Surface::Sphere { .. } => false,
        }
    }

    fn point_f(&self, uv: Uv, d0: Vec3, d1: Vec3) -> Vec3 {
        self.point_r(Surface::radial_at(uv.0, d0, d1), uv, (d0, d1))
    }

    fn normal_f(&self, uv: Uv, d0: Vec3, d1: Vec3) -> Vec3 {
        self.normal_r(Surface::radial_at(uv.0, d0, d1), uv.1)
    }

    pub fn signed_distance(&self, p: Vec3) -> f32 {
        match *self {
            Surface::Plane { origin, normal, .. } => (p - origin).dot(normal.vec()),
            Surface::Cylinder {
                base, axis, radius, ..
            } => {
                let (d, axis) = (p - base, axis.vec());
                (d - axis * d.dot(axis)).length() - radius
            }
            Surface::Sphere { center, radius, .. } => (p - center).length() - radius,
            Surface::Cone {
                apex,
                axis,
                half_angle,
                ..
            } => {
                let (d, axis) = (p - apex, axis.vec());
                let along = d.dot(axis);
                let perp = (d - axis * along).length();
                perp * half_angle.cos() - along.abs() * half_angle.sin()
            }
            Surface::Torus {
                center,
                axis,
                major_r,
                minor_r,
                ..
            } => {
                let (d, axis) = (p - center, axis.vec());
                let along = d.dot(axis);
                let perp = (d - axis * along).length();
                let near = ((perp - major_r).powi(2) + along * along).sqrt() - minor_r;
                if major_r >= minor_r {
                    return near;
                }
                let far = ((perp + major_r).powi(2) + along * along).sqrt() - minor_r;
                if near.abs() <= far.abs() { near } else { far }
            }
        }
    }

    pub fn gradient(&self, p: Vec3) -> Vec3 {
        match *self {
            Surface::Plane { normal, .. } => normal.vec(),
            Surface::Cylinder { base, axis, .. } => {
                let (d, axis) = (p - base, axis.vec());
                (d - axis * d.dot(axis)).normalize_or(axis)
            }
            Surface::Sphere { center, .. } => (p - center).normalize_or(Vec3::Z),
            Surface::Cone {
                apex,
                axis,
                half_angle,
                ..
            } => {
                let (d, axis) = (p - apex, axis.vec());
                let along = d.dot(axis);
                let radial = (d - axis * along).normalize_or(Vec3::ZERO);
                if radial == Vec3::ZERO {
                    return axis;
                }
                let side = if along < 0.0 { -1.0 } else { 1.0 };
                (radial * half_angle.cos() - axis * side * half_angle.sin()).normalize()
            }
            Surface::Torus {
                center,
                axis,
                major_r,
                ..
            } => {
                let (d, axis) = (p - center, axis.vec());
                let along = d.dot(axis);
                let radial = (d - axis * along).normalize_or(Vec3::ZERO);
                if radial == Vec3::ZERO {
                    return axis;
                }
                let spine = center + radial * major_r;
                (p - spine).normalize_or(radial)
            }
        }
    }

    pub fn uv_orientation(&self) -> f32 {
        match *self {
            Surface::Plane { .. }
            | Surface::Cylinder { .. }
            | Surface::Torus { .. }
            | Surface::Cone { .. } => 1.0,
            Surface::Sphere { .. } => -1.0,
        }
    }

    fn project_f(&self, p: Vec3, d0: Vec3, d1: Vec3) -> Uv {
        match *self {
            Surface::Plane { origin, .. } => {
                let d = p - origin;
                (d.dot(d0), d.dot(d1))
            }
            Surface::Cylinder {
                base,
                axis,
                ref_dir: _,
                ..
            } => {
                let rel = p - base;
                let u = wrap_angle(rel.dot(d1).atan2(rel.dot(d0)));
                (u, rel.dot(axis.vec()))
            }
            Surface::Cone {
                apex,
                axis,
                ref_dir: _,
                ..
            } => {
                let rel = p - apex;
                let u = wrap_angle(rel.dot(d1).atan2(rel.dot(d0)));
                (u, rel.dot(axis.vec()))
            }
            Surface::Torus {
                center,
                axis,
                major_r,
                ref_dir: _,
                ..
            } => {
                let rel = p - center;
                let u = wrap_angle(rel.dot(d1).atan2(rel.dot(d0)));
                let radial = u.cos() * d0 + u.sin() * d1;
                let ring = rel.dot(radial) - major_r;
                let v = wrap_angle(rel.dot(axis.vec()).atan2(ring));
                (u, v)
            }
            Surface::Sphere {
                center,
                axis,
                radius,
                ref_dir: _,
            } => {
                let rel = (p - center) / radius;
                let v = rel.dot(axis.vec()).clamp(-1.0, 1.0).acos();
                let u = wrap_angle(rel.dot(d1).atan2(rel.dot(d0)));
                (u, v)
            }
        }
    }
}

impl Surface {
    pub fn frame(&self) -> (Vec3, Vec3) {
        match *self {
            Surface::Plane { .. } => self.plane_axes(),
            Surface::Cylinder { axis, ref_dir, .. }
            | Surface::Cone { axis, ref_dir, .. }
            | Surface::Torus { axis, ref_dir, .. }
            | Surface::Sphere { axis, ref_dir, .. } => radial_frame(axis, ref_dir),
        }
    }

    pub fn prepare(&self) -> Prepared {
        let (d0, d1) = self.frame();
        Prepared {
            surface: *self,
            d0,
            d1,
        }
    }

    pub fn point(&self, uv: Uv) -> Vec3 {
        let (d0, d1) = self.frame();
        self.point_f(uv, d0, d1)
    }

    pub fn normal(&self, uv: Uv) -> Vec3 {
        let (d0, d1) = self.frame();
        self.normal_f(uv, d0, d1)
    }

    pub fn project(&self, p: Vec3) -> Uv {
        let (d0, d1) = self.frame();
        self.project_f(p, d0, d1)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Prepared {
    pub surface: Surface,
    d0: Vec3,
    d1: Vec3,
}

impl Prepared {
    #[inline]
    pub fn point(&self, uv: Uv) -> Vec3 {
        self.surface.point_f(uv, self.d0, self.d1)
    }
    #[inline]
    pub fn normal(&self, uv: Uv) -> Vec3 {
        self.surface.normal_f(uv, self.d0, self.d1)
    }
    #[inline]
    pub fn project(&self, p: Vec3) -> Uv {
        self.surface.project_f(p, self.d0, self.d1)
    }

    #[inline]
    pub fn radial(&self, u: f32) -> Vec3 {
        Surface::radial_at(u, self.d0, self.d1)
    }

    #[inline]
    pub fn point_at(&self, radial: Vec3, uv: Uv) -> Vec3 {
        self.surface.point_r(radial, uv, (self.d0, self.d1))
    }

    #[inline]
    pub fn normal_at(&self, radial: Vec3, v: f32) -> Vec3 {
        self.surface.normal_r(radial, v)
    }

    #[inline]
    pub fn normal_ignores_v(&self, v0: f32, v1: f32) -> bool {
        self.surface.normal_ignores_v(v0, v1)
    }
}

fn wrap_angle(a: f32) -> f32 {
    let mut a = a % (2.0 * PI);
    if a < 0.0 {
        a += 2.0 * PI;
    }
    a
}

#[derive(Clone, Copy, Debug)]
pub enum Curve {
    Line {
        p0: Vec3,
        dir: Dir,
    },
    Circle {
        center: Vec3,
        axis: Dir,
        radius: f32,
        ref_dir: Dir,
    },
    Ellipse {
        center: Vec3,
        a: Vec3,
        b: Vec3,
    },
    TorusSection {
        center: Vec3,
        axis: Dir,
        ref_dir: Dir,
        major: f32,
        minor: f32,
        offset: f32,
        branch: f32,
    },
}

impl Curve {
    pub fn line(a: Vec3, b: Vec3) -> Curve {
        assert!(
            a != b,
            "a line curve runs between two distinct points, but both ends are {a:?}"
        );
        Curve::Line {
            p0: a,
            dir: Dir::new(b - a),
        }
    }

    pub fn circle_z(center: Vec3, radius: f32) -> Curve {
        Curve::circle(center, Vec3::Z, radius, Vec3::X)
    }

    /// The circle of `radius` about `center` in the plane through it normal to
    /// `axis`, with its parameter measured from the projection of `ref_dir`.
    pub fn circle(center: Vec3, axis: Vec3, radius: f32, ref_dir: Vec3) -> Curve {
        let axis = Dir::new(axis);
        Curve::Circle {
            center,
            axis,
            radius,
            ref_dir: perp_unit(axis, ref_dir),
        }
    }

    pub fn point(&self, t: f32) -> Vec3 {
        match *self {
            Curve::Line { p0, dir } => p0 + t * dir.vec(),
            Curve::Circle {
                center,
                axis,
                radius,
                ref_dir,
            } => {
                let (d0, d1) = radial_frame(axis, ref_dir);
                center + radius * (t.cos() * d0 + t.sin() * d1)
            }
            Curve::Ellipse { center, a, b } => center + t.cos() * a + t.sin() * b,
            Curve::TorusSection {
                center,
                axis,
                ref_dir,
                major,
                minor,
                offset,
                branch,
            } => {
                let (d0, d1) = radial_frame(axis, ref_dir);
                let rad = major + minor * t.cos();
                let cos_u = (offset / rad).clamp(-1.0, 1.0);
                let sin_u = branch * (1.0 - cos_u * cos_u).max(0.0).sqrt();
                center + rad * (cos_u * d0 + sin_u * d1) + minor * t.sin() * axis.vec()
            }
        }
    }

    /// d(point)/dt, in closed form. Not normalised -- callers wanting a
    /// direction normalise it themselves, and a zero-length result means the
    /// parameterisation is stationary there.
    pub fn tangent(&self, t: f32) -> Vec3 {
        match *self {
            Curve::Line { dir, .. } => dir.vec(),
            Curve::Circle {
                axis,
                radius,
                ref_dir,
                ..
            } => {
                let (d0, d1) = radial_frame(axis, ref_dir);
                radius * (-t.sin() * d0 + t.cos() * d1)
            }
            Curve::Ellipse { a, b, .. } => -t.sin() * a + t.cos() * b,
            Curve::TorusSection {
                axis,
                ref_dir,
                major,
                minor,
                offset,
                branch,
                ..
            } => {
                let (d0, d1) = radial_frame(axis, ref_dir);
                let rad = major + minor * t.cos();
                let d_rad = -minor * t.sin();
                let cos_u = (offset / rad).clamp(-1.0, 1.0);
                let su = (1.0 - cos_u * cos_u).max(0.0).sqrt();
                let sin_u = branch * su;
                // cos_u = offset / rad, so d(cos_u)/dt follows the quotient
                // rule; su falls to zero where the section turns back on itself
                // and the radial term drops out with it.
                let d_cos_u = -offset * d_rad / (rad * rad);
                let d_sin_u = if su > 1e-6 {
                    branch * (-cos_u * d_cos_u / su)
                } else {
                    0.0
                };
                d_rad * (cos_u * d0 + sin_u * d1)
                    + rad * (d_cos_u * d0 + d_sin_u * d1)
                    + minor * t.cos() * axis.vec()
            }
        }
    }

    pub fn torus_section(
        center: Vec3,
        axis: Vec3,
        plane_normal: Vec3,
        plane_offset: f32,
        major: f32,
        minor: f32,
        branch: f32,
    ) -> Curve {
        let axis = Dir::new(axis);
        Curve::TorusSection {
            center,
            axis,
            ref_dir: Dir::new(plane_normal).perp_to(axis),
            major,
            minor,
            offset: plane_offset,
            branch: branch.signum(),
        }
    }

    pub fn torus_section_exists(major: f32, minor: f32, offset: f32, t: f32) -> bool {
        let rad = major + minor * t.cos();
        rad > 0.0 && offset.abs() <= rad
    }
}
