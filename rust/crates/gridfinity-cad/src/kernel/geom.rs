
use crate::kernel::math::Vec3;
use std::f32::consts::PI;

pub type Uv = (f32, f32);

pub fn perp_unit(axis: Vec3, hint: Vec3) -> Vec3 {
    let d = hint - axis * hint.dot(axis);
    if d.length_squared() < 1e-12 {
        let a = if axis.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
        (a - axis * a.dot(axis)).normalize()
    } else {
        d.normalize()
    }
}

pub fn radial_frame(axis: Vec3, ref_dir: Vec3) -> (Vec3, Vec3) {
    let d0 = perp_unit(axis, ref_dir);
    let d1 = axis.cross(d0);
    (d0, d1)
}

#[derive(Clone, Copy, Debug)]
pub enum Surface {
    Plane {
        origin: Vec3,
        normal: Vec3,
        u_dir: Vec3,
        v_dir: Vec3,
    },
    Cylinder { base: Vec3, axis: Vec3, radius: f32, ref_dir: Vec3 },
    Cone {
        apex: Vec3,
        axis: Vec3,
        half_angle: f32,
        ref_dir: Vec3,
    },
    Torus {
        center: Vec3,
        axis: Vec3,
        major_r: f32,
        minor_r: f32,
        ref_dir: Vec3,
    },
    Sphere { center: Vec3, axis: Vec3, radius: f32, ref_dir: Vec3 },
}

impl Surface {
    pub fn plane(origin: Vec3, normal: Vec3) -> Surface {
        let normal = normal.normalize();
        let a = if normal.z.abs() < 0.9 { Vec3::Z } else { Vec3::X };
        let u_dir = a.cross(normal).normalize();
        let v_dir = normal.cross(u_dir);
        Surface::Plane {
            origin,
            normal,
            u_dir,
            v_dir,
        }
    }

    pub fn plane_z(z: f32) -> Surface {
        Surface::Plane {
            origin: Vec3::new(0.0, 0.0, z),
            normal: Vec3::Z,
            u_dir: Vec3::X,
            v_dir: Vec3::Y,
        }
    }

    pub fn cylinder_z(base: Vec3, radius: f32) -> Surface {
        Surface::Cylinder { base, axis: Vec3::Z, radius, ref_dir: Vec3::X }
    }

    pub fn cylinder(base: Vec3, axis: Vec3, radius: f32, ref_dir: Vec3) -> Surface {
        Surface::Cylinder { base, axis, radius, ref_dir }
    }

    pub fn cone_z(apex: Vec3, half_angle: f32) -> Surface {
        Surface::Cone { apex, axis: Vec3::Z, half_angle, ref_dir: Vec3::X }
    }

    pub fn torus_z(center: Vec3, major_r: f32, minor_r: f32) -> Surface {
        Surface::Torus { center, axis: Vec3::Z, major_r, minor_r, ref_dir: Vec3::X }
    }

    pub fn sphere(center: Vec3, radius: f32) -> Surface {
        Surface::Sphere { center, axis: Vec3::Z, radius, ref_dir: Vec3::X }
    }

    #[inline]
    fn radial_at(u: f32, d0: Vec3, d1: Vec3) -> Vec3 {
        u.cos() * d0 + u.sin() * d1
    }

    fn point_r(&self, radial: Vec3, uv: Uv) -> Vec3 {
        let (u, v) = uv;
        match *self {
            Surface::Plane {
                origin,
                u_dir,
                v_dir,
                ..
            } => origin + u * u_dir + v * v_dir,
            Surface::Cylinder { base, axis, radius, .. } => base + radius * radial + v * axis,
            Surface::Cone {
                apex,
                axis,
                half_angle,
                ..
            } => {
                let r = v.abs() * half_angle.tan();
                apex + v * axis + r * radial
            }
            Surface::Torus {
                center,
                axis,
                major_r,
                minor_r,
                ..
            } => center + (major_r + minor_r * v.cos()) * radial + minor_r * v.sin() * axis,
            Surface::Sphere { center, axis, radius, .. } => {
                center + radius * (v.sin() * radial + v.cos() * axis)
            }
        }
    }

    fn normal_r(&self, radial: Vec3, v: f32) -> Vec3 {
        match *self {
            Surface::Plane { normal, .. } => normal,
            Surface::Cylinder { .. } => radial.normalize(),
            Surface::Cone { axis, half_angle, .. } => {
                let sgn = if v >= 0.0 { -1.0 } else { 1.0 };
                (half_angle.cos() * radial + sgn * half_angle.sin() * axis).normalize()
            }
            Surface::Torus { axis, .. } => (v.cos() * radial + v.sin() * axis).normalize(),
            Surface::Sphere { axis, .. } => (v.sin() * radial + v.cos() * axis).normalize(),
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
        self.point_r(Surface::radial_at(uv.0, d0, d1), uv)
    }

    fn normal_f(&self, uv: Uv, d0: Vec3, d1: Vec3) -> Vec3 {
        self.normal_r(Surface::radial_at(uv.0, d0, d1), uv.1)
    }

    pub fn signed_distance(&self, p: Vec3) -> f32 {
        match *self {
            Surface::Plane { origin, normal, .. } => (p - origin).dot(normal),
            Surface::Cylinder { base, axis, radius, .. } => {
                let d = p - base;
                (d - axis * d.dot(axis)).length() - radius
            }
            Surface::Sphere { center, radius, .. } => (p - center).length() - radius,
            Surface::Cone { apex, axis, half_angle, .. } => {
                let d = p - apex;
                let along = d.dot(axis);
                let perp = (d - axis * along).length();
                perp * half_angle.cos() - along.abs() * half_angle.sin()
            }
            Surface::Torus { center, axis, major_r, minor_r, .. } => {
                let d = p - center;
                let along = d.dot(axis);
                let perp = (d - axis * along).length();
                ((perp - major_r).powi(2) + along * along).sqrt() - minor_r
            }
        }
    }

    pub fn gradient(&self, p: Vec3) -> Vec3 {
        match *self {
            Surface::Plane { normal, .. } => normal,
            Surface::Cylinder { base, axis, .. } => {
                let d = p - base;
                (d - axis * d.dot(axis)).normalize_or(axis)
            }
            Surface::Sphere { center, .. } => (p - center).normalize_or(Vec3::Z),
            Surface::Cone { apex, axis, half_angle, .. } => {
                let d = p - apex;
                let along = d.dot(axis);
                let radial = (d - axis * along).normalize_or(Vec3::ZERO);
                if radial == Vec3::ZERO {
                    return axis;
                }
                let side = if along < 0.0 { -1.0 } else { 1.0 };
                (radial * half_angle.cos() - axis * side * half_angle.sin()).normalize()
            }
            Surface::Torus { center, axis, major_r, .. } => {
                let d = p - center;
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
            Surface::Plane {
                origin,
                u_dir,
                v_dir,
                ..
            } => {
                let d = p - origin;
                (d.dot(u_dir), d.dot(v_dir))
            }
            Surface::Cylinder { base, axis, ref_dir: _, .. } => {
                let rel = p - base;
                let u = wrap_angle(rel.dot(d1).atan2(rel.dot(d0)));
                (u, rel.dot(axis))
            }
            Surface::Cone { apex, axis, ref_dir: _, .. } => {
                let rel = p - apex;
                let u = wrap_angle(rel.dot(d1).atan2(rel.dot(d0)));
                (u, rel.dot(axis))
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
                let v = wrap_angle(rel.dot(axis).atan2(ring));
                (u, v)
            }
            Surface::Sphere { center, axis, radius, ref_dir: _ } => {
                let rel = (p - center) / radius;
                let v = rel.dot(axis).clamp(-1.0, 1.0).acos();
                let u = wrap_angle(rel.dot(d1).atan2(rel.dot(d0)));
                (u, v)
            }
        }
    }
}

impl Surface {
    pub fn frame(&self) -> (Vec3, Vec3) {
        match *self {
            Surface::Plane { .. } => (Vec3::ZERO, Vec3::ZERO),
            Surface::Cylinder { axis, ref_dir, .. }
            | Surface::Cone { axis, ref_dir, .. }
            | Surface::Torus { axis, ref_dir, .. }
            | Surface::Sphere { axis, ref_dir, .. } => radial_frame(axis, ref_dir),
        }
    }

    pub fn prepare(&self) -> Prepared {
        let (d0, d1) = self.frame();
        Prepared { surface: *self, d0, d1 }
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
        self.surface.point_r(radial, uv)
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
    Line { p0: Vec3, dir: Vec3 },
    Circle {
        center: Vec3,
        axis: Vec3,
        radius: f32,
        ref_dir: Vec3,
    },
    Ellipse { center: Vec3, a: Vec3, b: Vec3 },
    TorusSection {
        center: Vec3,
        axis: Vec3,
        ref_dir: Vec3,
        major: f32,
        minor: f32,
        offset: f32,
        branch: f32,
    },
}

impl Curve {
    pub fn line(a: Vec3, b: Vec3) -> Curve {
        Curve::Line {
            p0: a,
            dir: (b - a).normalize_or_zero(),
        }
    }

    pub fn circle_z(center: Vec3, radius: f32) -> Curve {
        Curve::Circle { center, axis: Vec3::Z, radius, ref_dir: Vec3::X }
    }

    pub fn point(&self, t: f32) -> Vec3 {
        match *self {
            Curve::Line { p0, dir } => p0 + t * dir,
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
            Curve::TorusSection { center, axis, ref_dir, major, minor, offset, branch } => {
                let (d0, d1) = radial_frame(axis, ref_dir);
                let rad = major + minor * t.cos();
                let cos_u = (offset / rad).clamp(-1.0, 1.0);
                let sin_u = branch * (1.0 - cos_u * cos_u).max(0.0).sqrt();
                center + rad * (cos_u * d0 + sin_u * d1) + minor * t.sin() * axis
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
        Curve::TorusSection {
            center,
            axis: axis.normalize(),
            ref_dir: plane_normal.normalize(),
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
