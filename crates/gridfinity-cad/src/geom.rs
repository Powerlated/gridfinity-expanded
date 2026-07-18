//! Analytic geometry: the exact surfaces faces live on and the exact curves
//! edges live on. Nothing here is faceted — tessellation happens only in
//! `tess.rs`. All cylinder/cone/torus axes are the +Z axis, which is what keeps
//! surface intersection closed-form (see `boolean.rs`).
//!
//! Every surface carries a local orthonormal frame so a 3D point maps to a
//! wrap-free `(u, v)` parameter pair. Faces are constructed with `ref_dir`
//! pointing at the *start* of their bounding arc, so a partial (quarter- or
//! half-) surface never straddles the `atan2` branch cut.

use crate::math::Vec3;
use std::f32::consts::PI;

/// A 2D parameter point on a surface.
pub type Uv = (f32, f32);

/// Right-handed frame `(d0, d1)` spanning the plane perpendicular to +Z, with
/// `d0` at the given reference direction (projected onto the XY plane).
fn radial_frame(ref_dir: Vec3) -> (Vec3, Vec3) {
    let mut d0 = Vec3::new(ref_dir.x, ref_dir.y, 0.0);
    if d0.length_squared() < 1e-12 {
        d0 = Vec3::X;
    }
    let d0 = d0.normalize();
    let d1 = Vec3::Z.cross(d0); // +90° about Z
    (d0, d1)
}

/// Exact surface a face lies on.
#[derive(Clone, Copy, Debug)]
pub enum Surface {
    /// Point = origin + u·u_dir + v·v_dir; `u_dir × v_dir = normal`.
    Plane {
        origin: Vec3,
        normal: Vec3,
        u_dir: Vec3,
        v_dir: Vec3,
    },
    /// Axis +Z through `base`. u = angle from `ref_dir`, v = height along Z.
    Cylinder { base: Vec3, radius: f32, ref_dir: Vec3 },
    /// Axis +Z, apex at `apex`, opening as v (height above apex) grows.
    /// `half_angle` is measured from the axis. u = angle, v = height along Z.
    Cone {
        apex: Vec3,
        half_angle: f32,
        ref_dir: Vec3,
    },
    /// Axis +Z through `center`. u = angle about Z, v = angle about the tube.
    Torus {
        center: Vec3,
        major_r: f32,
        minor_r: f32,
        ref_dir: Vec3,
    },
}

impl Surface {
    pub fn plane(origin: Vec3, normal: Vec3) -> Surface {
        let normal = normal.normalize();
        // Pick any u_dir perpendicular to the normal.
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

    /// Horizontal plane at height `z`, normal +Z.
    pub fn plane_z(z: f32) -> Surface {
        Surface::Plane {
            origin: Vec3::new(0.0, 0.0, z),
            normal: Vec3::Z,
            u_dir: Vec3::X,
            v_dir: Vec3::Y,
        }
    }

    pub fn point(&self, uv: Uv) -> Vec3 {
        let (u, v) = uv;
        match *self {
            Surface::Plane {
                origin,
                u_dir,
                v_dir,
                ..
            } => origin + u * u_dir + v * v_dir,
            Surface::Cylinder { base, radius, ref_dir } => {
                let (d0, d1) = radial_frame(ref_dir);
                base + radius * (u.cos() * d0 + u.sin() * d1) + v * Vec3::Z
            }
            Surface::Cone {
                apex,
                half_angle,
                ref_dir,
            } => {
                let (d0, d1) = radial_frame(ref_dir);
                let r = v * half_angle.tan();
                apex + v * Vec3::Z + r * (u.cos() * d0 + u.sin() * d1)
            }
            Surface::Torus {
                center,
                major_r,
                minor_r,
                ref_dir,
            } => {
                let (d0, d1) = radial_frame(ref_dir);
                let radial = u.cos() * d0 + u.sin() * d1;
                center + (major_r + minor_r * v.cos()) * radial + minor_r * v.sin() * Vec3::Z
            }
        }
    }

    /// Outward unit normal at `uv` (before the face `sense` flip).
    pub fn normal(&self, uv: Uv) -> Vec3 {
        let (u, v) = uv;
        match *self {
            Surface::Plane { normal, .. } => normal,
            Surface::Cylinder { ref_dir, .. } => {
                let (d0, d1) = radial_frame(ref_dir);
                (u.cos() * d0 + u.sin() * d1).normalize()
            }
            Surface::Cone {
                half_angle, ref_dir, ..
            } => {
                let (d0, d1) = radial_frame(ref_dir);
                let radial = u.cos() * d0 + u.sin() * d1;
                (half_angle.cos() * radial - half_angle.sin() * Vec3::Z).normalize()
            }
            Surface::Torus { ref_dir, .. } => {
                let (d0, d1) = radial_frame(ref_dir);
                let radial = u.cos() * d0 + u.sin() * d1;
                (v.cos() * radial + v.sin() * Vec3::Z).normalize()
            }
        }
    }

    /// Inverse of `point`: map a 3D point onto `(u, v)`. Angles are returned in
    /// a branch continuous from `ref_dir` (0..2π), so partial surfaces stay
    /// monotone.
    pub fn project(&self, p: Vec3) -> Uv {
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
            Surface::Cylinder { base, ref_dir, .. } => {
                let (d0, d1) = radial_frame(ref_dir);
                let rel = p - base;
                let u = wrap_angle(rel.dot(d1).atan2(rel.dot(d0)));
                (u, rel.z)
            }
            Surface::Cone { apex, ref_dir, .. } => {
                let (d0, d1) = radial_frame(ref_dir);
                let rel = p - apex;
                let u = wrap_angle(rel.dot(d1).atan2(rel.dot(d0)));
                (u, rel.z)
            }
            Surface::Torus {
                center,
                major_r,
                ref_dir,
                ..
            } => {
                let (d0, d1) = radial_frame(ref_dir);
                let rel = p - center;
                let u = wrap_angle(rel.dot(d1).atan2(rel.dot(d0)));
                let radial = u.cos() * d0 + u.sin() * d1;
                let ring = rel.dot(radial) - major_r;
                let v = wrap_angle(rel.z.atan2(ring));
                (u, v)
            }
        }
    }
}

/// Map an angle to `[0, 2π)`.
fn wrap_angle(a: f32) -> f32 {
    let mut a = a % (2.0 * PI);
    if a < 0.0 {
        a += 2.0 * PI;
    }
    a
}

/// Exact curve an edge lies on.
#[derive(Clone, Copy, Debug)]
pub enum Curve {
    /// Straight line through `p0` with unit direction `dir`.
    Line { p0: Vec3, dir: Vec3 },
    /// Circle of `radius` centred at `center`, axis +Z, angle from `ref_dir`.
    Circle {
        center: Vec3,
        radius: f32,
        ref_dir: Vec3,
    },
}

impl Curve {
    pub fn line(a: Vec3, b: Vec3) -> Curve {
        Curve::Line {
            p0: a,
            dir: (b - a).normalize_or_zero(),
        }
    }

    /// Point at parameter `t`: distance along a Line, angle (radians) on a Circle.
    pub fn point(&self, t: f32) -> Vec3 {
        match *self {
            Curve::Line { p0, dir } => p0 + t * dir,
            Curve::Circle {
                center,
                radius,
                ref_dir,
            } => {
                let (d0, d1) = radial_frame(ref_dir);
                center + radius * (t.cos() * d0 + t.sin() * d1)
            }
        }
    }
}
