//! Analytic geometry: the exact surfaces faces live on and the exact curves
//! edges live on. Nothing here is faceted — tessellation happens only in
//! `tess.rs`.
//!
//! Every radial surface (`Cylinder`/`Cone`/`Torus`/`Sphere`) carries its own
//! `axis` (unit) and a `ref_dir` perpendicular to it. `radial_frame` builds the
//! orthonormal `(d0, d1)` with `d0` toward `ref_dir` and `d1 = axis × d0`, so a
//! 3D point maps to a wrap-free `(u, v)` pair. Faces are constructed with
//! `ref_dir` pointing at the *start* of their bounding arc, so a partial
//! (quarter- or half-) surface never straddles the `atan2` branch cut.

use crate::kernel::math::Vec3;
use std::f32::consts::PI;

/// A 2D parameter point on a surface.
pub type Uv = (f32, f32);

/// Unit vector perpendicular to `axis`, as close as possible to `hint`.
pub fn perp_unit(axis: Vec3, hint: Vec3) -> Vec3 {
    let d = hint - axis * hint.dot(axis);
    if d.length_squared() < 1e-12 {
        // hint parallel to axis: pick any perpendicular basis vector.
        let a = if axis.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
        (a - axis * a.dot(axis)).normalize()
    } else {
        d.normalize()
    }
}

/// Right-handed frame `(d0, d1)` perpendicular to `axis`, `d0` toward `ref_dir`.
pub fn radial_frame(axis: Vec3, ref_dir: Vec3) -> (Vec3, Vec3) {
    let d0 = perp_unit(axis, ref_dir);
    let d1 = axis.cross(d0);
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
    /// `base` point on the axis; axis direction `axis`; u = angle from
    /// `ref_dir`, v = signed distance along `axis`.
    Cylinder { base: Vec3, axis: Vec3, radius: f32, ref_dir: Vec3 },
    /// Apex at `apex`, axis `axis`, `half_angle` from the axis. u = angle,
    /// v = signed distance along `axis` (a frustum on the lower nappe has v<0).
    Cone {
        apex: Vec3,
        axis: Vec3,
        half_angle: f32,
        ref_dir: Vec3,
    },
    /// `center` on the axis; u = angle about `axis`, v = angle about the tube.
    Torus {
        center: Vec3,
        axis: Vec3,
        major_r: f32,
        minor_r: f32,
        ref_dir: Vec3,
    },
    /// `center`; u = angle about `axis` (longitude), v = angle from `axis`
    /// (colatitude, 0 at +axis pole, π at −axis pole).
    Sphere { center: Vec3, axis: Vec3, radius: f32, ref_dir: Vec3 },
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

    /// Cylinder with axis +Z through `base` (the common vertical case).
    pub fn cylinder_z(base: Vec3, radius: f32) -> Surface {
        Surface::Cylinder { base, axis: Vec3::Z, radius, ref_dir: Vec3::X }
    }

    /// Cylinder with explicit axis.
    pub fn cylinder(base: Vec3, axis: Vec3, radius: f32, ref_dir: Vec3) -> Surface {
        Surface::Cylinder { base, axis, radius, ref_dir }
    }

    /// Cone with axis +Z, apex at `apex`.
    pub fn cone_z(apex: Vec3, half_angle: f32) -> Surface {
        Surface::Cone { apex, axis: Vec3::Z, half_angle, ref_dir: Vec3::X }
    }

    /// Torus with axis +Z through `center`.
    pub fn torus_z(center: Vec3, major_r: f32, minor_r: f32) -> Surface {
        Surface::Torus { center, axis: Vec3::Z, major_r, minor_r, ref_dir: Vec3::X }
    }

    /// Sphere centered at `center`, pole axis +Z.
    pub fn sphere(center: Vec3, radius: f32) -> Surface {
        Surface::Sphere { center, axis: Vec3::Z, radius, ref_dir: Vec3::X }
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
            Surface::Cylinder { base, axis, radius, ref_dir } => {
                let (d0, d1) = radial_frame(axis, ref_dir);
                base + radius * (u.cos() * d0 + u.sin() * d1) + v * axis
            }
            Surface::Cone {
                apex,
                axis,
                half_angle,
                ref_dir,
            } => {
                let (d0, d1) = radial_frame(axis, ref_dir);
                // `v` is a signed distance along the axis; a frustum can lie on
                // the lower nappe (v < 0). The radial distance is |v|·tan so the
                // point stays at angle `u` on either nappe.
                let r = v.abs() * half_angle.tan();
                apex + v * axis + r * (u.cos() * d0 + u.sin() * d1)
            }
            Surface::Torus {
                center,
                axis,
                major_r,
                minor_r,
                ref_dir,
            } => {
                let (d0, d1) = radial_frame(axis, ref_dir);
                let radial = u.cos() * d0 + u.sin() * d1;
                center + (major_r + minor_r * v.cos()) * radial + minor_r * v.sin() * axis
            }
            Surface::Sphere { center, axis, radius, ref_dir } => {
                let (d0, d1) = radial_frame(axis, ref_dir);
                let radial = u.cos() * d0 + u.sin() * d1;
                center + radius * (v.sin() * radial + v.cos() * axis)
            }
        }
    }

    /// Outward unit normal at `uv` (before the face `sense` flip).
    pub fn normal(&self, uv: Uv) -> Vec3 {
        let (u, v) = uv;
        match *self {
            Surface::Plane { normal, .. } => normal,
            Surface::Cylinder { axis, ref_dir, .. } => {
                let (d0, d1) = radial_frame(axis, ref_dir);
                (u.cos() * d0 + u.sin() * d1).normalize()
            }
            Surface::Cone {
                axis,
                half_angle,
                ref_dir,
                ..
            } => {
                let (d0, d1) = radial_frame(axis, ref_dir);
                let radial = u.cos() * d0 + u.sin() * d1;
                // Outward normal tilts away from the apex: −axis on the upper
                // nappe (v > 0), +axis on the lower nappe (v < 0).
                let sgn = if v >= 0.0 { -1.0 } else { 1.0 };
                (half_angle.cos() * radial + sgn * half_angle.sin() * axis).normalize()
            }
            Surface::Torus { axis, ref_dir, .. } => {
                let (d0, d1) = radial_frame(axis, ref_dir);
                let radial = u.cos() * d0 + u.sin() * d1;
                (v.cos() * radial + v.sin() * axis).normalize()
            }
            Surface::Sphere { axis, ref_dir, .. } => {
                let (d0, d1) = radial_frame(axis, ref_dir);
                let radial = u.cos() * d0 + u.sin() * d1;
                (v.sin() * radial + v.cos() * axis).normalize()
            }
        }
    }

    /// Exact sign of `(∂p/∂u × ∂p/∂v) · normal`: whether the `(u, v)`
    /// parameterization is right-handed about the outward normal. Constant per
    /// variant, derived in closed form from `point`/`normal`.
    /// Exact signed distance from `p` to this surface — negative on the inside
    /// of the closed ones, and the plane's own signed offset. Every case is
    /// closed form, not an iterative projection.
    ///
    /// This is the surface's implicit equation, normalised so `|∇f| = 1`. The
    /// intersection code in [`crate::kernel::isect`] uses it two ways: as the
    /// residual to test a candidate point against, and (with [`Self::gradient`])
    /// as one row of the Newton system that walks a curve neither surface can
    /// parameterise.
    pub fn signed_distance(&self, p: Vec3) -> f32 {
        match *self {
            Surface::Plane { origin, normal, .. } => (p - origin).dot(normal),
            Surface::Cylinder { base, axis, radius, .. } => {
                let d = p - base;
                (d - axis * d.dot(axis)).length() - radius
            }
            Surface::Sphere { center, radius, .. } => (p - center).length() - radius,
            Surface::Cone { apex, axis, half_angle, .. } => {
                // Measured in the axial half-plane through `p`: rotating the
                // (along, perp) pair by the half angle turns the cone into the
                // line "perp = 0", so the residual is the rotated perp.
                //
                // `|along|`, not `along`: a `Cone` is the full double cone, and
                // the kernel builds frusta on the lower nappe as readily as the
                // upper one (`v < 0`, see the type's own docs). Using the signed
                // value would describe only the +axis half and report every
                // point of a lower-nappe frustum as off-surface.
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

    /// Unit gradient of [`Self::signed_distance`] at `p`: the outward normal of
    /// the level set through `p`. Defined away from the degenerate axis of each
    /// radial surface (and the cone apex), where it falls back to the axis.
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
                // Outward normal: tilt the radial direction away from the axis
                // by the half angle, toward whichever nappe `p` sits on.
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
                // Vector from the tube's spine circle to `p`.
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
            // (∂u×∂v)·n = r/cos α > 0 on either nappe.
            | Surface::Cone { .. } => 1.0,
            // Colatitude v measured from the +axis pole makes (u, v) left-handed.
            Surface::Sphere { .. } => -1.0,
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
            Surface::Cylinder { base, axis, ref_dir, .. } => {
                let (d0, d1) = radial_frame(axis, ref_dir);
                let rel = p - base;
                let u = wrap_angle(rel.dot(d1).atan2(rel.dot(d0)));
                (u, rel.dot(axis))
            }
            Surface::Cone { apex, axis, ref_dir, .. } => {
                let (d0, d1) = radial_frame(axis, ref_dir);
                let rel = p - apex;
                let u = wrap_angle(rel.dot(d1).atan2(rel.dot(d0)));
                (u, rel.dot(axis))
            }
            Surface::Torus {
                center,
                axis,
                major_r,
                ref_dir,
                ..
            } => {
                let (d0, d1) = radial_frame(axis, ref_dir);
                let rel = p - center;
                let u = wrap_angle(rel.dot(d1).atan2(rel.dot(d0)));
                let radial = u.cos() * d0 + u.sin() * d1;
                let ring = rel.dot(radial) - major_r;
                let v = wrap_angle(rel.dot(axis).atan2(ring));
                (u, v)
            }
            Surface::Sphere { center, axis, radius, ref_dir } => {
                let (d0, d1) = radial_frame(axis, ref_dir);
                let rel = (p - center) / radius;
                let v = rel.dot(axis).clamp(-1.0, 1.0).acos();
                let u = wrap_angle(rel.dot(d1).atan2(rel.dot(d0)));
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
    /// Circle of `radius` centred at `center`, in the plane normal to `axis`,
    /// angle measured from `ref_dir` (⊥ axis) toward `axis × ref_dir`.
    Circle {
        center: Vec3,
        axis: Vec3,
        radius: f32,
        ref_dir: Vec3,
    },
    /// Ellipse arc `p(t) = center + cos t · a + sin t · b`, with `a`/`b` the
    /// conjugate semi-axis vectors. Arises where a cylinder is cut by a plane
    /// oblique to its axis (inner-wall ramp side edges).
    Ellipse { center: Vec3, a: Vec3, b: Vec3 },
}

impl Curve {
    pub fn line(a: Vec3, b: Vec3) -> Curve {
        Curve::Line {
            p0: a,
            dir: (b - a).normalize_or_zero(),
        }
    }

    /// Circle in the XY plane (axis +Z), centred at `center`.
    pub fn circle_z(center: Vec3, radius: f32) -> Curve {
        Curve::Circle { center, axis: Vec3::Z, radius, ref_dir: Vec3::X }
    }

    /// Point at parameter `t`: distance along a Line, angle (radians) on a Circle.
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
        }
    }
}
