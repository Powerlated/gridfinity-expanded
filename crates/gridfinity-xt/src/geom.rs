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

use crate::math::{Dir, Vec3};
use std::f64::consts::PI;

pub type Uv = (f64, f64);

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
/// orthogonalisation. The pair is returned in `f64` because it feeds the
/// kernel's own arithmetic; a caller needing the exact reference direction takes
/// it off the surface.
pub fn radial_frame(axis: Dir, ref_dir: Dir) -> (Vec3, Vec3) {
    (ref_dir.vec(), axis.cross_dir(ref_dir).vec())
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
        radius: f64,
        ref_dir: Dir,
    },
    Cone {
        pvec: Vec3,
        axis: Dir,
        radius: f64,
        half_angle: f64,
        ref_dir: Dir,
    },
    Torus {
        center: Vec3,
        axis: Dir,
        major_r: f64,
        minor_r: f64,
        ref_dir: Dir,
    },
    Sphere {
        center: Vec3,
        axis: Dir,
        radius: f64,
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

    pub fn plane_z(z: f64) -> Surface {
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
        (x_axis.vec(), normal.cross_dir(x_axis).vec())
    }

    pub fn cylinder_z(base: Vec3, radius: f64) -> Surface {
        Surface::cylinder(base, Vec3::Z, radius, Vec3::X)
    }

    pub fn cylinder(base: Vec3, axis: Vec3, radius: f64, ref_dir: Vec3) -> Surface {
        let axis = Dir::new(axis);
        Surface::Cylinder {
            base,
            axis,
            radius,
            ref_dir: perp_unit(axis, ref_dir),
        }
    }

    /// The half-cone whose apex is at `apex` and which opens along +Z, its
    /// radius reaching `tan(half_angle)` one millimetre above the apex.
    pub fn cone_z(apex: Vec3, half_angle: f64) -> Surface {
        Surface::cone(
            apex + Vec3::Z,
            Vec3::Z,
            half_angle.tan(),
            half_angle,
            Vec3::X,
        )
    }

    /// The half-cone of `half_angle` whose radius is `radius` at `pvec` and
    /// grows along `open`.
    ///
    /// One nappe, not two: the surface is the half on the `open` side of the
    /// apex, which is where `radius` being positive puts `pvec`. That is what
    /// the format names as a cone and what a face can lie on without its normal
    /// turning over, so the kernel names it too rather than leaving the choice
    /// to whoever reads the surface later. `axis` is stored pointing at the
    /// apex, opposite `open`, as a CONE node carries it.
    pub fn cone(pvec: Vec3, open: Vec3, radius: f64, half_angle: f64, ref_dir: Vec3) -> Surface {
        assert!(
            radius > 0.0,
            "a cone is named by a positive radius at a point of its axis, got {radius}"
        );
        assert!(
            half_angle > 0.0 && half_angle < std::f64::consts::FRAC_PI_2,
            "a cone's half angle lies strictly between zero and a quarter turn, got {half_angle}"
        );
        let axis = -Dir::new(open);
        Surface::Cone {
            pvec,
            axis,
            radius,
            half_angle,
            ref_dir: perp_unit(axis, ref_dir),
        }
    }

    /// The apex of the cone this surface lies on: the point along `axis` at
    /// which its radius falls to zero. Derived rather than stored, because
    /// `pvec` and `radius` already fix it and a second copy could disagree.
    pub fn cone_apex(&self) -> Vec3 {
        let Surface::Cone {
            pvec,
            axis,
            radius,
            half_angle,
            ..
        } = *self
        else {
            panic!("only a cone has an apex, got {self:?}");
        };
        pvec + *axis * (radius / half_angle.tan())
    }

    /// The direction a cone's radius grows in, away from its apex.
    pub fn cone_open(&self) -> Dir {
        let Surface::Cone { axis, .. } = *self else {
            panic!("only a cone opens, got {self:?}");
        };
        -axis
    }

    pub fn torus_z(center: Vec3, major_r: f64, minor_r: f64) -> Surface {
        Surface::torus(center, Vec3::Z, major_r, minor_r, Vec3::X)
    }

    /// The torus of the given radii about `axis` through `center`.
    ///
    /// `major_r` is signed, and its sign names which sheet of the surface this
    /// is. A torus whose minor radius exceeds its major one -- a spindle, which
    /// is what a rolling ball leaves at a concave corner -- meets itself on the
    /// axis and has two: the outer sheet takes the major radius positive, the
    /// inner one takes it negative, which is what turns that sheet into the
    /// surface's own outer one. For a ring torus the two coincide and the sign
    /// is positive. A face cannot span both, its normal vanishing where they
    /// meet, so the surface names the sheet rather than leaving a later reader
    /// to infer it from the face.
    pub fn torus(center: Vec3, axis: Vec3, major_r: f64, minor_r: f64, ref_dir: Vec3) -> Surface {
        assert!(
            major_r != 0.0 && minor_r > 0.0,
            "a torus has a non-zero major radius and a positive minor one, got major {major_r} \
             minor {minor_r}"
        );
        let axis = Dir::new(axis);
        Surface::Torus {
            center,
            axis,
            major_r,
            minor_r,
            ref_dir: perp_unit(axis, ref_dir),
        }
    }

    /// The torus of the given radii carrying the point `on`, whose sheet is
    /// read off that point.
    ///
    /// `major_r` is given unsigned, as the geometry that produced it states it;
    /// this is for the caller that has just solved a rolling-ball corner and
    /// knows a point of the face but not which sheet of a spindle it landed on.
    pub fn torus_through(
        center: Vec3,
        axis: Vec3,
        major_r: f64,
        minor_r: f64,
        ref_dir: Vec3,
        on: Vec3,
    ) -> Surface {
        assert!(
            major_r > 0.0,
            "the sheet is read from the point, so the major radius comes in unsigned and \
             positive, got {major_r}"
        );
        let a = Dir::new(axis);
        let d = on - center;
        let along = d.dot(*a);
        let perp = (d - *a * along).length();
        let near = ((perp - major_r).powi(2) + along * along).sqrt() - minor_r;
        let far = ((perp + major_r).powi(2) + along * along).sqrt() - minor_r;
        let sheet = if near.abs() <= far.abs() { 1.0 } else { -1.0 };
        let out = Surface::torus(center, axis, sheet * major_r, minor_r, ref_dir);
        let off = out.signed_distance(on);
        assert!(
            off.abs() <= 1e-3,
            "the point naming a torus's sheet lies on that sheet, but {on:?} stands {off} mm \
             off the one it chose"
        );
        out
    }

    pub fn sphere(center: Vec3, radius: f64) -> Surface {
        Surface::Sphere {
            center,
            axis: Dir::from_f64([0.0, 0.0, 1.0]),
            radius,
            ref_dir: Dir::from_f64([1.0, 0.0, 0.0]),
        }
    }

    #[inline]
    fn radial_at(u: f64, d0: Vec3, d1: Vec3) -> Vec3 {
        u.cos() * d0 + u.sin() * d1
    }

    fn point_r(&self, radial: Vec3, uv: Uv, basis: (Vec3, Vec3)) -> Vec3 {
        let (u, v) = uv;
        match *self {
            Surface::Plane { origin, .. } => origin + u * basis.0 + v * basis.1,
            Surface::Cylinder {
                base, axis, radius, ..
            } => base + radius * radial + v * axis.vec(),
            Surface::Cone { half_angle, .. } => {
                let r = v * half_angle.tan();
                self.cone_apex() + v * self.cone_open().vec() + r * radial
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

    fn normal_r(&self, radial: Vec3, v: f64) -> Vec3 {
        match *self {
            Surface::Plane { normal, .. } => normal.vec(),
            Surface::Cylinder { .. } => radial.normalize(),
            Surface::Cone { half_angle, .. } => {
                let open = self.cone_open().vec();
                (half_angle.cos() * radial - half_angle.sin() * open).normalize()
            }
            Surface::Torus { axis, .. } => {
                (v.cos() * radial + v.sin() * axis.vec()).normalize()
            }
            Surface::Sphere { axis, .. } => {
                (v.sin() * radial + v.cos() * axis.vec()).normalize()
            }
        }
    }

    fn normal_ignores_v(&self, _v0: f64, _v1: f64) -> bool {
        match *self {
            Surface::Plane { .. } | Surface::Cylinder { .. } => true,
            Surface::Cone { .. } => true,
            Surface::Torus { .. } | Surface::Sphere { .. } => false,
        }
    }

    fn point_f(&self, uv: Uv, d0: Vec3, d1: Vec3) -> Vec3 {
        self.point_r(Surface::radial_at(uv.0, d0, d1), uv, (d0, d1))
    }

    fn normal_f(&self, uv: Uv, d0: Vec3, d1: Vec3) -> Vec3 {
        self.normal_r(Surface::radial_at(uv.0, d0, d1), uv.1)
    }

    pub fn signed_distance(&self, p: Vec3) -> f64 {
        match *self {
            Surface::Plane { origin, normal, .. } => (p - origin).dot(normal.vec()),
            Surface::Cylinder {
                base, axis, radius, ..
            } => {
                let (d, axis) = (p - base, axis.vec());
                (d - axis * d.dot(axis)).length() - radius
            }
            Surface::Sphere { center, radius, .. } => (p - center).length() - radius,
            Surface::Cone { half_angle, .. } => {
                let open = self.cone_open().vec();
                let d = p - self.cone_apex();
                let along = d.dot(open);
                let perp = (d - open * along).length();
                perp * half_angle.cos() - along * half_angle.sin()
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
                ((perp - major_r).powi(2) + along * along).sqrt() - minor_r
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
            Surface::Cone { half_angle, .. } => {
                let open = self.cone_open().vec();
                let d = p - self.cone_apex();
                let along = d.dot(open);
                let radial = (d - open * along).normalize_or(Vec3::ZERO);
                if radial == Vec3::ZERO {
                    return open;
                }
                (radial * half_angle.cos() - open * half_angle.sin()).normalize()
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

    pub fn uv_orientation(&self) -> f64 {
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
            Surface::Cone { .. } => {
                let rel = p - self.cone_apex();
                let u = wrap_angle(rel.dot(d1).atan2(rel.dot(d0)));
                (u, rel.dot(self.cone_open().vec()))
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
    pub fn radial(&self, u: f64) -> Vec3 {
        Surface::radial_at(u, self.d0, self.d1)
    }

    #[inline]
    pub fn point_at(&self, radial: Vec3, uv: Uv) -> Vec3 {
        self.surface.point_r(radial, uv, (self.d0, self.d1))
    }

    #[inline]
    pub fn normal_at(&self, radial: Vec3, v: f64) -> Vec3 {
        self.surface.normal_r(radial, v)
    }

    #[inline]
    pub fn normal_ignores_v(&self, v0: f64, v1: f64) -> bool {
        self.surface.normal_ignores_v(v0, v1)
    }
}

fn wrap_angle(a: f64) -> f64 {
    let mut a = a % (2.0 * PI);
    if a < 0.0 {
        a += 2.0 * PI;
    }
    a
}

#[derive(Clone, Debug)]
pub enum Curve {
    Line {
        p0: Vec3,
        dir: Dir,
    },
    Circle {
        center: Vec3,
        axis: Dir,
        radius: f64,
        ref_dir: Dir,
    },
    Ellipse {
        center: Vec3,
        axis: Dir,
        x_axis: Dir,
        major: f64,
        minor: f64,
    },
    /// A curve the analytic set cannot name, carried as the ordered chart of
    /// points that identifies which branch of its two surfaces' intersection is
    /// meant. `t` runs 0 to 1 along the chart.
    ///
    /// This is what a boolean's own section edges are: a fillet running out
    /// mid-face, a cut through a blend. The chart is a *description* -- the
    /// reader recomputes the curve from the two exact surfaces -- so it need
    /// only be dense enough to be unambiguous, never dense enough to measure.
    Section {
        chart: Vec<Vec3>,
    },
    TorusSection {
        center: Vec3,
        axis: Dir,
        ref_dir: Dir,
        major: f64,
        minor: f64,
        offset: f64,
        branch: f64,
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

    pub fn circle_z(center: Vec3, radius: f64) -> Curve {
        Curve::circle(center, Vec3::Z, radius, Vec3::X)
    }

    /// The circle of `radius` about `center` in the plane through it normal to
    /// `axis`, with its parameter measured from the projection of `ref_dir`.
    pub fn circle(center: Vec3, axis: Vec3, radius: f64, ref_dir: Vec3) -> Curve {
        let axis = Dir::new(axis);
        Curve::Circle {
            center,
            axis,
            radius,
            ref_dir: perp_unit(axis, ref_dir),
        }
    }

    pub fn point(&self, t: f64) -> Vec3 {
        if let Curve::Section { chart } = self {
            return chart_point(chart, t);
        }
        match *self {
            Curve::Section { .. } => unreachable!("a section is answered from its chart above"),
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
            Curve::Ellipse {
                center,
                axis,
                x_axis,
                major,
                minor,
            } => {
                let y = axis.cross_dir(x_axis).vec();
                center + (major * t.cos()) * x_axis.vec() + (minor * t.sin()) * y
            }
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
    pub fn tangent(&self, t: f64) -> Vec3 {
        if let Curve::Section { chart } = self {
            return chart_tangent(chart, t);
        }
        match *self {
            Curve::Section { .. } => unreachable!("a section is answered from its chart above"),
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
            Curve::Ellipse {
                axis,
                x_axis,
                major,
                minor,
                ..
            } => {
                let y = axis.cross_dir(x_axis).vec();
                -(major * t.sin()) * x_axis.vec() + (minor * t.cos()) * y
            }
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

    /// The ellipse of `major` and `minor` radii centred at `centre`, lying in
    /// the plane normal to `axis` with its parameter measured from `x_axis`.
    pub fn ellipse(centre: Vec3, axis: Vec3, x_axis: Vec3, major: f64, minor: f64) -> Curve {
        assert!(
            major >= minor && minor > 0.0,
            "an ellipse is named by its major and then its minor radius, both positive, got \
             major {major} minor {minor}"
        );
        let axis = Dir::new(axis);
        Curve::Ellipse {
            center: centre,
            axis,
            x_axis: perp_unit(axis, x_axis),
            major,
            minor,
        }
    }

    /// The ellipse `centre + a cos t + b sin t` restated in principal axes,
    /// with the parameter range `(t0, t1)` carried onto the new parameter.
    ///
    /// `a` and `b` span the ellipse's plane but need be neither orthogonal nor
    /// ordered by length, which is how a conic section falls out of a solve;
    /// the format names an ellipse by a perpendicular pair, longer first.
    /// Rotating the pair by the angle that makes it orthogonal leaves the same
    /// point set traversed in the same direction and shifts every parameter by
    /// that same angle, so the range shifts with it -- returning the curve
    /// without the range would silently move both of its endpoints.
    pub fn ellipse_from_conjugate(
        centre: Vec3,
        a: Vec3,
        b: Vec3,
        t0: f64,
        t1: f64,
    ) -> (Curve, f64, f64) {
        let (aa, bb, ab) = (a.dot(a), b.dot(b), a.dot(b));
        let theta = 0.5 * (2.0 * ab).atan2(aa - bb);
        let (s, c) = theta.sin_cos();
        let (mut x, mut y) = (a * c + b * s, b * c - a * s);
        let mut shift = theta;
        assert!(
            x.dot(y).abs() <= 1e-3 * x.length() * y.length(),
            "rotating an ellipse's axes by {theta} must make them orthogonal, but {x:?} and \
             {y:?} meet at a cosine of {}",
            x.dot(y) / (x.length() * y.length())
        );
        if x.length() < y.length() {
            (x, y) = (y, -x);
            shift += std::f64::consts::FRAC_PI_2;
        }
        let normal = x.cross(y);
        assert!(
            a.cross(b).dot(normal) > 0.0,
            "an ellipse's principal axes are reached by a rotation, which cannot reverse the \
             direction {:?} it is traversed in",
            a.cross(b)
        );
        let out = Curve::ellipse(centre, normal, x, x.length(), y.length());
        for t in [t0, t1] {
            let was = centre + t.cos() * a + t.sin() * b;
            let now = out.point(t - shift);
            assert!(
                (now - was).length() <= 1e-3 * (1.0 + was.length()),
                "restating an ellipse in principal axes moves no point of it, but the end at \
                 {t} was {was:?} and is {now:?}"
            );
        }
        (out, t0 - shift, t1 - shift)
    }

    /// A conjugate pair spanning this ellipse, for a caller stating a question
    /// as `centre + a cos t + b sin t`. Perpendicular, since the stored form
    /// is, and scaled to the two radii.
    pub fn conjugate_axes(&self) -> (Vec3, Vec3) {
        let Curve::Ellipse {
            axis,
            x_axis,
            major,
            minor,
            ..
        } = *self
        else {
            panic!("only an ellipse has conjugate axes, got {self:?}");
        };
        (x_axis.vec() * major, axis.cross_dir(x_axis).vec() * minor)
    }

    pub fn torus_section(
        center: Vec3,
        axis: Vec3,
        plane_normal: Vec3,
        plane_offset: f64,
        major: f64,
        minor: f64,
        branch: f64,
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

    pub fn torus_section_exists(major: f64, minor: f64, offset: f64, t: f64) -> bool {
        let rad = major + minor * t.cos();
        rad > 0.0 && offset.abs() <= rad
    }
}

/// The point `t` of the way along `chart`, `t` running 0 to 1 and clamped, by
/// straight interpolation between the two samples it falls between.
///
/// A chart is a description of a branch rather than a measurement of it, so
/// interpolating one is only ever used to *name* a place on the curve -- which
/// sample a face check should take, which direction an edge runs -- never to
/// state where the curve is. What is transmitted is the chart's own points.
fn chart_point(chart: &[Vec3], t: f64) -> Vec3 {
    assert!(
        chart.len() >= 2,
        "a section's chart names a branch with at least two points, got {}",
        chart.len()
    );
    let at = t.clamp(0.0, 1.0) * (chart.len() - 1) as f64;
    let i = (at.floor() as usize).min(chart.len() - 2);
    chart[i] + (chart[i + 1] - chart[i]) * (at - i as f64)
}

/// The chord direction of `chart` at `t`, which is the direction the edge runs
/// there to within the chart's own resolution.
fn chart_tangent(chart: &[Vec3], t: f64) -> Vec3 {
    assert!(
        chart.len() >= 2,
        "a section's chart names a branch with at least two points, got {}",
        chart.len()
    );
    let at = t.clamp(0.0, 1.0) * (chart.len() - 1) as f64;
    let i = (at.floor() as usize).min(chart.len() - 2);
    chart[i + 1] - chart[i]
}
