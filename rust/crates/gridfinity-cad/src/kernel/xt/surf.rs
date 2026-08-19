//! The kernel's analytic geometry restated in the parameters an XT surface or
//! curve node carries.
//!
//! Each `XtSurface` and `XtCurve` holds exactly the fields the format defines
//! for that node and nothing else, so `write` is a transcription and every
//! decision about *how* to say a kernel `Surface` or `Curve` in XT terms is made
//! in `of_surface` / `of_curve`. Three of those decisions are not one-to-one.
//! A kernel cone is a double cone about its apex and XT's is a half named by a
//! point on its axis, a radius there, and an axis pointing *away* from the half
//! in use, so the face's own samples pick the half. A kernel ellipse is any
//! `centre + a cos t + b sin t` and XT's is a pair of principal axes, so the
//! pair is rotated until it is orthogonal. And a kernel torus face may sit on
//! either sheet of a spindle, which XT names as separate surfaces through the
//! sign of the major radius.
//!
//! `distance` and `natural_normal` restate each emitted node's own implicit
//! equation and its `dP/du x dP/dv`. They exist to be asserted against the
//! kernel's answer at points the face really contains: a mistranslation that
//! would otherwise surface as a rejected file in someone's CAD system fails
//! here, naming the surface.

use crate::kernel::geom::{Curve, Surface};
use crate::kernel::math::{Dir, Vec3};
use crate::kernel::xt::text::{self, Index, Writer};

/// How far a point the kernel says is on a surface or curve may sit from the
/// emitted node's own implicit form.
///
/// The kernel is `f32`, so a point 300 mm from the origin carries about 3e-5 mm
/// of representation noise and a curve evaluated two ways differs by a few
/// multiples of that. This bound is an order above that noise and three orders
/// below the thinnest feature the model makes, so it passes what `f32` costs and
/// fails any real error of translation, which moves a surface by a whole
/// millimetre or turns it inside out.
pub const ON_GEOMETRY_MM: f32 = 1.0e-3;

/// The fields every curve and surface node begins with, in schema order.
pub struct GeomLinks {
    pub node_id: i64,
    pub owner: Index,
    pub next: Index,
    pub prev: Index,
    pub geometric_owner: Index,
    /// `+` when the node's natural normal or tangent is the one the owning
    /// topology means, `-` when it is the reverse.
    pub sense: char,
}

pub enum XtSurface {
    Plane {
        pvec: Vec3,
        normal: Dir,
        x_axis: Dir,
    },
    Cylinder {
        pvec: Vec3,
        axis: Dir,
        radius: f32,
        x_axis: Dir,
    },
    Cone {
        pvec: Vec3,
        axis: Dir,
        radius: f32,
        half_angle: f32,
        x_axis: Dir,
    },
    Sphere {
        centre: Vec3,
        radius: f32,
        axis: Dir,
        x_axis: Dir,
    },
    Torus {
        centre: Vec3,
        axis: Dir,
        major: f32,
        minor: f32,
        x_axis: Dir,
    },
}

/// The XT node for `surface` as the face carrying `samples` uses it, or a
/// message naming what about that use the format cannot say.
///
/// `samples` are points of the face -- its loop vertices and edge midpoints --
/// and they decide the two things a kernel surface leaves open: which half of a
/// double cone is in use, and which sheet of a self-intersecting torus the face
/// lies on. A face straddling either boundary is refused rather than translated,
/// because the answer would be a surface whose normal turns over inside the
/// face.
pub fn of_surface(surface: &Surface, samples: &[Vec3]) -> Result<XtSurface, String> {
    assert!(
        !samples.is_empty(),
        "a surface is translated as some face uses it, so it needs at least one point of that face"
    );
    let out = match *surface {
        Surface::Plane {
            origin,
            normal,
            x_axis,
        } => XtSurface::Plane {
            pvec: origin,
            normal,
            x_axis,
        },
        Surface::Cylinder {
            base,
            axis,
            radius,
            ref_dir,
        } => {
            assert!(radius > 0.0, "a cylinder's radius must be positive, got {radius}");
            XtSurface::Cylinder {
                pvec: base,
                axis,
                radius,
                x_axis: ref_dir,
            }
        }
        Surface::Cone {
            pvec,
            axis,
            radius,
            half_angle,
            ref_dir,
        } => XtSurface::Cone {
            pvec,
            axis,
            radius,
            half_angle,
            x_axis: ref_dir,
        },
        Surface::Sphere {
            center,
            axis,
            radius,
            ref_dir,
        } => {
            assert!(radius > 0.0, "a sphere's radius must be positive, got {radius}");
            XtSurface::Sphere {
                centre: center,
                radius,
                axis,
                x_axis: ref_dir,
            }
        }
        Surface::Torus {
            center,
            axis,
            major_r,
            minor_r,
            ref_dir,
        } => torus_of(center, axis, major_r, minor_r, ref_dir, samples)?,
    };
    for &p in samples {
        let d = out.distance(p);
        if d.abs() > ON_GEOMETRY_MM {
            return Err(format!(
                "a point of the face, {p:?}, stands {d} mm off the {} node written for its \
                 {surface:?}",
                out.node_name()
            ));
        }
    }
    Ok(out)
}

/// The XT torus the face using `samples` lies on.
///
/// A torus whose minor radius exceeds its major one meets itself, and its two
/// sheets are two XT surfaces: the outer one takes the major radius as the
/// kernel states it, the inner one takes its negative, which is what turns that
/// sheet into the node's own outer sheet. A face with points on both sheets
/// passes through the axis, where the surface is singular and its normal
/// vanishes, and is refused.
fn torus_of(
    centre: Vec3,
    axis: Dir,
    major: f32,
    minor: f32,
    ref_dir: Dir,
    samples: &[Vec3],
) -> Result<XtSurface, String> {
    assert!(
        major > 0.0 && minor > 0.0,
        "the kernel states a torus with positive radii, got major {major} minor {minor}"
    );
    let sheet = |p: Vec3| -> f32 {
        let d = p - centre;
        let along = d.dot(*axis);
        let perp = (d - *axis * along).length();
        let near = ((perp - major).powi(2) + along * along).sqrt() - minor;
        let far = ((perp + major).powi(2) + along * along).sqrt() - minor;
        if near.abs() <= far.abs() { 1.0 } else { -1.0 }
    };
    let first = sheet(samples[0]);
    if samples.iter().any(|&p| sheet(p) != first) {
        return Err(format!(
            "a face on the torus at {centre:?} (major {major}, minor {minor}) crosses the axis \
             the two sheets of a spindle meet on, where its normal vanishes"
        ));
    }
    Ok(XtSurface::Torus {
        centre,
        axis,
        major: first * major,
        minor,
        x_axis: ref_dir,
    })
}

impl XtSurface {
    /// The node type this surface is written as.
    pub fn node_type(&self) -> u16 {
        match self {
            XtSurface::Plane { .. } => text::PLANE,
            XtSurface::Cylinder { .. } => text::CYLINDER,
            XtSurface::Cone { .. } => text::CONE,
            XtSurface::Sphere { .. } => text::SPHERE,
            XtSurface::Torus { .. } => text::TORUS,
        }
    }

    /// The node type's name, for a message about this surface.
    pub fn node_name(&self) -> &'static str {
        match self {
            XtSurface::Plane { .. } => "PLANE",
            XtSurface::Cylinder { .. } => "CYLINDER",
            XtSurface::Cone { .. } => "CONE",
            XtSurface::Sphere { .. } => "SPHERE",
            XtSurface::Torus { .. } => "TORUS",
        }
    }

    /// How far `p` stands from this surface, in millimetres, signed by the side
    /// -- the implicit form of the very parameters that get written.
    pub fn distance(&self, p: Vec3) -> f32 {
        match *self {
            XtSurface::Plane { pvec, normal, .. } => (p - pvec).dot(*normal),
            XtSurface::Cylinder {
                pvec, axis, radius, ..
            } => radial(p - pvec, *axis).1 - radius,
            XtSurface::Cone {
                pvec,
                axis,
                radius,
                half_angle,
                ..
            } => {
                let (sin_half, cos_half) = half_angle.sin_cos();
                let d = p - pvec;
                let (_, perp) = radial(d, *axis);
                perp * cos_half + d.dot(*axis) * sin_half - radius * cos_half
            }
            XtSurface::Sphere { centre, radius, .. } => (p - centre).length() - radius,
            XtSurface::Torus {
                centre,
                axis,
                major,
                minor,
                ..
            } => {
                let d = p - centre;
                let along = d.dot(*axis);
                let (_, perp) = radial(d, *axis);
                ((perp - major).powi(2) + along * along).sqrt() - minor
            }
        }
    }

    /// The radius a reader would reject as non-positive, which every curved
    /// surface variant carries and a plane does not (it reports `f32::INFINITY`,
    /// so the one assert in `write` never sees a plane).
    fn radius_of(&self) -> f32 {
        match *self {
            XtSurface::Plane { .. } => f32::INFINITY,
            XtSurface::Cylinder { radius, .. }
            | XtSurface::Cone { radius, .. }
            | XtSurface::Sphere { radius, .. } => radius,
            XtSurface::Torus { minor, .. } => minor,
        }
    }

    /// The surface's natural normal at `p`, the direction of `dP/du x dP/dv` of
    /// the parametric form the format defines for this node.
    ///
    /// It is what the emitted `sense` is measured against: the face's normal is
    /// this direction when the face and surface senses agree, and its reverse
    /// when they do not.
    pub fn natural_normal(&self, p: Vec3) -> Vec3 {
        match *self {
            XtSurface::Plane { normal, .. } => *normal,
            XtSurface::Cylinder { pvec, axis, .. } => radial(p - pvec, *axis).0,
            XtSurface::Cone {
                pvec,
                axis,
                half_angle,
                ..
            } => {
                let (sin_half, cos_half) = half_angle.sin_cos();
                -(cos_half * radial(p - pvec, *axis).0 + sin_half * *axis).normalize()
            }
            XtSurface::Sphere { centre, .. } => (p - centre).normalize(),
            XtSurface::Torus {
                centre,
                axis,
                major,
                ..
            } => {
                let spine = centre + radial(p - centre, *axis).0 * major;
                (p - spine).normalize()
            }
        }
    }

    /// Writes this surface as its node, `links` first in schema order and then
    /// the fields of its own type.
    pub fn write(&self, w: &mut Writer, index: Index, links: &GeomLinks) {
        assert!(
            self.radius_of() > 0.0,
            "every XT surface node with a radius carries a positive one, so {} cannot be written \
             with {}",
            self.node_name(),
            self.radius_of()
        );
        w.begin(self.node_type(), index);
        w.int(links.node_id);
        w.ptr(0);
        w.ptr(links.owner);
        w.ptr(links.next);
        w.ptr(links.prev);
        w.ptr(links.geometric_owner);
        w.ch(links.sense);
        match *self {
            XtSurface::Plane {
                pvec,
                normal,
                x_axis,
            } => {
                w.pos(pvec);
                w.dir(normal);
                w.dir(x_axis);
            }
            XtSurface::Cylinder {
                pvec,
                axis,
                radius,
                x_axis,
            } => {
                w.pos(pvec);
                w.dir(axis);
                w.dist(radius);
                w.dir(x_axis);
            }
            XtSurface::Cone {
                pvec,
                axis,
                radius,
                half_angle,
                x_axis,
            } => {
                let (sin_half, cos_half) = (half_angle as f64).sin_cos();
                assert!(
                    (sin_half * sin_half + cos_half * cos_half - 1.0).abs() <= text::UNIT_RESIDUE,
                    "a CONE node's half-angle sine and cosine are one f64 angle's, so their \
                     squares sum to one: sin {sin_half}, cos {cos_half}"
                );
                w.pos(pvec);
                w.dir(axis);
                w.dist(radius);
                w.real(sin_half);
                w.real(cos_half);
                w.dir(x_axis);
            }
            XtSurface::Sphere {
                centre,
                radius,
                axis,
                x_axis,
            } => {
                w.pos(centre);
                w.dist(radius);
                w.dir(axis);
                w.dir(x_axis);
            }
            XtSurface::Torus {
                centre,
                axis,
                major,
                minor,
                x_axis,
            } => {
                w.pos(centre);
                w.dir(axis);
                w.dist(major);
                w.dist(minor);
                w.dir(x_axis);
            }
        }
    }
}

/// The kernel surface's own natural normal at `p`: the gradient of its
/// implicit form, which is what the kernel's `Face::sense` is measured against.
/// The XT counterpart is `XtSurface::natural_normal`; the pair is what the
/// emitted surface sense reconciles.
pub fn kernel_normal(surface: &Surface, p: Vec3) -> Vec3 {
    let n = surface.gradient(p);
    let len = n.length();
    assert!(
        len > 1e-6,
        "the kernel surface {surface:?} has a natural normal at every point a face uses, but \
         it vanishes at {p:?}"
    );
    n / len
}

pub enum XtCurve {
    Line {
        pvec: Vec3,
        direction: Dir,
    },
    Circle {
        centre: Vec3,
        normal: Dir,
        x_axis: Dir,
        radius: f32,
    },
    Ellipse {
        centre: Vec3,
        normal: Dir,
        x_axis: Dir,
        major: f32,
        minor: f32,
    },
}

/// The XT node for `curve`, which the kernel's `Line`, `Circle` and `Ellipse`
/// each have, or `None` for the one it does not -- a torus section, which the
/// format has no analytic curve for and which `isect` writes as the exact
/// intersection of the two surfaces it lies on instead.
pub fn of_curve(curve: &Curve) -> Option<XtCurve> {
    match *curve {
        Curve::Line { p0, dir } => Some(XtCurve::Line {
            pvec: p0,
            direction: dir,
        }),
        Curve::Circle {
            center,
            axis,
            radius,
            ref_dir,
        } => Some(XtCurve::Circle {
            centre: center,
            normal: axis,
            x_axis: ref_dir,
            radius,
        }),
        Curve::Ellipse { center, a, b } => {
            let (x_axis, major, minor, normal) = principal_axes(a, b);
            Some(XtCurve::Ellipse {
                centre: center,
                normal: Dir::new(normal),
                x_axis: Dir::new(x_axis).perp_to(Dir::new(normal)),
                major,
                minor,
            })
        }
        Curve::TorusSection { .. } => None,
    }
}

/// The principal axes of the ellipse `centre + a cos t + b sin t`, as the
/// (x_axis, major radius, minor radius, normal) an XT ELLIPSE carries.
///
/// `a` and `b` span the ellipse's plane but need be neither orthogonal nor
/// ordered by length. Rotating the pair by the angle that makes it orthogonal
/// leaves the same point set, traversed in the same direction -- which is why
/// the normal is taken from `a x b` and survives the rotation -- and swapping a
/// too-short first axis for the second turns the pair by a further quarter,
/// preserving that direction rather than reflecting it.
fn principal_axes(a: Vec3, b: Vec3) -> (Vec3, f32, f32, Vec3) {
    let (aa, bb, ab) = (a.dot(a), b.dot(b), a.dot(b));
    let theta = 0.5 * (2.0 * ab).atan2(aa - bb);
    let (s, c) = theta.sin_cos();
    let (mut x, mut y) = (a * c + b * s, b * c - a * s);
    assert!(
        x.dot(y).abs() <= 1e-3 * x.length() * y.length(),
        "rotating an ellipse's axes by {theta} must make them orthogonal, but {x:?} and {y:?} \
         meet at a cosine of {}",
        x.dot(y) / (x.length() * y.length())
    );
    if x.length() < y.length() {
        (x, y) = (y, -x);
    }
    let normal = x.cross(y).normalize();
    assert!(
        a.cross(b).dot(normal) > 0.0,
        "an ellipse's principal axes must be reached by a rotation, which cannot reverse the \
         direction {:?} it is traversed in",
        a.cross(b)
    );
    (x.normalize(), x.length(), y.length(), normal)
}

impl XtCurve {
    /// The node type this curve is written as.
    pub fn node_type(&self) -> u16 {
        match self {
            XtCurve::Line { .. } => text::LINE,
            XtCurve::Circle { .. } => text::CIRCLE,
            XtCurve::Ellipse { .. } => text::ELLIPSE,
        }
    }

    /// How far `p` stands from this curve, in millimetres -- unsigned, since a
    /// curve has no side.
    pub fn distance(&self, p: Vec3) -> f32 {
        match *self {
            XtCurve::Line { pvec, direction } => {
                let d = p - pvec;
                (d - *direction * d.dot(*direction)).length()
            }
            XtCurve::Circle {
                centre,
                normal,
                radius,
                ..
            } => {
                let d = p - centre;
                let along = d.dot(*normal);
                ((d - *normal * along).length() - radius).hypot(along)
            }
            XtCurve::Ellipse {
                centre,
                normal,
                x_axis,
                major,
                minor,
            } => {
                let d = p - centre;
                let y_axis = normal.cross(*x_axis);
                let (u, v) = (d.dot(*x_axis) / major, d.dot(y_axis) / minor);
                let scale = u.hypot(v).max(1e-9);
                let on = centre + *x_axis * (major * u / scale) + y_axis * (minor * v / scale);
                (p - on).length()
            }
        }
    }

    /// The radius a reader would reject as non-positive: a line has none (it
    /// reports `f32::INFINITY`, so the one assert in `write` never sees one).
    fn radius_of(&self) -> f32 {
        match *self {
            XtCurve::Line { .. } => f32::INFINITY,
            XtCurve::Circle { radius, .. } => radius,
            XtCurve::Ellipse { minor, .. } => minor,
        }
    }

    /// The curve's natural tangent at `p`, the direction its parameter
    /// increases in, which the emitted `sense` is measured against.
    pub fn tangent(&self, p: Vec3) -> Vec3 {
        match *self {
            XtCurve::Line { direction, .. } => *direction,
            XtCurve::Circle { centre, normal, .. } => normal.cross(p - centre).normalize(),
            XtCurve::Ellipse {
                centre,
                normal,
                x_axis,
                major,
                minor,
            } => {
                let d = p - centre;
                let y_axis = normal.cross(*x_axis);
                let (cos_t, sin_t) = (d.dot(*x_axis) / major, d.dot(y_axis) / minor);
                (y_axis * (minor * cos_t) - *x_axis * (major * sin_t)).normalize()
            }
        }
    }

    /// Writes this curve as its node, `links` first in schema order and then the
    /// fields of its own type.
    pub fn write(&self, w: &mut Writer, index: Index, links: &GeomLinks) {
        assert!(
            self.radius_of() > 0.0,
            "every XT curve node with a radius carries a positive one, so {:?} cannot be \
             written with {}",
            self.node_type(),
            self.radius_of()
        );
        w.begin(self.node_type(), index);
        w.int(links.node_id);
        w.ptr(0);
        w.ptr(links.owner);
        w.ptr(links.next);
        w.ptr(links.prev);
        w.ptr(links.geometric_owner);
        w.ch(links.sense);
        match *self {
            XtCurve::Line { pvec, direction } => {
                w.pos(pvec);
                w.dir(direction);
            }
            XtCurve::Circle {
                centre,
                normal,
                x_axis,
                radius,
            } => {
                w.pos(centre);
                w.dir(normal);
                w.dir(x_axis);
                w.dist(radius);
            }
            XtCurve::Ellipse {
                centre,
                normal,
                x_axis,
                major,
                minor,
            } => {
                w.pos(centre);
                w.dir(normal);
                w.dir(x_axis);
                w.dist(major);
                w.dist(minor);
            }
        }
    }
}

/// The unit radial direction of `d` about an axis through the origin, and the
/// distance out to it.
///
/// The direction is `Vec3::ZERO` exactly when `d` lies on the axis, which every
/// caller here treats as the degenerate case it is.
fn radial(d: Vec3, axis: Vec3) -> (Vec3, f32) {
    let out = d - axis * d.dot(axis);
    let len = out.length();
    if len < 1e-9 {
        return (Vec3::ZERO, 0.0);
    }
    (out / len, len)
}
