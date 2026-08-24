//! The kernel's analytic geometry written as the surface and curve nodes of a
//! transmit file.
//!
//! There is no restatement here and no second copy of the geometry. A kernel
//! `Surface` carries exactly the fields the format's PLANE, CYLINDER, CONE,
//! SPHERE and TORUS carry, in the same terms -- a cone is one nappe named by a
//! point, a radius there and an axis toward the apex; a torus names its sheet
//! by the sign of its major radius; every direction is already unit in `f64`
//! and every reference direction already perpendicular to its axis -- so
//! `write_surface` is a transcription of the fields in schema order. The one
//! decision left is which way the node's natural normal points relative to the
//! kernel's gradient, and `natural_normal_opposes_gradient` answers it from the
//! variant alone rather than by sampling the face.
//!
//! `Curve` is the same story for LINE, CIRCLE and ELLIPSE. `TorusSection` has
//! no analytic node and `isect` writes it as an intersection instead.

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

/// Whether the format's natural normal for this surface's node points opposite
/// the kernel's `gradient`, which is what the emitted surface sense reconciles.
///
/// The format defines a surface's natural normal as `dP/du x dP/dv` of its own
/// parametric form. For a plane, cylinder, sphere and torus that is the
/// outward gradient the kernel already computes. For a cone it is
/// `-(cos(half) * radial + sin(half) * axis)` and the kernel's is
/// `cos(half) * radial - sin(half) * open`, and `axis` is `-open` -- so the two
/// are exact negatives, for every point of every cone. It is a property of the
/// variant, not of the face, so no face need be sampled to learn it.
pub fn natural_normal_opposes_gradient(surface: &Surface) -> bool {
    matches!(surface, Surface::Cone { .. })
}

/// The surface's natural normal at `p` as the format defines it.
pub fn natural_normal(surface: &Surface, p: Vec3) -> Vec3 {
    let n = surface.gradient(p);
    let len = n.length();
    assert!(
        len > 1e-6,
        "the surface {surface:?} has a natural normal at every point a face uses, but it \
         vanishes at {p:?}"
    );
    let n = n / len;
    if natural_normal_opposes_gradient(surface) { -n } else { n }
}

/// The node type `surface` is written as.
fn surface_node(surface: &Surface) -> u16 {
    match surface {
        Surface::Plane { .. } => text::PLANE,
        Surface::Cylinder { .. } => text::CYLINDER,
        Surface::Cone { .. } => text::CONE,
        Surface::Sphere { .. } => text::SPHERE,
        Surface::Torus { .. } => text::TORUS,
    }
}

/// The node type's name, for a message about this surface.
pub fn surface_name(surface: &Surface) -> &'static str {
    match surface {
        Surface::Plane { .. } => "PLANE",
        Surface::Cylinder { .. } => "CYLINDER",
        Surface::Cone { .. } => "CONE",
        Surface::Sphere { .. } => "SPHERE",
        Surface::Torus { .. } => "TORUS",
    }
}

/// Checks that `surface` says what a node of the format can say, given the
/// points of the face using it, and returns a message naming what it cannot.
///
/// Nothing about the surface is decided here -- the kernel has already named
/// the cone's nappe and the torus's sheet. What is left is the format's own
/// requirement that every radius it carries be positive, and the check that the
/// face's own points really do lie on the surface they are written as being on.
pub fn check_surface(surface: &Surface, samples: &[Vec3]) -> Result<(), String> {
    assert!(
        !samples.is_empty(),
        "a surface is written as some face uses it, so it needs at least one point of that face"
    );
    let radius = match *surface {
        Surface::Plane { .. } => f32::INFINITY,
        Surface::Cylinder { radius, .. } | Surface::Sphere { radius, .. } => radius,
        Surface::Cone { radius, .. } => radius,
        Surface::Torus { minor_r, .. } => minor_r,
    };
    if radius <= 0.0 {
        return Err(format!(
            "every {} node carries a positive radius, so one of {radius} cannot be written",
            surface_name(surface)
        ));
    }
    for &p in samples {
        let d = surface.signed_distance(p);
        if d.abs() > ON_GEOMETRY_MM {
            return Err(format!(
                "a point of the face, {p:?}, stands {d} mm off the {} node written for its \
                 {surface:?}",
                surface_name(surface)
            ));
        }
    }
    Ok(())
}

/// Writes `surface` as its node, `links` first in schema order and then the
/// fields of its own type.
pub fn write_surface(w: &mut Writer, surface: &Surface, index: Index, links: &GeomLinks) {
    w.begin(surface_node(surface), index);
    w.int(links.node_id);
    w.ptr(0);
    w.ptr(links.owner);
    w.ptr(links.next);
    w.ptr(links.prev);
    w.ptr(links.geometric_owner);
    w.ch(links.sense);
    match *surface {
        Surface::Plane {
            origin,
            normal,
            x_axis,
        } => {
            w.pos(origin);
            w.dir(normal);
            w.dir(x_axis);
        }
        Surface::Cylinder {
            base,
            axis,
            radius,
            ref_dir,
        } => {
            w.pos(base);
            w.dir(axis);
            w.dist(radius);
            w.dir(ref_dir);
        }
        Surface::Cone {
            pvec,
            axis,
            radius,
            half_angle,
            ref_dir,
        } => {
            let (sin_half, cos_half) = (half_angle as f64).sin_cos();
            assert!(
                (sin_half * sin_half + cos_half * cos_half - 1.0).abs() <= math_unit_residue(),
                "a CONE node's half-angle sine and cosine are one f64 angle's, so their squares \
                 sum to one: sin {sin_half}, cos {cos_half}"
            );
            w.pos(pvec);
            w.dir(axis);
            w.dist(radius);
            w.real(sin_half);
            w.real(cos_half);
            w.dir(ref_dir);
        }
        Surface::Sphere {
            center,
            axis,
            radius,
            ref_dir,
        } => {
            w.pos(center);
            w.dist(radius);
            w.dir(axis);
            w.dir(ref_dir);
        }
        Surface::Torus {
            center,
            axis,
            major_r,
            minor_r,
            ref_dir,
        } => {
            w.pos(center);
            w.dir(axis);
            w.dist(major_r);
            w.dist(minor_r);
            w.dir(ref_dir);
        }
    }
}

/// The writer's own bound on an emitted `f64` identity, re-exported here so the
/// CONE half-angle check states it in the same terms every direction does.
fn math_unit_residue() -> f64 {
    crate::kernel::math::UNIT_RESIDUE
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
        Curve::Ellipse {
            center,
            axis,
            x_axis,
            major,
            minor,
        } => Some(XtCurve::Ellipse {
            centre: center,
            normal: axis,
            x_axis,
            major,
            minor,
        }),
        Curve::TorusSection { .. } => None,
    }
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
