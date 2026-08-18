//! The one kernel curve the format has no analytic node for, written as the
//! exact intersection it is.
//!
//! `Curve::TorusSection` is a torus cut by a plane parallel to its axis -- a
//! quartic, which is why the kernel had to name it as its own variant and why XT
//! cannot name it as one of LINE, CIRCLE or ELLIPSE. XT's answer is
//! INTERSECTION: the two surfaces the curve lies on, an ordered chart of points
//! that says which branch of their intersection is meant and how it is
//! parameterised, and a limit at each end. Nothing here approximates the curve;
//! the chart is a chordal *description* of a curve the reader recomputes from
//! the two surfaces exactly.
//!
//! Two orderings have to agree with the format's own definitions or the branch
//! is misread. The chart's points run along the natural tangent, which the
//! format defines as the cross product of the two surfaces' normals *after*
//! their sense fields are applied; and the curve's sense then says whether the
//! edge runs with that tangent or against it.

use crate::kernel::geom::Curve;
use crate::kernel::math::Vec3;
use crate::kernel::xt::surf::{GeomLinks, ON_GEOMETRY_MM, XtSurface};
use crate::kernel::xt::text::{self, Index, Writer};

/// Chart points per quarter turn of the section's own parameter, and the fewest
/// a chart may have. The chart identifies a branch and carries its
/// parameterisation; it is not a tolerance on the curve, which the reader takes
/// from the surfaces, so this is a description dense enough to be unambiguous
/// rather than one fine enough to measure.
const CHART_PER_QUARTER: f32 = 8.0;
const CHART_MIN_POINTS: usize = 5;

/// An intersection curve ready to write: its chart in the natural tangent's
/// order, the limit at each end of that order, and how the edge's own direction
/// stands to it.
pub struct Intersection {
    pub chart: Vec<Vec3>,
    pub start: Vec3,
    pub end: Vec3,
    pub sense: char,
}

/// The node indices one intersection curve occupies.
pub struct Nodes {
    pub curve: Index,
    pub chart: Index,
    pub start: Index,
    pub end: Index,
}

/// The intersection curve for the edge running `t0` to `t1` along `curve`
/// between the points `ends`, lying on the two sensed surfaces `faces`.
///
/// Returns the chart ordered along the natural tangent and the sense the edge
/// stands to it, or a message naming the surface a sampled point of the curve
/// does not lie on -- which is the one thing that would make the emitted node
/// describe a different curve than the kernel built.
pub fn plan(
    curve: &Curve,
    t0: f32,
    t1: f32,
    ends: (Vec3, Vec3),
    faces: [(&XtSurface, char); 2],
) -> Result<Intersection, String> {
    assert!(
        t0.is_finite() && t1.is_finite() && (t1 - t0).abs() > 0.0,
        "an edge spans a real parameter range, got {t0} to {t1}"
    );
    let sweep = (t1 - t0).abs();
    let steps = ((sweep / std::f32::consts::FRAC_PI_2) * CHART_PER_QUARTER).ceil() as usize;
    let steps = steps.max(CHART_MIN_POINTS - 1);
    let mut chart: Vec<Vec3> = (0..=steps)
        .map(|i| curve.point(t0 + (t1 - t0) * (i as f32 / steps as f32)))
        .collect();

    for (surface, _) in faces {
        for &p in &chart {
            let d = surface.distance(p);
            if d.abs() > ON_GEOMETRY_MM {
                return Err(format!(
                    "a point of the torus section, {p:?}, stands {d} mm off the {} it is written \
                     as the intersection with",
                    surface.node_name()
                ));
            }
        }
    }

    let mid = curve.point((t0 + t1) * 0.5);
    let natural = sensed_normal(faces[0], mid).cross(sensed_normal(faces[1], mid));
    assert!(
        natural.length() > 1e-6,
        "an intersection curve's tangent is the cross product of its surfaces' normals, which \
         degenerates at {mid:?} where they meet at a cosine of {}",
        sensed_normal(faces[0], mid).dot(sensed_normal(faces[1], mid))
    );
    let natural = natural.normalize();
    let travel = (curve.tangent((t0 + t1) * 0.5) * (t1 - t0).signum()).normalize();
    let agree = natural.dot(travel);
    assert!(
        agree.abs() > 0.9,
        "the curve's own tangent {travel:?} and the intersection's natural tangent {natural:?} \
         describe the same curve and must be parallel, but meet at a cosine of {agree}"
    );

    let (start, end, sense) = if agree > 0.0 {
        (ends.0, ends.1, '+')
    } else {
        chart.reverse();
        (ends.1, ends.0, '-')
    };
    Ok(Intersection {
        chart,
        start,
        end,
        sense,
    })
}

/// A surface's normal at `p` as the format's intersection definition uses it:
/// the natural normal, reversed where the surface's own sense field reverses it.
fn sensed_normal((surface, sense): (&XtSurface, char), p: Vec3) -> Vec3 {
    let n = surface.natural_normal(p);
    if sense == '+' { n } else { -n }
}

impl Intersection {
    /// Writes the four nodes this curve occupies -- the curve, its chart and its
    /// two limits -- with `links` giving the curve node's common fields and
    /// `surfaces` the two surfaces it is the intersection of.
    pub fn write(&self, w: &mut Writer, nodes: &Nodes, links: &GeomLinks, surfaces: [Index; 2]) {
        assert_eq!(
            links.sense, self.sense,
            "an intersection curve's sense is the one its chart order settled"
        );
        w.begin(text::INTERSECTION, nodes.curve);
        w.int(links.node_id);
        w.ptr(0);
        w.ptr(links.owner);
        w.ptr(links.next);
        w.ptr(links.prev);
        w.ptr(links.geometric_owner);
        w.ch(links.sense);
        w.ptr(surfaces[0]);
        w.ptr(surfaces[1]);
        w.ptr(nodes.chart);
        w.ptr(nodes.start);
        w.ptr(nodes.end);

        w.begin_var(text::CHART, self.chart.len(), nodes.chart);
        w.real(0.0);
        w.real(1.0);
        w.int(self.chart.len() as i64);
        w.null();
        w.null();
        w.null();
        w.null();
        for &p in &self.chart {
            w.pos(p);
        }

        for (index, point) in [(nodes.start, self.start), (nodes.end, self.end)] {
            w.begin_var(text::LIMIT, 1, index);
            w.ch('L');
            w.pos(point);
        }
    }
}

/// Writes the geometric owner node at `index`, which records that `referencing`
/// depends on `shared`, chained into the ring `next` and `prev` of the other
/// owners of that same `shared` geometry.
pub fn write_geometric_owner(
    w: &mut Writer,
    index: Index,
    referencing: Index,
    next: Index,
    prev: Index,
    shared: Index,
) {
    w.begin(text::GEOMETRIC_OWNER, index);
    w.ptr(referencing);
    w.ptr(next);
    w.ptr(prev);
    w.ptr(shared);
}
