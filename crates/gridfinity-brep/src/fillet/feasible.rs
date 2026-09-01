//! What a rolling-ball blend can be asked for, decided in 2D before any of it
//! is built.
//!
//! `fillet_edges` works on a solid and reports a refusal only after it has tried
//! to build the blend surface, by which point the message names a face rather
//! than the request that was impossible. These four answer the same questions
//! from the profile the caller is about to extrude: how wide a ball fits between
//! two sides of a passage (`max_inward_radius`), whether an island leaves the
//! outer boundary room for both their blends (`island_clears`), which of a
//! loop's segments a chain may run along without crossing a corner it has no
//! tangent through (`blendable_segs`), and how far a radius must stay from the
//! radius of the arc it rolls along before the torus degenerates
//! (`blend_radius_along`).
//!
//! Every answer is an upper bound that errs towards allowing the request, so
//! passing them is necessary and not sufficient; `fillet_edges` remains the
//! authority on what actually builds.

use crate::math::Vec2;
use crate::region2d::{loops_within, point_seg_distance, seg_seg_points};
use crate::round::{seg_samples, sharp_between};
use crate::sketch::{Seg, loop_area};

/// A rolling-ball blend along an arc builds a torus whose major radius is the
/// gap between the arc and the ball, so equal radii put the blend's centre on
/// the arc's own axis and the torus degenerates to a ring -- which
/// `build_torus_blend` asserts against. Every radius handed to `fillet_edges`
/// keeps at least this much clearance from the arcs it rolls along.
pub const MIN_TORUS_MAJOR: f64 = 0.1;

/// `want`, pulled clear of `seg`'s own radius if a blend that size would
/// degenerate on it. A straight segment constrains nothing and returns `want`
/// unchanged; an arc returns at most `radius - MIN_TORUS_MAJOR`, which goes
/// small or negative on a tight arc and every caller reads as "leave this edge
/// sharp".
pub fn blend_radius_along(seg: &Seg, want: f64) -> f64 {
    match *seg {
        Seg::Arc { radius, .. } if (radius - want).abs() < MIN_TORUS_MAJOR => {
            radius - MIN_TORUS_MAJOR
        }
        _ => want,
    }
}

/// Whether `island` stands far enough inside `outer` for a blend of `needed`
/// total radius to roll between them -- the sum of the two radii where both are
/// blended, one radius where only one is. A non-positive `needed` asks for no
/// blend and always clears.
pub fn island_clears(island: &[Seg], outer: &[Seg], needed: f64) -> bool {
    needed <= 0.0 || !loops_within(island, outer, needed)
}

/// Which of a loop's segments a rolling-ball blend may run along, given which
/// ones the caller allows at all.
///
/// A blend chain has to stay tangent-continuous, because a vertex with two
/// blended edges *continues* the chain and joining two blends that do not share
/// a tangent there leaves a gap the size of the two radii. A sharp corner has to
/// terminate the chain instead, which costs one of its two segments and turns
/// the vertex into a runout `fillet` can close off. It costs one segment, not
/// the whole loop: an opening's pinch leaves sharp corners that used to delete
/// every fillet on the compartment.
pub fn blendable_segs(shape: &[Seg], allow: &[bool]) -> Vec<bool> {
    assert_eq!(
        shape.len(),
        allow.len(),
        "the allowed set names one segment of the loop each, got {} for {} segment(s)",
        allow.len(),
        shape.len()
    );
    let n = shape.len();
    let mut keep = allow.to_vec();
    for i in 0..n {
        let j = (i + 1) % n;
        if keep[i] && keep[j] && sharp_between(shape, i, j) {
            keep[j] = false;
        }
    }
    keep
}

/// The largest rolling-ball radius the inside of `segs` can carry.
///
/// A ball of radius `r` rolling along the boundary touches the floor `r` from
/// it, so across a passage `w` wide the touchdowns from the two sides cross as
/// soon as `r > w / 2` and the filleted floor's own boundary self-intersects --
/// which the kernel can only report after building the whole blend, as
/// `face N's boundary crosses itself`. The radius is impossible, so the caller
/// should never ask for it.
///
/// `w` is measured by casting a ray *inward* from points along the boundary and
/// taking the first crossing: that is the width through the **interior**, which
/// is what the ball has to fit in. Inward is the left of travel on a
/// counter-clockwise loop and the right on a clockwise one, read off the
/// winding. Taking the distance between nearby segments instead would clamp on a
/// thin finger of material -- its two sides are close, but the ball rolls around
/// the outside of it and nothing is in its way.
///
/// The ray decides **which** boundary is across the passage; the width itself is
/// then the distance from the sample to that whole segment, not the length of
/// the ray. The two differ whenever the ray leaves at anything but a right angle
/// to what it hits, and the difference is one-sided in the wrong direction: the
/// case that found this fires a ray off a wall finger's tip that is tilted 2°
/// against the cavity wall it crosses to, measuring a 3.0496 mm gap as 3.0518
/// and passing a 1.5259 mm radius where 1.5248 fits. Halving the distance to the
/// segment is exact for the disc that touches both, which is what a rolling ball
/// in the passage is.
///
/// Samples are the true segments, arcs included, never a chord approximation:
/// both endpoints of every segment plus interior points about `STEP` apart. The
/// endpoints matter -- a rounded finger tip's nearest point to the wall opposite
/// is the corner where its end cap meets its side, and interior sampling walks
/// straight past it.
///
/// Sampling can only miss a narrow spot, never invent one, so the bound errs
/// towards leaving the radius alone. It is an upper bound and not a guarantee:
/// a passage that narrows between samples still gets through.
pub fn max_inward_radius(segs: &[Seg]) -> f64 {
    const STEP: f64 = 0.5;
    const EPS: f64 = 1e-3;
    const REACH: f64 = 1e3;

    let area = loop_area(segs);
    assert!(
        area != 0.0,
        "fillet width: a loop of {} segment(s) encloses no area, so it has no interior for a \
         ball to roll in, and the inward direction every ray needs is the sign of noise",
        segs.len()
    );
    let ccw = area > 0.0;
    if segs.len() < 2 {
        return f64::INFINITY;
    }

    let mut best = f64::INFINITY;
    for s in segs {
        for (p, along) in seg_samples(s, STEP) {
            let inward = if ccw {
                Vec2::new(-along.y, along.x)
            } else {
                Vec2::new(along.y, -along.x)
            };
            let ray = Seg::Line {
                a: p,
                b: p + inward * REACH,
            };
            for other in segs {
                if !seg_seg_points(&ray, other)
                    .iter()
                    .any(|hit| (*hit - p).length() > EPS)
                {
                    continue;
                }
                let d = point_seg_distance(p, other);
                if d > EPS && d < best {
                    best = d;
                }
            }
        }
    }
    best / 2.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::round::loop_of_points;

    fn rect(w: f64, h: f64) -> Vec<Seg> {
        loop_of_points(&[
            Vec2::new(0.0, 0.0),
            Vec2::new(w, 0.0),
            Vec2::new(w, h),
            Vec2::new(0.0, h),
        ])
    }

    #[test]
    fn the_widest_ball_in_a_slot_is_half_its_narrow_dimension() {
        let r = max_inward_radius(&rect(40.0, 6.0));
        assert!(
            (r - 3.0).abs() < 1e-3,
            "a 6 mm slot carries a 3 mm rolling ball, got {r}"
        );
    }

    #[test]
    fn the_bound_reads_the_narrow_axis_whichever_it_is() {
        let a = max_inward_radius(&rect(40.0, 6.0));
        let b = max_inward_radius(&rect(6.0, 40.0));
        assert!(
            (a - b).abs() < 1e-3,
            "the passage is the same passage turned ninety degrees, got {a} and {b}"
        );
    }

    #[test]
    fn a_clockwise_loop_measures_its_own_interior_not_the_plane_outside_it() {
        let mut cw = rect(40.0, 6.0);
        cw.reverse();
        let cw: Vec<Seg> = cw
            .into_iter()
            .map(|s| match s {
                Seg::Line { a, b } => Seg::Line { a: b, b: a },
                other => other,
            })
            .collect();
        assert!(loop_area(&cw) < 0.0, "the fixture is wound clockwise");
        let r = max_inward_radius(&cw);
        assert!(
            (r - 3.0).abs() < 1e-3,
            "inward is read off the winding, so a reversed slot measures the same 3 mm, got {r}"
        );
    }

    #[test]
    fn a_blend_along_an_arc_keeps_clear_of_the_arcs_own_radius() {
        let arc = Seg::Arc {
            a: Vec2::new(2.0, 0.0),
            b: Vec2::new(0.0, 2.0),
            center: Vec2::ZERO,
            radius: 2.0,
            a0: 0.0,
            a1: std::f64::consts::FRAC_PI_2,
        };
        assert!(
            (blend_radius_along(&arc, 2.0) - (2.0 - MIN_TORUS_MAJOR)).abs() < 1e-6,
            "a radius equal to the arc's degenerates and is pulled clear"
        );
        assert!(
            (blend_radius_along(&arc, 0.5) - 0.5).abs() < 1e-6,
            "a radius well clear of the arc's is left alone"
        );
        let line = Seg::Line {
            a: Vec2::ZERO,
            b: Vec2::new(1.0, 0.0),
        };
        assert!(
            (blend_radius_along(&line, 9.0) - 9.0).abs() < 1e-6,
            "a straight segment constrains nothing"
        );
    }

    #[test]
    fn a_chain_terminates_at_a_sharp_corner_and_costs_one_segment() {
        let sq = rect(20.0, 20.0);
        let keep = blendable_segs(&sq, &[true; 4]);
        assert_eq!(
            keep.iter().filter(|k| **k).count(),
            2,
            "a square is four sharp corners, so alternate segments survive: {keep:?}"
        );
    }
}
