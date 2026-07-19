//! Exact 2D boolean algebra on seg-loop regions.
//!
//! A region is a set of closed `Seg` loops: outers CCW (`loop_area > 0`),
//! holes CW — so every run has material on its left.
//! [`region_union`] / [`region_difference`] / [`region_intersection`] combine
//! two of them; [`split_regions`] exposes the classified pieces underneath,
//! each carrying a caller-supplied provenance tag so results are identified
//! exactly rather than by comparing coordinates.
//!
//! Every intersection is closed form — line/line by determinant, line/circle
//! by the quadratic, circle/circle by the radical line — and each cut point is
//! computed once and shared verbatim by both sides, so the selected pieces
//! chain into closed loops exactly and weld in the B-rep builder later.
//!
//! [`presplit_regions`] gives several booleans over the same inputs one common
//! segmentation, which is what stops independently-computed results from
//! T-junctioning against each other.
//!
//! Degenerate inputs (collinear-overlapping edges, tangencies, concentric
//! arcs) are not supported — callers author generic positions.

use crate::kernel::math::Vec2;
use crate::kernel::sketch::{Seg, loop_area, point_in_segs};

const EPS: f32 = 1e-4;

// ── General region/region boolean ────────────────────────────────────────────

/// Both regions' boundaries split at their mutual intersections, with every
/// piece classified by whether it lies inside the *other* region. Cut points
/// are computed once and shared verbatim by both sides, so the selected
/// pieces chain into closed loops exactly.
pub struct RegionSplit<T> {
    pub a_outside: Vec<(Seg, T)>,
    pub a_inside: Vec<(Seg, T)>,
    pub b_outside: Vec<(Seg, T)>,
    pub b_inside: Vec<(Seg, T)>,
}

/// Split two regions (outers CCW, holes CW) against each other. This is the
/// general form of [`split_region_by_quad`]: either operand may have any
/// number of loops made of lines and arcs.
pub fn split_regions<T: Copy>(a: &[Vec<(Seg, T)>], b: &[Vec<(Seg, T)>]) -> RegionSplit<T> {
    let mut a_cuts: Vec<Vec<Vec<(f32, Vec2)>>> =
        a.iter().map(|l| vec![Vec::new(); l.len()]).collect();
    let mut b_cuts: Vec<Vec<Vec<(f32, Vec2)>>> =
        b.iter().map(|l| vec![Vec::new(); l.len()]).collect();
    for (ai, al) in a.iter().enumerate() {
        for (asi, (aseg, _)) in al.iter().enumerate() {
            for (bi, bl) in b.iter().enumerate() {
                for (bsi, (bseg, _)) in bl.iter().enumerate() {
                    for pt in seg_seg_points(aseg, bseg) {
                        a_cuts[ai][asi].push((seg_param(aseg, pt), pt));
                        b_cuts[bi][bsi].push((seg_param(bseg, pt), pt));
                    }
                }
            }
        }
    }

    let bare = |r: &[Vec<(Seg, T)>]| -> Vec<Vec<Seg>> {
        r.iter().map(|l| l.iter().map(|&(s, _)| s).collect()).collect()
    };
    let (a_bare, b_bare) = (bare(a), bare(b));
    // Even–odd: inside iff contained by an odd number of loops (an outer,
    // minus its holes).
    let inside = |loops: &[Vec<Seg>], p: Vec2| {
        loops.iter().filter(|l| point_in_segs(p, l)).count() % 2 == 1
    };

    let mut out = RegionSplit {
        a_outside: Vec::new(),
        a_inside: Vec::new(),
        b_outside: Vec::new(),
        b_inside: Vec::new(),
    };
    for (li, lp) in a.iter().enumerate() {
        for (si, &(ref seg, tag)) in lp.iter().enumerate() {
            let mut cuts = std::mem::take(&mut a_cuts[li][si]);
            for piece in split_seg(seg, &mut cuts) {
                if inside(&b_bare, seg_mid(&piece)) {
                    out.a_inside.push((piece, tag));
                } else {
                    out.a_outside.push((piece, tag));
                }
            }
        }
    }
    for (li, lp) in b.iter().enumerate() {
        for (si, &(ref seg, tag)) in lp.iter().enumerate() {
            let mut cuts = std::mem::take(&mut b_cuts[li][si]);
            for piece in split_seg(seg, &mut cuts) {
                if inside(&a_bare, seg_mid(&piece)) {
                    out.b_inside.push((piece, tag));
                } else {
                    out.b_outside.push((piece, tag));
                }
            }
        }
    }
    out
}

/// Split every seg of every region at its crossings with every *other*
/// region, so all of them end up sharing one common segmentation.
///
/// Needed whenever several booleans over the same inputs must agree on where
/// segment endpoints fall — e.g. the slab engine, where a boundary run that
/// survives into two adjacent z-bands has to come back as the identical
/// pieces in both, or the shared vertical face would T-junction against its
/// neighbour instead of pairing 1:1.
pub fn presplit_regions(regions: &[Vec<Vec<Seg>>]) -> Vec<Vec<Vec<Seg>>> {
    let mut cuts: Vec<Vec<Vec<Vec<(f32, Vec2)>>>> = regions
        .iter()
        .map(|r| r.iter().map(|l| vec![Vec::new(); l.len()]).collect())
        .collect();
    for (ri, r) in regions.iter().enumerate() {
        for (li, l) in r.iter().enumerate() {
            for (si, s) in l.iter().enumerate() {
                for (rj, r2) in regions.iter().enumerate() {
                    if ri == rj {
                        continue;
                    }
                    for l2 in r2 {
                        for s2 in l2 {
                            for pt in seg_seg_points(s, s2) {
                                cuts[ri][li][si].push((seg_param(s, pt), pt));
                            }
                        }
                    }
                }
            }
        }
    }
    regions
        .iter()
        .enumerate()
        .map(|(ri, r)| {
            r.iter()
                .enumerate()
                .map(|(li, l)| {
                    l.iter()
                        .enumerate()
                        .flat_map(|(si, s)| {
                            let mut c = std::mem::take(&mut cuts[ri][li][si]);
                            split_seg(s, &mut c)
                        })
                        .collect()
                })
                .collect()
        })
        .collect()
}

fn tagged(loops: &[Vec<Seg>]) -> Vec<Vec<(Seg, ())>> {
    loops.iter().map(|l| l.iter().map(|&s| (s, ())).collect()).collect()
}

fn untag(loops: Vec<Vec<(Seg, ())>>) -> Vec<Vec<Seg>> {
    loops.into_iter().map(|l| l.into_iter().map(|(s, _)| s).collect()).collect()
}

/// `A ∪ B`: the boundary runs where either region's boundary is outside the
/// other. Disjoint operands come back as separate loops and a contained
/// operand simply vanishes — both fall out of the classification, no
/// special-casing.
pub fn region_union(a: &[Vec<Seg>], b: &[Vec<Seg>]) -> Vec<Vec<Seg>> {
    let s = split_regions(&tagged(a), &tagged(b));
    let mut kept = s.a_outside;
    kept.extend(s.b_outside);
    untag(chain_loops(kept))
}

/// `A − B`: A's boundary outside B, plus B's boundary inside A traversed
/// backwards (so a fully contained B becomes a hole).
pub fn region_difference(a: &[Vec<Seg>], b: &[Vec<Seg>]) -> Vec<Vec<Seg>> {
    let s = split_regions(&tagged(a), &tagged(b));
    let mut kept = s.a_outside;
    kept.extend(s.b_inside.iter().map(|&(p, t)| (p.reversed(), t)));
    untag(chain_loops(kept))
}

/// `A ∩ B`: the parts of each boundary lying inside the other.
pub fn region_intersection(a: &[Vec<Seg>], b: &[Vec<Seg>]) -> Vec<Vec<Seg>> {
    let s = split_regions(&tagged(a), &tagged(b));
    let mut kept = s.a_inside;
    kept.extend(s.b_inside);
    untag(chain_loops(kept))
}

/// Param of `p` along the line a→b.
fn line_param(a: Vec2, b: Vec2, p: Vec2) -> f32 {
    let d = b - a;
    let l2 = d.length_squared();
    if l2 <= 0.0 { 0.0 } else { (p - a).dot(d) / l2 }
}

// ── Seg/seg intersection ─────────────────────────────────────────────────────

/// Parameter of `p` along `seg` in `split_seg`'s convention: `t in [0,1]` for
/// a line, unwrapped angle for an arc.
fn seg_param(seg: &Seg, p: Vec2) -> f32 {
    match *seg {
        Seg::Line { a, b } => line_param(a, b, p),
        Seg::Arc { center, a0, a1, .. } => {
            let (lo, hi) = (a0.min(a1), a0.max(a1));
            let mut th = (p.y - center.y).atan2(p.x - center.x);
            while th < lo - 1e-6 {
                th += std::f32::consts::TAU;
            }
            while th > hi + 1e-6 {
                th -= std::f32::consts::TAU;
            }
            th
        }
    }
}

/// Does `p` lie on `seg` (within its span, endpoints included)?
fn on_seg(seg: &Seg, p: Vec2) -> bool {
    match *seg {
        Seg::Line { a, b } => {
            let t = line_param(a, b, p);
            (-EPS..=1.0 + EPS).contains(&t) && (a + (b - a) * t - p).length() < 1e-3
        }
        Seg::Arc { center, radius, a0, a1, .. } => {
            if ((p - center).length() - radius).abs() > 1e-3 {
                return false;
            }
            let (lo, hi) = (a0.min(a1), a0.max(a1));
            let th = seg_param(seg, p);
            (lo - 1e-6..=hi + 1e-6).contains(&th)
        }
    }
}

/// Both intersections of a circle and an infinite line, unfiltered.
fn circle_line_pts(center: Vec2, radius: f32, q0: Vec2, q1: Vec2) -> Vec<Vec2> {
    let e = q1 - q0;
    let f = q0 - center;
    let qa = e.length_squared();
    let qb = 2.0 * f.dot(e);
    let qc = f.length_squared() - radius * radius;
    let disc = qb * qb - 4.0 * qa * qc;
    if disc <= 0.0 || qa <= 0.0 {
        return Vec::new();
    }
    let sq = disc.sqrt();
    [(-qb - sq) / (2.0 * qa), (-qb + sq) / (2.0 * qa)].into_iter().map(|u| q0 + e * u).collect()
}

/// Circle/circle by the radical line: the intersections lie on the chord
/// perpendicular to the centre line at distance `a` from the first centre.
/// Concentric or tangent-within-EPS inputs yield nothing.
fn circle_circle_pts(c0: Vec2, r0: f32, c1: Vec2, r1: f32) -> Vec<Vec2> {
    let d = (c1 - c0).length();
    if d < 1e-9 || d > r0 + r1 - EPS || d < (r0 - r1).abs() + EPS {
        return Vec::new();
    }
    let a = (r0 * r0 - r1 * r1 + d * d) / (2.0 * d);
    let h2 = r0 * r0 - a * a;
    if h2 <= 0.0 {
        return Vec::new();
    }
    let h = h2.sqrt();
    let u = (c1 - c0) / d;
    let base = c0 + u * a;
    let perp = Vec2::new(-u.y, u.x);
    vec![base + perp * h, base - perp * h]
}

/// Every point where two segs cross, closed form in all four combinations and
/// filtered to both spans.
fn seg_seg_points(p: &Seg, q: &Seg) -> Vec<Vec2> {
    let raw = match (*p, *q) {
        (Seg::Line { a, b }, Seg::Line { a: c, b: d }) => {
            let (e, f) = (b - a, d - c);
            let den = e.x * f.y - e.y * f.x;
            if den.abs() < 1e-9 {
                return Vec::new(); // parallel
            }
            let w = c - a;
            vec![a + e * ((w.x * f.y - w.y * f.x) / den)]
        }
        (Seg::Arc { center, radius, .. }, Seg::Line { a, b })
        | (Seg::Line { a, b }, Seg::Arc { center, radius, .. }) => {
            circle_line_pts(center, radius, a, b)
        }
        (Seg::Arc { center: c0, radius: r0, .. }, Seg::Arc { center: c1, radius: r1, .. }) => {
            circle_circle_pts(c0, r0, c1, r1)
        }
    };
    raw.into_iter().filter(|&pt| on_seg(p, pt) && on_seg(q, pt)).collect()
}

fn seg_mid(seg: &Seg) -> Vec2 {
    match *seg {
        Seg::Line { a, b } => (a + b) * 0.5,
        Seg::Arc { center, radius, a0, a1, .. } => {
            let t = (a0 + a1) * 0.5;
            center + Vec2::new(t.cos(), t.sin()) * radius
        }
    }
}

/// Split a seg at the given `(param, point)` cuts (param = t∈[0,1] for lines,
/// unwrapped angle for arcs). Cut points become sub-seg endpoints verbatim so
/// both sides of a split share exact coordinates.
fn split_seg(seg: &Seg, cuts: &mut Vec<(f32, Vec2)>) -> Vec<Seg> {
    match *seg {
        Seg::Line { a, b } => {
            cuts.retain(|&(t, _)| (EPS..=1.0 - EPS).contains(&t));
            cuts.sort_by(|x, y| x.0.total_cmp(&y.0));
            cuts.dedup_by(|x, y| (x.0 - y.0).abs() < EPS);
            let mut out = Vec::with_capacity(cuts.len() + 1);
            let mut prev = a;
            for &(_, p) in cuts.iter() {
                out.push(Seg::Line { a: prev, b: p });
                prev = p;
            }
            out.push(Seg::Line { a: prev, b });
            out
        }
        Seg::Arc { a, b, center, radius, a0, a1 } => {
            let fwd = a1 >= a0;
            let (lo, hi) = (a0.min(a1), a0.max(a1));
            let span = (hi - lo).max(1e-9);
            let aeps = EPS.max(1e-3 / radius.max(1e-3)); // ≈ EPS of arc length
            cuts.retain(|&(t, _)| t > lo + aeps * span.min(1.0) && t < hi - aeps * span.min(1.0));
            if fwd {
                cuts.sort_by(|x, y| x.0.total_cmp(&y.0));
            } else {
                cuts.sort_by(|x, y| y.0.total_cmp(&x.0));
            }
            cuts.dedup_by(|x, y| (x.0 - y.0).abs() < aeps);
            let mut out = Vec::with_capacity(cuts.len() + 1);
            let mut prev = (a0, a);
            for &(t, p) in cuts.iter() {
                out.push(Seg::Arc { a: prev.1, b: p, center, radius, a0: prev.0, a1: t });
                prev = (t, p);
            }
            out.push(Seg::Arc { a: prev.1, b, center, radius, a0: prev.0, a1 });
            out
        }
    }
}

/// Chain sub-segs into closed loops by endpoint proximity (endpoints of
/// matching pieces are bit-identical by construction; the tolerance only
/// bridges float noise in the rare mixed case). Provenance tags ride along.
pub fn chain_loops<T: Copy>(mut segs: Vec<(Seg, T)>) -> Vec<Vec<(Seg, T)>> {
    let mut out: Vec<Vec<(Seg, T)>> = Vec::new();
    while let Some(first) = segs.pop() {
        let start = first.0.start();
        let mut lp = vec![first];
        loop {
            let cur = lp.last().unwrap().0.end();
            if (cur - start).length() < EPS && lp.len() >= 2 {
                break;
            }
            let Some((idx, _)) = segs
                .iter()
                .enumerate()
                .map(|(i, (s, _))| (i, (s.start() - cur).length()))
                .filter(|&(_, d)| d < EPS)
                .min_by(|x, y| x.1.total_cmp(&y.1))
            else {
                break; // open chain: drop (degenerate input)
            };
            lp.push(segs.swap_remove(idx));
        }
        if (lp.last().unwrap().0.end() - start).length() < EPS && lp.len() >= 2 {
            // Drop zero-area slivers.
            let bare: Vec<Seg> = lp.iter().map(|&(s, _)| s).collect();
            if loop_area(&bare).abs() > EPS {
                out.push(lp);
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kernel::sketch::Sketch;

    fn area(loops: &[Vec<Seg>]) -> f32 {
        loops.iter().map(|l| loop_area(l)).sum()
    }

    fn rect(x0: f32, y0: f32, x1: f32, y1: f32) -> Vec<Seg> {
        Sketch::rectangle((x0 + x1) * 0.5, (y0 + y1) * 0.5, x1 - x0, y1 - y0).loops.remove(0)
    }

    /// area(A) + area(B) == area(A∪B) + area(A∩B), for any A and B. The
    /// strongest single check: it catches a dropped piece, a mis-signed loop
    /// and a mis-classified span all at once.
    fn assert_conserved(a: &[Vec<Seg>], b: &[Vec<Seg>]) {
        let lhs = area(a) + area(b);
        let rhs = area(&region_union(a, b)) + area(&region_intersection(a, b));
        assert!((lhs - rhs).abs() < 1e-2, "area not conserved: {lhs} vs {rhs}");
    }

    #[test]
    fn overlapping_rects() {
        let a = vec![rect(0.0, 0.0, 10.0, 10.0)];
        let b = vec![rect(5.0, 5.0, 15.0, 15.0)];
        assert!((area(&region_intersection(&a, &b)) - 25.0).abs() < 1e-3);
        assert!((area(&region_union(&a, &b)) - 175.0).abs() < 1e-3);
        assert!((area(&region_difference(&a, &b)) - 75.0).abs() < 1e-3);
        assert_conserved(&a, &b);
    }

    #[test]
    fn disjoint_rects_stay_separate() {
        let a = vec![rect(0.0, 0.0, 10.0, 10.0)];
        let b = vec![rect(20.0, 20.0, 30.0, 30.0)];
        let u = region_union(&a, &b);
        assert_eq!(u.len(), 2, "disjoint union should keep two loops");
        assert!(region_intersection(&a, &b).is_empty());
        assert!((area(&region_difference(&a, &b)) - 100.0).abs() < 1e-3);
        assert_conserved(&a, &b);
    }

    #[test]
    fn containment_both_directions() {
        let big = vec![rect(0.0, 0.0, 20.0, 20.0)];
        let small = vec![rect(5.0, 5.0, 10.0, 10.0)];
        // Union is just the big one; intersection is just the small one.
        assert!((area(&region_union(&big, &small)) - 400.0).abs() < 1e-3);
        assert!((area(&region_union(&small, &big)) - 400.0).abs() < 1e-3);
        assert!((area(&region_intersection(&big, &small)) - 25.0).abs() < 1e-3);
        // Subtracting an interior region punches a hole: two loops, net area.
        let d = region_difference(&big, &small);
        assert_eq!(d.len(), 2, "difference should be outer + hole");
        assert!((area(&d) - 375.0).abs() < 1e-3, "hole must subtract");
        assert_conserved(&big, &small);
    }

    #[test]
    fn rounded_rect_against_circle_uses_arc_arc() {
        // Geometry chosen so the cut really lands on the corner ARC, not on the
        // straight edges: the rect's corner arc is centred (5,5) r=5, and this
        // circle swallows that arc's midpoint (8.54, 8.54) while leaving both
        // its endpoints (10,5) and (5,10) outside — so it must cross the arc
        // twice. (An earlier version used circle (10,10) r=6, which crosses the
        // corner's *circle* outside the quarter-arc's span and so silently
        // exercised only circle/line.)
        let a = vec![Sketch::rounded_rect(0.0, 0.0, 20.0, 20.0, 5.0).loops.remove(0)];
        let b = vec![Sketch::circle(11.0, 11.0, 4.0).loops.remove(0)];
        let corner = Seg::Arc {
            a: Vec2::new(10.0, 5.0),
            b: Vec2::new(5.0, 10.0),
            center: Vec2::new(5.0, 5.0),
            radius: 5.0,
            a0: 0.0,
            a1: std::f32::consts::FRAC_PI_2,
        };
        // `Sketch::circle` is two semicircle arcs, so scan both.
        let hits: usize = b[0].iter().map(|s| seg_seg_points(&corner, s).len()).sum();
        assert_eq!(hits, 2, "arc/arc path must cross the corner arc exactly twice");
        let i = region_intersection(&a, &b);
        assert!(!i.is_empty(), "arc/arc intersection produced nothing");
        assert!(area(&i) > 0.0 && area(&i) < std::f32::consts::PI * 16.0);
        assert_conserved(&a, &b);
    }

    #[test]
    fn circle_hole_in_rect() {
        let a = vec![rect(0.0, 0.0, 20.0, 20.0)];
        let b = vec![Sketch::circle(10.0, 10.0, 4.0).loops.remove(0)];
        let d = region_difference(&a, &b);
        let expect = 400.0 - std::f32::consts::PI * 16.0;
        assert!((area(&d) - expect).abs() < 1e-2, "bore area {} vs {expect}", area(&d));
    }
}

/// Does `ang` (any branch) fall inside the sweep `a0 -> a1`?
fn arc_covers(a0: f32, a1: f32, ang: f32) -> bool {
    let (lo, hi) = (a0.min(a1), a0.max(a1));
    let two_pi = std::f32::consts::TAU;
    // Walk `ang` into [lo, lo + 2pi) and compare.
    let k = ((lo - ang) / two_pi).ceil();
    ang + k * two_pi <= hi + 1e-6
}

/// Exact distance from a point to one segment.
pub fn point_seg_distance(p: Vec2, seg: &Seg) -> f32 {
    match *seg {
        Seg::Line { a, b } => {
            let d = b - a;
            let l2 = d.dot(d);
            if l2 < 1e-12 {
                return (p - a).length();
            }
            let t = ((p - a).dot(d) / l2).clamp(0.0, 1.0);
            (p - (a + d * t)).length()
        }
        Seg::Arc { a, b, center, radius, a0, a1 } => {
            let v = p - center;
            if v.length() > 1e-9 && arc_covers(a0, a1, f32::atan2(v.y, v.x)) {
                // Foot of the radial projection is on the arc.
                (v.length() - radius).abs()
            } else {
                (p - a).length().min((p - b).length())
            }
        }
    }
}

/// Points on `seg` that can realise a closest approach to another segment
/// without being an endpoint: for an arc, the two points radially aligned with
/// `toward` (a line normal, or the other arc's centre direction).
fn extremal_points(seg: &Seg, toward: Vec2) -> Vec<Vec2> {
    match *seg {
        Seg::Line { .. } => Vec::new(),
        Seg::Arc { center, radius, a0, a1, .. } => {
            if toward.length() < 1e-9 {
                return Vec::new();
            }
            let u = toward.normalize();
            [u, -u]
                .iter()
                .map(|&d| center + d * radius)
                .filter(|q| {
                    let v = *q - center;
                    arc_covers(a0, a1, f32::atan2(v.y, v.x))
                })
                .collect()
        }
    }
}

/// Exact minimum distance between two segments. Zero when they cross.
pub fn seg_seg_distance(p: &Seg, q: &Seg) -> f32 {
    if !seg_seg_points(p, q).is_empty() {
        return 0.0;
    }
    // Endpoints of each against the other. For a non-crossing pair whose
    // closest approach is interior to both, the extremal points below supply
    // the missing candidate; every candidate is a genuine point on its curve,
    // so taking the minimum can only tighten toward the true distance.
    let mut best = f32::INFINITY;
    for (x, y) in [(p, q), (q, p)] {
        for e in [x.start(), x.end()] {
            best = best.min(point_seg_distance(e, y));
        }
    }
    let toward = |s: &Seg, other: &Seg| -> Vec2 {
        match (*s, *other) {
            // Closest approach of a circle to a line is along the line normal.
            (Seg::Arc { .. }, Seg::Line { a, b }) => {
                let d = b - a;
                Vec2::new(-d.y, d.x)
            }
            // ...and of two circles, along the line joining their centres.
            (Seg::Arc { center: c0, .. }, Seg::Arc { center: c1, .. }) => c1 - c0,
            _ => Vec2::ZERO,
        }
    };
    for (x, y) in [(p, q), (q, p)] {
        for e in extremal_points(x, toward(x, y)) {
            best = best.min(point_seg_distance(e, y));
        }
    }
    best
}

/// Exact minimum distance between two closed loops, measured between their
/// *boundaries* — a loop entirely inside another still reports the gap to it,
/// not zero.
pub fn min_loop_distance(a: &[Seg], b: &[Seg]) -> f32 {
    let mut best = f32::INFINITY;
    for p in a {
        for q in b {
            best = best.min(seg_seg_distance(p, q));
            if best == 0.0 {
                return 0.0;
            }
        }
    }
    best
}

#[cfg(test)]
mod distance_tests {
    use super::*;
    use crate::kernel::sketch::Sketch;
    use std::f32::consts::PI;

    fn line(ax: f32, ay: f32, bx: f32, by: f32) -> Seg {
        Seg::Line { a: Vec2::new(ax, ay), b: Vec2::new(bx, by) }
    }

    #[test]
    fn parallel_lines_measure_their_gap() {
        let d = seg_seg_distance(&line(0.0, 0.0, 10.0, 0.0), &line(0.0, 3.0, 10.0, 3.0));
        assert!((d - 3.0).abs() < 1e-5, "{d}");
    }

    #[test]
    fn crossing_segments_are_zero() {
        let d = seg_seg_distance(&line(0.0, 0.0, 10.0, 0.0), &line(5.0, -1.0, 5.0, 1.0));
        assert_eq!(d, 0.0);
    }

    /// Offset segments must measure end-to-end, not perpendicular.
    #[test]
    fn disjoint_collinear_segments_measure_the_gap() {
        let d = seg_seg_distance(&line(0.0, 0.0, 4.0, 0.0), &line(7.0, 0.0, 10.0, 0.0));
        assert!((d - 3.0).abs() < 1e-5, "{d}");
    }

    /// The interior-to-interior case: the closest point on the arc is not an
    /// endpoint, so it is only found via the extremal candidates.
    #[test]
    fn line_to_arc_uses_the_radial_closest_point() {
        // Upper half circle, radius 2 at the origin; the line sits above it.
        let arc = Seg::Arc {
            a: Vec2::new(2.0, 0.0),
            b: Vec2::new(-2.0, 0.0),
            center: Vec2::ZERO,
            radius: 2.0,
            a0: 0.0,
            a1: PI,
        };
        let d = seg_seg_distance(&line(-5.0, 5.0, 5.0, 5.0), &arc);
        assert!((d - 3.0).abs() < 1e-5, "want 5 - 2 = 3, got {d}");
    }

    /// Two circles' closest approach is along their centre line, again interior
    /// to both arcs.
    #[test]
    fn arc_to_arc_uses_the_centre_line() {
        let right = Seg::Arc {
            a: Vec2::new(0.0, 1.0), b: Vec2::new(0.0, -1.0),
            center: Vec2::ZERO, radius: 1.0, a0: PI / 2.0, a1: -PI / 2.0,
        };
        // Left half of the unit circle at (10, 0): pi/2 -> 3pi/2 through pi,
        // so its leftmost point (9, 0) is on the arc.
        let left = Seg::Arc {
            a: Vec2::new(10.0, 1.0), b: Vec2::new(10.0, -1.0),
            center: Vec2::new(10.0, 0.0), radius: 1.0, a0: PI / 2.0, a1: 1.5 * PI,
        };
        let d = seg_seg_distance(&right, &left);
        assert!((d - 8.0).abs() < 1e-4, "want 10 - 1 - 1 = 8, got {d}");
    }

    /// A loop strictly inside another reports the gap between boundaries, not
    /// zero — this is exactly how the island clearance test uses it.
    #[test]
    fn nested_loops_report_the_boundary_gap() {
        let outer = Sketch::rectangle(0.0, 0.0, 20.0, 20.0).loops[0].clone();
        let inner = Sketch::rectangle(0.0, 0.0, 10.0, 10.0).loops[0].clone();
        let d = min_loop_distance(&inner, &outer);
        assert!((d - 5.0).abs() < 1e-5, "want 5, got {d}");
    }

    #[test]
    fn overlapping_loops_are_zero() {
        let a = Sketch::rectangle(0.0, 0.0, 20.0, 20.0).loops[0].clone();
        let b = Sketch::rectangle(15.0, 0.0, 20.0, 20.0).loops[0].clone();
        assert_eq!(min_loop_distance(&a, &b), 0.0);
    }

    /// A rounded rect inside a plain one: the closest approach is corner arc to
    /// straight side, which only the radial candidate finds.
    #[test]
    fn rounded_island_in_a_square_measures_from_the_arc() {
        let outer = Sketch::rectangle(0.0, 0.0, 30.0, 30.0).loops[0].clone();
        let island = Sketch::rounded_rect(0.0, 0.0, 10.0, 10.0, 2.0).loops[0].clone();
        let d = min_loop_distance(&island, &outer);
        assert!((d - 10.0).abs() < 1e-5, "want 15 - 5 = 10, got {d}");
    }
}
