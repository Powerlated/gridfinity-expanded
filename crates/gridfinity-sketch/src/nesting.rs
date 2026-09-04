//! Which loops sit inside which, and the outer-with-holes grouping that falls
//! out of it.
//!
//! A face is an outer loop plus the loops that are holes of *it* specifically,
//! and a caller holding a bag of loops -- chained out of loose segments, or
//! returned by a boolean -- does not yet know that pairing. `containment`
//! answers it for every loop at once, and `stitch_loops_2d` is the whole journey
//! from loose segments to groups: chain, nest, and read the nesting depth's
//! parity as outer or hole, so a loop inside a hole is an outer again.
//!
//! Containment is decided by a point-in-loop test on one loop's own start point,
//! which is sound because the loops a caller brings here are disjoint except for
//! nesting -- two loops that properly cross are not a face and this says nothing
//! useful about them. The spatial bucketing exists only to keep that test off
//! every pair; it changes no answer.

use crate::hash::FxHashMap;
use crate::region2d::chain_loops;
use crate::sketch::{Aabb, Seg, point_in_segs, seg_crossings, segs_bbox};

/// Loose segments assembled into `(outer, holes)` groups.
///
/// The segments must partition into closed loops -- `chain_loops` drops any
/// chain it cannot close -- and those loops must be disjoint except for nesting.
/// A loop at even nesting depth becomes an outer, one at odd depth becomes a
/// hole of the *innermost* outer containing it, so a loop inside a hole opens a
/// new group rather than joining the one two levels out. Winding is passed
/// through untouched; grouping is by containment alone.
pub fn stitch_loops_2d(free: Vec<Seg>) -> Vec<(Vec<Seg>, Vec<Vec<Seg>>)> {
    let chained = chain_loops(free.into_iter().map(|s| (s, ())).collect());
    let loops: Vec<Vec<Seg>> = chained
        .into_iter()
        .map(|lp| lp.into_iter().map(|(s, _)| s).collect())
        .collect();
    if loops.is_empty() {
        return Vec::new();
    }
    let bbox: Vec<Aabb> = loops.iter().map(|l| segs_bbox(l)).collect();
    let containers = containment(&loops, &bbox);
    let depth = |i: usize| containers[i].len();

    let mut out: Vec<(Vec<Seg>, Vec<Vec<Seg>>)> = Vec::new();
    let mut out_idx: FxHashMap<usize, usize> = FxHashMap::default();
    for (i, lp) in loops.iter().enumerate() {
        if depth(i) % 2 == 0 {
            out_idx.insert(i, out.len());
            out.push((lp.clone(), Vec::new()));
        }
    }
    for (i, lp) in loops.iter().enumerate() {
        if depth(i) % 2 == 1 {
            let owner = *containers[i]
                .iter()
                .filter(|&&j| depth(j) % 2 == 0)
                .max_by_key(|&&j| depth(j))
                .expect("a loop at odd nesting depth is contained by an outer at even depth");
            let slot = out_idx[&owner];
            out[slot].1.push(lp.clone());
        }
    }
    out
}

/// For every loop, the indices of the loops that contain it.
///
/// A bin's bridge underside stitches into one loop per cell, and every one of
/// those has the same bounding-box area, so ordering candidates by area prunes
/// nothing and the scan is quadratic in cells. Bucketing the boxes on a uniform
/// grid keeps each query to its own neighbourhood; loops whose box spans an
/// unreasonable share of the grid are held aside and tested every time, which
/// bounds the insertion cost without losing candidates. A wide loop is tested by
/// every query, and for a whole-bin outline that is hundreds of segments each
/// time, so its segments are bucketed by the rows they span and only the handful
/// that can cross the query ray are counted.
///
/// `bbox[i]` must be `loops[i]`'s own bounding box; the two are taken as
/// parallel and the boxes are used only to reject.
pub fn containment(loops: &[Vec<Seg>], bbox: &[Aabb]) -> Vec<Vec<usize>> {
    assert_eq!(
        loops.len(),
        bbox.len(),
        "containment takes one bounding box per loop, got {} for {} loop(s)",
        bbox.len(),
        loops.len()
    );
    const MAX_CELLS: usize = 16;
    let n = loops.len();
    let all = bbox.iter().fold(Aabb::EMPTY, |a, b| a.union(*b));
    let side = (all.max - all.min).max_element();
    let k = (n as f64).sqrt().ceil().clamp(1.0, 256.0);
    let inv = if side > 0.0 { k / side } else { 0.0 };
    let (nx, ny) = (
        (((all.max.x - all.min.x) * inv) as usize + 1).min(256),
        (((all.max.y - all.min.y) * inv) as usize + 1).min(256),
    );
    let col = |x: f64| (((x - all.min.x) * inv).max(0.0) as usize).min(nx - 1);
    let row = |y: f64| (((y - all.min.y) * inv).max(0.0) as usize).min(ny - 1);

    let mut buckets: Vec<Vec<u32>> = vec![Vec::new(); nx * ny];
    let mut wide: Vec<u32> = Vec::new();
    for (j, b) in bbox.iter().enumerate() {
        let (i0, i1) = (col(b.min.x), col(b.max.x));
        let (j0, j1) = (row(b.min.y), row(b.max.y));
        if (i1 - i0 + 1) * (j1 - j0 + 1) > MAX_CELLS {
            wide.push(j as u32);
            continue;
        }
        for i in i0..=i1 {
            for r in j0..=j1 {
                buckets[i * ny + r].push(j as u32);
            }
        }
    }

    let rows: Vec<Vec<Vec<u32>>> = wide
        .iter()
        .map(|&j| {
            let mut rs: Vec<Vec<u32>> = vec![Vec::new(); ny];
            for (si, s) in loops[j as usize].iter().enumerate() {
                let b = s.bbox();
                for r in row(b.min.y)..=row(b.max.y) {
                    rs[r].push(si as u32);
                }
            }
            rs
        })
        .collect();

    (0..n)
        .map(|i| {
            let pt = loops[i][0].start();
            let mut out: Vec<usize> = buckets[col(pt.x) * ny + row(pt.y)]
                .iter()
                .map(|&j| j as usize)
                .filter(|&j| j != i && bbox[j].contains(pt) && point_in_segs(pt, &loops[j]))
                .collect();
            for (w, &j) in wide.iter().enumerate() {
                let j = j as usize;
                if j == i || !bbox[j].contains(pt) {
                    continue;
                }
                crate::perf::count(crate::perf::Metric::PointInSegs);
                let hits: u32 = rows[w][row(pt.y)]
                    .iter()
                    .map(|&si| seg_crossings(pt, &loops[j][si as usize]))
                    .sum();
                if hits % 2 == 1 {
                    out.push(j);
                }
            }
            out
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::Vec2;
    use crate::round::loop_of_points;

    fn square(x: f64, y: f64, s: f64) -> Vec<Seg> {
        loop_of_points(&[
            Vec2::new(x, y),
            Vec2::new(x + s, y),
            Vec2::new(x + s, y + s),
            Vec2::new(x, y + s),
        ])
    }

    #[test]
    fn a_ring_stitches_into_one_outer_holding_one_hole() {
        let mut free = square(0.0, 0.0, 40.0);
        free.extend(square(10.0, 10.0, 20.0));
        let groups = stitch_loops_2d(free);
        assert_eq!(groups.len(), 1, "the inner square is a hole, not a group");
        assert_eq!(groups[0].1.len(), 1, "and it is that group's only hole");
    }

    #[test]
    fn a_loop_inside_a_hole_opens_a_group_of_its_own() {
        let mut free = square(0.0, 0.0, 40.0);
        free.extend(square(5.0, 5.0, 30.0));
        free.extend(square(10.0, 10.0, 20.0));
        let groups = stitch_loops_2d(free);
        assert_eq!(
            groups.len(),
            2,
            "depth 0 and depth 2 are both outers, got {}",
            groups.len()
        );
    }

    #[test]
    fn disjoint_loops_are_separate_groups_with_no_holes() {
        let mut free = square(0.0, 0.0, 10.0);
        free.extend(square(50.0, 0.0, 10.0));
        let groups = stitch_loops_2d(free);
        assert_eq!(groups.len(), 2);
        assert!(groups.iter().all(|g| g.1.is_empty()));
    }

    #[test]
    fn a_hole_names_the_innermost_outer_that_contains_it() {
        let mut free = square(0.0, 0.0, 60.0);
        free.extend(square(5.0, 5.0, 50.0));
        free.extend(square(10.0, 10.0, 40.0));
        free.extend(square(15.0, 15.0, 30.0));
        let groups = stitch_loops_2d(free);
        assert_eq!(groups.len(), 2, "depths 0 and 2 are outers");
        assert!(
            groups.iter().all(|g| g.1.len() == 1),
            "each outer takes the hole immediately inside it, not both odd loops"
        );
    }
}
