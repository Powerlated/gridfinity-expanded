//! Rectilinear region engine: unions/differences of axis-aligned rectangles →
//! traced boundary loops with per-corner arc rounding and per-edge insets.
//!
//! This is the constructive counterpart of the reference's 2D pipeline
//! (`planCavity` rect layout + Clipper booleans/offsets): positive rects minus
//! negative rects are resolved on a compressed coordinate grid, the boundary is
//! traced with material kept on the left (outer loops CCW, holes CW), collinear
//! runs are merged, and corners are rounded with real arcs — convex corners and
//! concave corners may use different radii (the cavity uses the corner radius
//! for convex and the floor-fillet radius for concave, which keeps the
//! floor-wall blend chain tangent-continuous).
//!
//! Only axis-aligned input is supported; that is exactly the reference's cavity
//! model (cells, wall strips, divider strips, patches are all axis-aligned).

use crate::kernel::math::Vec2;
use crate::kernel::sketch::Seg;
use std::collections::HashMap;
use std::f32::consts::PI;

/// An axis-aligned rectangle (min corner + size).
#[derive(Clone, Copy, Debug)]
pub struct RectF {
    pub x: f32,
    pub y: f32,
    pub w: f32,
    pub h: f32,
}

impl RectF {
    pub fn new(x: f32, y: f32, w: f32, h: f32) -> RectF {
        RectF { x, y, w, h }
    }
}

/// One rectilinear boundary loop: corner points in traversal order (material on
/// the left ⇒ outer loops CCW, hole loops CW). Consecutive edges strictly
/// alternate horizontal/vertical.
#[derive(Clone, Debug)]
pub struct TracedLoop {
    pub pts: Vec<Vec2>,
}

impl TracedLoop {
    pub fn signed_area(&self) -> f32 {
        let n = self.pts.len();
        let mut s = 0.0;
        for i in 0..n {
            let a = self.pts[i];
            let b = self.pts[(i + 1) % n];
            s += a.x * b.y - b.x * a.y;
        }
        s * 0.5
    }
    pub fn is_hole(&self) -> bool {
        self.signed_area() < 0.0
    }
}

const KEY_SCALE: f32 = 1.0e3; // 1 µm coordinate merge

fn key(v: f32) -> i64 {
    (v * KEY_SCALE).round() as i64
}

/// Resolve `pos − neg` into boundary loops on the compressed grid.
pub fn trace_rects(pos: &[RectF], neg: &[RectF]) -> Vec<TracedLoop> {
    // Coordinate compression over every rect edge.
    let mut xs: Vec<i64> = Vec::new();
    let mut ys: Vec<i64> = Vec::new();
    for r in pos.iter().chain(neg) {
        xs.push(key(r.x));
        xs.push(key(r.x + r.w));
        ys.push(key(r.y));
        ys.push(key(r.y + r.h));
    }
    xs.sort_unstable();
    xs.dedup();
    ys.sort_unstable();
    ys.dedup();
    if xs.len() < 2 || ys.len() < 2 {
        return Vec::new();
    }
    let (nx, ny) = (xs.len(), ys.len());

    // Occupancy of each compressed cell, sampled at the cell centre.
    let xf: Vec<f32> = xs.iter().map(|&k| k as f32 / KEY_SCALE).collect();
    let yf: Vec<f32> = ys.iter().map(|&k| k as f32 / KEY_SCALE).collect();
    let mut occ = vec![vec![false; ny - 1]; nx - 1];
    let contains = |r: &RectF, px: f32, py: f32| -> bool {
        px > r.x && px < r.x + r.w && py > r.y && py < r.y + r.h
    };
    for i in 0..nx - 1 {
        for j in 0..ny - 1 {
            let cx = (xf[i] + xf[i + 1]) * 0.5;
            let cy = (yf[j] + yf[j + 1]) * 0.5;
            let inside = pos.iter().any(|r| contains(r, cx, cy))
                && !neg.iter().any(|r| contains(r, cx, cy));
            occ[i][j] = inside;
        }
    }

    // Directed boundary edges on lattice points, material on the left.
    type Pt = (usize, usize);
    let at = |i: isize, j: isize| -> bool {
        if i < 0 || j < 0 || i as usize >= nx - 1 || j as usize >= ny - 1 {
            false
        } else {
            occ[i as usize][j as usize]
        }
    };
    let mut adj: HashMap<Pt, Vec<Pt>> = HashMap::new();
    for i in 0..nx - 1 {
        for j in 0..ny - 1 {
            if !occ[i][j] {
                continue;
            }
            let (ii, jj) = (i as isize, j as isize);
            if !at(ii, jj - 1) {
                adj.entry((i, j)).or_default().push((i + 1, j));
            }
            if !at(ii + 1, jj) {
                adj.entry((i + 1, j)).or_default().push((i + 1, j + 1));
            }
            if !at(ii, jj + 1) {
                adj.entry((i + 1, j + 1)).or_default().push((i, j + 1));
            }
            if !at(ii - 1, jj) {
                adj.entry((i, j + 1)).or_default().push((i, j));
            }
        }
    }

    // Stitch into loops. At a lattice point with two outgoing edges (diagonal
    // contact), prefer the sharpest LEFT turn relative to the incoming
    // direction so the two touching regions stay separate simple loops.
    let mut used: std::collections::HashSet<(Pt, Pt)> = std::collections::HashSet::new();
    let mut starts: Vec<Pt> = adj.keys().copied().collect();
    starts.sort_unstable();
    let mut loops: Vec<Vec<Pt>> = Vec::new();
    for &start in &starts {
        loop {
            let first = adj[&start]
                .iter()
                .find(|n| !used.contains(&(start, **n)))
                .copied();
            let Some(first) = first else { break };
            let mut pts = vec![start];
            used.insert((start, first));
            let mut prev = start;
            let mut cur = first;
            while cur != start {
                pts.push(cur);
                let din = (cur.0 as isize - prev.0 as isize, cur.1 as isize - prev.1 as isize);
                // Candidate directions in left-most-first order.
                let left = (-din.1, din.0);
                let straight = din;
                let right = (din.1, -din.0);
                let mut next: Option<Pt> = None;
                for d in [left, straight, right] {
                    let cand = (cur.0 as isize + d.0, cur.1 as isize + d.1);
                    if cand.0 < 0 || cand.1 < 0 {
                        continue;
                    }
                    let cand = (cand.0 as usize, cand.1 as usize);
                    if adj
                        .get(&cur)
                        .map_or(false, |ns| ns.contains(&cand))
                        && !used.contains(&(cur, cand))
                    {
                        next = Some(cand);
                        break;
                    }
                }
                let Some(nxt) = next else { break };
                used.insert((cur, nxt));
                prev = cur;
                cur = nxt;
            }
            loops.push(pts);
        }
    }

    // Lattice points → mm, merging collinear runs.
    loops
        .into_iter()
        .filter_map(|pts| {
            let mm: Vec<Vec2> = pts.iter().map(|&(i, j)| Vec2::new(xf[i], yf[j])).collect();
            let merged = merge_collinear(&mm);
            if merged.len() >= 4 { Some(TracedLoop { pts: merged }) } else { None }
        })
        .collect()
}

fn merge_collinear(pts: &[Vec2]) -> Vec<Vec2> {
    let n = pts.len();
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let prev = pts[(i + n - 1) % n];
        let cur = pts[i];
        let next = pts[(i + 1) % n];
        let d0 = cur - prev;
        let d1 = next - cur;
        let cross = d0.x * d1.y - d0.y * d1.x;
        if cross.abs() > 1e-9 {
            out.push(cur);
        }
    }
    out
}

/// Per-edge / per-corner shaping of a traced loop.
pub struct LoopStyle<'a> {
    /// Inward inset of the edge starting at corner `i` (edge i → i+1).
    pub inset: &'a dyn Fn(usize, Vec2, Vec2) -> f32,
    /// Arc radius for corner `i`; `convex` = left turn (material corner points
    /// away from the region interior).
    pub radius: &'a dyn Fn(usize, bool) -> f32,
}

/// Apply per-edge insets to a rectilinear loop (material on the left ⇒ inward
/// is the LEFT normal of the traversal direction), then round corners with
/// per-corner radii, clamped so adjacent arcs never overlap. Returns analytic
/// segments in traversal order.
pub fn shape_loop(lp: &TracedLoop, style: &LoopStyle) -> Vec<Seg> {
    let n = lp.pts.len();
    if n < 4 {
        return Vec::new();
    }
    // 1) Shift each edge inward by its inset; recompute corners by intersecting
    //    consecutive (axis-aligned) shifted lines. Edges alternate H/V, so the
    //    intersection is simply (x of the vertical line, y of the horizontal).
    let dir = |i: usize| -> Vec2 {
        let a = lp.pts[i];
        let b = lp.pts[(i + 1) % n];
        (b - a).normalize()
    };
    // Shifted line for edge i: a point on it + its direction.
    let shifted: Vec<(Vec2, Vec2)> = (0..n)
        .map(|i| {
            let d = dir(i);
            let left = Vec2::new(-d.y, d.x); // interior side
            let ins = (style.inset)(i, lp.pts[i], lp.pts[(i + 1) % n]);
            (lp.pts[i] + left * ins, d)
        })
        .collect();
    let mut corners: Vec<Vec2> = Vec::with_capacity(n);
    for i in 0..n {
        // Corner i = intersection of edge i−1 and edge i.
        let (p0, d0) = shifted[(i + n - 1) % n];
        let (p1, d1) = shifted[i];
        let c = if d0.x.abs() > 0.5 {
            // edge i−1 horizontal, edge i vertical
            Vec2::new(p1.x, p0.y)
        } else {
            Vec2::new(p0.x, p1.y)
        };
        let _ = d1;
        corners.push(c);
    }

    // 2) Corner metadata: convexity (left turn) and clamped radius.
    let cdir = |i: usize| -> Vec2 {
        let a = corners[i];
        let b = corners[(i + 1) % n];
        (b - a).normalize()
    };
    let convex: Vec<bool> = (0..n)
        .map(|i| {
            let din = cdir((i + n - 1) % n);
            let dout = cdir(i);
            din.x * dout.y - din.y * dout.x > 0.0
        })
        .collect();
    let mut radius: Vec<f32> = (0..n)
        .map(|i| (style.radius)(i, convex[i]).max(0.0))
        .collect();
    // Clamp: the two radii sharing an edge must not overlap on it.
    for _ in 0..4 {
        for i in 0..n {
            let j = (i + 1) % n;
            let len = (corners[j] - corners[i]).length();
            let sum = radius[i] + radius[j];
            if sum > len && sum > 1e-9 {
                let f = len / sum;
                radius[i] *= f;
                radius[j] *= f;
            }
        }
    }

    // 3) Emit lines trimmed by corner arcs. For both convex and concave right
    //    angles the arc centre is `corner + (dout − din)·r`; only the sweep
    //    direction differs, which `short_arc` resolves.
    let mut segs: Vec<Seg> = Vec::with_capacity(n * 2);
    for i in 0..n {
        let j = (i + 1) % n;
        let d = cdir(i);
        let s = corners[i] + d * radius[i];
        let e = corners[j] - d * radius[j];
        if (e - s).length() > 1e-6 {
            segs.push(Seg::Line { a: s, b: e });
        }
        if radius[j] > 1e-6 {
            let din = d;
            let dout = cdir(j);
            let center = corners[j] + (dout - din) * radius[j];
            let arc_start = e;
            let arc_end = corners[j] + dout * radius[j];
            let a0 = f32::atan2(arc_start.y - center.y, arc_start.x - center.x);
            let a1 = f32::atan2(arc_end.y - center.y, arc_end.x - center.x);
            let (a0, a1) = short_arc(a0, a1);
            segs.push(Seg::Arc { a: arc_start, b: arc_end, center, radius: radius[j], a0, a1 });
        }
    }
    segs
}

/// Pick the representation of `(a0, a1)` whose sweep is ≤ π in magnitude.
fn short_arc(a0: f32, a1: f32) -> (f32, f32) {
    let mut d = a1 - a0;
    while d > PI {
        d -= 2.0 * PI;
    }
    while d < -PI {
        d += 2.0 * PI;
    }
    (a0, a0 + d)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kernel::sketch::loop_area;

    fn style(inset: f32, rc: f32, rf: f32) -> (Box<dyn Fn(usize, Vec2, Vec2) -> f32>, Box<dyn Fn(usize, bool) -> f32>) {
        (
            Box::new(move |_, _, _| inset),
            Box::new(move |_, cv| if cv { rc } else { rf }),
        )
    }

    fn shape(lp: &TracedLoop, inset: f32, rc: f32, rf: f32) -> Vec<Seg> {
        let (i, r) = style(inset, rc, rf);
        shape_loop(lp, &LoopStyle { inset: &i, radius: &r })
    }

    #[test]
    fn single_rect_traces_ccw() {
        let loops = trace_rects(&[RectF::new(0.0, 0.0, 10.0, 5.0)], &[]);
        assert_eq!(loops.len(), 1);
        assert!((loops[0].signed_area() - 50.0).abs() < 1e-3);
        assert_eq!(loops[0].pts.len(), 4);
    }

    #[test]
    fn union_of_overlapping_rects_is_one_loop() {
        let loops = trace_rects(
            &[RectF::new(0.0, 0.0, 10.0, 5.0), RectF::new(5.0, 0.0, 10.0, 5.0)],
            &[],
        );
        assert_eq!(loops.len(), 1);
        assert!((loops[0].signed_area() - 75.0).abs() < 1e-3);
    }

    #[test]
    fn subtracting_center_creates_hole() {
        let loops = trace_rects(
            &[RectF::new(0.0, 0.0, 10.0, 10.0)],
            &[RectF::new(4.0, 4.0, 2.0, 2.0)],
        );
        assert_eq!(loops.len(), 2);
        let outer = loops.iter().find(|l| !l.is_hole()).unwrap();
        let hole = loops.iter().find(|l| l.is_hole()).unwrap();
        assert!((outer.signed_area() - 100.0).abs() < 1e-3);
        assert!((hole.signed_area() + 4.0).abs() < 1e-3);
    }

    #[test]
    fn splitting_strip_creates_two_loops() {
        // A full-height strip splits the square into two disjoint regions.
        let loops = trace_rects(
            &[RectF::new(0.0, 0.0, 10.0, 10.0)],
            &[RectF::new(4.5, -1.0, 1.0, 12.0)],
        );
        assert_eq!(loops.len(), 2);
        assert!(loops.iter().all(|l| !l.is_hole()));
        let total: f32 = loops.iter().map(|l| l.signed_area()).sum();
        assert!((total - 90.0).abs() < 1e-3);
    }

    #[test]
    fn partial_strip_leaves_finger() {
        // A strip covering half the square's height leaves one loop with a
        // notch: 8 corners.
        let loops = trace_rects(
            &[RectF::new(0.0, 0.0, 10.0, 10.0)],
            &[RectF::new(4.5, -1.0, 1.0, 6.0)],
        );
        assert_eq!(loops.len(), 1);
        assert_eq!(loops[0].pts.len(), 8);
        assert!((loops[0].signed_area() - 95.0).abs() < 1e-3);
    }

    #[test]
    fn rounding_and_inset() {
        let loops = trace_rects(&[RectF::new(0.0, 0.0, 10.0, 10.0)], &[]);
        let segs = shape(&loops[0], 1.0, 2.0, 0.0);
        // Inset 1 → 8×8 square, rounded r=2 at 4 corners: 4 lines + 4 arcs.
        assert_eq!(segs.len(), 8);
        let area = loop_area(&segs);
        // Chord-approx area of an 8×8 with r2 rounded corners is between the
        // sharp square (64) and the octagon underestimate.
        assert!(area > 55.0 && area < 64.0, "area {area}");
        // Endpoints chain.
        for i in 0..segs.len() {
            let e = segs[i].end();
            let s = segs[(i + 1) % segs.len()].start();
            assert!((e - s).length() < 1e-4, "chain break at {i}");
        }
    }

    #[test]
    fn concave_corner_gets_own_radius() {
        // L-shape: concave corner rounded with rf, convex with rc.
        let loops = trace_rects(
            &[RectF::new(0.0, 0.0, 10.0, 5.0), RectF::new(0.0, 0.0, 5.0, 10.0)],
            &[],
        );
        assert_eq!(loops.len(), 1);
        assert_eq!(loops[0].pts.len(), 6);
        let segs = shape(&loops[0], 0.0, 1.0, 0.5);
        let arcs: Vec<f32> = segs
            .iter()
            .filter_map(|s| match s {
                Seg::Arc { radius, .. } => Some(*radius),
                _ => None,
            })
            .collect();
        assert_eq!(arcs.len(), 6, "5 convex + 1 concave corner arcs");
        assert_eq!(arcs.iter().filter(|&&r| (r - 0.5).abs() < 1e-5).count(), 1);
        assert_eq!(arcs.iter().filter(|&&r| (r - 1.0).abs() < 1e-5).count(), 5);
        // Chain remains closed.
        for i in 0..segs.len() {
            let e = segs[i].end();
            let s = segs[(i + 1) % segs.len()].start();
            assert!((e - s).length() < 1e-4, "chain break at {i}");
        }
    }

    #[test]
    fn diagonal_touch_stays_two_loops() {
        // Two squares touching only at one corner must trace as two loops.
        let loops = trace_rects(
            &[RectF::new(0.0, 0.0, 5.0, 5.0), RectF::new(5.0, 5.0, 5.0, 5.0)],
            &[],
        );
        assert_eq!(loops.len(), 2);
        assert!(loops.iter().all(|l| (l.signed_area() - 25.0).abs() < 1e-3));
    }
}
