use crate::kernel::math::Vec2;
use crate::kernel::round::short_arc;
use crate::kernel::sketch::{Seg, point_in_polygon, polygon_area};
use std::collections::HashMap;

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

#[derive(Clone, Debug)]
pub struct TracedLoop {
    pub pts: Vec<Vec2>,
}

impl TracedLoop {
    pub fn signed_area(&self) -> f32 {
        polygon_area(&self.pts)
    }
    pub fn is_hole(&self) -> bool {
        self.signed_area() < 0.0
    }

    /// Whether `pt` lies strictly inside this loop, by even-odd crossings of a
    /// ray cast in +x. A point exactly on the boundary is decided by which side
    /// of it the ray's tie-breaking puts the two edges there, so callers with a
    /// point that may sit on the loop must not ask.
    pub fn contains(&self, pt: Vec2) -> bool {
        point_in_polygon(&self.pts, pt)
    }
}

const KEY_SCALE: f32 = 1.0e3;

fn key(v: f32) -> i64 {
    (v * KEY_SCALE).round() as i64
}

pub fn trace_rects(pos: &[RectF], neg: &[RectF]) -> Vec<TracedLoop> {
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

    let xf: Vec<f32> = xs.iter().map(|&k| k as f32 / KEY_SCALE).collect();
    let yf: Vec<f32> = ys.iter().map(|&k| k as f32 / KEY_SCALE).collect();
    let mut occ = vec![false; (nx - 1) * (ny - 1)];
    let span = |v: &[i64], lo: f32, hi: f32| -> (usize, usize) {
        let a = v.partition_point(|&k| k < key(lo));
        let b = v.partition_point(|&k| k < key(hi));
        (a, b)
    };
    let paint = |rects: &[RectF], value: bool, occ: &mut [bool]| {
        for r in rects {
            let (i0, i1) = span(&xs, r.x, r.x + r.w);
            let (j0, j1) = span(&ys, r.y, r.y + r.h);
            for i in i0..i1 {
                occ[i * (ny - 1) + j0..i * (ny - 1) + j1].fill(value);
            }
        }
    };
    paint(pos, true, &mut occ);
    paint(neg, false, &mut occ);

    type Pt = (usize, usize);
    let at = |i: isize, j: isize| -> bool {
        if i < 0 || j < 0 || i as usize >= nx - 1 || j as usize >= ny - 1 {
            false
        } else {
            occ[i as usize * (ny - 1) + j as usize]
        }
    };
    let mut adj: HashMap<Pt, Vec<Pt>> = HashMap::new();
    for i in 0..nx - 1 {
        for j in 0..ny - 1 {
            if !occ[i * (ny - 1) + j] {
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
                let din = (
                    cur.0 as isize - prev.0 as isize,
                    cur.1 as isize - prev.1 as isize,
                );
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
                    if adj.get(&cur).map_or(false, |ns| ns.contains(&cand))
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

    loops
        .into_iter()
        .filter_map(|pts| {
            let mm: Vec<Vec2> = pts.iter().map(|&(i, j)| Vec2::new(xf[i], yf[j])).collect();
            let merged = merge_collinear(&mm);
            if merged.len() >= 4 {
                Some(TracedLoop { pts: merged })
            } else {
                None
            }
        })
        .collect()
}

/// Drop every point a straight run passes through, so a rectilinear loop carries
/// a point only where it turns. A collinear point is not merely redundant: the
/// corner-rounding pass reads a zero cross product as a *reentrant* corner and
/// rounds it by the floor-fillet radius.
pub fn merge_collinear(pts: &[Vec2]) -> Vec<Vec2> {
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

pub struct LoopStyle<'a> {
    pub inset: &'a dyn Fn(usize, Vec2, Vec2) -> f32,
    pub radius: &'a dyn Fn(usize, bool) -> f32,
}

pub fn shape_loop(lp: &TracedLoop, style: &LoopStyle) -> Vec<Seg> {
    let n = lp.pts.len();
    if n < 4 {
        return Vec::new();
    }
    let dir = |i: usize| -> Vec2 {
        let a = lp.pts[i];
        let b = lp.pts[(i + 1) % n];
        (b - a).normalize()
    };
    let shifted: Vec<(Vec2, Vec2)> = (0..n)
        .map(|i| {
            let d = dir(i);
            let left = Vec2::new(-d.y, d.x);
            let ins = (style.inset)(i, lp.pts[i], lp.pts[(i + 1) % n]);
            (lp.pts[i] + left * ins, d)
        })
        .collect();
    let mut corners: Vec<Vec2> = Vec::with_capacity(n);
    for i in 0..n {
        let (p0, d0) = shifted[(i + n - 1) % n];
        let (p1, d1) = shifted[i];
        let c = if d0.x.abs() > 0.5 {
            Vec2::new(p1.x, p0.y)
        } else {
            Vec2::new(p0.x, p1.y)
        };
        let _ = d1;
        corners.push(c);
    }

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
            segs.push(Seg::Arc {
                a: arc_start,
                b: arc_end,
                center,
                radius: radius[j],
                a0,
                a1,
            });
        }
    }
    segs
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kernel::sketch::loop_area;

    fn style(
        inset: f32,
        rc: f32,
        rf: f32,
    ) -> (
        Box<dyn Fn(usize, Vec2, Vec2) -> f32>,
        Box<dyn Fn(usize, bool) -> f32>,
    ) {
        (
            Box::new(move |_, _, _| inset),
            Box::new(move |_, cv| if cv { rc } else { rf }),
        )
    }

    fn shape(lp: &TracedLoop, inset: f32, rc: f32, rf: f32) -> Vec<Seg> {
        let (i, r) = style(inset, rc, rf);
        shape_loop(
            lp,
            &LoopStyle {
                inset: &i,
                radius: &r,
            },
        )
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
            &[
                RectF::new(0.0, 0.0, 10.0, 5.0),
                RectF::new(5.0, 0.0, 10.0, 5.0),
            ],
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
        assert_eq!(segs.len(), 8);
        let area = loop_area(&segs);
        assert!(area > 55.0 && area < 64.0, "area {area}");
        for i in 0..segs.len() {
            let e = segs[i].end();
            let s = segs[(i + 1) % segs.len()].start();
            assert!((e - s).length() < 1e-4, "chain break at {i}");
        }
    }

    #[test]
    fn concave_corner_gets_own_radius() {
        let loops = trace_rects(
            &[
                RectF::new(0.0, 0.0, 10.0, 5.0),
                RectF::new(0.0, 0.0, 5.0, 10.0),
            ],
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
        for i in 0..segs.len() {
            let e = segs[i].end();
            let s = segs[(i + 1) % segs.len()].start();
            assert!((e - s).length() < 1e-4, "chain break at {i}");
        }
    }

    #[test]
    fn diagonal_touch_stays_two_loops() {
        let loops = trace_rects(
            &[
                RectF::new(0.0, 0.0, 5.0, 5.0),
                RectF::new(5.0, 5.0, 5.0, 5.0),
            ],
            &[],
        );
        assert_eq!(loops.len(), 2);
        assert!(loops.iter().all(|l| (l.signed_area() - 25.0).abs() < 1e-3));
    }
}
