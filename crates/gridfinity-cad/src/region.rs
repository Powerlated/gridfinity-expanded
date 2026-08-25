use crate::kernel::math::Vec2;
use crate::kernel::round::short_arc;
use crate::kernel::sketch::{Seg, loop_area, polygon_area};
use crate::layout::GridCell;
use std::collections::{HashMap, HashSet};

type Pt = (i32, i32);

fn boundary_directed(cells: &[GridCell]) -> Vec<(Pt, Pt)> {
    let set: HashSet<GridCell> = cells.iter().copied().collect();
    let present = |x: i32, y: i32| set.contains(&GridCell { x, y });
    let mut edges = Vec::new();
    for &c in cells {
        let (x, y) = (c.x, c.y);
        if !present(x, y - 1) {
            edges.push(((x, y), (x + 1, y)));
        }
        if !present(x + 1, y) {
            edges.push(((x + 1, y), (x + 1, y + 1)));
        }
        if !present(x, y + 1) {
            edges.push(((x + 1, y + 1), (x, y + 1)));
        }
        if !present(x - 1, y) {
            edges.push(((x, y + 1), (x, y)));
        }
    }
    edges
}

fn trace_loops(cells: &[GridCell]) -> Vec<Vec<Pt>> {
    let edges = boundary_directed(cells);
    let mut adj: HashMap<Pt, Vec<Pt>> = HashMap::new();
    for (a, b) in &edges {
        adj.entry(*a).or_default().push(*b);
    }
    let mut used: HashSet<(Pt, Pt)> = HashSet::new();
    let mut starts: Vec<Pt> = adj.keys().copied().collect();
    starts.sort_unstable();
    let mut loops = Vec::new();
    for &start in &starts {
        while adj
            .get(&start)
            .map_or(false, |ns| ns.iter().any(|n| !used.contains(&(start, *n))))
        {
            let mut loop_pts = vec![start];
            let mut cur = start;
            loop {
                let pick = adj
                    .get(&cur)
                    .and_then(|ns| ns.iter().find(|n| !used.contains(&(cur, **n))))
                    .copied();
                let nxt = match pick {
                    Some(n) => n,
                    None => break,
                };
                used.insert((cur, nxt));
                if nxt == start {
                    break;
                }
                loop_pts.push(nxt);
                cur = nxt;
            }
            loops.push(loop_pts);
        }
    }
    loops
}

fn to_mm(p: Pt, pitch: f64, origin: Vec2) -> Vec2 {
    Vec2::new(origin.x + p.0 as f64 * pitch, origin.y + p.1 as f64 * pitch)
}

fn build_loop(pts: &[Pt], pitch: f64, origin: Vec2, r: f64) -> Vec<Seg> {
    let n = pts.len();
    if n < 2 {
        return Vec::new();
    }
    let mm: Vec<Vec2> = pts.iter().map(|&p| to_mm(p, pitch, origin)).collect();

    let mut max_r = f64::INFINITY;
    for i in 0..n {
        let a = mm[i];
        let b = mm[(i + 1) % n];
        max_r = max_r.min((a - b).length() * 0.5);
    }
    let r = r.min(max_r).max(0.0);

    let dir_out = |i: usize| -> Vec2 {
        let a = mm[i];
        let b = mm[(i + 1) % n];
        (b - a).normalize()
    };
    let convex = |i: usize| -> bool {
        let prev = mm[(i + n - 1) % n];
        let cur = mm[i];
        let next = mm[(i + 1) % n];
        let din = (cur - prev).normalize();
        let dout = (next - cur).normalize();
        let cross = din.x * dout.y - din.y * dout.x;
        cross > 0.0
    };

    let mut segs: Vec<Seg> = Vec::with_capacity(n * 2);
    for i in 0..n {
        let dout = dir_out(i);
        let dout_next = dir_out((i + 1) % n);
        let trim_start = if convex(i) { r } else { 0.0 };
        let trim_end = if convex((i + 1) % n) { r } else { 0.0 };
        let s = mm[i] + dout * trim_start;
        let e = mm[(i + 1) % n] - dout * trim_end;
        if (e - s).length() > 1e-6 {
            segs.push(Seg::Line { a: s, b: e });
        }
        if convex((i + 1) % n) && r > 1e-6 {
            let corner = mm[(i + 1) % n];
            let din = dout;
            let center = corner + (dout_next - din) * r;
            let arc_start = e;
            let arc_end = corner + dout_next * r;
            let a0 = f64::atan2(arc_start.y - center.y, arc_start.x - center.x);
            let a1 = f64::atan2(arc_end.y - center.y, arc_end.x - center.x);
            let (a0, a1) = short_arc(a0, a1);
            segs.push(Seg::Arc {
                a: arc_start,
                b: arc_end,
                center,
                radius: r,
                a0,
                a1,
            });
        }
    }
    segs
}

#[derive(Clone, Debug)]
pub struct RegionLoop {
    pub segs: Vec<Seg>,
    pub area: f64,
}

impl RegionLoop {
    pub fn is_hole(&self) -> bool {
        self.area < 0.0
    }
}

pub fn region_loops(
    cells: &[GridCell],
    pitch: f64,
    corner_radius: f64,
    origin: Vec2,
) -> Vec<RegionLoop> {
    let mut loops = trace_loops(cells);
    let mut with_area: Vec<(f64, Vec<Pt>)> = loops
        .drain(..)
        .map(|pts| {
            let mm: Vec<Vec2> = pts.iter().map(|&p| to_mm(p, pitch, origin)).collect();
            (polygon_area(&mm).abs(), pts)
        })
        .collect();
    with_area.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));
    with_area
        .into_iter()
        .map(|(_, pts)| {
            let segs = build_loop(&pts, pitch, origin, corner_radius);
            let area = loop_area(&segs);
            RegionLoop { segs, area }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kernel::sketch::loop_area;

    fn cells(coords: &[(i32, i32)]) -> Vec<GridCell> {
        coords.iter().map(|&(x, y)| GridCell { x, y }).collect()
    }

    fn total_segs(loops: &[RegionLoop]) -> usize {
        loops.iter().map(|l| l.segs.len()).sum()
    }

    #[test]
    fn single_cell_is_four_segments_cw_rounded() {
        let loops = region_loops(&cells(&[(0, 0)]), 1.0, 0.0, Vec2::ZERO);
        assert_eq!(loops.len(), 1, "one loop");
        assert!(loops[0].area > 0.0, "outer loop CCW");
        assert_eq!(loops[0].segs.len(), 4, "4 lines, no rounding at r=0");
        assert!((loop_area(&loops[0].segs) - 1.0).abs() < 1e-5);
    }

    #[test]
    fn single_cell_rounded_has_arcs() {
        let loops = region_loops(&cells(&[(0, 0)]), 10.0, 2.0, Vec2::ZERO);
        let segs = &loops[0].segs;
        let arcs = segs.iter().filter(|s| matches!(s, Seg::Arc { .. })).count();
        let lines = segs
            .iter()
            .filter(|s| matches!(s, Seg::Line { .. }))
            .count();
        assert_eq!(arcs, 4, "four rounded corners");
        assert!(lines >= 4, "four straight sides remain");
    }

    #[test]
    fn two_by_two_block_is_one_loop() {
        let loops = region_loops(
            &cells(&[(0, 0), (1, 0), (0, 1), (1, 1)]),
            1.0,
            0.0,
            Vec2::ZERO,
        );
        assert_eq!(loops.len(), 1);
        assert!(
            (loop_area(&loops[0].segs) - 4.0).abs() < 1e-5,
            "area = 4 cells"
        );
    }

    #[test]
    fn l_shape_is_one_loop_with_concave_corner() {
        let loops = region_loops(&cells(&[(0, 0), (1, 0), (0, 1)]), 1.0, 0.0, Vec2::ZERO);
        assert_eq!(loops.len(), 1);
        assert_eq!(loops[0].segs.len(), 8);
    }

    #[test]
    fn ring_has_outer_and_hole() {
        let mut c = Vec::new();
        for x in 0..3 {
            for y in 0..3 {
                if !(x == 1 && y == 1) {
                    c.push(GridCell { x, y });
                }
            }
        }
        let loops = region_loops(&c, 1.0, 0.0, Vec2::ZERO);
        assert_eq!(loops.len(), 2, "outer + hole");
        assert!(loops[0].area > 0.0, "outer CCW");
        assert!(loops[1].area < 0.0, "hole CW");
    }

    #[test]
    fn disjoint_regions_produce_two_outer_loops() {
        let loops = region_loops(&cells(&[(0, 0), (3, 3)]), 1.0, 0.0, Vec2::ZERO);
        assert_eq!(loops.len(), 2);
        assert!(loops.iter().all(|l| l.area > 0.0));
        let _ = total_segs(&loops);
    }
}
