//! What makes one packed layout nicer to look at than another, as a number the
//! search can minimise.
//!
//! `Tidiness` is a finished layout read six ways, every field a fraction of its
//! own worst case so the six are comparable and the weights in `score` are the
//! only judgement in the file. `tidiness` measures them; `score` combines them,
//! lower being tidier. Everything is derived from the placed claims and the
//! rectangle they were packed into, and from the same helpers `walls.rs` uses to
//! turn those claims into dividers -- what the viewer sees *is* the divider set,
//! so measuring the dividers is measuring the picture. `lattice` is the shared
//! step: the claims' own coordinate grid, clipped to the packing area, on which
//! the leftover space is exactly the cells no claim covers.

use super::pack::Placement;
use super::rects::{
    Rect, Segment, boundary_segments, merge_segments, parts_bounds, quantize, union_area,
};
use super::walls::on_area_boundary;
use crate::layout::Orientation;
use std::collections::{BTreeMap, BTreeSet};

/// How much a divider line matters against the rest: the strongest term,
/// because compartment edges that share a line are the whole difference between
/// a layout that reads as a grid and one that reads as a staircase.
const W_LINES: f64 = 1.0;

/// How much an unshared boundary run matters. Half of `W_LINES`: two
/// compartments that meet on one line but do not span the same stretch of it
/// are already most of the way to tidy.
const W_RUNS: f64 = 0.5;

/// How much leftover space broken into separate pieces matters.
const W_FRAGMENTS: f64 = 0.75;

/// How much leftover too narrow to hold anything matters. As strong as
/// `W_LINES`: a crack between two compartments is both wasted and ugly.
const W_SLIVERS: f64 = 1.0;

/// How much scattering the instances of one object matters.
const W_GROUPING: f64 = 0.5;

/// How much sitting off-centre matters. The smallest weight on purpose:
/// gathering the leftover into one block means the claims crowd one end, so
/// balance and `fragments` pull against each other and balance must lose. It
/// decides between layouts that are otherwise equally tidy, which is what
/// "prefer the balanced one" means.
const W_BALANCE: f64 = 0.1;

/// How a finished layout reads, term by term, each a fraction in `0..=1` with 0
/// the tidiest.
///
/// Fractions rather than counts because the terms count different things -- one
/// counts lines, one counts regions, three measure area -- and a weighted sum of
/// raw counts would be a sum of incomparable units whose weights silently
/// encoded the layout's size.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "camelCase"))]
pub struct Tidiness {
    /// Distinct interior divider lines, over the claim boundary runs there are.
    pub lines: f64,
    /// Interior divider runs after merging, over the runs there are: how much of
    /// the boundary failed to become a shared span.
    pub runs: f64,
    /// Leftover space broken into more than one region, over the placements.
    pub fragments: f64,
    /// Leftover too narrow to hold the narrowest thing packed, over the area.
    pub slivers: f64,
    /// Instances of one object standing apart, as the area their bounding boxes
    /// enclose but do not cover, over the area.
    pub grouping: f64,
    /// The claims' centre of area off the packing area's centre, over half its
    /// diagonal.
    pub balance: f64,
}

/// The weighted sum of the six terms: what the search minimises after it has
/// placed everything it can. Zero is a layout with no interior divider at all,
/// no leftover but one block of it, every object's instances touching, and its
/// centre of area on the drawer's own centre.
pub fn score(t: &Tidiness) -> f64 {
    let score = W_LINES * t.lines
        + W_RUNS * t.runs
        + W_FRAGMENTS * t.fragments
        + W_SLIVERS * t.slivers
        + W_GROUPING * t.grouping
        + W_BALANCE * t.balance;
    assert!(
        score.is_finite() && score >= 0.0,
        "a tidiness score is a non-negative weighted sum of fractions, but {t:?} scored {score}"
    );
    score
}

/// Every value quantised, deduplicated and sorted ascending, clipped to
/// `lo..=hi`, and with both ends present.
///
/// This is the lattice one axis of `lattice` runs on: the claim edges that fall
/// inside the packing area, plus the area's own two edges, so every cell of the
/// grid lies inside the area and no claim edge cuts a cell in half.
fn axis_lines(values: impl Iterator<Item = f64>, lo: f64, hi: f64) -> Vec<f64> {
    assert!(lo <= hi, "an axis of the packing area runs from {lo} to {hi}");
    let mut out: Vec<f64> = values
        .map(quantize)
        .filter(|v| *v > lo && *v < hi)
        .collect();
    out.push(quantize(lo));
    out.push(quantize(hi));
    out.sort_by(f64::total_cmp);
    out.dedup();
    out
}

/// The packing area cut into cells by the claim edges crossing it, and for each
/// cell whether some claim covers it.
///
/// A cell is covered wholly or not at all, because every claim edge inside the
/// area is itself a lattice line. So the cells no claim covers are exactly the
/// leftover space, cut into rectangles whose own extents are the gaps between
/// consecutive claim edges -- which is what makes a strip narrower than
/// `narrowest` a run of cells narrower than `narrowest` rather than something
/// that has to be searched for.
struct Lattice {
    xs: Vec<f64>,
    ys: Vec<f64>,
    covered: Vec<bool>,
}

impl Lattice {
    /// How many columns the lattice has: one fewer than its x lines.
    fn cols(&self) -> usize {
        self.xs.len() - 1
    }

    /// How many rows the lattice has: one fewer than its y lines.
    fn rows(&self) -> usize {
        self.ys.len() - 1
    }

    /// Whether the cell at these indices is covered by a claim, `false` outside
    /// the lattice so a flood fill needs no bounds special case.
    fn covered_at(&self, col: isize, row: isize) -> bool {
        if col < 0 || row < 0 || col >= self.cols() as isize || row >= self.rows() as isize {
            return true;
        }
        self.covered[row as usize * self.cols() + col as usize]
    }
}

/// The index of the lattice line at `value`, which must be one of them.
fn line_index(lines: &[f64], value: f64) -> usize {
    let wanted = quantize(value);
    lines
        .binary_search_by(|line| line.total_cmp(&wanted))
        .unwrap_or_else(|_| {
            panic!("{wanted} is not one of the lattice lines it was built from: {lines:?}")
        })
}

/// The packing area's lattice, with every claim's cells marked covered.
///
/// Marking is per claim over its own cell span rather than per cell over every
/// claim, so the whole grid costs the area it covers once and not once per
/// placement -- which is what makes this affordable inside the search loop.
/// Claims that reach outside the area are clipped to it; the packer keeps them
/// inside, so the clip only guards the lattice's own invariant.
fn lattice(parts: &[Rect], area: &Rect) -> Lattice {
    let xs = axis_lines(
        parts.iter().flat_map(|p| [p.x, p.right()]),
        area.x,
        area.right(),
    );
    let ys = axis_lines(
        parts.iter().flat_map(|p| [p.y, p.bottom()]),
        area.y,
        area.bottom(),
    );
    let (cols, rows) = (xs.len() - 1, ys.len() - 1);
    let mut covered = vec![false; cols * rows];
    for part in parts {
        let lo_x = line_index(&xs, part.x.max(area.x));
        let hi_x = line_index(&xs, part.right().min(area.right()));
        let lo_y = line_index(&ys, part.y.max(area.y));
        let hi_y = line_index(&ys, part.bottom().min(area.bottom()));
        for row in lo_y..hi_y {
            for col in lo_x..hi_x {
                covered[row * cols + col] = true;
            }
        }
    }
    Lattice { xs, ys, covered }
}

/// The leftover space of a lattice, as `(regions, sliver area in mm²)`.
///
/// A region is a set of uncovered cells reachable from one another by shared
/// edges -- the same walk `parts_connected` makes over covered cells, from the
/// other side. A cell counts towards the sliver area when its own width or
/// depth is below `narrowest`: the lattice lines are the claim edges, so a cell
/// that narrow is a gap that narrow, and a gap narrower than the narrowest thing
/// already packed can never hold anything.
fn leftover(lattice: &Lattice, narrowest: f64) -> (usize, f64) {
    assert!(
        narrowest >= 0.0,
        "the narrowest claim in a layout has a non-negative extent, not {narrowest}"
    );
    let (cols, rows) = (lattice.cols(), lattice.rows());
    let mut seen = vec![false; cols * rows];
    let mut regions = 0;
    let mut slivers = 0.0;
    let mut stack: Vec<usize> = Vec::new();
    for start in 0..cols * rows {
        if seen[start] || lattice.covered[start] {
            continue;
        }
        regions += 1;
        seen[start] = true;
        stack.push(start);
        while let Some(index) = stack.pop() {
            let (col, row) = ((index % cols) as isize, (index / cols) as isize);
            let width = lattice.xs[col as usize + 1] - lattice.xs[col as usize];
            let depth = lattice.ys[row as usize + 1] - lattice.ys[row as usize];
            if width < narrowest || depth < narrowest {
                slivers += width * depth;
            }
            for (dx, dy) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
                let (nc, nr) = (col + dx, row + dy);
                if lattice.covered_at(nc, nr) {
                    continue;
                }
                let next = nr as usize * cols + nc as usize;
                if seen[next] {
                    continue;
                }
                seen[next] = true;
                stack.push(next);
            }
        }
    }
    (regions, slivers)
}

/// How much of the boundary between compartments failed to be shared, as
/// `(distinct interior lines, interior runs)` over the runs the claims have
/// between them.
///
/// The runs are the ones `layout_walls` turns into dividers: every claim's own
/// boundary, merged per line so two compartments meeting along one line
/// contribute one span, with the runs lying on the packing area's edge dropped
/// because the bin's perimeter wall already stands there. A layout whose claims
/// share every edge merges its boundary down to a few long spans on a few lines;
/// a staircase merges nothing and keeps one line per edge.
fn boundary(placements: &[Placement], area: &Rect) -> (f64, f64) {
    let segments: Vec<Segment> = placements
        .iter()
        .flat_map(|p| boundary_segments(&p.parts))
        .collect();
    if segments.is_empty() {
        return (0.0, 0.0);
    }
    let mut lines: BTreeSet<(Orientation, i64)> = BTreeSet::new();
    let mut runs = 0usize;
    for run in merge_segments(&segments) {
        if on_area_boundary(&run, area) {
            continue;
        }
        runs += 1;
        lines.insert((run.orientation, (quantize(run.coordinate) * 1e4).round() as i64));
    }
    let total = segments.len() as f64;
    assert!(
        runs as f64 <= total && lines.len() <= runs,
        "merging {total} boundary run(s) gave {runs} run(s) on {} line(s), which is more of \
         either than it started with",
        lines.len()
    );
    (lines.len() as f64 / total, runs as f64 / total)
}

/// How far the instances of each object stand apart, as the area their bounding
/// boxes enclose but no claim of theirs covers, over `area`.
///
/// The area covered is a **union**, not a sum. Two claims of two placements
/// never overlap -- `pack_once` asserts it as it places them -- but the two
/// parts of *one* placement do by construction, each being a box of one object
/// grown by the claim margin, and `settle`'s growth pass widens that overlap
/// further. Summing counted the shared ground twice and made a plain L score as
/// though its instances were spread apart. Twelve sockets in a block enclose
/// exactly what they cover and score 0; twelve scattered across the drawer
/// enclose most of it.
///
/// An object whose instances tile their own bounding box exactly reaches that 0
/// through a subtraction of two numbers computed different ways -- a quantised
/// box's extents against a quantised union -- so it can land a few ulps below
/// it, which is what the clamp is for.
fn grouping(placements: &[Placement], area: &Rect) -> f64 {
    let mut boxes: BTreeMap<&str, Vec<Rect>> = BTreeMap::new();
    for placement in placements {
        boxes
            .entry(placement.object_id.as_str())
            .or_default()
            .extend(placement.parts.iter().copied());
    }
    let mut apart = 0.0;
    for (id, parts) in &boxes {
        let bounds = parts_bounds(parts);
        let covered = union_area(parts);
        assert!(
            bounds.area() + 1e-6 >= covered,
            "{id}'s claims cover {covered} mm2 inside a {} mm2 bounding box, which does not \
             contain them",
            bounds.area()
        );
        apart += (bounds.area() - covered).max(0.0);
    }
    if area.area() <= 0.0 {
        return 0.0;
    }
    (apart / area.area()).min(1.0)
}

/// How far the layout's centre of area sits from the packing area's own centre,
/// over half the area's diagonal, so a layout crowded into one corner scores
/// near 1 and one spread evenly about the middle scores near 0.
fn balance(placements: &[Placement], area: &Rect) -> f64 {
    let mut weight = 0.0;
    let (mut cx, mut cy) = (0.0, 0.0);
    for placement in placements {
        for part in &placement.parts {
            let a = part.area();
            weight += a;
            cx += a * (part.x + part.width / 2.0);
            cy += a * (part.y + part.depth / 2.0);
        }
    }
    let reach = (area.width.hypot(area.depth)) / 2.0;
    if weight <= 0.0 || reach <= 0.0 {
        return 0.0;
    }
    let off = (cx / weight - (area.x + area.width / 2.0))
        .hypot(cy / weight - (area.y + area.depth / 2.0));
    (off / reach).min(1.0)
}

/// How a finished layout reads: the six terms of `Tidiness`, measured from the
/// claims as placed and the rectangle they were packed into.
///
/// An empty layout is perfectly tidy in every term, which is the honest answer
/// -- there is nothing to look at -- and never the layout the search prefers,
/// because `better` compares the number of placements first.
pub fn tidiness(placements: &[Placement], area: &Rect) -> Tidiness {
    if placements.is_empty() {
        return Tidiness::default();
    }
    let parts: Vec<Rect> = placements.iter().flat_map(|p| p.parts.clone()).collect();
    let narrowest = parts
        .iter()
        .map(|p| p.width.min(p.depth))
        .fold(f64::INFINITY, f64::min);
    let (regions, slivers) = leftover(&lattice(&parts, area), narrowest);
    let (lines, runs) = boundary(placements, area);
    let t = Tidiness {
        lines,
        runs,
        fragments: (regions.saturating_sub(1) as f64 / placements.len() as f64).min(1.0),
        slivers: if area.area() > 0.0 {
            (slivers / area.area()).min(1.0)
        } else {
            0.0
        },
        grouping: grouping(placements, area),
        balance: balance(placements, area),
    };
    for (name, value) in [
        ("lines", t.lines),
        ("runs", t.runs),
        ("fragments", t.fragments),
        ("slivers", t.slivers),
        ("grouping", t.grouping),
        ("balance", t.balance),
    ] {
        assert!(
            (0.0..=1.0).contains(&value),
            "the {name} term of a tidiness reading is a fraction of its own worst case, not {value}"
        );
    }
    t
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::rects::Rotation;

    const AREA: Rect = Rect {
        x: 0.0,
        y: 0.0,
        width: 100.0,
        depth: 100.0,
    };

    /// A layout of one instance per rectangle, all of one object unless the id
    /// is given per rectangle.
    fn layout(rects: &[(&str, Rect)]) -> Vec<Placement> {
        rects
            .iter()
            .enumerate()
            .map(|(i, (id, rect))| Placement {
                object_id: (*id).to_string(),
                instance: i as u32,
                rotation: Rotation::Deg0,
                parts: vec![*rect],
            })
            .collect()
    }

    /// Four equal claims tiling the area exactly: every interior edge is shared,
    /// there is no leftover at all, and the centre of area is the area's own.
    fn grid() -> Vec<Placement> {
        layout(&[
            ("a", Rect::new(0.0, 0.0, 50.0, 50.0)),
            ("a", Rect::new(50.0, 0.0, 50.0, 50.0)),
            ("a", Rect::new(0.0, 50.0, 50.0, 50.0)),
            ("a", Rect::new(50.0, 50.0, 50.0, 50.0)),
        ])
    }

    #[test]
    fn a_shared_grid_is_the_tidiest_a_layout_gets() {
        let t = tidiness(&grid(), &AREA);
        assert_eq!(t.fragments, 0.0, "a tiled area leaves nothing over");
        assert_eq!(t.slivers, 0.0);
        assert_eq!(t.grouping, 0.0, "four claims tiling a square enclose what they cover");
        assert!(t.balance < 1e-9, "a tiled area's centre of area is its centre: {t:?}");
        assert!(
            t.lines < 0.2 && t.runs < 0.4,
            "sixteen boundary runs merge to two lines and four runs, not {t:?}"
        );
    }

    /// The same four claims as a pinwheel: each one offset so no edge lines up
    /// with another's, which is the staircase the search is meant to avoid.
    #[test]
    fn a_pinwheel_scores_worse_than_the_grid_it_could_have_been() {
        let pinwheel = layout(&[
            ("a", Rect::new(0.0, 0.0, 60.0, 40.0)),
            ("a", Rect::new(60.0, 0.0, 40.0, 60.0)),
            ("a", Rect::new(40.0, 60.0, 60.0, 40.0)),
            ("a", Rect::new(0.0, 40.0, 40.0, 60.0)),
        ]);
        let tidy = score(&tidiness(&grid(), &AREA));
        let ugly = score(&tidiness(&pinwheel, &AREA));
        assert!(
            ugly > tidy,
            "the pinwheel scored {ugly} against the grid's {tidy}, so the objective cannot tell \
             them apart"
        );
    }

    #[test]
    fn one_leftover_block_beats_two() {
        let together = layout(&[
            ("a", Rect::new(0.0, 0.0, 100.0, 30.0)),
            ("a", Rect::new(0.0, 30.0, 100.0, 30.0)),
        ]);
        let apart = layout(&[
            ("a", Rect::new(0.0, 0.0, 100.0, 30.0)),
            ("a", Rect::new(0.0, 50.0, 100.0, 30.0)),
        ]);
        assert_eq!(
            tidiness(&together, &AREA).fragments,
            0.0,
            "two bands against one edge leave one block over"
        );
        assert!(
            tidiness(&apart, &AREA).fragments > 0.0,
            "a band across the middle of the leftover cuts it in two"
        );
    }

    #[test]
    fn a_gap_narrower_than_the_narrowest_claim_is_a_sliver() {
        let crack = layout(&[
            ("a", Rect::new(0.0, 0.0, 40.0, 100.0)),
            ("a", Rect::new(43.0, 0.0, 57.0, 100.0)),
        ]);
        let room = layout(&[
            ("a", Rect::new(0.0, 0.0, 20.0, 100.0)),
            ("a", Rect::new(80.0, 0.0, 20.0, 100.0)),
        ]);
        assert!(
            (tidiness(&crack, &AREA).slivers - 300.0 / AREA.area()).abs() < 1e-9,
            "a 3 mm gap between 40 mm claims is 300 mm2 of sliver, not {:?}",
            tidiness(&crack, &AREA)
        );
        assert_eq!(
            tidiness(&room, &AREA).slivers,
            0.0,
            "a 60 mm gap holds a 20 mm claim, so it is leftover and not a crack"
        );
    }

    #[test]
    fn instances_of_one_object_score_better_together() {
        let together = layout(&[
            ("socket", Rect::new(0.0, 0.0, 20.0, 20.0)),
            ("socket", Rect::new(20.0, 0.0, 20.0, 20.0)),
        ]);
        let scattered = layout(&[
            ("socket", Rect::new(0.0, 0.0, 20.0, 20.0)),
            ("socket", Rect::new(80.0, 80.0, 20.0, 20.0)),
        ]);
        assert_eq!(tidiness(&together, &AREA).grouping, 0.0);
        assert!(
            tidiness(&scattered, &AREA).grouping > 0.5,
            "two 20 mm sockets in opposite corners enclose most of the drawer"
        );
    }

    /// Two objects that happen to be adjacent are not a group: `grouping` is per
    /// object id, or a layout would score better for mixing everything up.
    #[test]
    fn two_different_objects_side_by_side_are_not_a_group() {
        let mixed = layout(&[
            ("socket", Rect::new(0.0, 0.0, 20.0, 20.0)),
            ("spanner", Rect::new(80.0, 80.0, 20.0, 20.0)),
        ]);
        assert_eq!(
            tidiness(&mixed, &AREA).grouping,
            0.0,
            "one instance each is one instance each, wherever the two stand"
        );
    }

    #[test]
    fn a_layout_crowded_into_a_corner_is_less_balanced_than_one_about_the_middle() {
        let corner = layout(&[("a", Rect::new(0.0, 0.0, 20.0, 20.0))]);
        let middle = layout(&[("a", Rect::new(40.0, 40.0, 20.0, 20.0))]);
        assert!(tidiness(&middle, &AREA).balance < 1e-9);
        assert!(
            tidiness(&corner, &AREA).balance > 0.5,
            "a claim in the corner sits most of a half diagonal from the centre"
        );
    }

    #[test]
    fn an_empty_layout_reads_as_nothing_rather_than_as_untidy() {
        assert_eq!(tidiness(&[], &AREA), Tidiness::default());
        assert_eq!(score(&Tidiness::default()), 0.0);
    }
}
