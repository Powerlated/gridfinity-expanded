//! Tidying a finished layout: the claims a search has already placed, in, and
//! the same claims sitting square in their cavity, out.
//!
//! The packer is a bottom-left first fit, so it leaves every claim against the
//! corner it scanned from and piles whatever slack remains against the far
//! edges. `settle` is the pass that reads afterwards: it *absorbs* a strip of
//! leftover too narrow to be worth the material it costs, growing the
//! compartments facing it -- which is what turns a bin that is almost entirely
//! one pocket into a bin that is exactly one pocket -- and it *evens* the slack
//! at the two ends of every slab, which is what centres a compartment in the bin
//! around it. Nothing else moves.
//!
//! Every move is made across a **free band**: a column or row of the slab that
//! no claim covers any part of. That restriction is the correctness argument
//! rather than a simplification -- a free band separates the claims completely,
//! so widening it, narrowing it or deleting it slides whole sides of the layout
//! past one another without changing which claims touch which, and a monotone
//! remap of the band lines is all any of the three is. `settle_slab` applies the
//! two operations on each axis and then recurses into the blocks the surviving
//! bands leave, which partitions the claims and so terminates.
//!
//! What it therefore cannot reach is leftover that is not a full band of its
//! slab -- an L-shaped pocket between three claims stays exactly as the packer
//! left it. Reshaping that needs a move whose safety is not the band argument.

use super::pack::Placement;
use super::rects::{Rect, quantize, rect_contains, rect_covered_by, rects_overlap};

/// How far apart two millimetre values may be and still name the same length.
/// Every coordinate here is quantised, so this only absorbs the sum of a slab's
/// own band widths against its stated extent.
const SAME_MM: f64 = 1e-6;

/// What the pass is allowed to do to the layout it is given.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Settle {
    /// The widest strip of leftover worth absorbing into the compartments facing
    /// it, in millimetres. Zero absorbs nothing and leaves the pass evening the
    /// slack out.
    pub absorb: f64,
}

/// A settled layout and what settling it took: the claims in the order they were
/// given, how many free bands were absorbed, and how many slabs had the slack at
/// their two ends evened out.
#[derive(Clone, Debug, PartialEq)]
pub struct Settled {
    pub placements: Vec<Placement>,
    pub absorbed: usize,
    pub evened: usize,
}

impl Settled {
    /// Whether the pass changed the layout at all, which is what a report saying
    /// so reads.
    pub fn moved(&self) -> bool {
        self.absorbed > 0 || self.evened > 0
    }
}

/// One of the two axes of the drawer plane, as the three accessors a band pass
/// needs: a rectangle's two coordinates on it, and a rectangle rebuilt with a new
/// span on it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Axis {
    X,
    Y,
}

impl Axis {
    /// The rectangle's minimum coordinate on this axis.
    fn lo(self, r: &Rect) -> f64 {
        match self {
            Axis::X => quantize(r.x),
            Axis::Y => quantize(r.y),
        }
    }

    /// The rectangle's maximum coordinate on this axis, quantised as
    /// `Rect::right`/`bottom` are, so a claim's far edge and the band line it
    /// stands on are one number.
    fn hi(self, r: &Rect) -> f64 {
        match self {
            Axis::X => r.right(),
            Axis::Y => r.bottom(),
        }
    }

    /// The rectangle with its span on this axis replaced and the other axis
    /// untouched.
    fn spanned(self, r: &Rect, lo: f64, hi: f64) -> Rect {
        assert!(hi >= lo, "a span runs from {lo} to {hi}, which is backwards");
        match self {
            Axis::X => Rect::new(lo, r.y, hi - lo, r.depth),
            Axis::Y => Rect::new(r.x, lo, r.width, hi - lo),
        }
    }
}

/// The claims of one layout as a flat list of boxes, with the cavity they must
/// stay inside and the widest band worth absorbing.
///
/// Flat because a band never separates the parts of one placement -- they abut,
/// so no band can pass between them -- while a band may well end on one part of a
/// placement and not another, and only the part it ends on grows. `owner` is
/// which placement each part belongs to, which is what the disjointness
/// postcondition is asked *across*: two parts of one placement legitimately
/// overlap, each being a box of one object grown by the claim margin.
struct Relax<'a> {
    parts: Vec<Rect>,
    owner: Vec<usize>,
    region: &'a [Rect],
    absorb: f64,
    absorbed: usize,
    evened: usize,
}

/// The layout with every free band of every slab absorbed where it is no wider
/// than `opts.absorb` and evened at the slab's ends otherwise, each placement
/// carrying its parts in the order it was given them.
///
/// `region` is the cavity the claims stand in, as the rectangles whose union it
/// is; every claim must already lie inside it, and every claim still does
/// afterwards. A part never shrinks, so an object that fitted its compartment
/// before the pass fits it after; a part may move, and the object it was placed
/// for moves with it, being derived from the claim rather than stored beside it.
pub fn settle(placements: &[Placement], region: &[Rect], opts: Settle) -> Settled {
    assert!(
        opts.absorb >= 0.0 && opts.absorb.is_finite(),
        "the widest band worth absorbing is a non-negative number of millimetres, not {}",
        opts.absorb
    );
    let mut parts: Vec<Rect> = Vec::new();
    let mut owner: Vec<usize> = Vec::new();
    for (index, placement) in placements.iter().enumerate() {
        for part in &placement.parts {
            assert!(
                rect_covered_by(part, region),
                "{part:?} of {} stands outside the cavity it was packed into",
                placement.object_id
            );
            parts.push(*part);
            owner.push(index);
        }
    }
    let before = parts.clone();
    let items: Vec<usize> = (0..parts.len()).collect();
    let mut relax = Relax {
        parts,
        owner,
        region,
        absorb: opts.absorb,
        absorbed: 0,
        evened: 0,
    };
    if !items.is_empty() {
        let slab = cavity_bounds(&relax.parts, &items, region);
        relax.settle_slab(slab, &items);
    }
    relax.check(&before);

    let mut out: Vec<Placement> = placements.to_vec();
    let mut at = 0;
    for placement in &mut out {
        for part in &mut placement.parts {
            *part = relax.parts[at];
            at += 1;
        }
    }
    assert_eq!(
        at,
        relax.parts.len(),
        "every settled part is written back to the placement it came from, exactly once"
    );
    Settled {
        placements: out,
        absorbed: relax.absorbed,
        evened: relax.evened,
    }
}

/// The rectangle a whole layout is settled in: the cavity's own bounding box, so
/// the slack at a slab's ends is measured against the cavity rather than against
/// the claims already standing in it.
fn cavity_bounds(parts: &[Rect], items: &[usize], region: &[Rect]) -> Rect {
    assert!(!region.is_empty(), "a cavity is at least one rectangle");
    let lo_x = region.iter().map(|r| r.x).fold(f64::INFINITY, f64::min);
    let lo_y = region.iter().map(|r| r.y).fold(f64::INFINITY, f64::min);
    let hi_x = region
        .iter()
        .map(Rect::right)
        .fold(f64::NEG_INFINITY, f64::max);
    let hi_y = region
        .iter()
        .map(Rect::bottom)
        .fold(f64::NEG_INFINITY, f64::max);
    let slab = Rect::new(lo_x, lo_y, hi_x - lo_x, hi_y - lo_y);
    for item in items {
        assert!(
            rect_contains(&slab, &parts[*item]),
            "{:?} stands outside the cavity's own bounding box {slab:?}",
            parts[*item]
        );
    }
    slab
}

impl Relax<'_> {
    /// The slab settled on both axes and then each of its blocks settled in
    /// turn, `items` naming the parts standing in it.
    ///
    /// A block is a span of the slab that no surviving free band crosses, so the
    /// blocks partition `items` and a recursion is entered only with strictly
    /// fewer parts than it was called with. That is what makes this terminate.
    fn settle_slab(&mut self, slab: Rect, items: &[usize]) {
        assert!(
            !items.is_empty(),
            "a slab is settled around the parts standing in it, and {slab:?} holds none"
        );
        self.pass(Axis::X, slab, items);
        self.pass(Axis::Y, slab, items);
        let columns = self.blocks(Axis::X, slab, items);
        let rows = self.blocks(Axis::Y, slab, items);
        for column in &columns {
            for row in &rows {
                let sub = Rect::new(column.0, row.0, column.1 - column.0, row.1 - row.0);
                let mine: Vec<usize> = items
                    .iter()
                    .copied()
                    .filter(|i| rect_contains(&sub, &self.parts[*i]))
                    .collect();
                if !mine.is_empty() && mine.len() < items.len() {
                    self.settle_slab(sub, &mine);
                }
            }
        }
    }

    /// The lines one axis of a slab is cut on: the slab's own two edges, and
    /// every claim edge strictly inside it, sorted and named once.
    fn lines(&self, axis: Axis, slab: Rect, items: &[usize]) -> Vec<f64> {
        let (lo, hi) = (axis.lo(&slab), axis.hi(&slab));
        let mut out: Vec<f64> = items
            .iter()
            .flat_map(|i| [axis.lo(&self.parts[*i]), axis.hi(&self.parts[*i])])
            .filter(|v| *v > lo && *v < hi)
            .collect();
        out.push(lo);
        out.push(hi);
        out.sort_by(f64::total_cmp);
        out.dedup();
        assert!(
            out.len() >= 2,
            "a slab of positive extent is cut into at least one band, but {slab:?} was not"
        );
        out
    }

    /// Whether a claim standing in this slab spans the band between two lines.
    /// A claim edge is itself a line, so a band is covered wholly or not at all.
    fn band_covered(&self, axis: Axis, items: &[usize], lo: f64, hi: f64) -> bool {
        items
            .iter()
            .any(|i| axis.lo(&self.parts[*i]) <= lo && hi <= axis.hi(&self.parts[*i]))
    }

    /// A slab's bands on one axis as `(widths, free, front)`, with a zero-width
    /// free band added at each end that does not already have one and `front`
    /// saying whether the first line of `lines` now stands one band in.
    ///
    /// The two end bands are what the pass evens, and a slab whose claims stand
    /// flush against one of its edges has no band there to even -- so it is given
    /// an empty one, and evening it against the slack at the far end is exactly
    /// what centres the claims in the slab.
    fn bands(
        &self,
        axis: Axis,
        slab: Rect,
        items: &[usize],
        lines: &[f64],
    ) -> (Vec<f64>, Vec<bool>, bool) {
        let mut widths: Vec<f64> = lines.windows(2).map(|w| w[1] - w[0]).collect();
        let mut free: Vec<bool> = lines
            .windows(2)
            .map(|w| !self.band_covered(axis, items, w[0], w[1]))
            .collect();
        let front = !free[0];
        if front {
            widths.insert(0, 0.0);
            free.insert(0, true);
        }
        if !free[free.len() - 1] {
            widths.push(0.0);
            free.push(true);
        }
        for pair in free.windows(2) {
            assert!(
                !(pair[0] && pair[1]),
                "two free bands of {slab:?} meet on a line no claim has an edge on"
            );
        }
        (widths, free, front)
    }

    /// One axis of one slab settled: the slack at its two ends made equal, then
    /// every free band no wider than `absorb` deleted into the claims either side
    /// of it, and every part remapped onto the lines that leaves.
    ///
    /// The pass is taken only if every part it moves still lies in the cavity; a
    /// pass that would push one into a notch the bin does not have is dropped
    /// whole, which is how a bin whose cells are an L is settled without carrying
    /// a non-rectangular slab through the recursion.
    fn pass(&mut self, axis: Axis, slab: Rect, items: &[usize]) {
        let lines = self.lines(axis, slab, items);
        let (mut widths, free, front) = self.bands(axis, slab, items, &lines);
        let front = usize::from(front);
        let last = widths.len() - 1;

        let mean = (widths[0] + widths[last]) / 2.0;
        let evened = (widths[0] - mean).abs() > SAME_MM;
        widths[0] = mean;
        widths[last] = mean;

        let mut absorbed = 0;
        for band in 0..widths.len() {
            if !free[band] || widths[band] <= 0.0 || widths[band] > self.absorb {
                continue;
            }
            assert!(
                last > 0,
                "a slab whose only band is free holds no claim, but {slab:?} was settled around {} of them",
                items.len()
            );
            let width = widths[band];
            widths[band] = 0.0;
            if band > 0 && band < last {
                widths[band - 1] += width / 2.0;
                widths[band + 1] += width / 2.0;
            } else if band > 0 {
                widths[band - 1] += width;
            } else {
                widths[band + 1] += width;
            }
            absorbed += 1;
        }
        if !evened && absorbed == 0 {
            return;
        }

        let (lo, hi) = (axis.lo(&slab), axis.hi(&slab));
        let mut prefix: Vec<f64> = Vec::with_capacity(widths.len() + 1);
        let mut at = lo;
        prefix.push(at);
        for width in &widths {
            at += *width;
            prefix.push(at);
        }
        assert!(
            (at - hi).abs() < SAME_MM,
            "settling {slab:?} changed its extent: its bands now sum to {at}, where it ends at {hi}"
        );
        let moved: Vec<(usize, Rect)> = items
            .iter()
            .map(|i| {
                let part = self.parts[*i];
                let start = line_index(&lines, axis.lo(&part)) + front;
                let end = line_index(&lines, axis.hi(&part)) + front;
                (*i, axis.spanned(&part, prefix[start], prefix[end]))
            })
            .collect();
        if !moved
            .iter()
            .all(|(_, part)| rect_covered_by(part, self.region))
        {
            return;
        }
        for (index, part) in moved {
            let was = self.parts[index];
            assert!(
                axis.hi(&part) - axis.lo(&part) >= axis.hi(&was) - axis.lo(&was) - SAME_MM,
                "settling shrank {was:?} to {part:?}, so its object no longer fits it"
            );
            self.parts[index] = part;
        }
        self.absorbed += absorbed;
        self.evened += usize::from(evened);
    }

    /// The spans of a slab's axis that carry claims, in order: the runs of band
    /// that no surviving free band crosses, in the settled coordinates.
    fn blocks(&self, axis: Axis, slab: Rect, items: &[usize]) -> Vec<(f64, f64)> {
        let lines = self.lines(axis, slab, items);
        let mut out: Vec<(f64, f64)> = Vec::new();
        let mut open: Option<f64> = None;
        for band in 0..lines.len() - 1 {
            match (
                self.band_covered(axis, items, lines[band], lines[band + 1]),
                open,
            ) {
                (true, None) => open = Some(lines[band]),
                (false, Some(start)) => {
                    out.push((start, lines[band]));
                    open = None;
                }
                _ => {}
            }
        }
        if let Some(start) = open {
            out.push((start, lines[lines.len() - 1]));
        }
        assert!(
            !out.is_empty(),
            "a slab holding {} part(s) has at least one span carrying them",
            items.len()
        );
        out
    }

    /// The postconditions of a whole settling, against the claims it started
    /// from: nothing lost, nothing shrunk, no two claims overlapping, nothing
    /// outside the cavity.
    fn check(&self, before: &[Rect]) {
        assert_eq!(
            self.parts.len(),
            before.len(),
            "settling neither adds a compartment nor loses one"
        );
        for (after, was) in self.parts.iter().zip(before) {
            assert!(
                after.width >= was.width - SAME_MM && after.depth >= was.depth - SAME_MM,
                "settling shrank {was:?} to {after:?}, so its object no longer fits it"
            );
            assert!(
                rect_covered_by(after, self.region),
                "settling moved {was:?} to {after:?}, which stands outside the cavity"
            );
        }
        for (index, one) in self.parts.iter().enumerate() {
            for (across, other) in self.parts.iter().enumerate().skip(index + 1) {
                assert!(
                    self.owner[index] == self.owner[across] || !rects_overlap(one, other),
                    "settling left {one:?} and {other:?} claiming the same space"
                );
            }
        }
    }
}

/// The index of the band line at `value`, which must be one of them.
fn line_index(lines: &[f64], value: f64) -> usize {
    lines
        .binary_search_by(|line| line.total_cmp(&value))
        .unwrap_or_else(|_| panic!("{value} is not one of the band lines {lines:?}"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::rects::Rotation;

    const BIN: [Rect; 1] = [Rect {
        x: 0.0,
        y: 0.0,
        width: 100.0,
        depth: 100.0,
    }];

    /// One claim of one box, named so a failure says which.
    fn claim(id: &str, x: f64, y: f64, width: f64, depth: f64) -> Placement {
        Placement {
            object_id: id.to_string(),
            instance: 0,
            rotation: Rotation::Deg0,
            parts: vec![Rect::new(x, y, width, depth)],
        }
    }

    /// The one part of a settled layout's nth placement.
    fn part(settled: &Settled, index: usize) -> Rect {
        settled.placements[index].parts[0]
    }

    /// A bin a single compartment nearly fills becomes a bin that compartment
    /// fills exactly: the 5 mm ring of leftover around it is narrower than the
    /// widest band worth keeping, so it is absorbed on both axes.
    #[test]
    fn a_compartment_that_nearly_fills_its_bin_is_grown_to_fill_it() {
        let settled = settle(&[claim("a", 0.0, 0.0, 90.0, 90.0)], &BIN, Settle { absorb: 20.0 });
        assert_eq!(part(&settled, 0), Rect::new(0.0, 0.0, 100.0, 100.0));
        assert_eq!(settled.absorbed, 4, "one band at each end of each axis");
    }

    /// The same bin with nothing worth absorbing: the compartment keeps its size
    /// and the leftover is split evenly between the two ends of each axis, which
    /// is the compartment standing centred in its bin.
    #[test]
    fn a_compartment_too_small_to_grow_is_centred_instead() {
        let settled = settle(&[claim("a", 0.0, 0.0, 90.0, 90.0)], &BIN, Settle { absorb: 2.0 });
        assert_eq!(part(&settled, 0), Rect::new(5.0, 5.0, 90.0, 90.0));
        assert_eq!((settled.absorbed, settled.evened), (0, 2));
    }

    /// A sliver against the bin wall and a narrow gap between two compartments
    /// both go: the two compartments meet on the line the divider already stood
    /// on, and each reaches the wall it faces.
    #[test]
    fn a_sliver_and_a_narrow_gap_are_absorbed_into_the_compartments_facing_them() {
        let settled = settle(
            &[
                claim("a", 0.0, 0.0, 40.0, 100.0),
                claim("b", 42.0, 0.0, 55.0, 100.0),
            ],
            &BIN,
            Settle { absorb: 3.5 },
        );
        assert_eq!(part(&settled, 0), Rect::new(0.0, 0.0, 42.5, 100.0));
        assert_eq!(part(&settled, 1), Rect::new(42.5, 0.0, 57.5, 100.0));
    }

    /// Leftover wide enough to be worth keeping is kept, and a layout already
    /// flush against both ends of both axes has nothing to even, so the pass
    /// returns it untouched.
    #[test]
    fn leftover_worth_keeping_is_left_where_it_is() {
        let placements = [
            claim("a", 0.0, 0.0, 40.0, 100.0),
            claim("b", 60.0, 0.0, 40.0, 100.0),
        ];
        let settled = settle(&placements, &BIN, Settle { absorb: 5.0 });
        assert_eq!(settled.placements, placements);
        assert!(!settled.moved());
    }

    /// A bin whose cells are an L is settled without leaving the L. The
    /// compartment spreads along the arm it stands in, where the cavity really
    /// does reach both walls; the move that would centre it on the other axis
    /// would stand it over the notch, and is refused whole.
    #[test]
    fn a_move_that_would_stand_a_compartment_over_a_notch_is_refused() {
        let cells = [
            crate::layout::GridCell { x: 0, y: 0 },
            crate::layout::GridCell { x: 1, y: 0 },
            crate::layout::GridCell { x: 0, y: 1 },
        ];
        let region = super::super::drawer::cavity_region(&cells, 42.0, 1.45);
        let placements = [claim("a", 1.45, 1.45, 20.0, 20.0)];
        let settled = settle(&placements, &region, Settle { absorb: 40.0 });
        assert_eq!(
            part(&settled, 0),
            Rect::new(1.45, 1.45, 81.1, 20.0),
            "the arm is 81.1 mm of cavity across and the compartment may have all of it"
        );
    }

    /// An object of several boxes keeps every one of them, and the boxes still
    /// touch: a free band never passes between two parts of one placement, so
    /// the two grow and move together.
    #[test]
    fn an_l_shaped_compartment_stays_one_connected_shape() {
        let placements = [Placement {
            object_id: "l".to_string(),
            instance: 0,
            rotation: Rotation::Deg0,
            parts: vec![Rect::new(0.0, 0.0, 60.0, 30.0), Rect::new(0.0, 30.0, 30.0, 60.0)],
        }];
        let settled = settle(&placements, &BIN, Settle { absorb: 2.0 });
        let parts = &settled.placements[0].parts;
        assert_eq!(parts.len(), 2);
        assert!(
            super::super::rects::parts_connected(parts),
            "settling parted {parts:?}"
        );
    }

    /// Absorbing nothing is what `absorb: 0` means, and it is the setting a run
    /// turns the growth off with; the evening out still happens.
    #[test]
    fn absorbing_nothing_still_centres_the_layout() {
        let settled = settle(&[claim("a", 0.0, 0.0, 90.0, 90.0)], &BIN, Settle { absorb: 0.0 });
        assert_eq!(part(&settled, 0), Rect::new(5.0, 5.0, 90.0, 90.0));
        assert_eq!(settled.absorbed, 0);
    }
}
