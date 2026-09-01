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
//! What the band passes cannot reach is leftover that is not a full band of
//! any slab -- the L between three claims, which is most of what a real
//! two-dimensional packing leaves. So a third pass *grows* each claim face by
//! face into the space directly in front of it, which is a different safety
//! argument and a weaker one: not that the move cannot change which claims
//! touch which, but simply that the ground taken is ground the cavity covers
//! and no other claim stands on. It runs last, over what the two band passes
//! settled.

use super::pack::Placement;
use super::rects::{Rect, parts_bounds, quantize, rect_contains, rect_covered_by, rects_overlap};

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
/// given, how many free bands were absorbed, how many slabs had the slack at
/// their two ends evened out, and how many claim faces were grown into leftover
/// no band reached.
#[derive(Clone, Debug, PartialEq)]
pub struct Settled {
    pub placements: Vec<Placement>,
    pub absorbed: usize,
    pub evened: usize,
    pub grown: usize,
}

impl Settled {
    /// Whether the pass changed the layout at all, which is what a report saying
    /// so reads.
    pub fn moved(&self) -> bool {
        self.absorbed > 0 || self.evened > 0 || self.grown > 0
    }
}

/// The most a placement's claim may measure on each axis, in millimetres, or
/// `None` on an axis it is not held to. In the drawer's frame, not the object's:
/// a caller holding a `Placement` has already turned the object, so it turns the
/// limits with it.
pub type Extents = [Option<f64>; 2];

/// What clamping a settled layout took: the claims, and how many of them a limit
/// actually pulled back in.
#[derive(Clone, Debug, PartialEq)]
pub struct Clamped {
    pub placements: Vec<Placement>,
    pub clamped: usize,
}

/// The layout with every claim pulled back to the extents its object asks to be
/// held to, one `Extents` per placement in the order they are given.
///
/// This runs **after** `settle`, and undoes part of it on purpose. Settling
/// grows a compartment into whatever leftover faces it, which is right for
/// almost everything and wrong for an object that has to be held still: a
/// battery in a compartment 30 mm wider than itself lies over at an angle and is
/// no longer a battery you can pick up by the end. So an object may state the
/// most its compartment is allowed to become, and the space given back becomes
/// material like any other leftover the fit did not claim.
///
/// A claim is pulled back **about its own centre**, so what the compartment
/// keeps is the middle of what it had and the object stays where the drawn box
/// says it is. Every part of the placement is clipped to that window, which is
/// what keeps a multi-box object's parts in the one compartment they were packed
/// as. Nothing grows, nothing moves outside the cavity it already stood in, and
/// no two claims can newly overlap, so the properties `settle` established
/// survive the pass.
///
/// The limits are the *claim's*, not the object's: a caller states
/// `max_size + 2 * margin`, the same arithmetic that turned the object's size
/// into its claim in the first place.
pub fn clamp(placements: &[Placement], extents: &[Extents]) -> Clamped {
    assert_eq!(
        placements.len(),
        extents.len(),
        "every placement is held to its own extents, so there is one per placement"
    );
    let mut out = placements.to_vec();
    let mut clamped = 0;
    for (placement, limit) in out.iter_mut().zip(extents) {
        let mut pulled = false;
        for axis in [Axis::X, Axis::Y] {
            let Some(most) = limit[axis.index()] else {
                continue;
            };
            assert!(
                most > 0.0 && most.is_finite(),
                "a compartment of {} is held to at most {most} mm, which is not an extent",
                placement.object_id
            );
            let bounds = parts_bounds(&placement.parts);
            let span = axis.hi(&bounds) - axis.lo(&bounds);
            if span <= most + SAME_MM {
                continue;
            }
            let lo = 0.5 * (axis.lo(&bounds) + axis.hi(&bounds) - most);
            for part in &mut placement.parts {
                let (a, b) = (axis.lo(part).max(lo), axis.hi(part).min(lo + most));
                assert!(
                    b > a,
                    "clamping {} to {most} mm leaves one of its boxes nothing at all, so the \
                     limit is smaller than the object it was stated for",
                    placement.object_id
                );
                *part = axis.spanned(part, a, b);
            }
            pulled = true;
        }
        if pulled {
            clamped += 1;
        }
    }
    for (after, before) in out.iter().zip(placements) {
        for (a, b) in after.parts.iter().zip(&before.parts) {
            assert!(
                a.width <= b.width + SAME_MM && a.depth <= b.depth + SAME_MM,
                "clamping {} grew {b:?} to {a:?}",
                after.object_id
            );
        }
    }
    Clamped {
        placements: out,
        clamped,
    }
}

/// The four sides a claim can be pushed out on, in the order `grow` tries them.
const FACES: [Face; 4] = [Face::MinX, Face::MaxX, Face::MinY, Face::MaxY];

/// One side of a claim, as the three questions growing it asks: what the claim
/// becomes when that side moves out by a distance, how far that side is from
/// another rectangle's facing side, and how far it is from the far side of a
/// cavity rectangle it might grow into.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Face {
    MinX,
    MaxX,
    MinY,
    MaxY,
}

impl Face {
    /// The claim with this side moved out by `d`, the other three unmoved.
    fn grown(self, r: &Rect, d: f64) -> Rect {
        match self {
            Face::MinX => Rect::new(r.x - d, r.y, r.width + d, r.depth),
            Face::MaxX => Rect::new(r.x, r.y, r.width + d, r.depth),
            Face::MinY => Rect::new(r.x, r.y - d, r.width, r.depth + d),
            Face::MaxY => Rect::new(r.x, r.y, r.width, r.depth + d),
        }
    }

    /// How far this side of `r` stands from the side of `other` that faces it,
    /// or 0 where `other` is behind it. A candidate distance, not a bound: a
    /// rectangle that does not stand in front of this face at all still names
    /// the distance at which it would be met, and `grow_face` rejects it by
    /// testing the grown claim rather than by reasoning about which is which.
    fn gap(self, r: &Rect, other: &Rect) -> f64 {
        let d = match self {
            Face::MinX => quantize(r.x) - other.right(),
            Face::MaxX => quantize(other.x) - r.right(),
            Face::MinY => quantize(r.y) - other.bottom(),
            Face::MaxY => quantize(other.y) - r.bottom(),
        };
        d.max(0.0)
    }

    /// How far this side of `r` would move to land on the far side of `cover`,
    /// which is where a claim growing into a cavity rectangle stops.
    fn reach(self, r: &Rect, cover: &Rect) -> f64 {
        let d = match self {
            Face::MinX => quantize(r.x) - quantize(cover.x),
            Face::MaxX => cover.right() - r.right(),
            Face::MinY => quantize(r.y) - quantize(cover.y),
            Face::MaxY => cover.bottom() - r.bottom(),
        };
        d.max(0.0)
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
    /// Which of a pair of per-axis values is this axis's, x first.
    fn index(self) -> usize {
        match self {
            Axis::X => 0,
            Axis::Y => 1,
        }
    }

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
    grown: usize,
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
        grown: 0,
    };
    if !items.is_empty() {
        let slab = cavity_bounds(&relax.parts, &items, region);
        relax.settle_slab(slab, &items);
        relax.grow();
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
        grown: relax.grown,
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

    /// Every claim grown into the leftover directly in front of each of its four
    /// faces, to a fixed point, no face moving more than `absorb`.
    ///
    /// This is the move the band passes cannot make. A free band is a strong
    /// thing to have and most layouts do not have one: bin 1 of
    /// `examples/ikea-alex-drawer-1.toml` holds four objects in a proper
    /// two-dimensional packing where **every** row and column of the bin is
    /// covered by some claim, so `settle_slab` finds nothing to do and the 32 mm
    /// beside the swiss army knife stays air. Face by face it is reachable: the
    /// knife's `+x` face has nothing in front of it for the whole of its own
    /// depth.
    ///
    /// The safety argument is direct rather than structural. Growing one face
    /// sweeps the rectangle between that face and the first thing in front of
    /// it, and a distance is taken only if the grown claim is still covered by
    /// the cavity and still overlaps no claim of another placement -- so the
    /// three properties `check` states hold move by move, not merely at the end.
    /// Two parts of *one* placement may overlap and legitimately do, which is
    /// why `owner` is consulted rather than the part index.
    ///
    /// Deterministic, and **one pass**: faces are tried in part order and in the
    /// fixed order given by `FACES`, so where two claims want the same ground the
    /// earlier part takes it, and each face is offered its move exactly once so
    /// `absorb` bounds how far a wall travels rather than how far it travels per
    /// round. A second round could find nothing anyway -- a claim only ever
    /// grows, so what blocks a face is never afterwards out of the way.
    fn grow(&mut self) {
        if self.absorb <= 0.0 {
            return;
        }
        for index in 0..self.parts.len() {
            for face in FACES {
                if self.grow_face(index, face) {
                    self.grown += 1;
                }
            }
        }
    }

    /// One face of one claim pushed out as far as the cavity and the other
    /// placements allow, capped at `absorb`; whether it moved at all.
    ///
    /// The distance is chosen from the finite set of distances at which anything
    /// changes -- every cavity rectangle's far edge and every foreign claim's
    /// near edge, plus the cap -- so the face lands exactly on whatever stops it
    /// rather than a bisection's guess at where that is. Candidates are tried
    /// longest first and the first admissible one is taken, which is the largest
    /// by construction.
    fn grow_face(&mut self, index: usize, face: Face) -> bool {
        let part = self.parts[index];
        let mut steps: Vec<f64> = vec![self.absorb];
        for r in self.region {
            steps.push(face.reach(&part, r));
        }
        for other in 0..self.parts.len() {
            if self.owner[other] != self.owner[index] {
                steps.push(face.gap(&part, &self.parts[other]));
            }
        }
        steps.retain(|d| *d > SAME_MM && *d <= self.absorb + SAME_MM);
        steps.sort_by(f64::total_cmp);
        steps.dedup();
        for step in steps.into_iter().rev() {
            let grown = face.grown(&part, step);
            if !rect_covered_by(&grown, self.region) {
                continue;
            }
            if (0..self.parts.len()).any(|other| {
                self.owner[other] != self.owner[index] && rects_overlap(&grown, &self.parts[other])
            }) {
                continue;
            }
            assert!(
                grown.width >= part.width - SAME_MM && grown.depth >= part.depth - SAME_MM,
                "growing {part:?} by {step} mm returned the smaller {grown:?}"
            );
            self.parts[index] = grown;
            return true;
        }
        false
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
    use super::super::rects::{Rotation, union_area};

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

    /// The same bin with no band worth absorbing whole: the 5 mm ring is wider
    /// than `absorb`, so the band pass leaves it and evens it into 5 mm at each
    /// end -- and then the growth pass pushes each of the four walls out by the
    /// 2 mm a wall may move, leaving 3 mm all round. The compartment is still
    /// centred, which is what the evening was for.
    #[test]
    fn a_band_too_wide_to_absorb_is_centred_and_then_taken_a_wall_at_a_time() {
        let settled = settle(&[claim("a", 0.0, 0.0, 90.0, 90.0)], &BIN, Settle { absorb: 2.0 });
        assert_eq!(part(&settled, 0), Rect::new(3.0, 3.0, 94.0, 94.0));
        assert_eq!((settled.absorbed, settled.evened, settled.grown), (0, 2, 4));
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

    /// A compartment held to an extent keeps the middle of what settling gave
    /// it, and gives the rest back as material.
    ///
    /// The 100 mm bin is one claim's for the taking, so settling grows it to the
    /// whole of it; holding it to 60 mm on x leaves it 60 mm centred, and the
    /// 20 mm either side is material. The y axis is unheld and keeps everything.
    #[test]
    fn a_claim_held_to_an_extent_keeps_the_middle_of_what_it_grew_to() {
        let settled = settle(&[claim("a", 20.0, 20.0, 30.0, 30.0)], &BIN, Settle { absorb: 40.0 });
        assert_eq!(part(&settled, 0), Rect::new(0.0, 0.0, 100.0, 100.0));

        let held = clamp(&settled.placements, &[[Some(60.0), None]]);
        assert_eq!(held.clamped, 1);
        assert_eq!(
            held.placements[0].parts[0],
            Rect::new(20.0, 0.0, 60.0, 100.0),
            "60 mm of the 100 it had, centred, and the full depth it was not held to"
        );
    }

    /// An extent no smaller than what the claim already measures changes nothing
    /// and is not counted, so a report saying a compartment was pulled back means
    /// one was.
    #[test]
    fn an_extent_wider_than_the_claim_pulls_nothing_back() {
        let placements = [claim("a", 10.0, 10.0, 30.0, 30.0)];
        let held = clamp(&placements, &[[Some(30.0), Some(80.0)]]);
        assert_eq!(held.placements, placements);
        assert_eq!(held.clamped, 0);
    }

    /// Every box of a multi-box object is clipped to the one window, so an L
    /// held on one axis stays the one compartment it was packed as rather than
    /// falling into two.
    #[test]
    fn holding_a_multi_box_claim_clips_every_box_to_the_one_window() {
        let ell = Placement {
            object_id: "ell".to_string(),
            instance: 0,
            rotation: Rotation::Deg0,
            parts: vec![Rect::new(0.0, 0.0, 80.0, 20.0), Rect::new(0.0, 20.0, 20.0, 60.0)],
        };
        let held = clamp(&[ell], &[[Some(50.0), None]]);
        assert_eq!(
            held.placements[0].parts,
            vec![Rect::new(15.0, 0.0, 50.0, 20.0), Rect::new(15.0, 20.0, 5.0, 60.0)],
            "both boxes are cut to the 50 mm window centred on the 80 mm the object spans"
        );
    }

    /// The case the band passes cannot see, and the reason the growth pass
    /// exists. Three claims interlock so that **every** row and column of the
    /// bin is covered by one of them -- `a` across the top, `b` and `c` side by
    /// side below -- so there is no free band anywhere, at any level of the
    /// recursion, and `settle_slab` correctly finds nothing to do. The 20 mm
    /// beside `a` is still leftover, and it is reachable one wall at a time.
    ///
    /// This is bin 1 of `examples/ikea-alex-drawer-1.toml` in miniature: four
    /// objects packed two-dimensionally, 32 mm of air beside the swiss army
    /// knife, and a band pass that reports nothing absorbed.
    #[test]
    fn leftover_that_is_no_bands_at_all_is_still_taken_face_by_face() {
        let placements = [
            claim("a", 0.0, 0.0, 80.0, 40.0),
            claim("b", 0.0, 40.0, 50.0, 60.0),
            claim("c", 50.0, 40.0, 50.0, 60.0),
        ];
        let banded = settle(&placements, &BIN, Settle { absorb: 0.0 });
        assert_eq!(
            banded.absorbed, 0,
            "no row or column of this bin is free, so the band pass has nothing to absorb"
        );
        assert_eq!(banded.placements, placements, "and it moves nothing");

        let settled = settle(&placements, &BIN, Settle { absorb: 25.0 });
        assert_eq!(
            part(&settled, 0),
            Rect::new(0.0, 0.0, 100.0, 40.0),
            "a takes the 20 mm beside it, which no band of any slab covers"
        );
        assert_eq!(part(&settled, 1), Rect::new(0.0, 40.0, 50.0, 60.0));
        assert_eq!(part(&settled, 2), Rect::new(50.0, 40.0, 50.0, 60.0));
        assert_eq!(
            union_area(&[part(&settled, 0), part(&settled, 1), part(&settled, 2)]),
            BIN[0].area(),
            "the three compartments now cover the bin exactly"
        );
    }

    /// Leftover wider than the two walls facing it may move stays material: the
    /// 20 mm strip between these two claims gives 5 mm to each and keeps 10, and
    /// at `absorb` 0 it keeps all of it. `absorb` is one lever with one meaning
    /// -- how far a compartment wall may be pushed out into leftover -- and both
    /// passes are held to it.
    #[test]
    fn leftover_worth_keeping_is_left_where_it_is() {
        let placements = [
            claim("a", 0.0, 0.0, 40.0, 100.0),
            claim("b", 60.0, 0.0, 40.0, 100.0),
        ];
        let settled = settle(&placements, &BIN, Settle { absorb: 5.0 });
        assert_eq!(
            (part(&settled, 0), part(&settled, 1)),
            (
                Rect::new(0.0, 0.0, 45.0, 100.0),
                Rect::new(55.0, 0.0, 45.0, 100.0)
            ),
            "each wall takes the 5 mm it may and 10 mm of the 20 mm strip survives"
        );
        let tighter = settle(&placements, &BIN, Settle { absorb: 0.0 });
        assert_eq!(tighter.placements, placements);
        assert!(!tighter.moved(), "and at absorb 0 nothing moves at all");
    }

    /// A bin whose cells are an L is settled without leaving the L. The
    /// compartment spreads along the arm it stands in, where the cavity really
    /// does reach both walls; the move that would centre it on the other axis
    /// would stand it over the notch, and is refused whole.
    #[test]
    fn a_move_that_would_stand_a_compartment_over_a_notch_is_refused() {
        let cells = [
            gridfinity_model::layout::GridCell { x: 0, y: 0 },
            gridfinity_model::layout::GridCell { x: 1, y: 0 },
            gridfinity_model::layout::GridCell { x: 0, y: 1 },
        ];
        let region = super::super::drawer::cavity_region(&cells, 42.0, 1.45);
        let placements = [claim("a", 1.45, 1.45, 20.0, 20.0)];
        let settled = settle(&placements, &region, Settle { absorb: 40.0 });
        assert_eq!(
            part(&settled, 0),
            Rect::new(1.45, 1.45, 81.1, 39.1),
            "the compartment takes the whole 81.1 mm arm across and stops at the notch, 39.1 mm \
             down, rather than reaching into the cell the bin does not have"
        );
        assert!(
            rect_covered_by(&part(&settled, 0), &region),
            "a grown compartment stands in the cavity, notch and all"
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
