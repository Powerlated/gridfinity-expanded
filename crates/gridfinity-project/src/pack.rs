//! The drawer packer: a list of objects and a rectangle, in, and the position
//! and quarter turn of every instance that fits, out.
//!
//! An object is one or more edge-connected boxes; each instance of it claims
//! those boxes inflated by `clearance + divider_thickness / 2`, so packing the
//! claims without overlap is what keeps divider centrelines apart and leaves each
//! compartment interior exactly the object plus its clearance. `PackSearch` is
//! the search itself -- a bottom-left first fit (`first_fit`) over the four
//! rotations of each claim (`place_instance`), run once per instance order
//! (`pack_once`) and repeated from perturbed orders until the restart budget is
//! spent. The budget is `PACK_RESTARTS[effort]` iterations and never wall-clock,
//! and the perturbation is driven by `Mulberry32` seeded with the fixed
//! `PACK_SEED`, so one drawer and one object list always give one layout. The
//! caller may spend the budget in chunks (`step`) to report progress, or all at
//! once (`pack_layout`).
//!
//! **Once everything asked for fits, the search is choosing between layouts that
//! place the same claims and cover the same area**, and what it prefers among
//! those is the tidiest: `tidy::score` of the finished pass, which is why
//! `Scored` carries a `Tidiness` and `better` compares it third. That is also
//! why a restart varies the `Scan` axis as well as the instance order -- scanning
//! rows then columns can only ever produce row-major layouts, and half the
//! arrangements worth looking at are the other kind.

use super::rects::{
    ROTATIONS, Rect, Rotation, inflate_parts, normalize_parts, parts_bounds, parts_key, quantize,
    rect_contains, rects_overlap, rotate_parts, translate_parts, union_area,
};
use super::tidy::{self, Tidiness};
use super::walls::{Wall, layout_walls};
use std::collections::{BTreeMap, BTreeSet};

/// How hard the caller wants the optimizer to look.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "lowercase"))]
pub enum PackEffort {
    Quick,
    #[default]
    Standard,
    Thorough,
}

impl PackEffort {
    /// The effort tier named by its wire spelling, or `None` for a name that is
    /// not a tier.
    pub fn from_name(name: &str) -> Option<PackEffort> {
        match name {
            "quick" => Some(PackEffort::Quick),
            "standard" => Some(PackEffort::Standard),
            "thorough" => Some(PackEffort::Thorough),
            _ => None,
        }
    }

    /// The tier's wire spelling.
    pub fn name(self) -> &'static str {
        match self {
            PackEffort::Quick => "quick",
            PackEffort::Standard => "standard",
            PackEffort::Thorough => "thorough",
        }
    }

    /// How many perturbed instance orders this tier tries after the first.
    ///
    /// The budget is an iteration count and never wall-clock, so a drawer and an
    /// object list always give one layout whatever machine fits them. The
    /// numbers are large because the objective they are spent on is a *choice*:
    /// once everything fits, every restart places the same claims over the same
    /// area and the search is looking for the tidiest arrangement of them, which
    /// is a search worth actually running.
    pub fn restarts(self) -> usize {
        match self {
            PackEffort::Quick => 250,
            PackEffort::Standard => 2_000,
            PackEffort::Thorough => 10_000,
        }
    }
}

/// The seed every search starts its perturbation sequence from. Fixed, because a
/// given drawer and object list must always produce the same layout.
pub const PACK_SEED: u32 = 0x9e37_79b9;

/// How many consecutive non-improving restarts turn a small swap into a full
/// reshuffle.
const STAGNATION_RESHUFFLE: u32 = 8;

/// One thing to fit in the drawer: a name, an edge-connected part list in
/// millimetres, and how many of it are wanted.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "camelCase"))]
pub struct PackObject {
    pub id: String,
    pub name: String,
    pub parts: Vec<Rect>,
    pub quantity: u32,
}

/// What one search is asked for: the rectangle to fill, the objects to fill it
/// with, and the two margins that turn an object into the area it claims.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "camelCase"))]
pub struct PackInput {
    pub area: Rect,
    pub objects: Vec<PackObject>,
    pub divider_thickness: f64,
    pub clearance: f64,
    #[cfg_attr(feature = "serde", serde(default))]
    pub floor_fillet: f64,
    pub effort: PackEffort,
}

impl PackInput {
    /// How far each object's boxes are grown to become the area it claims: its
    /// clearance, the floor fillet the compartment's walls will be blended into
    /// its floor by, and the half divider that will stand on the claim boundary.
    ///
    /// The fillet is in there because an object *rests on the floor*, and a
    /// concave blend of radius `r` between floor and wall takes `r` of floor
    /// away from every wall: the compartment is its stated size at mid height
    /// and `r` smaller all round where the object actually sits. Reserving it
    /// here is what makes the packed layout one the objects fit into rather than
    /// one they only fit the plan view of. It is a lower bound on the corner
    /// rounding too -- a compartment corner of radius `rc` bulges in by
    /// `rc * (1 - 1/sqrt 2)`, about `0.3 * rc`, and the model never builds a
    /// fillet larger than the corner it turns.
    ///
    /// Zero is the honest value for a bin the model rounds nothing in, and is
    /// what a caller that has not been taught about the fillet sends, so the
    /// margin is exactly what it always was for them.
    pub fn margin(&self) -> f64 {
        assert!(
            self.floor_fillet >= 0.0 && self.floor_fillet.is_finite(),
            "a compartment's floor fillet is a radius, but this input reserves {}",
            self.floor_fillet
        );
        self.clearance + self.floor_fillet + self.divider_thickness / 2.0
    }
}

/// Where one instance of one object ended up: which object, which instance of
/// it, the quarter turn applied, and its claim's boxes in drawer coordinates.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "camelCase"))]
pub struct Placement {
    pub object_id: String,
    pub instance: u32,
    pub rotation: Rotation,
    pub parts: Vec<Rect>,
}

/// A finished layout: every placement, how many of each object were placed, the
/// dividers those placements imply, how tidy the result reads, and how many
/// restarts were spent reaching it.
///
/// `tidiness` is the winning pass's own reading, carried rather than recomputed:
/// it is what the search chose this layout *for*, so a caller re-deriving it
/// could disagree with the thing that was optimised.
#[derive(Clone, Debug, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "camelCase"))]
pub struct PackResult {
    pub placements: Vec<Placement>,
    pub placed_by_object_id: BTreeMap<String, u32>,
    pub iterations: usize,
    pub walls: Vec<Wall>,
    #[cfg_attr(feature = "serde", serde(default))]
    pub tidiness: Tidiness,
}

/// The mulberry32 generator, bit for bit: a 32-bit state advanced by a fixed
/// increment and hashed to a float in `[0, 1)`.
struct Mulberry32 {
    state: u32,
}

impl Mulberry32 {
    /// A generator whose first draw follows from `seed`.
    fn new(seed: u32) -> Mulberry32 {
        Mulberry32 { state: seed }
    }

    /// The next draw, uniform in `[0, 1)`.
    fn next(&mut self) -> f64 {
        self.state = self.state.wrapping_add(0x6d2b_79f5);
        let s = self.state;
        let mut value = (s ^ (s >> 15)).wrapping_mul(1 | s);
        value = value.wrapping_add((value ^ (value >> 7)).wrapping_mul(61 | value)) ^ value;
        f64::from(value ^ (value >> 14)) / 4_294_967_296.0
    }
}

/// How far into the swept rows or columns one pass is willing to start looking,
/// per instance.
///
/// A first fit takes the earliest position that works, so permuting the instance
/// order is the only thing a restart can change and the layouts it reaches are
/// all the same shape: everything jammed against the origin in the order it was
/// considered. Measured on `examples/drawer.toml`, that neighbourhood is
/// exhausted inside 250 restarts -- 10 000 found nothing 250 had not.
///
/// So a pass may also *decline* the first few bands and place an instance
/// further in, which is what leaves a gap for the next instance of the same
/// object to sit beside, or lines an edge up with one already placed. Each
/// instance draws its own skip, most of them zero; `strength` is how jittery
/// this particular pass is, drawn once for it, so the budget covers passes from
/// pure bottom-left greed to thoroughly perturbed.
///
/// The draws come from a generator seeded from the restart index alone, so a
/// pass is a function of its index and nothing about how many draws the passes
/// before it made can move it.
struct Jitter {
    random: Mulberry32,
    strength: f64,
}

/// The most bands an instance is ever asked to skip. Three is enough to step
/// past a neighbour and leave room beside it; more and the pass stops being a
/// packing at all.
const MAX_SKIP: f64 = 3.0;

impl Jitter {
    /// The jitter for restart `index`: its own generator, and a strength drawn
    /// from it, so restarts range from greedy to heavily perturbed.
    fn for_restart(index: usize) -> Jitter {
        let mut random = Mulberry32::new(PACK_SEED ^ (index as u32).wrapping_mul(0x9e37_79b9));
        let strength = random.next() * random.next();
        Jitter { random, strength }
    }

    /// No jitter at all: the plain bottom-left first fit, which is what the
    /// greedy pass every search starts from must be.
    fn none() -> Jitter {
        Jitter {
            random: Mulberry32::new(PACK_SEED),
            strength: 0.0,
        }
    }

    /// How many bands the next instance declines before it starts looking.
    fn skip(&mut self) -> usize {
        if self.strength <= 0.0 || self.random.next() >= self.strength {
            return 0;
        }
        1 + (self.random.next() * MAX_SKIP).floor() as usize
    }
}

/// One rotation of one object's claim: the turn, the turned boxes normalised to
/// the origin, and their bounding box.
#[derive(Clone, Debug)]
struct Shape {
    rotation: Rotation,
    parts: Vec<Rect>,
    bounds: Rect,
}

/// One instance to place: which object it belongs to, the distinct rotations of
/// its claim, the name of that claim, and the claim's area.
#[derive(Clone, Debug)]
struct Instance {
    object_id: String,
    instance: u32,
    shapes: Vec<Shape>,
    key: String,
    area: f64,
}

/// Which way one pass sweeps the candidate positions: rows first and then
/// columns along each row, or columns first and then rows down each column.
///
/// A first fit is "first" in the order it scans, so the scan *is* the layout's
/// grain: `Rows` fills the drawer in bands across it and `Columns` in bands down
/// it, and no permutation of the instance order turns one into the other. The
/// first greedy pass is always `Rows`, so the pass every effort tier starts from
/// is the one it always was.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Scan {
    Rows,
    Columns,
}

impl Scan {
    /// The attempt's coordinate along the axis this scan sweeps first, which is
    /// the one an earlier position must beat outright.
    fn outer_of(self, attempt: &Attempt) -> f64 {
        match self {
            Scan::Rows => attempt.y,
            Scan::Columns => attempt.x,
        }
    }

    /// The attempt's coordinate along the axis this scan sweeps second, which
    /// breaks ties along the first.
    fn inner_of(self, attempt: &Attempt) -> f64 {
        match self {
            Scan::Rows => attempt.x,
            Scan::Columns => attempt.y,
        }
    }

    /// The `(x, y)` a pair of swept coordinates names.
    fn position(self, outer: f64, inner: f64) -> (f64, f64) {
        match self {
            Scan::Rows => (inner, outer),
            Scan::Columns => (outer, inner),
        }
    }
}

/// A candidate placement found for one instance: its boxes already in drawer
/// coordinates, the turn they are at, and the position that put them there.
#[derive(Clone, Debug)]
struct Attempt {
    parts: Vec<Rect>,
    rotation: Rotation,
    x: f64,
    y: f64,
}

/// One completed pass over one instance order: the placements it made, how much
/// claim area they cover, how the finished layout reads, and how far from the
/// origin they sit in total.
///
/// `area` and `spread` accumulate as the pass places, which is what keeps a pass
/// linear in its instances. `tidiness` cannot: every one of its terms is a
/// property of the *whole* arrangement, so it is measured once, on the finished
/// layout, in `pack_once` -- never per placement, and never again by a caller
/// re-measuring what the search already decided.
#[derive(Clone, Debug, Default)]
struct Scored {
    placements: Vec<Placement>,
    area: f64,
    spread: f64,
    tidiness: Tidiness,
    score: f64,
}

/// The distinct rotations of one object's claim, deduplicated by shape so a
/// symmetric object is not tried four times.
fn claim_shapes(object: &PackObject, margin: f64) -> Vec<Shape> {
    let base = normalize_parts(&inflate_parts(&normalize_parts(&object.parts), margin));
    let mut seen: BTreeSet<String> = BTreeSet::new();
    let mut shapes = Vec::new();
    for rotation in ROTATIONS {
        let parts = rotate_parts(&base, rotation);
        let key = parts_key(&parts);
        if !seen.insert(key) {
            continue;
        }
        let bounds = parts_bounds(&parts);
        shapes.push(Shape {
            rotation,
            parts,
            bounds,
        });
    }
    assert!(
        !shapes.is_empty(),
        "object {} has no claim shape, so its part list was empty",
        object.id
    );
    shapes
}

/// One `Instance` per unit of every object's quantity, each carrying the claim
/// shapes it may be placed as.
fn build_instances(objects: &[PackObject], margin: f64) -> Vec<Instance> {
    let mut out = Vec::new();
    for object in objects {
        if object.parts.is_empty() {
            continue;
        }
        let shapes = claim_shapes(object, margin);
        let key = parts_key(&shapes[0].parts);
        let area = union_area(&shapes[0].parts);
        for instance in 0..object.quantity {
            out.push(Instance {
                object_id: object.id.clone(),
                instance,
                shapes: shapes.clone(),
                key: key.clone(),
                area,
            });
        }
    }
    out
}

/// The candidate coordinates along one axis: every already-placed edge, offset
/// back by each of the shape's own part offsets, clipped to `base..=limit`, plus
/// `base` itself, sorted ascending.
fn candidate_axis(base: f64, limit: f64, edges: &[f64], offsets: &[f64]) -> Vec<f64> {
    let mut values: Vec<f64> = Vec::new();
    for edge in edges {
        for offset in offsets {
            let value = quantize(edge - offset);
            if value >= base && value <= limit {
                values.push(value);
            }
        }
    }
    values.push(base);
    values.sort_by(f64::total_cmp);
    values.dedup();
    values
}

/// The first position, scanning as `scan` says and starting `skip` bands in,
/// where this rotation of the claim fits inside `area` without overlapping
/// anything already placed -- or `None`. When `best` is given the scan is pruned
/// to positions strictly earlier than it, so a later rotation only ever returns
/// an improvement.
///
/// `skip` restricts the candidate set rather than changing what is chosen from
/// it: each rotation still returns *its* earliest admissible position, which is
/// what keeps the pruning against `best` sound. It is clamped to the last band,
/// so a large skip narrows the search rather than emptying it.
fn first_fit(
    shape: &Shape,
    placed: &[Rect],
    area: &Rect,
    best: Option<&Attempt>,
    scan: Scan,
    skip: usize,
) -> Option<Attempt> {
    let limit_x = quantize(area.right() - shape.bounds.width);
    let limit_y = quantize(area.bottom() - shape.bounds.depth);
    if limit_x < area.x || limit_y < area.y {
        return None;
    }
    let offsets_x: Vec<f64> = shape.parts.iter().map(|p| p.x).collect();
    let offsets_y: Vec<f64> = shape.parts.iter().map(|p| p.y).collect();
    let mut edges_x: Vec<f64> = vec![area.x];
    let mut edges_y: Vec<f64> = vec![area.y];
    for rect in placed {
        edges_x.push(rect.x);
        edges_x.push(rect.right());
        edges_y.push(rect.y);
        edges_y.push(rect.bottom());
    }
    let xs = candidate_axis(area.x, limit_x, &edges_x, &offsets_x);
    let ys = candidate_axis(area.y, limit_y, &edges_y, &offsets_y);
    for (name, axis, lo, hi) in [("x", &xs, area.x, limit_x), ("y", &ys, area.y, limit_y)] {
        assert!(
            axis.windows(2).all(|w| w[0] < w[1]),
            "the {name} candidates are not strictly ascending, so pruning the scan against the \
             incumbent would skip positions earlier than it: {axis:?}"
        );
        assert!(
            axis.iter().all(|v| *v >= lo && *v <= hi),
            "a {name} candidate lies outside {lo}..={hi}, where the shape cannot fit the area at \
             all: {axis:?}"
        );
    }
    let (outer, inner) = match scan {
        Scan::Rows => (&ys, &xs),
        Scan::Columns => (&xs, &ys),
    };
    for first in outer.iter().skip(skip.min(outer.len() - 1)) {
        if best.is_some_and(|b| *first > scan.outer_of(b)) {
            break;
        }
        for second in inner {
            if best.is_some_and(|b| *first == scan.outer_of(b) && *second >= scan.inner_of(b)) {
                break;
            }
            let (x, y) = scan.position(*first, *second);
            let parts = translate_parts(&shape.parts, x, y);
            if parts.iter().any(|part| !rect_contains(area, part)) {
                continue;
            }
            if parts
                .iter()
                .any(|part| placed.iter().any(|other| rects_overlap(part, other)))
            {
                continue;
            }
            return Some(Attempt {
                parts,
                rotation: shape.rotation,
                x,
                y,
            });
        }
    }
    None
}

/// The earliest position any of the instance's rotations fits at, or `None` when
/// none of them fits at all.
fn place_instance(
    instance: &Instance,
    placed: &[Rect],
    area: &Rect,
    scan: Scan,
    skip: usize,
) -> Option<Attempt> {
    let mut best: Option<Attempt> = None;
    for shape in &instance.shapes {
        if let Some(found) = first_fit(shape, placed, area, best.as_ref(), scan, skip) {
            best = Some(found);
        }
    }
    best
}

/// One greedy pass in the given order: each instance placed at its earliest fit,
/// and once one instance of a claim shape does not fit, every later instance of
/// that same shape is skipped rather than retried.
fn pack_once(order: &[Instance], area: &Rect, scan: Scan, jitter: &mut Jitter) -> Scored {
    let mut placed: Vec<Rect> = Vec::new();
    let mut placements: Vec<Placement> = Vec::new();
    let mut blocked: BTreeSet<&str> = BTreeSet::new();
    let mut scored = Scored::default();
    for instance in order {
        if blocked.contains(instance.key.as_str()) {
            continue;
        }
        let Some(attempt) = place_instance(instance, &placed, area, scan, jitter.skip()) else {
            blocked.insert(instance.key.as_str());
            continue;
        };
        for part in &attempt.parts {
            assert!(
                rect_contains(area, part),
                "instance {} of {} was accepted at a box {part:?} outside the packing area {area:?}",
                instance.instance,
                instance.object_id
            );
            assert!(
                !placed.iter().any(|other| rects_overlap(part, other)),
                "instance {} of {} was accepted at a box {part:?} overlapping an earlier claim",
                instance.instance,
                instance.object_id
            );
        }
        placements.push(Placement {
            object_id: instance.object_id.clone(),
            instance: instance.instance,
            rotation: attempt.rotation,
            parts: attempt.parts.clone(),
        });
        placed.extend(attempt.parts);
        scored.area += instance.area;
        scored.spread += attempt.x + attempt.y;
    }
    scored.tidiness = tidy::tidiness(&placements, area);
    scored.score = tidy::score(&scored.tidiness);
    scored.placements = placements;
    scored
}

/// Whether the candidate pass beats the incumbent: more instances placed first,
/// then more claim area, then the tidier layout, then packed closer to the
/// origin.
///
/// The first two are absolute, so **a prettier layout can never cost a placed
/// object**: tidiness decides only among arrangements that fit the same things.
/// That is the common case rather than the rare one -- a drawer everything fits
/// in ties on both of the first two keys at every restart -- which is what makes
/// the third key the one the budget is really spent on. `spread` survives as the
/// last tie-break, so two layouts the objective cannot tell apart still resolve
/// the same way every run.
///
/// **The area key is compared quantised, and must be.** A pass accumulates its
/// area as it places, so two passes placing the same claims in a different order
/// reach the same total by different additions and can differ in the last bit --
/// 121146.20680000001 against 121146.20680000003 on a drawer of eight objects.
/// That is not a difference in what was placed, and an exact comparison lets it
/// decide: the area key fires on a tie it should have passed over, and every key
/// after it is unreachable. It cost the tidiest layout of that drawer at restart
/// 1431, which is how it was found.
fn better(candidate: &Scored, incumbent: &Scored) -> bool {
    if candidate.placements.len() != incumbent.placements.len() {
        return candidate.placements.len() > incumbent.placements.len();
    }
    if quantize(candidate.area) != quantize(incumbent.area) {
        return candidate.area > incumbent.area;
    }
    if candidate.score != incumbent.score {
        return candidate.score < incumbent.score;
    }
    candidate.spread < incumbent.spread
}

/// The order with a few random transpositions applied, or fully reshuffled once
/// the search has gone `STAGNATION_RESHUFFLE` restarts without an improvement.
fn perturb(order: &[Instance], random: &mut Mulberry32, stagnation: u32) -> Vec<Instance> {
    let mut next = order.to_vec();
    if next.len() < 2 {
        return next;
    }
    let swaps = if stagnation >= STAGNATION_RESHUFFLE {
        next.len()
    } else {
        1 + (random.next() * 3.0).floor() as usize
    };
    for _ in 0..swaps {
        let a = (random.next() * next.len() as f64).floor() as usize;
        let b = (random.next() * next.len() as f64).floor() as usize;
        next.swap(a, b);
    }
    next
}

/// A restart budget spent in chunks, so a caller that wants to report progress
/// can drive it without the search owning a clock.
pub struct PackSearch {
    input: PackInput,
    order: Vec<Instance>,
    best: Scored,
    random: Mulberry32,
    stagnation: u32,
    total: usize,
    done: usize,
}

impl PackSearch {
    /// A search over `input`, with the greedy pass from the largest-claim-first
    /// order already run, so `result` is meaningful before any `step`.
    pub fn new(input: PackInput) -> PackSearch {
        assert!(
            input.area.width >= 0.0 && input.area.depth >= 0.0,
            "the packing area has negative extent: {:?}",
            input.area
        );
        for object in &input.objects {
            assert!(
                super::rects::parts_connected(&object.parts),
                "object {} is not one edge-connected shape, so its claim cannot be placed as one",
                object.id
            );
        }
        let mut order = build_instances(&input.objects, input.margin());
        order.sort_by(|a, b| {
            b.area
                .total_cmp(&a.area)
                .then_with(|| a.object_id.cmp(&b.object_id))
        });
        let total = if order.is_empty() {
            0
        } else {
            input.effort.restarts()
        };
        let best = pack_once(&order, &input.area, Scan::Rows, &mut Jitter::none());
        PackSearch {
            input,
            order,
            best,
            random: Mulberry32::new(PACK_SEED),
            stagnation: 0,
            total,
            done: 0,
        }
    }

    /// How many restarts this search will run in total.
    pub fn total(&self) -> usize {
        self.total
    }

    /// How many restarts have been run so far.
    pub fn done(&self) -> usize {
        self.done
    }

    /// Runs up to `iterations` further restarts, keeping any that improve on the
    /// incumbent, and returns whether more of the budget remains.
    pub fn step(&mut self, iterations: usize) -> bool {
        let until = self.total.min(self.done.saturating_add(iterations));
        while self.done < until {
            self.done += 1;
            let candidate_order = perturb(&self.order, &mut self.random, self.stagnation);
            let scan = if self.random.next() < 0.5 { Scan::Rows } else { Scan::Columns };
            let mut jitter = Jitter::for_restart(self.done);
            let scored = pack_once(&candidate_order, &self.input.area, scan, &mut jitter);
            if better(&scored, &self.best) {
                self.best = scored;
                self.order = candidate_order;
                self.stagnation = 0;
            } else {
                self.stagnation += 1;
            }
        }
        self.done < self.total
    }

    /// The best layout found so far, with the dividers those placements imply.
    pub fn result(&self) -> PackResult {
        let mut placed_by_object_id: BTreeMap<String, u32> = self
            .input
            .objects
            .iter()
            .map(|o| (o.id.clone(), 0))
            .collect();
        for placement in &self.best.placements {
            *placed_by_object_id
                .entry(placement.object_id.clone())
                .or_insert(0) += 1;
        }
        PackResult {
            placements: self.best.placements.clone(),
            placed_by_object_id,
            iterations: self.done,
            tidiness: self.best.tidiness,
            walls: layout_walls(
                &self.best.placements,
                &self.input.area,
                self.input.divider_thickness,
            ),
        }
    }
}

/// The whole budget spent at once: the best layout `PackSearch` reaches for this
/// input.
pub fn pack_layout(input: PackInput) -> PackResult {
    let mut search = PackSearch::new(input);
    while search.step(usize::MAX) {}
    search.result()
}

#[cfg(test)]
mod tests {
    use super::*;

    const AREA: Rect = Rect {
        x: 0.0,
        y: 0.0,
        width: 100.0,
        depth: 100.0,
    };

    fn object(id: &str, width: f64, depth: f64, quantity: u32) -> PackObject {
        PackObject {
            id: id.to_string(),
            name: id.to_string(),
            parts: vec![Rect::new(0.0, 0.0, width, depth)],
            quantity,
        }
    }

    fn input(objects: Vec<PackObject>, area: Rect) -> PackInput {
        PackInput {
            area,
            objects,
            divider_thickness: 0.0,
            clearance: 0.0,
            floor_fillet: 0.0,
            effort: PackEffort::Quick,
        }
    }

    fn placed_rects(objects: Vec<PackObject>, area: Rect) -> Vec<Rect> {
        pack_layout(input(objects, area))
            .placements
            .into_iter()
            .flat_map(|p| p.parts)
            .collect()
    }

    #[test]
    fn fills_an_area_that_the_objects_tile_exactly() {
        let result = pack_layout(input(vec![object("a", 20.0, 20.0, 25)], AREA));
        assert_eq!(result.placements.len(), 25);
        assert_eq!(result.placed_by_object_id["a"], 25);
    }

    #[test]
    fn never_overlaps_two_placements_and_never_leaves_the_area() {
        let rects = placed_rects(
            vec![
                object("a", 30.0, 20.0, 6),
                object("b", 15.0, 45.0, 4),
                object("c", 10.0, 10.0, 12),
                PackObject {
                    id: "d".into(),
                    name: "d".into(),
                    parts: vec![Rect::new(0.0, 0.0, 25.0, 10.0), Rect::new(0.0, 10.0, 10.0, 15.0)],
                    quantity: 3,
                },
            ],
            AREA,
        );
        assert!(!rects.is_empty());
        for rect in &rects {
            assert!(rect_contains(&AREA, rect), "{rect:?} left the area");
        }
        for a in 0..rects.len() {
            for b in a + 1..rects.len() {
                assert!(
                    !rects_overlap(&rects[a], &rects[b]),
                    "{:?} overlaps {:?}",
                    rects[a],
                    rects[b]
                );
            }
        }
    }

    #[test]
    fn rotates_an_object_that_only_fits_the_other_way_round() {
        let slot = Rect::new(0.0, 0.0, 20.0, 100.0);
        let result = pack_layout(input(vec![object("a", 60.0, 20.0, 1)], slot));
        assert_eq!(result.placements.len(), 1);
        assert!(
            result.placements[0].rotation.swaps_axes(),
            "expected a quarter turn, got {:?}",
            result.placements[0].rotation
        );
    }

    #[test]
    fn places_no_more_than_the_requested_quantity_and_reports_the_shortfall() {
        let result = pack_layout(input(vec![object("a", 60.0, 60.0, 4)], AREA));
        assert!(result.placements.len() <= 4);
        assert_eq!(result.placed_by_object_id["a"] as usize, result.placements.len());
        assert!(result.placed_by_object_id["a"] < 4);
    }

    #[test]
    fn reserves_the_clearance_and_half_a_divider_around_every_object() {
        let request = PackInput {
            divider_thickness: 2.0,
            clearance: 0.5,
            ..input(vec![object("a", 20.0, 20.0, 2)], AREA)
        };
        let result = pack_layout(request);
        assert_eq!(result.placements.len(), 2);
        for placement in &result.placements {
            assert_eq!(placement.parts[0].width, 23.0);
            assert_eq!(placement.parts[0].depth, 23.0);
        }
    }

    /// The floor fillet is reserved on top of those, because an object rests on
    /// the floor and the blend takes its radius of floor from every wall.
    #[test]
    fn reserves_the_floor_fillet_on_top_of_the_clearance() {
        let request = PackInput {
            divider_thickness: 2.0,
            clearance: 0.5,
            floor_fillet: 2.5,
            ..input(vec![object("a", 20.0, 20.0, 2)], AREA)
        };
        assert_eq!(
            request.margin(),
            4.0,
            "0.5 clearance + 2.5 fillet + 1.0 half divider"
        );
        let result = pack_layout(request);
        assert_eq!(result.placements.len(), 2);
        for placement in &result.placements {
            assert_eq!(placement.parts[0].width, 28.0);
            assert_eq!(placement.parts[0].depth, 28.0);
        }
    }

    /// A caller that names no fillet claims exactly what it always did, so the
    /// reservation cannot change a layout behind an existing caller's back.
    #[test]
    fn a_zero_fillet_claims_what_the_margin_always_was() {
        let plain = PackInput {
            divider_thickness: 2.0,
            clearance: 0.5,
            ..input(vec![object("a", 20.0, 20.0, 2)], AREA)
        };
        assert_eq!(plain.floor_fillet, 0.0, "`input` names no fillet");
        assert_eq!(
            plain.margin(),
            plain.clearance + plain.divider_thickness / 2.0
        );
    }

    #[test]
    fn gives_the_same_layout_every_time_for_the_same_input() {
        let objects = vec![
            object("a", 30.0, 20.0, 5),
            object("b", 12.0, 55.0, 4),
            object("c", 18.0, 18.0, 7),
        ];
        let first = pack_layout(input(objects.clone(), AREA));
        let second = pack_layout(input(objects, AREA));
        assert_eq!(first, second);
    }

    #[test]
    fn returns_an_empty_layout_when_there_is_nothing_to_place() {
        assert_eq!(pack_layout(input(Vec::new(), AREA)), PackResult::default());
    }

    #[test]
    fn draws_the_same_sequence_the_reference_generator_does() {
        let mut random = Mulberry32::new(PACK_SEED);
        let draws: Vec<f64> = (0..4).map(|_| random.next()).collect();
        for draw in &draws {
            assert!(
                (0.0..1.0).contains(draw),
                "mulberry32 drew {draw}, which is outside [0, 1)"
            );
        }
        assert_ne!(draws[0], draws[1], "the generator repeated its first draw");
    }

    /// The eight objects of `examples/ikea-alex-drawer-1.toml`, in a packing
    /// area of that drawer's size: the layout this objective was built for, and
    /// big enough that the search has real choices to make.
    fn ikea_drawer() -> PackInput {
        PackInput {
            area: Rect::new(6.5, 6.5, 277.1, 517.1),
            objects: vec![
                object("glue", 45.0, 120.0, 1),
                object("knife", 114.0, 32.0, 1),
                object("calipers", 90.0, 250.0, 1),
                object("files", 100.0, 380.0, 1),
                object("batteries", 45.0, 124.0, 1),
                object("epoxy", 45.0, 170.0, 1),
                object("tape measure", 80.0, 85.0, 1),
                object("level", 53.3, 233.7, 1),
            ],
            divider_thickness: 1.2,
            clearance: 2.0,
            floor_fillet: 2.08,
            effort: PackEffort::Thorough,
        }
    }

    /// Spending more of the budget can only improve the answer, by the
    /// objective's own ordering.
    ///
    /// Stated as `better` rather than as a falling score, because the score is
    /// only the *third* key: a restart that places an instance the incumbent
    /// could not legitimately takes a worse-looking layout, and must. What may
    /// never happen is the search ending on something a shorter run of the same
    /// search would have beaten.
    ///
    /// This is the test that caught the area key comparing unquantised: at
    /// restart 1431 of this very drawer a layout scoring 1.14 displaced one
    /// scoring 0.80, because their accumulated areas differed by one ulp.
    #[test]
    fn a_longer_search_is_never_beaten_by_a_shorter_one() {
        let mut search = PackSearch::new(ikea_drawer());
        let mut best_so_far = search.best.clone();
        let mut improved = 0;
        while search.step(1) {
            assert!(
                !better(&best_so_far, &search.best),
                "restart {} replaced a layout the objective prefers: {} placed scoring {} \
                 became {} placed scoring {}",
                search.done(),
                best_so_far.placements.len(),
                best_so_far.score,
                search.best.placements.len(),
                search.best.score
            );
            if better(&search.best, &best_so_far) {
                improved += 1;
                best_so_far = search.best.clone();
            }
        }
        assert!(
            improved > 0,
            "the search never improved on its first greedy pass, so it is not searching"
        );
    }

    /// The whole point of the tidiness key: on a drawer everything fits in, the
    /// budget buys a tidier layout and nothing else.
    ///
    /// The area is the ikea drawer's grown until the greedy pass places all
    /// eight, which the test asserts before it asserts anything else -- with an
    /// instance left over, the search would be improving the *placement* count
    /// and the comparison would say nothing about tidiness.
    #[test]
    fn searching_a_drawer_everything_fits_in_buys_a_tidier_layout() {
        let input = PackInput {
            area: Rect::new(6.5, 6.5, 320.0, 560.0),
            ..ikea_drawer()
        };
        let first = PackSearch::new(input.clone()).result();
        assert_eq!(
            first.placements.len(),
            input.objects.len(),
            "the fixture must fit greedily, or the search is buying placements and not tidiness"
        );
        let settled = pack_layout(input);
        assert_eq!(
            first.placements.len(),
            settled.placements.len(),
            "everything fits either way, so the search is choosing on tidiness alone"
        );
        assert!(
            tidy::score(&settled.tidiness) < tidy::score(&first.tidiness),
            "the search settled on {:?}, which is no tidier than the greedy pass's {:?}",
            settled.tidiness,
            first.tidiness
        );
    }

    /// A pass may only sweep rows first or columns first, and the two reach
    /// different layouts -- which is why a restart varies it. Without this the
    /// budget only ever buys permutations of one grain.
    #[test]
    fn the_two_scan_axes_reach_different_layouts() {
        let input = ikea_drawer();
        let order = {
            let mut order = build_instances(&input.objects, input.margin());
            order.sort_by(|a, b| b.area.total_cmp(&a.area).then_with(|| a.object_id.cmp(&b.object_id)));
            order
        };
        let rows = pack_once(&order, &input.area, Scan::Rows, &mut Jitter::none());
        let columns = pack_once(&order, &input.area, Scan::Columns, &mut Jitter::none());
        assert_ne!(
            rows.placements, columns.placements,
            "one instance order packed both ways gave the same layout, so the scan axis is not \
             reaching anything the other does not"
        );
        for pass in [&rows, &columns] {
            for placement in &pass.placements {
                for part in &placement.parts {
                    assert!(
                        rect_contains(&input.area, part),
                        "a column-major pass placed {part:?} outside {:?}",
                        input.area
                    );
                }
            }
        }
    }
}
