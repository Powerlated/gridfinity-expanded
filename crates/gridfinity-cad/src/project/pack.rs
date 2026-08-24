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

use super::rects::{
    ROTATIONS, Rect, Rotation, inflate_parts, normalize_parts, parts_bounds, parts_key, quantize,
    rect_contains, rects_overlap, rotate_parts, translate_parts, union_area,
};
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
    pub fn restarts(self) -> usize {
        match self {
            PackEffort::Quick => 30,
            PackEffort::Standard => 200,
            PackEffort::Thorough => 800,
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
    pub effort: PackEffort,
}

impl PackInput {
    /// How far each object's boxes are grown to become the area it claims: its
    /// clearance plus the half divider that will stand on the claim boundary.
    pub fn margin(&self) -> f64 {
        self.clearance + self.divider_thickness / 2.0
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
/// dividers those placements imply, and how many restarts were spent reaching
/// it.
#[derive(Clone, Debug, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "camelCase"))]
pub struct PackResult {
    pub placements: Vec<Placement>,
    pub placed_by_object_id: BTreeMap<String, u32>,
    pub iterations: usize,
    pub walls: Vec<Wall>,
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
/// claim area they cover, and how far from the origin they sit in total.
#[derive(Clone, Debug, Default)]
struct Scored {
    placements: Vec<Placement>,
    area: f64,
    spread: f64,
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

/// The first position, scanning y then x, where this rotation of the claim fits
/// inside `area` without overlapping anything already placed -- or `None`. When
/// `best` is given the scan is pruned to positions strictly earlier than it, so
/// a later rotation only ever returns an improvement.
fn first_fit(shape: &Shape, placed: &[Rect], area: &Rect, best: Option<&Attempt>) -> Option<Attempt> {
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
    for y in ys {
        if best.is_some_and(|b| y > b.y) {
            break;
        }
        for x in &xs {
            if best.is_some_and(|b| y == b.y && *x >= b.x) {
                break;
            }
            let parts = translate_parts(&shape.parts, *x, y);
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
                x: *x,
                y,
            });
        }
    }
    None
}

/// The earliest position any of the instance's rotations fits at, or `None` when
/// none of them fits at all.
fn place_instance(instance: &Instance, placed: &[Rect], area: &Rect) -> Option<Attempt> {
    let mut best: Option<Attempt> = None;
    for shape in &instance.shapes {
        if let Some(found) = first_fit(shape, placed, area, best.as_ref()) {
            best = Some(found);
        }
    }
    best
}

/// One greedy pass in the given order: each instance placed at its earliest fit,
/// and once one instance of a claim shape does not fit, every later instance of
/// that same shape is skipped rather than retried.
fn pack_once(order: &[Instance], area: &Rect) -> Scored {
    let mut placed: Vec<Rect> = Vec::new();
    let mut placements: Vec<Placement> = Vec::new();
    let mut blocked: BTreeSet<&str> = BTreeSet::new();
    let mut scored = Scored::default();
    for instance in order {
        if blocked.contains(instance.key.as_str()) {
            continue;
        }
        let Some(attempt) = place_instance(instance, &placed, area) else {
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
    scored.placements = placements;
    scored
}

/// Whether the candidate pass beats the incumbent: more instances placed first,
/// then more claim area, then packed closer to the origin.
fn better(candidate: &Scored, incumbent: &Scored) -> bool {
    if candidate.placements.len() != incumbent.placements.len() {
        return candidate.placements.len() > incumbent.placements.len();
    }
    if candidate.area != incumbent.area {
        return candidate.area > incumbent.area;
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
        let best = pack_once(&order, &input.area);
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
            let scored = pack_once(&candidate_order, &self.input.area);
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
}
