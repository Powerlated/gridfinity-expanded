//! Which objects share a bin, and how one bin's worth of objects is planned.
//!
//! A discrete bin is a whole number of cells, so an object costs a whole cell
//! however little of it it uses -- which is why several small objects are often
//! cheaper to keep in one bin than in one bin each. This file decides where that
//! is true. `GroupPlan` is one bin's worth of objects, `plan_group_bin` finds the
//! smallest bin holding all of them (a group of one is what `--mode bins`
//! builds), `Grouping` reads a whole candidate partition six ways with every term
//! priced in cells, `score` adds them into the number of cells the search
//! minimises, and `choose_groups` is the search: start from one bin per object,
//! merge the best pair while merging pays, then move and swap single objects
//! between the groups that survived. Everything is planned at
//! `PackEffort::Quick` while the search is choosing and re-planned at the run's
//! own effort once it has chosen, and the whole search is budgeted in candidate
//! partitions rather than in time, so one drawer and one object list always give
//! one grouping.

use crate::input::{Object, Spec};
use crate::optimize::{cell_rect, claim_input, settle_within};
use gridfinity_model::layout::{GridCell, GridFootprint, compartments};
use gridfinity_model::printers::compute_auto_split_lines;
use gridfinity_project::drawer::{DrawerGrid, cavity_region, packing_area, packing_inset};
use gridfinity_project::pack::{
    PackEffort, PackInput, PackObject, PackResult, Placement, pack_layout,
};
use gridfinity_project::rects::{Rect, inflate_parts, rects_overlap, union_area};

use std::collections::BTreeMap;

/// Every term of a grouping is priced in **cells**, and these are the prices.
///
/// Fractions of their own worst cases were tried first, the way `Tidiness`
/// states its six, and they do not work here: a cell recovered is a small share
/// of a big drawer while an object joining another object's bin is a large share
/// of a handful of objects, so the second term buried the first and nothing was
/// ever grouped on either worked example. Cells are the currency the question is
/// actually asked in -- "is this worth a cell?" -- so each concern says how many
/// cells it is worth and the score is a number of cells.

/// What a cell of air inside a bin costs. A quarter of a cell: air is not waste
/// the way a whole extra cell is, but a bin packed loosely is one about to be
/// able to give a cell back.
const W_AIR: f64 = 0.25;

/// What each cell of the biggest bin costs on top of its own cell. A large bin
/// is a large print, a long time to lose to a failed one, and an awkward thing
/// to lift out of a drawer -- so the biggest bin pays for its size twice.
const W_LARGEST: f64 = 0.15;

/// What a bin the bed cannot take whole costs. Two cells: the same objection as
/// `W_LARGEST` arrived at, and worth more than either object's cell, because a
/// cut bin is one the user has to print in pieces and align.
const W_CUT: f64 = 2.0;

/// What an object joining another object's bin costs, as a share of what it
/// joins: each extra object is priced at half the bin's own cells per object, so
/// four bags of washers sharing one cell cost a fraction of a cell between them
/// and a file set joining a caliper box's tray costs several.
///
/// A flat price per extra object was tried and is wrong in both directions at
/// once: at three quarters of a cell four small objects would not share the one
/// cell they all fit in, and at three cells five big tools were still merged
/// into a single 68-cell body -- which is the drawer-wide bin `--mode walls`
/// already builds. What a separate tray is worth scales with the tray, so the
/// price does.
const W_SHARED: f64 = 0.5;

/// What a cell of an oblong bin costs. The smallest price on purpose: it only
/// separates groupings that are otherwise equal, `candidate_sizes` having
/// already preferred the squarest bin of each size.
const W_OBLONG: f64 = 0.1;

/// How close two scores must be to count as the same. Two partitions holding the
/// same claims in the same cells reach their terms by different sums, so the
/// last bits disagree; a merge is taken only where it improves by more than
/// this, which is what stops the search chasing float noise into a bin nobody
/// asked for.
const SCORE_TIE: f64 = 1e-9;

/// How many candidate partitions the search may evaluate, by effort. An
/// iteration count and never a wall clock, for the reason `PackEffort::restarts`
/// is: a fit must not depend on the machine that ran it.
fn search_budget(effort: PackEffort) -> usize {
    match effort {
        PackEffort::Quick => 60,
        PackEffort::Standard => 300,
        PackEffort::Thorough => 1500,
    }
}

/// One bin's worth of objects: which objects share it, the cells it covers, and
/// where every instance of every one of them sits inside it.
///
/// `objects` is sorted and is the group's identity -- the cache key of the plan
/// and what the layout names the bin by. The cells and the placements are in the
/// bin's own millimetres with its own grid starting at the origin, exactly as
/// `--mode bins` produced them for a group of one.
#[derive(Clone)]
pub struct GroupPlan {
    pub objects: Vec<String>,
    pub cells: Vec<GridCell>,
    pub placements: Vec<Placement>,
    pub iterations: usize,
    /// What settling the bin's own layout took: free bands absorbed into the
    /// compartments facing them, walls grown into the leftover no band reached,
    /// and slabs whose slack was evened out.
    pub absorbed: usize,
    pub evened: usize,
    pub grown: usize,
    pub clamped: usize,
}

impl GroupPlan {
    /// The area the claims standing in this bin cover, in square millimetres.
    ///
    /// A sum rather than a union: the packer places claims without overlap
    /// (`pack_once` asserts it), so the only rectangles that can overlap are the
    /// several parts of one instance, which `union_area` resolves.
    fn claimed(&self) -> f64 {
        self.placements.iter().map(|p| union_area(&p.parts)).sum()
    }

    /// How many instances stand in this bin.
    fn instances(&self) -> usize {
        self.placements.len()
    }
}

/// The cell rectangles a bin may be, smallest first and squarest among equals,
/// bounded by the drawer's own grid: the order a bin's size is searched in, so
/// the first size that holds the group's instances is the smallest one that
/// does.
fn candidate_sizes(cols: u32, rows: u32) -> Vec<(u32, u32)> {
    let mut out: Vec<(u32, u32)> = Vec::new();
    for n in 1..=cols {
        for m in 1..=rows {
            out.push((n, m));
        }
    }
    out.sort_by_key(|&(n, m)| (n * m, i64::from(n).abs_diff(i64::from(m)), n));
    out
}

/// The cells of a `cols` x `rows` bin that its packed claims actually reach:
/// every cell whose square meets a claim grown by `inset`, or the whole
/// rectangle when what is left of it is not edge-connected.
///
/// Growing each claim by the perimeter inset is what makes dropping the rest
/// safe. A dropped cell's edge becomes the bin's own outline, and the perimeter
/// wall plus its clearance stands inside that edge, so a cell is dropped only
/// where no compartment comes within a wall of it -- which leaves every claim
/// inside the kept cells and every pocket inside their cavity. The connectivity
/// fallback is the model's own precondition: the alpha generator assumes an
/// edge-connected bin, so a trim that would sever one is not taken.
fn footprint_cells(
    placements: &[Placement],
    cols: u32,
    rows: u32,
    pitch: f64,
    inset: f64,
) -> Vec<GridCell> {
    let reach: Vec<Rect> = placements
        .iter()
        .flat_map(|p| inflate_parts(&p.parts, inset))
        .collect();
    let whole: Vec<GridCell> = (0..rows as i32)
        .flat_map(|y| (0..cols as i32).map(move |x| GridCell { x, y }))
        .collect();
    let kept: Vec<GridCell> = whole
        .iter()
        .copied()
        .filter(|c| {
            let square = cell_rect(*c, pitch);
            reach.iter().any(|r| rects_overlap(&square, r))
        })
        .collect();
    assert!(
        !kept.is_empty(),
        "a bin holding {} claim(s) is reached by none of them",
        placements.len()
    );
    if compartments(&kept, &Default::default()).len() == 1 {
        return kept;
    }
    whole
}

/// The smallest bin that holds every instance of every object of one group, and
/// where those instances sit inside it, settled.
///
/// The layout is settled against the bin's own cavity *after* the footprint is
/// trimmed, never before: `footprint_cells` drops the cells no claim reaches,
/// and settling first would let a grown claim reach into a cell that was about
/// to be dropped, costing an L-shaped object its L-shaped bin.
///
/// The size search runs at `PackEffort::Quick` and the chosen size is then packed
/// at `effort`, which cannot place fewer: `PackSearch::new` runs the same greedy
/// pass at every tier and `step` only ever keeps an improvement. That is asserted
/// rather than assumed. A group no size within the drawer holds is an error
/// naming it, because a bin missing compartments is not the bin that was asked
/// for -- for a group of one that is the object that fits nowhere, and for a
/// larger group it is the merge that cannot be made.
pub fn plan_group_bin(
    spec: &Spec,
    objects: &[&Object],
    grid: DrawerGrid,
    floor_fillet: f64,
    effort: PackEffort,
) -> Result<GroupPlan, String> {
    assert!(!objects.is_empty(), "a group is at least one object");
    let wanted: usize = objects.iter().map(|o| o.pack.quantity as usize).sum();
    let request = |cols: u32, rows: u32, effort: PackEffort| {
        let size = DrawerGrid {
            cols,
            rows,
            margin_x: 0.0,
            margin_y: 0.0,
        };
        let area = packing_area(size, spec.wall_thickness, spec.pitch);
        let packed: Vec<PackObject> = objects.iter().map(|o| o.pack.clone()).collect();
        claim_input(spec, area, floor_fillet, packed, effort)
    };
    let margin = request(1, 1, PackEffort::Quick).margin();
    let needed: f64 = objects
        .iter()
        .map(|o| union_area(&inflate_parts(&o.pack.parts, margin)) * f64::from(o.pack.quantity))
        .sum();

    let mut chosen: Option<(u32, u32)> = None;
    for (cols, rows) in candidate_sizes(grid.cols, grid.rows) {
        let input = request(cols, rows, PackEffort::Quick);
        if input.area.width <= 0.0 || input.area.depth <= 0.0 || input.area.area() < needed {
            continue;
        }
        if pack_layout(input).placements.len() == wanted {
            chosen = Some((cols, rows));
            break;
        }
    }
    let Some((cols, rows)) = chosen else {
        return Err(format!(
            "{} does not fit any bin the {} x {} cell drawer holds: the {wanted} instance(s) \
             claim {needed:.1} mm2 together, including clearance, floor fillet and half divider",
            names(objects),
            grid.cols,
            grid.rows
        ));
    };
    let result = pack_layout(request(cols, rows, effort));
    assert_eq!(
        result.placements.len(),
        wanted,
        "a {cols} x {rows} cell bin held every instance of {} at the quick effort, so a longer \
         search cannot place fewer",
        names(objects)
    );
    let mut ids: Vec<String> = objects.iter().map(|o| o.pack.id.clone()).collect();
    ids.sort();
    let cells = footprint_cells(
        &result.placements,
        cols,
        rows,
        spec.pitch,
        packing_inset(spec.wall_thickness),
    );
    let cavity = cavity_region(&cells, spec.pitch, packing_inset(spec.wall_thickness));
    let (settled, clamped) = settle_within(&result.placements, &cavity, spec, margin);
    Ok(GroupPlan {
        objects: ids,
        cells,
        placements: settled.placements,
        iterations: result.iterations,
        absorbed: settled.absorbed,
        evened: settled.evened,
        grown: settled.grown,
        clamped,
    })
}

/// A group named the way a refusal should name it: the objects' own names, in
/// the order they were given, joined for reading.
fn names(objects: &[&Object]) -> String {
    objects
        .iter()
        .map(|o| format!("{} x{}", o.pack.name, o.pack.quantity))
        .collect::<Vec<String>>()
        .join(" + ")
}

/// How a grouping reads, term by term, every term a number of **cells**.
///
/// One currency for all six, so the weights in `score` say what each concern is
/// worth in the units the question is asked in and a reader of the report can
/// check the arithmetic. `cells` is the cells themselves and needs no price.
///
/// Nothing here measures height. `height_units` is one setting for the whole run
/// and every compartment of every bin is the same depth, so two objects sharing
/// a bin cannot disagree about it.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Grouping {
    /// Cells the bins stand on.
    pub cells: f64,
    /// Cells' worth of area inside those bins that no claim covers.
    pub air: f64,
    /// Cells of the biggest bin.
    pub largest: f64,
    /// Bins the printer's bed cannot take whole.
    pub cut: f64,
    /// What sharing came to: each bin's cells per object, once for every object
    /// beyond the first standing in it.
    pub shared: f64,
    /// Cells weighted by how far from square the bin they stand in is.
    pub oblong: f64,
}

/// What this grouping costs, in cells: the cells it stands on plus what each of
/// the other five concerns is priced at. Lower is better, and a difference of
/// one is one cell of the drawer.
pub fn score(g: &Grouping) -> f64 {
    let score = g.cells
        + W_AIR * g.air
        + W_LARGEST * g.largest
        + W_CUT * g.cut
        + W_SHARED * g.shared
        + W_OBLONG * g.oblong;
    assert!(
        score.is_finite() && score > 0.0,
        "a grouping costs a positive number of cells, but {g:?} came to {score}"
    );
    score
}

/// A term of a grouping, asserted to be a count of cells and not a mistake.
fn cells_of(value: f64, what: &str) -> f64 {
    assert!(
        value.is_finite() && value >= 0.0,
        "{what} came to {value} cells, which is not a quantity of drawer"
    );
    value
}

/// How a whole candidate partition reads: its six terms, measured over the bins
/// it plans and the drawer they stand in.
///
/// Nothing is capped against the drawer: a partition wanting more cells than the
/// drawer has is measured here too, and is ordered behind every partition that
/// fits by `better`'s first key rather than by its score.
pub fn measure(spec: &Spec, plans: &[GroupPlan]) -> Grouping {
    assert!(!plans.is_empty(), "a grouping is at least one bin");
    let cell_area = spec.pitch * spec.pitch;
    let used: f64 = plans.iter().map(|p| p.cells.len() as f64).sum();
    let claimed: f64 = plans.iter().map(GroupPlan::claimed).sum();
    let biggest = plans.iter().map(|p| p.cells.len()).max().unwrap_or(0) as f64;
    let shared: f64 = plans
        .iter()
        .map(|p| (p.objects.len() - 1) as f64 * p.cells.len() as f64 / p.objects.len() as f64)
        .sum();
    let cut = plans
        .iter()
        .filter(|p| !compute_auto_split_lines(&p.cells, spec.printer, spec.pitch).is_empty())
        .count() as f64;
    let oblong: f64 = plans
        .iter()
        .map(|p| {
            let f = GridFootprint::from_cells(&p.cells).expect("a planned bin has cells");
            let (w, d) = (f64::from(f.width_cells), f64::from(f.depth_cells));
            p.cells.len() as f64 * ((w - d).abs() / (w + d))
        })
        .sum();

    Grouping {
        cells: cells_of(used, "the cells the bins stand on"),
        air: cells_of(
            (used - claimed / cell_area).max(0.0),
            "the air inside the bins",
        ),
        largest: cells_of(biggest, "the biggest bin"),
        cut: cells_of(cut, "the bins the bed cannot take"),
        shared: cells_of(shared, "the objects sharing a bin"),
        oblong: cells_of(oblong, "how oblong the bins are"),
    }
}

/// One candidate partition as the search compares it: the bins it plans, how
/// many instances the drawer had no room for, and what the grouping scores.
struct Candidate {
    partition: Vec<Vec<String>>,
    unplaced: usize,
    grouping: Grouping,
    score: f64,
}

/// Whether `candidate` is a better grouping than `incumbent`.
///
/// Instances the drawer could not hold come first and absolutely, exactly as
/// placements do in the packer's own `better`: a tidier grouping may never cost
/// a placed object, and a partition that *fits* a drawer beats every partition
/// that does not -- which is how grouping rescues a drawer one bin per object
/// cannot be laid out in. The weighted score decides everything after that.
fn better(candidate: &Candidate, incumbent: &Candidate) -> bool {
    if candidate.unplaced != incumbent.unplaced {
        return candidate.unplaced < incumbent.unplaced;
    }
    incumbent.score - candidate.score > SCORE_TIE
}

/// Every group plan this search has made, by the objects sharing the bin.
///
/// The agglomeration re-forms the same groups constantly -- every round asks
/// again about every pair that did not win the last one -- so a group is planned
/// once and read many times. A group that cannot be built is remembered as its
/// refusal for the same reason: not being able to make a bin is as reusable an
/// answer as making one.
struct Plans<'a> {
    spec: &'a Spec,
    grid: DrawerGrid,
    floor_fillet: f64,
    by_id: BTreeMap<&'a str, &'a Object>,
    made: BTreeMap<Vec<String>, Result<GroupPlan, String>>,
}

impl<'a> Plans<'a> {
    fn new(spec: &'a Spec, grid: DrawerGrid, floor_fillet: f64) -> Plans<'a> {
        Plans {
            spec,
            grid,
            floor_fillet,
            by_id: spec
                .objects
                .iter()
                .map(|o| (o.pack.id.as_str(), o))
                .collect(),
            made: BTreeMap::new(),
        }
    }

    /// This group's plan, made if it has not been asked for before, or the
    /// refusal saying why the group cannot be a bin.
    fn plan(&mut self, key: &[String]) -> Result<&GroupPlan, String> {
        assert!(
            key.windows(2).all(|w| w[0] < w[1]),
            "a group is keyed by its object ids sorted and distinct, not by {key:?}"
        );
        if !self.made.contains_key(key) {
            let objects: Vec<&Object> = key
                .iter()
                .map(|id| {
                    *self
                        .by_id
                        .get(id.as_str())
                        .unwrap_or_else(|| panic!("{id} is not one of the run's objects"))
                })
                .collect();
            let plan = plan_group_bin(
                self.spec,
                &objects,
                self.grid,
                self.floor_fillet,
                PackEffort::Quick,
            );
            self.made.insert(key.to_vec(), plan);
        }
        match &self.made[key] {
            Ok(plan) => Ok(plan),
            Err(why) => Err(why.clone()),
        }
    }
}

/// The bins of a partition packed into the drawer, as the outer pack of
/// `--mode bins` does it: the cell footprints as pitch-sized squares in the
/// drawer's own rectangle, every margin zero because a Gridfinity bin already
/// stands `HALF_TOL` inside its cells.
pub fn outer_pack(
    spec: &Spec,
    grid: DrawerGrid,
    plans: &[GroupPlan],
    effort: PackEffort,
) -> PackResult {
    pack_layout(PackInput {
        area: Rect::new(
            0.0,
            0.0,
            f64::from(grid.cols) * spec.pitch,
            f64::from(grid.rows) * spec.pitch,
        ),
        objects: plans
            .iter()
            .map(|plan| PackObject {
                id: plan.objects.join("+"),
                name: plan.objects.join(" + "),
                parts: plan.cells.iter().map(|c| cell_rect(*c, spec.pitch)).collect(),
                quantity: 1,
            })
            .collect(),
        divider_thickness: 0.0,
        clearance: 0.0,
        floor_fillet: 0.0,
        effort,
    })
}

/// How many instances stand in bins the drawer had no room for.
fn unplaced_instances(plans: &[GroupPlan], outer: &PackResult) -> usize {
    plans
        .iter()
        .filter(|plan| {
            let id = plan.objects.join("+");
            !outer.placements.iter().any(|p| p.object_id == id)
        })
        .map(GroupPlan::instances)
        .sum()
}

/// What this partition comes to, or `None` when one of its groups cannot be a
/// bin at all -- a merge the drawer's own grid has no room for, which is not a
/// candidate rather than a failure.
fn evaluate(partition: &[Vec<String>], plans: &mut Plans) -> Option<Candidate> {
    let mut made: Vec<GroupPlan> = Vec::new();
    for key in partition {
        made.push(plans.plan(key).ok()?.clone());
    }
    let outer = outer_pack(plans.spec, plans.grid, &made, PackEffort::Quick);
    let grouping = measure(plans.spec, &made);
    Some(Candidate {
        partition: partition.to_vec(),
        unplaced: unplaced_instances(&made, &outer),
        grouping,
        score: score(&grouping),
    })
}

/// One partition with two of its groups merged, both groups' ids in one sorted
/// key and the partition itself in its canonical order, so the same merge of the
/// same partition is always the same list.
fn merged(partition: &[Vec<String>], a: usize, b: usize) -> Vec<Vec<String>> {
    let mut key: Vec<String> = partition[a].iter().chain(&partition[b]).cloned().collect();
    key.sort();
    let mut out: Vec<Vec<String>> = partition
        .iter()
        .enumerate()
        .filter(|(i, _)| *i != a && *i != b)
        .map(|(_, g)| g.clone())
        .collect();
    out.push(key);
    canonical(out)
}

/// A partition in its canonical order: each group's ids sorted, the groups
/// ordered by their first id. Two partitions of the same objects into the same
/// groups are then the same value, which is what lets the cache and the search
/// compare them at all.
fn canonical(mut partition: Vec<Vec<String>>) -> Vec<Vec<String>> {
    for group in &mut partition {
        group.sort();
        assert!(
            group.windows(2).all(|w| w[0] < w[1]),
            "an object appears twice in the group {group:?}"
        );
    }
    partition.retain(|g| !g.is_empty());
    partition.sort();
    partition
}

/// The fewest cells a bin holding `claimed` square millimetres of claims can
/// possibly be: a cell holds less than `pitch^2` of claim, the perimeter wall
/// standing inside it, so this is a bound and never an estimate.
fn cell_floor(claimed: f64, pitch: f64) -> usize {
    assert!(
        claimed >= 0.0 && pitch > 0.0,
        "{claimed} mm2 of claims on a {pitch} mm grid is not a bin"
    );
    (claimed / (pitch * pitch)).ceil() as usize
}

/// The grouping the search settled on: one plan per bin, re-planned at the run's
/// own effort, and how that grouping reads.
pub struct Groups {
    pub plans: Vec<GroupPlan>,
    pub grouping: Grouping,
}

/// Which objects share a bin, decided by search, and their bins planned.
///
/// Starts from one bin per object -- what `--mode bins` builds -- and can only
/// return something `better` than that, which is asserted before it returns and
/// is the whole justification for `--mode auto` preferring this to a bin per
/// object. An object that fits no bin the drawer holds is an error naming it,
/// the same refusal a bin per object gives, because no grouping can rescue an
/// object that fits nowhere on its own either.
///
/// Two phases over one budget. **Merging**: every pair of groups is asked, in
/// settled order, and the best improving merge is applied, until none improves.
/// **Moving**: on what merging left, every single object is offered to every
/// other group and to a bin of its own, and every pair of objects in different
/// groups is offered a swap, again taking the best improving move until none
/// improves -- which is what gets past the point where no *pair* of whole groups
/// can be merged but one object is in the wrong one.
pub fn choose_groups(
    spec: &Spec,
    grid: DrawerGrid,
    floor_fillet: f64,
) -> Result<Groups, String> {
    let mut plans = Plans::new(spec, grid, floor_fillet);
    let singletons = canonical(
        spec.objects
            .iter()
            .map(|o| vec![o.pack.id.clone()])
            .collect(),
    );
    for group in &singletons {
        plans.plan(group)?;
    }
    let start = evaluate(&singletons, &mut plans).expect("every object's own bin was planned");
    let mut best = Candidate {
        partition: start.partition.clone(),
        unplaced: start.unplaced,
        grouping: start.grouping,
        score: start.score,
    };
    let mut budget = search_budget(spec.effort);

    while budget > 0 {
        let Some(winner) = best_neighbour(&merges(&best.partition, &mut plans), &best, &mut plans, &mut budget)
        else {
            break;
        };
        best = winner;
    }
    while budget > 0 {
        let Some(winner) = best_neighbour(&moves(&best.partition), &best, &mut plans, &mut budget)
        else {
            break;
        };
        best = winner;
    }

    assert!(
        !better(&start, &best),
        "the search returned a grouping worse than the one bin per object it started from: \
         {:?} against {:?}",
        best.grouping,
        start.grouping
    );

    let mut chosen: Vec<GroupPlan> = Vec::new();
    for key in &best.partition {
        let objects: Vec<&Object> = key
            .iter()
            .map(|id| {
                *plans
                    .by_id
                    .get(id.as_str())
                    .unwrap_or_else(|| panic!("{id} is not one of the run's objects"))
            })
            .collect();
        chosen.push(plan_group_bin(spec, &objects, grid, floor_fillet, spec.effort)?);
    }
    assert_eq!(
        chosen.iter().map(GroupPlan::instances).sum::<usize>(),
        spec.objects
            .iter()
            .map(|o| o.pack.quantity as usize)
            .sum::<usize>(),
        "every instance of every object stands in exactly one of the chosen bins"
    );
    let grouping = measure(spec, &chosen);
    Ok(Groups {
        plans: chosen,
        grouping,
    })
}

/// The best candidate among `neighbours` that improves on `best`, or `None` when
/// none does or the budget runs out. Each evaluation costs one of the budget,
/// whether or not it improves, because the budget is what bounds the search and
/// not what bounds its success.
fn best_neighbour(
    neighbours: &[Vec<Vec<String>>],
    best: &Candidate,
    plans: &mut Plans,
    budget: &mut usize,
) -> Option<Candidate> {
    let mut winner: Option<Candidate> = None;
    for partition in neighbours {
        if *budget == 0 {
            break;
        }
        *budget -= 1;
        let Some(candidate) = evaluate(partition, plans) else {
            continue;
        };
        if !better(&candidate, best) {
            continue;
        }
        if winner.as_ref().is_none_or(|w| better(&candidate, w)) {
            winner = Some(candidate);
        }
    }
    winner
}

/// Every partition one merge away from this one, in settled order, minus the
/// merges that provably cannot recover a cell.
///
/// That prune is a pruning rule and no more: of the six terms, five are worse or
/// unchanged under a merge that recovers no cell and only `shape` can improve,
/// so what it can miss is a merge that would have won on squareness alone --
/// `W_SHAPE` being the smallest weight, deliberately. What it buys is a packing
/// search not run, which is the expensive half of a candidate.
fn merges(partition: &[Vec<String>], plans: &mut Plans) -> Vec<Vec<Vec<String>>> {
    let pitch = plans.spec.pitch;
    let mut out: Vec<Vec<Vec<String>>> = Vec::new();
    for a in 0..partition.len() {
        let (cells, claimed) = match plans.plan(&partition[a]) {
            Ok(plan) => (plan.cells.len(), plan.claimed()),
            Err(_) => continue,
        };
        for b in (a + 1)..partition.len() {
            let Ok(other) = plans.plan(&partition[b]) else {
                continue;
            };
            if cell_floor(claimed + other.claimed(), pitch) >= cells + other.cells.len() {
                continue;
            }
            out.push(merged(partition, a, b));
        }
    }
    out
}

/// Every partition one object's move or one pair's swap away from this one, in
/// settled order: each object into each other group, each object into a bin of
/// its own, and each pair of objects in different groups exchanged.
fn moves(partition: &[Vec<String>]) -> Vec<Vec<Vec<String>>> {
    let mut out: Vec<Vec<Vec<String>>> = Vec::new();
    for (from, group) in partition.iter().enumerate() {
        for id in group {
            for (to, _) in partition.iter().enumerate() {
                if to == from {
                    continue;
                }
                let mut next: Vec<Vec<String>> = partition.to_vec();
                next[from].retain(|other| other != id);
                next[to].push(id.clone());
                out.push(canonical(next));
            }
            if group.len() > 1 {
                let mut next: Vec<Vec<String>> = partition.to_vec();
                next[from].retain(|other| other != id);
                next.push(vec![id.clone()]);
                out.push(canonical(next));
            }
        }
    }
    for a in 0..partition.len() {
        for b in (a + 1)..partition.len() {
            for one in &partition[a] {
                for two in &partition[b] {
                    let mut next: Vec<Vec<String>> = partition.to_vec();
                    next[a].retain(|other| other != one);
                    next[b].retain(|other| other != two);
                    next[a].push(two.clone());
                    next[b].push(one.clone());
                    out.push(canonical(next));
                }
            }
        }
    }
    out.dedup();
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::input;
    use gridfinity_model::gridfinity::GRID_PITCH;
    use gridfinity_project::drawer::{MAX_GRID, drawer_grid};

    /// Four objects small enough that all four claims fit inside one cell's
    /// packing area: 12 mm blocks claim 19.16 mm square at these settings, and
    /// two of them across is 38.32 mm of a cell's 39.1. One bin per object costs
    /// four cells for what one holds.
    const SMALL_FOUR: &str = "[drawer]
width = 84
depth = 84

[settings]
effort = \"quick\"

[[objects]]
name = \"washers\"
size = [12, 12]

[[objects]]
name = \"nuts\"
size = [12, 12]

[[objects]]
name = \"grub screws\"
size = [12, 12]

[[objects]]
name = \"o-rings\"
size = [12, 12]
";

    /// Two objects that each fill a bin of their own: two 30 mm blocks need a
    /// 1 x 2 cell bin and a 20 x 60 mm rod needs another, and putting them
    /// together needs 2 x 2 -- the same four cells, for a bin that now has to be
    /// emptied to reach either.
    const TWO_APART: &str = "[drawer]
width = 200
depth = 200

[settings]
effort = \"quick\"

[[objects]]
name = \"block\"
quantity = 2
size = [30, 30]

[[objects]]
name = \"rod\"
size = [20, 60]
";

    /// The groups a file's objects are put into, by name, each group's names
    /// sorted and the groups in settled order.
    fn grouped(text: &str) -> Vec<Vec<String>> {
        let spec = input::parse(text).expect("the fixture is a valid run");
        let grid = drawer_grid(spec.drawer_width, spec.drawer_depth, MAX_GRID, GRID_PITCH);
        let groups = choose_groups(&spec, grid, spec.built_floor_fillet())
            .expect("the fixture's objects each fit a bin");
        groups.plans.iter().map(|p| p.objects.clone()).collect()
    }

    /// The whole point of grouping: four objects that share one cell are put in
    /// one bin, where one bin each costs four.
    #[test]
    fn puts_objects_that_share_a_cell_in_one_bin() {
        assert_eq!(
            grouped(SMALL_FOUR),
            vec![vec![
                "grub screws".to_string(),
                "nuts".to_string(),
                "o-rings".to_string(),
                "washers".to_string(),
            ]],
            "four claims of 19.16 mm square stand in one cell's 39.1 mm packing area"
        );
    }

    /// And the other half of it: a merge that recovers no cell is declined, so
    /// the search is shown to decide rather than to always group.
    #[test]
    fn leaves_objects_apart_where_sharing_recovers_nothing() {
        assert_eq!(
            grouped(TWO_APART),
            vec![vec!["block".to_string()], vec!["rod".to_string()]],
            "two cells each apart against four cells together is no saving and one more \
             thing to empty"
        );
    }

    /// The grouping the search returns is never worse than the one bin per
    /// object it starts from -- the property `--mode auto` leans on when it
    /// prefers a hybrid fit to a bin per object.
    #[test]
    fn never_returns_a_grouping_worse_than_one_bin_per_object() {
        for text in [SMALL_FOUR, TWO_APART] {
            let spec = input::parse(text).expect("the fixture is a valid run");
            let grid = drawer_grid(spec.drawer_width, spec.drawer_depth, MAX_GRID, GRID_PITCH);
            let floor_fillet = spec.built_floor_fillet();
            let chosen = choose_groups(&spec, grid, floor_fillet).expect("the fixture fits");

            let mut singles: Vec<GroupPlan> = Vec::new();
            for object in &spec.objects {
                singles.push(
                    plan_group_bin(&spec, &[object], grid, floor_fillet, spec.effort)
                        .expect("every object fits a bin of its own"),
                );
            }
            let apart = measure(&spec, &singles);
            assert!(
                score(&chosen.grouping) <= score(&apart) + SCORE_TIE,
                "grouping scored {} where one bin per object scores {}",
                score(&chosen.grouping),
                score(&apart)
            );
            assert!(
                chosen.plans.iter().map(|p| p.cells.len()).sum::<usize>()
                    <= singles.iter().map(|p| p.cells.len()).sum::<usize>(),
                "a grouping may not cost the drawer a cell"
            );
        }
    }

    /// Every term is a count of cells, so the weights price each concern in the
    /// currency the question is asked in and the score can be checked by hand.
    #[test]
    fn every_term_is_a_number_of_cells_and_the_score_is_their_weighted_sum() {
        let spec = input::parse(SMALL_FOUR).expect("the fixture is a valid run");
        let grid = drawer_grid(spec.drawer_width, spec.drawer_depth, MAX_GRID, GRID_PITCH);
        let plan = plan_group_bin(
            &spec,
            &spec.objects.iter().collect::<Vec<&Object>>(),
            grid,
            spec.built_floor_fillet(),
            PackEffort::Quick,
        )
        .expect("all four fit one bin");
        let g = measure(&spec, std::slice::from_ref(&plan));

        assert_eq!(g.cells, 1.0, "all four objects stand on one cell");
        assert_eq!(g.largest, 1.0, "which is also the biggest bin there is");
        assert_eq!(g.cut, 0.0, "an 84 mm bin fits every profile's bed");
        assert_eq!(
            g.shared, 0.75,
            "three objects joined the first one, each priced at the bin's one cell over four"
        );
        assert_eq!(g.oblong, 0.0, "one cell is square");
        assert!(
            g.air > 0.0 && g.air < 1.0,
            "four 19.16 mm claims leave part of a cell empty, not all or none of it: {}",
            g.air
        );
        assert!(
            (score(&g)
                - (g.cells
                    + W_AIR * g.air
                    + W_LARGEST * g.largest
                    + W_CUT * g.cut
                    + W_SHARED * g.shared
                    + W_OBLONG * g.oblong))
                .abs()
                < 1e-12
        );
    }

    /// A grouping that leaves an instance out of the drawer loses to one that
    /// does not, however much prettier its score is. The packer's own `better`
    /// puts placements first for the same reason.
    #[test]
    fn prefers_the_grouping_that_holds_more_however_it_scores() {
        let fits = Candidate {
            partition: vec![vec!["a".to_string()]],
            unplaced: 0,
            grouping: Grouping::default(),
            score: 9.0,
        };
        let spills = Candidate {
            partition: vec![vec!["a".to_string()]],
            unplaced: 1,
            grouping: Grouping::default(),
            score: 0.0,
        };
        assert!(better(&fits, &spills));
        assert!(!better(&spills, &fits));
    }
}
