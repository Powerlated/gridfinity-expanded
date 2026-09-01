//! The cavity, authored one compartment at a time.
//!
//! A compartment is a connected group of the piece's cells, and its cavity is
//! the walk over that group's own boundary edges: `cavity_inset` gives each edge
//! its own standoff -- a wall's thickness plus the bin's tolerance where a wall
//! stands, half a divider where one divides, nothing at all at an opening --
//! and `compartment_corners` resolves each corner from the pair of insets
//! meeting there, emitting *two* corners where the inset changes along a
//! straight run, because the two lines are then parallel and never meet.
//!
//! Two things the walk cannot see are subtracted afterwards. A divider whose two
//! cells stay in one compartment lies on no boundary edge, so `finger_strips`
//! collects those and `compartment_cavity_corners` takes them out with one
//! region difference -- which can split a compartment in two or open a hole in
//! it, so `walked_cavity` re-pairs outers and holes by containment rather than
//! by which compartment produced them. `shape_cavity_loop` then rounds the
//! rectilinear result, convex corners by the cavity radius and reentrant ones by
//! the floor fillet, so the blend that follows stays tangent continuous.
//!
//! `pocket_cavity` is the other way in: a caller that already knows its
//! compartments states them as rectangles and the walk is not run at all, which
//! leaves the bin solid everywhere a pocket is not.

use super::*;
use gridfinity_brep::math::Vec2;
use gridfinity_brep::rectregion::{LoopStyle, RectF, TracedLoop, shape_loop, trace_rects};
use gridfinity_brep::region2d::region_difference;
use gridfinity_brep::round::{corners_of, loop_of_points};
use gridfinity_brep::sketch::{COINCIDENT, Seg, loop_area, reverse_loop};
use crate::layout::{EffectiveWalls, GridCell, GridEdge, Orientation};
use std::collections::HashSet;

/// How far one compartment's cavity stands in from a boundary edge's pitch line.
///
/// Three cases and no fourth: a wall's own thickness plus the bin's tolerance
/// where a wall stands, half a divider's thickness where the compartment is
/// divided from its neighbour, and nothing at all at an opening, where the
/// cavity runs out to the pitch line and is pulled back by the outline instead.
///
/// A compartment boundary edge is a piece perimeter edge or a divider and can be
/// nothing else -- two cells the piece holds either side of an undivided edge
/// are the same compartment by construction -- so the three cases are a
/// partition, which is what the `else` asserts rather than assumes.
pub(super) fn cavity_inset(walls: &EffectiveWalls, wt: f64, e: &GridEdge) -> f64 {
    if walls.dividers.contains(e) {
        wt / 2.0
    } else if walls.walled.contains(e) {
        HALF_TOL + wt
    } else {
        assert!(
            walls.open.contains(e),
            "compartment boundary edge {e:?} is neither divider, wall nor opening"
        );
        0.0
    }
}

/// One compartment's cavity boundary, as the rectilinear loop of corners its own
/// edges put it through.
///
/// Each edge contributes a line at its own inset, and each corner is where the
/// two inset lines either side of it meet -- `c + nrm·ins + n1·ins_next`, the
/// `q` of the corner construction, which for two axis-aligned edges is just one
/// coordinate from each. That is the whole of it where the boundary turns.
///
/// Where the boundary *does not* turn there is still a corner to emit whenever
/// the inset changes: a wall meeting an opening, or a wall meeting a divider,
/// along one straight run. The two inset lines are then parallel and never meet,
/// so the boundary steps across at the lattice point and the run contributes
/// **two** corners rather than one. A collinear pair at one inset contributes
/// none, which is the collinear merge `trace_rects` does with `merge_collinear`.
///
/// This is `plan_cavity` + `trace_rects` said directly, and over 1588 swept
/// compartments it is the same answer corner for corner. The two disagree only
/// where a divider meets one of the tracer's rectangle-shaped fixups -- a
/// junction between two dividers, or the reentrant corner patch -- and neither
/// case is one the walk has to reproduce; see
/// `the_walk_reproduces_the_tracer_except_where_a_divider_meets_a_notch` for the
/// enumeration and `CLAUDE.md` for what each is.
pub(super) fn compartment_corners(
    steps: &[Step],
    pitch: f64,
    inset: &dyn Fn(&GridEdge) -> f64,
) -> Vec<Vec2> {
    let n = steps.len();
    assert!(
        n >= 4,
        "a compartment's boundary loop turns at least four times, got {n} step(s)"
    );
    let mut pts: Vec<Vec2> = Vec::with_capacity(n);
    for k in 0..n {
        let s = &steps[k];
        let s_next = &steps[(k + 1) % n];
        let d = dirv(s.dir());
        let d1 = dirv(s_next.dir());
        let nrm = left_of(s.dir());
        let n1 = left_of(s_next.dir());
        let ins = inset(&s.edge);
        let ins_next = inset(&s_next.edge);
        let c = mm(s.to, pitch);
        // A cell region's boundary turns by a right angle or not at all; a
        // reversal would need a spike of zero width, which no set of cells has.
        assert!(
            d.dot(d1) > -TURN_SIGN,
            "the boundary reverses at {c:?}: {d:?} then {d1:?}"
        );
        let cross = d.x * d1.y - d.y * d1.x;
        if cross.abs() > TURN_SIGN {
            pts.push(c + nrm * ins + n1 * ins_next);
        } else if (ins - ins_next).abs() > COINCIDENT {
            pts.push(c + nrm * ins);
            pts.push(c + n1 * ins_next);
        }
    }
    assert!(
        pts.len() >= 4,
        "a compartment's cavity loop has at least four corners, got {}",
        pts.len()
    );
    pts
}

/// The divider strips that stand *inside* one compartment: the dividers whose
/// two cells the compartment keeps, so they separate nothing.
///
/// The walk cannot see these -- a boundary walk over cells only ever visits
/// edges between a compartment and something else — and they are real geometry:
/// a stub of wall reaching in from the perimeter, or, when neither end of the
/// divider touches a wall, a free-standing island in the middle of the cavity.
/// `partial_divider_finger_is_watertight` is one of them.
///
/// The strip is `plan_cavity`'s, verbatim: the full pitch length of the edge by
/// `wall_thickness`, centred on the divider line. Taking it out of the walked
/// loop reproduces that planner's answer exactly, which is what
/// `the_walk_reproduces_the_tracer_except_where_a_divider_meets_a_notch` checks
/// on every finger configuration it sweeps.
pub(super) fn finger_strips(
    comp: &[GridCell],
    pitch: f64,
    walls: &EffectiveWalls,
    wt: f64,
) -> Vec<RectF> {
    let set: HashSet<GridCell> = comp.iter().copied().collect();
    let mut out = Vec::new();
    for e in &walls.dividers {
        let [a, b] = match e.orientation {
            Orientation::V => [GridCell { x: e.x - 1, y: e.y }, GridCell { x: e.x, y: e.y }],
            Orientation::H => [GridCell { x: e.x, y: e.y - 1 }, GridCell { x: e.x, y: e.y }],
        };
        if !set.contains(&a) || !set.contains(&b) {
            continue;
        }
        out.push(divider_strip(e, pitch, wt));
    }
    out
}

/// The material a divider stands as: `plan_cavity`'s own rectangle, the full
/// pitch length of the edge by `wall_thickness`, centred on the divider line.
pub(super) fn divider_strip(e: &GridEdge, pitch: f64, wt: f64) -> RectF {
    let p = pitch;
    match e.orientation {
        Orientation::H => RectF::new(e.x as f64 * p, e.y as f64 * p - wt / 2.0, p, wt),
        Orientation::V => RectF::new(e.x as f64 * p - wt / 2.0, e.y as f64 * p, wt, p),
    }
}

/// One compartment's cavity boundary, rectilinear and complete: the walk over
/// its own edges, less the divider strips standing inside it.
///
/// The two halves are deliberately different machinery. The **perimeter** is
/// authored, because that is where the insets differ edge to edge and where the
/// corners have to land on the bin's outline rather than on a lattice point --
/// the thing a rectilinear tracer cannot do. A **finger** is a plain rectangle
/// in the middle of a cavity, nowhere near an arc, so it comes out with one
/// `region_difference` and no case analysis at all.
///
/// Outer loops come back wound positive and holes negative, as the walk emits
/// them; a finger that reaches no wall becomes a new hole, and one that cuts the
/// loop in two returns two outers.
pub(super) fn compartment_cavity_corners(
    comp: &[GridCell],
    pitch: f64,
    walls: &EffectiveWalls,
    wt: f64,
) -> Vec<Vec<Vec2>> {
    let inset = |e: &GridEdge| cavity_inset(walls, wt, e);
    let walked: Vec<Vec<Vec2>> = boundary_steps(comp)
        .iter()
        .map(|steps| compartment_corners(steps, pitch, &inset))
        .collect();
    let fingers = finger_strips(comp, pitch, walls, wt);
    if fingers.is_empty() {
        return walked;
    }
    // Two dividers running end to end give two strips sharing a boundary run,
    // and a region whose own loops touch is not an operand a sweep can classify
    // -- it left a 0.001 mm^2 sliver on 36 of the swept compartments. Unioning
    // them first is what the rectilinear engine is for: they are axis-aligned
    // rectangles nowhere near an arc, which is the whole of what it handles.
    let joined: Vec<Vec<Seg>> = trace_rects(&fingers, &[])
        .iter()
        .map(|l| loop_of_points(&l.pts))
        .collect();
    let as_segs: Vec<Vec<Seg>> = walked.iter().map(|pts| loop_of_points(pts)).collect();
    let cut = region_difference(&as_segs, &joined);
    let out: Vec<Vec<Vec2>> = cut.iter().map(|l| corners_of(l)).collect();
    assert!(
        out.iter().all(|l| l.len() >= 4),
        "taking {} finger(s) out of a compartment left a loop of under four corners",
        fingers.len()
    );
    out
}

/// Whichever of the two cells `e` divides `set` holds, or `None` when it holds
/// neither -- the edge then lies wholly outside the piece and bounds nothing of
/// it. A `V` edge divides the cells left and right of `(e.x, e.y)`, an `H` edge
/// the cells below and above it; when `set` holds both, the lower-indexed one is
/// returned and either answer names the same edge.
pub(super) fn edge_inside_cell(set: &HashSet<GridCell>, e: &GridEdge) -> Option<GridCell> {
    let (a, b) = match e.orientation {
        Orientation::V => (GridCell { x: e.x - 1, y: e.y }, GridCell { x: e.x, y: e.y }),
        Orientation::H => (GridCell { x: e.x, y: e.y - 1 }, GridCell { x: e.x, y: e.y }),
    };
    if set.contains(&a) {
        Some(a)
    } else if set.contains(&b) {
        Some(b)
    } else {
        None
    }
}

pub(super) fn contained_holes(all: &[TracedLoop], outer: &TracedLoop) -> Vec<TracedLoop> {
    all.iter()
        .filter(|l| l.is_hole() && outer.contains(l.pts[0]))
        .cloned()
        .collect()
}

/// Every compartment's authored cavity, each outer loop with the holes it holds.
///
/// A compartment contributes one outer loop and one hole per enclosed hole of
/// its own cell set, plus whatever its finger strips carve: a finger reaching no
/// wall becomes a hole, and one that cuts the compartment across becomes a
/// second outer. Holes are matched to outers by containment rather than by
/// which compartment produced them, so a split compartment gives each half only
/// the holes inside it.
pub(super) fn walked_cavity(
    cells: &[GridCell],
    pitch: f64,
    walls: &EffectiveWalls,
    wt: f64,
) -> Vec<(TracedLoop, Vec<TracedLoop>)> {
    let loops: Vec<TracedLoop> = crate::layout::compartments(cells, &walls.dividers)
        .iter()
        .flat_map(|comp| compartment_cavity_corners(comp, pitch, walls, wt))
        .map(|pts| TracedLoop { pts })
        .collect();
    let outers: Vec<&TracedLoop> = loops.iter().filter(|l| !l.is_hole()).collect();
    assert!(
        !outers.is_empty(),
        "a piece of {} cell(s) authored no cavity at all",
        cells.len()
    );
    outers
        .iter()
        .map(|ol| ((*ol).clone(), contained_holes(&loops, ol)))
        .collect()
}

/// The bin's compartments as the caller stated them: the rectangles unioned into
/// rectilinear loops, each outer paired with the holes it holds.
///
/// This replaces `walked_cavity` rather than adding to it. The walk derives the
/// cavity from the cells and the walls between them, so every square millimetre
/// inside the perimeter is either compartment or divider; stating the pockets
/// instead makes the bin **solid everywhere a pocket is not**, which is the
/// whole difference and the reason a fitted drawer wants it -- the space no
/// object was packed into is material rather than a pocket of air nothing can
/// reach. Downstream nothing changes: these loops are rounded, blended, stacked
/// and capped exactly as walked ones are.
///
/// **Overlapping pockets merge, on purpose.** One compartment is however many
/// rectangles the caller needed to describe it, so an L-shaped object gets its
/// L-shaped pocket by stating two overlapping boxes; the union is what
/// `trace_rects` returns. Two compartments that must stay apart are the
/// caller's to keep apart -- the packer does it by construction, its claims
/// being disjoint and each pocket inset inside its own claim.
pub(super) fn pocket_cavity(pockets: &[Pocket]) -> Vec<(TracedLoop, Vec<TracedLoop>)> {
    let mut rects = Vec::with_capacity(pockets.len());
    for (i, k) in pockets.iter().enumerate() {
        assert!(
            k.width > 0.0 && k.depth > 0.0,
            "pocket {i} is {} x {} mm, which encloses no compartment",
            k.width,
            k.depth
        );
        rects.push(RectF::new(k.x, k.y, k.width, k.depth));
    }
    let loops = trace_rects(&rects, &[]);
    let outers: Vec<&TracedLoop> = loops.iter().filter(|l| !l.is_hole()).collect();
    assert!(
        !outers.is_empty(),
        "{} pocket(s) traced {} loop(s) and not one of them bounds a compartment",
        pockets.len(),
        loops.len()
    );
    assert!(
        outers.len() <= pockets.len(),
        "{} pocket(s) cannot bound {} compartments; a union has no more parts than operands",
        pockets.len(),
        outers.len()
    );
    for (i, k) in pockets.iter().enumerate() {
        let centre = Vec2::new(k.x + k.width / 2.0, k.y + k.depth / 2.0);
        assert!(
            outers.iter().any(|ol| ol.contains(centre)),
            "pocket {i} at {centre:?} lies in none of the {} compartment(s) its own union traced",
            outers.len()
        );
    }
    outers
        .iter()
        .map(|ol| ((*ol).clone(), contained_holes(&loops, ol)))
        .collect()
}

pub(super) fn shape_cavity_loop_open(
    lp: &TracedLoop,
    rc: f64,
    rf: f64,
    spans: &[OpenSpan],
) -> Vec<Seg> {
    let n = lp.pts.len();
    let suppressed: Vec<bool> = lp.pts.iter().map(|&p| point_on_spans(spans, p)).collect();
    let inset = |_: usize, _: Vec2, _: Vec2| 0.0f64;
    let radius = move |i: usize, convex: bool| {
        if suppressed[i] {
            return 0.0;
        }
        let mut r = if convex { rc } else { rf };
        let prev = (i + n - 1) % n;
        let next = (i + 1) % n;
        if suppressed[prev] {
            r = r.min(((lp.pts[i] - lp.pts[prev]).length() - OPEN_CORNER_CLEARANCE).max(0.0));
        }
        if suppressed[next] {
            r = r.min(((lp.pts[next] - lp.pts[i]).length() - OPEN_CORNER_CLEARANCE).max(0.0));
        }
        r
    };
    shape_loop(
        lp,
        &LoopStyle {
            inset: &inset,
            radius: &radius,
        },
    )
}

pub(super) fn shape_cavity_loop(lp: &TracedLoop, rc: f64, rf: f64) -> Vec<Seg> {
    let inset = |_: usize, _: Vec2, _: Vec2| 0.0f64;
    let radius = move |_: usize, convex: bool| if convex { rc } else { rf };
    let segs = shape_loop(
        lp,
        &LoopStyle {
            inset: &inset,
            radius: &radius,
        },
    );
    if loop_area(&segs) < 0.0 {
        reverse_loop(&segs)
    } else {
        segs
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::layout::effective_walls;

    fn cells(coords: &[(i32, i32)]) -> Vec<GridCell> {
        coords.iter().map(|&(x, y)| GridCell { x, y }).collect()
    }

    /// A cyclic point sequence in a canonical rotation, so two tracings of one
    /// loop compare regardless of where each of them chose to start.
    fn rotated(pts: &[Vec2]) -> Vec<Vec2> {
        let n = pts.len();
        let first = (0..n)
            .min_by(|&i, &j| {
                (pts[i].x, pts[i].y)
                    .partial_cmp(&(pts[j].x, pts[j].y))
                    .expect("traced corners are finite")
            })
            .expect("a non-empty loop");
        (0..n).map(|i| pts[(first + i) % n]).collect()
    }

    fn same_loop(a: &[Vec2], b: &[Vec2]) -> bool {
        if a.len() != b.len() {
            return false;
        }
        let (a, b) = (rotated(a), rotated(b));
        a.iter().zip(&b).all(|(p, q)| (*p - *q).length() < 1e-3)
    }

    /// What the walk gives at the two places the tracer does something else, at
    /// the default wall (`t = 1.45`, half a divider `0.6`).
    ///
    /// A divider junction: cell (1,1) divided off a 2x2 leaves an L whose
    /// reentrant corner is `(41.4, 41.4)`, where the two divider inset lines
    /// meet. The tracer puts three corners there -- `(42, 41.4) (42, 42)
    /// (41.4, 42)` -- keeping a `0.6 x 0.6` square of the junction as cavity.
    ///
    /// A divider by a notch: the L piece divided across its own arm leaves a 2x1
    /// whose north side steps from the walled inset to the divider's, and the
    /// step belongs at the lattice point. The tracer additionally carves the
    /// reentrant corner patch, which by then is `1.45 x 0.85` of material
    /// hanging into this compartment below the divider.
    #[test]
    fn a_divider_junction_and_a_divider_by_a_notch_walk_to_a_single_corner() {
        let wt = 1.2f64;
        let walk = |piece: &[GridCell], dividers: &[GridEdge], comp: usize| -> Vec<Vec2> {
            let walls = effective_walls(piece, piece, &[], dividers);
            let comps = crate::layout::compartments(piece, &walls.dividers);
            let steps = boundary_steps(&comps[comp]);
            assert_eq!(steps.len(), 1, "the compartment has one boundary loop");
            compartment_corners(&steps[0], GRID_PITCH, &|e| cavity_inset(&walls, wt, e))
        };
        let close = |a: &[Vec2], b: &[(f64, f64)]| {
            assert!(
                same_loop(
                    a,
                    &b.iter().map(|&(x, y)| Vec2::new(x, y)).collect::<Vec<_>>()
                ),
                "{a:?}"
            );
        };

        let block = cells(&[(0, 0), (1, 0), (0, 1), (1, 1)]);
        let junction = [
            GridEdge {
                x: 1,
                y: 1,
                orientation: Orientation::V,
            },
            GridEdge {
                x: 1,
                y: 1,
                orientation: Orientation::H,
            },
        ];
        close(
            &walk(&block, &junction, 0),
            &[
                (1.45, 1.45),
                (82.55, 1.45),
                (82.55, 41.4),
                (41.4, 41.4),
                (41.4, 82.55),
                (1.45, 82.55),
            ],
        );

        let ell = cells(&[(0, 0), (1, 0), (0, 1)]);
        let across = [GridEdge {
            x: 0,
            y: 1,
            orientation: Orientation::H,
        }];
        close(
            &walk(&ell, &across, 0),
            &[
                (1.45, 1.45),
                (82.55, 1.45),
                (82.55, 40.55),
                (42.0, 40.55),
                (42.0, 41.4),
                (1.45, 41.4),
            ],
        );
    }
}
