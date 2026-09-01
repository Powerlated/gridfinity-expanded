//! The construction of one printable piece, as a `Program` of kernel ops.
//!
//! `plan_piece` is the sequence the whole model comes down to, and it is five
//! phases with two values carried between them. `author_outline` settles
//! everything about the *outside* of the piece -- the profile, the wall
//! thickness it can carry, where the pegs will have to weld -- into a
//! `PieceOutline`. `plan_cavities` authors each compartment against that,
//! producing one `PlannedCavity` per compartment and the loops of the standing
//! wall. `emit_cavity_ops` turns those into slab stacks and blend requests,
//! `emit_pegs` writes the base, and `emit_base_and_rim` closes the piece with
//! the outer walls, the bridge undersides, the wall sectors and the rim.
//!
//! Only the last three write to the `Program`, and they write in the order the
//! construction debugger steps through: pegs, outer walls, bridge undersides,
//! cavities, wall sectors, rim faces, fillet. Everything here writes ops rather
//! than geometry, which is what lets that debugger run any prefix or subset.

use super::*;
use gridfinity_brep::fillet::feasible::{
    MIN_TORUS_MAJOR, blendable_segs, island_clears, max_inward_radius,
};
use gridfinity_brep::math::{Vec2, Vec3};
use gridfinity_brep::nesting::stitch_loops_2d;
use crate::perf::{Scope, scope};
use gridfinity_brep::program::{
    DirLoop as POpDirLoop, HoleProfile as PHoleProfile, Op as POp, PlaneRef as PPlaneRef, Program,
};
use gridfinity_brep::region2d::{chain_loops, presplit_regions, region_difference, split_regions};
use gridfinity_brep::round::{
    drop_degenerate, has_sharp_corner, is_convex_arc, round_sharp_corners, seg_mid,
};
use gridfinity_brep::sketch::{COINCIDENT, Seg, Sketch, loop_area, point_in_segs, reverse_loop};
use crate::layout::{EdgeClass, EffectiveWalls, GridCell, GridEdge, classify_edge_in};
use std::collections::HashMap;

/// Everything settled about the outside of one piece before any cavity is
/// authored: the outer profile and the peg cuts it will need, the open spans,
/// the wall thickness and the two z heights the whole plan is measured against.
///
/// `slope` is the piece's own. An opened sloped bin keeps its ramp: the opened
/// compartment's floor lies in it, the standing wall stands on it, and a plinth
/// carries the outline from `floor_z` up to it. It used to be dropped outright
/// whenever any edge was open, which built a flat part for a user who had asked
/// for a ramp -- and built it cleanly, so only `fuzz_params_broad`'s floor
/// comparison ever saw it.
struct PieceOutline {
    loops: OuterLoops,
    shared: SharedWithPegs,
    pitch: f64,
    spans: Vec<OpenSpan>,
    peg_splits: HashMap<GridEdge, Vec<f64>>,
    peg_arcs: Vec<Vec2>,
    wt: f64,
    openish: bool,
    slope: Option<BinSlope>,
    total_h: f64,
    floor_z: f64,
}

impl PieceOutline {
    /// The cavity's depth: floor to rim.
    fn cavity_depth(&self) -> f64 {
        self.total_h - self.floor_z
    }
}

/// One compartment, planned: its boundary as clipped against the outline, the
/// islands standing in it, the floor-fillet radius that survived every clamp,
/// and the band stack a partial-height wall crossing it needs.
struct PlannedCavity {
    lp: CavityLoop,
    islands: Vec<Island>,
    fr: f64,
    banded: Option<Banded>,
}

/// What the compartments come to before anything is pushed to the `Program`:
/// the ops that build them, the edges the floor fillet is requested along, and
/// the loops the rim face has to close over them.
struct CavityOps {
    ops: Vec<(String, POp)>,
    fillet_edges: Vec<(Seg, f64, f64)>,
    rim_holes: Vec<Vec<Seg>>,
    island_tops: Vec<Vec<Seg>>,
}

/// One peg's three lofted rings, already cut wherever the outline above was.
type PegRings = (GridCell, Vec<Seg>, Vec<Seg>, Vec<Seg>);

/// The ops one peg always writes, whatever the fasteners: three registered
/// sketches, the loft through them, and the bottom cap that closes it.
const PEG_OPS: usize = 5;

pub(super) fn plan_piece(
    p: &Params,
    cells: &[GridCell],
    bin_cells: &[GridCell],
    walls: EffectiveWalls,
    slope: Option<BinSlope>,
    pockets: &[Pocket],
    tag: &str,
    prog: &mut Program,
) {
    let _perf = scope(Scope::PlanPiece);
    let mut outline = author_outline(p, cells, bin_cells, &walls, slope);
    let (planned, wall_loops) = plan_cavities(p, cells, &walls, pockets, &mut outline, tag);
    let ramp = outline.slope.map(|sl| SlopedFloor::of(sl, bin_cells, p.pitch, &outline));
    let cavity_ops = emit_cavity_ops(ramp.as_ref(), &outline, planned);
    let pegs = peg_rings(cells, p.pitch, &outline);
    emit_pegs(p, &pegs, tag, prog);
    emit_base_and_rim(
        &outline,
        ramp.as_ref(),
        &pegs,
        wall_loops,
        cavity_ops,
        tag,
        prog,
    );
}

/// The one tilted plane a sloped bin's cavity floor lies in, and the height it
/// reaches over any point of the bin.
///
/// The gradient is the **bin's**, not a compartment's -- `slope_span` measures
/// across `bin_cells`, so every compartment of one bin rides one ramp -- and it
/// is flattened until the high end clears the rim by `SLOPE_RIM_HEADROOM`. The
/// plane is derived once per piece because three emitters have to agree on it
/// exactly: the cavity floor lies in it, the standing wall stands on it, and the
/// plinth beneath that wall is capped by it. Two of them solving it separately
/// would put their shared ring a float apart and open the solid along it.
pub(super) struct SlopedFloor {
    origin: Vec3,
    normal: Vec3,
    ux: f64,
    uy: f64,
    eff_m: f64,
}

impl SlopedFloor {
    fn of(sl: BinSlope, bin_cells: &[GridCell], pitch: f64, outline: &PieceOutline) -> SlopedFloor {
        let (ux, uy) = uphill_unit(sl.dir);
        let (min_a, span) = slope_span(bin_cells, pitch, ux, uy);
        let m = sl
            .angle_deg
            .to_radians()
            .tan()
            .clamp(0.0, MAX_SLOPE_GRADIENT);
        let h_max = (m * span)
            .min(outline.cavity_depth() - SLOPE_RIM_HEADROOM)
            .max(0.0);
        let eff_m = if span > MIN_SLOPE_SPAN {
            h_max / span
        } else {
            0.0
        };
        let normal = Vec3::new(-eff_m * ux, -eff_m * uy, 1.0).normalize();
        assert!(
            (normal.length() - 1.0).abs() < 1e-5 && normal.z > 0.0,
            "a sloped floor's normal is a unit vector pointing into the cavity, got {normal:?}"
        );
        assert!(
            eff_m * span <= outline.cavity_depth() - SLOPE_RIM_HEADROOM + COINCIDENT,
            "a ramp rising {} over a {span} mm run clears the rim by SLOPE_RIM_HEADROOM in a \
             cavity {} mm deep",
            eff_m * span,
            outline.cavity_depth()
        );
        SlopedFloor {
            origin: Vec3::new(0.0, 0.0, outline.floor_z - eff_m * min_a),
            normal,
            ux,
            uy,
            eff_m,
        }
    }

    /// The ramp's height over `pt`, never below the flat floor it starts from.
    fn z_of(&self, pt: Vec2) -> f64 {
        self.origin.z + self.eff_m * (self.ux * pt.x + self.uy * pt.y)
    }

    fn plane(&self) -> PPlaneRef {
        PPlaneRef::Tilted {
            origin: self.origin,
            normal: self.normal,
        }
    }
}

/// The piece's outer profile and everything read off it, from the piece's cells
/// and the bin they belong to.
///
/// Perimeter edges are inset `HALF_TOL` and seam edges -- those internal to the
/// *bin* but on this piece's boundary -- not at all, so two pieces of one bin
/// abut exactly on the cut plane. `SharedWithPegs::corners` is narrowed to the
/// convex corners the walk actually rounded: a diagonal pinch visits one lattice
/// point twice, rounding it on one visit and squaring it on the other, and only
/// the rounded visit welds to the peg.
fn author_outline(
    p: &Params,
    cells: &[GridCell],
    bin_cells: &[GridCell],
    walls: &EffectiveWalls,
    slope: Option<BinSlope>,
) -> PieceOutline {
    let _g = scope(Scope::PlanOuter);
    let openish = !walls.open.is_empty();

    let bin_set = crate::layout::cell_set(bin_cells);
    let seam = |e: &GridEdge| classify_edge_in(&bin_set, *e) == EdgeClass::Internal;
    let inset = |e: &GridEdge| -> f64 { if seam(e) { 0.0 } else { HALF_TOL } };
    let walled = |e: &GridEdge| walls.walled.contains(e);
    let mut shared = SharedWithPegs::default();
    let outer_loops: Vec<Vec<OuterPiece>> = boundary_steps(cells)
        .iter()
        .map(|steps| author_outer_loop(steps, p.pitch, &inset, &walled, &mut shared))
        .collect();
    shared.corners = shared
        .corners
        .difference(&shared.squared)
        .copied()
        .collect();

    PieceOutline {
        loops: OuterLoops::new(outer_loops),
        shared,
        pitch: p.pitch,
        spans: open_spans(cells, p.pitch, walls),
        peg_splits: HashMap::new(),
        peg_arcs: Vec::new(),
        wt: buildable_wall_thickness(p.wall_thickness, openish, slope.is_some()),
        openish,
        slope,
        total_h: p.total_height(),
        floor_z: BASE_TOTAL_HEIGHT + FLOOR_THICKNESS,
    }
}

/// Every compartment of the piece, planned, plus the loops of the wall left
/// standing around them.
///
/// The cavity is authored by the walk and then clipped: an opening is an inset
/// of *nothing*, so an opened cavity runs out past the outline to the pitch line
/// and `clip_cavity_to_outline` pulls it back. The clip is also where the
/// standing wall's inner boundary comes from, and where `outline` picks up the
/// vertices the wall above and the base below have to share -- hence the `&mut`.
///
/// A closed piece returns no wall loops at all: its wall is whatever its own
/// cavity stack did not carve out of the solid outline.
fn plan_cavities(
    p: &Params,
    cells: &[GridCell],
    walls: &EffectiveWalls,
    pockets: &[Pocket],
    outline: &mut PieceOutline,
    tag: &str,
) -> (Vec<PlannedCavity>, Vec<Vec<Seg>>) {
    let _g = scope(Scope::PlanCavity);
    let rc = p.cavity_corner_radius.max(0.0);
    let fr = buildable_floor_fillet(
        p.floor_fillet,
        outline.cavity_depth(),
        rc,
        outline.slope.is_some(),
    );
    let cavity = if pockets.is_empty() {
        walked_cavity(cells, p.pitch, walls, outline.wt)
    } else {
        pocket_cavity(pockets)
    };

    let corner_r = (OUTER_R - outline.wt).max(0.0);
    let (convex_r, concave_r) = if outline.slope.is_some() {
        (0.0, 0.0)
    } else {
        (rc.max(corner_r), fr)
    };
    let shapes: Vec<Vec<Seg>> = cavity
        .iter()
        .map(|(ol, _)| {
            if outline.openish {
                shape_cavity_loop_open(ol, convex_r, concave_r, &outline.spans)
            } else {
                shape_cavity_loop(ol, convex_r, concave_r)
            }
        })
        .collect();
    // The cavity and the wall are two booleans over the same pair of regions,
    // and their results have to weld to each other along the wall they share.
    // One presplit gives both a common segmentation; running each sweep on raw
    // inputs instead leaves the shared boundary cut at f64-different points and
    // the solid open at an edge nothing pairs with.
    let (shapes, clipped_outline) = if outline.openish {
        let mut pre = presplit_regions(&[shapes, outline_region(&outline.loops)]);
        let region = pre.pop().expect("two regions in, two out");
        (pre.pop().expect("two regions in, two out"), region)
    } else {
        (shapes, Vec::new())
    };

    let mut planned: Vec<PlannedCavity> = Vec::new();
    let mut opened: Vec<Vec<Seg>> = Vec::new();
    let mut wall_inner: Vec<Seg> = Vec::new();
    for (oi, (_, holes)) in cavity.iter().enumerate() {
        let shape = shapes[oi].clone();
        let cls = if outline.openish {
            clip_and_collect_wall(&shape, &clipped_outline, &mut opened, &mut wall_inner)
        } else {
            vec![CavityLoop::untouched(shape)]
        };
        assert!(
            !cls.is_empty(),
            "a cavity loop vanished when clipped to the bin outline"
        );
        for cl in cls {
            let islands: Vec<Island> = holes
                .iter()
                .map(|il| Island {
                    segs: shape_cavity_loop(il, rc, fr),
                    top: None,
                    fr: 0.0,
                })
                .collect();
            let mut entries = carve_full_walls(p, cl, islands, rc, fr, outline);
            band_partial_walls(p, &mut entries, fr, outline);
            planned.extend(settle_blend_radii(entries, fr));
        }
    }

    let wall_loops = if outline.openish {
        author_standing_wall(&clipped_outline, &opened, &wall_inner, tag, outline)
    } else {
        Vec::new()
    };
    (planned, wall_loops)
}

/// One compartment's shape clipped to the outline, with the wall it leaves
/// behind recorded into `opened` and `wall_inner`.
///
/// A clipped loop is made of exactly two kinds of run: the ones lying *on* the
/// outline, which are the openings and bound no material at all, and the ones a
/// wall still stands against. The second kind, reversed so it winds as a hole,
/// is the whole of the standing wall's inner boundary -- there is nothing else
/// between the cavity and the outline. Every loop this shape clipped into
/// contributes, not just the touched ones: the whole shape is what leaves the
/// wall, so a sub-loop falling clear of the outline is an ordinary hole in it.
fn clip_and_collect_wall(
    shape: &[Seg],
    outline_region: &[Vec<Seg>],
    opened: &mut Vec<Vec<Seg>>,
    wall_inner: &mut Vec<Seg>,
) -> Vec<CavityLoop> {
    let cls = clip_cavity_to_outline(shape, outline_region);
    if cls.iter().any(|c| c.touched()) {
        opened.push(shape.to_vec());
        for cl in &cls {
            for (i, sg) in cl.segs.iter().enumerate() {
                if !cl.coincident[i] {
                    wall_inner.push(sg.reversed());
                }
            }
        }
    }
    cls
}

/// The compartment carved around every full-height inner wall that lies inside
/// it, as one entry per piece the carving left.
///
/// A wall crossing the compartment cuts it in two, and one standing free in the
/// middle opens a hole, so the difference is re-read by winding: positive loops
/// are outers, negative ones are holes matched to the smallest outer containing
/// them. A compartment the outline touched is left alone -- its wall is already
/// the opening's -- and so is one no wall reaches.
///
/// A sloped bin takes no free-form inner wall at all. The wall is carved out of
/// the cavity as a z-prism island whose bottom ring sits at a flat `floor_z`,
/// but a sloped floor is a tilted plane, so the island's bottom edges do not lie
/// on the floor they meet -- `audit` reports `EdgeOnSurface` and
/// `EdgeVertexGeometry` in equal numbers, and a plain straight divider is as
/// broken as a diagonal one. Dropping it means a sloped bin builds without its
/// dividers rather than failing outright, which is what the partial-height
/// branch does too.
fn carve_full_walls(
    p: &Params,
    cl: CavityLoop,
    islands: Vec<Island>,
    rc: f64,
    fr: f64,
    outline: &PieceOutline,
) -> Vec<(CavityLoop, Vec<Island>, Option<Banded>)> {
    let full_walls: Vec<Vec<Seg>> = if outline.slope.is_some() {
        Vec::new()
    } else {
        p.inner_walls
            .iter()
            .filter(|w| w.height.is_none_or(|h| h >= outline.cavity_depth()))
            .filter_map(|w| inner_wall_quad_in(w, fr, &cl.segs))
            .collect()
    };
    if cl.touched() || full_walls.is_empty() {
        return vec![(cl, islands, None)];
    }

    let before = loop_area(&cl.segs);
    let mut region: Vec<Vec<Seg>> = vec![cl.segs.clone()];
    region.extend(islands.iter().map(|il| reverse_loop(&il.segs)));
    for q in &full_walls {
        region = region_difference(&region, std::slice::from_ref(q));
    }
    let mut outs: Vec<(Vec<Seg>, Vec<Island>)> = Vec::new();
    let mut hole_loops: Vec<Vec<Seg>> = Vec::new();
    for lp in region {
        let lp = round_sharp_corners(&lp, rc, fr);
        if loop_area(&lp) > 0.0 {
            outs.push((lp, Vec::new()));
        } else {
            hole_loops.push(lp);
        }
    }
    for h in hole_loops {
        let pt = h[0].start();
        let mut best: Option<usize> = None;
        for (i, (o, _)) in outs.iter().enumerate() {
            if point_in_segs(pt, o)
                && best.is_none_or(|bi| loop_area(o).abs() < loop_area(&outs[bi].0).abs())
            {
                best = Some(i);
            }
        }
        if let Some(bi) = best {
            outs[bi].1.push(Island {
                segs: reverse_loop(&h),
                top: None,
                fr: 0.0,
            });
        }
    }
    let carved: f64 = outs.iter().map(|(o, _)| loop_area(o)).sum();
    assert!(
        carved <= before + COINCIDENT,
        "carving inner walls out of a compartment only removes material, but its {before} mm^2          came back as {carved} mm^2 over {} piece(s)",
        outs.len()
    );
    outs.into_iter()
        .map(|(o, isls)| (CavityLoop::untouched(o), isls, None))
        .collect()
}

/// Every partial-height inner wall folded into whichever entry holds it: an
/// `Island` with a `top` where the wall lies wholly inside one, a `Banded` slab
/// stack where it crosses the boundary, and nothing at all where it is not
/// inside any of them or where it lands on an island already there.
///
/// A wall crossing the boundary is the case the bands exist for. `outline_a`
/// carries the notch's provenance tag so `plan_cavity_banded` can name the
/// contact runs its ramp blends along without comparing coordinates;
/// `outline_b` is the same boundary cut at the same points, which is what the
/// band above the wall is built from. Both are re-chained, and a boundary that
/// does not chain back into the one loop it was is a defect rather than a case.
///
/// **A sloped bin takes none of them**, the same rule `carve_full_walls` holds
/// full-height walls to and for the same reason: every one of these is carved
/// as a z-prism whose bottom ring sits at a flat `floor_z`, and a tilted floor
/// is not there. The guard used to sit between the two branches, so the island
/// branch -- a wall lying wholly inside one compartment -- reached a sloped bin
/// anyway and left an edge along the wall unpaired at the rim. It is both
/// classes `fuzz_params_broad` still had on a slope.
fn band_partial_walls(
    p: &Params,
    entries: &mut [(CavityLoop, Vec<Island>, Option<Banded>)],
    fr: f64,
    outline: &PieceOutline,
) {
    if outline.slope.is_some() {
        return;
    }
    let partial_walls: Vec<(&InnerWall, Vec<Seg>, f64)> = p
        .inner_walls
        .iter()
        .filter_map(|w| {
            let h = w.height?;
            if h >= outline.cavity_depth() {
                return None;
            }
            Some((
                w,
                inner_wall_quad(w, 0.0)?,
                outline.floor_z + h.max(MIN_PARTIAL_WALL_HEIGHT),
            ))
        })
        .collect();
    'walls: for &(w, ref q, t) in &partial_walls {
        for (ecl, eisl, band) in entries.iter_mut() {
            if ecl.touched() {
                continue;
            }
            let corners: Vec<Vec2> = q.iter().map(|s| s.start()).collect();
            let n_in = corners
                .iter()
                .filter(|&&c| point_in_segs(c, &ecl.segs))
                .count();
            if n_in == 0 {
                continue;
            }
            let clear = corners
                .iter()
                .all(|&c| eisl.iter().all(|il| !point_in_segs(c, &il.segs)));
            if !clear {
                continue 'walls;
            }
            if n_in == corners.len() {
                let rounded = inner_wall_quad(w, fr).expect("non-degenerate (filtered above)");
                eisl.push(Island {
                    segs: rounded,
                    top: Some(t),
                    fr: 0.0,
                });
                continue 'walls;
            }
            let bd = band.get_or_insert_with(|| Banded {
                outline_a: vec![ecl.segs.iter().map(|&s| (s, None)).collect()],
                outline_b: ecl.segs.clone(),
                notches: Vec::new(),
            });
            let ni = bd.notches.len();
            let qa: Vec<Vec<(Seg, Option<usize>)>> =
                vec![q.iter().map(|&s| (s, Some(ni))).collect()];
            let sa = split_regions(&bd.outline_a, &qa);
            if sa.b_inside.is_empty() || sa.a_inside.is_empty() {
                continue 'walls;
            }
            let mut kept = sa.a_outside.clone();
            kept.extend(sa.b_inside.iter().map(|&(s, t)| (s.reversed(), t)));
            bd.outline_a = chain_loops(kept);
            let ob: Vec<Vec<(Seg, ())>> = vec![
                std::mem::take(&mut bd.outline_b)
                    .into_iter()
                    .map(|s| (s, ()))
                    .collect(),
            ];
            let qb: Vec<Vec<(Seg, ())>> = vec![q.iter().map(|&s| (s, ())).collect()];
            let sb = split_regions(&ob, &qb);
            let mut b_all = sb.a_outside;
            b_all.extend(sb.a_inside);
            bd.outline_b = one_closed_loop(b_all);
            bd.notches.push(Notch {
                quad: q.clone(),
                contact: sa.a_inside.into_iter().map(|(s, _)| s).collect(),
                top: t,
            });
            continue 'walls;
        }
    }
    for (_, _, band) in entries.iter() {
        let Some(bd) = band else { continue };
        assert!(
            !bd.notches.is_empty(),
            "a compartment carries a band only because a partial wall crossed it, so its band              holds at least one notch"
        );
        for n in &bd.notches {
            assert!(
                n.top > outline.floor_z && n.top < outline.total_h,
                "a partial wall's top stands strictly between the floor {} and the rim {}, got {}",
                outline.floor_z,
                outline.total_h,
                n.top
            );
            assert!(
                !n.contact.is_empty(),
                "a notch names the runs its ramp blends along, and one that named none would be                  a band with nothing to cap"
            );
        }
    }
}

/// The single closed loop `pieces` chain into, which is what cutting one closed
/// loop at points along it must give back. Chaining into anything else means a
/// piece was lost or a cut landed off the boundary, and `chain_loops` drops a
/// chain it cannot close, so both are asserted rather than absorbed.
fn one_closed_loop(pieces: Vec<(Seg, ())>) -> Vec<Seg> {
    let n_pieces = pieces.len();
    let mut chained = chain_loops(pieces);
    assert_eq!(
        chained.len(),
        1,
        "a compartment's boundary cut by a partial wall is the same single closed loop \
         subdivided, but {n_pieces} piece(s) chained into {} loop(s)",
        chained.len()
    );
    let one = chained.pop().expect("exactly one loop, asserted above");
    assert_eq!(
        one.len(),
        n_pieces,
        "chaining a compartment's cut boundary kept {} of its {n_pieces} piece(s)",
        one.len()
    );
    one.into_iter().map(|(s, _)| s).collect()
}

/// Each entry with the blend radius it can actually carry, for the compartment
/// and for every island in it.
///
/// Four things take a radius away, and the widest requested is `fr`. An arc no
/// wider than the ball plus `MIN_TORUS_MAJOR` clamps it to that arc's own radius
/// less `MIN_TORUS_MAJOR`, convex arcs binding the compartment and concave ones
/// the islands. That bound is deliberately **one-sided** and deliberately not
/// `blend_radius_along`'s degeneracy band: in theory a ball wider than the
/// corner it rolls into is an ordinary spindle torus and only the band is
/// unbuildable, and relaxing it to the band was tried -- it takes
/// `fuzz_wall_openings` 0 to 9/150 and `fuzz_stripped_polyominoes` 0 to 26/150,
/// blends across a 0.066 mm corner arriving as self-intersecting faces and
/// tessellation leaks. A compartment carrying an arc the ball cannot roll along
/// therefore still gives up its fillet entirely. A passage narrower than twice the radius clamps it to what fits. A
/// sharp corner terminates a blend chain rather than continuing it, so a loop
/// carrying one takes no fillet -- except when the corner is the pinch a wall
/// opening leaves, where the chain runs out onto the mouth and the rest of the
/// compartment keeps its rounding. And a banded compartment takes none at all,
/// its ramp blends being `plan_cavity_banded`'s to request.
fn settle_blend_radii(
    entries: Vec<(CavityLoop, Vec<Island>, Option<Banded>)>,
    fr: f64,
) -> Vec<PlannedCavity> {
    let clamp = |segs: &[Seg], ball_inside_convex: bool, loop_fr: &mut f64| {
        for s in segs {
            if let Seg::Arc { radius, .. } = s {
                if *radius < *loop_fr + MIN_TORUS_MAJOR
                    && is_convex_arc(segs, s) == ball_inside_convex
                {
                    *loop_fr = (*radius - MIN_TORUS_MAJOR).max(0.0);
                }
            }
        }
    };
    let sharp_kills = |segs: &[Seg]| fr > 0.0 && has_sharp_corner(segs);

    let mut out = Vec::with_capacity(entries.len());
    for (lp, mut islands, banded) in entries {
        let mut loop_fr = fr;
        clamp(&lp.segs, true, &mut loop_fr);
        loop_fr = loop_fr.min(max_inward_radius(&lp.segs));
        if !lp.touched() && sharp_kills(&lp.segs) {
            loop_fr = 0.0;
        }
        for isl in &islands {
            if !island_clears(&isl.segs, &lp.segs, loop_fr) {
                loop_fr = 0.0;
            }
        }
        for isl in &mut islands {
            let mut island_fr = fr;
            clamp(&isl.segs, false, &mut island_fr);
            if sharp_kills(&isl.segs) {
                island_fr = 0.0;
            }
            if !island_clears(&isl.segs, &lp.segs, island_fr + loop_fr) {
                island_fr = 0.0;
            }
            isl.fr = island_fr;
        }
        if banded.is_some() {
            loop_fr = 0.0;
        }
        assert!(
            loop_fr >= 0.0 && loop_fr <= max_inward_radius(&lp.segs),
            "a compartment's settled blend radius is a radius the rolling ball fits: {loop_fr} \
             against a widest inward radius of {} over {} segment(s)",
            max_inward_radius(&lp.segs),
            lp.segs.len()
        );
        assert!(
            loop_fr == 0.0 || lp.touched() || !has_sharp_corner(&lp.segs),
            "a closed compartment keeping its blend is tangent-continuous, but this one asks for \
             {loop_fr} across a sharp corner of its {} segment(s)",
            lp.segs.len()
        );
        out.push(PlannedCavity {
            lp,
            islands,
            fr: loop_fr,
            banded,
        });
    }
    out
}

/// The loops of the wall left standing once every opened compartment has taken
/// its share of the outline, and the outline cut at every vertex they need.
///
/// The wall is **authored, not differenced**. Its outer boundary is the outline
/// wherever no opening replaced it, and its inner boundary is `wall_inner` --
/// each opened compartment's wall-facing runs, reversed. Those are the only two
/// kinds of boundary material has here, so the wall is exactly their union, and
/// the two sets meet precisely where a cavity leaves the outline.
///
/// `presplit_regions` is what makes the outer half a per-segment test rather
/// than a sweep: it gave the outline and the compartment shapes one
/// segmentation, so every outline segment lies wholly inside or wholly outside
/// each shape and its midpoint decides the whole segment.
///
/// This replaces `outline - shape` folded over the compartments one at a time.
/// That fold was forced: the raw shapes overlap out past the outline wherever
/// two compartments face the same empty cell, so they could not be subtracted
/// together, and they could not be clipped first either, because a clipped
/// shape's long runs coincident with the outline resolve to nothing. Containment
/// per segment asks the boolean for none of it -- overlap is harmless, since a
/// segment swallowed twice is still just swallowed -- and it cannot lose a run
/// to a coincidence, because coincident runs are exactly the ones `wall_inner`
/// drops on purpose.
fn author_standing_wall(
    clipped_outline: &[Vec<Seg>],
    opened: &[Vec<Seg>],
    wall_inner: &[Seg],
    tag: &str,
    outline: &mut PieceOutline,
) -> Vec<Vec<Seg>> {
    let mut frags: Vec<(Seg, ())> = Vec::new();
    for sg in clipped_outline.iter().flatten() {
        if !opened.iter().any(|sh| point_in_segs(seg_mid(sg), sh)) {
            frags.push((*sg, ()));
        }
    }
    frags.extend(wall_inner.iter().map(|s| (*s, ())));
    let n_frags = frags.len();
    let chained = chain_loops(frags);
    let kept: usize = chained.iter().map(|l| l.len()).sum();
    assert_eq!(
        kept,
        n_frags,
        "{tag}: the standing wall's boundary is {n_frags} segment(s) but chained into {} closed \
         loop(s) covering only {kept} of them",
        chained.len()
    );
    let w: Vec<Vec<Seg>> = chained
        .into_iter()
        .map(|l| l.into_iter().map(|(s, _)| s).collect())
        .collect();
    let w = drop_degenerate(w);
    // The lip carries the wall's vertices: the standing wall above the floor and
    // the base's outer wall below it meet along it, so the outline is cut at
    // every vertex of the wall. That cut is also where the peg stations come
    // from, so the peg profile still welds to the wall's bottom ring.
    for pt in w.iter().flatten().map(|sg| sg.start()) {
        let (splits, arcs) = (&mut outline.peg_splits, &mut outline.peg_arcs);
        outline.loops.split_outline_at(pt, splits, arcs);
    }
    w
}

/// Every planned compartment turned into the ops that build it, the edges its
/// floor fillet is requested along, and the loops the rim has to close.
///
/// Three shapes of compartment and no fourth. An **opened** one is authored face
/// by face, because its wall is the standing wall's and only its floor and its
/// island towers are its own; its blend request follows the wall, skipping the
/// runs the outer walk replaced. A **banded** one is a slab stack with a band
/// per partial-height wall. Everything else is a plain stack, or -- on a slope
/// -- a tilted floor with sloped walls, which no stack can express.
fn emit_cavity_ops(
    ramp: Option<&SlopedFloor>,
    outline: &PieceOutline,
    planned: Vec<PlannedCavity>,
) -> CavityOps {
    let _g = scope(Scope::PlanOps);
    let (floor_z, total_h) = (outline.floor_z, outline.total_h);
    let mut out = CavityOps {
        ops: Vec::new(),
        fillet_edges: Vec::new(),
        rim_holes: Vec::new(),
        island_tops: Vec::new(),
    };
    let n_planned = planned.len();

    for (ci, pc) in planned.into_iter().enumerate() {
        let PlannedCavity {
            lp,
            islands,
            fr,
            banded,
        } = pc;
        if lp.touched() {
            emit_open_cavity(ci, &lp, &islands, fr, ramp, floor_z, total_h, &mut out);
            continue;
        }
        if let Some(bd) = banded {
            let (stack, opts, tops, rim, blends) =
                plan_cavity_banded(&bd, &islands, floor_z, total_h);
            out.island_tops.extend(tops);
            out.rim_holes.extend(rim);
            out.fillet_edges.extend(blends);
            out.ops.push((
                format!("cavity {ci}: banded slab stack"),
                POp::Slabs { stack, opts },
            ));
            continue;
        }
        match ramp {
            Some(ramp) => emit_sloped_cavity(ci, &lp, &islands, ramp, outline, &mut out),
            None => {
                let (stack, opts, tops, rim, blends) =
                    plan_cavity_flat(&lp.segs, &islands, floor_z, total_h, fr);
                out.island_tops.extend(tops);
                out.rim_holes.extend(rim);
                out.fillet_edges.extend(blends);
                out.ops.push((
                    format!("cavity {ci}: slab stack"),
                    POp::Slabs { stack, opts },
                ));
            }
        }
    }
    assert!(
        out.ops.len() >= n_planned,
        "every planned compartment builds at least one face or stack, but {n_planned} of them          emitted only {} op(s)",
        out.ops.len()
    );
    out
}

/// An opened compartment's own faces: its floor, one tower per island, and the
/// blend requests along the floor edges a wall still stands against.
///
/// An opening deletes the wall over its own run, not the rest of the
/// compartment's. The segments the outer walk replaced are the ones with no wall
/// left to roll against; every other floor-wall edge still gets its fillet, and
/// the chain runs out on the mouth.
#[allow(clippy::too_many_arguments)]
fn emit_open_cavity(
    ci: usize,
    lp: &CavityLoop,
    islands: &[Island],
    fr: f64,
    ramp: Option<&SlopedFloor>,
    floor_z: f64,
    total_h: f64,
    out: &mut CavityOps,
) {
    assert!(
        islands.iter().all(|i| i.top.is_none()) || ramp.is_none(),
        "a sloped bin takes no inner wall, so every island an opened compartment on one carries \
         is an enclosed hole and stands to the rim; cavity {ci} has one with a top"
    );
    for isl in islands {
        out.island_tops.push(isl.segs.clone());
    }
    let floor_holes: Vec<POpDirLoop> = islands.iter().map(|i| (i.segs.clone(), false)).collect();
    out.ops.push((
        format!("cavity {ci} (open): floor"),
        POp::PlanarFace {
            plane: ramp.map_or(
                PPlaneRef::Z {
                    z: floor_z,
                    up: true,
                },
                |r| r.plane(),
            ),
            outer: (lp.segs.clone(), true),
            holes: floor_holes,
        },
    ));
    for (ii, isl) in islands.iter().enumerate() {
        out.ops.push((
            format!("cavity {ci} (open): tower {ii}"),
            match ramp {
                Some(r) => POp::SlopedWall {
                    lower: isl.segs.clone(),
                    upper: isl.segs.clone(),
                    lower_plane: r.plane(),
                    upper_plane: PPlaneRef::Z {
                        z: total_h,
                        up: true,
                    },
                    outward: true,
                },
                None => POp::WallFaces {
                    lower: isl.segs.clone(),
                    upper: isl.segs.clone(),
                    z0: floor_z,
                    z1: total_h,
                    outward: true,
                },
            },
        ));
    }
    assert!(
        ramp.is_none() || fr <= MIN_USEFUL_BLEND,
        "a sloped floor takes no fillet -- `buildable_floor_fillet` zeroes it -- so an opened \
         compartment on a ramp asks for none, got {fr} in cavity {ci}"
    );
    if fr > MIN_USEFUL_BLEND {
        let walled: Vec<bool> = lp.coincident.iter().map(|&c| !c).collect();
        for (s, keep) in lp.segs.iter().zip(blendable_segs(&lp.segs, &walled)) {
            if keep {
                out.fillet_edges.push((*s, floor_z, fr));
            }
        }
    }
    for isl in islands {
        if isl.fr > MIN_USEFUL_BLEND {
            out.fillet_edges
                .extend(isl.segs.iter().map(|s| (*s, floor_z, isl.fr)));
        }
    }
}

/// A sloped compartment's faces: a tilted floor, walls swept between it and the
/// rim, and one island tower per island capped at its own top or at the rim.
///
/// The gradient is the bin's, not the compartment's -- `slope_span` measures
/// across `bin_cells` so every compartment of one bin shares one ramp -- and it
/// is flattened until the high end clears the rim by `SLOPE_RIM_HEADROOM`. An
/// island whose top the ramp has already risen past is capped at the rim
/// instead, since a cap below the floor it stands on is not a face.
fn emit_sloped_cavity(
    ci: usize,
    lp: &CavityLoop,
    islands: &[Island],
    ramp: &SlopedFloor,
    outline: &PieceOutline,
    out: &mut CavityOps,
) {
    let (floor_z, total_h) = (outline.floor_z, outline.total_h);
    for isl in islands {
        if isl.top.is_none() {
            out.island_tops.push(isl.segs.clone());
        }
    }
    let z_of = |pt: Vec2| ramp.z_of(pt);
    let floor_plane = ramp.plane();
    let top_plane = PPlaneRef::Z {
        z: total_h,
        up: true,
    };

    out.ops.push((
        format!("cavity {ci}: sloped walls"),
        POp::SlopedWall {
            lower: lp.segs.clone(),
            upper: lp.segs.clone(),
            lower_plane: floor_plane.clone(),
            upper_plane: top_plane.clone(),
            outward: false,
        },
    ));
    let mut floor_holes: Vec<POpDirLoop> = Vec::new();
    for (ii, isl) in islands.iter().enumerate() {
        let slope_max = isl
            .segs
            .iter()
            .map(|s| z_of(s.start()))
            .fold(floor_z, f64::max);
        let t = isl
            .top
            .filter(|&t| t > slope_max + SLOPE_ISLAND_HEADROOM)
            .unwrap_or(total_h);
        let island_top_plane = PPlaneRef::Z { z: t, up: true };
        out.ops.push((
            format!("cavity {ci}: sloped island {ii} walls"),
            POp::SlopedWall {
                lower: isl.segs.clone(),
                upper: isl.segs.clone(),
                lower_plane: floor_plane.clone(),
                upper_plane: island_top_plane.clone(),
                outward: true,
            },
        ));
        if t < total_h {
            out.ops.push((
                format!("cavity {ci}: sloped island {ii} top"),
                POp::PlanarFace {
                    plane: island_top_plane,
                    outer: (isl.segs.clone(), true),
                    holes: vec![],
                },
            ));
        }
        floor_holes.push((isl.segs.clone(), false));
    }
    out.ops.push((
        format!("cavity {ci}: sloped floor"),
        POp::PlanarFace {
            plane: floor_plane,
            outer: (lp.segs.clone(), false),
            holes: floor_holes,
        },
    ));
    out.rim_holes.push(lp.segs.clone());
}

/// One peg per cell, each of its three rings cut wherever the outline above was
/// cut, so the peg tops weld to the wall's bottom ring rather than meeting it at
/// an edge nothing pairs with.
fn peg_rings(cells: &[GridCell], pitch: f64, outline: &PieceOutline) -> Vec<PegRings> {
    let _g = scope(Scope::PlanOps);
    let (splits, arcs) = (&outline.peg_splits, &outline.peg_arcs);
    let (w_bot, w_mid, w_top) = peg_widths(pitch);
    cells
        .iter()
        .map(|&c| {
            (
                c,
                split_peg_profile(peg_profile(c, pitch, w_bot, PEG_R_BOTTOM), c, pitch, splits, arcs),
                split_peg_profile(peg_profile(c, pitch, w_mid, PEG_R_MID), c, pitch, splits, arcs),
                split_peg_profile(peg_profile(c, pitch, w_top, OUTER_R), c, pitch, splits, arcs),
            )
        })
        .collect()
}

/// The base, pushed to `prog`: each peg as three registered sketches lofted
/// between z=0 and `PEG_HEIGHT`, its four fastener bores where `Params` asks for
/// them, and its bottom cap with those bores as holes.
///
/// A magnet and a screw together become one stepped counterbore rather than two
/// concentric holes, because two coaxial cylinders meeting at a shoulder is a
/// shape the kernel would have to boolean and a counterbore is one it lofts.
fn emit_pegs(p: &Params, pegs: &[PegRings], tag: &str, prog: &mut Program) {
    let _g = scope(Scope::PlanOps);
    let fastener_profile: Option<PHoleProfile> = match (p.magnet_holes, p.screw_holes) {
        (true, true) => Some(PHoleProfile::Counterbore {
            bore_r: SCREW_RADIUS,
            bore_d: SCREW_DEPTH,
            head_r: MAGNET_RADIUS,
            head_d: MAGNET_DEPTH,
        }),
        (true, false) => Some(PHoleProfile::Plain {
            radius: MAGNET_RADIUS,
            depth: MAGNET_DEPTH,
        }),
        (false, true) => Some(PHoleProfile::Plain {
            radius: SCREW_RADIUS,
            depth: SCREW_DEPTH,
        }),
        (false, false) => None,
    };
    let before = prog.steps.len();
    for (ci, (c, s_bot, s_mid, s_top)) in pegs.iter().enumerate() {
        let bot_name = format!("{tag}: peg {ci} bot");
        let mid_name = format!("{tag}: peg {ci} mid");
        let top_name = format!("{tag}: peg {ci} top");
        prog.push(
            format!("register {bot_name}"),
            POp::Sketch {
                name: bot_name.clone(),
                profile: s_bot.clone(),
            },
        );
        prog.push(
            format!("register {mid_name}"),
            POp::Sketch {
                name: mid_name.clone(),
                profile: s_mid.clone(),
            },
        );
        prog.push(
            format!("register {top_name}"),
            POp::Sketch {
                name: top_name.clone(),
                profile: s_top.clone(),
            },
        );
        prog.push(
            format!("{tag}: peg {ci} loft"),
            POp::Loft {
                profiles: vec![
                    (bot_name.clone(), 0.0),
                    (mid_name.clone(), PEG_Z1),
                    (mid_name.clone(), PEG_Z2),
                    (top_name.clone(), PEG_HEIGHT),
                ],
                outward: true,
            },
        );
        let ccx = (c.x as f64 + 0.5) * p.pitch;
        let ccy = (c.y as f64 + 0.5) * p.pitch;
        if let Some(profile) = &fastener_profile {
            let inset = fastener_inset(p.pitch);
            for (dx, dy) in FASTENER_QUADRANTS {
                let hx = ccx + dx * inset;
                let hy = ccy + dy * inset;
                prog.push(
                    format!("{tag}: peg {ci} fastener ({hx:.0},{hy:.0})"),
                    POp::Hole {
                        at: Vec2::new(hx, hy),
                        from_z: 0.0,
                        profile: profile.clone(),
                    },
                );
            }
        }
        let mut cap_holes: Vec<POpDirLoop> = Vec::new();
        if let Some(profile) = &fastener_profile {
            let mouth_r = profile.mouth_radius();
            let inset = fastener_inset(p.pitch);
            for (dx, dy) in FASTENER_QUADRANTS {
                let hx = ccx + dx * inset;
                let hy = ccy + dy * inset;
                let mouth = Sketch::circle(hx, hy, mouth_r).loops.remove(0);
                cap_holes.push((mouth, false));
            }
        }
        prog.push(
            format!("{tag}: peg {ci} bottom cap"),
            POp::PlanarFace {
                plane: PPlaneRef::Z { z: 0.0, up: false },
                outer: (s_bot.clone(), false),
                holes: cap_holes,
            },
        );
    }
    let per_peg = if fastener_profile.is_some() {
        PEG_OPS + FASTENER_QUADRANTS.len()
    } else {
        PEG_OPS
    };
    assert_eq!(
        prog.steps.len() - before,
        pegs.len() * per_peg,
        "each of the {} peg(s) writes {per_peg} op(s): three sketches, a loft, its bores and a          bottom cap",
        pegs.len()
    );
}

/// Everything that closes the piece, pushed to `prog` in construction order:
/// the outer walls, the bridge undersides between the pegs, the cavity ops, the
/// standing wall's sectors, the rim faces and finally the floor fillet.
///
/// The bridge underside is every peg-top segment facing open air plus every
/// outer-profile segment no peg welded to, stitched into planar faces at
/// `PEG_HEIGHT` -- a loop contained in another becomes a hole of it, so an
/// interior peg's ring is a hole and not a disk.
///
/// The rim is the piece's top face: the outer walls as outers, island tops as
/// outers of their own, and every cavity opening as a hole of the smallest
/// outer containing it.
#[allow(clippy::too_many_arguments)]
fn emit_base_and_rim(
    outline: &PieceOutline,
    ramp: Option<&SlopedFloor>,
    pegs: &[PegRings],
    wall_loops: Vec<Vec<Seg>>,
    cavity_ops: CavityOps,
    tag: &str,
    prog: &mut Program,
) {
    let _g = scope(Scope::PlanStitch);
    let (floor_z, total_h) = (outline.floor_z, outline.total_h);
    let outer_rings: Vec<(Vec<Seg>, Vec<bool>)> = outline
        .loops
        .loops
        .iter()
        .map(|pieces| {
            (
                pieces.iter().map(|p| p.seg).collect(),
                pieces.iter().map(|p| p.shared).collect(),
            )
        })
        .collect();

    assert!(
        !outer_rings.is_empty(),
        "{tag}: a piece has at least one outer profile loop to stand its walls on"
    );
    for (segs, flags) in &outer_rings {
        assert_eq!(
            segs.len(),
            flags.len(),
            "{tag}: each outer segment carries its own peg-shared flag, got {} flag(s) for {}              segment(s)",
            flags.len(),
            segs.len()
        );
    }
    for (li, (segs, _)) in outer_rings.iter().enumerate() {
        let z1 = if outline.openish { floor_z } else { total_h };
        prog.push(
            format!("{tag}: outer wall {li}"),
            POp::Wall {
                lower: segs.clone(),
                upper: segs.clone(),
                z0: PEG_HEIGHT,
                z1,
                outward: true,
            },
        );
    }

    let mut free: Vec<Seg> = Vec::new();
    for (c, _, _, s_top) in pegs {
        for seg in s_top {
            if peg_seg_free(seg, *c, outline.pitch, &outline.shared) {
                free.push(*seg);
            }
        }
    }
    for (segs, shared_flags) in &outer_rings {
        for (k, seg) in segs.iter().enumerate() {
            if !shared_flags[k] {
                free.push(seg.reversed());
            }
        }
    }
    for (i, (outer, holes)) in stitch_loops_2d(free).into_iter().enumerate() {
        let label = if i == 0 {
            format!("{tag}: bridge underside")
        } else {
            format!("{tag}: bridge underside {i}")
        };
        let hole_loops: Vec<POpDirLoop> = holes.into_iter().map(|h| (h, true)).collect();
        prog.push(
            label,
            POp::PlanarFace {
                plane: PPlaneRef::Z {
                    z: PEG_HEIGHT,
                    up: false,
                },
                outer: (outer, true),
                holes: hole_loops,
            },
        );
    }

    // A sloped bin's opened compartments put their floor on the ramp, so the
    // standing wall stands on the ramp too and a plinth carries the outline from
    // the flat floor up to it. Swept prismatically from `floor_z` instead, the
    // wall's *inner* surface would go on existing below the ramp, where both
    // sides of it are material.
    if let Some(ramp) = ramp
        && outline.openish
    {
        for (li, (segs, _)) in outer_rings.iter().enumerate() {
            prog.push(
                format!("{tag}: ramp plinth {li}"),
                POp::SlopedWall {
                    lower: segs.clone(),
                    upper: segs.clone(),
                    lower_plane: PPlaneRef::Z {
                        z: floor_z,
                        up: true,
                    },
                    upper_plane: ramp.plane(),
                    outward: true,
                },
            );
        }
    }

    for (label, op) in cavity_ops.ops {
        prog.push(format!("{tag}: {label}"), op);
    }

    emit_wall_sectors(&wall_loops, ramp, floor_z, total_h, tag, prog);

    let top_walls: Vec<Vec<Seg>> = if outline.openish {
        wall_loops
    } else {
        outer_rings.iter().map(|(s, _)| s.clone()).collect()
    };
    emit_rim_faces(
        &top_walls,
        &cavity_ops.island_tops,
        &cavity_ops.rim_holes,
        total_h,
        tag,
        prog,
    );

    if !cavity_ops.fillet_edges.is_empty() {
        prog.push(
            format!("{tag}: floor fillet"),
            POp::Fillet {
                edges: cavity_ops.fillet_edges,
            },
        );
    }
}

/// The standing wall swept from the floor to the rim, one op per loop.
///
/// `wall_between` takes the material side from the loop's own direction -- the
/// surface normal it builds points to the right of travel -- so a region wound
/// material-on-the-left needs `outward: true` for every loop it has, outer and
/// hole alike. `region_difference` returns exactly that region, and the area
/// assertion is what says so: the signed areas sum to the material's own area,
/// so every hole is wound against its outer loop and no loop is wound twice.
///
/// The rule this replaced, `outward: loop_area(sl) > 0.0`, flipped the normal on
/// every hole. It agreed with the base's outer wall below `floor_z` only while
/// the two never shared a ring -- an enclosed hole is where they do, and there
/// the wall and the base met at `floor_z` with opposing normals.
fn emit_wall_sectors(
    wall_loops: &[Vec<Seg>],
    ramp: Option<&SlopedFloor>,
    floor_z: f64,
    total_h: f64,
    tag: &str,
    prog: &mut Program,
) {
    let area: f64 = wall_loops.iter().map(|l| loop_area(l)).sum();
    assert!(
        wall_loops.is_empty() || area > 0.0,
        "{tag}: the standing wall's {} loop(s) enclose {area} mm^2, so they are not one region \
         wound material-on-the-left",
        wall_loops.len()
    );
    for (si, sl) in wall_loops.iter().enumerate() {
        assert!(
            loop_area(sl) != 0.0,
            "{tag}: wall sector {si} encloses no area"
        );
        let top = PPlaneRef::Z {
            z: total_h,
            up: true,
        };
        prog.push(
            format!("{tag}: wall sector {si}"),
            match ramp {
                Some(ramp) => POp::SlopedWall {
                    lower: sl.clone(),
                    upper: sl.clone(),
                    lower_plane: ramp.plane(),
                    upper_plane: top,
                    outward: true,
                },
                None => POp::Wall {
                    lower: sl.clone(),
                    upper: sl.clone(),
                    z0: floor_z,
                    z1: total_h,
                    outward: true,
                },
            },
        );
    }
}

/// The piece's top face, as one planar face per outer loop with the openings
/// inside it as holes.
///
/// Outers are the positively wound wall loops plus every island top, which is
/// its own little face standing in a cavity. Holes are the negatively wound wall
/// loops and every cavity's rim opening, each matched to the *smallest* outer
/// containing it so a hole inside an island top does not attach to the bin's
/// whole outline instead.
fn emit_rim_faces(
    top_walls: &[Vec<Seg>],
    island_tops: &[Vec<Seg>],
    rim_holes: &[Vec<Seg>],
    total_h: f64,
    tag: &str,
    prog: &mut Program,
) {
    let mut outers: Vec<(Vec<Seg>, Vec<Vec<Seg>>)> = Vec::new();
    for segs in top_walls {
        if loop_area(segs) > 0.0 {
            outers.push((segs.clone(), Vec::new()));
        }
    }
    for segs in island_tops {
        outers.push((segs.clone(), Vec::new()));
    }
    let holes: Vec<Vec<Seg>> = top_walls
        .iter()
        .filter(|s| loop_area(s) < 0.0)
        .cloned()
        .chain(rim_holes.iter().cloned())
        .collect();
    for hole in &holes {
        let pt = hole[0].start();
        let mut best: Option<usize> = None;
        for (i, (outer, _)) in outers.iter().enumerate() {
            let a = loop_area(outer).abs();
            if point_in_segs(pt, outer) && best.is_none_or(|bi| a < loop_area(&outers[bi].0).abs())
            {
                best = Some(i);
            }
        }
        let bi = best.expect("total_h hole without a containing face");
        outers[bi].1.push(hole.clone());
    }
    let net: f64 = outers
        .iter()
        .map(|(o, hs)| loop_area(o).abs() - hs.iter().map(|h| loop_area(h).abs()).sum::<f64>())
        .sum();
    assert!(
        outers.is_empty() || net > 0.0,
        "{tag}: the rim is material, so its {} face(s) enclose more than the {} opening(s) in          them, but they net {net} mm^2",
        outers.len(),
        holes.len()
    );
    for (i, (outer, holes)) in outers.into_iter().enumerate() {
        let label = if island_tops.is_empty() && top_walls.len() == 1 {
            format!("{tag}: rim face")
        } else {
            format!("{tag}: rim face {i}")
        };
        let hole_loops: Vec<POpDirLoop> = holes.into_iter().map(|h| (h, true)).collect();
        prog.push(
            label,
            POp::PlanarFace {
                plane: PPlaneRef::Z {
                    z: total_h,
                    up: true,
                },
                outer: (outer, true),
                holes: hole_loops,
            },
        );
    }
}
