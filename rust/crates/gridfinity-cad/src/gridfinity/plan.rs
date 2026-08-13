//! The construction of one printable piece, as a `Program` of kernel ops.
//!
//! `plan_piece` is the sequence the whole model comes down to: walk the outer
//! profile, author each compartment's cavity and clip it against that profile,
//! settle every blend radius the compartments can actually carry, emit the
//! cavity stacks and the pegs, then stitch the rings and the bridge undersides
//! that close the base. It writes ops rather than geometry, which is what lets
//! the construction debugger run any prefix or subset of it.

use super::*;
use crate::kernel::fillet::feasible::{
    MIN_TORUS_MAJOR, blendable_segs, island_clears, max_inward_radius,
};
use crate::kernel::math::{Vec2, Vec3};
use crate::kernel::nesting::stitch_loops_2d;
use crate::kernel::program::{
    DirLoop as POpDirLoop, HoleProfile as PHoleProfile, Op as POp, PlaneRef as PPlaneRef, Program,
};
use crate::kernel::region2d::{chain_loops, presplit_regions, region_difference, split_regions};
use crate::kernel::round::{
    drop_degenerate, has_sharp_corner, is_convex_arc, round_sharp_corners, seg_mid,
};
use crate::kernel::sketch::{Seg, Sketch, loop_area, point_in_segs, reverse_loop};
use crate::layout::{EdgeClass, EffectiveWalls, GridCell, GridEdge, classify_edge_in};
use std::collections::HashMap;

pub(super) fn plan_piece(
    p: &Params,
    cells: &[GridCell],
    bin_cells: &[GridCell],
    walls: EffectiveWalls,
    slope: Option<BinSlope>,
    tag: &str,
    prog: &mut Program,
) {
    let _perf = crate::kernel::perf::scope(crate::kernel::perf::Metric::PlanPiece);
    let mut _g = crate::kernel::perf::scope(crate::kernel::perf::Metric::PlanOuter);
    let total_h = p.total_height();
    let floor_z = BASE_TOTAL_HEIGHT + FLOOR_THICKNESS;
    let openish = !walls.open.is_empty();
    let slope = if openish { None } else { slope };

    let bin_set = crate::layout::cell_set(bin_cells);
    let seam = |e: &GridEdge| classify_edge_in(&bin_set, *e) == EdgeClass::Internal;
    let inset = |e: &GridEdge| -> f32 { if seam(e) { 0.0 } else { HALF_TOL } };
    let walled = |e: &GridEdge| walls.walled.contains(e);
    let loops = boundary_steps(cells);
    let mut shared = SharedWithPegs::default();
    let outer_loops: Vec<Vec<OuterPiece>> = loops
        .iter()
        .map(|steps| author_outer_loop(steps, &inset, &walled, &mut shared))
        .collect();
    shared.corners = shared
        .corners
        .difference(&shared.squared)
        .copied()
        .collect();
    let mut o = OuterLoops::new(outer_loops);
    let spans = open_spans(cells, &walls);
    let mut peg_splits: HashMap<GridEdge, Vec<f32>> = HashMap::new();
    // Corner cuts, kept as points: a corner has no station, and the ring it
    // welds to is found by distance from the corner's own centre.
    let mut peg_arcs: Vec<Vec2> = Vec::new();

    let wt = buildable_wall_thickness(p.wall_thickness, openish, slope.is_some());
    drop(_g);
    let mut _g = crate::kernel::perf::scope(crate::kernel::perf::Metric::PlanCavity);
    let cavity_depth = total_h - floor_z;
    let rc = p.cavity_corner_radius.max(0.0);
    let fr = buildable_floor_fillet(p.floor_fillet, cavity_depth, rc, slope.is_some());

    // The cavity is authored: one walk per compartment, each edge at its own
    // inset, less the divider strips standing inside it. An opening is still an
    // inset of nothing, so an opened cavity still runs out to the pitch line and
    // is pulled back by `clip_cavity_to_outline` below -- that boolean is the
    // one this replacement is working towards and it is still here.
    let cavity = walked_cavity(cells, &walls, wt);

    let mut planned: Vec<(CavityLoop, Vec<Island>, f32, Option<Banded>)> = Vec::new();
    let corner_r = (OUTER_R - wt).max(0.0);
    let (convex_r, concave_r) = if slope.is_some() {
        (0.0, 0.0)
    } else {
        (rc.max(corner_r), fr)
    };
    let shapes: Vec<Vec<Seg>> = cavity
        .iter()
        .map(|(ol, _)| {
            if openish {
                shape_cavity_loop_open(ol, convex_r, concave_r, &spans)
            } else {
                shape_cavity_loop(ol, convex_r, concave_r)
            }
        })
        .collect();
    // The cavity and the wall are two booleans over the same pair of regions,
    // and their results have to weld to each other along the wall they share.
    // One presplit gives both a common segmentation; running each sweep on raw
    // inputs instead leaves the shared boundary cut at f32-different points and
    // the solid open at an edge nothing pairs with.
    let (shapes, outline) = if openish {
        let mut pre = presplit_regions(&[shapes, outline_region(&o)]);
        let outline = pre.pop().expect("two regions in, two out");
        (pre.pop().expect("two regions in, two out"), outline)
    } else {
        (shapes, Vec::new())
    };
    let mut opened: Vec<Vec<Seg>> = Vec::new();
    // The standing wall's inner boundary, gathered as the cavities are clipped.
    //
    // A clipped cavity loop is made of exactly two kinds of run: the ones lying
    // *on* the outline, which are the openings and bound no material at all, and
    // the ones a wall still stands against. The second kind, reversed so it
    // winds as a hole, is the whole of the wall's inner boundary -- there is
    // nothing else between the cavity and the outline.
    let mut wall_inner: Vec<Seg> = Vec::new();
    for (oi, (_, holes)) in cavity.iter().enumerate() {
        let shape = shapes[oi].clone();
        let cls = if openish {
            let cls = clip_cavity_to_outline(&shape, &outline);
            if cls.iter().any(|c| c.touched()) {
                opened.push(shape.clone());
                // Every loop this shape clipped into, not just the touched ones:
                // the whole shape is what leaves the wall, so a sub-loop that
                // happens to fall clear of the outline is an ordinary hole in it.
                for cl in &cls {
                    for (i, sg) in cl.segs.iter().enumerate() {
                        if !cl.coincident[i] {
                            wall_inner.push(sg.reversed());
                        }
                    }
                }
            }
            cls
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
            // A sloped bin takes no free-form inner wall at all. The wall is carved
            // out of the cavity as a z-prism island whose bottom ring sits at a
            // flat `floor_z`, but a sloped floor is a tilted plane, so the island's
            // bottom edges do not lie on the floor they meet -- `audit` reports
            // `EdgeOnSurface` and `EdgeVertexGeometry` in equal numbers, and a
            // plain straight divider is as broken as a diagonal one. Dropping the
            // wall is what the partial-height branch below already does on a slope;
            // doing it here too means a sloped bin builds without its dividers
            // rather than failing outright.
            let full_walls: Vec<Vec<Seg>> = if slope.is_some() {
                Vec::new()
            } else {
                p.inner_walls
                    .iter()
                    .filter(|w| w.height.is_none_or(|h| h >= cavity_depth))
                    .filter_map(|w| inner_wall_quad_in(w, fr, &cl.segs))
                    .collect()
            };
            let mut entries: Vec<(CavityLoop, Vec<Island>, Option<Banded>)> = Vec::new();
            if cl.touched() || full_walls.is_empty() {
                entries.push((cl, islands.clone(), None));
            } else {
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
                            && best
                                .is_none_or(|bi| loop_area(o).abs() < loop_area(&outs[bi].0).abs())
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
                for (o, isls) in outs {
                    entries.push((CavityLoop::untouched(o), isls, None));
                }
            }
            let partial_walls: Vec<(&InnerWall, Vec<Seg>, f32)> = p
                .inner_walls
                .iter()
                .filter_map(|w| {
                    let h = w.height?;
                    if h >= cavity_depth {
                        return None;
                    }
                    Some((w, inner_wall_quad(w, 0.0)?, floor_z + h.max(0.5)))
                })
                .collect();
            'walls: for &(w, ref q, t) in &partial_walls {
                for (ecl, eisl, band) in &mut entries {
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
                        let rounded =
                            inner_wall_quad(w, fr).expect("non-degenerate (filtered above)");
                        eisl.push(Island {
                            segs: rounded,
                            top: Some(t),
                            fr: 0.0,
                        });
                        continue 'walls;
                    }
                    if slope.is_some() {
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
                    let n_pieces = b_all.len();
                    let mut chained = chain_loops(b_all);
                    assert_eq!(
                        chained.len(),
                        1,
                        "{tag}: a compartment's boundary cut by a partial wall is the same single \
                         closed loop subdivided, but {n_pieces} piece(s) chained into {} loop(s)",
                        chained.len()
                    );
                    let one = chained.pop().expect("exactly one loop, asserted above");
                    assert_eq!(
                        one.len(),
                        n_pieces,
                        "{tag}: chaining a compartment's cut boundary kept {} of its {n_pieces} \
                         piece(s)",
                        one.len()
                    );
                    bd.outline_b = one.into_iter().map(|(s, _)| s).collect();
                    bd.notches.push(Notch {
                        quad: q.clone(),
                        contact: sa.a_inside.into_iter().map(|(s, _)| s).collect(),
                        top: t,
                    });
                    continue 'walls;
                }
            }
            let clamp = |segs: &[Seg], ball_inside_convex: bool, loop_fr: &mut f32| {
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
            // A sharp corner terminates a blend chain rather than continuing it, so
            // a loop carrying one takes no fillet -- except when the corner is the
            // pinch a wall opening leaves, where the chain runs out onto the mouth
            // and the rest of the compartment keeps its rounding. See `fillet.rs`'s
            // capped runout.
            let sharp_kills = |segs: &[Seg]| fr > 0.0 && has_sharp_corner(segs);
            for (cl, mut islands, banded) in entries {
                let mut loop_fr = fr;
                clamp(&cl.segs, true, &mut loop_fr);
                // A radius wider than the compartment is not a fillet the model
                // may ask for -- the rolling ball does not fit.
                loop_fr = loop_fr.min(max_inward_radius(&cl.segs));
                if !cl.touched() && sharp_kills(&cl.segs) {
                    loop_fr = 0.0;
                }
                for isl in &islands {
                    if !island_clears(&isl.segs, &cl.segs, loop_fr) {
                        loop_fr = 0.0;
                    }
                }
                for isl in &mut islands {
                    let mut island_fr = fr;
                    clamp(&isl.segs, false, &mut island_fr);
                    if sharp_kills(&isl.segs) {
                        island_fr = 0.0;
                    }
                    if !island_clears(&isl.segs, &cl.segs, island_fr + loop_fr) {
                        island_fr = 0.0;
                    }
                    isl.fr = island_fr;
                }
                if banded.is_some() {
                    loop_fr = 0.0;
                }
                assert!(
                    loop_fr >= 0.0 && loop_fr <= max_inward_radius(&cl.segs),
                    "a compartment's settled blend radius is a radius the rolling ball fits: \
                     {loop_fr} against a widest inward radius of {} over {} segment(s)",
                    max_inward_radius(&cl.segs),
                    cl.segs.len()
                );
                assert!(
                    loop_fr == 0.0 || cl.touched() || !has_sharp_corner(&cl.segs),
                    "a closed compartment keeping its blend is tangent-continuous, but this one \
                     asks for {loop_fr} across a sharp corner of its {} segment(s)",
                    cl.segs.len()
                );
                planned.push((cl, islands, loop_fr, banded));
            }
        }
    }

    // The standing wall is the outline with every opened compartment taken out
    // of it. A compartment that keeps all its walls is left solid here and
    // carved by its own cavity stack, exactly as before.
    let wall_loops = if openish {
        // The standing wall is **authored, not differenced**.
        //
        // Its outer boundary is the outline wherever no opening replaced it,
        // and its inner boundary is `wall_inner` -- each opened compartment's
        // wall-facing runs, reversed. Those are the only two kinds of boundary
        // material has here, so the wall is exactly their union, and the two
        // sets meet precisely where a cavity leaves the outline.
        //
        // `presplit_regions` is what makes the outer half a per-segment test
        // rather than a sweep: it gave the outline and the compartment shapes
        // one segmentation, so every outline segment lies wholly inside or
        // wholly outside each shape and its midpoint decides the whole segment.
        //
        // This replaces `outline - shape` folded over the compartments one at a
        // time. That fold was forced: the raw shapes overlap out past the
        // outline wherever two compartments face the same empty cell, so they
        // could not be subtracted together, and they could not be clipped first
        // either, because a clipped shape's long runs coincident with the
        // outline resolve to nothing. Containment per segment asks the boolean
        // for none of it -- overlap is harmless, since a segment swallowed twice
        // is still just swallowed -- and it cannot lose a run to a coincidence,
        // because coincident runs are exactly the ones `wall_inner` drops on
        // purpose.
        let mut frags: Vec<(Seg, ())> = Vec::new();
        for sg in outline.iter().flatten() {
            if !opened.iter().any(|sh| point_in_segs(seg_mid(sg), sh)) {
                frags.push((*sg, ()));
            }
        }
        frags.extend(wall_inner.iter().map(|s| (*s, ())));
        let n_frags = frags.len();
        let chained = chain_loops(frags);
        // `chain_loops` drops a chain it cannot close, so a boundary that does
        // not partition into loops would leave the wall quietly missing a piece
        // rather than failing. Every fragment belongs to exactly one closed
        // loop, and that is the whole claim being made here.
        let kept: usize = chained.iter().map(|l| l.len()).sum();
        assert_eq!(
            kept,
            n_frags,
            "{tag}: the standing wall's boundary is {n_frags} segment(s) but chained into \
             {} closed loop(s) covering only {kept} of them",
            chained.len()
        );
        let w: Vec<Vec<Seg>> = chained
            .into_iter()
            .map(|l| l.into_iter().map(|(s, _)| s).collect())
            .collect();
        let w = drop_degenerate(w);
        for p in w.iter().flatten().map(|sg| sg.start()) {
            o.split_outline_at(p, &mut peg_splits, &mut peg_arcs);
        }
        w
    } else {
        Vec::new()
    };

    drop(_g);
    let mut _g = crate::kernel::perf::scope(crate::kernel::perf::Metric::PlanOps);
    let peg_profiles: Vec<(GridCell, Vec<Seg>, Vec<Seg>, Vec<Seg>)> = cells
        .iter()
        .map(|&c| {
            (
                c,
                split_peg_profile(
                    peg_profile(c, PEG_W_BOTTOM, PEG_R_BOTTOM),
                    c,
                    &peg_splits,
                    &peg_arcs,
                ),
                split_peg_profile(
                    peg_profile(c, PEG_W_MID, PEG_R_MID),
                    c,
                    &peg_splits,
                    &peg_arcs,
                ),
                split_peg_profile(
                    peg_profile(c, PEG_W_TOP, OUTER_R),
                    c,
                    &peg_splits,
                    &peg_arcs,
                ),
            )
        })
        .collect();
    let outer_rings: Vec<(Vec<Seg>, Vec<bool>)> = o
        .loops
        .iter()
        .map(|pieces| {
            (
                pieces.iter().map(|p| p.seg).collect(),
                pieces.iter().map(|p| p.shared).collect(),
            )
        })
        .collect();
    let full_hi_rings: Vec<Vec<Seg>> = if openish {
        Vec::new()
    } else {
        outer_rings.iter().map(|(s, _)| s.clone()).collect()
    };

    let mut cav_ops: Vec<(String, POp)> = Vec::new();
    let mut fillet_edges: Vec<(Seg, f32, f32)> = Vec::new();
    let mut rim_holes: Vec<Vec<Seg>> = Vec::new();
    let mut island_tops: Vec<Vec<Seg>> = Vec::new();

    for (ci, (cl, island_shapes, loop_fr, banded)) in planned.into_iter().enumerate() {
        if cl.touched() {
            for isl in &island_shapes {
                island_tops.push(isl.segs.clone());
            }
            let floor_holes: Vec<POpDirLoop> = island_shapes
                .iter()
                .map(|i| (i.segs.clone(), false))
                .collect();
            cav_ops.push((
                format!("cavity {ci} (open): floor"),
                POp::PlanarFace {
                    plane: PPlaneRef::Z {
                        z: floor_z,
                        up: true,
                    },
                    outer: (cl.segs.clone(), true),
                    holes: floor_holes,
                },
            ));
            for (ii, isl) in island_shapes.iter().enumerate() {
                cav_ops.push((
                    format!("cavity {ci} (open): tower {ii}"),
                    POp::WallFaces {
                        lower: isl.segs.clone(),
                        upper: isl.segs.clone(),
                        z0: floor_z,
                        z1: total_h,
                        outward: true,
                    },
                ));
            }
            // An opening deletes the wall over its own run, not the rest of
            // the compartment's. The segments the outer walk replaced are the
            // ones with no wall left to roll against; every other floor-wall
            // edge still gets its fillet, and the chain runs out on the mouth.
            if loop_fr > MIN_USEFUL_BLEND {
                let walled: Vec<bool> = cl.coincident.iter().map(|&c| !c).collect();
                for (s, keep) in cl.segs.iter().zip(blendable_segs(&cl.segs, &walled)) {
                    if keep {
                        fillet_edges.push((*s, floor_z, loop_fr));
                    }
                }
            }
            for isl in &island_shapes {
                if isl.fr > MIN_USEFUL_BLEND {
                    fillet_edges.extend(isl.segs.iter().map(|s| (*s, floor_z, isl.fr)));
                }
            }
            continue;
        }

        if let Some(bd) = banded {
            let (stack, opts, tops, rim, blends) =
                plan_cavity_banded(&bd, &island_shapes, floor_z, total_h);
            island_tops.extend(tops);
            rim_holes.extend(rim);
            fillet_edges.extend(blends);
            cav_ops.push((
                format!("cavity {ci}: banded slab stack"),
                POp::Slabs { stack, opts },
            ));
            continue;
        }

        match slope {
            Some(sl) => {
                for isl in &island_shapes {
                    if isl.top.is_none() {
                        island_tops.push(isl.segs.clone());
                    }
                }
                let (ux, uy) = uphill_unit(sl.dir);
                let (min_a, span) = slope_span(bin_cells, ux, uy);
                let m = sl.angle_deg.to_radians().tan().clamp(0.0, 3.0);
                let cavity_depth = total_h - floor_z;
                let h_max = (m * span).min(cavity_depth - SLOPE_RIM_HEADROOM).max(0.0);
                let eff_m = if span > MIN_SLOPE_SPAN {
                    h_max / span
                } else {
                    0.0
                };
                let z_of = |pt: Vec2| floor_z + eff_m * (ux * pt.x + uy * pt.y - min_a);
                let origin = Vec3::new(
                    cl.segs[0].start().x,
                    cl.segs[0].start().y,
                    z_of(cl.segs[0].start()),
                );
                let normal = Vec3::new(-eff_m * ux, -eff_m * uy, 1.0).normalize();
                let floor_plane = PPlaneRef::Tilted { origin, normal };
                let top_plane = PPlaneRef::Z {
                    z: total_h,
                    up: true,
                };

                cav_ops.push((
                    format!("cavity {ci}: sloped walls"),
                    POp::SlopedWall {
                        lower: cl.segs.clone(),
                        upper: cl.segs.clone(),
                        lower_plane: floor_plane.clone(),
                        upper_plane: top_plane.clone(),
                        outward: false,
                    },
                ));
                let mut floor_holes: Vec<POpDirLoop> = Vec::new();
                for (ii, isl) in island_shapes.iter().enumerate() {
                    let slope_max = isl
                        .segs
                        .iter()
                        .map(|s| z_of(s.start()))
                        .fold(floor_z, f32::max);
                    let t = isl
                        .top
                        .filter(|&t| t > slope_max + SLOPE_ISLAND_HEADROOM)
                        .unwrap_or(total_h);
                    let island_top_plane = PPlaneRef::Z { z: t, up: true };
                    cav_ops.push((
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
                        cav_ops.push((
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
                cav_ops.push((
                    format!("cavity {ci}: sloped floor"),
                    POp::PlanarFace {
                        plane: floor_plane,
                        outer: (cl.segs.clone(), false),
                        holes: floor_holes,
                    },
                ));
                rim_holes.push(cl.segs.clone());
            }
            None => {
                let (stack, opts, tops, rim, blends) =
                    plan_cavity_flat(&cl.segs, &island_shapes, floor_z, total_h, loop_fr);
                island_tops.extend(tops);
                rim_holes.extend(rim);
                fillet_edges.extend(blends);
                cav_ops.push((
                    format!("cavity {ci}: slab stack"),
                    POp::Slabs { stack, opts },
                ));
            }
        }
    }

    let sector_segs: Vec<Vec<Seg>> = wall_loops;
    let top_walls: Vec<Vec<Seg>> = if openish {
        sector_segs.clone()
    } else {
        full_hi_rings.clone()
    };

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
    for (ci, (c, s_bot, s_mid, s_top)) in peg_profiles.iter().enumerate() {
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
        let ccx = (c.x as f32 + 0.5) * GRID_PITCH;
        let ccy = (c.y as f32 + 0.5) * GRID_PITCH;
        if let Some(profile) = &fastener_profile {
            for (dx, dy) in [(-1.0, -1.0), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0)] {
                let hx = ccx + dx * FASTENER_INSET;
                let hy = ccy + dy * FASTENER_INSET;
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
            for (dx, dy) in [(-1.0, -1.0), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0)] {
                let hx = ccx + dx * FASTENER_INSET;
                let hy = ccy + dy * FASTENER_INSET;
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

    drop(_g);
    let _g = crate::kernel::perf::scope(crate::kernel::perf::Metric::PlanStitch);
    for (li, (segs, _)) in outer_rings.iter().enumerate() {
        let z1 = if openish { floor_z } else { total_h };
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

    {
        let mut free: Vec<Seg> = Vec::new();
        for (c, _, _, s_top) in &peg_profiles {
            for seg in s_top {
                if peg_seg_free(seg, *c, &shared) {
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
        let stitched = stitch_loops_2d(free);
        for (i, (outer, holes)) in stitched.into_iter().enumerate() {
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
    }

    for (label, op) in cav_ops {
        prog.push(format!("{tag}: {label}"), op);
    }

    // `wall_between` takes the material side from the loop's own direction --
    // the surface normal it builds points to the right of travel -- so a region
    // wound material-on-the-left needs `outward: true` for every loop it has,
    // outer and hole alike. `region_difference` returns exactly that region, and
    // this is what says so: the signed areas sum to the material's own area, so
    // every hole is wound against its outer loop and no loop is wound twice.
    //
    // The rule this replaced, `outward: loop_area(sl) > 0.0`, flipped the normal
    // on every hole. It agreed with the base's outer wall below `floor_z` only
    // while the two never shared a ring -- an enclosed hole is where they do,
    // and there the wall and the base met at `floor_z` with opposing normals.
    let sector_area: f32 = sector_segs.iter().map(|l| loop_area(l)).sum();
    assert!(
        sector_segs.is_empty() || sector_area > 0.0,
        "{tag}: the standing wall's {} loop(s) enclose {sector_area} mm^2, so they are not one \
         region wound material-on-the-left",
        sector_segs.len()
    );
    for (si, sl) in sector_segs.iter().enumerate() {
        assert!(
            loop_area(sl) != 0.0,
            "{tag}: wall sector {si} encloses no area"
        );
        prog.push(
            format!("{tag}: wall sector {si}"),
            POp::Wall {
                lower: sl.clone(),
                upper: sl.clone(),
                z0: floor_z,
                z1: total_h,
                outward: true,
            },
        );
    }
    {
        let (tw, it, rh) = (&top_walls, &island_tops, &rim_holes);
        let (tw, it, rh) = (tw.as_slice(), it.as_slice(), rh.as_slice());
        let mut outers: Vec<(Vec<Seg>, Vec<Vec<Seg>>)> = Vec::new();
        for segs in tw {
            if loop_area(segs) > 0.0 {
                outers.push((segs.clone(), Vec::new()));
            }
        }
        for segs in it {
            outers.push((segs.clone(), Vec::new()));
        }
        let holes: Vec<Vec<Seg>> = tw
            .iter()
            .filter(|s| loop_area(s) < 0.0)
            .cloned()
            .chain(rh.iter().cloned())
            .collect();
        for hole in &holes {
            let pt = hole[0].start();
            let mut best: Option<usize> = None;
            for (i, (outer, _)) in outers.iter().enumerate() {
                let a = loop_area(outer).abs();
                if point_in_segs(pt, outer)
                    && best.is_none_or(|bi| a < loop_area(&outers[bi].0).abs())
                {
                    best = Some(i);
                }
            }
            let bi = best.expect("total_h hole without a containing face");
            outers[bi].1.push(hole.clone());
        }
        for (i, (outer, holes)) in outers.into_iter().enumerate() {
            let label = if it.is_empty() && tw.len() == 1 {
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

    if !fillet_edges.is_empty() {
        prog.push(
            format!("{tag}: floor fillet"),
            POp::Fillet {
                edges: fillet_edges,
            },
        );
    }
}
