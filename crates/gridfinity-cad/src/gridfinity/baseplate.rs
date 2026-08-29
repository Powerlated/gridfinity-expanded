//! The baseplate: the other thing `Params` can ask for.
//!
//! Full `42 x n` rather than the bin's `42 n - 0.5`, with a peg-shaped through
//! socket per cell so a bin drops into it. It is built on the rectilinear region
//! engine rather than the boundary walk, because a baseplate has no cavity, no
//! walls and no compartments -- only an outline and a socket per cell. It reads
//! no fastener setting: `magnet_holes` and `screw_holes` are the bin's, and a
//! counterbore under a socket is unbuilt.
//!
//! A cell set may fall into several disjoint islands, and each island is its own
//! plate: `island_of` pairs every traced hole and every cell's socket ring with
//! the outer loop that contains it, so the top and bottom caps are emitted once
//! per island carrying only the holes that are really theirs.

use super::*;
use crate::kernel::build::{loop_of, ring, wall_between};
use crate::kernel::math::Vec2;
use crate::kernel::nesting::containment;
use crate::kernel::rectregion::{LoopStyle, RectF, shape_loop, trace_rects};
use crate::kernel::sketch::{Aabb, Seg, loop_area, point_in_segs, reverse_loop, segs_bbox};
use crate::kernel::topo::{Builder, Loop, Solid};

/// One connected plate: the outer loop's top and bottom rings, and the loops
/// that are holes of *that* plate -- its traced interior holes and the socket
/// ring of every cell standing inside it.
struct Island {
    outer_top: Loop,
    outer_bot: Loop,
    holes_top: Vec<Loop>,
    holes_bot: Vec<Loop>,
}

/// The index into `outers` of the innermost outline island containing `point`,
/// or `None` when the point lies in none of them.
///
/// `outers` are the traced region's non-hole loops, which are disjoint except
/// for nesting, so "innermost" is the containing loop with the most containers
/// of its own and the answer is unique.
fn island_of(point: Vec2, outers: &[usize], loops: &[Vec<Seg>], containers: &[Vec<usize>]) -> Option<usize> {
    outers
        .iter()
        .enumerate()
        .filter(|&(_, &l)| point_in_segs(point, &loops[l]))
        .max_by_key(|&(_, &l)| containers[l].len())
        .map(|(i, _)| i)
}

pub(super) fn build_baseplate(p: &Params) -> Solid {
    let cells = p.all_cells();
    if cells.is_empty() {
        return Builder::new().build();
    }
    let mut b = Builder::new();

    let traced = trace_rects(
        &cells
            .iter()
            .map(|c| {
                RectF::new(
                    c.x as f64 * GRID_PITCH,
                    c.y as f64 * GRID_PITCH,
                    GRID_PITCH,
                    GRID_PITCH,
                )
            })
            .collect::<Vec<_>>(),
        &[],
    );
    let inset = |_: usize, _: Vec2, _: Vec2| 0.0f64;
    let radius = |_: usize, convex: bool| if convex { OUTER_R } else { 0.0 };
    let mut outline: Vec<Vec<Seg>> = Vec::with_capacity(traced.len());
    let mut rings: Vec<(Loop, Loop)> = Vec::with_capacity(traced.len());
    for lp in &traced {
        let segs = {
            let s = shape_loop(
                lp,
                &LoopStyle {
                    inset: &inset,
                    radius: &radius,
                },
            );
            if loop_area(&s) < 0.0 && !lp.is_hole() {
                reverse_loop(&s)
            } else {
                s
            }
        };
        let r_bot = ring(&mut b, &segs, 0.0);
        let r_top = ring(&mut b, &segs, PEG_HEIGHT);
        wall_between(&mut b, &segs, &segs, &r_bot, &r_top, 0.0, PEG_HEIGHT, true);
        rings.push((loop_of(&r_top, true), loop_of(&r_bot, false)));
        outline.push(segs);
    }

    let bbox: Vec<Aabb> = outline.iter().map(|l| segs_bbox(l)).collect();
    let containers = containment(&outline, &bbox);
    let outers: Vec<usize> = (0..traced.len()).filter(|&i| !traced[i].is_hole()).collect();
    assert!(
        !outers.is_empty(),
        "a baseplate of {} cell(s) traced {} loop(s), every one of them a hole, so it bounds \
         no material",
        cells.len(),
        traced.len()
    );
    let mut islands: Vec<Island> = outers
        .iter()
        .map(|&o| Island {
            outer_top: rings[o].0.clone(),
            outer_bot: rings[o].1.clone(),
            holes_top: Vec::new(),
            holes_bot: Vec::new(),
        })
        .collect();

    for h in (0..traced.len()).filter(|&i| traced[i].is_hole()) {
        let owner = island_of(outline[h][0].start(), &outers, &outline, &containers)
            .unwrap_or_else(|| {
                panic!(
                    "the baseplate's traced hole {h} lies inside none of its {} island(s), so \
                     there is no cap it is a hole of",
                    outers.len()
                )
            });
        islands[owner].holes_top.push(rings[h].0.clone());
        islands[owner].holes_bot.push(rings[h].1.clone());
    }

    for c in &cells {
        let s_bot = peg_profile(*c, PEG_W_BOTTOM, PEG_R_BOTTOM);
        let s_mid = peg_profile(*c, PEG_W_MID, PEG_R_MID);
        let s_top = peg_profile(*c, PEG_W_TOP, OUTER_R);
        let r0 = ring(&mut b, &s_bot, 0.0);
        let r1 = ring(&mut b, &s_mid, PEG_Z1);
        let r2 = ring(&mut b, &s_mid, PEG_Z2);
        let r3 = ring(&mut b, &s_top, PEG_HEIGHT);
        wall_between(&mut b, &s_bot, &s_mid, &r0, &r1, 0.0, PEG_Z1, false);
        wall_between(&mut b, &s_mid, &s_mid, &r1, &r2, PEG_Z1, PEG_Z2, false);
        wall_between(&mut b, &s_mid, &s_top, &r2, &r3, PEG_Z2, PEG_HEIGHT, false);
        let centre = Vec2::new(
            (c.x as f64 + 0.5) * GRID_PITCH,
            (c.y as f64 + 0.5) * GRID_PITCH,
        );
        let owner = island_of(centre, &outers, &outline, &containers).unwrap_or_else(|| {
            panic!(
                "cell ({}, {}) of the baseplate stands inside none of its {} island(s), but its \
                 own rectangle is what they were traced from",
                c.x,
                c.y,
                outers.len()
            )
        });
        islands[owner].holes_top.push(loop_of(&r3, true));
        islands[owner].holes_bot.push(loop_of(&r0, false));
    }

    for island in islands {
        assert_eq!(
            island.holes_top.len(),
            island.holes_bot.len(),
            "a baseplate island's two caps are holed by the same rings, so they must count the \
             same, not {} against {}",
            island.holes_top.len(),
            island.holes_bot.len()
        );
        planar(&mut b, PEG_HEIGHT, true, island.outer_top, island.holes_top);
        planar(&mut b, 0.0, false, island.outer_bot, island.holes_bot);
    }
    b.build()
}
