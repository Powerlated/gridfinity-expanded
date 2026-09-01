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
use gridfinity_brep::build::{loop_of, ring, wall_between};
use gridfinity_brep::math::Vec2;
use gridfinity_brep::nesting::containment;
use gridfinity_brep::rectregion::{LoopStyle, RectF, shape_loop, trace_rects};
use gridfinity_brep::sketch::{Aabb, Seg, loop_area, point_in_segs, reverse_loop, segs_bbox};
use gridfinity_brep::topo::{Builder, Loop, Solid};
use crate::layout::{GridCell, GridFootprint};

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

/// How far one cell's plate rectangle reaches past its own pitch square on each
/// of its four sides, as `(west, east, south, north)` millimetres: half the
/// plate margin wherever the cell sits on a side of `grid`'s bounding rectangle,
/// and nothing anywhere else.
///
/// The margins are *totals* per axis, split evenly between the two sides, so the
/// cell grid sits centred in the plate. A side facing into the grid's interior
/// -- a notch, an enclosed hole -- never grows, which is what keeps two pieces
/// of one plate from claiming the same empty cell. `grid` is the whole plate's
/// cell set even when `cell` belongs to one carved piece of it, so every piece
/// grows on the sides the finished plate grows on and on no others.
pub(super) fn plate_cell_overhang(
    cell: GridCell,
    grid: GridFootprint,
    margin_x: f64,
    margin_y: f64,
) -> (f64, f64, f64, f64) {
    assert!(
        margin_x >= 0.0 && margin_y >= 0.0,
        "a plate margin is how much wider than its grid the plate is, so it is not \
         {margin_x} x {margin_y} mm"
    );
    let (hx, hy) = (margin_x / 2.0, margin_y / 2.0);
    let (max_x, max_y) = (
        grid.min_x + grid.width_cells - 1,
        grid.min_y + grid.depth_cells - 1,
    );
    (
        if cell.x == grid.min_x { hx } else { 0.0 },
        if cell.x == max_x { hx } else { 0.0 },
        if cell.y == grid.min_y { hy } else { 0.0 },
        if cell.y == max_y { hy } else { 0.0 },
    )
}

/// One cell's plate rectangle: its pitch square grown by `plate_cell_overhang`.
pub(super) fn plate_cell_rect(
    cell: GridCell,
    grid: GridFootprint,
    pitch: f64,
    margin_x: f64,
    margin_y: f64,
) -> RectF {
    let (west, east, south, north) = plate_cell_overhang(cell, grid, margin_x, margin_y);
    let rect = RectF::new(
        cell.x as f64 * pitch - west,
        cell.y as f64 * pitch - south,
        pitch + west + east,
        pitch + south + north,
    );
    assert!(
        rect.x <= cell.x as f64 * pitch && rect.w >= pitch && rect.h >= pitch,
        "a plate rectangle grows a cell's pitch square outward, but cell ({}, {}) came back as \
         {rect:?}",
        cell.x,
        cell.y
    );
    rect
}

pub(super) fn build_baseplate(p: &Params) -> Solid {
    let cells = p.all_cells();
    if cells.is_empty() {
        return Builder::new().build();
    }
    let mut b = Builder::new();

    let grid = GridFootprint::from_cells(&cells)
        .expect("a plate of at least one cell has a bounding rectangle");
    let traced = trace_rects(
        &cells
            .iter()
            .map(|c| plate_cell_rect(*c, grid, p.pitch, p.plate_margin_x, p.plate_margin_y))
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

    let (w_bot, w_mid, w_top) = peg_widths(p.pitch);
    for c in &cells {
        let s_bot = peg_profile(*c, p.pitch, w_bot, PEG_R_BOTTOM);
        let s_mid = peg_profile(*c, p.pitch, w_mid, PEG_R_MID);
        let s_top = peg_profile(*c, p.pitch, w_top, OUTER_R);
        let r0 = ring(&mut b, &s_bot, 0.0);
        let r1 = ring(&mut b, &s_mid, PEG_Z1);
        let r2 = ring(&mut b, &s_mid, PEG_Z2);
        let r3 = ring(&mut b, &s_top, PEG_HEIGHT);
        wall_between(&mut b, &s_bot, &s_mid, &r0, &r1, 0.0, PEG_Z1, false);
        wall_between(&mut b, &s_mid, &s_mid, &r1, &r2, PEG_Z1, PEG_Z2, false);
        wall_between(&mut b, &s_mid, &s_top, &r2, &r3, PEG_Z2, PEG_HEIGHT, false);
        let centre = Vec2::new(
            (c.x as f64 + 0.5) * p.pitch,
            (c.y as f64 + 0.5) * p.pitch,
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
