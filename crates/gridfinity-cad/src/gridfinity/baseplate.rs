//! The baseplate: the other thing `Params` can ask for.
//!
//! Full `42 x n` rather than the bin's `42 n - 0.5`, with a peg-shaped through
//! socket per cell so a bin drops into it, and a stepped counterbore where a
//! magnet and a screw are both asked for. It is built on the rectilinear region
//! engine rather than the boundary walk, because a baseplate has no cavity, no
//! walls and no compartments -- only an outline and a socket per cell.

use super::*;
use crate::kernel::build::{loop_of, ring, wall_between};
use crate::kernel::math::Vec2;
use crate::kernel::rectregion::{LoopStyle, RectF, shape_loop, trace_rects};
use crate::kernel::sketch::{loop_area, reverse_loop};
use crate::kernel::topo::{Builder, Loop, Solid};

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
                    c.x as f32 * GRID_PITCH,
                    c.y as f32 * GRID_PITCH,
                    GRID_PITCH,
                    GRID_PITCH,
                )
            })
            .collect::<Vec<_>>(),
        &[],
    );
    let inset = |_: usize, _: Vec2, _: Vec2| 0.0f32;
    let radius = |_: usize, convex: bool| if convex { OUTER_R } else { 0.0 };
    let mut outer_top: Vec<Loop> = Vec::new();
    let mut outer_bot: Vec<Loop> = Vec::new();
    let mut first_outer_top: Option<Loop> = None;
    let mut first_outer_bot: Option<Loop> = None;
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
        if lp.is_hole() {
            outer_top.push(loop_of(&r_top, true));
            outer_bot.push(loop_of(&r_bot, false));
        } else if first_outer_top.is_none() {
            first_outer_top = Some(loop_of(&r_top, true));
            first_outer_bot = Some(loop_of(&r_bot, false));
        }
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
        outer_top.push(loop_of(&r3, true));
        outer_bot.push(loop_of(&r0, false));
    }

    if let Some(t) = first_outer_top {
        planar(&mut b, PEG_HEIGHT, true, t, outer_top);
    }
    if let Some(bt) = first_outer_bot {
        planar(&mut b, 0.0, false, bt, outer_bot);
    }
    b.build()
}
