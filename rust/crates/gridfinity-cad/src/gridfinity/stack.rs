//! A compartment's cavity expressed as a slab stack for the kernel to build.
//!
//! `plan_cavity_flat` is the ordinary case: the compartment void from the floor
//! to the rim, minus one slab per island tower standing in it.
//! `plan_cavity_banded` is the same plus one slab per partial-height inner wall,
//! floor to that wall's top, which is what lets a wall reaching the boundary,
//! one wholly inside it and one crossing it all be the same difference -- the
//! band machinery caps each where its slab ends. Both return the stack, the
//! island tops and rim holes the caller must close itself, and the blend
//! requests along each notch's contact runs. `slope_span` and `uphill_unit` are
//! the sloped floor's arithmetic, kept here because a slope is the one thing a
//! stack cannot express and the planner has to hand back instead.

use super::*;
use crate::kernel::fillet::feasible::blend_radius_along;
use crate::kernel::sketch::{Seg, loop_area};
use crate::kernel::slab::{Op as SlabOp, Slab, SlabOpts, plan_bands};
use crate::layout::GridCell;

pub(super) fn plan_cavity_flat(
    shape: &[Seg],
    islands: &[Island],
    floor_z: f32,
    total_h: f32,
    loop_fr: f32,
) -> (
    Vec<(SlabOp, Slab)>,
    SlabOpts,
    Vec<Vec<Seg>>,
    Vec<Vec<Seg>>,
    Vec<(Seg, f32, f32)>,
) {
    let mut stack = vec![(
        SlabOp::Union,
        Slab::new(vec![shape.to_vec()], floor_z, total_h),
    )];
    for isl in islands {
        stack.push((
            SlabOp::Difference,
            Slab::new(vec![isl.segs.clone()], floor_z, isl.top.unwrap_or(total_h)),
        ));
    }
    let mut blends: Vec<(Seg, f32, f32)> = Vec::new();
    if loop_fr > MIN_USEFUL_BLEND {
        blends.extend(shape.iter().map(|s| (*s, floor_z, loop_fr)));
    }
    for isl in islands {
        if isl.fr > MIN_USEFUL_BLEND {
            blends.extend(isl.segs.iter().map(|s| (*s, floor_z, isl.fr)));
        }
    }
    let top_band = plan_bands(&stack)
        .map(|(_, bands)| bands.last().cloned().unwrap_or_default())
        .unwrap_or_default();
    let tops: Vec<Vec<Seg>> = top_band
        .iter()
        .filter(|l| loop_area(l) < 0.0)
        .cloned()
        .collect();
    let rim: Vec<Vec<Seg>> = top_band
        .iter()
        .filter(|l| loop_area(l) > 0.0)
        .cloned()
        .collect();
    assert_eq!(
        tops.len() + rim.len(),
        top_band.len(),
        "a top-band loop has zero area, so it is neither void nor island"
    );
    (
        stack,
        SlabOpts {
            cavity: true,
            open_at: vec![total_h],
        },
        tops,
        rim,
        blends,
    )
}

pub(super) fn plan_cavity_banded(
    bd: &Banded,
    islands: &[Island],
    floor_z: f32,
    total_h: f32,
) -> (
    Vec<(SlabOp, Slab)>,
    SlabOpts,
    Vec<Vec<Seg>>,
    Vec<Vec<Seg>>,
    Vec<(Seg, f32, f32)>,
) {
    const TRANSITION_R: f32 = 4.0;

    let mut stack = vec![(
        SlabOp::Union,
        Slab::new(vec![bd.outline_b.clone()], floor_z, total_h),
    )];
    for n in &bd.notches {
        stack.push((
            SlabOp::Difference,
            Slab::new(vec![n.quad.clone()], floor_z, n.top),
        ));
    }
    for isl in islands {
        stack.push((
            SlabOp::Difference,
            Slab::new(vec![isl.segs.clone()], floor_z, isl.top.unwrap_or(total_h)),
        ));
    }
    let opts = SlabOpts {
        cavity: true,
        open_at: vec![total_h],
    };

    let top_band = plan_bands(&stack)
        .map(|(_, bands)| bands.last().cloned().unwrap_or_default())
        .unwrap_or_default();
    let rim: Vec<Vec<Seg>> = top_band
        .iter()
        .filter(|l| loop_area(l) > 0.0)
        .cloned()
        .collect();
    let tops: Vec<Vec<Seg>> = top_band
        .iter()
        .filter(|l| loop_area(l) < 0.0)
        .cloned()
        .collect();
    assert_eq!(
        tops.len() + rim.len(),
        top_band.len(),
        "a top-band loop has zero area, so it is neither void nor island"
    );

    let mut blends: Vec<(Seg, f32, f32)> = Vec::new();
    for n in &bd.notches {
        let want = (total_h - n.top).min(TRANSITION_R);
        // Per contact segment, not per notch: the run a ramp blends along can
        // mix straight pieces with the cavity's corner arcs, and only the arcs
        // constrain the radius.
        for s in &n.contact {
            let r = blend_radius_along(s, want);
            if r < MIN_ROUNDED_CORNER {
                continue;
            }
            blends.push((*s, n.top, r));
        }
    }

    (stack, opts, tops, rim, blends)
}

pub(super) fn slope_span(cells: &[GridCell], ux: f32, uy: f32) -> (f32, f32) {
    let mut min_a = f32::INFINITY;
    let mut max_a = f32::NEG_INFINITY;
    for c in cells {
        for (dx, dy) in [(0, 0), (1, 0), (0, 1), (1, 1)] {
            let x = (c.x + dx) as f32 * GRID_PITCH;
            let y = (c.y + dy) as f32 * GRID_PITCH;
            let a = ux * x + uy * y;
            min_a = min_a.min(a);
            max_a = max_a.max(a);
        }
    }
    (min_a, (max_a - min_a).max(1e-6))
}

pub(super) fn uphill_unit(dir: SlopeDir) -> (f32, f32) {
    match dir {
        SlopeDir::PlusX => (-1.0, 0.0),
        SlopeDir::MinusX => (1.0, 0.0),
        SlopeDir::PlusY => (0.0, -1.0),
        SlopeDir::MinusY => (0.0, 1.0),
    }
}
