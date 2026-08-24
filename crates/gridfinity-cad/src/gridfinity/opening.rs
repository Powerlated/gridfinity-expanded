//! What an open edge does to a compartment's cavity.
//!
//! An opening is an inset of *nothing*, so the authored cavity already runs out
//! past the bin's outline to the pitch line, and the whole of what an opening
//! means is one boolean: `clip_cavity_to_outline` intersects the cavity with the
//! outline, keeping the compartment's own wall where a wall stands and following
//! the outer profile where none does, rounded corners and all. `CavityLoop`
//! carries the result with a flag per segment saying whether that run ended up
//! lying *on* the outline -- those runs bound no material, and they are exactly
//! what the standing wall's boundary leaves out and what the blend request must
//! skip. `OpenSpan` is the cheaper question the corner rounding asks: whether a
//! given point of the profile falls in an opened run at all.

use super::*;
use crate::kernel::math::Vec2;
use crate::kernel::region2d::region_intersection;
use crate::kernel::round::{drop_degenerate, seg_mid};
use crate::kernel::sketch::{COINCIDENT, Seg, loop_area};
use crate::layout::{EffectiveWalls, GridCell, Orientation};
use std::collections::HashSet;

#[derive(Clone, Copy, Debug)]
pub(super) struct OpenSpan {
    pub(super) horiz: bool,
    pub(super) coord: f32,
    pub(super) lo: f32,
    pub(super) hi: f32,
}

pub(super) fn open_spans(cells: &[GridCell], walls: &EffectiveWalls) -> Vec<OpenSpan> {
    let set: HashSet<GridCell> = cells.iter().copied().collect();
    walls
        .open
        .iter()
        .filter(|e| edge_inside_cell(&set, e).is_some())
        .map(|e| {
            let p = GRID_PITCH;
            match e.orientation {
                Orientation::H => OpenSpan {
                    horiz: true,
                    coord: e.y as f32 * p,
                    lo: e.x as f32 * p,
                    hi: (e.x + 1) as f32 * p,
                },
                Orientation::V => OpenSpan {
                    horiz: false,
                    coord: e.x as f32 * p,
                    lo: e.y as f32 * p,
                    hi: (e.y + 1) as f32 * p,
                },
            }
        })
        .collect()
}

pub(super) fn point_on_spans(spans: &[OpenSpan], pt: Vec2) -> bool {
    spans.iter().any(|s| {
        let (c, a) = if s.horiz { (pt.y, pt.x) } else { (pt.x, pt.y) };
        (c - s.coord).abs() < COINCIDENT && a > s.lo - COINCIDENT && a < s.hi + COINCIDENT
    })
}

pub(super) struct CavityLoop {
    pub(super) segs: Vec<Seg>,
    pub(super) coincident: Vec<bool>,
}

impl CavityLoop {
    pub(super) fn untouched(segs: Vec<Seg>) -> CavityLoop {
        let n = segs.len();
        CavityLoop {
            segs,
            coincident: vec![false; n],
        }
    }
    pub(super) fn touched(&self) -> bool {
        self.coincident.iter().any(|&c| c)
    }
}

/// Clip a cavity loop to the bin's outline, marking the runs that end up lying
/// *on* it -- those are the spans where no wall stands.
///
/// `plan_cavity` subtracts a wall strip for every walled edge and none for an
/// open one, so an opened cavity already runs out past the outline to the pitch
/// line. Intersecting it with the outline is therefore the whole of what an
/// opening means: the cavity keeps its own wall where one stands and follows the
/// outer profile where one does not, rounded corners and all.
///
/// This replaces a ray-cast pinch that walked the outline piece by piece. That
/// needed the cavity wall either side of a run to be straight and to meet the
/// outline within reach; at a reentrant corner it is the concave fillet arc and
/// there is nothing to cast along, and two openings meeting at a notch produced
/// outline walks that did not compose. The boolean has no such cases -- it is
/// the same computation the cavity is traced by.
pub(super) fn clip_cavity_to_outline(shape: &[Seg], outline: &[Vec<Seg>]) -> Vec<CavityLoop> {
    drop_degenerate(region_intersection(&[shape.to_vec()], outline))
        .into_iter()
        .filter(|l| loop_area(l) > 0.0)
        .map(|segs| {
            let coincident = segs
                .iter()
                .map(|sg| on_outline(outline, seg_mid(sg)))
                .collect();
            CavityLoop { segs, coincident }
        })
        .collect()
}
