//! The connector pegs under the bin, and welding them to the wall above.
//!
//! One peg per cell, lofted through three profiles between z=0 and `PEG_HEIGHT`,
//! with `PEG_R_MID` chosen so all three corner arcs share an axis and the
//! chamfers come out as coaxial cones. `peg_profile` draws one ring;
//! `peg_seg_free` says which of its segments face open air rather than the
//! neighbouring cell's peg, and are therefore the ones the bridge underside has
//! to close.
//!
//! `split_peg_profile` is the welding: it cuts each ring wherever the outline
//! above was cut, by absolute coordinate on a straight run and by angle about
//! the corner's own centre on an arc. The angle works because the three rings'
//! corner arcs are coaxial, so one angle names the same place on each, and a
//! point is matched to its corner by distance from that centre, since two
//! corners of one ring can otherwise claim the same angle.

use super::*;
use gridfinity_brep::math::{Vec2, wrap_angle_into};
use gridfinity_brep::sketch::{COINCIDENT, Seg, Sketch, ccw_segs};
use crate::layout::{GridCell, GridEdge, Orientation, cell_edges};
use std::collections::HashMap;

pub(super) fn peg_profile(c: GridCell, pitch: f64, w: f64, r: f64) -> Vec<Seg> {
    let cx = (c.x as f64 + 0.5) * pitch;
    let cy = (c.y as f64 + 0.5) * pitch;
    ccw_segs(&Sketch::rounded_rect(cx, cy, w, w, r))
}

pub(super) fn peg_seg_free(s: &Seg, c: GridCell, pitch: f64, shared: &SharedWithPegs) -> bool {
    let cx = (c.x as f64 + 0.5) * pitch;
    let cy = (c.y as f64 + 0.5) * pitch;
    match *s {
        Seg::Line { a, b } => {
            let m = (a + b) * 0.5;
            let horiz = (a.y - b.y).abs() < COINCIDENT;
            let e = if horiz {
                let y = if m.y < cy { c.y } else { c.y + 1 };
                GridEdge {
                    x: c.x,
                    y,
                    orientation: Orientation::H,
                }
            } else {
                let x = if m.x < cx { c.x } else { c.x + 1 };
                GridEdge {
                    x,
                    y: c.y,
                    orientation: Orientation::V,
                }
            };
            !shared.sides.contains(&e)
        }
        Seg::Arc { center, .. } => {
            let lx = if center.x > cx { c.x + 1 } else { c.x };
            let ly = if center.y > cy { c.y + 1 } else { c.y };
            !shared.corners.contains(&(lx, ly))
        }
    }
}

/// Split a peg ring so its edges weld to whatever the outer profile was cut at.
///
/// A straight run is cut by *station* -- an absolute coordinate, which lands on
/// every ring because they are concentric. A corner is cut by **angle** about
/// the corner's centre, which works for the same reason and is what the three
/// rings sharing one corner axis buys: `PEG_R_MID` is chosen so all three
/// corner arcs are coaxial, so one angle names the same place on each. A point
/// is matched to its corner by its distance from that centre, since two corners
/// of the same ring can otherwise claim the same angle.
pub(super) fn split_peg_profile(
    segs: Vec<Seg>,
    c: GridCell,
    pitch: f64,
    splits: &HashMap<GridEdge, Vec<f64>>,
    arc_points: &[Vec2],
) -> Vec<Seg> {
    let [west, east, south, north] = cell_edges(c);
    let cx = (c.x as f64 + 0.5) * pitch;
    let cy = (c.y as f64 + 0.5) * pitch;
    let mut out = Vec::with_capacity(segs.len());
    for s in segs {
        let Seg::Line { a, b } = s else {
            if let Seg::Arc {
                a,
                b,
                center,
                radius,
                a0,
                a1,
            } = s
            {
                let (lo, hi) = (a0.min(a1), a0.max(a1));
                let mut cuts: Vec<f64> = arc_points
                    .iter()
                    .filter(|p| ((**p - center).length() - OUTER_R).abs() < COINCIDENT)
                    .map(|p| {
                        wrap_angle_into(
                            (p.y - center.y).atan2(p.x - center.x),
                            lo,
                            hi,
                            ARC_ENDPOINT_ANGLE,
                        )
                    })
                    .filter(|t| *t > lo + ARC_ENDPOINT_ANGLE && *t < hi - ARC_ENDPOINT_ANGLE)
                    .collect();
                if cuts.is_empty() {
                    out.push(s);
                    continue;
                }
                if a1 >= a0 {
                    cuts.sort_by(f64::total_cmp);
                } else {
                    cuts.sort_by(|x, y| y.total_cmp(x));
                }
                cuts.dedup_by(|x, y| (*x - *y).abs() < ARC_ENDPOINT_ANGLE);
                let at = |t: f64| center + Vec2::new(t.cos(), t.sin()) * radius;
                let (mut prev_p, mut prev_t) = (a, a0);
                for t in cuts {
                    out.push(Seg::Arc {
                        a: prev_p,
                        b: at(t),
                        center,
                        radius,
                        a0: prev_t,
                        a1: t,
                    });
                    prev_p = at(t);
                    prev_t = t;
                }
                out.push(Seg::Arc {
                    a: prev_p,
                    b,
                    center,
                    radius,
                    a0: prev_t,
                    a1,
                });
                continue;
            }
            out.push(s);
            continue;
        };
        let horiz = (a.y - b.y).abs() < COINCIDENT;
        let e = if horiz {
            if a.y < cy { south } else { north }
        } else if a.x < cx {
            west
        } else {
            east
        };
        let Some(stations) = splits.get(&e) else {
            out.push(s);
            continue;
        };
        let coord = |p: Vec2| if horiz { p.x } else { p.y };
        let (c0, c1) = (coord(a), coord(b));
        let mut cuts: Vec<f64> = stations
            .iter()
            .copied()
            .filter(|&t| (t - c0.min(c1)) > COINCIDENT && (c0.max(c1) - t) > COINCIDENT)
            .collect();
        cuts.sort_by(|x, y| {
            if c1 > c0 {
                x.total_cmp(y)
            } else {
                y.total_cmp(x)
            }
        });
        cuts.dedup_by(|x, y| (*x - *y).abs() < COINCIDENT);
        let mut prev = a;
        for t in cuts {
            let p = if horiz {
                Vec2::new(t, a.y)
            } else {
                Vec2::new(a.x, t)
            };
            out.push(Seg::Line { a: prev, b: p });
            prev = p;
        }
        out.push(Seg::Line { a: prev, b });
    }
    out
}
