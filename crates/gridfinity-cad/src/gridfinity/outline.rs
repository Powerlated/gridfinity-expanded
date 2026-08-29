//! The bin's outer profile, walked off its cells.
//!
//! `boundary_steps` turns a polyomino into one `Step` list per boundary loop,
//! travelling with material on the left; `author_outer_loop` turns each of those
//! into segments -- pitch lines inset `HALF_TOL`, corners struck at `OUTER_R`,
//! convex ones marked as shared with the corner cell's peg top and reentrant
//! ones not. `OuterLoops` holds the result and is where the profile gets cut
//! afterwards: `split_outline_at` puts a vertex wherever the wall above or the
//! peg below needs one, recording peg stations by coordinate on a straight run
//! and by angle on a corner arc, since a corner has no station.
//!
//! Everything here is about the *outside* of the bin. What the cavity does
//! inside it is `cavity`'s, and the two meet only through `outline_region`.

use super::*;
use crate::kernel::math::{Vec2, wrap_angle_into};
use crate::kernel::region2d::point_seg_distance;
use crate::kernel::round::{short_arc, v2_eq};
use crate::kernel::sketch::{COINCIDENT, Seg};
use crate::layout::{GridCell, GridEdge, Orientation};
use std::collections::{HashMap, HashSet};

#[derive(Clone, Copy, Debug)]
pub(super) struct Step {
    pub(super) from: (i32, i32),
    pub(super) to: (i32, i32),
    pub(super) edge: GridEdge,
}

impl Step {
    pub(super) fn dir(&self) -> (i32, i32) {
        (self.to.0 - self.from.0, self.to.1 - self.from.1)
    }
}

pub(super) fn boundary_steps(cells: &[GridCell]) -> Vec<Vec<Step>> {
    let set: HashSet<GridCell> = cells.iter().copied().collect();
    let present = |x: i32, y: i32| set.contains(&GridCell { x, y });
    let mut adj: HashMap<(i32, i32), Vec<Step>> = HashMap::new();
    for &c in cells {
        let (x, y) = (c.x, c.y);
        if !present(x, y - 1) {
            let s = Step {
                from: (x, y),
                to: (x + 1, y),
                edge: GridEdge {
                    x,
                    y,
                    orientation: Orientation::H,
                },
            };
            adj.entry(s.from).or_default().push(s);
        }
        if !present(x + 1, y) {
            let s = Step {
                from: (x + 1, y),
                to: (x + 1, y + 1),
                edge: GridEdge {
                    x: x + 1,
                    y,
                    orientation: Orientation::V,
                },
            };
            adj.entry(s.from).or_default().push(s);
        }
        if !present(x, y + 1) {
            let s = Step {
                from: (x + 1, y + 1),
                to: (x, y + 1),
                edge: GridEdge {
                    x,
                    y: y + 1,
                    orientation: Orientation::H,
                },
            };
            adj.entry(s.from).or_default().push(s);
        }
        if !present(x - 1, y) {
            let s = Step {
                from: (x, y + 1),
                to: (x, y),
                edge: GridEdge {
                    x,
                    y,
                    orientation: Orientation::V,
                },
            };
            adj.entry(s.from).or_default().push(s);
        }
    }

    let mut used: HashSet<((i32, i32), (i32, i32))> = HashSet::new();
    let mut starts: Vec<(i32, i32)> = adj.keys().copied().collect();
    starts.sort_unstable();
    let mut loops = Vec::new();
    for &start in &starts {
        loop {
            let Some(&first) = adj[&start].iter().find(|s| !used.contains(&(s.from, s.to))) else {
                break;
            };
            used.insert((first.from, first.to));
            let mut steps = vec![first];
            let mut cur = first;
            while cur.to != start {
                let din = cur.dir();
                let prefs = [(-din.1, din.0), din, (din.1, -din.0)];
                let mut next: Option<Step> = None;
                'outer: for d in prefs {
                    if let Some(cands) = adj.get(&cur.to) {
                        for &s in cands {
                            if s.dir() == d && !used.contains(&(s.from, s.to)) {
                                next = Some(s);
                                break 'outer;
                            }
                        }
                    }
                }
                let Some(nxt) = next else { break };
                used.insert((nxt.from, nxt.to));
                steps.push(nxt);
                cur = nxt;
            }
            loops.push(steps);
        }
    }
    loops
}

pub(super) fn mm(p: (i32, i32), pitch: f64) -> Vec2 {
    Vec2::new(p.0 as f64 * pitch, p.1 as f64 * pitch)
}

pub(super) fn left_of(d: (i32, i32)) -> Vec2 {
    Vec2::new(-d.1 as f64, d.0 as f64)
}

pub(super) fn dirv(d: (i32, i32)) -> Vec2 {
    Vec2::new(d.0 as f64, d.1 as f64)
}

#[derive(Clone, Copy)]
pub(super) struct OuterPiece {
    pub(super) seg: Seg,
    pub(super) shared: bool,
    pub(super) edge: Option<GridEdge>,
}

#[derive(Default, Clone)]
pub(super) struct SharedWithPegs {
    pub(super) sides: HashSet<GridEdge>,
    pub(super) corners: HashSet<(i32, i32)>,
    /// Lattice points some traversal left square. A **diagonal pinch** -- two
    /// cells meeting only at a corner, with both their shared neighbours absent
    /// -- puts the outline through the same point twice, and the two visits can
    /// disagree about rounding it when an opening runs into one of them. The
    /// peg looks corners up by lattice point, so it would see the rounded visit
    /// and weld its arc to an outline that squared the other one, leaving the
    /// arc paired with nothing. A corner counts as shared only if *every* visit
    /// rounded it.
    pub(super) squared: HashSet<(i32, i32)>,
}

pub(super) fn author_outer_loop(
    steps: &[Step],
    pitch: f64,
    inset: &dyn Fn(&GridEdge) -> f64,
    walled: &dyn Fn(&GridEdge) -> bool,
    shared: &mut SharedWithPegs,
) -> Vec<OuterPiece> {
    let n = steps.len();
    let mut pieces: Vec<OuterPiece> = Vec::new();
    for k in 0..n {
        let s = &steps[k];
        let s_next = &steps[(k + 1) % n];
        let d = dirv(s.dir());
        let nrm = left_of(s.dir());
        let ins = inset(&s.edge);
        let ins_next = inset(&s_next.edge);
        let from = mm(s.from, pitch);
        let to = mm(s.to, pitch);
        let is_std = (ins - HALF_TOL).abs() < INSET_SAME;

        let a = from + d * PEG_TANGENT + nrm * ins;
        let b = to - d * PEG_TANGENT + nrm * ins;
        pieces.push(OuterPiece {
            seg: Seg::Line { a, b },
            shared: is_std,
            edge: Some(s.edge),
        });
        if is_std {
            shared.sides.insert(s.edge);
        }

        let d1 = dirv(s_next.dir());
        let n1 = left_of(s_next.dir());
        let cross = d.x * d1.y - d.y * d1.x;
        let start = to - d * PEG_TANGENT + nrm * ins;
        let end = to + d1 * PEG_TANGENT + n1 * ins_next;
        let both_std = is_std && (ins_next - HALF_TOL).abs() < INSET_SAME;
        let same_side = walled(&s.edge) == walled(&s_next.edge);
        // Rounding a reentrant corner is a property of the *wall* that turns it,
        // so it takes a wall on both edges. The arc is struck about a centre
        // `OUTER_R` inside each inset line, which puts it `OUTER_R - ins` past
        // both **pitch** lines, out over a cell the bin does not occupy -- the
        // overhang `carve_to_cells` reaches for with `REENTRANT_FILLET_OVERHANG`.
        // That bulge is legitimate only as the outside of a wall.
        //
        // Open one of the two edges and `same_side` already squares the corner.
        // Open *both* and `same_side` held, so the corner was still rounded --
        // while the cavity, which subtracts no strip for an open edge, ran out
        // only as far as the pitch lines. `cavity & outline` then clipped away
        // every other scrap of wall and left the bulge standing alone: a
        // 2.154 mm curved triangle rising 19.8 mm from the floor, unattached
        // above it, in the middle of the doorway the two openings made.
        let corner_walled = walled(&s.edge) && walled(&s_next.edge);
        if cross.abs() < TURN_SIGN {
            pieces.push(OuterPiece {
                seg: Seg::Line { a: start, b: end },
                shared: false,
                edge: None,
            });
        } else if cross > 0.0 && both_std && same_side {
            let c = mm(s.to, pitch);
            let center = c + nrm * (ins + OUTER_R) + n1 * (ins_next + OUTER_R);
            let a0 = f64::atan2(start.y - center.y, start.x - center.x);
            let a1 = f64::atan2(end.y - center.y, end.x - center.x);
            let (a0, a1) = short_arc(a0, a1);
            pieces.push(OuterPiece {
                seg: Seg::Arc {
                    a: start,
                    b: end,
                    center,
                    radius: OUTER_R,
                    a0,
                    a1,
                },
                shared: true,
                edge: None,
            });
            shared.corners.insert(s.to);
        } else if cross < 0.0 && both_std && same_side && corner_walled {
            let q = mm(s.to, pitch) + nrm * ins + n1 * ins_next;
            let center = q - nrm * OUTER_R - n1 * OUTER_R;
            let t1 = center + nrm * OUTER_R;
            let t2 = center + n1 * OUTER_R;
            // The tangent points stand `OUTER_R - ins` beyond the pitch corner
            // along each axis, and the arc between them stays nearer than that,
            // so this is the whole of the bulge. `carve_to_cells` claims exactly
            // `REENTRANT_FILLET_OVERHANG` of the empty cell to keep it; if the
            // arc ever reached further, a split would shave the excess off and
            // lose it from every piece.
            assert!(
                OUTER_R - ins <= REENTRANT_FILLET_OVERHANG
                    && OUTER_R - ins_next <= REENTRANT_FILLET_OVERHANG,
                "a reentrant fillet overhangs the pitch corner by {} x {} mm, past the {} mm \
                 `carve_to_cells` reserves in the empty cell",
                OUTER_R - ins,
                OUTER_R - ins_next,
                REENTRANT_FILLET_OVERHANG
            );
            let a0 = f64::atan2(t1.y - center.y, t1.x - center.x);
            let a1 = f64::atan2(t2.y - center.y, t2.x - center.x);
            let (a0, a1) = short_arc(a0, a1);
            pieces.push(OuterPiece {
                seg: Seg::Line { a: start, b: t1 },
                shared: false,
                edge: None,
            });
            pieces.push(OuterPiece {
                seg: Seg::Arc {
                    a: t1,
                    b: t2,
                    center,
                    radius: OUTER_R,
                    a0,
                    a1,
                },
                shared: false,
                edge: None,
            });
            pieces.push(OuterPiece {
                seg: Seg::Line { a: t2, b: end },
                shared: false,
                edge: None,
            });
        } else {
            shared.squared.insert(s.to);
            let q = mm(s.to, pitch) + nrm * ins + n1 * ins_next;
            pieces.push(OuterPiece {
                seg: Seg::Line { a: start, b: q },
                shared: false,
                edge: None,
            });
            pieces.push(OuterPiece {
                seg: Seg::Line { a: q, b: end },
                shared: false,
                edge: None,
            });
        }
    }
    pieces
}

pub(super) struct OuterLoops {
    pub(super) loops: Vec<Vec<OuterPiece>>,
    pub(super) consumed: Vec<Vec<bool>>,
}

impl OuterLoops {
    pub(super) fn new(loops: Vec<Vec<OuterPiece>>) -> OuterLoops {
        let consumed = loops.iter().map(|l| vec![false; l.len()]).collect();
        OuterLoops { loops, consumed }
    }

    /// Cut whichever outer loop passes through `p`, if any.
    ///
    /// The standing wall above the floor and the base's outer wall below it
    /// share the lip between them, so the lip has to carry a vertex wherever
    /// the wall above starts or stops. Without it the base emits one long edge
    /// across a span the floor and the wall above have already divided, and
    /// nothing pairs with it.
    pub(super) fn split_outline_at(
        &mut self,
        p: Vec2,
        peg_splits: &mut HashMap<GridEdge, Vec<f64>>,
        peg_arcs: &mut Vec<Vec2>,
    ) {
        for li in 0..self.loops.len() {
            let hit = self.loops[li]
                .iter()
                .find(|pc| point_seg_distance(p, &pc.seg) < COINCIDENT);
            let Some(pc) = hit else { continue };
            if pc.shared && matches!(pc.seg, Seg::Arc { .. }) {
                peg_arcs.push(p);
            }
            self.split_at(li, p, peg_splits);
            return;
        }
    }

    /// Cut the outer loop at `p` so a walk can start or stop there.
    ///
    /// The pinch can land on a rounded corner as readily as on a straight run --
    /// a notch's outer fillet is exactly where a cavity wall that falls short of
    /// the outline meets it -- so an arc piece splits the same way a line does.
    pub(super) fn split_at(
        &mut self,
        li: usize,
        p: Vec2,
        peg_splits: &mut HashMap<GridEdge, Vec<f64>>,
    ) {
        let pieces = &mut self.loops[li];
        for i in 0..pieces.len() {
            let pc = pieces[i];
            if v2_eq(pc.seg.start(), p) || v2_eq(pc.seg.end(), p) {
                if crate::kernel::region2d::point_seg_distance(p, &pc.seg) < COINCIDENT {
                    return;
                }
                continue;
            }
            if crate::kernel::region2d::point_seg_distance(p, &pc.seg) > COINCIDENT {
                continue;
            }
            let (lo, hi) = match pc.seg {
                Seg::Line { a, b } => (Seg::Line { a, b: p }, Seg::Line { a: p, b }),
                Seg::Arc {
                    a,
                    b,
                    center,
                    radius,
                    a0,
                    a1,
                } => {
                    let (amin, amax) = (a0.min(a1), a0.max(a1));
                    let t = wrap_angle_into(
                        (p.y - center.y).atan2(p.x - center.x),
                        amin,
                        amax,
                        ARC_ENDPOINT_ANGLE,
                    );
                    if t <= amin + ARC_ENDPOINT_ANGLE || t >= amax - ARC_ENDPOINT_ANGLE {
                        return;
                    }
                    (
                        Seg::Arc {
                            a,
                            b: p,
                            center,
                            radius,
                            a0,
                            a1: t,
                        },
                        Seg::Arc {
                            a: p,
                            b,
                            center,
                            radius,
                            a0: t,
                            a1,
                        },
                    )
                }
            };
            // A peg's top ring welds to the wall's bottom ring along a shared
            // straight run, so only a line's split has a station to record.
            if pc.shared && matches!(pc.seg, Seg::Line { .. }) {
                if let Some(e) = pc.edge {
                    let station = match e.orientation {
                        Orientation::H => p.x,
                        Orientation::V => p.y,
                    };
                    peg_splits.entry(e).or_default().push(station);
                }
            }
            pieces[i] = OuterPiece { seg: lo, ..pc };
            pieces.insert(i + 1, OuterPiece { seg: hi, ..pc });
            let was = self.consumed[li][i];
            self.consumed[li].insert(i + 1, was);
            return;
        }
        panic!("open-face pinch point {p:?} is not on the outer loop");
    }
}

pub(super) fn outline_region(o: &OuterLoops) -> Vec<Vec<Seg>> {
    o.loops
        .iter()
        .map(|l| l.iter().map(|p| p.seg).collect())
        .collect()
}

pub(super) fn on_outline(outline: &[Vec<Seg>], p: Vec2) -> bool {
    outline
        .iter()
        .flatten()
        .any(|sg| point_seg_distance(p, sg) < COINCIDENT)
}
