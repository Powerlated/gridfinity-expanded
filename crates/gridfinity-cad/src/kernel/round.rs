//! Reading and reshaping a closed 2D loop of `Seg`s at its corners.
//!
//! Three concerns, all about the joints rather than the runs. **Tangency**:
//! `seg_tangent` and `sharp_between` say whether two consecutive segments meet
//! smoothly, which is the question every blend chain and every corner rounding
//! asks first. **Rounding**: `round_sharp_corners` inscribes an arc at each
//! corner of a loop that already exists as segments, shrinking the radii until
//! neighbouring trims fit in the run between them -- the counterpart to
//! `rectregion::shape_loop`, which rounds a loop given as *points* and cannot be
//! handed a loop a boolean has already cut into pieces. **Reading back**:
//! `corners_of` and `loop_of_points` convert between the two forms, and
//! `seg_mid`, `seg_samples`, `drop_degenerate` and `is_convex_arc` are the
//! sampling and hygiene a caller needs to ask anything else of a loop a sweep
//! produced.
//!
//! Nothing here builds geometry or knows what the loop bounds.

use crate::kernel::math::{Vec2, wrap_pi};
use crate::kernel::rectregion::merge_collinear;
use crate::kernel::sketch::{COINCIDENT, Seg, loop_area};

/// How near `cos phi` must be to 1 for two consecutive segments to count as
/// meeting smoothly. 0.9995 is a turn of about 1.8 degrees -- three orders above
/// the float noise in a tangent read off a boolean's output, and well below any
/// turn a bin's boundary makes on purpose, the shallowest of which is the
/// 45-degree step where a cavity's inset changes along one run.
pub const TANGENT_DOT: f64 = 0.9995;

/// How far `|cos phi|` must fall below 1 for the turn between two runs to be a
/// corner worth rounding. Below it the pair is collinear (`phi = 0`) or doubles
/// back on itself (`phi = pi`), and neither admits an inscribed arc: the first
/// has no corner, the second no interior. It also floors the half-angle tangent
/// the trim divides by -- `tan(acos(1 - CORNER_TURNS) / 2)`, about 7.1e-4.
pub const CORNER_TURNS: f64 = 1e-6;

/// The closed loop of straight segments through `pts` in order, last back to
/// first. Each point becomes one segment's start and the previous one's end, so
/// the result has exactly `pts.len()` segments and inherits the winding of the
/// point sequence.
pub fn loop_of_points(pts: &[Vec2]) -> Vec<Seg> {
    let n = pts.len();
    assert!(n >= 3, "a closed loop has at least three corners, got {n}");
    (0..n)
        .map(|i| Seg::Line {
            a: pts[i],
            b: pts[(i + 1) % n],
        })
        .collect()
}

/// A rectilinear seg loop read back as the corners it turns at.
///
/// A boolean cuts segments wherever the other operand crossed, so the run that
/// arrives is subdivided; `merge_collinear` takes it back to one point per turn,
/// which the corner-rounding pass requires -- it reads a collinear point as a
/// reentrant corner and rounds it by the floor fillet.
pub fn corners_of(segs: &[Seg]) -> Vec<Vec2> {
    let pts: Vec<Vec2> = segs
        .iter()
        .map(|sg| {
            assert!(
                matches!(sg, Seg::Line { .. }),
                "a rectilinear boundary is straight throughout, got {sg:?}"
            );
            sg.start()
        })
        .collect();
    let mut kept: Vec<Vec2> = Vec::with_capacity(pts.len());
    for (i, &p) in pts.iter().enumerate() {
        if (p - pts[(i + pts.len() - 1) % pts.len()]).length() > COINCIDENT {
            kept.push(p);
        }
    }
    merge_collinear(&kept)
}

/// `(a0, a1)` re-expressed so the sweep from one to the other is the short way
/// round: the returned pair starts at `a0` and ends within `PI` of it, naming
/// the same two points of the circle as the input. An exact half turn, where
/// neither way round is shorter, sweeps positive -- `wrap_pi`'s closed end.
pub fn short_arc(a0: f64, a1: f64) -> (f64, f64) {
    (a0, a0 + wrap_pi(a1 - a0))
}

/// Whether two points are the same point of a boundary, within `COINCIDENT`.
pub fn v2_eq(a: Vec2, b: Vec2) -> bool {
    (a - b).length() < COINCIDENT
}

/// The mid-point of a segment, for asking which side of a boundary it is on.
/// Taken on the true line or the true circle, so on an arc it is the point at
/// the mean parameter and never the chord's midpoint.
pub fn seg_mid(seg: &Seg) -> Vec2 {
    match *seg {
        Seg::Line { a, b } => (a + b) * 0.5,
        Seg::Arc {
            center,
            radius,
            a0,
            a1,
            ..
        } => {
            let t = (a0 + a1) * 0.5;
            center + Vec2::new(t.cos(), t.sin()) * radius
        }
    }
}

/// Drop segments a boolean left with coincident endpoints, and loops left with
/// fewer than three segments once those are gone.
///
/// A region sweep can cut a run twice at what is the same point in f64 and hand
/// back a hair between the two cuts. The loop stays continuous without it --
/// the neighbours already meet there -- and `build::wall_between` takes a
/// segment's plane normal from the quad it sweeps, so a zero-length one gives a
/// zero normal and no plane at all.
pub fn drop_degenerate(loops: Vec<Vec<Seg>>) -> Vec<Vec<Seg>> {
    loops
        .into_iter()
        .map(|l| {
            l.into_iter()
                .filter(|sg| (sg.end() - sg.start()).length() > COINCIDENT)
                .collect::<Vec<Seg>>()
        })
        .filter(|l| l.len() >= 3)
        .collect()
}

/// The unit direction `s` travels in at one of its ends -- its far end when
/// `end`, its near end otherwise. On an arc that is the circle's tangent there,
/// signed by the sweep's direction, so two segments that meet smoothly return
/// the same vector from either side of the joint.
pub fn seg_tangent(s: &Seg, end: bool) -> Vec2 {
    match *s {
        Seg::Line { a, b } => (b - a).normalize(),
        Seg::Arc { a0, a1, .. } => {
            let t = if end { a1 } else { a0 };
            let dir = if a1 >= a0 { 1.0 } else { -1.0 };
            Vec2::new(-t.sin(), t.cos()) * dir
        }
    }
}

/// Whether the joint where `shape[i]` ends and `shape[j]` begins turns by more
/// than `TANGENT_DOT` allows -- a corner rather than a smooth continuation.
pub fn sharp_between(shape: &[Seg], i: usize, j: usize) -> bool {
    seg_tangent(&shape[i], true).dot(seg_tangent(&shape[j], false)) < TANGENT_DOT
}

/// Whether the closed loop `shape` turns sharply anywhere, its last-to-first
/// joint included.
pub fn has_sharp_corner(shape: &[Seg]) -> bool {
    let n = shape.len();
    (0..n).any(|i| sharp_between(shape, i, (i + 1) % n))
}

/// Points along `s`, each with the unit direction the segment travels there:
/// both endpoints and interior points no more than `step` apart, taken on the
/// true line or the true circle so neither the point nor the tangent is ever
/// read off a chord. A degenerate segment yields nothing.
pub fn seg_samples(s: &Seg, step: f64) -> Vec<(Vec2, Vec2)> {
    assert!(
        step > 0.0,
        "sampling a segment needs a positive spacing, got {step}"
    );
    let mut out = Vec::new();
    let (len, at): (f64, Box<dyn Fn(f64) -> (Vec2, Vec2)>) = match *s {
        Seg::Line { a, b } => {
            let d = b - a;
            let len = d.length();
            if !(len > 0.0) {
                return out;
            }
            let t = d / len;
            (len, Box::new(move |u| (a + d * u, t)))
        }
        Seg::Arc {
            center,
            radius,
            a0,
            a1,
            ..
        } => {
            let sweep = a1 - a0;
            (
                sweep.abs() * radius,
                Box::new(move |u| {
                    let ang = a0 + sweep * u;
                    let radial = Vec2::new(ang.cos(), ang.sin());
                    (
                        center + radial * radius,
                        Vec2::new(-radial.y, radial.x) * sweep.signum(),
                    )
                }),
            )
        }
    };
    if !(len > 0.0) {
        return out;
    }
    let n = (len / step).ceil() as usize;
    for k in 0..=n {
        out.push(at(k as f64 / n as f64));
    }
    out
}

/// Whether `s` is an arc of `shape` that bulges *away* from the material -- a
/// convex corner -- read off the loop's winding. A straight segment is neither
/// and answers `false`.
pub fn is_convex_arc(shape: &[Seg], s: &Seg) -> bool {
    let ccw = loop_area(shape) > 0.0;
    match s {
        Seg::Arc { a0, a1, .. } => (a1 > a0) == ccw,
        _ => false,
    }
}

/// The smallest corner arc worth inscribing: below it the arc is shorter than
/// `topo`'s weld quantum, so its two tangent points are one vertex as far as the
/// builder is concerned and the corner is square in the solid whatever the
/// sketch says. **Every** entry point that rounds a corner honours it, or two
/// roundings of the same corner disagree about whether it is an arc at all --
/// `rectregion::shape_loop` emitted a 0.066 mm arc where `round_sharp_corners`
/// would have left the corner sharp, and a hair-thin *arc* is not a chain
/// terminator the way a sharp corner is, so it dragged a whole compartment's
/// 2.48 mm floor fillet down to its own radius instead of ending the chain.
pub const MIN_ARC_R: f64 = 0.1;

/// `segs` with an arc inscribed at every sharp corner between two straight
/// runs: radius `convex_r` where the loop turns away from its material and
/// `concave_r` where it turns into it, both read off the loop's own winding, and
/// zero meaning "leave that corner sharp".
///
/// Radii are not honoured blindly. Two corners of one run each trim it by
/// `r * tan(phi/2)`, so where the two trims exceed the run the pair is scaled
/// down to fit it (with `USABLE` of margin), which can in turn take a radius
/// under `MIN_ARC_R` -- an arc shorter than the weld quantum -- and that corner
/// reverts to sharp, freeing its neighbour's trim again. Hence the fixed point:
/// the loop is swept until nothing changes.
///
/// Arcs already in `segs` are passed through untouched; only line/line joints
/// are rounded. The result is the same closed loop with the same winding, one
/// segment per input plus one arc per rounded corner.
pub fn round_sharp_corners(segs: &[Seg], convex_r: f64, concave_r: f64) -> Vec<Seg> {
    let n = segs.len();
    if n < 2 || (convex_r <= 0.0 && concave_r <= 0.0) {
        return segs.to_vec();
    }
    let ccw = loop_area(segs) > 0.0;

    let mut trim = vec![0.0f64; n];
    let mut arc_r = vec![0.0f64; n];
    let mut tan_half = vec![0.0f64; n];
    for i in 0..n {
        let (cur, next) = (&segs[i], &segs[(i + 1) % n]);
        let (Seg::Line { .. }, Seg::Line { .. }) = (cur, next) else {
            continue;
        };
        let d_in = seg_tangent(cur, true);
        let d_out = seg_tangent(next, false);
        let dot = d_in.dot(d_out).clamp(-1.0, 1.0);
        if dot > 1.0 - CORNER_TURNS || dot < -1.0 + CORNER_TURNS {
            continue;
        }
        let cross = d_in.x * d_out.y - d_in.y * d_out.x;
        let r = if (cross > 0.0) == ccw {
            convex_r
        } else {
            concave_r
        };
        if r <= 0.0 {
            continue;
        }
        let phi = dot.acos();
        tan_half[i] = (phi / 2.0).tan();
        arc_r[i] = r;
        trim[i] = r * tan_half[i];
    }
    const USABLE: f64 = 0.98;
    let mut changed = true;
    while changed {
        changed = false;
        for i in 0..n {
            let Seg::Line { a, b } = segs[i] else {
                continue;
            };
            let prev = (i + n - 1) % n;
            let want = trim[prev] + trim[i];
            let len = (b - a).length() * USABLE;
            if want <= len || want <= 0.0 {
                continue;
            }
            let k = len / want;
            for idx in [prev, i] {
                if trim[idx] > 0.0 {
                    assert!(
                        tan_half[idx] > 0.0,
                        "a corner with a trim to shrink turns by more than CORNER_TURNS, so its \
                         half-angle tangent is positive, got {} at corner {idx}",
                        tan_half[idx]
                    );
                    trim[idx] *= k;
                    arc_r[idx] = trim[idx] / tan_half[idx];
                    changed = true;
                }
            }
        }
        for i in 0..n {
            if arc_r[i] > 0.0 && arc_r[i] < MIN_ARC_R {
                arc_r[i] = 0.0;
                trim[i] = 0.0;
                changed = true;
            }
        }
    }

    let mut out: Vec<Seg> = Vec::with_capacity(n * 2);
    for i in 0..n {
        let prev = (i + n - 1) % n;
        let seg = segs[i];
        let seg = match seg {
            Seg::Line { a, b } => {
                let d = (b - a).normalize_or_zero();
                Seg::Line {
                    a: a + d * trim[prev],
                    b: b - d * trim[i],
                }
            }
            other => other,
        };
        out.push(seg);
        if trim[i] <= 0.0 || arc_r[i] <= 0.0 {
            continue;
        }
        let v = segs[i].end();
        let d_in = seg_tangent(&segs[i], true);
        let d_out = seg_tangent(&segs[(i + 1) % n], false);
        let cross = d_in.x * d_out.y - d_in.y * d_out.x;
        let p_in = v - d_in * trim[i];
        let p_out = v + d_out * trim[i];
        let nrm = if cross > 0.0 {
            Vec2::new(-d_in.y, d_in.x)
        } else {
            Vec2::new(d_in.y, -d_in.x)
        };
        let center = p_in + nrm * arc_r[i];
        let a0 = f64::atan2(p_in.y - center.y, p_in.x - center.x);
        let a1 = f64::atan2(p_out.y - center.y, p_out.x - center.x);
        let (a0, a1) = short_arc(a0, a1);
        out.push(Seg::Arc {
            a: p_in,
            b: p_out,
            center,
            radius: arc_r[i],
            a0,
            a1,
        });
    }
    for i in 0..out.len() {
        let gap = (out[(i + 1) % out.len()].start() - out[i].end()).length();
        assert!(
            gap <= COINCIDENT,
            "rounding leaves the loop closed, but segment {i} of {} ends {gap} from where the \
             next begins -- two corners' trims consumed more of the run between them than it has",
            out.len()
        );
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square(side: f64) -> Vec<Seg> {
        loop_of_points(&[
            Vec2::new(0.0, 0.0),
            Vec2::new(side, 0.0),
            Vec2::new(side, side),
            Vec2::new(0.0, side),
        ])
    }

    #[test]
    fn a_square_rounds_to_four_runs_and_four_arcs_of_the_requested_radius() {
        let out = round_sharp_corners(&square(20.0), 3.0, 3.0);
        let arcs: Vec<&Seg> = out
            .iter()
            .filter(|s| matches!(s, Seg::Arc { .. }))
            .collect();
        assert_eq!(arcs.len(), 4, "one arc per corner, got {out:?}");
        for a in arcs {
            let Seg::Arc { radius, .. } = a else {
                unreachable!()
            };
            assert!(
                (*radius - 3.0).abs() < 1e-4,
                "a radius the runs have room for is honoured exactly, got {radius}"
            );
        }
        assert!(
            !has_sharp_corner(&out),
            "every corner of a fully rounded loop is tangent-continuous"
        );
    }

    #[test]
    fn a_radius_two_corners_cannot_share_is_shrunk_until_the_runs_survive() {
        let out = round_sharp_corners(&square(4.0), 3.0, 3.0);
        for s in &out {
            if let Seg::Arc { radius, .. } = s {
                assert!(
                    *radius < 3.0,
                    "a 3 mm radius does not fit twice into a 4 mm run, so every corner gives \
                     something up; got {radius}"
                );
            }
            if let Seg::Line { a, b } = s {
                assert!(
                    (*b - *a).length() >= 0.0 && (*b - *a).length() < 4.0,
                    "a trimmed run is shorter than the run it came from and never inverts, got \
                     {a:?} -> {b:?}"
                );
            }
        }
        assert!(
            !has_sharp_corner(&out),
            "shrinking the radii keeps every corner rounded rather than dropping one"
        );
    }

    #[test]
    fn rounding_by_zero_returns_the_loop_unchanged() {
        let sq = square(20.0);
        assert_eq!(round_sharp_corners(&sq, 0.0, 0.0), sq);
    }

    #[test]
    fn a_rounded_square_reads_back_as_its_four_corners() {
        let sq = square(20.0);
        let pts = corners_of(&sq);
        assert_eq!(pts.len(), 4, "a square turns at four points, got {pts:?}");
        assert_eq!(loop_of_points(&pts), sq);
    }

    #[test]
    fn a_boolean_subdivided_run_reads_back_as_one_corner() {
        let split = vec![
            Seg::Line {
                a: Vec2::new(0.0, 0.0),
                b: Vec2::new(10.0, 0.0),
            },
            Seg::Line {
                a: Vec2::new(10.0, 0.0),
                b: Vec2::new(20.0, 0.0),
            },
            Seg::Line {
                a: Vec2::new(20.0, 0.0),
                b: Vec2::new(20.0, 20.0),
            },
            Seg::Line {
                a: Vec2::new(20.0, 20.0),
                b: Vec2::new(0.0, 20.0),
            },
            Seg::Line {
                a: Vec2::new(0.0, 20.0),
                b: Vec2::new(0.0, 0.0),
            },
        ];
        assert_eq!(
            corners_of(&split).len(),
            4,
            "a collinear pair contributes no corner"
        );
    }

    #[test]
    fn a_zero_length_segment_is_dropped_and_its_loop_survives() {
        let mut sq = square(20.0);
        sq.insert(
            1,
            Seg::Line {
                a: Vec2::new(20.0, 0.0),
                b: Vec2::new(20.0, 0.0),
            },
        );
        let out = drop_degenerate(vec![sq]);
        assert_eq!(out.len(), 1, "the loop is still a loop");
        assert_eq!(out[0].len(), 4, "only the hair is gone");
    }

    #[test]
    fn sampling_covers_a_segment_end_to_end_at_no_more_than_the_step() {
        let s = Seg::Line {
            a: Vec2::new(0.0, 0.0),
            b: Vec2::new(7.0, 0.0),
        };
        let pts = seg_samples(&s, 0.5);
        assert!(v2_eq(pts[0].0, s.start()), "sampling starts at the start");
        assert!(
            v2_eq(pts[pts.len() - 1].0, s.end()),
            "sampling ends at the end"
        );
        for w in pts.windows(2) {
            assert!(
                (w[1].0 - w[0].0).length() <= 0.5 + 1e-4,
                "consecutive samples are no more than the step apart"
            );
        }
    }

    #[test]
    fn an_arcs_samples_and_tangents_lie_on_the_true_circle() {
        let c = Vec2::new(1.0, 2.0);
        let s = Seg::Arc {
            a: c + Vec2::new(3.0, 0.0),
            b: c + Vec2::new(0.0, 3.0),
            center: c,
            radius: 3.0,
            a0: 0.0,
            a1: std::f64::consts::FRAC_PI_2,
        };
        for (p, d) in seg_samples(&s, 0.25) {
            assert!(
                ((p - c).length() - 3.0).abs() < 1e-4,
                "a sample sits on the circle, not on a chord"
            );
            assert!(
                (p - c).normalize().dot(d).abs() < 1e-3,
                "the direction at a sample is perpendicular to the radius there"
            );
        }
    }
}
