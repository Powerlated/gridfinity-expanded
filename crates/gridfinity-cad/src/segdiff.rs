//! Analytic 2D boolean difference of a seg-loop region and a convex quad.
//!
//! The region is a set of closed `Seg` loops: outer boundaries CCW
//! (`loop_area > 0`), holes CW. The subtrahend is a convex CCW quad of `Line`
//! segments (an inner wall's footprint). The difference boundary is exact:
//! region sub-segments outside the quad, plus quad sub-segments inside the
//! region traversed reversed. All intersection points come from closed-form
//! line/line and line/circle solutions and are computed once, so both sides
//! of every split share bit-identical endpoints and the chained loops weld
//! exactly in the B-rep builder later.
//!
//! Degenerate inputs (a quad edge collinear-overlapping a region edge,
//! tangencies) are not supported — callers author walls at generic positions.

use crate::math::Vec2;
use crate::sketch::{Seg, loop_area, point_in_segs};

const EPS: f32 = 1e-4;

/// The pieces of a region and a quad split at their mutual intersections,
/// classified by side. Endpoints of matching pieces are shared verbatim.
/// Every piece carries a provenance tag `T`: region pieces inherit their
/// source segment's tag, quad pieces get the per-call `quad_tag` — so callers
/// identify pieces exactly, never by comparing float coordinates.
pub struct QuadSplit<T> {
    /// Region boundary pieces outside the quad (original traversal direction).
    pub outside: Vec<(Seg, T)>,
    /// Region boundary pieces inside the quad — the contact runs.
    pub inside: Vec<(Seg, T)>,
    /// Quad boundary pieces inside the region, in quad (CCW) direction.
    pub quad_inside: Vec<(Seg, T)>,
}

/// Subtract a convex CCW quad from a region (outers CCW, holes CW). Returns
/// the difference's loops in the same convention.
pub fn subtract_convex_quad(loops: &[Vec<Seg>], quad: &[Seg]) -> Vec<Vec<Seg>> {
    let tagged: Vec<Vec<(Seg, ())>> =
        loops.iter().map(|l| l.iter().map(|&s| (s, ())).collect()).collect();
    let s = split_region_by_quad(&tagged, quad, ());
    let mut kept = s.outside;
    kept.extend(s.quad_inside.iter().map(|(p, t)| (p.reversed(), *t)));
    chain_loops(kept)
        .into_iter()
        .map(|lp| lp.into_iter().map(|(s, _)| s).collect())
        .collect()
}

/// Split a region's boundary and a convex CCW quad's boundary at their mutual
/// intersections and classify every piece (the shared machinery behind
/// [`subtract_convex_quad`] and the inner-wall band construction).
pub fn split_region_by_quad<T: Copy>(
    loops: &[Vec<(Seg, T)>],
    quad: &[Seg],
    quad_tag: T,
) -> QuadSplit<T> {
    let qpts: Vec<Vec2> = quad.iter().map(|s| s.start()).collect();
    let plain: Vec<Vec<Seg>> =
        loops.iter().map(|l| l.iter().map(|&(s, _)| s).collect()).collect();

    // Split points per region seg (param along seg) and per quad edge (param u).
    // (loop idx, seg idx, param) → shared point.
    let mut region_cuts: Vec<Vec<Vec<(f32, Vec2)>>> =
        loops.iter().map(|l| vec![Vec::new(); l.len()]).collect();
    let mut quad_cuts: Vec<Vec<(f32, Vec2)>> = vec![Vec::new(); 4];

    for (li, lp) in loops.iter().enumerate() {
        for (si, (seg, _)) in lp.iter().enumerate() {
            for (qi, qe) in quad.iter().enumerate() {
                let (Seg::Line { a: q0, b: q1 }, hits) = (qe, seg_line_hits(seg, qe)) else {
                    continue;
                };
                for (t, p) in hits {
                    let u = line_param(*q0, *q1, p);
                    if !(-EPS..=1.0 + EPS).contains(&u) {
                        continue;
                    }
                    region_cuts[li][si].push((t, p));
                    quad_cuts[qi].push((u, p));
                }
            }
        }
    }

    let inside_quad = |p: Vec2| -> bool {
        (0..4).all(|i| {
            let a = qpts[i];
            let b = qpts[(i + 1) % 4];
            (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x) > EPS
        })
    };
    // Even–odd over every loop: inside the region iff contained by an odd
    // number of loops (an outer, minus its holes).
    let inside_region = |p: Vec2| -> bool {
        plain.iter().filter(|l| point_in_segs(p, l)).count() % 2 == 1
    };

    let mut out = QuadSplit { outside: Vec::new(), inside: Vec::new(), quad_inside: Vec::new() };
    for (li, lp) in loops.iter().enumerate() {
        for (si, &(ref seg, tag)) in lp.iter().enumerate() {
            let mut cuts = std::mem::take(&mut region_cuts[li][si]);
            for piece in split_seg(seg, &mut cuts) {
                if inside_quad(seg_mid(&piece)) {
                    out.inside.push((piece, tag));
                } else {
                    out.outside.push((piece, tag));
                }
            }
        }
    }
    for (qi, qe) in quad.iter().enumerate() {
        let mut cuts = std::mem::take(&mut quad_cuts[qi]);
        for piece in split_seg(qe, &mut cuts) {
            if inside_region(seg_mid(&piece)) {
                out.quad_inside.push((piece, quad_tag));
            }
        }
    }
    out
}

/// Param of `p` along the line a→b.
fn line_param(a: Vec2, b: Vec2, p: Vec2) -> f32 {
    let d = b - a;
    let l2 = d.length_squared();
    if l2 <= 0.0 { 0.0 } else { (p - a).dot(d) / l2 }
}

/// Intersections of a region seg with a quad edge line segment, as
/// `(param on seg, point)`. Line/line and circle/line are closed form.
fn seg_line_hits(seg: &Seg, edge: &Seg) -> Vec<(f32, Vec2)> {
    let Seg::Line { a: q0, b: q1 } = *edge else { return Vec::new() };
    let mut out = Vec::new();
    match *seg {
        Seg::Line { a, b } => {
            let d = b - a;
            let e = q1 - q0;
            let den = d.x * e.y - d.y * e.x;
            if den.abs() < 1e-9 {
                return out; // parallel (collinear overlap unsupported)
            }
            let w = q0 - a;
            let t = (w.x * e.y - w.y * e.x) / den;
            let u = (w.x * d.y - w.y * d.x) / den;
            if (-EPS..=1.0 + EPS).contains(&t) && (-EPS..=1.0 + EPS).contains(&u) {
                out.push((t.clamp(0.0, 1.0), a + d * t));
            }
        }
        Seg::Arc { center, radius, a0, a1, .. } => {
            // |q0 + u·e − c|² = r²  →  quadratic in u.
            let e = q1 - q0;
            let f = q0 - center;
            let qa = e.length_squared();
            let qb = 2.0 * f.dot(e);
            let qc = f.length_squared() - radius * radius;
            let disc = qb * qb - 4.0 * qa * qc;
            if disc <= 0.0 || qa <= 0.0 {
                return out;
            }
            let sq = disc.sqrt();
            for u in [(-qb - sq) / (2.0 * qa), (-qb + sq) / (2.0 * qa)] {
                if !(-EPS..=1.0 + EPS).contains(&u) {
                    continue;
                }
                let p = q0 + e * u;
                let ang = (p.y - center.y).atan2(p.x - center.x);
                // Map into the arc's (unwrapped) angle span.
                let (lo, hi) = (a0.min(a1), a0.max(a1));
                let mut th = ang;
                while th < lo - 1e-6 {
                    th += 2.0 * std::f32::consts::PI;
                }
                while th > hi + 1e-6 {
                    th -= 2.0 * std::f32::consts::PI;
                }
                if th < lo - 1e-6 {
                    continue;
                }
                out.push((th, p));
            }
        }
    }
    out
}

fn seg_mid(seg: &Seg) -> Vec2 {
    match *seg {
        Seg::Line { a, b } => (a + b) * 0.5,
        Seg::Arc { center, radius, a0, a1, .. } => {
            let t = (a0 + a1) * 0.5;
            center + Vec2::new(t.cos(), t.sin()) * radius
        }
    }
}

/// Split a seg at the given `(param, point)` cuts (param = t∈[0,1] for lines,
/// unwrapped angle for arcs). Cut points become sub-seg endpoints verbatim so
/// both sides of a split share exact coordinates.
fn split_seg(seg: &Seg, cuts: &mut Vec<(f32, Vec2)>) -> Vec<Seg> {
    match *seg {
        Seg::Line { a, b } => {
            cuts.retain(|&(t, _)| (EPS..=1.0 - EPS).contains(&t));
            cuts.sort_by(|x, y| x.0.total_cmp(&y.0));
            cuts.dedup_by(|x, y| (x.0 - y.0).abs() < EPS);
            let mut out = Vec::with_capacity(cuts.len() + 1);
            let mut prev = a;
            for &(_, p) in cuts.iter() {
                out.push(Seg::Line { a: prev, b: p });
                prev = p;
            }
            out.push(Seg::Line { a: prev, b });
            out
        }
        Seg::Arc { a, b, center, radius, a0, a1 } => {
            let fwd = a1 >= a0;
            let (lo, hi) = (a0.min(a1), a0.max(a1));
            let span = (hi - lo).max(1e-9);
            let aeps = EPS.max(1e-3 / radius.max(1e-3)); // ≈ EPS of arc length
            cuts.retain(|&(t, _)| t > lo + aeps * span.min(1.0) && t < hi - aeps * span.min(1.0));
            if fwd {
                cuts.sort_by(|x, y| x.0.total_cmp(&y.0));
            } else {
                cuts.sort_by(|x, y| y.0.total_cmp(&x.0));
            }
            cuts.dedup_by(|x, y| (x.0 - y.0).abs() < aeps);
            let mut out = Vec::with_capacity(cuts.len() + 1);
            let mut prev = (a0, a);
            for &(t, p) in cuts.iter() {
                out.push(Seg::Arc { a: prev.1, b: p, center, radius, a0: prev.0, a1: t });
                prev = (t, p);
            }
            out.push(Seg::Arc { a: prev.1, b, center, radius, a0: prev.0, a1 });
            out
        }
    }
}

/// Chain sub-segs into closed loops by endpoint proximity (endpoints of
/// matching pieces are bit-identical by construction; the tolerance only
/// bridges float noise in the rare mixed case). Provenance tags ride along.
pub fn chain_loops<T: Copy>(mut segs: Vec<(Seg, T)>) -> Vec<Vec<(Seg, T)>> {
    let mut out: Vec<Vec<(Seg, T)>> = Vec::new();
    while let Some(first) = segs.pop() {
        let start = first.0.start();
        let mut lp = vec![first];
        loop {
            let cur = lp.last().unwrap().0.end();
            if (cur - start).length() < EPS && lp.len() >= 2 {
                break;
            }
            let Some((idx, _)) = segs
                .iter()
                .enumerate()
                .map(|(i, (s, _))| (i, (s.start() - cur).length()))
                .filter(|&(_, d)| d < EPS)
                .min_by(|x, y| x.1.total_cmp(&y.1))
            else {
                break; // open chain: drop (degenerate input)
            };
            lp.push(segs.swap_remove(idx));
        }
        if (lp.last().unwrap().0.end() - start).length() < EPS && lp.len() >= 2 {
            // Drop zero-area slivers.
            let bare: Vec<Seg> = lp.iter().map(|&(s, _)| s).collect();
            if loop_area(&bare).abs() > EPS {
                out.push(lp);
            }
        }
    }
    out
}
