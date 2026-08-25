use crate::kernel::hash::FxHashMap;
use crate::kernel::math::Vec2;
use crate::kernel::perf;
use crate::kernel::round::seg_mid;
use crate::kernel::sketch::{Aabb, COINCIDENT, Seg, loop_area, point_in_segs};

const EPS: f64 = 1e-4;

/// Shortest piece a split is allowed to leave behind, in millimetres. A cut
/// nearer than this to an end of the segment it lands on is dropped and the
/// existing vertex stands in for it. It has to exceed `EPS` -- the distance
/// `chain_loops` welds endpoints with -- or a split can leave a piece whose two
/// ends weld to each other. It is `COINCIDENT`, the same figure `on_seg` accepts
/// as "on" and every reader of a sweep's output treats as one point.
const SLIVER: f64 = COINCIDENT;

const _: () = assert!(
    SLIVER > EPS,
    "a split may not leave a piece shorter than the distance chain_loops welds endpoints with"
);

const BOX_TOL: f64 = 1e-2;

const _: () = assert!(
    BOX_TOL > 1e-3,
    "boxes are grown by BOX_TOL to cover the 1e-3 that on_seg accepts; below that the prune      drops real crossings and the boolean silently loses a cut"
);

static VERIFY_PRUNE: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

pub fn set_verify_prune(on: bool) {
    VERIFY_PRUNE.store(on, std::sync::atomic::Ordering::Relaxed);
}

fn verify_prune() -> bool {
    VERIFY_PRUNE.load(std::sync::atomic::Ordering::Relaxed)
}

fn assert_prune_kept_every_crossing(a: &[&[Seg]], b: &[&[Seg]]) {
    for al in a {
        for aseg in al.iter() {
            for bl in b {
                for bseg in bl.iter() {
                    if !boxes_meet(aseg.bbox(), bseg.bbox())
                        && !seg_seg_points(aseg, bseg).is_empty()
                    {
                        panic!("box prune dropped a real crossing: {aseg:?} vs {bseg:?}");
                    }
                }
            }
        }
    }
}

#[inline]
fn boxes_meet(x: Aabb, y: Aabb) -> bool {
    x.min.x - BOX_TOL <= y.max.x
        && y.min.x - BOX_TOL <= x.max.x
        && x.min.y - BOX_TOL <= y.max.y
        && y.min.y - BOX_TOL <= x.max.y
}

pub struct RegionSplit<T> {
    pub a_outside: Vec<(Seg, T)>,
    pub a_inside: Vec<(Seg, T)>,
    pub b_outside: Vec<(Seg, T)>,
    pub b_inside: Vec<(Seg, T)>,
    pub on_same: Vec<(Seg, T)>,
    pub on_opposite: Vec<(Seg, T)>,
}

fn seg_dir_at(seg: &Seg, p: Vec2) -> Vec2 {
    match *seg {
        Seg::Line { a, b } => (b - a).normalize_or_zero(),
        Seg::Arc { center, a0, a1, .. } => {
            let r = p - center;
            let t = Vec2::new(-r.y, r.x).normalize_or_zero();
            if a1 >= a0 { t } else { -t }
        }
    }
}

fn coincident_with<'a>(loops: &'a [Vec<Seg>], piece: &Seg) -> Option<&'a Seg> {
    let m = seg_mid(piece);
    loops.iter().flatten().find(|s| on_seg(s, m))
}

pub fn split_regions<T: Copy>(a: &[Vec<(Seg, T)>], b: &[Vec<(Seg, T)>]) -> RegionSplit<T> {
    if verify_prune() {
        let strip = |r: &[Vec<(Seg, T)>]| -> Vec<Vec<Seg>> {
            r.iter()
                .map(|l| l.iter().map(|&(s, _)| s).collect())
                .collect()
        };
        let (sa, sb) = (strip(a), strip(b));
        assert_prune_kept_every_crossing(
            &sa.iter().map(|l| l.as_slice()).collect::<Vec<_>>(),
            &sb.iter().map(|l| l.as_slice()).collect::<Vec<_>>(),
        );
    }
    let _perf = perf::scope(perf::Metric::SplitRegions);
    let mut a_cuts: Vec<Vec<Vec<(f64, Vec2)>>> =
        a.iter().map(|l| vec![Vec::new(); l.len()]).collect();
    let mut b_cuts: Vec<Vec<Vec<(f64, Vec2)>>> =
        b.iter().map(|l| vec![Vec::new(); l.len()]).collect();
    let b_boxes: Vec<Vec<Aabb>> = b
        .iter()
        .map(|l| l.iter().map(|(s, _)| s.bbox()).collect())
        .collect();
    let b_loop_box: Vec<Aabb> = b_boxes
        .iter()
        .map(|l| l.iter().fold(Aabb::EMPTY, |acc, x| acc.union(*x)))
        .collect();
    for (ai, al) in a.iter().enumerate() {
        for (asi, (aseg, _)) in al.iter().enumerate() {
            let abox = aseg.bbox();
            for (bi, bl) in b.iter().enumerate() {
                if !boxes_meet(abox, b_loop_box[bi]) {
                    continue;
                }
                for (bsi, (bseg, _)) in bl.iter().enumerate() {
                    if !boxes_meet(abox, b_boxes[bi][bsi]) {
                        continue;
                    }
                    for pt in seg_seg_points(aseg, bseg) {
                        a_cuts[ai][asi].push((seg_param(aseg, pt), pt));
                        b_cuts[bi][bsi].push((seg_param(bseg, pt), pt));
                    }
                }
            }
        }
    }

    let bare = |r: &[Vec<(Seg, T)>]| -> Vec<Vec<Seg>> {
        r.iter()
            .map(|l| l.iter().map(|&(s, _)| s).collect())
            .collect()
    };
    let (a_bare, b_bare) = (bare(a), bare(b));
    let inside =
        |loops: &[Vec<Seg>], p: Vec2| loops.iter().filter(|l| point_in_segs(p, l)).count() % 2 == 1;

    let mut out = RegionSplit {
        a_outside: Vec::new(),
        a_inside: Vec::new(),
        b_outside: Vec::new(),
        b_inside: Vec::new(),
        on_same: Vec::new(),
        on_opposite: Vec::new(),
    };
    for (li, lp) in a.iter().enumerate() {
        for (si, &(ref seg, tag)) in lp.iter().enumerate() {
            let mut cuts = std::mem::take(&mut a_cuts[li][si]);
            for piece in split_seg(seg, &mut cuts) {
                match coincident_with(&b_bare, &piece) {
                    Some(other) => {
                        let m = seg_mid(&piece);
                        if seg_dir_at(&piece, m).dot(seg_dir_at(other, m)) >= 0.0 {
                            out.on_same.push((piece, tag));
                        } else {
                            out.on_opposite.push((piece, tag));
                        }
                    }
                    None if inside(&b_bare, seg_mid(&piece)) => out.a_inside.push((piece, tag)),
                    None => out.a_outside.push((piece, tag)),
                }
            }
        }
    }
    for (li, lp) in b.iter().enumerate() {
        for (si, &(ref seg, tag)) in lp.iter().enumerate() {
            let mut cuts = std::mem::take(&mut b_cuts[li][si]);
            for piece in split_seg(seg, &mut cuts) {
                if coincident_with(&a_bare, &piece).is_some() {
                    continue;
                }
                if inside(&a_bare, seg_mid(&piece)) {
                    out.b_inside.push((piece, tag));
                } else {
                    out.b_outside.push((piece, tag));
                }
            }
        }
    }
    out
}

pub fn presplit_regions(regions: &[Vec<Vec<Seg>>]) -> Vec<Vec<Vec<Seg>>> {
    if verify_prune() {
        for ri in 0..regions.len() {
            for rj in ri + 1..regions.len() {
                let a: Vec<&[Seg]> = regions[ri].iter().map(|l| l.as_slice()).collect();
                let b: Vec<&[Seg]> = regions[rj].iter().map(|l| l.as_slice()).collect();
                assert_prune_kept_every_crossing(&a, &b);
            }
        }
    }
    let mut cuts: Vec<Vec<Vec<Vec<(f64, Vec2)>>>> = regions
        .iter()
        .map(|r| r.iter().map(|l| vec![Vec::new(); l.len()]).collect())
        .collect();
    let boxes: Vec<Vec<Vec<Aabb>>> = regions
        .iter()
        .map(|r| {
            r.iter()
                .map(|l| l.iter().map(|s| s.bbox()).collect())
                .collect()
        })
        .collect();
    let loop_box: Vec<Vec<Aabb>> = boxes
        .iter()
        .map(|r| {
            r.iter()
                .map(|l| l.iter().fold(Aabb::EMPTY, |acc, x| acc.union(*x)))
                .collect()
        })
        .collect();
    let region_box: Vec<Aabb> = loop_box
        .iter()
        .map(|r| r.iter().fold(Aabb::EMPTY, |acc, x| acc.union(*x)))
        .collect();
    for ri in 0..regions.len() {
        for rj in ri + 1..regions.len() {
            if !boxes_meet(region_box[ri], region_box[rj]) {
                continue;
            }
            let (before, from_rj) = cuts.split_at_mut(rj);
            let (cuts_i, cuts_j) = (&mut before[ri], &mut from_rj[0]);
            for (li, l) in regions[ri].iter().enumerate() {
                if !boxes_meet(loop_box[ri][li], region_box[rj]) {
                    continue;
                }
                for (si, s) in l.iter().enumerate() {
                    let sbox = boxes[ri][li][si];
                    for (lj, l2) in regions[rj].iter().enumerate() {
                        if !boxes_meet(sbox, loop_box[rj][lj]) {
                            continue;
                        }
                        for (sj, s2) in l2.iter().enumerate() {
                            if !boxes_meet(sbox, boxes[rj][lj][sj]) {
                                continue;
                            }
                            for pt in seg_seg_points(s, s2) {
                                cuts_i[li][si].push((seg_param(s, pt), pt));
                                cuts_j[lj][sj].push((seg_param(s2, pt), pt));
                            }
                        }
                    }
                }
            }
        }
    }
    regions
        .iter()
        .enumerate()
        .map(|(ri, r)| {
            r.iter()
                .enumerate()
                .map(|(li, l)| {
                    l.iter()
                        .enumerate()
                        .flat_map(|(si, s)| {
                            let mut c = std::mem::take(&mut cuts[ri][li][si]);
                            split_seg(s, &mut c)
                        })
                        .collect()
                })
                .collect()
        })
        .collect()
}

fn tagged(loops: &[Vec<Seg>]) -> Vec<Vec<(Seg, ())>> {
    loops
        .iter()
        .map(|l| l.iter().map(|&s| (s, ())).collect())
        .collect()
}

fn untag(loops: Vec<Vec<(Seg, ())>>) -> Vec<Vec<Seg>> {
    loops
        .into_iter()
        .map(|l| l.into_iter().map(|(s, _)| s).collect())
        .collect()
}

pub fn region_union(a: &[Vec<Seg>], b: &[Vec<Seg>]) -> Vec<Vec<Seg>> {
    let s = split_regions(&tagged(a), &tagged(b));
    let mut kept = s.a_outside;
    kept.extend(s.b_outside);
    kept.extend(s.on_same);
    untag(chain_loops(kept))
}

pub fn region_difference(a: &[Vec<Seg>], b: &[Vec<Seg>]) -> Vec<Vec<Seg>> {
    let s = split_regions(&tagged(a), &tagged(b));
    let mut kept = s.a_outside;
    kept.extend(s.b_inside.iter().map(|&(p, t)| (p.reversed(), t)));
    kept.extend(s.on_opposite);
    untag(chain_loops(kept))
}

pub fn region_intersection(a: &[Vec<Seg>], b: &[Vec<Seg>]) -> Vec<Vec<Seg>> {
    let s = split_regions(&tagged(a), &tagged(b));
    let mut kept = s.a_inside;
    kept.extend(s.b_inside);
    kept.extend(s.on_same);
    untag(chain_loops(kept))
}

fn line_param(a: Vec2, b: Vec2, p: Vec2) -> f64 {
    let d = b - a;
    let l2 = d.length_squared();
    if l2 <= 0.0 { 0.0 } else { (p - a).dot(d) / l2 }
}

fn seg_param(seg: &Seg, p: Vec2) -> f64 {
    match *seg {
        Seg::Line { a, b } => line_param(a, b, p),
        Seg::Arc { center, a0, a1, .. } => {
            let (lo, hi) = (a0.min(a1), a0.max(a1));
            let mut th = (p.y - center.y).atan2(p.x - center.x);
            while th < lo - 1e-6 {
                th += std::f64::consts::TAU;
            }
            while th > hi + 1e-6 {
                th -= std::f64::consts::TAU;
            }
            th
        }
    }
}

fn on_seg(seg: &Seg, p: Vec2) -> bool {
    match *seg {
        Seg::Line { a, b } => {
            let t = line_param(a, b, p);
            (-EPS..=1.0 + EPS).contains(&t) && (a + (b - a) * t - p).length() < 1e-3
        }
        Seg::Arc {
            center,
            radius,
            a0,
            a1,
            ..
        } => {
            if ((p - center).length() - radius).abs() > 1e-3 {
                return false;
            }
            let (lo, hi) = (a0.min(a1), a0.max(a1));
            let th = seg_param(seg, p);
            (lo - 1e-6..=hi + 1e-6).contains(&th)
        }
    }
}

fn circle_line_pts(center: Vec2, radius: f64, q0: Vec2, q1: Vec2) -> Vec<Vec2> {
    let e = q1 - q0;
    let f = q0 - center;
    let qa = e.length_squared();
    let qb = 2.0 * f.dot(e);
    let qc = f.length_squared() - radius * radius;
    let disc = qb * qb - 4.0 * qa * qc;
    if disc <= 0.0 || qa <= 0.0 {
        return Vec::new();
    }
    let sq = disc.sqrt();
    [(-qb - sq) / (2.0 * qa), (-qb + sq) / (2.0 * qa)]
        .into_iter()
        .map(|u| q0 + e * u)
        .collect()
}

fn circle_circle_pts(c0: Vec2, r0: f64, c1: Vec2, r1: f64) -> Vec<Vec2> {
    let d = (c1 - c0).length();
    if d < 1e-9 || d > r0 + r1 - EPS || d < (r0 - r1).abs() + EPS {
        return Vec::new();
    }
    let a = (r0 * r0 - r1 * r1 + d * d) / (2.0 * d);
    let h2 = r0 * r0 - a * a;
    if h2 <= 0.0 {
        return Vec::new();
    }
    let h = h2.sqrt();
    let u = (c1 - c0) / d;
    let base = c0 + u * a;
    let perp = Vec2::new(-u.y, u.x);
    vec![base + perp * h, base - perp * h]
}

/// Every point the two segments actually share, each lying on both of them.
/// Exposed because the open-run planner has to find where a cavity wall leaves
/// the bin's outline, and that wall may be an arc.
pub fn seg_seg_points(p: &Seg, q: &Seg) -> Vec<Vec2> {
    perf::count(perf::Metric::SegSegPoints);
    let raw = match (*p, *q) {
        (Seg::Line { a, b }, Seg::Line { a: c, b: d }) => {
            let (e, f) = (b - a, d - c);
            let den = e.x * f.y - e.y * f.x;
            if den.abs() < 1e-9 {
                return Vec::new();
            }
            let w = c - a;
            vec![a + e * ((w.x * f.y - w.y * f.x) / den)]
        }
        (Seg::Arc { center, radius, .. }, Seg::Line { a, b })
        | (Seg::Line { a, b }, Seg::Arc { center, radius, .. }) => {
            circle_line_pts(center, radius, a, b)
        }
        (
            Seg::Arc {
                center: c0,
                radius: r0,
                ..
            },
            Seg::Arc {
                center: c1,
                radius: r1,
                ..
            },
        ) => circle_circle_pts(c0, r0, c1, r1),
    };
    raw.into_iter()
        .filter(|&pt| on_seg(p, pt) && on_seg(q, pt))
        .collect()
}

/// Cut `seg` at `cuts` and return the pieces, in order.
///
/// Two properties everything downstream leans on, so both are checked on the
/// way out. The pieces **partition** the segment: the first starts where `seg`
/// starts, the last ends where it ends, and each one begins exactly where the
/// last left off -- a gap or an overlap here is a hole in the region. And no
/// piece is shorter than `SLIVER`, unless `seg` was already shorter than that
/// itself, because `chain_loops` welds endpoints within `EPS` and would join a
/// shorter piece's two ends to each other.
fn split_seg(seg: &Seg, cuts: &mut Vec<(f64, Vec2)>) -> Vec<Seg> {
    let out = split_seg_inner(seg, cuts);
    assert!(!out.is_empty(), "a split left no pieces of {seg:?}");
    assert!(
        (out[0].start() - seg.start()).length() < EPS,
        "the first piece {:?} does not start where {seg:?} does",
        out[0]
    );
    assert!(
        (out[out.len() - 1].end() - seg.end()).length() < EPS,
        "the last piece {:?} does not end where {seg:?} does",
        out[out.len() - 1]
    );
    let whole = (seg.end() - seg.start()).length();
    for (i, w) in out.windows(2).enumerate() {
        assert!(
            (w[0].end() - w[1].start()).length() < EPS,
            "pieces {i} and {} of {seg:?} do not meet: {:?} then {:?}",
            i + 1,
            w[0],
            w[1]
        );
    }
    if out.len() > 1 {
        for (i, p) in out.iter().enumerate() {
            let len = (p.end() - p.start()).length();
            assert!(
                len >= SLIVER || whole < SLIVER,
                "piece {i} of {seg:?} is {len} long, under the {SLIVER} a split may leave"
            );
        }
    }
    out
}

fn split_seg_inner(seg: &Seg, cuts: &mut Vec<(f64, Vec2)>) -> Vec<Seg> {
    match *seg {
        Seg::Line { a, b } => {
            // The margin is a distance, not a fraction. A parametric epsilon is
            // generous on a long segment and vanishing on a short one, and a cut
            // a fraction of a micron in from a vertex leaves a sliver shorter
            // than the tolerance `chain_loops` welds endpoints with -- the two
            // regions then disagree about where that vertex is and the chain
            // breaks. The arc branch below already measures in arc length.
            let margin = SLIVER / (b - a).length().max(1e-6);
            cuts.retain(|&(t, _)| (margin..=1.0 - margin).contains(&t));
            cuts.sort_by(|x, y| x.0.total_cmp(&y.0));
            cuts.dedup_by(|x, y| (x.0 - y.0).abs() < margin);
            let mut out = Vec::with_capacity(cuts.len() + 1);
            let mut prev = a;
            for &(_, p) in cuts.iter() {
                out.push(Seg::Line { a: prev, b: p });
                prev = p;
            }
            out.push(Seg::Line { a: prev, b });
            out
        }
        Seg::Arc {
            a,
            b,
            center,
            radius,
            a0,
            a1,
        } => {
            let fwd = a1 >= a0;
            let (lo, hi) = (a0.min(a1), a0.max(a1));
            // `SLIVER` millimetres of arc, as an angle. It is not scaled by the
            // arc's own span: a margin proportional to the span vanishes on a
            // short arc, which is the same mistake the line branch above used to
            // make with a parametric epsilon, and it lets a cut a fraction of a
            // micron from the end through.
            let aeps = EPS.max(SLIVER / radius.max(1e-3));
            cuts.retain(|&(t, _)| t > lo + aeps && t < hi - aeps);
            if fwd {
                cuts.sort_by(|x, y| x.0.total_cmp(&y.0));
            } else {
                cuts.sort_by(|x, y| y.0.total_cmp(&x.0));
            }
            cuts.dedup_by(|x, y| (x.0 - y.0).abs() < aeps);
            let mut out = Vec::with_capacity(cuts.len() + 1);
            let mut prev = (a0, a);
            for &(t, p) in cuts.iter() {
                out.push(Seg::Arc {
                    a: prev.1,
                    b: p,
                    center,
                    radius,
                    a0: prev.0,
                    a1: t,
                });
                prev = (t, p);
            }
            out.push(Seg::Arc {
                a: prev.1,
                b,
                center,
                radius,
                a0: prev.0,
                a1,
            });
            out
        }
    }
}

pub fn chain_loops<T: Copy>(segs: Vec<(Seg, T)>) -> Vec<Vec<(Seg, T)>> {
    let n = segs.len();
    let mut out: Vec<Vec<(Seg, T)>> = Vec::new();
    if n == 0 {
        return out;
    }

    let cell = EPS * 2.0;
    let key = |p: Vec2| ((p.x / cell).floor() as i64, (p.y / cell).floor() as i64);
    let mut buckets: FxHashMap<(i64, i64), Vec<u32>> =
        FxHashMap::with_capacity_and_hasher(n * 2, Default::default());
    for (i, (s, _)) in segs.iter().enumerate() {
        buckets.entry(key(s.start())).or_default().push(i as u32);
    }

    let mut used = vec![false; n];
    let mut bare: Vec<Seg> = Vec::new();
    for seed in (0..n).rev() {
        if used[seed] {
            continue;
        }
        used[seed] = true;
        let start = segs[seed].0.start();
        let mut lp = vec![segs[seed]];
        loop {
            let cur = lp
                .last()
                .expect("the chain was seeded with one piece")
                .0
                .end();
            if (cur - start).length() < EPS && lp.len() >= 2 {
                break;
            }
            let (kx, ky) = key(cur);
            let mut best: Option<(usize, f64)> = None;
            for dx in -1..=1 {
                for dy in -1..=1 {
                    let Some(list) = buckets.get(&(kx + dx, ky + dy)) else {
                        continue;
                    };
                    for &i in list {
                        let i = i as usize;
                        if used[i] {
                            continue;
                        }
                        let d = (segs[i].0.start() - cur).length();
                        if d < EPS && best.is_none_or(|(_, bd)| d < bd) {
                            best = Some((i, d));
                        }
                    }
                }
            }
            let Some((idx, _)) = best else { break };
            used[idx] = true;
            lp.push(segs[idx]);
        }
        if (lp
            .last()
            .expect("the chain was seeded with one piece")
            .0
            .end()
            - start)
            .length()
            < EPS
            && lp.len() >= 2
        {
            bare.clear();
            bare.extend(lp.iter().map(|&(s, _)| s));
            if loop_area(&bare).abs() > EPS {
                out.push(lp);
            }
        }
    }
    // Every loop returned is closed and every consecutive pair meets, because a
    // caller turns each one straight into a face and an open chain would leave
    // that face's boundary hanging. A piece is used at most once by
    // construction (`used`), so the loops are also disjoint.
    for (li, lp) in out.iter().enumerate() {
        assert!(lp.len() >= 2, "chained loop {li} has {} piece(s)", lp.len());
        for k in 0..lp.len() {
            let (a, b) = (lp[k].0, lp[(k + 1) % lp.len()].0);
            assert!(
                (a.end() - b.start()).length() < EPS,
                "chained loop {li} breaks between piece {k} and the next: {a:?} then {b:?}"
            );
            if let (Seg::Line { a: a0, b: a1 }, Seg::Line { a: b0, b: b1 }) = (a, b) {
                let (da, db) = (a1 - a0, b1 - b0);
                assert!(
                    da.x * db.y - da.y * db.x != 0.0 || da.dot(db) >= 0.0,
                    "chained loop {li} doubles back along itself between piece {k} and the \
                     next: {a:?} then {b:?}; the two runs overlap, so the loop encloses a \
                     zero-width spur that no face can be triangulated from"
                );
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kernel::sketch::Sketch;

    fn area(loops: &[Vec<Seg>]) -> f64 {
        loops.iter().map(|l| loop_area(l)).sum()
    }

    fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Vec<Seg> {
        Sketch::rectangle((x0 + x1) * 0.5, (y0 + y1) * 0.5, x1 - x0, y1 - y0)
            .loops
            .remove(0)
    }

    fn assert_conserved(a: &[Vec<Seg>], b: &[Vec<Seg>]) {
        let lhs = area(a) + area(b);
        let rhs = area(&region_union(a, b)) + area(&region_intersection(a, b));
        assert!(
            (lhs - rhs).abs() < 1e-2,
            "area not conserved: {lhs} vs {rhs}"
        );
    }

    /// A vertex the two regions already agree on, but which sits a fraction of
    /// a micron off the curve it is supposed to lie on, makes the sweep find a
    /// "crossing" a hair in from the end of the segment beside it. Splitting
    /// there leaves a piece shorter than the distance `chain_loops` welds
    /// endpoints with, and the two regions stop agreeing about where that
    /// vertex is -- the chain then runs off the end and the whole difference
    /// comes back empty. The cut has to be dropped in favour of the vertex.
    #[test]
    fn a_crossing_a_hair_from_a_vertex_does_not_split_off_a_sliver() {
        let mut cuts = vec![(1.8e-4 / 1.0, Vec2::new(9.99982, 0.0))];
        let pieces = split_seg(
            &Seg::Line {
                a: Vec2::new(10.0, 0.0),
                b: Vec2::new(9.0, 0.0),
            },
            &mut cuts,
        );
        assert_eq!(pieces.len(), 1, "sliver split: {pieces:?}");

        // Far enough in, the same cut is a real one.
        let mut cuts = vec![(0.5, Vec2::new(9.5, 0.0))];
        let pieces = split_seg(
            &Seg::Line {
                a: Vec2::new(10.0, 0.0),
                b: Vec2::new(9.0, 0.0),
            },
            &mut cuts,
        );
        assert_eq!(pieces.len(), 2, "real split lost: {pieces:?}");
    }

    #[test]
    fn difference_against_a_subset_sharing_most_of_its_boundary() {
        let outline = vec![rect(0.0, 0.0, 100.0, 100.0)];
        let notch = vec![rect(40.0, 80.0, 60.0, 120.0)];
        let below = region_difference(&outline, &notch);
        assert!(
            (area(&below) - 9600.0).abs() < 1e-2,
            "notched area {}",
            area(&below)
        );

        let cap = region_difference(&outline, &below);
        assert!(
            (area(&cap) - 400.0).abs() < 1e-2,
            "cap should be the 20x20 bite, got area {} from {} loop(s)",
            area(&cap),
            cap.len()
        );
        assert!(
            area(&region_difference(&below, &outline)).abs() < 1e-2,
            "subset minus superset must be empty"
        );
    }

    #[test]
    fn overlapping_rects() {
        let a = vec![rect(0.0, 0.0, 10.0, 10.0)];
        let b = vec![rect(5.0, 5.0, 15.0, 15.0)];
        assert!((area(&region_intersection(&a, &b)) - 25.0).abs() < 1e-3);
        assert!((area(&region_union(&a, &b)) - 175.0).abs() < 1e-3);
        assert!((area(&region_difference(&a, &b)) - 75.0).abs() < 1e-3);
        assert_conserved(&a, &b);
    }

    #[test]
    fn disjoint_rects_stay_separate() {
        let a = vec![rect(0.0, 0.0, 10.0, 10.0)];
        let b = vec![rect(20.0, 20.0, 30.0, 30.0)];
        let u = region_union(&a, &b);
        assert_eq!(u.len(), 2, "disjoint union should keep two loops");
        assert!(region_intersection(&a, &b).is_empty());
        assert!((area(&region_difference(&a, &b)) - 100.0).abs() < 1e-3);
        assert_conserved(&a, &b);
    }

    #[test]
    fn containment_both_directions() {
        let big = vec![rect(0.0, 0.0, 20.0, 20.0)];
        let small = vec![rect(5.0, 5.0, 10.0, 10.0)];
        assert!((area(&region_union(&big, &small)) - 400.0).abs() < 1e-3);
        assert!((area(&region_union(&small, &big)) - 400.0).abs() < 1e-3);
        assert!((area(&region_intersection(&big, &small)) - 25.0).abs() < 1e-3);
        let d = region_difference(&big, &small);
        assert_eq!(d.len(), 2, "difference should be outer + hole");
        assert!((area(&d) - 375.0).abs() < 1e-3, "hole must subtract");
        assert_conserved(&big, &small);
    }

    #[test]
    fn rounded_rect_against_circle_uses_arc_arc() {
        let a = vec![
            Sketch::rounded_rect(0.0, 0.0, 20.0, 20.0, 5.0)
                .loops
                .remove(0),
        ];
        let b = vec![Sketch::circle(11.0, 11.0, 4.0).loops.remove(0)];
        let corner = Seg::Arc {
            a: Vec2::new(10.0, 5.0),
            b: Vec2::new(5.0, 10.0),
            center: Vec2::new(5.0, 5.0),
            radius: 5.0,
            a0: 0.0,
            a1: std::f64::consts::FRAC_PI_2,
        };
        let hits: usize = b[0].iter().map(|s| seg_seg_points(&corner, s).len()).sum();
        assert_eq!(
            hits, 2,
            "arc/arc path must cross the corner arc exactly twice"
        );
        let i = region_intersection(&a, &b);
        assert!(!i.is_empty(), "arc/arc intersection produced nothing");
        assert!(area(&i) > 0.0 && area(&i) < std::f64::consts::PI * 16.0);
        assert_conserved(&a, &b);
    }

    #[test]
    fn circle_hole_in_rect() {
        let a = vec![rect(0.0, 0.0, 20.0, 20.0)];
        let b = vec![Sketch::circle(10.0, 10.0, 4.0).loops.remove(0)];
        let d = region_difference(&a, &b);
        let expect = 400.0 - std::f64::consts::PI * 16.0;
        assert!(
            (area(&d) - expect).abs() < 1e-2,
            "bore area {} vs {expect}",
            area(&d)
        );
    }
}

fn arc_covers(a0: f64, a1: f64, ang: f64) -> bool {
    let (lo, hi) = (a0.min(a1), a0.max(a1));
    let two_pi = std::f64::consts::TAU;
    let k = ((lo - ang) / two_pi).ceil();
    ang + k * two_pi <= hi + 1e-6
}

pub fn point_seg_distance(p: Vec2, seg: &Seg) -> f64 {
    match *seg {
        Seg::Line { a, b } => {
            let d = b - a;
            let l2 = d.dot(d);
            if l2 < 1e-12 {
                return (p - a).length();
            }
            let t = ((p - a).dot(d) / l2).clamp(0.0, 1.0);
            (p - (a + d * t)).length()
        }
        Seg::Arc {
            a,
            b,
            center,
            radius,
            a0,
            a1,
        } => {
            let v = p - center;
            if v.length() > 1e-9 && arc_covers(a0, a1, f64::atan2(v.y, v.x)) {
                (v.length() - radius).abs()
            } else {
                (p - a).length().min((p - b).length())
            }
        }
    }
}

fn extremal_points(seg: &Seg, toward: Vec2) -> Vec<Vec2> {
    match *seg {
        Seg::Line { .. } => Vec::new(),
        Seg::Arc {
            center,
            radius,
            a0,
            a1,
            ..
        } => {
            if toward.length() < 1e-9 {
                return Vec::new();
            }
            let u = toward.normalize();
            [u, -u]
                .iter()
                .map(|&d| center + d * radius)
                .filter(|q| {
                    let v = *q - center;
                    arc_covers(a0, a1, f64::atan2(v.y, v.x))
                })
                .collect()
        }
    }
}

pub fn seg_seg_distance(p: &Seg, q: &Seg) -> f64 {
    if !seg_seg_points(p, q).is_empty() {
        return 0.0;
    }
    let mut best = f64::INFINITY;
    for (x, y) in [(p, q), (q, p)] {
        for e in [x.start(), x.end()] {
            best = best.min(point_seg_distance(e, y));
        }
    }
    let toward = |s: &Seg, other: &Seg| -> Vec2 {
        match (*s, *other) {
            (Seg::Arc { .. }, Seg::Line { a, b }) => {
                let d = b - a;
                Vec2::new(-d.y, d.x)
            }
            (Seg::Arc { center: c0, .. }, Seg::Arc { center: c1, .. }) => c1 - c0,
            _ => Vec2::ZERO,
        }
    };
    for (x, y) in [(p, q), (q, p)] {
        for e in extremal_points(x, toward(x, y)) {
            best = best.min(point_seg_distance(e, y));
        }
    }
    best
}

pub fn min_loop_distance(a: &[Seg], b: &[Seg]) -> f64 {
    let _perf = perf::scope(perf::Metric::MinLoopDistance);
    let mut best = f64::INFINITY;
    for p in a {
        for q in b {
            best = best.min(seg_seg_distance(p, q));
            if best == 0.0 {
                return 0.0;
            }
        }
    }
    best
}

#[inline]
fn aabb_gap(x: Aabb, y: Aabb) -> f64 {
    let dx = (y.min.x - x.max.x).max(x.min.x - y.max.x).max(0.0);
    let dy = (y.min.y - x.max.y).max(x.min.y - y.max.y).max(0.0);
    (dx * dx + dy * dy).sqrt()
}

pub fn loops_within(a: &[Seg], b: &[Seg], limit: f64) -> bool {
    let _perf = perf::scope(perf::Metric::MinLoopDistance);
    if a.is_empty() || b.is_empty() || limit <= 0.0 {
        return false;
    }
    let boxes: Vec<Aabb> = b.iter().map(|s| s.bbox()).collect();
    let all = boxes.iter().fold(Aabb::EMPTY, |acc, x| acc.union(*x));
    for p in a {
        let pb = p.bbox();
        if aabb_gap(pb, all) >= limit {
            continue;
        }
        for (q, qb) in b.iter().zip(&boxes) {
            if aabb_gap(pb, *qb) < limit && seg_seg_distance(p, q) < limit {
                return true;
            }
        }
    }
    false
}

#[cfg(test)]
mod distance_tests {
    use super::*;
    use crate::kernel::sketch::Sketch;
    use std::f64::consts::PI;

    fn line(ax: f64, ay: f64, bx: f64, by: f64) -> Seg {
        Seg::Line {
            a: Vec2::new(ax, ay),
            b: Vec2::new(bx, by),
        }
    }

    #[test]
    fn parallel_lines_measure_their_gap() {
        let d = seg_seg_distance(&line(0.0, 0.0, 10.0, 0.0), &line(0.0, 3.0, 10.0, 3.0));
        assert!((d - 3.0).abs() < 1e-5, "{d}");
    }

    #[test]
    fn crossing_segments_are_zero() {
        let d = seg_seg_distance(&line(0.0, 0.0, 10.0, 0.0), &line(5.0, -1.0, 5.0, 1.0));
        assert_eq!(d, 0.0);
    }

    #[test]
    fn disjoint_collinear_segments_measure_the_gap() {
        let d = seg_seg_distance(&line(0.0, 0.0, 4.0, 0.0), &line(7.0, 0.0, 10.0, 0.0));
        assert!((d - 3.0).abs() < 1e-5, "{d}");
    }

    #[test]
    fn line_to_arc_uses_the_radial_closest_point() {
        let arc = Seg::Arc {
            a: Vec2::new(2.0, 0.0),
            b: Vec2::new(-2.0, 0.0),
            center: Vec2::ZERO,
            radius: 2.0,
            a0: 0.0,
            a1: PI,
        };
        let d = seg_seg_distance(&line(-5.0, 5.0, 5.0, 5.0), &arc);
        assert!((d - 3.0).abs() < 1e-5, "want 5 - 2 = 3, got {d}");
    }

    #[test]
    fn arc_to_arc_uses_the_centre_line() {
        let right = Seg::Arc {
            a: Vec2::new(0.0, 1.0),
            b: Vec2::new(0.0, -1.0),
            center: Vec2::ZERO,
            radius: 1.0,
            a0: PI / 2.0,
            a1: -PI / 2.0,
        };
        let left = Seg::Arc {
            a: Vec2::new(10.0, 1.0),
            b: Vec2::new(10.0, -1.0),
            center: Vec2::new(10.0, 0.0),
            radius: 1.0,
            a0: PI / 2.0,
            a1: 1.5 * PI,
        };
        let d = seg_seg_distance(&right, &left);
        assert!((d - 8.0).abs() < 1e-4, "want 10 - 1 - 1 = 8, got {d}");
    }

    #[test]
    fn nested_loops_report_the_boundary_gap() {
        let outer = Sketch::rectangle(0.0, 0.0, 20.0, 20.0).loops[0].clone();
        let inner = Sketch::rectangle(0.0, 0.0, 10.0, 10.0).loops[0].clone();
        let d = min_loop_distance(&inner, &outer);
        assert!((d - 5.0).abs() < 1e-5, "want 5, got {d}");
    }

    #[test]
    fn overlapping_loops_are_zero() {
        let a = Sketch::rectangle(0.0, 0.0, 20.0, 20.0).loops[0].clone();
        let b = Sketch::rectangle(15.0, 0.0, 20.0, 20.0).loops[0].clone();
        assert_eq!(min_loop_distance(&a, &b), 0.0);
    }

    #[test]
    fn rounded_island_in_a_square_measures_from_the_arc() {
        let outer = Sketch::rectangle(0.0, 0.0, 30.0, 30.0).loops[0].clone();
        let island = Sketch::rounded_rect(0.0, 0.0, 10.0, 10.0, 2.0).loops[0].clone();
        let d = min_loop_distance(&island, &outer);
        assert!((d - 10.0).abs() < 1e-5, "want 15 - 5 = 10, got {d}");
    }
}

#[cfg(test)]
mod prune_verification {
    use super::*;

    static VERIFY_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    struct Guard(#[allow(dead_code)] std::sync::MutexGuard<'static, ()>);

    impl Drop for Guard {
        fn drop(&mut self) {
            set_verify_prune(false);
        }
    }

    fn verifying() -> Guard {
        let held = VERIFY_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        set_verify_prune(true);
        Guard(held)
    }

    #[test]
    fn a_real_build_keeps_every_crossing_the_boxes_pruned() {
        let _g = verifying();
        for p in [
            crate::gridfinity::Params::rect(1, 1),
            crate::gridfinity::Params::rect(3, 2),
            crate::gridfinity::Params {
                inner_walls: vec![crate::gridfinity::InnerWall {
                    x1: 90.0,
                    y1: 30.0,
                    x2: 40.0,
                    y2: 50.0,
                    width: 3.0,
                    height: None,
                }],
                ..crate::gridfinity::Params::default()
            },
        ] {
            crate::gridfinity::build(&p);
        }
    }

    #[test]
    fn the_verification_pass_actually_runs_when_enabled() {
        let _g = verifying();
        let apart = |x: f64| {
            vec![vec![vec![
                Seg::Line {
                    a: Vec2::new(x, 0.0),
                    b: Vec2::new(x + 1.0, 0.0),
                },
                Seg::Line {
                    a: Vec2::new(x + 1.0, 0.0),
                    b: Vec2::new(x + 1.0, 1.0),
                },
                Seg::Line {
                    a: Vec2::new(x + 1.0, 1.0),
                    b: Vec2::new(x, 0.0),
                },
            ]]]
        };
        let mut regions = apart(0.0);
        regions.extend(apart(50.0));
        presplit_regions(&regions);
        assert!(
            verify_prune(),
            "the flag must still be set inside the scope"
        );
    }
}
