//! Siting the rolling ball. A `Corner` is one blended edge's solved geometry --
//! its two faces, its radius, and a `CornerEnd` at each vertex holding the ball
//! centre there and the two points where the ball touches down on the faces.
//! `solve` produces one per requested edge from the solid alone, and
//! `reconcile_shared_ends` then makes the two that meet at a vertex agree on
//! their shared point exactly, which is what the rest of the pipeline builds on:
//! every surface, runout and rebuilt loop downstream is expressed in these
//! points, so a disagreement here is a crack there.

use std::collections::HashMap;

use crate::kernel::curvedge::loop_edge_dir;
use crate::kernel::math::Vec3;
use crate::kernel::topo::{EdgeFaces, EdgeId, Solid};

use super::{MAX_JOIN_KINK, NON_FINITE_NORMAL, join_agree};

#[derive(Clone, Copy)]
pub(super) struct CornerEnd {
    pub v: usize,
    pub cv: Vec3,
    pub ta_p: Vec3,
    pub tb_p: Vec3,
}

#[derive(Clone, Copy)]
pub(super) struct Corner {
    pub e: EdgeId,
    pub fa: usize,
    pub fb: usize,
    pub r: f32,
    pub ma: Vec3,
    pub na0: Vec3,
    pub fwd_a: bool,
    pub ends: [CornerEnd; 2],
}

/// Maps each requested `(edge, radius)` to the ball sited at both of that edge's
/// vertices: the centre a distance `r` off each of the two faces, and the
/// touchdown on each face, that centre offset back along the face's outward
/// normal there. Returns one `Corner` per request in ascending edge id, so the
/// output is a function of the request set and not of `want`'s hash order.
/// Errors when a requested edge's faces are parallel at its midpoint or the
/// radius is not positive, since no ball sits in that corner.
///
/// Which side of the edge the ball sits on is decided **locally**, from the
/// direction `fa`'s material lies in at that edge: the orientation invariant says
/// a loop keeps its face's material on the left, so that direction is
/// `outward normal x edge tangent`. Reading it off `face_centroid` instead only
/// works for a face whose centroid is inside it -- an L-shaped cavity floor, or
/// any floor with an opening's mouth in it, pulls the centroid far enough to flip
/// the choice on one edge of a chain, which lands that edge's tangent points 2r
/// from its neighbour's and tears the loop open. The side is chosen once from the
/// normals at the *curve's* midpoint, not the chord's: the two agree on a line,
/// but on a semicircle the chord midpoint is the circle's centre and every normal
/// there is meaningless.
pub(super) fn solve(
    solid: &Solid,
    want: &HashMap<EdgeId, f32>,
    edge_faces: &EdgeFaces,
) -> Result<Vec<Corner>, String> {
    let face_outward = |fid: usize, p: Vec3| -> Vec3 {
        let f = &solid.faces[fid];
        let n = f.surface.normal(f.surface.project(p));
        if f.sense { n } else { -n }
    };

    let mut want_sorted: Vec<EdgeId> = want.keys().copied().collect();
    want_sorted.sort_unstable();
    let mut corners: Vec<Corner> = Vec::with_capacity(want.len());
    for &e in &want_sorted {
        let ed = solid.edges[e];
        let (fa, fb) = (edge_faces[e][0], edge_faces[e][1]);
        let (p0, p1) = (solid.verts[ed.v0].point, solid.verts[ed.v1].point);
        let t_mid = (ed.t0 + ed.t1) * 0.5;
        let mid = ed.curve.point(t_mid);
        let r = want[&e];
        let na_mid = face_outward(fa, mid);
        let nb_mid = face_outward(fb, mid);
        let sin_mid = na_mid.cross(nb_mid).length();
        if sin_mid < 1e-6 || r <= 0.0 {
            return Err(format!(
                "blend: edge {e} degenerate (parallel faces or r≤0)"
            ));
        }
        let fwd_a = loop_edge_dir(solid, fa, e);
        let rising = if ed.t1 >= ed.t0 { 1.0 } else { -1.0 };
        let tan_mid = (ed.curve.tangent(t_mid) * rising).normalize_or_zero();
        let along_a = if fwd_a { tan_mid } else { -tan_mid };
        let into_a = na_mid.cross(along_a);
        let ta_plus = mid + r * (na_mid + nb_mid) / sin_mid - r * na_mid;
        let ta_minus = mid - r * (na_mid + nb_mid) / sin_mid + r * na_mid;
        let s = if into_a.dot(ta_minus - mid) > into_a.dot(ta_plus - mid) {
            -1.0
        } else {
            1.0
        };
        let (na0, nb0) = (face_outward(fa, p0), face_outward(fb, p0));
        let (na1, nb1) = (face_outward(fa, p1), face_outward(fb, p1));
        for (which, fid, n) in [(0, fa, na0), (0, fb, nb0), (1, fa, na1), (1, fb, nb1)] {
            assert!(
                n.is_finite(),
                "{NON_FINITE_NORMAL}: face {fid} at edge {e}'s v{which} ({n:?}), surface {:?}; \
                 a NaN normal poisons the ball centre and only surfaces as a non-finite vertex \
                 in the builder, so it is named here",
                solid.faces[fid].surface
            );
        }
        let (ma0, mb0) = (s * na0, s * nb0);
        let (ma1, mb1) = (s * na1, s * nb1);
        let sin0 = ma0.cross(mb0).length().max(1e-9);
        let sin1 = ma1.cross(mb1).length().max(1e-9);
        let cv0 = p0 + r * (ma0 + mb0) / sin0;
        let cv1 = p1 + r * (ma1 + mb1) / sin1;
        let ta_p0 = cv0 - r * ma0;
        let ta_p1 = cv1 - r * ma1;
        let tb_p0 = cv0 - r * mb0;
        let tb_p1 = cv1 - r * mb1;
        corners.push(Corner {
            e,
            fa,
            fb,
            r,
            ma: ma0,
            na0,
            fwd_a,
            ends: [
                CornerEnd {
                    v: ed.v0,
                    cv: cv0,
                    ta_p: ta_p0,
                    tb_p: tb_p0,
                },
                CornerEnd {
                    v: ed.v1,
                    cv: cv1,
                    ta_p: ta_p1,
                    tb_p: tb_p1,
                },
            ],
        });
    }
    Ok(corners)
}

/// Rewrites `corners` in place so that where two blends meet at a vertex, both
/// carry bit-identical values for the ball centre and for each of the two
/// touchdowns; corners meeting nothing are left as `solve` produced them. Errors
/// when the two edges at a shared vertex do not have exactly one face in common,
/// and asserts that the values it is about to unify already agree to within
/// `join_agree` -- past that the two blends do not share a tangent there, and
/// unifying them would weld a visible kink shut rather than close float noise.
///
/// Each blend derives that point from its own faces' normals; along a
/// tangent-continuous chain those are equal in exact arithmetic and differ in the
/// last bits in f32, which puts the answers a few tenths of a micron apart --
/// above `topo`'s weld quantum, so the builder interns two vertices and the face
/// they both border is left with an open loop. Deriving the point once and
/// handing it to both is what closes it; no weld tolerance can, because the gap
/// is real. The shared face names one touchdown and the two tangent neighbours,
/// which share a normal there, name the other. The lower edge id wins, so the
/// answer does not depend on iteration order; averaging would invent a third
/// point neither blend built.
pub(super) fn reconcile_shared_ends(
    corners: &mut [Corner],
    vertex_blends: &super::chain::VertexBlends,
) -> Result<(), String> {
    let mut at: HashMap<usize, Vec<(usize, usize)>> = HashMap::new();
    for (i, c) in corners.iter().enumerate() {
        for (k, end) in c.ends.iter().enumerate() {
            at.entry(end.v).or_default().push((i, k));
        }
    }
    let mut vs: Vec<usize> = at.keys().copied().collect();
    vs.sort_unstable();
    for v in vs {
        let slots = at[&v].clone();
        if slots.len() < 2 {
            continue;
        }
        assert!(
            slots.len() == 2 && vertex_blends[&v].len() == 2,
            "blend chain: vertex {v} carries {} blended edge ends (want 2); \
             a chain is a path, not a junction",
            slots.len()
        );
        let ((i0, k0), (i1, k1)) = (slots[0], slots[1]);
        if i0 == i1 {
            let d = (corners[i0].ends[k0].cv - corners[i0].ends[k1].cv).length();
            assert!(
                d <= join_agree(corners[i0].r),
                "blend chain: closed edge {} disagrees with itself at vertex {v} by {d:.3e} mm; \
                 both ends came off the same faces and the same arithmetic",
                corners[i0].e
            );
            continue;
        }
        let (c0, c1) = (corners[i0], corners[i1]);
        let shared: Vec<usize> = [c0.fa, c0.fb]
            .into_iter()
            .filter(|f| *f == c1.fa || *f == c1.fb)
            .collect();
        if shared.len() != 1 {
            return Err(format!(
                "blend chain: edges {} and {} meet at vertex {v} across {} shared face(s) \
                 (want 1); faces {:?} and {:?}",
                c0.e,
                c1.e,
                shared.len(),
                (c0.fa, c0.fb),
                (c1.fa, c1.fb)
            ));
        }
        let s = shared[0];
        let on_shared = |c: &Corner, k: usize| {
            if c.fa == s {
                c.ends[k].ta_p
            } else {
                c.ends[k].tb_p
            }
        };
        let on_tangent = |c: &Corner, k: usize| {
            if c.fa == s {
                c.ends[k].tb_p
            } else {
                c.ends[k].ta_p
            }
        };
        for (name, a, b) in [
            ("ball centre", c0.ends[k0].cv, c1.ends[k1].cv),
            (
                "touchdown on the shared face",
                on_shared(&c0, k0),
                on_shared(&c1, k1),
            ),
            (
                "touchdown on the tangent face",
                on_tangent(&c0, k0),
                on_tangent(&c1, k1),
            ),
        ] {
            let d = (a - b).length();
            let bound = join_agree(c0.r.max(c1.r));
            assert!(
                d <= bound,
                "blend chain: edges {} and {} put the {name} at vertex {v} {d:.3e} mm apart \
                 ({a:?} vs {b:?}), over the {bound:.3e} mm a {MAX_JOIN_KINK} rad kink allows; \
                 they do not share a tangent there",
                c0.e,
                c1.e
            );
        }
        let (cv, ts, tt) = (c0.ends[k0].cv, on_shared(&c0, k0), on_tangent(&c0, k0));
        let c = &mut corners[i1];
        c.ends[k1].cv = cv;
        if c.fa == s {
            c.ends[k1].ta_p = ts;
            c.ends[k1].tb_p = tt;
        } else {
            c.ends[k1].tb_p = ts;
            c.ends[k1].ta_p = tt;
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use crate::kernel::math::Vec3;

    #[test]
    fn rolling_ball_corner_math() {
        let p = Vec3::new(10.0, 0.0, 0.0);
        let ma = Vec3::new(0.0, 0.0, 1.0);
        let mb = Vec3::new(-1.0, 0.0, 0.0);
        let r = 2.0_f32;
        let sin_theta = ma.cross(mb).length();
        let c = p + r * (ma + mb) / sin_theta;
        assert!(
            (c - Vec3::new(8.0, 0.0, 2.0)).length() < 1e-4,
            "ball centre {c}"
        );
        let ta = c - r * ma;
        let tb = c - r * mb;
        assert!(
            (ta - Vec3::new(8.0, 0.0, 0.0)).length() < 1e-4,
            "floor tangent {ta}"
        );
        assert!(
            (tb - Vec3::new(10.0, 0.0, 2.0)).length() < 1e-4,
            "wall tangent {tb}"
        );
    }
}
