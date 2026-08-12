//! How a blend chain's end is closed off: the `RunoutEnd` choice itself, the
//! `Runout` record the rebuild reads it back through, and the tests that decide
//! between them.
//!
//! The three ends differ in who owns the trim curve the blend stops on.
//! `Absorb` gives it to an existing face beyond the chain, which trims its two
//! edges back to the tangent points and splices the curve between them. `Cap`
//! emits a new face for it, which is what an opening's mouth needs: there the
//! corner is *void* rather than material -- the wall stops and the blend's
//! quarter-round cross-section is left standing in mid-air -- so no existing face
//! can take it, and the two neighbours either side keep their vertex and have
//! their edge split at the tangent point instead. `Flat` stops the blend in its
//! own last cross-section and owns everything itself.
//!
//! They are tried in that order, which is the order of how well they hide the
//! termination, and only the first two can fail to apply -- so what the order
//! decides is how good the end looks, not whether there is one.

use std::collections::HashMap;

use crate::kernel::curvedge::{CurvEdge, as_plane};
use crate::kernel::math::Vec3;
use crate::kernel::topo::{Edge, EdgeFaces, EdgeId, Solid};

use super::ON_EDGE;
use super::query::{across_at, coplanar, dist_to_curve, faces_at_vertex};

#[derive(Clone, Copy, PartialEq)]
pub(super) enum RunoutEnd {
    Absorb { face: usize },
    Cap { fa_side: usize, fb_side: usize },
    Flat { away: Vec3 },
}

#[derive(Clone, Copy)]
pub(super) struct Runout {
    pub end: RunoutEnd,
    pub corner: Vec3,
    pub arc: CurvEdge,
    pub ta_p: Vec3,
    pub tb_p: Vec3,
    pub fa: usize,
    pub fb: usize,
}

pub(super) type Runouts = HashMap<usize, Runout>;

impl Runout {
    /// The face that owns this runout's trim curve when the end is `Absorb`, and
    /// `None` for the two ends that own their curve themselves.
    pub fn absorbing(&self) -> Option<usize> {
        match self.end {
            RunoutEnd::Absorb { face } => Some(face),
            RunoutEnd::Cap { .. } | RunoutEnd::Flat { .. } => None,
        }
    }

    /// Whether the blend stops in its own last cross-section rather than running
    /// into any face.
    pub fn is_flat(&self) -> bool {
        matches!(self.end, RunoutEnd::Flat { .. })
    }

    /// The touchdown that lands **on** `ed` -- within `ON_EDGE` of its
    /// supporting curve and strictly between its two vertices -- taking `ta_p`
    /// first if both do, and `None` when neither does or `ed` is degenerate.
    /// It reads only the edge, never the face asking, so the face that retreats
    /// its endpoint to this point and the face across the edge that splits there
    /// compute the same point and their results weld.
    pub fn on_edge(&self, ed: &Edge, solid: &Solid, v: usize) -> Option<Vec3> {
        let far = if ed.v0 == v { ed.v1 } else { ed.v0 };
        let (from, to) = (solid.verts[v].point, solid.verts[far].point);
        let along = to - from;
        let len_sq = along.length_squared();
        if len_sq <= 0.0 {
            return None;
        }
        [self.ta_p, self.tb_p].into_iter().find(|&p| {
            dist_to_curve(p, ed, solid) <= ON_EDGE
                && (1e-4..=1.0 - 1e-4).contains(&((p - from).dot(along) / len_sq))
        })
    }

    /// Where face `fi` must split the edge whose faces are `edge_faces`, given a
    /// `Cap` end: the touchdown on whichever of the blend's two faces that edge
    /// also borders, and `None` when `fi` is not one of the cap's two sides, when
    /// the edge borders neither blend face, or when the end is not a cap.
    pub fn cap_split(&self, fi: usize, edge_faces: &[usize]) -> Option<Vec3> {
        match self.end {
            RunoutEnd::Cap { fa_side, fb_side } => {
                if fi == fa_side && edge_faces.contains(&self.fa) {
                    Some(self.ta_p)
                } else if fi == fb_side && edge_faces.contains(&self.fb) {
                    Some(self.tb_p)
                } else {
                    None
                }
            }
            RunoutEnd::Absorb { .. } | RunoutEnd::Flat { .. } => None,
        }
    }

    /// Whether face `fi` keeps the original corner vertex where this runout
    /// ends: true for a face that merely touches a `Cap` or `Flat` end, since
    /// those leave the corner standing, and false for the blend's own two faces
    /// and for every face of an `Absorb`, which retreat to a touchdown instead.
    pub fn keeps_corner(&self, fi: usize) -> bool {
        matches!(self.end, RunoutEnd::Cap { .. } | RunoutEnd::Flat { .. })
            && fi != self.fa
            && fi != self.fb
    }
}

/// Chooses how the chain ending at vertex `v` along edge `e`, blended between
/// faces `fa` and `fb`, is closed off. Considers the faces at `v` other than the
/// blend's own two and anything coplanar with them, and returns `Absorb` when
/// exactly one of those can take the trim curve, `Cap` when the faces either side
/// of the corner are a distinct coplanar pair, and `Flat`, pointing `away`, when
/// neither holds -- so it always yields an end unless `land` itself errors.
/// `land` maps a candidate's plane to where the blend's two touchdowns run out
/// on it, and is called only for the planar candidates actually in the running.
/// `Flat` is always available because the ball's two touchdowns and the corner
/// they retreat from are three points of one plane -- the plane of the ball's own
/// connect arc -- whatever the blend rolled along; it is the end of a fillet that
/// runs out of wall rather than into a face, and it retreats nothing.
///
/// Several candidates in one plane are not an ambiguity about *where* the blend
/// ends: the bin's outer wall is cut into bands by the peg profile and three of
/// them meet at the pinch a wall opening leaves, so the plane is settled and only
/// the owner of the trim curve is open -- which is why the many-candidate arm
/// tests containment as well as fit, and why landing in none of the bands means
/// the corner is void and wants a cap. A cap needs only its own two sides
/// coplanar, not every candidate: a corner can offer a third face going off in
/// its own direction and still be perfectly cappable.
#[allow(clippy::too_many_arguments)]
pub(super) fn plan_runout_end(
    solid: &Solid,
    v: usize,
    e: EdgeId,
    fa: usize,
    fb: usize,
    ef: &EdgeFaces,
    away: Vec3,
    land: impl Fn((Vec3, Vec3)) -> Result<(Vec3, Vec3), String>,
) -> Result<RunoutEnd, String> {
    let flat = RunoutEnd::Flat { away };
    let mut cands = Vec::new();
    for fi in faces_at_vertex(solid, v) {
        if fi == fa
            || fi == fb
            || coplanar(&solid.faces[fi].surface, &solid.faces[fa].surface)
            || coplanar(&solid.faces[fi].surface, &solid.faces[fb].surface)
        {
            continue;
        }
        cands.push(fi);
    }
    if cands.is_empty() {
        return Ok(flat);
    }
    let cap_at = |cands: &[usize]| -> Option<RunoutEnd> {
        let pick = |xs: &[usize]| -> Option<usize> {
            let hit: Vec<usize> = xs.iter().copied().filter(|c| cands.contains(c)).collect();
            (hit.len() == 1).then(|| hit[0])
        };
        let a = pick(&across_at(solid, v, fa, e, ef))?;
        let b = pick(&across_at(solid, v, fb, e, ef))?;
        (a != b && coplanar(&solid.faces[a].surface, &solid.faces[b].surface)).then_some(
            RunoutEnd::Cap {
                fa_side: a,
                fb_side: b,
            },
        )
    };

    if cands.len() == 1 {
        if let Some((ta, tb)) = as_plane(&solid.faces[cands[0]].surface)
            .map(&land)
            .transpose()?
            && absorb_fits(solid, cands[0], v, ta, tb)
        {
            return Ok(RunoutEnd::Absorb { face: cands[0] });
        }
        return Ok(cap_at(&cands).unwrap_or(flat));
    }
    let plane = as_plane(&solid.faces[cands[0]].surface);
    let one_plane = cands[1..]
        .iter()
        .all(|&c| coplanar(&solid.faces[c].surface, &solid.faces[cands[0]].surface));
    if let (true, Some(plane)) = (one_plane, plane) {
        let (ta, tb) = land(plane)?;
        let at = (ta + tb) * 0.5;
        let inside: Vec<usize> = cands
            .iter()
            .copied()
            .filter(|&c| planar_face_contains(solid, c, at) && absorb_fits(solid, c, v, ta, tb))
            .collect();
        if inside.len() == 1 {
            return Ok(RunoutEnd::Absorb { face: inside[0] });
        }
        if inside.is_empty()
            && let Some(cap) = cap_at(&cands)
        {
            return Ok(cap);
        }
    }
    Ok(cap_at(&cands).unwrap_or(flat))
}

/// Whether face `ft` can absorb a blend that retreats to `ta` and `tb`: true
/// when exactly two of `ft`'s edges end at `v` and each of them, matched to
/// whichever touchdown lies nearer its curve, retreats to a point at or before
/// its far vertex. Past that far end the face ran out before the blend did, and
/// the spliced loop doubles back over ground the blend never covered, reported
/// far downstream as a loop that does not close.
///
/// Being the only candidate is not the same as being able to take the curve, and
/// this is deliberately a question about the two **edges** rather than about the
/// face's area: asking `planar_face_contains` instead refuses runouts that land
/// *on* their face's boundary and absorb perfectly well.
fn absorb_fits(solid: &Solid, ft: usize, v: usize, ta: Vec3, tb: Vec3) -> bool {
    let mut seen = 0;
    for &(e, _) in solid.face_loops(ft).flatten() {
        let ed = solid.edges[e];
        if ed.v0 != v && ed.v1 != v {
            continue;
        }
        seen += 1;
        let far = if ed.v0 == v { ed.v1 } else { ed.v0 };
        let (from, to) = (solid.verts[v].point, solid.verts[far].point);
        let p = if dist_to_curve(ta, &ed, solid) <= dist_to_curve(tb, &ed, solid) {
            ta
        } else {
            tb
        };
        let along = to - from;
        let len_sq = along.length_squared();
        assert!(
            len_sq > 0.0,
            "blend runout: face {ft}'s edge {e} into vertex {v} runs from {from:?} back to \
             itself; an edge joins two distinct vertices"
        );
        if (p - from).dot(along) / len_sq > 1.0 {
            return false;
        }
    }
    seen == 2
}

/// Whether `p` lies inside face `f`: false unless `f` is planar and `p` is
/// within 1e-3 mm of its plane, and otherwise the even-odd parity of a ray from
/// `p` against every loop of `f` -- outer and inner together, so a point in a
/// hole comes back outside -- projected into the surface's own uv. A curved
/// edge contributes its midpoint as well as its endpoints, so a mostly-curved
/// loop does not collapse onto its chords. Boundary cases are whichever side the
/// parity falls on, which is why this decides ownership among candidates and
/// never decides fit.
fn planar_face_contains(solid: &Solid, f: usize, p: Vec3) -> bool {
    let surf = &solid.faces[f].surface;
    let Some((origin, n)) = as_plane(surf) else {
        return false;
    };
    if (p - origin).dot(n.normalize_or_zero()).abs() > 1e-3 {
        return false;
    }
    let uv = surf.project(p);
    let mut crossings = 0u32;
    let mut poly: Vec<(f32, f32)> = Vec::new();
    for lp in solid.face_loops(f) {
        poly.clear();
        for &(e, fwd) in lp {
            let ed = solid.edges[e];
            let a = if fwd { ed.v0 } else { ed.v1 };
            poly.push(surf.project(solid.verts[a].point));
            if !matches!(ed.curve, crate::kernel::geom::Curve::Line { .. }) {
                poly.push(surf.project(ed.curve.point((ed.t0 + ed.t1) * 0.5)));
            }
        }
        let m = poly.len();
        for i in 0..m {
            let (q0, q1) = (poly[i], poly[(i + 1) % m]);
            if (q0.1 > uv.1) != (q1.1 > uv.1)
                && q0.0 + (uv.1 - q0.1) / (q1.1 - q0.1) * (q1.0 - q0.0) > uv.0
            {
                crossings += 1;
            }
        }
    }
    crossings % 2 == 1
}
