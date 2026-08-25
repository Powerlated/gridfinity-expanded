//! Rolling-ball blending: the crate's two entry points and the tolerances every
//! phase of them shares.
//!
//! The phases below run in order, one per submodule: `chain` reads the request,
//! `corner` sites the ball at every blended edge end, `blend` builds the surfaces
//! and runs each chain's end out onto whatever `runout` chose for it (`section`
//! cuts the surface there), and `rebuild` rewrites the loops of every face the
//! blend touched. `query` holds the questions several of them ask of the solid.
//!
//! Three tolerances live here because every phase shares them. `END_AGREE` is how
//! far apart two faces' answers for one edge's moved endpoint may be. Zero is the
//! honest bound -- both run the same arithmetic on the same inputs -- but a
//! tangent point can be selected through different expressions along a chain, so
//! this allows the last bits of an `f64` near the model's 100 mm scale, where an
//! ulp is 1.4e-14 mm, and nothing more. It is well under `topo`'s weld quantum,
//! the distance at which a
//! disagreement starts interning two vertices, so anything it catches would have
//! cracked the solid open. `ON_EDGE` is how far off an edge's supporting curve a
//! touchdown may be and still count as standing *on* that edge: a touchdown that
//! lands on an edge does so exactly, being the ball centre offset by a face
//! normal where the two faces meet, so this is float noise only -- an order above
//! `END_AGREE` because it is reached through the normals rather than by the same
//! arithmetic twice, and eight orders below the thinnest wall the model will
//! build, so it cannot mistake one edge at a corner for another. Both were 1e-4
//! and 1e-3 when the kernel modelled in `f32`, where they really did have to
//! absorb the last bits of a float at 100 mm; five orders of that allowance was
//! the precision and is gone with it. `MAX_JOIN_KINK`
//! is the largest kink two edges of one chain may have at the vertex they share.
//! It is an **angle** because that is the quantity in question: the two blends
//! site the same ball there, so in exact arithmetic they agree, and a kink of `d`
//! radians moves the centre by about `r * d` -- stating it as a distance would
//! tighten with the radius exactly where a blend has the most room to absorb the
//! error. Half a degree is two orders above f64 normal noise at model scale and
//! two orders below the turn any corner makes, and at the largest fillet the
//! model offers it costs under 0.07 mm, which no printer resolves.

mod blend;
mod chain;
mod corner;
pub mod feasible;
mod query;
mod rebuild;
mod runout;
mod section;

use std::collections::HashMap;

use crate::kernel::topo::{Builder, EdgeFaces, EdgeId, Solid};

pub const NON_FINITE_NORMAL: &str = "blend: face normal is not finite";

const END_AGREE: f64 = 1e-9;

const ON_EDGE: f64 = 1e-8;

const MAX_JOIN_KINK: f64 = 0.5 * std::f64::consts::PI / 180.0;

/// Maps a blend radius to how far apart, in millimetres, two edges of one chain
/// may put the point they share. A kink of `MAX_JOIN_KINK` radians moves the ball
/// centre by about `r * MAX_JOIN_KINK`, so the bound scales with the radius, and
/// never falls below `END_AGREE`, which is the float noise floor a zero radius
/// would otherwise demand the answer beat.
fn join_agree(r: f64) -> f64 {
    (r * MAX_JOIN_KINK).max(END_AGREE)
}

/// Blends what it can of `blends` and returns `(solid, dropped, refusal)`: the
/// solid carrying every blend it built, the edge ids it could not build and left
/// sharp, and `None` when the whole request landed or the message the full
/// attempt refused with when it did not. Errors only for a request the solid
/// cannot answer at all -- an edge id out of range, or one not bordered by
/// exactly two faces -- so a caller that passes a well-formed request always gets
/// a part back, which is the point: a part with a sharp corner beats no part, and
/// `dropped` tells the caller which corners those are.
///
/// The solid it blends is not the input but the input with its outline's coplanar
/// seams fused, because those splits are not geometry and a runout has to be able
/// to retreat across one; the requested edges are held back from the fuse so
/// their ids and their two faces survive it. When nothing at all can be blended
/// it returns the **original** solid rather than the merged one: the merged solid
/// describes the same space but carries the edges the fuse left unreferenced, and
/// only the builder's own compaction clears those.
pub fn fillet_best_effort(
    solid: &Solid,
    blends: &[(EdgeId, f64)],
) -> Result<(Solid, Vec<EdgeId>, Option<String>), String> {
    const MAX_SPLIT: u32 = 3;

    if blends.is_empty() {
        return Ok((solid.clone(), Vec::new(), None));
    }
    let kept_edges: Vec<EdgeId> = blends.iter().map(|&(e, _)| e).collect();
    let original = solid;
    let merged = solid.merge_coplanar_faces(&kept_edges);
    let solid = &merged;
    let edge_faces = solid.edge_faces();
    check_edges(solid, blends.iter().map(|&(e, _)| e), &edge_faces)?;
    let refusal = match fillet_edges_with(solid, blends, &edge_faces) {
        Ok(s) => return Ok((s, Vec::new(), None)),
        Err(e) => e,
    };

    let mut kept: Vec<(EdgeId, f64)> = Vec::new();
    for run in chain::chains(solid, blends) {
        let salvaged = chain::salvage(solid, &edge_faces, &kept, &run, MAX_SPLIT);
        kept.extend(salvaged);
    }

    let dropped: Vec<EdgeId> = blends
        .iter()
        .map(|&(e, _)| e)
        .filter(|e| !kept.iter().any(|&(k, _)| k == *e))
        .collect();
    assert!(
        kept.len() + dropped.len() == blends.len(),
        "blend: {} requested but {} kept and {} dropped; salvage partitions the request, and a \
         blend falling out of both is a corner silently left sharp",
        blends.len(),
        kept.len(),
        dropped.len()
    );

    match fillet_edges_with(solid, &kept, &edge_faces) {
        Ok(s) => Ok((s, dropped, Some(refusal))),
        Err(e) => Ok((
            original.clone(),
            blends.iter().map(|&(e, _)| e).collect(),
            Some(format!("{refusal}; and after salvage: {e}")),
        )),
    }
}

/// Maps a solid and a set of `(edge, radius)` requests to the solid with every
/// one of them blended, or an error naming the first request that could not be:
/// all of them or none, with nothing left sharp and nothing approximated. The
/// input is blended as given -- no coplanar fuse, unlike `fillet_best_effort` --
/// so an edge id means the same thing on the way in and in any error out.
pub fn fillet_edges(solid: &Solid, blends: &[(EdgeId, f64)]) -> Result<Solid, String> {
    let edge_faces = solid.edge_faces();
    fillet_edges_with(solid, blends, &edge_faces)
}

/// Accepts when every edge in `edges` is in range for `solid` and borders
/// exactly two faces, and otherwise returns the message naming the first that is
/// not. Two faces is what a rolling ball needs to sit between; the rest of the
/// pipeline indexes `edge_faces[e][0]` and `[1]` on the strength of this check.
fn check_edges(
    solid: &Solid,
    edges: impl Iterator<Item = EdgeId>,
    edge_faces: &EdgeFaces,
) -> Result<(), String> {
    for e in edges {
        if e >= solid.edges.len() {
            return Err(format!("blend: edge {e} out of range"));
        }
        if edge_faces[e].len() != 2 {
            return Err(format!(
                "blend: edge {e} has {} faces (want 2)",
                edge_faces[e].len()
            ));
        }
    }
    Ok(())
}

/// Runs the whole pipeline over a caller-supplied `edge_faces`, so a salvage
/// bisection pays for that adjacency once rather than once per trial: request to
/// chain ends, ends to ball corners, corners to blend surfaces and runouts, and
/// those to a rebuilt solid. Returns that solid only once it validates and no
/// face's boundary crosses itself; every other outcome is an error message, and
/// `solid` is left untouched either way. Duplicated edge ids in `blends` collapse
/// to the last radius given, since the request is read into a map first.
fn fillet_edges_with(
    solid: &Solid,
    blends: &[(EdgeId, f64)],
    edge_faces: &EdgeFaces,
) -> Result<Solid, String> {
    let _perf = crate::kernel::perf::scope(crate::kernel::perf::Metric::FilletEdges);
    let mut want: HashMap<EdgeId, f64> = HashMap::with_capacity(blends.len());
    want.extend(blends.iter().copied());

    check_edges(solid, want.keys().copied(), edge_faces)?;

    let (vertex_blends, terminating) = chain::ends(solid, &want)?;
    let mut corners = corner::solve(solid, &want, edge_faces)?;
    corner::reconcile_shared_ends(solid, &mut corners, &vertex_blends)?;
    let (bm, runouts) = blend::build_all(solid, &corners, &terminating, edge_faces)?;

    let (touched, vinfo) = rebuild::touched_faces(solid, &bm, &want);
    let mut b = Builder::resume(solid, &touched);
    rebuild::faces(
        solid, &bm, &vinfo, &runouts, &want, &touched, edge_faces, &mut b,
    )?;
    let arc_used = rebuild::blend_faces(solid, &bm, &mut b);
    rebuild::runout_caps(solid, &runouts, &arc_used, &mut b);

    let s = b.build_compact_unvalidated();
    if let Err(e) = s.validate() {
        return Err(format!("blend: rebuilt solid invalid: {e}"));
    }
    for fid in 0..s.faces.len() {
        if let Some(x) = crate::kernel::audit::face_loop_self_crossing(&s, fid) {
            return Err(format!(
                "blend: face {fid}'s boundary crosses itself on {:?}: {x}",
                s.faces[fid].surface
            ));
        }
    }
    Ok(s)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kernel::build::extrude;
    use crate::kernel::math::Vec2;
    use crate::kernel::sketch::{Seg, Sketch};

    #[test]
    fn best_effort_matches_fillet_edges_when_nothing_fails() {
        let sk = Sketch::rounded_rect(0.0, 0.0, 20.0, 20.0, 4.0);
        let solid = extrude(&sk, 0.0, 5.0);
        let top: Vec<(EdgeId, f64)> = (0..solid.edges.len())
            .filter(|&e| {
                let ed = solid.edges[e];
                let (a, b) = (solid.vertex(ed.v0), solid.vertex(ed.v1));
                (a.z - 5.0).abs() < 1e-5 && (b.z - 5.0).abs() < 1e-5
            })
            .map(|e| (e, 1.0))
            .collect();
        assert!(!top.is_empty(), "expected a top rim to blend");

        let direct = fillet_edges(&solid, &top).expect("rim blends");
        let (best, dropped, _) = fillet_best_effort(&solid, &top).expect("sound input");
        assert!(
            dropped.is_empty(),
            "nothing should be dropped, got {dropped:?}"
        );
        assert_eq!(best.faces.len(), direct.faces.len());
        best.validate().expect("best-effort result is manifold");
    }

    /// A run of one face's boundary cut in two by a seam the coplanar fuse
    /// dissolves: both halves are requested, so both survive the fuse bordering
    /// the *same* pair of faces, and the chain across them is a straight
    /// continuation rather than a junction. The packed drawer produces dozens of
    /// these -- a divider's own vertex splits the compartment's floor run -- and
    /// refusing them cost `examples/drawer.toml` 24 of its 283 floor fillets.
    #[test]
    fn a_chain_running_on_across_a_fused_seam_still_blends() {
        let (x0, x1, y) = (-10.0, 10.0, -10.0);
        let segs = vec![
            Seg::Line {
                a: Vec2::new(x0, y),
                b: Vec2::new(0.0, y),
            },
            Seg::Line {
                a: Vec2::new(0.0, y),
                b: Vec2::new(x1, y),
            },
            Seg::Line {
                a: Vec2::new(x1, y),
                b: Vec2::new(x1, 10.0),
            },
            Seg::Line {
                a: Vec2::new(x1, 10.0),
                b: Vec2::new(x0, 10.0),
            },
            Seg::Line {
                a: Vec2::new(x0, 10.0),
                b: Vec2::new(x0, y),
            },
        ];
        let solid = extrude(&Sketch::single(segs), 0.0, 5.0);
        let run: Vec<(EdgeId, f64)> = (0..solid.edges.len())
            .filter(|&e| {
                let ed = solid.edges[e];
                let (a, b) = (solid.vertex(ed.v0), solid.vertex(ed.v1));
                [a, b]
                    .iter()
                    .all(|p| p.z.abs() < 1e-9 && (p.y - y).abs() < 1e-9)
            })
            .map(|e| (e, 1.0))
            .collect();
        assert_eq!(
            run.len(),
            2,
            "the split side must reach the blend as two collinear edges, got {run:?}"
        );

        let (best, dropped, refusal) = fillet_best_effort(&solid, &run).expect("sound input");
        assert!(
            dropped.is_empty() && refusal.is_none(),
            "a chain running straight on across a fused seam is not a junction, but \
             {dropped:?} was dropped and it refused with {refusal:?}"
        );
        best.validate().expect("blended result is manifold");
    }

    #[test]
    fn best_effort_still_reports_a_non_manifold_input() {
        let solid = extrude(&Sketch::rectangle(0.0, 0.0, 10.0, 10.0), 0.0, 5.0);
        let err = fillet_best_effort(&solid, &[(solid.edges.len() + 7, 1.0)])
            .expect_err("out-of-range edge must be reported");
        assert!(err.contains("out of range"), "unexpected error: {err}");
    }
}
