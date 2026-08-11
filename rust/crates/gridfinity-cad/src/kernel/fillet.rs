use crate::kernel::geom::{Curve, Surface};
use crate::kernel::math::Vec3;
use crate::kernel::topo::{Builder, EdgeId, Solid};
use std::collections::HashMap;

/// A face whose analytic normal evaluates to NaN at one of a blended edge's
/// endpoints. Pre-existing and undiagnosed; `fuzz_stripped_polyominoes` is what
/// reaches it. See `rust/CLAUDE.md`.
pub const NON_FINITE_NORMAL: &str = "blend: face normal is not finite";

/// How far apart two faces' answers for one edge's moved endpoint may be.
///
/// Zero would be the honest bound -- both faces run the same arithmetic on the
/// same inputs -- but the two tangent points can be selected through different
/// expressions along a chain, so this allows the last bits of an `f32` near the
/// model's 100 mm scale and nothing more. It is well under `topo`'s weld
/// quantum, which is the distance at which a disagreement starts interning two
/// vertices, so anything this catches would have cracked the solid open.
const END_AGREE: f32 = 1e-4;

/// How far off an edge's supporting curve a blend's touchdown may be and still
/// count as standing *on* that edge.
///
/// A touchdown that lands on an edge does so exactly: it is the ball centre
/// offset by a face normal, and the edge is where the two faces the ball touches
/// meet, so the two expressions agree to the last bits of an `f32`. The bound is
/// therefore only float noise at the model's 100 mm scale, an order above
/// `END_AGREE` because the point is reached through the normals rather than by
/// the same arithmetic twice, and three orders below the thinnest wall the model
/// will build -- so it cannot mistake one edge at a corner for another.
const ON_EDGE: f32 = 1e-3;

#[derive(Clone, Copy)]
struct CurvEdge {
    curve: Curve,
    t0: f32,
    t1: f32,
}

#[derive(Clone)]
struct Fillet {
    ta: CurvEdge,
    tb: CurvEdge,
    ca0: CurvEdge,
    ca1: CurvEdge,
    ta_p0: Vec3,
    ta_p1: Vec3,
    tb_p0: Vec3,
    tb_p1: Vec3,
    surface: Surface,
    sense: bool,
    fwd_a: bool,
}

/// One end of a blended edge: the rolling ball's centre there and the two points
/// where it touches down, on `fa` and on `fb`.
#[derive(Clone, Copy)]
struct CornerEnd {
    v: usize,
    cv: Vec3,
    ta_p: Vec3,
    tb_p: Vec3,
}

/// A blended edge's corner geometry, before the ends it shares with the rest of
/// its chain have been reconciled.
#[derive(Clone, Copy)]
struct Corner {
    e: EdgeId,
    fa: usize,
    fb: usize,
    r: f32,
    ma: Vec3,
    na0: Vec3,
    fwd_a: bool,
    ends: [CornerEnd; 2],
}

/// The largest kink, in radians, two edges of one blend chain may have at the
/// vertex they share.
///
/// The two blends site the same rolling ball at that vertex, so in exact
/// arithmetic they agree; what separates their answers is the angle between the
/// two edges' tangents there, and a kink of `d` radians moves the ball centre by
/// about `r * d`. That is why the bound is an angle: it is the quantity actually
/// in question, and stating it as a distance would tighten with the radius
/// exactly where a blend has the most room to absorb the error.
///
/// Half a degree is two orders above the float noise in an `f32` normal at the
/// model's 100 mm scale and two orders below the turn any corner makes, so it
/// separates "the same tangent, computed twice" from "these edges do not belong
/// to one chain" with a decade to spare on each side. It also bounds what
/// reconciling costs the part: at the largest fillet the model offers, half a
/// degree is under 0.07 mm, which no printer resolves.
const MAX_JOIN_KINK: f32 = 0.5 * std::f32::consts::PI / 180.0;

/// What `MAX_JOIN_KINK` allows a blend of radius `r`, in millimetres, never
/// tighter than the last bits of an `f32` at the model's scale.
fn join_agree(r: f32) -> f32 {
    (r * MAX_JOIN_KINK).max(END_AGREE)
}

/// Make the two blends that meet at a vertex agree, exactly, on where they meet.
///
/// Each blend derives the meeting point from its own faces' normals at the
/// shared vertex. Along a tangent-continuous chain those normals are equal in
/// exact arithmetic and differ in the last bits in `f32`, which puts the two
/// answers up to a few tenths of a micron apart -- above `topo`'s weld quantum,
/// so the builder interns them as two vertices and the face they both border is
/// left with an open loop. Deriving the point once and handing it to both blends
/// is what closes it; no weld tolerance can, because the gap is real.
///
/// The two edges meeting at `v` share exactly one face -- a chain runs along the
/// boundary of one face and turns from one of its neighbours to the next -- so
/// the shared face names one touchdown point and the two tangent neighbours,
/// which have a common normal at `v`, name the other.
fn reconcile_shared_ends(
    corners: &mut [Corner],
    vertex_blends: &HashMap<usize, Vec<EdgeId>>,
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
        // A closed edge visits its own vertex twice. Both ends already came off
        // the same faces and the same arithmetic, so there is nothing to settle.
        if i0 == i1 {
            let d = (corners[i0].ends[k0].cv - corners[i0].ends[k1].cv).length();
            assert!(
                d <= join_agree(corners[i0].r),
                "blend chain: closed edge {} disagrees with itself at vertex {v} by {d:.3e} mm",
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
        // `ta_p` sits on `fa` and `tb_p` on `fb`, so which of the two is the
        // touchdown on the shared face follows from where that face landed.
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
        // The lower edge id wins, so the answer does not depend on iteration
        // order. Averaging would only invent a third point neither blend built.
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

pub fn fillet_best_effort(
    solid: &Solid,
    blends: &[(EdgeId, f32)],
) -> Result<(Solid, Vec<EdgeId>, Option<String>), String> {
    const MAX_SPLIT: u32 = 3;

    if blends.is_empty() {
        return Ok((solid.clone(), Vec::new(), None));
    }
    // The outline splits that band the flat walls are not geometry, and a runout
    // has to be able to retreat across one. Fusing them first is what gives it
    // somewhere to land; the blended edges are held back so their ids and their
    // two faces survive.
    let kept_edges: Vec<EdgeId> = blends.iter().map(|&(e, _)| e).collect();
    let original = solid;
    let merged = solid.merge_coplanar_faces(&kept_edges);
    let solid = &merged;
    let edge_faces = solid.edge_faces();
    for &(e, _) in blends {
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
    let refusal = match fillet_edges_with(solid, blends, &edge_faces) {
        Ok(s) => return Ok((s, Vec::new(), None)),
        Err(e) => e,
    };

    let mut kept: Vec<(EdgeId, f32)> = Vec::new();
    for chain in chains(solid, blends) {
        let salvaged = salvage(solid, &edge_faces, &kept, &chain, MAX_SPLIT);
        kept.extend(salvaged);
    }

    let dropped: Vec<EdgeId> = blends
        .iter()
        .map(|&(e, _)| e)
        .filter(|e| !kept.iter().any(|&(k, _)| k == *e))
        .collect();
    // Salvage partitions the request: every blend asked for is either made or
    // reported dropped. A blend that fell out of both is a corner silently left
    // sharp, which is the one outcome the report exists to make impossible.
    assert!(
        kept.len() + dropped.len() == blends.len(),
        "blend: {} requested but {} kept and {} dropped",
        blends.len(),
        kept.len(),
        dropped.len()
    );

    match fillet_edges_with(solid, &kept, &edge_faces) {
        Ok(s) => Ok((s, dropped, Some(refusal))),
        // Nothing could be blended, so hand back exactly what came in. The
        // merged solid describes the same space but carries the edges the fuse
        // left unreferenced, and only the builder's own compaction clears
        // those -- returning it here would hand the caller a solid whose
        // manifold check counts an edge no face uses.
        Err(e) => Ok((
            original.clone(),
            blends.iter().map(|&(e, _)| e).collect(),
            Some(format!("{refusal}; and after salvage: {e}")),
        )),
    }
}

fn salvage(
    solid: &Solid,
    ef: &crate::kernel::topo::EdgeFaces,
    base: &[(EdgeId, f32)],
    run: &[(EdgeId, f32)],
    depth: u32,
) -> Vec<(EdgeId, f32)> {
    if run.is_empty() {
        return Vec::new();
    }
    let mut trial = base.to_vec();
    trial.extend_from_slice(run);
    if fillet_edges_with(solid, &trial, ef).is_ok() {
        return run.to_vec();
    }
    if depth == 0 || run.len() < 2 {
        return Vec::new();
    }
    let mid = run.len() / 2;
    let head = salvage(solid, ef, base, &run[..mid], depth - 1);
    let mut base2 = base.to_vec();
    base2.extend_from_slice(&head);
    let tail = salvage(solid, ef, &base2, &run[mid..], depth - 1);
    let mut out = head;
    out.extend(tail);
    out
}

fn chains(solid: &Solid, blends: &[(EdgeId, f32)]) -> Vec<Vec<(EdgeId, f32)>> {
    let mut parent: Vec<usize> = (0..blends.len()).collect();
    fn find(parent: &mut Vec<usize>, i: usize) -> usize {
        if parent[i] != i {
            let r = find(parent, parent[i]);
            parent[i] = r;
        }
        parent[i]
    }
    let mut by_vertex: HashMap<usize, usize> = HashMap::new();
    for (i, &(e, _)) in blends.iter().enumerate() {
        if e >= solid.edges.len() {
            continue;
        }
        let ed = solid.edges[e];
        for v in [ed.v0, ed.v1] {
            match by_vertex.get(&v) {
                Some(&j) => {
                    let (a, b) = (find(&mut parent, i), find(&mut parent, j));
                    parent[a] = b;
                }
                None => {
                    by_vertex.insert(v, i);
                }
            }
        }
    }
    let mut groups: Vec<(usize, Vec<(EdgeId, f32)>)> = Vec::new();
    for (i, &b) in blends.iter().enumerate() {
        let root = find(&mut parent, i);
        match groups.iter_mut().find(|(r, _)| *r == root) {
            Some((_, v)) => v.push(b),
            None => groups.push((root, vec![b])),
        }
    }
    groups.into_iter().map(|(_, v)| v).collect()
}

pub fn fillet_edges(solid: &Solid, blends: &[(EdgeId, f32)]) -> Result<Solid, String> {
    let edge_faces = solid.edge_faces();
    fillet_edges_with(solid, blends, &edge_faces)
}

fn fillet_edges_with(
    solid: &Solid,
    blends: &[(EdgeId, f32)],
    edge_faces: &crate::kernel::topo::EdgeFaces,
) -> Result<Solid, String> {
    let _perf = crate::kernel::perf::scope(crate::kernel::perf::Metric::FilletEdges);
    let mut want: HashMap<EdgeId, f32> = HashMap::with_capacity(blends.len());
    want.extend(blends.iter().copied());

    for &e in want.keys() {
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

    let mut vertex_blends: HashMap<usize, Vec<EdgeId>> = HashMap::new();
    for &e in want.keys() {
        let ed = solid.edges[e];
        vertex_blends.entry(ed.v0).or_default().push(e);
        vertex_blends.entry(ed.v1).or_default().push(e);
    }
    let mut terminating: HashMap<usize, EdgeId> = HashMap::new();
    for (v, es) in &vertex_blends {
        match es.len() {
            2 => {}
            1 => {
                terminating.insert(*v, es[0]);
            }
            n => {
                return Err(format!(
                    "blend: vertex {v} has {n} blended edges (want 1 or 2; \
                     spherical corners unsupported)"
                ));
            }
        }
    }

    let face_outward = |fid: usize, p: Vec3| -> Vec3 {
        let f = &solid.faces[fid];
        let n = f.surface.normal(f.surface.project(p));
        if f.sense { n } else { -n }
    };

    let mut bm: HashMap<EdgeId, Fillet> = HashMap::with_capacity(want.len());
    let mut runouts: HashMap<usize, Runout> = HashMap::new();
    let mut want_sorted: Vec<EdgeId> = want.keys().copied().collect();
    want_sorted.sort_unstable();
    let mut corners: Vec<Corner> = Vec::with_capacity(want.len());
    for &e in &want_sorted {
        let ed = solid.edges[e];
        let (fa, fb) = (edge_faces[e][0], edge_faces[e][1]);
        let (p0, p1) = (solid.verts[ed.v0].point, solid.verts[ed.v1].point);
        // The *curve's* midpoint, not the chord's. They agree on a line, and on
        // a semicircle the chord midpoint is the circle's centre -- nowhere near
        // either surface, so every normal taken there is meaningless.
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
        // Which side of the edge the blend sits on is decided *locally*, from
        // the direction fa's material lies in at this edge. The orientation
        // invariant says a loop keeps its face's material on the left, so that
        // direction is `outward normal x edge tangent`. Reading it off
        // `face_centroid` instead only works for a face whose centroid is
        // inside it: an L-shaped cavity floor, and any floor with an opening's
        // mouth in it, pull the centroid far enough to flip the choice on one
        // edge of a chain, which lands that edge's tangent points 2r away from
        // its neighbour's and tears the loop open.
        let fwd_a = loop_edge_dir(solid, fa, e);
        // `tangent` differentiates in increasing t, and an edge whose stored
        // range runs backwards traverses v0 -> v1 the other way.
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
        // A non-finite normal poisons everything downstream -- `sin.max(1e-9)`
        // keeps a NaN cross product from being caught, the ball centre comes out
        // NaN, and the failure only surfaces much later as a non-finite vertex
        // in the builder. Name the surface that produced it instead.
        for (which, fid, n) in [(0, fa, na0), (0, fb, nb0), (1, fa, na1), (1, fb, nb1)] {
            assert!(
                n.is_finite(),
                "{NON_FINITE_NORMAL}: face {fid} at edge {e}'s v{which} ({n:?}), surface {:?}",
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

    reconcile_shared_ends(&mut corners, &vertex_blends)?;

    for &Corner {
        e,
        fa,
        fb,
        r,
        ma,
        na0,
        fwd_a,
        ends,
    } in corners.iter()
    {
        let ed = solid.edges[e];
        let (cv0, ta_p0, tb_p0) = (ends[0].cv, ends[0].ta_p, ends[0].tb_p);
        let (cv1, ta_p1, tb_p1) = (ends[1].cv, ends[1].ta_p, ends[1].tb_p);

        let plane_a = as_plane(&solid.faces[fa].surface);
        let plane_b = as_plane(&solid.faces[fb].surface);
        let cyl = as_cyl(&solid.faces[fa].surface).or_else(|| as_cyl(&solid.faces[fb].surface));
        let is_circle = matches!(ed.curve, Curve::Circle { .. });

        let mut blend = if plane_a.is_some() && plane_b.is_some() {
            build_cyl_blend(ed, cv0, cv1, ma, na0, ta_p0, ta_p1, tb_p0, tb_p1, r, fwd_a)?
        } else if cyl.is_some() && is_circle && (plane_a.is_some() || plane_b.is_some()) {
            build_torus_blend(
                ed,
                cv0,
                cv1,
                na0,
                ta_p0,
                ta_p1,
                tb_p0,
                tb_p1,
                r,
                cyl.expect("this arm is guarded by cyl.is_some()"),
                fwd_a,
            )?
        } else {
            return Err(format!(
                "blend: edge {e} pair not supported (only plane/plane or plane/coaxial-cylinder)"
            ));
        };

        for (at_v0, v) in [(true, ed.v0), (false, ed.v1)] {
            if terminating.get(&v) != Some(&e) {
                continue;
            }
            let (cv, tap, tbp) = if at_v0 {
                (cv0, ta_p0, tb_p0)
            } else {
                (cv1, ta_p1, tb_p1)
            };
            // Where the chain was heading when it stopped. It orients a flat
            // end's cap, and it settles a runout whose blend sits square in the
            // terminating plane. The chord from the blend's other ball centre to
            // this one is that direction on every blend the kernel builds -- a
            // straight one and an arc alike.
            let away = (cv - if at_v0 { cv1 } else { cv0 }).normalize_or_zero();
            let land = |plane| {
                runout_on(&blend.surface, cv, r, tap, tbp, plane, away).map(|(a, b, _)| (a, b))
            };
            let end = plan_runout_end(solid, v, e, fa, fb, edge_faces, away, land)?;
            let (ta_new, tb_new, arc) = match end {
                // A flat end trims nothing: the blend keeps the tangent points
                // and the connect arc it was already built with, and the cap is
                // the face across them.
                RunoutEnd::Flat { .. } => (tap, tbp, if at_v0 { blend.ca0 } else { blend.ca1 }),
                RunoutEnd::Absorb { face } | RunoutEnd::Cap { fa_side: face, .. } => {
                    let plane = as_plane(&solid.faces[face].surface).ok_or_else(|| {
                        format!(
                            "blend: runout face {face} at vertex {v} is not planar ({:?})",
                            solid.faces[face].surface
                        )
                    })?;
                    runout_on(&blend.surface, cv, r, tap, tbp, plane, away)?
                }
            };
            if at_v0 {
                blend.ta_p0 = ta_new;
                blend.tb_p0 = tb_new;
                blend.ca0 = arc;
            } else {
                blend.ta_p1 = ta_new;
                blend.tb_p1 = tb_new;
                blend.ca1 = arc;
            }
            runouts.insert(
                v,
                Runout {
                    end,
                    corner: solid.verts[v].point,
                    arc,
                    ta_p: ta_new,
                    tb_p: tb_new,
                    fa,
                    fb,
                },
            );
            // A torus blend's tangent curves are **circles**, and a runout moves
            // a touchdown *along* its own circle. The parameter range has to
            // follow it: a circle edge is emitted over the range it stores, so
            // leaving the old one behind draws the arc the blend used to span
            // and hands the builder an edge that misses its own vertex by the
            // distance the touchdown moved. A cylindrical blend's tangents are
            // lines, which carry no range worth the name.
            respan(&mut blend.ta, blend.ta_p0, blend.ta_p1);
            respan(&mut blend.tb, blend.tb_p0, blend.tb_p1);
        }
        bm.insert(e, blend);
    }

    let mut vinfo: HashMap<usize, (Vec3, Vec3)> = HashMap::with_capacity(bm.len() * 2);
    let mut vinfo_order: Vec<EdgeId> = bm.keys().copied().collect();
    vinfo_order.sort_unstable();
    for e in vinfo_order {
        let bld = &bm[&e];
        let ed = solid.edges[e];
        vinfo.insert(ed.v0, (bld.ta_p0, bld.tb_p0));
        vinfo.insert(ed.v1, (bld.ta_p1, bld.tb_p1));
    }

    let mut edge_moved = vec![false; solid.edges.len()];
    for (e, moved) in edge_moved.iter_mut().enumerate() {
        let ed = solid.edges[e];
        *moved = want.contains_key(&e) || vinfo.contains_key(&ed.v0) || vinfo.contains_key(&ed.v1);
    }
    let mut touched = vec![false; solid.faces.len()];
    for (fi, t) in touched.iter_mut().enumerate() {
        *t = solid
            .face_loops(fi)
            .any(|lp| lp.iter().any(|&(e, _)| edge_moved[e]));
    }

    let mut b = Builder::resume(solid, &touched);

    let mut loop_scratch: Vec<(EdgeId, bool)> = Vec::new();
    let mut items_scratch: Vec<Emitted> = Vec::new();
    let mut inner_ranges: Vec<usize> = Vec::new();
    // Every unblended edge's new endpoints, keyed by the edge and the original
    // vertex, so the second face to rebuild an edge is checked against the
    // first. See the assertion in `rebuild_loop`.
    let mut moved_ends: HashMap<(EdgeId, usize), Vec3> = HashMap::new();

    for (fi, &is_touched) in touched.iter().enumerate() {
        if !is_touched {
            b.copy_face(solid, fi);
            continue;
        }
        loop_scratch.clear();
        inner_ranges.clear();
        rebuild_loop(
            solid,
            &bm,
            &vinfo,
            &runouts,
            &want,
            fi,
            solid.outer_edges(fi),
            edge_faces,
            &mut b,
            &mut items_scratch,
            &mut loop_scratch,
            &mut moved_ends,
        )?;
        let outer_len = loop_scratch.len();
        for lp in solid.inner_loops(fi) {
            let before = loop_scratch.len();
            rebuild_loop(
                solid,
                &bm,
                &vinfo,
                &runouts,
                &want,
                fi,
                lp,
                edge_faces,
                &mut b,
                &mut items_scratch,
                &mut loop_scratch,
                &mut moved_ends,
            )?;
            inner_ranges.push(loop_scratch.len() - before);
        }
        let outer = &loop_scratch[..outer_len];
        let mut inners: Vec<&[(EdgeId, bool)]> = Vec::with_capacity(inner_ranges.len());
        let mut off = outer_len;
        for &len in &inner_ranges {
            inners.push(&loop_scratch[off..off + len]);
            off += len;
        }
        let (surface, sense) = (solid.faces[fi].surface, solid.faces[fi].sense);
        b.face_from(surface, sense, outer, &inners);
    }

    let mut blend_faces: Vec<usize> = Vec::with_capacity(bm.len());
    let mut blend_keys: Vec<EdgeId> = bm.keys().copied().collect();
    blend_keys.sort_unstable();
    // How the blend face traversed each capped end's trim curve. The cap shares
    // that edge and has to run the other way, which is what fixes its winding
    // against all three of its neighbours at once.
    let mut arc_used: HashMap<usize, (EdgeId, bool)> = HashMap::new();
    for k in blend_keys {
        let bld = &bm[&k];
        let e_ta = emit_curv(&mut b, bld.ta_p0, bld.ta_p1, bld.ta);
        let e_tb = emit_curv(&mut b, bld.tb_p0, bld.tb_p1, bld.tb);
        let e_ca0 = emit_curv(&mut b, bld.ta_p0, bld.tb_p0, bld.ca0);
        let e_ca1 = emit_curv(&mut b, bld.ta_p1, bld.tb_p1, bld.ca1);
        let lp: [(EdgeId, bool); 4] = if bld.fwd_a {
            [(e_ta.0, !e_ta.1), e_ca0, e_tb, (e_ca1.0, !e_ca1.1)]
        } else {
            [e_ta, e_ca1, (e_tb.0, !e_tb.1), (e_ca0.0, !e_ca0.1)]
        };
        let ed = solid.edges[k];
        let (used0, used1) = if bld.fwd_a {
            (e_ca0, (e_ca1.0, !e_ca1.1))
        } else {
            ((e_ca0.0, !e_ca0.1), e_ca1)
        };
        arc_used.insert(ed.v0, used0);
        arc_used.insert(ed.v1, used1);
        blend_faces.push(b.face_from(bld.surface, bld.sense, &lp, &[]));
    }

    // Where the chain died on an opening's mouth there was no face to take the
    // trim curve, so the cap is emitted here. Every one of its three edges is
    // interned by its endpoints -- the ellipse is the edge the blend face just
    // built, and the two stubs are the lines the neighbours emitted when their
    // edge was split at the tangent point -- so the cap pairs up without being
    // told who its neighbours are.
    let mut cap_vs: Vec<usize> = runouts.keys().copied().collect();
    cap_vs.sort_unstable();
    for v in cap_vs {
        let ro = runouts[&v];
        let (surface, sense) = match ro.end {
            RunoutEnd::Absorb { .. } => continue,
            RunoutEnd::Cap { fa_side, .. } => {
                let f = &solid.faces[fa_side];
                (f.surface, f.sense)
            }
            // A flat end carries no neighbour's surface: its plane is the one
            // through the corner and the two touchdowns, which is the plane of
            // the ball's own connect arc. `away` orients it -- the blend the cap
            // closes lies behind the cap, so the normal points along the
            // direction the chain was heading.
            RunoutEnd::Flat { away } => {
                let n = (ro.ta_p - ro.corner)
                    .cross(ro.tb_p - ro.corner)
                    .normalize_or_zero();
                let n = if n.dot(away) >= 0.0 { n } else { -n };
                assert!(
                    n.dot(away).abs() > 0.1,
                    "blend runout: the flat end's plane (normal {n:?}) is edge-on to the \
                     direction {away:?} the chain was heading, so it does not close the blend"
                );
                (Surface::plane(ro.corner, n), true)
            }
        };
        let (va, vc, vb) = (b.vertex(ro.ta_p), b.vertex(ro.corner), b.vertex(ro.tb_p));
        let arc = emit_curv(&mut b, ro.tb_p, ro.ta_p, ro.arc);
        let mut lp = vec![b.line(va, vc), b.line(vc, vb), arc];
        if arc_used.get(&v) == Some(&arc) {
            lp.reverse();
            for x in &mut lp {
                x.1 = !x.1;
            }
        }
        b.face_from(surface, sense, &lp, &[]);
    }

    let s = b.build_compact_unvalidated();
    if let Err(e) = s.validate() {
        return Err(format!("blend: rebuilt solid invalid: {e}"));
    }
    let _ = &blend_faces;
    for fid in 0..s.faces.len() {
        if crate::kernel::audit::face_loops_self_intersect(&s, fid) {
            return Err(format!("blend: face {fid}'s boundary crosses itself"));
        }
    }
    Ok(s)
}

fn loop_edge_dir(solid: &Solid, fid: usize, e: EdgeId) -> bool {
    for lp in solid.face_loops(fid) {
        for &(ee, f) in lp {
            if ee == e {
                return f;
            }
        }
    }
    true
}

fn as_plane(s: &Surface) -> Option<(Vec3, Vec3)> {
    if let Surface::Plane { origin, normal, .. } = s {
        Some((*origin, *normal))
    } else {
        None
    }
}

fn as_cyl(s: &Surface) -> Option<(Vec3, Vec3, f32)> {
    if let Surface::Cylinder {
        base, axis, radius, ..
    } = s
    {
        Some((*base, *axis, *radius))
    } else {
        None
    }
}

/// How a blend chain's end is closed off.
///
/// `Absorb` is the original case: one face beyond the chain owns the corner at
/// the vertex, so trimming its two edges back to the tangent points and
/// splicing the trim curve between them closes the gap.
///
/// `Cap` is what an opening's mouth needs. There the chain dies on a boundary
/// where the corner is *void* rather than material -- the wall stops and the
/// blend's quarter-round cross-section is left standing in mid-air -- so no
/// existing face can absorb the curve and one has to be emitted. The two
/// neighbours either side of the corner keep their vertex; each has its edge
/// split at the tangent point instead, and the piece from there to the corner
/// pairs with the new cap.
#[derive(Clone, Copy, PartialEq)]
enum RunoutEnd {
    Absorb {
        face: usize,
    },
    Cap {
        /// The neighbour across the fa-side edge, whose edge splits at `ta_p`.
        fa_side: usize,
        /// The neighbour across the fb-side edge, whose edge splits at `tb_p`.
        fb_side: usize,
    },
    /// Nothing at the corner can take the curve, so the blend simply **stops**
    /// there, closed by a flat face in its own last cross-section.
    ///
    /// This is the end of a fillet that runs out of wall rather than into a
    /// face, and it is the only end that always exists: the ball's two
    /// touchdowns and the corner they retreat from are three points of one
    /// plane -- the plane of the ball's own connect arc -- whatever the blend
    /// was rolling along. It is a worse *looking* end than absorbing into a
    /// neighbour, which is why it is the last resort and not the first, but it
    /// is a real end, and a commercial modeller's flat-ended fillet is the same
    /// construction.
    ///
    /// `away` is the direction the chain was heading at the corner, which is
    /// what orients the cap: the material it closes lies behind it.
    Flat {
        away: Vec3,
    },
}

#[derive(Clone, Copy)]
struct Runout {
    end: RunoutEnd,
    /// The corner the chain dies on. `Cap` keeps it; `Absorb` trims it away.
    corner: Vec3,
    arc: CurvEdge,
    ta_p: Vec3,
    tb_p: Vec3,
    fa: usize,
    fb: usize,
}

impl Runout {
    /// The face whose loop the trim curve is spliced into, if any. A capped
    /// runout has none -- the curve bounds the cap instead.
    fn absorbing(&self) -> Option<usize> {
        match self.end {
            RunoutEnd::Absorb { face } => Some(face),
            RunoutEnd::Cap { .. } | RunoutEnd::Flat { .. } => None,
        }
    }

    /// Whether this end closes the blend with a face of its own rather than
    /// folding it into a neighbour.
    fn is_flat(&self) -> bool {
        matches!(self.end, RunoutEnd::Flat { .. })
    }

    /// Where the edge `ed` running into the corner meets the blend, for a flat
    /// end -- the tangent point that lies **on that edge**, if either does.
    ///
    /// A flat end retreats no corner: it stops the blend where the chain
    /// stopped, and the two faces the blend trims reach their tangent points
    /// through their own loops. An edge is only involved when a touchdown
    /// happens to land on it, which is the ordinary case for the edge running
    /// up the wall out of the corner -- there the blend's tangent point stands
    /// part way along it, and the edge has to end (or split) there rather than
    /// at the corner.
    ///
    /// The answer is a property of the **edge**, not of the face asking: both
    /// faces sharing it run this same test on the same curve, so one retreating
    /// while the other splits still agree on the point.
    fn on_edge(&self, ed: &crate::kernel::topo::Edge, solid: &Solid, v: usize) -> Option<Vec3> {
        let far = if ed.v0 == v { ed.v1 } else { ed.v0 };
        let (from, to) = (solid.verts[v].point, solid.verts[far].point);
        let along = to - from;
        let len_sq = along.length_squared();
        // A closed edge -- a full circle, whose two ends are the same vertex --
        // has no far end to retreat towards, so there is no parameter to place a
        // touchdown at and nothing to split.
        if len_sq <= 0.0 {
            return None;
        }
        [self.ta_p, self.tb_p].into_iter().find(|&p| {
            // On the edge's own supporting curve, and strictly between the
            // corner and the far end -- at the ends it is the corner standing
            // still or the edge consumed whole, and neither is a split.
            dist_to_curve(p, ed, solid) <= ON_EDGE
                && (1e-4..=1.0 - 1e-4).contains(&((p - from).dot(along) / len_sq))
        })
    }

    /// Where face `fi`'s edge into the corner has to be split, for a capped
    /// runout. `None` when `fi` is not one of the two neighbours, or when the
    /// edge in hand is not the one that reaches the blend.
    fn cap_split(&self, fi: usize, edge_faces: &[usize]) -> Option<Vec3> {
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

    /// Whether face `fi` keeps the corner vertex where it is. Only the two
    /// blended faces retreat to their tangent points; a capped or flat runout
    /// leaves the corner standing for every other face that meets it, including
    /// the two the cap pairs with.
    fn keeps_corner(&self, fi: usize) -> bool {
        matches!(self.end, RunoutEnd::Cap { .. } | RunoutEnd::Flat { .. })
            && fi != self.fa
            && fi != self.fb
    }
}

fn faces_at_vertex(solid: &Solid, v: usize) -> Vec<usize> {
    let mut out = Vec::new();
    for fi in 0..solid.faces.len() {
        let mut hit = false;
        for lp in solid.face_loops(fi) {
            for &(e, _) in lp {
                let ed = solid.edges[e];
                if ed.v0 == v || ed.v1 == v {
                    hit = true;
                }
            }
        }
        if hit {
            out.push(fi);
        }
    }
    out
}

fn coplanar(x: &Surface, y: &Surface) -> bool {
    match (as_plane(x), as_plane(y)) {
        (Some((o0, n0)), Some((o1, n1))) => {
            let (n0, n1) = (n0.normalize_or_zero(), n1.normalize_or_zero());
            n0.cross(n1).length() < 1e-5 && (o1 - o0).dot(n0).abs() < 1e-4
        }
        _ => false,
    }
}

/// The face across `side`'s edge at `v` -- the neighbour the blend meets when
/// it runs off the end of `side` at that corner. `skip` is the blended edge
/// itself, which is not a way out.
fn across_at(
    solid: &Solid,
    v: usize,
    side: usize,
    skip: EdgeId,
    ef: &crate::kernel::topo::EdgeFaces,
) -> Vec<usize> {
    let mut out = Vec::new();
    for &(e, _) in solid.face_loops(side).flatten() {
        let ed = solid.edges[e];
        if e == skip || (ed.v0 != v && ed.v1 != v) {
            continue;
        }
        for &f in &ef[e] {
            if f != side && !out.contains(&f) {
                out.push(f);
            }
        }
    }
    out
}

/// Decide how the chain's end at `v` is closed. `land` reports where the runout
/// would come to rest on a given plane, which is what tells a face the blend
/// actually runs into from one that merely touches the vertex.
///
/// The three ends are tried in order of how well they hide the blend's
/// termination: `Absorb` folds the trim curve into a neighbour's loop, `Cap`
/// closes it against the pair of neighbours either side of the corner, and
/// `Flat` -- which is always available -- stops the blend in its own last cross
/// section. Only the first two can fail to apply, so the chain now always ends
/// somewhere; what the order decides is how good the end looks, not whether
/// there is one.
#[allow(clippy::too_many_arguments)]
fn plan_runout_end(
    solid: &Solid,
    v: usize,
    e: EdgeId,
    fa: usize,
    fb: usize,
    ef: &crate::kernel::topo::EdgeFaces,
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
    // A cap needs one neighbour either side of the corner, across the two edges
    // the blend arrives on.
    let cap_at = |cands: &[usize]| -> Option<RunoutEnd> {
        let pick = |xs: &[usize]| -> Option<usize> {
            let hit: Vec<usize> = xs.iter().copied().filter(|c| cands.contains(c)).collect();
            (hit.len() == 1).then(|| hit[0])
        };
        let a = pick(&across_at(solid, v, fa, e, ef))?;
        let b = pick(&across_at(solid, v, fb, e, ef))?;
        // The cap is *one planar face*, emitted on `fa_side`'s surface and
        // bounded by the end ellipse and a stub into each neighbour. Both stubs
        // have to lie in that surface, so the two neighbours have to share it.
        // This is a weaker demand than every candidate sharing a plane, which is
        // what the coplanar-absorb branch below happens to guarantee: a corner
        // can offer a third face going off in its own direction and still be
        // perfectly cappable.
        (a != b && coplanar(&solid.faces[a].surface, &solid.faces[b].surface)).then_some(
            RunoutEnd::Cap {
                fa_side: a,
                fb_side: b,
            },
        )
    };

    // Being the only candidate is not the same as being able to take the curve.
    // Absorbing works by retreating the terminating face's two edges at the
    // corner back to the tangent points, so it is available exactly when those
    // points still lie on those edges; past their far ends the face has run out
    // before the blend has, and splicing there emits a loop that doubles back
    // over an edge the blend never entered.
    //
    // A partial-height inner wall meeting the bin's perimeter is the case that
    // makes the difference: the corner sits exactly on the wall's top, so the
    // tangent point up the perimeter stands above the wall's side face
    // altogether and the trim curve runs through open cavity. There is nothing
    // there to absorb it and the end is a cap, the same as an opening's mouth.
    //
    // Asking `planar_face_contains` instead was tried and is not the same
    // question: it refuses `partial_wall_one_end_on_boundary_is_watertight`,
    // whose runout lands *on* its face's boundary rather than strictly inside
    // it, which absorbs perfectly well. Fit is about the edges, not the area.
    if cands.len() == 1 {
        if let Some((ta, tb)) = as_plane(&solid.faces[cands[0]].surface)
            .map(&land)
            .transpose()?
            && absorb_fits(solid, cands[0], v, ta, tb)
        {
            return Ok(RunoutEnd::Absorb { face: cands[0] });
        }
        // The blend reaches past that face's edges, and no pair of neighbours
        // can cap it. It still has to end: a flat end stops it at the corner
        // rather than folding it into a face it overshoots.
        return Ok(cap_at(&cands).unwrap_or(flat));
    }
    // Several candidates in one plane are not an ambiguity about *where* the
    // blend ends. The bin's outer wall is cut into bands by the peg profile and
    // three of them meet at the pinch a wall opening leaves, so the plane is
    // settled and only the owner of the trim curve is open. Whichever band the
    // runout actually lands in absorbs it; if it lands in none of them the
    // corner is void -- the mouth of the opening -- and the blend needs a cap.
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
    // No face absorbs it. Capping does not care how many candidates there are
    // or whether they share a plane -- only that the two either side of the
    // corner do -- so it is worth asking before falling back to a flat end.
    Ok(cap_at(&cands).unwrap_or(flat))
}

/// Whether face `ft` can absorb a runout that comes to rest at `ta`/`tb`.
///
/// Absorbing trims the two edges `ft` brings into the corner back to the
/// tangent points and splices the trim curve between them, so it is available
/// exactly when each of those points still lies **on the edge it retreats**.
/// Where the blend reaches past an edge's far end that face has run out before
/// the blend did: the retreat would leave the edge pointing backwards and the
/// spliced loop would double back over ground the blend never covered, which
/// `validate` reports far downstream as a loop that does not close.
///
/// Each edge takes whichever tangent point lies on its own supporting curve --
/// the same rule `move_vertex` uses, and for the same reason: the answer has to
/// be a property of the edge, since both faces sharing it decide independently.
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
        // The retreat runs from the corner towards the far end and has to stop
        // short of it. `t == 0` is the corner standing still, which is what a
        // face that merely touches the blend does, so only the far end bounds.
        if (p - from).dot(along) / len_sq > 1.0 {
            return false;
        }
    }
    seen == 2
}

/// Whether `p` lies inside planar face `f`, by even-odd parity against every
/// loop projected into the surface's own uv. A curved edge contributes its
/// midpoint as well as its endpoints, so a mostly-curved loop does not collapse
/// onto its chords.
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
            if !matches!(ed.curve, Curve::Line { .. }) {
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

/// Where the blend surface comes to rest on a terminating plane, and the curve
/// the plane trims it by.
///
/// The blend's own surface decides this, not the edge it was built from: a
/// cylindrical blend runs out along a straight axis and is cut in an ellipse, a
/// torus blend runs out around a circle and is cut in a `Curve::TorusSection`.
/// Both answers are the plane's exact section of the surface, which is what
/// makes the trim curve lie on the blend and on the terminating face at once.
#[allow(clippy::too_many_arguments)]
fn runout_on(
    surface: &Surface,
    cv: Vec3,
    r: f32,
    ta_p: Vec3,
    tb_p: Vec3,
    plane: (Vec3, Vec3),
    away: Vec3,
) -> Result<(Vec3, Vec3, CurvEdge), String> {
    match *surface {
        Surface::Cylinder { axis, radius, .. } => {
            assert!(
                (radius - r).abs() <= END_AGREE,
                "blend runout: the blend cylinder's radius {radius} is not the blend radius {r}"
            );
            runout_cyl(cv, axis, r, ta_p, tb_p, plane)
        }
        Surface::Torus {
            center,
            axis,
            major_r,
            minor_r,
            ..
        } => {
            assert!(
                (minor_r - r).abs() <= END_AGREE,
                "blend runout: the blend torus's minor radius {minor_r} is not the blend radius {r}"
            );
            runout_torus(center, axis, major_r, r, cv, ta_p, tb_p, plane, away)
        }
        _ => Err(format!(
            "blend runout: no section curve for a {surface:?} blend"
        )),
    }
}

fn runout_cyl(
    cv: Vec3,
    axis: Vec3,
    r: f32,
    ta_p: Vec3,
    tb_p: Vec3,
    plane: (Vec3, Vec3),
) -> Result<(Vec3, Vec3, CurvEdge), String> {
    let (q, n) = plane;
    let n = n.normalize_or_zero();
    let d = axis.normalize_or_zero();
    let dn = d.dot(n);
    if dn.abs() < 1e-6 {
        return Err("blend runout: terminating face is parallel to the blend axis".into());
    }
    let onto = |p: Vec3| p + d * ((q - p).dot(n) / dn);
    let e1 = (ta_p - cv).normalize_or_zero();
    let e2 = d.cross(e1);
    let a_vec = d * (-r * e1.dot(n) / dn) + e1 * r;
    let b_vec = d * (-r * e2.dot(n) / dn) + e2 * r;
    let u = (tb_p - cv).normalize_or_zero();
    let t1 = u.dot(e2).atan2(u.dot(e1));
    let arc = CurvEdge {
        curve: Curve::Ellipse {
            center: onto(cv),
            a: a_vec,
            b: b_vec,
        },
        t0: 0.0,
        t1,
    };
    Ok((onto(ta_p), onto(tb_p), arc))
}

/// Move a circular tangent curve's parameter range onto the endpoints it now
/// runs between, keeping the direction it already swept.
///
/// Direction, not the shorter way round: the sweep's sign is which way the blend
/// travels along its corner, which a runout extends but never reverses, while
/// the *magnitude* is exactly what a runout changes. Recomputing both from the
/// endpoints alone would flip a sweep that grew past a half turn.
///
/// A curve with no meaningful range -- a line, whose edge is built from its two
/// vertices -- is left alone.
fn respan(ce: &mut CurvEdge, from: Vec3, to: Vec3) {
    let Curve::Circle {
        center,
        axis,
        ref_dir,
        ..
    } = ce.curve
    else {
        return;
    };
    let (d0, d1) = crate::kernel::geom::radial_frame(axis, ref_dir);
    let angle = |p: Vec3| {
        let v = p - center;
        v.dot(d1).atan2(v.dot(d0))
    };
    let was = ce.t1 - ce.t0;
    let t0 = angle(from);
    let mut sweep = wrap_pi(angle(to) - t0);
    if was != 0.0 && sweep != 0.0 && sweep.signum() != was.signum() {
        sweep += was.signum() * std::f32::consts::TAU;
    }
    assert!(
        sweep.abs() <= std::f32::consts::TAU,
        "blend runout: the tangent circle would sweep {sweep} rad, more than a full turn"
    );
    ce.t0 = t0;
    ce.t1 = t0 + sweep;
}

/// Wrap an angle into `(-pi, pi]`.
fn wrap_pi(mut a: f32) -> f32 {
    while a > std::f32::consts::PI {
        a -= std::f32::consts::TAU;
    }
    while a <= -std::f32::consts::PI {
        a += std::f32::consts::TAU;
    }
    a
}

/// Whether the closed interval `[lo, hi]` contains an odd multiple of pi -- the
/// minor angle at which a torus ring is at its narrowest.
fn spans_pi(lo: f32, hi: f32) -> bool {
    let k = ((lo - std::f32::consts::PI) / std::f32::consts::TAU).ceil();
    std::f32::consts::PI + k * std::f32::consts::TAU <= hi
}

/// Where a **torus** blend comes to rest on a terminating plane, and the curve
/// that trims it. The torus analogue of `runout_cyl`.
///
/// A cylindrical blend slides the rolling ball along a straight axis, so the
/// plane cuts it in an ellipse. A torus blend rolls the ball around a circle
/// instead, and a plane's section of a torus is a quartic in general -- outside
/// the analytic curve set, and the reason a chain could not terminate on an arc
/// at all. A plane **parallel to the torus axis** is the exception, and it is
/// the case the model produces: fixing the minor angle `t` fixes the ring
/// radius `rad = major + minor * cos t`, and the plane meets that ring exactly
/// where `cos u = offset / rad`, which is `Curve::TorusSection`.
///
/// Both touchdown curves are circles of **constant minor angle** -- that is what
/// a tangent circle of a torus blend is -- so running them out to the plane
/// changes `u` and leaves `t` alone, and the section's parameter range is read
/// straight off the two touchdown points that already exist. The landing points
/// are then the section evaluated at those two parameters, which is why they lie
/// on the blend surface and in the terminating plane by construction rather than
/// by a tolerance.
///
/// `branch` picks which of the plane's two crossings of each ring the blend runs
/// into. The plane cuts the ring at `+-u`, and the nearer of the two, measured
/// around the axis from where the blend already is, is always the one on the
/// blend's own side of the plane normal: for `u_v` and `u_p` both in `(0, pi)`,
/// `|wrap(u_v - u_p)| <= |wrap(u_v + u_p)|` reduces to `u_v <= pi`. So the sign
/// of the ball centre's component across the normal is the whole decision.
#[allow(clippy::too_many_arguments)]
fn runout_torus(
    center: Vec3,
    axis: Vec3,
    major: f32,
    minor: f32,
    cv: Vec3,
    ta_p: Vec3,
    tb_p: Vec3,
    plane: (Vec3, Vec3),
    away: Vec3,
) -> Result<(Vec3, Vec3, CurvEdge), String> {
    let (q, n) = plane;
    let n = n.normalize_or_zero();
    let axis = axis.normalize_or_zero();
    assert!(
        (n.length() - 1.0).abs() < 1e-4 && (axis.length() - 1.0).abs() < 1e-4,
        "blend runout: the terminating plane's normal {n:?} and the torus axis {axis:?} must \
         both be unit vectors"
    );
    // Only a plane containing the axis direction sections the torus in closed
    // form; a tilted one is the quartic, and guessing at it would be exactly the
    // numerical stand-in the kernel forbids.
    if n.dot(axis).abs() > 1e-4 {
        return Err(format!(
            "blend runout: the terminating plane's normal {n:?} is not perpendicular to the \
             torus blend's axis {axis:?}, so the section is a quartic"
        ));
    }
    // The ball centre rides the torus's spine, and `major` is the radius of that
    // circle -- if it does not, the surface handed in is not this blend's.
    let spine = {
        let rel = cv - center;
        (rel - axis * rel.dot(axis)).length()
    };
    // `join_agree`, not `END_AGREE`: the ball centre may have been moved by
    // `reconcile_shared_ends`, which is allowed a `MAX_JOIN_KINK` of slack, so
    // that is the disagreement this can legitimately see.
    assert!(
        (spine - major).abs() <= join_agree(minor),
        "blend runout: the ball centre {cv:?} stands {spine} from the torus axis, not the \
         major radius {major}"
    );

    let offset = (q - center).dot(n);
    let across = axis.cross(n);
    let side = (cv - center).dot(across);
    // Where the ball centre sits exactly *in* the terminating plane the two
    // crossings are equidistant and the side test says nothing, so the
    // direction the chain was heading breaks the tie -- which is the same
    // question the side test answers everywhere else.
    let side = if side.abs() > 1e-6 {
        side
    } else {
        away.dot(across)
    };
    if side.abs() <= 1e-6 {
        return Err(format!(
            "blend runout: the blend at {cv:?} straddles the terminating plane and runs along \
             it, so neither crossing of it is the one the blend runs into"
        ));
    }
    let branch = side.signum();

    // A point of the torus, back to the minor angle whose ring it is on.
    //
    // The ring radius `major + minor * cos t` is **signed**: on a spindle torus
    // (`minor > major`, which every corner blend tighter than its own corner is)
    // it goes negative past the axis, and a point sitting there is half a turn
    // round from where its unsigned distance suggests. `Surface::signed_distance`
    // makes the same distinction for the same reason; taking the unsigned radius
    // measures such a point `2 * major` off a surface it is exactly on.
    let minor_angle = |p: Vec3| -> f32 {
        let rel = p - center;
        let h = rel.dot(axis);
        let rad = (rel - axis * h).length();
        let near = ((rad - major).powi(2) + h * h).sqrt();
        let far = ((rad + major).powi(2) + h * h).sqrt();
        let (signed, off) = if (near - minor).abs() <= (far - minor).abs() {
            (rad, near)
        } else {
            (-rad, far)
        };
        assert!(
            (off - minor).abs() <= join_agree(minor),
            "blend runout: the touchdown {p:?} lies {off} from the blend torus's spine, not \
             its minor radius {minor}"
        );
        h.atan2(signed - major)
    };
    let t_a = minor_angle(ta_p);
    // The connect arc across the corner is at most a half turn, so the second
    // touchdown's minor angle is taken the short way round from the first.
    let t_b = t_a + wrap_pi(minor_angle(tb_p) - t_a);

    // The section exists only where the plane actually reaches the ring, i.e.
    // `|offset| <= rad` for every `t` the trim curve spans. `rad` is monotone in
    // `cos t`, so its minimum over the span is at an endpoint unless the span
    // crosses the narrowest ring at `t = pi`.
    let (lo, hi) = if t_a <= t_b { (t_a, t_b) } else { (t_b, t_a) };
    let cos_min = if spans_pi(lo, hi) {
        -1.0
    } else {
        lo.cos().min(hi.cos())
    };
    let rad_min = major + minor * cos_min;
    if !(rad_min > 0.0 && offset.abs() <= rad_min) {
        return Err(format!(
            "blend runout: the terminating plane stands {offset} from the torus blend's axis, \
             past the narrowest ring the trim curve crosses ({rad_min})"
        ));
    }

    let curve = Curve::torus_section(center, axis, n, offset, major, minor, branch);
    let (ta_new, tb_new) = (curve.point(t_a), curve.point(t_b));
    for (was, now) in [(ta_p, ta_new), (tb_p, tb_new)] {
        assert!(
            ((now - q).dot(n)).abs() <= 1e-4,
            "blend runout: the touchdown ran out from {was:?} to {now:?}, which is not on the \
             terminating plane"
        );
        // Running out moves a touchdown around the axis; it must not move it off
        // the ring it was on, or the trim curve does not meet the tangent curve
        // it is supposed to close.
        let ring = |p: Vec3| {
            let rel = p - center;
            let h = rel.dot(axis);
            ((rel - axis * h).length(), h)
        };
        let ((r0, h0), (r1, h1)) = (ring(was), ring(now));
        assert!(
            (r0 - r1).abs() <= 1e-3 && (h0 - h1).abs() <= 1e-3,
            "blend runout: the touchdown moved off its own ring, from radius {r0} height {h0} \
             to radius {r1} height {h1}"
        );
    }
    Ok((
        ta_new,
        tb_new,
        CurvEdge {
            curve,
            t0: t_a,
            t1: t_b,
        },
    ))
}

#[allow(clippy::too_many_arguments)]
fn build_cyl_blend(
    ed: crate::kernel::topo::Edge,
    cv0: Vec3,
    cv1: Vec3,
    ma: Vec3,
    na0: Vec3,
    ta_p0: Vec3,
    ta_p1: Vec3,
    tb_p0: Vec3,
    tb_p1: Vec3,
    r: f32,
    fwd_a: bool,
) -> Result<Fillet, String> {
    let dir = match ed.curve {
        Curve::Line { dir, .. } => dir,
        _ => return Err("cyl blend: edge not a line".into()),
    };
    let ref_dir = (-ma).normalize_or(Vec3::X);

    let ta = CurvEdge {
        curve: Curve::Line { p0: ta_p0, dir },
        t0: 0.0,
        t1: (ta_p1 - ta_p0).length(),
    };
    let tb = CurvEdge {
        curve: Curve::Line { p0: tb_p0, dir },
        t0: 0.0,
        t1: (tb_p1 - tb_p0).length(),
    };

    let ca0 = connect_arc(cv0, dir, ta_p0, tb_p0)?;
    let ca1 = connect_arc(cv1, dir, ta_p1, tb_p1)?;

    let surface = Surface::Cylinder {
        base: cv0,
        axis: dir,
        radius: r,
        ref_dir,
    };
    let sense = surface.normal(surface.project(ta_p0)).dot(na0) > 0.0;

    Ok(Fillet {
        ta,
        tb,
        ca0,
        ca1,
        ta_p0,
        ta_p1,
        tb_p0,
        tb_p1,
        surface,
        sense,
        fwd_a,
    })
}

fn circle_span(
    center: Vec3,
    axis: Vec3,
    ref_dir: Vec3,
    p0: Vec3,
    p1: Vec3,
    src: (f32, f32),
) -> (f32, f32) {
    let (d0, d1) = crate::kernel::geom::radial_frame(axis, ref_dir);
    let angle = |p: Vec3| {
        let v = p - center;
        v.dot(d1).atan2(v.dot(d0))
    };
    let span = src.1 - src.0;
    let t0 = angle(p0);
    if span.abs() >= std::f32::consts::TAU - 1e-3 {
        return (t0, t0 + span);
    }
    let wrapped = |mut a: f32| {
        while a > std::f32::consts::PI {
            a -= std::f32::consts::TAU;
        }
        while a < -std::f32::consts::PI {
            a += std::f32::consts::TAU;
        }
        a
    };
    let want = angle(p1);
    let miss = |s: f32| wrapped(t0 + s - want).abs();
    let sweep = if miss(span.abs()) <= miss(-span.abs()) {
        span.abs()
    } else {
        -span.abs()
    };
    (t0, t0 + sweep)
}

#[allow(clippy::too_many_arguments)]
fn build_torus_blend(
    ed: crate::kernel::topo::Edge,
    cv0: Vec3,
    cv1: Vec3,
    na0: Vec3,
    ta_p0: Vec3,
    ta_p1: Vec3,
    tb_p0: Vec3,
    tb_p1: Vec3,
    r: f32,
    cyl: (Vec3, Vec3, f32),
    fwd_a: bool,
) -> Result<Fillet, String> {
    let (cyl_base, cyl_axis, _cyl_radius) = cyl;
    let (edge_center, edge_axis, edge_radius, edge_ref_dir) = match ed.curve {
        Curve::Circle {
            center,
            axis,
            radius,
            ref_dir,
        } => (center, axis, radius, ref_dir),
        _ => return Err("torus blend: edge not a circle".into()),
    };
    let (a0, a1) = (ed.t0, ed.t1);
    let cv0_on = cyl_base + cyl_axis * (cv0 - cyl_base).dot(cyl_axis);
    let major = (cv0 - cv0_on).length();
    let torus_center = cv0_on;
    let torus_axis = edge_axis;
    let ref_dir = edge_ref_dir;
    assert!(
        major > 0.05,
        "blend torus degenerates to a ring: major {major} minor {r}"
    );
    let surface = Surface::Torus {
        center: torus_center,
        axis: torus_axis,
        major_r: major,
        minor_r: r,
        ref_dir,
    };
    let _ = (edge_center, edge_radius);

    let ta_center = torus_center + torus_axis * (ta_p0 - torus_center).dot(torus_axis);
    let ta_r = (ta_p0 - ta_center).length();
    let tb_center = torus_center + torus_axis * (tb_p0 - torus_center).dot(torus_axis);
    let tb_r = (tb_p0 - tb_center).length();
    let (ta_t0, ta_t1) = circle_span(ta_center, torus_axis, ref_dir, ta_p0, ta_p1, (a0, a1));
    let (tb_t0, tb_t1) = circle_span(tb_center, torus_axis, ref_dir, tb_p0, tb_p1, (a0, a1));
    let ta = CurvEdge {
        curve: Curve::Circle {
            center: ta_center,
            axis: torus_axis,
            radius: ta_r,
            ref_dir,
        },
        t0: ta_t0,
        t1: ta_t1,
    };
    let tb = CurvEdge {
        curve: Curve::Circle {
            center: tb_center,
            axis: torus_axis,
            radius: tb_r,
            ref_dir,
        },
        t0: tb_t0,
        t1: tb_t1,
    };

    let p0 = ed.curve.point(a0);
    let tan_at = |p: Vec3| {
        let v = p - torus_center;
        let perp = v - torus_axis * v.dot(torus_axis);
        torus_axis.cross(perp.normalize_or(Vec3::X))
    };
    let ca0 = connect_arc(cv0, tan_at(p0), ta_p0, tb_p0)?;
    let p1 = ed.curve.point(a1);
    let ca1 = connect_arc(cv1, tan_at(p1), ta_p1, tb_p1)?;

    let sense = surface.normal(surface.project(ta_p0)).dot(na0) > 0.0;

    Ok(Fillet {
        ta,
        tb,
        ca0,
        ca1,
        ta_p0,
        ta_p1,
        tb_p0,
        tb_p1,
        surface,
        sense,
        fwd_a,
    })
}

/// The arc the rolling ball leaves across a corner, from one touchdown to the
/// other.
///
/// Its plane is the ball's own: both touchdowns are a radius from the centre, so
/// the two of them and the centre fix the plane exactly, and `along` -- the
/// blended edge's tangent, which is perpendicular to that plane in exact
/// arithmetic -- is used only to keep the sweep's sign. Taking the plane from
/// `along` directly instead is what used to make two blends of one chain bound
/// two different arcs: their tangents at the vertex they share agree only to
/// float noise, and a ten-thousandth of a radian across a 2.4 mm radius already
/// moves the arc's midpoint past `topo`'s weld quantum, so the shared edge
/// interned twice and each blend face was left holding one of them.
fn connect_arc(center: Vec3, along: Vec3, from_pt: Vec3, to_pt: Vec3) -> Result<CurvEdge, String> {
    let (ra, rb) = ((from_pt - center).length(), (to_pt - center).length());
    assert!(
        (ra - rb).abs() <= join_agree(ra.max(rb)),
        "blend corner: the ball at {center:?} touches down {ra} from its centre on one face \
         and {rb} on the other; a rolling ball has one radius"
    );
    let ref_dir = (from_pt - center).normalize_or(Vec3::X);
    let normal = (from_pt - center).cross(to_pt - center).normalize_or_zero();
    let axis = if normal == Vec3::ZERO {
        along.normalize_or(Vec3::Z)
    } else if normal.dot(along) >= 0.0 {
        normal
    } else {
        -normal
    };
    let d1 = axis.cross(ref_dir);
    let sweep = {
        let v = to_pt - center;
        let mut a = v.dot(d1).atan2(v.dot(ref_dir));
        while a > std::f32::consts::PI {
            a -= 2.0 * std::f32::consts::PI;
        }
        while a < -std::f32::consts::PI {
            a += 2.0 * std::f32::consts::PI;
        }
        a
    };
    Ok(CurvEdge {
        curve: Curve::Circle {
            center,
            axis,
            radius: (from_pt - center).length(),
            ref_dir,
        },
        t0: 0.0,
        t1: sweep,
    })
}

struct Emitted {
    edge: (EdgeId, bool),
    start: Vec3,
    end_v: usize,
    end: Vec3,
}

#[allow(clippy::too_many_arguments)]
fn rebuild_loop(
    solid: &Solid,
    bm: &HashMap<EdgeId, Fillet>,
    vinfo: &HashMap<usize, (Vec3, Vec3)>,
    runouts: &HashMap<usize, Runout>,
    want: &HashMap<EdgeId, f32>,
    fi: usize,
    lp: &[(EdgeId, bool)],
    ef: &crate::kernel::topo::EdgeFaces,
    b: &mut Builder,
    items: &mut Vec<Emitted>,
    out: &mut Vec<(EdgeId, bool)>,
    moved_ends: &mut HashMap<(EdgeId, usize), Vec3>,
) -> Result<(), String> {
    let split_at = |v: usize, e: EdgeId| -> Option<Vec3> {
        let ro = runouts.get(&v)?;
        if ro.absorbing() != Some(fi) {
            return None;
        }
        if ef[e].contains(&ro.fa) {
            Some(ro.ta_p)
        } else if ef[e].contains(&ro.fb) {
            Some(ro.tb_p)
        } else {
            None
        }
    };
    // A capped or flat runout leaves the corner standing, so a face that only
    // touches it must not follow the blend back to a tangent point.
    let point_at = |v: usize, e: EdgeId, fallback: Vec3| -> Vec3 {
        match runouts.get(&v) {
            Some(ro) if ro.keeps_corner(fi) => fallback,
            // A flat end retreats an edge only where a touchdown actually lands
            // on it. `move_vertex` would pull every edge at the corner to the
            // nearer tangent point, which for an edge the blend does not touch
            // is a point off its own curve -- a different edge entirely, and the
            // face across it says so.
            Some(ro) if ro.is_flat() => ro.on_edge(&solid.edges[e], solid, v).unwrap_or(fallback),
            _ => split_at(v, e).unwrap_or_else(|| move_vertex(vinfo, v, e, solid, fallback)),
        }
    };

    items.clear();
    items.reserve(lp.len());
    for &(e, fwd) in lp {
        let ed = solid.edges[e];
        let end_v = if fwd { ed.v1 } else { ed.v0 };
        if want.contains_key(&e) {
            let bld = &bm[&e];
            let side_a = ef[e][0] == fi;
            let ce = if side_a { bld.ta } else { bld.tb };
            let (tp0, tp1) = if side_a {
                (bld.ta_p0, bld.ta_p1)
            } else {
                (bld.tb_p0, bld.tb_p1)
            };
            let (start, end) = if fwd { (tp0, tp1) } else { (tp1, tp0) };
            items.push(Emitted {
                edge: emit_curv(b, start, end, ce),
                start,
                end_v,
                end,
            });
        } else {
            let pos0 = solid.verts[ed.v0].point;
            let pos1 = solid.verts[ed.v1].point;
            let new0 = point_at(ed.v0, e, pos0);
            let new1 = point_at(ed.v1, e, pos1);
            let (start, end) = if fwd { (new0, new1) } else { (new1, new0) };
            // The cap pairs with the stub between the tangent point and the
            // corner, so the edge running into the corner is cut in two there.
            let cut_at = |v: usize| {
                let ro = runouts.get(&v)?;
                if ro.is_flat() {
                    // The face keeping the corner still has to carry the point
                    // the face across the same edge retreated to, or the two
                    // disagree about where that edge ends.
                    return (fi != ro.fa && fi != ro.fb)
                        .then(|| ro.on_edge(&ed, solid, v))
                        .flatten();
                }
                ro.cap_split(fi, &ef[e])
            };
            let cut = [ed.v0, ed.v1].into_iter().find_map(cut_at);
            // **How far along an edge the blend reaches belongs to the edge,
            // not to the face asking.** Both faces rebuild it independently and
            // the two results have to weld; where they disagree the builder
            // interns two edges and the solid opens along the seam, which it
            // reports far from here as `edge N used fwd=1 bwd=0`.
            //
            // The quantity they must agree on is the point where this edge
            // stops being ordinary wall and becomes blend, and a face reaches
            // it two ways: one *retreats* its endpoint there, the other keeps
            // the corner and *splits* the edge there. Comparing raw endpoints
            // would call that legitimate pair a defect, so the terminal point
            // is the split if there is one and the moved endpoint otherwise.
            for (v, moved) in [(ed.v0, new0), (ed.v1, new1)] {
                let term = cut_at(v).unwrap_or(moved);
                match moved_ends.get(&(e, v)) {
                    Some(&had) => assert!(
                        (had - term).length() <= END_AGREE,
                        "blend: face {fi} ends edge {e} at vertex {v} at {term:?}, but the face \
                         across it ended the same edge at {had:?}"
                    ),
                    None => {
                        moved_ends.insert((e, v), term);
                    }
                }
            }
            if let Some(p) = cut {
                if !matches!(ed.curve, Curve::Line { .. }) {
                    return Err(format!(
                        "blend runout: capped end needs a straight edge into the corner (face {fi})"
                    ));
                }
                // The tangent point has to land *inside* the edge it splits. A
                // blend wider than the run leaves it beyond the far end, and
                // splitting there emits a stub that doubles back over the rest
                // of the edge -- a loop no triangulation can tile. Refusing
                // costs the chain a blend, which `fillet_best_effort` already
                // treats as a corner left sharp.
                let d = end - start;
                let t = (p - start).dot(d) / d.length_squared();
                if !(1e-4..=1.0 - 1e-4).contains(&t) {
                    return Err(format!(
                        "blend runout: capped end at {p:?} is not inside the edge it splits \
                         ({start:?} -> {end:?})"
                    ));
                }
                for (s, t) in [(start, p), (p, end)] {
                    let (vs, ve) = (b.vertex(s), b.vertex(t));
                    items.push(Emitted {
                        edge: b.line(vs, ve),
                        start: s,
                        end_v: end_v,
                        end: t,
                    });
                }
                continue;
            }
            let vs = b.vertex(start);
            let ve = b.vertex(end);
            let eid = match ed.curve {
                Curve::Line { .. } => b.line(vs, ve),
                Curve::Circle {
                    center,
                    axis,
                    radius,
                    ref_dir,
                } => {
                    let (a0, a1) = if fwd { (ed.t0, ed.t1) } else { (ed.t1, ed.t0) };
                    b.arc(vs, ve, center, axis, radius, ref_dir, a0, a1)
                }
                Curve::Ellipse {
                    center,
                    a: ea,
                    b: eb,
                } => {
                    let (t0, t1) = if fwd { (ed.t0, ed.t1) } else { (ed.t1, ed.t0) };
                    b.ellipse(vs, ve, center, ea, eb, t0, t1)
                }
                Curve::TorusSection { .. } => {
                    let (t0, t1) = if fwd { (ed.t0, ed.t1) } else { (ed.t1, ed.t0) };
                    b.torus_section(vs, ve, ed.curve, t0, t1)
                }
            };
            items.push(Emitted {
                edge: eid,
                start,
                end_v,
                end,
            });
        }
    }

    let n = items.len();
    out.reserve(n + 2);
    for i in 0..n {
        out.push(items[i].edge);
        let next_start = items[(i + 1) % n].start;
        if let Some(ro) = runouts.get(&items[i].end_v) {
            let gap = (next_start - items[i].end).length() > 1e-6;
            if ro.absorbing() == Some(fi) && gap {
                out.push(emit_curv(b, items[i].end, next_start, ro.arc));
            } else if ro.is_flat() && (fi == ro.fa || fi == ro.fb) && gap {
                // The blend's tangent curve stops short of the corner the rest
                // of the loop still runs through, and the cap's stub is what
                // spans that. It is the same edge from both sides -- the cap
                // interns it by its endpoints -- so the two pair up without
                // either being told about the other.
                let (vs, ve) = (b.vertex(items[i].end), b.vertex(next_start));
                let stub = b.line(vs, ve);
                // The stub is a boundary of `fi`, so it has to lie in `fi`'s own
                // surface; a straight line between two of its points does only
                // because the touchdown and the corner share a ruling of it.
                let mid = (items[i].end + next_start) * 0.5;
                let off = solid.faces[fi].surface.signed_distance(mid).abs();
                if off > ON_EDGE {
                    return Err(format!(
                        "blend runout: the flat end's stub across face {fi} leaves its surface \
                         by {off} at {mid:?}"
                    ));
                }
                out.push(stub);
            }
        }
    }
    Ok(())
}

/// Where the edge `e` running into blended vertex `v` puts that endpoint.
///
/// **This is a property of the edge, never of the face asking.** Both faces
/// sharing an edge rebuild it independently, and if they disagree about where
/// its endpoint moved, the builder interns two edges and the solid opens along
/// the seam -- `edge N used fwd=1 bwd=0`, from a blend that is otherwise
/// perfectly well formed.
///
/// Choosing by distance to the *asking face's* surface is exactly such a
/// disagreement, and it was the bug here. A partial-height inner wall meeting
/// the bin's perimeter wall puts both tangent points on the wall's side plane,
/// so that face's test is a tie; the cavity-wall face across the same edge sees
/// only `ta` on itself and picks it, and the two answers differ. The two
/// tangent points are the corner retreating along the two edges that meet
/// there, so what settles it is which of them lies on *this* edge's own
/// supporting curve -- a quantity both faces compute identically because
/// neither the curve nor the corner belongs to either of them.
fn move_vertex(
    vinfo: &HashMap<usize, (Vec3, Vec3)>,
    v: usize,
    e: EdgeId,
    solid: &Solid,
    fallback: Vec3,
) -> Vec3 {
    let Some(&(pa, pb)) = vinfo.get(&v) else {
        return fallback;
    };
    let ed = solid.edges[e];
    let (da, db) = (dist_to_curve(pa, &ed, solid), dist_to_curve(pb, &ed, solid));
    assert!(
        da.is_finite() && db.is_finite(),
        "blend: tangent points {pa:?}/{pb:?} are {da}/{db} from edge {e}'s curve"
    );
    if da <= db { pa } else { pb }
}

/// Distance from `p` to the curve supporting edge `ed`, **extended** past its
/// own parameter range: a retreating corner lands beyond the edge's stored ends
/// as often as inside them, so a range-clamped distance would rank the two
/// tangent points by how far past the end they fall rather than by which line
/// they are on.
///
/// Closed form for the two curve kinds a corner actually retreats along. A
/// blend chain may not terminate on an ellipse or a torus section (see
/// `plan_runout_end`), so the remaining kinds fall back to the chord, which is
/// still a function of the edge alone -- which is the property that matters.
fn dist_to_curve(p: Vec3, ed: &crate::kernel::topo::Edge, solid: &Solid) -> f32 {
    match ed.curve {
        Curve::Line { p0, dir } => {
            let rel = p - p0;
            (rel - dir * rel.dot(dir)).length()
        }
        Curve::Circle {
            center,
            axis,
            radius,
            ..
        } => {
            let rel = p - center;
            let along = rel.dot(axis);
            let radial = (rel - axis * along).length();
            ((radial - radius).powi(2) + along * along).sqrt()
        }
        _ => {
            let (a, b) = (solid.verts[ed.v0].point, solid.verts[ed.v1].point);
            let d = (b - a).normalize_or_zero();
            let rel = p - a;
            (rel - d * rel.dot(d)).length()
        }
    }
}

fn emit_curv(b: &mut Builder, start: Vec3, end: Vec3, ce: CurvEdge) -> (EdgeId, bool) {
    let vs = b.vertex(start);
    let ve = b.vertex(end);
    let forward = || {
        let at_start = ce.curve.point(ce.t0);
        (at_start - start).length() < (ce.curve.point(ce.t1) - start).length()
    };
    match ce.curve {
        Curve::Line { .. } => b.line(vs, ve),
        Curve::Circle {
            center,
            axis,
            radius,
            ref_dir,
        } => {
            let (t0, t1) = if forward() {
                (ce.t0, ce.t1)
            } else {
                (ce.t1, ce.t0)
            };
            b.arc(vs, ve, center, axis, radius, ref_dir, t0, t1)
        }
        Curve::Ellipse {
            center,
            a: ea,
            b: eb,
        } => {
            let (t0, t1) = if forward() {
                (ce.t0, ce.t1)
            } else {
                (ce.t1, ce.t0)
            };
            b.ellipse(vs, ve, center, ea, eb, t0, t1)
        }
        Curve::TorusSection { .. } => {
            let (t0, t1) = if forward() {
                (ce.t0, ce.t1)
            } else {
                (ce.t1, ce.t0)
            };
            b.torus_section(vs, ve, ce.curve, t0, t1)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kernel::math::Vec3;

    fn approx(a: Vec3, b: Vec3) -> bool {
        (a - b).length() < 1e-4
    }

    #[test]
    fn best_effort_matches_fillet_edges_when_nothing_fails() {
        use crate::kernel::build::extrude;
        use crate::kernel::sketch::Sketch;

        let sk = Sketch::rounded_rect(0.0, 0.0, 20.0, 20.0, 4.0);
        let solid = extrude(&sk, 0.0, 5.0);
        let top: Vec<(EdgeId, f32)> = (0..solid.edges.len())
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

    #[test]
    fn best_effort_still_reports_a_non_manifold_input() {
        use crate::kernel::build::extrude;
        use crate::kernel::sketch::Sketch;

        let solid = extrude(&Sketch::rectangle(0.0, 0.0, 10.0, 10.0), 0.0, 5.0);
        let err = fillet_best_effort(&solid, &[(solid.edges.len() + 7, 1.0)])
            .expect_err("out-of-range edge must be reported");
        assert!(err.contains("out of range"), "unexpected error: {err}");
    }

    #[test]
    fn rolling_ball_corner_math() {
        let p = Vec3::new(10.0, 0.0, 0.0);
        let ma = Vec3::new(0.0, 0.0, 1.0);
        let mb = Vec3::new(-1.0, 0.0, 0.0);
        let r = 2.0_f32;
        let sin_theta = ma.cross(mb).length();
        let c = p + r * (ma + mb) / sin_theta;
        assert!(approx(c, Vec3::new(8.0, 0.0, 2.0)), "ball centre {c}");
        let ta = c - r * ma;
        let tb = c - r * mb;
        assert!(approx(ta, Vec3::new(8.0, 0.0, 0.0)), "floor tangent {ta}");
        assert!(approx(tb, Vec3::new(10.0, 0.0, 2.0)), "wall tangent {tb}");
    }

    #[test]
    fn connect_arc_endpoints() {
        let center = Vec3::new(8.0, 0.0, 2.0);
        let axis = Vec3::new(0.0, 1.0, 0.0);
        let from = Vec3::new(8.0, 0.0, 0.0);
        let to = Vec3::new(10.0, 0.0, 2.0);
        let ce = connect_arc(center, axis, from, to).unwrap();
        assert!(approx(ce.curve.point(ce.t0), from), "arc start");
        assert!(approx(ce.curve.point(ce.t1), to), "arc end");
        assert!(((ce.t1 - ce.t0).abs() - std::f32::consts::FRAC_PI_2).abs() < 1e-4);
    }

    #[test]
    fn circle_span_lands_on_its_own_endpoints_not_the_source_edges() {
        let center = Vec3::new(80.0, 4.0, 23.7);
        let axis = Vec3::Z;
        let ref_dir = Vec3::X;
        let p0 = Vec3::new(80.0, 5.45, 23.7);
        let p1 = Vec3::new(78.55, 4.0, 23.7);
        let src = (0.0, -std::f32::consts::FRAC_PI_2);
        let (t0, t1) = circle_span(center, axis, ref_dir, p0, p1, src);
        let c = Curve::Circle {
            center,
            axis,
            radius: 1.45,
            ref_dir,
        };
        assert!(approx(c.point(t0), p0), "span start {:?}", c.point(t0));
        assert!(approx(c.point(t1), p1), "span end {:?}", c.point(t1));
    }

    #[test]
    fn circle_span_keeps_a_full_turn_a_full_turn() {
        let center = Vec3::ZERO;
        let p = Vec3::new(3.0, 0.0, 0.0);
        let src = (0.0, std::f32::consts::TAU);
        let (t0, t1) = circle_span(center, Vec3::Z, Vec3::X, p, p, src);
        assert!(
            (t1 - t0 - std::f32::consts::TAU).abs() < 1e-4,
            "sweep {}",
            t1 - t0
        );
    }
}
