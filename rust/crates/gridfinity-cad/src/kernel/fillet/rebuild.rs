//! Rewriting the solid around the finished blends. The three phases run into one
//! `Builder` in order -- `faces` rebuilds every face whose loop touches a moved
//! vertex, `blend_faces` emits the blend patches, `runout_caps` closes the chain
//! ends no neighbour absorbed -- and they pair up only through the vertices and
//! curves they intern, never by being told about each other.
//!
//! One rule runs through all of it. **How far along an edge the blend reaches
//! belongs to the edge, not to the face asking.** Both faces sharing an edge
//! rebuild it independently and the two results have to weld; where they disagree
//! the builder interns two edges and the solid opens along the seam, reported far
//! from here as `edge N used fwd=1 bwd=0`.

use std::collections::HashMap;

use crate::kernel::curvedge::emit_curv;
use crate::kernel::geom::{Curve, Surface};
use crate::kernel::math::Vec3;
use crate::kernel::topo::{Builder, EdgeFaces, EdgeId, Solid};

use super::blend::Blends;
use super::query::dist_to_curve;
use super::runout::{RunoutEnd, Runouts};
use super::{END_AGREE, ON_EDGE};

pub(super) type VertexInfo = HashMap<usize, (Vec3, Vec3)>;

/// Maps the finished blends to what the rebuild has to know about them: a
/// `VertexInfo` giving, for every vertex a blended edge ends at, the two tangent
/// points the blend retreats it to, and a per-face flag that is true exactly when
/// one of that face's loops names a blended edge or an edge ending at such a
/// vertex. A face whose flag is false is untouched by the blend and can be copied
/// across verbatim. The blends are visited in ascending edge id, so a vertex two
/// blends share takes the lower one's points regardless of hash order -- which is
/// consistent because `reconcile_shared_ends` already made them equal.
pub(super) fn touched_faces(
    solid: &Solid,
    bm: &Blends,
    want: &HashMap<EdgeId, f32>,
) -> (Vec<bool>, VertexInfo) {
    let mut vinfo: VertexInfo = HashMap::with_capacity(bm.len() * 2);
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
    (touched, vinfo)
}

/// Emits one face into `b` for every face of the input, in input order: a
/// verbatim copy where `touched` is false, and otherwise the same surface and
/// sense with every loop rebuilt against the blends. That one-for-one, in-order
/// correspondence is asserted per face, and is what makes a face index reported
/// downstream mean the same thing as the input's. Errors carry the message of the
/// first loop that could not be rebuilt, and `moved_ends` is threaded across
/// every face so the two sides of an edge are checked against each other.
#[allow(clippy::too_many_arguments)]
pub(super) fn faces(
    solid: &Solid,
    bm: &Blends,
    vinfo: &VertexInfo,
    runouts: &Runouts,
    want: &HashMap<EdgeId, f32>,
    touched: &[bool],
    edge_faces: &EdgeFaces,
    b: &mut Builder,
) -> Result<(), String> {
    let mut loop_scratch: Vec<(EdgeId, bool)> = Vec::new();
    let mut items_scratch: Vec<Emitted> = Vec::new();
    let mut inner_ranges: Vec<usize> = Vec::new();
    let mut moved_ends: HashMap<(EdgeId, usize), Vec3> = HashMap::new();

    for (fi, &is_touched) in touched.iter().enumerate() {
        if !is_touched {
            let id = b.copy_face(solid, fi);
            assert!(
                id == fi,
                "blend: copying face {fi} of the input emitted face {id}; the rebuild emits \
                 exactly one face per input face, in order, which is what makes every face \
                 index it reports mean the same thing as the input's"
            );
            continue;
        }
        loop_scratch.clear();
        inner_ranges.clear();
        rebuild_loop(
            solid,
            bm,
            vinfo,
            runouts,
            want,
            fi,
            solid.outer_edges(fi),
            edge_faces,
            b,
            &mut items_scratch,
            &mut loop_scratch,
            &mut moved_ends,
        )?;
        let outer_len = loop_scratch.len();
        for lp in solid.inner_loops(fi) {
            let before = loop_scratch.len();
            rebuild_loop(
                solid,
                bm,
                vinfo,
                runouts,
                want,
                fi,
                lp,
                edge_faces,
                b,
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
        let id = b.face_from(surface, sense, outer, &inners);
        assert!(
            id == fi,
            "blend: rebuilding face {fi} of the input emitted face {id}; the rebuild emits \
             exactly one face per input face, in order"
        );
    }
    Ok(())
}

/// Emits one face per blend patch into `b`, in ascending blended edge id, each
/// bounded by its two tangent curves and its two connect arcs wound to agree with
/// the blend's own sense. Returns, for every blended edge end, the `(edge id,
/// direction)` under which that end's connect arc was traversed -- exactly one
/// entry per end, asserted -- which is how `runout_caps` winds a cap against the
/// blend without a second orientation pass.
pub(super) fn blend_faces(
    solid: &Solid,
    bm: &Blends,
    b: &mut Builder,
) -> HashMap<usize, (EdgeId, bool)> {
    let mut blend_keys: Vec<EdgeId> = bm.keys().copied().collect();
    blend_keys.sort_unstable();
    let ends: std::collections::BTreeSet<usize> = blend_keys
        .iter()
        .flat_map(|&k| [solid.edges[k].v0, solid.edges[k].v1])
        .collect();
    let mut arc_used: HashMap<usize, (EdgeId, bool)> = HashMap::new();
    for k in blend_keys {
        let bld = &bm[&k];
        let e_ta = emit_curv(b, bld.ta_p0, bld.ta_p1, bld.ta);
        let e_tb = emit_curv(b, bld.tb_p0, bld.tb_p1, bld.tb);
        let e_ca0 = emit_curv(b, bld.ta_p0, bld.tb_p0, bld.ca0);
        let e_ca1 = emit_curv(b, bld.ta_p1, bld.tb_p1, bld.ca1);
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
        b.face_from(bld.surface, bld.sense, &lp, &[]);
    }
    assert!(
        arc_used.len() == ends.len(),
        "blend: {} blended edge ends but {} trim curves recorded; every end of every blend \
         names exactly one traversal, and a cap reads its winding from that entry",
        ends.len(),
        arc_used.len()
    );
    arc_used
}

/// Emits the face that closes each chain end no neighbour absorbed, in ascending
/// vertex order: a triangle of the trim curve and the two stubs back to the
/// corner, on the cap's own side surface for a `Cap` end and on the plane through
/// the corner and the two touchdowns for a `Flat` one -- the plane of the ball's
/// own connect arc, oriented so the blend it closes lies behind it, which is
/// asserted by requiring that plane not be edge-on to the direction the chain was
/// heading. `Absorb` ends emit nothing, since a neighbouring face took the curve.
///
/// All three edges are interned by their endpoints -- the trim curve is the edge
/// the blend face just built, the stubs are the lines the neighbours emitted when
/// their edge was split -- so a cap pairs up without being told who its
/// neighbours are. Its winding comes from `arc_used`, the blend face's own
/// traversal of the shared curve, reversed: one comparison fixing all three
/// edges, since `orient::normalize` cannot re-wind a single face against its own
/// component.
pub(super) fn runout_caps(
    solid: &Solid,
    runouts: &Runouts,
    arc_used: &HashMap<usize, (EdgeId, bool)>,
    b: &mut Builder,
) {
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
        let arc = emit_curv(b, ro.tb_p, ro.ta_p, ro.arc);
        let mut lp = vec![b.line(va, vc), b.line(vc, vb), arc];
        if arc_used.get(&v) == Some(&arc) {
            lp.reverse();
            for x in &mut lp {
                x.1 = !x.1;
            }
        }
        b.face_from(surface, sense, &lp, &[]);
    }
}

struct Emitted {
    edge: (EdgeId, bool),
    start: Vec3,
    end_v: usize,
    end: Vec3,
}

/// Rebuilds one loop `lp` of face `fi` against the blends and appends its edges
/// to `out`, interning each into `b` as it goes. A blended edge becomes the
/// tangent curve on whichever side `fi` is; every other edge is re-emitted on its
/// own curve between endpoints moved to wherever the blend reaches, split in two
/// where a runout terminates inside it, and followed by the trim curve or stub
/// that bridges the gap to the next edge's new start. Errors when a capped end
/// wants to split a curved edge, when the split point is not strictly inside the
/// edge, or when a flat end's stub leaves the face's own surface by more than
/// `ON_EDGE`.
///
/// A capped or flat runout leaves the corner standing, so a face that merely
/// touches it must not follow the blend back to a tangent point, and a flat end
/// retreats an edge only where a touchdown actually lands on it. The stub from a
/// touchdown to the corner is a straight line inside a possibly curved face,
/// legitimate only because the two share a ruling of it, which is what the
/// surface check before emission states. Because a face reaches its endpoint two
/// ways -- one *retreats* it to a touchdown, the other keeps the corner and
/// *splits* the edge there -- `moved_ends` records the split where there is one
/// and the moved endpoint otherwise, and asserts that the face across each edge
/// agreed on it to within `END_AGREE`.
#[allow(clippy::too_many_arguments)]
fn rebuild_loop(
    solid: &Solid,
    bm: &Blends,
    vinfo: &VertexInfo,
    runouts: &Runouts,
    want: &HashMap<EdgeId, f32>,
    fi: usize,
    lp: &[(EdgeId, bool)],
    ef: &EdgeFaces,
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
    let point_at = |v: usize, e: EdgeId, fallback: Vec3| -> Vec3 {
        match runouts.get(&v) {
            Some(ro) if ro.keeps_corner(fi) => fallback,
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
            let cut_at = |v: usize| {
                let ro = runouts.get(&v)?;
                if ro.is_flat() {
                    return (fi != ro.fa && fi != ro.fb)
                        .then(|| ro.on_edge(&ed, solid, v))
                        .flatten();
                }
                ro.cap_split(fi, &ef[e])
            };
            let cut = [ed.v0, ed.v1].into_iter().find_map(cut_at);
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
                        end_v,
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
                let (vs, ve) = (b.vertex(items[i].end), b.vertex(next_start));
                let stub = b.line(vs, ve);
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

/// Maps vertex `v`, seen from edge `e`, to where the blend retreats it to:
/// whichever of the two tangent points recorded for `v` lies nearer *`e`'s own*
/// supporting curve, or `fallback` when no blend moved `v`. Asserts both
/// distances are finite, since a NaN would make the choice arbitrary and the two
/// sides of the edge disagree.
///
/// The choice is by distance to the edge's curve rather than to the asking face's
/// surface precisely because both faces sharing `e` compute the former
/// identically -- neither the curve nor the corner belongs to either of them.
/// Choosing by the face's surface was the bug: a partial-height inner wall
/// meeting the perimeter puts both tangent points on the wall's side plane, so
/// that face's test is a tie while the cavity-wall face across the same edge sees
/// only `ta` on itself.
fn move_vertex(vinfo: &VertexInfo, v: usize, e: EdgeId, solid: &Solid, fallback: Vec3) -> Vec3 {
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
