//! What the request looks like as chains: the phase that reads a flat list of
//! `(edge, radius)` pairs as the runs of edges the ball actually rolls along.
//! `ends` classifies the vertices of one accepted request, `chains` partitions a
//! request into independent runs, and `salvage` cuts a run down to what builds --
//! together they are how `fillet_best_effort` turns one refusal into a part with
//! most of its corners rounded rather than none.

use std::collections::HashMap;

use crate::kernel::topo::{EdgeFaces, EdgeId, Solid};

use super::fillet_edges_with;

pub(super) type VertexBlends = HashMap<usize, Vec<EdgeId>>;
pub(super) type Terminating = HashMap<usize, EdgeId>;

/// Classifies the vertices of the requested edges. Returns `VertexBlends`,
/// mapping every vertex an edge of `want` ends at to the edges of `want` meeting
/// there, and `Terminating`, its restriction to the vertices carrying exactly
/// one -- the chain ends, which are the vertices a runout will have to close.
/// A vertex carrying two is interior to a chain and appears only in the first
/// map. Three or more is an error rather than a classification: that corner
/// needs a spherical blend, which is unsupported.
pub(super) fn ends(
    solid: &Solid,
    want: &HashMap<EdgeId, f64>,
) -> Result<(VertexBlends, Terminating), String> {
    let mut vertex_blends: VertexBlends = HashMap::new();
    for &e in want.keys() {
        let ed = solid.edges[e];
        vertex_blends.entry(ed.v0).or_default().push(e);
        vertex_blends.entry(ed.v1).or_default().push(e);
    }
    let mut terminating: Terminating = HashMap::new();
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
    Ok((vertex_blends, terminating))
}

/// Maps a run of requested blends to the largest sub-run of it that builds on
/// top of `base`, which is taken as already accepted and is never returned.
/// Returns `run` unchanged when `base + run` builds, the empty vector when the
/// budget runs out or a single edge fails alone, and otherwise the concatenation
/// of what the two halves salvage -- the head first, then the tail retried with
/// the head added to `base`, so the answer is a set that builds together rather
/// than two that only build apart. `depth` bounds the bisection, so a
/// pathological request cannot spend the whole build proving every edge fails on
/// its own; the cost is that a survivor deeper than `depth` splits is dropped.
pub(super) fn salvage(
    solid: &Solid,
    ef: &EdgeFaces,
    base: &[(EdgeId, f64)],
    run: &[(EdgeId, f64)],
    depth: u32,
) -> Vec<(EdgeId, f64)> {
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

/// Partitions the request into runs of edges that touch, by union-find over
/// shared vertices: two requested edges land in the same group when a path of
/// requested edges joins them end to end. Every input pair appears in exactly
/// one group, so the groups partition the request -- asserted, since a blend
/// falling out of every group is a corner silently left sharp. An out-of-range
/// edge id contributes no vertices and so comes back as its own group, leaving
/// the diagnosis to `check_edges`. Groups carry their edges in request order,
/// not in path order along the chain.
pub(super) fn chains(solid: &Solid, blends: &[(EdgeId, f64)]) -> Vec<Vec<(EdgeId, f64)>> {
    let mut parent: Vec<usize> = (0..blends.len()).collect();
    /// Maps a request index to the representative of its group, compressing the
    /// path it walked so later lookups of the same group are one step.
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
    let mut groups: Vec<(usize, Vec<(EdgeId, f64)>)> = Vec::new();
    for (i, &b) in blends.iter().enumerate() {
        let root = find(&mut parent, i);
        match groups.iter_mut().find(|(r, _)| *r == root) {
            Some((_, v)) => v.push(b),
            None => groups.push((root, vec![b])),
        }
    }
    let total: usize = groups.iter().map(|(_, v)| v.len()).sum();
    assert!(
        total == blends.len(),
        "blend chain: {} edges requested fell into chains holding {total}; the chains partition \
         the request",
        blends.len()
    );
    groups.into_iter().map(|(_, v)| v).collect()
}
