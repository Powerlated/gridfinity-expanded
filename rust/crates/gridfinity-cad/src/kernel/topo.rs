//! Boundary-representation topology: an explicit face / loop / edge / vertex
//! structure with shared edges. Each `Edge` is referenced by the faces on both
//! its sides; each `Face` is trimmed by ordered loops of directed edges. This is
//! a genuine B-rep (topology + analytic geometry, shared edges), just without
//! half-edge next/prev pointers — loops are stored as explicit ordered edge
//! lists, which makes constructing correct shared topology far less error-prone.
//!
//! The manifold invariant (`validate`): every edge is used exactly twice across
//! all loops, once in each direction.

use crate::kernel::geom::{Curve, Surface};
use crate::kernel::math::{Vec3, weld_key};
use std::collections::HashMap;

pub type VertexId = usize;
pub type EdgeId = usize;

#[derive(Clone, Copy, Debug)]
pub struct Vertex {
    pub point: Vec3,
}

/// An edge on an analytic `Curve`, running from `v0` to `v1` as the parameter
/// goes `t0 → t1` (raw, unwrapped — so an arc's sweep is unambiguous).
#[derive(Clone, Copy, Debug)]
pub struct Edge {
    pub curve: Curve,
    pub t0: f32,
    pub t1: f32,
    pub v0: VertexId,
    pub v1: VertexId,
}

impl Edge {
    /// Sample the edge as it is traversed in `forward` direction, `n` segments,
    /// returning `n + 1` points (endpoints included).
    pub fn sample(&self, forward: bool, n: usize) -> Vec<Vec3> {
        let (a, b) = if forward {
            (self.t0, self.t1)
        } else {
            (self.t1, self.t0)
        };
        (0..=n)
            .map(|i| {
                let t = a + (b - a) * (i as f32 / n as f32);
                self.curve.point(t)
            })
            .collect()
    }

    /// Segment count appropriate for this edge at the given angular resolution.
    pub fn seg_count(&self, arc_segs_per_quarter: usize) -> usize {
        match self.curve {
            Curve::Line { .. } => 1,
            Curve::Circle { .. } | Curve::Ellipse { .. } => {
                let sweep = (self.t1 - self.t0).abs();
                ((sweep / (std::f32::consts::PI / 2.0)) * arc_segs_per_quarter as f32)
                    .ceil()
                    .max(1.0) as usize
            }
        }
    }
}

/// An ordered ring of directed edges. `true` = traverse the edge `v0 → v1`.
///
/// A **transient** value type used only to hand a loop to [`Builder::face`];
/// the `Solid` no longer stores `Loop`s — they are flattened into its
/// `loop_edges`/`loops` CSR arenas, so a built solid holds no per-loop `Vec`.
#[derive(Clone, Debug, Default)]
pub struct Loop {
    pub edges: Vec<(EdgeId, bool)>,
}

impl Loop {
    pub fn new(edges: Vec<(EdgeId, bool)>) -> Loop {
        Loop { edges }
    }
}

/// A trimmed face: an analytic `surface` plus a contiguous run of loop ids in
/// the owning `Solid`'s CSR arena — the first is the outer boundary, the rest
/// are holes. `sense` = whether the surface's own normal is the outward
/// (solid-exterior) normal; `false` flips it.
///
/// The loops live in the `Solid`, not the `Face`: access them via
/// [`Solid::outer_edges`], [`Solid::face_loops`], [`Solid::inner_loops`].
#[derive(Clone, Debug)]
pub struct Face {
    pub surface: Surface,
    pub sense: bool,
    /// First loop id; loops are `[loop0, loop0 + n_loops)`, outer at `loop0`.
    loop0: u32,
    /// Number of loops, `>= 1` (outer plus zero or more inners).
    n_loops: u32,
}

/// A solid: flat, `Copy`-element arenas. One shell, manifold.
///
/// Two-level CSR for the loops: a face names a contiguous range of loop ids;
/// each loop id indexes `loops` (offsets into the flat `loop_edges`). Cloning a
/// `Solid` is therefore five flat `Vec` `memcpy`s with **zero per-loop
/// allocation** — which is what makes `fillet::fillet_edges`' repeated clones
/// cheap.
#[derive(Clone, Debug)]
pub struct Solid {
    pub verts: Vec<Vertex>,
    pub edges: Vec<Edge>,
    /// Flat directed-edge storage for every loop, concatenated.
    loop_edges: Vec<(EdgeId, bool)>,
    /// CSR offsets into `loop_edges`: loop `l` owns `loop_edges[loops[l]..loops[l+1]]`.
    /// Always non-empty (`[0]` for a solid with no loops).
    loops: Vec<u32>,
    pub faces: Vec<Face>,
}

impl Default for Solid {
    fn default() -> Solid {
        Solid {
            verts: Vec::new(),
            edges: Vec::new(),
            loop_edges: Vec::new(),
            loops: vec![0],
            faces: Vec::new(),
        }
    }
}

impl Solid {
    pub fn vertex(&self, id: VertexId) -> Vec3 {
        self.verts[id].point
    }

    /// Directed endpoints of an edge use.
    pub fn directed(&self, edge: EdgeId, forward: bool) -> (VertexId, VertexId) {
        let e = &self.edges[edge];
        if forward { (e.v0, e.v1) } else { (e.v1, e.v0) }
    }

    /// The directed edges of loop `lid`.
    fn loop_slice(&self, lid: u32) -> &[(EdgeId, bool)] {
        let s = self.loops[lid as usize] as usize;
        let e = self.loops[lid as usize + 1] as usize;
        &self.loop_edges[s..e]
    }

    /// The outer boundary of face `fid`.
    pub fn outer_edges(&self, fid: usize) -> &[(EdgeId, bool)] {
        self.loop_slice(self.faces[fid].loop0)
    }

    /// Total directed-edge entries across all loops (for sizing a rebuild).
    pub fn loop_edges_len(&self) -> usize {
        self.loop_edges.len()
    }

    /// Number of hole loops of face `fid`.
    pub fn n_inners(&self, fid: usize) -> usize {
        self.faces[fid].n_loops as usize - 1
    }

    /// Every loop of face `fid` (outer first, then holes), each as a slice.
    pub fn face_loops(&self, fid: usize) -> impl Iterator<Item = &[(EdgeId, bool)]> {
        let f = &self.faces[fid];
        (f.loop0..f.loop0 + f.n_loops).map(move |lid| self.loop_slice(lid))
    }

    /// Just the hole loops of face `fid`.
    pub fn inner_loops(&self, fid: usize) -> impl Iterator<Item = &[(EdgeId, bool)]> {
        let f = &self.faces[fid];
        (f.loop0 + 1..f.loop0 + f.n_loops).map(move |lid| self.loop_slice(lid))
    }

    /// Check the manifold invariant: every edge used exactly twice, once each
    /// direction, and each loop forms a closed chain (consecutive edges meet).
    pub fn validate(&self) -> Result<(), String> {
        let mut fwd = vec![0u32; self.edges.len()];
        let mut bwd = vec![0u32; self.edges.len()];
        for fi in 0..self.faces.len() {
            for lp in self.face_loops(fi) {
                if lp.is_empty() {
                    return Err(format!("face {fi} has an empty loop"));
                }
                for w in 0..lp.len() {
                    let (e, f) = lp[w];
                    if f { fwd[e] += 1 } else { bwd[e] += 1 }
                    let (_, end) = self.directed(e, f);
                    let (ne, nf) = lp[(w + 1) % lp.len()];
                    let (nstart, _) = self.directed(ne, nf);
                    if end != nstart {
                        return Err(format!(
                            "face {fi}: loop not closed (edge {e} end {end} != next start {nstart})"
                        ));
                    }
                }
            }
        }
        for e in 0..self.edges.len() {
            if fwd[e] != 1 || bwd[e] != 1 {
                return Err(format!(
                    "edge {e} used fwd={} bwd={} (want 1/1)",
                    fwd[e], bwd[e]
                ));
            }
        }
        Ok(())
    }

    /// Faces on each side of every edge, `[EdgeId] -> up to two FaceIds`.
    ///
    /// Returned as a flat CSR ([`EdgeFaces`]) rather than `Vec<Vec<usize>>`: a
    /// solid with E edges used to cost E tiny `Vec` allocations here, and
    /// `fillet_edges`/`chamfer` call this repeatedly — the single biggest churn
    /// source after the earcut floor. Now it is five flat `Vec`s regardless of E.
    /// Indexing (`ef[e]`) yields the face-id slice, so callers are unchanged.
    pub fn edge_faces(&self) -> EdgeFaces {
        let ne = self.edges.len();
        // Pass 1: count distinct faces per edge. Faces are visited in ascending
        // id order, so an edge's uses within one face are contiguous — dedup by
        // "last face seen" reproduces the old `contains` check exactly.
        let mut counts = vec![0u32; ne];
        let mut last = vec![usize::MAX; ne];
        for fi in 0..self.faces.len() {
            for lp in self.face_loops(fi) {
                for &(e, _) in lp {
                    if last[e] != fi {
                        last[e] = fi;
                        counts[e] += 1;
                    }
                }
            }
        }
        let mut off = vec![0u32; ne + 1];
        for e in 0..ne {
            off[e + 1] = off[e] + counts[e];
        }
        // Pass 2: scatter face ids into the flat array at each edge's cursor.
        let mut flat = vec![0usize; off[ne] as usize];
        let mut cursor: Vec<u32> = off[..ne].to_vec();
        last.iter_mut().for_each(|x| *x = usize::MAX);
        for fi in 0..self.faces.len() {
            for lp in self.face_loops(fi) {
                for &(e, _) in lp {
                    if last[e] != fi {
                        last[e] = fi;
                        flat[cursor[e] as usize] = fi;
                        cursor[e] += 1;
                    }
                }
            }
        }
        EdgeFaces { off, flat }
    }
}

/// Flat CSR of the faces touching each edge (see [`Solid::edge_faces`]).
/// `ef[e]` is the slice of face ids for edge `e`.
#[derive(Clone, Debug)]
pub struct EdgeFaces {
    off: Vec<u32>,
    flat: Vec<usize>,
}

impl std::ops::Index<usize> for EdgeFaces {
    type Output = [usize];
    fn index(&self, e: usize) -> &[usize] {
        &self.flat[self.off[e] as usize..self.off[e + 1] as usize]
    }
}

/// Constructs a `Solid` while deduplicating coincident vertices and shared
/// edges, so faces built independently still reference the same topology.
#[derive(Default)]
pub struct Builder {
    verts: Vec<Vertex>,
    vert_index: HashMap<(i64, i64, i64), VertexId>,
    edges: Vec<Edge>,
    // Keyed on (sorted endpoints, welded midpoint): two arcs that share endpoints
    // but bulge differently (a circle's two semicircles) stay distinct, while a
    // genuinely shared edge and its reverse still merge.
    edge_index: HashMap<(VertexId, VertexId, (i64, i64, i64)), EdgeId>,
    loop_edges: Vec<(EdgeId, bool)>,
    loops: Vec<u32>,
    faces: Vec<Face>,
}

impl Builder {
    pub fn new() -> Builder {
        Builder { loops: vec![0], ..Builder::default() }
    }

    /// A builder pre-sized for a known output. `fillet_edges`/`chamfer` rebuild a
    /// whole solid through a fresh builder; sizing the arenas and the two intern
    /// maps up front removes the incremental table rehash and arena regrowth that
    /// otherwise dominates the rebuild's allocation churn.
    pub fn with_capacity(nv: usize, ne: usize, nloops: usize, nloop_edges: usize, nfaces: usize) -> Builder {
        let mut loops = Vec::with_capacity(nloops + 1);
        loops.push(0);
        Builder {
            verts: Vec::with_capacity(nv),
            vert_index: HashMap::with_capacity(nv),
            edges: Vec::with_capacity(ne),
            edge_index: HashMap::with_capacity(ne),
            loop_edges: Vec::with_capacity(nloop_edges),
            loops,
            faces: Vec::with_capacity(nfaces),
        }
    }

    /// Append a loop's directed edges to the flat arena, returning its loop id.
    fn intern_loop(&mut self, edges: &[(EdgeId, bool)]) -> u32 {
        let lid = self.loops.len() as u32 - 1;
        self.loop_edges.extend_from_slice(edges);
        self.loops.push(self.loop_edges.len() as u32);
        lid
    }

    /// Intern a vertex by welded position.
    pub fn vertex(&mut self, p: Vec3) -> VertexId {
        crate::kernel::perf::count(crate::kernel::perf::Metric::BuilderVertex);
        *self.vert_index.entry(weld_key(p)).or_insert_with(|| {
            let id = self.verts.len();
            self.verts.push(Vertex { point: p });
            id
        })
    }

    /// Intern a straight edge between two vertices; returns the edge id and the
    /// direction (`true` if the stored edge already runs `a → b`).
    pub fn line(&mut self, a: VertexId, b: VertexId) -> (EdgeId, bool) {
        let (pa, pb) = (self.verts[a].point, self.verts[b].point);
        let mid = (pa + pb) * 0.5;
        self.edge_between(a, b, mid, || {
            let dir = (pb - pa).normalize_or_zero();
            Edge {
                curve: Curve::Line { p0: pa, dir },
                t0: 0.0,
                t1: (pb - pa).length(),
                v0: a,
                v1: b,
            }
        })
    }

    /// Intern a circular-arc edge. `center`/`axis`/`radius`/`ref_dir` define the
    /// circle; `a0`/`a1` are the raw (unwrapped) angles at `a`/`b`.
    pub fn arc(
        &mut self,
        a: VertexId,
        b: VertexId,
        center: Vec3,
        axis: Vec3,
        radius: f32,
        ref_dir: Vec3,
        a0: f32,
        a1: f32,
    ) -> (EdgeId, bool) {
        crate::kernel::perf::count(crate::kernel::perf::Metric::BuilderArc);
        let curve = Curve::Circle { center, axis, radius, ref_dir };
        let mid = curve.point((a0 + a1) * 0.5);
        self.edge_between(a, b, mid, || Edge {
            curve,
            t0: a0,
            t1: a1,
            v0: a,
            v1: b,
        })
    }

    /// Intern an ellipse-arc edge (`p(t) = center + cos t·ea + sin t·eb`);
    /// `t0`/`t1` are the raw parameters at `a`/`b`.
    #[allow(clippy::too_many_arguments)]
    pub fn ellipse(
        &mut self,
        a: VertexId,
        b: VertexId,
        center: Vec3,
        ea: Vec3,
        eb: Vec3,
        t0: f32,
        t1: f32,
    ) -> (EdgeId, bool) {
        let curve = Curve::Ellipse { center, a: ea, b: eb };
        let mid = curve.point((t0 + t1) * 0.5);
        self.edge_between(a, b, mid, || Edge {
            curve,
            t0,
            t1,
            v0: a,
            v1: b,
        })
    }

    fn edge_between(
        &mut self,
        a: VertexId,
        b: VertexId,
        mid: crate::kernel::math::Vec3,
        make: impl FnOnce() -> Edge,
    ) -> (EdgeId, bool) {
        let (lo, hi) = if a < b { (a, b) } else { (b, a) };
        let key = (lo, hi, weld_key(mid));
        if let Some(&e) = self.edge_index.get(&key) {
            // Match direction against how the edge was stored.
            return (e, self.edges[e].v0 == a);
        }
        let id = self.edges.len();
        self.edges.push(make());
        self.edge_index.insert(key, id);
        (id, true)
    }

    pub fn face(&mut self, surface: Surface, sense: bool, outer: Loop, inners: Vec<Loop>) -> usize {
        let inner_slices: Vec<&[(EdgeId, bool)]> = inners.iter().map(|l| l.edges.as_slice()).collect();
        self.face_from(surface, sense, &outer.edges, &inner_slices)
    }

    /// Like [`Builder::face`] but takes borrowed directed-edge slices instead of
    /// owned `Loop`s, so a caller that already holds the edge lists in reusable
    /// scratch buffers (the fillet/chamfer rebuild) needn't allocate a `Loop`
    /// per loop. The loops are interned contiguously, outer first.
    pub fn face_from(
        &mut self,
        surface: Surface,
        sense: bool,
        outer: &[(EdgeId, bool)],
        inners: &[&[(EdgeId, bool)]],
    ) -> usize {
        crate::kernel::perf::count(crate::kernel::perf::Metric::BuilderFace);
        let loop0 = self.intern_loop(outer);
        for inn in inners {
            self.intern_loop(inn);
        }
        let id = self.faces.len();
        self.faces.push(Face {
            surface,
            sense,
            loop0,
            n_loops: 1 + inners.len() as u32,
        });
        id
    }

    pub fn edge(&self, id: EdgeId) -> Edge {
        self.edges[id]
    }

    pub fn point(&self, id: VertexId) -> Vec3 {
        self.verts[id].point
    }

    pub fn build(self) -> Solid {
        Solid {
            verts: self.verts,
            edges: self.edges,
            loop_edges: self.loop_edges,
            loops: self.loops,
            faces: self.faces,
        }
    }
}
