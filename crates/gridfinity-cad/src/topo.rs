//! Boundary-representation topology: an explicit face / loop / edge / vertex
//! structure with shared edges. Each `Edge` is referenced by the faces on both
//! its sides; each `Face` is trimmed by ordered loops of directed edges. This is
//! a genuine B-rep (topology + analytic geometry, shared edges), just without
//! half-edge next/prev pointers — loops are stored as explicit ordered edge
//! lists, which makes constructing correct shared topology far less error-prone.
//!
//! The manifold invariant (`validate`): every edge is used exactly twice across
//! all loops, once in each direction.

use crate::geom::{Curve, Surface};
use crate::math::{Vec3, weld_key};
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
            Curve::Circle { .. } => {
                let sweep = (self.t1 - self.t0).abs();
                ((sweep / (std::f32::consts::PI / 2.0)) * arc_segs_per_quarter as f32)
                    .ceil()
                    .max(1.0) as usize
            }
        }
    }
}

/// An ordered ring of directed edges. `true` = traverse the edge `v0 → v1`.
#[derive(Clone, Debug, Default)]
pub struct Loop {
    pub edges: Vec<(EdgeId, bool)>,
}

impl Loop {
    pub fn new(edges: Vec<(EdgeId, bool)>) -> Loop {
        Loop { edges }
    }
}

/// A trimmed face: an analytic `surface`, an outer loop, and zero or more inner
/// (hole) loops. `sense` = whether the surface's own normal is the outward
/// (solid-exterior) normal; `false` flips it.
#[derive(Clone, Debug)]
pub struct Face {
    pub surface: Surface,
    pub sense: bool,
    pub outer: Loop,
    pub inners: Vec<Loop>,
}

impl Face {
    pub fn loops(&self) -> impl Iterator<Item = &Loop> {
        std::iter::once(&self.outer).chain(self.inners.iter())
    }
}

/// A solid: arenas of vertices, edges, and faces. One shell, manifold.
#[derive(Clone, Debug, Default)]
pub struct Solid {
    pub verts: Vec<Vertex>,
    pub edges: Vec<Edge>,
    pub faces: Vec<Face>,
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

    /// Check the manifold invariant: every edge used exactly twice, once each
    /// direction, and each loop forms a closed chain (consecutive edges meet).
    pub fn validate(&self) -> Result<(), String> {
        let mut fwd = vec![0u32; self.edges.len()];
        let mut bwd = vec![0u32; self.edges.len()];
        for (fi, face) in self.faces.iter().enumerate() {
            for lp in face.loops() {
                if lp.edges.is_empty() {
                    return Err(format!("face {fi} has an empty loop"));
                }
                for w in 0..lp.edges.len() {
                    let (e, f) = lp.edges[w];
                    if f { fwd[e] += 1 } else { bwd[e] += 1 }
                    let (_, end) = self.directed(e, f);
                    let (ne, nf) = lp.edges[(w + 1) % lp.edges.len()];
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
    pub fn edge_faces(&self) -> Vec<Vec<usize>> {
        let mut map = vec![Vec::new(); self.edges.len()];
        for (fi, face) in self.faces.iter().enumerate() {
            for lp in face.loops() {
                for &(e, _) in &lp.edges {
                    if !map[e].contains(&fi) {
                        map[e].push(fi);
                    }
                }
            }
        }
        map
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
    faces: Vec<Face>,
}

impl Builder {
    pub fn new() -> Builder {
        Builder::default()
    }

    /// Intern a vertex by welded position.
    pub fn vertex(&mut self, p: Vec3) -> VertexId {
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

    /// Intern a circular-arc edge. `center`/`radius`/`ref_dir` define the
    /// circle; `a0`/`a1` are the raw (unwrapped) angles at `a`/`b`.
    pub fn arc(
        &mut self,
        a: VertexId,
        b: VertexId,
        center: Vec3,
        radius: f32,
        ref_dir: Vec3,
        a0: f32,
        a1: f32,
    ) -> (EdgeId, bool) {
        let curve = Curve::Circle { center, radius, ref_dir };
        let mid = curve.point((a0 + a1) * 0.5);
        self.edge_between(a, b, mid, || Edge {
            curve,
            t0: a0,
            t1: a1,
            v0: a,
            v1: b,
        })
    }

    fn edge_between(
        &mut self,
        a: VertexId,
        b: VertexId,
        mid: crate::math::Vec3,
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
        let id = self.faces.len();
        self.faces.push(Face {
            surface,
            sense,
            outer,
            inners,
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
            faces: self.faces,
        }
    }
}
