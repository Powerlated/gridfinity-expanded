
use crate::kernel::geom::{Curve, Surface};
use crate::kernel::math::{Vec3, weld_key};
use crate::kernel::hash::FxHashMap;

pub type VertexId = usize;
pub type EdgeId = usize;

#[derive(Clone, Copy, Debug)]
pub struct Vertex {
    pub point: Vec3,
}

#[derive(Clone, Copy, Debug)]
pub struct Edge {
    pub curve: Curve,
    pub t0: f32,
    pub t1: f32,
    pub v0: VertexId,
    pub v1: VertexId,
}

impl Edge {
    pub fn sample(&self, forward: bool, n: usize) -> Vec<Vec3> {
        let mut out = Vec::with_capacity(n + 1);
        self.sample_into(forward, n, &mut out);
        out
    }

    pub fn sample_into(&self, forward: bool, n: usize, out: &mut Vec<Vec3>) {
        let (a, b) = if forward {
            (self.t0, self.t1)
        } else {
            (self.t1, self.t0)
        };
        out.reserve(n + 1);
        for i in 0..=n {
            let t = a + (b - a) * (i as f32 / n as f32);
            out.push(self.curve.point(t));
        }
    }

    pub fn seg_count(&self, arc_segs_per_quarter: usize) -> usize {
        match self.curve {
            Curve::Line { .. } => 1,
            Curve::Circle { .. } | Curve::Ellipse { .. } | Curve::TorusSection { .. } => {
                let sweep = (self.t1 - self.t0).abs();
                let exact = (sweep / (std::f32::consts::PI / 2.0)) * arc_segs_per_quarter as f32;
                (exact - 1e-3).ceil().max(1.0) as usize
            }
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct Loop {
    pub edges: Vec<(EdgeId, bool)>,
}

impl Loop {
    pub fn new(edges: Vec<(EdgeId, bool)>) -> Loop {
        Loop { edges }
    }
}

#[derive(Clone, Debug)]
pub struct Face {
    pub surface: Surface,
    pub sense: bool,
    loop0: u32,
    n_loops: u32,
}

#[derive(Clone, Debug)]
pub struct Solid {
    pub verts: Vec<Vertex>,
    pub edges: Vec<Edge>,
    loop_edges: Vec<(EdgeId, bool)>,
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

    pub fn directed(&self, edge: EdgeId, forward: bool) -> (VertexId, VertexId) {
        let e = &self.edges[edge];
        if forward { (e.v0, e.v1) } else { (e.v1, e.v0) }
    }

    fn loop_slice(&self, lid: u32) -> &[(EdgeId, bool)] {
        let s = self.loops[lid as usize] as usize;
        let e = self.loops[lid as usize + 1] as usize;
        &self.loop_edges[s..e]
    }

    pub fn outer_edges(&self, fid: usize) -> &[(EdgeId, bool)] {
        self.loop_slice(self.faces[fid].loop0)
    }

    pub fn loop_edges_len(&self) -> usize {
        self.loop_edges.len()
    }

    pub fn n_inners(&self, fid: usize) -> usize {
        self.faces[fid].n_loops as usize - 1
    }

    pub fn face_loops(&self, fid: usize) -> impl Iterator<Item = &[(EdgeId, bool)]> {
        let f = &self.faces[fid];
        (f.loop0..f.loop0 + f.n_loops).map(move |lid| self.loop_slice(lid))
    }

    pub fn loop_ids(&self, fid: usize) -> std::ops::Range<u32> {
        let f = &self.faces[fid];
        f.loop0..f.loop0 + f.n_loops
    }

    pub fn loop_by_id(&self, lid: u32) -> &[(EdgeId, bool)] {
        self.loop_slice(lid)
    }

    pub fn reverse_loop(&mut self, lid: u32) {
        let s = self.loops[lid as usize] as usize;
        let e = self.loops[lid as usize + 1] as usize;
        self.loop_edges[s..e].reverse();
        for d in &mut self.loop_edges[s..e] {
            d.1 = !d.1;
        }
    }

    pub fn inner_loops(&self, fid: usize) -> impl Iterator<Item = &[(EdgeId, bool)]> {
        let f = &self.faces[fid];
        (f.loop0 + 1..f.loop0 + f.n_loops).map(move |lid| self.loop_slice(lid))
    }

    pub fn validate(&self) -> Result<(), String> {
        self.manifold_check(false)
    }

    pub fn validate_ignoring_unused_edges(&self) -> Result<(), String> {
        self.manifold_check(true)
    }

    fn manifold_check(&self, allow_unused: bool) -> Result<(), String> {
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
            if allow_unused && fwd[e] == 0 && bwd[e] == 0 {
                continue;
            }
            if fwd[e] != 1 || bwd[e] != 1 {
                return Err(format!(
                    "edge {e} used fwd={} bwd={} (want 1/1)",
                    fwd[e], bwd[e]
                ));
            }
        }
        Ok(())
    }

    pub fn edge_faces(&self) -> EdgeFaces {
        let ne = self.edges.len();
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

    fn compact_edges(&mut self) {
        let mut used = vec![false; self.edges.len()];
        for &(e, _) in &self.loop_edges {
            used[e] = true;
        }
        if used.iter().all(|u| *u) {
            return;
        }
        let mut remap = vec![usize::MAX; self.edges.len()];
        let mut next = 0usize;
        for e in 0..self.edges.len() {
            if used[e] {
                remap[e] = next;
                self.edges.swap(next, e);
                next += 1;
            }
        }
        self.edges.truncate(next);
        for (e, _) in &mut self.loop_edges {
            *e = remap[*e];
        }
    }
}

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


#[derive(Default)]
pub struct Builder {
    verts: Vec<Vertex>,
    vert_index: FxHashMap<(i64, i64, i64), VertexId>,
    edges: Vec<Edge>,
    edge_index: FxHashMap<(VertexId, VertexId, (i64, i64, i64)), EdgeId>,
    loop_edges: Vec<(EdgeId, bool)>,
    loops: Vec<u32>,
    faces: Vec<Face>,
}

impl Builder {
    pub fn new() -> Builder {
        Builder { loops: vec![0], ..Builder::default() }
    }

    pub fn with_capacity(nv: usize, ne: usize, nloops: usize, nloop_edges: usize, nfaces: usize) -> Builder {
        let mut loops = Vec::with_capacity(nloops + 1);
        loops.push(0);
        Builder {
            verts: Vec::with_capacity(nv),
            vert_index: FxHashMap::with_capacity_and_hasher(nv, Default::default()),
            edges: Vec::with_capacity(ne),
            edge_index: FxHashMap::with_capacity_and_hasher(ne, Default::default()),
            loop_edges: Vec::with_capacity(nloop_edges),
            loops,
            faces: Vec::with_capacity(nfaces),
        }
    }

    pub fn resume(solid: &Solid, seed: &[bool]) -> Builder {
        let mut b = Builder {
            verts: solid.verts.clone(),
            edges: solid.edges.clone(),
            vert_index: FxHashMap::default(),
            edge_index: FxHashMap::default(),
            loop_edges: Vec::with_capacity(solid.loop_edges.len()),
            loops: {
                let mut l = Vec::with_capacity(solid.loops.len());
                l.push(0);
                l
            },
            faces: Vec::with_capacity(solid.faces.len()),
        };
        for (fi, &wanted) in seed.iter().enumerate() {
            if !wanted {
                continue;
            }
            for lp in solid.face_loops(fi) {
                for &(e, _) in lp {
                    let ed = solid.edges[e];
                    for v in [ed.v0, ed.v1] {
                        b.vert_index.entry(weld_key(solid.verts[v].point)).or_insert(v);
                    }
                    let (lo, hi) = if ed.v0 < ed.v1 { (ed.v0, ed.v1) } else { (ed.v1, ed.v0) };
                    let mid = match ed.curve {
                        Curve::Line { .. } => {
                            (solid.verts[ed.v0].point + solid.verts[ed.v1].point) * 0.5
                        }
                        _ => ed.curve.point((ed.t0 + ed.t1) * 0.5),
                    };
                    b.edge_index.entry((lo, hi, weld_key(mid))).or_insert(e);
                }
            }
        }
        b
    }

    pub fn copy_face(&mut self, solid: &Solid, fid: usize) -> usize {
        let f = &solid.faces[fid];
        let loop0 = self.loops.len() as u32 - 1;
        for lp in solid.face_loops(fid) {
            self.loop_edges.extend_from_slice(lp);
            self.loops.push(self.loop_edges.len() as u32);
        }
        let id = self.faces.len();
        self.faces.push(Face {
            surface: f.surface,
            sense: f.sense,
            loop0,
            n_loops: f.n_loops,
        });
        id
    }

    fn intern_loop(&mut self, edges: &[(EdgeId, bool)]) -> u32 {
        let lid = self.loops.len() as u32 - 1;
        self.loop_edges.extend_from_slice(edges);
        self.loops.push(self.loop_edges.len() as u32);
        lid
    }

    /// The edge already interned between `a` and `b` through `mid`, without
    /// interning anything. A selection that names geometry the build did not
    /// produce must come back empty: interning it would mint an edge no face
    /// uses, which fails `validate` whatever the caller then does with it.
    pub fn find_edge(&self, a: Vec3, b: Vec3, mid: Vec3) -> Option<EdgeId> {
        let va = *self.vert_index.get(&weld_key(a))?;
        let vb = *self.vert_index.get(&weld_key(b))?;
        let (lo, hi) = if va < vb { (va, vb) } else { (vb, va) };
        self.edge_index.get(&(lo, hi, weld_key(mid))).copied()
    }

    pub fn vertex(&mut self, p: Vec3) -> VertexId {
        crate::kernel::perf::count(crate::kernel::perf::Metric::BuilderVertex);
        *self.vert_index.entry(weld_key(p)).or_insert_with(|| {
            let id = self.verts.len();
            self.verts.push(Vertex { point: p });
            id
        })
    }

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

    pub fn torus_section(
        &mut self,
        a: VertexId,
        b: VertexId,
        curve: Curve,
        t0: f32,
        t1: f32,
    ) -> (EdgeId, bool) {
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

    pub fn directed_ends(&self, d: (EdgeId, bool)) -> (VertexId, VertexId) {
        let e = self.edges[d.0];
        if d.1 { (e.v0, e.v1) } else { (e.v1, e.v0) }
    }

    pub fn edge(&self, id: EdgeId) -> Edge {
        self.edges[id]
    }

    pub fn point(&self, id: VertexId) -> Vec3 {
        self.verts[id].point
    }

    pub fn build(self) -> Solid {
        let solid = self.build_unvalidated();
        if let Err(e) = solid.validate_ignoring_unused_edges() {
            panic!("Builder::build produced a non-manifold solid: {e}");
        }
        solid
    }

    pub fn build_unvalidated(self) -> Solid {
        let mut solid = Solid {
            verts: self.verts,
            edges: self.edges,
            loop_edges: self.loop_edges,
            loops: self.loops,
            faces: self.faces,
        };
        crate::kernel::orient::normalize(&mut solid);
        solid
    }

    pub fn build_compact_unvalidated(self) -> Solid {
        let mut s = self.build_unvalidated();
        s.compact_edges();
        s
    }
}
