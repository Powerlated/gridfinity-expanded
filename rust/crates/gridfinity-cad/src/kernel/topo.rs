use crate::kernel::geom::{Curve, Surface};
use crate::kernel::hash::FxHashMap;
use crate::kernel::math::{Vec3, WELD_NEAR_SQ, weld_key};

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

/// How far apart two faces' planes may be and still be the same plane, as a
/// distance in millimetres and as a sine of the angle between their normals.
///
/// Both bound quantities the model builds exactly: coplanar faces here are one
/// flat wall that an outline split cut into bands, so they carry the *same*
/// plane through the same arithmetic and differ only in the last bits of an
/// `f32`. These are four orders above that noise and four below the thinnest
/// feature the model makes, so nothing that is genuinely two planes can pass.
const SAME_PLANE_DIST: f32 = 1e-4;
const SAME_PLANE_SIN: f32 = 1e-5;

impl Solid {
    pub fn vertex(&self, id: VertexId) -> Vec3 {
        self.verts[id].point
    }

    /// The outward normal of face `fi`, which is its surface's normal turned
    /// around when the face is used against its surface's own sense.
    fn outward_normal(&self, fi: usize) -> Vec3 {
        let f = &self.faces[fi];
        let n = match f.surface {
            Surface::Plane { normal, .. } => normal,
            _ => return Vec3::ZERO,
        };
        let n = n.normalize_or_zero();
        if f.sense { n } else { -n }
    }

    /// Whether faces `a` and `b` are two pieces of one flat surface: the same
    /// plane, facing the same way. Two coplanar faces facing *opposite* ways are
    /// the two sides of a zero-thickness sliver and must never be fused.
    fn same_plane(&self, a: usize, b: usize) -> bool {
        let (Surface::Plane { origin: oa, .. }, Surface::Plane { origin: ob, .. }) =
            (self.faces[a].surface, self.faces[b].surface)
        else {
            return false;
        };
        let (na, nb) = (self.outward_normal(a), self.outward_normal(b));
        if na == Vec3::ZERO || nb == Vec3::ZERO {
            return false;
        }
        // The offset allowance carries the lever arm. Two faces whose normals
        // agree only to `SAME_PLANE_SIN` and whose origins are 73 mm apart are
        // already that far times the angle out of each other's plane, with
        // nothing wrong: it is the same plane, sampled at two distant points
        // through `f32`. A flat allowance calls that a second plane, and
        // does so more readily the larger the bin.
        let arm = (ob - oa).length();
        na.cross(nb).length() <= SAME_PLANE_SIN
            && na.dot(nb) > 0.0
            && (ob - oa).dot(na).abs() <= SAME_PLANE_DIST + arm * SAME_PLANE_SIN
    }

    /// Fuse neighbouring faces that lie in one plane and face the same way,
    /// dissolving the edges between them.
    ///
    /// The bin's flat outer wall reaches the fillet as a row of narrow bands,
    /// because `split_outline_at` cuts the outline wherever the peg profile or
    /// an opening needs a point and every cut runs the wall's full height. Those
    /// seams are not geometry -- the wall either side of one is the same plane
    /// facing the same way -- but a fillet cannot tell that, and a blend running
    /// out across a 1.55 mm band has to retreat past the far side of a face that
    /// only exists as an artefact of the outline. Fusing them first is what
    /// gives the retreat somewhere to land, and it is a simplification of the
    /// B-rep rather than a change to it: the solid occupies the same space, with
    /// fewer faces describing the same surface.
    ///
    /// `keep` names edges that must survive -- the blended ones, whose ids the
    /// caller has already resolved and whose two faces the blend needs.
    /// Vertex and edge numbering is untouched, so every id the caller holds
    /// stays valid; only loops and faces are rewritten.
    ///
    /// A group whose boundary cannot be chained unambiguously is left alone. The
    /// merge is an optimisation and no result may depend on it happening.
    pub fn merge_coplanar_faces(&self, keep: &[EdgeId]) -> Solid {
        let ef = self.edge_faces();
        let mut parent: Vec<usize> = (0..self.faces.len()).collect();
        fn find(parent: &mut [usize], i: usize) -> usize {
            let mut r = i;
            while parent[r] != r {
                r = parent[r];
            }
            let mut c = i;
            while parent[c] != c {
                let n = parent[c];
                parent[c] = r;
                c = n;
            }
            r
        }
        let mut dissolved = vec![false; self.edges.len()];
        for e in 0..self.edges.len() {
            let faces = &ef[e];
            if faces.len() != 2 || keep.contains(&e) {
                continue;
            }
            let (f, g) = (faces[0], faces[1]);
            if !self.same_plane(f, g) {
                continue;
            }
            dissolved[e] = true;
            let (rf, rg) = (find(&mut parent, f), find(&mut parent, g));
            parent[rf] = rg;
        }
        if !dissolved.iter().any(|&d| d) {
            return self.fuse_collinear_edges(keep);
        }

        let mut groups: FxHashMap<usize, Vec<usize>> = Default::default();
        for fi in 0..self.faces.len() {
            let r = find(&mut parent, fi);
            groups.entry(r).or_default().push(fi);
        }
        let mut roots: Vec<usize> = groups.keys().copied().collect();
        roots.sort_unstable();

        let mut out = Solid {
            verts: self.verts.clone(),
            edges: self.edges.clone(),
            loop_edges: Vec::with_capacity(self.loop_edges.len()),
            loops: vec![0],
            faces: Vec::with_capacity(self.faces.len()),
        };
        let mut push_face = |out: &mut Solid, surface, sense, lps: &[Vec<(EdgeId, bool)>]| {
            let loop0 = out.loops.len() as u32 - 1;
            for lp in lps {
                out.loop_edges.extend_from_slice(lp);
                out.loops.push(out.loop_edges.len() as u32);
            }
            out.faces.push(Face {
                surface,
                sense,
                loop0,
                n_loops: lps.len() as u32,
            });
        };

        for r in roots {
            let members = &groups[&r];
            let copy_as_is =
                |out: &mut Solid, push: &mut dyn FnMut(&mut Solid, _, _, &[Vec<_>])| {
                    for &fi in members {
                        let lps: Vec<Vec<(EdgeId, bool)>> =
                            self.face_loops(fi).map(|lp| lp.to_vec()).collect();
                        push(out, self.faces[fi].surface, self.faces[fi].sense, &lps);
                    }
                };
            if members.len() == 1 {
                copy_as_is(&mut out, &mut push_face);
                continue;
            }
            // Coplanarity is transitive in exact arithmetic, and the union-find
            // treats it as if it were: a group is grown one neighbour at a time,
            // so a long row of bands could in principle walk `SAME_PLANE_DIST`
            // per step and end up describing a plane the far members are not on.
            // The merged face carries *one* surface, so they all have to lie on
            // that one.
            let rep = members[0];
            for &fi in members {
                assert!(
                    self.same_plane(rep, fi),
                    "merge: face {fi} was fused into the group led by {rep} but is not on \
                     its plane ({:?} vs {:?})",
                    self.faces[fi].surface,
                    self.faces[rep].surface
                );
            }
            // The group's boundary is every directed edge of every member that
            // the group does not also traverse the other way. A dissolved seam
            // is traversed twice, once from each side, and cancels.
            let mut boundary: Vec<(EdgeId, bool)> = Vec::new();
            for &fi in members {
                for &(e, fwd) in self.face_loops(fi).flatten() {
                    if !dissolved[e] {
                        boundary.push((e, fwd));
                    }
                }
            }
            match self.chain_boundary(&boundary) {
                Some(lps) => {
                    let kept: usize = lps.iter().map(|l| l.len()).sum();
                    assert!(
                        kept == boundary.len(),
                        "merge: the group led by {rep} had {} boundary edges but chained {kept}",
                        boundary.len()
                    );
                    push_face(
                        &mut out,
                        self.faces[rep].surface,
                        self.faces[rep].sense,
                        &lps,
                    );
                }
                None => copy_as_is(&mut out, &mut push_face),
            }
        }
        out.fuse_collinear_edges(keep)
    }

    /// Fuse consecutive collinear edges that divide the same two faces.
    ///
    /// Dissolving the wall's banding seams leaves its *top* still cut into one
    /// edge per band: those pieces are collinear, they separate the same pair of
    /// faces, and the vertices between them carry nothing else. They are one
    /// edge described three times, and a blend retreating along them runs out
    /// after the first piece for no reason in the geometry.
    ///
    /// Only straight edges fuse here. Two arcs of one circle are the same
    /// situation and the outline splits produce them too, but a merged arc has
    /// to agree on a sweep direction as well as a support, so it is a separate
    /// step and not one this needs.
    ///
    /// The surviving edge keeps the lower id and is rewritten to span the whole
    /// run; the others simply stop being referenced. `keep` is held back, so no
    /// id the caller resolved before this moves.
    fn fuse_collinear_edges(&self, keep: &[EdgeId]) -> Solid {
        let ef = self.edge_faces();
        let mut incident: FxHashMap<VertexId, Vec<EdgeId>> = Default::default();
        for lp in (0..self.faces.len()).flat_map(|fi| self.face_loops(fi)) {
            for &(e, _) in lp {
                for v in [self.edges[e].v0, self.edges[e].v1] {
                    let slot = incident.entry(v).or_default();
                    if !slot.contains(&e) {
                        slot.push(e);
                    }
                }
            }
        }
        let dir_of = |e: EdgeId| match self.edges[e].curve {
            Curve::Line { dir, .. } => Some(dir.normalize_or_zero()),
            _ => None,
        };
        let mut parent: Vec<usize> = (0..self.edges.len()).collect();
        fn find(parent: &mut [usize], i: usize) -> usize {
            let mut r = i;
            while parent[r] != r {
                r = parent[r];
            }
            r
        }
        let mut fused_any = false;
        let mut vs: Vec<VertexId> = incident.keys().copied().collect();
        vs.sort_unstable();
        for v in vs {
            let es = &incident[&v];
            if es.len() != 2 {
                continue;
            }
            let (a, z) = (es[0], es[1]);
            if keep.contains(&a) || keep.contains(&z) {
                continue;
            }
            let (Some(da), Some(dz)) = (dir_of(a), dir_of(z)) else {
                continue;
            };
            if da.cross(dz).length() > SAME_PLANE_SIN {
                continue;
            }
            // Collinear is not enough: the run has to pass *through* the vertex.
            // Two edges leaving it the same way lie on top of one another, and
            // fusing those describes a span neither of them covers.
            let far = |e: EdgeId| {
                let ed = self.edges[e];
                let o = if ed.v0 == v { ed.v1 } else { ed.v0 };
                self.verts[o].point - self.verts[v].point
            };
            if far(a).dot(far(z)) >= 0.0 {
                continue;
            }
            let (mut fa, mut fz) = (ef[a].to_vec(), ef[z].to_vec());
            fa.sort_unstable();
            fz.sort_unstable();
            if fa.len() != 2 || fa != fz {
                continue;
            }
            fused_any = true;
            let (ra, rz) = (find(&mut parent, a), find(&mut parent, z));
            parent[ra.max(rz)] = ra.min(rz);
        }
        if !fused_any {
            return self.clone();
        }

        let roots: Vec<usize> = (0..self.edges.len())
            .map(|e| find(&mut parent, e))
            .collect();

        // Pass one: cut every loop into runs of consecutive entries that belong
        // to one fused edge, and record which vertices each run ran between.
        // A run bounded by two faces is walked once from each, in opposite
        // directions, so the two occurrences report reversed spans.
        struct Run {
            edge: EdgeId,
            from: VertexId,
            to: VertexId,
            fused: bool,
        }
        let mut per_face: Vec<Vec<Vec<Run>>> = Vec::with_capacity(self.faces.len());
        for fi in 0..self.faces.len() {
            let mut lps: Vec<Vec<Run>> = Vec::new();
            for lp in self.face_loops(fi) {
                let n = lp.len();
                // Begin where a run begins, so none straddles the wrap and is
                // emitted as two.
                let start = (0..n)
                    .find(|&i| roots[lp[i].0] != roots[lp[(i + n - 1) % n].0])
                    .unwrap_or(0);
                let mut runs: Vec<Run> = Vec::with_capacity(n);
                let mut i = 0;
                while i < n {
                    let at = (start + i) % n;
                    let r = roots[lp[at].0];
                    let (from, mut to) = self.directed(lp[at].0, lp[at].1);
                    let mut j = i + 1;
                    while j < n && roots[lp[(start + j) % n].0] == r {
                        to = self
                            .directed(lp[(start + j) % n].0, lp[(start + j) % n].1)
                            .1;
                        j += 1;
                    }
                    runs.push(Run {
                        edge: if j == i + 1 { lp[at].0 } else { r },
                        from,
                        to,
                        fused: j > i + 1,
                    });
                    i = j;
                }
                lps.push(runs);
            }
            per_face.push(lps);
        }

        // Pass two: the first face to report a run fixes its direction, and the
        // face across it traverses the same edge backwards.
        let mut spans: FxHashMap<EdgeId, (VertexId, VertexId)> = Default::default();
        for runs in per_face.iter().flatten().flatten() {
            if runs.fused {
                spans.entry(runs.edge).or_insert((runs.from, runs.to));
            }
        }

        let mut out = self.clone();
        for (&e, &(a, z)) in &spans {
            let (pa, pz) = (self.verts[a].point, self.verts[z].point);
            out.edges[e] = Edge {
                curve: Curve::Line {
                    p0: pa,
                    dir: (pz - pa).normalize_or_zero(),
                },
                t0: 0.0,
                t1: (pz - pa).length(),
                v0: a,
                v1: z,
            };
        }
        out.loop_edges.clear();
        out.loops = vec![0];
        out.faces.clear();
        for (fi, lps) in per_face.into_iter().enumerate() {
            let loop0 = out.loops.len() as u32 - 1;
            let n_loops = lps.len() as u32;
            for runs in lps {
                for run in runs {
                    let fwd = match spans.get(&run.edge) {
                        Some(&(a, z)) => {
                            assert!(
                                (a, z) == (run.from, run.to) || (z, a) == (run.from, run.to),
                                "fuse: edge {} runs {}->{} on face {fi} but {a}->{z} \
                                 on the face across it",
                                run.edge,
                                run.from,
                                run.to
                            );
                            (a, z) == (run.from, run.to)
                        }
                        None => self.edges[run.edge].v0 == run.from,
                    };
                    out.loop_edges.push((run.edge, fwd));
                }
                out.loops.push(out.loop_edges.len() as u32);
            }
            out.faces.push(Face {
                surface: self.faces[fi].surface,
                sense: self.faces[fi].sense,
                loop0,
                n_loops,
            });
        }
        out
    }

    /// Chain a set of directed edges into closed loops, largest first.
    ///
    /// `None` when the set is not a disjoint union of simple closed walks --
    /// where two boundary edges leave one vertex there is no one answer, and
    /// guessing would silently reshape the face.
    fn chain_boundary(&self, boundary: &[(EdgeId, bool)]) -> Option<Vec<Vec<(EdgeId, bool)>>> {
        let mut next: FxHashMap<VertexId, (EdgeId, bool)> = Default::default();
        for &(e, fwd) in boundary {
            let (start, _) = self.directed(e, fwd);
            if next.insert(start, (e, fwd)).is_some() {
                return None;
            }
        }
        let mut used: FxHashMap<EdgeId, bool> = Default::default();
        let mut lps: Vec<Vec<(EdgeId, bool)>> = Vec::new();
        for &(e0, f0) in boundary {
            if used.contains_key(&e0) {
                continue;
            }
            let mut lp = Vec::new();
            let (from0, _) = self.directed(e0, f0);
            let (mut cur, mut curf) = (e0, f0);
            loop {
                if used.insert(cur, true).is_some() {
                    return None;
                }
                lp.push((cur, curf));
                let (_, end) = self.directed(cur, curf);
                if end == from0 {
                    break;
                }
                let &(ne, nf) = next.get(&end)?;
                (cur, curf) = (ne, nf);
            }
            let (first, _) = self.directed(lp[0].0, lp[0].1);
            let (_, last) = self.directed(lp[lp.len() - 1].0, lp[lp.len() - 1].1);
            assert!(
                first == last && !lp.is_empty(),
                "merge: chained a boundary walk of {} edges that starts at {first} and ends \
                 at {last}; a face loop is closed",
                lp.len()
            );
            lps.push(lp);
        }
        if lps.is_empty() || used.len() != boundary.len() {
            return None;
        }
        // The outer loop is the one that encloses the rest, which for loops that
        // do not cross is the one of greatest area.
        let area = |lp: &Vec<(EdgeId, bool)>| {
            let mut a = 0.0f32;
            for &(e, fwd) in lp {
                let (s, t) = self.directed(e, fwd);
                let (p, q) = (self.verts[s].point, self.verts[t].point);
                a += (p.y * q.z - p.z * q.y) + (p.z * q.x - p.x * q.z) + (p.x * q.y - p.y * q.x);
            }
            a.abs()
        };
        let big = (0..lps.len()).max_by(|&i, &j| {
            area(&lps[i])
                .partial_cmp(&area(&lps[j]))
                .unwrap_or(std::cmp::Ordering::Equal)
        })?;
        lps.swap(0, big);
        Some(lps)
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
                    if f {
                        fwd[e] += 1
                    } else {
                        bwd[e] += 1
                    }
                    let (_, end) = self.directed(e, f);
                    let (ne, nf) = lp[(w + 1) % lp.len()];
                    let (nstart, _) = self.directed(ne, nf);
                    if end != nstart {
                        return Err(format!(
                            "face {fi}: loop not closed (edge {e} end {end} at {:?} != \
                             next edge {ne} start {nstart} at {:?}, {:.3e} apart)",
                            self.verts[end].point,
                            self.verts[nstart].point,
                            (self.verts[end].point - self.verts[nstart].point).length(),
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
                    "edge {e} used fwd={} bwd={} (want 1/1); {:?} from {:?} to {:?}",
                    fwd[e],
                    bwd[e],
                    self.edges[e].curve,
                    self.verts[self.edges[e].v0].point,
                    self.verts[self.edges[e].v1].point,
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
        Builder {
            loops: vec![0],
            ..Builder::default()
        }
    }

    pub fn with_capacity(
        nv: usize,
        ne: usize,
        nloops: usize,
        nloop_edges: usize,
        nfaces: usize,
    ) -> Builder {
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
        // Every vertex the solid carries, not just those on the faces being
        // rebuilt. The array is cloned whole, so each id already exists here;
        // leaving one out of the index only means a later `vertex()` at the same
        // point mints a second id for it, and the solid ends up with two
        // vertices in one weld cell. A vertex on no face at all -- what fusing
        // collinear edges leaves behind at the junctions it dissolves -- could
        // never be indexed by a walk over faces.
        for (v, vert) in solid.verts.iter().enumerate() {
            b.vert_index.entry(weld_key(vert.point)).or_insert(v);
        }
        for (fi, &wanted) in seed.iter().enumerate() {
            if !wanted {
                continue;
            }
            for lp in solid.face_loops(fi) {
                for &(e, _) in lp {
                    let ed = solid.edges[e];
                    let (lo, hi) = if ed.v0 < ed.v1 {
                        (ed.v0, ed.v1)
                    } else {
                        (ed.v1, ed.v0)
                    };
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
        let va = self.find_vertex(a)?;
        let vb = self.find_vertex(b)?;
        let (lo, hi) = if va < vb { (va, vb) } else { (vb, va) };
        self.find_edge_id(lo, hi, mid)
    }

    pub fn find_vertex(&self, p: Vec3) -> Option<VertexId> {
        assert!(p.is_finite(), "vertex at a non-finite point {p:?}");
        let k = weld_key(p);
        if let Some(&id) = self.vert_index.get(&k) {
            return Some(id);
        }
        for dz in -1..=1 {
            for dy in -1..=1 {
                for dx in -1..=1 {
                    let n = (k.0 + dx, k.1 + dy, k.2 + dz);
                    if let Some(&id) = self.vert_index.get(&n)
                        && (self.verts[id].point - p).length_squared() <= WELD_NEAR_SQ
                    {
                        return Some(id);
                    }
                }
            }
        }
        None
    }

    pub fn vertex(&mut self, p: Vec3) -> VertexId {
        crate::kernel::perf::count(crate::kernel::perf::Metric::BuilderVertex);
        let k = weld_key(p);
        if let Some(id) = self.find_vertex(p) {
            self.vert_index.insert(k, id);
            return id;
        }
        let id = self.verts.len();
        self.verts.push(Vertex { point: p });
        self.vert_index.insert(k, id);
        id
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
        let curve = Curve::Circle {
            center,
            axis,
            radius,
            ref_dir,
        };
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
        let curve = Curve::Ellipse {
            center,
            a: ea,
            b: eb,
        };
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
        if let Some(e) = self.find_edge_id(lo, hi, mid) {
            return (e, self.edges[e].v0 == a);
        }
        let id = self.edges.len();
        self.edges.push(make());
        self.edge_index.insert((lo, hi, weld_key(mid)), id);
        (id, true)
    }

    fn find_edge_id(&self, lo: VertexId, hi: VertexId, mid: Vec3) -> Option<EdgeId> {
        assert!(mid.is_finite(), "edge midpoint is not finite: {mid:?}");
        let k = weld_key(mid);
        if let Some(&e) = self.edge_index.get(&(lo, hi, k)) {
            return Some(e);
        }
        for dz in -1..=1 {
            for dy in -1..=1 {
                for dx in -1..=1 {
                    let n = (k.0 + dx, k.1 + dy, k.2 + dz);
                    if let Some(&e) = self.edge_index.get(&(lo, hi, n)) {
                        let ed = self.edges[e];
                        let stored = match ed.curve {
                            Curve::Line { .. } => {
                                (self.verts[ed.v0].point + self.verts[ed.v1].point) * 0.5
                            }
                            _ => ed.curve.point((ed.t0 + ed.t1) * 0.5),
                        };
                        if (stored - mid).length_squared() <= WELD_NEAR_SQ {
                            return Some(e);
                        }
                    }
                }
            }
        }
        None
    }

    pub fn face(&mut self, surface: Surface, sense: bool, outer: Loop, inners: Vec<Loop>) -> usize {
        let inner_slices: Vec<&[(EdgeId, bool)]> =
            inners.iter().map(|l| l.edges.as_slice()).collect();
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
