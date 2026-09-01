use crate::geom::{Curve, Surface};
use crate::math::{Vec3, weld_key};
use crate::topo::{EdgeId, Solid, VertexId};
use std::collections::HashMap;

const GEO_TOL: f64 = 1e-6;

#[derive(Debug, Clone, Default)]
pub struct AuditReport {
    pub defects: Vec<Defect>,
}

impl AuditReport {
    pub fn is_ok(&self) -> bool {
        !self.defects.iter().any(|d| d.severity == Severity::Error)
    }

    pub fn errors(&self) -> impl Iterator<Item = &Defect> {
        self.defects
            .iter()
            .filter(|d| d.severity == Severity::Error)
    }

    pub fn warnings(&self) -> impl Iterator<Item = &Defect> {
        self.defects
            .iter()
            .filter(|d| d.severity == Severity::Warning)
    }

    pub fn counts(&self) -> Vec<(Category, usize)> {
        let mut m: HashMap<Category, usize> = HashMap::new();
        for d in &self.defects {
            *m.entry(d.category).or_default() += 1;
        }
        let mut out: Vec<(Category, usize)> = m.into_iter().collect();
        out.sort_by_key(|(c, _)| format!("{c:?}"));
        out
    }
}

impl std::fmt::Display for AuditReport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if self.defects.is_empty() {
            return write!(f, "audit: clean (no defects)");
        }
        let (e, w) = (self.errors().count(), self.warnings().count());
        writeln!(f, "audit: {e} error(s), {w} warning(s)")?;
        for (c, n) in self.counts() {
            writeln!(f, "  {c:?}: {n}")?;
        }
        for d in &self.defects {
            writeln!(f, "  {d}")?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone)]
pub struct Defect {
    pub severity: Severity,
    pub category: Category,
    pub message: String,
    pub location: Option<Location>,
}

impl std::fmt::Display for Defect {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let sev = match self.severity {
            Severity::Error => "ERROR",
            Severity::Warning => "warn",
        };
        write!(f, "[{sev} {:?}] {}", self.category, self.message)?;
        if let Some(loc) = &self.location {
            write!(f, " @ {loc}")?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Severity {
    Error,
    Warning,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Category {
    Manifold,
    LoopClosure,
    VertexWeld,
    EdgeVertexGeometry,
    EdgeOnSurface,
    Degenerate,
    Orientation,
    LoopContainment,
}

#[derive(Debug, Clone)]
pub enum Location {
    Edge(EdgeId),
    Vertex(VertexId),
    Face(usize),
    LoopAt(usize, usize, usize),
    Point(Vec3),
}

impl std::fmt::Display for Location {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Location::Edge(e) => write!(f, "edge {e}"),
            Location::Vertex(v) => write!(f, "vert {v}"),
            Location::Face(i) => write!(f, "face {i}"),
            Location::LoopAt(fi, li, pi) => write!(f, "face {fi} loop {li} [{pi}]"),
            Location::Point(p) => write!(f, "{p:?}"),
        }
    }
}

pub fn audit(solid: &Solid) -> AuditReport {
    let mut defects = Vec::new();
    audit_manifold(solid, &mut defects);
    audit_loop_closure(solid, &mut defects);
    audit_vertex_weld(solid, &mut defects);
    audit_edge_vertex_geometry(solid, &mut defects);
    audit_edge_on_surface(solid, &mut defects);
    audit_degenerate(solid, &mut defects);
    audit_orientation(solid, &mut defects);
    audit_loop_containment(solid, &mut defects);
    AuditReport { defects }
}

#[derive(Debug, Clone)]
pub struct TessLeak {
    pub a: Vec3,
    pub b: Vec3,
    pub imbalance: i32,
    pub count: usize,
    pub faces: Vec<usize>,
}

pub fn tessellation_leaks(tess: &crate::tess::Tessellation) -> Vec<TessLeak> {
    use std::collections::HashMap;
    let tris: Vec<[Vec3; 3]> = tess.tris.iter().map(|t| t.pos).collect();
    let key = |p: Vec3| {
        (
            (p.x * 1e3).round() as i64,
            (p.y * 1e3).round() as i64,
            (p.z * 1e3).round() as i64,
        )
    };
    let mut vid_of: HashMap<(i64, i64, i64), usize> = HashMap::new();
    let mut verts: Vec<Vec3> = Vec::new();
    let mut tris_idx: Vec<[usize; 3]> = Vec::with_capacity(tris.len());
    for t in &tris {
        let mut idx = [0usize; 3];
        for k in 0..3 {
            let k_ = key(t[k]);
            let v = *vid_of.entry(k_).or_insert_with(|| {
                let id = verts.len();
                verts.push(t[k]);
                id
            });
            idx[k] = v;
        }
        tris_idx.push(idx);
    }
    let mut undirected: HashMap<(usize, usize), usize> = HashMap::new();
    let mut directed: HashMap<(usize, usize), usize> = HashMap::new();
    let mut faces_of: HashMap<(usize, usize), Vec<usize>> = HashMap::new();
    for (ti, t) in tris_idx.iter().enumerate() {
        let fi = tess.face_of_tri.get(ti).copied();
        for k in 0..3 {
            let a = t[k];
            let b = t[(k + 1) % 3];
            *directed.entry((a, b)).or_default() += 1;
            let key = if a < b { (a, b) } else { (b, a) };
            *undirected.entry(key).or_default() += 1;
            if let Some(f) = fi {
                let v = faces_of.entry(key).or_default();
                if !v.contains(&f) {
                    v.push(f);
                }
            }
        }
    }
    let mut leaks = Vec::new();
    for ((a, b), count) in &undirected {
        let fwd = directed.get(&(*a, *b)).copied().unwrap_or(0);
        let bwd = directed.get(&(*b, *a)).copied().unwrap_or(0);
        let balanced = fwd == 1 && bwd == 1;
        if !balanced {
            let imbalance: i32 = fwd as i32 - bwd as i32;
            leaks.push(TessLeak {
                a: verts[*a],
                b: verts[*b],
                imbalance,
                count: *count,
                faces: faces_of.get(&(*a, *b)).cloned().unwrap_or_default(),
            });
        }
    }
    leaks.sort_by(|l, r| {
        l.a.z
            .partial_cmp(&r.a.z)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then(
                l.a.x
                    .partial_cmp(&r.a.x)
                    .unwrap_or(std::cmp::Ordering::Equal),
            )
            .then(
                l.a.y
                    .partial_cmp(&r.a.y)
                    .unwrap_or(std::cmp::Ordering::Equal),
            )
    });
    leaks
}

/// Where a face's boundary crosses itself, as the two offending runs in world
/// millimetres, and `None` when it does not.
///
/// The two runs are what a caller has to see: "face N's boundary crosses
/// itself" says a blend was refused and nothing about which part of it doubled
/// back, and a refusal is read far from where the loop was authored.
pub struct SelfCrossing {
    pub a: (Vec3, Vec3),
    pub b: (Vec3, Vec3),
}

impl std::fmt::Display for SelfCrossing {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "the run {:?} -> {:?} crosses the run {:?} -> {:?}",
            self.a.0, self.a.1, self.b.0, self.b.1
        )
    }
}

pub fn face_loops_self_intersect(solid: &Solid, fid: usize) -> bool {
    face_loop_self_crossing(solid, fid).is_some()
}

pub fn face_loop_self_crossing(solid: &Solid, fid: usize) -> Option<SelfCrossing> {
    use crate::math::Vec2;
    const PER_EDGE: usize = 4;
    let face = &solid.faces[fid];
    if matches!(face.surface, Surface::Sphere { .. }) {
        return None;
    }
    let prep = face.surface.prepare();
    let planar = matches!(face.surface, Surface::Plane { .. });
    let mut pts: Vec<Vec2> = Vec::new();
    let mut spans: Vec<(usize, usize)> = Vec::new();
    for lp in solid.face_loops(fid) {
        let start = pts.len();
        for &(e, fwd) in lp {
            let ed = solid.edges[e];
            let n = match ed.curve {
                Curve::Line { .. } => 1,
                _ => PER_EDGE,
            };
            for k in 0..n {
                let f = k as f64 / n as f64;
                let t = if fwd {
                    ed.t0 + (ed.t1 - ed.t0) * f
                } else {
                    ed.t1 + (ed.t0 - ed.t1) * f
                };
                let (u, v) = prep.project(ed.curve.point(t));
                pts.push(Vec2::new(u, v));
            }
        }
        spans.push((start, pts.len()));
    }
    if !planar {
        for &(s, e) in &spans {
            let mut prev = pts[s].x;
            for p in pts.iter_mut().take(e).skip(s + 1) {
                while p.x - prev > std::f64::consts::PI {
                    p.x -= std::f64::consts::TAU;
                }
                while p.x - prev < -std::f64::consts::PI {
                    p.x += std::f64::consts::TAU;
                }
                prev = p.x;
            }
        }
    }
    if matches!(face.surface, Surface::Torus { .. }) {
        for &(s, e) in &spans {
            let mut prev = pts[s].y;
            for p in pts.iter_mut().take(e).skip(s + 1) {
                while p.y - prev > std::f64::consts::PI {
                    p.y -= std::f64::consts::TAU;
                }
                while p.y - prev < -std::f64::consts::PI {
                    p.y += std::f64::consts::TAU;
                }
                prev = p.y;
            }
        }
    }
    let mut segs: Vec<(Vec2, Vec2)> = Vec::new();
    for &(s, e) in &spans {
        if e - s < 3 {
            continue;
        }
        for i in s..e {
            let j = if i + 1 == e { s } else { i + 1 };
            segs.push((pts[i], pts[j]));
        }
    }
    let side = |a: Vec2, b: Vec2, c: Vec2| (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    for i in 0..segs.len() {
        for j in (i + 1)..segs.len() {
            let (p, q) = segs[i];
            let (r, t) = segs[j];
            if p == r || p == t || q == r || q == t {
                continue;
            }
            let (d1, d2) = (side(p, q, r), side(p, q, t));
            let (d3, d4) = (side(r, t, p), side(r, t, q));
            if d1 * d2 < 0.0 && d3 * d4 < 0.0 {
                let at = |uv: Vec2| face.surface.point((uv.x, uv.y));
                return Some(SelfCrossing {
                    a: (at(p), at(q)),
                    b: (at(r), at(t)),
                });
            }
        }
    }
    None
}

fn audit_manifold(solid: &Solid, defects: &mut Vec<Defect>) {
    let edge_faces = solid.edge_faces();
    let mut fwd = vec![0u32; solid.edges.len()];
    let mut bwd = vec![0u32; solid.edges.len()];
    for fi in 0..solid.faces.len() {
        for lp in solid.face_loops(fi) {
            for &(e, f) in lp {
                if f {
                    fwd[e] += 1;
                } else {
                    bwd[e] += 1;
                }
            }
        }
    }
    for e in 0..solid.edges.len() {
        if fwd[e] != 1 || bwd[e] != 1 {
            defects.push(Defect {
                severity: Severity::Error,
                category: Category::Manifold,
                message: format!(
                    "edge used fwd={} bwd={} (want 1/1); faces {:?}",
                    fwd[e], bwd[e], &edge_faces[e]
                ),
                location: Some(Location::Edge(e)),
            });
        }
    }
}

fn audit_loop_closure(solid: &Solid, defects: &mut Vec<Defect>) {
    for fi in 0..solid.faces.len() {
        for (li, lp) in solid.face_loops(fi).enumerate() {
            let n = lp.len();
            if n == 0 {
                defects.push(Defect {
                    severity: Severity::Error,
                    category: Category::LoopClosure,
                    message: "empty loop".into(),
                    location: Some(Location::LoopAt(fi, li, 0)),
                });
                continue;
            }
            for w in 0..n {
                let (e, f) = lp[w];
                let (_, end) = solid.directed(e, f);
                let (ne, nf) = lp[(w + 1) % n];
                let (nstart, _) = solid.directed(ne, nf);
                if end != nstart {
                    defects.push(Defect {
                        severity: Severity::Error,
                        category: Category::LoopClosure,
                        message: format!(
                            "edge {e} ends at vert {end}; next edge {ne} starts at vert {nstart}"
                        ),
                        location: Some(Location::LoopAt(fi, li, w)),
                    });
                }
            }
        }
    }
}

fn audit_vertex_weld(solid: &Solid, defects: &mut Vec<Defect>) {
    let mut by_key: HashMap<(i64, i64, i64), Vec<VertexId>> = HashMap::new();
    for (vi, v) in solid.verts.iter().enumerate() {
        by_key.entry(weld_key(v.point)).or_default().push(vi);
    }
    for (_, ids) in by_key {
        if ids.len() > 1 {
            let p = solid.verts[ids[0]].point;
            defects.push(Defect {
                severity: Severity::Error,
                category: Category::VertexWeld,
                message: format!(
                    "{} vertices share one weld cell but have distinct ids: {:?}",
                    ids.len(),
                    ids
                ),
                location: Some(Location::Point(p)),
            });
        }
    }
    let n = solid.verts.len();
    for i in 0..n {
        for j in (i + 1)..n {
            let d = (solid.verts[i].point - solid.verts[j].point).length();
            if d < GEO_TOL && weld_key(solid.verts[i].point) != weld_key(solid.verts[j].point) {
                defects.push(Defect {
                    severity: Severity::Warning,
                    category: Category::VertexWeld,
                    message: format!(
                        "verts {i}/{j} are {d:.2e} mm apart but not welded (sub-weld collision)"
                    ),
                    location: Some(Location::Point(solid.verts[i].point)),
                });
            }
        }
    }
}

fn audit_edge_vertex_geometry(solid: &Solid, defects: &mut Vec<Defect>) {
    for (ei, edge) in solid.edges.iter().enumerate() {
        let p0 = edge.curve.point(edge.t0);
        let p1 = edge.curve.point(edge.t1);
        let v0 = solid.verts[edge.v0].point;
        let v1 = solid.verts[edge.v1].point;
        let d0 = (p0 - v0).length();
        let d1 = (p1 - v1).length();
        if d0 > GEO_TOL {
            defects.push(Defect {
                severity: Severity::Error,
                category: Category::EdgeVertexGeometry,
                message: format!(
                    "curve at t0={:.4} is {d0:.2e} mm from v{} ({:?} vs {:?})",
                    edge.t0, edge.v0, p0, v0
                ),
                location: Some(Location::Edge(ei)),
            });
        }
        if d1 > GEO_TOL {
            defects.push(Defect {
                severity: Severity::Error,
                category: Category::EdgeVertexGeometry,
                message: format!(
                    "curve at t1={:.4} is {d1:.2e} mm from v{} ({:?} vs {:?})",
                    edge.t1, edge.v1, p1, v1
                ),
                location: Some(Location::Edge(ei)),
            });
        }
    }
}

fn audit_edge_on_surface(solid: &Solid, defects: &mut Vec<Defect>) {
    let edge_faces = solid.edge_faces();
    const SAMPLES: usize = 6;
    for (ei, edge) in solid.edges.iter().enumerate() {
        let mut mid_p = Vec3::ZERO;
        for &fi in &edge_faces[ei] {
            let surface = solid.faces[fi].surface;
            let mut worst = 0.0f64;
            let mut worst_p = Vec3::ZERO;
            for k in 0..=SAMPLES {
                let t = edge.t0 + (edge.t1 - edge.t0) * (k as f64 / SAMPLES as f64);
                let p = edge.curve.point(t);
                if k == SAMPLES / 2 {
                    mid_p = p;
                }
                let d = dist_to_surface(p, surface);
                if d > worst {
                    worst = d;
                    worst_p = p;
                }
            }
            if worst > GEO_TOL {
                defects.push(Defect {
                    severity: Severity::Error,
                    category: Category::EdgeOnSurface,
                    message: format!(
                        "edge curve deviates {worst:.2e} mm from face {fi}'s surface {:?} \
                         (sampled at {SAMPLES} points along t∈[{:.4},{:.4}])",
                        surface, edge.t0, edge.t1
                    ),
                    location: Some(Location::Point(worst_p)),
                });
            }
            let _ = mid_p;
        }
    }
}

fn audit_degenerate(solid: &Solid, defects: &mut Vec<Defect>) {
    for (ei, edge) in solid.edges.iter().enumerate() {
        let p0 = solid.verts[edge.v0].point;
        let p1 = solid.verts[edge.v1].point;
        let span = (p1 - p0).length();
        if span < 1e-6 && edge.v0 != edge.v1 {
            defects.push(Defect {
                severity: Severity::Error,
                category: Category::Degenerate,
                message: format!("zero-length edge (span {span:.2e} mm)"),
                location: Some(Location::Edge(ei)),
            });
        }
        if let Curve::Circle { radius, .. } = edge.curve {
            if radius < 1e-6 {
                defects.push(Defect {
                    severity: Severity::Error,
                    category: Category::Degenerate,
                    message: format!("zero-radius circle ({radius:.2e} mm)"),
                    location: Some(Location::Edge(ei)),
                });
            }
        }
        for (name, p) in [("v0", p0), ("v1", p1)] {
            if p.x.is_nan() || p.y.is_nan() || p.z.is_nan() {
                defects.push(Defect {
                    severity: Severity::Error,
                    category: Category::Degenerate,
                    message: format!("NaN in {name} position"),
                    location: Some(Location::Edge(ei)),
                });
            }
        }
    }
}

fn audit_orientation(solid: &Solid, defects: &mut Vec<Defect>) {
    let edge_faces = solid.edge_faces();
    for (ei, edge) in solid.edges.iter().enumerate() {
        let faces = &edge_faces[ei];
        if faces.len() != 2 {
            continue;
        }
        let (fa, fb) = (faces[0], faces[1]);
        let (sa, sb) = (solid.faces[fa].surface, solid.faces[fb].surface);
        if matches!(sa, Surface::Plane { .. }) || matches!(sb, Surface::Plane { .. }) {
            continue;
        }
        let t = (edge.t0 + edge.t1) * 0.5;
        let p = edge.curve.point(t);
        let na = face_outward_normal(solid, fa, p);
        let nb = face_outward_normal(solid, fb, p);
        let dot = na.dot(nb);
        if dot > 0.9999 {
            defects.push(Defect {
                severity: Severity::Warning,
                category: Category::Orientation,
                message: format!(
                    "non-planar faces {fa}/{fb} share edge {ei} with near-identical normals (dot {dot:.6})"
                ),
                location: Some(Location::Edge(ei)),
            });
        }
    }
}

fn audit_loop_containment(solid: &Solid, defects: &mut Vec<Defect>) {
    use std::collections::HashMap;

    let mut edge_pts: HashMap<EdgeId, Vec<Vec3>> = HashMap::new();
    for (id, e) in solid.edges.iter().enumerate() {
        let n = e.seg_count(6).max(2);
        edge_pts.insert(id, e.sample(true, n));
    }

    'faces: for (fi, face) in solid.faces.iter().enumerate() {
        let loops: Vec<_> = solid.face_loops(fi).collect();
        if loops.len() < 2 {
            continue;
        }
        let to_uv = |p: Vec3| face.surface.project(p);
        let mut outer_uv: Vec<[f64; 2]> = Vec::new();
        for &(e, fwd) in loops[0] {
            let s = &edge_pts[&e];
            let chain: Vec<Vec3> = if fwd {
                s.iter().copied().collect()
            } else {
                s.iter().rev().copied().collect()
            };
            for p in &chain[..chain.len() - 1] {
                outer_uv.push({
                    let uv = to_uv(*p);
                    [uv.0, uv.1]
                });
            }
        }
        if outer_uv.len() < 3 {
            continue;
        }
        unwrap_angular(&mut outer_uv, face.surface);

        for (li, lp) in loops[1..].iter().enumerate() {
            let mut worst_out: f64 = 0.0;
            let mut worst_uv = [0.0, 0.0];
            let mut worst_p = Vec3::ZERO;
            let mut any_out = false;
            for &(e, fwd) in lp.iter() {
                let s = &edge_pts[&e];
                let chain: Vec<Vec3> = if fwd {
                    s.iter().copied().collect()
                } else {
                    s.iter().rev().copied().collect()
                };
                for p in &chain[..chain.len() - 1] {
                    let uv = to_uv(*p);
                    let mut uv_arr = [uv.0, uv.1];
                    if !matches!(face.surface, Surface::Plane { .. }) {
                        snap_to_unwrapped(&mut uv_arr, &outer_uv);
                    }
                    let probe = (uv_arr[0], uv_arr[1]);
                    let outside = signed_outside_distance(probe, &outer_uv);
                    if outside > worst_out {
                        worst_out = outside;
                        worst_uv = uv_arr;
                        worst_p = *p;
                        any_out = outside > GEO_TOL;
                    }
                }
            }
            if any_out {
                defects.push(Defect {
                    severity: Severity::Error,
                    category: Category::LoopContainment,
                    message: format!(
                        "hole loop {} of face {fi} pokes {worst_out:.4} mm outside the outer \
                         boundary (at uv ({:.4},{:.4}), world ({:.4},{:.4},{:.4})); the trimmed \
                         surface is a self-intersecting polygon and will tessellate with leaks",
                        li + 1,
                        worst_uv[0],
                        worst_uv[1],
                        worst_p.x,
                        worst_p.y,
                        worst_p.z
                    ),
                    location: Some(Location::LoopAt(fi, li + 1, 0)),
                });
                continue 'faces;
            }
        }

        if !matches!(face.surface, Surface::Plane { .. }) {
            continue;
        }
        let area = |uv: &[[f64; 2]]| -> f64 {
            let mut a = 0.0;
            for i in 0..uv.len() {
                let j = (i + 1) % uv.len();
                a += uv[i][0] * uv[j][1] - uv[j][0] * uv[i][1];
            }
            a * 0.5
        };
        let mut net = area(&outer_uv).abs();
        for lp in loops[1..].iter() {
            let mut hole: Vec<[f64; 2]> = Vec::new();
            for &(e, fwd) in lp.iter() {
                let sp = &edge_pts[&e];
                let chain: Vec<Vec3> = if fwd {
                    sp.to_vec()
                } else {
                    sp.iter().rev().copied().collect()
                };
                for p in &chain[..chain.len() - 1] {
                    let uv = to_uv(*p);
                    hole.push([uv.0, uv.1]);
                }
            }
            if hole.len() >= 3 {
                net -= area(&hole).abs();
            }
        }
        if net <= 0.0 {
            defects.push(Defect {
                severity: Severity::Error,
                category: Category::LoopContainment,
                message: format!(
                    "face {fi}'s holes cover {:.4} mm² more than its outer boundary encloses, so                      they overlap each other; the face has no interior left to triangulate",
                    -net
                ),
                location: Some(Location::Face(fi)),
            });
        }
    }
}

fn unwrap_angular(uv: &mut Vec<[f64; 2]>, surface: crate::geom::Surface) {
    use crate::geom::Surface;
    if matches!(surface, Surface::Plane { .. }) {
        return;
    }
    for k in 1..uv.len() {
        while uv[k][0] - uv[k - 1][0] > std::f64::consts::PI {
            uv[k][0] -= 2.0 * std::f64::consts::PI;
        }
        while uv[k][0] - uv[k - 1][0] < -std::f64::consts::PI {
            uv[k][0] += 2.0 * std::f64::consts::PI;
        }
    }
}

fn snap_to_unwrapped(uv: &mut [f64; 2], outer: &[[f64; 2]]) {
    let nearest = outer.iter().min_by_key(|o| {
        let dv = (uv[1] - o[1]).abs();
        dv.to_bits()
    });
    if let Some(o) = nearest {
        let du = uv[0] - o[0];
        let shift = (du / (2.0 * std::f64::consts::PI)).round() * 2.0 * std::f64::consts::PI;
        uv[0] -= shift;
    }
}

fn signed_outside_distance(uv: (f64, f64), poly: &[[f64; 2]]) -> f64 {
    let (px, py) = (uv.0, uv.1);
    let n = poly.len();
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let (xi, yi) = (poly[i][0], poly[i][1]);
        let (xj, yj) = (poly[j][0], poly[j][1]);
        if (yi > py) != (yj > py) {
            let x_cross = xi + (py - yi) / (yj - yi) * (xj - xi);
            if px < x_cross {
                inside = !inside;
            }
        }
        j = i;
    }
    let mut mind2 = f64::INFINITY;
    for w in 0..n {
        let a = poly[w];
        let b = poly[(w + 1) % n];
        let d = dist2_point_to_seg(px, py, a[0], a[1], b[0], b[1]);
        if d < mind2 {
            mind2 = d;
        }
    }
    let dist = mind2.sqrt();
    if inside { -dist } else { dist }
}

fn dist2_point_to_seg(px: f64, py: f64, ax: f64, ay: f64, bx: f64, by: f64) -> f64 {
    let dx = bx - ax;
    let dy = by - ay;
    let l2 = dx * dx + dy * dy;
    let t = if l2 > 0.0 {
        (((px - ax) * dx + (py - ay) * dy) / l2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let cx = ax + t * dx;
    let cy = ay + t * dy;
    (px - cx) * (px - cx) + (py - cy) * (py - cy)
}

fn face_outward_normal(solid: &Solid, fi: usize, p: Vec3) -> Vec3 {
    let f = &solid.faces[fi];
    let n = f.surface.normal(f.surface.project(p));
    if f.sense { n } else { -n }
}

fn dist_to_surface(p: Vec3, s: Surface) -> f64 {
    match s {
        Surface::Plane { origin, normal, .. } => (p - origin).dot(*normal).abs(),
        Surface::Cylinder {
            base, axis, radius, ..
        } => {
            let rel = p - base;
            let radial = rel - *axis * rel.dot(*axis);
            (radial.length() - radius).abs()
        }
        Surface::Cone { half_angle, .. } => {
            let open = s.cone_open().vec();
            let rel = p - s.cone_apex();
            let along = rel.dot(open);
            let perp = (rel - open * along).length();
            let r = rel.length();
            if r < 1e-9 {
                0.0
            } else {
                let theta = perp.atan2(along);
                r * (theta - half_angle).sin().abs()
            }
        }
        Surface::Sphere { center, radius, .. } => ((p - center).length() - radius).abs(),
        Surface::Torus { .. } => s.signed_distance(p).abs(),
    }
    .abs()
}
