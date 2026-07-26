//! Heavy soundness checker for the B-rep model.
//!
//! [`audit`] goes well beyond [`Solid::validate`](crate::kernel::topo::Solid::validate):
//! it checks *geometric* consistency, not just topology. Where `validate` only
//! confirms that every edge is referenced exactly twice and that loops chain
//! vertex-to-vertex by id, `audit` also confirms that
//!
//! - each edge's curve, sampled at its parameter endpoints, actually lands on
//!   the vertices it claims to connect ([`Category::EdgeVertexGeometry`]);
//! - each edge's curve, sampled along its interior, actually lies on the
//!   surface of every face that borders it ([`Category::EdgeOnSurface`]) —
//!   this is the one that catches a mis-authored blend whose tangent circle
//!   is at the wrong radius to weld with the neighbour face;
//! - no two distinct vertices sit at the same position, and none sit closer
//!   than the weld resolution without having been merged
//!   ([`Category::VertexWeld`]);
//! - no edge is degenerate (zero length, zero-radius arc, NaN)
//!   ([`Category::Degenerate`]);
//! - adjacent faces across each edge are consistently oriented
//!   ([`Category::Orientation`]).
//!
//! It is O(n²) in vertex count and projects every edge onto every surface it
//! borders — slow, intended for tests and diagnostics, not the build path.
//!
//! The printability gate (mesh watertightness) is a *consequence* of all of
//! the above: if the B-rep passes `audit` and the tessellator samples each
//! edge exactly once, the mesh is watertight by construction. So when the
//! mesh leaks, `audit` is the tool that pins the failure to a specific edge,
//! vertex, or face rather than to a vague "something is wrong with the
//! fillet".

use crate::kernel::geom::{Curve, Surface};
use crate::kernel::math::{weld_key, Vec3};
use crate::kernel::topo::{EdgeId, Solid, VertexId};
use std::collections::HashMap;

/// Tolerance for "this point is on this surface / vertex". 1 µm — tighter
/// than the weld resolution (0.1 µm) so that a sub-weld mismatch still shows
/// up as a defect rather than being rounded away.
const GEO_TOL: f32 = 1e-3;

/// Result of auditing a [`Solid`].
#[derive(Debug, Clone, Default)]
pub struct AuditReport {
    /// Every defect found, in the order they were detected.
    pub defects: Vec<Defect>,
}

impl AuditReport {
    /// `true` iff there are no [`Severity::Error`] defects.
    pub fn is_ok(&self) -> bool {
        !self.defects.iter().any(|d| d.severity == Severity::Error)
    }

    pub fn errors(&self) -> impl Iterator<Item = &Defect> {
        self.defects.iter().filter(|d| d.severity == Severity::Error)
    }

    pub fn warnings(&self) -> impl Iterator<Item = &Defect> {
        self.defects.iter().filter(|d| d.severity == Severity::Warning)
    }

    /// Count by category, sorted by category for stable display.
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

/// A single defect found by [`audit`].
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

/// What kind of invariant a defect violates. Stable for grouping/reporting.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Category {
    /// An edge is not used exactly once in each direction across all face
    /// loops (the manifold invariant). Subsumes the `validate()` check.
    Manifold,
    /// Two consecutive edges in a loop do not share their endpoint vertex.
    LoopClosure,
    /// Two distinct vertex ids sit at the same position, or two vertices are
    /// closer than the weld resolution yet were not merged — both indicate
    /// the tessellator will emit near-but-not-coincident boundary points.
    VertexWeld,
    /// `edge.curve.point(t0)` does not match `verts[v0].point` (or t1/v1).
    /// A loop whose edges don't land on their vertices cannot weld in the
    /// mesh even when the topology says the chain is closed.
    EdgeVertexGeometry,
    /// An edge shared by a face does not actually lie on that face's surface.
    /// This is the blend bug: the tangent circle is at the wrong radius, so
    /// the blend face and the neighbour face disagree about where the shared
    /// boundary lives.
    EdgeOnSurface,
    /// Zero-length edge, zero-radius arc, or NaN in a position/parameter.
    Degenerate,
    /// Two faces share an edge but their surface normals there point the same
    /// way (a zero-thickness membrane or a duplicated face).
    Orientation,
    /// A hole loop in a face crosses or lies outside the face's outer loop.
    /// Projected into the face's 2-D parameter space, every hole must be
    /// strictly inside the outer boundary; otherwise the trimmed surface is
    /// not a valid polygon and the tessellator (earcut) will silently drop
    /// vertices, producing a leaking mesh. The B-rep can be topologically
    /// perfect (every edge paired, every loop closed, every curve on-surface)
    /// and still fail this — a fillet torus whose tangent circle extends past
    /// the trimmed floor edge is the canonical case.
    LoopContainment,
}

/// Where a defect is anchored, for navigation back into the source.
#[derive(Debug, Clone)]
pub enum Location {
    Edge(EdgeId),
    Vertex(VertexId),
    Face(usize),
    /// `(face, loop_index_in_face, position_in_loop)`. `Loop 0` is the outer.
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

/// Run every check and return the aggregated report.
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

/// A mesh-edge that doesn't pair with its reverse — the tessellation is not
/// watertight. Carries enough to attribute it back to the B-rep face(s) that
/// emitted it, so a leak can be localised to a specific blend / cap / wall.
#[derive(Debug, Clone)]
pub struct TessLeak {
    /// The two triangle-vertex positions, in millimetres.
    pub a: Vec3,
    pub b: Vec3,
    /// `+1` if only the forward direction is present, `-1` if only reverse.
    pub imbalance: i32,
    /// How many times the edge appears in the mesh (should be 2 for closed).
    pub count: usize,
    /// The face index of every triangle that owns an endpoint of this leak
    /// edge — the union of the two faces that should share it but don't.
    pub faces: Vec<usize>,
}

/// Find every non-watertight edge in the tessellated mesh and attribute each
/// to the B-rep face(s) whose triangles touch it.
///
/// This is the bridge between the (geometrically exact) B-rep and the
/// (sampled) mesh: if [`audit`] is clean but this returns leaks, the defect
/// is in the tessellator's sampling/winding, not in the model. The faces list
/// names the suspects.
pub fn tessellation_leaks(tess: &crate::kernel::tess::Tessellation) -> Vec<TessLeak> {
    use std::collections::HashMap;
    let tris: Vec<[Vec3; 3]> = tess.tris.iter().map(|t| t.pos).collect();
    // Weld by rounded position (1 µm) so that two faces emitting the "same"
    // boundary point at float-epsilon distance still pair. The mesh's own
    // `to_mesh` welds at 0.1 µm; we are slightly more generous here to focus
    // on geometric leaks rather than float noise.
    let key = |p: Vec3| (
        (p.x * 1e3).round() as i64,
        (p.y * 1e3).round() as i64,
        (p.z * 1e3).round() as i64,
    );
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
    // Count directed and undirected; also collect face indices touching each
    // undirected edge.
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
    // Sort by axis for stable diffing across runs.
    leaks.sort_by(|l, r| {
        l.a.z.partial_cmp(&r.a.z)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then(l.a.x.partial_cmp(&r.a.x).unwrap_or(std::cmp::Ordering::Equal))
            .then(l.a.y.partial_cmp(&r.a.y).unwrap_or(std::cmp::Ordering::Equal))
    });
    leaks
}

// ── checks ───────────────────────────────────────────────────────────────────

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
    // Two distinct ids at the same welded key — the Builder dedupes by this
    // key, so a collision here means two points that should have been one
    // vertex were interned separately (a position was computed two different
    // ways upstream and the difference landed inside one weld cell).
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
    // Below-weld near-collisions: closer than 1 µm but with different keys.
    // The tessellator samples each edge independently; two this-close verts
    // produce two this-close mesh vertices and a sliver.
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
            let mut worst = 0.0f32;
            let mut worst_p = Vec3::ZERO;
            for k in 0..=SAMPLES {
                let t = edge.t0 + (edge.t1 - edge.t0) * (k as f32 / SAMPLES as f32);
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
    // Adjacent faces across an edge with surface normals that agree to ~1.0
    // at the shared edge: a zero-thickness membrane or a duplicated face.
    // Restricted to *non-planar* pairs — a planar continuation (peg-top arc
    // meeting wall bottom on the same plane) legitimately has matching
    // normals and is the common case, so filtering those out keeps the check
    // quiet on a normal bin while still flagging two cylindrical faces
    // wrapped around each other.
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

/// Each hole loop of each face must lie strictly inside the face's outer loop
/// in the surface's 2-D parameter space. A hole that pokes through the outer
/// boundary makes the trimmed surface a self-intersecting polygon; the ear-cut
/// tessellator then silently drops the vertices it can't place, and the mesh
/// leaks. The canonical trigger is a blend (torus) whose tangent circle is
/// trimmed into a floor face whose outer boundary has been pulled in by a
/// neighbouring blend — both blends are individually correct, but the floor
/// face's loops now cross.
///
/// Planar faces are checked exactly in `(u, v)`; radial surfaces (cylinder,
/// cone, torus, sphere) are checked by unrolling the angular parameter and
/// ray-casting in `(u, v)` with the same wrap-awareness the tessellator uses.
/// A point sitting within `GEO_TOL` of the outer boundary is considered on it
/// (legitimate tangent contact, not a violation).
fn audit_loop_containment(solid: &Solid, defects: &mut Vec<Defect>) {
    use std::collections::HashMap;

    // Sample each edge once at modest resolution — containment is a global
    // property, not a local one, so a few samples per edge suffice.
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
        // Project every loop into 2-D, unwrapping the angular parameter on
        // radial surfaces so a branch cut at `u = 0` can't fake a crossing.
        let to_uv = |p: Vec3| face.surface.project(p);
        let mut outer_uv: Vec<[f32; 2]> = Vec::new();
        for &(e, fwd) in loops[0] {
            let s = &edge_pts[&e];
            let chain: Vec<Vec3> = if fwd { s.iter().copied().collect() }
                                  else { s.iter().rev().copied().collect() };
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
            let mut worst_out: f32 = 0.0;
            let mut worst_uv = [0.0, 0.0];
            let mut worst_p = Vec3::ZERO;
            let mut any_out = false;
            for &(e, fwd) in lp.iter() {
                let s = &edge_pts[&e];
                let chain: Vec<Vec3> = if fwd { s.iter().copied().collect() }
                                      else { s.iter().rev().copied().collect() };
                for p in &chain[..chain.len() - 1] {
                    let uv = to_uv(*p);
                    // On radial surfaces the hole point's angular parameter
                    // may sit on a different 2π branch than the outer polygon;
                    // snap it onto the outer's branch before ray-casting.
                    // Planar faces have no branch cut — skip the snap entirely,
                    // otherwise the multiple-vertex sweep corrupts the u value.
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
                        li + 1, worst_uv[0], worst_uv[1], worst_p.x, worst_p.y, worst_p.z
                    ),
                    location: Some(Location::LoopAt(fi, li + 1, 0)),
                });
                continue 'faces;
            }
        }
    }
}

// ── helpers ──────────────────────────────────────────────────────────────────

/// In-place unwrap of a possibly-angular first coordinate on radial surfaces.
/// Mirrors the tessellator's own branch-cut handling so the audit's notion of
/// "inside" matches what the triangulator actually sees.
fn unwrap_angular(uv: &mut Vec<[f32; 2]>, surface: crate::kernel::geom::Surface) {
    use crate::kernel::geom::Surface;
    if matches!(surface, Surface::Plane { .. }) {
        return;
    }
    for k in 1..uv.len() {
        while uv[k][0] - uv[k - 1][0] > std::f32::consts::PI {
            uv[k][0] -= 2.0 * std::f32::consts::PI;
        }
        while uv[k][0] - uv[k - 1][0] < -std::f32::consts::PI {
            uv[k][0] += 2.0 * std::f32::consts::PI;
        }
    }
}

/// Move a single hole point's `u` onto the same 2π branch as the nearest
/// outer vertex (by `v` distance). Only meaningful on radial surfaces.
fn snap_to_unwrapped(uv: &mut [f32; 2], outer: &[[f32; 2]]) {
    // Find the single nearest outer vertex by v-distance, then apply ONE shift.
    let nearest = outer.iter().min_by_key(|o| {
        let dv = (uv[1] - o[1]).abs();
        dv.to_bits()
    });
    if let Some(o) = nearest {
        let du = uv[0] - o[0];
        let shift = (du / (2.0 * std::f32::consts::PI)).round() * 2.0 * std::f32::consts::PI;
        uv[0] -= shift;
    }
}

/// Positive outside the polygon, negative inside, by this many mm. Uses the
/// nearest-edge distance once the ray-cast establishes which side we are on.
fn signed_outside_distance(uv: (f32, f32), poly: &[[f32; 2]]) -> f32 {
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
    let mut mind2 = f32::INFINITY;
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

fn dist2_point_to_seg(px: f32, py: f32, ax: f32, ay: f32, bx: f32, by: f32) -> f32 {
    let dx = bx - ax;
    let dy = by - ay;
    let l2 = dx * dx + dy * dy;
    let t = if l2 > 0.0 { (((px - ax) * dx + (py - ay) * dy) / l2).clamp(0.0, 1.0) } else { 0.0 };
    let cx = ax + t * dx;
    let cy = ay + t * dy;
    (px - cx) * (px - cx) + (py - cy) * (py - cy)
}

fn face_outward_normal(solid: &Solid, fi: usize, p: Vec3) -> Vec3 {
    let f = &solid.faces[fi];
    let n = f.surface.normal(f.surface.project(p));
    if f.sense {
        n
    } else {
        -n
    }
}

/// Signed distance from `p` to `surface` (absolute for radial surfaces).
/// Matches the geometry kernel's own notion of "on surface" so the auditor
/// and the modeller agree about what counts as a violation.
fn dist_to_surface(p: Vec3, s: Surface) -> f32 {
    match s {
        Surface::Plane { origin, normal, .. } => (p - origin).dot(normal).abs(),
        Surface::Cylinder { base, axis, radius, .. } => {
            let rel = p - base;
            let radial = rel - axis * rel.dot(axis);
            (radial.length() - radius).abs()
        }
        Surface::Cone { apex, axis, half_angle, .. } => {
            // Angle of `p` from the cone apex, measured off the axis. The
            // perpendicular distance to the nappe is then `|p - apex|·sin(Δ)`.
            let rel = p - apex;
            let along = rel.dot(axis);
            let perp = (rel - axis * along).length();
            let r = rel.length();
            if r < 1e-9 {
                0.0
            } else {
                let theta = perp.atan2(along);
                r * (theta - half_angle).sin().abs()
            }
        }
        Surface::Sphere { center, radius, .. } => (p - center).length() - radius,
        Surface::Torus { center, axis, major_r, minor_r, .. } => {
            // Distance from a point to the generating circle of the torus,
            // less the tube radius: the exact signed distance to the surface.
            let rel = p - center;
            let axial = rel.dot(axis);
            let radial = (rel - axis * axial).length();
            (((radial - major_r).powi(2) + axial * axial).sqrt() - minor_r).abs()
        }
    }
    .abs()
}
