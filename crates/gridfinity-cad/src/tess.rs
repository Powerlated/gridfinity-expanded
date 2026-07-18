//! Tessellation: trimmed analytic faces → triangles, for preview and STL.
//!
//! Watertight by construction: each edge is sampled exactly once (as a function
//! of the edge, not the face), so the two faces sharing it emit identical
//! boundary points. Curved faces get their curvature from those sampled arc
//! edges; planar faces with holes are triangulated by ear clipping with hole
//! bridging. Vertex normals come from the analytic surface, so cylinders and
//! fillets shade smoothly while creases stay hard.

use crate::math::{Vec2, Vec3};
use crate::mesh::{Mesh, weld_triangles};
use crate::topo::{EdgeId, Solid};
use std::collections::HashMap;

/// A triangle with a position and an outward normal at each corner.
#[derive(Clone, Copy)]
pub struct Tri {
    pub pos: [Vec3; 3],
    pub nrm: [Vec3; 3],
}

/// The tessellated result: a triangle soup carrying smooth analytic normals.
#[derive(Clone, Default)]
pub struct Tessellation {
    pub tris: Vec<Tri>,
}

impl Tessellation {
    /// Weld by position into an indexed `Mesh` (for STL + watertight tests).
    pub fn to_mesh(&self) -> Mesh {
        weld_triangles(self.tris.iter().map(|t| t.pos))
    }

    /// Interleaved `[x,y,z, nx,ny,nz]` per vertex, non-indexed — the render VBO.
    pub fn render_buffer(&self) -> Vec<f32> {
        let mut out = Vec::with_capacity(self.tris.len() * 3 * 6);
        for t in &self.tris {
            for i in 0..3 {
                let (p, n) = (t.pos[i], t.nrm[i]);
                out.extend_from_slice(&[p.x, p.y, p.z, n.x, n.y, n.z]);
            }
        }
        out
    }

    /// Axis-aligned bounds of all triangle vertices.
    pub fn bounds(&self) -> (Vec3, Vec3) {
        let mut min = Vec3::splat(f32::INFINITY);
        let mut max = Vec3::splat(f32::NEG_INFINITY);
        for t in &self.tris {
            for p in t.pos {
                min = min.min(p);
                max = max.max(p);
            }
        }
        if self.tris.is_empty() {
            (Vec3::ZERO, Vec3::ZERO)
        } else {
            (min, max)
        }
    }
}

/// Tessellate a solid. `arc_segs_per_quarter` sets curve resolution.
pub fn tessellate(solid: &Solid, arc_segs_per_quarter: usize) -> Tessellation {
    // Sample every edge once, forward, so both incident faces agree.
    let mut edge_pts: HashMap<EdgeId, Vec<Vec3>> = HashMap::new();
    for (id, e) in solid.edges.iter().enumerate() {
        let n = e.seg_count(arc_segs_per_quarter);
        edge_pts.insert(id, e.sample(true, n));
    }

    let mut out = Tessellation::default();
    for face in &solid.faces {
        let sign = if face.sense { 1.0 } else { -1.0 };

        // Flatten every loop into 3D boundary points + uv + analytic normals.
        let mut pts3: Vec<Vec3> = Vec::new();
        let mut uv: Vec<Vec2> = Vec::new();
        let mut nrm: Vec<Vec3> = Vec::new();
        let mut loop_spans: Vec<(usize, usize)> = Vec::new(); // [start,end) into pts3

        for lp in face.loops() {
            let start = pts3.len();
            for &(e, fwd) in &lp.edges {
                let samples = &edge_pts[&e];
                // Walk the edge in its loop direction, dropping the final point
                // (shared with the next edge's first).
                let iter: Box<dyn Iterator<Item = &Vec3>> = if fwd {
                    Box::new(samples.iter())
                } else {
                    Box::new(samples.iter().rev())
                };
                let collected: Vec<Vec3> = iter.copied().collect();
                for &p in &collected[..collected.len() - 1] {
                    let p_uv = face.surface.project(p);
                    pts3.push(p);
                    uv.push(Vec2::new(p_uv.0, p_uv.1));
                    nrm.push(face.surface.normal(p_uv) * sign);
                }
            }
            loop_spans.push((start, pts3.len()));
        }
        if loop_spans.is_empty() || loop_spans[0].1 - loop_spans[0].0 < 3 {
            continue;
        }

        // Curved faces parameterise u as an angle; unwrap it along each loop so
        // the uv polygon stays continuous (no ±2π branch jump) and simple.
        if !matches!(face.surface, crate::geom::Surface::Plane { .. }) {
            for &(s, e) in &loop_spans {
                let mut prev = uv[s].x;
                for p in uv.iter_mut().take(e).skip(s + 1) {
                    while p.x - prev > std::f32::consts::PI {
                        p.x -= 2.0 * std::f32::consts::PI;
                    }
                    while p.x - prev < -std::f32::consts::PI {
                        p.x += 2.0 * std::f32::consts::PI;
                    }
                    prev = p.x;
                }
            }
        }

        // Triangulate in uv: outer loop first, holes after.
        let outer: Vec<usize> = (loop_spans[0].0..loop_spans[0].1).collect();
        let holes: Vec<Vec<usize>> = loop_spans[1..]
            .iter()
            .map(|&(s, e)| (s..e).collect())
            .collect();
        let tris = triangulate(&uv, &outer, &holes);

        // Decide winding ONCE per face: the uv→3D orientation is uniform across
        // a single (developable) face, so an area-weighted vote over all its
        // triangles gives one flip decision — flipping per-triangle would break
        // internal shared edges on curved faces where the geometric normal is
        // numerically noisy.
        let mut vote = 0.0f32;
        for &[a, b, c] in &tris {
            let geo = (pts3[b] - pts3[a]).cross(pts3[c] - pts3[a]);
            let avg = nrm[a] + nrm[b] + nrm[c];
            vote += geo.dot(avg);
        }
        let flip = vote < 0.0;

        for [a, b, c] in tris {
            let (p, nm) = if flip {
                ([pts3[a], pts3[c], pts3[b]], [nrm[a], nrm[c], nrm[b]])
            } else {
                ([pts3[a], pts3[b], pts3[c]], [nrm[a], nrm[b], nrm[c]])
            };
            out.tris.push(Tri { pos: p, nrm: nm });
        }
    }
    out
}

// ─────────────────────────── 2D triangulation ───────────────────────────────
// Planar faces (possibly with holes) are triangulated in uv by `earcutr` — a
// robust, pure-Rust ear-cutter. It operates on a flat coordinate array; we map
// its output indices back to the shared `pts` index space.

/// Triangulate a planar polygon with holes. `outer`/`holes` index into `pts`
/// (uv). Returns triangles as index triples into `pts`.
fn triangulate(pts: &[Vec2], outer: &[usize], holes: &[Vec<usize>]) -> Vec<[usize; 3]> {
    let mut local: Vec<usize> = outer.to_vec();
    let mut data: Vec<f64> = Vec::new();
    for &i in outer {
        data.push(pts[i].x as f64);
        data.push(pts[i].y as f64);
    }
    let mut hole_starts: Vec<usize> = Vec::new();
    for h in holes {
        if h.len() < 3 {
            continue;
        }
        hole_starts.push(local.len());
        for &i in h {
            local.push(i);
            data.push(pts[i].x as f64);
            data.push(pts[i].y as f64);
        }
    }
    let idx = earcutr::earcut(&data, &hole_starts, 2).unwrap_or_default();
    idx.chunks_exact(3)
        .map(|c| [local[c[0]], local[c[1]], local[c[2]]])
        .collect()
}
