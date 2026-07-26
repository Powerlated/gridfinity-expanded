
use crate::kernel::math::{Vec2, Vec3};
use crate::kernel::mesh::{Mesh, weld_triangles};
use crate::kernel::topo::{EdgeId, Solid};
use std::collections::HashMap;

#[derive(Clone, Copy)]
pub struct Tri {
    pub pos: [Vec3; 3],
    pub nrm: [Vec3; 3],
}

#[derive(Clone, Default)]
pub struct Tessellation {
    pub tris: Vec<Tri>,
    pub face_of_tri: Vec<usize>,
}

impl Tessellation {
    pub fn to_mesh(&self) -> Mesh {
        weld_triangles(self.tris.iter().map(|t| t.pos))
    }

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

pub fn tessellate(solid: &Solid, arc_segs_per_quarter: usize) -> Tessellation {
    let _perf = crate::kernel::perf::scope(crate::kernel::perf::Metric::Tessellate);
    let mut edge_pts: HashMap<EdgeId, Vec<Vec3>> = HashMap::new();
    for (id, e) in solid.edges.iter().enumerate() {
        let n = e.seg_count(arc_segs_per_quarter);
        edge_pts.insert(id, e.sample(true, n));
    }

    let mut out = Tessellation::default();
    for (fi, face) in solid.faces.iter().enumerate() {
        let sign = if face.sense { 1.0 } else { -1.0 };

        if let Some(grid) = tess_grid_face(solid, fi, &edge_pts, sign) {
            out.face_of_tri.extend(std::iter::repeat(fi).take(grid.len()));
            out.tris.extend(grid);
            continue;
        }

        let mut pts3: Vec<Vec3> = Vec::new();
        let mut uv: Vec<Vec2> = Vec::new();
        let mut nrm: Vec<Vec3> = Vec::new();
        let mut loop_spans: Vec<(usize, usize)> = Vec::new();

        for lp in solid.face_loops(fi) {
            let start = pts3.len();
            for &(e, fwd) in lp {
                let samples = &edge_pts[&e];
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

        if !matches!(face.surface, crate::kernel::geom::Surface::Plane { .. }) {
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

        let outer: Vec<usize> = (loop_spans[0].0..loop_spans[0].1).collect();
        let holes: Vec<Vec<usize>> = loop_spans[1..]
            .iter()
            .map(|&(s, e)| (s..e).collect())
            .collect();
        let mut tris = triangulate(&uv, &outer, &holes);
        tris.retain(|&[a, b, c]| {
            let cr = (uv[b].x - uv[a].x) * (uv[c].y - uv[a].y)
                - (uv[b].y - uv[a].y) * (uv[c].x - uv[a].x);
            cr.abs() > 1e-10
        });
        let tris = split_boundary_chords(tris, &uv, &loop_spans);

        let mut uv_area = 0.0f32;
        for &[a, b, c] in &tris {
            uv_area += (uv[b].x - uv[a].x) * (uv[c].y - uv[a].y)
                - (uv[b].y - uv[a].y) * (uv[c].x - uv[a].x);
        }
        let flip = uv_area * face.surface.uv_orientation() * sign < 0.0;

        for [a, b, c] in tris {
            let (p, nm) = if flip {
                ([pts3[a], pts3[c], pts3[b]], [nrm[a], nrm[c], nrm[b]])
            } else {
                ([pts3[a], pts3[b], pts3[c]], [nrm[a], nrm[b], nrm[c]])
            };
            out.face_of_tri.push(fi);
            out.tris.push(Tri { pos: p, nrm: nm });
        }
    }
    out
}


fn split_boundary_chords(
    tris: Vec<[usize; 3]>,
    uv: &[Vec2],
    loop_spans: &[(usize, usize)],
) -> Vec<[usize; 3]> {
    let span_of = |i: usize| loop_spans.iter().position(|&(s, e)| i >= s && i < e);
    let mut used = vec![false; uv.len()];
    let walk = |used: &[bool], from: usize, to: usize, s: usize, e: usize| -> Option<Vec<usize>> {
        let len = e - s;
        let mut v = Vec::new();
        let mut i = (from - s + 1) % len;
        while i != (to - s) % len {
            v.push(s + i);
            i = (i + 1) % len;
        }
        if v.is_empty() {
            return None;
        }
        let (pa, pb) = (uv[from], uv[to]);
        let d = Vec2::new(pb.x - pa.x, pb.y - pa.y);
        let l2 = d.x * d.x + d.y * d.y;
        if l2 <= 0.0 {
            return None;
        }
        for &m in &v {
            if used[m] {
                return None;
            }
            let r = Vec2::new(uv[m].x - pa.x, uv[m].y - pa.y);
            let cr = r.x * d.y - r.y * d.x;
            if cr.abs() > 1e-4 * l2.sqrt() {
                return None;
            }
            let t = (r.x * d.x + r.y * d.y) / l2;
            if !(0.0..=1.0).contains(&t) {
                return None;
            }
        }
        Some(v)
    };

    let mut work = tris;
    for t in &work {
        for &i in t {
            used[i] = true;
        }
    }
    let mut out = Vec::with_capacity(work.len());
    'tri: while let Some(t) = work.pop() {
        for k in 0..3 {
            let (a, b, c) = (t[k], t[(k + 1) % 3], t[(k + 2) % 3]);
            let (Some(sa), Some(sb)) = (span_of(a), span_of(b)) else {
                continue;
            };
            if sa != sb {
                continue;
            }
            let (s, e) = loop_spans[sa];
            let chain = walk(&used, a, b, s, e).or_else(|| {
                walk(&used, b, a, s, e).map(|mut v| {
                    v.reverse();
                    v
                })
            });
            if let Some(chain) = chain {
                let mut prev = a;
                for &m in &chain {
                    used[m] = true;
                    work.push([prev, m, c]);
                    prev = m;
                }
                work.push([prev, b, c]);
                continue 'tri;
            }
        }
        out.push(t);
    }
    out
}

fn tess_grid_face(
    solid: &crate::kernel::topo::Solid,
    fid: usize,
    edge_pts: &HashMap<EdgeId, Vec<Vec3>>,
    sign: f32,
) -> Option<Vec<Tri>> {
    use crate::kernel::geom::Surface;
    let face = &solid.faces[fid];
    if matches!(face.surface, Surface::Plane { .. }) {
        return None;
    }
    let outer = solid.outer_edges(fid);
    if solid.n_inners(fid) != 0 || outer.len() != 4 {
        return None;
    }

    let trav = |(e, fwd): (EdgeId, bool)| -> Vec<Vec3> {
        let s = &edge_pts[&e];
        if fwd { s.clone() } else { s.iter().rev().copied().collect() }
    };
    let p0 = trav(outer[0]);
    let p1 = trav(outer[1]);
    let p2 = trav(outer[2]);
    let p3 = trav(outer[3]);
    let (m, n) = (p0.len(), p1.len());
    if p2.len() != m || p3.len() != n || m < 2 || n < 2 {
        return None;
    }

    let s = &face.surface;
    let uv0a = s.project(p0[0]);
    let uv0b = s.project(p0[m - 1]);
    if (uv0a.1 - uv0b.1).abs() > 1e-4 || (uv0a.0 - uv0b.0).abs() < 1e-4 {
        return None;
    }
    let uv1a = s.project(p1[0]);
    let uv1b = s.project(p1[n - 1]);
    if (uv1a.0 - uv1b.0).abs() > 1e-4 {
        return None;
    }

    let unwrap = |vals: &mut [f32]| {
        for k in 1..vals.len() {
            while vals[k] - vals[k - 1] > std::f32::consts::PI {
                vals[k] -= 2.0 * std::f32::consts::PI;
            }
            while vals[k] - vals[k - 1] < -std::f32::consts::PI {
                vals[k] += 2.0 * std::f32::consts::PI;
            }
        }
    };
    let mut u_i: Vec<f32> = (0..m).map(|i| s.project(p0[i]).0).collect();
    let mut v_j: Vec<f32> = (0..n).map(|j| s.project(p1[j]).1).collect();
    unwrap(&mut u_i);
    if matches!(s, Surface::Torus { .. } | Surface::Sphere { .. }) {
        unwrap(&mut v_j);
    }
    let (u_i, v_j) = (u_i, v_j);

    let mut g = vec![vec![Vec3::ZERO; n]; m];
    for i in 0..m {
        for j in 0..n {
            g[i][j] = if j == 0 {
                p0[i]
            } else if j == n - 1 {
                p2[m - 1 - i]
            } else if i == m - 1 {
                p1[j]
            } else if i == 0 {
                p3[n - 1 - j]
            } else {
                s.point((u_i[i], v_j[j]))
            };
        }
    }
    let nrm_at = |i: usize, j: usize| face.surface.normal((u_i[i], v_j[j])) * sign;

    let mut cells: Vec<[(usize, usize); 3]> = Vec::with_capacity((m - 1) * (n - 1) * 2);
    for i in 0..m - 1 {
        for j in 0..n - 1 {
            let a = (i, j);
            let b = (i + 1, j);
            let c = (i + 1, j + 1);
            let d = (i, j + 1);
            if (i + j) % 2 == 0 {
                cells.push([a, b, c]);
                cells.push([a, c, d]);
            } else {
                cells.push([a, b, d]);
                cells.push([b, c, d]);
            }
        }
    }
    let du = u_i[m - 1] - u_i[0];
    let dv = v_j[n - 1] - v_j[0];
    let flip = du * dv * face.surface.uv_orientation() * sign < 0.0;

    let mut tris = Vec::with_capacity(cells.len());
    for [a, b, c] in cells {
        let (pa, pb, pc) = if flip {
            (g[a.0][a.1], g[c.0][c.1], g[b.0][b.1])
        } else {
            (g[a.0][a.1], g[b.0][b.1], g[c.0][c.1])
        };
        let (na, nb, nc) = if flip {
            (nrm_at(a.0, a.1), nrm_at(c.0, c.1), nrm_at(b.0, b.1))
        } else {
            (nrm_at(a.0, a.1), nrm_at(b.0, b.1), nrm_at(c.0, c.1))
        };
        tris.push(Tri { pos: [pa, pb, pc], nrm: [na, nb, nc] });
    }
    Some(tris)
}

fn triangulate(pts: &[Vec2], outer: &[usize], holes: &[Vec<usize>]) -> Vec<[usize; 3]> {    let mut local: Vec<usize> = outer.to_vec();
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
