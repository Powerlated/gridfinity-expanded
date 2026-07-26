
use crate::kernel::math::{Vec2, Vec3, weld_key};
use crate::kernel::mesh::{Mesh, weld_triangles};
use crate::kernel::topo::{EdgeId, Solid};

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

pub struct EdgeSamples {
    pts: Vec<Vec3>,
    off: Vec<u32>,
}

impl EdgeSamples {
    fn build(solid: &Solid, arc_segs_per_quarter: usize) -> EdgeSamples {
        let mut off = Vec::with_capacity(solid.edges.len() + 1);
        let mut pts = Vec::with_capacity(solid.edges.len() * 2);
        off.push(0);
        for e in &solid.edges {
            e.sample_into(true, e.seg_count(arc_segs_per_quarter), &mut pts);
            off.push(pts.len() as u32);
        }
        EdgeSamples { pts, off }
    }

    #[inline]
    fn get(&self, e: EdgeId) -> &[Vec3] {
        &self.pts[self.off[e] as usize..self.off[e + 1] as usize]
    }
}

#[derive(Default)]
struct Scratch {
    pts3: Vec<Vec3>,
    uv: Vec<Vec2>,
    nrm: Vec<Vec3>,
    spans: Vec<(usize, usize)>,
    tris: Vec<[usize; 3]>,
    planar: crate::kernel::planar::Planar,
    u_i: Vec<f32>,
    v_j: Vec<f32>,
    grid: Vec<Vec3>,
    gnrm: Vec<Vec3>,
}

thread_local! {
    static GRID_NS: std::cell::Cell<u64> = const { std::cell::Cell::new(0) };
    static SAMPLE_NS: std::cell::Cell<u64> = const { std::cell::Cell::new(0) };
    static TRI_NS: std::cell::Cell<u64> = const { std::cell::Cell::new(0) };
    static RETAIN_NS: std::cell::Cell<u64> = const { std::cell::Cell::new(0) };
}

pub fn tess_diag() -> [u64; 4] {
    [
        GRID_NS.with(|c| c.replace(0)),
        SAMPLE_NS.with(|c| c.replace(0)),
        TRI_NS.with(|c| c.replace(0)),
        RETAIN_NS.with(|c| c.replace(0)),
    ]
}

pub fn tessellate(solid: &Solid, arc_segs_per_quarter: usize) -> Tessellation {
    let _perf = crate::kernel::perf::scope(crate::kernel::perf::Metric::Tessellate);
    let es = EdgeSamples::build(solid, arc_segs_per_quarter);

    let mut out = Tessellation::default();
    let mut sc = Scratch::default();
    let diag = std::env::var("TESS_DIAG").is_ok();
    let now = || std::time::Instant::now();
    for (fi, face) in solid.faces.iter().enumerate() {
        let t0 = diag.then(now);
        let sign = if face.sense { 1.0 } else { -1.0 };

        if tess_grid_face(solid, fi, &es, sign, &mut sc, &mut out) {
            if let Some(t) = t0 {
                GRID_NS.with(|c| c.set(c.get() + t.elapsed().as_nanos() as u64));
            }
            continue;
        }

        sc.pts3.clear();
        sc.uv.clear();
        sc.nrm.clear();
        sc.spans.clear();

        let prep = face.surface.prepare();
        let flat_normal = match face.surface {
            crate::kernel::geom::Surface::Plane { normal, .. } => Some(normal * sign),
            _ => None,
        };
        for lp in solid.face_loops(fi) {
            let start = sc.pts3.len();
            for &(e, fwd) in lp {
                let samples = es.get(e);
                for k in 0..samples.len() - 1 {
                    let p = if fwd { samples[k] } else { samples[samples.len() - 1 - k] };
                    let p_uv = prep.project(p);
                    sc.pts3.push(p);
                    sc.uv.push(Vec2::new(p_uv.0, p_uv.1));
                    sc.nrm.push(match flat_normal {
                        Some(n) => n,
                        None => prep.normal(p_uv) * sign,
                    });
                }
            }
            sc.spans.push((start, sc.pts3.len()));
        }
        if sc.spans.is_empty() || sc.spans[0].1 - sc.spans[0].0 < 3 {
            continue;
        }

        if !matches!(face.surface, crate::kernel::geom::Surface::Plane { .. }) {
            for &(s, e) in &sc.spans {
                let mut prev = sc.uv[s].x;
                for p in sc.uv.iter_mut().take(e).skip(s + 1) {
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

        let t1 = diag.then(now);
        triangulate(&mut sc);
        let t2 = diag.then(now);
        // Drop only what a weld would collapse anyway. A triangle whose three
        // vertices are distinct but collinear has zero area, yet its three
        // edges are still paired against its neighbours -- discarding it on
        // area alone punches a three-edge slit in the mesh, which is what a
        // staircase polyomino's cavity floor used to do.
        let pts3 = &sc.pts3;
        sc.tris.retain(|&[a, b, c]| {
            let (ka, kb, kc) =
                (weld_key(pts3[a]), weld_key(pts3[b]), weld_key(pts3[c]));
            ka != kb && kb != kc && ka != kc
        });
        if let (Some(t0), Some(t1), Some(t2)) = (t0, t1, t2) {
            SAMPLE_NS.with(|c| c.set(c.get() + (t1 - t0).as_nanos() as u64));
            TRI_NS.with(|c| c.set(c.get() + (t2 - t1).as_nanos() as u64));
            RETAIN_NS.with(|c| c.set(c.get() + t2.elapsed().as_nanos() as u64));
        }

        let mut uv_area = 0.0f32;
        for &[a, b, c] in &sc.tris {
            uv_area += (sc.uv[b].x - sc.uv[a].x) * (sc.uv[c].y - sc.uv[a].y)
                - (sc.uv[b].y - sc.uv[a].y) * (sc.uv[c].x - sc.uv[a].x);
        }
        let flip = uv_area * face.surface.uv_orientation() * sign < 0.0;

        out.tris.reserve(sc.tris.len());
        out.face_of_tri.reserve(sc.tris.len());
        for &[a, b, c] in &sc.tris {
            let (p, nm) = if flip {
                ([sc.pts3[a], sc.pts3[c], sc.pts3[b]], [sc.nrm[a], sc.nrm[c], sc.nrm[b]])
            } else {
                ([sc.pts3[a], sc.pts3[b], sc.pts3[c]], [sc.nrm[a], sc.nrm[b], sc.nrm[c]])
            };
            out.face_of_tri.push(fi);
            out.tris.push(Tri { pos: p, nrm: nm });
        }
    }
    out
}


fn tess_grid_face(
    solid: &crate::kernel::topo::Solid,
    fid: usize,
    es: &EdgeSamples,
    sign: f32,
    sc: &mut Scratch,
    out: &mut Tessellation,
) -> bool {
    use crate::kernel::geom::Surface;
    let face = &solid.faces[fid];
    if matches!(face.surface, Surface::Plane { .. }) {
        return false;
    }
    let outer = solid.outer_edges(fid);
    if solid.n_inners(fid) != 0 || outer.len() != 4 {
        return false;
    }

    let at = |(e, fwd): (EdgeId, bool), i: usize| -> Vec3 {
        let s = es.get(e);
        if fwd { s[i] } else { s[s.len() - 1 - i] }
    };
    let (e0, e1, e2, e3) = (outer[0], outer[1], outer[2], outer[3]);
    let (m, n) = (es.get(e0.0).len(), es.get(e1.0).len());
    if es.get(e2.0).len() != m || es.get(e3.0).len() != n || m < 2 || n < 2 {
        return false;
    }

    let prep = face.surface.prepare();
    let s = &prep;
    let uv0a = s.project(at(e0, 0));
    let uv0b = s.project(at(e0, m - 1));
    if (uv0a.1 - uv0b.1).abs() > 1e-4 || (uv0a.0 - uv0b.0).abs() < 1e-4 {
        return false;
    }
    let uv1a = s.project(at(e1, 0));
    let uv1b = s.project(at(e1, n - 1));
    if (uv1a.0 - uv1b.0).abs() > 1e-4 {
        return false;
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
    sc.u_i.clear();
    sc.v_j.clear();
    sc.u_i.extend((0..m).map(|i| s.project(at(e0, i)).0));
    sc.v_j.extend((0..n).map(|j| s.project(at(e1, j)).1));
    unwrap(&mut sc.u_i);
    if matches!(face.surface, Surface::Torus { .. } | Surface::Sphere { .. }) {
        unwrap(&mut sc.v_j);
    }
    let (u_i, v_j) = (&sc.u_i, &sc.v_j);

    sc.grid.clear();
    sc.grid.resize(m * n, Vec3::ZERO);
    for i in 0..m {
        for j in 0..n {
            sc.grid[i * n + j] = if j == 0 {
                at(e0, i)
            } else if j == n - 1 {
                at(e2, m - 1 - i)
            } else if i == m - 1 {
                at(e1, j)
            } else if i == 0 {
                at(e3, n - 1 - j)
            } else {
                s.point((u_i[i], v_j[j]))
            };
        }
    }
    sc.gnrm.clear();
    sc.gnrm.reserve(m * n);
    for i in 0..m {
        for j in 0..n {
            sc.gnrm.push(prep.normal((u_i[i], v_j[j])) * sign);
        }
    }
    let du = u_i[m - 1] - u_i[0];
    let dv = v_j[n - 1] - v_j[0];
    let flip = du * dv * face.surface.uv_orientation() * sign < 0.0;

    let count = (m - 1) * (n - 1) * 2;
    out.tris.reserve(count);
    out.face_of_tri.reserve(count);
    let mut emit = |a: (usize, usize), b: (usize, usize), c: (usize, usize)| {
        let g = |q: (usize, usize)| sc.grid[q.0 * n + q.1];
        let nrm_at = |i: usize, j: usize| sc.gnrm[i * n + j];
        let (pa, pb, pc) = if flip { (g(a), g(c), g(b)) } else { (g(a), g(b), g(c)) };
        let (na, nb, nc) = if flip {
            (nrm_at(a.0, a.1), nrm_at(c.0, c.1), nrm_at(b.0, b.1))
        } else {
            (nrm_at(a.0, a.1), nrm_at(b.0, b.1), nrm_at(c.0, c.1))
        };
        out.face_of_tri.push(fid);
        out.tris.push(Tri { pos: [pa, pb, pc], nrm: [na, nb, nc] });
    };
    for i in 0..m - 1 {
        for j in 0..n - 1 {
            let (a, b, c, d) = ((i, j), (i + 1, j), (i + 1, j + 1), (i, j + 1));
            if (i + j) % 2 == 0 {
                emit(a, b, c);
                emit(a, c, d);
            } else {
                emit(a, b, d);
                emit(b, c, d);
            }
        }
    }
    true
}

#[inline]
fn cross2(uv: &[Vec2], a: usize, b: usize, c: usize) -> f32 {
    (uv[b].x - uv[a].x) * (uv[c].y - uv[a].y) - (uv[b].y - uv[a].y) * (uv[c].x - uv[a].x)
}

fn triangulate(sc: &mut Scratch) {
    sc.tris.clear();
    let (s, e) = sc.spans[0];

    if sc.spans.len() == 1 {
        let n = e - s;
        if n == 3 {
            sc.tris.push([s, s + 1, s + 2]);
            return;
        }
        if n == 4 {
            let (a, b, c, d) = (s, s + 1, s + 2, s + 3);
            let (d0, d1) = (cross2(&sc.uv, a, b, c), cross2(&sc.uv, a, c, d));
            let (e0, e1) = (cross2(&sc.uv, b, c, d), cross2(&sc.uv, b, d, a));
            if d0 * d1 > 0.0 {
                sc.tris.push([a, b, c]);
                sc.tris.push([a, c, d]);
                return;
            }
            if e0 * e1 > 0.0 {
                sc.tris.push([b, c, d]);
                sc.tris.push([b, d, a]);
                return;
            }
        }
    }

    sc.planar.run(&sc.uv, &sc.spans, &mut sc.tris);
}
