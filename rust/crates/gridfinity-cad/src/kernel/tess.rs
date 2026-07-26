
use crate::kernel::math::{Vec2, Vec3};
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
    work: Vec<[usize; 3]>,
    used: Vec<bool>,
    span_id: Vec<u32>,
    chain: Vec<usize>,
    ec_data: Vec<f64>,
    ec_holes: Vec<usize>,
    ec_local: Vec<usize>,
    ring: bridge::Ring,
    u_i: Vec<f32>,
    v_j: Vec<f32>,
    grid: Vec<Vec3>,
    gnrm: Vec<Vec3>,
}

thread_local! {
    static GRID_NS: std::cell::Cell<u64> = const { std::cell::Cell::new(0) };
    static SAMPLE_NS: std::cell::Cell<u64> = const { std::cell::Cell::new(0) };
    static EARCUT_NS: std::cell::Cell<u64> = const { std::cell::Cell::new(0) };
    static RETAIN_NS: std::cell::Cell<u64> = const { std::cell::Cell::new(0) };
    static CHORD_NS: std::cell::Cell<u64> = const { std::cell::Cell::new(0) };
}

pub fn tess_diag() -> [u64; 5] {
    [
        GRID_NS.with(|c| c.replace(0)),
        SAMPLE_NS.with(|c| c.replace(0)),
        EARCUT_NS.with(|c| c.replace(0)),
        RETAIN_NS.with(|c| c.replace(0)),
        CHORD_NS.with(|c| c.replace(0)),
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
        let uv = &sc.uv;
        sc.tris.retain(|&[a, b, c]| {
            let cr = (uv[b].x - uv[a].x) * (uv[c].y - uv[a].y)
                - (uv[b].y - uv[a].y) * (uv[c].x - uv[a].x);
            cr.abs() > 1e-10
        });
        let t3 = diag.then(now);
        split_boundary_chords(&mut sc);
        if let (Some(t0), Some(t1), Some(t2), Some(t3)) = (t0, t1, t2, t3) {
            SAMPLE_NS.with(|c| c.set(c.get() + (t1 - t0).as_nanos() as u64));
            EARCUT_NS.with(|c| c.set(c.get() + (t2 - t1).as_nanos() as u64));
            RETAIN_NS.with(|c| c.set(c.get() + (t3 - t2).as_nanos() as u64));
            CHORD_NS.with(|c| c.set(c.get() + t3.elapsed().as_nanos() as u64));
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


fn walk_chain(
    uv: &[Vec2],
    used: &[bool],
    chain: &mut Vec<usize>,
    from: usize,
    to: usize,
    s: usize,
    e: usize,
) -> bool {
    let len = e - s;
    chain.clear();
    let mut i = (from - s + 1) % len;
    while i != (to - s) % len {
        chain.push(s + i);
        i = (i + 1) % len;
    }
    if chain.is_empty() {
        return false;
    }
    let (pa, pb) = (uv[from], uv[to]);
    let d = Vec2::new(pb.x - pa.x, pb.y - pa.y);
    let l2 = d.x * d.x + d.y * d.y;
    if l2 <= 0.0 {
        return false;
    }
    for &m in chain.iter() {
        if used[m] {
            return false;
        }
        let r = Vec2::new(uv[m].x - pa.x, uv[m].y - pa.y);
        let cr = r.x * d.y - r.y * d.x;
        if cr.abs() > 1e-4 * l2.sqrt() {
            return false;
        }
        let t = (r.x * d.x + r.y * d.y) / l2;
        if !(0.0..=1.0).contains(&t) {
            return false;
        }
    }
    true
}

fn split_boundary_chords(sc: &mut Scratch) {
    sc.span_id.clear();
    sc.span_id.resize(sc.uv.len(), u32::MAX);
    for (si, &(s, e)) in sc.spans.iter().enumerate() {
        sc.span_id[s..e].fill(si as u32);
    }

    sc.used.clear();
    sc.used.resize(sc.uv.len(), false);
    for t in &sc.tris {
        for &i in t {
            sc.used[i] = true;
        }
    }

    sc.work.clear();
    std::mem::swap(&mut sc.work, &mut sc.tris);

    'tri: while let Some(t) = sc.work.pop() {
        for k in 0..3 {
            let (a, b, c) = (t[k], t[(k + 1) % 3], t[(k + 2) % 3]);
            let (sa, sb) = (sc.span_id[a], sc.span_id[b]);
            if sa != sb || sa == u32::MAX {
                continue;
            }
            let (s, e) = sc.spans[sa as usize];
            let mut chain = std::mem::take(&mut sc.chain);
            let mut ok = walk_chain(&sc.uv, &sc.used, &mut chain, a, b, s, e);
            if !ok && walk_chain(&sc.uv, &sc.used, &mut chain, b, a, s, e) {
                chain.reverse();
                ok = true;
            }
            if ok {
                let mut prev = a;
                for &m in chain.iter() {
                    sc.used[m] = true;
                    sc.work.push([prev, m, c]);
                    prev = m;
                }
                sc.work.push([prev, b, c]);
                sc.chain = chain;
                continue 'tri;
            }
            sc.chain = chain;
        }
        sc.tris.push(t);
    }
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

/// Bridging a planar face's holes into one simple polygon.
///
/// `earcutr` eliminates holes by rescanning the whole merged ring once per
/// hole, which is quadratic: a Gridfinity bridge underside with 2016 peg-top
/// holes spends ~200 ms there, over 4/5 of a large bin's tessellation. The
/// bridge search itself is local, so a uniform grid over the boundary segments
/// answers it in constant time and the ring is handed to earcut hole-free.
///
/// The search is a faithful port of earcut's `findHoleBridge` (leftward ray
/// from each hole's leftmost vertex, then the minimum-angle visible vertex
/// inside the resulting triangle); only the two ring scans become grid queries.
mod bridge {
    use super::Vec2;

    const CELL_TARGET: usize = 2;

    #[inline]
    fn area3(px: f32, py: f32, qx: f32, qy: f32, rx: f32, ry: f32) -> f32 {
        (qy - py) * (rx - qx) - (qx - px) * (ry - qy)
    }

    #[inline]
    fn in_tri(ax: f32, ay: f32, bx: f32, by: f32, cx: f32, cy: f32, px: f32, py: f32) -> bool {
        (cx - px) * (ay - py) - (ax - px) * (cy - py) >= 0.0
            && (ax - px) * (by - py) - (bx - px) * (ay - py) >= 0.0
            && (bx - px) * (cy - py) - (cx - px) * (by - py) >= 0.0
    }

    #[derive(Default)]
    pub struct Ring {
        x: Vec<f32>,
        y: Vec<f32>,
        v: Vec<u32>,
        next: Vec<u32>,
        prev: Vec<u32>,
        cells: Vec<Vec<u32>>,
        stamp: Vec<u32>,
        epoch: u32,
        nx: usize,
        ny: usize,
        min: Vec2,
        inv: f32,
        holes: Vec<u32>,
    }

    impl Ring {
        fn node(&mut self, x: f32, y: f32, v: u32) -> u32 {
            let i = self.x.len() as u32;
            self.x.push(x);
            self.y.push(y);
            self.v.push(v);
            self.next.push(i);
            self.prev.push(i);
            self.stamp.push(0);
            i
        }

        #[inline]
        fn col(&self, x: f32) -> usize {
            (((x - self.min.x) * self.inv) as isize).clamp(0, self.nx as isize - 1) as usize
        }
        #[inline]
        fn row(&self, y: f32) -> usize {
            (((y - self.min.y) * self.inv) as isize).clamp(0, self.ny as isize - 1) as usize
        }

        /// Register `n`'s outgoing edge in every cell its bounding box touches.
        fn index(&mut self, n: u32) {
            let m = self.next[n as usize];
            let (ax, ay) = (self.x[n as usize], self.y[n as usize]);
            let (bx, by) = (self.x[m as usize], self.y[m as usize]);
            let (i0, i1) = (self.col(ax.min(bx)), self.col(ax.max(bx)));
            let (j0, j1) = (self.row(ay.min(by)), self.row(ay.max(by)));
            for i in i0..=i1 {
                for j in j0..=j1 {
                    self.cells[i * self.ny + j].push(n);
                }
            }
        }
    }

    /// Signed area of `uv[s..e]` traversed in order; positive is CCW.
    fn span_ccw(uv: &[Vec2], s: usize, e: usize) -> bool {
        let mut a = 0.0f32;
        for i in s..e {
            let p = uv[i];
            let q = uv[if i + 1 == e { s } else { i + 1 }];
            a += p.x * q.y - q.x * p.y;
        }
        a > 0.0
    }

    /// Merge every hole span into span 0, writing the resulting single-loop
    /// vertex order (indices into `uv`) to `order`. Returns false when the
    /// topology is not bridgeable, in which case the caller falls back to
    /// earcut's own hole handling.
    pub fn merge(uv: &[Vec2], spans: &[(usize, usize)], r: &mut Ring, order: &mut Vec<usize>) -> bool {
        r.x.clear();
        r.y.clear();
        r.v.clear();
        r.next.clear();
        r.prev.clear();
        r.stamp.clear();
        r.holes.clear();
        r.epoch = 0;

        let total: usize = spans.iter().map(|&(s, e)| e - s).sum();
        let (mut lo, mut hi) = (Vec2::splat(f32::INFINITY), Vec2::splat(f32::NEG_INFINITY));
        for &(s, e) in spans {
            for p in &uv[s..e] {
                lo = lo.min(*p);
                hi = hi.max(*p);
            }
        }
        let side = (hi - lo).max_element();
        if !side.is_finite() || side <= 0.0 {
            return false;
        }
        let k = ((total / CELL_TARGET) as f32).sqrt().max(1.0).min(512.0);
        r.inv = k / side;
        r.min = lo;
        r.nx = (((hi.x - lo.x) * r.inv) as usize + 1).min(512);
        r.ny = (((hi.y - lo.y) * r.inv) as usize + 1).min(512);
        r.cells.clear();
        r.cells.resize(r.nx * r.ny, Vec::new());
        for c in &mut r.cells {
            c.clear();
        }

        let mut outer_start = 0u32;
        for (si, &(s, e)) in spans.iter().enumerate() {
            if e - s < 3 {
                if si == 0 {
                    return false;
                }
                continue;
            }
            let want_ccw = si == 0;
            let fwd = span_ccw(uv, s, e) == want_ccw;
            let first = r.x.len() as u32;
            let mut leftmost = first;
            for k in 0..e - s {
                let i = if fwd { s + k } else { e - 1 - k };
                let p = uv[i];
                let n = r.node(p.x, p.y, i as u32);
                if p.x < r.x[leftmost as usize] {
                    leftmost = n;
                }
            }
            let last = r.x.len() as u32 - 1;
            for n in first..=last {
                r.next[n as usize] = if n == last { first } else { n + 1 };
                r.prev[n as usize] = if n == first { last } else { n - 1 };
            }
            if si == 0 {
                outer_start = first;
            } else {
                r.holes.push(leftmost);
            }
        }
        for n in 0..r.x.len() as u32 {
            r.index(n);
        }

        let mut holes = std::mem::take(&mut r.holes);
        holes.sort_unstable_by(|&a, &b| r.x[a as usize].total_cmp(&r.x[b as usize]));
        for &h in &holes {
            let Some(m) = find_bridge(r, h) else {
                r.holes = holes;
                return false;
            };
            splice(r, m, h);
        }
        r.holes = holes;

        order.clear();
        order.reserve(r.x.len());
        let mut n = outer_start;
        loop {
            order.push(r.v[n as usize] as usize);
            n = r.next[n as usize];
            if n == outer_start {
                break;
            }
            if order.len() > r.x.len() {
                return false;
            }
        }
        order.len() >= 3
    }

    fn find_bridge(r: &mut Ring, hole: u32) -> Option<u32> {
        let (hx, hy) = (r.x[hole as usize], r.y[hole as usize]);
        let (mut qx, mut m) = (f32::NEG_INFINITY, None::<u32>);
        let hj = r.row(hy);
        let mut i = r.col(hx) as isize;
        while i >= 0 {
            for k in 0..r.cells[i as usize * r.ny + hj].len() {
                let n = r.cells[i as usize * r.ny + hj][k];
                let p = r.next[n as usize];
                let (ax, ay) = (r.x[n as usize], r.y[n as usize]);
                let (bx, by) = (r.x[p as usize], r.y[p as usize]);
                if !(hy <= ay && hy >= by) || ay == by {
                    continue;
                }
                let cx = ax + (hy - ay) * (bx - ax) / (by - ay);
                if cx > hx || cx <= qx {
                    continue;
                }
                qx = cx;
                if cx == hx && hy == ay {
                    return Some(n);
                }
                if cx == hx && hy == by {
                    return Some(p);
                }
                m = Some(if ax < bx { n } else { p });
            }
            let left = r.min.x + i as f32 / r.inv;
            if m.is_some() && qx >= left {
                break;
            }
            i -= 1;
        }
        let mut m = m?;
        if hx == qx {
            return Some(r.prev[m as usize]);
        }

        let (mx, my) = (r.x[m as usize], r.y[m as usize]);
        let (x1, x2) = if hy < my { (hx, qx) } else { (qx, hx) };
        let mut tan_min = f32::INFINITY;
        r.epoch += 1;
        let g = r.epoch;
        let (i0, i1) = (r.col(qx.min(hx)), r.col(qx.max(hx)));
        let (j0, j1) = (r.row(hy.min(my)), r.row(hy.max(my)));
        for i in i0..=i1 {
            for j in j0..=j1 {
                for k in 0..r.cells[i * r.ny + j].len() {
                    let cell = r.cells[i * r.ny + j][k];
                    for p in [cell, r.next[cell as usize]] {
                        if r.stamp[p as usize] == g || p == m {
                            continue;
                        }
                        r.stamp[p as usize] = g;
                        let (px, py) = (r.x[p as usize], r.y[p as usize]);
                        if !(hx > px && px >= mx) {
                            continue;
                        }
                        if !in_tri(x1, hy, mx, my, x2, hy, px, py) {
                            continue;
                        }
                        let tan = (hy - py).abs() / (hx - px);
                        if (tan < tan_min || (tan == tan_min && px > r.x[m as usize]))
                            && locally_inside(r, p, hole)
                        {
                            m = p;
                            tan_min = tan;
                        }
                    }
                }
            }
        }
        Some(m)
    }

    fn locally_inside(r: &Ring, a: u32, b: u32) -> bool {
        let (pv, nx) = (r.prev[a as usize], r.next[a as usize]);
        let (ax, ay) = (r.x[a as usize], r.y[a as usize]);
        let (bx, by) = (r.x[b as usize], r.y[b as usize]);
        let (px, py) = (r.x[pv as usize], r.y[pv as usize]);
        let (qx, qy) = (r.x[nx as usize], r.y[nx as usize]);
        if area3(px, py, ax, ay, qx, qy) < 0.0 {
            area3(ax, ay, bx, by, qx, qy) >= 0.0 && area3(ax, ay, px, py, bx, by) >= 0.0
        } else {
            area3(ax, ay, bx, by, px, py) < 0.0 || area3(ax, ay, qx, qy, bx, by) < 0.0
        }
    }

    /// earcut's `splitPolygon`: duplicate both endpoints so the bridge is
    /// traversed once in each direction, joining hole and outer into one ring.
    fn splice(r: &mut Ring, a: u32, b: u32) {
        let c = r.node(r.x[a as usize], r.y[a as usize], r.v[a as usize]);
        let d = r.node(r.x[b as usize], r.y[b as usize], r.v[b as usize]);
        let an = r.next[a as usize];
        let bp = r.prev[b as usize];

        r.next[a as usize] = b;
        r.prev[b as usize] = a;
        r.next[c as usize] = an;
        r.prev[an as usize] = c;
        r.next[d as usize] = c;
        r.prev[c as usize] = d;
        r.next[bp as usize] = d;
        r.prev[d as usize] = bp;

        for n in [a, c, d, bp] {
            r.index(n);
        }
    }
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

    // Many-hole faces go through our own bridging; earcut's is quadratic in the
    // hole count. Few-hole faces stay on earcut's proven path.
    const BRIDGE_ABOVE: usize = 24;
    if sc.spans.len() > BRIDGE_ABOVE
        && bridge::merge(&sc.uv, &sc.spans, &mut sc.ring, &mut sc.ec_local)
    {
        sc.ec_data.clear();
        sc.ec_data.reserve(sc.ec_local.len() * 2);
        for &i in &sc.ec_local {
            sc.ec_data.push(sc.uv[i].x as f64);
            sc.ec_data.push(sc.uv[i].y as f64);
        }
        let idx = earcutr::earcut(&sc.ec_data, &[], 2).unwrap_or_default();
        sc.tris.extend(
            idx.chunks_exact(3)
                .map(|c| [sc.ec_local[c[0]], sc.ec_local[c[1]], sc.ec_local[c[2]]]),
        );
        return;
    }

    sc.ec_local.clear();
    sc.ec_data.clear();
    sc.ec_holes.clear();
    for i in s..e {
        sc.ec_local.push(i);
        sc.ec_data.push(sc.uv[i].x as f64);
        sc.ec_data.push(sc.uv[i].y as f64);
    }
    for &(hs, he) in &sc.spans[1..] {
        if he - hs < 3 {
            continue;
        }
        sc.ec_holes.push(sc.ec_local.len());
        for i in hs..he {
            sc.ec_local.push(i);
            sc.ec_data.push(sc.uv[i].x as f64);
            sc.ec_data.push(sc.uv[i].y as f64);
        }
    }
    let idx = earcutr::earcut(&sc.ec_data, &sc.ec_holes, 2).unwrap_or_default();
    sc.tris.extend(
        idx.chunks_exact(3)
            .map(|c| [sc.ec_local[c[0]], sc.ec_local[c[1]], sc.ec_local[c[2]]]),
    );
}
