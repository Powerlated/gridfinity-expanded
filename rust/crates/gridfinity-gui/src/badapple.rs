
use crate::MESH_STRIDE;
use gridfinity_cad::gridfinity::{self, Params};
use gridfinity_cad::layout::GridCell;
use gridfinity_cad::tessellate;

pub const W: usize = 64;
pub const H: usize = 48;
pub const FPS: f64 = 30.0;

const PITCH: f32 = gridfinity::GRID_PITCH;

const ROW_BYTES: usize = W / 8;
const FRAME_BYTES: usize = ROW_BYTES * H;

static FRAMES: &[u8] = include_bytes!("../assets/badapple.raw");

pub fn frame_count() -> usize {
    FRAMES.len() / FRAME_BYTES
}

pub fn bounds() -> ([f32; 3], [f32; 3]) {
    let h = 2.0 * gridfinity::HEIGHT_PER_UNIT;
    ([0.0, 0.0, 0.0], [W as f32 * PITCH, H as f32 * PITCH, h])
}

#[inline]
fn white(frame: &[u8], x: usize, y: usize) -> bool {
    (frame[y * ROW_BYTES + x / 8] >> (7 - (x % 8))) & 1 == 1
}

fn cell_params() -> Params {
    Params {
        height_units: 2,
        floor_fillet: 0.0,
        magnet_holes: false,
        screw_holes: false,
        ..Params::default()
    }
}

fn components(f: &[u8]) -> Vec<Vec<GridCell>> {
    let mut seen = vec![false; W * H];
    let mut out = Vec::new();
    let mut stack = Vec::new();
    for sy in 0..H {
        for sx in 0..W {
            if seen[sy * W + sx] || !white(f, sx, sy) {
                continue;
            }
            let mut cells = Vec::new();
            seen[sy * W + sx] = true;
            stack.push((sx, sy));
            while let Some((x, y)) = stack.pop() {
                cells.push(GridCell { x: x as i32, y: (H - 1 - y) as i32 });
                let mut nb = |nx: usize, ny: usize, stack: &mut Vec<(usize, usize)>| {
                    if !seen[ny * W + nx] && white(f, nx, ny) {
                        seen[ny * W + nx] = true;
                        stack.push((nx, ny));
                    }
                };
                if x > 0 { nb(x - 1, y, &mut stack); }
                if x + 1 < W { nb(x + 1, y, &mut stack); }
                if y > 0 { nb(x, y - 1, &mut stack); }
                if y + 1 < H { nb(x, y + 1, &mut stack); }
            }
            out.push(cells);
        }
    }
    out
}

fn emit(solid: &gridfinity_cad::kernel::topo::Solid, verts: &mut Vec<f32>) -> usize {
    let src = tessellate(solid, 1).render_buffer();
    for v in src.chunks_exact(6) {
        verts.extend_from_slice(v);
        verts.push(0.0);
    }
    src.len() / (6 * 3)
}

pub fn build_frame(frame: usize) -> (Vec<f32>, usize) {
    let f = &FRAMES[frame * FRAME_BYTES..(frame + 1) * FRAME_BYTES];
    let p = cell_params();
    let mut verts: Vec<f32> = Vec::new();
    let mut tris = 0usize;

    for cells in components(f) {
        match gridfinity::build_piece(&p, &cells, &cells, None) {
            Ok(solid) => tris += emit(&solid, &mut verts),
            Err(_) => {
                for c in &cells {
                    let one = [*c];
                    if let Ok(s) = gridfinity::build_piece(&p, &one, &one, None) {
                        tris += emit(&s, &mut verts);
                    }
                }
            }
        }
    }
    debug_assert_eq!(verts.len() % MESH_STRIDE, 0);
    (verts, tris)
}

use std::sync::mpsc::{Receiver, Sender, channel};
use std::thread::JoinHandle;
use std::time::Instant;

pub struct FrameResult {
    pub frame: usize,
    pub verts: Vec<f32>,
    pub tris: usize,
    pub build_secs: f64,
}

pub struct Worker {
    req: Option<Sender<usize>>,
    res: Receiver<FrameResult>,
    handle: Option<JoinHandle<()>>,
}

impl Worker {
    pub fn spawn() -> Worker {
        let (req_tx, req_rx) = channel::<usize>();
        let (res_tx, res_rx) = channel::<FrameResult>();
        let handle = std::thread::spawn(move || {
            while let Ok(frame) = req_rx.recv() {
                let t = Instant::now();
                let (verts, tris) = build_frame(frame);
                let build_secs = t.elapsed().as_secs_f64();
                if res_tx.send(FrameResult { frame, verts, tris, build_secs }).is_err() {
                    break;
                }
            }
        });
        Worker { req: Some(req_tx), res: res_rx, handle: Some(handle) }
    }

    pub fn request(&self, frame: usize) {
        if let Some(req) = &self.req {
            let _ = req.send(frame);
        }
    }

    pub fn try_recv(&self) -> Option<FrameResult> {
        let mut last = None;
        while let Ok(r) = self.res.try_recv() {
            last = Some(r);
        }
        last
    }
}

impl Drop for Worker {
    fn drop(&mut self) {
        self.req = None;
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Instant;

    #[test]
    fn frames_decode_and_build() {
        let n = frame_count();
        assert!(n > 1000, "asset should hold the whole clip, got {n}");
        let (v0, t0) = build_frame(0);
        assert_eq!(t0, 0, "frame 0 is a black screen");
        assert!(v0.is_empty());
        let (v, t) = build_frame(n / 2);
        assert!(t > 0 && !v.is_empty(), "mid clip has geometry");
    }

    #[test]
    #[ignore]
    fn bin_cost() {
        use gridfinity_cad::gridfinity::{self, Params};
        use gridfinity_cad::layout::GridCell;
        use gridfinity_cad::tessellate;
        let p = Params { height_units: 2, floor_fillet: 0.0, ..Params::default() };
        let cells = vec![GridCell { x: 0, y: 0 }];
        let _ = gridfinity::build_piece(&p, &cells, &cells, None).unwrap();
        let t = Instant::now();
        let iters = 200;
        let mut tris = 0;
        for _ in 0..iters {
            let s = gridfinity::build_piece(&p, &cells, &cells, None).unwrap();
            tris = tessellate(&s, 1).render_buffer().len() / 18;
        }
        let per = t.elapsed().as_secs_f64() / iters as f64;
        println!("one bin: {tris} tris, {:.3} ms build+tess", per * 1e3);
        println!("=> 500 bins/frame = {:.1} ms/frame = {:.1} fps", per * 500.0 * 1e3, 1.0 / (per * 500.0));
    }

    /// Shrink a leaking blob to the smallest still-leaking connected subset, so
    /// the defect can be read off a shape small enough to reason about.
    #[test]
    #[ignore]
    fn leak_hunt() {
        let p = cell_params();
        let connected = |cs: &[GridCell]| -> bool {
            if cs.is_empty() {
                return false;
            }
            let set: std::collections::HashSet<GridCell> = cs.iter().copied().collect();
            let mut seen = std::collections::HashSet::new();
            let mut stack = vec![cs[0]];
            seen.insert(cs[0]);
            while let Some(c) = stack.pop() {
                for d in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
                    let n = GridCell { x: c.x + d.0, y: c.y + d.1 };
                    if set.contains(&n) && seen.insert(n) {
                        stack.push(n);
                    }
                }
            }
            seen.len() == cs.len()
        };
        let leaks = |cs: &[GridCell]| -> usize {
            match gridfinity::build_piece(&p, cs, cs, None) {
                Ok(s) => gridfinity_cad::tessellation_leaks(&tessellate(&s, 1)).len(),
                Err(_) => 0,
            }
        };

        // The scan is slow; the shape it converges on is checked in directly so
        // the analysis below can be iterated on. Set LEAK_SCAN to redo it.
        let mut best: Option<Vec<GridCell>> = (std::env::var("LEAK_SCAN").is_err()).then(|| {
            [(0, 5), (1, 5), (1, 4), (1, 3), (2, 3), (2, 2), (2, 1), (3, 1), (3, 0), (4, 0)]
                .iter()
                .map(|&(x, y)| GridCell { x, y })
                .collect()
        });
        for f in (0..(10.0 * FPS) as usize).take(if best.is_some() { 0 } else { usize::MAX }) {
            for c in components(&FRAMES[f * FRAME_BYTES..(f + 1) * FRAME_BYTES]) {
                if leaks(&c) > 0 && best.as_ref().is_none_or(|b| c.len() < b.len()) {
                    best = Some(c);
                }
            }
        }
        let mut cur = best.expect("no leaking blob in the sample");
        println!("smallest leaking blob: {} cells, {} leaks", cur.len(), leaks(&cur));

        loop {
            let mut shrunk = false;
            for i in 0..cur.len() {
                let mut trial = cur.clone();
                trial.remove(i);
                if connected(&trial) && leaks(&trial) > 0 {
                    cur = trial;
                    shrunk = true;
                    break;
                }
            }
            if !shrunk {
                break;
            }
        }

        let (mx, my) = (
            cur.iter().map(|c| c.x).min().unwrap(),
            cur.iter().map(|c| c.y).min().unwrap(),
        );
        let cur: Vec<GridCell> = cur.iter().map(|c| GridCell { x: c.x - mx, y: c.y - my }).collect();
        println!("minimal: {} cells {:?}", cur.len(), cur);
        let (w, h) = (
            cur.iter().map(|c| c.x).max().unwrap() + 1,
            cur.iter().map(|c| c.y).max().unwrap() + 1,
        );
        for y in (0..h).rev() {
            let row: String = (0..w)
                .map(|x| if cur.contains(&GridCell { x, y }) { '#' } else { '.' })
                .collect();
            println!("  {row}");
        }

        let s = gridfinity::build_piece(&p, &cur, &cur, None).unwrap();
        println!("validate: {:?}", s.validate());
        let report = gridfinity_cad::audit(&s);
        println!("{report}");
        let tess = tessellate(&s, 1);
        let mut bad: Vec<usize> = Vec::new();
        for l in gridfinity_cad::tessellation_leaks(&tess) {
            println!(
                "  leak {:?} -> {:?} imbalance {} count {} faces {:?}",
                l.a, l.b, l.imbalance, l.count, l.faces
            );
            bad.extend(l.faces.iter().copied());
        }
        bad.sort_unstable();
        bad.dedup();
        for fi in bad {
            let f = &s.faces[fi];
            println!(
                "\nface {fi}: {:?} sense {} loops {:?}",
                std::mem::discriminant(&f.surface),
                f.sense,
                s.face_loops(fi).map(|l| l.len()).collect::<Vec<_>>()
            );
            println!("  surface {:?}", f.surface);
            for (li, lp) in s.face_loops(fi).enumerate() {
                println!("  loop {li}:");
                for &(e, fwd) in lp {
                    let ed = s.edges[e];
                    let (v0, v1) = s.directed(e, fwd);
                    println!(
                        "    e{e} {} {:?} -> {:?}",
                        if fwd { '+' } else { '-' },
                        s.vertex(v0),
                        s.vertex(v1)
                    );
                    let _ = ed;
                }
            }
            let tris: Vec<usize> = tess
                .face_of_tri
                .iter()
                .enumerate()
                .filter(|&(_, &f)| f == fi)
                .map(|(i, _)| i)
                .collect();
            println!("  {} triangles", tris.len());
            for i in tris {
                let t = tess.tris[i];
                let n = (t.pos[1] - t.pos[0]).cross(t.pos[2] - t.pos[0]).length();
                println!("    {:?} {:?} {:?}  |n|={n:.6}", t.pos[0], t.pos[1], t.pos[2]);
            }
        }
    }

    #[test]
    #[ignore]
    fn face_shapes() {
        let p = cell_params();
        let mut best: Vec<GridCell> = Vec::new();
        for f in 0..(10.0 * FPS) as usize {
            for c in components(&FRAMES[f * FRAME_BYTES..(f + 1) * FRAME_BYTES]) {
                if c.len() > best.len() {
                    best = c;
                }
            }
        }
        let solid = gridfinity::build_piece(&p, &best, &best, None).unwrap();
        {
            let mut pp = p.clone();
            pp.bins = vec![gridfinity_cad::gridfinity::LogicalBin {
                cells: best.clone(),
                ..Default::default()
            }];
            let t = Instant::now();
            let prog = gridfinity::program(&pp);
            let plan_ms = t.elapsed().as_secs_f64() * 1e3;
            let t = Instant::now();
            let s = gridfinity_cad::kernel::program::run_all(&prog).unwrap();
            println!(
                "plan {plan_ms:.1} ms, run {:.1} ms ({} faces)",
                t.elapsed().as_secs_f64() * 1e3,
                s.faces.len()
            );
        }
        println!(
            "biggest blob: {} cells, {} faces, {} edges, {} verts",
            best.len(),
            solid.faces.len(),
            solid.edges.len(),
            solid.verts.len()
        );
        let mut hist = [0usize; 8];
        let mut worst: Vec<(usize, usize, usize)> = Vec::new();
        for fi in 0..solid.faces.len() {
            let loops = solid.face_loops(fi).count();
            let pts: usize = solid.face_loops(fi).map(|l| l.len()).sum();
            let b = (usize::BITS - pts.leading_zeros()) as usize;
            hist[b.min(7)] += 1;
            if pts > 64 {
                worst.push((fi, loops, pts));
            }
        }
        for (i, n) in hist.iter().enumerate() {
            if *n > 0 {
                println!("  faces with <2^{i} boundary edges: {n}");
            }
        }
        worst.sort_by_key(|w| std::cmp::Reverse(w.2));
        for &(fi, loops, pts) in worst.iter().take(10) {
            println!("  big face {fi}: {loops} loops, {pts} boundary edges");
        }
        unsafe { std::env::set_var("TESS_DIAG", "1") };
        let _ = gridfinity_cad::kernel::tess::tess_diag();
        let t = Instant::now();
        let tess = tessellate(&solid, 1);
        let ms = t.elapsed().as_secs_f64() * 1e3;
        let d = gridfinity_cad::kernel::tess::tess_diag();
        println!("tessellate: {ms:.1} ms, {} tris", tess.tris.len());
        for (n, v) in ["grid", "sample", "earcut", "retain", "chords"].iter().zip(d) {
            println!("  {n:<8} {:>8.1} ms", v as f64 / 1e6);
        }
        let leaks = gridfinity_cad::tessellation_leaks(&tess);
        assert!(leaks.is_empty(), "{} leaks in the biggest blob", leaks.len());

        // The many-hole bridging path only fires on faces with thousands of
        // loops, which no ordinary bin produces -- these blobs are its only
        // real exercise, so sweep the sample rather than trusting one shape.
        let (mut checked, mut cells_seen, mut leaky, mut total) = (0usize, 0usize, 0usize, 0usize);
        for f in 0..(10.0 * FPS) as usize {
            for c in components(&FRAMES[f * FRAME_BYTES..(f + 1) * FRAME_BYTES]) {
                if c.len() < 40 {
                    continue;
                }
                let s = gridfinity::build_piece(&p, &c, &c, None).unwrap();
                let n = gridfinity_cad::tessellation_leaks(&tessellate(&s, 1)).len();
                leaky += (n > 0) as usize;
                total += n;
                checked += 1;
                cells_seen += c.len();
            }
        }
        println!(
            "swept {checked} blobs ({cells_seen} cells): {leaky} leaky, {total} leak edges"
        );
    }

    #[test]
    #[ignore]
    fn profile() {
        use gridfinity_cad::kernel::perf;
        let sample: Vec<usize> = (0..(10.0 * FPS) as usize).collect();

        let (mut blobs, mut cells, mut biggest) = (0usize, 0usize, 0usize);
        for &f in &sample {
            for c in components(&FRAMES[f * FRAME_BYTES..(f + 1) * FRAME_BYTES]) {
                blobs += 1;
                cells += c.len();
                biggest = biggest.max(c.len());
            }
        }
        println!(
            "{} frames: {blobs} blobs ({:.1}/frame), {cells} cells ({:.1}/frame), biggest blob {biggest} cells",
            sample.len(),
            blobs as f64 / sample.len() as f64,
            cells as f64 / sample.len() as f64,
        );

        for &f in &sample {
            let _ = build_frame(f);
        }

        perf::set_enabled(true);
        perf::reset();
        let t = Instant::now();
        let mut tris = 0;
        let mut each: Vec<f64> = Vec::new();
        for &f in &sample {
            let ft = Instant::now();
            tris += build_frame(f).1;
            each.push(ft.elapsed().as_secs_f64() * 1e3);
        }
        let secs = t.elapsed().as_secs_f64();
        let mut sorted = each.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let pct = |p: f64| sorted[((sorted.len() - 1) as f64 * p) as usize];
        println!(
            "\nper-frame ms: p50 {:.1}  p90 {:.1}  p99 {:.1}  max {:.1}  ({} of {} frames over 33.3)",
            pct(0.50),
            pct(0.90),
            pct(0.99),
            sorted[sorted.len() - 1],
            each.iter().filter(|&&m| m > 33.3).count(),
            each.len(),
        );
        let rows = perf::snapshot();
        let allocs = perf::allocs();
        perf::set_enabled(false);

        println!(
            "\n{:.1} ms/frame, {tris} tris ({:.0}/frame), {:.2}M tri/s",
            secs / sample.len() as f64 * 1e3,
            tris as f64 / sample.len() as f64,
            tris as f64 / secs / 1e6
        );
        println!("{:<28} {:>10} {:>9} {:>6}  {:>10} {:>9}", "metric", "calls", "ms", "%", "allocs", "MB");
        for r in &rows {
            println!(
                "{:<28} {:>10} {:>9.1} {:>5.1}%  {:>10} {:>9.1}",
                r.name,
                r.calls,
                r.nanos as f64 / 1e6,
                r.nanos as f64 / 1e9 / secs * 100.0,
                r.alloc_calls,
                r.alloc_bytes as f64 / 1e6,
            );
        }
        println!(
            "total allocs {} ({:.1} MB churn, {:.1} MB peak)",
            allocs.count,
            allocs.bytes as f64 / 1e6,
            allocs.peak_live_bytes as f64 / 1e6
        );
    }

    #[test]
    #[ignore]
    fn throughput() {
        let n = frame_count();
        let (mut tris, mut secs) = (0usize, 0.0f64);
        for frame in (0..n).step_by(37) {
            let t = Instant::now();
            let (_, tc) = build_frame(frame);
            secs += t.elapsed().as_secs_f64();
            tris += tc;
        }
        println!(
            "built {tris} triangles in {secs:.3}s = {:.2}M tri/s",
            tris as f64 / secs / 1e6
        );
    }
}
