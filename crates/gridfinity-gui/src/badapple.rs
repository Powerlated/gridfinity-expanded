
#[cfg(test)]
use crate::MESH_STRIDE;
#[cfg(test)]
use gridfinity_cad::layout::GridCell;
use gridfinity_cad::badapple::{cell_params, components, frame as frame_bits};
use gridfinity_cad::gridfinity;
use gridfinity_cad::tessellate;

pub use gridfinity_cad::badapple::{FPS, bounds, frame_count};

pub const PIPELINE_DEPTH: usize = 3;

fn emit(solid: &gridfinity_cad::kernel::topo::Solid, verts: &mut Vec<f32>) -> usize {
    let src = tessellate(solid, 1).render_buffer();
    gridfinity_render::append_smooth_shaded(
        verts,
        &src,
        glam::Vec3::ZERO,
        gridfinity_render::color_of(crate::DEBUG_BASE_COLOR),
        false,
    );
    src.len() / (6 * 3)
}

#[cfg(test)]
pub fn build_frame(frame: usize) -> (Vec<f32>, usize) {
    let f = frame_bits(frame);
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
    assert_eq!(verts.len() % MESH_STRIDE, 0);
    (verts, tris)
}

use std::sync::mpsc::{Receiver, Sender, SyncSender, channel, sync_channel};
use std::thread::JoinHandle;
use std::time::Instant;

pub struct FrameResult {
    pub frame: usize,
    pub verts: Vec<f32>,
    pub tris: usize,
    pub build_secs: f64,
}

enum Piece {
    Solid(Box<gridfinity_cad::kernel::topo::Solid>),
    End { frame: usize, started: Instant },
}

const PIECE_QUEUE: usize = 4;

pub struct Worker {
    req: Option<Sender<usize>>,
    res: Receiver<FrameResult>,
    build: Option<JoinHandle<()>>,
    tess: Option<JoinHandle<()>>,
}

impl Worker {
    pub fn spawn() -> Worker {
        let (req_tx, req_rx) = channel::<usize>();
        let (piece_tx, piece_rx) = sync_channel::<Piece>(PIECE_QUEUE);
        let (res_tx, res_rx) = channel::<FrameResult>();

        let build = std::thread::spawn(move || build_loop(&req_rx, &piece_tx));
        let tess = std::thread::spawn(move || {
            let mut verts: Vec<f32> = Vec::new();
            let mut tris = 0usize;
            while let Ok(msg) = piece_rx.recv() {
                match msg {
                    Piece::Solid(s) => tris += emit(&s, &mut verts),
                    Piece::End { frame, started } => {
                        let r = FrameResult {
                            frame,
                            verts: std::mem::take(&mut verts),
                            tris,
                            build_secs: started.elapsed().as_secs_f64(),
                        };
                        tris = 0;
                        if res_tx.send(r).is_err() {
                            return;
                        }
                    }
                }
            }
        });

        Worker { req: Some(req_tx), res: res_rx, build: Some(build), tess: Some(tess) }
    }

    pub fn request(&self, frame: usize) {
        if let Some(req) = &self.req {
            let _ = req.send(frame);
        }
    }

    pub fn try_recv(&self) -> Option<(FrameResult, usize)> {
        let mut last = None;
        let mut seen = 0usize;
        while let Ok(r) = self.res.try_recv() {
            last = Some(r);
            seen += 1;
        }
        last.map(|r| (r, seen))
    }
}

impl Drop for Worker {
    fn drop(&mut self) {
        self.req = None;
        if let Some(h) = self.build.take() {
            let _ = h.join();
        }
        if let Some(h) = self.tess.take() {
            let _ = h.join();
        }
    }
}

fn build_loop(req: &Receiver<usize>, out: &SyncSender<Piece>) {
    while let Ok(frame) = req.recv() {
        let started = Instant::now();
        let f = frame_bits(frame);
        let p = cell_params();
        for cells in components(f) {
            match gridfinity::build_piece(&p, &cells, &cells, None) {
                Ok(solid) => {
                    if out.send(Piece::Solid(Box::new(solid))).is_err() {
                        return;
                    }
                }
                Err(_) => {
                    for c in &cells {
                        let one = [*c];
                        if let Ok(s) = gridfinity::build_piece(&p, &one, &one, None) {
                            if out.send(Piece::Solid(Box::new(s))).is_err() {
                                return;
                            }
                        }
                    }
                }
            }
        }
        if out.send(Piece::End { frame, started }).is_err() {
            return;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rayon::prelude::*;
    use std::time::Instant;

    #[test]
    fn pipelined_worker_matches_serial_build() {
        let n = frame_count();
        let probes = [n / 7, n / 3, n / 2, (n * 3) / 4];
        let w = Worker::spawn();
        for f in probes {
            let (want_verts, want_tris) = build_frame(f);
            w.request(f);
            let deadline = Instant::now() + std::time::Duration::from_secs(120);
            let mut got = None;
            while Instant::now() < deadline {
                if let Some((r, _)) = w.try_recv() {
                    got = Some(r);
                    break;
                }
                std::thread::sleep(std::time::Duration::from_millis(2));
            }
            let got = got.unwrap_or_else(|| panic!("frame {f} never came back"));
            assert_eq!(got.frame, f);
            assert_eq!(got.tris, want_tris, "frame {f} triangle count");
            assert_eq!(got.verts.len(), want_verts.len(), "frame {f} vertex payload");
            assert!(got.verts == want_verts, "frame {f} geometry differs from the serial build");
        }
    }

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
    #[ignore = "benchmark: cargo test --release -p gridfinity-gui -- --ignored --nocapture"]
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
    #[ignore = "benchmark: cargo test --release -p gridfinity-gui -- --ignored --nocapture"]
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
            for c in components(frame_bits(f)) {
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
    fn face_shapes() {
        let p = cell_params();
        let blobs: Vec<Vec<GridCell>> = (0..(10.0 * FPS) as usize)
            .into_par_iter()
            .flat_map_iter(|f| components(frame_bits(f)))
            .collect();
        let mut best: Vec<GridCell> = Vec::new();
        for c in &blobs {
            if c.len() > best.len() {
                best = c.clone();
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
        for (n, v) in ["grid", "sample", "triangulate", "retain"].iter().zip(d) {
            println!("  {n:<8} {:>8.1} ms", v as f64 / 1e6);
        }
        let leaks = gridfinity_cad::tessellation_leaks(&tess);
        assert!(leaks.is_empty(), "{} leaks in the biggest blob", leaks.len());

        // The many-hole bridging path only fires on faces with thousands of
        // loops, which no ordinary bin produces -- these blobs are its only
        // real exercise, so sweep the sample rather than trusting one shape.
        let mut seen: std::collections::HashSet<Vec<(i32, i32)>> = std::collections::HashSet::new();
        let sweep: Vec<&Vec<GridCell>> = blobs
            .iter()
            .filter(|c| c.len() >= 40)
            .filter(|c| {
                let mut key: Vec<(i32, i32)> = c.iter().map(|g| (g.x, g.y)).collect();
                key.sort_unstable();
                seen.insert(key)
            })
            .collect();
        let (checked, cells_seen) = (sweep.len(), sweep.iter().map(|c| c.len()).sum::<usize>());
        let (leaky, total) = sweep
            .par_iter()
            .map(|c| {
                let s = gridfinity::build_piece(&p, c, c, None).unwrap();
                let n = gridfinity_cad::tessellation_leaks(&tessellate(&s, 1)).len();
                ((n > 0) as usize, n)
            })
            .reduce(|| (0, 0), |a, b| (a.0 + b.0, a.1 + b.1));
        println!(
            "swept {checked} blobs ({cells_seen} cells): {leaky} leaky, {total} leak edges"
        );
    }

    #[test]
    #[ignore = "benchmark: cargo test --release -p gridfinity-gui -- --ignored --nocapture"]
    fn profile() {
        use gridfinity_cad::kernel::perf;
        let sample: Vec<usize> = (0..(10.0 * FPS) as usize).collect();

        let (mut blobs, mut cells, mut biggest) = (0usize, 0usize, 0usize);
        for &f in &sample {
            for c in components(frame_bits(f)) {
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
    #[ignore = "benchmark: cargo test --release -p gridfinity-gui -- --ignored --nocapture"]
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
            "serial: {tris} triangles in {secs:.3}s = {:.2}M tri/s, {:.1} ms/frame",
            tris as f64 / secs / 1e6,
            secs * 1e3 / (n as f64 / 37.0)
        );
    }

    #[test]
    #[ignore = "benchmark: cargo test --release -p gridfinity-gui -- --ignored --nocapture"]
    fn build_profile() {
        use gridfinity_cad::kernel::perf;
        let n = frame_count();
        let p = cell_params();
        let step = 37;

        let mut comps = 0usize;
        let mut cells_total = 0usize;
        let mut hist = [0usize; 6];
        for frame in (0..n).step_by(step) {
            let f = frame_bits(frame);
            for c in components(f) {
                comps += 1;
                cells_total += c.len();
                let b = match c.len() {
                    1 => 0,
                    2..=4 => 1,
                    5..=16 => 2,
                    17..=64 => 3,
                    65..=256 => 4,
                    _ => 5,
                };
                hist[b] += 1;
            }
        }
        let frames = n.div_ceil(step);
        println!(
            "
{frames} frames: {comps} components ({:.1}/frame), {cells_total} cells ({:.1}/frame, {:.1}/component)",
            comps as f64 / frames as f64,
            cells_total as f64 / frames as f64,
            cells_total as f64 / comps as f64
        );
        println!(
            "component sizes: 1 cell {} | 2-4 {} | 5-16 {} | 17-64 {} | 65-256 {} | 257+ {}",
            hist[0], hist[1], hist[2], hist[3], hist[4], hist[5]
        );

        for frame in (0..n).step_by(step * 5) {
            let f = frame_bits(frame);
            for cells in components(f) {
                let _ = gridfinity::build_piece(&p, &cells, &cells, None);
            }
        }

        perf::set_enabled(true);
        perf::reset();
        let t = Instant::now();
        for frame in (0..n).step_by(step) {
            let f = frame_bits(frame);
            for cells in components(f) {
                let _ = gridfinity::build_piece(&p, &cells, &cells, None);
            }
        }
        let wall = t.elapsed();
        perf::set_enabled(false);

        println!("
build only: {:?} total, {:.2} ms/frame
", wall, wall.as_secs_f64() * 1e3 / frames as f64);
        println!("{:<34} {:>10} {:>10} {:>12}", "metric", "time", "calls", "allocs");
        let mut rows = perf::snapshot();
        rows.sort_by_key(|r| std::cmp::Reverse(r.nanos));
        for r in &rows {
            println!(
                "{:<34} {:>10} {:>10} {:>12}",
                r.name,
                format!("{:?}", std::time::Duration::from_nanos(r.nanos)),
                r.calls,
                r.alloc_calls
            );
        }
        let a = perf::allocs();
        println!("total allocs {} · churn {} kB", a.count, a.bytes / 1000);
    }

    #[test]
    #[ignore = "benchmark: cargo test --release -p gridfinity-gui -- --ignored --nocapture"]
    fn phase_split() {
        let n = frame_count();
        let p = cell_params();
        let (mut bs, mut ts) = (0.0f64, 0.0f64);
        for frame in (0..n).step_by(37) {
            let f = frame_bits(frame);
            for cells in components(f) {
                let t0 = Instant::now();
                let Ok(solid) = gridfinity::build_piece(&p, &cells, &cells, None) else { continue };
                bs += t0.elapsed().as_secs_f64();
                let t1 = Instant::now();
                let mut v = Vec::new();
                emit(&solid, &mut v);
                ts += t1.elapsed().as_secs_f64();
            }
        }
        println!("badapple split: build {bs:.3}s tess {ts:.3}s -> tess is {:.1}% ; pipeline ceiling {:.2}x",
            100.0*ts/(bs+ts), (bs+ts)/bs.max(ts));
    }

    #[test]
    #[ignore = "benchmark: cargo test --release -p gridfinity-gui -- --ignored --nocapture"]
    fn throughput_pipelined() {
        let n = frame_count();
        let depth: usize =
            std::env::var("INFLIGHT").ok().and_then(|v| v.parse().ok()).unwrap_or(1);
        let list: Vec<usize> = (0..n).step_by(37).collect();
        let w = Worker::spawn();
        let t = Instant::now();
        let mut next = 0usize;
        let mut done = 0usize;
        while done < list.len() {
            while next < list.len() && next - done < depth {
                w.request(list[next]);
                next += 1;
            }
            if let Some((r, _)) = w.try_recv() {
                let at = list.iter().position(|&f| f == r.frame).unwrap();
                done = at + 1;
            } else {
                std::hint::spin_loop();
            }
        }
        let secs = t.elapsed().as_secs_f64();
        println!(
            "pipelined(depth={depth}): {} frames in {secs:.3}s = {:.1} ms/frame",
            list.len(),
            secs * 1e3 / list.len() as f64
        );
    }
}
