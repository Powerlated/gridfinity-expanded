#[global_allocator]
static MI: mimalloc::MiMalloc = mimalloc::MiMalloc;


use gridfinity_cad::gridfinity::{LogicalBin, Params, try_build};
use gridfinity_cad::kernel::tess::tessellate;
use gridfinity_cad::layout::GridCell;
use std::time::Instant;

fn blob(w: i32, h: i32) -> Vec<GridCell> {
    let mut cells = Vec::new();
    let (cx, cy) = (w as f32 / 2.0, h as f32 / 2.0);
    let r = (w.min(h) as f32) * 0.45;
    for x in 0..w {
        for y in 0..h {
            let (dx, dy) = (x as f32 + 0.5 - cx, y as f32 + 0.5 - cy);
            let wob = 1.0 + 0.18 * (dy * 0.9).sin() + 0.12 * (dx * 1.3).cos();
            if (dx * dx + dy * dy).sqrt() <= r * wob {
                cells.push(GridCell { x, y });
            }
        }
    }
    cells
}

fn blob_phase(w: i32, h: i32, phase: f32) -> Vec<GridCell> {
    let mut cells = Vec::new();
    let (cx, cy) = (w as f32 / 2.0, h as f32 / 2.0);
    let r = (w.min(h) as f32) * 0.45;
    for x in 0..w {
        for y in 0..h {
            let (dx, dy) = (x as f32 + 0.5 - cx, y as f32 + 0.5 - cy);
            let wob =
                1.0 + 0.18 * (dy * 0.9 + phase).sin() + 0.12 * (dx * 1.3 + phase).cos();
            if (dx * dx + dy * dy).sqrt() <= r * wob {
                cells.push(GridCell { x, y });
            }
        }
    }
    cells
}

#[test]
fn pipeline_bench() {
    use std::sync::mpsc::sync_channel;
    let (w, h) = match std::env::var("SCALE_WH") {
        Ok(s) => {
            let mut it = s.split('x').map(|v| v.parse().unwrap());
            (it.next().unwrap(), it.next().unwrap())
        }
        Err(_) => (32, 32),
    };
    const FRAMES: usize = 12;
    let frames: Vec<Params> = (0..FRAMES)
        .map(|i| params_for(blob_phase(w, h, i as f32 * 0.7), Params::default()))
        .collect();

    let mut serial = f64::INFINITY;
    for _ in 0..3 {
        let t = Instant::now();
        let mut acc = 0usize;
        for p in &frames {
            let s = try_build(p).expect("build");
            acc += tessellate(&s, 4).tris.len();
        }
        serial = serial.min(t.elapsed().as_secs_f64() * 1e3);
        std::hint::black_box(acc);
    }

    let mut piped = f64::INFINITY;
    for _ in 0..3 {
        let t = Instant::now();
        let (tx, rx) = sync_channel::<gridfinity_cad::Solid>(1);
        let fs = frames.clone();
        let builder = std::thread::spawn(move || {
            for p in &fs {
                if tx.send(try_build(p).expect("build")).is_err() {
                    break;
                }
            }
        });
        let mut acc = 0usize;
        for s in rx {
            acc += tessellate(&s, 4).tris.len();
        }
        builder.join().unwrap();
        piped = piped.min(t.elapsed().as_secs_f64() * 1e3);
        std::hint::black_box(acc);
    }

    println!(
        "\n{w}x{h} x{FRAMES} frames: serial {serial:.1}ms ({:.1}ms/frame) | pipelined {piped:.1}ms ({:.1}ms/frame) = {:.2}x",
        serial / FRAMES as f64,
        piped / FRAMES as f64,
        serial / piped
    );
}

fn params_for(cells: Vec<GridCell>, base: Params) -> Params {
    Params { bins: vec![LogicalBin { cells, ..Default::default() }], ..base }
}

fn time_one(w: i32, h: i32) {
    let cells = blob(w, h);
    let n = cells.len();
    let p = params_for(cells, Params::default());

    let first = match try_build(&p) {
        Ok(s) => s,
        Err(e) => {
            println!("{w:>3}x{h:<3} {n:>5} cells   BUILD FAILED: {e}");
            return;
        }
    };
    let faces = first.faces.len();

    let t0 = Instant::now();
    let solid = try_build(&p).expect("build");
    let build_ms = t0.elapsed().as_secs_f64() * 1e3;

    let t1 = Instant::now();
    let tess = tessellate(&solid, 4);
    let tess_ms = t1.elapsed().as_secs_f64() * 1e3;

    let total = build_ms + tess_ms;
    println!(
        "{w:>3}x{h:<3} {n:>5} cells  {faces:>6} faces  {:>7} tris   build {build_ms:>8.2}ms  tess {tess_ms:>7.2}ms  total {total:>8.2}ms  = {:>7.1} fps",
        tess.tris.len(),
        1000.0 / total
    );
}

#[test]
fn tess_bench() {
    let (bw, bh) = match std::env::var("SCALE_WH") {
        Ok(s) => {
            let mut it = s.split('x').map(|v| v.parse().unwrap());
            (it.next().unwrap(), it.next().unwrap())
        }
        Err(_) => (32, 32),
    };
    let p = params_for(blob(bw, bh), Params::default());
    let solid = try_build(&p).expect("build");
    let mut best = f64::INFINITY;
    let mut tris = 0;
    let _ = gridfinity_cad::kernel::tess::tess_diag();
    for _ in 0..25 {
        let t = Instant::now();
        let tess = tessellate(&solid, 4);
        best = best.min(t.elapsed().as_secs_f64() * 1e3);
        tris = tess.tris.len();
    }
    println!("\ntess best of 25: {best:.3}ms for {tris} tris ({} faces)", solid.faces.len());
    let d = gridfinity_cad::kernel::tess::tess_diag();
    let avg = |x: u64| x as f64 / 1e6 / 25.0;
    println!(
        "  per-run avg: grid {:.2} sample {:.2} triangulate {:.2} retain {:.2}",
        avg(d[0]),
        avg(d[1]),
        avg(d[2]),
        avg(d[3])
    );
}

#[test]
fn build_bench() {
    let (w, h) = match std::env::var("SCALE_WH") {
        Ok(s) => {
            let mut it = s.split('x').map(|v| v.parse().unwrap());
            (it.next().unwrap(), it.next().unwrap())
        }
        Err(_) => (32, 32),
    };
    let p = params_for(blob(w, h), Params::default());
    let mut best = f64::INFINITY;
    let mut faces = 0;
    for _ in 0..15 {
        let t = Instant::now();
        let s = try_build(&p).expect("build");
        best = best.min(t.elapsed().as_secs_f64() * 1e3);
        faces = s.faces.len();
    }
    println!("\nbuild best of 15 at {w}x{h}: {best:.3}ms for {faces} faces");
}

#[test]
fn plan_bench() {
    let (w, h) = match std::env::var("SCALE_WH") {
        Ok(s) => {
            let mut it = s.split('x').map(|v| v.parse().unwrap());
            (it.next().unwrap(), it.next().unwrap())
        }
        Err(_) => (48, 48),
    };
    let p = params_for(blob(w, h), Params::default());
    let mut best = f64::INFINITY;
    let mut ops = 0;
    for _ in 0..15 {
        let t = Instant::now();
        let prog = gridfinity_cad::gridfinity::program(&p);
        best = best.min(t.elapsed().as_secs_f64() * 1e3);
        ops = prog.steps.len();
    }
    println!("\nplan best of 15 at {w}x{h}: {best:.3}ms for {ops} ops");
}

#[test]
fn scale_report() {
    println!();
    for (w, h) in [(2, 2), (4, 4), (6, 6), (8, 8), (12, 12), (16, 16), (24, 24), (32, 32)] {
        time_one(w, h);
    }
}

#[test]
fn scale_features() {
    let variants: [(&str, Params); 4] = [
        ("default (h3, fillet 3.0, rc 2.5)", Params::default()),
        ("no floor fillet", Params { floor_fillet: 0.0, ..Default::default() }),
        (
            "no fillet, no cavity radius",
            Params { floor_fillet: 0.0, cavity_corner_radius: 0.0, ..Default::default() },
        ),
        (
            "no fillet/radius, 1 height unit",
            Params {
                floor_fillet: 0.0,
                cavity_corner_radius: 0.0,
                height_units: 1,
                ..Default::default()
            },
        ),
    ];
    println!();
    for (label, base) in variants {
        let p = params_for(blob(24, 24), base);
        let Ok(warm) = try_build(&p) else {
            println!("{label:<34} BUILD FAILED");
            continue;
        };
        let _ = tessellate(&warm, 4);
        let t = Instant::now();
        let solid = try_build(&p).expect("build");
        let tess = tessellate(&solid, 4);
        let ms = t.elapsed().as_secs_f64() * 1e3;
        println!(
            "{label:<34} {:>6} faces {:>7} tris  {ms:>7.2}ms = {:>6.1} fps",
            solid.faces.len(),
            tess.tris.len(),
            1000.0 / ms
        );
    }
}

#[test]
fn scale_profile() {
    let (w, h) = match std::env::var("SCALE_WH") {
        Ok(s) => {
            let mut it = s.split('x').map(|v| v.parse().unwrap());
            (it.next().unwrap(), it.next().unwrap())
        }
        Err(_) => (24, 24),
    };
    profile_at(w, h);
}

fn profile_at(w: i32, h: i32) {
    use gridfinity_cad::kernel::perf;
    let p = params_for(blob(w, h), Params::default());
    let n = p.bins[0].cells.len();

    perf::set_enabled(true);
    let _ = tessellate(&try_build(&p).expect("warm"), 4);
    perf::reset();
    let t = Instant::now();
    let solid = try_build(&p).expect("build");
    let tess = tessellate(&solid, 4);
    let wall = t.elapsed();
    perf::set_enabled(false);

    println!(
        "\n24x24 blob: {n} cells -> {} faces, {} tris in {:?}\n",
        solid.faces.len(),
        tess.tris.len(),
        wall
    );
    println!("{:<34} {:>10} {:>10}", "metric", "time", "calls");
    for r in perf::snapshot() {
        println!(
            "{:<34} {:>10} {:>10}",
            r.name,
            format!("{:?}", std::time::Duration::from_nanos(r.nanos)),
            r.calls
        );
    }
}
