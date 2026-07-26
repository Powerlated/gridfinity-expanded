
use gridfinity_cad::gridfinity::{
    self, BinSlope, InnerWall, LogicalBin, Mode, Params, SlopeDir, rect_cells,
};
use gridfinity_cad::kernel::tess::tessellate;
use gridfinity_cad::layout::{GridCell, GridEdge, Orientation};
use gridfinity_cad::{audit, tessellation_leaks};
use std::collections::BTreeMap;
use std::panic::{AssertUnwindSafe, catch_unwind};


struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Rng {
        Rng(seed | 1)
    }
    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    fn below(&mut self, n: u32) -> u32 {
        (self.next_u64() >> 32) as u32 % n.max(1)
    }
    fn range(&mut self, lo: f32, hi: f32) -> f32 {
        let t = (self.next_u64() >> 40) as f32 / (1u64 << 24) as f32;
        lo + (hi - lo) * t
    }
    fn quantised(&mut self, lo: f32, hi: f32, step: f32) -> f32 {
        (self.range(lo, hi) / step).round() * step
    }
    fn chance(&mut self, num: u32, den: u32) -> bool {
        self.below(den) < num
    }
    fn pick<T: Copy>(&mut self, xs: &[T]) -> T {
        xs[self.below(xs.len() as u32) as usize]
    }
}

#[derive(Clone, Copy, PartialEq)]
enum Profile {
    InnerWalls,
    Broad,
}

fn gen_cells(rng: &mut Rng) -> Vec<GridCell> {
    let (gx, gy) = (rng.below(3) + 1, rng.below(3) + 1);
    let mut cells = rect_cells(gx, gy);
    if cells.len() > 2 && rng.chance(1, 3) {
        let victim = rng.below(cells.len() as u32) as usize;
        cells.remove(victim);
    }
    cells
}

fn gen_inner_wall(rng: &mut Rng, cells: &[GridCell]) -> InnerWall {
    let span = |sel: fn(&GridCell) -> i32| {
        let hi = cells.iter().map(sel).max().unwrap_or(0);
        (hi + 1) as f32 * 42.0
    };
    let (w, h) = (span(|c| c.x), span(|c| c.y));
    let m = 12.0;
    InnerWall {
        x1: rng.quantised(-m, w + m, 0.5),
        y1: rng.quantised(-m, h + m, 0.5),
        x2: rng.quantised(-m, w + m, 0.5),
        y2: rng.quantised(-m, h + m, 0.5),
        width: rng.quantised(0.8, 6.0, 0.2),
        height: if rng.chance(1, 3) {
            Some(rng.quantised(2.0, 16.0, 0.5))
        } else {
            None
        },
    }
}

fn gen_case(rng: &mut Rng, profile: Profile) -> Params {
    let base = Params::default();
    let cells = if profile == Profile::Broad {
        gen_cells(rng)
    } else {
        rect_cells(2, 2)
    };

    let mut p = Params {
        bins: vec![LogicalBin { cells: cells.clone(), ..Default::default() }],
        ..base
    };

    let n_walls = match profile {
        Profile::InnerWalls => rng.below(3) + 1,
        Profile::Broad => rng.below(3),
    };
    p.inner_walls = (0..n_walls).map(|_| gen_inner_wall(rng, &cells)).collect();

    if profile == Profile::Broad {
        p.height_units = rng.below(6) + 1;
        p.wall_thickness = rng.quantised(0.4, 3.0, 0.1);
        p.cavity_corner_radius = rng.quantised(0.0, 5.0, 0.5);
        p.floor_fillet = rng.quantised(0.0, 5.6, 0.2);
        p.magnet_holes = rng.chance(1, 3);
        p.screw_holes = p.magnet_holes && rng.chance(1, 2);
        p.mode = if rng.chance(1, 8) { Mode::Baseplate } else { Mode::Bin };
        if rng.chance(1, 6) {
            p.bins[0].slope = Some(BinSlope {
                angle_deg: rng.quantised(2.0, 20.0, 1.0),
                dir: rng.pick(&[
                    SlopeDir::PlusX,
                    SlopeDir::MinusX,
                    SlopeDir::PlusY,
                    SlopeDir::MinusY,
                ]),
            });
        }
        for _ in 0..rng.below(3) {
            let c = cells[rng.below(cells.len() as u32) as usize];
            p.divider_edges.push(GridEdge {
                x: c.x,
                y: c.y,
                orientation: if rng.chance(1, 2) { Orientation::H } else { Orientation::V },
            });
        }
    }
    p
}


fn check(p: &Params) -> Result<(), String> {
    let prev = std::panic::take_hook();
    std::panic::set_hook(Box::new(|_| {}));
    let outcome = catch_unwind(AssertUnwindSafe(|| {
        let solid = gridfinity::try_build(p).map_err(|e| format!("build error: {e}"))?;
        solid.validate().map_err(|e| format!("validate: {e}"))?;
        let report = audit(&solid);
        if !report.is_ok() {
            return Err(format!("audit: {report}"));
        }
        let leaks = tessellation_leaks(&tessellate(&solid, 6));
        if !leaks.is_empty() {
            return Err(format!("tessellation: {} leak(s), first {:?}", leaks.len(), leaks[0]));
        }
        Ok(())
    }));
    std::panic::set_hook(prev);

    match outcome {
        Ok(r) => r,
        Err(e) => {
            let msg = e
                .downcast_ref::<&str>()
                .map(|s| s.to_string())
                .or_else(|| e.downcast_ref::<String>().cloned())
                .unwrap_or_else(|| "<non-string payload>".into());
            Err(format!("panic: {msg}"))
        }
    }
}

fn signature(err: &str) -> String {
    let mut out = String::new();
    let mut in_num = false;
    for c in err.chars() {
        if c.is_ascii_digit() || (c == '.' && in_num) {
            if !in_num {
                out.push('#');
                in_num = true;
            }
        } else {
            in_num = false;
            out.push(c);
        }
    }
    out.chars().take(140).collect()
}


fn shrink(p: &Params, sig: &str) -> Params {
    let same = |q: &Params| check(q).is_err_and(|e| signature(&e) == sig);
    let mut best = p.clone();

    for i in (0..best.inner_walls.len()).rev() {
        let mut q = best.clone();
        q.inner_walls.remove(i);
        if same(&q) {
            best = q;
        }
    }
    for i in (0..best.bins[0].cells.len()).rev() {
        if best.bins[0].cells.len() <= 1 {
            break;
        }
        let mut q = best.clone();
        q.bins[0].cells.remove(i);
        if same(&q) {
            best = q;
        }
    }
    for i in (0..best.divider_edges.len()).rev() {
        let mut q = best.clone();
        q.divider_edges.remove(i);
        if same(&q) {
            best = q;
        }
    }
    if best.bins[0].slope.is_some() {
        let mut q = best.clone();
        q.bins[0].slope = None;
        if same(&q) {
            best = q;
        }
    }
    let d = Params::default();
    for (get, set) in [
        (
            (|p: &Params| p.floor_fillet) as fn(&Params) -> f32,
            (|p: &mut Params, v: f32| p.floor_fillet = v) as fn(&mut Params, f32),
        ),
        (|p| p.cavity_corner_radius, |p, v| p.cavity_corner_radius = v),
        (|p| p.wall_thickness, |p, v| p.wall_thickness = v),
    ] {
        let mut q = best.clone();
        set(&mut q, get(&d));
        if same(&q) {
            best = q;
        }
    }
    best
}

fn repro(p: &Params) -> String {
    let d = Params::default();
    let mut f: Vec<String> = Vec::new();
    let cells = &p.bins[0].cells;
    let slope = p.bins[0].slope;
    if *cells != d.bins[0].cells || slope.is_some() {
        let cs: Vec<String> =
            cells.iter().map(|c| format!("GridCell {{ x: {}, y: {} }}", c.x, c.y)).collect();
        let sl = match slope {
            Some(s) => format!(
                ", slope: Some(BinSlope {{ angle_deg: {:?}, dir: SlopeDir::{:?} }})",
                s.angle_deg, s.dir
            ),
            None => String::new(),
        };
        f.push(format!(
            "bins: vec![LogicalBin {{ cells: vec![{}]{sl}, ..Default::default() }}]",
            cs.join(", ")
        ));
    }
    if p.height_units != d.height_units {
        f.push(format!("height_units: {}", p.height_units));
    }
    for (name, v, dv) in [
        ("wall_thickness", p.wall_thickness, d.wall_thickness),
        ("cavity_corner_radius", p.cavity_corner_radius, d.cavity_corner_radius),
        ("floor_fillet", p.floor_fillet, d.floor_fillet),
    ] {
        if v != dv {
            f.push(format!("{name}: {v:?}"));
        }
    }
    if p.magnet_holes {
        f.push("magnet_holes: true".into());
    }
    if p.screw_holes {
        f.push("screw_holes: true".into());
    }
    if p.mode != d.mode {
        f.push(format!("mode: Mode::{:?}", p.mode));
    }
    if !p.divider_edges.is_empty() {
        let es: Vec<String> = p
            .divider_edges
            .iter()
            .map(|e| {
                format!(
                    "GridEdge {{ x: {}, y: {}, orientation: Orientation::{:?} }}",
                    e.x, e.y, e.orientation
                )
            })
            .collect();
        f.push(format!("divider_edges: vec![{}]", es.join(", ")));
    }
    if !p.inner_walls.is_empty() {
        let ws: Vec<String> = p
            .inner_walls
            .iter()
            .map(|w| {
                let h = match w.height {
                    Some(h) => format!("Some({h:?})"),
                    None => "None".into(),
                };
                format!(
                    "InnerWall {{ x1: {:?}, y1: {:?}, x2: {:?}, y2: {:?}, width: {:?}, height: {h} }}",
                    w.x1, w.y1, w.x2, w.y2, w.width
                )
            })
            .collect();
        f.push(format!("inner_walls: vec![{}]", ws.join(", ")));
    }
    format!("Params {{ {}, ..Params::default() }}", f.join(", "))
}


struct Finding {
    count: usize,
    repro: String,
    detail: String,
}

fn run(profile: Profile, cases: u32, seed: u64) -> String {
    let mut rng = Rng::new(seed);
    let mut found: BTreeMap<String, Finding> = BTreeMap::new();
    let mut failures = 0usize;

    for _ in 0..cases {
        let p = gen_case(&mut rng, profile);
        let Err(err) = check(&p) else { continue };
        failures += 1;
        let sig = signature(&err);
        match found.get_mut(&sig) {
            Some(f) => f.count += 1,
            None => {
                let small = shrink(&p, &sig);
                found.insert(
                    sig,
                    Finding { count: 1, repro: repro(&small), detail: err },
                );
            }
        }
    }

    if found.is_empty() {
        return String::new();
    }
    let mut out = format!(
        "{failures}/{cases} cases failed, {} distinct defect(s) (seed {seed}):\n",
        found.len()
    );
    for (i, f) in found.values().enumerate() {
        out.push_str(&format!(
            "\n[{}] x{}  {}\n     {}\n",
            i + 1,
            f.count,
            f.detail.lines().next().unwrap_or(""),
            f.repro
        ));
    }
    out
}

fn env_u64(key: &str, default: u64) -> u64 {
    std::env::var(key).ok().and_then(|v| v.parse().ok()).unwrap_or(default)
}

#[test]
fn fuzz_inner_walls() {
    let cases = env_u64("FUZZ_CASES", 150) as u32;
    let seed = env_u64("FUZZ_SEED", 0x9E37_79B9_7F4A_7C15);
    let report = run(Profile::InnerWalls, cases, seed);
    assert!(report.is_empty(), "{report}");
}

#[test]
fn fuzz_params_broad() {
    let cases = env_u64("FUZZ_CASES", 400) as u32;
    let seed = env_u64("FUZZ_SEED", 0x9E37_79B9_7F4A_7C15);
    let report = run(Profile::Broad, cases, seed);
    if report.is_empty() {
        println!("{cases} cases, all clean");
    } else {
        println!("{report}");
    }
}
