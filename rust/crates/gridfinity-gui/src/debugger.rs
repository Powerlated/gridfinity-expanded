
use eframe::egui::{self, Color32, RichText, Sense};
use glam::Vec3;
use gridfinity_cad::kernel::perf;
use gridfinity_cad::kernel::program::{Op, Program, run};
use gridfinity_cad::kernel::sketch::Seg;
use gridfinity_cad::kernel::topo::Solid;
use gridfinity_cad::{Params, gridfinity};
use std::collections::HashMap;

enum Status {
    Valid,
    Partial,
    Error(String),
}

struct Row {
    label: String,
    kind: &'static str,
    cum_faces: usize,
}

pub struct Debugger {
    show: bool,
    program: Program,
    enabled: Vec<bool>,
    rows: Vec<Row>,
    rows_key: RowsKey,
    last_status: Status,
    last_faces: usize,
    profile: bool,
    perf_rows: Vec<perf::Row>,
    perf_allocs: perf::Allocs,
    perf_wall_nanos: u64,
}

#[derive(Default)]
struct RowsKey {
    len: usize,
    labels_fingerprint: u64,
}

impl Default for Debugger {
    fn default() -> Debugger {
        Debugger {
            show: false,
            program: Program::default(),
            enabled: Vec::new(),
            rows: Vec::new(),
            rows_key: RowsKey::default(),
            last_status: Status::Valid,
            last_faces: 0,
            profile: false,
            perf_rows: Vec::new(),
            perf_allocs: perf::Allocs { count: 0, bytes: 0, peak_live_bytes: 0 },
            perf_wall_nanos: 0,
        }
    }
}

impl Debugger {
    pub fn is_shown(&self) -> bool {
        self.show
    }

    pub fn set_shown(&mut self, shown: bool) {
        self.show = shown;
    }

    pub fn refresh(&mut self, p: &Params) {
        self.program = gridfinity::program(p);
        let n = self.program.len();
        if self.enabled.len() != n {
            let old = std::mem::take(&mut self.enabled);
            self.enabled = (0..n).map(|i| old.get(i).copied().unwrap_or(true)).collect();
        }
        let new_key = fingerprint(&self.program);
        if new_key.len != self.rows_key.len || new_key.labels_fingerprint != self.rows_key.labels_fingerprint {
            self.rows.clear();
            self.rows_key = new_key;
        }
    }

    fn ensure_rows(&mut self) {
        if self.rows.len() == self.program.len() {
            return;
        }
        self.rows.clear();
        for n in 0..self.program.len() {
            let step = &self.program.steps[n];
            let faces = run(&self.program, |i| i <= n)
                .map(|s| s.faces.len())
                .unwrap_or(0);
            self.rows.push(Row {
                label: step.label.clone(),
                kind: step.op.kind(),
                cum_faces: faces,
            });
        }
    }

    /// Index of the last enabled step, `-1` when nothing is on.
    fn cursor(&self) -> i32 {
        self.enabled.iter().rposition(|&on| on).map(|i| i as i32).unwrap_or(-1)
    }

    /// Roll the construction back one step, as the panel's ◀ does. Returns
    /// whether anything moved.
    pub fn step_back(&mut self) -> bool {
        let cursor = self.cursor();
        if cursor <= 0 {
            return false;
        }
        self.enabled[cursor as usize] = false;
        true
    }

    /// Roll forward one step, as ▶ does.
    pub fn step_forward(&mut self) -> bool {
        let cursor = self.cursor();
        if cursor + 1 >= self.program.len() as i32 {
            return false;
        }
        self.enabled[(cursor + 1) as usize] = true;
        true
    }

    /// Enable or disable every step, as the panel's `all` / `none` do.
    pub fn set_all(&mut self, on: bool) {
        for e in &mut self.enabled {
            *e = on;
        }
    }

    pub fn build_solid(&mut self) -> Option<Solid> {
        if !self.show {
            return None;
        }
        let measure = self.profile;
        if measure {
            perf::reset();
            perf::set_enabled(true);
        }
        let wall = std::time::Instant::now();
        let enabled = self.enabled.clone();
        let prog = &self.program;
        let outcome = run(prog, |i| enabled.get(i).copied().unwrap_or(true));
        if measure {
            perf::set_enabled(false);
            self.perf_rows = perf::snapshot();
            self.perf_allocs = perf::allocs();
            self.perf_wall_nanos = wall.elapsed().as_nanos() as u64;
        }
        match outcome {
            Ok(solid) => {
                self.last_faces = solid.faces.len();
                self.last_status = match solid.validate() {
                    Ok(()) => Status::Valid,
                    Err(_) => Status::Partial,
                };
                Some(solid)
            }
            Err(e) => {
                self.last_status = Status::Error(e);
                self.last_faces = 0;
                None
            }
        }
    }

    pub fn sketch_planes(&self) -> Vec<(&[Seg], (Vec3, Vec3))> {
        let mut planes: HashMap<&str, (Vec3, Vec3)> = HashMap::new();
        for step in &self.program.steps {
            match &step.op {
                Op::Loft { profiles, .. } => {
                    for (name, z) in profiles {
                        planes.entry(name).or_insert((Vec3::new(0.0, 0.0, *z), Vec3::Z));
                    }
                }
                Op::Extrude { sketch, from, .. } | Op::ExtrudeCut { sketch, from, .. } => {
                    planes.entry(sketch).or_insert(from.resolve(&self.program));
                }
                _ => {}
            }
        }
        self.program
            .steps
            .iter()
            .enumerate()
            .filter(|(i, _)| self.enabled.get(*i).copied().unwrap_or(true))
            .filter_map(|(_, step)| match &step.op {
                Op::Sketch { name, profile } => {
                    let plane = planes
                        .get(name.as_str())
                        .copied()
                        .unwrap_or((Vec3::ZERO, Vec3::Z));
                    Some((profile.as_slice(), plane))
                }
                _ => None,
            })
            .collect()
    }

    pub fn panel(&mut self, ui: &mut egui::Ui) -> bool {
        self.ensure_rows();

        let mut changed = false;

        ui.heading("Construction");
        ui.add_space(2.0);
        ui.label(format!("{} operations", self.program.len()));

        ui.add_space(4.0);
        ui.horizontal(|ui| {
            let cursor = self.cursor();
            let n = self.program.len() as i32;

            if ui.button("◀").on_disabled_hover_text("No earlier op").clicked() {
                changed |= self.step_back();
            }
            if ui.button("▶").on_disabled_hover_text("Already at the end").clicked() {
                changed |= self.step_forward();
            }
            ui.separator();
            if ui.button("all").clicked() {
                self.set_all(true);
                changed = true;
            }
            if ui.button("none").clicked() {
                self.set_all(false);
                changed = true;
            }
            ui.separator();
            ui.label(format!("{}/{}", cursor + 1, n));
        });

        ui.add_space(4.0);
        let on = self.enabled.iter().filter(|&&e| e).count();
        match &self.last_status {
            Status::Valid => {
                ui.colored_label(Color32::from_rgb(0x59, 0xa1, 0x4f),
                    format!("{on} on · {} faces · manifold", self.last_faces));
            }
            Status::Partial => {
                ui.colored_label(Color32::from_rgb(0xf2, 0x8e, 0x2b),
                    format!("{on} on · {} faces · open shell (partial subset)", self.last_faces));
            }
            Status::Error(e) => {
                ui.colored_label(Color32::from_rgb(0xe1, 0x57, 0x59),
                    format!("{on} on · error: {e}"));
            }
        }

        ui.add_space(4.0);
        ui.separator();
        ui.horizontal(|ui| {
            if ui.checkbox(&mut self.profile, "Profile rebuilds").changed() {
                changed = true;
            }
            if self.profile && self.perf_wall_nanos > 0 {
                ui.label(
                    RichText::new(format!("{}", Ms(self.perf_wall_nanos)))
                        .color(Color32::from_rgb(0x8c, 0xb4, 0xd8)),
                );
            }
        });

        if self.profile {
            let a = &self.perf_allocs;
            ui.label(
                RichText::new(format!(
                    "{} allocs · {} churn · {} peak",
                    a.count,
                    Bytes(a.bytes),
                    Bytes(a.peak_live_bytes)
                ))
                .small()
                .color(Color32::from_gray(0xaa)),
            );
            if self.perf_rows.is_empty() {
                ui.label(RichText::new("no samples yet").small().italics());
            }
            let max = self.perf_rows.first().map(|r| r.nanos).unwrap_or(0).max(1);
            for r in &self.perf_rows {
                let frac = r.nanos as f32 / max as f32;
                ui.horizontal(|ui| {
                    ui.add_sized(
                        [150.0, 14.0],
                        egui::Label::new(RichText::new(short_metric(r.name)).small()).truncate(),
                    );
                    ui.add_sized(
                        [52.0, 14.0],
                        egui::Label::new(
                            RichText::new(if r.nanos > 0 {
                                format!("{}", Ms(r.nanos))
                            } else {
                                "—".into()
                            })
                            .small()
                            .monospace(),
                        ),
                    );
                    ui.add_sized(
                        [58.0, 14.0],
                        egui::Label::new(
                            RichText::new(format!("{}", Count(r.calls))).small().monospace(),
                        ),
                    );
                    ui.add_sized(
                        [56.0, 14.0],
                        egui::Label::new(
                            RichText::new(if r.alloc_bytes > 0 {
                                format!("{}", Bytes(r.alloc_bytes))
                            } else {
                                "—".into()
                            })
                            .small()
                            .monospace()
                            .color(Color32::from_rgb(0xc8, 0x9b, 0x6a)),
                        ),
                    );
                    let (rect, _) = ui.allocate_exact_size([40.0, 6.0].into(), Sense::hover());
                    ui.painter().rect_filled(rect, 1.0, Color32::from_gray(0x33));
                    let mut fill = rect;
                    fill.set_width(rect.width() * frac);
                    ui.painter().rect_filled(fill, 1.0, Color32::from_rgb(0x5b, 0x8f, 0xb9));
                });
            }
        }

        ui.add_space(4.0);
        ui.separator();

        egui::ScrollArea::vertical().show(ui, |ui| {
            for i in 0..self.rows.len() {
                let prev = if i == 0 { 0 } else { self.rows[i - 1].cum_faces };
                let delta = self.rows[i].cum_faces as i64 - prev as i64;
                let kind = self.rows[i].kind;
                let label = self.rows[i].label.clone();
                let is_on = self.enabled[i];

                let kind_color = kind_color(kind);
                let row = ui.horizontal(|ui| {
                    let resp = ui.add(egui::Checkbox::without_text(&mut self.enabled[i]));
                    if resp.clicked() {
                        changed = true;
                    }
                    ui.label(RichText::new(format!("{i:>2}")).weak().color(Color32::from_gray(0x80)));
                    ui.label(RichText::new(kind).color(kind_color).small());
                    let label_color = if is_on {
                        ui.visuals().text_color()
                    } else {
                        ui.visuals().weak_text_color()
                    };
                    ui.label(RichText::new(label).color(label_color));
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        let s = if delta >= 0 { format!("+{delta}") } else { format!("{delta}") };
                        let c = if delta > 0 {
                            Color32::from_rgb(0x59, 0xa1, 0x4f)
                        } else if delta < 0 {
                            Color32::from_rgb(0xe1, 0x57, 0x59)
                        } else {
                            ui.visuals().weak_text_color()
                        };
                        ui.label(RichText::new(format!("{s:>3} f")).small().color(c));
                    });
                });
                let sense = ui.interact(row.response.rect, ui.id().with(("row", i)), Sense::click());
                if sense.clicked() {
                    self.enabled[i] = !self.enabled[i];
                    changed = true;
                }
            }
            ui.add_space(8.0);
        });

        changed
    }
}


fn fingerprint(prog: &Program) -> RowsKey {
    use std::hash::{Hash, Hasher};
    let mut h = std::collections::hash_map::DefaultHasher::new();
    for st in &prog.steps {
        st.label.hash(&mut h);
        st.op.kind().hash(&mut h);
    }
    RowsKey {
        len: prog.len(),
        labels_fingerprint: h.finish(),
    }
}

fn kind_color(kind: &str) -> Color32 {
    match kind {
        "sketch" => Color32::from_rgb(0xed, 0xc9, 0x48),
        "plane" => Color32::from_rgb(0xb0, 0x7a, 0xa1),
        "extrude" => Color32::from_rgb(0x4e, 0x79, 0xa7),
        "cut" => Color32::from_rgb(0xe1, 0x57, 0x59),
        "loft" => Color32::from_rgb(0x4e, 0x79, 0xa7),
        "hole" => Color32::from_rgb(0xe1, 0x57, 0x59),
        "face" => Color32::from_rgb(0x76, 0xb7, 0xb2),
        "wall" => Color32::from_rgb(0x4e, 0x79, 0xa7),
        "cap" => Color32::from_rgb(0x76, 0xb7, 0xb2),
        "slabs" => Color32::from_rgb(0xf2, 0x8e, 0x2b),
        "blend" => Color32::from_rgb(0xb0, 0x7a, 0xa1),
        "fillet" => Color32::from_rgb(0xb0, 0x7a, 0xa1),
        "chamfer" => Color32::from_rgb(0xf2, 0x8e, 0x2b),
        "custom" => Color32::from_rgb(0xed, 0xc9, 0x48),
        _ => Color32::from_gray(0xb0),
    }
}

fn short_metric(name: &str) -> &str {
    name.rsplit("::").next().unwrap_or(name)
}

struct Ms(u64);

impl std::fmt::Display for Ms {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let ns = self.0 as f64;
        if ns >= 1.0e9 {
            write!(f, "{:.2}s", ns / 1.0e9)
        } else if ns >= 1.0e6 {
            write!(f, "{:.1}ms", ns / 1.0e6)
        } else if ns >= 1.0e3 {
            write!(f, "{:.0}µs", ns / 1.0e3)
        } else {
            write!(f, "{ns:.0}ns")
        }
    }
}

struct Bytes(u64);

impl std::fmt::Display for Bytes {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let b = self.0 as f64;
        if b >= 1.0e9 {
            write!(f, "{:.2} GB", b / 1.0e9)
        } else if b >= 1.0e6 {
            write!(f, "{:.1} MB", b / 1.0e6)
        } else if b >= 1.0e3 {
            write!(f, "{:.0} kB", b / 1.0e3)
        } else {
            write!(f, "{b:.0} B")
        }
    }
}

struct Count(u64);

impl std::fmt::Display for Count {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let n = self.0 as f64;
        if n >= 1.0e6 {
            write!(f, "{:.1}M", n / 1.0e6)
        } else if n >= 1.0e3 {
            write!(f, "{:.1}k", n / 1.0e3)
        } else {
            write!(f, "{}", self.0)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use gridfinity_cad::gridfinity::{BinSlope, LogicalBin, Mode, SlopeDir};
    use gridfinity_cad::layout::{internal_edges, perimeter_edges};
    use std::panic::{AssertUnwindSafe, catch_unwind};

    struct Rng(u64);

    impl Rng {
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
        fn chance(&mut self, num: u32, den: u32) -> bool {
            self.below(den) < num
        }
        fn pick<T: Copy>(&mut self, xs: &[T]) -> T {
            xs[self.below(xs.len() as u32) as usize]
        }
    }

    /// A bin small enough that a rollback sweep over it is affordable -- every
    /// prefix is a whole program run, so the sweep is quadratic in the op count
    /// and the op count is roughly linear in cells.
    fn gen_params(rng: &mut Rng) -> Params {
        let (gx, gy) = (1 + rng.below(2), 1 + rng.below(2));
        let mut cells = gridfinity::rect_cells(gx, gy);
        if cells.len() == 4 && rng.chance(1, 3) {
            cells.pop();
        }
        let slope = rng.chance(1, 6).then(|| BinSlope {
            angle_deg: rng.pick(&[5.0, 12.0, 25.0]),
            dir: rng.pick(&[SlopeDir::PlusX, SlopeDir::MinusX, SlopeDir::PlusY, SlopeDir::MinusY]),
        });
        let mut p = Params {
            bins: vec![LogicalBin { cells: cells.clone(), slope, ..Default::default() }],
            height_units: 1 + rng.below(3),
            wall_thickness: rng.pick(&[0.8, 1.2, 2.4]),
            cavity_corner_radius: rng.pick(&[0.0, 1.6, 2.5]),
            floor_fillet: rng.pick(&[0.0, 1.5, 3.0]),
            magnet_holes: rng.chance(1, 3),
            screw_holes: rng.chance(1, 4),
            // Bin only: `gridfinity::program` returns an empty program for a
            // baseplate, so there is no construction to step through.
            mode: Mode::Bin,
            ..Params::default()
        };
        for e in perimeter_edges(&cells) {
            if rng.chance(1, 6) {
                p.open_edges.push(e);
            }
        }
        for e in internal_edges(&cells) {
            if rng.chance(1, 3) {
                p.divider_edges.push(e);
            }
        }
        p
    }

    /// One rollback step's worth of the viewer's work: the subset build the
    /// panel's ◀ leaves behind, tessellated and staged exactly as `regenerate`
    /// stages it.
    ///
    /// Deliberately *not* routed through `main`'s `catch`. The app keeps that
    /// net so a user never loses the window, and a fuzzer behind it would pass
    /// on a viewer that panics on every step -- which is what this one found.
    fn view(dbg: &mut Debugger) -> Result<usize, String> {
        let solid = dbg.build_solid();
        let Some(solid) = solid else {
            // The subset failed to build. That is a state the panel reports in
            // words and the viewer survives, so it is not a crash.
            return Ok(0);
        };
        let (verts, wf) = crate::debug_view(dbg, &solid);
        Ok(verts.len() + wf.lines.len())
    }

    /// Rolling the construction back through every step must never take the
    /// viewer down.
    ///
    /// The panel's ◀ leaves a subset that is an *open shell* by design, and an
    /// open shell's mesh leaks along its boundary by construction. Feeding it
    /// to `tessellate`, whose postcondition is a watertight mesh, asserted on
    /// essentially every intermediate step: 72 leaked edges one step back from
    /// a default bin. `tessellate_shell` is the entry point that keeps the
    /// postcondition where it applies and exempts an open shell.
    #[test]
    fn rolling_back_the_construction_never_crashes_the_viewer() {
        let mut rng = Rng(0x9E37_79B9_7F4A_7C15);
        let mut failures: Vec<String> = Vec::new();
        for case in 0..60 {
            let p = gen_params(&mut rng);
            let mut dbg = Debugger::default();
            dbg.set_shown(true);
            dbg.refresh(&p);
            let steps = dbg.program.len();
            assert!(
                steps > 0,
                "case {case} generated a program with no ops to roll back:\n{}",
                p.rust_literal()
            );
            dbg.set_all(true);

            let before = failures.len();
            let mut visited = 1;
            let mut drawn = 0;
            loop {
                let at = dbg.cursor();
                match catch(|| view(&mut dbg)) {
                    Ok(payload) if at + 1 < steps as i32 && payload > 0 => drawn += 1,
                    Ok(_) => {}
                    Err(msg) => {
                        failures.push(format!(
                            "case {case}: rolling back to op {at}/{steps} of\n{}\n  {msg}",
                            p.rust_literal()
                        ));
                        break;
                    }
                }
                if !dbg.step_back() {
                    break;
                }
                visited += 1;
            }
            // The panel's `none`, which is the one rollback ◀ cannot reach: it
            // stops at the first op rather than disabling it.
            dbg.set_all(false);
            if let Err(msg) = catch(|| view(&mut dbg)) {
                failures.push(format!(
                    "case {case}: with every op off, of\n{}\n  {msg}",
                    p.rust_literal()
                ));
            }
            dbg.set_all(true);
            for _ in 0..steps {
                dbg.step_back();
            }

            // ...and forward again, which is the other half of scrubbing.
            while dbg.step_forward() {
                let at = dbg.cursor();
                if let Err(msg) = catch(|| view(&mut dbg)) {
                    failures.push(format!(
                        "case {case}: rolling forward to op {at}/{steps} of\n{}\n  {msg}",
                        p.rust_literal()
                    ));
                    break;
                }
            }
            // A crash above stops the sweep at the op that crashed, and its own
            // message is the report. Otherwise every op must have been visited:
            // a rollback that stops early would pass this vacuously.
            if failures.len() == before {
                assert_eq!(
                    visited, steps,
                    "case {case} must roll back through every one of its ops"
                );
                // ...and a subset that renders nothing is not a subset this
                // test exercised, whatever it did not crash on.
                assert!(
                    drawn > 0,
                    "case {case} never uploaded a rolled-back subset's geometry:\n{}",
                    p.rust_literal()
                );
            }
        }
        assert!(
            failures.is_empty(),
            "{} rollback step(s) crashed the viewer:\n{}",
            failures.len(),
            failures.join("\n")
        );
    }

    /// The sweep above is worthless if a rolled-back subset quietly renders
    /// nothing, so pin that a prefix of the default bin's construction reaches
    /// the viewer as geometry -- and that it is an *open* shell, which is what
    /// makes the watertight postcondition inapplicable to it.
    #[test]
    fn a_rolled_back_subset_still_reaches_the_viewer_as_geometry() {
        let p = Params::default();
        let mut dbg = Debugger::default();
        dbg.set_shown(true);
        dbg.refresh(&p);
        dbg.set_all(true);

        let whole = dbg.build_solid().expect("the default bin builds");
        assert!(whole.validate().is_ok(), "the whole bin is a closed manifold");

        let mut open_shells = 0;
        let mut drawn = 0;
        while dbg.step_back() {
            let Some(s) = dbg.build_solid() else { continue };
            if s.validate_ignoring_unused_edges().is_err() {
                open_shells += 1;
            }
            let (verts, _) = crate::debug_view(&dbg, &s);
            if !verts.is_empty() {
                drawn += 1;
            }
        }
        assert!(open_shells > 0, "rolling back must leave open shells to draw");
        assert!(drawn > 0, "a rolled-back subset must still upload triangles");
    }

    fn catch<T>(f: impl FnOnce() -> Result<T, String>) -> Result<T, String> {
        let prev = std::panic::take_hook();
        std::panic::set_hook(Box::new(|_| {}));
        let caught = catch_unwind(AssertUnwindSafe(f));
        std::panic::set_hook(prev);
        match caught {
            Ok(r) => r,
            Err(e) => Err(e
                .downcast_ref::<&str>()
                .map(|s| s.to_string())
                .or_else(|| e.downcast_ref::<String>().cloned())
                .unwrap_or_else(|| "panicked".to_string())),
        }
    }
}
