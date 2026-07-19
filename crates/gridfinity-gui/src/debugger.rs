//! Geometry debugger panel: the Gridfinity model is a kernel [`Program`] (a
//! flat, labelled list of ops). This panel exposes that list directly — step
//! through prefixes, toggle individual ops off, and see the live 3D result.
//!
//! See `crates/gridfinity-cad/src/kernel/program.rs` for the execution model:
//! any subset of a Program runs, and partial subsets are generally **not**
//! manifold (by design — that's what makes the debugger useful).

use eframe::egui::{self, Color32, RichText, Sense};
use glam::Vec3;
use gridfinity_cad::kernel::perf;
use gridfinity_cad::kernel::program::{Op, Program, run};
use gridfinity_cad::kernel::sketch::Seg;
use gridfinity_cad::kernel::topo::Solid;
use gridfinity_cad::{Params, gridfinity};
use std::collections::HashMap;

/// The live status of the currently-displayed subset.
enum Status {
    /// `validate()` passed — the subset is a closed manifold.
    Valid,
    /// `validate()` failed, which is expected for a partial subset (a hole in
    /// the model). Not reported as an error.
    Partial,
    /// The subset failed to even `run` (an op threw, e.g. a blend on a missing
    /// edge). That's a real error worth surfacing.
    Error(String),
}

/// One entry in the per-op list, cached so the panel doesn't recompute prefix
/// face counts every frame.
struct Row {
    label: String,
    kind: &'static str,
    /// Face count after running the prefix `0..=i`.
    cum_faces: usize,
}

pub struct Debugger {
    show: bool,
    program: Program,
    enabled: Vec<bool>,
    rows: Vec<Row>,
    /// Cache key — recomputed when the program's length or any label changes.
    rows_key: RowsKey,
    last_status: Status,
    last_faces: usize,
    /// Measure the next rebuild. Off by default: instrumenting costs a little
    /// on hot paths, and the numbers are only wanted when someone is looking.
    profile: bool,
    perf_rows: Vec<perf::Row>,
    perf_allocs: perf::Allocs,
    perf_wall_nanos: u64,
}

#[derive(Default)]
struct RowsKey {
    len: usize,
    /// A cheap hash of labels, so a rename forces a recompute.
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
        // The caller already flips its local `changed`, which marks the App
        // dirty — nothing else to do here. Kept as a named method so the call
        // site reads clearly.
    }

    /// Replace the cached program. The enabled mask is preserved at its current
    /// length and extended with `true` for any new ops, so a param change that
    /// grows/shrinks the program doesn't lose the user's mask.
    pub fn refresh(&mut self, p: &Params) {
        self.program = gridfinity::program(p);
        let n = self.program.len();
        if self.enabled.len() != n {
            let old = std::mem::take(&mut self.enabled);
            self.enabled = (0..n).map(|i| old.get(i).copied().unwrap_or(true)).collect();
        }
        // The rows cache is stale until proven otherwise.
        let new_key = fingerprint(&self.program);
        if new_key.len != self.rows_key.len || new_key.labels_fingerprint != self.rows_key.labels_fingerprint {
            self.rows.clear();
            self.rows_key = new_key;
        }
    }

    /// Recompute the per-op prefix face counts lazily and only when something
    /// visible changed. Each prefix is run with the others masked off, which is
    /// the same cost as building the whole thing once.
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

    /// Build the solid for the currently-enabled subset. Caller owns the
    /// decision of whether to use this (debug on) or fall back to the full
    /// `gridfinity::build`.
    pub fn build_solid(&mut self) -> Option<Solid> {
        if !self.show {
            return None;
        }
        // Measure this rebuild if the user asked for it. Instrumentation is
        // enabled only for the duration, so the numbers describe one build and
        // idle frames pay nothing.
        let measure = self.profile;
        if measure {
            perf::reset();
            perf::set_enabled(true);
        }
        let wall = std::time::Instant::now();
        // Borrow the program immutably while running; the mask is read by
        // closure over `self.enabled`.
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

    /// The enabled `Op::Sketch` profiles, each paired with the plane it is used
    /// at, for the wireframe overlay.
    ///
    /// A sketch carries no plane of its own — it is a bare 2D profile, and only
    /// the op that *consumes* it says where it lives. So this resolves the plane
    /// from the consumer, and deliberately does so whether or not that consumer
    /// is itself enabled: toggling a loft off should not leave its sketches with
    /// nowhere to draw. A sketch no op references falls back to z=0.
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

    /// Render the right-hand panel. Returns true if anything changed that
    /// should trigger a viewport rebuild.
    pub fn panel(&mut self, ui: &mut egui::Ui) -> bool {
        self.ensure_rows();

        let mut changed = false;

        ui.heading("Construction");
        ui.add_space(2.0);
        ui.label(format!("{} operations", self.program.len()));

        // ── Toolbar: prefix controls ──────────────────────────────────────
        ui.add_space(4.0);
        ui.horizontal(|ui| {
            // Cursor = the highest enabled prefix index. -1 means none.
            let cursor = self
                .enabled
                .iter()
                .rposition(|&on| on)
                .map(|i| i as i32)
                .unwrap_or(-1);
            let n = self.program.len() as i32;

            if ui.button("◀").on_disabled_hover_text("No earlier op").clicked()
                && cursor > 0
            {
                // Drop the last enabled op.
                self.enabled[cursor as usize] = false;
                changed = true;
            }
            if ui.button("▶").on_disabled_hover_text("Already at the end").clicked()
                && cursor + 1 < n
            {
                self.enabled[(cursor + 1) as usize] = true;
                changed = true;
            }
            ui.separator();
            if ui.button("all").clicked() {
                for e in &mut self.enabled {
                    *e = true;
                }
                changed = true;
            }
            if ui.button("none").clicked() {
                for e in &mut self.enabled {
                    *e = false;
                }
                changed = true;
            }
            ui.separator();
            ui.label(format!("{}/{}", cursor + 1, n));
        });

        // ── Status line ───────────────────────────────────────────────────
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

        // ── Performance ───────────────────────────────────────────────────
        ui.add_space(4.0);
        ui.separator();
        ui.horizontal(|ui| {
            if ui.checkbox(&mut self.profile, "Profile rebuilds").changed() {
                // Rebuild now so the numbers appear immediately rather than on
                // whatever the user happens to touch next.
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
            // Timings nest (a region boolean contains its seg/seg solves), so
            // these do not sum to the wall time and the bar is scaled to the
            // heaviest row rather than to the total.
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

        // ── Op list ───────────────────────────────────────────────────────
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
                // Click anywhere else on the row toggles too.
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

/// `region2d::split_regions` → `split_regions`. The module prefix is useful in
/// the source and just eats column width in a 150 px label.
fn short_metric(name: &str) -> &str {
    name.rsplit("::").next().unwrap_or(name)
}

/// Nanoseconds, at a fixed three significant figures so the column stays
/// readable as values range over six orders of magnitude.
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

/// Call counts reach the millions; `1.2M` reads better than `1234567`.
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
