//! Geometry debugger panel: the Gridfinity model is a kernel [`Program`] (a
//! flat, labelled list of ops). This panel exposes that list directly — step
//! through prefixes, toggle individual ops off, and see the live 3D result.
//!
//! See `crates/gridfinity-cad/src/kernel/program.rs` for the execution model:
//! any subset of a Program runs, and partial subsets are generally **not**
//! manifold (by design — that's what makes the debugger useful).

use eframe::egui::{self, Color32, RichText, Sense};
use gridfinity_cad::kernel::program::{Program, run};
use gridfinity_cad::kernel::topo::Solid;
use gridfinity_cad::{Params, gridfinity};

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
        // Borrow the program immutably while running; the mask is read by
        // closure over `self.enabled`.
        let enabled = self.enabled.clone();
        let prog = &self.program;
        match run(prog, |i| enabled.get(i).copied().unwrap_or(true)) {
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
        "custom" => Color32::from_rgb(0xed, 0xc9, 0x48),
        _ => Color32::from_gray(0xb0),
    }
}
