//! Native-backend diagnostics shown beside the parametric editor.
//!
//! The former panel stepped through the retired custom-kernel command stream.
//! OCCT constructs and validates finished bodies directly, so keeping that
//! command stream solely for UI would keep the old kernel alive unnecessarily.

use gridfinity_model::gridfinity::Params;

#[derive(Default)]
pub struct Debugger {
    shown: bool,
}

impl Debugger {
    pub fn refresh(&mut self, _params: &Params) {}

    pub fn is_shown(&self) -> bool {
        self.shown
    }

    pub fn set_shown(&mut self, shown: bool) {
        self.shown = shown;
    }

    pub fn panel(&mut self, ui: &mut egui::Ui) -> bool {
        ui.heading("OCCT diagnostics");
        ui.label("Geometry is built, validated, and tessellated by Open CASCADE.");
        ui.label("Build errors and triangle counts appear with the viewport.");
        false
    }
}
