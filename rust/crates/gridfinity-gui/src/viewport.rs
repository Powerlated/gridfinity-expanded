use egui::Vec2 as EVec2;
use glam::Vec3;
use std::sync::Arc;

pub use gridfinity_render::{Camera, Quality, Renderer, Viewport};

pub trait CameraExt {
    fn project(&self, p: Vec3, rect: egui::Rect) -> Option<egui::Pos2>;
    fn handle_input(&mut self, drag: EVec2, scroll: f32);
}

impl CameraExt for Camera {
    fn project(&self, p: Vec3, rect: egui::Rect) -> Option<egui::Pos2> {
        self.project_to_viewport(p, rect.width(), rect.height())
            .map(|(x, y)| egui::pos2(rect.left() + x, rect.top() + y))
    }

    fn handle_input(&mut self, drag: EVec2, scroll: f32) {
        self.orbit(drag.x, drag.y);
        self.zoom(scroll);
    }
}

pub fn callback(
    rect: egui::Rect,
    renderer: Arc<std::sync::Mutex<Renderer>>,
    cam_snapshot: Camera,
    time: f32,
) -> egui::PaintCallback {
    let cb = egui_glow::CallbackFn::new(move |info, painter| {
        let vp = info.viewport_in_pixels();
        let viewport = Viewport::new(vp.left_px, vp.from_bottom_px, vp.width_px, vp.height_px);
        renderer.lock().unwrap().paint(painter.gl(), &cam_snapshot, viewport, time);
    });
    egui::PaintCallback { rect, callback: Arc::new(cb) }
}
