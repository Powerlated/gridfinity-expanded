use egui::Vec2 as EVec2;
use egui_wgpu::{CallbackTrait, ScreenDescriptor};
use glam::Vec3;
use std::sync::{Arc, Mutex};

pub use gridfinity_render::{Camera, Quality, Renderer, Viewport};

/// The most physical pixels the 3D view is rendered per logical point, however
/// dense the display is. A HiDPI screen reports two or three, and rendering the
/// scene at every one of them costs four or nine times a 1x display's pixels for
/// a preview the user judges shape and fit in.
const MAX_RENDER_PIXELS_PER_POINT: f32 = 1.5;

/// The fraction of a viewport's physical pixels the scene is rendered at, given
/// the display's `pixels_per_point`: 1.0 up to `MAX_RENDER_PIXELS_PER_POINT`,
/// and below it thereafter, so the rendered image is that many pixels per point
/// whatever the display does. The blit stretches it back over the whole
/// rectangle, so only the 3D view softens -- the panels and their text are drawn
/// by egui at the display's own density.
fn render_scale(pixels_per_point: f32) -> f32 {
    assert!(
        pixels_per_point.is_finite() && pixels_per_point > 0.0,
        "a display reports how many physical pixels a point is worth, which is positive, not {pixels_per_point}"
    );
    (MAX_RENDER_PIXELS_PER_POINT / pixels_per_point).min(1.0)
}

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

#[derive(Clone)]
pub struct Gpu {
    pub device: wgpu::Device,
    pub queue: wgpu::Queue,
    pub format: wgpu::TextureFormat,
}

struct ViewportCallback {
    gpu: Gpu,
    renderer: Arc<Mutex<Renderer>>,
    rect: egui::Rect,
    camera: Camera,
    time: f32,
}

impl CallbackTrait for ViewportCallback {
    fn prepare(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        screen: &ScreenDescriptor,
        _encoder: &mut wgpu::CommandEncoder,
        _resources: &mut egui_wgpu::CallbackResources,
    ) -> Vec<wgpu::CommandBuffer> {
        let scale = screen.pixels_per_point;
        let viewport = Viewport::new(
            (self.rect.left() * scale).round() as i32,
            (self.rect.top() * scale).round() as i32,
            (self.rect.width() * scale).round() as i32,
            (self.rect.height() * scale).round() as i32,
        );
        let mut renderer = self.renderer.lock().unwrap();
        renderer.set_render_scale(render_scale(scale));
        vec![renderer.prepare(device, queue, &self.camera, viewport, self.time)]
    }

    fn paint(
        &self,
        _info: egui::PaintCallbackInfo,
        pass: &mut wgpu::RenderPass<'static>,
        _resources: &egui_wgpu::CallbackResources,
    ) {
        self.renderer.lock().unwrap().blit(
            &self.gpu.device,
            &self.gpu.queue,
            pass,
            self.gpu.format,
        );
    }
}

pub fn callback(
    rect: egui::Rect,
    gpu: Gpu,
    renderer: Arc<Mutex<Renderer>>,
    cam_snapshot: Camera,
    time: f32,
) -> egui::PaintCallback {
    egui_wgpu::Callback::new_paint_callback(
        rect,
        ViewportCallback { gpu, renderer, rect, camera: cam_snapshot, time },
    )
}

#[cfg(test)]
mod tests {
    use super::{MAX_RENDER_PIXELS_PER_POINT, render_scale};

    #[test]
    fn a_standard_display_is_rendered_at_every_pixel_it_has() {
        assert_eq!(render_scale(1.0), 1.0);
        assert_eq!(render_scale(MAX_RENDER_PIXELS_PER_POINT), 1.0);
    }

    #[test]
    fn a_hidpi_display_is_rendered_at_the_ceiling_however_dense_it_is() {
        for ppp in [1.75, 2.0, 2.5, 3.0, 4.0] {
            let rendered = render_scale(ppp) * ppp;
            assert!(
                (rendered - MAX_RENDER_PIXELS_PER_POINT).abs() < 1e-6,
                "a {ppp}x display renders {rendered} pixels per point, not {MAX_RENDER_PIXELS_PER_POINT}"
            );
            assert!(render_scale(ppp) > 0.0 && render_scale(ppp) < 1.0, "{ppp}");
        }
    }
}
