use egui::Vec2 as EVec2;
use egui_wgpu::{CallbackTrait, ScreenDescriptor};
use glam::Vec3;
use std::sync::{Arc, Mutex};

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
