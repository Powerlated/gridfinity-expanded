use glam::Vec3;
use glow::HasContext;
use gridfinity_render::{Camera, Renderer, append_flat_shaded, bounds_of, color_of};
use wasm_bindgen::JsCast;
use wasm_bindgen::prelude::*;
use web_sys::{HtmlCanvasElement, WebGl2RenderingContext};

#[wasm_bindgen]
pub struct Viewer {
    gl: glow::Context,
    renderer: Renderer,
    camera: Camera,
    staged: Vec<f32>,
    width: i32,
    height: i32,
    clear: Vec3,
}

#[wasm_bindgen]
impl Viewer {
    #[wasm_bindgen(constructor)]
    pub fn new(canvas: &HtmlCanvasElement, clear_rgb: u32) -> Result<Viewer, JsValue> {
        let context = canvas
            .get_context("webgl2")
            .map_err(|_| JsValue::from_str("webgl2 context request failed"))?
            .ok_or_else(|| JsValue::from_str("webgl2 is unavailable"))?
            .dyn_into::<WebGl2RenderingContext>()?;
        let gl = glow::Context::from_webgl2_context(context);
        let renderer = Renderer::new(&gl).map_err(|e| JsValue::from_str(&e))?;
        Ok(Viewer {
            gl,
            renderer,
            camera: Camera::default(),
            staged: Vec::new(),
            width: canvas.width().max(1) as i32,
            height: canvas.height().max(1) as i32,
            clear: color_of(clear_rgb),
        })
    }

    pub fn begin_scene(&mut self) {
        self.staged.clear();
    }

    pub fn add_piece(&mut self, soup: &[f32], offset_x: f32, offset_y: f32, rgb: u32) {
        append_flat_shaded(
            &mut self.staged,
            soup,
            Vec3::new(offset_x, offset_y, 0.0),
            color_of(rgb),
            false,
        );
    }

    pub fn commit_scene(&mut self, refit: bool) {
        if refit {
            match bounds_of(&self.staged) {
                Some((min, max)) => self.camera.frame(min, max),
                None => self.camera = Camera::default(),
            }
        }
        self.renderer.upload(&self.gl, &self.staged);
    }

    pub fn resize(&mut self, width: u32, height: u32) {
        self.width = width.max(1) as i32;
        self.height = height.max(1) as i32;
    }

    pub fn orbit(&mut self, dx: f32, dy: f32) {
        self.camera.orbit(dx, dy);
    }

    pub fn zoom(&mut self, delta: f32) {
        self.camera.zoom(delta);
    }

    pub fn reset_view(&mut self) {
        self.camera.reset_angles();
        if let Some((min, max)) = bounds_of(&self.staged) {
            self.camera.frame(min, max);
        } else {
            self.camera = Camera::default();
        }
    }

    pub fn frame_bounds(&mut self, min: &[f32], max: &[f32]) {
        if min.len() < 3 || max.len() < 3 {
            return;
        }
        self.camera.frame(
            Vec3::new(min[0], min[1], min[2]),
            Vec3::new(max[0], max[1], max[2]),
        );
    }

    pub fn look_down(&mut self) {
        self.camera.look_down();
    }

    pub fn render(&mut self, time: f32) {
        unsafe {
            self.gl.viewport(0, 0, self.width, self.height);
            self.gl.clear_color(self.clear.x, self.clear.y, self.clear.z, 1.0);
            self.gl.clear(glow::COLOR_BUFFER_BIT | glow::DEPTH_BUFFER_BIT);
        }
        let aspect = self.width as f32 / self.height as f32;
        let viewport_px = (self.width as f32, self.height as f32);
        self.renderer.paint(&self.gl, &self.camera, aspect, viewport_px, time);
    }

    pub fn destroy(&mut self) {
        self.renderer.destroy(&self.gl);
        self.staged = Vec::new();
    }
}
