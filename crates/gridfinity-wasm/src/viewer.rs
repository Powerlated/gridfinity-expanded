use glam::Vec3;
use gridfinity_render::{
    Camera, FLAG_CUT, KERNEL_STRIDE, Quality, Renderer, VERTEX_STRIDE, Viewport,
    append_smooth_shaded, bounds_of, color_of,
};
use wasm_bindgen::prelude::*;
use web_sys::HtmlCanvasElement;

#[wasm_bindgen]
pub struct Viewer {
    surface: wgpu::Surface<'static>,
    device: wgpu::Device,
    queue: wgpu::Queue,
    format: wgpu::TextureFormat,
    renderer: Renderer,
    camera: Camera,
    staged: Vec<f32>,
    displaced: Vec<f32>,
    spans: Vec<PieceSpan>,
    explode: f32,
    width: u32,
    height: u32,
    clear: Vec3,
}

#[wasm_bindgen]
pub async fn create_viewer(
    canvas: HtmlCanvasElement,
    clear_rgb: u32,
) -> Result<Viewer, JsValue> {
    let width = canvas.width().max(1);
    let height = canvas.height().max(1);
    let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
        backends: wgpu::Backends::BROWSER_WEBGPU | wgpu::Backends::GL,
        flags: wgpu::InstanceFlags::default(),
        memory_budget_thresholds: wgpu::MemoryBudgetThresholds::default(),
        backend_options: wgpu::BackendOptions::default(),
        display: None,
    });
    let surface = instance
        .create_surface(wgpu::SurfaceTarget::Canvas(canvas))
        .map_err(|e| JsValue::from_str(&format!("no drawing surface for the canvas: {e}")))?;
    let adapter = instance
        .request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::HighPerformance,
            compatible_surface: Some(&surface),
            force_fallback_adapter: false,
        })
        .await
        .map_err(|e| JsValue::from_str(&format!("no webgpu or webgl2 adapter: {e}")))?;
    let (device, queue) = adapter
        .request_device(&wgpu::DeviceDescriptor {
            label: Some("gridfinity"),
            required_features: wgpu::Features::empty(),
            required_limits: wgpu::Limits::downlevel_webgl2_defaults()
                .using_resolution(adapter.limits()),
            experimental_features: wgpu::ExperimentalFeatures::disabled(),
            memory_hints: wgpu::MemoryHints::default(),
            trace: wgpu::Trace::Off,
        })
        .await
        .map_err(|e| JsValue::from_str(&format!("the adapter refused a device: {e}")))?;

    let capabilities = surface.get_capabilities(&adapter);
    let format = capabilities
        .formats
        .iter()
        .copied()
        .find(|f| !f.is_srgb())
        .or_else(|| capabilities.formats.first().copied())
        .ok_or_else(|| JsValue::from_str("the surface offers no texture format"))?;
    let renderer = Renderer::new(&device, &adapter).map_err(|e| JsValue::from_str(&e))?;

    let mut viewer = Viewer {
        surface,
        device,
        queue,
        format,
        renderer,
        camera: Camera::default(),
        staged: Vec::new(),
        displaced: Vec::new(),
        spans: Vec::new(),
        explode: 0.0,
        width,
        height,
        clear: color_of(clear_rgb),
    };
    viewer.configure();
    Ok(viewer)
}

#[wasm_bindgen]
impl Viewer {
    pub fn begin_scene(&mut self) {
        self.staged.clear();
        self.spans.clear();
    }

    pub fn add_piece(
        &mut self,
        vertices: &[f32],
        apart_x: f32,
        apart_y: f32,
        rgb: u32,
        cuts: &[f32],
    ) {
        let start = self.staged.len();
        let color = color_of(rgb);
        for tri in vertices.chunks_exact(3 * KERNEL_STRIDE) {
            let severed =
                tri.chunks_exact(KERNEL_STRIDE).all(|v| on_a_cut(cuts, v[0], v[1]));
            let from = self.staged.len();
            append_smooth_shaded(&mut self.staged, tri, Vec3::ZERO, color, false);
            if severed {
                for v in self.staged[from..].chunks_exact_mut(VERTEX_STRIDE) {
                    v[9] = FLAG_CUT;
                }
            }
        }
        self.spans.push(PieceSpan {
            start,
            end: self.staged.len(),
            apart: Vec3::new(apart_x, apart_y, 0.0),
        });
    }

    pub fn upload_vertices(&mut self, verts: &[f32]) {
        self.staged.clear();
        self.spans.clear();
        self.staged.extend_from_slice(verts);
        self.lay_out();
        self.renderer.upload(&self.device, &self.queue, &self.displaced);
    }

    pub fn commit_scene(&mut self, refit: bool) {
        self.lay_out();
        if refit {
            match bounds_of(&self.displaced) {
                Some((min, max)) => self.camera.frame(min, max),
                None => self.camera = Camera::default(),
            }
        }
        self.renderer.upload(&self.device, &self.queue, &self.displaced);
    }

    pub fn set_explode(&mut self, distance: f32) {
        if (distance - self.explode).abs() < 1e-4 {
            return;
        }
        self.explode = distance;
        self.lay_out();
        self.renderer.upload(&self.device, &self.queue, &self.displaced);
    }

    pub fn explode(&self) -> f32 {
        self.explode
    }

    pub fn resize(&mut self, width: u32, height: u32) {
        self.width = width.max(1);
        self.height = height.max(1);
        self.configure();
    }

    pub fn orbit(&mut self, dx: f32, dy: f32) {
        self.camera.orbit(dx, dy);
    }

    pub fn zoom(&mut self, delta: f32) {
        self.camera.zoom(delta);
    }

    pub fn reset_view(&mut self) {
        self.camera.reset_angles();
        if let Some((min, max)) = bounds_of(&self.displaced) {
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

    pub fn set_quality(&mut self, level: i32) {
        self.renderer.set_quality(Quality::from_index(level.max(0) as u32));
    }

    pub fn quality(&self) -> i32 {
        self.renderer.quality().index() as i32
    }

    pub fn render(&mut self, time: f32) {
        let frame = match self.surface.get_current_texture() {
            wgpu::CurrentSurfaceTexture::Success(frame)
            | wgpu::CurrentSurfaceTexture::Suboptimal(frame) => frame,
            wgpu::CurrentSurfaceTexture::Outdated | wgpu::CurrentSurfaceTexture::Lost => {
                self.configure();
                return;
            }
            _ => return,
        };
        let view = frame.texture.create_view(&wgpu::TextureViewDescriptor::default());
        let viewport = Viewport::new(0, 0, self.width as i32, self.height as i32);
        let offscreen =
            self.renderer.prepare(&self.device, &self.queue, &self.camera, viewport, time);

        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("present") });
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("present"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: self.clear.x as f64,
                            g: self.clear.y as f64,
                            b: self.clear.z as f64,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            self.renderer.blit(&self.device, &self.queue, &mut pass, self.format);
        }
        self.queue.submit([offscreen, encoder.finish()]);
        frame.present();
    }

    pub fn destroy(&mut self) {
        self.renderer.destroy();
        self.staged = Vec::new();
        self.displaced = Vec::new();
        self.spans = Vec::new();
    }
}

struct PieceSpan {
    start: usize,
    end: usize,
    apart: Vec3,
}

const CUT_TOLERANCE: f32 = 0.01;

fn on_a_cut(cuts: &[f32], x: f32, y: f32) -> bool {
    cuts.chunks_exact(4).any(|c| {
        let (along, across) = if c[0] < 0.5 { (x, y) } else { (y, x) };
        (along - c[1]).abs() <= CUT_TOLERANCE
            && across >= c[2] - CUT_TOLERANCE
            && across <= c[3] + CUT_TOLERANCE
    })
}

impl Viewer {
    fn lay_out(&mut self) {
        self.displaced.clear();
        self.displaced.extend_from_slice(&self.staged);
        if self.explode == 0.0 {
            return;
        }
        for span in &self.spans {
            let shift = span.apart * self.explode;
            for v in self.displaced[span.start..span.end].chunks_exact_mut(VERTEX_STRIDE) {
                v[0] += shift.x;
                v[1] += shift.y;
                v[2] += shift.z;
            }
        }
    }

    fn configure(&mut self) {
        self.surface.configure(&self.device, &wgpu::SurfaceConfiguration {
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            format: self.format,
            width: self.width,
            height: self.height,
            present_mode: wgpu::PresentMode::Fifo,
            desired_maximum_frame_latency: 2,
            alpha_mode: wgpu::CompositeAlphaMode::Opaque,
            view_formats: vec![],
        });
    }
}
