//! Custom glow renderer for the 3D preview: one shader, one interleaved
//! position+normal buffer, an orbit camera, and a depth-tested paint callback
//! that draws inside the egui `CentralPanel` and restores GL state afterward.

use eframe::glow::{self, HasContext};
use egui::Vec2 as EVec2;
use glam::{Mat4, Vec3};
use std::sync::Arc;

/// Orbit camera state (persisted across frames).
#[derive(Clone, Copy)]
pub struct Camera {
    pub yaw: f32,
    pub pitch: f32,
    pub distance: f32,
    pub target: Vec3,
}

impl Default for Camera {
    fn default() -> Camera {
        Camera {
            yaw: 0.9,
            pitch: 0.5,
            distance: 200.0,
            target: Vec3::ZERO,
        }
    }
}

impl Camera {
    /// Frame the camera around an axis-aligned bounding box.
    pub fn frame(&mut self, min: Vec3, max: Vec3) {
        self.target = (min + max) * 0.5;
        let radius = (max - min).length() * 0.5;
        self.distance = (radius / (22.5f32.to_radians()).tan()).max(20.0);
    }

    fn eye(&self) -> Vec3 {
        let (sp, cp) = self.pitch.sin_cos();
        let (sy, cy) = self.yaw.sin_cos();
        self.target + self.distance * Vec3::new(cp * cy, cp * sy, sp)
    }

    fn view_proj(&self, aspect: f32) -> Mat4 {
        let proj = Mat4::perspective_rh_gl(45f32.to_radians(), aspect.max(0.01), 1.0, 5000.0);
        let view = Mat4::look_at_rh(self.eye(), self.target, Vec3::Z);
        proj * view
    }

    /// Apply drag (orbit) and scroll (zoom) from an egui response.
    pub fn handle_input(&mut self, drag: EVec2, scroll: f32) {
        self.yaw -= drag.x * 0.01;
        self.pitch = (self.pitch + drag.y * 0.01).clamp(-1.54, 1.54);
        if scroll != 0.0 {
            self.distance = (self.distance * (1.0 - scroll * 0.001)).clamp(10.0, 4000.0);
        }
    }
}

/// GPU resources for drawing one mesh.
pub struct Renderer {
    program: glow::Program,
    vao: glow::VertexArray,
    vbo: glow::Buffer,
    vertex_count: i32,
}

impl Renderer {
    pub fn new(gl: &glow::Context) -> Renderer {
        unsafe {
            let shader_version = if cfg!(target_arch = "wasm32") {
                "#version 300 es"
            } else {
                "#version 330"
            };
            let program = gl.create_program().expect("create program");

            let vs_src = r#"
                layout (location = 0) in vec3 a_pos;
                layout (location = 1) in vec3 a_normal;
                uniform mat4 u_mvp;
                out vec3 v_normal;
                void main() {
                    v_normal = a_normal;
                    gl_Position = u_mvp * vec4(a_pos, 1.0);
                }
            "#;
            let fs_src = r#"
                precision mediump float;
                in vec3 v_normal;
                out vec4 frag;
                uniform vec3 u_light;
                void main() {
                    vec3 n = normalize(v_normal);
                    float d = max(dot(n, normalize(u_light)), 0.0);
                    vec3 base = vec3(0.30, 0.55, 0.85);
                    vec3 color = base * (0.28 + 0.72 * d);
                    frag = vec4(color, 1.0);
                }
            "#;

            let shaders = [
                (glow::VERTEX_SHADER, vs_src),
                (glow::FRAGMENT_SHADER, fs_src),
            ]
            .map(|(ty, src)| {
                let s = gl.create_shader(ty).expect("create shader");
                gl.shader_source(s, &format!("{shader_version}\n{src}"));
                gl.compile_shader(s);
                assert!(gl.get_shader_compile_status(s), "shader: {}", gl.get_shader_info_log(s));
                gl.attach_shader(program, s);
                s
            });
            gl.link_program(program);
            assert!(gl.get_program_link_status(program), "link: {}", gl.get_program_info_log(program));
            for s in shaders {
                gl.detach_shader(program, s);
                gl.delete_shader(s);
            }

            let vao = gl.create_vertex_array().expect("vao");
            let vbo = gl.create_buffer().expect("vbo");
            Renderer { program, vao, vbo, vertex_count: 0 }
        }
    }

    /// Upload an interleaved `[pos(3), normal(3)]` vertex buffer.
    pub fn upload(&mut self, gl: &glow::Context, verts: &[f32]) {
        unsafe {
            gl.bind_vertex_array(Some(self.vao));
            gl.bind_buffer(glow::ARRAY_BUFFER, Some(self.vbo));
            gl.buffer_data_u8_slice(glow::ARRAY_BUFFER, bytemuck::cast_slice(verts), glow::DYNAMIC_DRAW);
            let stride = 6 * std::mem::size_of::<f32>() as i32;
            gl.enable_vertex_attrib_array(0);
            gl.vertex_attrib_pointer_f32(0, 3, glow::FLOAT, false, stride, 0);
            gl.enable_vertex_attrib_array(1);
            gl.vertex_attrib_pointer_f32(1, 3, glow::FLOAT, false, stride, 3 * std::mem::size_of::<f32>() as i32);
            gl.bind_vertex_array(None);
            self.vertex_count = (verts.len() / 6) as i32;
        }
    }

    pub fn paint(&self, gl: &glow::Context, cam: &Camera, aspect: f32) {
        if self.vertex_count == 0 {
            return;
        }
        let mvp = cam.view_proj(aspect);
        let light = (cam.eye() - cam.target).normalize() + Vec3::new(0.3, 0.2, 0.6);
        unsafe {
            gl.enable(glow::DEPTH_TEST);
            gl.depth_mask(true);
            gl.clear(glow::DEPTH_BUFFER_BIT);
            gl.enable(glow::CULL_FACE);
            gl.cull_face(glow::BACK);

            gl.use_program(Some(self.program));
            let loc = gl.get_uniform_location(self.program, "u_mvp");
            gl.uniform_matrix_4_f32_slice(loc.as_ref(), false, &mvp.to_cols_array());
            let loc = gl.get_uniform_location(self.program, "u_light");
            gl.uniform_3_f32(loc.as_ref(), light.x, light.y, light.z);

            gl.bind_vertex_array(Some(self.vao));
            gl.draw_arrays(glow::TRIANGLES, 0, self.vertex_count);
            gl.bind_vertex_array(None);

            // Restore state egui expects.
            gl.disable(glow::DEPTH_TEST);
            gl.disable(glow::CULL_FACE);
        }
    }

    pub fn destroy(&self, gl: &glow::Context) {
        unsafe {
            gl.delete_program(self.program);
            gl.delete_vertex_array(self.vao);
            gl.delete_buffer(self.vbo);
        }
    }
}

/// A paint callback that draws the shared renderer for one frame.
pub fn callback(
    rect: egui::Rect,
    renderer: Arc<std::sync::Mutex<Renderer>>,
    cam_snapshot: Camera,
) -> egui::PaintCallback {
    let cb = egui_glow::CallbackFn::new(move |_info, painter| {
        let aspect = rect.width() / rect.height().max(1.0);
        renderer.lock().unwrap().paint(painter.gl(), &cam_snapshot, aspect);
    });
    egui::PaintCallback { rect, callback: Arc::new(cb) }
}
