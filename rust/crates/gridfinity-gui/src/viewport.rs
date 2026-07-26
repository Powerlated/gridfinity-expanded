
use eframe::glow::{self, HasContext};
use egui::Vec2 as EVec2;
use glam::{Mat4, Vec3};
use std::sync::Arc;

const LINE_WIDTH_PX: f32 = 1.6;

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

    pub fn project(&self, p: Vec3, rect: egui::Rect) -> Option<egui::Pos2> {
        let aspect = rect.width() / rect.height().max(1.0);
        let clip = self.view_proj(aspect) * p.extend(1.0);
        if clip.w <= 1e-6 {
            return None;
        }
        let ndc = clip.truncate() / clip.w;
        Some(egui::pos2(
            rect.left() + (ndc.x * 0.5 + 0.5) * rect.width(),
            rect.top() + (0.5 - ndc.y * 0.5) * rect.height(),
        ))
    }

    pub fn handle_input(&mut self, drag: EVec2, scroll: f32) {
        self.yaw -= drag.x * 0.01;
        self.pitch = (self.pitch + drag.y * 0.01).clamp(-1.54, 1.54);
        if scroll != 0.0 {
            self.distance = (self.distance * (1.0 - scroll * 0.001)).clamp(10.0, 4000.0);
        }
    }
}

pub struct Renderer {
    program: glow::Program,
    vao: glow::VertexArray,
    vbo: glow::Buffer,
    vertex_count: i32,
    line_program: glow::Program,
    line_vao: glow::VertexArray,
    line_vbo: glow::Buffer,
    line_count: i32,
}

unsafe fn link(gl: &glow::Context, vs_src: &str, fs_src: &str) -> glow::Program {
    unsafe {
        let shader_version = if cfg!(target_arch = "wasm32") {
            "#version 300 es"
        } else {
            "#version 330"
        };
        let program = gl.create_program().expect("create program");
        let shaders = [(glow::VERTEX_SHADER, vs_src), (glow::FRAGMENT_SHADER, fs_src)].map(
            |(ty, src)| {
                let s = gl.create_shader(ty).expect("create shader");
                gl.shader_source(s, &format!("{shader_version}\n{src}"));
                gl.compile_shader(s);
                assert!(gl.get_shader_compile_status(s), "shader: {}", gl.get_shader_info_log(s));
                gl.attach_shader(program, s);
                s
            },
        );
        gl.link_program(program);
        assert!(gl.get_program_link_status(program), "link: {}", gl.get_program_info_log(program));
        for s in shaders {
            gl.detach_shader(program, s);
            gl.delete_shader(s);
        }
        program
    }
}

impl Renderer {
    pub fn new(gl: &glow::Context) -> Renderer {
        unsafe {
            let vs_src = r#"
                layout (location = 0) in vec3 a_pos;
                layout (location = 1) in vec3 a_normal;
                layout (location = 2) in float a_bad;
                uniform mat4 u_mvp;
                out vec3 v_normal;
                out vec3 v_wpos;
                out float v_bad;
                void main() {
                    v_normal = a_normal;
                    v_wpos = a_pos;
                    v_bad = a_bad;
                    gl_Position = u_mvp * vec4(a_pos, 1.0);
                }
            "#;
            let fs_src = r#"
                precision mediump float;
                in vec3 v_normal;
                in vec3 v_wpos;
                in float v_bad;
                out vec4 frag;
                uniform vec3 u_light;
                uniform vec3 u_eye;
                uniform float u_time;
                void main() {
                    vec3 n = normalize(v_normal);
                    float d = max(dot(n, normalize(u_light)), 0.0);
                    if (v_bad > 0.5) {
                        // Rim term peaks where the surface turns away from the
                        // eye, so the failed bin reads as lit from inside
                        // rather than merely painted red — it stays legible
                        // against the blue even when it is the only thing on
                        // screen. The pulse is what makes it read as an alarm.
                        vec3 v = normalize(u_eye - v_wpos);
                        float rim = pow(1.0 - max(dot(n, v), 0.0), 2.5);
                        float pulse = 0.70 + 0.30 * sin(u_time * 3.2);
                        vec3 color = vec3(0.40, 0.02, 0.03) * (0.35 + 0.65 * d)
                                   + vec3(1.00, 0.12, 0.06) * rim * pulse;
                        frag = vec4(color, 1.0);
                    } else {
                        vec3 base = vec3(0.30, 0.55, 0.85);
                        frag = vec4(base * (0.28 + 0.72 * d), 1.0);
                    }
                }
            "#;

            let line_vs_src = r#"
                layout (location = 0) in vec3 a_p0;
                layout (location = 1) in vec3 a_p1;
                layout (location = 2) in vec3 a_color;
                layout (location = 3) in float a_end;
                layout (location = 4) in float a_side;
                uniform mat4 u_mvp;
                uniform vec2 u_half_vp;   // half the viewport, in pixels
                uniform float u_width;    // line width, in pixels
                out vec3 v_color;
                out float v_across;       // signed distance from centreline, px
                out float v_half;
                void main() {
                    vec4 c0 = u_mvp * vec4(a_p0, 1.0);
                    vec4 c1 = u_mvp * vec4(a_p1, 1.0);
                    vec4 c = (a_end < 0.5) ? c0 : c1;
                    // Guard the perspective divide: a vertex at or behind the
                    // eye plane would otherwise produce inf/NaN and blow the
                    // whole quad up. Such segments are clipped anyway.
                    vec2 s0 = c0.xy / max(abs(c0.w), 1e-4) * u_half_vp;
                    vec2 s1 = c1.xy / max(abs(c1.w), 1e-4) * u_half_vp;
                    vec2 d = s1 - s0;
                    float len = length(d);
                    vec2 dir = len > 1e-6 ? d / len : vec2(1.0, 0.0);
                    vec2 nrm = vec2(-dir.y, dir.x);
                    float half_w = 0.5 * u_width;
                    float ext = half_w + 1.0;   // 1px of feather either side
                    gl_Position = c;
                    gl_Position.xy += nrm * a_side * ext / u_half_vp * c.w;
                    v_color = a_color;
                    v_across = a_side * ext;
                    v_half = half_w;
                }
            "#;
            let line_fs_src = r#"
                precision mediump float;
                in vec3 v_color;
                in float v_across;
                in float v_half;
                out vec4 frag;
                uniform float u_alpha;
                void main() {
                    // Feather the last pixel with the screen-space derivative,
                    // so lines are antialiased regardless of framebuffer MSAA.
                    float d = abs(v_across);
                    float aa = fwidth(d);
                    float a = 1.0 - smoothstep(v_half - aa, v_half + aa, d);
                    if (a <= 0.0) discard;
                    frag = vec4(v_color, a * u_alpha);
                }
            "#;

            let program = link(gl, vs_src, fs_src);
            let line_program = link(gl, line_vs_src, line_fs_src);

            let vao = gl.create_vertex_array().expect("vao");
            let vbo = gl.create_buffer().expect("vbo");
            let line_vao = gl.create_vertex_array().expect("line vao");
            let line_vbo = gl.create_buffer().expect("line vbo");
            Renderer {
                program,
                vao,
                vbo,
                vertex_count: 0,
                line_program,
                line_vao,
                line_vbo,
                line_count: 0,
            }
        }
    }

    pub fn upload(&mut self, gl: &glow::Context, verts: &[f32]) {
        unsafe {
            gl.bind_vertex_array(Some(self.vao));
            gl.bind_buffer(glow::ARRAY_BUFFER, Some(self.vbo));
            gl.buffer_data_u8_slice(glow::ARRAY_BUFFER, bytemuck::cast_slice(verts), glow::DYNAMIC_DRAW);
            let f = std::mem::size_of::<f32>() as i32;
            let stride = crate::MESH_STRIDE as i32 * f;
            for (loc, size, offset) in [(0, 3, 0), (1, 3, 3), (2, 1, 6)] {
                gl.enable_vertex_attrib_array(loc);
                gl.vertex_attrib_pointer_f32(loc, size, glow::FLOAT, false, stride, offset * f);
            }
            gl.bind_vertex_array(None);
            self.vertex_count = (verts.len() / crate::MESH_STRIDE) as i32;
        }
    }

    pub fn upload_lines(&mut self, gl: &glow::Context, verts: &[f32]) {
        unsafe {
            gl.bind_vertex_array(Some(self.line_vao));
            gl.bind_buffer(glow::ARRAY_BUFFER, Some(self.line_vbo));
            gl.buffer_data_u8_slice(glow::ARRAY_BUFFER, bytemuck::cast_slice(verts), glow::DYNAMIC_DRAW);
            let f = std::mem::size_of::<f32>() as i32;
            let stride = crate::wireframe::LINE_STRIDE as i32 * f;
            for (loc, size, offset) in [(0, 3, 0), (1, 3, 3), (2, 3, 6), (3, 1, 9), (4, 1, 10)] {
                gl.enable_vertex_attrib_array(loc);
                gl.vertex_attrib_pointer_f32(loc, size, glow::FLOAT, false, stride, offset * f);
            }
            gl.bind_vertex_array(None);
            self.line_count = (verts.len() / crate::wireframe::LINE_STRIDE) as i32;
        }
    }

    pub fn paint(
        &self,
        gl: &glow::Context,
        cam: &Camera,
        aspect: f32,
        viewport_px: (f32, f32),
        time: f32,
    ) {
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

            if self.line_count > 0 {
                gl.enable(glow::POLYGON_OFFSET_FILL);
                gl.polygon_offset(1.0, 1.0);
            }

            gl.use_program(Some(self.program));
            let loc = gl.get_uniform_location(self.program, "u_mvp");
            gl.uniform_matrix_4_f32_slice(loc.as_ref(), false, &mvp.to_cols_array());
            let loc = gl.get_uniform_location(self.program, "u_light");
            gl.uniform_3_f32(loc.as_ref(), light.x, light.y, light.z);
            let eye = cam.eye();
            let loc = gl.get_uniform_location(self.program, "u_eye");
            gl.uniform_3_f32(loc.as_ref(), eye.x, eye.y, eye.z);
            let loc = gl.get_uniform_location(self.program, "u_time");
            gl.uniform_1_f32(loc.as_ref(), time);

            gl.bind_vertex_array(Some(self.vao));
            gl.draw_arrays(glow::TRIANGLES, 0, self.vertex_count);
            gl.bind_vertex_array(None);

            if self.line_count > 0 {
                gl.disable(glow::POLYGON_OFFSET_FILL);
                self.paint_lines(gl, &mvp, viewport_px);
            }

            gl.disable(glow::DEPTH_TEST);
            gl.disable(glow::CULL_FACE);
        }
    }

    unsafe fn paint_lines(&self, gl: &glow::Context, mvp: &Mat4, viewport_px: (f32, f32)) {
        unsafe {
            gl.use_program(Some(self.line_program));
            let u = |n: &str| gl.get_uniform_location(self.line_program, n);
            gl.uniform_matrix_4_f32_slice(u("u_mvp").as_ref(), false, &mvp.to_cols_array());
            gl.uniform_2_f32(u("u_half_vp").as_ref(), viewport_px.0 * 0.5, viewport_px.1 * 0.5);
            gl.uniform_1_f32(u("u_width").as_ref(), LINE_WIDTH_PX);

            gl.enable(glow::BLEND);
            gl.blend_func(glow::SRC_ALPHA, glow::ONE_MINUS_SRC_ALPHA);
            gl.depth_mask(false);
            gl.disable(glow::CULL_FACE);
            gl.bind_vertex_array(Some(self.line_vao));

            for (func, alpha) in [(glow::LEQUAL, 1.0f32), (glow::GREATER, 0.22)] {
                gl.depth_func(func);
                gl.uniform_1_f32(u("u_alpha").as_ref(), alpha);
                gl.draw_arrays(glow::TRIANGLES, 0, self.line_count);
            }

            gl.bind_vertex_array(None);
            gl.depth_func(glow::LESS);
            gl.depth_mask(true);
            gl.disable(glow::BLEND);
        }
    }

    pub fn destroy(&self, gl: &glow::Context) {
        unsafe {
            gl.delete_program(self.program);
            gl.delete_vertex_array(self.vao);
            gl.delete_buffer(self.vbo);
            gl.delete_program(self.line_program);
            gl.delete_vertex_array(self.line_vao);
            gl.delete_buffer(self.line_vbo);
        }
    }
}

pub fn callback(
    rect: egui::Rect,
    renderer: Arc<std::sync::Mutex<Renderer>>,
    cam_snapshot: Camera,
    time: f32,
) -> egui::PaintCallback {
    let cb = egui_glow::CallbackFn::new(move |info, painter| {
        let aspect = rect.width() / rect.height().max(1.0);
        let vp = info.viewport_in_pixels();
        let viewport_px = (vp.width_px as f32, vp.height_px as f32);
        renderer.lock().unwrap().paint(painter.gl(), &cam_snapshot, aspect, viewport_px, time);
    });
    egui::PaintCallback { rect, callback: Arc::new(cb) }
}
