use crate::camera::Camera;
use crate::vertex::{LINE_STRIDE, VERTEX_STRIDE};
use glam::{Mat4, Vec3};
use glow::HasContext;

const LINE_WIDTH_PX: f32 = 1.6;

const MESH_VS: &str = r#"
    layout (location = 0) in vec3 a_pos;
    layout (location = 1) in vec3 a_normal;
    layout (location = 2) in vec3 a_color;
    layout (location = 3) in float a_bad;
    uniform mat4 u_mvp;
    out vec3 v_normal;
    out vec3 v_wpos;
    out vec3 v_color;
    out float v_bad;
    void main() {
        v_normal = a_normal;
        v_wpos = a_pos;
        v_color = a_color;
        v_bad = a_bad;
        gl_Position = u_mvp * vec4(a_pos, 1.0);
    }
"#;

const MESH_FS: &str = r#"
    precision mediump float;
    in vec3 v_normal;
    in vec3 v_wpos;
    in vec3 v_color;
    in float v_bad;
    out vec4 frag;
    uniform vec3 u_light;
    uniform vec3 u_eye;
    uniform float u_time;
    void main() {
        vec3 n = normalize(v_normal);
        float d = max(dot(n, normalize(u_light)), 0.0);
        if (v_bad > 0.5) {
            vec3 v = normalize(u_eye - v_wpos);
            float rim = pow(1.0 - max(dot(n, v), 0.0), 2.5);
            float pulse = 0.70 + 0.30 * sin(u_time * 3.2);
            vec3 color = vec3(0.40, 0.02, 0.03) * (0.35 + 0.65 * d)
                       + vec3(1.00, 0.12, 0.06) * rim * pulse;
            frag = vec4(color, 1.0);
        } else {
            frag = vec4(v_color * (0.28 + 0.72 * d), 1.0);
        }
    }
"#;

const LINE_VS: &str = r#"
    layout (location = 0) in vec3 a_p0;
    layout (location = 1) in vec3 a_p1;
    layout (location = 2) in vec3 a_color;
    layout (location = 3) in float a_end;
    layout (location = 4) in float a_side;
    uniform mat4 u_mvp;
    uniform vec2 u_half_vp;
    uniform float u_width;
    out vec3 v_color;
    out float v_across;
    out float v_half;
    void main() {
        vec4 c0 = u_mvp * vec4(a_p0, 1.0);
        vec4 c1 = u_mvp * vec4(a_p1, 1.0);
        vec4 c = (a_end < 0.5) ? c0 : c1;
        vec2 s0 = c0.xy / max(abs(c0.w), 1e-4) * u_half_vp;
        vec2 s1 = c1.xy / max(abs(c1.w), 1e-4) * u_half_vp;
        vec2 d = s1 - s0;
        float len = length(d);
        vec2 dir = len > 1e-6 ? d / len : vec2(1.0, 0.0);
        vec2 nrm = vec2(-dir.y, dir.x);
        float half_w = 0.5 * u_width;
        float ext = half_w + 1.0;
        gl_Position = c;
        gl_Position.xy += nrm * a_side * ext / u_half_vp * c.w;
        v_color = a_color;
        v_across = a_side * ext;
        v_half = half_w;
    }
"#;

const LINE_FS: &str = r#"
    precision mediump float;
    in vec3 v_color;
    in float v_across;
    in float v_half;
    out vec4 frag;
    uniform float u_alpha;
    void main() {
        float d = abs(v_across);
        float aa = fwidth(d);
        float a = 1.0 - smoothstep(v_half - aa, v_half + aa, d);
        if (a <= 0.0) discard;
        frag = vec4(v_color, a * u_alpha);
    }
"#;

unsafe fn link(gl: &glow::Context, vs_src: &str, fs_src: &str) -> Result<glow::Program, String> {
    unsafe {
        let version = if cfg!(target_arch = "wasm32") { "#version 300 es" } else { "#version 330" };
        let program = gl.create_program()?;
        let mut shaders = Vec::new();
        for (kind, src) in [(glow::VERTEX_SHADER, vs_src), (glow::FRAGMENT_SHADER, fs_src)] {
            let shader = gl.create_shader(kind)?;
            gl.shader_source(shader, &format!("{version}\n{src}"));
            gl.compile_shader(shader);
            if !gl.get_shader_compile_status(shader) {
                return Err(format!("shader: {}", gl.get_shader_info_log(shader)));
            }
            gl.attach_shader(program, shader);
            shaders.push(shader);
        }
        gl.link_program(program);
        if !gl.get_program_link_status(program) {
            return Err(format!("link: {}", gl.get_program_info_log(program)));
        }
        for shader in shaders {
            gl.detach_shader(program, shader);
            gl.delete_shader(shader);
        }
        Ok(program)
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

impl Renderer {
    pub fn new(gl: &glow::Context) -> Result<Renderer, String> {
        unsafe {
            let program = link(gl, MESH_VS, MESH_FS)?;
            let line_program = link(gl, LINE_VS, LINE_FS)?;
            Ok(Renderer {
                program,
                vao: gl.create_vertex_array()?,
                vbo: gl.create_buffer()?,
                vertex_count: 0,
                line_program,
                line_vao: gl.create_vertex_array()?,
                line_vbo: gl.create_buffer()?,
                line_count: 0,
            })
        }
    }

    pub fn upload(&mut self, gl: &glow::Context, verts: &[f32]) {
        unsafe {
            bind_attributes(gl, self.vao, self.vbo, verts, VERTEX_STRIDE, &[
                (0, 3, 0),
                (1, 3, 3),
                (2, 3, 6),
                (3, 1, 9),
            ]);
        }
        self.vertex_count = (verts.len() / VERTEX_STRIDE) as i32;
    }

    pub fn upload_lines(&mut self, gl: &glow::Context, verts: &[f32]) {
        unsafe {
            bind_attributes(gl, self.line_vao, self.line_vbo, verts, LINE_STRIDE, &[
                (0, 3, 0),
                (1, 3, 3),
                (2, 3, 6),
                (3, 1, 9),
                (4, 1, 10),
            ]);
        }
        self.line_count = (verts.len() / LINE_STRIDE) as i32;
    }

    pub fn is_empty(&self) -> bool {
        self.vertex_count == 0
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
            let u = |n: &str| gl.get_uniform_location(self.program, n);
            gl.uniform_matrix_4_f32_slice(u("u_mvp").as_ref(), false, &mvp.to_cols_array());
            gl.uniform_3_f32(u("u_light").as_ref(), light.x, light.y, light.z);
            let eye = cam.eye();
            gl.uniform_3_f32(u("u_eye").as_ref(), eye.x, eye.y, eye.z);
            gl.uniform_1_f32(u("u_time").as_ref(), time);

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

unsafe fn bind_attributes(
    gl: &glow::Context,
    vao: glow::VertexArray,
    vbo: glow::Buffer,
    verts: &[f32],
    stride: usize,
    layout: &[(u32, i32, i32)],
) {
    unsafe {
        gl.bind_vertex_array(Some(vao));
        gl.bind_buffer(glow::ARRAY_BUFFER, Some(vbo));
        gl.buffer_data_u8_slice(
            glow::ARRAY_BUFFER,
            bytemuck::cast_slice(verts),
            glow::DYNAMIC_DRAW,
        );
        let f = size_of::<f32>() as i32;
        for &(loc, size, offset) in layout {
            gl.enable_vertex_attrib_array(loc);
            gl.vertex_attrib_pointer_f32(
                loc,
                size,
                glow::FLOAT,
                false,
                stride as i32 * f,
                offset * f,
            );
        }
        gl.bind_vertex_array(None);
    }
}
