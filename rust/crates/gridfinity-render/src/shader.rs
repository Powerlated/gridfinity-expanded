use glam::{Mat4, Vec3};
use glow::HasContext;
use std::collections::HashMap;

pub struct Shader {
    program: glow::Program,
    uniforms: HashMap<String, glow::UniformLocation>,
}

impl Shader {
    pub fn new(gl: &glow::Context, vertex: &str, fragment: &str) -> Result<Shader, String> {
        unsafe {
            let version =
                if cfg!(target_arch = "wasm32") { "#version 300 es" } else { "#version 330" };
            let program = gl.create_program()?;
            let mut shaders = Vec::new();
            for (kind, source) in
                [(glow::VERTEX_SHADER, vertex), (glow::FRAGMENT_SHADER, fragment)]
            {
                let shader = gl.create_shader(kind)?;
                gl.shader_source(shader, &format!("{version}\n{source}"));
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
            let mut uniforms = HashMap::new();
            for index in 0..gl.get_active_uniforms(program) {
                let Some(active) = gl.get_active_uniform(program, index) else { continue };
                let name = active.name.split('[').next().unwrap_or(&active.name).to_string();
                if let Some(location) = gl.get_uniform_location(program, &name) {
                    uniforms.insert(name, location);
                }
            }
            Ok(Shader { program, uniforms })
        }
    }

    pub fn bind(&self, gl: &glow::Context) {
        unsafe { gl.use_program(Some(self.program)) };
    }

    fn location(&self, name: &str) -> Option<&glow::UniformLocation> {
        self.uniforms.get(name)
    }

    pub fn mat4(&self, gl: &glow::Context, name: &str, value: &Mat4) {
        if let Some(location) = self.location(name) {
            unsafe {
                gl.uniform_matrix_4_f32_slice(Some(location), false, &value.to_cols_array());
            }
        }
    }

    pub fn vec3(&self, gl: &glow::Context, name: &str, value: Vec3) {
        if let Some(location) = self.location(name) {
            unsafe { gl.uniform_3_f32(Some(location), value.x, value.y, value.z) };
        }
    }

    pub fn vec2(&self, gl: &glow::Context, name: &str, x: f32, y: f32) {
        if let Some(location) = self.location(name) {
            unsafe { gl.uniform_2_f32(Some(location), x, y) };
        }
    }

    pub fn float(&self, gl: &glow::Context, name: &str, value: f32) {
        if let Some(location) = self.location(name) {
            unsafe { gl.uniform_1_f32(Some(location), value) };
        }
    }

    pub fn int(&self, gl: &glow::Context, name: &str, value: i32) {
        if let Some(location) = self.location(name) {
            unsafe { gl.uniform_1_i32(Some(location), value) };
        }
    }

    pub fn texture(&self, gl: &glow::Context, name: &str, unit: u32, texture: glow::Texture) {
        if let Some(location) = self.location(name) {
            unsafe {
                gl.active_texture(glow::TEXTURE0 + unit);
                gl.bind_texture(glow::TEXTURE_2D, Some(texture));
                gl.uniform_1_i32(Some(location), unit as i32);
                gl.active_texture(glow::TEXTURE0);
            }
        }
    }

    pub fn destroy(&self, gl: &glow::Context) {
        unsafe { gl.delete_program(self.program) };
    }
}
