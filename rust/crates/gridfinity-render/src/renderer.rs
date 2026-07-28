use crate::camera::Camera;
use crate::quality::Quality;
use crate::scene;
use crate::shader::Shader;
use crate::shaders;
use crate::target::{Attachments, Target};
use crate::vertex::{LINE_STRIDE, VERTEX_STRIDE, bounds_of};
use glam::{Mat4, Vec3};
use glow::HasContext;

const LINE_WIDTH_PX: f32 = 1.6;
const OCCLUSION_RADIUS_FRACTION: f32 = 0.055;
const OCCLUSION_RADIUS_RANGE: (f32, f32) = (0.5, 9.0);
const OCCLUSION_DEPTH_TOLERANCE_FRACTION: f32 = 0.02;

const FXAA_SAMPLE_LIMIT: u32 = 1;

#[derive(Clone, Copy, PartialEq)]
struct AccumulationKey {
    view_proj: [u32; 16],
    viewport: (i32, i32, i32, i32),
    quality: u32,
    generation: u64,
}

fn halton(index: u32, base: u32) -> f32 {
    let mut fraction = 1.0f32;
    let mut result = 0.0f32;
    let mut i = index;
    while i > 0 {
        fraction /= base as f32;
        result += fraction * (i % base) as f32;
        i /= base;
    }
    result
}

fn jitter_offset(sample: u32) -> (f32, f32) {
    (halton(sample + 1, 2) - 0.5, halton(sample + 1, 3) - 0.5)
}

const UNIT_SHADOW: i32 = 0;
const UNIT_REFLECTION: i32 = 1;
const UNIT_OCCLUSION: i32 = 2;

#[derive(Clone, Copy)]
pub struct Viewport {
    pub x: i32,
    pub y: i32,
    pub width: i32,
    pub height: i32,
}

impl Viewport {
    pub fn new(x: i32, y: i32, width: i32, height: i32) -> Viewport {
        Viewport { x, y, width: width.max(1), height: height.max(1) }
    }

    pub fn aspect(&self) -> f32 {
        self.width as f32 / self.height as f32
    }
}

struct Frame {
    level: Quality,
    view_proj: Mat4,
    eye: Vec3,
    time: f32,
    bounds: Option<(Vec3, Vec3)>,
    shadow_texture: Option<glow::Texture>,
    shadow_view_proj: Option<Mat4>,
    shadow_world_texel: f32,
    occlusion_texture: Option<glow::Texture>,
    reflection_texture: Option<glow::Texture>,
    reflection_weight: f32,
    near_far: (f32, f32),
    focus_distance: f32,
    aperture: f32,
    layer_lines: f32,
    scene_ldr: f32,
}

pub struct Renderer {
    mesh: Shader,
    backdrop: Shader,
    floor: Shader,
    depth_normal: Shader,
    occlusion: Shader,
    blur: Shader,
    gaussian: Shader,
    line: Shader,
    bloom_bright: Shader,
    resolve: Shader,
    fxaa: Shader,
    copy: Shader,
    vao: glow::VertexArray,
    vbo: glow::Buffer,
    vertex_count: i32,
    line_vao: glow::VertexArray,
    line_vbo: glow::Buffer,
    line_count: i32,
    empty_vao: glow::VertexArray,
    bounds: Option<(Vec3, Vec3)>,
    prepass: Target,
    occlusion_target: Target,
    occlusion_blur: Target,
    reflection: Target,
    reflection_ping: Target,
    reflection_pong: Target,
    shadow: Target,
    scene: Target,
    bloom_source: Target,
    bloom_ping: Target,
    bloom_pong: Target,
    resolved: Target,
    accumulation: Target,
    accumulated: u32,
    accumulation_key: Option<AccumulationKey>,
    generation: u64,
    quality: Quality,
    hdr: bool,
}

impl Renderer {
    pub fn new(gl: &glow::Context) -> Result<Renderer, String> {
        let hdr = supports_float_colour(gl);
        let colour = if hdr { glow::RGBA16F } else { glow::RGBA8 };
        unsafe {
            Ok(Renderer {
                mesh: Shader::new(gl, shaders::MESH_VS, &shaders::mesh_fs())?,
                backdrop: Shader::new(gl, shaders::FULLSCREEN_VS, &shaders::backdrop_fs())?,
                floor: Shader::new(gl, shaders::FLOOR_VS, &shaders::floor_fs())?,
                depth_normal: Shader::new(gl, shaders::MESH_VS, shaders::DEPTH_NORMAL_FS)?,
                occlusion: Shader::new(gl, shaders::FULLSCREEN_VS, &shaders::occlusion_fs())?,
                blur: Shader::new(gl, shaders::FULLSCREEN_VS, shaders::BILATERAL_BLUR_FS)?,
                gaussian: Shader::new(gl, shaders::FULLSCREEN_VS, shaders::GAUSSIAN_BLUR_FS)?,
                line: Shader::new(gl, shaders::LINE_VS, &shaders::line_fs())?,
                bloom_bright: Shader::new(
                    gl,
                    shaders::FULLSCREEN_VS,
                    &shaders::bloom_bright_fs(),
                )?,
                resolve: Shader::new(gl, shaders::FULLSCREEN_VS, &shaders::resolve_fs())?,
                fxaa: Shader::new(gl, shaders::FULLSCREEN_VS, shaders::FXAA_FS)?,
                copy: Shader::new(gl, shaders::FULLSCREEN_VS, shaders::COPY_FS)?,
                vao: gl.create_vertex_array()?,
                vbo: gl.create_buffer()?,
                vertex_count: 0,
                line_vao: gl.create_vertex_array()?,
                line_vbo: gl.create_buffer()?,
                line_count: 0,
                empty_vao: gl.create_vertex_array()?,
                bounds: None,
                prepass: Target::new(Attachments::ColourDepth(glow::RGBA8)),
                occlusion_target: Target::new(Attachments::Colour(colour)),
                occlusion_blur: Target::new(Attachments::Colour(colour)),
                reflection: Target::new(Attachments::ColourDepth(colour)),
                reflection_ping: Target::new(Attachments::Colour(colour)),
                reflection_pong: Target::new(Attachments::Colour(colour)),
                shadow: Target::new(Attachments::ShadowDepth),
                scene: Target::new(Attachments::ColourDepth(colour)),
                bloom_source: Target::new(Attachments::Colour(colour)),
                bloom_ping: Target::new(Attachments::Colour(colour)),
                bloom_pong: Target::new(Attachments::Colour(colour)),
                resolved: Target::new(Attachments::Colour(glow::RGBA8)),
                accumulation: Target::new(Attachments::Colour(colour)),
                accumulated: 0,
                accumulation_key: None,
                generation: 0,
                quality: Quality::default(),
                hdr,
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
        self.bounds = bounds_of(verts);
        self.generation = self.generation.wrapping_add(1);
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
        self.generation = self.generation.wrapping_add(1);
    }

    pub fn is_empty(&self) -> bool {
        self.vertex_count == 0
    }

    pub fn quality(&self) -> Quality {
        self.quality
    }

    pub fn set_quality(&mut self, level: Quality) {
        self.quality = level;
    }

    pub fn is_accumulating(&self) -> bool {
        self.accumulated < self.quality.accumulation_samples()
            && (self.quality.antialias() || self.quality.bloom())
    }

    pub fn is_high_dynamic_range(&self) -> bool {
        self.hdr
    }

    pub fn paint(&mut self, gl: &glow::Context, cam: &Camera, viewport: Viewport, time: f32) {
        let level = self.quality;
        let bounds = self.bounds.filter(|_| self.vertex_count > 0);
        let steady = cam.view_proj(viewport.aspect());
        let posts = level.antialias() || level.bloom();
        let key = AccumulationKey {
            view_proj: steady.to_cols_array().map(f32::to_bits),
            viewport: (viewport.x, viewport.y, viewport.width, viewport.height),
            quality: level.index(),
            generation: self.generation,
        };
        if self.accumulation_key != Some(key) {
            self.accumulation_key = Some(key);
            self.accumulated = 0;
        }
        let converged = posts && self.accumulated >= level.accumulation_samples();
        let view_proj = if posts && !converged {
            let (ox, oy) = jitter_offset(self.accumulated);
            Mat4::from_translation(Vec3::new(
                2.0 * ox / viewport.width as f32,
                2.0 * oy / viewport.height as f32,
                0.0,
            )) * steady
        } else {
            steady
        };
        let mut frame = Frame {
            level,
            view_proj,
            eye: cam.eye(),
            time,
            bounds,
            shadow_texture: None,
            shadow_view_proj: bounds.map(|(min, max)| scene::shadow_view_proj(min, max)),
            shadow_world_texel: bounds
                .map(|(min, max)| scene::shadow_world_texel(min, max, level.shadow_resolution()))
                .unwrap_or(0.0),
            occlusion_texture: None,
            reflection_texture: None,
            reflection_weight: if bounds.is_some() {
                scene::reflection_weight(cam.pitch)
            } else {
                0.0
            },
            near_far: cam.near_far(),
            focus_distance: bounds
                .map(|(min, max)| (cam.eye() - (min + max) * 0.5).length())
                .unwrap_or(1.0),
            aperture: if level.depth_of_field() && bounds.is_some() {
                scene::DOF_APERTURE
            } else {
                0.0
            },
            layer_lines: if level.layer_lines() { 1.0 } else { 0.0 },
            scene_ldr: 1.0,
        };

        let restore = unsafe { bound_framebuffer(gl) };
        let scissored = unsafe { gl.is_enabled(glow::SCISSOR_TEST) };
        let blended = unsafe { gl.is_enabled(glow::BLEND) };
        unsafe {
            gl.disable(glow::SCISSOR_TEST);
            claim_pipeline_state(gl);
        };

        if bounds.is_some() && !converged {
            if level.shadow() {
                let shadow = self.draw_shadow_map(gl, level, frame.shadow_view_proj);
                frame.shadow_texture = shadow;
            }
            if level.reflection() && frame.reflection_weight > 0.0 {
                let reflection = self.draw_reflection(gl, &frame, viewport);
                frame.reflection_texture = reflection;
            }
            if level.ambient_occlusion() {
                let (view_proj, eye) = (frame.view_proj, frame.eye);
                let bounce = self.accumulated > 0;
                let occlusion =
                    self.draw_occlusion(gl, &frame, viewport, &view_proj, eye, bounce);
                frame.occlusion_texture = occlusion;
            }
        }

        let posted = self.paint_through_post_chain(gl, &mut frame, viewport, restore, scissored);
        if !posted {
            unsafe {
                bind_framebuffer(gl, restore);
                if scissored {
                    gl.enable(glow::SCISSOR_TEST);
                }
                gl.viewport(viewport.x, viewport.y, viewport.width, viewport.height);
                gl.depth_mask(true);
                gl.clear(glow::DEPTH_BUFFER_BIT);
            }
            frame.scene_ldr = 1.0;
            unsafe { self.draw_scene(gl, &frame, viewport) };
        }

        unsafe {
            gl.disable(glow::DEPTH_TEST);
            gl.disable(glow::CULL_FACE);
            gl.bind_vertex_array(None);
            if blended {
                gl.enable(glow::BLEND);
            }
        }
    }

    fn paint_through_post_chain(
        &mut self,
        gl: &glow::Context,
        frame: &mut Frame,
        viewport: Viewport,
        restore: Option<glow::Framebuffer>,
        scissored: bool,
    ) -> bool {
        let level = frame.level;
        if !level.antialias() && !level.bloom() {
            return false;
        }
        let (width, height) = (viewport.width, viewport.height);
        let Some(scene_fbo) = self.scene.ensure(gl, width, height) else { return false };
        let Some(accumulation_fbo) = self.accumulation.ensure(gl, width, height) else {
            return false;
        };
        let Some(resolve_fbo) = self.resolved.ensure(gl, width, height) else { return false };

        frame.scene_ldr = if self.hdr { 0.0 } else { 1.0 };
        let offscreen = Viewport::new(0, 0, width, height);
        if self.accumulated < level.accumulation_samples() {
            unsafe {
                gl.bind_framebuffer(glow::FRAMEBUFFER, Some(scene_fbo));
                gl.viewport(0, 0, width, height);
                gl.clear_color(0.0, 0.0, 0.0, 1.0);
                gl.depth_mask(true);
                gl.clear(glow::COLOR_BUFFER_BIT | glow::DEPTH_BUFFER_BIT);
                self.draw_scene(gl, frame, offscreen);
            }
            let Some(scene_texture) = self.scene.colour_texture() else { return false };
            let weight = 1.0 / (self.accumulated + 1) as f32;
            unsafe {
                gl.bind_framebuffer(glow::FRAMEBUFFER, Some(accumulation_fbo));
                gl.viewport(0, 0, width, height);
                gl.disable(glow::DEPTH_TEST);
                gl.depth_mask(false);
                gl.disable(glow::CULL_FACE);
                gl.enable(glow::BLEND);
                gl.blend_equation(glow::FUNC_ADD);
                gl.blend_color(0.0, 0.0, 0.0, weight);
                gl.blend_func(glow::CONSTANT_ALPHA, glow::ONE_MINUS_CONSTANT_ALPHA);
                self.copy.bind(gl);
                self.copy.texture(gl, "u_source", 0, scene_texture);
                self.copy.vec2(gl, "u_target_size", width as f32, height as f32);
                self.copy.vec2(gl, "u_origin", 0.0, 0.0);
                gl.bind_vertex_array(Some(self.empty_vao));
                gl.draw_arrays(glow::TRIANGLES, 0, 3);
                gl.bind_vertex_array(None);
                gl.disable(glow::BLEND);
            }
            self.accumulated += 1;
        }

        let scene_depth = self.scene.depth_texture();
        let Some(scene_texture) = self.accumulation.colour_texture() else { return false };
        let bloom = if self.hdr && level.bloom() {
            self.draw_bloom(gl, level, scene_texture, width, height)
        } else {
            None
        };

        unsafe {
            gl.bind_framebuffer(glow::FRAMEBUFFER, Some(resolve_fbo));
            gl.viewport(0, 0, width, height);
            gl.disable(glow::DEPTH_TEST);
            gl.depth_mask(false);
            gl.disable(glow::CULL_FACE);
            self.resolve.bind(gl);
            self.resolve.texture(gl, "u_scene", 0, scene_texture);
            self.resolve.vec2(gl, "u_target_size", width as f32, height as f32);
            self.resolve.float(gl, "u_scene_linear", if self.hdr { 1.0 } else { 0.0 });
            self.resolve.float(gl, "u_near", frame.near_far.0);
            self.resolve.float(gl, "u_far", frame.near_far.1);
            self.resolve.float(gl, "u_focus", frame.focus_distance);
            self.resolve.float(gl, "u_aperture", frame.aperture);
            self.resolve.int(gl, "u_depth", 2);
            if let Some(depth) = scene_depth {
                self.resolve.texture(gl, "u_depth", 2, depth);
            }
            match bloom {
                Some(texture) => {
                    self.resolve.texture(gl, "u_bloom", 1, texture);
                    self.resolve.float(gl, "u_bloom_enabled", 1.0);
                }
                None => {
                    self.resolve.int(gl, "u_bloom", 1);
                    self.resolve.float(gl, "u_bloom_enabled", 0.0);
                }
            }
            gl.bind_vertex_array(Some(self.empty_vao));
            gl.draw_arrays(glow::TRIANGLES, 0, 3);
            gl.bind_vertex_array(None);
        }

        let Some(resolved) = self.resolved.colour_texture() else { return false };
        unsafe {
            bind_framebuffer(gl, restore);
            if scissored {
                gl.enable(glow::SCISSOR_TEST);
            }
            gl.viewport(viewport.x, viewport.y, viewport.width, viewport.height);
            gl.disable(glow::DEPTH_TEST);
            gl.depth_mask(false);
            gl.disable(glow::CULL_FACE);
            let blit =
                if level.antialias() && self.accumulated <= FXAA_SAMPLE_LIMIT {
                    &self.fxaa
                } else {
                    &self.copy
                };
            blit.bind(gl);
            blit.texture(gl, "u_source", 0, resolved);
            blit.vec2(gl, "u_target_size", width as f32, height as f32);
            blit.vec2(gl, "u_origin", viewport.x as f32, viewport.y as f32);
            gl.bind_vertex_array(Some(self.empty_vao));
            gl.draw_arrays(glow::TRIANGLES, 0, 3);
            gl.bind_vertex_array(None);
            gl.depth_mask(true);
        }
        true
    }

    unsafe fn draw_scene(&self, gl: &glow::Context, frame: &Frame, viewport: Viewport) {
        unsafe {
            gl.disable(glow::DEPTH_TEST);
            gl.disable(glow::CULL_FACE);
            gl.depth_mask(false);
            self.backdrop.bind(gl);
            self.set_scene_uniforms(gl, &self.backdrop, frame, viewport);
            gl.bind_vertex_array(Some(self.empty_vao));
            gl.draw_arrays(glow::TRIANGLES, 0, 3);

            gl.enable(glow::DEPTH_TEST);
            gl.depth_func(glow::LESS);
            gl.depth_mask(true);
            gl.enable(glow::CULL_FACE);
            gl.cull_face(glow::BACK);

            if let Some((min, max)) = frame.bounds {
                self.floor.bind(gl);
                self.set_scene_uniforms(gl, &self.floor, frame, viewport);
                self.set_shadow_uniforms(gl, &self.floor, frame);
                self.floor.mat4(gl, "u_view_proj", &frame.view_proj);
                self.floor.vec3(gl, "u_eye", frame.eye);
                self.floor.vec3(gl, "u_fill_dir", scene::fill_direction());
                self.floor.vec3(gl, "u_floor_centre", scene::floor_centre(min, max));
                self.floor.float(gl, "u_floor_radius", scene::floor_radius(min, max));
                self.floor.int(gl, "u_reflection", UNIT_REFLECTION);
                match frame.reflection_texture {
                    Some(texture) => {
                        self.floor.texture(gl, "u_reflection", UNIT_REFLECTION as u32, texture);
                        self.floor.float(gl, "u_reflection_weight", frame.reflection_weight);
                    }
                    None => self.floor.float(gl, "u_reflection_weight", 0.0),
                }
                gl.bind_vertex_array(Some(self.empty_vao));
                gl.draw_arrays(glow::TRIANGLE_STRIP, 0, 4);
            }

            if self.vertex_count > 0 {
                if self.line_count > 0 {
                    gl.enable(glow::POLYGON_OFFSET_FILL);
                    gl.polygon_offset(1.0, 1.0);
                }
                self.mesh.bind(gl);
                self.set_scene_uniforms(gl, &self.mesh, frame, viewport);
                self.set_shadow_uniforms(gl, &self.mesh, frame);
                self.mesh.mat4(gl, "u_view_proj", &frame.view_proj);
                self.mesh.mat4(gl, "u_model", &Mat4::IDENTITY);
                self.mesh.vec3(gl, "u_eye", frame.eye);
                self.mesh.vec3(gl, "u_fill_dir", scene::fill_direction());
                self.mesh.float(gl, "u_time", frame.time);
                self.mesh.float(gl, "u_layer_lines", frame.layer_lines);
                self.mesh.int(gl, "u_occlusion", UNIT_OCCLUSION);
                match frame.occlusion_texture {
                    Some(texture) => {
                        self.mesh.texture(gl, "u_occlusion", UNIT_OCCLUSION as u32, texture);
                        self.mesh.float(gl, "u_occlusion_enabled", 1.0);
                    }
                    None => self.mesh.float(gl, "u_occlusion_enabled", 0.0),
                }
                gl.bind_vertex_array(Some(self.vao));
                gl.draw_arrays(glow::TRIANGLES, 0, self.vertex_count);
                gl.bind_vertex_array(None);

                if self.line_count > 0 {
                    gl.disable(glow::POLYGON_OFFSET_FILL);
                    self.paint_lines(gl, frame, viewport);
                }
            }
        }
    }

    fn set_scene_uniforms(
        &self,
        gl: &glow::Context,
        shader: &Shader,
        frame: &Frame,
        viewport: Viewport,
    ) {
        shader.vec2(gl, "u_vp_origin", viewport.x as f32, viewport.y as f32);
        shader.vec2(gl, "u_vp_size", viewport.width as f32, viewport.height as f32);
        shader.float(gl, "u_scene_ldr", frame.scene_ldr);
    }

    fn set_shadow_uniforms(&self, gl: &glow::Context, shader: &Shader, frame: &Frame) {
        shader.vec3(gl, "u_key_dir", scene::key_direction());
        shader.int(gl, "u_shadow", UNIT_SHADOW);
        match (frame.shadow_texture, frame.shadow_view_proj) {
            (Some(texture), Some(view_proj)) => {
                shader.texture(gl, "u_shadow", UNIT_SHADOW as u32, texture);
                shader.mat4(gl, "u_light_vp", &view_proj);
                shader.float(gl, "u_shadow_enabled", 1.0);
                shader.float(
                    gl,
                    "u_shadow_texel",
                    1.0 / frame.level.shadow_resolution().max(1) as f32,
                );
                shader.float(gl, "u_shadow_world_texel", frame.shadow_world_texel);
                shader.int(gl, "u_shadow_taps", frame.level.shadow_taps());
            }
            _ => shader.float(gl, "u_shadow_enabled", 0.0),
        }
    }

    fn draw_shadow_map(
        &mut self,
        gl: &glow::Context,
        level: Quality,
        view_proj: Option<Mat4>,
    ) -> Option<glow::Texture> {
        let view_proj = view_proj?;
        let size = level.shadow_resolution();
        let fbo = self.shadow.ensure(gl, size, size)?;
        unsafe {
            gl.bind_framebuffer(glow::FRAMEBUFFER, Some(fbo));
            gl.viewport(0, 0, size, size);
            gl.enable(glow::DEPTH_TEST);
            gl.depth_func(glow::LESS);
            gl.depth_mask(true);
            gl.clear(glow::DEPTH_BUFFER_BIT);
            gl.enable(glow::CULL_FACE);
            gl.cull_face(glow::FRONT);
            self.depth_normal.bind(gl);
            self.depth_normal.mat4(gl, "u_view_proj", &view_proj);
            self.depth_normal.mat4(gl, "u_model", &Mat4::IDENTITY);
            gl.bind_vertex_array(Some(self.vao));
            gl.draw_arrays(glow::TRIANGLES, 0, self.vertex_count);
            gl.bind_vertex_array(None);
            gl.cull_face(glow::BACK);
        }
        self.shadow.depth_texture()
    }

    fn draw_occlusion(
        &mut self,
        gl: &glow::Context,
        frame: &Frame,
        viewport: Viewport,
        view_proj: &Mat4,
        eye: Vec3,
        bounce: bool,
    ) -> Option<glow::Texture> {
        let (min, max) = frame.bounds?;
        let divisor = frame.level.occlusion_divisor();
        let width = (viewport.width / divisor).max(1);
        let height = (viewport.height / divisor).max(1);

        let prepass = self.prepass.ensure(gl, width, height)?;
        unsafe {
            gl.bind_framebuffer(glow::FRAMEBUFFER, Some(prepass));
            gl.viewport(0, 0, width, height);
            gl.clear_color(0.5, 0.5, 1.0, 1.0);
            gl.enable(glow::DEPTH_TEST);
            gl.depth_func(glow::LESS);
            gl.depth_mask(true);
            gl.clear(glow::COLOR_BUFFER_BIT | glow::DEPTH_BUFFER_BIT);
            gl.enable(glow::CULL_FACE);
            gl.cull_face(glow::BACK);
            self.depth_normal.bind(gl);
            self.depth_normal.mat4(gl, "u_view_proj", view_proj);
            self.depth_normal.mat4(gl, "u_model", &Mat4::IDENTITY);
            gl.bind_vertex_array(Some(self.vao));
            gl.draw_arrays(glow::TRIANGLES, 0, self.vertex_count);
            gl.bind_vertex_array(None);
        }

        let depth = self.prepass.depth_texture()?;
        let normal = self.prepass.colour_texture()?;
        let previous = self.accumulation.colour_texture();
        let ao = self.occlusion_target.ensure(gl, width, height)?;
        let radius = (scene::scene_radius(min, max) * OCCLUSION_RADIUS_FRACTION)
            .clamp(OCCLUSION_RADIUS_RANGE.0, OCCLUSION_RADIUS_RANGE.1);
        unsafe {
            gl.bind_framebuffer(glow::FRAMEBUFFER, Some(ao));
            gl.viewport(0, 0, width, height);
            gl.disable(glow::DEPTH_TEST);
            gl.depth_mask(false);
            gl.disable(glow::CULL_FACE);
            self.occlusion.bind(gl);
            self.occlusion.texture(gl, "u_depth", 0, depth);
            self.occlusion.texture(gl, "u_normal", 1, normal);
            self.occlusion.mat4(gl, "u_view_proj", view_proj);
            self.occlusion.mat4(gl, "u_inv_view_proj", &view_proj.inverse());
            self.occlusion.vec2(gl, "u_target_size", width as f32, height as f32);
            self.occlusion.vec3(gl, "u_eye", eye);
            self.occlusion.float(gl, "u_radius", radius);
            self.occlusion.int(gl, "u_samples", frame.level.occlusion_samples());
            self.occlusion.float(gl, "u_frame", self.accumulated as f32);
            self.occlusion.int(gl, "u_previous", 2);
            match previous {
                Some(texture) if bounce => {
                    self.occlusion.texture(gl, "u_previous", 2, texture);
                    self.occlusion.float(gl, "u_bounce", 1.0);
                }
                _ => self.occlusion.float(gl, "u_bounce", 0.0),
            }
            gl.bind_vertex_array(Some(self.empty_vao));
            gl.draw_arrays(glow::TRIANGLES, 0, 3);
        }

        let tolerance = scene::scene_radius(min, max) * OCCLUSION_DEPTH_TOLERANCE_FRACTION;
        let mut source = self.occlusion_target.colour_texture()?;
        for (index, direction) in [(0, (1.0f32, 0.0f32)), (1, (0.0f32, 1.0f32))] {
            let target = if index == 0 {
                self.occlusion_blur.ensure(gl, width, height)?
            } else {
                self.occlusion_target.ensure(gl, width, height)?
            };
            unsafe {
                gl.bind_framebuffer(glow::FRAMEBUFFER, Some(target));
                gl.viewport(0, 0, width, height);
                self.blur.bind(gl);
                self.blur.texture(gl, "u_source", 0, source);
                self.blur.texture(gl, "u_depth", 1, depth);
                self.blur.vec2(gl, "u_target_size", width as f32, height as f32);
                self.blur.vec2(gl, "u_direction", direction.0, direction.1);
                self.blur.float(gl, "u_near", frame.near_far.0);
                self.blur.float(gl, "u_far", frame.near_far.1);
                self.blur.float(gl, "u_tolerance", tolerance);
                gl.bind_vertex_array(Some(self.empty_vao));
                gl.draw_arrays(glow::TRIANGLES, 0, 3);
                gl.bind_vertex_array(None);
            }
            source = if index == 0 {
                self.occlusion_blur.colour_texture()?
            } else {
                self.occlusion_target.colour_texture()?
            };
        }
        Some(source)
    }

    fn draw_reflection(
        &mut self,
        gl: &glow::Context,
        frame: &Frame,
        viewport: Viewport,
    ) -> Option<glow::Texture> {
        let (min, _) = frame.bounds?;
        let (width, height) = (viewport.width, viewport.height);
        let mirror = scene::mirror_about_height(min.z);
        let mirrored_view_proj = frame.view_proj * mirror;
        let mirrored_eye = mirror.transform_point3(frame.eye);

        let occlusion = if frame.level.ambient_occlusion() {
            self.draw_occlusion(gl, frame, viewport, &mirrored_view_proj, mirrored_eye, false)
        } else {
            None
        };

        let fbo = self.reflection.ensure(gl, width, height)?;
        unsafe {
            gl.bind_framebuffer(glow::FRAMEBUFFER, Some(fbo));
            gl.viewport(0, 0, width, height);
            gl.clear_color(0.0, 0.0, 0.0, 0.0);
            gl.enable(glow::DEPTH_TEST);
            gl.depth_func(glow::LESS);
            gl.depth_mask(true);
            gl.clear(glow::COLOR_BUFFER_BIT | glow::DEPTH_BUFFER_BIT);
            gl.enable(glow::CULL_FACE);
            gl.cull_face(glow::FRONT);
            self.mesh.bind(gl);
            self.mesh.vec2(gl, "u_vp_origin", 0.0, 0.0);
            self.mesh.vec2(gl, "u_vp_size", width as f32, height as f32);
            self.mesh.float(gl, "u_scene_ldr", 0.0);
            self.set_shadow_uniforms(gl, &self.mesh, frame);
            self.mesh.mat4(gl, "u_view_proj", &mirrored_view_proj);
            self.mesh.mat4(gl, "u_model", &Mat4::IDENTITY);
            self.mesh.vec3(gl, "u_eye", mirrored_eye);
            self.mesh.vec3(gl, "u_fill_dir", scene::fill_direction());
            self.mesh.float(gl, "u_time", frame.time);
            self.mesh.float(gl, "u_layer_lines", frame.layer_lines);
            self.mesh.int(gl, "u_occlusion", UNIT_OCCLUSION);
            match occlusion {
                Some(texture) => {
                    self.mesh.texture(gl, "u_occlusion", UNIT_OCCLUSION as u32, texture);
                    self.mesh.float(gl, "u_occlusion_enabled", 1.0);
                }
                None => self.mesh.float(gl, "u_occlusion_enabled", 0.0),
            }
            gl.bind_vertex_array(Some(self.vao));
            gl.draw_arrays(glow::TRIANGLES, 0, self.vertex_count);
            gl.bind_vertex_array(None);
            gl.cull_face(glow::BACK);
        }

        let source = self.reflection.colour_texture()?;
        unsafe {
            gl.disable(glow::DEPTH_TEST);
            gl.depth_mask(false);
            gl.disable(glow::CULL_FACE);
        }
        gaussian_blur(
            gl,
            &self.gaussian,
            self.empty_vao,
            &mut self.reflection_ping,
            &mut self.reflection_pong,
            source,
            width,
            height,
            scene::REFLECTION_GLOSS_RADIUS,
        )
    }

    fn draw_bloom(
        &mut self,
        gl: &glow::Context,
        level: Quality,
        scene_texture: glow::Texture,
        width: i32,
        height: i32,
    ) -> Option<glow::Texture> {
        let bright = level.bloom_bright_divisor();
        let blur = level.bloom_blur_divisor();
        let (bw, bh) = ((width / bright).max(1), (height / bright).max(1));
        let (gw, gh) = ((width / blur).max(1), (height / blur).max(1));

        let fbo = self.bloom_source.ensure(gl, bw, bh)?;
        unsafe {
            gl.bind_framebuffer(glow::FRAMEBUFFER, Some(fbo));
            gl.viewport(0, 0, bw, bh);
            gl.disable(glow::DEPTH_TEST);
            gl.depth_mask(false);
            gl.disable(glow::CULL_FACE);
            self.bloom_bright.bind(gl);
            self.bloom_bright.texture(gl, "u_source", 0, scene_texture);
            self.bloom_bright.vec2(gl, "u_target_size", bw as f32, bh as f32);
            gl.bind_vertex_array(Some(self.empty_vao));
            gl.draw_arrays(glow::TRIANGLES, 0, 3);
        }

        let source = self.bloom_source.colour_texture()?;
        gaussian_blur(
            gl,
            &self.gaussian,
            self.empty_vao,
            &mut self.bloom_ping,
            &mut self.bloom_pong,
            source,
            gw,
            gh,
            scene::BLOOM_BLUR_RADIUS,
        )
    }

    unsafe fn paint_lines(&self, gl: &glow::Context, frame: &Frame, viewport: Viewport) {
        unsafe {
            self.line.bind(gl);
            self.line.float(gl, "u_scene_ldr", frame.scene_ldr);
            self.line.mat4(gl, "u_view_proj", &frame.view_proj);
            self.line.vec2(
                gl,
                "u_half_vp",
                viewport.width as f32 * 0.5,
                viewport.height as f32 * 0.5,
            );
            self.line.float(gl, "u_width", LINE_WIDTH_PX);

            gl.enable(glow::BLEND);
            gl.blend_equation(glow::FUNC_ADD);
            gl.blend_func(glow::SRC_ALPHA, glow::ONE_MINUS_SRC_ALPHA);
            gl.depth_mask(false);
            gl.disable(glow::CULL_FACE);
            gl.bind_vertex_array(Some(self.line_vao));

            for (func, alpha) in [(glow::LEQUAL, 1.0f32), (glow::GREATER, 0.22)] {
                gl.depth_func(func);
                self.line.float(gl, "u_alpha", alpha);
                gl.draw_arrays(glow::TRIANGLES, 0, self.line_count);
            }

            gl.bind_vertex_array(None);
            gl.depth_func(glow::LESS);
            gl.depth_mask(true);
            gl.disable(glow::BLEND);
        }
    }

    pub fn destroy(&mut self, gl: &glow::Context) {
        for shader in [
            &self.mesh,
            &self.backdrop,
            &self.floor,
            &self.depth_normal,
            &self.occlusion,
            &self.blur,
            &self.gaussian,
            &self.line,
            &self.bloom_bright,
            &self.resolve,
            &self.fxaa,
            &self.copy,
        ] {
            shader.destroy(gl);
        }
        for target in [
            &mut self.prepass,
            &mut self.occlusion_target,
            &mut self.occlusion_blur,
            &mut self.reflection,
            &mut self.reflection_ping,
            &mut self.reflection_pong,
            &mut self.shadow,
            &mut self.scene,
            &mut self.bloom_source,
            &mut self.bloom_ping,
            &mut self.bloom_pong,
            &mut self.resolved,
            &mut self.accumulation,
        ] {
            target.release(gl);
        }
        unsafe {
            gl.delete_vertex_array(self.vao);
            gl.delete_buffer(self.vbo);
            gl.delete_vertex_array(self.line_vao);
            gl.delete_buffer(self.line_vbo);
            gl.delete_vertex_array(self.empty_vao);
        }
    }
}

unsafe fn claim_pipeline_state(gl: &glow::Context) {
    unsafe {
        gl.disable(glow::BLEND);
        gl.blend_equation_separate(glow::FUNC_ADD, glow::FUNC_ADD);
        gl.disable(glow::STENCIL_TEST);
        gl.disable(glow::POLYGON_OFFSET_FILL);
        gl.disable(glow::DITHER);
        gl.color_mask(true, true, true, true);
        gl.front_face(glow::CCW);
        gl.depth_range_f32(0.0, 1.0);
        gl.clear_depth_f32(1.0);
        disable_framebuffer_srgb(gl);
    }
}

#[cfg(not(target_arch = "wasm32"))]
unsafe fn disable_framebuffer_srgb(gl: &glow::Context) {
    unsafe { gl.disable(glow::FRAMEBUFFER_SRGB) };
}

#[cfg(target_arch = "wasm32")]
unsafe fn disable_framebuffer_srgb(_gl: &glow::Context) {}

#[allow(clippy::too_many_arguments)]
fn gaussian_blur(
    gl: &glow::Context,
    shader: &Shader,
    empty_vao: glow::VertexArray,
    ping: &mut Target,
    pong: &mut Target,
    source: glow::Texture,
    width: i32,
    height: i32,
    radius: f32,
) -> Option<glow::Texture> {
    let mut input = source;
    for (target, direction) in
        [(&mut *ping, (1.0f32, 0.0f32)), (&mut *pong, (0.0f32, 1.0f32))]
    {
        let fbo = target.ensure(gl, width, height)?;
        unsafe {
            gl.bind_framebuffer(glow::FRAMEBUFFER, Some(fbo));
            gl.viewport(0, 0, width, height);
            shader.bind(gl);
            shader.texture(gl, "u_source", 0, input);
            shader.vec2(gl, "u_target_size", width as f32, height as f32);
            shader.vec2(gl, "u_direction", direction.0, direction.1);
            shader.float(gl, "u_radius", radius);
            gl.bind_vertex_array(Some(empty_vao));
            gl.draw_arrays(glow::TRIANGLES, 0, 3);
            gl.bind_vertex_array(None);
        }
        input = target.colour_texture()?;
    }
    Some(input)
}

fn supports_float_colour(gl: &glow::Context) -> bool {
    let mut probe = Target::new(Attachments::ColourDepth(glow::RGBA16F));
    let complete = probe.ensure(gl, 4, 4).is_some();
    probe.release(gl);
    complete
}

#[cfg(not(target_arch = "wasm32"))]
unsafe fn bound_framebuffer(gl: &glow::Context) -> Option<glow::Framebuffer> {
    unsafe {
        let raw = gl.get_parameter_i32(glow::DRAW_FRAMEBUFFER_BINDING);
        std::num::NonZeroU32::new(raw as u32).map(glow::NativeFramebuffer)
    }
}

#[cfg(target_arch = "wasm32")]
unsafe fn bound_framebuffer(_gl: &glow::Context) -> Option<glow::Framebuffer> {
    None
}

unsafe fn bind_framebuffer(gl: &glow::Context, fbo: Option<glow::Framebuffer>) {
    unsafe { gl.bind_framebuffer(glow::FRAMEBUFFER, fbo) };
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
