use crate::arena::Arena;
use crate::camera::Camera;
use crate::pipelines::{PREPASS_FORMAT, Pipelines, RESOLVED_FORMAT};
use crate::quality::Quality;
use crate::scene;
use crate::target::{Attachments, Target, supports_float_colour};
use crate::uniforms::{LineUniform, PostUniform, SceneUniform, mat, vec4};
use crate::vertex::{LINE_STRIDE, VERTEX_STRIDE, bounds_of};
use glam::{Mat4, Vec3};

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
    shadow_view_proj: Option<Mat4>,
    shadow_world_texel: f32,
    has_shadow: bool,
    has_occlusion: bool,
    has_reflection: bool,
    floor_presence: f32,
    near_far: (f32, f32),
    focus_distance: f32,
    aperture: f32,
    layer_lines: f32,
    scene_ldr: f32,
}

struct Arenas {
    scene: Arena,
    post: Arena,
    line: Arena,
}

impl Arenas {
    fn reset(&mut self) {
        self.scene.reset();
        self.post.reset();
        self.line.reset();
    }
}

pub struct Renderer {
    pipelines: Pipelines,
    arenas: Arenas,
    vertices: Option<wgpu::Buffer>,
    vertex_count: u32,
    lines: Option<wgpu::Buffer>,
    line_count: u32,
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
    presented: Option<Viewport>,
    presented_is_resolved: bool,
}

impl Renderer {
    pub fn new(device: &wgpu::Device, adapter: &wgpu::Adapter) -> Result<Renderer, String> {
        let hdr = supports_float_colour(adapter);
        let colour_format =
            if hdr { wgpu::TextureFormat::Rgba16Float } else { wgpu::TextureFormat::Rgba8Unorm };
        Ok(Renderer {
            pipelines: Pipelines::new(device, colour_format),
            arenas: Arenas {
                scene: Arena::new(device, "scene-uniforms", size_of::<SceneUniform>()),
                post: Arena::new(device, "post-uniforms", size_of::<PostUniform>()),
                line: Arena::new(device, "line-uniforms", size_of::<LineUniform>()),
            },
            vertices: None,
            vertex_count: 0,
            lines: None,
            line_count: 0,
            bounds: None,
            prepass: Target::new(Attachments::ColourDepth(PREPASS_FORMAT)),
            occlusion_target: Target::new(Attachments::Colour(colour_format)),
            occlusion_blur: Target::new(Attachments::Colour(colour_format)),
            reflection: Target::new(Attachments::ColourDepth(colour_format)),
            reflection_ping: Target::new(Attachments::Colour(colour_format)),
            reflection_pong: Target::new(Attachments::Colour(colour_format)),
            shadow: Target::new(Attachments::ShadowDepth),
            scene: Target::new(Attachments::ColourDepth(colour_format)),
            bloom_source: Target::new(Attachments::Colour(colour_format)),
            bloom_ping: Target::new(Attachments::Colour(colour_format)),
            bloom_pong: Target::new(Attachments::Colour(colour_format)),
            resolved: Target::new(Attachments::Colour(RESOLVED_FORMAT)),
            accumulation: Target::new(Attachments::Colour(colour_format)),
            accumulated: 0,
            accumulation_key: None,
            generation: 0,
            quality: Quality::default(),
            hdr,
            presented: None,
            presented_is_resolved: false,
        })
    }

    pub fn upload(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, verts: &[f32]) {
        self.vertices = write_vertices(device, queue, self.vertices.take(), verts, "mesh");
        self.vertex_count = (verts.len() / VERTEX_STRIDE) as u32;
        self.bounds = bounds_of(verts);
        self.generation = self.generation.wrapping_add(1);
    }

    pub fn upload_lines(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, verts: &[f32]) {
        self.lines = write_vertices(device, queue, self.lines.take(), verts, "line");
        self.line_count = (verts.len() / LINE_STRIDE) as u32;
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

    pub fn prepare(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        cam: &Camera,
        viewport: Viewport,
        time: f32,
    ) -> wgpu::CommandBuffer {
        self.arenas.reset();
        let mut encoder =
            device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("scene") });

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
            shadow_view_proj: bounds.map(|(min, max)| scene::shadow_view_proj(min, max)),
            shadow_world_texel: bounds
                .map(|(min, max)| scene::shadow_world_texel(min, max, level.shadow_resolution()))
                .unwrap_or(0.0),
            has_shadow: false,
            has_occlusion: false,
            has_reflection: false,
            floor_presence: if bounds.is_some() {
                scene::floor_presence(cam.pitch)
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
            scene_ldr: if posts && self.hdr { 0.0 } else { 1.0 },
        };

        let (width, height) = (viewport.width, viewport.height);
        let offscreen = Viewport::new(0, 0, width, height);

        if bounds.is_some() && !converged {
            if level.shadow() {
                frame.has_shadow =
                    self.draw_shadow_map(device, queue, &mut encoder, level, frame.shadow_view_proj);
            }
            if level.reflection() && frame.floor_presence > 0.0 {
                frame.has_reflection =
                    self.draw_reflection(device, queue, &mut encoder, &frame, offscreen);
            }
            if level.ambient_occlusion() {
                let (view_proj, eye) = (frame.view_proj, frame.eye);
                let bounce = self.accumulated > 0;
                frame.has_occlusion = self.draw_occlusion(
                    device, queue, &mut encoder, &frame, offscreen, &view_proj, eye, bounce,
                );
            }
        }

        if !self.scene.ensure(device, width, height) {
            self.presented = None;
            return encoder.finish();
        }

        if !posts {
            if !converged {
                self.draw_scene(device, queue, &mut encoder, &frame, offscreen);
            }
            self.presented = Some(viewport);
            self.presented_is_resolved = false;
            return encoder.finish();
        }

        if !self.accumulation.ensure(device, width, height)
            || !self.resolved.ensure(device, width, height)
        {
            self.presented = None;
            return encoder.finish();
        }

        if self.accumulated < level.accumulation_samples() {
            self.draw_scene(device, queue, &mut encoder, &frame, offscreen);
            self.accumulate(device, queue, &mut encoder, width, height);
            self.accumulated += 1;
        }

        let bloom = if self.hdr && level.bloom() {
            self.draw_bloom(device, queue, &mut encoder, level, width, height)
        } else {
            false
        };
        self.resolve(device, queue, &mut encoder, &frame, width, height, bloom);
        self.presented = Some(viewport);
        self.presented_is_resolved = true;
        encoder.finish()
    }

    pub fn blit(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        pass: &mut wgpu::RenderPass<'_>,
        format: wgpu::TextureFormat,
    ) {
        let Some(viewport) = self.presented else { return };
        let source = if self.presented_is_resolved {
            self.resolved.colour_view()
        } else {
            self.scene.colour_view()
        };
        let Some(source) = source else { return };

        let antialias = self.presented_is_resolved
            && self.quality.antialias()
            && self.accumulated <= FXAA_SAMPLE_LIMIT;
        let mut post = PostUniform {
            target_size: [viewport.width as f32, viewport.height as f32],
            origin: [viewport.x as f32, viewport.y as f32],
            ..Default::default()
        };
        post.flags[0] = if format.is_srgb() { 1.0 } else { 0.0 };
        let offset = self.arenas.post.push(queue, &post);
        let bind_group = post_bind_group(
            device,
            &self.pipelines,
            &self.arenas.post,
            offset,
            Some(source),
            None,
            None,
            None,
        );
        let pipeline = self.pipelines.blit(device, format, antialias);
        pass.set_viewport(
            viewport.x as f32,
            viewport.y as f32,
            viewport.width as f32,
            viewport.height as f32,
            0.0,
            1.0,
        );
        pass.set_pipeline(pipeline);
        pass.set_bind_group(0, &bind_group, &[]);
        pass.draw(0..3, 0..1);
    }

    fn scene_uniform(
        &self,
        frame: &Frame,
        viewport: Viewport,
        view_proj: &Mat4,
        eye: Vec3,
        scene_ldr: f32,
    ) -> SceneUniform {
        let mut uniform = SceneUniform {
            view_proj: mat(view_proj),
            light_vp: mat(&frame.shadow_view_proj.unwrap_or(Mat4::IDENTITY)),
            eye_time: vec4(eye, frame.time),
            fill_lines: vec4(scene::fill_direction(), frame.layer_lines),
            key_ldr: vec4(scene::key_direction(), scene_ldr),
            viewport: [
                viewport.x as f32,
                viewport.y as f32,
                viewport.width as f32,
                viewport.height as f32,
            ],
            ..Default::default()
        };
        if frame.has_shadow {
            uniform.shadow = [
                1.0,
                1.0 / frame.level.shadow_resolution().max(1) as f32,
                frame.shadow_world_texel,
                frame.level.shadow_taps() as f32,
            ];
        }
        if let Some((min, max)) = frame.bounds {
            uniform.floor_plane =
                vec4(scene::floor_centre(min, max), scene::floor_radius(min, max));
        }
        uniform
    }

    fn draw_shadow_map(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        encoder: &mut wgpu::CommandEncoder,
        level: Quality,
        view_proj: Option<Mat4>,
    ) -> bool {
        let Some(view_proj) = view_proj else { return false };
        let size = level.shadow_resolution();
        if !self.shadow.ensure(device, size, size) {
            return false;
        }
        let Some(depth) = self.shadow.depth_view() else { return false };
        let Some(vertices) = self.vertices.as_ref() else { return false };

        let uniform = SceneUniform { view_proj: mat(&view_proj), ..Default::default() };
        let offset = self.arenas.scene.push(queue, &uniform);
        let bind_group =
            scene_bind_group(device, &self.pipelines, &self.arenas.scene, offset, None, None);

        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("shadow"),
            color_attachments: &[],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: depth,
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(1.0),
                    store: wgpu::StoreOp::Store,
                }),
                stencil_ops: None,
            }),
            timestamp_writes: None,
            occlusion_query_set: None,
            multiview_mask: None,
        });
        pass.set_pipeline(&self.pipelines.shadow);
        pass.set_bind_group(0, &bind_group, &[]);
        pass.set_vertex_buffer(0, vertices.slice(..));
        pass.draw(0..self.vertex_count, 0..1);
        true
    }

    #[allow(clippy::too_many_arguments)]
    fn draw_occlusion(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        encoder: &mut wgpu::CommandEncoder,
        frame: &Frame,
        viewport: Viewport,
        view_proj: &Mat4,
        eye: Vec3,
        bounce: bool,
    ) -> bool {
        let Some((min, max)) = frame.bounds else { return false };
        let divisor = frame.level.occlusion_divisor();
        let width = (viewport.width / divisor).max(1);
        let height = (viewport.height / divisor).max(1);

        if !self.prepass.ensure(device, width, height)
            || !self.occlusion_target.ensure(device, width, height)
            || !self.occlusion_blur.ensure(device, width, height)
        {
            return false;
        }
        let Some(vertices) = self.vertices.as_ref() else { return false };

        let prepass_uniform =
            SceneUniform { view_proj: mat(view_proj), ..Default::default() };
        let prepass_offset = self.arenas.scene.push(queue, &prepass_uniform);
        let prepass_bind = scene_bind_group(
            device,
            &self.pipelines,
            &self.arenas.scene,
            prepass_offset,
            None,
            None,
        );
        {
            let Some(colour) = self.prepass.colour_view() else { return false };
            let Some(depth) = self.prepass.depth_view() else { return false };
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("depth-normal"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: colour,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.5,
                            g: 0.5,
                            b: 1.0,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: depth,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            pass.set_pipeline(&self.pipelines.depth_normal);
            pass.set_bind_group(0, &prepass_bind, &[]);
            pass.set_vertex_buffer(0, vertices.slice(..));
            pass.draw(0..self.vertex_count, 0..1);
        }

        let radius = (scene::scene_radius(min, max) * OCCLUSION_RADIUS_FRACTION)
            .clamp(OCCLUSION_RADIUS_RANGE.0, OCCLUSION_RADIUS_RANGE.1);
        let mut uniform = PostUniform {
            view_proj: mat(view_proj),
            inv_view_proj: mat(&view_proj.inverse()),
            eye: vec4(eye, 0.0),
            target_size: [width as f32, height as f32],
            near_far: [frame.near_far.0, frame.near_far.1],
            ..Default::default()
        };
        uniform.params[0] = radius;
        uniform.flags[0] = frame.level.occlusion_samples() as f32;
        uniform.flags[1] = self.accumulated as f32;
        uniform.flags[2] = if bounce { 1.0 } else { 0.0 };
        let offset = self.arenas.post.push(queue, &uniform);
        {
            let Some(normal) = self.prepass.colour_view() else { return false };
            let Some(depth) = self.prepass.depth_view() else { return false };
            let previous =
                if bounce { self.accumulation.colour_view() } else { None };
            let bind_group = post_bind_group(
                device,
                &self.pipelines,
                &self.arenas.post,
                offset,
                None,
                Some(normal),
                previous,
                Some(depth),
            );
            let Some(target) = self.occlusion_target.colour_view() else { return false };
            let mut pass = colour_pass(encoder, "occlusion", target, None);
            pass.set_pipeline(&self.pipelines.occlusion);
            pass.set_bind_group(0, &bind_group, &[]);
            pass.draw(0..3, 0..1);
        }

        let tolerance = scene::scene_radius(min, max) * OCCLUSION_DEPTH_TOLERANCE_FRACTION;
        for (index, direction) in [(0usize, [1.0f32, 0.0f32]), (1, [0.0, 1.0])] {
            let mut blur = PostUniform {
                target_size: [width as f32, height as f32],
                direction,
                near_far: [frame.near_far.0, frame.near_far.1],
                ..Default::default()
            };
            blur.params[1] = tolerance;
            let offset = self.arenas.post.push(queue, &blur);
            let Some(depth) = self.prepass.depth_view() else { return false };
            let (source, target) = if index == 0 {
                (self.occlusion_target.colour_view(), self.occlusion_blur.colour_view())
            } else {
                (self.occlusion_blur.colour_view(), self.occlusion_target.colour_view())
            };
            let (Some(source), Some(target)) = (source, target) else { return false };
            let bind_group = post_bind_group(
                device,
                &self.pipelines,
                &self.arenas.post,
                offset,
                Some(source),
                None,
                None,
                Some(depth),
            );
            let mut pass = colour_pass(encoder, "occlusion-blur", target, None);
            pass.set_pipeline(&self.pipelines.bilateral);
            pass.set_bind_group(0, &bind_group, &[]);
            pass.draw(0..3, 0..1);
        }
        true
    }

    fn draw_reflection(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        encoder: &mut wgpu::CommandEncoder,
        frame: &Frame,
        viewport: Viewport,
    ) -> bool {
        let Some((min, _)) = frame.bounds else { return false };
        let (width, height) = (viewport.width, viewport.height);
        let mirror = scene::mirror_about_height(min.z);
        let mirrored_view_proj = frame.view_proj * mirror;
        let mirrored_eye = mirror.transform_point3(frame.eye);

        let occluded = frame.level.ambient_occlusion()
            && self.draw_occlusion(
                device,
                queue,
                encoder,
                frame,
                viewport,
                &mirrored_view_proj,
                mirrored_eye,
                false,
            );

        if !self.reflection.ensure(device, width, height) {
            return false;
        }
        let mut uniform =
            self.scene_uniform(frame, Viewport::new(0, 0, width, height), &mirrored_view_proj, mirrored_eye, 0.0);
        uniform.toggles[0] = if occluded { 1.0 } else { 0.0 };
        let offset = self.arenas.scene.push(queue, &uniform);
        {
            let Some(vertices) = self.vertices.as_ref() else { return false };
            let screen = if occluded { self.occlusion_target.colour_view() } else { None };
            let shadow = if frame.has_shadow { self.shadow.depth_view() } else { None };
            let bind_group = scene_bind_group(
                device,
                &self.pipelines,
                &self.arenas.scene,
                offset,
                shadow,
                screen,
            );
            let Some(colour) = self.reflection.colour_view() else { return false };
            let Some(depth) = self.reflection.depth_view() else { return false };
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("reflection"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: colour,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color::TRANSPARENT),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: depth,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            pass.set_pipeline(&self.pipelines.mesh_reflection);
            pass.set_bind_group(0, &bind_group, &[]);
            pass.set_vertex_buffer(0, vertices.slice(..));
            pass.draw(0..self.vertex_count, 0..1);
        }

        self.gaussian_blur(
            device,
            queue,
            encoder,
            Source::Reflection,
            width,
            height,
            scene::REFLECTION_GLOSS_RADIUS,
        )
    }

    fn draw_bloom(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        encoder: &mut wgpu::CommandEncoder,
        level: Quality,
        width: i32,
        height: i32,
    ) -> bool {
        let bright = level.bloom_bright_divisor();
        let blur = level.bloom_blur_divisor();
        let (bw, bh) = ((width / bright).max(1), (height / bright).max(1));
        let (gw, gh) = ((width / blur).max(1), (height / blur).max(1));

        if !self.bloom_source.ensure(device, bw, bh) {
            return false;
        }
        let uniform =
            PostUniform { target_size: [bw as f32, bh as f32], ..Default::default() };
        let offset = self.arenas.post.push(queue, &uniform);
        {
            let Some(source) = self.accumulation.colour_view() else { return false };
            let bind_group = post_bind_group(
                device,
                &self.pipelines,
                &self.arenas.post,
                offset,
                Some(source),
                None,
                None,
                None,
            );
            let Some(target) = self.bloom_source.colour_view() else { return false };
            let mut pass = colour_pass(encoder, "bloom-bright", target, None);
            pass.set_pipeline(&self.pipelines.bloom_bright);
            pass.set_bind_group(0, &bind_group, &[]);
            pass.draw(0..3, 0..1);
        }

        self.gaussian_blur(
            device,
            queue,
            encoder,
            Source::Bloom,
            gw,
            gh,
            scene::BLOOM_BLUR_RADIUS,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn gaussian_blur(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        encoder: &mut wgpu::CommandEncoder,
        source: Source,
        width: i32,
        height: i32,
        radius: f32,
    ) -> bool {
        let (ping, pong) = match source {
            Source::Reflection => (&mut self.reflection_ping, &mut self.reflection_pong),
            Source::Bloom => (&mut self.bloom_ping, &mut self.bloom_pong),
        };
        if !ping.ensure(device, width, height) || !pong.ensure(device, width, height) {
            return false;
        }
        for (index, direction) in [(0usize, [1.0f32, 0.0f32]), (1, [0.0, 1.0])] {
            let mut uniform = PostUniform {
                target_size: [width as f32, height as f32],
                direction,
                ..Default::default()
            };
            uniform.params[0] = radius;
            let offset = self.arenas.post.push(queue, &uniform);
            let (input, output) = match (source, index) {
                (Source::Reflection, 0) => {
                    (self.reflection.colour_view(), self.reflection_ping.colour_view())
                }
                (Source::Reflection, _) => {
                    (self.reflection_ping.colour_view(), self.reflection_pong.colour_view())
                }
                (Source::Bloom, 0) => {
                    (self.bloom_source.colour_view(), self.bloom_ping.colour_view())
                }
                (Source::Bloom, _) => {
                    (self.bloom_ping.colour_view(), self.bloom_pong.colour_view())
                }
            };
            let (Some(input), Some(output)) = (input, output) else { return false };
            let bind_group = post_bind_group(
                device,
                &self.pipelines,
                &self.arenas.post,
                offset,
                Some(input),
                None,
                None,
                None,
            );
            let mut pass = colour_pass(encoder, "gaussian-blur", output, None);
            pass.set_pipeline(&self.pipelines.gaussian);
            pass.set_bind_group(0, &bind_group, &[]);
            pass.draw(0..3, 0..1);
        }
        true
    }

    fn draw_scene(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        encoder: &mut wgpu::CommandEncoder,
        frame: &Frame,
        viewport: Viewport,
    ) {
        let backdrop = self.scene_uniform(frame, viewport, &frame.view_proj, frame.eye, frame.scene_ldr);
        let backdrop_offset = self.arenas.scene.push(queue, &backdrop);

        let mut floor = backdrop;
        floor.toggles[1] = frame.floor_presence;
        floor.toggles[2] = if frame.has_reflection { 1.0 } else { 0.0 };
        let floor_offset = self.arenas.scene.push(queue, &floor);

        let mut mesh = backdrop;
        mesh.toggles[0] = if frame.has_occlusion { 1.0 } else { 0.0 };
        let mesh_offset = self.arenas.scene.push(queue, &mesh);

        let line = LineUniform {
            view_proj: mat(&frame.view_proj),
            half_vp: [viewport.width as f32 * 0.5, viewport.height as f32 * 0.5],
            width: LINE_WIDTH_PX,
            alpha: 1.0,
            settings: [frame.scene_ldr, 0.0, 0.0, 0.0],
        };
        let near_offset = self.arenas.line.push(queue, &line);
        let far = LineUniform { alpha: 0.22, ..line };
        let far_offset = self.arenas.line.push(queue, &far);

        let shadow = if frame.has_shadow { self.shadow.depth_view() } else { None };
        let reflection =
            if frame.has_reflection { self.reflection_pong.colour_view() } else { None };
        let occlusion = if frame.has_occlusion { self.occlusion_target.colour_view() } else { None };

        let backdrop_bind = scene_bind_group(
            device,
            &self.pipelines,
            &self.arenas.scene,
            backdrop_offset,
            shadow,
            None,
        );
        let floor_bind = scene_bind_group(
            device,
            &self.pipelines,
            &self.arenas.scene,
            floor_offset,
            shadow,
            reflection,
        );
        let mesh_bind = scene_bind_group(
            device,
            &self.pipelines,
            &self.arenas.scene,
            mesh_offset,
            shadow,
            occlusion,
        );
        let near_bind = line_bind_group(device, &self.pipelines, &self.arenas.line, near_offset);
        let far_bind = line_bind_group(device, &self.pipelines, &self.arenas.line, far_offset);

        let Some(colour) = self.scene.colour_view() else { return };
        let Some(depth) = self.scene.depth_view() else { return };
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("scene"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: colour,
                depth_slice: None,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: depth,
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(1.0),
                    store: wgpu::StoreOp::Store,
                }),
                stencil_ops: None,
            }),
            timestamp_writes: None,
            occlusion_query_set: None,
            multiview_mask: None,
        });

        pass.set_pipeline(&self.pipelines.backdrop);
        pass.set_bind_group(0, &backdrop_bind, &[]);
        pass.draw(0..3, 0..1);

        if frame.bounds.is_some() {
            pass.set_pipeline(&self.pipelines.floor);
            pass.set_bind_group(0, &floor_bind, &[]);
            pass.draw(0..4, 0..1);
        }

        let Some(vertices) = self.vertices.as_ref() else { return };
        if self.vertex_count == 0 {
            return;
        }
        let biased = self.line_count > 0;
        pass.set_pipeline(if biased {
            &self.pipelines.mesh_biased
        } else {
            &self.pipelines.mesh
        });
        pass.set_bind_group(0, &mesh_bind, &[]);
        pass.set_vertex_buffer(0, vertices.slice(..));
        pass.draw(0..self.vertex_count, 0..1);

        if let Some(lines) = self.lines.as_ref() {
            if self.line_count > 0 {
                pass.set_vertex_buffer(0, lines.slice(..));
                pass.set_pipeline(&self.pipelines.line_near);
                pass.set_bind_group(0, &near_bind, &[]);
                pass.draw(0..self.line_count, 0..1);
                pass.set_pipeline(&self.pipelines.line_far);
                pass.set_bind_group(0, &far_bind, &[]);
                pass.draw(0..self.line_count, 0..1);
            }
        }
    }

    fn accumulate(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        encoder: &mut wgpu::CommandEncoder,
        width: i32,
        height: i32,
    ) {
        let weight = 1.0 / (self.accumulated + 1) as f32;
        let uniform =
            PostUniform { target_size: [width as f32, height as f32], ..Default::default() };
        let offset = self.arenas.post.push(queue, &uniform);
        let Some(source) = self.scene.colour_view() else { return };
        let bind_group = post_bind_group(
            device,
            &self.pipelines,
            &self.arenas.post,
            offset,
            Some(source),
            None,
            None,
            None,
        );
        let Some(target) = self.accumulation.colour_view() else { return };
        let mut pass = colour_pass(encoder, "accumulate", target, Some(wgpu::LoadOp::Load));
        pass.set_pipeline(&self.pipelines.accumulate);
        pass.set_blend_constant(wgpu::Color {
            r: weight as f64,
            g: weight as f64,
            b: weight as f64,
            a: weight as f64,
        });
        pass.set_bind_group(0, &bind_group, &[]);
        pass.draw(0..3, 0..1);
    }

    #[allow(clippy::too_many_arguments)]
    fn resolve(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        encoder: &mut wgpu::CommandEncoder,
        frame: &Frame,
        width: i32,
        height: i32,
        bloom: bool,
    ) {
        let mut uniform = PostUniform {
            target_size: [width as f32, height as f32],
            near_far: [frame.near_far.0, frame.near_far.1],
            ..Default::default()
        };
        uniform.params[2] = frame.focus_distance;
        uniform.params[3] = frame.aperture;
        uniform.flags[2] = if bloom { 1.0 } else { 0.0 };
        uniform.flags[3] = if self.hdr { 1.0 } else { 0.0 };
        let offset = self.arenas.post.push(queue, &uniform);

        let Some(source) = self.accumulation.colour_view() else { return };
        let bloom_view = if bloom { self.bloom_pong.colour_view() } else { None };
        let depth = self.scene.depth_view();
        let bind_group = post_bind_group(
            device,
            &self.pipelines,
            &self.arenas.post,
            offset,
            Some(source),
            bloom_view,
            None,
            depth,
        );
        let Some(target) = self.resolved.colour_view() else { return };
        let mut pass = colour_pass(encoder, "resolve", target, None);
        pass.set_pipeline(&self.pipelines.resolve);
        pass.set_bind_group(0, &bind_group, &[]);
        pass.draw(0..3, 0..1);
    }

    pub fn destroy(&mut self) {
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
            target.release();
        }
        self.vertices = None;
        self.lines = None;
        self.vertex_count = 0;
        self.line_count = 0;
        self.presented = None;
    }
}

#[derive(Clone, Copy)]
enum Source {
    Reflection,
    Bloom,
}

fn colour_pass<'a>(
    encoder: &'a mut wgpu::CommandEncoder,
    label: &str,
    view: &'a wgpu::TextureView,
    load: Option<wgpu::LoadOp<wgpu::Color>>,
) -> wgpu::RenderPass<'a> {
    encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
        label: Some(label),
        color_attachments: &[Some(wgpu::RenderPassColorAttachment {
            view,
            depth_slice: None,
            resolve_target: None,
            ops: wgpu::Operations {
                load: load.unwrap_or(wgpu::LoadOp::Clear(wgpu::Color::TRANSPARENT)),
                store: wgpu::StoreOp::Store,
            },
        })],
        depth_stencil_attachment: None,
        timestamp_writes: None,
        occlusion_query_set: None,
        multiview_mask: None,
    })
}

fn scene_bind_group(
    device: &wgpu::Device,
    pipelines: &Pipelines,
    arena: &Arena,
    offset: u64,
    shadow: Option<&wgpu::TextureView>,
    screen: Option<&wgpu::TextureView>,
) -> wgpu::BindGroup {
    device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("scene"),
        layout: &pipelines.scene_layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: arena.binding(offset, size_of::<SceneUniform>()),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: wgpu::BindingResource::TextureView(
                    shadow.unwrap_or(&pipelines.dummy_depth),
                ),
            },
            wgpu::BindGroupEntry {
                binding: 2,
                resource: wgpu::BindingResource::Sampler(&pipelines.shadow_sampler),
            },
            wgpu::BindGroupEntry {
                binding: 3,
                resource: wgpu::BindingResource::TextureView(
                    screen.unwrap_or(&pipelines.dummy_colour),
                ),
            },
            wgpu::BindGroupEntry {
                binding: 4,
                resource: wgpu::BindingResource::Sampler(&pipelines.linear_sampler),
            },
        ],
    })
}

#[allow(clippy::too_many_arguments)]
fn post_bind_group(
    device: &wgpu::Device,
    pipelines: &Pipelines,
    arena: &Arena,
    offset: u64,
    source: Option<&wgpu::TextureView>,
    aux: Option<&wgpu::TextureView>,
    previous: Option<&wgpu::TextureView>,
    depth: Option<&wgpu::TextureView>,
) -> wgpu::BindGroup {
    device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("post"),
        layout: &pipelines.post_layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: arena.binding(offset, size_of::<PostUniform>()),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: wgpu::BindingResource::TextureView(
                    source.unwrap_or(&pipelines.dummy_colour),
                ),
            },
            wgpu::BindGroupEntry {
                binding: 2,
                resource: wgpu::BindingResource::Sampler(&pipelines.linear_sampler),
            },
            wgpu::BindGroupEntry {
                binding: 3,
                resource: wgpu::BindingResource::TextureView(
                    aux.unwrap_or(&pipelines.dummy_colour),
                ),
            },
            wgpu::BindGroupEntry {
                binding: 4,
                resource: wgpu::BindingResource::TextureView(
                    previous.unwrap_or(&pipelines.dummy_colour),
                ),
            },
            wgpu::BindGroupEntry {
                binding: 5,
                resource: wgpu::BindingResource::TextureView(
                    depth.unwrap_or(&pipelines.dummy_depth),
                ),
            },
            wgpu::BindGroupEntry {
                binding: 6,
                resource: wgpu::BindingResource::Sampler(&pipelines.depth_sampler),
            },
        ],
    })
}

fn line_bind_group(
    device: &wgpu::Device,
    pipelines: &Pipelines,
    arena: &Arena,
    offset: u64,
) -> wgpu::BindGroup {
    device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("line"),
        layout: &pipelines.line_layout,
        entries: &[wgpu::BindGroupEntry {
            binding: 0,
            resource: arena.binding(offset, size_of::<LineUniform>()),
        }],
    })
}

fn write_vertices(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    existing: Option<wgpu::Buffer>,
    verts: &[f32],
    label: &str,
) -> Option<wgpu::Buffer> {
    if verts.is_empty() {
        return existing;
    }
    let bytes: &[u8] = bytemuck::cast_slice(verts);
    let size = bytes.len() as wgpu::BufferAddress;
    let buffer = match existing {
        Some(buffer) if buffer.size() >= size => buffer,
        _ => device.create_buffer(&wgpu::BufferDescriptor {
            label: Some(label),
            size: size.next_power_of_two(),
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        }),
    };
    queue.write_buffer(&buffer, 0, bytes);
    Some(buffer)
}
