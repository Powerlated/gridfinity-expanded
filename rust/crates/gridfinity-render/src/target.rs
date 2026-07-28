use glow::HasContext;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Attachments {
    ColourDepth(u32),
    Colour(u32),
    ShadowDepth,
}

pub struct Target {
    fbo: Option<glow::Framebuffer>,
    colour: Option<glow::Texture>,
    depth: Option<glow::Texture>,
    attachments: Attachments,
    width: i32,
    height: i32,
}

impl Target {
    pub fn new(attachments: Attachments) -> Target {
        Target { fbo: None, colour: None, depth: None, attachments, width: 0, height: 0 }
    }

    pub fn colour_texture(&self) -> Option<glow::Texture> {
        self.colour
    }

    pub fn depth_texture(&self) -> Option<glow::Texture> {
        self.depth
    }

    pub fn size(&self) -> (i32, i32) {
        (self.width, self.height)
    }

    pub fn ensure(&mut self, gl: &glow::Context, width: i32, height: i32) -> Option<glow::Framebuffer> {
        let width = width.max(1);
        let height = height.max(1);
        if self.fbo.is_some() && self.width == width && self.height == height {
            return self.fbo;
        }
        self.release(gl);
        unsafe {
            let fbo = gl.create_framebuffer().ok()?;
            gl.bind_framebuffer(glow::FRAMEBUFFER, Some(fbo));
            match self.attachments {
                Attachments::ColourDepth(format) => {
                    self.colour = Some(colour_texture(gl, format, width, height)?);
                    self.depth = Some(depth_texture(gl, width, height, false)?);
                }
                Attachments::Colour(format) => {
                    self.colour = Some(colour_texture(gl, format, width, height)?);
                }
                Attachments::ShadowDepth => {
                    self.depth = Some(depth_texture(gl, width, height, true)?);
                }
            }
            if let Some(colour) = self.colour {
                gl.framebuffer_texture_2d(
                    glow::FRAMEBUFFER,
                    glow::COLOR_ATTACHMENT0,
                    glow::TEXTURE_2D,
                    Some(colour),
                    0,
                );
                gl.draw_buffers(&[glow::COLOR_ATTACHMENT0]);
            } else {
                gl.draw_buffers(&[glow::NONE]);
                gl.read_buffer(glow::NONE);
            }
            if let Some(depth) = self.depth {
                gl.framebuffer_texture_2d(
                    glow::FRAMEBUFFER,
                    glow::DEPTH_ATTACHMENT,
                    glow::TEXTURE_2D,
                    Some(depth),
                    0,
                );
            }
            let complete =
                gl.check_framebuffer_status(glow::FRAMEBUFFER) == glow::FRAMEBUFFER_COMPLETE;
            gl.bind_framebuffer(glow::FRAMEBUFFER, None);
            if !complete {
                self.fbo = Some(fbo);
                self.release(gl);
                return None;
            }
            self.fbo = Some(fbo);
            self.width = width;
            self.height = height;
        }
        self.fbo
    }

    pub fn release(&mut self, gl: &glow::Context) {
        unsafe {
            if let Some(fbo) = self.fbo.take() {
                gl.delete_framebuffer(fbo);
            }
            if let Some(texture) = self.colour.take() {
                gl.delete_texture(texture);
            }
            if let Some(texture) = self.depth.take() {
                gl.delete_texture(texture);
            }
        }
        self.width = 0;
        self.height = 0;
    }
}

unsafe fn colour_texture(
    gl: &glow::Context,
    format: u32,
    width: i32,
    height: i32,
) -> Option<glow::Texture> {
    unsafe {
        let texture = gl.create_texture().ok()?;
        gl.bind_texture(glow::TEXTURE_2D, Some(texture));
        gl.tex_storage_2d(glow::TEXTURE_2D, 1, format, width, height);
        set_filter(gl, glow::LINEAR);
        gl.bind_texture(glow::TEXTURE_2D, None);
        Some(texture)
    }
}

unsafe fn depth_texture(
    gl: &glow::Context,
    width: i32,
    height: i32,
    comparison: bool,
) -> Option<glow::Texture> {
    unsafe {
        let texture = gl.create_texture().ok()?;
        gl.bind_texture(glow::TEXTURE_2D, Some(texture));
        gl.tex_storage_2d(glow::TEXTURE_2D, 1, glow::DEPTH_COMPONENT24, width, height);
        set_filter(gl, if comparison { glow::LINEAR } else { glow::NEAREST });
        if comparison {
            gl.tex_parameter_i32(
                glow::TEXTURE_2D,
                glow::TEXTURE_COMPARE_MODE,
                glow::COMPARE_REF_TO_TEXTURE as i32,
            );
            gl.tex_parameter_i32(
                glow::TEXTURE_2D,
                glow::TEXTURE_COMPARE_FUNC,
                glow::LEQUAL as i32,
            );
        }
        gl.bind_texture(glow::TEXTURE_2D, None);
        Some(texture)
    }
}

unsafe fn set_filter(gl: &glow::Context, filter: u32) {
    unsafe {
        gl.tex_parameter_i32(glow::TEXTURE_2D, glow::TEXTURE_MIN_FILTER, filter as i32);
        gl.tex_parameter_i32(glow::TEXTURE_2D, glow::TEXTURE_MAG_FILTER, filter as i32);
        gl.tex_parameter_i32(
            glow::TEXTURE_2D,
            glow::TEXTURE_WRAP_S,
            glow::CLAMP_TO_EDGE as i32,
        );
        gl.tex_parameter_i32(
            glow::TEXTURE_2D,
            glow::TEXTURE_WRAP_T,
            glow::CLAMP_TO_EDGE as i32,
        );
    }
}
