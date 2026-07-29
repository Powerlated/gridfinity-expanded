pub const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth24Plus;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Attachments {
    ColourDepth(wgpu::TextureFormat),
    Colour(wgpu::TextureFormat),
    ShadowDepth,
}

pub struct Target {
    attachments: Attachments,
    colour: Option<wgpu::TextureView>,
    depth: Option<wgpu::TextureView>,
    width: u32,
    height: u32,
    generation: u64,
}

impl Target {
    pub fn new(attachments: Attachments) -> Target {
        Target { attachments, colour: None, depth: None, width: 0, height: 0, generation: 0 }
    }

    pub fn colour_view(&self) -> Option<&wgpu::TextureView> {
        self.colour.as_ref()
    }

    pub fn depth_view(&self) -> Option<&wgpu::TextureView> {
        self.depth.as_ref()
    }

    pub fn size(&self) -> (u32, u32) {
        (self.width, self.height)
    }

    pub fn generation(&self) -> u64 {
        self.generation
    }

    pub fn ensure(&mut self, device: &wgpu::Device, width: i32, height: i32) -> bool {
        let width = width.max(1) as u32;
        let height = height.max(1) as u32;
        if self.width == width && self.height == height && (self.colour.is_some() || self.depth.is_some()) {
            return true;
        }
        self.colour = None;
        self.depth = None;
        match self.attachments {
            Attachments::ColourDepth(format) => {
                self.colour = Some(view(device, format, width, height));
                self.depth = Some(view(device, DEPTH_FORMAT, width, height));
            }
            Attachments::Colour(format) => {
                self.colour = Some(view(device, format, width, height));
            }
            Attachments::ShadowDepth => {
                self.depth = Some(view(device, DEPTH_FORMAT, width, height));
            }
        }
        self.width = width;
        self.height = height;
        self.generation = self.generation.wrapping_add(1);
        true
    }

    pub fn release(&mut self) {
        self.colour = None;
        self.depth = None;
        self.width = 0;
        self.height = 0;
        self.generation = self.generation.wrapping_add(1);
    }
}

fn view(
    device: &wgpu::Device,
    format: wgpu::TextureFormat,
    width: u32,
    height: u32,
) -> wgpu::TextureView {
    let texture = device.create_texture(&wgpu::TextureDescriptor {
        label: None,
        size: wgpu::Extent3d { width, height, depth_or_array_layers: 1 },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
        view_formats: &[],
    });
    texture.create_view(&wgpu::TextureViewDescriptor::default())
}

pub fn supports_float_colour(adapter: &wgpu::Adapter) -> bool {
    let features = adapter.get_texture_format_features(wgpu::TextureFormat::Rgba16Float);
    features.allowed_usages.contains(wgpu::TextureUsages::RENDER_ATTACHMENT)
        && features.flags.contains(wgpu::TextureFormatFeatureFlags::BLENDABLE)
        && features.flags.contains(wgpu::TextureFormatFeatureFlags::FILTERABLE)
}
