use crate::shaders;
use crate::target::DEPTH_FORMAT;
use crate::uniforms::{LineUniform, PostUniform, SceneUniform};
use crate::vertex::{LINE_STRIDE, VERTEX_STRIDE};
use std::collections::HashMap;
use std::num::NonZeroU64;

pub const PREPASS_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8Unorm;
pub const RESOLVED_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8Unorm;

static MESH_ATTRIBUTES: [wgpu::VertexAttribute; 4] = wgpu::vertex_attr_array![
    0 => Float32x3,
    1 => Float32x3,
    2 => Float32x3,
    3 => Float32,
];

static LINE_ATTRIBUTES: [wgpu::VertexAttribute; 5] = wgpu::vertex_attr_array![
    0 => Float32x3,
    1 => Float32x3,
    2 => Float32x3,
    3 => Float32,
    4 => Float32,
];

fn mesh_buffer() -> wgpu::VertexBufferLayout<'static> {
    wgpu::VertexBufferLayout {
        array_stride: (VERTEX_STRIDE * size_of::<f32>()) as wgpu::BufferAddress,
        step_mode: wgpu::VertexStepMode::Vertex,
        attributes: &MESH_ATTRIBUTES,
    }
}

fn line_buffer() -> wgpu::VertexBufferLayout<'static> {
    wgpu::VertexBufferLayout {
        array_stride: (LINE_STRIDE * size_of::<f32>()) as wgpu::BufferAddress,
        step_mode: wgpu::VertexStepMode::Vertex,
        attributes: &LINE_ATTRIBUTES,
    }
}

const ALPHA_BLEND: wgpu::BlendState = wgpu::BlendState {
    color: wgpu::BlendComponent {
        src_factor: wgpu::BlendFactor::SrcAlpha,
        dst_factor: wgpu::BlendFactor::OneMinusSrcAlpha,
        operation: wgpu::BlendOperation::Add,
    },
    alpha: wgpu::BlendComponent {
        src_factor: wgpu::BlendFactor::SrcAlpha,
        dst_factor: wgpu::BlendFactor::OneMinusSrcAlpha,
        operation: wgpu::BlendOperation::Add,
    },
};

const RUNNING_MEAN_BLEND: wgpu::BlendState = wgpu::BlendState {
    color: wgpu::BlendComponent {
        src_factor: wgpu::BlendFactor::Constant,
        dst_factor: wgpu::BlendFactor::OneMinusConstant,
        operation: wgpu::BlendOperation::Add,
    },
    alpha: wgpu::BlendComponent {
        src_factor: wgpu::BlendFactor::Constant,
        dst_factor: wgpu::BlendFactor::OneMinusConstant,
        operation: wgpu::BlendOperation::Add,
    },
};

fn depth_state(write: bool, compare: wgpu::CompareFunction, biased: bool) -> wgpu::DepthStencilState {
    wgpu::DepthStencilState {
        format: DEPTH_FORMAT,
        depth_write_enabled: Some(write),
        depth_compare: Some(compare),
        stencil: wgpu::StencilState::default(),
        bias: if biased {
            wgpu::DepthBiasState { constant: 1, slope_scale: 1.0, clamp: 0.0 }
        } else {
            wgpu::DepthBiasState::default()
        },
    }
}

fn primitive(
    topology: wgpu::PrimitiveTopology,
    cull: Option<wgpu::Face>,
) -> wgpu::PrimitiveState {
    wgpu::PrimitiveState {
        topology,
        strip_index_format: None,
        front_face: wgpu::FrontFace::Ccw,
        cull_mode: cull,
        unclipped_depth: false,
        polygon_mode: wgpu::PolygonMode::Fill,
        conservative: false,
    }
}

struct Blit {
    copy: wgpu::RenderPipeline,
    fxaa: wgpu::RenderPipeline,
}

pub struct Pipelines {
    pub scene_layout: wgpu::BindGroupLayout,
    pub post_layout: wgpu::BindGroupLayout,
    pub line_layout: wgpu::BindGroupLayout,

    pub mesh: wgpu::RenderPipeline,
    pub mesh_biased: wgpu::RenderPipeline,
    pub mesh_reflection: wgpu::RenderPipeline,
    pub depth_normal: wgpu::RenderPipeline,
    pub shadow: wgpu::RenderPipeline,
    pub backdrop: wgpu::RenderPipeline,
    pub floor: wgpu::RenderPipeline,
    pub line_near: wgpu::RenderPipeline,
    pub line_far: wgpu::RenderPipeline,

    pub occlusion: wgpu::RenderPipeline,
    pub bilateral: wgpu::RenderPipeline,
    pub gaussian: wgpu::RenderPipeline,
    pub bloom_bright: wgpu::RenderPipeline,
    pub resolve: wgpu::RenderPipeline,
    pub accumulate: wgpu::RenderPipeline,

    pub linear_sampler: wgpu::Sampler,
    pub depth_sampler: wgpu::Sampler,
    pub shadow_sampler: wgpu::Sampler,
    pub dummy_colour: wgpu::TextureView,
    pub dummy_depth: wgpu::TextureView,

    post_module: wgpu::ShaderModule,
    post_pipeline_layout: wgpu::PipelineLayout,
    blits: HashMap<wgpu::TextureFormat, Blit>,
}

impl Pipelines {
    pub fn new(device: &wgpu::Device, colour: wgpu::TextureFormat) -> Pipelines {
        let scene_module = module(device, "scene", shaders::scene_module());
        let post_module = module(device, "post", shaders::post_module());
        let line_module = module(device, "line", shaders::line_module());

        let scene_layout = scene_bind_group_layout(device);
        let post_layout = post_bind_group_layout(device);
        let line_layout = line_bind_group_layout(device);

        let scene_pipeline_layout = pipeline_layout(device, "scene", &scene_layout);
        let post_pipeline_layout = pipeline_layout(device, "post", &post_layout);
        let line_pipeline_layout = pipeline_layout(device, "line", &line_layout);

        let mesh_pipeline = |biased: bool, cull: wgpu::Face| {
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some("mesh"),
                layout: Some(&scene_pipeline_layout),
                vertex: wgpu::VertexState {
                    module: &scene_module,
                    entry_point: Some("vs_mesh"),
                    compilation_options: Default::default(),
                    buffers: &[mesh_buffer()],
                },
                fragment: Some(wgpu::FragmentState {
                    module: &scene_module,
                    entry_point: Some("fs_mesh"),
                    compilation_options: Default::default(),
                    targets: &[Some(wgpu::ColorTargetState {
                        format: colour,
                        blend: None,
                        write_mask: wgpu::ColorWrites::ALL,
                    })],
                }),
                primitive: primitive(wgpu::PrimitiveTopology::TriangleList, Some(cull)),
                depth_stencil: Some(depth_state(true, wgpu::CompareFunction::Less, biased)),
                multisample: wgpu::MultisampleState::default(),
                multiview_mask: None,
                cache: None,
            })
        };

        let fullscreen = |label: &str, entry: &str, format: wgpu::TextureFormat, blend| {
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(label),
                layout: Some(&post_pipeline_layout),
                vertex: wgpu::VertexState {
                    module: &post_module,
                    entry_point: Some("vs_fullscreen"),
                    compilation_options: Default::default(),
                    buffers: &[],
                },
                fragment: Some(wgpu::FragmentState {
                    module: &post_module,
                    entry_point: Some(entry),
                    compilation_options: Default::default(),
                    targets: &[Some(wgpu::ColorTargetState {
                        format,
                        blend,
                        write_mask: wgpu::ColorWrites::ALL,
                    })],
                }),
                primitive: primitive(wgpu::PrimitiveTopology::TriangleList, None),
                depth_stencil: None,
                multisample: wgpu::MultisampleState::default(),
                multiview_mask: None,
                cache: None,
            })
        };

        let line_pipeline = |label: &str, compare: wgpu::CompareFunction| {
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(label),
                layout: Some(&line_pipeline_layout),
                vertex: wgpu::VertexState {
                    module: &line_module,
                    entry_point: Some("vs_line"),
                    compilation_options: Default::default(),
                    buffers: &[line_buffer()],
                },
                fragment: Some(wgpu::FragmentState {
                    module: &line_module,
                    entry_point: Some("fs_line"),
                    compilation_options: Default::default(),
                    targets: &[Some(wgpu::ColorTargetState {
                        format: colour,
                        blend: Some(ALPHA_BLEND),
                        write_mask: wgpu::ColorWrites::ALL,
                    })],
                }),
                primitive: primitive(wgpu::PrimitiveTopology::TriangleList, None),
                depth_stencil: Some(depth_state(false, compare, false)),
                multisample: wgpu::MultisampleState::default(),
                multiview_mask: None,
                cache: None,
            })
        };

        let mesh = mesh_pipeline(false, wgpu::Face::Back);
        let mesh_biased = mesh_pipeline(true, wgpu::Face::Back);
        let mesh_reflection = mesh_pipeline(false, wgpu::Face::Front);

        let depth_normal = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("depth-normal"),
            layout: Some(&scene_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &scene_module,
                entry_point: Some("vs_mesh"),
                compilation_options: Default::default(),
                buffers: &[mesh_buffer()],
            },
            fragment: Some(wgpu::FragmentState {
                module: &scene_module,
                entry_point: Some("fs_depth_normal"),
                compilation_options: Default::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format: PREPASS_FORMAT,
                    blend: None,
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            primitive: primitive(wgpu::PrimitiveTopology::TriangleList, Some(wgpu::Face::Back)),
            depth_stencil: Some(depth_state(true, wgpu::CompareFunction::Less, false)),
            multisample: wgpu::MultisampleState::default(),
            multiview_mask: None,
            cache: None,
        });

        let shadow = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("shadow"),
            layout: Some(&scene_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &scene_module,
                entry_point: Some("vs_mesh"),
                compilation_options: Default::default(),
                buffers: &[mesh_buffer()],
            },
            fragment: None,
            primitive: primitive(wgpu::PrimitiveTopology::TriangleList, Some(wgpu::Face::Front)),
            depth_stencil: Some(depth_state(true, wgpu::CompareFunction::Less, false)),
            multisample: wgpu::MultisampleState::default(),
            multiview_mask: None,
            cache: None,
        });

        let backdrop = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("backdrop"),
            layout: Some(&scene_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &scene_module,
                entry_point: Some("vs_backdrop"),
                compilation_options: Default::default(),
                buffers: &[],
            },
            fragment: Some(wgpu::FragmentState {
                module: &scene_module,
                entry_point: Some("fs_backdrop"),
                compilation_options: Default::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format: colour,
                    blend: None,
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            primitive: primitive(wgpu::PrimitiveTopology::TriangleList, None),
            depth_stencil: Some(depth_state(false, wgpu::CompareFunction::Always, false)),
            multisample: wgpu::MultisampleState::default(),
            multiview_mask: None,
            cache: None,
        });

        let floor = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("floor"),
            layout: Some(&scene_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &scene_module,
                entry_point: Some("vs_floor"),
                compilation_options: Default::default(),
                buffers: &[],
            },
            fragment: Some(wgpu::FragmentState {
                module: &scene_module,
                entry_point: Some("fs_floor"),
                compilation_options: Default::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format: colour,
                    blend: None,
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            primitive: primitive(wgpu::PrimitiveTopology::TriangleStrip, Some(wgpu::Face::Back)),
            depth_stencil: Some(depth_state(true, wgpu::CompareFunction::Less, false)),
            multisample: wgpu::MultisampleState::default(),
            multiview_mask: None,
            cache: None,
        });

        Pipelines {
            mesh,
            mesh_biased,
            mesh_reflection,
            depth_normal,
            shadow,
            backdrop,
            floor,
            line_near: line_pipeline("line-near", wgpu::CompareFunction::LessEqual),
            line_far: line_pipeline("line-far", wgpu::CompareFunction::Greater),
            occlusion: fullscreen("occlusion", "fs_occlusion", colour, None),
            bilateral: fullscreen("bilateral-blur", "fs_bilateral_blur", colour, None),
            gaussian: fullscreen("gaussian-blur", "fs_gaussian_blur", colour, None),
            bloom_bright: fullscreen("bloom-bright", "fs_bloom_bright", colour, None),
            resolve: fullscreen("resolve", "fs_resolve", RESOLVED_FORMAT, None),
            accumulate: fullscreen("accumulate", "fs_copy", colour, Some(RUNNING_MEAN_BLEND)),
            linear_sampler: sampler(device, wgpu::FilterMode::Linear, None),
            depth_sampler: sampler(device, wgpu::FilterMode::Nearest, None),
            shadow_sampler: sampler(
                device,
                wgpu::FilterMode::Linear,
                Some(wgpu::CompareFunction::LessEqual),
            ),
            dummy_colour: dummy(device, PREPASS_FORMAT),
            dummy_depth: dummy(device, DEPTH_FORMAT),
            scene_layout,
            post_layout,
            line_layout,
            post_module,
            post_pipeline_layout,
            blits: HashMap::new(),
        }
    }

    pub fn blit(
        &mut self,
        device: &wgpu::Device,
        format: wgpu::TextureFormat,
        antialias: bool,
    ) -> &wgpu::RenderPipeline {
        let entry = self.blits.entry(format).or_insert_with(|| {
            let build = |label: &str, entry_point: &str| {
                device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                    label: Some(label),
                    layout: Some(&self.post_pipeline_layout),
                    vertex: wgpu::VertexState {
                        module: &self.post_module,
                        entry_point: Some("vs_fullscreen"),
                        compilation_options: Default::default(),
                        buffers: &[],
                    },
                    fragment: Some(wgpu::FragmentState {
                        module: &self.post_module,
                        entry_point: Some(entry_point),
                        compilation_options: Default::default(),
                        targets: &[Some(wgpu::ColorTargetState {
                            format,
                            blend: None,
                            write_mask: wgpu::ColorWrites::ALL,
                        })],
                    }),
                    primitive: primitive(wgpu::PrimitiveTopology::TriangleList, None),
                    depth_stencil: None,
                    multisample: wgpu::MultisampleState::default(),
                    multiview_mask: None,
                    cache: None,
                })
            };
            Blit { copy: build("blit-copy", "fs_copy"), fxaa: build("blit-fxaa", "fs_fxaa") }
        });
        if antialias { &entry.fxaa } else { &entry.copy }
    }
}

fn module(device: &wgpu::Device, label: &str, source: String) -> wgpu::ShaderModule {
    device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some(label),
        source: wgpu::ShaderSource::Wgsl(source.into()),
    })
}

fn pipeline_layout(
    device: &wgpu::Device,
    label: &str,
    bind_group: &wgpu::BindGroupLayout,
) -> wgpu::PipelineLayout {
    device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some(label),
        bind_group_layouts: &[Some(bind_group)],
        immediate_size: 0,
    })
}

fn uniform_entry(binding: u32, size: usize) -> wgpu::BindGroupLayoutEntry {
    wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
        ty: wgpu::BindingType::Buffer {
            ty: wgpu::BufferBindingType::Uniform,
            has_dynamic_offset: false,
            min_binding_size: NonZeroU64::new(size as u64),
        },
        count: None,
    }
}

fn texture_entry(binding: u32, sample_type: wgpu::TextureSampleType) -> wgpu::BindGroupLayoutEntry {
    wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::FRAGMENT,
        ty: wgpu::BindingType::Texture {
            sample_type,
            view_dimension: wgpu::TextureViewDimension::D2,
            multisampled: false,
        },
        count: None,
    }
}

fn sampler_entry(binding: u32, ty: wgpu::SamplerBindingType) -> wgpu::BindGroupLayoutEntry {
    wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::FRAGMENT,
        ty: wgpu::BindingType::Sampler(ty),
        count: None,
    }
}

const FILTERABLE: wgpu::TextureSampleType =
    wgpu::TextureSampleType::Float { filterable: true };

fn scene_bind_group_layout(device: &wgpu::Device) -> wgpu::BindGroupLayout {
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("scene"),
        entries: &[
            uniform_entry(0, size_of::<SceneUniform>()),
            texture_entry(1, wgpu::TextureSampleType::Depth),
            sampler_entry(2, wgpu::SamplerBindingType::Comparison),
            texture_entry(3, FILTERABLE),
            sampler_entry(4, wgpu::SamplerBindingType::Filtering),
        ],
    })
}

fn post_bind_group_layout(device: &wgpu::Device) -> wgpu::BindGroupLayout {
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("post"),
        entries: &[
            uniform_entry(0, size_of::<PostUniform>()),
            texture_entry(1, FILTERABLE),
            sampler_entry(2, wgpu::SamplerBindingType::Filtering),
            texture_entry(3, FILTERABLE),
            texture_entry(4, FILTERABLE),
            texture_entry(5, wgpu::TextureSampleType::Depth),
            sampler_entry(6, wgpu::SamplerBindingType::NonFiltering),
        ],
    })
}

fn line_bind_group_layout(device: &wgpu::Device) -> wgpu::BindGroupLayout {
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("line"),
        entries: &[uniform_entry(0, size_of::<LineUniform>())],
    })
}

fn sampler(
    device: &wgpu::Device,
    filter: wgpu::FilterMode,
    compare: Option<wgpu::CompareFunction>,
) -> wgpu::Sampler {
    device.create_sampler(&wgpu::SamplerDescriptor {
        label: None,
        address_mode_u: wgpu::AddressMode::ClampToEdge,
        address_mode_v: wgpu::AddressMode::ClampToEdge,
        address_mode_w: wgpu::AddressMode::ClampToEdge,
        mag_filter: filter,
        min_filter: filter,
        mipmap_filter: wgpu::MipmapFilterMode::Nearest,
        lod_min_clamp: 0.0,
        lod_max_clamp: 0.0,
        compare,
        anisotropy_clamp: 1,
        border_color: None,
    })
}

fn dummy(device: &wgpu::Device, format: wgpu::TextureFormat) -> wgpu::TextureView {
    let texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("dummy"),
        size: wgpu::Extent3d { width: 1, height: 1, depth_or_array_layers: 1 },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format,
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::RENDER_ATTACHMENT,
        view_formats: &[],
    });
    texture.create_view(&wgpu::TextureViewDescriptor::default())
}
