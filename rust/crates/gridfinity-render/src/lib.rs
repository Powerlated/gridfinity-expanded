pub mod camera;
pub mod renderer;
pub mod vertex;

pub use camera::Camera;
pub use renderer::Renderer;
pub use vertex::{
    KERNEL_STRIDE, LINE_STRIDE, VERTEX_STRIDE, append_smooth_shaded, bounds_of, color_of,
};
