use bytemuck::{Pod, Zeroable};
use glam::{Mat4, Vec3};

#[repr(C)]
#[derive(Clone, Copy, Default, Pod, Zeroable)]
pub struct SceneUniform {
    pub view_proj: [f32; 16],
    pub light_vp: [f32; 16],
    pub eye_time: [f32; 4],
    pub fill_lines: [f32; 4],
    pub key_ldr: [f32; 4],
    pub viewport: [f32; 4],
    pub shadow: [f32; 4],
    pub floor_plane: [f32; 4],
    pub toggles: [f32; 4],
}

#[repr(C)]
#[derive(Clone, Copy, Default, Pod, Zeroable)]
pub struct PostUniform {
    pub view_proj: [f32; 16],
    pub inv_view_proj: [f32; 16],
    pub eye: [f32; 4],
    pub target_size: [f32; 2],
    pub direction: [f32; 2],
    pub origin: [f32; 2],
    pub near_far: [f32; 2],
    pub params: [f32; 4],
    pub flags: [f32; 4],
    pub source_texel: [f32; 4],
}

#[repr(C)]
#[derive(Clone, Copy, Default, Pod, Zeroable)]
pub struct LineUniform {
    pub view_proj: [f32; 16],
    pub half_vp: [f32; 2],
    pub width: f32,
    pub alpha: f32,
    pub settings: [f32; 4],
}

const _: () = {
    assert!(size_of::<SceneUniform>() == 240);
    assert!(size_of::<PostUniform>() == 224);
    assert!(size_of::<LineUniform>() == 96);
    assert!(size_of::<SceneUniform>() % 16 == 0);
    assert!(size_of::<PostUniform>() % 16 == 0);
    assert!(size_of::<LineUniform>() % 16 == 0);
    assert!(std::mem::offset_of!(SceneUniform, eye_time) % 16 == 0);
    assert!(std::mem::offset_of!(SceneUniform, fill_lines) % 16 == 0);
    assert!(std::mem::offset_of!(SceneUniform, key_ldr) % 16 == 0);
    assert!(std::mem::offset_of!(SceneUniform, viewport) % 16 == 0);
    assert!(std::mem::offset_of!(SceneUniform, shadow) % 16 == 0);
    assert!(std::mem::offset_of!(SceneUniform, floor_plane) % 16 == 0);
    assert!(std::mem::offset_of!(SceneUniform, toggles) % 16 == 0);
    assert!(std::mem::offset_of!(PostUniform, target_size) == 144);
    assert!(std::mem::offset_of!(PostUniform, params) == 176);
    assert!(std::mem::offset_of!(PostUniform, flags) == 192);
    assert!(std::mem::offset_of!(PostUniform, source_texel) == 208);
};

pub fn mat(value: &Mat4) -> [f32; 16] {
    value.to_cols_array()
}

pub fn vec4(v: Vec3, w: f32) -> [f32; 4] {
    [v.x, v.y, v.z, w]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_direction_packs_into_the_first_three_lanes_and_keeps_its_scalar() {
        assert_eq!(vec4(Vec3::new(1.0, 2.0, 3.0), 4.0), [1.0, 2.0, 3.0, 4.0]);
    }

    #[test]
    fn a_matrix_reaches_the_shader_in_column_major_order() {
        let m = Mat4::from_translation(Vec3::new(5.0, 6.0, 7.0));
        let cols = mat(&m);
        assert_eq!(&cols[12..15], &[5.0, 6.0, 7.0]);
    }
}
