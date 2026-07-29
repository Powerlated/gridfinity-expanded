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
    fn every_uniform_block_is_a_whole_number_of_sixteen_byte_rows() {
        for size in [
            size_of::<SceneUniform>(),
            size_of::<PostUniform>(),
            size_of::<LineUniform>(),
        ] {
            assert_eq!(size % 16, 0, "a uniform block of {size} bytes cannot be laid out");
        }
    }

    #[test]
    fn the_blocks_match_the_sizes_their_shader_structs_declare() {
        assert_eq!(size_of::<SceneUniform>(), 240);
        assert_eq!(size_of::<PostUniform>(), 208);
        assert_eq!(size_of::<LineUniform>(), 96);
    }

    #[test]
    fn every_four_component_field_starts_on_a_sixteen_byte_boundary() {
        let scene = SceneUniform::default();
        let base = &scene as *const SceneUniform as usize;
        for offset in [
            &scene.eye_time as *const _ as usize,
            &scene.fill_lines as *const _ as usize,
            &scene.key_ldr as *const _ as usize,
            &scene.viewport as *const _ as usize,
            &scene.shadow as *const _ as usize,
            &scene.floor_plane as *const _ as usize,
            &scene.toggles as *const _ as usize,
        ] {
            assert_eq!((offset - base) % 16, 0);
        }
    }

    #[test]
    fn the_post_blocks_vector_fields_land_where_wgsl_puts_them() {
        let post = PostUniform::default();
        let base = &post as *const PostUniform as usize;
        assert_eq!(&post.target_size as *const _ as usize - base, 144);
        assert_eq!(&post.params as *const _ as usize - base, 176);
        assert_eq!(&post.flags as *const _ as usize - base, 192);
    }

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
