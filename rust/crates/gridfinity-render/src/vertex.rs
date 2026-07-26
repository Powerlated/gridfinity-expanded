use glam::Vec3;

pub const VERTEX_STRIDE: usize = 10;
pub const LINE_STRIDE: usize = 11;

const SOUP_FLOATS_PER_TRIANGLE: usize = 9;
const KERNEL_STRIDE: usize = 6;

pub fn color_of(rgb: u32) -> Vec3 {
    Vec3::new(
        ((rgb >> 16) & 0xff) as f32 / 255.0,
        ((rgb >> 8) & 0xff) as f32 / 255.0,
        (rgb & 0xff) as f32 / 255.0,
    )
}

fn push(out: &mut Vec<f32>, position: Vec3, normal: Vec3, color: Vec3, bad: bool) {
    out.extend_from_slice(&[
        position.x,
        position.y,
        position.z,
        normal.x,
        normal.y,
        normal.z,
        color.x,
        color.y,
        color.z,
        if bad { 1.0 } else { 0.0 },
    ]);
}

pub fn append_smooth_shaded(
    out: &mut Vec<f32>,
    kernel_buffer: &[f32],
    offset: Vec3,
    color: Vec3,
    bad: bool,
) {
    out.reserve(kernel_buffer.len() / KERNEL_STRIDE * VERTEX_STRIDE);
    for v in kernel_buffer.chunks_exact(KERNEL_STRIDE) {
        let position = Vec3::new(v[0], v[1], v[2]) + offset;
        push(out, position, Vec3::new(v[3], v[4], v[5]), color, bad);
    }
}

pub fn append_flat_shaded(out: &mut Vec<f32>, soup: &[f32], offset: Vec3, color: Vec3, bad: bool) {
    out.reserve(soup.len() / SOUP_FLOATS_PER_TRIANGLE * 3 * VERTEX_STRIDE);
    for tri in soup.chunks_exact(SOUP_FLOATS_PER_TRIANGLE) {
        let a = Vec3::new(tri[0], tri[1], tri[2]) + offset;
        let b = Vec3::new(tri[3], tri[4], tri[5]) + offset;
        let c = Vec3::new(tri[6], tri[7], tri[8]) + offset;
        let Some(normal) = (b - a).cross(c - a).try_normalize() else {
            continue;
        };
        for position in [a, b, c] {
            push(out, position, normal, color, bad);
        }
    }
}

pub fn bounds_of(vertices: &[f32]) -> Option<(Vec3, Vec3)> {
    let mut min = Vec3::splat(f32::INFINITY);
    let mut max = Vec3::splat(f32::NEG_INFINITY);
    let mut any = false;
    for v in vertices.chunks_exact(VERTEX_STRIDE) {
        let p = Vec3::new(v[0], v[1], v[2]);
        min = min.min(p);
        max = max.max(p);
        any = true;
    }
    any.then_some((min, max))
}

#[cfg(test)]
mod tests {
    use super::*;

    const TRI: [f32; 9] = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0];

    #[test]
    fn a_soup_triangle_becomes_three_vertices_sharing_one_flat_normal() {
        let mut out = Vec::new();
        append_flat_shaded(&mut out, &TRI, Vec3::ZERO, Vec3::X, false);
        assert_eq!(out.len(), 3 * VERTEX_STRIDE);
        for v in out.chunks_exact(VERTEX_STRIDE) {
            assert_eq!((v[3], v[4], v[5]), (0.0, 0.0, 1.0));
            assert_eq!((v[6], v[7], v[8]), (1.0, 0.0, 0.0));
            assert_eq!(v[9], 0.0);
        }
    }

    #[test]
    fn an_offset_moves_positions_and_leaves_normals_alone() {
        let mut out = Vec::new();
        append_flat_shaded(&mut out, &TRI, Vec3::new(10.0, -4.0, 2.0), Vec3::ONE, false);
        assert_eq!((out[0], out[1], out[2]), (10.0, -4.0, 2.0));
        assert_eq!((out[3], out[4], out[5]), (0.0, 0.0, 1.0));
    }

    #[test]
    fn degenerate_triangles_are_dropped_rather_than_shaded_with_a_zero_normal() {
        let mut out = Vec::new();
        append_flat_shaded(&mut out, &[0.0; 9], Vec3::ZERO, Vec3::ONE, false);
        assert!(out.is_empty());
    }

    #[test]
    fn the_kernel_buffer_keeps_its_own_analytic_normals() {
        let kernel = [1.0, 2.0, 3.0, 0.0, 1.0, 0.0];
        let mut out = Vec::new();
        append_smooth_shaded(&mut out, &kernel, Vec3::ZERO, Vec3::ONE, true);
        assert_eq!(out.len(), VERTEX_STRIDE);
        assert_eq!((out[0], out[1], out[2]), (1.0, 2.0, 3.0));
        assert_eq!((out[3], out[4], out[5]), (0.0, 1.0, 0.0));
        assert_eq!(out[9], 1.0);
    }

    #[test]
    fn bounds_cover_every_appended_vertex() {
        let mut out = Vec::new();
        append_flat_shaded(&mut out, &TRI, Vec3::ZERO, Vec3::ONE, false);
        append_flat_shaded(&mut out, &TRI, Vec3::new(5.0, 5.0, 2.0), Vec3::ONE, false);
        let (min, max) = bounds_of(&out).expect("appended vertices have bounds");
        assert_eq!(min, Vec3::ZERO);
        assert_eq!(max, Vec3::new(6.0, 6.0, 2.0));
    }

    #[test]
    fn an_empty_buffer_has_no_bounds() {
        assert!(bounds_of(&[]).is_none());
    }

    #[test]
    fn hex_colours_decode_channelwise() {
        assert_eq!(color_of(0xff0000), Vec3::X);
        assert_eq!(color_of(0x00ff00), Vec3::Y);
        assert_eq!(color_of(0x0000ff), Vec3::Z);
        assert_eq!(color_of(0xffffff), Vec3::ONE);
    }
}
