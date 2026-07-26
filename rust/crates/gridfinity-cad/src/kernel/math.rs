//! Small math surface for the kernel. We lean on `glam` for the vector/matrix
//! types and only add the couple of helpers the geometry code wants.

pub use glam::{Mat4, Quat, Vec2, Vec3};

/// Convenience constructor (a touch shorter than `Vec3::new` at call sites).
#[inline]
pub fn vec3_of(x: f32, y: f32, z: f32) -> Vec3 {
    Vec3::new(x, y, z)
}

/// Quantize a coordinate to an integer grid so exactly-coincident vertices weld
/// together. 1e4 → 0.1 µm resolution: far finer than any real feature, coarse
/// enough to fuse the sub-micron near-duplicates booleans leave behind.
pub const WELD: f32 = 1.0e4;

#[inline]
pub fn weld_key(p: Vec3) -> (i64, i64, i64) {
    (
        (p.x * WELD).round() as i64,
        (p.y * WELD).round() as i64,
        (p.z * WELD).round() as i64,
    )
}
