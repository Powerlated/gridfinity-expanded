
pub use glam::{Mat4, Quat, Vec2, Vec3};

#[inline]
pub fn vec3_of(x: f32, y: f32, z: f32) -> Vec3 {
    Vec3::new(x, y, z)
}

pub const WELD: f32 = 1.0e4;

pub const WELD_NEAR: f32 = 0.5 / WELD;

pub const WELD_NEAR_SQ: f32 = WELD_NEAR * WELD_NEAR;

#[inline]
pub fn weld_key(p: Vec3) -> (i64, i64, i64) {
    (
        (p.x * WELD).round() as i64,
        (p.y * WELD).round() as i64,
        (p.z * WELD).round() as i64,
    )
}
