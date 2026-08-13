//! The kernel's scalar and vector vocabulary: `glam`'s types re-exported under
//! one name so nothing downstream depends on that crate directly, the weld
//! quantisation every construction interns positions through, and the angle
//! arithmetic a rotational curve's parameter needs. Nothing here knows about
//! surfaces, topology or Gridfinity; it is the layer everything else counts on
//! agreeing about.

pub use glam::{Mat4, Quat, Vec2, Vec3};

#[inline]
pub fn vec3_of(x: f32, y: f32, z: f32) -> Vec3 {
    Vec3::new(x, y, z)
}

/// `angle` shifted by whole turns until it lies in `[lo - slack, hi + slack]`,
/// which is what reading a point's angle about a circle's centre and matching it
/// to an arc's stored parameter range needs: `atan2` answers in `(-pi, pi]`
/// while an arc's range may sit anywhere, and the two differ by a whole number
/// of turns whenever the point is on the arc at all.
///
/// `lo` and `hi` bound the range in either traversal direction, so a caller
/// holding `(a0, a1)` passes `(min, max)` of the pair. The result is `angle`
/// plus an integer multiple of `TAU`; it lands inside the widened range whenever
/// the range spans at least a full turn or `angle` names a point of it, and is
/// otherwise the nearest shift below `hi + slack`, which a caller rejects by
/// testing the range itself.
pub fn wrap_angle_into(angle: f32, lo: f32, hi: f32, slack: f32) -> f32 {
    assert!(
        angle.is_finite() && lo.is_finite() && hi.is_finite(),
        "wrapping an angle into a range needs all three finite, got {angle} into [{lo}, {hi}]"
    );
    assert!(
        lo <= hi,
        "an angle range is given low end first, got [{lo}, {hi}]"
    );
    assert!(
        slack >= 0.0,
        "angular slack widens the range and cannot be negative, got {slack}"
    );
    let mut out = angle;
    while out < lo - slack {
        out += std::f32::consts::TAU;
    }
    while out > hi + slack {
        out -= std::f32::consts::TAU;
    }
    let turns = (out - angle) / std::f32::consts::TAU;
    assert!(
        (turns - turns.round()).abs() < 1e-3,
        "wrapping moves an angle by whole turns only, but {angle} became {out}, a shift of \
         {turns} turn(s)"
    );
    out
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
