//! The kernel's scalar and vector vocabulary: `glam`'s types re-exported under
//! one name so nothing downstream depends on that crate directly, the weld
//! quantisation every construction interns positions through, and the angle
//! arithmetic a rotational curve's parameter needs. Nothing here knows about
//! surfaces, topology or Gridfinity; it is the layer everything else counts on
//! agreeing about.

pub use glam::{DMat4 as Mat4, DQuat as Quat, DVec2 as Vec2, DVec3 as Vec3};

#[inline]
pub fn vec3_of(x: f64, y: f64, z: f64) -> Vec3 {
    Vec3::new(x, y, z)
}

/// `a` shifted by whole turns into `(-PI, PI]`, which is the half-turn either
/// side of zero that a *difference* of two angles belongs in: the short way
/// round from one to the other, signed by which way that is.
///
/// The closed end is at `+PI`, so an exact half turn comes back positive and
/// two angles diametrically opposite are one turn apart in a definite
/// direction rather than an arbitrary one. Every caller here is subtracting two
/// angles -- a sweep against its source, a sample's drift from the last one, a
/// corner's turn -- which is what distinguishes this from `wrap_angle_into`,
/// where the range is the caller's and the answer must land inside it.
pub fn wrap_pi(a: f64) -> f64 {
    assert!(
        a.is_finite(),
        "wrapping an angle into a half turn either side of zero needs it finite, got {a}"
    );
    let mut out = a;
    while out > std::f64::consts::PI {
        out -= std::f64::consts::TAU;
    }
    while out <= -std::f64::consts::PI {
        out += std::f64::consts::TAU;
    }
    let turns = (a - out) / std::f64::consts::TAU;
    assert!(
        (turns - turns.round()).abs() < 1e-3,
        "wrapping moves an angle by whole turns only, but {a} became {out}, a shift of {turns} \
         turn(s)"
    );
    out
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
pub fn wrap_angle_into(angle: f64, lo: f64, hi: f64, slack: f64) -> f64 {
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
        out += std::f64::consts::TAU;
    }
    while out > hi + slack {
        out -= std::f64::consts::TAU;
    }
    let turns = (out - angle) / std::f64::consts::TAU;
    assert!(
        (turns - turns.round()).abs() < 1e-3,
        "wrapping moves an angle by whole turns only, but {angle} became {out}, a shift of \
         {turns} turn(s)"
    );
    out
}

pub const WELD: f64 = 1.0e4;

pub const WELD_NEAR: f64 = 0.5 / WELD;

pub const WELD_NEAR_SQ: f64 = WELD_NEAR * WELD_NEAR;

#[inline]
pub fn weld_key(p: Vec3) -> (i64, i64, i64) {
    (
        (p.x * WELD).round() as i64,
        (p.y * WELD).round() as i64,
        (p.z * WELD).round() as i64,
    )
}

/// How far the length of a `Dir`, or the cosine between a `Dir` and the axis it
/// was made perpendicular to, may sit from its exact value.
///
/// A few ulps, and nothing more: a `Dir` is normalised at construction, so this
/// bounds one square root and nothing else. It is seven orders tighter than the
/// 1e-8 linear resolution a transmit file declares, which is the point. That
/// margin was once a real defect rather than a formality -- the kernel modelled
/// in `f32`, where a unit vector is unit only to about 6.7e-8 once widened, six
/// times the declared resolution, and a Parasolid frustrum measured it as
/// non-unit and faulted every face carrying a tilted one.
pub const UNIT_RESIDUE: f64 = 1.0e-15;

/// A direction: a vector of unit length.
///
/// Every surface axis, plane normal and reference direction is a `Dir`, so the
/// property "this is a unit vector" is established once where the value is made
/// rather than re-established by each consumer, and a writer emitting one has
/// nothing left to fix up. A transmit file declares `res_linear` 1e-8 and a
/// Parasolid frustrum measures a direction against it, so `UNIT_RESIDUE` is what
/// a direction is held to -- seven orders inside that.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Dir(Vec3);

impl Dir {
    /// `v` as a direction: normalised, so any non-degenerate finite vector goes
    /// in and a vector of length 1 to `UNIT_RESIDUE` comes out.
    pub fn new(v: Vec3) -> Dir {
        Dir::from_f64([v.x, v.y, v.z])
    }

    /// `v` as a direction, from components a caller holds loose rather than in a
    /// `Vec3`.
    pub fn from_f64(v: [f64; 3]) -> Dir {
        let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
        assert!(
            len.is_finite() && len > 1e-12,
            "a direction is made from a finite vector of some length, got {v:?} of length {len}"
        );
        let unit = Vec3::new(v[0] / len, v[1] / len, v[2] / len);
        let residue = unit.length() - 1.0;
        assert!(
            residue.abs() <= UNIT_RESIDUE,
            "one normalisation leaves a direction unit to the precision the kernel models in, \
             but {unit:?} has length {}",
            residue + 1.0
        );
        Dir(unit)
    }

    /// This direction as a vector. Also reached by `Deref`, which is what lets a
    /// consumer call `Vec3`'s own methods on a direction unchanged.
    #[inline]
    pub fn vec(self) -> Vec3 {
        self.0
    }

    /// The three components, for a caller that wants them loose -- which is
    /// every caller writing this direction out to a file.
    #[inline]
    pub fn components(self) -> [f64; 3] {
        self.0.to_array()
    }

    /// This direction with its component along `axis` removed and the remainder
    /// renormalised, which is one Gram-Schmidt step.
    ///
    /// The result is perpendicular to `axis` to `UNIT_RESIDUE` and unit to the
    /// same. `self` must already lean less than `PERP_TOL` out of perpendicular:
    /// this refines a frame the caller built, and a direction genuinely along
    /// the axis is the caller's defect rather than something a projection can
    /// repair.
    pub fn perp_to(self, axis: Dir) -> Dir {
        let dot = axis.0.dot(self.0);
        assert!(
            dot.abs() < PERP_TOL,
            "a reference direction is perpendicular to its axis, but {self:?} and {axis:?} meet \
             at a cosine of {dot}"
        );
        let out = Dir::new(self.0 - axis.0 * dot);
        let residual = axis.0.dot(out.0);
        assert!(
            residual.abs() <= UNIT_RESIDUE,
            "one Gram-Schmidt step leaves a direction perpendicular to the precision the kernel \
             models in, but {out:?} meets {axis:?} at a cosine of {residual}"
        );
        out
    }

    /// The unit direction perpendicular to both. The two must not be parallel,
    /// where the cross product vanishes and no direction is named.
    pub fn cross_dir(self, other: Dir) -> Dir {
        Dir::new(self.0.cross(other.0))
    }
}

/// How far a reference direction may lean out of perpendicular to its axis
/// before it is the frame the caller built that is wrong, rather than the
/// precision of the cast that widened it.
const PERP_TOL: f64 = 1.0e-9;

impl std::ops::Neg for Dir {
    type Output = Dir;
    #[inline]
    fn neg(self) -> Dir {
        Dir(-self.0)
    }
}

impl std::ops::Deref for Dir {
    type Target = Vec3;
    #[inline]
    fn deref(&self) -> &Vec3 {
        &self.0
    }
}

impl From<Dir> for Vec3 {
    #[inline]
    fn from(d: Dir) -> Vec3 {
        d.vec()
    }
}
