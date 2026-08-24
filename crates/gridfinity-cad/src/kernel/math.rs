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
pub fn wrap_pi(a: f32) -> f32 {
    assert!(
        a.is_finite(),
        "wrapping an angle into a half turn either side of zero needs it finite, got {a}"
    );
    let mut out = a;
    while out > std::f32::consts::PI {
        out -= std::f32::consts::TAU;
    }
    while out <= -std::f32::consts::PI {
        out += std::f32::consts::TAU;
    }
    let turns = (a - out) / std::f32::consts::TAU;
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

/// How far the length of a `Dir`, or the cosine between a `Dir` and the axis it
/// was made perpendicular to, may sit from its exact value.
///
/// A few `f64` ulps, and nothing more: a `Dir` is normalised in `f64` at
/// construction, so this bounds one `f64` square root and nothing else. It is
/// eight orders tighter than the 1e-8 linear resolution a transmit file
/// declares, which is the point -- an `f32` unit vector is unit only to about
/// 6.7e-8 once widened, six times that resolution, and a Parasolid frustrum
/// measures it as non-unit and faults the face carrying it.
pub const UNIT_RESIDUE: f64 = 1.0e-15;

/// A direction: a vector of unit length in `f64`.
///
/// The kernel models in `f32` and a direction is the one quantity that cannot
/// afford to. Every surface axis, plane normal and reference direction is a
/// `Dir`, so the property "this is a unit vector" is established once where the
/// value is made rather than re-established by each consumer, and a writer
/// emitting one has nothing left to fix up.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Dir {
    exact: [f64; 3],
    approx: Vec3,
}

impl Dir {
    /// `v` as a direction: widened to `f64`, then normalised there.
    ///
    /// `v` may be any non-degenerate finite vector -- this is where an arbitrary
    /// direction *becomes* unit, so it does not require one going in. What it
    /// guarantees coming out is a vector whose length is 1 to `UNIT_RESIDUE`.
    pub fn new(v: Vec3) -> Dir {
        assert!(
            v.is_finite() && v.length_squared() > 1e-24,
            "a direction is made from a finite vector of some length, got {v:?}"
        );
        Dir::from_f64([v.x as f64, v.y as f64, v.z as f64])
    }

    /// `v` as a direction, normalised in `f64` from `f64` components, for a
    /// caller that already holds the exact numbers rather than their `f32` cast.
    pub fn from_f64(v: [f64; 3]) -> Dir {
        let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
        assert!(
            len.is_finite() && len > 1e-12,
            "a direction is made from a finite vector of some length, got {v:?} of length {len}"
        );
        let unit = [v[0] / len, v[1] / len, v[2] / len];
        let residue =
            (unit[0] * unit[0] + unit[1] * unit[1] + unit[2] * unit[2]).sqrt() - 1.0;
        assert!(
            residue.abs() <= UNIT_RESIDUE,
            "one f64 normalisation leaves a direction unit to f64 precision, but {unit:?} has \
             length {}",
            residue + 1.0
        );
        Dir {
            exact: unit,
            approx: Vec3::new(unit[0] as f32, unit[1] as f32, unit[2] as f32),
        }
    }

    /// The `f32` view of this direction, which is what the kernel's own
    /// arithmetic works in. Unit to `f32` precision only -- about 6.7e-8 off, at
    /// a tilt -- so it is the wrong thing to write to a file; the exact value is
    /// `components`. Also reached by `Deref`, which is what lets a consumer call
    /// `Vec3`'s own methods on a direction unchanged.
    #[inline]
    pub fn vec(self) -> Vec3 {
        self.approx
    }

    /// The exact components, for a caller that must not lose the `f64`
    /// normalisation -- which is every caller writing this direction out.
    #[inline]
    pub fn components(self) -> [f64; 3] {
        self.exact
    }

    /// This direction with its component along `axis` removed and the remainder
    /// renormalised, which is one Gram-Schmidt step in `f64`.
    ///
    /// The result is perpendicular to `axis` to `UNIT_RESIDUE` and unit to the
    /// same. `self` must already lean less than `PERP_TOL` out of perpendicular:
    /// this refines a frame the caller built, and a direction genuinely along
    /// the axis is the caller's defect rather than something a projection can
    /// repair.
    pub fn perp_to(self, axis: Dir) -> Dir {
        let (x, a) = (self.exact, axis.exact);
        let dot = a[0] * x[0] + a[1] * x[1] + a[2] * x[2];
        assert!(
            dot.abs() < PERP_TOL,
            "a reference direction is perpendicular to its axis, but {self:?} and {axis:?} meet \
             at a cosine of {dot}"
        );
        let out = Dir::from_f64([
            x[0] - a[0] * dot,
            x[1] - a[1] * dot,
            x[2] - a[2] * dot,
        ]);
        let e = out.exact;
        let residual = a[0] * e[0] + a[1] * e[1] + a[2] * e[2];
        assert!(
            residual.abs() <= UNIT_RESIDUE,
            "one Gram-Schmidt step leaves a direction perpendicular to f64 precision, but \
             {out:?} meets {axis:?} at a cosine of {residual}"
        );
        out
    }

    /// The unit direction perpendicular to both, normalised in `f64`. The two
    /// must not be parallel, where the cross product vanishes and no direction
    /// is named.
    ///
    /// Named apart from `Vec3::cross`, which `Deref` also offers on a `Dir`,
    /// because the two differ in precision and a caller must say which it
    /// meant: this one is for a direction that will be written out, the `f32`
    /// one for the kernel's own arithmetic.
    pub fn cross_exact(self, other: Dir) -> Dir {
        let (a, b) = (self.exact, other.exact);
        Dir::from_f64([
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ])
    }

    /// The cosine of the angle between the two, in `f64`. Named apart from
    /// `Vec3::dot` for the reason `cross_exact` is.
    #[inline]
    pub fn dot_exact(self, other: Dir) -> f64 {
        self.exact[0] * other.exact[0]
            + self.exact[1] * other.exact[1]
            + self.exact[2] * other.exact[2]
    }
}

/// How far a reference direction may lean out of perpendicular to its axis
/// before it is the frame the caller built that is wrong, rather than the
/// precision of the cast that widened it.
const PERP_TOL: f64 = 1.0e-4;

impl std::ops::Neg for Dir {
    type Output = Dir;
    #[inline]
    fn neg(self) -> Dir {
        Dir {
            exact: [-self.exact[0], -self.exact[1], -self.exact[2]],
            approx: -self.approx,
        }
    }
}

impl std::ops::Deref for Dir {
    type Target = Vec3;
    #[inline]
    fn deref(&self) -> &Vec3 {
        &self.approx
    }
}

impl From<Dir> for Vec3 {
    #[inline]
    fn from(d: Dir) -> Vec3 {
        d.vec()
    }
}
