//! Where a blend surface comes to rest on a terminating plane, and the curve
//! that plane trims it by. The blend's **own surface** decides this, not the edge
//! it was built from, so there is one function per blend surface kind behind
//! `runout_on`, plus `respan`, which the caller uses afterwards to drag a tangent
//! curve's parameter range along to the touchdowns that moved. Every answer here
//! is the plane's exact section of the surface, so a trim curve lies on the blend
//! and in the terminating face by construction rather than by a tolerance.

use crate::kernel::curvedge::CurvEdge;
use crate::kernel::geom::Curve;
use crate::kernel::geom::Surface;
use crate::kernel::math::{Vec3, wrap_pi};

use super::{END_AGREE, join_agree};

/// Runs a blend of radius `r`, whose ball ends centred at `cv` touching down at
/// `ta_p` and `tb_p`, out onto the terminating `plane`, and returns where those
/// two touchdowns land on it and the trim curve joining them there. Dispatches on
/// the blend surface: a cylinder to `runout_cyl`, a torus to `runout_torus`, and
/// anything else to an error, since no other blend surface has an analytic
/// section. Asserts the surface's radius really is the blend radius it was handed
/// -- the two come from different phases, and a mismatch would put the trim curve
/// on a surface parallel to the blend rather than on it. `away` is the direction
/// the chain was heading, used only where the section alone cannot say which of
/// two crossings the blend runs into.
#[allow(clippy::too_many_arguments)]
pub(super) fn runout_on(
    surface: &Surface,
    cv: Vec3,
    r: f32,
    ta_p: Vec3,
    tb_p: Vec3,
    plane: (Vec3, Vec3),
    away: Vec3,
) -> Result<(Vec3, Vec3, CurvEdge), String> {
    match *surface {
        Surface::Cylinder { axis, radius, .. } => {
            assert!(
                (radius - r).abs() <= END_AGREE,
                "blend runout: the blend cylinder's radius {radius} is not the blend radius {r}"
            );
            runout_cyl(cv, *axis, r, ta_p, tb_p, plane)
        }
        Surface::Torus {
            center,
            axis,
            major_r,
            minor_r,
            ..
        } => {
            assert!(
                (minor_r - r).abs() <= END_AGREE,
                "blend runout: the blend torus's minor radius {minor_r} is not the blend radius {r}"
            );
            runout_torus(center, *axis, major_r, r, cv, ta_p, tb_p, plane, away)
        }
        _ => Err(format!(
            "blend runout: no section curve for a {surface:?} blend"
        )),
    }
}

/// Sections a cylindrical blend with the terminating plane. A cylinder rolls the
/// ball along a straight axis, so the plane cuts it in an ellipse: each touchdown
/// slides **along the axis** onto the plane, and the trim curve is the ellipse
/// centred where the ball centre lands, with conjugate axes the images of the two
/// radial directions under that same slide. The parameter range runs from `ta_p`
/// to the angle `tb_p` sits at, so the curve's endpoints are the returned
/// touchdowns. Errors when the plane is parallel to the axis, where the slide has
/// no answer.
fn runout_cyl(
    cv: Vec3,
    axis: Vec3,
    r: f32,
    ta_p: Vec3,
    tb_p: Vec3,
    plane: (Vec3, Vec3),
) -> Result<(Vec3, Vec3, CurvEdge), String> {
    let (q, n) = plane;
    let n = n.normalize_or_zero();
    let d = axis.normalize_or_zero();
    let dn = d.dot(n);
    if dn.abs() < 1e-6 {
        return Err("blend runout: terminating face is parallel to the blend axis".into());
    }
    let onto = |p: Vec3| p + d * ((q - p).dot(n) / dn);
    let e1 = (ta_p - cv).normalize_or_zero();
    let e2 = d.cross(e1);
    let a_vec = d * (-r * e1.dot(n) / dn) + e1 * r;
    let b_vec = d * (-r * e2.dot(n) / dn) + e2 * r;
    let u = (tb_p - cv).normalize_or_zero();
    let t1 = u.dot(e2).atan2(u.dot(e1));
    let (curve, s0, s1) = Curve::ellipse_from_conjugate(onto(cv), a_vec, b_vec, 0.0, t1);
    let arc = CurvEdge {
        curve,
        t0: s0,
        t1: s1,
    };
    Ok((onto(ta_p), onto(tb_p), arc))
}

/// Moves a circular curve's parameter range onto the endpoints it now runs
/// between, leaving every other curve kind alone: the range starts at `from`'s
/// angle and sweeps to `to`'s, keeping the direction it already swept and taking
/// only the magnitude from the new endpoints. Direction is which way the blend
/// travels along its corner, which a runout extends but never reverses, so a
/// sweep that comes back the other way is turned the long way round instead --
/// recomputing both from the endpoints would flip a sweep that grew past a half
/// turn. Asserts the result stays within one full turn. Without this an edge
/// spans the arc the blend used to and misses its own vertex by however far the
/// touchdown ran.
pub(super) fn respan(ce: &mut CurvEdge, from: Vec3, to: Vec3) {
    let Curve::Circle {
        center,
        axis,
        ref_dir,
        ..
    } = ce.curve
    else {
        return;
    };
    let (d0, d1) = crate::kernel::geom::radial_frame(axis, ref_dir);
    let angle = |p: Vec3| {
        let v = p - center;
        v.dot(d1).atan2(v.dot(d0))
    };
    let was = ce.t1 - ce.t0;
    let t0 = angle(from);
    let mut sweep = wrap_pi(angle(to) - t0);
    if was != 0.0 && sweep != 0.0 && sweep.signum() != was.signum() {
        sweep += was.signum() * std::f32::consts::TAU;
    }
    assert!(
        sweep.abs() <= std::f32::consts::TAU,
        "blend runout: the tangent circle would sweep {sweep} rad, more than a full turn"
    );
    ce.t0 = t0;
    ce.t1 = t0 + sweep;
}

/// Whether the closed angular interval `[lo, hi]` contains pi modulo a full
/// turn -- that is, whether a sweep from `lo` to `hi` passes the point where a
/// ring's cosine bottoms out at -1.
fn spans_pi(lo: f32, hi: f32) -> bool {
    let k = ((lo - std::f32::consts::PI) / std::f32::consts::TAU).ceil();
    std::f32::consts::PI + k * std::f32::consts::TAU <= hi
}

/// Sections a torus blend with a terminating plane **parallel to its axis**, and
/// errors on any other plane: a plane's section of a torus is a quartic in
/// general, outside the analytic curve set, which is why a chain could not
/// terminate on an arc at all. Parallel is the case the model produces, and there
/// fixing the minor angle `t` fixes the ring radius `major + minor * cos t`, and
/// the plane meets that ring exactly where `cos u = offset / rad` -- which is
/// `Curve::TorusSection`. Returns the two touchdowns turned about the axis onto
/// the plane and that section curve spanning them, and errors when the plane
/// stands off past the narrowest ring the curve would have to cross, or when the
/// blend straddles the plane so that neither crossing is the one it runs into.
///
/// Three things it has to get right, each a bug first. The tangent curves are
/// circles of **constant minor angle**, so running one out changes `u` and leaves
/// `t` alone, and the section's parameter range is read straight off the two
/// touchdowns that already exist -- asserted afterwards, by checking each moved
/// touchdown kept its own ring radius and height. The ring radius is **signed**:
/// on a spindle torus (`minor > major`, which every corner blend tighter than its
/// own corner is) it goes negative past the axis, and reading a touchdown's minor
/// angle off the unsigned radius puts it half a turn away, the same distinction
/// `Surface::signed_distance` makes. And `branch` picks which of the plane's two
/// crossings of each ring the blend runs into: the nearer one measured around the
/// axis is always the one on the blend's own side of the plane normal, since for
/// `u_v` and `u_p` both in `(0, pi)`, `|wrap(u_v - u_p)| <= |wrap(u_v + u_p)|`
/// reduces to `u_v <= pi`, so the sign of the ball centre's component across the
/// normal is the whole decision.
#[allow(clippy::too_many_arguments)]
fn runout_torus(
    center: Vec3,
    axis: Vec3,
    major: f32,
    minor: f32,
    cv: Vec3,
    ta_p: Vec3,
    tb_p: Vec3,
    plane: (Vec3, Vec3),
    away: Vec3,
) -> Result<(Vec3, Vec3, CurvEdge), String> {
    let (q, n) = plane;
    let n = n.normalize_or_zero();
    let axis = axis.normalize_or_zero();
    assert!(
        (n.length() - 1.0).abs() < 1e-4 && (axis.length() - 1.0).abs() < 1e-4,
        "blend runout: the terminating plane's normal {n:?} and the torus axis {axis:?} must \
         both be unit vectors"
    );
    if n.dot(axis).abs() > 1e-4 {
        return Err(format!(
            "blend runout: the terminating plane's normal {n:?} is not perpendicular to the \
             torus blend's axis {axis:?}, so the section is a quartic"
        ));
    }
    let spine = {
        let rel = cv - center;
        (rel - axis * rel.dot(axis)).length()
    };
    assert!(
        (spine - major.abs()).abs() <= join_agree(minor),
        "blend runout: the ball centre {cv:?} stands {spine} from the torus axis, not the \
         major radius {major}, whose sign names the sheet and whose magnitude is the spine's \
         radius; the ball centre rides the torus's spine, and `reconcile_shared_ends` \
         may move it by at most a MAX_JOIN_KINK's worth"
    );

    let offset = (q - center).dot(n);
    let across = axis.cross(n);
    let side = (cv - center).dot(across);
    let side = if side.abs() > 1e-6 {
        side
    } else {
        away.dot(across)
    };
    if side.abs() <= 1e-6 {
        return Err(format!(
            "blend runout: the blend at {cv:?} straddles the terminating plane and runs along \
             it, so neither crossing of it is the one the blend runs into"
        ));
    }
    let branch = side.signum();

    let minor_angle = |p: Vec3| -> f32 {
        let rel = p - center;
        let h = rel.dot(axis);
        let rad = (rel - axis * h).length();
        let near = ((rad - major).powi(2) + h * h).sqrt();
        let far = ((rad + major).powi(2) + h * h).sqrt();
        let (signed, off) = if (near - minor).abs() <= (far - minor).abs() {
            (rad, near)
        } else {
            (-rad, far)
        };
        assert!(
            (off - minor).abs() <= join_agree(minor),
            "blend runout: the touchdown {p:?} lies {off} from the blend torus's spine, not \
             its minor radius {minor}"
        );
        h.atan2(signed - major)
    };
    let t_a = minor_angle(ta_p);
    let t_b = t_a + wrap_pi(minor_angle(tb_p) - t_a);

    let (lo, hi) = if t_a <= t_b { (t_a, t_b) } else { (t_b, t_a) };
    let cos_min = if spans_pi(lo, hi) {
        -1.0
    } else {
        lo.cos().min(hi.cos())
    };
    let rad_min = major + minor * cos_min;
    if !(rad_min > 0.0 && offset.abs() <= rad_min) {
        return Err(format!(
            "blend runout: the terminating plane stands {offset} from the torus blend's axis, \
             past the narrowest ring the trim curve crosses ({rad_min})"
        ));
    }

    let curve = Curve::torus_section(center, axis, n, offset, major, minor, branch);
    let (ta_new, tb_new) = (curve.point(t_a), curve.point(t_b));
    for (was, now) in [(ta_p, ta_new), (tb_p, tb_new)] {
        assert!(
            ((now - q).dot(n)).abs() <= 1e-4,
            "blend runout: the touchdown ran out from {was:?} to {now:?}, which is not on the \
             terminating plane"
        );
        let ring = |p: Vec3| {
            let rel = p - center;
            let h = rel.dot(axis);
            ((rel - axis * h).length(), h)
        };
        let ((r0, h0), (r1, h1)) = (ring(was), ring(now));
        assert!(
            (r0 - r1).abs() <= 1e-3 && (h0 - h1).abs() <= 1e-3,
            "blend runout: the touchdown moved off its own ring, from radius {r0} height {h0} \
             to radius {r1} height {h1}; running out turns a touchdown about the axis and must \
             leave the ring it was on, or the trim curve misses the tangent curve it closes"
        );
    }
    Ok((
        ta_new,
        tb_new,
        CurvEdge {
            curve,
            t0: t_a,
            t1: t_b,
        },
    ))
}
