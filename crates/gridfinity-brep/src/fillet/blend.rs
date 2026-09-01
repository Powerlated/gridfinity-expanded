//! The blend surfaces themselves, one per blended edge. A `Fillet` is the whole
//! patch: the surface, the two tangent curves where it meets the faces it blends,
//! the connect arc closing each end, and the four touchdown points those curves
//! run between. Two faces meeting along a line give a cylinder; a plane meeting a
//! coaxial cylinder along a circle gives a torus; nothing else is supported, and
//! asking for one is an error rather than an approximation. `build_all` drives
//! the two builders and runs each chain end out; the rest of the file is the
//! curve arithmetic they share.

use std::collections::HashMap;

use crate::curvedge::{CurvEdge, as_plane};
use crate::geom::{Curve, Surface};
use crate::math::{Dir, Vec3, wrap_pi};
use crate::topo::{Edge, EdgeFaces, EdgeId, Solid};

use super::corner::Corner;
use super::join_agree;
use super::query::as_cyl;
use super::runout::{Runout, RunoutEnd, Runouts, plan_runout_end};
use super::section::{respan, runout_on};

#[derive(Clone)]
pub(super) struct Fillet {
    pub ta: CurvEdge,
    pub tb: CurvEdge,
    pub ca0: CurvEdge,
    pub ca1: CurvEdge,
    pub ta_p0: Vec3,
    pub ta_p1: Vec3,
    pub tb_p0: Vec3,
    pub tb_p1: Vec3,
    pub surface: Surface,
    pub sense: bool,
    pub fwd_a: bool,
}

pub(super) type Blends = HashMap<EdgeId, Fillet>;

/// Maps every solved corner to its blend patch, returning the patches keyed by
/// blended edge together with one `Runout` per chain end in `terminating` --
/// exactly one patch per corner, asserted, since a corner *is* one blended edge.
/// Picks the cylinder builder when both faces are planar and the torus builder
/// when one is a cylinder met along a circular edge, and errors on any other
/// pair. Each patch comes back with its ends already run out onto whatever
/// `plan_runout_end` chose for them, its touchdowns moved to where they landed,
/// and its tangent curves respanned to follow: a runout moves a touchdown *along*
/// its own curve, and an edge emitted over a stale range misses its own vertex by
/// however far the touchdown ran.
pub(super) fn build_all(
    solid: &Solid,
    corners: &[Corner],
    terminating: &super::chain::Terminating,
    edge_faces: &EdgeFaces,
) -> Result<(Blends, Runouts), String> {
    let mut bm: Blends = HashMap::with_capacity(corners.len());
    let mut runouts: Runouts = HashMap::new();
    for &Corner {
        e,
        fa,
        fb,
        r,
        ma,
        na0,
        fwd_a,
        ends,
    } in corners.iter()
    {
        let ed = solid.edges[e];
        let (cv0, ta_p0, tb_p0) = (ends[0].cv, ends[0].ta_p, ends[0].tb_p);
        let (cv1, ta_p1, tb_p1) = (ends[1].cv, ends[1].ta_p, ends[1].tb_p);

        let plane_a = as_plane(&solid.faces[fa].surface);
        let plane_b = as_plane(&solid.faces[fb].surface);
        let cyl = as_cyl(&solid.faces[fa].surface).or_else(|| as_cyl(&solid.faces[fb].surface));
        let is_circle = matches!(ed.curve, Curve::Circle { .. });

        let mut blend = if plane_a.is_some() && plane_b.is_some() {
            build_cyl_blend(ed, cv0, cv1, ma, na0, ta_p0, ta_p1, tb_p0, tb_p1, r, fwd_a)?
        } else if cyl.is_some() && is_circle && (plane_a.is_some() || plane_b.is_some()) {
            build_torus_blend(
                ed,
                cv0,
                cv1,
                na0,
                ta_p0,
                ta_p1,
                tb_p0,
                tb_p1,
                r,
                cyl.expect("this arm is guarded by cyl.is_some()"),
                fwd_a,
            )?
        } else {
            return Err(format!(
                "blend: edge {e} pair not supported (only plane/plane or plane/coaxial-cylinder)"
            ));
        };

        for (at_v0, v) in [(true, ed.v0), (false, ed.v1)] {
            if terminating.get(&v) != Some(&e) {
                continue;
            }
            let (cv, tap, tbp) = if at_v0 {
                (cv0, ta_p0, tb_p0)
            } else {
                (cv1, ta_p1, tb_p1)
            };
            let away = (cv - if at_v0 { cv1 } else { cv0 }).normalize_or_zero();
            let land = |plane| {
                runout_on(&blend.surface, cv, r, tap, tbp, plane, away).map(|(a, b, _)| (a, b))
            };
            let end = plan_runout_end(solid, v, e, fa, fb, edge_faces, away, land);
            let (ta_new, tb_new, arc) = match end {
                RunoutEnd::Flat { .. } => (tap, tbp, if at_v0 { blend.ca0 } else { blend.ca1 }),
                RunoutEnd::Absorb { face } | RunoutEnd::Cap { fa_side: face, .. } => {
                    let plane = as_plane(&solid.faces[face].surface).ok_or_else(|| {
                        format!(
                            "blend: runout face {face} at vertex {v} is not planar ({:?})",
                            solid.faces[face].surface
                        )
                    })?;
                    runout_on(&blend.surface, cv, r, tap, tbp, plane, away)?
                }
            };
            if at_v0 {
                blend.ta_p0 = ta_new;
                blend.tb_p0 = tb_new;
                blend.ca0 = arc;
            } else {
                blend.ta_p1 = ta_new;
                blend.tb_p1 = tb_new;
                blend.ca1 = arc;
            }
            runouts.insert(
                v,
                Runout {
                    end,
                    corner: solid.verts[v].point,
                    arc,
                    ta_p: ta_new,
                    tb_p: tb_new,
                    fa,
                    fb,
                },
            );
            respan(&mut blend.ta, blend.ta_p0, blend.ta_p1);
            respan(&mut blend.tb, blend.tb_p0, blend.tb_p1);
        }
        bm.insert(e, blend);
    }
    assert!(
        bm.len() == corners.len(),
        "blend: {} corners solved but {} blend surfaces built; a corner is one blended edge",
        corners.len(),
        bm.len()
    );
    Ok((bm, runouts))
}

/// Builds the patch for a ball rolled along a straight edge between two planes:
/// a cylinder of radius `r` about the line through the two ball centres, whose
/// tangent curves are the lines from `ta_p0` to `ta_p1` and `tb_p0` to `tb_p1`
/// and whose ends are the ball's connect arcs there. `sense` comes out true when
/// the surface's own normal at the first touchdown agrees with the face normal
/// `na0`, which is what makes the patch face outward like the faces it joins.
/// Errors when the edge is not a line, since then the swept surface is not a
/// cylinder.
#[allow(clippy::too_many_arguments)]
fn build_cyl_blend(
    ed: Edge,
    cv0: Vec3,
    cv1: Vec3,
    ma: Vec3,
    na0: Vec3,
    ta_p0: Vec3,
    ta_p1: Vec3,
    tb_p0: Vec3,
    tb_p1: Vec3,
    r: f64,
    fwd_a: bool,
) -> Result<Fillet, String> {
    let dir = match ed.curve {
        Curve::Line { dir, .. } => dir,
        _ => return Err("cyl blend: edge not a line".into()),
    };
    let ref_dir = (-ma).normalize_or(Vec3::X);

    let ta = CurvEdge {
        curve: Curve::Line { p0: ta_p0, dir },
        t0: 0.0,
        t1: (ta_p1 - ta_p0).length(),
    };
    let tb = CurvEdge {
        curve: Curve::Line { p0: tb_p0, dir },
        t0: 0.0,
        t1: (tb_p1 - tb_p0).length(),
    };

    let ca0 = connect_arc(cv0, *dir, ta_p0, tb_p0)?;
    let ca1 = connect_arc(cv1, *dir, ta_p1, tb_p1)?;

    let surface = Surface::cylinder(cv0, *dir, r, ref_dir);
    let sense = surface.normal(surface.project(ta_p0)).dot(na0) > 0.0;

    Ok(Fillet {
        ta,
        tb,
        ca0,
        ca1,
        ta_p0,
        ta_p1,
        tb_p0,
        tb_p1,
        surface,
        sense,
        fwd_a,
    })
}

/// Maps a circle and the two points a tangent curve now runs between to the
/// parameter range that traverses it: starting at `p0`'s own angle in the
/// circle's frame and sweeping by `src`'s magnitude in whichever direction lands
/// nearer `p1`. Only the magnitude is inherited from the source edge -- once the
/// blend radius exceeds the corner radius the tangent circle lands on the far
/// side of the axis, so the inherited range would need rotating by pi and
/// reversing. A source spanning a full turn is passed through untouched, since
/// then `p0` and `p1` coincide and cannot tell 2pi from 0.
fn circle_span(
    center: Vec3,
    axis: Dir,
    ref_dir: Dir,
    p0: Vec3,
    p1: Vec3,
    src: (f64, f64),
) -> (f64, f64) {
    let (d0, d1) = crate::geom::radial_frame(axis, ref_dir);
    let angle = |p: Vec3| {
        let v = p - center;
        v.dot(d1).atan2(v.dot(d0))
    };
    let span = src.1 - src.0;
    let t0 = angle(p0);
    if span.abs() >= std::f64::consts::TAU - 1e-3 {
        return (t0, t0 + span);
    }
    let want = angle(p1);
    let miss = |s: f64| wrap_pi(t0 + s - want).abs();
    let sweep = if miss(span.abs()) <= miss(-span.abs()) {
        span.abs()
    } else {
        -span.abs()
    };
    (t0, t0 + sweep)
}

/// Builds the patch for a ball rolled around a circular edge where a plane meets
/// the coaxial cylinder `cyl`: a torus on the edge's axis whose minor radius is
/// the blend radius `r` and whose major radius is how far the ball centre stands
/// off that axis. Its tangent curves are circles of constant minor angle, spanned
/// by `circle_span` from their own touchdowns, and its ends are the ball's
/// connect arcs about the edge's tangent there. Asserts the major radius has not
/// collapsed to a ring, and errors when the edge is not a circle.
#[allow(clippy::too_many_arguments)]
fn build_torus_blend(
    ed: Edge,
    cv0: Vec3,
    cv1: Vec3,
    na0: Vec3,
    ta_p0: Vec3,
    ta_p1: Vec3,
    tb_p0: Vec3,
    tb_p1: Vec3,
    r: f64,
    cyl: (Vec3, Vec3, f64),
    fwd_a: bool,
) -> Result<Fillet, String> {
    let (cyl_base, cyl_axis, _cyl_radius) = cyl;
    let (edge_center, edge_axis, edge_radius, edge_ref_dir) = match ed.curve {
        Curve::Circle {
            center,
            axis,
            radius,
            ref_dir,
        } => (center, axis, radius, ref_dir),
        _ => return Err("torus blend: edge not a circle".into()),
    };
    let (a0, a1) = (ed.t0, ed.t1);
    let cv0_on = cyl_base + cyl_axis * (cv0 - cyl_base).dot(cyl_axis);
    let major = (cv0 - cv0_on).length();
    let torus_center = cv0_on;
    let torus_axis = edge_axis;
    let ref_dir = edge_ref_dir;
    assert!(
        major > 0.05,
        "blend torus degenerates to a ring: major {major} minor {r}"
    );
    let surface =
        Surface::torus_through(torus_center, *torus_axis, major, r, *ref_dir, ta_p0);
    let _ = (edge_center, edge_radius);

    let ta_center = torus_center + *torus_axis * (ta_p0 - torus_center).dot(*torus_axis);
    let ta_r = (ta_p0 - ta_center).length();
    let tb_center = torus_center + *torus_axis * (tb_p0 - torus_center).dot(*torus_axis);
    let tb_r = (tb_p0 - tb_center).length();
    let (ta_t0, ta_t1) = circle_span(ta_center, torus_axis, ref_dir, ta_p0, ta_p1, (a0, a1));
    let (tb_t0, tb_t1) = circle_span(tb_center, torus_axis, ref_dir, tb_p0, tb_p1, (a0, a1));
    let ta = CurvEdge {
        curve: Curve::Circle {
            center: ta_center,
            axis: torus_axis,
            radius: ta_r,
            ref_dir,
        },
        t0: ta_t0,
        t1: ta_t1,
    };
    let tb = CurvEdge {
        curve: Curve::Circle {
            center: tb_center,
            axis: torus_axis,
            radius: tb_r,
            ref_dir,
        },
        t0: tb_t0,
        t1: tb_t1,
    };

    let p0 = ed.curve.point(a0);
    let tan_at = |p: Vec3| {
        let v = p - torus_center;
        let perp = v - *torus_axis * v.dot(*torus_axis);
        torus_axis.cross(perp.normalize_or(Vec3::X))
    };
    let ca0 = connect_arc(cv0, tan_at(p0), ta_p0, tb_p0)?;
    let p1 = ed.curve.point(a1);
    let ca1 = connect_arc(cv1, tan_at(p1), ta_p1, tb_p1)?;

    let sense = surface.normal(surface.project(ta_p0)).dot(na0) > 0.0;

    Ok(Fillet {
        ta,
        tb,
        ca0,
        ca1,
        ta_p0,
        ta_p1,
        tb_p0,
        tb_p1,
        surface,
        sense,
        fwd_a,
    })
}

/// The arc the rolling ball leaves across the corner: centred on the ball centre,
/// running from one touchdown to the other the short way round, its sweep signed
/// so the arc turns with `along`. Asserts the two touchdowns stand the same
/// distance from the centre, to within `join_agree` -- a rolling ball has one
/// radius, and two answers here mean the corner was solved twice over.
///
/// The plane comes from the **touchdowns**, not from the blended edge. Both are a
/// radius from the centre, so they and the centre fix it exactly, while two edges
/// of one chain agree on their shared tangent only to float noise: 8e-5 rad
/// across a 2.4 mm radius already moves the arc's midpoint past `topo`'s weld
/// quantum, which interned the shared edge twice and left each blend face holding
/// one of them. `along` now only chooses the sweep's sign, a binary call nowhere
/// near flipping, and is the fallback axis only when the two touchdowns are
/// collinear with the centre.
fn connect_arc(center: Vec3, along: Vec3, from_pt: Vec3, to_pt: Vec3) -> Result<CurvEdge, String> {
    let (ra, rb) = ((from_pt - center).length(), (to_pt - center).length());
    assert!(
        (ra - rb).abs() <= join_agree(ra.max(rb)),
        "blend corner: the ball at {center:?} touches down {ra} from its centre on one face \
         and {rb} on the other; a rolling ball has one radius"
    );
    let ref_dir = (from_pt - center).normalize_or(Vec3::X);
    let normal = (from_pt - center).cross(to_pt - center).normalize_or_zero();
    let axis = if normal == Vec3::ZERO {
        along.normalize_or(Vec3::Z)
    } else if normal.dot(along) >= 0.0 {
        normal
    } else {
        -normal
    };
    let d1 = axis.cross(ref_dir);
    let sweep = {
        let v = to_pt - center;
        let mut a = v.dot(d1).atan2(v.dot(ref_dir));
        while a > std::f64::consts::PI {
            a -= 2.0 * std::f64::consts::PI;
        }
        while a < -std::f64::consts::PI {
            a += 2.0 * std::f64::consts::PI;
        }
        a
    };
    Ok(CurvEdge {
        curve: Curve::circle(center, axis, (from_pt - center).length(), ref_dir),
        t0: 0.0,
        t1: sweep,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx(a: Vec3, b: Vec3) -> bool {
        (a - b).length() < 1e-4
    }

    #[test]
    fn connect_arc_endpoints() {
        let center = Vec3::new(8.0, 0.0, 2.0);
        let axis = Vec3::new(0.0, 1.0, 0.0);
        let from = Vec3::new(8.0, 0.0, 0.0);
        let to = Vec3::new(10.0, 0.0, 2.0);
        let ce = connect_arc(center, axis, from, to).expect("a quarter-turn ball corner");
        assert!(approx(ce.curve.point(ce.t0), from), "arc start");
        assert!(approx(ce.curve.point(ce.t1), to), "arc end");
        assert!(((ce.t1 - ce.t0).abs() - std::f64::consts::FRAC_PI_2).abs() < 1e-4);
    }

    #[test]
    fn circle_span_lands_on_its_own_endpoints_not_the_source_edges() {
        let center = Vec3::new(80.0, 4.0, 23.7);
        let axis = Dir::new(Vec3::Z);
        let ref_dir = Dir::new(Vec3::X);
        let p0 = Vec3::new(80.0, 5.45, 23.7);
        let p1 = Vec3::new(78.55, 4.0, 23.7);
        let src = (0.0, -std::f64::consts::FRAC_PI_2);
        let (t0, t1) = circle_span(center, axis, ref_dir, p0, p1, src);
        let c = Curve::circle(center, *axis, 1.45, *ref_dir);
        assert!(approx(c.point(t0), p0), "span start {:?}", c.point(t0));
        assert!(approx(c.point(t1), p1), "span end {:?}", c.point(t1));
    }

    #[test]
    fn circle_span_keeps_a_full_turn_a_full_turn() {
        let center = Vec3::ZERO;
        let p = Vec3::new(3.0, 0.0, 0.0);
        let src = (0.0, std::f64::consts::TAU);
        let (t0, t1) = circle_span(center, Dir::new(Vec3::Z), Dir::new(Vec3::X), p, p, src);
        assert!(
            (t1 - t0 - std::f64::consts::TAU).abs() < 1e-4,
            "sweep {}",
            t1 - t0
        );
    }
}
