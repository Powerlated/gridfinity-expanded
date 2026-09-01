use crate::geom::{Curve, Surface};
use crate::math::{Vec2, Vec3, vec3_of};
use crate::sketch::{Seg, Sketch, loop_area, reverse_loop};
use crate::topo::{Builder, EdgeId, Loop, Solid, VertexId};

#[derive(Default, Clone)]
pub struct RingEdges {
    pub verts: Vec<VertexId>,
    pub edges: Vec<(EdgeId, bool)>,
}

fn ccw(segs: Vec<Seg>) -> Vec<Seg> {
    if loop_area(&segs) < 0.0 {
        reverse_loop(&segs)
    } else {
        segs
    }
}

fn seg_radius(s: &Seg) -> Option<f64> {
    match s {
        Seg::Arc { radius, .. } => Some(*radius),
        _ => None,
    }
}

pub fn ring(b: &mut Builder, segs: &[Seg], z: f64) -> RingEdges {
    let mut out = RingEdges::default();
    ring_into(b, segs, z, &mut out);
    out
}

/// `ring` into a caller-owned buffer. Emitting one ring per band allocates two
/// `Vec`s per call, and a loft of thousands of pegs does that millions of times.
pub fn ring_into(b: &mut Builder, segs: &[Seg], z: f64, out: &mut RingEdges) {
    let n = segs.len();
    out.verts.clear();
    out.edges.clear();
    out.verts.reserve(n);
    out.edges.reserve(n);
    for s in segs {
        let p = s.start();
        let v = b.vertex(vec3_of(p.x, p.y, z));
        out.verts.push(v);
    }
    for k in 0..n {
        let k1 = (k + 1) % n;
        let (v0, v1) = (out.verts[k], out.verts[k1]);
        out.edges.push(match segs[k] {
            Seg::Line { .. } => b.line(v0, v1),
            Seg::Arc {
                center,
                radius,
                a0,
                a1,
                ..
            } => b.arc(
                v0,
                v1,
                vec3_of(center.x, center.y, z),
                Vec3::Z,
                radius,
                Vec3::X,
                a0,
                a1,
            ),
        });
    }
}

/// A 2D seg loop lifted onto `plane`, as one edge per segment, with each
/// segment's own vertex at the height the plane gives its start.
///
/// **An arc lifted onto a tilted plane is an ellipse, not a circle.** Its two
/// axes are the arc's radius along x and y, each carrying the plane's gradient
/// in that direction, so `point(t)` at the arc's own angle `t` lands on both the
/// cylinder over the arc and the plane -- and the parameter range carries over
/// unchanged, because the ellipse is parameterised by that same angle. Building
/// it as a Z-axis circle at the *centre's* height instead puts every point of it
/// at one z: the ends then miss the vertices this function placed by the ramp's
/// rise across the arc, which `audit` reports as `EdgeVertexGeometry` and which
/// is why a sloped bin's rounded features had to be left square. `Vec3::Z` is
/// still used where the plane is level, so nothing about a flat ring changes.
pub fn ring_on_plane(b: &mut Builder, segs: &[Seg], plane: (Vec3, Vec3)) -> RingEdges {
    let (origin, normal) = plane;
    let normal = normal.normalize_or(Vec3::Z);
    let level = normal.z.abs() < 1e-9 || (normal.x.abs() < 1e-9 && normal.y.abs() < 1e-9);
    let (gx, gy) = if normal.z.abs() < 1e-9 {
        (0.0, 0.0)
    } else {
        (-normal.x / normal.z, -normal.y / normal.z)
    };
    let z_of = |p: crate::math::Vec2| -> f64 {
        if normal.z.abs() < 1e-9 {
            return origin.z;
        }
        let p3 = vec3_of(p.x, p.y, 0.0);
        (origin - p3).dot(normal) / normal.z
    };
    let n = segs.len();
    let verts: Vec<VertexId> = segs
        .iter()
        .map(|s| {
            let p = s.start();
            b.vertex(vec3_of(p.x, p.y, z_of(p)))
        })
        .collect();
    let mut edges = Vec::with_capacity(n);
    for k in 0..n {
        let k1 = (k + 1) % n;
        edges.push(match segs[k] {
            Seg::Line { .. } => b.line(verts[k], verts[k1]),
            Seg::Arc {
                center,
                radius,
                a0,
                a1,
                ..
            } => {
                let c3 = vec3_of(center.x, center.y, z_of(center));
                if level {
                    b.arc(verts[k], verts[k1], c3, Vec3::Z, radius, Vec3::X, a0, a1)
                } else {
                    let (ea, eb) = (
                        vec3_of(radius, 0.0, radius * gx),
                        vec3_of(0.0, radius, radius * gy),
                    );
                    for (t, s2) in [(a0, segs[k].start()), (a1, segs[k1].start())] {
                        let on_curve = c3 + t.cos() * ea + t.sin() * eb;
                        let at_vert = vec3_of(s2.x, s2.y, z_of(s2));
                        assert!(
                            (on_curve - at_vert).length() <= 1e-3,
                            "ring on a tilted plane: the lifted arc reaches {on_curve:?} at its \
                             own angle {t} but the ring puts that end at {at_vert:?}; the two \
                             are one point, which is what makes the lift an ellipse in the \
                             arc's own parameter"
                        );
                    }
                    let (curve, s0, s1) =
                        Curve::ellipse_from_conjugate(c3, ea, eb, a0, a1);
                    b.ellipse(verts[k], verts[k1], curve, s0, s1)
                }
            }
        });
    }
    RingEdges { verts, edges }
}

pub fn wall_between(
    b: &mut Builder,
    segs_lo: &[Seg],
    segs_hi: &[Seg],
    lo: &RingEdges,
    hi: &RingEdges,
    za: f64,
    zb: f64,
    outward: bool,
) {
    let n = segs_lo.len();
    // The band is a quad per seg, so the two rings have to agree segment for
    // segment: a mismatch here does not fail, it silently pairs seg k of one
    // ring with seg k of a different subdivision and the wall comes out
    // twisted. The rings must also be the same length as their seg lists, since
    // `lo.edges[k]` is the edge under `segs_lo[k]`.
    assert_eq!(
        n,
        segs_hi.len(),
        "a wall's two rings have different segment counts"
    );
    assert!(n >= 1, "a wall needs a ring, got no segments");
    // Closed, not merely long enough: a cylinder's ring is two semicircles and a
    // circle's is one segment, so a minimum count says nothing. The band's quads
    // only tile if each seg ends where the next begins.
    for (ring, which) in [(segs_lo, "lower"), (segs_hi, "upper")] {
        for k in 0..n {
            let (a, b) = (ring[k], ring[(k + 1) % n]);
            assert!(
                (a.end() - b.start()).length() < 1e-3,
                "a wall's {which} ring is open between segment {k} and {}: {a:?} then {b:?}",
                (k + 1) % n
            );
        }
    }
    assert_eq!(
        (lo.edges.len(), hi.edges.len()),
        (n, n),
        "a wall's ring edges do not match its {n} segment(s)"
    );
    assert_eq!(
        (lo.verts.len(), hi.verts.len()),
        (n, n),
        "a wall's ring vertices do not match its {n} segment(s)"
    );
    // Each vertical is shared by two side faces, so carry it forward rather
    // than re-interning it: a peg loft's bands are half the kernel's edge
    // lookups.
    let first_v = b.line(lo.verts[0], hi.verts[0]);
    let mut va = first_v;
    for k in 0..n {
        let k1 = (k + 1) % n;
        let vb = if k1 == 0 {
            first_v
        } else {
            b.line(lo.verts[k1], hi.verts[k1])
        };
        let (be, bd) = lo.edges[k];
        let (te, td) = hi.edges[k];
        let surface = match segs_lo[k] {
            Seg::Line { a, b: bb } => {
                let p0 = b.point(lo.verts[k]);
                let p1 = b.point(lo.verts[k1]);
                let p2 = b.point(hi.verts[k]);
                let _ = (a, bb, za, zb);
                let mut n = (p1 - p0).cross(p2 - p0).normalize_or_zero();
                let dir = bb - a;
                if n.dot(Vec3::new(dir.y, -dir.x, 0.0)) < 0.0 {
                    n = -n;
                }
                Surface::plane(p0, n)
            }
            Seg::Arc { center, radius, .. } => {
                let r_hi = seg_radius(&segs_hi[k]).unwrap_or(radius);
                cone_or_cylinder(center, radius, za, r_hi, zb)
            }
        };
        let sense = match segs_lo[k] {
            Seg::Arc { a0, a1, .. } => outward == (a1 > a0),
            _ => outward,
        };
        let lp = [(be, bd), (vb.0, vb.1), (te, !td), (va.0, !va.1)];
        b.face_from(surface, sense, &lp, &[]);
        va = vb;
    }
}

pub fn seg_edge(b: &mut Builder, seg: &Seg, z: f64) -> (EdgeId, bool) {
    let v0 = b.vertex(vec3_of(seg.start().x, seg.start().y, z));
    let v1 = b.vertex(vec3_of(seg.end().x, seg.end().y, z));
    match *seg {
        Seg::Line { .. } => b.line(v0, v1),
        Seg::Arc {
            center,
            radius,
            a0,
            a1,
            ..
        } => b.arc(
            v0,
            v1,
            vec3_of(center.x, center.y, z),
            Vec3::Z,
            radius,
            Vec3::X,
            a0,
            a1,
        ),
    }
}

pub fn wall_seg(
    b: &mut Builder,
    seg: &Seg,
    za: f64,
    zb: f64,
    breaks_l: &[f64],
    breaks_r: &[f64],
    outward: bool,
) {
    let (p0, p1) = (seg.start(), seg.end());
    let bottom = seg_edge(b, seg, za);
    let top = seg_edge(b, seg, zb);
    let chain = |b: &mut Builder, p: Vec2, breaks: &[f64]| -> Vec<(EdgeId, bool)> {
        let mut hs: Vec<f64> = vec![za];
        hs.extend(
            breaks
                .iter()
                .copied()
                .filter(|&h| h > za + 1e-4 && h < zb - 1e-4),
        );
        hs.sort_by(f64::total_cmp);
        hs.push(zb);
        hs.dedup_by(|x, y| (*x - *y).abs() < 1e-4);
        let mut out = Vec::new();
        for w in hs.windows(2) {
            let v0 = b.vertex(vec3_of(p.x, p.y, w[0]));
            let v1 = b.vertex(vec3_of(p.x, p.y, w[1]));
            out.push(b.line(v0, v1));
        }
        out
    };
    let left = chain(b, p0, breaks_l);
    let right = chain(b, p1, breaks_r);
    let surface = match *seg {
        Seg::Line { a, b: bb } => {
            let dir = bb - a;
            Surface::plane(vec3_of(a.x, a.y, za), Vec3::new(dir.y, -dir.x, 0.0))
        }
        Seg::Arc { center, radius, .. } => {
            Surface::cylinder_z(vec3_of(center.x, center.y, 0.0), radius)
        }
    };
    let sense = match *seg {
        Seg::Arc { a0, a1, .. } => outward == (a1 > a0),
        _ => outward,
    };
    let mut lp: Vec<(EdgeId, bool)> = vec![bottom];
    lp.extend(right.iter().copied());
    lp.push((top.0, !top.1));
    lp.extend(left.iter().rev().map(|&(e, d)| (e, !d)));
    b.face(surface, sense, Loop::new(lp), vec![]);
}

pub fn loop_of(r: &RingEdges, forward: bool) -> Loop {
    if forward {
        Loop::new(r.edges.clone())
    } else {
        Loop::new(r.edges.iter().rev().map(|&(e, d)| (e, !d)).collect())
    }
}

pub fn cap(b: &mut Builder, z: f64, up: bool, outer: &RingEdges, holes: &[&RingEdges]) {
    let surface = if up {
        Surface::plane_z(z)
    } else {
        Surface::plane(vec3_of(0.0, 0.0, z), -Vec3::Z)
    };
    let mk = |r: &RingEdges| -> Loop {
        if up {
            Loop::new(r.edges.clone())
        } else {
            Loop::new(r.edges.iter().rev().map(|&(e, d)| (e, !d)).collect())
        }
    };
    let outer_loop = mk(outer);
    let inner_loops = holes.iter().map(|h| mk(h)).collect();
    b.face(surface, true, outer_loop, inner_loops);
}

pub fn extrude(sketch: &Sketch, z0: f64, z1: f64) -> Solid {
    prism(sketch, &[], z0, z1)
}

pub fn prism(outer: &Sketch, holes: &[Sketch], z0: f64, z1: f64) -> Solid {
    let mut b = Builder::new();
    let outer_segs = ccw(outer.loops[0].clone());
    let hole_segs: Vec<Vec<Seg>> = holes.iter().map(|h| ccw(h.loops[0].clone())).collect();

    let o_lo = ring(&mut b, &outer_segs, z0);
    let o_hi = ring(&mut b, &outer_segs, z1);
    wall_between(&mut b, &outer_segs, &outer_segs, &o_lo, &o_hi, z0, z1, true);

    let mut h_lo = Vec::new();
    let mut h_hi = Vec::new();
    for hs in &hole_segs {
        let lo = ring(&mut b, hs, z0);
        let hi = ring(&mut b, hs, z1);
        wall_between(&mut b, hs, hs, &lo, &hi, z0, z1, false);
        h_lo.push(lo);
        h_hi.push(hi);
    }

    let hi_refs: Vec<&RingEdges> = h_hi.iter().collect();
    let lo_refs: Vec<&RingEdges> = h_lo.iter().collect();
    cap(&mut b, z1, true, &o_hi, &hi_refs);
    cap(&mut b, z0, false, &o_lo, &lo_refs);
    b.build()
}

pub struct Ring<'a> {
    pub z: f64,
    pub sketch: &'a Sketch,
}

pub fn loft(rings: &[Ring]) -> Solid {
    assert!(rings.len() >= 2, "loft needs at least two rings");
    let mut b = Builder::new();
    let leveled: Vec<Vec<Seg>> = rings
        .iter()
        .map(|r| ccw(r.sketch.loops[0].clone()))
        .collect();
    let re: Vec<RingEdges> = rings
        .iter()
        .zip(&leveled)
        .map(|(r, segs)| ring(&mut b, segs, r.z))
        .collect();
    for i in 0..rings.len() - 1 {
        wall_between(
            &mut b,
            &leveled[i],
            &leveled[i + 1],
            &re[i],
            &re[i + 1],
            rings[i].z,
            rings[i + 1].z,
            true,
        );
    }
    cap(
        &mut b,
        rings[rings.len() - 1].z,
        true,
        &re[rings.len() - 1],
        &[],
    );
    cap(&mut b, rings[0].z, false, &re[0], &[]);
    b.build()
}

fn cone_or_cylinder(center: Vec2, r0: f64, z0: f64, r1: f64, z1: f64) -> Surface {
    if (r1 - r0).abs() < 1e-5 {
        return Surface::cylinder_z(vec3_of(center.x, center.y, 0.0), r0);
    }
    let k = (r1 - r0) / (z1 - z0);
    let ((z_at, r_at), open) = if r1 > r0 {
        ((z1, r1), Vec3::Z)
    } else {
        ((z0, r0), -Vec3::Z)
    };
    assert!(
        r_at > 0.0,
        "a cone face is named at the end where it is wider, which has a positive radius, but \
         the ring pair ({r0} at {z0}, {r1} at {z1}) is widest at {r_at}"
    );
    Surface::cone(
        vec3_of(center.x, center.y, z_at),
        open,
        r_at,
        k.abs().atan(),
        Vec3::X,
    )
}
