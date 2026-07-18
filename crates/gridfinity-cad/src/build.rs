//! Features: sketches → analytic B-rep solids.
//!
//! Everything is expressed with three primitives that write into a shared
//! [`Builder`] (`ring`, `wall_between`, `cap`), so higher-level models — the
//! whole Gridfinity bin — can assemble many features into one solid with
//! automatically shared edges (no post-hoc stitching). `extrude` / `prism` /
//! `loft` are thin wrappers over those primitives.
//!
//! Orientation convention: every profile loop is authored **CCW**. An `outward`
//! flag says whether solid material is *inside* the loop (`true` → face normals
//! point out of the loop) or *outside* it (`false` → a hole/cavity, normals
//! point into the loop).

use crate::geom::Surface;
use crate::math::{Vec2, Vec3, vec3_of};
use crate::sketch::{Seg, Sketch, loop_area, reverse_loop};
use crate::topo::{Builder, EdgeId, Loop, Solid, VertexId};

/// The vertices and profile edges of one ring (a profile realised at a height).
pub struct RingEdges {
    pub verts: Vec<VertexId>,
    /// One `(edge, forward)` per segment, `forward` = `verts[k] → verts[k+1]`.
    pub edges: Vec<(EdgeId, bool)>,
}

fn ccw(segs: Vec<Seg>) -> Vec<Seg> {
    if loop_area(&segs) < 0.0 {
        reverse_loop(&segs)
    } else {
        segs
    }
}

fn seg_radius(s: &Seg) -> Option<f32> {
    match s {
        Seg::Arc { radius, .. } => Some(*radius),
        _ => None,
    }
}

/// Create the vertices + profile edges for `segs` at height `z`.
pub fn ring(b: &mut Builder, segs: &[Seg], z: f32) -> RingEdges {
    let n = segs.len();
    let verts: Vec<VertexId> = segs
        .iter()
        .map(|s| {
            let p = s.start();
            b.vertex(vec3_of(p.x, p.y, z))
        })
        .collect();
    let mut edges = Vec::with_capacity(n);
    for k in 0..n {
        let k1 = (k + 1) % n;
        edges.push(match segs[k] {
            Seg::Line { .. } => b.line(verts[k], verts[k1]),
            Seg::Arc { center, radius, a0, a1, .. } => {
                b.arc(verts[k], verts[k1], vec3_of(center.x, center.y, z), radius, Vec3::X, a0, a1)
            }
        });
    }
    RingEdges { verts, edges }
}

/// Side faces connecting a lower ring to an upper ring with the same segment
/// structure. `outward` orients the normals (see module docs). Straight runs →
/// `Plane`; arcs → `Cylinder` if the radius is constant, else `Cone`.
pub fn wall_between(
    b: &mut Builder,
    segs_lo: &[Seg],
    segs_hi: &[Seg],
    lo: &RingEdges,
    hi: &RingEdges,
    za: f32,
    zb: f32,
    outward: bool,
) {
    let n = segs_lo.len();
    for k in 0..n {
        let k1 = (k + 1) % n;
        let va = b.line(lo.verts[k], hi.verts[k]);
        let vb = b.line(lo.verts[k1], hi.verts[k1]);
        let (be, bd) = lo.edges[k];
        let (te, td) = hi.edges[k];
        let surface = match segs_lo[k] {
            Seg::Line { a, b: bb } => {
                // General (possibly slanted, for a loft) plane through the quad:
                // lo edge a→b at za and the slant a(za)→a'(zb).
                let a_hi = segs_hi[k].start();
                let p0 = vec3_of(a.x, a.y, za);
                let p1 = vec3_of(bb.x, bb.y, za);
                let p2 = vec3_of(a_hi.x, a_hi.y, zb);
                let mut n = (p1 - p0).cross(p2 - p0).normalize_or_zero();
                // Orient outward: agree with the CCW-outward horizontal (dy,−dx).
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
        let lp = Loop::new(vec![(be, bd), (vb.0, vb.1), (te, !td), (va.0, !va.1)]);
        b.face(surface, outward, lp, vec![]);
    }
}

/// A ring realised as a directed loop, forward (`verts[k]→verts[k+1]`) or
/// reversed. Callers pick the direction so each shared edge is used once in
/// each direction (the manifold invariant); the analytic surface normal — not
/// the loop winding — drives triangle facing, so direction is purely
/// topological.
pub fn loop_of(r: &RingEdges, forward: bool) -> Loop {
    if forward {
        Loop::new(r.edges.clone())
    } else {
        Loop::new(r.edges.iter().rev().map(|&(e, d)| (e, !d)).collect())
    }
}

/// A horizontal planar cap at height `z`. `up` picks the +Z or −Z outward
/// normal. `outer`/`holes` are rings already created at `z`.
pub fn cap(b: &mut Builder, z: f32, up: bool, outer: &RingEdges, holes: &[&RingEdges]) {
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

/// Extrude a single closed profile between `z0` and `z1`.
pub fn extrude(sketch: &Sketch, z0: f32, z1: f32) -> Solid {
    prism(sketch, &[], z0, z1)
}

/// Extrude a region (outer profile + hole profiles) between `z0` and `z1`.
pub fn prism(outer: &Sketch, holes: &[Sketch], z0: f32, z1: f32) -> Solid {
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

/// A profile ring positioned at a height, for `loft`.
pub struct Ring<'a> {
    pub z: f32,
    pub sketch: &'a Sketch,
}

/// Loft through a stack of rings (bottom → top), each with matching segment
/// structure. Builds the chamfered connector-peg foot (planar + conical faces).
pub fn loft(rings: &[Ring]) -> Solid {
    assert!(rings.len() >= 2, "loft needs at least two rings");
    let mut b = Builder::new();
    let leveled: Vec<Vec<Seg>> = rings.iter().map(|r| ccw(r.sketch.loops[0].clone())).collect();
    let re: Vec<RingEdges> = rings
        .iter()
        .zip(&leveled)
        .map(|(r, segs)| ring(&mut b, segs, r.z))
        .collect();
    for i in 0..rings.len() - 1 {
        wall_between(&mut b, &leveled[i], &leveled[i + 1], &re[i], &re[i + 1], rings[i].z, rings[i + 1].z, true);
    }
    cap(&mut b, rings[rings.len() - 1].z, true, &re[rings.len() - 1], &[]);
    cap(&mut b, rings[0].z, false, &re[0], &[]);
    b.build()
}

/// Surface of revolution for an arc run between two levels: cylinder if the
/// radius is constant, otherwise a cone.
fn cone_or_cylinder(center: Vec2, r0: f32, z0: f32, r1: f32, z1: f32) -> Surface {
    if (r1 - r0).abs() < 1e-5 {
        return Surface::Cylinder {
            base: vec3_of(center.x, center.y, 0.0),
            radius: r0,
            ref_dir: Vec3::X,
        };
    }
    let k = (r1 - r0) / (z1 - z0);
    let z_apex = z0 - r0 / k;
    Surface::Cone {
        apex: vec3_of(center.x, center.y, z_apex),
        half_angle: k.abs().atan(),
        ref_dir: Vec3::X,
    }
}
