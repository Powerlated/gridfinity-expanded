//! 2.5D constructive solid geometry: solids as stacks of signed slabs.
//!
//! A [`Slab`] is a 2D region swept over a z-range. A sequence of union /
//! difference slabs is resolved into one B-rep solid **exactly**: the z-range
//! endpoints cut the stack into bands, each band's cross-section is the 2D
//! boolean of the slabs covering it (see [`crate::kernel::region2d`]), and the
//! solid is assembled band by band.
//!
//! This is the restricted boolean the kernel offers instead of general CSG.
//! Both operands being z-prisms is what keeps every intersection curve either
//! vertical or horizontal — so the whole thing stays inside the analytic
//! surface/curve set, with no quartics (which a general cylinder/cylinder
//! boolean would demand).
//!
//! Cones, spheres and tori are *not* expressible here; a chamfered peg stays a
//! `loft`, and a rolling blend stays [`crate::kernel::fillet`].
//!
//! Every region follows the sketch convention: outer loops CCW
//! (`loop_area > 0`), holes CW.

use crate::kernel::build::{RingEdges, ring, wall_seg};
use crate::kernel::region2d::{presplit_regions, region_difference, region_union};
use crate::kernel::sketch::{Seg, loop_area, point_in_segs};
use crate::kernel::geom::Surface;
use crate::kernel::math::{Vec3, vec3_of};
use crate::kernel::topo::{Builder, Loop, Solid};

/// How a slab combines with everything stacked before it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Op {
    Union,
    Difference,
}

/// A 2D region swept between two heights.
#[derive(Clone, Debug)]
pub struct Slab {
    pub region: Vec<Vec<Seg>>,
    pub z0: f32,
    pub z1: f32,
}

impl Slab {
    pub fn new(region: Vec<Vec<Seg>>, z0: f32, z1: f32) -> Slab {
        let (z0, z1) = if z0 <= z1 { (z0, z1) } else { (z1, z0) };
        Slab { region, z0, z1 }
    }
}

const Z_EPS: f32 = 1e-5;

/// How a stack is emitted into a builder that may already hold other geometry.
#[derive(Clone, Debug, Default)]
pub struct SlabOpts {
    /// Emit the stack as a **void** — material outside the regions — instead
    /// of a solid. Used to carve a pocket whose surrounding body is built by
    /// the caller.
    pub cavity: bool,
    /// Heights at which the caller supplies the horizontal face itself (an
    /// opening onto other geometry, e.g. a cavity's rim). No cap is emitted
    /// there, so the caller must close it or `validate` will fail.
    pub open_at: Vec<f32>,
}

/// Resolve a slab stack into one standalone solid. The result is validated
/// before it is returned.
pub fn build_slabs(ops: &[(Op, Slab)]) -> Result<Solid, String> {
    let _perf = crate::kernel::perf::scope(crate::kernel::perf::Metric::BuildSlabs);
    let mut b = Builder::new();
    emit_slabs(&mut b, ops, &SlabOpts::default())?;
    let solid = b.build();
    solid.validate().map_err(|e| format!("slab: {e}"))?;
    Ok(solid)
}

/// The z breakpoints and per-band cross-sections of a stack, computed without
/// emitting anything.
///
/// [`emit_slabs`] is this plus assembly. Callers that need to know a stack's
/// shape *before* building it — to weld their own faces onto an `open_at`
/// interface, say — use this directly.
pub fn plan_bands(ops: &[(Op, Slab)]) -> Result<(Vec<f32>, Vec<Vec<Vec<Seg>>>), String> {
    if ops.is_empty() {
        return Err("slab: empty stack".into());
    }

    // One common segmentation across every operand, so a boundary run shared
    // by two bands comes back as the same pieces in both.
    let split: Vec<Vec<Vec<Seg>>> =
        presplit_regions(&ops.iter().map(|(_, s)| s.region.clone()).collect::<Vec<_>>());

    // Band boundaries: every slab's z endpoints.
    let mut zs: Vec<f32> = ops.iter().flat_map(|(_, s)| [s.z0, s.z1]).collect();
    zs.sort_by(f32::total_cmp);
    zs.dedup_by(|a, b| (*a - *b).abs() < Z_EPS);
    if zs.len() < 2 {
        return Err("slab: stack has no height".into());
    }

    // Cross-section of each band: fold the ops that cover its midpoint.
    let bands: Vec<Vec<Vec<Seg>>> = zs
        .windows(2)
        .map(|w| {
            let mid = (w[0] + w[1]) * 0.5;
            let mut acc: Vec<Vec<Seg>> = Vec::new();
            for (i, (op, s)) in ops.iter().enumerate() {
                if mid <= s.z0 || mid >= s.z1 {
                    continue;
                }
                acc = match op {
                    Op::Union => region_union(&acc, &split[i]),
                    Op::Difference => region_difference(&acc, &split[i]),
                };
            }
            acc
        })
        .collect();

    Ok((zs, bands))
}

/// Emit a slab stack into an existing builder, so it can share edges with
/// geometry the caller builds around it. Returns the per-band cross-sections
/// (bottom band first) — callers need them to weld their own faces onto an
/// `open_at` interface.
pub fn emit_slabs(
    b: &mut Builder,
    ops: &[(Op, Slab)],
    opts: &SlabOpts,
) -> Result<Vec<Vec<Vec<Seg>>>, String> {
    let _perf = crate::kernel::perf::scope(crate::kernel::perf::Metric::EmitSlabs);
    let (zs, bands) = plan_bands(ops)?;

    // Side walls, one span per band. Bands are delimited by *all* z
    // breakpoints, so nothing changes strictly inside a band and the vertical
    // edges need no further splitting.
    //
    // Outers CCW and holes CW both put material on the left, so the outward
    // normal is uniformly to the right of travel — one flag covers every loop
    // and outers/holes need no distinction here. As a cavity the material is
    // on the other side, so the whole thing flips.
    let solid_side = !opts.cavity;
    for (k, band) in bands.iter().enumerate() {
        let (za, zb) = (zs[k], zs[k + 1]);
        for lp in band {
            for s in lp {
                wall_seg(b, s, za, zb, &[], &[], solid_side);
            }
        }
    }

    // Horizontal caps at every interface, treating outside the stack as empty:
    // material below but not above faces up, above but not below faces down
    // (both inverted for a cavity).
    let empty: Vec<Vec<Seg>> = Vec::new();
    for (k, &z) in zs.iter().enumerate() {
        if opts.open_at.iter().any(|&o| (o - z).abs() < Z_EPS) {
            continue; // caller closes this interface
        }
        let below = if k == 0 { &empty } else { &bands[k - 1] };
        let above = if k == bands.len() { &empty } else { &bands[k] };
        for (up, region) in
            [(true, region_difference(below, above)), (false, region_difference(above, below))]
        {
            for (outer, holes) in group_loops(&region) {
                let o = ring(b, &outer, z);
                let hs: Vec<RingEdges> = holes.iter().map(|h| ring(b, h, z)).collect();
                emit_cap(b, z, up, solid_side, &o, &hs);
            }
        }
    }

    Ok(bands)
}

/// A horizontal cap whose *traversal* is fixed by `up` but whose outward
/// normal is `sense`-flipped for a cavity.
///
/// `build::cap` couples the two — its `up` flips normal and winding together —
/// which is wrong here. `wall_seg`'s `outward` flag only flips the surface
/// normal and leaves the loop direction alone, so a cavity's walls traverse
/// exactly like a solid's. Flipping the cap winding too would break the
/// pairing between them; only the normal may flip.
fn emit_cap(
    b: &mut Builder,
    z: f32,
    up: bool,
    sense: bool,
    outer: &RingEdges,
    holes: &[RingEdges],
) {
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
    b.face(surface, sense, mk(outer), holes.iter().map(mk).collect());
}

/// Group a region's loops into `(outer, holes)`, each hole attached to the
/// smallest outer containing it.
///
/// Directions are left **exactly** as the boolean produced them. That matters:
/// a loop can be a hole of one cap and the outer of the band wall above it (a
/// shoulder under a tower), and reorienting it for one role breaks its pairing
/// in the other. The boolean already emits every run material-on-the-left,
/// which is precisely the direction each side needs.
fn group_loops(loops: &[Vec<Seg>]) -> Vec<(Vec<Seg>, Vec<Vec<Seg>>)> {
    let mut out: Vec<(Vec<Seg>, Vec<Vec<Seg>>)> = loops
        .iter()
        .filter(|l| loop_area(l) > 0.0)
        .map(|l| (l.clone(), Vec::new()))
        .collect();
    for h in loops.iter().filter(|l| loop_area(l) < 0.0) {
        let pt = h[0].start();
        let mut best: Option<usize> = None;
        for (i, (o, _)) in out.iter().enumerate() {
            if point_in_segs(pt, o)
                && best.is_none_or(|bi| loop_area(o).abs() < loop_area(&out[bi].0).abs())
            {
                best = Some(i);
            }
        }
        if let Some(bi) = best {
            out[bi].1.push(h.clone());
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kernel::sketch::Sketch;
    use crate::kernel::tess::tessellate;

    fn rect(x0: f32, y0: f32, x1: f32, y1: f32) -> Vec<Vec<Seg>> {
        vec![Sketch::rectangle((x0 + x1) * 0.5, (y0 + y1) * 0.5, x1 - x0, y1 - y0).loops.remove(0)]
    }

    fn circ(cx: f32, cy: f32, r: f32) -> Vec<Vec<Seg>> {
        vec![Sketch::circle(cx, cy, r).loops.remove(0)]
    }

    /// Volume of the tessellated solid. Curved walls are inscribed polygons,
    /// so a bore reads slightly small; 32 segments/quarter keeps that under
    /// ~0.07 mm3 for the radii used here, well inside the tolerances below.
    fn volume(s: &Solid) -> f64 {
        let mesh = tessellate(s, 32).to_mesh();
        let mut v = 0.0f64;
        for [a, b, c] in mesh.triangles() {
            v += a.dot(b.cross(c)) as f64;
        }
        v / 6.0
    }

    fn watertight(s: &Solid) {
        let mesh = tessellate(s, 8).to_mesh();
        let mut dir = std::collections::HashMap::new();
        for t in mesh.indices.chunks_exact(3) {
            for &(a, b) in &[(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                *dir.entry((a, b)).or_insert(0) += 1;
            }
        }
        for (&(a, b), &n) in dir.iter() {
            assert_eq!(n, 1, "edge ({a},{b}) used {n}x");
            assert_eq!(dir.get(&(b, a)).copied().unwrap_or(0), 1, "edge ({a},{b}) unpaired");
        }
    }

    #[test]
    fn single_slab_is_a_box() {
        let s = build_slabs(&[(Op::Union, Slab::new(rect(0.0, 0.0, 10.0, 20.0), 0.0, 5.0))])
            .expect("single slab");
        watertight(&s);
        assert!((volume(&s) - 1000.0).abs() < 1e-2, "vol {}", volume(&s));
    }

    #[test]
    fn stacked_union_different_footprints() {
        // A wide base with a narrower tower: the interface needs a partial
        // up-facing cap (the shoulder) computed as below - above.
        let s = build_slabs(&[
            (Op::Union, Slab::new(rect(0.0, 0.0, 20.0, 20.0), 0.0, 5.0)),
            (Op::Union, Slab::new(rect(5.0, 5.0, 15.0, 15.0), 5.0, 10.0)),
        ])
        .expect("stacked union");
        watertight(&s);
        assert!((volume(&s) - (2000.0 + 500.0)).abs() < 1e-2, "vol {}", volume(&s));
    }

    #[test]
    fn overlapping_union_merges_in_z() {
        // Same z-range, overlapping footprints: one merged prism.
        let s = build_slabs(&[
            (Op::Union, Slab::new(rect(0.0, 0.0, 10.0, 10.0), 0.0, 4.0)),
            (Op::Union, Slab::new(rect(5.0, 5.0, 15.0, 15.0), 0.0, 4.0)),
        ])
        .expect("overlapping union");
        watertight(&s);
        assert!((volume(&s) - 175.0 * 4.0).abs() < 1e-2, "vol {}", volume(&s));
    }

    #[test]
    fn pocket_difference_makes_walls() {
        // The "thin extrude to produce walls" step: a box minus an inset
        // pocket that stops short of the bottom.
        let s = build_slabs(&[
            (Op::Union, Slab::new(rect(0.0, 0.0, 20.0, 20.0), 0.0, 10.0)),
            (Op::Difference, Slab::new(rect(2.0, 2.0, 18.0, 18.0), 2.0, 10.0)),
        ])
        .expect("pocket");
        watertight(&s);
        assert!((volume(&s) - (4000.0 - 256.0 * 8.0)).abs() < 1e-2, "vol {}", volume(&s));
    }

    #[test]
    fn through_bore_is_a_hole() {
        // A cylindrical bore all the way through: exercises arc walls plus a
        // cap with a hole loop.
        let s = build_slabs(&[
            (Op::Union, Slab::new(rect(0.0, 0.0, 20.0, 20.0), 0.0, 5.0)),
            (Op::Difference, Slab::new(circ(10.0, 10.0, 3.0), 0.0, 5.0)),
        ])
        .expect("bore");
        watertight(&s);
        let expect = (400.0 - std::f32::consts::PI as f64 * 9.0) * 5.0;
        assert!((volume(&s) - expect).abs() < 0.2, "vol {} vs {expect}", volume(&s));
    }

    #[test]
    fn cavity_mode_carves_into_caller_geometry() {
        // Exactly how the model would use it: the caller builds the body
        // shell, `emit_slabs` carves the pocket as a void, and the interface
        // the caller closes itself (the rim) is declared open.
        use crate::kernel::build::{cap, ring, wall_between};

        let outer = rect(0.0, 0.0, 20.0, 20.0).remove(0);
        let pocket = rect(2.0, 2.0, 18.0, 18.0);
        let mut b = Builder::new();

        let o_lo = ring(&mut b, &outer, 0.0);
        let o_hi = ring(&mut b, &outer, 10.0);
        wall_between(&mut b, &outer, &outer, &o_lo, &o_hi, 0.0, 10.0, true);
        cap(&mut b, 0.0, false, &o_lo, &[]);

        emit_slabs(
            &mut b,
            &[(Op::Union, Slab::new(pocket.clone(), 2.0, 10.0))],
            &SlabOpts { cavity: true, open_at: vec![10.0] },
        )
        .expect("carve pocket");

        let p_hi = ring(&mut b, &pocket[0], 10.0);
        cap(&mut b, 10.0, true, &o_hi, &[&p_hi]);

        let s = b.build();
        s.validate().expect("carved solid is manifold");
        watertight(&s);
        assert!((volume(&s) - (4000.0 - 2048.0)).abs() < 1e-2, "vol {}", volume(&s));
    }

    #[test]
    fn blind_bore_leaves_a_floor() {
        // Bore stops short of the bottom: needs an up-facing cap at the bore
        // floor AND a down-facing annulus nowhere - the classic counterbore.
        let s = build_slabs(&[
            (Op::Union, Slab::new(rect(0.0, 0.0, 20.0, 20.0), 0.0, 10.0)),
            (Op::Difference, Slab::new(circ(10.0, 10.0, 3.0), 4.0, 10.0)),
        ])
        .expect("blind bore");
        watertight(&s);
        let expect = 4000.0 - std::f32::consts::PI as f64 * 9.0 * 6.0;
        assert!((volume(&s) - expect).abs() < 0.2, "vol {} vs {expect}", volume(&s));
    }
}
