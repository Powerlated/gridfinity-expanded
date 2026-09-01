//! Kernel properties held against a real bin.
//!
//! Each test here states something about a `gridfinity-brep` operator -- that
//! `orient::normalize` restores the material-consistency invariant and moves no
//! geometry, that `region2d`'s crossing-box prune discards nothing its exact
//! pass would keep, that `split::trim_half_space` conserves volume and leaves
//! every blend on its own surface -- and checks it on a bin, because a bin is a
//! far harder solid than anything a kernel test writes by hand: rounded
//! corners, a cavity, a peg profile and a floor fillet in one body. The kernel
//! crate cannot state them, knowing nothing about a bin, so they live on this
//! side of the boundary with their fixtures intact.

use gridfinity_brep::geom::Surface;
use gridfinity_brep::math::Vec3;
use gridfinity_brep::orient::{misoriented_loops, normalize};
use gridfinity_brep::region2d::set_verify_prune;
use gridfinity_brep::split::{Side, trim_half_space};
use gridfinity_brep::tess::tessellate;
use gridfinity_brep::topo::Solid;
use gridfinity_model::gridfinity::{self, InnerWall, Params};
use std::collections::HashMap;

/// The plane `x = c`, facing +X.
fn plane_x(c: f64) -> Surface {
    Surface::plane(Vec3::new(c, 0.0, 0.0), Vec3::X)
}

/// `solid`'s enclosed volume in mm^3, from a mesh at `segs` segments per arc.
/// Positive for the outward winding every solid the builder produces has.
fn volume_at(solid: &Solid, segs: usize) -> f64 {
    let mut v = 0.0f64;
    for [a, b, c] in tessellate(solid, segs).to_mesh().triangles() {
        v += a.dot(b.cross(c)) as f64;
    }
    v / 6.0
}

/// `volume_at` at the density the cut tests compare against.
fn volume(solid: &Solid) -> f64 {
    volume_at(solid, 12)
}

/// Panics unless every directed triangle edge of `solid`'s mesh is matched by
/// its reverse, which is what watertight means once the analytic surfaces have
/// been sampled.
fn assert_mesh_closed(solid: &Solid) {
    let mesh = tessellate(solid, 12).to_mesh();
    let mut dir: HashMap<(u32, u32), i32> = HashMap::new();
    for t in mesh.indices.chunks_exact(3) {
        for &(a, b) in &[(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
            *dir.entry((a, b)).or_default() += 1;
        }
    }
    for (&(a, b), &f) in dir.iter() {
        let r = dir.get(&(b, a)).copied().unwrap_or(0);
        assert_eq!(f, r, "edge ({a},{b}) unpaired: {f} vs {r}");
    }
}

#[test]
fn every_bin_the_model_builds_is_already_oriented() {
    for p in [Params::rect(1, 1), Params::rect(3, 2)] {
        let solid = gridfinity::build(&p);
        assert!(
            misoriented_loops(&solid).is_empty(),
            "the model must leave every loop material-consistent"
        );
    }
}

/// A bin's cavity shell is the part `normalize` has to reorient: its faces
/// carry their normals into the material, so a build that got the winding from
/// the outer shell alone would have it backwards there.
#[test]
fn a_cavity_shell_is_the_part_that_needs_reorienting() {
    let mut raw = gridfinity::build(&Params::rect(2, 1));
    let all: Vec<u32> = (0..raw.faces.len()).flat_map(|f| raw.loop_ids(f)).collect();
    for lid in all {
        raw.reverse_loop(lid);
    }
    let flipped = misoriented_loops(&raw).len();
    assert!(flipped > 0, "reversing every loop must break the invariant");
    normalize(&mut raw);
    assert!(misoriented_loops(&raw).is_empty());
    raw.validate()
        .expect("renormalising restores a manifold solid");
}

#[test]
fn normalising_changes_no_geometry() {
    let solid = gridfinity::build(&Params::rect(2, 1));
    let before = volume_at(&solid, 16);
    let mut flipped = solid.clone();
    let all: Vec<u32> = (0..flipped.faces.len())
        .flat_map(|f| flipped.loop_ids(f))
        .collect();
    for lid in all {
        flipped.reverse_loop(lid);
    }
    normalize(&mut flipped);
    let after = volume_at(&flipped, 16);
    assert!(
        (before - after).abs() < 1e-3,
        "orientation must not move geometry: {before} vs {after}"
    );
}

/// `region2d` prunes candidate crossings by bounding box before intersecting
/// exactly. `set_verify_prune` makes it check every pair it discarded, which is
/// quadratic and so off by default; a real build is the input that says the
/// prune keeps everything the exact pass would.
#[test]
fn a_real_build_keeps_every_crossing_the_boxes_pruned() {
    set_verify_prune(true);
    for p in [
        Params::rect(1, 1),
        Params::rect(3, 2),
        Params {
            inner_walls: vec![InnerWall {
                x1: 90.0,
                y1: 30.0,
                x2: 40.0,
                y2: 50.0,
                width: 3.0,
                height: None,
            }],
            ..Params::default()
        },
    ] {
        gridfinity::build(&p);
    }
    set_verify_prune(false);
}

#[test]
fn cutting_a_bin_gives_two_watertight_halves_that_conserve_volume() {
    for cut in [21.0, 42.0, 50.0] {
        let solid = gridfinity::build(&Params::rect(2, 1));
        let plane = plane_x(cut);
        let lo = trim_half_space(&solid, &plane, Side::Negative)
            .unwrap_or_else(|e| panic!("x={cut} negative half: {e}"));
        let hi = trim_half_space(&solid, &plane, Side::Positive)
            .unwrap_or_else(|e| panic!("x={cut} positive half: {e}"));
        lo.validate().expect("low half manifold");
        hi.validate().expect("high half manifold");
        assert_mesh_closed(&lo);
        assert_mesh_closed(&hi);
        let (vw, vl, vh) = (volume(&solid), volume(&lo), volume(&hi));
        assert!(
            vl > 0.0 && vh > 0.0,
            "x={cut}: halves must have volume: {vl} {vh}"
        );
        assert!((vl + vh - vw).abs() < 0.05, "x={cut}: {vl} + {vh} != {vw}");
    }
}

#[test]
fn a_cut_through_a_floor_fillet_keeps_the_blend_on_its_own_surface() {
    let solid = gridfinity::build(&Params::rect(2, 1));
    let plane = plane_x(21.0);
    for keep in [Side::Negative, Side::Positive] {
        let half = trim_half_space(&solid, &plane, keep).expect("half");
        let tess = tessellate(&half, 24);
        for (ti, tri) in tess.tris.iter().enumerate() {
            let face = &half.faces[tess.face_of_tri[ti]];
            let Surface::Cylinder {
                base, axis, radius, ..
            } = face.surface
            else {
                continue;
            };
            let c = (tri.pos[0] + tri.pos[1] + tri.pos[2]) / 3.0;
            let v = c - base;
            let d = (v - *axis * v.dot(*axis)).length();
            assert!(
                (d - radius).abs() < 0.05,
                "a cylinder triangle sits {d} from the axis, not {radius}"
            );
        }
    }
}
