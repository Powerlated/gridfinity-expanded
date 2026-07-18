//! `gridfinity-cad`: a minimalistic analytic-surface B-rep CAD kernel and a
//! parametric Gridfinity model built on it.
//!
//! Pipeline: [`sketch`] → [`build`] features → [`topo`] B-rep solid →
//! `boolean`/`fillet` → [`tess`] → [`mesh`] → STL.

pub mod build;
pub mod fillet;
pub mod geom;
pub mod gridfinity;
pub mod layout;
pub mod math;
pub mod mesh;
pub mod printers;
pub mod region;
pub mod sketch;
pub mod tess;
pub mod topo;

pub use gridfinity::{Params, build as build_gridfinity};
pub use mesh::Mesh;
pub use tess::{Tessellation, tessellate};
pub use topo::Solid;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::build::{Ring, extrude, loft};
    use crate::sketch::Sketch;
    use std::collections::HashMap;

    /// Every directed edge of the indexed mesh must be matched by its reverse —
    /// the watertight / 2-manifold invariant.
    fn assert_watertight(mesh: &Mesh) {
        let mut dir: HashMap<(u32, u32), i32> = HashMap::new();
        for t in mesh.indices.chunks_exact(3) {
            for &(a, b) in &[(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                *dir.entry((a, b)).or_default() += 1;
            }
        }
        for (&(a, b), _) in dir.iter() {
            let fwd = dir.get(&(a, b)).copied().unwrap_or(0);
            let bwd = dir.get(&(b, a)).copied().unwrap_or(0);
            assert_eq!(fwd, bwd, "edge ({a},{b}) not balanced: fwd={fwd} bwd={bwd}");
            assert_eq!(fwd, 1, "edge ({a},{b}) used {fwd} times (want 1)");
        }
    }

    #[test]
    fn box_prism_is_valid_and_watertight() {
        let s = Sketch::rectangle(0.0, 0.0, 10.0, 20.0);
        let solid = extrude(&s, 0.0, 5.0);
        solid.validate().expect("box topology valid");

        let mesh = tessellate(&solid, 8).to_mesh();
        assert_watertight(&mesh);

        let (min, max) = mesh.bounds();
        let size = max - min;
        assert!((size.x - 10.0).abs() < 1e-3, "x {}", size.x);
        assert!((size.y - 20.0).abs() < 1e-3, "y {}", size.y);
        assert!((size.z - 5.0).abs() < 1e-3, "z {}", size.z);
    }

    #[test]
    fn rounded_rect_prism_is_valid_and_watertight() {
        let s = Sketch::rounded_rect(0.0, 0.0, 40.0, 30.0, 5.0);
        let solid = extrude(&s, 0.0, 7.0);
        solid.validate().expect("rounded-rect topology valid");
        let mesh = tessellate(&solid, 8).to_mesh();
        assert_watertight(&mesh);
        let (min, max) = mesh.bounds();
        let size = max - min;
        assert!((size.x - 40.0).abs() < 1e-3, "x {}", size.x);
        assert!((size.y - 30.0).abs() < 1e-3, "y {}", size.y);
    }

    #[test]
    fn foot_loft_is_valid_and_watertight() {
        // Chamfered peg-ish foot: three stacked rounded-rect rings.
        let r0 = Sketch::rounded_rect(0.0, 0.0, 35.6, 35.6, 0.8);
        let r1 = Sketch::rounded_rect(0.0, 0.0, 37.2, 37.2, 1.5);
        let r2 = Sketch::rounded_rect(0.0, 0.0, 41.5, 41.5, 3.75);
        let solid = loft(&[
            Ring { z: 0.0, sketch: &r0 },
            Ring { z: 0.8, sketch: &r1 },
            Ring { z: 2.6, sketch: &r1 },
            Ring { z: 4.75, sketch: &r2 },
        ]);
        solid.validate().expect("foot topology valid");
        let mesh = tessellate(&solid, 6).to_mesh();
        assert_watertight(&mesh);
    }

    #[test]
    fn annulus_prism_watertight() {
        use crate::build::prism;
        let outer = Sketch::rectangle(0.0, 0.0, 40.0, 40.0);
        let hole = Sketch::rectangle(0.0, 0.0, 20.0, 20.0);
        let solid = prism(&outer, &[hole], 0.0, 10.0);
        solid.validate().expect("annulus topology valid");
        let mesh = tessellate(&solid, 6).to_mesh();
        assert_watertight(&mesh);
    }

    #[test]
    fn default_bin_is_valid_watertight_and_sized() {
        let p = gridfinity::Params::default();
        let solid = gridfinity::build(&p);
        solid.validate().expect("default bin topology valid");
        let mesh = tessellate(&solid, 8).to_mesh();
        assert_watertight(&mesh);
        let (min, max) = mesh.bounds();
        let size = max - min;
        // 2×2 grid → 2·42 − 0.5 = 83.5 mm (0.25 clearance/side); 7 + 3·7 = 28 tall.
        assert!((size.x - 83.5).abs() < 1e-2, "x {}", size.x);
        assert!((size.y - 83.5).abs() < 1e-2, "y {}", size.y);
        assert!((size.z - 28.0).abs() < 1e-2, "z {}", size.z);
    }

    #[test]
    fn baseplate_is_valid_and_watertight() {
        for (gx, gy) in [(1, 1), (2, 3), (3, 3)] {
            let p = gridfinity::Params {
                mode: gridfinity::Mode::Baseplate,
                grid_x: gx,
                grid_y: gy,
                ..Default::default()
            };
            let solid = gridfinity::build(&p);
            solid.validate().unwrap_or_else(|e| panic!("baseplate {gx}x{gy}: {e}"));
            let mesh = tessellate(&solid, 8).to_mesh();
            assert_watertight(&mesh);
        }
    }

    #[test]
    fn featured_bins_stay_watertight() {
        for (magnet, screw, dx, dy, fillet) in
            [(true, false, 2, 1, 3.0), (false, true, 1, 3, 0.0), (true, true, 2, 2, 2.0)]
        {
            let base = gridfinity::Params::default().divisions(dx, dy);
            let p = gridfinity::Params {
                magnet_holes: magnet,
                screw_holes: screw,
                floor_fillet: fillet,
                ..base
            };
            let solid = gridfinity::build(&p);
            solid
                .validate()
                .unwrap_or_else(|e| panic!("topology {magnet}/{screw}/{dx}x{dy}: {e}"));
            let mesh = tessellate(&solid, 6).to_mesh();
            assert_watertight(&mesh);
        }
    }

    #[test]
    fn meshes_have_outward_consistent_winding() {
        // Positive signed volume ⇒ every facet wound outward (CCW seen from
        // outside) ⇒ the STL normals are outward and the GL preview back-face
        // cull shows the outside, not an inside-out shell.
        for p in [
            gridfinity::Params::default(),
            gridfinity::Params { magnet_holes: true, screw_holes: true, ..gridfinity::Params::default().divisions(3, 1) },
            gridfinity::Params { mode: gridfinity::Mode::Baseplate, grid_x: 3, grid_y: 2, ..Default::default() },
        ] {
            let mesh = tessellate(&gridfinity::build(&p), 12).to_mesh();
            let mut vol = 0.0f64;
            for [a, b, c] in mesh.triangles() {
                vol += a.dot(b.cross(c)) as f64;
            }
            vol /= 6.0;
            assert!(vol > 1.0, "expected positive volume, got {vol}");
        }
    }

    #[test]
    fn stl_export_roundtrip() {
        let mesh = tessellate(&gridfinity::build(&gridfinity::Params::default()), 24).to_mesh();
        let stl = mesh.to_stl_binary();
        let header_tris = u32::from_le_bytes(stl[80..84].try_into().unwrap()) as usize;
        assert_eq!(header_tris, mesh.tri_count());
        assert_eq!(stl.len(), 84 + 50 * header_tris);
        assert!(header_tris > 200, "a real bin should have many facets, got {header_tris}");
    }

    #[test]
    fn stl_length_matches_tri_count() {
        let s = Sketch::rectangle(0.0, 0.0, 5.0, 5.0);
        let mesh = tessellate(&extrude(&s, 0.0, 5.0), 4).to_mesh();
        let stl = mesh.to_stl_binary();
        assert_eq!(stl.len(), 84 + 50 * mesh.tri_count());
    }

    #[test]
    fn divider_edges_split_bin_is_watertight() {
        use crate::layout::{GridEdge, Orientation};
        // 3×1 bin with a full divider after column 1 (one V edge at x=1).
        let divider = vec![GridEdge { x: 1, y: 0, orientation: Orientation::V }];
        let p = gridfinity::Params {
            grid_x: 3,
            grid_y: 1,
            divider_edges: divider,
            ..Default::default()
        };
        let solid = gridfinity::build(&p);
        solid.validate().expect("divider bin topology valid");
        let mesh = tessellate(&solid, 6).to_mesh();
        assert_watertight(&mesh);
        // Two compartments → the rim top face has two holes, so the triangle
        // count is clearly higher than a single-compartment 3×1 bin.
        let single = tessellate(
            &gridfinity::build(&gridfinity::Params { grid_x: 3, grid_y: 1, ..Default::default() }),
            6,
        )
        .to_mesh();
        assert!(
            mesh.tri_count() > single.tri_count(),
            "divider should add cavity geometry"
        );
    }

    #[test]
    fn uneven_dividers_stay_watertight() {
        use crate::layout::{GridEdge, Orientation};
        // 4×3 grid split at non-midpoint lines on both axes.
        let mut dividers = Vec::new();
        for y in 0..3 {
            dividers.push(GridEdge { x: 1, y, orientation: Orientation::V });
            dividers.push(GridEdge { x: 3, y, orientation: Orientation::V });
        }
        for x in 0..4 {
            dividers.push(GridEdge { x, y: 2, orientation: Orientation::H });
        }
        let p = gridfinity::Params { grid_x: 4, grid_y: 3, divider_edges: dividers, ..Default::default() };
        let solid = gridfinity::build(&p);
        solid.validate().expect("uneven divider topology valid");
        assert_watertight(&tessellate(&solid, 6).to_mesh());
    }

    fn signed_volume(mesh: &Mesh) -> f64 {
        let mut vol = 0.0f64;
        for [a, b, c] in mesh.triangles() {
            vol += a.dot(b.cross(c)) as f64;
        }
        vol / 6.0
    }

    #[test]
    fn sloped_bin_is_watertight_and_outward() {
        for dir in [
            gridfinity::SlopeDir::PlusX,
            gridfinity::SlopeDir::MinusX,
            gridfinity::SlopeDir::PlusY,
            gridfinity::SlopeDir::MinusY,
        ] {
            let p = gridfinity::Params {
                slope: Some(gridfinity::BinSlope { angle_deg: 15.0, dir }),
                ..Default::default()
            };
            let solid = gridfinity::build(&p);
            solid.validate().unwrap_or_else(|e| panic!("slope {dir:?}: {e}"));
            let mesh = tessellate(&solid, 6).to_mesh();
            assert_watertight(&mesh);
            let vol = signed_volume(&mesh);
            assert!(vol > 1.0, "slope {dir:?}: expected positive volume, got {vol}");
        }
    }

    #[test]
    fn sloped_floor_displaces_volume() {
        // A slope fills part of the cavity, so the sloped solid has strictly
        // more material volume than the flat one.
        let flat = tessellate(&gridfinity::build(&gridfinity::Params::default()), 8).to_mesh();
        let sloped = tessellate(
            &gridfinity::build(&gridfinity::Params {
                slope: Some(gridfinity::BinSlope { angle_deg: 25.0, dir: gridfinity::SlopeDir::MinusX }),
                ..Default::default()
            }),
            8,
        )
        .to_mesh();
        assert!(
            signed_volume(&sloped) > signed_volume(&flat) + 1.0,
            "slope should add material volume"
        );
    }

    #[test]
    fn fillet_cylinder_top_is_watertight() {
        use crate::fillet::blend_edges;
        // Cylinder radius 10, height 5. Filleting the top circle (between the
        // top plane face and the cylinder side face) yields a torus blend.
        let s = Sketch::circle(0.0, 0.0, 10.0);
        let solid = extrude(&s, 0.0, 5.0);
        // The two top semicircle arcs: Circle edges whose endpoints are at z=5.
        let top_edges: Vec<_> = (0..solid.edges.len())
            .filter(|&i| {
                let e = solid.edges[i];
                if !matches!(e.curve, crate::geom::Curve::Circle { .. }) {
                    return false;
                }
                (solid.verts[e.v0].point.z - 5.0).abs() < 1e-3
                    && (solid.verts[e.v1].point.z - 5.0).abs() < 1e-3
            })
            .map(|i| i as crate::topo::EdgeId)
            .collect();
        assert_eq!(top_edges.len(), 2, "cylinder top should have 2 arc edges");
        let blends: Vec<_> = top_edges.iter().map(|&e| (e, 2.0_f32)).collect();
        let blended = blend_edges(&solid, &blends).expect("cylinder top fillet");
        blended.validate().expect("blended topology valid");
        let mesh = tessellate(&blended, 8).to_mesh();
        assert_watertight(&mesh);
    }

    #[test]
    fn sloped_floor_low_side_is_at_floor_z() {
        let floor_z = gridfinity::BASE_TOTAL_HEIGHT + gridfinity::FLOOR_THICKNESS;
        let p = gridfinity::Params {
            slope: Some(gridfinity::BinSlope { angle_deg: 20.0, dir: gridfinity::SlopeDir::MinusX }),
            ..Default::default()
        };
        let mesh = tessellate(&gridfinity::build(&p), 10).to_mesh();
        // MinusX ⇒ low side at x=0. Some cavity-floor vertex at x≈ wall inset
        // should sit at ~floor_z; the high side (x≈fw) strictly above. Restrict
        // to z above the foot so outer-shell vertices don't win the fold.
        let low = mesh.positions.iter().copied().filter(|v| v.x < 2.0 && v.z > 4.9).map(|v| v.z).fold(f32::INFINITY, f32::min);
        let high = mesh.positions.iter().copied().filter(|v| v.x > 80.0).map(|v| v.z).fold(f32::NEG_INFINITY, f32::max);
        assert!((low - floor_z).abs() < 0.6, "low-side floor z {low} ≈ floor_z {floor_z}");
        assert!(high > floor_z + 3.0, "high-side floor z {high} should rise above floor_z");
    }
}
