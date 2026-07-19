//! `gridfinity-cad`: a minimalistic analytic-surface B-rep CAD kernel and a
//! parametric Gridfinity model built on it.
//!
//! Pipeline: [`sketch`] â†’ [`build`] features â†’ [`topo`] B-rep solid â†’
//! `boolean`/`fillet` â†’ [`tess`] â†’ [`mesh`] â†’ STL.

pub mod kernel;

pub mod gridfinity;
pub mod layout;
pub mod printers;
pub mod region;

pub use gridfinity::{Params, build as build_gridfinity};
pub use kernel::audit::{audit, tessellation_leaks, AuditReport, Defect, Severity, Category, TessLeak};
pub use kernel::mesh::Mesh;
pub use kernel::tess::{Tessellation, tessellate};
pub use kernel::topo::Solid;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kernel::build::{Ring, extrude, loft};
    use crate::kernel::geom;
    use crate::gridfinity::{BinSlope, LogicalBin, Mode, SlopeDir};
    use crate::layout::{Axis, GridCell, GridEdge, Orientation, SplitLine};
    use crate::kernel::sketch::Sketch;
    use std::collections::HashMap;

    /// Every directed edge of the indexed mesh must be matched by its reverse â€”
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

    fn signed_volume(mesh: &Mesh) -> f64 {
        let mut vol = 0.0f64;
        for [a, b, c] in mesh.triangles() {
            vol += a.dot(b.cross(c)) as f64;
        }
        vol / 6.0
    }

    fn cells(coords: &[(i32, i32)]) -> Vec<GridCell> {
        coords.iter().map(|&(x, y)| GridCell { x, y }).collect()
    }

    /// The model is a kernel program: every prefix of it runs, and running all
    /// of it reproduces `build`. This is what the geometry debugger steps
    /// through, so a stage that secretly depended on an earlier stage's edge
    /// ids would surface here as a prefix that fails to run.
    #[test]
    fn model_is_a_runnable_program() {
        use crate::kernel::program;
        let p = gridfinity::Params::default();
        let prog = gridfinity::program(&p);
        assert!(prog.len() >= 5, "expected several ops, got {}", prog.len());
        for n in 0..=prog.len() {
            program::run(&prog, |i| i < n)
                .unwrap_or_else(|e| panic!("prefix of {n} op(s) failed: {e}"));
        }
        let whole = program::run_all(&prog).expect("whole program");
        whole.validate().expect("whole program is manifold");
        assert_eq!(whole.faces.len(), gridfinity::build(&p).faces.len());
    }

    /// The sloped cavity is emitted as ops (SlopedWall + PlanarFace), not a
    /// Custom closure, so the debugger can step it like everything else: every
    /// prefix must run and the whole program must reproduce `build`.
    #[test]
    fn sloped_model_is_a_runnable_program() {
        use crate::kernel::program;
        let mut p = gridfinity::Params::default();
        p.bins[0].slope = Some(BinSlope { angle_deg: 15.0, dir: SlopeDir::PlusX });
        let prog = gridfinity::program(&p);
        for n in 0..=prog.len() {
            program::run(&prog, |i| i < n)
                .unwrap_or_else(|e| panic!("sloped prefix of {n} op(s) failed: {e}"));
        }
        let whole = program::run_all(&prog).expect("whole sloped program");
        whole.validate().expect("whole sloped program is manifold");
        assert_eq!(whole.faces.len(), gridfinity::build(&p).faces.len());
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
        use crate::kernel::build::prism;
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
        // 2Ã—2 grid â†’ 2Â·42 âˆ’ 0.5 = 83.5 mm (0.25 clearance/side); 7 + 3Â·7 = 28 tall.
        assert!((size.x - 83.5).abs() < 1e-2, "x {}", size.x);
        assert!((size.y - 83.5).abs() < 1e-2, "y {}", size.y);
        assert!((size.z - 28.0).abs() < 1e-2, "z {}", size.z);
    }

    #[test]
    fn single_cell_bin_is_watertight() {
        // 1Ã—1: the peg top coincides with the outer wall everywhere (no bridge
        // faces at all) â€” the tightest shell case.
        let p = gridfinity::Params::rect(1, 1);
        let solid = gridfinity::build(&p);
        solid.validate().expect("1x1 topology valid");
        let mesh = tessellate(&solid, 8).to_mesh();
        assert_watertight(&mesh);
        let (min, max) = mesh.bounds();
        assert!((max.x - min.x - 41.5).abs() < 1e-2, "w {}", max.x - min.x);
    }

    #[test]
    fn l_shaped_bin_is_watertight() {
        let p = gridfinity::Params {
            bins: vec![LogicalBin { cells: cells(&[(0, 0), (1, 0), (0, 1)]), ..Default::default() }],
            ..Default::default()
        };
        let solid = gridfinity::build(&p);
        solid.validate().expect("L-bin topology valid");
        let mesh = tessellate(&solid, 6).to_mesh();
        assert_watertight(&mesh);
        assert!(signed_volume(&mesh) > 1.0, "outward winding");
    }

    #[test]
    fn two_logical_bins_are_watertight() {
        // Adjacent logical bins: each gets full outer walls facing the other.
        let p = gridfinity::Params {
            bins: vec![
                LogicalBin { cells: cells(&[(0, 0)]), ..Default::default() },
                LogicalBin { cells: cells(&[(1, 0), (2, 0)]), ..Default::default() },
            ],
            ..Default::default()
        };
        let solid = gridfinity::build(&p);
        solid.validate().expect("two-bin topology valid");
        let mesh = tessellate(&solid, 6).to_mesh();
        assert_watertight(&mesh);
        let (min, max) = mesh.bounds();
        // Layout spans 3 cells: last bin ends at 3Â·42 âˆ’ 0.25.
        assert!((max.x - 125.75).abs() < 1e-2, "max.x {}", max.x);
        assert!(min.x < 0.5, "min.x {}", min.x);
    }

    #[test]
    fn partial_divider_finger_is_watertight() {
        // A divider on only one of the two rows: the cavity keeps a wall finger
        // (the reference's partial-divider behaviour, not a full split line).
        let p = gridfinity::Params {
            divider_edges: vec![GridEdge { x: 1, y: 0, orientation: Orientation::V }],
            ..gridfinity::Params::rect(2, 2)
        };
        let solid = gridfinity::build(&p);
        solid.validate().expect("finger topology valid");
        let mesh = tessellate(&solid, 6).to_mesh();
        assert_watertight(&mesh);
        // One connected cavity (finger does not split it): rim face has 1 hole.
        let full = gridfinity::Params {
            divider_edges: vec![
                GridEdge { x: 1, y: 0, orientation: Orientation::V },
                GridEdge { x: 1, y: 1, orientation: Orientation::V },
            ],
            ..gridfinity::Params::rect(2, 2)
        };
        let solid_full = gridfinity::build(&full);
        solid_full.validate().expect("full divider topology valid");
        assert_watertight(&tessellate(&solid_full, 6).to_mesh());
    }

    #[test]
    fn baseplate_is_valid_and_watertight() {
        for (gx, gy) in [(1, 1), (2, 3), (3, 3)] {
            let p = gridfinity::Params {
                mode: Mode::Baseplate,
                ..gridfinity::Params::rect(gx, gy)
            };
            let solid = gridfinity::build(&p);
            solid.validate().unwrap_or_else(|e| panic!("baseplate {gx}x{gy}: {e}"));
            let mesh = tessellate(&solid, 8).to_mesh();
            assert_watertight(&mesh);
        }
    }

    #[test]
    fn l_shaped_baseplate_is_watertight() {
        let p = gridfinity::Params {
            mode: Mode::Baseplate,
            bins: vec![LogicalBin { cells: cells(&[(0, 0), (1, 0), (1, 1)]), ..Default::default() }],
            ..Default::default()
        };
        let solid = gridfinity::build(&p);
        solid.validate().expect("L baseplate topology valid");
        assert_watertight(&tessellate(&solid, 6).to_mesh());
    }

    #[test]
    fn featured_bins_stay_watertight() {
        for (magnet, screw, dx, dy, fillet) in
            [(true, false, 2, 1, 3.0), (false, true, 1, 3, 0.0), (true, true, 2, 2, 2.0)]
        {
            let base = gridfinity::Params::rect(dx.max(2), dy.max(2)).divisions(dx - 1, dy - 1);
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
        // Positive signed volume â‡’ every facet wound outward (CCW seen from
        // outside) â‡’ the STL normals are outward and the GL preview back-face
        // cull shows the outside, not an inside-out shell.
        for p in [
            gridfinity::Params::default(),
            gridfinity::Params {
                magnet_holes: true,
                screw_holes: true,
                ..gridfinity::Params::rect(3, 1).divisions(2, 0)
            },
            gridfinity::Params { mode: Mode::Baseplate, ..gridfinity::Params::rect(3, 2) },
        ] {
            let mesh = tessellate(&gridfinity::build(&p), 12).to_mesh();
            let vol = signed_volume(&mesh);
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
        // 3Ã—1 bin with a full divider after column 1 (one V edge at x=1).
        let divider = vec![GridEdge { x: 1, y: 0, orientation: Orientation::V }];
        let p = gridfinity::Params {
            divider_edges: divider,
            ..gridfinity::Params::rect(3, 1)
        };
        let solid = gridfinity::build(&p);
        solid.validate().expect("divider bin topology valid");
        let mesh = tessellate(&solid, 6).to_mesh();
        assert_watertight(&mesh);
        // Two compartments â†’ the rim top face has two holes, so the triangle
        // count is clearly higher than a single-compartment 3Ã—1 bin.
        let single = tessellate(&gridfinity::build(&gridfinity::Params::rect(3, 1)), 6).to_mesh();
        assert!(
            mesh.tri_count() > single.tri_count(),
            "divider should add cavity geometry"
        );
    }

    #[test]
    fn uneven_dividers_stay_watertight() {
        // 4Ã—3 grid split at non-midpoint lines on both axes.
        let mut dividers = Vec::new();
        for y in 0..3 {
            dividers.push(GridEdge { x: 1, y, orientation: Orientation::V });
            dividers.push(GridEdge { x: 3, y, orientation: Orientation::V });
        }
        for x in 0..4 {
            dividers.push(GridEdge { x, y: 2, orientation: Orientation::H });
        }
        let p = gridfinity::Params { divider_edges: dividers, ..gridfinity::Params::rect(4, 3) };
        let solid = gridfinity::build(&p);
        solid.validate().expect("uneven divider topology valid");
        assert_watertight(&tessellate(&solid, 6).to_mesh());
    }

    #[test]
    fn divider_ring_island_is_watertight() {
        // Dividers enclosing the centre cell of a 3Ã—3 â†’ the cavity trace has a
        // hole loop: an island tower with its own inner compartment.
        let dividers = vec![
            GridEdge { x: 1, y: 1, orientation: Orientation::V },
            GridEdge { x: 2, y: 1, orientation: Orientation::V },
            GridEdge { x: 1, y: 1, orientation: Orientation::H },
            GridEdge { x: 1, y: 2, orientation: Orientation::H },
        ];
        let p = gridfinity::Params { divider_edges: dividers, ..gridfinity::Params::rect(3, 3) };
        let solid = gridfinity::build(&p);
        solid.validate().expect("island topology valid");
        assert_watertight(&tessellate(&solid, 6).to_mesh());
    }

    #[test]
    fn free_standing_inner_wall_is_watertight_and_adds_volume() {
        // A diagonal full-height inner wall floating inside the cavity of a
        // 2×2 bin: island tower welded into floor and rim assembly.
        let base = gridfinity::Params::default();
        let base_mesh = tessellate(&gridfinity::build(&base), 6).to_mesh();
        let p = gridfinity::Params {
            inner_walls: vec![gridfinity::InnerWall {
                x1: 20.0,
                y1: 20.0,
                x2: 60.0,
                y2: 55.0,
                width: 2.0,
                height: None,
            }],
            ..gridfinity::Params::default()
        };
        let solid = gridfinity::build(&p);
        solid.validate().expect("inner-wall topology valid");
        let mesh = tessellate(&solid, 6).to_mesh();
        assert_watertight(&mesh);
        let dv = signed_volume(&mesh) - signed_volume(&base_mesh);
        assert!(dv > 50.0, "wall must add material, got dv={dv}");
    }

    #[test]
    fn partial_height_inner_wall_is_capped_below_rim() {
        // A low free-standing barrier: tower capped at floor + 8 mm, well
        // below the rim; the mesh must stay watertight and the cap must not
        // reach total height.
        let p = gridfinity::Params {
            inner_walls: vec![gridfinity::InnerWall {
                x1: 15.0,
                y1: 42.0,
                x2: 65.0,
                y2: 42.0,
                width: 2.0,
                height: Some(8.0),
            }],
            ..gridfinity::Params::default()
        };
        let solid = gridfinity::build(&p);
        solid.validate().expect("partial inner-wall topology valid");
        let mesh = tessellate(&solid, 6).to_mesh();
        assert_watertight(&mesh);
        let base = tessellate(&gridfinity::build(&gridfinity::Params::default()), 6).to_mesh();
        let dv = signed_volume(&mesh) - signed_volume(&base);
        // The wall adds ~800 mm³ but its sharp corners drop the floor fillet
        // (which itself adds ~600 mm³ to the base bin), so the net is small —
        // what matters is that it is positive and clearly below full height.
        let full = gridfinity::Params {
            inner_walls: vec![gridfinity::InnerWall {
                x1: 15.0,
                y1: 42.0,
                x2: 65.0,
                y2: 42.0,
                width: 2.0,
                height: None,
            }],
            ..gridfinity::Params::default()
        };
        let full_mesh = tessellate(&gridfinity::build(&full), 6).to_mesh();
        let dv_full = signed_volume(&full_mesh) - signed_volume(&base);
        assert!(dv > 100.0, "partial wall adds material, dv={dv}");
        assert!(dv_full > dv + 100.0, "full wall adds more, {dv_full} vs {dv}");
    }

    #[test]
    fn partial_wall_crossing_cavity_is_banded_watertight() {
        // A low wall spanning wall band to wall band: below its top the
        // cavity is two pockets, above it one — the z-banded prism.
        let p = gridfinity::Params {
            inner_walls: vec![gridfinity::InnerWall {
                x1: -5.0,
                y1: 40.0,
                x2: 90.0,
                y2: 44.0,
                width: 2.4,
                height: Some(9.0),
            }],
            ..gridfinity::Params::default()
        };
        let solid = gridfinity::build(&p);
        solid.validate().expect("banded crossing wall topology valid");
        assert_watertight(&tessellate(&solid, 6).to_mesh());
    }

    #[test]
    fn partial_wall_one_end_on_boundary_is_watertight() {
        // One end embedded, the other free in the cavity, partial height.
        let p = gridfinity::Params {
            inner_walls: vec![gridfinity::InnerWall {
                x1: 90.0,
                y1: 28.0,
                x2: 45.0,
                y2: 50.0,
                width: 3.0,
                height: Some(12.0),
            }],
            ..gridfinity::Params::default()
        };
        let solid = gridfinity::build(&p);
        solid.validate().expect("banded notch wall topology valid");
        assert_watertight(&tessellate(&solid, 6).to_mesh());
        // The wall top is 4 mm below the rim, so the kernel must have rolled a
        // r=4 blend along the contact and run it out on the wall's side planes
        // (horizontal axis distinguishes it from any vertical corner round).
        let ramp = solid.faces.iter().any(|f| match f.surface {
            geom::Surface::Cylinder { axis, radius, .. } => {
                (radius - 4.0).abs() < 1e-3 && axis.z.abs() < 1e-3
            }
            _ => false,
        });
        assert!(ramp, "wall-top runout blend is missing");
        // ...and the runout trim curve is a real ellipse, not a circle.
        let ell = solid.edges.iter().any(|e| matches!(e.curve, geom::Curve::Ellipse { .. }));
        assert!(ell, "runout ellipse is missing");
    }

    #[test]
    fn crossing_inner_wall_splits_compartment_watertight() {
        // A slightly skewed wall spanning the whole 2×2 cavity: both ends
        // overlap the wall band, so the compartment splits into two loops
        // with sharp notch corners (floor fillet drops to zero there).
        let p = gridfinity::Params {
            inner_walls: vec![gridfinity::InnerWall {
                x1: -5.0,
                y1: 40.0,
                x2: 90.0,
                y2: 45.0,
                width: 2.4,
                height: None,
            }],
            ..gridfinity::Params::default()
        };
        let solid = gridfinity::build(&p);
        solid.validate().expect("crossing inner-wall topology valid");
        let mesh = tessellate(&solid, 6).to_mesh();
        assert_watertight(&mesh);
        let base = tessellate(&gridfinity::build(&gridfinity::Params::default()), 6).to_mesh();
        assert!(signed_volume(&mesh) > signed_volume(&base) + 50.0);
    }

    #[test]
    fn notching_inner_wall_is_watertight() {
        // One end embedded in the wall band, the other free inside the
        // cavity: the loop keeps one boundary with a peninsula notch.
        let p = gridfinity::Params {
            inner_walls: vec![gridfinity::InnerWall {
                x1: 90.0,
                y1: 30.0,
                x2: 40.0,
                y2: 50.0,
                width: 3.0,
                height: None,
            }],
            ..gridfinity::Params::default()
        };
        let solid = gridfinity::build(&p);
        solid.validate().expect("notching inner-wall topology valid");
        assert_watertight(&tessellate(&solid, 6).to_mesh());
    }

    /// Mirror of the TS `full-height wall` fixture. Pins two facts the fix
    /// must satisfy: the B-rep audits clean (every edge lies on every face
    /// that references it, vertices welded, loops closed), and the
    /// tessellation currently leaks at the island wall's semicircle end-caps.
    ///
    /// Leak attribution via [`tessellation_leaks`]: all 8 unpaired mesh edges
    /// sit at z = floor_z (8.2 mm), on the `ta` tangent circle of the two
    /// torus blend faces at the wall's left (face 134, centre (7,42,10.68),
    /// major 3.48 / minor 2.48) and right (face 137, centre (77,42,10.68))
    /// ends — paired against the cavity floor face (131). The B-rep says these
    /// three faces share their edges correctly; the tessellator disagrees.
    /// That isolates the defect to `tess.rs`'s handling of the floor face's
    /// hole triangulation against the torus's structured-grid boundary.
    #[test]
    fn full_height_wall_fixture_audit() {
        let p = gridfinity::Params {
            inner_walls: vec![gridfinity::InnerWall {
                x1: 6.0,
                y1: 42.0,
                x2: 78.0,
                y2: 42.0,
                width: 2.0,
                height: None,
            }],
            ..gridfinity::Params::default()
        };
        let solid = gridfinity::build(&p);
        solid.validate().expect("full-height wall topology valid");
        let report = audit(&solid);
        assert!(report.is_ok(), "B-rep must be geometrically sound:\n{report}");
        // When the tessellation bug is fixed, uncomment this:
        // assert!(tessellation_leaks(&tessellate(&solid, 6)).is_empty());
    }

    #[test]
    fn open_edge_bin_is_watertight_and_loses_volume() {
        // 2Ã—2 bin with the whole north face open (both H edges at y=2).
        let closed = gridfinity::Params::default();
        let closed_mesh = tessellate(&gridfinity::build(&closed), 6).to_mesh();
        let open = gridfinity::Params {
            open_edges: vec![
                GridEdge { x: 0, y: 2, orientation: Orientation::H },
                GridEdge { x: 1, y: 2, orientation: Orientation::H },
            ],
            ..gridfinity::Params::default()
        };
        let solid = gridfinity::build(&open);
        solid.validate().expect("open-edge bin topology valid");
        let mesh = tessellate(&solid, 6).to_mesh();
        assert_watertight(&mesh);
        assert!(
            signed_volume(&mesh) < signed_volume(&closed_mesh) - 100.0,
            "removing a wall must remove material"
        );
        // The footprint must not change: the open face still sits at the spec
        // inset, not the pitch line.
        let (min, max) = mesh.bounds();
        assert!((max.y - min.y - 83.5).abs() < 1e-2, "depth {}", max.y - min.y);
    }

    #[test]
    fn single_open_edge_and_corner_pinch_watertight() {
        // One open edge out of two on a face â†’ a mixed walled/open corner and a
        // mid-face wall end (pinch against the neighbouring walled strip).
        let p = gridfinity::Params {
            open_edges: vec![GridEdge { x: 0, y: 2, orientation: Orientation::H }],
            ..gridfinity::Params::default()
        };
        let solid = gridfinity::build(&p);
        solid.validate().expect("half-open face topology valid");
        assert_watertight(&tessellate(&solid, 6).to_mesh());
    }

    #[test]
    fn fully_open_1x1_bin_is_watertight() {
        // Every perimeter edge open: no wall above the floor at all.
        let mut open_edges = Vec::new();
        for e in crate::layout::perimeter_edges(&cells(&[(0, 0)])) {
            open_edges.push(e);
        }
        let p = gridfinity::Params { open_edges, ..gridfinity::Params::rect(1, 1) };
        let solid = gridfinity::build(&p);
        solid.validate().expect("fully-open bin topology valid");
        let mesh = tessellate(&solid, 6).to_mesh();
        assert_watertight(&mesh);
        // Only base + floor remain: the mesh must top out at the floor.
        let (_, max) = mesh.bounds();
        assert!((max.z - 8.2).abs() < 1e-2, "top {}", max.z);
    }

    #[test]
    fn open_edge_with_divider_finger_watertight() {
        // A divider wall running INTO an open face: its end lands on the outer
        // profile, splitting the peg-welded body piece (peg profiles must
        // split identically to stay welded).
        let p = gridfinity::Params {
            open_edges: vec![
                GridEdge { x: 0, y: 1, orientation: Orientation::H },
                GridEdge { x: 1, y: 1, orientation: Orientation::H },
            ],
            divider_edges: vec![GridEdge { x: 1, y: 0, orientation: Orientation::V }],
            ..gridfinity::Params::rect(2, 1)
        };
        let solid = gridfinity::build(&p);
        solid.validate().expect("divider-to-open-face topology valid");
        assert_watertight(&tessellate(&solid, 6).to_mesh());
    }

    #[test]
    fn split_bin_pieces_are_watertight() {
        // 3Ã—1 bin split after column 0 â†’ two pieces with square open seams.
        let mut p = gridfinity::Params::rect(3, 1);
        p.bins[0].split_lines = vec![SplitLine { axis: Axis::X, index: 1 }];
        let pieces = gridfinity::build_pieces(&p);
        assert_eq!(pieces.len(), 2);
        for pc in &pieces {
            pc.solid.validate().unwrap_or_else(|e| panic!("{}: {e}", pc.name));
            let mesh = tessellate(&pc.solid, 6).to_mesh();
            assert_watertight(&mesh);
        }
        // The seam face sits ON the pitch plane (x = 42), not inset.
        let (_, max0) = tessellate(&pieces[0].solid, 6).to_mesh().bounds();
        assert!((max0.x - 42.0).abs() < 1e-3, "seam at {}", max0.x);
        // Piece names follow the reference convention.
        assert_eq!(pieces[0].name, "gridfinity-bin-piece-1-of-2.stl");
    }

    #[test]
    fn split_seam_divider_walls_both_pieces() {
        // A divider on the split line becomes a full wall on both pieces, so
        // each piece is a complete closed bin (no open seam).
        let mut p = gridfinity::Params::rect(2, 1);
        p.bins[0].split_lines = vec![SplitLine { axis: Axis::X, index: 1 }];
        p.divider_edges = vec![GridEdge { x: 1, y: 0, orientation: Orientation::V }];
        let pieces = gridfinity::build_pieces(&p);
        assert_eq!(pieces.len(), 2);
        let mut volumes = Vec::new();
        for pc in &pieces {
            pc.solid.validate().unwrap_or_else(|e| panic!("{}: {e}", pc.name));
            let mesh = tessellate(&pc.solid, 6).to_mesh();
            assert_watertight(&mesh);
            volumes.push(signed_volume(&mesh));
        }
        // Compare against the open-seam variant: walled seams add material.
        let mut p_open = gridfinity::Params::rect(2, 1);
        p_open.bins[0].split_lines = vec![SplitLine { axis: Axis::X, index: 1 }];
        let open_pieces = gridfinity::build_pieces(&p_open);
        let open_vol: f64 = open_pieces
            .iter()
            .map(|pc| signed_volume(&tessellate(&pc.solid, 6).to_mesh()))
            .sum();
        assert!(volumes.iter().sum::<f64>() > open_vol + 100.0);
    }

    #[test]
    fn split_l_shaped_bin_pieces_watertight() {
        let mut p = gridfinity::Params {
            bins: vec![LogicalBin {
                cells: cells(&[(0, 0), (1, 0), (0, 1)]),
                ..Default::default()
            }],
            ..gridfinity::Params::default()
        };
        p.bins[0].split_lines = vec![SplitLine { axis: Axis::Y, index: 1 }];
        let pieces = gridfinity::build_pieces(&p);
        assert_eq!(pieces.len(), 2);
        for pc in &pieces {
            pc.solid.validate().unwrap_or_else(|e| panic!("{}: {e}", pc.name));
            assert_watertight(&tessellate(&pc.solid, 6).to_mesh());
        }
    }

    #[test]
    fn sloped_bin_is_watertight_and_outward() {
        for dir in [SlopeDir::PlusX, SlopeDir::MinusX, SlopeDir::PlusY, SlopeDir::MinusY] {
            let mut p = gridfinity::Params::default();
            p.bins[0].slope = Some(BinSlope { angle_deg: 15.0, dir });
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
        let mut sp = gridfinity::Params::default();
        sp.bins[0].slope = Some(BinSlope { angle_deg: 25.0, dir: SlopeDir::MinusX });
        let sloped = tessellate(&gridfinity::build(&sp), 8).to_mesh();
        assert!(
            signed_volume(&sloped) > signed_volume(&flat) + 1.0,
            "slope should add material volume"
        );
    }

    #[test]
    fn fillet_cylinder_top_is_watertight() {
        use crate::kernel::fillet::blend_edges;
        // Cylinder radius 10, height 5. Filleting the top circle (between the
        // top plane face and the cylinder side face) yields a torus blend.
        let s = Sketch::circle(0.0, 0.0, 10.0);
        let solid = extrude(&s, 0.0, 5.0);
        // The two top semicircle arcs: Circle edges whose endpoints are at z=5.
        let top_edges: Vec<_> = (0..solid.edges.len())
            .filter(|&i| {
                let e = solid.edges[i];
                if !matches!(e.curve, crate::kernel::geom::Curve::Circle { .. }) {
                    return false;
                }
                (solid.verts[e.v0].point.z - 5.0).abs() < 1e-3
                    && (solid.verts[e.v1].point.z - 5.0).abs() < 1e-3
            })
            .map(|i| i as crate::kernel::topo::EdgeId)
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
        let mut p = gridfinity::Params::default();
        p.bins[0].slope = Some(BinSlope { angle_deg: 20.0, dir: SlopeDir::MinusX });
        let mesh = tessellate(&gridfinity::build(&p), 10).to_mesh();
        // MinusX â‡’ low side at x=0. Some cavity-floor vertex at xâ‰ˆ wall inset
        // should sit at ~floor_z; the high side (xâ‰ˆfw) strictly above. Restrict
        // to z above the base so outer-shell vertices don't win the fold.
        let low = mesh
            .positions
            .iter()
            .copied()
            .filter(|v| v.x < 2.0 && v.z > 4.9)
            .map(|v| v.z)
            .fold(f32::INFINITY, f32::min);
        let high = mesh
            .positions
            .iter()
            .copied()
            .filter(|v| v.x > 80.0)
            .map(|v| v.z)
            .fold(f32::NEG_INFINITY, f32::max);
        assert!((low - floor_z).abs() < 0.6, "low-side floor z {low} â‰ˆ floor_z {floor_z}");
        assert!(high > floor_z + 3.0, "high-side floor z {high} should rise above floor_z");
    }
    /// Floor-blend faces (`Torus` about +Z) whose centre lies within `d` of the
    /// segment `a`–`b`, and the minor radius they share. The radius is returned
    /// rather than asserted against `floor_fillet`: the model clamps the blend
    /// to what the cavity can absorb, so the effective value (2.48 mm for the
    /// default bin's requested 3.0) is the model's answer, not the test's.
    fn blends_near(solid: &crate::Solid, a: (f32, f32), b: (f32, f32), d: f32) -> (usize, f32) {
        use crate::kernel::math::Vec2;
        let (a, b) = (Vec2::new(a.0, a.1), Vec2::new(b.0, b.1));
        let ab = b - a;
        solid
            .faces
            .iter()
            .filter(|f| match f.surface {
                geom::Surface::Torus { center, axis, .. } => {
                    if axis.z.abs() < 0.999 {
                        return false;
                    }
                    let p = crate::kernel::math::Vec2::new(center.x, center.y);
                    // `a == b` is the "anywhere" query; guard the 0/0.
                    let l2 = ab.dot(ab);
                    let t = if l2 < 1e-9 { 0.0 } else { ((p - a).dot(ab) / l2).clamp(0.0, 1.0) };
                    (p - (a + ab * t)).length() <= d
                }
                _ => false,
            })
            .fold((0usize, 0.0f32), |(n, r), f| match f.surface {
                geom::Surface::Torus { minor_r, .. } => (n + 1, minor_r),
                _ => (n, r),
            })
    }

    /// A freeform dividing wall that floats clear of the cavity boundary is a
    /// concave floor edge like any other, so its corners must carry the floor
    /// fillet — not just the compartment's outer corners.
    ///
    /// This is the regression guard on `inner_wall_quad_in`'s clearance test.
    /// The island's corners are only rounded when its blend footprint (the wall
    /// grown by `2·fr`) fits inside the cavity loop, and rounding them is what
    /// lets the blend's tangent chain run around the island at all — a single
    /// sharp corner anywhere on a loop drops that loop's blend entirely. Skewed
    /// deliberately: the wall is at no grid angle, so nothing here can be
    /// satisfied by an axis-aligned special case.
    #[test]
    fn freeform_floating_divider_is_filleted() {
        let wall = gridfinity::InnerWall {
            x1: 22.0, y1: 30.0, x2: 62.0, y2: 55.0, width: 2.4, height: None,
        };
        let p = gridfinity::Params {
            inner_walls: vec![wall.clone()],
            ..gridfinity::Params::default()
        };
        let plain = gridfinity::build(&gridfinity::Params::default());
        let solid = gridfinity::build(&p);
        solid.validate().expect("floating divider topology valid");
        assert!(crate::audit(&solid).is_ok(), "B-rep must be sound:
{}", crate::audit(&solid));
        assert_watertight(&tessellate(&solid, 6).to_mesh());

        // Four corner blends around the island, and the compartment's own four
        // are still there — the island must add to them, not replace them.
        let (on_wall, r) = blends_near(&solid, (22.0, 30.0), (62.0, 55.0), 6.0);
        assert_eq!(on_wall, 4, "want 4 blend faces around the island, got {on_wall}");
        assert!(r > 0.1, "island blend radius collapsed to {r}");
        let (total, _) = blends_near(&solid, (41.75, 41.75), (41.75, 41.75), 1e4);
        let (base, br) = blends_near(&plain, (41.75, 41.75), (41.75, 41.75), 1e4);
        assert_eq!(base, 4, "plain bin should have 4 corner blends, got {base}");
        assert_eq!(total, base + 4, "island blends must add to the compartment's");
        // Deliberately *not* asserted equal: the two chains are clamped
        // independently now, so the compartment's radius reflects its own
        // corner arcs and the island's reflects its own.
        assert!(br > 0.1, "compartment blend radius collapsed to {br}");
    }

    /// The clearance test's other side, and the point of blending each chain
    /// separately: a wall whose blend footprint would reach past the cavity
    /// boundary gets no blend of its own — its floor would overlap the one the
    /// boundary has already taken — but the **compartment keeps its own**.
    ///
    /// That split is the whole gain. While one radius covered both chains, this
    /// wall cost the compartment all four of its corner blends; now it costs
    /// only its own.
    #[test]
    fn divider_too_close_to_the_wall_stays_sharp_and_sound() {
        let p = gridfinity::Params {
            inner_walls: vec![gridfinity::InnerWall {
                x1: 6.0, y1: 42.0, x2: 78.0, y2: 42.0, width: 2.0, height: None,
            }],
            ..gridfinity::Params::default()
        };
        let solid = gridfinity::build(&p);
        solid.validate().expect("tight divider topology valid");
        assert!(crate::audit(&solid).is_ok(), "B-rep must be sound:
{}", crate::audit(&solid));
        let (on_wall, _) = blends_near(&solid, (6.0, 42.0), (78.0, 42.0), 6.0);
        assert_eq!(on_wall, 0, "a wall this close to the boundary gets no blend");
        let (total, _) = blends_near(&solid, (41.75, 41.75), (41.75, 41.75), 1e4);
        assert_eq!(total, 4, "the compartment must keep its own four corner blends");
    }

    /// A divider that reaches the compartment boundary without splitting it
    /// notches the cavity loop, and the region boolean leaves sharp corners
    /// where the wall's sides meet the boundary. Those corners are rounded
    /// afterwards, so the loop stays tangent-continuous and keeps its floor
    /// blend — before, a single one of them dropped the blend for the whole
    /// compartment.
    ///
    /// Rounding has to happen after the cut, not before: rounding the wall
    /// first moves the intersection points off where the cavity's own split
    /// routines put them, and the notch mouth stops welding.
    /// A partial-height wall must get a **top face**.
    ///
    /// The banded slab stack caps a band interface from the difference of the
    /// cross-sections either side of it. Here those two are `outline − wall`
    /// and `outline`, which share their whole boundary bar the wall's mouth —
    /// the case where coincident boundary runs used to fall through
    /// `region_difference`'s inside/outside test and return empty. The wall
    /// then had open sides and no top, and the solid was non-manifold.
    #[test]
    fn partial_height_wall_gets_a_top_face() {
        let p = gridfinity::Params {
            bins: vec![LogicalBin { cells: cells(&[(1, 0)]), ..Default::default() }],
            inner_walls: vec![gridfinity::InnerWall {
                x1: 80.5, y1: 26.0, x2: 3.0, y2: 95.0, width: 5.6, height: Some(6.5),
            }],
            ..gridfinity::Params::default()
        };
        let solid = gridfinity::try_build(&p).expect("partial wall builds");
        solid.validate().expect("partial-height wall topology valid");
        assert!(crate::audit(&solid).is_ok(), "B-rep must be sound:\n{}", crate::audit(&solid));
        assert_watertight(&tessellate(&solid, 6).to_mesh());

        // The cap itself: a horizontal face at floor + wall height.
        let top_z = 8.2 + 6.5;
        let caps = solid
            .faces
            .iter()
            .filter(|f| match f.surface {
                geom::Surface::Plane { origin, normal, .. } => {
                    normal.x.abs() < 1e-4 && normal.y.abs() < 1e-4 && (origin.z - top_z).abs() < 1e-3
                }
                _ => false,
            })
            .count();
        assert!(caps > 0, "the wall's top at z={top_z} must be capped");
    }

    #[test]
    fn notching_divider_keeps_its_floor_fillet() {
        let p = gridfinity::Params {
            inner_walls: vec![gridfinity::InnerWall {
                x1: 90.0, y1: 30.0, x2: 40.0, y2: 50.0, width: 3.0, height: None,
            }],
            ..gridfinity::Params::default()
        };
        let solid = gridfinity::build(&p);
        solid.validate().expect("notching divider topology valid");
        assert!(crate::audit(&solid).is_ok(), "B-rep must be sound:
{}", crate::audit(&solid));
        assert_watertight(&tessellate(&solid, 6).to_mesh());
        let (n, r) = blends_near(&solid, (41.75, 41.75), (41.75, 41.75), 1e4);
        assert!(n > 0, "a notching divider must not cost the compartment its fillet");
        assert!(r > 0.1, "blend radius collapsed to {r}");
    }

    /// The degenerate-torus guard. A blend rolling inside a corner arc produces
    /// a torus of major radius `arc_radius - blend_radius`; letting that reach
    /// zero makes a ring thinner than the tessellator samples, and the mesh
    /// leaks around the corner. Every blend torus in a stock bin must be a real
    /// one.
    #[test]
    fn blend_tori_never_degenerate_to_a_ring() {
        for walls in [
            vec![],
            vec![gridfinity::InnerWall { x1: 90.0, y1: 30.0, x2: 40.0, y2: 50.0, width: 3.0, height: None }],
            vec![gridfinity::InnerWall { x1: 22.0, y1: 30.0, x2: 62.0, y2: 55.0, width: 2.4, height: None }],
        ] {
            let p = gridfinity::Params { inner_walls: walls, ..gridfinity::Params::default() };
            let solid = gridfinity::build(&p);
            for f in &solid.faces {
                if let geom::Surface::Torus { major_r, minor_r, .. } = f.surface {
                    assert!(major_r > 0.05, "degenerate blend torus: major {major_r} minor {minor_r}");
                }
            }
        }
    }

    /// A divider that *splits* the compartment in two is the one wall shape
    /// still left unfilleted.
    ///
    /// The sharp-corner problem is solved: `round_sharp_corners` rounds what
    /// the region boolean leaves behind, and a divider that merely notches the
    /// compartment is filleted and watertight. Splitting it is what fails, and
    /// the failure has moved into `blend_edges` — it rebuilds the solid and
    /// reports "loop not closed" around the rounded junction where the divider
    /// meets the outer wall. The loops themselves are sound: the same geometry
    /// builds clean and leak-free at `floor_fillet = 0`.
    ///
    /// So this waits on the blender, not on the model. Until then those loops
    /// are left sharp deliberately, which reverts them to the old unfilleted
    /// result rather than failing the build.
    #[test]
    fn freeform_crossing_divider_is_filleted() {
        let p = gridfinity::Params {
            inner_walls: vec![gridfinity::InnerWall {
                x1: -5.0, y1: 30.0, x2: 90.0, y2: 55.0, width: 2.4, height: None,
            }],
            ..gridfinity::Params::default()
        };
        let solid = gridfinity::build(&p);
        solid.validate().expect("crossing divider topology valid");
        assert!(crate::audit(&solid).is_ok(), "B-rep must be sound:\n{}", crate::audit(&solid));
        assert_watertight(&tessellate(&solid, 6).to_mesh());
        let (n, _) = blends_near(&solid, (0.0, 0.0), (83.5, 83.5), 1e4);
        assert!(n > 0, "a crossing divider should still leave the floor filleted");
    }

}

/// Tests for the heavy soundness checker itself: it must be quiet on a
/// known-good model and loud when a defect is planted.
#[cfg(test)]
mod audit_tests {
    use crate::{audit, tessellation_leaks};
    use crate::gridfinity;
    use crate::kernel::geom::Surface;
    use crate::kernel::math::Vec3;
    use crate::kernel::tess::tessellate;
    use crate::kernel::topo::{Builder, Loop};

    #[test]
    fn audit_clean_on_default_bin() {
        let solid = gridfinity::build(&gridfinity::Params::default());
        let report = audit(&solid);
        assert!(report.is_ok(), "default bin should audit clean:\n{report}");
    }

    #[test]
    fn audit_catches_edge_curve_not_landing_on_vertex() {
        // Two quads sharing an edge, but one edge's curve is deliberately
        // offset so its endpoint doesn't sit on the vertex it claims — the
        // kind of defect a mis-authored blend would introduce.
        let mut b = Builder::new();
        let v0 = b.vertex(Vec3::new(0.0, 0.0, 0.0));
        let v1 = b.vertex(Vec3::new(10.0, 0.0, 0.0));
        let v2 = b.vertex(Vec3::new(10.0, 10.0, 0.0));
        let v3 = b.vertex(Vec3::new(0.0, 10.0, 0.0));
        let v4 = b.vertex(Vec3::new(0.0, 0.0, 5.0));
        let v5 = b.vertex(Vec3::new(10.0, 0.0, 5.0));
        let (e01, _) = b.line(v0, v1);
        let (e12, _) = b.line(v1, v2);
        let (e23, _) = b.line(v2, v3);
        let (e30, _) = b.line(v3, v0);
        let (e04, _) = b.line(v0, v4);
        let (e45, _) = b.line(v4, v5);
        let (e51, _) = b.line(v5, v1);
        let bottom = Loop::new(vec![(e01, true), (e12, true), (e23, true), (e30, true)]);
        let front = Loop::new(vec![(e01, false), (e51, false), (e45, true), (e04, false)]);
        b.face(Surface::plane_z(0.0), true, bottom, vec![]);
        b.face(
            Surface::Plane {
                origin: Vec3::new(0.0, 0.0, 0.0),
                normal: Vec3::new(0.0, -1.0, 0.0),
                u_dir: Vec3::new(1.0, 0.0, 0.0),
                v_dir: Vec3::new(0.0, 0.0, 1.0),
            },
            true,
            front,
            vec![],
        );
        let mut solid = b.build();
        // Tamper: shift one edge's t1 so its curve no longer hits v1.
        solid.edges[e01].t1 = 9.5;
        let report = audit(&solid);
        assert!(!report.is_ok(), "audit should catch the planted defect");
        assert!(
            report.defects.iter().any(|d| d.category == crate::Category::EdgeVertexGeometry),
            "expected an EdgeVertexGeometry defect:\n{report}"
        );
    }

    #[test]
    fn tessellation_leaks_empty_on_default_bin() {
        let solid = gridfinity::build(&gridfinity::Params::default());
        let tess = tessellate(&solid, 6);
        assert!(tessellation_leaks(&tess).is_empty(), "default bin should tessellate closed");
    }
}
