

pub mod kernel;

#[cfg(feature = "badapple")]
pub mod badapple;
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
#[global_allocator]
static TEST_ALLOC: kernel::perf::CountingAlloc<mimalloc::MiMalloc> =
    kernel::perf::CountingAlloc::new(mimalloc::MiMalloc);

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kernel::build::{Ring, extrude, loft};
    use crate::kernel::geom;
    use crate::gridfinity::{BinSlope, LogicalBin, Mode, SlopeDir};
    use crate::layout::{Axis, GridCell, GridEdge, Orientation, SplitLine};
    use crate::kernel::sketch::Sketch;
    use std::collections::HashMap;

    static PERF_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    fn perf_guard() -> std::sync::MutexGuard<'static, ()> {
        PERF_LOCK.lock().unwrap_or_else(|e| e.into_inner())
    }

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
    fn the_welded_render_buffer_carries_the_welded_mesh_positions_and_analytic_normals() {
        let s = Sketch::rounded_rect(0.0, 0.0, 40.0, 30.0, 5.0);
        let tess = tessellate(&extrude(&s, 0.0, 7.0), 8);
        let mesh = tess.to_mesh();
        let welded: std::collections::HashSet<(i64, i64, i64)> = mesh
            .positions
            .iter()
            .map(|&p| kernel::math::weld_key(p))
            .collect();

        let buffer = tess.welded_render_buffer();
        assert_eq!(buffer.len(), mesh.indices.len() * 6);
        for v in buffer.chunks_exact(6) {
            let p = kernel::math::Vec3::new(v[0], v[1], v[2]);
            assert!(welded.contains(&kernel::math::weld_key(p)), "{p:?} is not a welded position");
            assert!((kernel::math::Vec3::new(v[3], v[4], v[5]).length() - 1.0).abs() < 1e-3);
        }

        let mut positions: Vec<kernel::math::Vec3> = Vec::new();
        let mut index: HashMap<(i64, i64, i64), u32> = HashMap::new();
        let mut indices: Vec<u32> = Vec::new();
        for v in buffer.chunks_exact(6) {
            let p = kernel::math::Vec3::new(v[0], v[1], v[2]);
            let next = positions.len() as u32;
            let id = *index.entry(kernel::math::weld_key(p)).or_insert_with(|| {
                positions.push(p);
                next
            });
            indices.push(id);
        }
        assert_watertight(&Mesh { positions, indices });
    }

    #[test]
    fn a_partial_height_wall_leaving_the_footprint_stays_manifold() {
        let p = gridfinity::Params {
            bins: vec![LogicalBin {
                cells: cells(&[(0, 0), (0, 1), (1, 0)]),
                ..Default::default()
            }],
            inner_walls: vec![gridfinity::InnerWall {
                x1: 80.5,
                y1: 26.0,
                x2: 3.0,
                y2: 95.0,
                width: 5.6,
                height: Some(6.5),
            }],
            ..gridfinity::Params::default()
        };
        let bin = &p.bins[0];
        let solid = gridfinity::build_piece(&p, &bin.cells, &bin.cells, None).expect("builds");
        solid.validate().expect("manifold");
    }

    #[test]
    fn a_torus_section_lies_on_both_the_torus_and_the_cutting_plane() {
        use crate::kernel::geom::Curve;
        use crate::kernel::math::Vec3;
        let center = Vec3::new(3.0, -2.0, 5.0);
        let (major, minor) = (4.0f32, 1.5f32);
        for &offset in &[0.0f32, 1.0, -2.5, 3.9, 5.4] {
            let curve = Curve::torus_section(center, Vec3::Z, Vec3::X, offset, major, minor, 1.0);
            for i in 0..=64 {
                let t = -std::f32::consts::PI + 2.0 * std::f32::consts::PI * (i as f32 / 64.0);
                if !Curve::torus_section_exists(major, minor, offset, t) {
                    continue;
                }
                let p = curve.point(t) - center;
                assert!((p.x - offset).abs() < 1e-4, "offset {offset} t {t}: x {}", p.x);
                let radial = (p.x * p.x + p.y * p.y).sqrt() - major;
                let on_torus = radial * radial + p.z * p.z - minor * minor;
                assert!(on_torus.abs() < 1e-3, "offset {offset} t {t}: torus residual {on_torus}");
            }
        }
    }

    #[test]
    fn both_torus_section_branches_are_mirror_images_across_the_plane_normal() {
        use crate::kernel::geom::Curve;
        use crate::kernel::math::Vec3;
        let c = Vec3::ZERO;
        let pos = Curve::torus_section(c, Vec3::Z, Vec3::X, 1.0, 4.0, 1.5, 1.0);
        let neg = Curve::torus_section(c, Vec3::Z, Vec3::X, 1.0, 4.0, 1.5, -1.0);
        for i in 0..16 {
            let t = i as f32 * 0.3;
            let (a, b) = (pos.point(t), neg.point(t));
            assert!((a.x - b.x).abs() < 1e-5 && (a.z - b.z).abs() < 1e-5);
            assert!((a.y + b.y).abs() < 1e-5, "branches must mirror in y");
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
        assert!((size.x - 83.5).abs() < 1e-2, "x {}", size.x);
        assert!((size.y - 83.5).abs() < 1e-2, "y {}", size.y);
        assert!((size.z - 28.0).abs() < 1e-2, "z {}", size.z);
    }

    #[test]
    fn single_cell_bin_is_watertight() {
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
        assert!((max.x - 125.75).abs() < 1e-2, "max.x {}", max.x);
        assert!(min.x < 0.5, "min.x {}", min.x);
    }

    #[test]
    fn partial_divider_finger_is_watertight() {
        let p = gridfinity::Params {
            divider_edges: vec![GridEdge { x: 1, y: 0, orientation: Orientation::V }],
            ..gridfinity::Params::rect(2, 2)
        };
        let solid = gridfinity::build(&p);
        solid.validate().expect("finger topology valid");
        let mesh = tessellate(&solid, 6).to_mesh();
        assert_watertight(&mesh);
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
        let divider = vec![GridEdge { x: 1, y: 0, orientation: Orientation::V }];
        let p = gridfinity::Params {
            divider_edges: divider,
            ..gridfinity::Params::rect(3, 1)
        };
        let solid = gridfinity::build(&p);
        solid.validate().expect("divider bin topology valid");
        let mesh = tessellate(&solid, 6).to_mesh();
        assert_watertight(&mesh);
        let single = tessellate(&gridfinity::build(&gridfinity::Params::rect(3, 1)), 6).to_mesh();
        assert!(
            mesh.tri_count() > single.tri_count(),
            "divider should add cavity geometry"
        );
    }

    #[test]
    fn uneven_dividers_stay_watertight() {
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
        let ramp = solid.faces.iter().any(|f| match f.surface {
            geom::Surface::Cylinder { axis, radius, .. } => {
                (radius - 4.0).abs() < 1e-3 && axis.z.abs() < 1e-3
            }
            _ => false,
        });
        assert!(ramp, "wall-top runout blend is missing");
        let ell = solid.edges.iter().any(|e| matches!(e.curve, geom::Curve::Ellipse { .. }));
        assert!(ell, "runout ellipse is missing");
    }

    #[test]
    fn crossing_inner_wall_splits_compartment_watertight() {
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
    }

    #[test]
    fn open_edge_bin_is_watertight_and_loses_volume() {
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
        let (min, max) = mesh.bounds();
        assert!((max.y - min.y - 83.5).abs() < 1e-2, "depth {}", max.y - min.y);
    }

    #[test]
    fn single_open_edge_and_corner_pinch_watertight() {
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
        let mut open_edges = Vec::new();
        for e in crate::layout::perimeter_edges(&cells(&[(0, 0)])) {
            open_edges.push(e);
        }
        let p = gridfinity::Params { open_edges, ..gridfinity::Params::rect(1, 1) };
        let solid = gridfinity::build(&p);
        solid.validate().expect("fully-open bin topology valid");
        let mesh = tessellate(&solid, 6).to_mesh();
        assert_watertight(&mesh);
        let (_, max) = mesh.bounds();
        assert!((max.z - 8.2).abs() < 1e-2, "top {}", max.z);
    }

    #[test]
    fn open_edge_with_divider_finger_watertight() {
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
        let mut p = gridfinity::Params::rect(3, 1);
        p.bins[0].split_lines = vec![SplitLine { axis: Axis::X, index: 1 }];
        let pieces = gridfinity::build_pieces(&p);
        assert_eq!(pieces.len(), 2);
        for pc in &pieces {
            pc.solid.validate().unwrap_or_else(|e| panic!("{}: {e}", pc.name));
            let mesh = tessellate(&pc.solid, 6).to_mesh();
            assert_watertight(&mesh);
        }
        let (_, max0) = tessellate(&pieces[0].solid, 6).to_mesh().bounds();
        assert!((max0.x - 42.0).abs() < 1e-3, "seam at {}", max0.x);
        assert_eq!(pieces[0].name, "gridfinity-bin-piece-1-of-2.stl");
    }

    #[test]
    fn split_seam_divider_walls_both_pieces() {
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
    fn a_staircase_piece_carves_to_its_cells_not_its_bounding_box() {
        let p = gridfinity::Params {
            bins: vec![LogicalBin {
                cells: cells(&[(0, 0), (1, 0), (0, 1), (1, 1)]),
                ..Default::default()
            }],
            ..gridfinity::Params::default()
        };
        let whole = gridfinity::build_bin_solid(&p, &p.bins[0].cells, None).unwrap();
        let corner = gridfinity::carve_to_cells(&whole, &cells(&[(1, 0)])).unwrap();
        let ell = gridfinity::carve_to_cells(&whole, &cells(&[(0, 0), (0, 1), (1, 1)])).unwrap();
        for piece in [&corner, &ell] {
            piece.validate().expect("manifold");
            assert_watertight(&tessellate(piece, 6).to_mesh());
        }
        let vol = |s: &Solid| signed_volume(&tessellate(s, 6).to_mesh());
        let (whole_v, sum) = (vol(&whole), vol(&corner) + vol(&ell));
        assert!(
            (sum - whole_v).abs() < 1e-2,
            "the two pieces must partition the bin, not overlap: {sum} vs {whole_v}"
        );
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
        use crate::kernel::fillet::fillet_edges;
        let s = Sketch::circle(0.0, 0.0, 10.0);
        let solid = extrude(&s, 0.0, 5.0);
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
        let blended = fillet_edges(&solid, &blends).expect("cylinder top fillet");
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

        let (on_wall, r) = blends_near(&solid, (22.0, 30.0), (62.0, 55.0), 6.0);
        assert_eq!(on_wall, 4, "want 4 blend faces around the island, got {on_wall}");
        assert!(r > 0.1, "island blend radius collapsed to {r}");
        let (total, _) = blends_near(&solid, (41.75, 41.75), (41.75, 41.75), 1e4);
        let (base, br) = blends_near(&plain, (41.75, 41.75), (41.75, 41.75), 1e4);
        assert_eq!(base, 4, "plain bin should have 4 corner blends, got {base}");
        assert_eq!(total, base + 4, "island blends must add to the compartment's");
        assert!(br > 0.1, "compartment blend radius collapsed to {br}");
    }

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

    #[test]
    fn perf_counters_see_a_real_build() {
        use crate::kernel::perf::{self, Metric};
        let _g = perf_guard();

        let p = gridfinity::Params {
            inner_walls: vec![gridfinity::InnerWall {
                x1: -5.0, y1: 40.0, x2: 90.0, y2: 45.0, width: 2.4, height: None,
            }],
            ..gridfinity::Params::default()
        };
        perf::reset();
        perf::set_enabled(true);
        let solid = gridfinity::build(&p);
        let _ = tessellate(&solid, 6);
        perf::set_enabled(false);
        let rows = perf::snapshot();

        for want in [
            Metric::SplitRegions,
            Metric::SegSegPoints,
            Metric::PointInSegs,
            Metric::BuilderVertex,
            Metric::BuilderArc,
            Metric::BuilderFace,
            Metric::Tessellate,
            Metric::EmitSlabs,
        ] {
            let row = rows.iter().find(|r| r.name == want.name());
            assert!(row.is_some_and(|r| r.calls > 0), "{} never fired", want.name());
        }
    }

    #[test]
    fn alloc_report() {
        use crate::kernel::perf;
        let _g = perf_guard();
        let (w, h) = match std::env::var("SCALE_WH") {
            Ok(s) => {
                let mut it = s.split('x').map(|v| v.parse::<i32>().unwrap());
                (it.next().unwrap(), it.next().unwrap())
            }
            Err(_) => (32, 32),
        };
        let mut cells = Vec::new();
        let (cx, cy) = (w as f32 / 2.0, h as f32 / 2.0);
        let r = (w.min(h) as f32) * 0.45;
        for x in 0..w {
            for y in 0..h {
                let (dx, dy) = (x as f32 + 0.5 - cx, y as f32 + 0.5 - cy);
                let wob = 1.0 + 0.18 * (dy * 0.9).sin() + 0.12 * (dx * 1.3).cos();
                if (dx * dx + dy * dy).sqrt() <= r * wob {
                    cells.push(layout::GridCell { x, y });
                }
            }
        }
        let n = cells.len();
        let p = gridfinity::Params {
            bins: vec![gridfinity::LogicalBin { cells, ..Default::default() }],
            ..Default::default()
        };
        perf::set_enabled(true);
        let _ = tessellate(&gridfinity::build(&p), 4);
        perf::reset();
        let t = std::time::Instant::now();
        let solid = gridfinity::build(&p);
        let tess = tessellate(&solid, 4);
        let wall = t.elapsed();
        perf::set_enabled(false);

        println!(
            "\n{w}x{h} blob: {n} cells -> {} faces, {} tris in {:?}\n",
            solid.faces.len(),
            tess.tris.len(),
            wall
        );
        println!("{:<34} {:>10} {:>12} {:>12}", "metric", "calls", "churn kB", "allocs");
        let mut rows = perf::snapshot();
        rows.sort_by_key(|r| std::cmp::Reverse(r.alloc_calls));
        for r in &rows {
            println!(
                "{:<34} {:>10} {:>12} {:>12}",
                r.name,
                r.calls,
                r.alloc_bytes / 1000,
                r.alloc_calls
            );
        }
        let a = perf::allocs();
        let attributed: u64 = rows.iter().map(|r| r.alloc_bytes).sum();
        let att_calls: u64 = rows.iter().map(|r| r.alloc_calls).sum();
        println!(
            "total allocs {} · churn {} kB · peak {} kB · unattributed {} allocs / {} kB",
            a.count,
            a.bytes / 1000,
            a.peak_live_bytes / 1000,
            a.count.saturating_sub(att_calls),
            a.bytes.saturating_sub(attributed) / 1000,
        );
    }

    #[test]
    fn perf_report() {
        use crate::kernel::perf;
        let _g = perf_guard();
        let p = gridfinity::Params {
            inner_walls: vec![
                gridfinity::InnerWall { x1: 22.0, y1: 30.0, x2: 62.0, y2: 55.0, width: 2.4, height: None },
                gridfinity::InnerWall { x1: 80.5, y1: 26.0, x2: 3.0, y2: 95.0, width: 5.6, height: Some(6.5) },
            ],
            ..gridfinity::Params::default()
        };
        perf::set_enabled(true);
        let _ = tessellate(&gridfinity::build(&p), 6);
        perf::reset();
        let t = std::time::Instant::now();
        let solid = gridfinity::build(&p);
        let tess = tessellate(&solid, 6);
        let wall = t.elapsed();
        perf::set_enabled(false);

        println!("
rebuild #2 {:?} -> {} faces, {} tris", wall, solid.faces.len(), tess.to_mesh().indices.len() / 3);
        println!("{:<34} {:>10} {:>10} {:>12} {:>10}", "metric", "time", "calls", "churn B", "allocs");
        for r in perf::snapshot() {
            println!(
                "{:<34} {:>10} {:>10} {:>12} {:>10}",
                r.name,
                format!("{:?}", std::time::Duration::from_nanos(r.nanos)),
                r.calls,
                r.alloc_bytes,
                r.alloc_calls,
            );
        }
        let a = perf::allocs();
        if a.count == 0 {
            println!("(allocations unmeasured: no CountingAlloc global_allocator in this binary)");
        } else {
            let attributed: u64 = perf::snapshot().iter().map(|r| r.alloc_bytes).sum();
            println!(
                "total allocs {} · churn {} kB · peak {} kB · unattributed churn {} kB",
                a.count,
                a.bytes / 1000,
                a.peak_live_bytes / 1000,
                a.bytes.saturating_sub(attributed) / 1000,
            );
        }
    }

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

    /// Multi-cell polyomino bins at `arc_segs_per_quarter = 1`, the shape and
    /// resolution the Bad Apple stress test drives. Coarse arcs put most planar
    /// faces on the small-polygon fast path in `triangulate`, and reentrant
    /// corners exercise the holes-and-chords path, so this is where a
    /// tessellator regression would surface first.
    #[test]
    fn polyomino_bins_tessellate_closed_at_coarse_resolution() {
        use crate::layout::GridCell;
        let p = gridfinity::Params {
            height_units: 2,
            floor_fillet: 0.0,
            magnet_holes: false,
            screw_holes: false,
            ..gridfinity::Params::default()
        };
        let cell = |x: i32, y: i32| GridCell { x, y };
        let shapes: Vec<(&str, Vec<GridCell>)> = vec![
            ("single", vec![cell(0, 0)]),
            ("domino", vec![cell(0, 0), cell(1, 0)]),
            ("L", vec![cell(0, 0), cell(0, 1), cell(1, 0)]),
            ("S", vec![cell(0, 0), cell(1, 0), cell(1, 1), cell(2, 1)]),
            ("T", vec![cell(0, 0), cell(1, 0), cell(2, 0), cell(1, 1)]),
            ("plus", vec![cell(1, 0), cell(0, 1), cell(1, 1), cell(2, 1), cell(1, 2)]),
            (
                "square3x3",
                (0..3).flat_map(|x| (0..3).map(move |y| cell(x, y))).collect(),
            ),
            (
                "U",
                vec![
                    cell(0, 0),
                    cell(0, 1),
                    cell(0, 2),
                    cell(2, 0),
                    cell(2, 1),
                    cell(2, 2),
                    cell(1, 0),
                ],
            ),
        ];
        for (name, cells) in shapes {
            let solid = gridfinity::build_piece(&p, &cells, &cells, None)
                .unwrap_or_else(|e| panic!("{name} failed to build: {e:?}"));
            assert!(solid.validate().is_ok(), "{name} is not a manifold solid");
            let tess = tessellate(&solid, 1);
            let leaks = tessellation_leaks(&tess);
            assert!(leaks.is_empty(), "{name} leaked {} edges", leaks.len());
        }
    }
}
