pub mod kernel;

pub mod gridfinity;
pub mod layout;
pub mod printers;
pub mod project;
pub mod region;

pub use gridfinity::{Params, build as build_gridfinity};
pub use kernel::audit::{
    AuditReport, Category, Defect, Severity, TessLeak, audit, tessellation_leaks,
};
pub use kernel::mesh::Mesh;
pub use kernel::tess::{Tessellation, tessellate, tessellate_shell};
pub use kernel::topo::Solid;
pub use kernel::xt::to_xt_text;

#[cfg(test)]
#[global_allocator]
static TEST_ALLOC: kernel::perf::CountingAlloc<mimalloc::MiMalloc> =
    kernel::perf::CountingAlloc::new(mimalloc::MiMalloc);

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gridfinity::{BinSlope, LogicalBin, Mode, Pocket, SlopeDir};
    use crate::kernel::geom::Surface;
    use crate::kernel::build::{Ring, extrude, loft};
    use crate::kernel::geom;
    use crate::kernel::sketch::Sketch;
    use crate::layout::{Axis, GridCell, GridEdge, Orientation, SplitLine};
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
            let p = kernel::math::Vec3::new(v[0] as f64, v[1] as f64, v[2] as f64);
            assert!(
                welded.contains(&kernel::math::weld_key(p)),
                "{p:?} is not a welded position"
            );
            assert!((kernel::math::Vec3::new(v[3] as f64, v[4] as f64, v[5] as f64).length() - 1.0).abs() < 1e-3);
        }

        let mut positions: Vec<kernel::math::Vec3> = Vec::new();
        let mut index: HashMap<(i64, i64, i64), u32> = HashMap::new();
        let mut indices: Vec<u32> = Vec::new();
        for v in buffer.chunks_exact(6) {
            let p = kernel::math::Vec3::new(v[0] as f64, v[1] as f64, v[2] as f64);
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
        let _solid = gridfinity::build_piece(&p, &bin.cells, &bin.cells, None, &[]).expect("builds");
    }

    #[test]
    fn a_torus_section_lies_on_both_the_torus_and_the_cutting_plane() {
        use crate::kernel::geom::Curve;
        use crate::kernel::math::Vec3;
        let center = Vec3::new(3.0, -2.0, 5.0);
        let (major, minor) = (4.0f64, 1.5f64);
        for &offset in &[0.0f64, 1.0, -2.5, 3.9, 5.4] {
            let curve = Curve::torus_section(center, Vec3::Z, Vec3::X, offset, major, minor, 1.0);
            for i in 0..=64 {
                let t = -std::f64::consts::PI + 2.0 * std::f64::consts::PI * (i as f64 / 64.0);
                if !Curve::torus_section_exists(major, minor, offset, t) {
                    continue;
                }
                let p = curve.point(t) - center;
                assert!(
                    (p.x - offset).abs() < 1e-4,
                    "offset {offset} t {t}: x {}",
                    p.x
                );
                let radial = (p.x * p.x + p.y * p.y).sqrt() - major;
                let on_torus = radial * radial + p.z * p.z - minor * minor;
                assert!(
                    on_torus.abs() < 1e-3,
                    "offset {offset} t {t}: torus residual {on_torus}"
                );
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
            let t = i as f64 * 0.3;
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
        p.bins[0].slope = Some(BinSlope {
            angle_deg: 15.0,
            dir: SlopeDir::PlusX,
        });
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

    /// Emits a closed box into `b`. `outward` false makes it a void: the faces
    /// carry their normals into the box, so the material is whatever is outside
    /// it.
    fn emit_box(
        b: &mut crate::kernel::topo::Builder,
        rect: &Sketch,
        z0: f64,
        z1: f64,
        outward: bool,
    ) {
        use crate::kernel::build::{ring, wall_between};
        use crate::kernel::geom::Surface;
        use crate::kernel::math::{Vec3, vec3_of};
        use crate::kernel::topo::Loop;
        let segs = rect.loops[0].clone();
        let lo = ring(b, &segs, z0);
        let hi = ring(b, &segs, z1);
        wall_between(b, &segs, &segs, &lo, &hi, z0, z1, outward);
        // A cavity's walls traverse exactly like a solid's -- `outward` flips
        // only the surface normal -- so its caps must too, or the two disagree
        // and the ring is left unpaired. Winding is solid either way and
        // `sense` is what carries the material side.
        b.face(
            Surface::plane_z(z1),
            outward,
            Loop::new(hi.edges.clone()),
            vec![],
        );
        b.face(
            Surface::plane(vec3_of(0.0, 0.0, z0), -Vec3::Z),
            outward,
            Loop::new(lo.edges.iter().rev().map(|&(e, d)| (e, !d)).collect()),
            vec![],
        );
    }

    /// Two lumps of material in one solid are two shells, and both of them have
    /// their material inside. This is the reading `carve_to_cells` refuses when
    /// the piece's cells are one island: a second shell there is material that
    /// broke off the part.
    #[test]
    fn two_separated_lumps_of_material_are_two_shells_that_both_enclose_it() {
        let mut b = crate::kernel::topo::Builder::new();
        emit_box(&mut b, &Sketch::rectangle(0.0, 0.0, 10.0, 10.0), 0.0, 5.0, true);
        emit_box(&mut b, &Sketch::rectangle(50.0, 0.0, 10.0, 10.0), 0.0, 5.0, true);
        let solid = b.build();
        solid.validate().expect("two disjoint boxes are each closed");

        let shells = solid.shells();
        assert_eq!(shells.len(), 2, "two lumps of material are two shells");
        assert!(
            shells.iter().all(|sh| sh.encloses_material),
            "each lump has its own material inside it"
        );
    }

    /// A void sealed inside material is a second shell with the material
    /// *outside* it. Nothing downstream of the carve can see this -- it
    /// tessellates and welds like any other closed surface, and only the X_T
    /// writer refused it -- so it is the shell's own material side that names
    /// it.
    #[test]
    fn a_void_sealed_inside_material_is_a_shell_that_encloses_none() {
        let mut b = crate::kernel::topo::Builder::new();
        emit_box(&mut b, &Sketch::rectangle(0.0, 0.0, 20.0, 20.0), 0.0, 10.0, true);
        emit_box(&mut b, &Sketch::rectangle(0.0, 0.0, 10.0, 10.0), 2.0, 8.0, false);
        let solid = b.build();
        solid.validate().expect("a box around a sealed cavity is closed");

        let shells = solid.shells();
        assert_eq!(shells.len(), 2, "the outer surface and the cavity's");
        assert_eq!(
            shells.iter().filter(|sh| sh.encloses_material).count(),
            1,
            "exactly one of the two has material inside it; the other bounds the void"
        );
    }

    /// Every solid the kernel hands out is free of geometry nothing names: a
    /// vertex on no edge and an edge on no face are both left behind by a local
    /// rebuild, which resumes on the whole arena so that ids resolved before it
    /// survive it, and `build_compact_unvalidated` is where they are dropped.
    ///
    /// The floor fillet is the one op in a bin that strands any: before it every
    /// prefix of the program is clean, and it leaves the cavity corner's old
    /// tangent points and the junctions `fuse_collinear_edges` dissolved.
    #[test]
    fn a_built_bin_carries_no_vertex_or_edge_that_nothing_names() {
        for (gx, gy) in [(1, 1), (2, 2), (3, 2)] {
            let solid = gridfinity::build(&Params::rect(gx, gy));
            assert!(
                solid.orphan_vertices().is_empty(),
                "{gx}x{gy} keeps {} vertex(es) no edge names",
                solid.orphan_vertices().len()
            );
            assert!(
                solid.orphan_edges().is_empty(),
                "{gx}x{gy} keeps {} edge(s) no face uses",
                solid.orphan_edges().len()
            );
        }
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
            Ring {
                z: 0.0,
                sketch: &r0,
            },
            Ring {
                z: 0.8,
                sketch: &r1,
            },
            Ring {
                z: 2.6,
                sketch: &r1,
            },
            Ring {
                z: 4.75,
                sketch: &r2,
            },
        ]);
        let _mesh = tessellate(&solid, 6).to_mesh();
    }

    #[test]
    fn annulus_prism_watertight() {
        use crate::kernel::build::prism;
        let outer = Sketch::rectangle(0.0, 0.0, 40.0, 40.0);
        let hole = Sketch::rectangle(0.0, 0.0, 20.0, 20.0);
        let solid = prism(&outer, &[hole], 0.0, 10.0);
        let _mesh = tessellate(&solid, 6).to_mesh();
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
            bins: vec![LogicalBin {
                cells: cells(&[(0, 0), (1, 0), (0, 1)]),
                ..Default::default()
            }],
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
                LogicalBin {
                    cells: cells(&[(0, 0)]),
                    ..Default::default()
                },
                LogicalBin {
                    cells: cells(&[(1, 0), (2, 0)]),
                    ..Default::default()
                },
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
            divider_edges: vec![GridEdge {
                x: 1,
                y: 0,
                orientation: Orientation::V,
            }],
            ..gridfinity::Params::rect(2, 2)
        };
        let solid = gridfinity::build(&p);
        let _mesh = tessellate(&solid, 6).to_mesh();
        let full = gridfinity::Params {
            divider_edges: vec![
                GridEdge {
                    x: 1,
                    y: 0,
                    orientation: Orientation::V,
                },
                GridEdge {
                    x: 1,
                    y: 1,
                    orientation: Orientation::V,
                },
            ],
            ..gridfinity::Params::rect(2, 2)
        };
        let _solid_full = gridfinity::build(&full);
    }

    #[test]
    fn baseplate_is_valid_and_watertight() {
        for (gx, gy) in [(1, 1), (2, 3), (3, 3)] {
            let p = gridfinity::Params {
                mode: Mode::Baseplate,
                ..gridfinity::Params::rect(gx, gy)
            };
            let solid = gridfinity::build(&p);
            let _mesh = tessellate(&solid, 8).to_mesh();
        }
    }

    /// A drawer-sized baseplate is far bigger than any bed, so it must come back
    /// as several pieces that between them are the whole plate: one shell each
    /// with material inside it, and volumes summing to the intact plate's.
    /// Conservation is the sharp statement -- per-piece manifoldness passes
    /// straight over a carve that lost or duplicated material.
    #[test]
    fn a_drawer_sized_baseplate_carves_into_printable_pieces() {
        let printer = crate::printers::DEFAULT_PRINTER;
        let cells = crate::project::drawer::drawer_cells(crate::project::drawer::drawer_grid(
            400.0,
            300.0,
            crate::project::drawer::MAX_GRID,
        ));
        let lines = crate::printers::compute_auto_split_lines(&cells, printer);
        assert!(
            !lines.is_empty(),
            "a 9 x 7 baseplate is 378 x 294 mm and fits no {} bed, so it must be split",
            printer.name
        );
        let p = gridfinity::Params {
            mode: Mode::Baseplate,
            bins: vec![LogicalBin {
                cells: cells.clone(),
                split_lines: lines.clone(),
                ..Default::default()
            }],
            ..Default::default()
        };
        let pieces = gridfinity::try_build_pieces(&p).expect("a drawer baseplate builds");
        assert_eq!(pieces.len(), crate::layout::partition_cells(&cells, &lines).len());
        for piece in &pieces {
            let shells = piece.solid.shells();
            assert_eq!(shells.len(), 1, "{} is one plate", piece.name);
            assert!(shells[0].encloses_material, "{} bounds no material", piece.name);
            assert!(
                crate::printers::check_bed_fit(
                    &crate::layout::partition_cells(&cells, &lines)[piece.piece].cells,
                    printer
                )
                .fits,
                "{} still does not fit the bed it was split for",
                piece.name
            );
        }
        let vol = |s: &Solid| signed_volume(&tessellate(s, 6).to_mesh());
        let whole = vol(&gridfinity::build(&p));
        let summed: f64 = pieces.iter().map(|pc| vol(&pc.solid)).sum();
        assert!(
            (summed - whole).abs() < whole * 1e-3,
            "the {} pieces hold {summed} mm3 of the intact plate's {whole} mm3",
            pieces.len()
        );
    }

    /// Two disjoint cell sets are two plates, and each needs its own top and
    /// bottom cap carrying only its own sockets.
    #[test]
    fn a_baseplate_of_two_islands_caps_each_of_them() {
        let p = gridfinity::Params {
            mode: Mode::Baseplate,
            bins: vec![
                LogicalBin {
                    cells: cells(&[(0, 0), (1, 0)]),
                    ..Default::default()
                },
                LogicalBin {
                    cells: cells(&[(4, 0), (4, 1)]),
                    ..Default::default()
                },
            ],
            ..Default::default()
        };
        let solid = gridfinity::build(&p);
        let shells = solid.shells();
        assert_eq!(shells.len(), 2, "two separated plates are two shells");
        assert!(shells.iter().all(|sh| sh.encloses_material));
    }

    /// A bin whose cavity is *stated* is solid everywhere the pockets are not.
    /// The walk would have made the whole interior one compartment; two pockets
    /// make two, and the material between them and around them is the bin.
    ///
    /// Volume is the statement: the pocketed bin holds more material than the
    /// walked one, by the space the walk would have hollowed and this does not.
    #[test]
    fn a_bin_with_stated_pockets_is_solid_where_no_pocket_is() {
        let walked = gridfinity::Params::rect(2, 2);
        let pocketed = gridfinity::Params {
            bins: vec![LogicalBin {
                pockets: vec![
                    Pocket { x: 6.0, y: 6.0, width: 28.0, depth: 28.0 },
                    Pocket { x: 48.0, y: 6.0, width: 28.0, depth: 28.0 },
                ],
                ..gridfinity::Params::rect(2, 2).bins[0].clone()
            }],
            ..walked.clone()
        };
        let vol = |p: &gridfinity::Params| signed_volume(&tessellate(&gridfinity::build(p), 8).to_mesh());
        let (open, filled) = (vol(&walked), vol(&pocketed));
        assert!(
            filled > open,
            "stating two small pockets leaves more material than hollowing the whole bin: {filled} against {open}"
        );
        let floors = gridfinity::build(&pocketed);
        assert!(
            floors.validate().is_ok() && crate::audit(&floors).is_ok(),
            "a pocketed bin is a sound solid"
        );
    }
    /// The z-extent a face occupies, from its own edges' samples.
    fn face_z_range(solid: &Solid, fid: usize) -> (f64, f64) {
        let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
        for &(e, fwd) in solid.outer_edges(fid) {
            let edge = solid.edges[e];
            for p in edge.sample(fwd, edge.seg_count(12)) {
                lo = lo.min(p.z);
                hi = hi.max(p.z);
            }
        }
        assert!(
            lo.is_finite() && hi.is_finite(),
            "face {fid} bounds no sampled point, so it has no z-extent"
        );
        (lo, hi)
    }

    /// Whether a face is swept vertically, so its cross-section does not vary
    /// with z, or is horizontal, so it is an interface between two bands. Those
    /// are exactly the faces a boolean can resolve in 2D.
    fn is_prismatic(surface: &Surface) -> bool {
        match surface {
            Surface::Plane { normal, .. } => {
                let nz = normal.vec().z.abs();
                nz < 1e-9 || nz > 1.0 - 1e-9
            }
            Surface::Cylinder { axis, .. } => axis.vec().z.abs() > 1.0 - 1e-9,
            _ => false,
        }
    }

    /// The precondition a CAD-style feature timeline for this model rests on:
    /// that its features can be joined by a boolean confined to horizontal
    /// planes and vertical prisms, needing no surface-surface intersection.
    ///
    /// Two claims, checked on the finished bin because that is where they have
    /// to hold. **No face straddles `PEG_HEIGHT`**, so the base and the body
    /// meet in that one plane and their union is the 2D boolean of the two
    /// cross-sections there -- which is what the `bridge underside` face
    /// already is, hand-stitched. And **every face above the cavity floor is
    /// prismatic**, so the body and the cavity are both z-prisms where they
    /// overlap and their difference is 2D as well. The peg's chamfer cones are
    /// the one non-prismatic thing in the model and they live wholly below
    /// `PEG_HEIGHT`, sharing a band with nothing they must be cut against.
    ///
    /// `floor_fillet` is off because the blend is applied *after* the cut and
    /// its tori are the deliberate exception: `fillet_edges` is already a
    /// valid-solid-in, valid-solid-out operator and needs no boolean at all.
    #[test]
    fn the_model_joins_at_planes_and_prisms_never_at_a_curved_intersection() {
        let peg_height = 4.75;
        let floor_z =
            f64::from(gridfinity::BASE_TOTAL_HEIGHT) + f64::from(gridfinity::FLOOR_THICKNESS);
        for (gx, gy) in [(1u32, 1u32), (2, 1), (2, 2), (3, 2)] {
            let p = gridfinity::Params {
                floor_fillet: 0.0,
                ..gridfinity::Params::rect(gx, gy)
            };
            let solid = gridfinity::build(&p);
            let mut curved_below = 0;
            for fid in 0..solid.faces.len() {
                let (lo, hi) = face_z_range(&solid, fid);
                assert!(
                    hi <= peg_height + 1e-6 || lo >= peg_height - 1e-6,
                    "face {fid} of a {gx}x{gy} bin spans z {lo}..{hi} and straddles the \
                     {peg_height} interface, so base and body would not join in one plane"
                );
                if !is_prismatic(&solid.faces[fid].surface) {
                    curved_below += 1;
                    assert!(
                        hi <= peg_height + 1e-6,
                        "the only non-prismatic faces are the peg chamfers below {peg_height}, \
                         but face {fid} reaches z {hi}: {:?}",
                        solid.faces[fid].surface
                    );
                }
                if lo >= floor_z - 1e-6 {
                    assert!(
                        is_prismatic(&solid.faces[fid].surface),
                        "face {fid} sits above the cavity floor at z {lo}..{hi} and is not a \
                         prism face: {:?}",
                        solid.faces[fid].surface
                    );
                }
            }
            assert!(
                curved_below > 0,
                "a {gx}x{gy} bin has chamfered pegs, so it must carry curved faces below the \
                 interface -- finding none means this proves nothing"
            );
        }
    }

    #[test]
    fn l_shaped_baseplate_is_watertight() {
        let p = gridfinity::Params {
            mode: Mode::Baseplate,
            bins: vec![LogicalBin {
                cells: cells(&[(0, 0), (1, 0), (1, 1)]),
                ..Default::default()
            }],
            ..Default::default()
        };
        let _solid = gridfinity::build(&p);
    }

    #[test]
    fn featured_bins_stay_watertight() {
        for (magnet, screw, dx, dy, fillet) in [
            (true, false, 2, 1, 3.0),
            (false, true, 1, 3, 0.0),
            (true, true, 2, 2, 2.0),
        ] {
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
            let _mesh = tessellate(&solid, 6).to_mesh();
        }
    }

    #[test]
    fn stl_export_roundtrip() {
        let mesh = tessellate(&gridfinity::build(&gridfinity::Params::default()), 24).to_mesh();
        let stl = mesh.to_stl_binary();
        let header_tris = u32::from_le_bytes(stl[80..84].try_into().unwrap()) as usize;
        assert_eq!(header_tris, mesh.tri_count());
        assert_eq!(stl.len(), 84 + 50 * header_tris);
        assert!(
            header_tris > 200,
            "a real bin should have many facets, got {header_tris}"
        );
    }

    #[test]
    fn divider_edges_split_bin_is_watertight() {
        let divider = vec![GridEdge {
            x: 1,
            y: 0,
            orientation: Orientation::V,
        }];
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
            dividers.push(GridEdge {
                x: 1,
                y,
                orientation: Orientation::V,
            });
            dividers.push(GridEdge {
                x: 3,
                y,
                orientation: Orientation::V,
            });
        }
        for x in 0..4 {
            dividers.push(GridEdge {
                x,
                y: 2,
                orientation: Orientation::H,
            });
        }
        let p = gridfinity::Params {
            divider_edges: dividers,
            ..gridfinity::Params::rect(4, 3)
        };
        let _solid = gridfinity::build(&p);
    }

    #[test]
    fn divider_ring_island_is_watertight() {
        let dividers = vec![
            GridEdge {
                x: 1,
                y: 1,
                orientation: Orientation::V,
            },
            GridEdge {
                x: 2,
                y: 1,
                orientation: Orientation::V,
            },
            GridEdge {
                x: 1,
                y: 1,
                orientation: Orientation::H,
            },
            GridEdge {
                x: 1,
                y: 2,
                orientation: Orientation::H,
            },
        ];
        let p = gridfinity::Params {
            divider_edges: dividers,
            ..gridfinity::Params::rect(3, 3)
        };
        let _solid = gridfinity::build(&p);
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
        assert!(
            dv_full > dv + 100.0,
            "full wall adds more, {dv_full} vs {dv}"
        );
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
        let _solid = gridfinity::build(&p);
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
        let ell = solid
            .edges
            .iter()
            .any(|e| matches!(e.curve, geom::Curve::Ellipse { .. }));
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
        solid
            .validate()
            .expect("crossing inner-wall topology valid");
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
        let _solid = gridfinity::build(&p);
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
        let _solid = gridfinity::build(&p);
    }

    #[test]
    fn open_edge_bin_is_watertight_and_loses_volume() {
        let closed = gridfinity::Params::default();
        let closed_mesh = tessellate(&gridfinity::build(&closed), 6).to_mesh();
        let open = gridfinity::Params {
            open_edges: vec![
                GridEdge {
                    x: 0,
                    y: 2,
                    orientation: Orientation::H,
                },
                GridEdge {
                    x: 1,
                    y: 2,
                    orientation: Orientation::H,
                },
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
        assert!(
            (max.y - min.y - 83.5).abs() < 1e-2,
            "depth {}",
            max.y - min.y
        );
    }

    #[test]
    fn single_open_edge_and_corner_pinch_watertight() {
        let p = gridfinity::Params {
            open_edges: vec![GridEdge {
                x: 0,
                y: 2,
                orientation: Orientation::H,
            }],
            ..gridfinity::Params::default()
        };
        let _solid = gridfinity::build(&p);
    }

    /// An L-shaped bin whose opening runs into the reentrant corner rounds
    /// every corner it asks for.
    ///
    /// Two capabilities meet here and the chain needs both. The cavity's
    /// concave corner is an **arc**, so its floor fillet is a torus and the
    /// runout has to section a torus rather than a cylinder; and the wall that
    /// arc rolls against tapers to nothing where the opened cavity meets the
    /// outline, so the chain ends where no face can take the curve and only a
    /// flat end closes it. Before either existed the bin came back with all ten
    /// of its blends refused.
    #[test]
    fn an_opening_into_a_reentrant_corner_keeps_every_blend() {
        let p = gridfinity::Params {
            bins: vec![gridfinity::LogicalBin {
                cells: cells(&[(0, 0), (0, 1), (1, 0)]),
                ..Default::default()
            }],
            open_edges: vec![GridEdge {
                x: 1,
                y: 1,
                orientation: Orientation::H,
            }],
            ..gridfinity::Params::default()
        };
        let (solid, report) =
            gridfinity::try_build_reporting(&p).expect("the reentrant opened bin builds");
        assert!(
            report.is_clean() && report.made() > 0,
            "the bin asked for {} blend(s), {} matched no edge and {} were refused: {:?}",
            report.requested,
            report.unresolved,
            report.dropped.len(),
            report.refusal
        );
        assert_watertight(&tessellate(&solid, 6).to_mesh());
    }

    #[test]
    fn fully_open_1x1_bin_is_watertight() {
        let mut open_edges = Vec::new();
        for e in crate::layout::perimeter_edges(&cells(&[(0, 0)])) {
            open_edges.push(e);
        }
        let p = gridfinity::Params {
            open_edges,
            ..gridfinity::Params::rect(1, 1)
        };
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
                GridEdge {
                    x: 0,
                    y: 1,
                    orientation: Orientation::H,
                },
                GridEdge {
                    x: 1,
                    y: 1,
                    orientation: Orientation::H,
                },
            ],
            divider_edges: vec![GridEdge {
                x: 1,
                y: 0,
                orientation: Orientation::V,
            }],
            ..gridfinity::Params::rect(2, 1)
        };
        let _solid = gridfinity::build(&p);
    }

    #[test]
    fn split_bin_pieces_are_watertight() {
        let mut p = gridfinity::Params::rect(3, 1);
        p.bins[0].split_lines = vec![SplitLine {
            axis: Axis::X,
            index: 1,
        }];
        let pieces = gridfinity::build_pieces(&p);
        assert_eq!(pieces.len(), 2);
        for pc in &pieces {
            pc.solid
                .validate()
                .unwrap_or_else(|e| panic!("{}: {e}", pc.name));
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
        p.bins[0].split_lines = vec![SplitLine {
            axis: Axis::X,
            index: 1,
        }];
        p.divider_edges = vec![GridEdge {
            x: 1,
            y: 0,
            orientation: Orientation::V,
        }];
        let pieces = gridfinity::build_pieces(&p);
        assert_eq!(pieces.len(), 2);
        let mut volumes = Vec::new();
        for pc in &pieces {
            pc.solid
                .validate()
                .unwrap_or_else(|e| panic!("{}: {e}", pc.name));
            let mesh = tessellate(&pc.solid, 6).to_mesh();
            assert_watertight(&mesh);
            volumes.push(signed_volume(&mesh));
        }
        let mut p_open = gridfinity::Params::rect(2, 1);
        p_open.bins[0].split_lines = vec![SplitLine {
            axis: Axis::X,
            index: 1,
        }];
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
        p.bins[0].split_lines = vec![SplitLine {
            axis: Axis::Y,
            index: 1,
        }];
        let pieces = gridfinity::build_pieces(&p);
        assert_eq!(pieces.len(), 2);
        for pc in &pieces {
            pc.solid
                .validate()
                .unwrap_or_else(|e| panic!("{}: {e}", pc.name));
            assert_watertight(&tessellate(&pc.solid, 6).to_mesh());
        }
    }

    /// A cut through a bin whose floor fillet is wider than its cavity corner
    /// still finds a connector along every face it crosses.
    ///
    /// Shrunk out of `fuzz_split_pieces`, which this was the only failure of:
    /// `floor_fillet` 4.0 against `cavity_corner_radius` 3.5 puts a blend on the
    /// reentrant corner of an L, and the split plane runs straight through it.
    #[test]
    fn a_cut_through_a_corner_blend_wider_than_its_corner_carves() {
        let p = gridfinity::Params {
            bins: vec![LogicalBin {
                cells: cells(&[(1, 0), (2, 0), (2, 1)]),
                split_lines: vec![SplitLine {
                    axis: Axis::Y,
                    index: 1,
                }],
                ..Default::default()
            }],
            height_units: 1,
            wall_thickness: 0.7,
            cavity_corner_radius: 3.5,
            floor_fillet: 4.0,
            ..gridfinity::Params::default()
        };
        let whole = gridfinity::build_bin_solid(&p, &p.bins[0].cells, None, &[]).expect("the bin builds");
        let parts = layout::partition_cells(&p.bins[0].cells, &p.bins[0].split_lines);
        assert_eq!(parts.len(), 2, "one split line cuts the L in two");
        let vol = |s: &Solid| signed_volume(&tessellate(s, 6).to_mesh());
        let mut sum = 0.0;
        for (i, part) in parts.iter().enumerate() {
            let piece = gridfinity::carve_to_cells(&whole, &p.bins[0].cells, &part.cells)
                .unwrap_or_else(|e| panic!("piece {i}: {e}"));
            piece.validate().unwrap_or_else(|e| panic!("piece {i}: {e}"));
            assert_watertight(&tessellate(&piece, 6).to_mesh());
            sum += vol(&piece);
        }
        let whole_v = vol(&whole);
        assert!(
            (sum - whole_v).abs() < 1e-2 * whole_v.abs(),
            "the pieces partition the bin: {sum} mm^3 against {whole_v} mm^3"
        );
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
        let whole = gridfinity::build_bin_solid(&p, &p.bins[0].cells, None, &[]).unwrap();
        let corner =
            gridfinity::carve_to_cells(&whole, &p.bins[0].cells, &cells(&[(1, 0)])).unwrap();
        let ell =
            gridfinity::carve_to_cells(&whole, &p.bins[0].cells, &cells(&[(0, 0), (0, 1), (1, 1)]))
                .unwrap();
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

    fn carve_conserves_volume(shape: &[(i32, i32)], parts: &[&[(i32, i32)]], height_units: u32) {
        let p = gridfinity::Params {
            bins: vec![LogicalBin {
                cells: cells(shape),
                ..Default::default()
            }],
            height_units,
            ..gridfinity::Params::default()
        };
        let whole = gridfinity::build_bin_solid(&p, &p.bins[0].cells, None, &[]).unwrap();
        let vol = |s: &Solid| signed_volume(&tessellate(s, 12).to_mesh());
        let whole_v = vol(&whole);
        let mut sum = 0.0;
        for part in parts {
            let piece = gridfinity::carve_to_cells(&whole, &p.bins[0].cells, &cells(part))
                .unwrap_or_else(|e| panic!("{shape:?} -> {part:?}: {e}"));
            piece.validate().unwrap_or_else(|e| panic!("{part:?}: {e}"));
            assert_watertight(&tessellate(&piece, 12).to_mesh());
            sum += vol(&piece);
        }
        let drift = (sum - whole_v).abs();
        assert!(
            drift < whole_v.abs() * 5e-4,
            "{shape:?} split {parts:?} must conserve volume: pieces {sum} vs whole {whole_v} \
             ({drift} mm^3 adrift)"
        );
    }

    #[test]
    fn carving_a_reentrant_bin_keeps_the_corner_fillet_that_overhangs_the_grid() {
        carve_conserves_volume(
            &[(0, 0), (1, 0), (1, 1)],
            &[&[(0, 0)], &[(1, 0), (1, 1)]],
            4,
        );
        carve_conserves_volume(
            &[(0, 0), (1, 0), (2, 0), (1, 1)],
            &[&[(1, 1)], &[(0, 0), (1, 0), (2, 0)]],
            4,
        );
        carve_conserves_volume(
            &[(0, 0), (1, 0), (1, 1), (2, 1)],
            &[&[(0, 0), (1, 0)], &[(1, 1), (2, 1)]],
            2,
        );
    }

    #[test]
    fn an_opening_beside_a_reentrant_corner_keeps_every_floor_fillet() {
        // The cavity steps by the outline's own tolerance where an opened run
        // meets the reentrant corner, leaving a run shorter than the fillet
        // radius for the chain to end on. The flat end's stub covers the whole
        // of that run, so the floor must hand it to the cap rather than walk out
        // to the corner and back along it.
        for (shape, open) in [
            (
                &[(0, 1), (0, 2), (1, 2)][..],
                GridEdge {
                    x: 1,
                    y: 2,
                    orientation: Orientation::H,
                },
            ),
            (
                &[(0, 0), (1, 0), (0, 1), (0, 2), (0, 3)][..],
                GridEdge {
                    x: 1,
                    y: 1,
                    orientation: Orientation::V,
                },
            ),
        ] {
            let p = gridfinity::Params {
                bins: vec![LogicalBin {
                    cells: cells(shape),
                    ..Default::default()
                }],
                open_edges: vec![open],
                ..gridfinity::Params::default()
            };
            let (solid, report) = gridfinity::try_build_reporting(&p)
                .unwrap_or_else(|e| panic!("{shape:?} with {open:?}: {e}"));
            solid
                .validate()
                .unwrap_or_else(|e| panic!("{shape:?}: {e}"));
            assert!(
                report.is_clean() && report.requested > 0,
                "{shape:?} with {open:?} must ask for its floor fillets and land all of them: \
                 {report:?}"
            );
            let tess = tessellate(&solid, 16);
            let leaks = tessellation_leaks(&tess);
            assert!(
                leaks.is_empty(),
                "{shape:?} with {open:?} leaks {} edge(s): {:?}",
                leaks.len(),
                &leaks[..leaks.len().min(2)]
            );
        }
    }

    /// One opening may not cost a bin *every* blend request it has, including
    /// the ones in compartments the opening is nowhere near.
    ///
    /// Shrunk out of `fuzz_openings_and_inner_walls`, whose only defect this
    /// was: opening the top edge of a 2x3 takes it from 38 requested / 38 landed
    /// to **0 requested**, which `FILLET_FAILED` calls perfectly clean because a
    /// report of nothing asked and nothing refused is vacuously clean.
    #[test]
    fn an_opening_does_not_cost_the_far_compartments_their_blend_request() {
        let bin = |open: Vec<GridEdge>| {
            let p = gridfinity::Params {
                bins: vec![LogicalBin {
                    cells: cells(&[(1, 0), (1, 1), (1, 2), (2, 0), (2, 1), (2, 2)]),
                    ..Default::default()
                }],
                open_edges: open,
                divider_edges: vec![
                    GridEdge {
                        x: 1,
                        y: 2,
                        orientation: Orientation::H,
                    },
                    GridEdge {
                        x: 2,
                        y: 1,
                        orientation: Orientation::V,
                    },
                ],
                inner_walls: vec![gridfinity::InnerWall {
                    x1: -10.0,
                    y1: 63.0,
                    x2: 136.0,
                    y2: 63.0,
                    width: 2.2,
                    height: None,
                }],
                ..gridfinity::Params::default()
            };
            gridfinity::try_build_reporting(&p).expect("the bin builds")
        };
        let (_, closed) = bin(Vec::new());
        assert!(
            closed.made() > 0,
            "the fixture only says anything if the closed bin rounds something: {closed:?}"
        );
        let (_, opened) = bin(vec![GridEdge {
            x: 1,
            y: 3,
            orientation: Orientation::H,
        }]);
        assert!(
            opened.made() > 0,
            "one opening on the far side left the bin asking for {} blend(s) where closed it \
             asks {} and lands {}",
            opened.requested,
            closed.requested,
            closed.made()
        );
    }

    #[test]
    fn carving_a_middle_cell_splits_the_rim_into_two_faces_not_a_hole() {
        carve_conserves_volume(
            &[(0, 0), (1, 0), (2, 0)],
            &[&[(0, 0)], &[(1, 0)], &[(2, 0)]],
            4,
        );
        let p = gridfinity::Params {
            bins: vec![LogicalBin {
                cells: cells(&[(0, 0), (1, 0), (2, 0)]),
                ..Default::default()
            }],
            height_units: 4,
            ..gridfinity::Params::default()
        };
        let whole = gridfinity::build_bin_solid(&p, &p.bins[0].cells, None, &[]).unwrap();
        let middle =
            gridfinity::carve_to_cells(&whole, &p.bins[0].cells, &cells(&[(1, 0)])).unwrap();
        let rim_faces = |s: &crate::Solid| -> (usize, usize) {
            let top = tessellate(s, 6).bounds().1.z;
            let mut n = 0;
            let mut inners = 0;
            for (fi, f) in s.faces.iter().enumerate() {
                if matches!(f.surface, geom::Surface::Plane { normal, origin, .. }
                    if normal.z > 0.9 && (origin.z - top).abs() < 1e-3)
                {
                    n += 1;
                    inners += s.n_inners(fi);
                }
            }
            (n, inners)
        };
        assert_eq!(
            rim_faces(&whole),
            (1, 1),
            "the whole strip's rim is one face around one cavity"
        );
        assert_eq!(
            rim_faces(&middle),
            (2, 0),
            "carving the middle cell leaves the rim as two strips, not one face with a hole"
        );
    }

    #[test]
    fn a_thin_walled_bin_keeps_its_cavity_inside_the_rounded_corner() {
        for wall_thickness in [0.4f64, 0.8, 1.0, 1.2, 2.0, 3.0] {
            for cavity_corner_radius in [0.0f64, 0.5, 1.0, 2.5, 4.0] {
                let p = gridfinity::Params {
                    bins: vec![LogicalBin {
                        cells: cells(&[(0, 0)]),
                        ..Default::default()
                    }],
                    wall_thickness,
                    cavity_corner_radius,
                    ..gridfinity::Params::default()
                };
                let solid =
                    gridfinity::build_bin_solid(&p, &p.bins[0].cells, None, &[]).unwrap_or_else(|e| {
                        panic!("wt {wall_thickness} rc {cavity_corner_radius}: {e}")
                    });
                solid.validate().unwrap_or_else(|e| {
                    panic!("wt {wall_thickness} rc {cavity_corner_radius}: {e}")
                });
            }
        }
    }

    #[test]
    fn a_piece_enclosed_by_the_rest_of_the_bin_is_refused_not_mangled() {
        for shape in [
            vec![(1, 0), (0, 1), (1, 1), (2, 1), (1, 2)],
            vec![(0, 0), (1, 0), (0, 1), (1, 1), (2, 1), (1, 2)],
            vec![
                (0, 0),
                (1, 0),
                (2, 0),
                (0, 1),
                (1, 1),
                (2, 1),
                (0, 2),
                (1, 2),
                (2, 2),
            ],
        ] {
            let p = gridfinity::Params {
                bins: vec![LogicalBin {
                    cells: cells(&shape),
                    ..Default::default()
                }],
                ..gridfinity::Params::default()
            };
            let whole = gridfinity::build_bin_solid(&p, &p.bins[0].cells, None, &[]).unwrap();
            let err = gridfinity::carve_to_cells(&whole, &p.bins[0].cells, &cells(&[(1, 1)]))
                .expect_err("an enclosed piece must be refused");
            assert!(
                err.contains("surrounded on every side"),
                "{shape:?} should name the enclosed piece, got: {err}"
            );
        }
    }

    #[test]
    fn a_piece_touching_the_bin_edge_is_still_carved() {
        let p = gridfinity::Params {
            bins: vec![LogicalBin {
                cells: cells(&[(1, 0), (0, 1), (1, 1), (2, 1), (1, 2)]),
                ..Default::default()
            }],
            ..gridfinity::Params::default()
        };
        let whole = gridfinity::build_bin_solid(&p, &p.bins[0].cells, None, &[]).unwrap();
        let whole_vol = signed_volume(&tessellate(&whole, 6).to_mesh());
        for arm in [(1, 0), (0, 1), (2, 1), (1, 2)] {
            let piece = gridfinity::carve_to_cells(&whole, &p.bins[0].cells, &cells(&[arm]))
                .unwrap_or_else(|e| panic!("arm {arm:?}: {e}"));
            let vol = signed_volume(&tessellate(&piece, 6).to_mesh());
            assert!(
                vol > 1.0 && vol < whole_vol * 0.45,
                "arm {arm:?} carved to {vol} of the bin's {whole_vol}, so it was not carved"
            );
        }
    }

    #[test]
    fn a_full_height_island_in_a_banded_cavity_gets_one_top_face_not_two() {
        let p = gridfinity::Params {
            bins: vec![LogicalBin {
                cells: cells(&[(0, 0), (0, 1)]),
                ..Default::default()
            }],
            inner_walls: vec![
                gridfinity::InnerWall {
                    x1: 20.0,
                    y1: 2.0,
                    x2: 10.0,
                    y2: 60.0,
                    width: 2.0,
                    height: None,
                },
                gridfinity::InnerWall {
                    x1: 40.0,
                    y1: 20.0,
                    x2: -40.0,
                    y2: 110.0,
                    width: 2.0,
                    height: Some(10.0),
                },
            ],
            ..gridfinity::Params::default()
        };
        let b = &p.bins[0];
        let _solid = gridfinity::build_piece(&p, &b.cells, &b.cells, b.slope, &[]).expect("builds");
    }

    /// The exported literal has to name every field that is not at its default,
    /// or a bin pasted back in is a *different* bin and the defect it was
    /// exported to show does not reproduce. Checking the string mentions each
    /// one is what catches a field added to `Params` and not to the printer --
    /// the failure mode is silent, and it lands on whoever tries to use the
    /// export months later.
    #[test]
    fn an_exported_config_names_every_field_it_changed() {
        use crate::gridfinity::InnerWall;
        let p = Params {
            bins: vec![
                LogicalBin {
                    cells: vec![GridCell { x: 1, y: 2 }],
                    split_lines: vec![SplitLine {
                        axis: Axis::Y,
                        index: 1,
                    }],
                    pockets: vec![Pocket {
                        x: 3.0,
                        y: 4.0,
                        width: 20.0,
                        depth: 10.0,
                    }],
                    slope: Some(BinSlope {
                        angle_deg: 9.0,
                        dir: SlopeDir::PlusX,
                    }),
                },
                LogicalBin::rect(1, 1),
            ],
            height_units: 6,
            wall_thickness: 2.2,
            cavity_corner_radius: 0.5,
            floor_fillet: 1.25,
            magnet_holes: true,
            screw_holes: true,
            open_edges: vec![GridEdge {
                x: 1,
                y: 0,
                orientation: Orientation::H,
            }],
            divider_edges: vec![GridEdge {
                x: 2,
                y: 0,
                orientation: Orientation::V,
            }],
            inner_walls: vec![InnerWall {
                x1: 4.5,
                y1: 18.0,
                x2: 91.0,
                y2: 15.0,
                width: 1.2,
                height: Some(6.5),
            }],
            mode: Mode::Baseplate,
        };
        let s = p.rust_literal();
        for want in [
            "GridCell { x: 1, y: 2 }",
            "SplitLine { axis: Axis::Y, index: 1 }",
            "BinSlope { angle_deg: 9.0, dir: SlopeDir::PlusX }",
            "height_units: 6",
            "wall_thickness: 2.2",
            "cavity_corner_radius: 0.5",
            "floor_fillet: 1.25",
            "magnet_holes: true",
            "screw_holes: true",
            "mode: Mode::Baseplate",
            "orientation: Orientation::H",
            "orientation: Orientation::V",
            "InnerWall { x1: 4.5",
            "Pocket { x: 3.0, y: 4.0, width: 20.0, depth: 10.0 }",
            "height: Some(6.5)",
            "..Params::default()",
        ] {
            assert!(s.contains(want), "exported config omits {want}:\n{s}");
        }
        // Both bins, not just the one the fuzzer would have made.
        assert!(
            s.matches("LogicalBin {").count() == 2,
            "exported config lists {} bin(s), want 2:\n{s}",
            s.matches("LogicalBin {").count()
        );
        // A default `Params` carries none of that noise.
        let d = Params::default().rust_literal();
        for unwanted in ["height_units", "magnet_holes", "mode:", "inner_walls"] {
            assert!(
                !d.contains(unwanted),
                "a default config should not mention {unwanted}:\n{d}"
            );
        }
    }

    #[test]
    fn probe_blend() {
        use crate::gridfinity::InnerWall;
        use std::io::Write;
        let hook = std::panic::take_hook();
        std::panic::set_hook(Box::new(|_| {}));
        fn legal(v: &[f64]) -> bool {
            let l0 = ((v[2] - v[0]).powi(2) + (v[3] - v[1]).powi(2)).sqrt();
            let l1 = ((v[7] - v[5]).powi(2) + (v[8] - v[6]).powi(2)).sqrt();
            l0 > 5.0
                && l1 > 5.0
                && (0.8..=8.0).contains(&v[4])
                && (0.8..=8.0).contains(&v[9])
                && (2.0..=18.0).contains(&v[10])
        }
        fn outcome(v: &[f64]) -> String {
            let walls = vec![
                InnerWall {
                    x1: v[0],
                    y1: v[1],
                    x2: v[2],
                    y2: v[3],
                    width: v[4],
                    height: None,
                },
                InnerWall {
                    x1: v[5],
                    y1: v[6],
                    x2: v[7],
                    y2: v[8],
                    width: v[9],
                    height: Some(v[10]),
                },
            ];
            let p = gridfinity::Params {
                inner_walls: walls,
                ..gridfinity::Params::default()
            };
            let b = &p.bins[0];
            let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                gridfinity::build_piece(&p, &b.cells, &b.cells, b.slope, &[])
            }));
            match r {
                Ok(Ok(s)) => match s.validate() {
                    Ok(()) => "VALID".into(),
                    Err(e) => e
                        .chars()
                        .map(|c| if c.is_ascii_digit() { '#' } else { c })
                        .collect(),
                },
                Ok(Err(e)) => e
                    .chars()
                    .map(|c| if c.is_ascii_digit() { '#' } else { c })
                    .collect(),
                Err(_) => "PANIC".into(),
            }
        }
        let names = [
            "w0.x1",
            "w0.y1",
            "w0.x2",
            "w0.y2",
            "w0.width",
            "w1.x1",
            "w1.y1",
            "w1.x2",
            "w1.y2",
            "w1.width",
            "w1.height",
        ];
        let mut v: Vec<f64> = vec![
            51.0, 20.5, 20.5, 58.5, 3.2, 7.0, 30.5, 41.0, 60.5, 3.4, 12.5,
        ];
        let target = outcome(&v);
        println!("start: {target}");
        for _ in 0..3 {
            for i in 0..v.len() {
                let orig = v[i];
                let mut cands: Vec<f64> = vec![
                    (orig / 21.0).round() * 21.0,
                    (orig / 10.0).round() * 10.0,
                    (orig / 5.0).round() * 5.0,
                    orig.round(),
                ];
                if i == 4 || i == 9 {
                    cands.insert(0, 2.0);
                }
                if i == 10 {
                    cands.insert(0, 7.0);
                }
                for c in cands {
                    if (c - orig).abs() < 1e-6 {
                        continue;
                    }
                    v[i] = c;
                    if !legal(&v) {
                        v[i] = orig;
                        continue;
                    }
                    print!("  try {}={c} .. ", names[i]);
                    std::io::stdout().flush().ok();
                    let got = outcome(&v);
                    println!("{got}");
                    if got == target {
                        break;
                    }
                    v[i] = orig;
                }
            }
        }
        println!("shrunk: {target}");
        for (n, x) in names.iter().zip(&v) {
            println!("   {n} = {x}");
        }
        std::panic::set_hook(hook);
    }

    #[test]
    fn a_blend_selection_the_boolean_split_does_not_fail_the_build() {
        let p = gridfinity::Params {
            inner_walls: vec![
                gridfinity::InnerWall {
                    x1: 42.0,
                    y1: 21.0,
                    x2: 21.0,
                    y2: 63.0,
                    width: 2.0,
                    height: None,
                },
                gridfinity::InnerWall {
                    x1: 10.0,
                    y1: 21.0,
                    x2: 42.0,
                    y2: 63.0,
                    width: 2.0,
                    height: Some(7.0),
                },
            ],
            ..gridfinity::Params::default()
        };
        let b = &p.bins[0];
        let _solid = gridfinity::build_piece(&p, &b.cells, &b.cells, b.slope, &[]).expect("builds");
    }

    #[test]
    fn sloped_bin_is_watertight_and_outward() {
        for dir in [
            SlopeDir::PlusX,
            SlopeDir::MinusX,
            SlopeDir::PlusY,
            SlopeDir::MinusY,
        ] {
            let mut p = gridfinity::Params::default();
            p.bins[0].slope = Some(BinSlope {
                angle_deg: 15.0,
                dir,
            });
            let solid = gridfinity::build(&p);
            solid
                .validate()
                .unwrap_or_else(|e| panic!("slope {dir:?}: {e}"));
            let mesh = tessellate(&solid, 6).to_mesh();
            assert_watertight(&mesh);
            let vol = signed_volume(&mesh);
            assert!(
                vol > 1.0,
                "slope {dir:?}: expected positive volume, got {vol}"
            );
        }
    }

    /// A sloped bin takes no inner wall at all -- the wall is carved as a
    /// z-prism whose bottom ring sits at a flat `floor_z`, and a tilted floor is
    /// not there -- so a sloped bin must *build*, without the wall, rather than
    /// emit unsound geometry.
    ///
    /// Shrunk out of `fuzz_params_broad`, where both remaining sloped classes
    /// are a partial-height wall reaching a sloped bin anyway.
    #[test]
    fn a_sloped_bin_drops_a_partial_height_wall_instead_of_leaking() {
        for (shape, slope, units, wall) in [
            (
                &[(0, 0), (0, 1), (1, 1), (2, 1)][..],
                BinSlope {
                    angle_deg: 18.0,
                    dir: SlopeDir::PlusX,
                },
                2u32,
                gridfinity::InnerWall {
                    x1: 6.5,
                    y1: 10.5,
                    x2: 101.5,
                    y2: 60.5,
                    width: 1.4,
                    height: Some(6.0),
                },
            ),
            (
                &[(0, 1), (0, 2), (1, 0), (1, 1), (1, 2)][..],
                BinSlope {
                    angle_deg: 14.0,
                    dir: SlopeDir::MinusX,
                },
                1,
                gridfinity::InnerWall {
                    x1: 48.5,
                    y1: 122.0,
                    x2: 55.0,
                    y2: 31.5,
                    width: 3.4,
                    height: Some(3.5),
                },
            ),
        ] {
            let p = gridfinity::Params {
                bins: vec![LogicalBin {
                    cells: cells(shape),
                    slope: Some(slope),
                    ..Default::default()
                }],
                height_units: units,
                inner_walls: vec![wall],
                ..gridfinity::Params::default()
            };
            let solid = gridfinity::try_build(&p)
                .unwrap_or_else(|e| panic!("{shape:?} on a {slope:?} slope: {e}"));
            solid
                .validate()
                .unwrap_or_else(|e| panic!("{shape:?} on a {slope:?} slope: {e}"));
            assert_watertight(&tessellate(&solid, 6).to_mesh());
        }
    }

    /// An opening does not flatten a sloped bin's floor.
    ///
    /// The opened compartment's floor used to come off `emit_open_cavity`'s
    /// `PlanarFace` at `floor_z` -- the touched branch runs before the slope
    /// dispatch -- so a user who asked for a ramp and opened one edge got a flat
    /// part that built, validated and tessellated cleanly. `fuzz_params_broad`
    /// saw it only through the floor-fillet comparison, as a bin going from no
    /// cavity floor at all to three unrounded ones.
    #[test]
    fn an_opening_does_not_flatten_a_sloped_floor() {
        let bin = |open: Vec<GridEdge>| {
            let p = gridfinity::Params {
                bins: vec![LogicalBin {
                    cells: cells(&[(1, 2)]),
                    slope: Some(BinSlope {
                        angle_deg: 9.0,
                        dir: SlopeDir::PlusX,
                    }),
                    ..Default::default()
                }],
                height_units: 2,
                cavity_corner_radius: 0.0,
                open_edges: open,
                ..gridfinity::Params::default()
            };
            gridfinity::try_build(&p).expect("the bin builds")
        };
        let flat_floors = |s: &Solid| {
            s.faces
                .iter()
                .filter(|f| match f.surface {
                    geom::Surface::Plane { origin, normal, .. } => {
                        normal.z.abs() > 1.0 - 1e-3
                            && (origin.z - (gridfinity::BASE_TOTAL_HEIGHT
                                + gridfinity::FLOOR_THICKNESS))
                                .abs()
                                < 1e-3
                    }
                    _ => false,
                })
                .count()
        };
        let closed = bin(Vec::new());
        assert_eq!(
            flat_floors(&closed),
            0,
            "a sloped bin's floor is the ramp, so nothing of it lies flat at floor_z"
        );
        let opened = bin(vec![GridEdge {
            x: 1,
            y: 3,
            orientation: Orientation::H,
        }]);
        opened.validate().expect("manifold");
        assert_watertight(&tessellate(&opened, 8).to_mesh());
        for (i, f) in opened.faces.iter().enumerate() {
            if let geom::Surface::Plane { origin, normal, .. } = f.surface {
                if normal.z.abs() > 1.0 - 1e-3 && (origin.z - 8.2).abs() < 1e-3 {
                    println!("PROBE flat floor face {i} origin={origin:?} n={normal:?} sense={}", f.sense);
                }
            }
        }
        assert_eq!(
            flat_floors(&opened),
            0,
            "opening one edge must not lay the ramp flat"
        );
        assert!(
            signed_volume(&tessellate(&opened, 8).to_mesh())
                < signed_volume(&tessellate(&closed, 8).to_mesh()),
            "the opening takes material away"
        );
    }

    #[test]
    fn sloped_floor_displaces_volume() {
        let flat = tessellate(&gridfinity::build(&gridfinity::Params::default()), 8).to_mesh();
        let mut sp = gridfinity::Params::default();
        sp.bins[0].slope = Some(BinSlope {
            angle_deg: 25.0,
            dir: SlopeDir::MinusX,
        });
        let sloped = tessellate(&gridfinity::build(&sp), 8).to_mesh();
        assert!(
            signed_volume(&sloped) > signed_volume(&flat) + 1.0,
            "slope should add material volume"
        );
    }

    /// A free-form wall's leak, from `fuzz_inner_walls` at seed 7. The B-rep is
    /// sound -- `validate` passes and `audit` reports nothing -- and the mesh
    /// still leaked 4 edges at every tessellation density, which is what ruled
    /// out a sampling artifact and pointed at winding. All four were the edges
    /// of one small face, the partial-height wall's top cap: it took
    /// `triangulate`'s 4-vertex fast path while its neighbours went through
    /// `planar`, and the two disagree about output winding.
    #[test]
    fn a_partial_height_walls_top_cap_is_wound_like_its_neighbours() {
        let p = gridfinity::Params {
            bins: vec![gridfinity::LogicalBin {
                cells: vec![GridCell { x: 1, y: 0 }],
                ..Default::default()
            }],
            inner_walls: vec![
                gridfinity::InnerWall {
                    x1: 54.0,
                    y1: 32.5,
                    x2: -10.0,
                    y2: 53.5,
                    width: 2.0,
                    height: Some(9.5),
                },
                gridfinity::InnerWall {
                    x1: 51.5,
                    y1: 36.5,
                    x2: 35.5,
                    y2: -12.0,
                    width: 0.8,
                    height: None,
                },
            ],
            ..Default::default()
        };
        let solid = gridfinity::build(&p);
        solid.validate().expect("closed manifold");
        assert!(audit(&solid).is_ok(), "{}", audit(&solid));
        // Density is the tell: a sampling artifact moves with it, a winding
        // mismatch does not.
        for segs in [4, 8, 16, 24] {
            assert_watertight(&tessellate(&solid, segs).to_mesh());
        }
    }

    /// An opening takes the wall over its own run and nothing else. The chain
    /// dies on the mouth, where the corner is void rather than material, so
    /// `fillet.rs` emits a cap to close it off -- see the capped runout in
    /// `crates/CLAUDE.md`. Before that existed the model zeroed the whole
    /// compartment's fillet rather than ask for a blend it could not build.
    #[test]
    fn an_opening_keeps_the_rest_of_the_compartments_floor_fillet() {
        let closed = gridfinity::Params::rect(2, 2);
        let (_, before) = gridfinity::try_build_reporting(&closed).expect("closed bin builds");
        assert!(before.made() > 0, "the closed bin should blend its floor");

        for open in [
            GridEdge {
                x: 0,
                y: 0,
                orientation: Orientation::H,
            },
            GridEdge {
                x: 0,
                y: 0,
                orientation: Orientation::V,
            },
        ] {
            let mut p = gridfinity::Params::rect(2, 2);
            p.open_edges = vec![open];
            let (solid, after) = gridfinity::try_build_reporting(&p).expect("opened bin builds");
            assert!(
                after.made() > 0,
                "{open:?} left the compartment with no floor fillet at all"
            );
            assert_eq!(
                (after.unresolved, after.dropped.len()),
                (0, 0),
                "{open:?}: {} of {} blends did not land",
                after.unresolved + after.dropped.len(),
                after.requested
            );
            solid.validate().expect("opened bin is a closed manifold");
            assert!(audit(&solid).is_ok(), "{open:?}: {}", audit(&solid));
            assert_watertight(&tessellate(&solid, 8).to_mesh());
        }
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
        let blends: Vec<_> = top_edges.iter().map(|&e| (e, 2.0_f64)).collect();
        let blended = fillet_edges(&solid, &blends).expect("cylinder top fillet");
        blended.validate().expect("blended topology valid");
        let mesh = tessellate(&blended, 8).to_mesh();
        assert_watertight(&mesh);
    }

    #[test]
    fn sloped_floor_low_side_is_at_floor_z() {
        let floor_z = gridfinity::BASE_TOTAL_HEIGHT + gridfinity::FLOOR_THICKNESS;
        let mut p = gridfinity::Params::default();
        p.bins[0].slope = Some(BinSlope {
            angle_deg: 20.0,
            dir: SlopeDir::MinusX,
        });
        let mesh = tessellate(&gridfinity::build(&p), 10).to_mesh();
        let low = mesh
            .positions
            .iter()
            .copied()
            .filter(|v| v.x < 2.0 && v.z > 4.9)
            .map(|v| v.z)
            .fold(f64::INFINITY, f64::min);
        let high = mesh
            .positions
            .iter()
            .copied()
            .filter(|v| v.x > 80.0)
            .map(|v| v.z)
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(
            (low - floor_z).abs() < 0.6,
            "low-side floor z {low} â‰ˆ floor_z {floor_z}"
        );
        assert!(
            high > floor_z + 3.0,
            "high-side floor z {high} should rise above floor_z"
        );
    }
    fn blends_near(solid: &crate::Solid, a: (f64, f64), b: (f64, f64), d: f64) -> (usize, f64) {
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
                    let t = if l2 < 1e-9 {
                        0.0
                    } else {
                        ((p - a).dot(ab) / l2).clamp(0.0, 1.0)
                    };
                    (p - (a + ab * t)).length() <= d
                }
                _ => false,
            })
            .fold((0usize, 0.0f64), |(n, r), f| match f.surface {
                geom::Surface::Torus { minor_r, .. } => (n + 1, minor_r),
                _ => (n, r),
            })
    }

    #[test]
    fn freeform_floating_divider_is_filleted() {
        let wall = gridfinity::InnerWall {
            x1: 22.0,
            y1: 30.0,
            x2: 62.0,
            y2: 55.0,
            width: 2.4,
            height: None,
        };
        let p = gridfinity::Params {
            inner_walls: vec![wall.clone()],
            ..gridfinity::Params::default()
        };
        let plain = gridfinity::build(&gridfinity::Params::default());
        let solid = gridfinity::build(&p);
        solid.validate().expect("floating divider topology valid");
        assert!(
            crate::audit(&solid).is_ok(),
            "B-rep must be sound:
{}",
            crate::audit(&solid)
        );
        assert_watertight(&tessellate(&solid, 6).to_mesh());

        let (on_wall, r) = blends_near(&solid, (22.0, 30.0), (62.0, 55.0), 6.0);
        assert_eq!(
            on_wall, 4,
            "want 4 blend faces around the island, got {on_wall}"
        );
        assert!(r > 0.1, "island blend radius collapsed to {r}");
        let (total, _) = blends_near(&solid, (41.75, 41.75), (41.75, 41.75), 1e4);
        let (base, br) = blends_near(&plain, (41.75, 41.75), (41.75, 41.75), 1e4);
        assert_eq!(base, 4, "plain bin should have 4 corner blends, got {base}");
        assert_eq!(
            total,
            base + 4,
            "island blends must add to the compartment's"
        );
        assert!(br > 0.1, "compartment blend radius collapsed to {br}");
    }

    #[test]
    fn divider_too_close_to_the_wall_stays_sharp_and_sound() {
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
        solid.validate().expect("tight divider topology valid");
        assert!(
            crate::audit(&solid).is_ok(),
            "B-rep must be sound:
{}",
            crate::audit(&solid)
        );
        let (on_wall, _) = blends_near(&solid, (6.0, 42.0), (78.0, 42.0), 6.0);
        assert_eq!(
            on_wall, 0,
            "a wall this close to the boundary gets no blend"
        );
        let (total, _) = blends_near(&solid, (41.75, 41.75), (41.75, 41.75), 1e4);
        assert_eq!(
            total, 4,
            "the compartment must keep its own four corner blends"
        );
    }

    #[test]
    fn perf_counters_see_a_real_build() {
        use crate::kernel::perf::{self, Metric};
        let _g = perf_guard();

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
            assert!(
                row.is_some_and(|r| r.calls > 0),
                "{} never fired",
                want.name()
            );
        }
    }

    #[test]
    #[ignore = "benchmark: cargo test --release -p gridfinity-cad --lib -- --ignored --nocapture"]
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
        let (cx, cy) = (w as f64 / 2.0, h as f64 / 2.0);
        let r = (w.min(h) as f64) * 0.45;
        for x in 0..w {
            for y in 0..h {
                let (dx, dy) = (x as f64 + 0.5 - cx, y as f64 + 0.5 - cy);
                let wob = 1.0 + 0.18 * (dy * 0.9).sin() + 0.12 * (dx * 1.3).cos();
                if (dx * dx + dy * dy).sqrt() <= r * wob {
                    cells.push(layout::GridCell { x, y });
                }
            }
        }
        let n = cells.len();
        let p = gridfinity::Params {
            bins: vec![gridfinity::LogicalBin {
                cells,
                ..Default::default()
            }],
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
        println!(
            "{:<34} {:>10} {:>12} {:>12}",
            "metric", "calls", "churn kB", "allocs"
        );
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
    #[ignore = "benchmark: cargo test --release -p gridfinity-cad --lib -- --ignored --nocapture"]
    fn perf_report() {
        use crate::kernel::perf;
        let _g = perf_guard();
        let p = gridfinity::Params {
            inner_walls: vec![
                gridfinity::InnerWall {
                    x1: 22.0,
                    y1: 30.0,
                    x2: 62.0,
                    y2: 55.0,
                    width: 2.4,
                    height: None,
                },
                gridfinity::InnerWall {
                    x1: 80.5,
                    y1: 26.0,
                    x2: 3.0,
                    y2: 95.0,
                    width: 5.6,
                    height: Some(6.5),
                },
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

        println!(
            "
rebuild #2 {:?} -> {} faces, {} tris",
            wall,
            solid.faces.len(),
            tess.to_mesh().indices.len() / 3
        );
        println!(
            "{:<34} {:>10} {:>10} {:>12} {:>10}",
            "metric", "time", "calls", "churn B", "allocs"
        );
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
            bins: vec![LogicalBin {
                cells: cells(&[(1, 0)]),
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
        let solid = gridfinity::try_build(&p).expect("partial wall builds");
        solid
            .validate()
            .expect("partial-height wall topology valid");
        assert!(
            crate::audit(&solid).is_ok(),
            "B-rep must be sound:\n{}",
            crate::audit(&solid)
        );
        assert_watertight(&tessellate(&solid, 6).to_mesh());

        let top_z = 8.2 + 6.5;
        let caps = solid
            .faces
            .iter()
            .filter(|f| match f.surface {
                geom::Surface::Plane { origin, normal, .. } => {
                    normal.x.abs() < 1e-4
                        && normal.y.abs() < 1e-4
                        && (origin.z - top_z).abs() < 1e-3
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
        solid.validate().expect("notching divider topology valid");
        assert!(
            crate::audit(&solid).is_ok(),
            "B-rep must be sound:
{}",
            crate::audit(&solid)
        );
        assert_watertight(&tessellate(&solid, 6).to_mesh());
        let (n, r) = blends_near(&solid, (41.75, 41.75), (41.75, 41.75), 1e4);
        assert!(
            n > 0,
            "a notching divider must not cost the compartment its fillet"
        );
        assert!(r > 0.1, "blend radius collapsed to {r}");
    }

    #[test]
    fn freeform_crossing_divider_is_filleted() {
        let p = gridfinity::Params {
            inner_walls: vec![gridfinity::InnerWall {
                x1: -5.0,
                y1: 30.0,
                x2: 90.0,
                y2: 55.0,
                width: 2.4,
                height: None,
            }],
            ..gridfinity::Params::default()
        };
        let solid = gridfinity::build(&p);
        solid.validate().expect("crossing divider topology valid");
        assert!(
            crate::audit(&solid).is_ok(),
            "B-rep must be sound:\n{}",
            crate::audit(&solid)
        );
        assert_watertight(&tessellate(&solid, 6).to_mesh());
        let (n, _) = blends_near(&solid, (0.0, 0.0), (83.5, 83.5), 1e4);
        assert!(
            n > 0,
            "a crossing divider should still leave the floor filleted"
        );
    }

    #[test]
    fn a_drawer_bin_partitioned_into_compartments_is_watertight() {
        const DIVIDERS: [(f64, f64, f64, f64); 23] = [
            (0.85, 23.65, 64.25, 23.65),
            (217.45, 28.65, 278.65, 28.65),
            (82.45, 38.65, 120.85, 38.65),
            (63.05, 41.45, 124.25, 41.45),
            (217.45, 65.85, 255.85, 65.85),
            (45.25, 68.65, 218.65, 68.65),
            (0.85, 85.85, 24.25, 85.85),
            (123.05, 90.85, 186.45, 90.85),
            (45.25, 105.85, 83.65, 105.85),
            (23.65, 23.05, 23.65, 86.45),
            (45.85, 68.05, 45.85, 106.45),
            (63.65, 0.85, 63.65, 24.25),
            (63.65, 40.85, 63.65, 69.25),
            (83.05, 0.85, 83.05, 39.25),
            (83.05, 68.05, 83.05, 106.45),
            (120.25, 0.85, 120.25, 39.25),
            (123.65, 0.85, 123.65, 42.05),
            (123.65, 68.05, 123.65, 91.45),
            (155.85, 0.85, 155.85, 69.25),
            (185.85, 0.85, 185.85, 91.45),
            (218.05, 28.05, 218.05, 69.25),
            (255.25, 28.05, 255.25, 66.45),
            (278.05, 0.85, 278.05, 29.25),
        ];
        let footprint: Vec<(i32, i32)> = (0..5).flat_map(|y| (0..7).map(move |x| (x, y))).collect();
        let p = gridfinity::Params {
            bins: vec![LogicalBin {
                cells: cells(&footprint),
                ..Default::default()
            }],
            wall_thickness: 1.2,
            inner_walls: DIVIDERS
                .iter()
                .map(|&(x1, y1, x2, y2)| gridfinity::InnerWall {
                    x1,
                    y1,
                    x2,
                    y2,
                    width: 1.2,
                    height: None,
                })
                .collect(),
            ..gridfinity::Params::default()
        };
        let solid = gridfinity::try_build(&p).expect("drawer bin builds");
        solid.validate().expect("drawer bin topology valid");
        assert!(
            crate::audit(&solid).is_ok(),
            "B-rep must be sound:\n{}",
            crate::audit(&solid)
        );
        assert_watertight(&tessellate(&solid, 6).to_mesh());
    }
}

#[cfg(test)]
mod audit_tests {
    use crate::audit;
    use crate::gridfinity;
    use crate::kernel::geom::Surface;
    use crate::kernel::math::Vec3;
    use crate::kernel::tess::tessellate;
    use crate::kernel::topo::{Builder, Loop};

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
            Surface::plane_with_x(
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(0.0, -1.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
            ),
            true,
            front,
            vec![],
        );
        let mut solid = b.build_unvalidated();
        solid.edges[e01].t1 = 9.5;
        let report = audit(&solid);
        assert!(!report.is_ok(), "audit should catch the planted defect");
        assert!(
            report
                .defects
                .iter()
                .any(|d| d.category == crate::Category::EdgeVertexGeometry),
            "expected an EdgeVertexGeometry defect:\n{report}"
        );
    }

    #[test]
    fn a_partial_wall_leaves_the_rim_hole_segmented_the_way_the_bands_below_it_are() {
        use crate::gridfinity::InnerWall;
        use crate::layout::GridCell;
        let p = gridfinity::Params {
            bins: vec![gridfinity::LogicalBin {
                cells: vec![
                    GridCell { x: 0, y: 0 },
                    GridCell { x: 0, y: 1 },
                    GridCell { x: 1, y: 1 },
                ],
                ..Default::default()
            }],
            inner_walls: vec![
                InnerWall {
                    x1: 26.5,
                    y1: 62.0,
                    x2: 51.5,
                    y2: 79.0,
                    width: 1.6,
                    height: Some(11.5),
                },
                InnerWall {
                    x1: 27.5,
                    y1: 26.5,
                    x2: 33.0,
                    y2: 89.0,
                    width: 5.0,
                    height: None,
                },
            ],
            ..gridfinity::Params::default()
        };
        let _solid = gridfinity::build(&p);
    }

    #[test]
    fn two_solves_of_one_corner_that_straddle_a_weld_bucket_intern_to_one_vertex() {
        use crate::gridfinity::InnerWall;
        use crate::layout::GridCell;
        let p = gridfinity::Params {
            bins: vec![gridfinity::LogicalBin {
                cells: vec![
                    GridCell { x: 0, y: 0 },
                    GridCell { x: 0, y: 1 },
                    GridCell { x: 1, y: 1 },
                ],
                ..Default::default()
            }],
            inner_walls: vec![InnerWall {
                x1: -5.0,
                y1: 9.5,
                x2: 66.0,
                y2: 64.5,
                width: 1.0,
                height: Some(11.0),
            }],
            ..gridfinity::Params::default()
        };
        let _solid = gridfinity::build(&p);
    }

    #[test]
    fn an_inner_wall_meeting_a_cavity_corner_arc_tessellates_closed_at_the_shared_vertex() {
        use crate::gridfinity::InnerWall;
        use crate::layout::GridCell;
        let p = gridfinity::Params {
            bins: vec![gridfinity::LogicalBin {
                cells: vec![GridCell { x: 1, y: 0 }],
                ..Default::default()
            }],
            inner_walls: vec![
                InnerWall {
                    x1: 91.5,
                    y1: -1.0,
                    x2: 27.0,
                    y2: 36.0,
                    width: 1.4,
                    height: None,
                },
                InnerWall {
                    x1: 40.0,
                    y1: 1.5,
                    x2: 92.0,
                    y2: 20.0,
                    width: 3.2,
                    height: None,
                },
            ],
            ..gridfinity::Params::default()
        };
        let solid = gridfinity::build(&p);
        let _ = tessellate(&solid, 6);
    }

    /// Multi-cell polyomino bins at `arc_segs_per_quarter = 1`, the coarsest
    /// resolution the tessellator is asked for. Coarse arcs put most planar
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
            (
                "plus",
                vec![cell(1, 0), cell(0, 1), cell(1, 1), cell(2, 1), cell(1, 2)],
            ),
            (
                "square3x3",
                (0..3)
                    .flat_map(|x| (0..3).map(move |y| cell(x, y)))
                    .collect(),
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
            let solid = gridfinity::build_piece(&p, &cells, &cells, None, &[])
                .unwrap_or_else(|e| panic!("{name} failed to build: {e:?}"));
            assert!(solid.validate().is_ok(), "{name} is not a manifold solid");
            let tess = tessellate(&solid, 1);
            let _ = (name, tess);
        }
    }
}

