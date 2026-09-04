//! A minimalistic analytic-surface B-rep CAD kernel.
//!
//! Everything the kernel models is exact: `sketch` states a profile in closed
//! form, `build` sweeps it into the `topo` half-edge solid, `region2d`,
//! `slab`, `split` and `isect` resolve booleans and cuts by intersecting the
//! surfaces themselves, and `fillet`, `chamfer` and `round` are
//! valid-solid-in, valid-solid-out operators over the result. Triangles appear
//! once, at the end, in `tess`, and are never read back; `xt` writes the same
//! solid as a Parasolid transmit file with no tessellation at all.
//!
//! The kernel knows nothing about what is built on it: no module here names a
//! bin, a cell or a drawer. Its two-dimensional vocabulary is shared with the
//! OCCT backend through `gridfinity-sketch`.

pub mod audit;
pub mod boolean;
pub mod build;
pub mod chamfer;
pub mod curvedge;
pub mod fillet;
pub mod geom;
pub mod isect;
pub mod mesh;
pub mod orient;
pub mod occt_api;
pub mod planar;
pub mod program;
pub mod slab;
pub mod split;
pub mod tess;
pub mod topo;
pub mod xt;

pub use gridfinity_sketch::{hash, math, nesting, perf, rectregion, region2d, round, sketch};
pub use occt_api::Shape;

#[cfg(test)]
#[global_allocator]
static TEST_ALLOC: perf::CountingAlloc<mimalloc::MiMalloc> =
    perf::CountingAlloc::new(mimalloc::MiMalloc);

#[cfg(test)]
mod tests {
    use crate::build::{Ring, extrude, loft};
    use crate::mesh::Mesh;
    use crate::sketch::Sketch;
    use crate::tess::tessellate;
    use std::collections::HashMap;


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
            .map(|&p| crate::math::weld_key(p))
            .collect();

        let buffer = tess.welded_render_buffer();
        assert_eq!(buffer.len(), mesh.indices.len() * 6);
        for v in buffer.chunks_exact(6) {
            let p = crate::math::Vec3::new(v[0] as f64, v[1] as f64, v[2] as f64);
            assert!(
                welded.contains(&crate::math::weld_key(p)),
                "{p:?} is not a welded position"
            );
            assert!((crate::math::Vec3::new(v[3] as f64, v[4] as f64, v[5] as f64).length() - 1.0).abs() < 1e-3);
        }

        let mut positions: Vec<crate::math::Vec3> = Vec::new();
        let mut index: HashMap<(i64, i64, i64), u32> = HashMap::new();
        let mut indices: Vec<u32> = Vec::new();
        for v in buffer.chunks_exact(6) {
            let p = crate::math::Vec3::new(v[0] as f64, v[1] as f64, v[2] as f64);
            let next = positions.len() as u32;
            let id = *index.entry(crate::math::weld_key(p)).or_insert_with(|| {
                positions.push(p);
                next
            });
            indices.push(id);
        }
        assert_watertight(&Mesh { positions, indices });
    }

    #[test]
    fn a_torus_section_lies_on_both_the_torus_and_the_cutting_plane() {
        use crate::geom::Curve;
        use crate::math::Vec3;
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
        use crate::geom::Curve;
        use crate::math::Vec3;
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
        b: &mut crate::topo::Builder,
        rect: &Sketch,
        z0: f64,
        z1: f64,
        outward: bool,
    ) {
        use crate::build::{ring, wall_between};
        use crate::geom::Surface;
        use crate::math::{Vec3, vec3_of};
        use crate::topo::Loop;
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
    /// their material inside. That is what makes a shell count meaningful to a
    /// caller carving one part out of another: a second shell where the caller
    /// asked for one connected part is material that broke off it.
    #[test]
    fn two_separated_lumps_of_material_are_two_shells_that_both_enclose_it() {
        let mut b = crate::topo::Builder::new();
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
    /// *outside* it. Nothing downstream of a boolean can see this -- it
    /// tessellates and welds like any other closed surface, and only the X_T
    /// writer refuses it -- so it is the shell's own material side that names
    /// it.
    #[test]
    fn a_void_sealed_inside_material_is_a_shell_that_encloses_none() {
        let mut b = crate::topo::Builder::new();
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
        use crate::build::prism;
        let outer = Sketch::rectangle(0.0, 0.0, 40.0, 40.0);
        let hole = Sketch::rectangle(0.0, 0.0, 20.0, 20.0);
        let solid = prism(&outer, &[hole], 0.0, 10.0);
        let _mesh = tessellate(&solid, 6).to_mesh();
    }
}
