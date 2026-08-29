//! The transmit-file entry point: one or more kernel solids as a Parasolid
//! XT text file.
//!
//! `to_xt_text` is the whole public surface. It owns the one thing no body
//! writer can -- the root node. The node at index 1 is where a reader starts,
//! and what it is depends on how many bodies there are: one body puts its BODY
//! node there, several put a POINTER_LIS_BLOCK listing them. Each body's own
//! nodes come from `topo::write_body`, which is free to allocate from wherever
//! the writer stands because pointers are indices, not positions.
//!
//! The tests here are the format's own first reader. They re-parse the emitted
//! text with a schema-driven parser -- character-level, because a char or null
//! field takes no trailing space and the next number runs straight into it --
//! and check that every index resolves, that the chains a reader walks (the
//! body's regions, each region's shells, each shell's faces, each edge's pair
//! of fins) close, and that the node counts are the counts the solid has. A
//! file only a real Parasolid frustrum would reject is meant to be caught here
//! first.

pub mod isect;
pub mod reader;
pub mod surf;
pub mod topo;
pub mod text;
pub mod validate;

pub use text::MM_PER_M;

use crate::kernel::topo::Solid;
use text::{Index, Writer, POINTER_LIS_BLOCK};

/// `bodies` as one XT transmit file: one BODY per solid, in the order given,
/// with a single BODY as the root when there is one and a POINTER_LIS_BLOCK
/// listing them when there are more. Fails, naming the body, where a solid
/// uses geometry the format cannot state -- see `topo::write_body` for what
/// those are.
pub fn to_xt_text(bodies: &[&Solid]) -> Result<String, String> {
    let mut w = Writer::new();
    match bodies {
        [] => {
            return Err(
                "a transmit file holds at least one body, so an export with no pieces is not \
                 one"
                    .to_string(),
            );
        }
        [only] => {
            topo::write_body(&mut w, only)?;
        }
        many => {
            let root = w.alloc();
            let mut entries: Vec<Index> = Vec::with_capacity(many.len());
            for (i, solid) in many.iter().enumerate() {
                entries.push(
                    topo::write_body(&mut w, solid)
                        .map_err(|e| format!("body {} of {}: {e}", i + 1, many.len()))?,
                );
            }
            w.begin_var(POINTER_LIS_BLOCK, entries.len(), root);
            w.int(entries.len() as i64);
            w.ptr(0);
            for &e in &entries {
                w.ptr(e);
            }
        }
    }
    Ok(w.finish())
}

/// The solids every `xt` test writes: the smallest thing the kernel builds,
/// a stock bin, and the one shape whose pieces carry a `Curve::TorusSection`.
/// They live here rather than in either test module because both read them,
/// and a second copy of the split L is a second thing to keep in step with
/// `carve_to_cells`.
#[cfg(test)]
pub(crate) mod fixtures {
    use crate::gridfinity;
    use crate::kernel::build::extrude;
    use crate::kernel::sketch::Sketch;
    use crate::kernel::topo::Solid;
    use crate::layout::{Axis, GridCell, SplitLine};

/// A plain extruded box, the smallest solid the kernel builds.
pub(crate) fn cube_solid() -> Solid {
    extrude(&Sketch::rounded_rect(0.0, 0.0, 10.0, 10.0, 0.0), 0.0, 5.0)
}

/// A default-parameter rectangular bin.
pub(crate) fn rect_solid(gx: u32, gy: u32) -> Solid {
    gridfinity::build(&gridfinity::Params::rect(gx, gy))
}

/// An L-shaped bin split in two, whose carve planes cross the reentrant
/// corner's fillet torus -- the shape whose pieces carry
/// `Curve::TorusSection` edges and therefore INTERSECTION nodes.
pub(crate) fn split_l_solids() -> Vec<Solid> {
    let cells = vec![
        GridCell { x: 0, y: 0 },
        GridCell { x: 1, y: 0 },
        GridCell { x: 0, y: 1 },
    ];
    let mut p = gridfinity::Params {
        bins: vec![gridfinity::LogicalBin {
            cells: cells.clone(),
            ..Default::default()
        }],
        ..gridfinity::Params::default()
    };
    p.bins[0].split_lines = vec![SplitLine { axis: Axis::X, index: 1 }];
    let whole = gridfinity::build_bin_solid(&p, &cells, None, &[]).expect("the L-shaped bin builds");
    crate::layout::partition_cells(&cells, &p.bins[0].split_lines)
        .iter()
        .map(|part| {
            gridfinity::carve_to_cells(&whole, &cells, &part.cells).expect("each piece carves")
        })
        .collect()
}
}

#[cfg(test)]
mod tests {
    use super::reader::{self, Node};
    use super::text::{
        BODY, CHART, CIRCLE, CONE, CYLINDER, EDGE, ELLIPSE, FACE, FIN, GEOMETRIC_OWNER,
        INTERSECTION, LIMIT, LINE, LOOP, PLANE, POINT, POINTER_LIS_BLOCK, SPHERE, TORUS, VERTEX,
    };
    use super::fixtures::{cube_solid, rect_solid, split_l_solids};
    use super::validate::validate_xt;
    use super::*;
    use crate::gridfinity;
    use crate::kernel::math::Vec3;
    use crate::kernel::xt::surf;

    /// `text` parsed, having first held it to everything `validate_xt` knows a
    /// transmit file of this schema must satisfy. Every test here reads its
    /// file through this, so none can pass on a file a reader would reject.
    fn parsed(text: &str) -> Vec<Node> {
        let findings = validate_xt(text);
        assert!(
            findings.is_empty(),
            "the file validates, but:\n{}",
            findings
                .iter()
                .map(|f| format!("  node {}: {}\n", f.node, f.message))
                .collect::<String>()
        );
        reader::parse(text).expect("a file the validator passed parses").nodes
    }

    /// The node of index `i`, for tests that walk the file directly.
    fn f_of(nodes: &[Node], i: u32) -> &Node {
        nodes
            .iter()
            .find(|n| n.index == i)
            .unwrap_or_else(|| panic!("node {i} was emitted, so the file holds it"))
    }

    /// Holds `text` against the solids it claims to carry: the count of each
    /// topology node type equals the kernel's own count of faces, loops,
    /// face-used edges and edge-reached vertices, and the root is the node the
    /// body count calls for. Everything a reader checks about the graph itself
    /// -- indices, pointers, pointer classes, every chain and its coverage,
    /// and the geometry -- belongs to `validate_xt`, which `parsed` has
    /// already run over the same text.
    fn check_round_trip(text: &str, bodies: &[&Solid]) {
        let nodes = parsed(text);
        let root = f_of(&nodes, 1);
        if bodies.len() == 1 {
            assert_eq!(root.ty, BODY, "one body puts its BODY node at the root");
        } else {
            assert_eq!(root.ty, POINTER_LIS_BLOCK, "many bodies share a list root");
            assert_eq!(root.len, Some(bodies.len()));
            assert_eq!(root.ptrs.len(), 1 + bodies.len());
            assert_eq!(root.ints[0], bodies.len() as i64);
        }

        let mut want_faces = 0;
        let mut want_loops = 0;
        let mut want_edges = 0;
        let mut want_verts = 0;
        for solid in bodies {
            want_faces += solid.faces.len();
            want_loops += (0..solid.faces.len())
                .map(|fi| 1 + solid.n_inners(fi))
                .sum::<usize>();
            let used = used_edges(solid);
            let mut live = vec![false; solid.verts.len()];
            for (e, u) in used.iter().enumerate() {
                if *u {
                    live[solid.edges[e].v0] = true;
                    live[solid.edges[e].v1] = true;
                }
            }
            want_edges += used.iter().filter(|u| **u).count();
            want_verts += live.iter().filter(|l| **l).count();
        }
        let n = |t| nodes.iter().filter(|x| x.ty == t).count();
        assert_eq!(n(BODY), bodies.len());
        assert_eq!(n(FACE), want_faces, "one FACE per kernel face");
        assert_eq!(n(LOOP), want_loops, "one LOOP per kernel loop");
        assert_eq!(n(EDGE), want_edges, "one EDGE per face-used edge");
        assert_eq!(n(VERTEX), want_verts, "one VERTEX per edge-reached vertex");
        assert_eq!(n(POINT), want_verts, "one POINT per edge-reached vertex");
        assert_eq!(n(FIN), 2 * want_edges, "two fins per face-used edge");
    }

    #[test]
    fn a_cube_is_one_body_whose_graph_reads_back() {
        let cube = cube_solid();
        let text = to_xt_text(&[&cube]).expect("a cube is stateable in the format");
        check_round_trip(&text, &[&cube]);
        assert!(
            !parsed(&text).iter().any(|n| n.ty == INTERSECTION),
            "a cube has no intersection curves"
        );
    }

    #[test]
    fn two_bodies_root_in_a_pointer_list_block() {
        let cube = cube_solid();
        let bin = rect_solid(1, 1);
        let text = to_xt_text(&[&cube, &bin]).expect("both bodies are stateable");
        check_round_trip(&text, &[&cube, &bin]);
    }

    #[test]
    fn a_split_l_shaped_bin_writes_its_torus_sections_as_exact_intersections() {
        let pieces = split_l_solids();
        let sections: usize = pieces
            .iter()
            .map(|s| {
                s.edges
                    .iter()
                    .filter(|e| {
                        matches!(e.curve, crate::kernel::geom::Curve::TorusSection { .. })
                    })
                    .count()
            })
            .sum();
        assert_eq!(sections, 2, "the L's carve planes cross the corner fillet twice");
        let bodies: Vec<&Solid> = pieces.iter().collect();
        let text = to_xt_text(&bodies).expect("the split L is stateable in the format");
        check_round_trip(&text, &bodies);

        let nodes = parsed(&text);
        let isects: Vec<&Node> = nodes.iter().filter(|n| n.ty == INTERSECTION).collect();
        assert_eq!(isects.len(), sections, "one INTERSECTION node per torus section");
        for isect in &isects {
            assert!(
                isect.chars[0] == '+' || isect.chars[0] == '-',
                "an intersection curve's sense says which way its edge runs along the chart, so \
                 it is a sign"
            );
            // Geometry ptrs: attrs, owner, next, previous, geometric_owner;
            // INTERSECTION continues: surface[2], chart, start, end.
            let chart = f_of(&nodes, isect.ptrs[7]);
            assert_eq!(chart.ty, CHART);
            assert_eq!(chart.len, Some(chart.ints[0] as usize), "the chart count matches");
            assert!(chart.vecs.len() >= 2, "a chart spans its curve");
            assert_eq!(
                chart.vecs.len(),
                chart.len.expect("a chart carries its own length"),
                "a chart's points are its length"
            );
            for limit in [isect.ptrs[8], isect.ptrs[9]] {
                let l = f_of(&nodes, limit);
                assert_eq!(l.ty, LIMIT);
                assert_eq!(l.chars[0], 'L', "a split's curve ends are arbitrary limits");
                assert_eq!(l.len, Some(1));
            }
            let surfaces: Vec<&Node> = isect.ptrs[5..7].iter().map(|&s| f_of(&nodes, s)).collect();
            assert_eq!(
                surfaces[0].ty,
                TORUS,
                "a torus section is the intersection of its torus and the cut plane"
            );
            assert_eq!(surfaces[1].ty, PLANE);
            for surface in surfaces {
                let owner = f_of(&nodes, surface.ptrs[4]);
                assert_eq!(
                    owner.ty, GEOMETRIC_OWNER,
                    "a surface an intersection depends on carries its owner ring"
                );
                assert_eq!(owner.ptrs[3], surface.index, "the ring names the shared surface");
                assert_eq!(
                    owner.ptrs[0], isect.index,
                    "the ring's referencing geometry is the intersection itself"
                );
            }
        }
    }

    #[test]
    fn a_default_bin_transmits_its_analytic_surfaces_and_reports_its_deviation() {
        let bin = rect_solid(1, 1);
        let text = to_xt_text(&[&bin]).expect("a default bin is stateable in the format");
        check_round_trip(&text, &[&bin]);
        let nodes = parsed(&text);
        let kinds = |t| nodes.iter().filter(|n| n.ty == t).count();
        assert!(
            kinds(TORUS) + kinds(CYLINDER) + kinds(SPHERE) + kinds(CONE) > 0,
            "the bin's rounded corners and baseplate features reach the file as analytic \
             surfaces, not planes"
        );
        assert!(kinds(PLANE) > 0);
        let used = used_edges(&bin);
        let want_curves = used.iter().filter(|u| **u).count();
        assert_eq!(
            kinds(LINE) + kinds(CIRCLE) + kinds(ELLIPSE) + kinds(INTERSECTION),
            want_curves,
            "one curve node per used edge"
        );

        let (max_surface, max_curve) = deviation(&bin);
        println!(
            "xt deviation over rect(1,1): surface {max_surface:.3e} mm, curve {max_curve:.3e} mm"
        );
        println!(
            "in the file's metres: surface {:+.3e} m, curve {:+.3e} m, against a declared \
             res_linear of 1e-8 m",
            max_surface as f64 / MM_PER_M,
            max_curve as f64 / MM_PER_M
        );
        assert!(
            max_surface < surf::ON_GEOMETRY_MM && max_curve < surf::ON_GEOMETRY_MM,
            "every point the kernel puts on geometry stands within the writer's own bound of \
             the node written for it"
        );
    }

    /// Which edges any face loop of `solid` uses.
    fn used_edges(solid: &Solid) -> Vec<bool> {
        let mut used = vec![false; solid.edges.len()];
        for fi in 0..solid.faces.len() {
            for lp in solid.face_loops(fi) {
                for &(e, _) in lp {
                    used[e] = true;
                }
            }
        }
        used
    }

    /// The largest distance, in millimetres, from any point the kernel places
    /// on geometry to the node the writer states for it: each face's samples
    /// against its surface node, and each used edge's midpoint and endpoints
    /// against its curve node -- or, for a torus section written as an
    /// intersection, against both of the surfaces that intersection lies on.
    /// This is the f64-versus-res_linear number: it says how far the
    /// millimetres the kernel computes can sit from the analytic forms a file
    /// declaring 1e-8 m of linear resolution implies.
    fn deviation(solid: &Solid) -> (f64, f64) {
        let mut max_surface = 0.0f64;
        let mut max_curve = 0.0f64;
        let used = used_edges(solid);
        let ef = solid.edge_faces();
        for fi in 0..solid.faces.len() {
            let face = &solid.faces[fi];
            let mut samples: Vec<Vec3> = Vec::new();
            for lp in solid.face_loops(fi) {
                for &(e, _) in lp {
                    samples.push(solid.vertex(solid.edges[e].v0));
                    samples.push(
                        solid.edges[e]
                            .curve
                            .point((solid.edges[e].t0 + solid.edges[e].t1) * 0.5),
                    );
                }
            }
            surf::check_surface(&face.surface, &samples)
                .expect("the writer wrote this face, so the test can measure it");
            for &p in &samples {
                max_surface = max_surface.max(face.surface.signed_distance(p).abs());
            }
        }
        for (e, edge) in solid.edges.iter().enumerate() {
            if !used[e] {
                continue;
            }
            let points = [
                solid.vertex(edge.v0),
                edge.curve.point((edge.t0 + edge.t1) * 0.5),
                solid.vertex(edge.v1),
            ];
            match surf::of_curve(&edge.curve) {
                Some(curve) => {
                    for p in points {
                        max_curve = max_curve.max(curve.distance(p).abs());
                    }
                }
                None => {
                    for p in points {
                        for &fi in &ef[e] {
                            max_curve = max_curve
                                .max(solid.faces[fi].surface.signed_distance(p).abs());
                        }
                    }
                }
            }
        }
        (max_surface, max_curve)
    }

    /// A `rect(1, 1)` bin with no floor fillet, so its cavity meets its floor
    /// sharp: the one bin in the ladder below carrying neither a blend TORUS
    /// nor the ELLIPSE a blend's runout section is.
    fn unfilleted_bin() -> Solid {
        let mut p = gridfinity::Params::rect(1, 1);
        p.floor_fillet = 0.0;
        gridfinity::build(&p)
    }

    /// Writes the import ladder to `XT_LADDER_DIR` (default `xt-ladder`), one
    /// file per rung, each adding exactly one thing the rung before it lacks:
    /// the frame alone, then curved surfaces, then the blend torus and its
    /// runout ellipse, then a list root, then intersection curves. A CAD
    /// system that refuses one of them names the node class at fault by which
    /// one it is -- the only signal a real Parasolid reader gives us from
    /// here, since nothing in this repo can run one.
    #[test]
    #[ignore]
    fn writes_the_import_ladder() {
        let dir = std::env::var("XT_LADDER_DIR").unwrap_or_else(|_| "xt-ladder".to_string());
        std::fs::create_dir_all(&dir).expect("the ladder's directory is writable");
        let cube = cube_solid();
        let flat = unfilleted_bin();
        let bin = rect_solid(1, 1);
        let split = split_l_solids();
        let split_refs: Vec<&Solid> = split.iter().collect();
        let ladder: Vec<(&str, Vec<&Solid>)> = vec![
            ("1-cube", vec![&cube]),
            ("2-bin-nofillet", vec![&flat]),
            ("3-bin", vec![&bin]),
            ("4-two-bodies", vec![&cube, &bin]),
            ("5-split-l", split_refs),
        ];
        for (name, bodies) in ladder {
            let text = to_xt_text(&bodies).unwrap_or_else(|e| panic!("{name} is stateable: {e}"));
            let findings = validate_xt(&text);
            let nodes = reader::parse(&text).expect("the ladder's files parse").nodes;
            let kinds = |t| nodes.iter().filter(|n| n.ty == t).count();
            let path = format!("{dir}/{name}.x_t");
            std::fs::write(&path, &text).expect("each ladder file is writable");
            println!(
                "{path}: {} bytes, {} nodes, {} bodies -- plane {} cyl {} cone {} sphere {} \
                 torus {} line {} circle {} ellipse {} intersection {} -- {} finding(s)",
                text.len(),
                nodes.len(),
                kinds(BODY),
                kinds(PLANE),
                kinds(CYLINDER),
                kinds(CONE),
                kinds(SPHERE),
                kinds(TORUS),
                kinds(LINE),
                kinds(CIRCLE),
                kinds(ELLIPSE),
                kinds(INTERSECTION),
                findings.len()
            );
            for f in &findings {
                println!("    node {}: {}", f.node, f.message);
            }
        }
    }
}
