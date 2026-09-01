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

use crate::topo::Solid;
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

/// The solids every `xt` test writes, each chosen for the node classes it
/// forces the writer to emit: a box for planes and lines alone, a blended
/// prism for cylinders, tori and circles, a loft for cones, and a cut through
/// the blended prism for the `Curve::TorusSection` edges that become
/// INTERSECTION nodes. They live here rather than in either test module
/// because both read them.
#[cfg(test)]
pub(crate) mod fixtures {
    use crate::build::{Ring, extrude, loft};
    use crate::fillet::fillet_edges;
    use crate::geom::Surface;
    use crate::sketch::Sketch;
    use crate::split::{Side, trim_half_space};
    use crate::math::{Vec3, vec3_of};
    use crate::topo::{EdgeId, Solid};

    /// How many `Curve::TorusSection` edges `split_blended_solids` carries
    /// between its two pieces: the cut plane crosses two of the prism's four
    /// top corner blends, and each piece keeps its own half of each crossing.
    pub(crate) const SPLIT_TORUS_SECTIONS: usize = 4;

    /// A plain extruded box, the smallest solid the kernel builds.
    pub(crate) fn cube_solid() -> Solid {
        extrude(&Sketch::rounded_rect(0.0, 0.0, 10.0, 10.0, 0.0), 0.0, 5.0)
    }

    /// A rounded-rect prism with its whole top rim blended at `r`. Its four
    /// vertical corners are cylinders, its four top corners tori, the four
    /// runs between them cylinders again, and the rim's own edges circles --
    /// every surface and curve class the writer emits except the cone.
    pub(crate) fn blended_solid() -> Solid {
        let (w, d, h, corner) = (40.0, 30.0, 12.0, 5.0);
        let solid = extrude(&Sketch::rounded_rect(0.0, 0.0, w, d, corner), 0.0, h);
        let rim: Vec<(EdgeId, f64)> = (0..solid.edges.len())
            .filter(|&e| {
                let ed = solid.edges[e];
                [solid.vertex(ed.v0), solid.vertex(ed.v1)]
                    .iter()
                    .all(|p| (p.z - h).abs() < 1e-9)
            })
            .map(|e| (e, 2.0))
            .collect();
        assert_eq!(
            rim.len(),
            8,
            "a rounded rect's top rim is four straight runs and four arcs, not {} edge(s)",
            rim.len()
        );
        fillet_edges(&solid, &rim).expect("a closed rim blend lands on every edge of the run")
    }

    /// A loft from a wide rounded rect to one 3 mm narrower on every side with
    /// its corner radius 3 mm smaller, so each corner arc keeps its centre and
    /// only its radius changes: that is what makes the four corner faces true
    /// cones about a vertical axis, the one surface class a prism never
    /// produces.
    pub(crate) fn lofted_solid() -> Solid {
        let lo = Sketch::rounded_rect(0.0, 0.0, 40.0, 30.0, 6.0);
        let hi = Sketch::rounded_rect(0.0, 0.0, 34.0, 24.0, 3.0);
        let solid = loft(&[
            Ring { z: 0.0, sketch: &lo },
            Ring { z: 9.0, sketch: &hi },
        ]);
        assert!(
            solid
                .faces
                .iter()
                .any(|f| matches!(f.surface, Surface::Cone { .. })),
            "a loft between corners of different radii is what makes a cone, and this one made none"
        );
        solid
    }

    /// `blended_solid` cut in two by the plane `x = 17`, which passes through
    /// both of its right-hand top corner blends -- their tori are centred on
    /// `x = 15` and reach to `x = 20`. Each piece keeps the part of each torus
    /// on its own side, so the cut edge across each is a
    /// `Curve::TorusSection`.
    pub(crate) fn split_blended_solids() -> Vec<Solid> {
        let solid = blended_solid();
        let plane = Surface::plane(vec3_of(17.0, 0.0, 0.0), Vec3::X);
        [Side::Negative, Side::Positive]
            .into_iter()
            .map(|keep| {
                trim_half_space(&solid, &plane, keep)
                    .expect("a plane through a blended prism cuts it in two")
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
    use super::fixtures::{
        SPLIT_TORUS_SECTIONS, blended_solid, cube_solid, lofted_solid, split_blended_solids,
    };
    use super::validate::validate_xt;
    use super::*;
    use crate::math::Vec3;
    use crate::xt::surf;

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
        let blended = blended_solid();
        let text = to_xt_text(&[&cube, &blended]).expect("both bodies are stateable");
        check_round_trip(&text, &[&cube, &blended]);
    }

    #[test]
    fn a_cut_through_a_corner_blend_writes_its_torus_sections_as_exact_intersections() {
        let pieces = split_blended_solids();
        let sections: usize = pieces
            .iter()
            .map(|s| {
                s.edges
                    .iter()
                    .filter(|e| {
                        matches!(e.curve, crate::geom::Curve::TorusSection { .. })
                    })
                    .count()
            })
            .sum();
        assert_eq!(
            sections, SPLIT_TORUS_SECTIONS,
            "the cut plane crosses the prism's corner blends, and each piece keeps its own              half of every crossing"
        );
        let bodies: Vec<&Solid> = pieces.iter().collect();
        let text = to_xt_text(&bodies).expect("the cut prism is stateable in the format");
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
                let mut walk = owner;
                let mut referencing = Vec::new();
                loop {
                    assert_eq!(walk.ty, GEOMETRIC_OWNER, "every node of the ring is an owner");
                    assert_eq!(
                        walk.ptrs[3], surface.index,
                        "every owner in the ring names the one shared surface"
                    );
                    let back = f_of(&nodes, walk.ptrs[1]);
                    assert_eq!(
                        back.ptrs[2], walk.index,
                        "the ring's next and previous are mutual"
                    );
                    referencing.push(walk.ptrs[0]);
                    walk = back;
                    assert!(
                        referencing.len() <= nodes.len(),
                        "the owner ring closes rather than running on forever"
                    );
                    if walk.index == owner.index {
                        break;
                    }
                }
                assert!(
                    referencing.contains(&isect.index),
                    "the surface's owner ring holds this intersection: it lists {referencing:?},                      not {}",
                    isect.index
                );
            }
        }
    }

    #[test]
    fn a_blended_prism_transmits_its_analytic_surfaces_and_reports_its_deviation() {
        let bin = blended_solid();
        let text = to_xt_text(&[&bin]).expect("a blended prism is stateable in the format");
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

    /// Writes the import ladder to `XT_LADDER_DIR` (default `xt-ladder`), one
    /// file per rung, each adding exactly one node class the rung before it
    /// lacks: planes and lines alone, then the cone, then the blend torus and
    /// the circles bounding it, then a list root, then intersection curves. A CAD
    /// system that refuses one of them names the node class at fault by which
    /// one it is -- the only signal a real Parasolid reader gives us from
    /// here, since nothing in this repo can run one.
    #[test]
    #[ignore]
    fn writes_the_import_ladder() {
        let dir = std::env::var("XT_LADDER_DIR").unwrap_or_else(|_| "xt-ladder".to_string());
        std::fs::create_dir_all(&dir).expect("the ladder's directory is writable");
        let cube = cube_solid();
        let lofted = lofted_solid();
        let blended = blended_solid();
        let split = split_blended_solids();
        let split_refs: Vec<&Solid> = split.iter().collect();
        let ladder: Vec<(&str, Vec<&Solid>)> = vec![
            ("1-cube", vec![&cube]),
            ("2-loft", vec![&lofted]),
            ("3-blended", vec![&blended]),
            ("4-two-bodies", vec![&cube, &blended]),
            ("5-split-blend", split_refs),
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
