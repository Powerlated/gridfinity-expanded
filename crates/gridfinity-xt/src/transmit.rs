//! The transmit-file entry point: one or more kernel solids as a Parasolid
//! XT text file.
//!
//! `to_xt_text` is the whole public surface. It owns the one thing no body
//! writer can -- the root node. The node at index 1 is where a reader starts,
//! and what it is depends on how many bodies there are: one body puts its BODY
//! node there, several put a POINTER_LIS_BLOCK listing them. Each body's own
//! nodes come from `body::write_body`, which is free to allocate from wherever
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



use crate::body;
use crate::topo::Solid;
use crate::text::{Index, Writer, POINTER_LIS_BLOCK};

/// `bodies` as one XT transmit file: one BODY per solid, in the order given,
/// with a single BODY as the root when there is one and a POINTER_LIS_BLOCK
/// listing them when there are more. Fails, naming the body, where a solid
/// uses geometry the format cannot state -- see `body::write_body` for what
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
            body::write_body(&mut w, only)?;
        }
        many => {
            let root = w.alloc();
            let mut entries: Vec<Index> = Vec::with_capacity(many.len());
            for (i, solid) in many.iter().enumerate() {
                entries.push(
                    body::write_body(&mut w, solid)
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
#[cfg(all(test, feature = "occt"))]
pub(crate) mod fixtures {
    use crate::occt::to_solid;
    use crate::topo::Solid;
    use gridfinity_occt::{FilletEdge, Profile, Seg, Shape};
    use std::f64::consts::PI;

    /// A plain box, the smallest body there is: six planes and twelve lines.
    pub(crate) fn cube_solid() -> Solid {
        to_solid(&Shape::box_solid(10.0, 10.0, 5.0).expect("OCCT box")).expect("read a box")
    }

    /// The counter-clockwise loop of a `w` by `d` rectangle with its corners
    /// rounded to `r`, which sweeps to cylinders and circles as well as planes.
    fn rounded_rect(w: f64, d: f64, r: f64) -> Profile {
        rounded_rect_at(0.0, 0.0, w, d, r)
    }

    /// The same loop with its lower-left corner at `(x0, y0)`.
    fn rounded_rect_at(x0: f64, y0: f64, w: f64, d: f64, r: f64) -> Profile {
        let shift = |p: [f64; 2]| [p[0] + x0, p[1] + y0];
        Profile {
            loops: rounded_rect_origin(w, d, r)
                .loops
                .iter()
                .map(|one| {
                    one.iter()
                        .map(|seg| match *seg {
                            Seg::Line { a, b } => Seg::Line {
                                a: shift(a),
                                b: shift(b),
                            },
                            Seg::Arc {
                                a,
                                b,
                                center,
                                radius,
                                a0,
                                a1,
                            } => Seg::Arc {
                                a: shift(a),
                                b: shift(b),
                                center: shift(center),
                                radius,
                                a0,
                                a1,
                            },
                        })
                        .collect()
                })
                .collect(),
        }
    }

    /// The loop itself, at the origin.
    fn rounded_rect_origin(w: f64, d: f64, r: f64) -> Profile {
        let arc = |a: [f64; 2], b: [f64; 2], center: [f64; 2], a0: f64, a1: f64| Seg::Arc {
            a,
            b,
            center,
            radius: r,
            a0,
            a1,
        };
        Profile::of(vec![
            Seg::Line {
                a: [r, 0.0],
                b: [w - r, 0.0],
            },
            arc([w - r, 0.0], [w, r], [w - r, r], -PI / 2.0, 0.0),
            Seg::Line {
                a: [w, r],
                b: [w, d - r],
            },
            arc([w, d - r], [w - r, d], [w - r, d - r], 0.0, PI / 2.0),
            Seg::Line {
                a: [w - r, d],
                b: [r, d],
            },
            arc([r, d], [0.0, d - r], [r, d - r], PI / 2.0, PI),
            Seg::Line {
                a: [0.0, d - r],
                b: [0.0, r],
            },
            arc([0.0, r], [r, 0.0], [r, r], PI, 1.5 * PI),
        ])
    }

    /// A rounded-rect prism with its whole top rim blended. Its four vertical
    /// corners are cylinders, its four top corners tori, the four runs between
    /// them cylinders again, and the rim's own edges circles -- every surface
    /// and curve class the writer emits except the cone.
    pub(crate) fn blended_solid() -> Solid {
        let (w, d, h, corner, blend) = (40.0, 30.0, 12.0, 5.0, 2.0);
        let prism = Shape::prism(&rounded_rect(w, d, corner), 0.0, h).expect("OCCT prism");
        let straight = [
            [w / 2.0, 0.0, h],
            [w / 2.0, d, h],
            [0.0, d / 2.0, h],
            [w, d / 2.0, h],
        ];
        let diagonal = corner - corner / 2f64.sqrt();
        let corners = [
            [diagonal, diagonal, h],
            [w - diagonal, diagonal, h],
            [diagonal, d - diagonal, h],
            [w - diagonal, d - diagonal, h],
        ];
        let rim: Vec<FilletEdge> = straight
            .into_iter()
            .chain(corners)
            .map(|midpoint| FilletEdge {
                midpoint,
                radius: blend,
            })
            .collect();
        assert_eq!(
            rim.len(),
            8,
            "a rounded rect's top rim is four straight runs and four arcs"
        );
        let rounded = prism.fillet(&rim, 1e-6).expect("a closed rim blend lands on every edge");
        to_solid(&rounded).expect("read a blended prism")
    }

    /// A rounded-rect prism hollowed by a smaller one, which is a body whose
    /// rim face carries a hole loop as well as an outer -- the nesting a bin's
    /// own rim has, and a class no solid prism reaches.
    ///
    /// This stood as a truncated cone, for the one analytic surface an
    /// extrusion never makes. A full cone of revolution has a **seam** edge,
    /// used twice by one loop, which `occt::to_solid` refuses; until the bridge
    /// can split a closed surface there is no seam-free OCCT source for a cone
    /// face, so the CONE node is written by nothing. See the crate docs.
    /// The blended prism cut through its corner blends, as the two pieces an
    /// export writes. The cut meets each corner torus in a quartic, so the
    /// pieces carry the section curves nothing analytic names -- the one input
    /// that exercises INTERSECTION, CHART, LIMIT and GEOMETRIC_OWNER.
    pub(crate) fn cut_solids() -> Vec<Solid> {
        let (w, d, h, corner, blend) = (40.0, 30.0, 12.0, 5.0, 2.0);
        let prism = Shape::prism(&rounded_rect(w, d, corner), 0.0, h).expect("prism");
        let diagonal = corner - corner / 2f64.sqrt();
        let rim: Vec<FilletEdge> = [
            [w / 2.0, 0.0, h],
            [w / 2.0, d, h],
            [0.0, d / 2.0, h],
            [w, d / 2.0, h],
            [diagonal, diagonal, h],
            [w - diagonal, diagonal, h],
            [diagonal, d - diagonal, h],
            [w - diagonal, d - diagonal, h],
        ]
        .into_iter()
        .map(|midpoint| FilletEdge {
            midpoint,
            radius: blend,
        })
        .collect();
        let body = prism.fillet(&rim, 1e-6).expect("a closed rim blend");
        let tool = Shape::box_solid(corner - 1.0, d * 2.0, h * 2.0).expect("the cutting tool");
        let right = body
            .boolean(&tool, gridfinity_occt::Boolean::Cut)
            .expect("keep the right");
        let left = body
            .boolean(&right, gridfinity_occt::Boolean::Cut)
            .expect("keep the left");
        vec![
            to_solid(&left).expect("read the left piece"),
            to_solid(&right).expect("read the right piece"),
        ]
    }

    pub(crate) fn hollow_solid() -> Solid {
        let outer = Shape::prism(&rounded_rect(40.0, 30.0, 5.0), 0.0, 12.0).expect("outer");
        let inner =
            Shape::prism(&rounded_rect_at(5.0, 5.0, 30.0, 20.0, 3.0), 3.0, 12.0).expect("inner");
        let hollow = outer
            .boolean(&inner, gridfinity_occt::Boolean::Cut)
            .expect("hollow it");
        to_solid(&hollow).expect("read a hollow prism")
    }
}

#[cfg(all(test, feature = "occt"))]
mod tests {
    use crate::reader::{self, Node};
    use crate::text::{
        BODY, CHART, CIRCLE, CONE, CYLINDER, EDGE, ELLIPSE, FACE, FIN, GEOMETRIC_OWNER,
        INTERSECTION, LIMIT, LINE, LOOP, PLANE, POINT, POINTER_LIS_BLOCK, SPHERE, TORUS, VERTEX,
    };
    use super::fixtures::{blended_solid, cube_solid, hollow_solid};
    use crate::validate::validate_xt;
    use super::*;
    use crate::MM_PER_M;
    use crate::math::Vec3;
    use crate::surf;

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
    /// the circles bounding it, then a list root. A CAD system that refuses one
    /// of them names the node class at fault by which one it is -- the only
    /// signal a real Parasolid reader gives us from here, since nothing in this
    /// repo can run one.
    #[test]
    #[ignore]
    fn writes_the_import_ladder() {
        let dir = std::env::var("XT_LADDER_DIR").unwrap_or_else(|_| "xt-ladder".to_string());
        std::fs::create_dir_all(&dir).expect("the ladder's directory is writable");
        let cube = cube_solid();
        let lofted = hollow_solid();
        let blended = blended_solid();
        let cut = fixtures::cut_solids();
        let cut_refs: Vec<&Solid> = cut.iter().collect();
        let ladder: Vec<(&str, Vec<&Solid>)> = vec![
            ("1-cube", vec![&cube]),
            ("2-hollow", vec![&lofted]),
            ("3-blended", vec![&blended]),
            ("4-two-bodies", vec![&cube, &blended]),
            ("5-cut-blend", cut_refs),
        ];
        for (name, bodies) in ladder {
            let text = to_xt_text(&bodies).unwrap_or_else(|e| panic!("{name} is stateable: {e}"));
            let findings = validate_xt(&text);
            let path = format!("{dir}/{name}.x_t");
            std::fs::write(&path, &text).expect("each ladder file is writable");
            println!("{path}: {} bytes, {} finding(s)", text.len(), findings.len());
            assert!(findings.is_empty(), "{name} validates clean before it is shipped");
        }
    }
}
