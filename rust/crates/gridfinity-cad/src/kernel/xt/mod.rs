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
pub mod surf;
pub mod topo;
pub mod text;

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

#[cfg(test)]
mod tests {
    use super::text::{BODY, CHART, CONE, CYLINDER, EDGE, ELLIPSE, FACE, FIN, GEOMETRIC_OWNER,
        INTERSECTION, LIMIT, LINE, LOOP, PLANE, POINT, POINTER_LIS_BLOCK, REGION, SHELL, SPHERE,
        TORUS, VERTEX, CIRCLE};
    use super::*;
    use crate::gridfinity;
    use crate::kernel::build::extrude;
    use crate::kernel::math::Vec3;
    use crate::kernel::sketch::Sketch;
    use crate::kernel::xt::surf;

    /// What one field of a node is, which says how it sits in the text stream:
    /// a number ends at the one space after it, a char or a null is one
    /// character the next field runs straight into, and a vector is three
    /// numbers.
    #[derive(Clone, Copy, PartialEq)]
    enum Kind {
        Int,
        Dbl,
        Ptr,
        NullableDbl,
        Chr,
        Vec,
    }

    /// One parsed node: its index, its variable length for the types that have
    /// one, and its fields split by kind so a pointer can be checked as a
    /// pointer and an int as an int.
    struct Node {
        ty: u16,
        len: Option<usize>,
        index: u32,
        ints: Vec<i64>,
        dbls: Vec<f64>,
        ptrs: Vec<u32>,
        chars: Vec<char>,
        vecs: Vec<[f64; 3]>,
        nulls: usize,
    }

    /// The field kinds of node type `ty` in schema order, `len` variable fields
    /// expanded -- transcribed from the format's own node structures, which are
    /// the same tables `topo` and `surf` write from.
    fn schema(ty: u16, len: usize) -> Vec<Kind> {
        let common = [
            Kind::Int,
            Kind::Ptr,
            Kind::Ptr,
            Kind::Ptr,
            Kind::Ptr,
            Kind::Ptr,
        ];
        let mut k: Vec<Kind> = match ty {
            BODY => vec![
                Kind::Int, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr,
                Kind::Dbl, Kind::Dbl, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Int, Kind::Ptr,
                Kind::Int, Kind::Int, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr,
                Kind::Ptr, Kind::Ptr,
            ],
            REGION => vec![
                Kind::Int, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Chr,
            ],
            SHELL => vec![
                Kind::Int, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr,
                Kind::Ptr, Kind::Ptr,
            ],
            FACE => vec![
                Kind::Int, Kind::Ptr, Kind::NullableDbl, Kind::Ptr, Kind::Ptr, Kind::Ptr,
                Kind::Ptr, Kind::Ptr, Kind::Chr, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr,
                Kind::Ptr,
            ],
            LOOP => vec![
                Kind::Int, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr,
            ],
            EDGE => vec![
                Kind::Int, Kind::Ptr, Kind::NullableDbl, Kind::Ptr, Kind::Ptr, Kind::Ptr,
                Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr,
            ],
            FIN => vec![
                Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr,
                Kind::Ptr, Kind::Ptr, Kind::Chr,
            ],
            VERTEX => vec![
                Kind::Int, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr,
                Kind::NullableDbl, Kind::Ptr,
            ],
            POINT => vec![
                Kind::Int, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Vec,
            ],
            LINE => common_plus(&common, vec![Kind::Chr, Kind::Vec, Kind::Vec]),
            CIRCLE => common_plus(
                &common,
                vec![Kind::Chr, Kind::Vec, Kind::Vec, Kind::Vec, Kind::Dbl],
            ),
            ELLIPSE => common_plus(
                &common,
                vec![
                    Kind::Chr,
                    Kind::Vec,
                    Kind::Vec,
                    Kind::Vec,
                    Kind::Dbl,
                    Kind::Dbl,
                ],
            ),
            INTERSECTION => common_plus(
                &common,
                vec![
                    Kind::Chr,
                    Kind::Ptr,
                    Kind::Ptr,
                    Kind::Ptr,
                    Kind::Ptr,
                    Kind::Ptr,
                ],
            ),
            PLANE => common_plus(&common, vec![Kind::Chr, Kind::Vec, Kind::Vec, Kind::Vec]),
            CYLINDER => common_plus(
                &common,
                vec![Kind::Chr, Kind::Vec, Kind::Vec, Kind::Dbl, Kind::Vec],
            ),
            CONE => common_plus(
                &common,
                vec![
                    Kind::Chr,
                    Kind::Vec,
                    Kind::Vec,
                    Kind::Dbl,
                    Kind::Dbl,
                    Kind::Dbl,
                    Kind::Vec,
                ],
            ),
            SPHERE => common_plus(
                &common,
                vec![Kind::Chr, Kind::Vec, Kind::Dbl, Kind::Vec, Kind::Vec],
            ),
            TORUS => common_plus(
                &common,
                vec![Kind::Chr, Kind::Vec, Kind::Vec, Kind::Dbl, Kind::Dbl, Kind::Vec],
            ),
            GEOMETRIC_OWNER => vec![Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr],
            CHART | LIMIT | POINTER_LIS_BLOCK => Vec::new(),
            _ => panic!("this suite reads back only the node types the writer emits, not {ty}"),
        };
        match ty {
            CHART => {
                assert!(k.is_empty());
                k.extend([
                    Kind::Dbl,
                    Kind::Dbl,
                    Kind::Int,
                    Kind::NullableDbl,
                    Kind::NullableDbl,
                    Kind::NullableDbl,
                    Kind::NullableDbl,
                ]);
                k.extend(std::iter::repeat_n(Kind::Vec, len));
            }
            LIMIT => {
                assert!(k.is_empty());
                k.push(Kind::Chr);
                k.extend(std::iter::repeat_n(Kind::Vec, len));
            }
            POINTER_LIS_BLOCK => {
                assert!(k.is_empty());
                k.extend([Kind::Int, Kind::Ptr]);
                k.extend(std::iter::repeat_n(Kind::Ptr, len));
            }
            _ => assert_eq!(len, 0, "only CHART, LIMIT and POINTER_LIS_BLOCK carry a length"),
        }
        k
    }

    /// The six fields every curve and surface node begins with, then `own`.
    fn common_plus(common: &[Kind], own: Vec<Kind>) -> Vec<Kind> {
        let mut k = common.to_vec();
        k.extend(own);
        k
    }

    /// Reads the node sequence out of `text` by the schema alone: every number
    /// ends at the single space after it, every char or null is one character
    /// with the next field hard against it, and the sequence ends at the
    /// `1 0` terminator. The stream starts at the userfield size, whose record
    /// the first node's fields share, because records are only a wrapping of
    /// the token stream.
    fn parse(text: &str) -> Vec<Node> {
        let lines: Vec<&str> = text.lines().collect();
        assert!(lines.len() > 9, "a transmit file has a header and nodes");
        assert_eq!(lines[6], "T", "the format flag sequence starts at line 7");
        let stream: String = lines[9..].concat();
        assert!(
            stream.starts_with("0 "),
            "the node stream opens with the zero userfield size"
        );
        assert!(stream.ends_with("1 0"), "the node sequence ends at the 1 0 terminator");
        let chars: Vec<char> = stream.chars().collect();
        let mut at = 0usize;
        let mut nodes = Vec::new();
        let mut where_am_i = String::from("the userfield size");
        assert_eq!(read_number(&chars, &mut at, &where_am_i) as i64, 0, "USFLD_SIZE is zero");
        loop {
            let ty = read_number(&chars, &mut at, "the node type").round() as i64;
            let mut len = None;
            if matches!(ty as u16, CHART | LIMIT | POINTER_LIS_BLOCK) {
                len = Some(read_number(&chars, &mut at, "a variable length").round() as usize);
            }
            let index = read_token(&chars, &mut at).parse::<f64>().expect(
                "a node index, or the terminator's zero, is a number",
            ) as u32;
            if ty == 1 && index == 0 {
                assert_eq!(at, chars.len(), "nothing follows the terminator");
                return nodes;
            }
            let kinds = schema(ty as u16, len.unwrap_or(0));
            where_am_i = format!("node {index} of type {ty}");
            let mut node = Node {
                ty: ty as u16,
                len,
                index,
                ints: Vec::new(),
                dbls: Vec::new(),
                ptrs: Vec::new(),
                chars: Vec::new(),
                vecs: Vec::new(),
                nulls: 0,
            };
            for kind in kinds {
                match kind {
                    Kind::Int => node.ints.push(
                        read_number(&chars, &mut at, &where_am_i).round() as i64,
                    ),
                    Kind::Dbl => node.dbls.push(read_number(&chars, &mut at, &where_am_i)),
                    Kind::Ptr => {
                        node.ptrs.push(read_number(&chars, &mut at, &where_am_i).round() as u32)
                    }
                    Kind::NullableDbl => {
                        if chars[at] == '?' {
                            at += 1;
                            node.nulls += 1;
                        } else {
                            node.dbls.push(read_number(&chars, &mut at, &where_am_i));
                        }
                    }
                    Kind::Chr => {
                        node.chars.push(chars[at]);
                        at += 1;
                    }
                    Kind::Vec => {
                        let v = [
                            read_number(&chars, &mut at, &where_am_i),
                            read_number(&chars, &mut at, &where_am_i),
                            read_number(&chars, &mut at, &where_am_i),
                        ];
                        node.vecs.push(v);
                    }
                }
            }
            nodes.push(node);
        }
    }

    /// One number and its trailing space, from `chars` at `at`.
    fn read_number(chars: &[char], at: &mut usize, place: &str) -> f64 {
        let s = read_token(chars, at);
        assert!(
            *at < chars.len(),
            "a number is followed by one space, never the end of the stream, while reading {place}"
        );
        s.parse::<f64>().unwrap_or_else(|e| {
            let around: String = chars[(*at).saturating_sub(30)..((*at) + 20).min(chars.len())]
                .iter()
                .collect();
            panic!("a non-char field must parse as a number, while reading {place} near {around:?}: {s:?}: {e}")
        })
    }

    /// One whitespace-delimited token and the space after it if there is one,
    /// leaving the cursor at the start of the next token or at the end of the
    /// stream -- which is what reading the terminator's final zero needs, since
    /// nothing follows it.
    fn read_token(chars: &[char], at: &mut usize) -> String {
        let start = *at;
        while *at < chars.len() && chars[*at] != ' ' {
            *at += 1;
        }
        let s: String = chars[start..*at].iter().collect();
        if *at < chars.len() {
            *at += 1;
        }
        s
    }

    /// A file reader's view of `text`: where each node index is and what each
    /// node is, with the chains a reader walks followed and counted.
    struct File {
        nodes: Vec<Node>,
        at: std::collections::HashMap<u32, usize>,
    }

    impl File {
        fn node(&self, i: u32) -> &Node {
            let at = *self.at.get(&i).unwrap_or_else(|| panic!("node {i} exists"));
            &self.nodes[at]
        }

        /// The chain a `next`-field at `ptrs[next]` threads, ended by null,
        /// asserting the `previous` field at `ptrs[back]` points back the other
        /// way when the chain is doubly linked -- the walk a reader performs to
        /// enumerate a shell's faces or a body's regions.
        fn walk(&self, head: u32, next: usize, back: Option<usize>, what: &str) -> Vec<u32> {
            let mut out = Vec::new();
            let mut cur = head;
            while cur != 0 {
                assert!(
                    !out.contains(&cur),
                    "the {what} chain reaches node {cur} twice, so it is a ring or a tangle"
                );
                out.push(cur);
                let node = self.node(cur);
                let next_i = node.ptrs[next];
                if next_i != 0 {
                    if let Some(back) = back {
                        assert_eq!(
                            self.node(next_i).ptrs[back],
                            cur,
                            "the {what} chain is doubly linked at node {cur}"
                        );
                    }
                }
                cur = next_i;
                assert!(out.len() <= self.nodes.len(), "the {what} chain terminates");
            }
            out
        }
    }

    /// Parses `text` and holds it against the solids it claims to carry,
    /// checking everything a reader checks before trusting the graph: indices
    /// 1..=n with no gaps or repeats, the root at 1, every pointer resolving,
    /// each body's region chain holding exactly one void region, both of each
    /// shell pair's face chains covering the solid's faces, and the topology
    /// node counts equalling the solid's own.
    fn check_round_trip(text: &str, bodies: &[&Solid]) {
        let nodes = parse(text);
        let at: std::collections::HashMap<u32, usize> = nodes
            .iter()
            .enumerate()
            .map(|(i, n)| (n.index, i))
            .collect();
        assert_eq!(at.len(), nodes.len(), "node indices are unique");
        let max_index = nodes.iter().map(|n| n.index).max().unwrap();
        assert_eq!(
            max_index as usize,
            nodes.len(),
            "node indices are 1..=n with no gaps"
        );
        for n in &nodes {
            for &p in &n.ptrs {
                assert!(
                    p == 0 || at.contains_key(&p),
                    "pointer {p} from node {} resolves to an emitted node",
                    n.index
                );
            }
        }
        let f = File { nodes, at };

        let root = f.node(1);
        assert_eq!(root.index, 1, "the node at index 1 is the root");
        let body_indices: Vec<u32> = if bodies.len() == 1 {
            assert_eq!(root.ty, BODY, "one body puts its BODY node at the root");
            vec![root.index]
        } else {
            assert_eq!(root.ty, POINTER_LIS_BLOCK, "many bodies share a list root");
            assert_eq!(root.len, Some(bodies.len()));
            assert_eq!(root.ptrs.len(), 1 + bodies.len());
            assert_eq!(root.ints[0], bodies.len() as i64);
            root.ptrs[1..].to_vec()
        };

        let mut want_faces = 0;
        let mut want_loops = 0;
        let mut want_edges = 0;
        let mut want_verts = 0;
        for solid in bodies {
            want_faces += solid.faces.len();
            want_loops += (0..solid.faces.len())
                .map(|fi| 1 + solid.n_inners(fi))
                .sum::<usize>();
            let mut used = vec![false; solid.edges.len()];
            let mut live = vec![false; solid.verts.len()];
            for fi in 0..solid.faces.len() {
                for lp in solid.face_loops(fi) {
                    for &(e, _) in lp {
                        used[e] = true;
                        live[solid.edges[e].v0] = true;
                        live[solid.edges[e].v1] = true;
                    }
                }
            }
            want_edges += used.iter().filter(|u| **u).count();
            want_verts += live.iter().filter(|l| **l).count();
        }
        let n = |t| f.nodes.iter().filter(|x| x.ty == t).count();
        assert_eq!(n(BODY), bodies.len());
        assert_eq!(n(FACE), want_faces, "one FACE per kernel face");
        assert_eq!(n(LOOP), want_loops, "one LOOP per kernel loop");
        assert_eq!(n(EDGE), want_edges, "one EDGE per face-used edge");
        assert_eq!(n(VERTEX), want_verts, "one VERTEX per edge-reached vertex");
        assert_eq!(n(POINT), want_verts, "one POINT per edge-reached vertex");
        assert_eq!(n(FIN), 2 * want_edges, "two fins per face-used edge");

        let mut faces_as_back = 0;
        let mut faces_as_front = 0;
        for &bi in &body_indices {
            let body = f.node(bi);
            assert_eq!(body.ty, BODY);
            assert_eq!(body.ints[2], 1, "body_type is solid");
            // BODY ptrs: attrs, attr_chains, surface, curve, point, key,
            // ref_instance, next, previous, owner, shell, boundary_surface,
            // boundary_curve, boundary_point, region, edge, vertex.
            let regions = f.walk(body.ptrs[14], 2, Some(3), "region");
            let voids = regions
                .iter()
                .filter(|&&r| f.node(r).chars[0] == 'V')
                .count();
            assert_eq!(voids, 1, "a solid body has exactly one infinite void region");
            let void_region = regions
                .iter()
                .find(|&&r| f.node(r).chars[0] == 'V')
                .copied()
                .unwrap();
            for &r in &regions {
                let region = f.node(r);
                assert!(region.chars[0] == 'V' || region.chars[0] == 'S');
                let solid_region = region.chars[0] == 'S';
                // SHELL ptrs: attrs, body, next, face, edge, vertex, region,
                // front_face.
                let shells = f.walk(region.ptrs[4], 2, None, "shell");
                assert!(!shells.is_empty(), "every region has a shell");
                for &s in &shells {
                    let shell = f.node(s);
                    assert_eq!(shell.ty, SHELL);
                    if solid_region {
                        assert_eq!(shell.ptrs[7], 0, "a solid region's shell has no front-faces");
                        faces_as_back +=
                            f.walk(shell.ptrs[3], 1, Some(2), "back-face").len();
                    } else {
                        assert_eq!(shell.ptrs[3], 0, "a void region's shell has no back-faces");
                        faces_as_front +=
                            f.walk(shell.ptrs[7], 8, Some(9), "front-face").len();
                    }
                }
            }
            let void_shells = f.walk(f.node(void_region).ptrs[4], 2, None, "void shell");
            for &s in &void_shells {
                assert_eq!(f.node(s).ptrs[1], 0, "a void shell in a solid body has no body");
            }
        }
        assert_eq!(
            faces_as_back, want_faces,
            "the solid shells' back-face chains cover every face exactly once"
        );
        assert_eq!(
            faces_as_front, want_faces,
            "the void shells' front-face chains cover every face exactly once"
        );

        // FIN ptrs: attrs, loop, forward, backward, vertex, other, edge, curve,
        // next_at_vx. EDGE ptrs: attrs, fin, previous, next, curve,
        // next_on_curve, previous_on_curve, owner.
        for node in &f.nodes {
            if node.ty != EDGE {
                continue;
            }
            let fin = node.ptrs[1];
            let other = f.node(fin).ptrs[5];
            assert_ne!(fin, other, "an edge's two fins are distinct nodes");
            assert_eq!(
                f.node(other).ptrs[6],
                node.index,
                "a fin's other fin belongs to the same edge"
            );
            assert_ne!(
                f.node(fin).chars[0],
                f.node(other).chars[0],
                "an edge's positive and negative fin carry opposite senses"
            );
            assert_eq!(
                f.node(fin).ptrs[5],
                other,
                "the other relation is symmetric"
            );
        }
    }

    fn cube() -> Solid {
        extrude(&Sketch::rounded_rect(0.0, 0.0, 10.0, 10.0, 0.0), 0.0, 5.0)
    }

    /// An L-shaped bin split in two, whose carve planes cross the reentrant
    /// corner's fillet torus -- the one shape whose pieces carry
    /// `Curve::TorusSection` edges, and therefore the one that exercises the
    /// INTERSECTION path of the writer.
    fn split_l_bin() -> Vec<Solid> {
        use crate::layout::{Axis, GridCell, SplitLine};
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
        p.bins[0].split_lines = vec![SplitLine {
            axis: Axis::X,
            index: 1,
        }];
        let whole = gridfinity::build_bin_solid(&p, &cells, None)
            .expect("the L-shaped bin builds");
        let parts = crate::layout::partition_cells(&cells, &p.bins[0].split_lines);
        assert_eq!(parts.len(), 2, "the split line cuts the L in two");
        parts
            .iter()
            .map(|part| {
                gridfinity::carve_to_cells(&whole, &cells, &part.cells)
                    .expect("each piece carves")
            })
            .collect()
    }

    #[test]
    fn a_cube_is_one_body_whose_graph_reads_back() {
        let cube = cube();
        let text = to_xt_text(&[&cube]).expect("a cube is stateable in the format");
        check_round_trip(&text, &[&cube]);
        assert!(
            !f_has(&text, INTERSECTION),
            "a cube has no intersection curves"
        );
    }

    #[test]
    fn two_bodies_root_in_a_pointer_list_block() {
        let cube = cube();
        let bin = gridfinity::build(&gridfinity::Params::rect(1, 1));
        let text = to_xt_text(&[&cube, &bin]).expect("both bodies are stateable");
        check_round_trip(&text, &[&cube, &bin]);
    }

    fn f_has(text: &str, ty: u16) -> bool {
        parse(text).iter().any(|n| n.ty == ty)
    }

    /// The node of index `i`, for tests that walk the file directly.
    fn f_of(nodes: &[Node], i: u32) -> &Node {
        nodes
            .iter()
            .find(|n| n.index == i)
            .unwrap_or_else(|| panic!("node {i} was emitted, so the file holds it"))
    }

    #[test]
    fn a_split_l_shaped_bin_writes_its_torus_sections_as_exact_intersections() {
        let pieces = split_l_bin();
        let sections: usize = pieces
            .iter()
            .map(|s| {
                s.edges
                    .iter()
                    .filter(|e| {
                        matches!(
                            e.curve,
                            crate::kernel::geom::Curve::TorusSection { .. }
                        )
                    })
                    .count()
            })
            .sum();
        assert_eq!(sections, 2, "the L's carve planes cross the corner fillet twice");
        let bodies: Vec<&Solid> = pieces.iter().collect();
        let text = to_xt_text(&bodies).expect("the split L is stateable in the format");
        check_round_trip(&text, &bodies);

        let nodes = parse(&text);
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
            assert!(chart.len.unwrap() >= 2, "a chart spans its curve");
            assert_eq!(chart.vecs.len(), chart.len.unwrap());
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
        let bin = gridfinity::build(&gridfinity::Params::rect(1, 1));
        let text = to_xt_text(&[&bin]).expect("a default bin is stateable in the format");
        check_round_trip(&text, &[&bin]);
        let nodes = parse(&text);
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
    /// This is the f32-versus-res_linear number: it says how far the
    /// millimetres the kernel computes can sit from the analytic forms a file
    /// declaring 1e-8 m of linear resolution implies.
    fn deviation(solid: &Solid) -> (f32, f32) {
        let mut max_surface = 0.0f32;
        let mut max_curve = 0.0f32;
        let used = used_edges(solid);
        let ef = solid.edge_faces();
        let mut surfaces: Vec<surf::XtSurface> = Vec::with_capacity(solid.faces.len());
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
            let xt = surf::of_surface(&face.surface, &samples)
                .expect("the writer translated this face, so the test can");
            for &p in &samples {
                max_surface = max_surface.max(xt.distance(p).abs());
            }
            surfaces.push(xt);
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
                            max_curve = max_curve.max(surfaces[fi].distance(p).abs());
                        }
                    }
                }
            }
        }
        (max_surface, max_curve)
    }
}
