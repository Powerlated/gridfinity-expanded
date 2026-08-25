//! The transmit-file validator: everything a reader checks before trusting a
//! file, as findings rather than panics.
//!
//! `validate_xt` takes the text of a `.x_t` file and returns every way it
//! fails to be a transmit file of this schema: lexical rules the manual
//! states, the header's own cross-references, index and pointer resolution,
//! the class of every pointer field, the chains a reader walks (regions,
//! shells, faces, loops, fins, boundary geometry) including their coverage,
//! and the geometry itself -- directions unit, radii positive, chart points
//! on both surfaces they are the intersection of, limits at the chart's ends,
//! every edge's end vertices on its curve, every face's loop vertices on its
//! surface. A clean result is an empty vec.
//!
//! Every geometric predicate here is recomputed from the parsed node's own
//! fields in the file's units (metres), not from anything the writer remembers
//! -- the point of a validator is that writer and specification agree, so the
//! two are stated independently and a defect in either shows up as a finding.
//! Findings name the node they are about; the first parse failure is fatal and
//! returned as the only finding, because a mis-parsed stream has no node graph
//! to check.

use super::reader::{self, Node};
use super::text;
use std::collections::HashMap;

/// How far a point the file claims is on a surface, curve or chart may stand
/// off it, in metres.
///
/// The file's own declared `res_linear`, because that is the number the reader
/// holds it to -- a tolerance the validator picks for itself can only be a
/// number the reader has never heard of. It is left at `res_linear` and not
/// tightened to the kernel's own precision for exactly that reason: what it
/// asks is whether a *reader* would accept the file. The measured deviation was
/// ~3e-9 m when the kernel modelled in `f32`, clearing it by three times; in
/// `f64` the margin is orders wider still, and a battery entry that fails is a
/// bin a Parasolid frustrum would also reject.
const ON_GEOMETRY_M: f64 = 1.0e-8;

/// How far a direction field may be from unit length.
///
/// Four orders tighter than `ON_GEOMETRY_M`, because the writer normalises in
/// f64 and the emitted decimals round-trip exactly, so nothing here has any f64
/// residue left in it. Loosening this to f64's own 6e-8 is what let a chamfer's
/// tilted normals ship six times the file's declared resolution out of unit --
/// invisible to every test here and a fault in the first CAD system to read it.
const UNIT_TOL: f64 = 1.0e-12;

/// How far an x_axis may lean out of its surface's plane. Tight for the same
/// reason as `UNIT_TOL`: one f64 Gram-Schmidt step against the axis leaves no
/// more than a few ulps.
const ORTHO_TOL: f64 = 1.0e-12;

/// How far a cone's half-angle sine and cosine may sit from one angle's. The
/// pair is `f64::sin_cos` of one angle, so the same few ulps as `UNIT_TOL`.
const CONE_IDENTITY_TOL: f64 = 1.0e-12;

/// The least magnitude an intersection curve's natural tangent may have at a
/// chart point before the two surfaces count as tangent there.
const TANGENT_CROSS_MIN: f64 = 1.0e-3;

/// One way a file is not a transmit file: the node the finding is about (0 for
/// the file as a whole) and a sentence stating the violated property.
pub struct Finding {
    pub node: u32,
    pub message: String,
}

/// Everything wrong with `text` as a Parasolid XT transmit file of schema
/// SCH_1200000_12006, in the order the checks ran. An empty result means the
/// file parses, its graph resolves and chains close, and its geometry says
/// what its topology claims.
pub fn validate_xt(text: &str) -> Vec<Finding> {
    let mut out = Vec::new();
    lexical(text, &mut out);
    header(text, &mut out);
    let parsed = match reader::parse(text) {
        Ok(p) => p,
        Err(e) => {
            out.push(Finding {
                node: 0,
                message: format!("the node stream does not parse: {e}"),
            });
            return out;
        }
    };
    let g = Graph::of(parsed);
    indices_and_root(&g, &mut out);
    if out.is_empty() {
        classes(&g, &mut out);
        edges_and_fins(&g, &mut out);
        loops_and_faces(&g, &mut out);
        bodies(&g, &mut out);
        geometry_forms(&g, &mut out);
        geometry_membership(&g, &mut out);
        intersections(&g, &mut out);
    }
    assert!(
        out.iter().all(|f| f.node == 0 || g.get(f.node).is_some()),
        "every finding names an emitted node, or 0 for the file as a whole -- a finding about a          node index nobody can look up says nothing"
    );
    out
}

/// The record rules the manual states for text files: every line is ASCII,
/// and no line holds two adjacent spaces away from its end -- trailing spaces
/// are explicitly ignored on read, so only the interior violation is a
/// defect.
fn lexical(text: &str, out: &mut Vec<Finding>) {
    for (li, line) in text.lines().enumerate() {
        if !line.is_ascii() {
            out.push(Finding {
                node: 0,
                message: format!("line {} holds non-ASCII bytes, which a text file cannot", li + 1),
            });
        }
        let trimmed = line.trim_end();
        if trimmed.contains("  ") {
            out.push(Finding {
                node: 0,
                message: format!(
                    "line {} contains adjacent spaces away from its end, which the format \
                     forbids",
                    li + 1
                ),
            });
        }
    }
}

/// The header's own cross-references: a PART1 naming text and transmit, a
/// PART2 naming the same schema as the flag sequence's schema line, a zero
/// userfield size there, and a version line of the prescribed shape.
fn header(text: &str, out: &mut Vec<Finding>) {
    let lines: Vec<&str> = text.lines().collect();
    let t_line = lines.iter().position(|l| *l == "T").unwrap_or(0);
    let header = &lines[..t_line.min(lines.len())];
    let wants = |needle: &str| header.iter().any(|l| l.contains(needle));
    if !wants("FORMAT=text") {
        out.push(finding(0, "the header's PART1 line declares FORMAT=text"));
    }
    if !wants("GUISE=transmit") {
        out.push(finding(0, "the header's PART1 line declares GUISE=transmit"));
    }
    if !wants("USFLD_SIZE=0") {
        out.push(finding(0, "the header's PART2 line declares USFLD_SIZE=0"));
    }
    let part2_schema = header
        .iter()
        .find_map(|l| l.split("SCH=").nth(1).map(|s| s.split(';').next().unwrap_or("").to_string()));
    let schema_line = lines.get(t_line + 2).map(|l| *l).unwrap_or("");
    let flag_schema = schema_line.split_once(' ').map(|(_, s)| s.trim());
    match (part2_schema, flag_schema) {
        (Some(a), Some(b)) if !a.is_empty() => {
            if a != b {
                out.push(finding(
                    0,
                    format!("the header's schema {a} is the flag sequence's schema {}", b),
                ));
            }
        }
        _ => out.push(finding(0, "the flag sequence carries a schema name the header repeats")),
    }
    let version = lines.get(t_line + 1).map(|l| *l).unwrap_or("");
    if !version.contains(": TRANSMIT FILE") || !version.contains("modeller version") {
        out.push(finding(
            0,
            "the flag sequence's first line is `<length> : TRANSMIT FILE created by modeller \
             version <n>`"
                .to_string(),
        ));
    }
}

/// A parsed file with node lookup, which every semantic check walks.
struct Graph {
    nodes: Vec<Node>,
    at: HashMap<u32, usize>,
}

impl Graph {
    /// Lookup over the parsed nodes, first occurrence winning.
    fn of(parsed: reader::Parsed) -> Graph {
        let at = parsed
            .nodes
            .iter()
            .enumerate()
            .map(|(i, n)| (n.index, i))
            .collect();
        Graph { nodes: parsed.nodes, at }
    }

    fn get(&self, index: u32) -> Option<&Node> {
        self.at.get(&index).map(|&i| &self.nodes[i])
    }

    fn ty(&self, index: u32) -> Option<u16> {
        self.get(index).map(|n| n.ty)
    }

    fn of_type(&self, ty: u16) -> Vec<u32> {
        self.nodes.iter().filter(|n| n.ty == ty).map(|n| n.index).collect()
    }

    /// Walks the chain starting at `head` through pointer slot `next`,
    /// checking the reverse pointer slot `back` when the chain is doubly
    /// linked, stopping at null or a dangling pointer (which the resolution
    /// pass has already named) and reporting a chain that revisits a node.
    fn chain(
        &self,
        head: u32,
        next: usize,
        back: Option<usize>,
        what: &str,
        out: &mut Vec<Finding>,
    ) -> Vec<u32> {
        let mut seen: Vec<u32> = Vec::new();
        let mut cur = head;
        while cur != 0 {
            if seen.contains(&cur) {
                out.push(finding(
                    cur,
                    format!("the {what} chain revisits node {cur}, so it is a ring or a tangle"),
                ));
                return seen;
            }
            seen.push(cur);
            let Some(node) = self.get(cur) else { return seen };
            let next_i = node.ptrs[next];
            if let (Some(back), Some(next_node)) = (back, self.get(next_i).filter(|_| next_i != 0)) {
                if next_node.ptrs[back] != cur {
                    out.push(finding(
                        cur,
                        format!("the {what} chain is doubly linked, but node {next_i}'s back \
                                 pointer is {}",
                                next_node.ptrs[back]),
                    ));
                    return seen;
                }
            }
            cur = next_i;
        }
        seen
    }

    /// Walks the ring starting at `head` through pointer slot `next`, which
    /// closes back onto `head` rather than ending at null -- a loop's fins and
    /// an edge's fins are rings, not chains. Reports a ring that runs into a
    /// null, revisits a node other than its head, or whose `back` pointers do
    /// not mirror it.
    fn ring(
        &self,
        head: u32,
        next: usize,
        back: Option<usize>,
        what: &str,
        out: &mut Vec<Finding>,
    ) -> Vec<u32> {
        let mut seen: Vec<u32> = Vec::new();
        let mut cur = head;
        loop {
            if cur == 0 {
                out.push(finding(
                    head,
                    format!("the {what} ring closes back onto its head, but runs into a null"),
                ));
                return seen;
            }
            if seen.contains(&cur) {
                if cur != head {
                    out.push(finding(
                        cur,
                        format!("the {what} ring rejoins itself at node {cur}, not at its head"),
                    ));
                }
                return seen;
            }
            seen.push(cur);
            let Some(node) = self.get(cur) else { return seen };
            let next_i = node.ptrs[next];
            if let (Some(back), Some(next_node)) = (back, self.get(next_i).filter(|_| next_i != 0)) {
                if next_node.ptrs[back] != cur {
                    out.push(finding(
                        cur,
                        format!(
                            "the {what} ring is doubly linked, but node {next_i}'s back pointer                              is {}",
                            next_node.ptrs[back]
                        ),
                    ));
                    return seen;
                }
            }
            cur = next_i;
        }
    }
}

fn finding(node: u32, message: impl Into<String>) -> Finding {
    Finding { node, message: message.into() }
}

/// Index completeness and the root node's shape: indices are 1..=n with no
/// gaps or repeats, every pointer resolves, and node 1 is a BODY for a
/// single-body file or a POINTER_LIS_BLOCK listing the bodies otherwise.
fn indices_and_root(g: &Graph, out: &mut Vec<Finding>) {
    if g.nodes.is_empty() {
        out.push(finding(0, "the file holds at least one node, the root"));
        return;
    }
    for w in 1..g.at.len() {
        if !g.at.contains_key(&(w as u32)) {
            out.push(finding(0, format!("index {w} is missing from the 1..=n index range")));
        }
    }
    for node in &g.nodes {
        for &p in &node.ptrs {
            if p != 0 && !g.at.contains_key(&p) {
                out.push(finding(
                    node.index,
                    format!("pointer {p} resolves to an emitted node"),
                ));
            }
        }
    }
    if g.nodes.len() != g.at.len() {
        out.push(finding(
            0,
            "every node index appears exactly once, but a repeat was found",
        ));
    }
    let max_index = g.nodes.iter().map(|n| n.index).max().unwrap_or(0);
    for w in 1..=max_index {
        if !g.at.contains_key(&w) {
            out.push(finding(0, format!("index {w} is missing from the 1..=n index range")));
        }
    }
    let Some(root) = g.get(1) else {
        out.push(finding(0, "the node at index 1 is the root"));
        return;
    };
    match root.ty {
        text::BODY => {
            if g.of_type(text::BODY).len() != 1 {
                out.push(finding(1, "a file of several bodies roots in a POINTER_LIS_BLOCK"));
            }
        }
        text::POINTER_LIS_BLOCK => {
            let entries = &root.ptrs[1..];
            if root.ints[0] != entries.len() as i64 {
                out.push(finding(1, "the root list's entry count is its own length"));
            }
            for &e in entries {
                if g.ty(e) != Some(text::BODY) {
                    out.push(finding(e, "a root list entry is a BODY"));
                }
            }
            if g.of_type(text::BODY).len() != entries.len() {
                out.push(finding(
                    1,
                    "the root list holds every body the file contains, no more and no fewer",
                ));
            }
        }
        other => out.push(finding(
            1,
            format!("the root node is a BODY or a POINTER_LIS_BLOCK, not type {other}"),
        )),
    }
}

/// A surface node of any of the five analytic kinds the kernel writes.
const SURFACES: &[u16] = &[
    text::PLANE,
    text::CYLINDER,
    text::CONE,
    text::SPHERE,
    text::TORUS,
];

/// A curve node of any of the kinds the kernel writes, the intersection
/// included.
const CURVES: &[u16] = &[text::LINE, text::CIRCLE, text::ELLIPSE, text::INTERSECTION];

/// Any geometry node a GEOMETRIC_OWNER ring can be about.
const GEOMETRY: &[u16] = &[
    text::LINE,
    text::CIRCLE,
    text::ELLIPSE,
    text::INTERSECTION,
    text::PLANE,
    text::CYLINDER,
    text::CONE,
    text::SPHERE,
    text::TORUS,
];

/// The class of every pointer field, and whether the schema lets it be null:
/// a fin's vertex is a VERTEX, a face's surface is a surface node, and a
/// field the schema types `pointer` rather than `pointer0` is additionally
/// required to be set.
fn classes(g: &Graph, out: &mut Vec<Finding>) {
    const REQ: bool = true;
    const OPT: bool = false;
    for node in &g.nodes {
        let demands: Vec<(usize, &[u16], &str, bool)> = match node.ty {
            text::REGION => vec![
                (1, &[text::BODY], "body", REQ),
                (4, &[text::SHELL], "shell", OPT),
            ],
            text::SHELL => vec![
                (3, &[text::FACE], "face", OPT),
                (7, &[text::FACE], "front_face", OPT),
                (6, &[text::REGION], "region", REQ),
            ],
            text::FACE => vec![
                (3, &[text::LOOP], "loop", OPT),
                (4, &[text::SHELL], "shell", REQ),
                (10, &[text::SHELL], "front_shell", REQ),
                (5, SURFACES, "surface", OPT),
            ],
            text::LOOP => vec![
                (1, &[text::FIN], "fin", REQ),
                (2, &[text::FACE], "face", REQ),
            ],
            text::EDGE => vec![
                (1, &[text::FIN], "fin", REQ),
                (4, CURVES, "curve", OPT),
                (7, &[text::BODY], "owner", REQ),
            ],
            text::FIN => vec![
                (1, &[text::LOOP], "loop", OPT),
                (2, &[text::FIN], "forward", OPT),
                (3, &[text::FIN], "backward", OPT),
                (4, &[text::VERTEX], "vertex", OPT),
                (5, &[text::FIN], "other", OPT),
                (6, &[text::EDGE], "edge", OPT),
            ],
            text::VERTEX => vec![
                (1, &[text::FIN], "fin", OPT),
                (4, &[text::POINT], "point", REQ),
            ],
            text::POINT => vec![(1, &[text::VERTEX], "owner", OPT)],
            text::INTERSECTION => vec![
                (7, &[text::CHART], "chart", REQ),
                (8, &[text::LIMIT], "start", REQ),
                (9, &[text::LIMIT], "end", REQ),
                (5, SURFACES, "first surface", REQ),
                (6, SURFACES, "second surface", REQ),
            ],
            text::GEOMETRIC_OWNER => vec![(3, GEOMETRY, "shared geometry", REQ)],
            _ => continue,
        };
        for (slot, allowed, what, required) in demands {
            let target = node.ptrs[slot];
            if target == 0 {
                if required {
                    out.push(finding(
                        node.index,
                        format!("its {what} pointer is never null"),
                    ));
                }
                continue;
            }
            let actual = g.ty(target);
            if !actual.is_some_and(|t| allowed.contains(&t)) {
                out.push(finding(
                    node.index,
                    format!("its {what} pointer, {target}, is one of the right node types"),
                ));
            }
        }
    }
}

/// Every edge's pair of fins: the edge names the positive one, its `other` is
/// the negative one, the relation is symmetric, both belong to the same edge,
/// and each fin's loop membership is consistent.
fn edges_and_fins(g: &Graph, out: &mut Vec<Finding>) {
    for edge in g.of_type(text::EDGE) {
        let node = g.get(edge).expect("of_type yields emitted nodes");
        let fin = node.ptrs[1];
        let other = g.get(fin).map(|f| f.ptrs[5]).unwrap_or(0);
        let fin_sense = g.get(fin).map(|f| f.chars[0]);
        let other_sense = g.get(other).map(|f| f.chars[0]);
        if fin == 0 || other == 0 || fin == other {
            out.push(finding(edge, "an edge names two distinct fins, itself the positive one"));
            continue;
        }
        if fin_sense != Some('+') || other_sense != Some('-') {
            out.push(finding(
                edge,
                "its two fins carry opposite senses, the named one positive",
            ));
        }
        if g.get(other).map(|f| f.ptrs[5]) != Some(fin) {
            out.push(finding(edge, "the fin pairing is symmetric"));
        }
        for f in [fin, other] {
            if g.get(f).map(|f| f.ptrs[6]) != Some(edge) {
                out.push(finding(f, "a fin's edge pointer names the edge that owns it"));
            }
            let Some(fin_node) = g.get(f) else { continue };
            let (fwd, bwd) = (fin_node.ptrs[2], fin_node.ptrs[3]);
            if fwd != 0 || bwd != 0 {
                if g.get(fwd).map(|n| n.ptrs[3]) != Some(f)
                    || g.get(bwd).map(|n| n.ptrs[2]) != Some(f)
                {
                    out.push(finding(f, "a fin's forward and backward links are mutual"));
                }
            }
        }
    }
}

/// Every face's loops: the fin ring closes nose-to-tail, every fin in it
/// names the same loop, and the loop's face names the face that reached it.
fn loops_and_faces(g: &Graph, out: &mut Vec<Finding>) {
    for face in g.of_type(text::FACE) {
        let node = g.get(face).expect("of_type yields emitted nodes");
        if node.ptrs[3] == 0 {
            out.push(finding(face, "a face has at least one loop"));
            continue;
        }
        let loops = g.chain(node.ptrs[3], 3, None, "loop", out);
        if loops.len() != g.of_type(text::LOOP).iter().filter(|l| g.get(**l).map(|n| n.ptrs[2]) == Some(face)).count() {
            out.push(finding(face, "the face's loop chain holds every loop naming it"));
        }
        for &loop_i in &loops {
            let lp = g.get(loop_i).expect("the chain resolved");
            if lp.ptrs[2] != face {
                out.push(finding(loop_i, "a loop's face pointer names the face that reached it"));
            }
            let ring = g.ring(lp.ptrs[1], 2, Some(3), "fin", out);
            if ring.is_empty() || ring.first() != Some(&lp.ptrs[1]) {
                out.push(finding(loop_i, "a loop names one of its own fins"));
                continue;
            }
            for &f in &ring {
                if g.get(f).map(|n| n.ptrs[1]) != Some(loop_i) {
                    out.push(finding(f, "a fin in a loop's ring names that loop"));
                }
                let end = g.get(f).and_then(|n| g.get(n.ptrs[4])).map(|v| v.index);
                let next_start = g
                    .get(f)
                    .and_then(|n| g.get(n.ptrs[2]))
                    .and_then(|n| g.get(n.ptrs[5]))
                    .and_then(|n| g.get(n.ptrs[4]))
                    .map(|v| v.index);
                if end.is_some() && end != next_start {
                    out.push(finding(
                        f,
                        "each fin ends at the vertex the next fin in the ring starts from",
                    ));
                }
            }
        }
    }
    for vertex in g.of_type(text::VERTEX) {
        let node = g.get(vertex).expect("of_type yields emitted nodes");
        let chain = g.chain(node.ptrs[1], 8, None, "vertex fin", out);
        let claiming: Vec<u32> = g
            .of_type(text::FIN)
            .into_iter()
            .filter(|f| g.get(*f).map(|n| n.ptrs[4]) == Some(vertex))
            .collect();
        if !same_set(&chain, &claiming) {
            out.push(finding(
                vertex,
                "the vertex's fin chain holds exactly the fins ending at it",
            ));
        }
    }
}

/// The bodies the root names: itself where a single BODY roots the file, its
/// entries where a POINTER_LIS_BLOCK does, and none where the root resolves to
/// neither -- which `indices_and_root` has already reported.
fn bodies_of(g: &Graph) -> Vec<u32> {
    match g.ty(1) {
        Some(text::BODY) => vec![1],
        Some(_) => g.get(1).map(|r| r.ptrs[1..].to_vec()).unwrap_or_default(),
        None => Vec::new(),
    }
}

/// Per body: one infinite void region and at least one solid region, the
/// region chain covering the body's regions, each region's shells covering
/// its shells, the solid shells' back-face chains and the void shells'
/// front-face chains each covering every face of the body, and the boundary
/// geometry chains covering the surfaces, curves, edges, points and vertices
/// the body owns.
fn bodies(g: &Graph, out: &mut Vec<Finding>) {
    let bodies = bodies_of(g);
    if bodies.is_empty() {
        return;
    }
    for &body in &bodies {
        let node = g.get(body).expect("the root named emitted bodies");
        if node.ty != text::BODY {
            continue;
        }
        if node.ints[2] != 1 {
            out.push(finding(
                body,
                format!("a transmitted body is a solid, body_type 1, not {}", node.ints[2]),
            ));
        }
        let regions = g.chain(node.ptrs[14], 2, Some(3), "region", out);
        let mine: Vec<u32> = g
            .of_type(text::REGION)
            .into_iter()
            .filter(|r| g.get(*r).map(|n| n.ptrs[1]) == Some(body))
            .collect();
        if !same_set(&regions, &mine) {
            out.push(finding(body, "the region chain holds every region naming this body"));
        }
        let voids = mine
            .iter()
            .filter(|&&r| g.get(r).map(|n| n.chars[0]) == Some('V'))
            .count();
        if voids != 1 {
            out.push(finding(body, "a solid body has exactly one infinite void region"));
        }
        let mut faces_as_back = Vec::new();
        let mut faces_as_front = Vec::new();
        for &region in &mine {
            let rn = g.get(region).expect("of_type yields emitted nodes");
            let solid = rn.chars[0] == 'S';
            if !solid && rn.chars[0] != 'V' {
                out.push(finding(region, "a region is solid ('S') or void ('V')"));
            }
            let shells = g.chain(rn.ptrs[4], 2, None, "shell", out);
            let mine_shells: Vec<u32> = g
                .of_type(text::SHELL)
                .into_iter()
                .filter(|s| g.get(*s).map(|n| n.ptrs[6]) == Some(region))
                .collect();
            if !same_set(&shells, &mine_shells) {
                out.push(finding(region, "the region's shell chain holds its shells"));
            }
            for &shell in &mine_shells {
                let sn = g.get(shell).expect("of_type yields emitted nodes");
                if solid && sn.ptrs[7] != 0 {
                    out.push(finding(shell, "a solid region's shell has no front-faces"));
                }
                if !solid && sn.ptrs[3] != 0 {
                    out.push(finding(shell, "a void region's shell has no back-faces"));
                }
                if solid {
                    faces_as_back.extend(g.chain(sn.ptrs[3], 1, Some(2), "back-face", out));
                } else {
                    faces_as_front.extend(g.chain(sn.ptrs[7], 8, Some(9), "front-face", out));
                }
            }
        }
        let faces_of_body: Vec<u32> = g
            .of_type(text::FACE)
            .into_iter()
            .filter(|f| {
                g.get(*f)
                    .and_then(|n| g.get(n.ptrs[4]))
                    .and_then(|s| g.get(s.ptrs[1]))
                    .map(|b| b.index == body)
                    .unwrap_or(false)
            })
            .collect();
        if !same_set(&faces_as_back, &faces_of_body) {
            out.push(finding(
                body,
                "the solid shells' back-face chains cover the body's faces exactly",
            ));
        }
        if !same_set(&faces_as_front, &faces_of_body) {
            out.push(finding(
                body,
                "the void shells' front-face chains cover the body's faces exactly",
            ));
        }
        boundary_chain(g, body, 15, 3, 2, |n| n.ty == text::EDGE && n.ptrs[7] == body, "edge", out);
        boundary_chain(g, body, 16, 3, 2, |n| n.ty == text::VERTEX && n.ptrs[5] == body, "vertex", out);
        boundary_chain(g, body, 11, 2, 3, owned_by(g, body, SURFACES, text::FACE), "boundary surface", out);
        boundary_chain(g, body, 12, 2, 3, owned_by(g, body, CURVES, text::EDGE), "boundary curve", out);
        boundary_chain(g, body, 13, 2, 3, owned_by(g, body, &[text::POINT], text::VERTEX), "boundary point", out);
    }
}

/// Whether two index lists hold the same set.
fn same_set(a: &[u32], b: &[u32]) -> bool {
    let mut a = a.to_vec();
    let mut b = b.to_vec();
    a.sort_unstable();
    b.sort_unstable();
    a.dedup();
    b.dedup();
    a == b
}

/// The set of geometry nodes of one of `kinds`, owned by a topology node of
/// kind `owner_kind` that belongs to `body`, as a predicate for
/// `boundary_chain`. The kind test comes first because a node type is what
/// says whether slot 1 is an owner at all -- a FACE's slot 1 is its next
/// face, and a CHART has no pointers whatever.
fn owned_by<'a>(
    g: &'a Graph,
    body: u32,
    kinds: &'static [u16],
    owner_kind: u16,
) -> impl Fn(&Node) -> bool + 'a {
    move |n| {
        if !kinds.contains(&n.ty) {
            return false;
        }
        let Some(&owner) = n.ptrs.get(1).filter(|&&o| o != 0) else {
            return false;
        };
        g.ty(owner) == Some(owner_kind)
            && g
                .get(owner)
                .and_then(|owner| match owner_kind {
                    k if k == text::FACE => g.get(owner.ptrs[4]).and_then(|s| g.get(s.ptrs[1])),
                    k if k == text::EDGE => g.get(owner.ptrs[7]),
                    _ => g.get(owner.ptrs[5]),
                })
                .map(|b| b.index == body)
                .unwrap_or(false)
    }
}

/// Checks that the chain rooted at `chain_slot` of `body` covers exactly the
/// nodes `is_mine` accepts.
fn boundary_chain(
    g: &Graph,
    body: u32,
    chain_slot: usize,
    next: usize,
    back: usize,
    is_mine: impl Fn(&Node) -> bool,
    what: &str,
    out: &mut Vec<Finding>,
) {
    let head = g.get(body).map(|n| n.ptrs[chain_slot]).unwrap_or(0);
    let walked = g.chain(head, next, Some(back), what, out);
    let mine: Vec<u32> = g.nodes.iter().filter(|n| is_mine(n)).map(|n| n.index).collect();
    if !same_set(&walked, &mine) {
        out.push(finding(
            body,
            format!("the body's {what} chain holds exactly its own, no more and no fewer"),
        ));
    }
}

/// The numeric form of every geometry node: directions unit, x axes
/// orthogonal, radii positive, a cone's half angle one angle, everything
/// finite and inside the size box the body declares.
fn geometry_forms(g: &Graph, out: &mut Vec<Finding>) {
    let size_box = g
        .of_type(text::BODY)
        .iter()
        .filter_map(|b| g.get(*b).map(|n| n.dbls[0]))
        .fold(0.0f64, f64::max)
        .max(1.0);
    for node in &g.nodes {
        for v in &node.vecs {
            for &c in v {
                if !c.is_finite() || c.abs() > size_box {
                    out.push(finding(
                        node.index,
                        format!("every coordinate is finite and inside the {size_box} size box"),
                    ));
                }
            }
        }
        let mut unit = |slot: usize, what: &str| {
            let Some(&v) = node.vecs.get(slot) else { return };
            let len = length(v);
            if (len - 1.0).abs() > UNIT_TOL {
                out.push(finding(
                    node.index,
                    format!("its {what} must be a unit vector, but has length {len}"),
                ));
            }
        };
        match node.ty {
            text::LINE => unit(1, "direction"),
            text::CIRCLE | text::ELLIPSE => {
                unit(1, "normal");
                unit(2, "x_axis");
            }
            text::PLANE => {
                unit(1, "normal");
                unit(2, "x_axis");
            }
            text::CYLINDER | text::SPHERE => {
                unit(1, "axis");
                unit(2, "x_axis");
            }
            text::CONE => {
                unit(1, "axis");
                unit(2, "x_axis");
                let (s, c) = (node.dbls[1], node.dbls[2]);
                if (s * s + c * c - 1.0).abs() > CONE_IDENTITY_TOL || s <= 0.0 || c <= 0.0 {
                    out.push(finding(
                        node.index,
                        "a cone's sine and cosine are one angle's, strictly inside the first \
                         quadrant",
                    ));
                }
            }
            text::TORUS => {
                unit(1, "axis");
                unit(2, "x_axis");
                if node.dbls[1] <= 0.0 || node.dbls[0].abs() <= 0.0 {
                    out.push(finding(
                        node.index,
                        "a torus carries a positive minor radius and a non-zero major radius",
                    ));
                }
            }
            _ => continue,
        }
        let axis_slot = match node.ty {
            t if t == text::CYLINDER || t == text::CONE || t == text::SPHERE || t == text::TORUS => {
                Some(1)
            }
            t if t == text::CIRCLE || t == text::ELLIPSE || t == text::PLANE => Some(1),
            _ => None,
        };
        if let Some(axis) = axis_slot {
            let dot = dot(node.vecs[axis], node.vecs[2]);
            if dot.abs() > ORTHO_TOL {
                out.push(finding(
                    node.index,
                    format!("its x_axis lies in the surface's plane, but meets it at {dot}"),
                ));
            }
        }
        match node.ty {
            text::CIRCLE | text::CYLINDER | text::SPHERE if node.dbls[0] <= 0.0 => {
                out.push(finding(node.index, "a radius is positive"));
            }
            text::ELLIPSE if node.dbls[0] < node.dbls[1] || node.dbls[1] <= 0.0 => {
                out.push(finding(node.index, "an ellipse's major radius bounds its minor, both positive"));
            }
            _ => {}
        }
    }
}

/// Membership geometry: each edge's end vertices lie on its curve, and each
/// face's loop vertices lie on its surface -- the file's own claim that its
/// topology sits on its geometry.
fn geometry_membership(g: &Graph, out: &mut Vec<Finding>) {
    for edge in g.of_type(text::EDGE) {
        let node = g.get(edge).expect("of_type yields emitted nodes");
        let Some(curve) = g.get(node.ptrs[4]) else { continue };
        let ends = [node.ptrs[1]]
            .into_iter()
            .chain(g.get(node.ptrs[1]).map(|f| vec![f.ptrs[5]]).unwrap_or_default())
            .filter_map(|f| g.get(f))
            .map(|f| f.ptrs[4])
            .filter_map(|v| g.get(v))
            .map(|v| v.ptrs[4])
            .filter_map(|p| g.get(p))
            .map(|p| p.vecs[0]);
        for point in ends {
            if let Some(d) = curve_distance(curve, point) {
                if d > ON_GEOMETRY_M {
                    out.push(finding(
                        edge,
                        format!("its end vertex stands {d:.3e} m off its curve"),
                    ));
                }
            }
        }
    }
    for face in g.of_type(text::FACE) {
        let node = g.get(face).expect("of_type yields emitted nodes");
        let Some(surface) = g.get(node.ptrs[5]) else { continue };
        for loop_i in g.chain(node.ptrs[3], 3, None, "loop", out) {
            let Some(lp) = g.get(loop_i) else { continue };
            for fin in g.ring(lp.ptrs[1], 2, Some(3), "fin", out) {
                let Some(point) = g
                    .get(fin)
                    .and_then(|f| g.get(f.ptrs[4]))
                    .and_then(|v| g.get(v.ptrs[4]))
                    .map(|p| p.vecs[0])
                else {
                    continue;
                };
                if let Some(d) = surface_distance(surface, point) {
                    if d.abs() > ON_GEOMETRY_M {
                        out.push(finding(
                            face,
                            format!("a loop vertex stands {d:.3e} m off the face's surface"),
                        ));
                    }
                }
            }
        }
    }
}

/// Every intersection curve's chart: its count field matches its points, its
/// points lie on both surfaces, it runs along the surfaces' sensed normal
/// cross product, and its limits sit at the chart's two ends.
fn intersections(g: &Graph, out: &mut Vec<Finding>) {
    for isect in g.of_type(text::INTERSECTION) {
        let node = g.get(isect).expect("of_type yields emitted nodes");
        let (Some(s0), Some(s1), Some(chart)) =
            (g.get(node.ptrs[5]), g.get(node.ptrs[6]), g.get(node.ptrs[7]))
        else {
            continue;
        };
        if chart.ints[0] != chart.vecs.len() as i64 {
            out.push(finding(
                chart.index,
                "the chart's count field is the length of its own hvec array",
            ));
        }
        if chart.dbls[1] <= 0.0 {
            out.push(finding(chart.index, "a chart's base scale is positive"));
        }
        if chart.vecs.len() < 2 {
            out.push(finding(chart.index, "a chart spans its curve with at least two points"));
            continue;
        }
        for (k, &p) in chart.vecs.iter().enumerate() {
            for (surface, which) in [(s0, "first"), (s1, "second")] {
                if let Some(d) = surface_distance(surface, p) {
                    if d.abs() > ON_GEOMETRY_M {
                        out.push(finding(
                            isect,
                            format!(
                                "chart point {k} stands {d:.3e} m off the {which} surface it is \
                                 written as the intersection with"
                            ),
                        ));
                    }
                }
            }
        }
        let tangent = |p: [f64; 3]| -> Option<[f64; 3]> {
            let n0 = sensed_normal(s0, p)?;
            let n1 = sensed_normal(s1, p)?;
            let t = cross(n0, n1);
            let len = length(t);
            (len >= TANGENT_CROSS_MIN).then(|| scale(t, 1.0 / len))
        };
        for pair in chart.vecs.windows(2) {
            if let Some(t) = tangent(pair[0]) {
                let along = dot(sub(pair[1], pair[0]), t);
                if along <= -ON_GEOMETRY_M {
                    out.push(finding(
                        isect,
                        "the chart runs along the natural tangent of its surfaces' intersection",
                    ));
                }
            } else {
                out.push(finding(
                    isect,
                    "its surfaces' normals cross to a tangent at every chart point",
                ));
            }
        }
        for (limit_slot, end, what) in [(8usize, 0usize, "start"), (9, chart.vecs.len() - 1, "end")] {
            let Some(limit) = g.get(node.ptrs[limit_slot]) else { continue };
            let expected = match limit.chars[0] {
                'T' => 2,
                'H' | 'L' | 'B' => 1,
                other => {
                    out.push(finding(
                        limit.index,
                        format!("a limit's type is H, T, L or B, not {other:?}"),
                    ));
                    continue;
                }
            };
            if limit.len != Some(expected) {
                out.push(finding(
                    limit.index,
                    format!("a {what} limit of type {:?} carries {expected} hvecs", limit.chars[0]),
                ));
            }
            if let Some(&p) = limit.vecs.first() {
                let d = length(sub(p, chart.vecs[end]));
                if d > ON_GEOMETRY_M {
                    out.push(finding(
                        isect,
                        format!("its {what} limit stands {d:.3e} m from the chart's {what} point"),
                    ));
                }
            }
        }
        for &surface in &[node.ptrs[5], node.ptrs[6]] {
            let head = g.get(surface).map(|s| s.ptrs[4]).unwrap_or(0);
            let ring = g.ring(head, 1, Some(2), "geometric owner", out);
            let referencing: Vec<u32> = ring
                .iter()
                .filter_map(|&o| g.get(o).map(|n| n.ptrs[0]))
                .collect();
            if !referencing.contains(&isect) {
                out.push(finding(
                    isect,
                    format!("the surface {surface}'s owner ring records this intersection"),
                ));
            }
            for &owner in &ring {
                if g.get(owner).map(|n| n.ptrs[3]) != Some(surface) {
                    out.push(finding(owner, "an owner ring's members share one geometry node"));
                }
            }
        }
    }
}

/// The signed distance of `p` from the surface node's own implicit form, in
/// metres, or `None` for a node that is not a surface.
/// How far the point `p` stands from the surface `node` states, signed the
/// way that surface's own implicit form signs it.
fn surface_distance(node: &Node, p: [f64; 3]) -> Option<f64> {
    Some(match node.ty {
        text::PLANE => dot(sub(p, node.vecs[0]), node.vecs[1]),
        text::CYLINDER => {
            let d = sub(p, node.vecs[0]);
            let radial = sub(d, scale(node.vecs[1], dot(d, node.vecs[1])));
            length(radial) - node.dbls[0]
        }
        text::CONE => {
            let d = sub(p, node.vecs[0]);
            let along = dot(d, node.vecs[1]);
            let perp = length(sub(d, scale(node.vecs[1], along)));
            perp * node.dbls[2] + along * node.dbls[1] - node.dbls[0] * node.dbls[2]
        }
        text::SPHERE => length(sub(p, node.vecs[0])) - node.dbls[0],
        text::TORUS => {
            let d = sub(p, node.vecs[0]);
            let along = dot(d, node.vecs[1]);
            let perp = length(sub(d, scale(node.vecs[1], along)));
            ((perp - node.dbls[0]).powi(2) + along * along).sqrt() - node.dbls[1]
        }
        _ => return None,
    })
}

/// The natural normal of the surface node at `p`, reversed where the node's
/// own sense field reverses it -- the form the format's intersection
/// definition uses.
fn sensed_normal(node: &Node, p: [f64; 3]) -> Option<[f64; 3]> {
    let n = match node.ty {
        text::PLANE => node.vecs[1],
        text::CYLINDER => radial_dir(sub(p, node.vecs[0]), node.vecs[1]),
        text::CONE => neg(normalize(add(
            scale(radial_dir(sub(p, node.vecs[0]), node.vecs[1]), node.dbls[2]),
            scale(node.vecs[1], node.dbls[1]),
        ))?),
        text::SPHERE => normalize(sub(p, node.vecs[0]))?,
        text::TORUS => {
            let radial = radial_dir(sub(p, node.vecs[0]), node.vecs[1]);
            normalize(sub(p, add(node.vecs[0], scale(radial, node.dbls[0]))))?
        }
        _ => return None,
    };
    Some(if node.chars.first() == Some(&'-') { neg(n) } else { n })
}

/// The distance of `p` from the analytic curve node's own form, in metres, or
/// `None` for a node that is not one of LINE, CIRCLE or ELLIPSE.
fn curve_distance(node: &Node, p: [f64; 3]) -> Option<f64> {
    Some(match node.ty {
        text::LINE => {
            let d = sub(p, node.vecs[0]);
            length(sub(d, scale(node.vecs[1], dot(d, node.vecs[1]))))
        }
        text::CIRCLE => {
            let d = sub(p, node.vecs[0]);
            let along = dot(d, node.vecs[1]);
            let perp = length(sub(d, scale(node.vecs[1], along)));
            (perp - node.dbls[0]).hypot(along)
        }
        text::ELLIPSE => {
            let d = sub(p, node.vecs[0]);
            let y = cross(node.vecs[1], node.vecs[2]);
            let (major, minor) = (node.dbls[0], node.dbls[1]);
            let (u, v) = (dot(d, node.vecs[2]) / major, dot(d, y) / minor);
            let s = (u * u + v * v).sqrt().max(1e-12);
            let on = add(
                node.vecs[0],
                add(scale(node.vecs[2], major * u / s), scale(y, minor * v / s)),
            );
            length(sub(p, on))
        }
        _ => return None,
    })
}

/// The unit direction of `d` about `axis` through the origin, or the zero
/// direction on the axis itself.
fn radial_dir(d: [f64; 3], axis: [f64; 3]) -> [f64; 3] {
    let out = sub(d, scale(axis, dot(d, axis)));
    let len = length(out);
    if len < 1e-12 {
        [0.0; 3]
    } else {
        scale(out, 1.0 / len)
    }
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn neg(a: [f64; 3]) -> [f64; 3] {
    [-a[0], -a[1], -a[2]]
}

fn scale(a: [f64; 3], k: f64) -> [f64; 3] {
    [a[0] * k, a[1] * k, a[2] * k]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

fn length(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}

fn normalize(a: [f64; 3]) -> Option<[f64; 3]> {
    let len = length(a);
    (len > 1e-12).then(|| scale(a, 1.0 / len))
}

#[cfg(test)]
mod tests {
    use super::super::text;
    use super::super::to_xt_text;
    use super::super::fixtures::{cube_solid, rect_solid, split_l_solids};
    use super::*;
    use crate::kernel::topo::Solid;

    /// The export battery: every shape class the app can put in a file,
    /// single- and multi-body.
    fn battery() -> Vec<(&'static str, String)> {
        let cube = cube_solid();
        let rect = rect_solid(1, 1);
        let wide = rect_solid(3, 2);
        let split = split_l_solids();
        let split_refs: Vec<&Solid> = split.iter().collect();
        vec![
            ("a cube", to_xt_text(&[&cube]).expect("a cube exports")),
            ("a 1x1 bin", to_xt_text(&[&rect]).expect("a 1x1 bin exports")),
            ("a 3x2 bin", to_xt_text(&[&wide]).expect("a 3x2 bin exports")),
            (
                "a split L, two bodies",
                to_xt_text(&split_refs).expect("the split L exports"),
            ),
            (
                "a cube and a bin, list-rooted",
                to_xt_text(&[&cube, &wide]).expect("the pair exports"),
            ),
        ]
    }

    #[test]
    fn every_export_in_the_battery_validates_clean() {
        for (name, text) in battery() {
            let findings = validate_xt(&text);
            assert!(
                findings.is_empty(),
                "{name} validates clean, but:\n{}",
                findings
                    .iter()
                    .map(|f| format!("  node {}: {}", f.node, f.message))
                    .collect::<String>()
            );
        }
        let split = battery()[3].1.clone();
        let parsed = reader::parse(&split).expect("a clean file parses");
        assert!(
            parsed.nodes.iter().any(|n| n.ty == text::INTERSECTION),
            "the battery holds an intersection curve, so the chart checks are not vacuous"
        );
        assert!(
            parsed.nodes.iter().any(|n| n.ty == text::TORUS),
            "the battery holds a torus, so the torus form checks are not vacuous"
        );
    }

    /// One finding of the corrupted `text` containing `marker`, or a panic
    /// naming what the validator said instead.
    fn expect_finding(text: &str, marker: &str) {
        let findings = validate_xt(text);
        let hit = findings.iter().find(|f| f.message.contains(marker));
        assert!(
            hit.is_some(),
            "the validator reports {marker:?}, but said:\n{}",
            findings
                .iter()
                .map(|f| format!("  node {}: {}", f.node, f.message))
                .collect::<String>()
        );
    }

    /// The 1x1 bin's text, parsed, for the corruption tests to work on.
    fn parsed_bin() -> reader::Parsed {
        let rect = rect_solid(1, 1);
        reader::parse(&to_xt_text(&[&rect]).expect("a 1x1 bin exports"))
            .expect("a clean file parses")
    }

    #[test]
    fn a_dangling_pointer_is_reported() {
        let mut p = parsed_bin();
        let face = p.nodes.iter().find(|n| n.ty == text::FACE).expect("a face").index;
        p.set_ptr(face, 5, 999);
        expect_finding(&p.render(), "resolves");
    }

    #[test]
    fn a_repeated_index_is_reported() {
        let mut p = parsed_bin();
        let second = p.nodes[1].index;
        p.set_index(second, 1);
        expect_finding(&p.render(), "exactly once");
    }

    #[test]
    fn a_dropped_node_is_reported() {
        let mut p = parsed_bin();
        let victim = p.nodes[p.nodes.len() / 2].index;
        p.drop_node(victim);
        expect_finding(&p.render(), "missing");
    }

    #[test]
    fn an_edge_whose_fins_agree_is_reported() {
        let mut p = parsed_bin();
        let edge = p.nodes.iter().find(|n| n.ty == text::EDGE).expect("an edge").index;
        let fin = p.node(edge).expect("the edge resolves").ptrs[1];
        p.flip_sense(fin, 0);
        expect_finding(&p.render(), "opposite senses");
    }

    #[test]
    fn a_broken_fin_ring_is_reported() {
        let mut p = parsed_bin();
        let edge = p.nodes.iter().find(|n| n.ty == text::EDGE).expect("an edge").index;
        let fin = p.node(edge).expect("the edge resolves").ptrs[1];
        p.set_ptr(fin, 2, 0);
        expect_finding(&p.render(), "mutual");
    }

    #[test]
    fn a_chart_point_off_its_surface_is_reported() {
        let split = split_l_solids();
        let refs: Vec<&Solid> = split.iter().collect();
        let text_out = to_xt_text(&refs).expect("the split L exports");
        let mut p = reader::parse(&text_out).expect("a clean file parses");
        let isect = p
            .nodes
            .iter()
            .find(|n| n.ty == text::INTERSECTION)
            .expect("the split L holds an intersection")
            .index;
        let chart = p.node(isect).expect("it resolves").ptrs[7];
        p.offset_vec(chart, 0, 0, 1.0e-4);
        expect_finding(&p.render(), "stands");
    }

    #[test]
    fn a_moved_limit_is_reported() {
        let split = split_l_solids();
        let refs: Vec<&Solid> = split.iter().collect();
        let text_out = to_xt_text(&refs).expect("the split L exports");
        let mut p = reader::parse(&text_out).expect("a clean file parses");
        let isect = p
            .nodes
            .iter()
            .find(|n| n.ty == text::INTERSECTION)
            .expect("the split L holds an intersection")
            .index;
        let limit = p.node(isect).expect("it resolves").ptrs[8];
        p.offset_vec(limit, 0, 1, 1.0e-4);
        expect_finding(&p.render(), "limit stands");
    }

    #[test]
    fn a_non_unit_direction_is_reported() {
        let mut p = parsed_bin();
        let line = p.nodes.iter().find(|n| n.ty == text::LINE).expect("a line").index;
        p.offset_vec(line, 1, 0, 0.01);
        expect_finding(&p.render(), "unit vector");
    }

    #[test]
    fn an_edge_end_off_its_curve_is_reported() {
        let mut p = parsed_bin();
        let edge = p
            .nodes
            .iter()
            .find(|n| n.ty == text::EDGE && p.node(n.ptrs[4]).is_some_and(|c| c.ty == text::LINE))
            .expect("an edge on a line")
            .index;
        let fin = p.node(edge).expect("it resolves").ptrs[1];
        let vertex = p.node(fin).expect("it resolves").ptrs[4];
        let point = p.node(vertex).expect("it resolves").ptrs[4];
        p.offset_vec(point, 0, 0, 1.0e-4);
        expect_finding(&p.render(), "off its curve");
    }

    #[test]
    fn a_non_solid_body_is_reported() {
        let mut p = parsed_bin();
        let body = p.nodes.iter().find(|n| n.ty == text::BODY).expect("the body").index;
        p.set_int(body, 2, 2);
        expect_finding(&p.render(), "body_type");
    }

    #[test]
    fn a_header_schema_mismatch_is_reported() {
        let rect = rect_solid(1, 1);
        let text_out = to_xt_text(&[&rect]).expect("a 1x1 bin exports");
        let corrupted = text_out.replacen("SCH_1200000_12006", "SCH_9999999_99999", 1);
        assert_ne!(&corrupted, &text_out, "the surgery landed in the header");
        expect_finding(&corrupted, "schema");
    }

    #[test]
    fn adjacent_interior_spaces_are_reported() {
        let rect = rect_solid(1, 1);
        let text_out = to_xt_text(&[&rect]).expect("a 1x1 bin exports");
        let mut lines: Vec<String> = text_out.lines().map(|l| l.to_string()).collect();
        let stream_line = lines
            .iter()
            .rposition(|l| l.starts_with("0 "))
            .expect("the node stream's first record");
        lines[stream_line].insert(1, ' ');
        expect_finding(&lines.join("\n"), "adjacent spaces");
    }

    #[test]
    fn an_unparseable_stream_is_reported() {
        let rect = rect_solid(1, 1);
        let text_out = to_xt_text(&[&rect]).expect("a 1x1 bin exports");
        let corrupted = text_out.replacen(" 12 ", " xx ", 1);
        assert_ne!(&corrupted, &text_out, "the surgery landed on the root node");
        expect_finding(&corrupted, "does not parse");
    }
}
