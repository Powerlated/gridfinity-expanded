//! One kernel `Solid` written as one XT body.
//!
//! The kernel's B-rep and Parasolid's agree about almost everything: faces carry
//! a surface and a sense, loops are ordered rings of directed edge uses with the
//! face on their left, an edge is a curve between two vertices, and every edge is
//! used exactly twice in opposite directions. So most of this file is naming: a
//! loop's k'th entry becomes a FIN, a `Face` becomes a FACE, and the chains and
//! back-pointers a transmit file threads through its nodes get built from those.
//!
//! Three things the kernel leaves implicit have to be made explicit here. Space
//! is divided into *regions*, so a closed shell bounds a solid region on the
//! inside and the one infinite void region on the outside, and each shell is
//! therefore written twice -- once in the solid region with its faces as
//! back-faces, once in the void region with the same faces as front-faces. A
//! solid may be several disconnected lumps, which are separate regions, so the
//! faces are grouped into connected components first. And a face's surface is
//! written per face rather than shared, which is what lets the emitted surface
//! carry the sense that face needs.
//!
//! Nothing here decides how a surface or curve is spelled -- that is `surf` and
//! `isect` -- and nothing here formats a field, which is `text`.

use crate::kernel::geom::{Curve, Surface};
use crate::kernel::topo::{Solid, VertexId};
use crate::kernel::xt::isect::{self, Intersection};
use crate::kernel::xt::surf::{self, GeomLinks, XtCurve};
use crate::kernel::xt::text::{self, Index, Writer};

/// The size box and linear resolution a body declares. A transmit file carries
/// no units; these two numbers are what tell a reader the scale it is in, and
/// 1000 by 1e-8 is the pair every Parasolid application writes -- metres, with a
/// modelling resolution of ten nanometres.
const RES_SIZE: f64 = 1000.0;
const RES_LINEAR: f64 = 1.0e-8;

const BODY_TYPE_SOLID: i64 = 1;
const PART_STATE_NEW: i64 = 1;
const NOM_GEOM_STATE: i64 = 1;

/// What an edge's curve is written as: one of the format's analytic curves, or
/// the exact intersection of the two surfaces meeting along it.
enum EdgeGeometry {
    Analytic(XtCurve),
    Section(Intersection),
}

/// The face-side data every emitted face needs: the sense relating the emitted
/// surface's natural normal to the face's own outward one. The surface itself
/// is the kernel's, written as it stands.
struct FaceGeometry {
    sense: char,
}

/// Everything one loop entry becomes as a fin.
struct Fin {
    loop_node: Index,
    forward: Index,
    backward: Index,
    vertex: VertexId,
    other: Index,
    edge: Index,
    sense: char,
    next_at_vx: Index,
}

/// Writes `solid` as one BODY and every node under it, returning the body's
/// index.
///
/// The solid must be a closed oriented manifold, which is what the kernel's own
/// invariant already gives; edges it interned but no face kept are skipped, as
/// `Builder` allows them to survive. Fails, naming the face or edge, where the
/// solid uses geometry the format cannot state: a cone face spanning its apex, a
/// spindle-torus face crossing the axis its two sheets meet on, or an internal
/// void, which needs a containment analysis this does not do.
pub fn write_body(w: &mut Writer, solid: &Solid) -> Result<Index, String> {
    solid
        .validate_ignoring_unused_edges()
        .map_err(|e| format!("only a closed manifold solid can be transmitted: {e}"))?;
    assert!(
        !solid.faces.is_empty(),
        "a solid body has at least one face, so an empty solid is not one"
    );

    let geometry = face_geometry(solid)?;
    let used = used_edges(solid);
    let curves = edge_geometry(solid, &geometry, &used)?;
    let shells = solid.shells();
    let n_components = shells.len();
    if let Some(void) = shells.iter().position(|sh| !sh.encloses_material) {
        return Err(format!(
            "shell {void} of the solid's {n_components} has its material on the outside, so it \
             bounds a void inside the body, which needs a containment analysis to transmit"
        ));
    }
    let mut component = vec![0usize; solid.faces.len()];
    for (c, sh) in shells.iter().enumerate() {
        for &f in &sh.faces {
            component[f] = c;
        }
    }

    let ids = allocate(w, solid, &used, &curves, n_components);
    let (fins, fin_at_vertex) = build_fins(solid, &ids, &used);
    emit(w, solid, &geometry, &curves, &ids, &fins, &fin_at_vertex, &component, n_components);
    Ok(ids.body)
}

/// Which edges any face loop uses. `Builder` interns edges that no face keeps,
/// and only a compacting build drops them, so an edge with no uses is skipped
/// rather than written as a dangling one.
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

/// Every face's surface sense: the one that makes the emitted surface's natural
/// normal answer the kernel's outward one.
///
/// A face's normal is its surface's natural normal when the face and surface
/// senses agree and the reverse when they do not, so writing the face's own
/// sense unchanged and putting the difference between the two normals in the
/// surface's sense reproduces the kernel's outward normal exactly. That
/// difference is a property of the surface's variant rather than of the face --
/// only a cone's natural normal opposes the kernel's gradient -- so no face is
/// sampled to decide it, only to check that its points lie on it.
fn face_geometry(solid: &Solid) -> Result<Vec<FaceGeometry>, String> {
    let mut out = Vec::with_capacity(solid.faces.len());
    for (fi, face) in solid.faces.iter().enumerate() {
        let samples = solid.face_points(fi);
        surf::check_surface(&face.surface, &samples).map_err(|e| format!("face {fi}: {e}"))?;
        let opposes = surf::natural_normal_opposes_gradient(&face.surface);
        let p = samples[0];
        let agree = surf::natural_normal(&face.surface, p).dot(face.surface.gradient(p).normalize());
        assert!(
            (agree > 0.0) != opposes && agree.abs() > 0.99,
            "face {fi}'s {} has a natural normal that is the kernel's gradient up to a sign the \
             variant fixes, but they meet at a cosine of {agree}",
            surf::surface_name(&face.surface)
        );
        out.push(FaceGeometry {
            sense: if opposes { '-' } else { '+' },
        });
    }
    Ok(out)
}

/// Every used edge's curve as XT states it.
///
/// A torus section is the one kernel curve with no analytic node, and it is
/// written as the exact intersection of the two faces' surfaces meeting along
/// it -- which needs those two faces, so an edge both of whose uses belong to
/// one face has no such pair and is refused.
fn edge_geometry(
    solid: &Solid,
    geometry: &[FaceGeometry],
    used: &[bool],
) -> Result<Vec<Option<EdgeGeometry>>, String> {
    let ef = solid.edge_faces();
    let mut out = Vec::with_capacity(solid.edges.len());
    for (e, edge) in solid.edges.iter().enumerate() {
        if !used[e] {
            out.push(None);
            continue;
        }
        if let Some(curve) = surf::of_curve(&edge.curve) {
            let sampled = curve.distance(edge.curve.point((edge.t0 + edge.t1) * 0.5));
            assert!(
                sampled < surf::ON_GEOMETRY_MM,
                "edge {e}'s midpoint stands {sampled} mm off the curve node written for its \
                 {:?}",
                edge.curve
            );
            out.push(Some(EdgeGeometry::Analytic(curve)));
            continue;
        }
        let faces = &ef[e];
        if faces.len() != 2 {
            return Err(format!(
                "edge {e} is a torus section, which is written as the intersection of the two \
                 surfaces meeting along it, but {} face(s) use it",
                faces.len()
            ));
        }
        let pair = [
            (&solid.faces[faces[0]].surface, geometry[faces[0]].sense),
            (&solid.faces[faces[1]].surface, geometry[faces[1]].sense),
        ];
        let ends = (solid.vertex(edge.v0), solid.vertex(edge.v1));
        let plan = isect::plan(&edge.curve, edge.t0, edge.t1, ends, pair)
            .map_err(|m| format!("edge {e}: {m}"))?;
        out.push(Some(EdgeGeometry::Section(plan)));
    }
    Ok(out)
}

/// Every node index one body occupies, allocated in one pass so any field may
/// point at any node.
struct Ids {
    body: Index,
    void_region: Index,
    solid_region: Vec<Index>,
    solid_shell: Vec<Index>,
    void_shell: Vec<Index>,
    face: Vec<Index>,
    surface: Vec<Index>,
    loops: Vec<Vec<Index>>,
    edge: Vec<Index>,
    curve: Vec<Index>,
    section: Vec<Option<isect::Nodes>>,
    fin: Vec<[Index; 2]>,
    vertex: Vec<Index>,
    point: Vec<Index>,
    /// Per surface node index, the ring of geometric owners recording the
    /// intersection curves that depend on it.
    owners: Vec<(Index, Index, Index)>,
    highest: Index,
}

/// Reserves an index for every node the body will contain.
///
/// Order is arbitrary -- a transmit file's nodes are unordered and pointers are
/// indices -- but it is fixed here so a file is a function of the solid alone.
fn allocate(
    w: &mut Writer,
    solid: &Solid,
    used: &[bool],
    curves: &[Option<EdgeGeometry>],
    n_components: usize,
) -> Ids {
    let body = w.alloc();
    let void_region = w.alloc();
    let solid_region: Vec<Index> = (0..n_components).map(|_| w.alloc()).collect();
    let solid_shell: Vec<Index> = (0..n_components).map(|_| w.alloc()).collect();
    let void_shell: Vec<Index> = (0..n_components).map(|_| w.alloc()).collect();

    let mut face = Vec::with_capacity(solid.faces.len());
    let mut surface = Vec::with_capacity(solid.faces.len());
    let mut loops = Vec::with_capacity(solid.faces.len());
    for fi in 0..solid.faces.len() {
        face.push(w.alloc());
        surface.push(w.alloc());
        loops.push(solid.loop_ids(fi).map(|_| w.alloc()).collect());
    }

    let mut edge = vec![0; solid.edges.len()];
    let mut curve = vec![0; solid.edges.len()];
    let mut section: Vec<Option<isect::Nodes>> = (0..solid.edges.len()).map(|_| None).collect();
    let mut fin = vec![[0, 0]; solid.edges.len()];
    let mut owners: Vec<(Index, Index, Index)> = Vec::new();
    for e in 0..solid.edges.len() {
        if !used[e] {
            continue;
        }
        edge[e] = w.alloc();
        curve[e] = w.alloc();
        fin[e] = [w.alloc(), w.alloc()];
        if matches!(curves[e], Some(EdgeGeometry::Section(_))) {
            section[e] = Some(isect::Nodes {
                curve: curve[e],
                chart: w.alloc(),
                start: w.alloc(),
                end: w.alloc(),
            });
        }
    }

    let mut vertex = vec![0; solid.verts.len()];
    let mut point = vec![0; solid.verts.len()];
    let live = live_vertices(solid, used);
    for v in 0..solid.verts.len() {
        if !live[v] {
            continue;
        }
        vertex[v] = w.alloc();
        point[v] = w.alloc();
    }
    for (e, is_used) in used.iter().enumerate() {
        assert!(
            !is_used || (edge[e] != 0 && curve[e] != 0 && fin[e][0] != fin[e][1] && fin[e][0] != 0),
            "a used edge's node, curve and pair of distinct fins are allocated, but edge {e} \
             came out as {} {} {:?}",
            edge[e],
            curve[e],
            fin[e]
        );
    }

    let ef = solid.edge_faces();
    for e in 0..solid.edges.len() {
        if section[e].is_none() {
            continue;
        }
        for &fi in &ef[e] {
            owners.push((w.alloc(), curve[e], surface[fi]));
        }
    }

    Ids {
        body,
        void_region,
        solid_region,
        solid_shell,
        void_shell,
        face,
        surface,
        loops,
        edge,
        curve,
        section,
        fin,
        vertex,
        point,
        owners,
        highest: w.allocated() as Index,
    }
}

/// Which vertices the used edges reach. A vertex no used edge touches belongs
/// to no topology and is skipped with the edges that stranded it.
fn live_vertices(solid: &Solid, used: &[bool]) -> Vec<bool> {
    let mut live = vec![false; solid.verts.len()];
    for (e, edge) in solid.edges.iter().enumerate() {
        if used[e] {
            live[edge.v0] = true;
            live[edge.v1] = true;
        }
    }
    live
}

/// Every fin of the body, keyed by its node index.
///
/// A fin is one loop entry: it knows the loop it is in, the entries either side
/// of it in that ring, the vertex the edge use ends at, the edge's other fin,
/// and whether the use runs with the edge or against it. The chain of fins at a
/// vertex is threaded afterwards, once every fin knows its vertex.
fn build_fins(solid: &Solid, ids: &Ids, used: &[bool]) -> (Vec<(Index, Fin)>, Vec<Index>) {
    let kernel = solid.fins();
    let loop_node: std::collections::HashMap<u32, Index> = (0..solid.faces.len())
        .flat_map(|fi| solid.loop_ids(fi).enumerate().map(move |(li, lid)| (fi, li, lid)))
        .map(|(fi, li, lid)| (lid, ids.loops[fi][li]))
        .collect();
    let node = |slot: usize| -> Index {
        let f = &kernel[slot];
        ids.fin[f.edge][usize::from(!f.forward)]
    };
    let mut fins: Vec<(Index, Fin)> = kernel
        .iter()
        .enumerate()
        .map(|(slot, f)| {
            assert!(used[f.edge], "a loop entry names an edge, so that edge is used");
            (
                node(slot),
                Fin {
                    loop_node: loop_node[&f.loop_id],
                    forward: node(f.next_in_loop),
                    backward: node(f.prev_in_loop),
                    vertex: f.vertex,
                    other: node(f.partner),
                    edge: ids.edge[f.edge],
                    sense: if f.forward { '+' } else { '-' },
                    next_at_vx: f.next_at_vertex.map(node).unwrap_or(0),
                },
            )
        })
        .collect();
    fins.sort_by_key(|(index, _)| *index);
    assert_eq!(
        fins.len(),
        2 * used.iter().filter(|u| **u).count(),
        "a closed manifold uses every edge exactly twice, so it has two fins per used edge"
    );
    let mut head = vec![0; solid.verts.len()];
    for (slot, f) in kernel.iter().enumerate().rev() {
        head[f.vertex] = node(slot);
    }
    (fins, head)
}

/// A doubly-linked chain over `items`: the (next, previous) pair for each, null
/// at the ends.
fn chain(items: &[Index]) -> Vec<(Index, Index)> {
    (0..items.len())
        .map(|i| {
            (
                items.get(i + 1).copied().unwrap_or(0),
                if i == 0 { 0 } else { items[i - 1] },
            )
        })
        .collect()
}

/// The head of a chain, or null for an empty one.
fn head(items: &[Index]) -> Index {
    items.first().copied().unwrap_or(0)
}

/// Writes every node of the body.
fn emit(
    w: &mut Writer,
    solid: &Solid,
    geometry: &[FaceGeometry],
    curves: &[Option<EdgeGeometry>],
    ids: &Ids,
    fins: &[(Index, Fin)],
    fin_at_vertex: &[Index],
    component: &[usize],
    n_components: usize,
) {
    let faces_of: Vec<Vec<usize>> = (0..n_components)
        .map(|c| (0..solid.faces.len()).filter(|&f| component[f] == c).collect())
        .collect();
    let live_edges: Vec<Index> = ids.edge.iter().copied().filter(|&i| i != 0).collect();
    let live_curves: Vec<Index> = ids.curve.iter().copied().filter(|&i| i != 0).collect();
    let live_vertices: Vec<Index> = ids.vertex.iter().copied().filter(|&i| i != 0).collect();
    let live_points: Vec<Index> = ids.point.iter().copied().filter(|&i| i != 0).collect();

    emit_body(w, ids, &live_edges, &live_curves, &live_vertices, &live_points);
    emit_regions(w, ids, n_components);
    emit_shells(w, ids, &faces_of);
    emit_faces(w, solid, geometry, ids, &faces_of, component);
    emit_loops(w, solid, ids);
    for (index, fin) in fins {
        emit_fin(w, *index, fin, ids);
    }
    emit_edges(w, solid, curves, ids, &live_edges);
    emit_vertices(w, solid, ids, fin_at_vertex, &live_vertices, &live_points);
    for &(index, referencing, shared) in &ids.owners {
        let ring: Vec<Index> = ids
            .owners
            .iter()
            .filter(|(_, _, s)| *s == shared)
            .map(|(i, _, _)| *i)
            .collect();
        let at = ring.iter().position(|&i| i == index).expect("an owner is in its own ring");
        let n = ring.len();
        isect::write_geometric_owner(
            w,
            index,
            referencing,
            ring[(at + 1) % n],
            ring[(at + n - 1) % n],
            shared,
        );
    }
}

/// Writes the BODY node, whose fields are the heads of every chain in it.
fn emit_body(
    w: &mut Writer,
    ids: &Ids,
    edges: &[Index],
    curves: &[Index],
    vertices: &[Index],
    points: &[Index],
) {
    assert!(
        head(edges) != 0 && head(vertices) != 0 && head(curves) != 0 && head(points) != 0,
        "a closed body has at least one of every boundary geometry kind, so every chain head \
         it names is a node"
    );
    w.begin(text::BODY, ids.body);
    w.int(ids.highest as i64);
    w.ptr(0);
    w.ptr(0);
    w.ptr(0);
    w.ptr(0);
    w.ptr(0);
    w.ptr(0);
    w.real(RES_SIZE);
    w.real(RES_LINEAR);
    w.ptr(0);
    w.ptr(0);
    w.ptr(0);
    w.int(PART_STATE_NEW);
    w.ptr(0);
    w.int(BODY_TYPE_SOLID);
    w.int(NOM_GEOM_STATE);
    w.ptr(ids.solid_shell[0]);
    w.ptr(head(&ids.surface));
    w.ptr(head(curves));
    w.ptr(head(points));
    w.ptr(ids.void_region);
    w.ptr(head(edges));
    w.ptr(head(vertices));
}

/// Writes the body's regions: the one infinite void region every solid body
/// has, and one solid region per connected shell.
fn emit_regions(w: &mut Writer, ids: &Ids, n_components: usize) {
    let regions: Vec<Index> = std::iter::once(ids.void_region)
        .chain(ids.solid_region.iter().copied())
        .collect();
    let links = chain(&regions);
    w.begin(text::REGION, ids.void_region);
    w.int(ids.void_region as i64);
    w.ptr(0);
    w.ptr(ids.body);
    w.ptr(links[0].0);
    w.ptr(links[0].1);
    w.ptr(head(&ids.void_shell));
    w.ch('V');
    for c in 0..n_components {
        w.begin(text::REGION, ids.solid_region[c]);
        w.int(ids.solid_region[c] as i64);
        w.ptr(0);
        w.ptr(ids.body);
        w.ptr(links[c + 1].0);
        w.ptr(links[c + 1].1);
        w.ptr(ids.solid_shell[c]);
        w.ch('S');
    }
}

/// Writes each component's two shells: the one bounding its solid region, whose
/// faces are back-faces, and the one bounding the void region outside it, whose
/// faces are the same faces as front-faces.
fn emit_shells(w: &mut Writer, ids: &Ids, faces_of: &[Vec<usize>]) {
    let void_links = chain(&ids.void_shell);
    for (c, faces) in faces_of.iter().enumerate() {
        assert!(
            !faces.is_empty(),
            "a shell bounds at least one face, but component {c} of the body has none"
        );
        let nodes: Vec<Index> = faces.iter().map(|&f| ids.face[f]).collect();
        w.begin(text::SHELL, ids.solid_shell[c]);
        w.int(ids.solid_shell[c] as i64);
        w.ptr(0);
        w.ptr(ids.body);
        w.ptr(0);
        w.ptr(head(&nodes));
        w.ptr(0);
        w.ptr(0);
        w.ptr(ids.solid_region[c]);
        w.ptr(0);

        w.begin(text::SHELL, ids.void_shell[c]);
        w.int(ids.void_shell[c] as i64);
        w.ptr(0);
        w.ptr(0);
        w.ptr(void_links[c].0);
        w.ptr(0);
        w.ptr(0);
        w.ptr(0);
        w.ptr(ids.void_region);
        w.ptr(head(&nodes));
    }
}

/// Writes every FACE, chained into its component's shell twice over -- once as a
/// back-face of the solid region's shell and once as a front-face of the void
/// region's -- and its surface beside it.
fn emit_faces(
    w: &mut Writer,
    solid: &Solid,
    geometry: &[FaceGeometry],
    ids: &Ids,
    faces_of: &[Vec<usize>],
    component: &[usize],
) {
    let mut links: Vec<(Index, Index)> = vec![(0, 0); solid.faces.len()];
    let mut chained = 0;
    for faces in faces_of {
        let nodes: Vec<Index> = faces.iter().map(|&f| ids.face[f]).collect();
        for (slot, &f) in faces.iter().enumerate() {
            links[f] = chain(&nodes)[slot];
            chained += 1;
        }
    }
    assert_eq!(
        chained, solid.faces.len(),
        "every face is chained into exactly one component's shell, once as a back-face and once \
         as a front-face"
    );
    let surfaces = chain(&ids.surface);
    for fi in 0..solid.faces.len() {
        let c = component[fi];
        w.begin(text::FACE, ids.face[fi]);
        w.int(ids.face[fi] as i64);
        w.ptr(0);
        w.null();
        w.ptr(links[fi].0);
        w.ptr(links[fi].1);
        w.ptr(ids.loops[fi][0]);
        w.ptr(ids.solid_shell[c]);
        w.ptr(ids.surface[fi]);
        w.ch(if solid.faces[fi].sense { '+' } else { '-' });
        w.ptr(0);
        w.ptr(0);
        w.ptr(links[fi].0);
        w.ptr(links[fi].1);
        w.ptr(ids.void_shell[c]);

        surf::write_surface(
            w,
            &solid.faces[fi].surface,
            ids.surface[fi],
            &GeomLinks {
                node_id: ids.surface[fi] as i64,
                owner: ids.face[fi],
                next: surfaces[fi].0,
                prev: surfaces[fi].1,
                geometric_owner: ring_head(ids, ids.surface[fi]),
                sense: geometry[fi].sense,
            },
        );
    }
}

/// The head of the geometric-owner ring recording what depends on the geometry
/// at `shared`, or null when nothing does.
fn ring_head(ids: &Ids, shared: Index) -> Index {
    ids.owners
        .iter()
        .find(|(_, _, s)| *s == shared)
        .map(|(i, _, _)| *i)
        .unwrap_or(0)
}

/// Writes every LOOP, chained under the face it bounds, each naming one fin of
/// its ring.
fn emit_loops(w: &mut Writer, solid: &Solid, ids: &Ids) {
    for fi in 0..solid.faces.len() {
        let nodes = &ids.loops[fi];
        for (li, lp) in solid.face_loops(fi).enumerate() {
            let (first, forward) = lp[0];
            w.begin(text::LOOP, nodes[li]);
            w.int(nodes[li] as i64);
            w.ptr(0);
            w.ptr(ids.fin[first][usize::from(!forward)]);
            w.ptr(ids.face[fi]);
            w.ptr(nodes.get(li + 1).copied().unwrap_or(0));
        }
    }
}

/// Writes one FIN, which has no node id of its own.
fn emit_fin(w: &mut Writer, index: Index, fin: &Fin, ids: &Ids) {
    w.begin(text::FIN, index);
    w.ptr(0);
    w.ptr(fin.loop_node);
    w.ptr(fin.forward);
    w.ptr(fin.backward);
    w.ptr(ids.vertex[fin.vertex]);
    w.ptr(fin.other);
    w.ptr(fin.edge);
    w.ptr(0);
    w.ptr(fin.next_at_vx);
    w.ch(fin.sense);
}

/// Writes every EDGE and the curve beside it.
///
/// The edge names its positive fin first, so a reader takes its forward vertex
/// from that fin and its backward vertex from the other; and the curve's sense
/// is what says whether the edge runs with the curve's own parameter or against
/// it, since an edge carries no parameter range of its own.
fn emit_edges(
    w: &mut Writer,
    solid: &Solid,
    curves: &[Option<EdgeGeometry>],
    ids: &Ids,
    live: &[Index],
) {
    let links = chain(live);
    let curve_links =
        chain(&ids.curve.iter().copied().filter(|&i| i != 0).collect::<Vec<_>>());
    let ef = solid.edge_faces();
    let mut slot = 0;
    for (e, edge) in solid.edges.iter().enumerate() {
        let Some(geom) = &curves[e] else { continue };
        w.begin(text::EDGE, ids.edge[e]);
        w.int(ids.edge[e] as i64);
        w.ptr(0);
        w.null();
        w.ptr(ids.fin[e][0]);
        w.ptr(links[slot].1);
        w.ptr(links[slot].0);
        w.ptr(ids.curve[e]);
        w.ptr(0);
        w.ptr(0);
        w.ptr(ids.body);
        slot += 1;

        let at = slot - 1;
        match geom {
            EdgeGeometry::Analytic(curve) => {
                let mid = edge.curve.point((edge.t0 + edge.t1) * 0.5);
                let travel = edge.curve.tangent((edge.t0 + edge.t1) * 0.5)
                    * (edge.t1 - edge.t0).signum();
                let agree = travel.normalize().dot(curve.tangent(mid));
                let with_curve = edge.runs_with_curve();
                assert!(
                    agree.abs() > 0.9 && (agree > 0.0) == with_curve,
                    "edge {e} runs along its own curve, so its direction and the emitted curve's \
                     tangent must be parallel and must agree with the way its parameter range \
                     runs, but they meet at a cosine of {agree} over a range running {}",
                    if with_curve { "forward" } else { "backward" }
                );
                curve.write(
                    w,
                    ids.curve[e],
                    &GeomLinks {
                        node_id: ids.curve[e] as i64,
                        owner: ids.edge[e],
                        next: curve_links[at].0,
                        prev: curve_links[at].1,
                        geometric_owner: 0,
                        sense: if with_curve { '+' } else { '-' },
                    },
                );
            }
            EdgeGeometry::Section(plan) => {
                let nodes =
                    ids.section[e].as_ref().expect("a section edge has its section nodes");
                let faces = &ef[e];
                plan.write(
                    w,
                    nodes,
                    &GeomLinks {
                        node_id: ids.curve[e] as i64,
                        owner: ids.edge[e],
                        next: curve_links[at].0,
                        prev: curve_links[at].1,
                        geometric_owner: 0,
                        sense: plan.sense,
                    },
                    [ids.surface[faces[0]], ids.surface[faces[1]]],
                );
            }
        }
    }
}

/// Writes every VERTEX and the point beside it, each vertex heading the chain of
/// fins that end at it.
fn emit_vertices(
    w: &mut Writer,
    solid: &Solid,
    ids: &Ids,
    first_fin: &[Index],
    live: &[Index],
    points: &[Index],
) {
    let links = chain(live);
    let point_links = chain(points);
    let mut slot = 0;
    for v in 0..solid.verts.len() {
        if ids.vertex[v] == 0 {
            continue;
        }
        assert!(
            first_fin[v] != 0,
            "a vertex the body writes is reached by at least one fin, but vertex {v} has none"
        );
        w.begin(text::VERTEX, ids.vertex[v]);
        w.int(ids.vertex[v] as i64);
        w.ptr(0);
        w.ptr(first_fin[v]);
        w.ptr(links[slot].1);
        w.ptr(links[slot].0);
        w.ptr(ids.point[v]);
        w.null();
        w.ptr(ids.body);

        w.begin(text::POINT, ids.point[v]);
        w.int(ids.point[v] as i64);
        w.ptr(0);
        w.ptr(ids.vertex[v]);
        w.ptr(point_links[slot].0);
        w.ptr(point_links[slot].1);
        w.pos(solid.vertex(v));
        slot += 1;
    }
}

/// The surfaces and curves a solid uses, for a caller reporting what an export
/// contains.
pub fn geometry_census(solid: &Solid) -> (usize, usize) {
    let planes = solid
        .faces
        .iter()
        .filter(|f| matches!(f.surface, Surface::Plane { .. }))
        .count();
    let sections = solid
        .edges
        .iter()
        .filter(|e| matches!(e.curve, Curve::TorusSection { .. }))
        .count();
    (planes, sections)
}
