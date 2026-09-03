//! An OCCT shape read as a solid this crate can transmit.
//!
//! OCCT and a transmit file agree about almost everything a body is: a face
//! carries a surface and an orientation, its boundary is wires of edges, an
//! edge is a curve between two vertices, and every edge is used twice in
//! opposite directions. So this is a translation of naming rather than of
//! meaning -- an OCCT `Geom_CylindricalSurface` becomes `Surface::Cylinder`
//! with the same axis, reference direction and radius, and the parameters come
//! across untouched.
//!
//! Only what Parasolid can state exactly crosses. The bridge refuses a face
//! carrying a lofted or swept surface, an edge that is not a line, circle or
//! ellipse, and a loop that uses one edge twice as a seam on a closed surface;
//! each is named where it is found. A transmit file that said "plane" about a
//! B-spline would be worse than no file at all.

use crate::geom::{Curve, Surface};
use crate::math::{Dir, Vec3};
use crate::topo::{Builder, Loop, Solid, VertexId};
use gridfinity_occt::{Counts, EDGE_STRIDE, FACE_STRIDE, Shape};

/// What went wrong reading a shape: the bridge's own refusal, or a record this
/// translation does not know.
#[derive(Debug, Clone, PartialEq)]
pub struct Error(pub String);

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for Error {}

impl From<gridfinity_occt::Error> for Error {
    fn from(e: gridfinity_occt::Error) -> Error {
        Error(e.0)
    }
}

/// The three doubles at `at` as a point.
fn point(values: &[f64], at: usize) -> Vec3 {
    Vec3::new(values[at], values[at + 1], values[at + 2])
}

/// The three doubles at `at` as a direction. OCCT normalises every `gp_Dir` on
/// construction, so what arrives is already unit and `Dir::new` only restates
/// it to the precision this crate holds a direction to.
fn direction(values: &[f64], at: usize) -> Dir {
    Dir::new(point(values, at))
}

/// The surface one face record names, and whether its own normal points out of
/// the material.
fn surface_of(record: &[f64]) -> Result<(Surface, bool), Error> {
    let sense = record[1] != 0.0;
    let surface = match record[0] as i64 {
        0 => Surface::Plane {
            origin: point(record, 2),
            normal: direction(record, 5),
            x_axis: direction(record, 8),
        },
        1 => Surface::Cylinder {
            base: point(record, 2),
            axis: direction(record, 5),
            ref_dir: direction(record, 8),
            radius: record[11],
        },
        2 => Surface::Cone {
            pvec: point(record, 2),
            axis: direction(record, 5),
            ref_dir: direction(record, 8),
            radius: record[11],
            half_angle: record[12],
        },
        3 => Surface::Torus {
            center: point(record, 2),
            axis: direction(record, 5),
            ref_dir: direction(record, 8),
            major_r: record[11],
            minor_r: record[12],
        },
        4 => Surface::Sphere {
            center: point(record, 2),
            axis: direction(record, 5),
            ref_dir: direction(record, 8),
            radius: record[11],
        },
        other => {
            return Err(Error(format!(
                "the bridge named surface kind {other}, which this translation does not know"
            )));
        }
    };
    Ok((surface, sense))
}

/// One shape's B-rep, in the flat records the bridge fills.
struct Exported {
    counts: Counts,
    vertices: Vec<f64>,
    edges: Vec<f64>,
    faces: Vec<f64>,
    loops_per_face: Vec<usize>,
    fins_per_loop: Vec<usize>,
    fins: Vec<i64>,
    charts: Vec<f64>,
}

impl Exported {
    /// `shape`'s topology, read across the ABI in the bridge's two calls.
    fn of(shape: &Shape) -> Result<Exported, Error> {
        let counts = shape.topology_counts()?;
        let mut exported = Exported {
            counts,
            vertices: vec![0.0; counts.vertices * 3],
            edges: vec![0.0; counts.edges * EDGE_STRIDE],
            faces: vec![0.0; counts.faces * FACE_STRIDE],
            loops_per_face: vec![0; counts.faces],
            fins_per_loop: vec![0; counts.loops],
            fins: vec![0; counts.fins * 2],
            charts: vec![0.0; counts.chart_points * 3],
        };
        shape.topology_copy(
            &mut exported.vertices,
            &mut exported.edges,
            &mut exported.faces,
            &mut exported.loops_per_face,
            &mut exported.fins_per_loop,
            &mut exported.fins,
            &mut exported.charts,
        )?;
        Ok(exported)
    }
}

/// `shape` as a solid carrying the same vertices, edges, loops and faces, every
/// surface and curve in the analytic form OCCT holds it in.
///
/// Each edge is interned once in its curve's own direction, so a loop running
/// against a curve names the same edge with the opposite sense. That is what
/// makes every edge used exactly twice in opposite directions, which is the
/// property `transmit` asserts before it emits a fin.
pub fn to_solid(shape: &Shape) -> Result<Solid, Error> {
    let exported = Exported::of(shape)?;
    let mut builder = Builder::new();

    let vertices: Vec<VertexId> = exported
        .vertices
        .chunks_exact(3)
        .map(|p| builder.vertex(Vec3::new(p[0], p[1], p[2])))
        .collect();

    let mut edges = Vec::with_capacity(exported.counts.edges);
    for record in exported.edges.chunks_exact(EDGE_STRIDE) {
        let (t0, t1) = (record[1], record[2]);
        let v0 = vertices[record[3] as usize];
        let v1 = vertices[record[4] as usize];
        edges.push(match record[0] as i64 {
            0 => builder.line(v0, v1),
            1 => builder.arc(
                v0,
                v1,
                point(record, 5),
                direction(record, 8).vec(),
                record[14],
                direction(record, 11).vec(),
                t0,
                t1,
            ),
            3 => {
                let at = record[5] as usize * 3;
                let points = record[6] as usize;
                builder.section(
                    v0,
                    v1,
                    exported.charts[at..at + points * 3]
                        .chunks_exact(3)
                        .map(|p| Vec3::new(p[0], p[1], p[2]))
                        .collect(),
                )
            }
            2 => builder.ellipse(
                v0,
                v1,
                Curve::Ellipse {
                    center: point(record, 5),
                    axis: direction(record, 8),
                    x_axis: direction(record, 11),
                    major: record[14],
                    minor: record[15],
                },
                t0,
                t1,
            ),
            other => {
                return Err(Error(format!(
                    "the bridge named curve kind {other}, which this translation does not know"
                )));
            }
        });
    }

    let mut fin = 0usize;
    let mut ring_index = 0usize;
    for (f, record) in exported.faces.chunks_exact(FACE_STRIDE).enumerate() {
        let (surface, sense) = surface_of(record)?;
        let mut rings: Vec<Loop> = Vec::with_capacity(exported.loops_per_face[f]);
        for _ in 0..exported.loops_per_face[f] {
            let mut ring = Vec::with_capacity(exported.fins_per_loop[ring_index]);
            for _ in 0..exported.fins_per_loop[ring_index] {
                let (edge, forward) = edges[exported.fins[fin * 2] as usize];
                let along = exported.fins[fin * 2 + 1] != 0;
                ring.push((edge, if along { forward } else { !forward }));
                fin += 1;
            }
            rings.push(Loop { edges: ring });
            ring_index += 1;
        }
        assert!(
            !rings.is_empty(),
            "face {f} of the shape has no loops, so it bounds nothing"
        );
        let outer = rings.remove(0);
        builder.face(surface, sense, outer, rings);
    }
    assert_eq!(
        fin,
        exported.counts.fins,
        "every fin the bridge exported belongs to exactly one loop"
    );

    Ok(builder.build_unvalidated())
}

/// `shapes` as one transmit file, one body per shape in the order given.
///
/// Each body is validated as a closed manifold before it is emitted, so a shape
/// OCCT built but did not close is refused here rather than written out.
pub fn to_xt_text(shapes: &[&Shape]) -> Result<String, Error> {
    let solids: Vec<Solid> = shapes.iter().map(|s| to_solid(s)).collect::<Result<_, _>>()?;
    let borrowed: Vec<&Solid> = solids.iter().collect();
    crate::to_xt_text(&borrowed).map_err(Error)
}

#[cfg(test)]
mod tests {
    use super::{to_solid, to_xt_text};
    use crate::validate::validate_xt;
    use gridfinity_occt::{Boolean, FilletEdge, Profile, Seg, Shape};
    use std::f64::consts::PI;

    /// The counter-clockwise loop of the `w` by `h` rectangle at the origin
    /// with its corners rounded to `r`: a Gridfinity outline in miniature, and
    /// a profile whose prism carries both of the analytic surfaces a bin's
    /// outline does -- planes for the sides, cylinders for the corners.
    fn rounded_rect(w: f64, h: f64, r: f64) -> Profile {
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
                b: [w, h - r],
            },
            arc([w, h - r], [w - r, h], [w - r, h - r], 0.0, PI / 2.0),
            Seg::Line {
                a: [w - r, h],
                b: [r, h],
            },
            arc([r, h], [0.0, h - r], [r, h - r], PI / 2.0, PI),
            Seg::Line {
                a: [0.0, h - r],
                b: [0.0, r],
            },
            arc([0.0, r], [r, 0.0], [r, r], PI, 1.5 * PI),
        ])
    }

    #[track_caller]
    fn assert_transmits(shape: &Shape, what: &str) -> String {
        let text = to_xt_text(&[shape]).unwrap_or_else(|e| panic!("{what} does not transmit: {e}"));
        let findings = validate_xt(&text);
        assert!(
            findings.is_empty(),
            "the transmit file for {what} is not sound: {findings:?}"
        );
        text
    }

    #[test]
    fn a_box_from_occt_transmits() {
        let shape = Shape::box_solid(20.0, 30.0, 10.0).expect("OCCT box");
        let solid = to_solid(&shape).expect("read box");
        assert_eq!(solid.faces.len(), 6, "a box is bounded by six planes");
        assert_eq!(solid.edges.len(), 12, "a box has twelve edges");
        assert_eq!(solid.verts.len(), 8, "a box has eight vertices");
        assert_transmits(&shape, "a box");
    }

    #[test]
    fn a_rounded_prism_from_occt_transmits() {
        let shape = Shape::prism(&rounded_rect(41.5, 41.5, 4.0), 0.0, 7.0).expect("OCCT prism");
        let solid = to_solid(&shape).expect("read prism");
        assert_eq!(
            solid.faces.len(),
            10,
            "a rounded rectangle sweeps four walls, four corner cylinders and two caps"
        );
        assert_transmits(&shape, "a rounded prism");
    }

    #[test]
    fn a_hollow_body_from_occt_transmits() {
        let outer = Shape::prism(&rounded_rect(41.5, 41.5, 4.0), 0.0, 20.0).expect("outer");
        let inner = Shape::prism(&rounded_rect(35.5, 35.5, 2.0), 3.0, 20.0).expect("inner");
        let hollow = outer.boolean(&inner, Boolean::Cut).expect("cut");
        assert_transmits(&hollow, "a hollowed bin");
    }

    /// Both blends transmit, and the single-edge one is why the section path
    /// exists: OCCT holds its two end arcs as B-splines rather than as circles,
    /// so nothing analytic names them and each crosses as the intersection of
    /// the cylinder and the plane it actually is.
    #[test]
    fn a_blend_transmits_whether_or_not_its_edges_are_analytic() {
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
        let rounded = prism.fillet(&rim, 1e-6).expect("a closed rim blend");
        assert_transmits(&rounded, "a prism with its whole rim blended");

        let box_solid = Shape::box_solid(20.0, 20.0, 10.0).expect("box");
        let one_edge = box_solid
            .fillet(
                &[FilletEdge {
                    midpoint: [0.0, 0.0, 5.0],
                    radius: 3.0,
                }],
                1e-6,
            )
            .expect("fillet");
        let solid = to_solid(&one_edge).expect("read a single-edge blend");
        let sections = solid
            .edges
            .iter()
            .filter(|e| matches!(e.curve, crate::geom::Curve::Section { .. }))
            .count();
        assert_eq!(
            sections, 2,
            "the blend's two end arcs reach this crate as sections, so the intersection path is              not vacuous here"
        );
        assert_transmits(&one_edge, "a box with one edge blended");
    }

    /// A lofted body is refused for its **surface**, and the refusal says so
    /// rather than approximating it. Its edges no longer refuse anything: a
    /// B-spline edge crosses as a section, so what is left is the one thing
    /// with no analytic escape.
    /// A blended body cut in two, which is what every printed piece is. The cut
    /// runs through the **corner** blends, whose surface is a torus, so where it
    /// meets them the section is a quartic -- the curve that has no analytic
    /// node and the reason the intersection path exists. Cut through the
    /// straight run instead and every section is a line or a circle, which is
    /// why this one is placed where it is. Both pieces transmit, and they transmit together
    /// as one file the way an export writes them.
    #[test]
    fn a_blended_body_cut_in_two_transmits_as_both_of_its_pieces() {
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

        let cut_at = corner - 1.0;
        let left_tool = Shape::box_solid(cut_at, d * 2.0, h * 2.0).expect("left tool");
        let right = body.boolean(&left_tool, Boolean::Cut).expect("keep the right");
        let left = body.boolean(&right, Boolean::Cut).expect("keep the left");

        let pieces = [&left, &right];
        let sections: usize = pieces
            .iter()
            .map(|p| {
                to_solid(p)
                    .expect("read a piece")
                    .edges
                    .iter()
                    .filter(|e| matches!(e.curve, crate::geom::Curve::Section { .. }))
                    .count()
            })
            .sum();
        assert!(
            sections > 0,
            "the cut crosses the rim blend, so the pieces carry section curves and this is not              a test about plain prisms"
        );
        for (piece, name) in pieces.iter().zip(["the left piece", "the right piece"]) {
            assert_transmits(piece, name);
        }
        let text = to_xt_text(&pieces).expect("both pieces export as one file");
        let findings = validate_xt(&text);
        assert!(
            findings.is_empty(),
            "the two pieces transmit as one file: {findings:?}"
        );
    }

    #[test]
    fn a_lofted_body_is_refused_by_name() {
        let lower = rounded_rect(20.0, 20.0, 2.0);
        let upper = rounded_rect(16.0, 16.0, 2.0);
        let peg = Shape::loft(&[(&lower, 0.0), (&upper, 5.0)]).expect("loft");
        let refused = to_solid(&peg).expect_err("a lofted body cannot be transmitted");
        assert!(
            refused.to_string().contains("lofted or swept"),
            "the refusal names the surface it could not state, got {refused}"
        );
    }
}
