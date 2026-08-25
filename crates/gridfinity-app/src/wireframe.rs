//! The line overlay the viewer draws on top of the shaded mesh: sketch profiles,
//! B-rep edges, and plain axis-aligned boxes. A `Wireframe` accumulates one
//! `lines` buffer in `gridfinity-render`'s `LINE_STRIDE` layout -- each segment
//! expanded here into the six vertices of a screen-space quad, since a line
//! primitive has no width -- plus the `Label`s the app paints over them. Nothing
//! in it reads the solid it is drawn beside; every entry point takes the geometry
//! and the colour to draw it in.

use gridfinity_cad::kernel::math::Vec3;
use gridfinity_cad::kernel::build::ring_on_plane;
use gridfinity_cad::kernel::geom::Curve;
use gridfinity_cad::kernel::sketch::Seg;
use gridfinity_cad::kernel::topo::{Builder, Solid};



pub const SKETCH_BLACK: [f32; 3] = [0.05, 0.05, 0.06];
pub const EDGE_ORANGE: [f32; 3] = [1.0, 0.45, 0.05];
pub const OBJECT_BLUE: [f32; 3] = [0.15, 0.55, 0.95];
pub const OBJECT_RED: [f32; 3] = [0.95, 0.2, 0.2];

pub struct Label {
    pub at: Vec3,
    pub text: &'static str,
    pub color: [f32; 3],
}

#[derive(Default)]
pub struct Wireframe {
    pub lines: Vec<f32>,
    pub labels: Vec<Label>,
}

impl Wireframe {
    fn push_segment(&mut self, a: Vec3, b: Vec3, color: [f32; 3]) {
        const CORNERS: [(f32, f32); 4] = [(0.0, -1.0), (0.0, 1.0), (1.0, -1.0), (1.0, 1.0)];
        for idx in [0usize, 1, 2, 2, 1, 3] {
            let (end, side) = CORNERS[idx];
            self.lines.extend_from_slice(&[
                a.x as f32,
                a.y as f32,
                a.z as f32,
                b.x as f32,
                b.y as f32,
                b.z as f32,
            ]);
            self.lines.extend_from_slice(&color);
            self.lines.push(end);
            self.lines.push(side);
        }
    }

    fn push_polyline(&mut self, pts: &[Vec3], color: [f32; 3]) {
        for w in pts.windows(2) {
            self.push_segment(w[0], w[1], color);
        }
    }

    pub fn add_brep_edges(&mut self, solid: &Solid, res: usize, color: [f32; 3]) {
        for e in &solid.edges {
            let pts = e.sample(true, e.seg_count(res));
            self.push_polyline(&pts, color);
            self.labels.push(Label {
                at: midpoint(&pts),
                text: curve_kind(&e.curve),
                color,
            });
        }
    }

    /// Adds the twelve edges of the axis-aligned box spanning `min` to `max`, in
    /// the same millimetre coordinates as the solid beside it, as unlabelled
    /// segments in `color`. A box with a zero extent on some axis degenerates to
    /// the rectangle or segment it really is rather than being refused, since a
    /// caller measuring a real object may legitimately have one.
    pub fn add_box(&mut self, min: Vec3, max: Vec3, color: [f32; 3]) {
        assert!(
            min.x <= max.x && min.y <= max.y && min.z <= max.z,
            "a box runs from its minimum corner to its maximum, but {min} is not under {max}"
        );
        let corner = |i: usize| {
            Vec3::new(
                if i & 1 == 0 { min.x } else { max.x },
                if i & 2 == 0 { min.y } else { max.y },
                if i & 4 == 0 { min.z } else { max.z },
            )
        };
        for i in 0..8 {
            for axis in [1usize, 2, 4] {
                if i & axis == 0 {
                    self.push_segment(corner(i), corner(i | axis), color);
                }
            }
        }
    }

    pub fn add_sketch(&mut self, profile: &[Seg], plane: (Vec3, Vec3), res: usize, color: [f32; 3]) {
        if profile.is_empty() {
            return;
        }
        let mut b = Builder::default();
        let ring = ring_on_plane(&mut b, profile, plane);
        for (k, &(id, fwd)) in ring.edges.iter().enumerate() {
            let e = b.edge(id);
            let pts = e.sample(fwd, e.seg_count(res));
            self.push_polyline(&pts, color);
            self.labels.push(Label {
                at: midpoint(&pts),
                text: seg_kind(&profile[k]),
                color,
            });
        }
    }
}

fn midpoint(pts: &[Vec3]) -> Vec3 {
    match pts.len() {
        0 => Vec3::ZERO,
        2 => (pts[0] + pts[1]) * 0.5,
        n => pts[n / 2],
    }
}

fn curve_kind(c: &Curve) -> &'static str {
    match c {
        Curve::Line { .. } => "Line",
        Curve::Circle { .. } => "Circle",
        Curve::Ellipse { .. } => "Ellipse",
        Curve::TorusSection { .. } => "TorusSection",
    }
}

fn seg_kind(s: &Seg) -> &'static str {
    match s {
        Seg::Line { .. } => "Line",
        Seg::Arc { .. } => "Arc",
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use gridfinity_render::LINE_STRIDE;
    use gridfinity_cad::kernel::build::extrude;
    use gridfinity_cad::kernel::sketch::Sketch;

    fn verts(wf: &Wireframe) -> usize {
        assert_eq!(wf.lines.len() % LINE_STRIDE, 0, "buffer must be whole vertices");
        wf.lines.len() / LINE_STRIDE
    }

    fn vertex(wf: &Wireframe, i: usize) -> &[f32] {
        &wf.lines[i * LINE_STRIDE..(i + 1) * LINE_STRIDE]
    }

    #[test]
    fn box_edges_expand_to_quads_and_are_all_labelled_line() {
        let solid = extrude(&Sketch::rectangle(0.0, 0.0, 10.0, 20.0), 0.0, 5.0);
        let mut wf = Wireframe::default();
        wf.add_brep_edges(&solid, 5, EDGE_ORANGE);

        assert_eq!(solid.edges.len(), 12);
        assert_eq!(verts(&wf), 12 * 6);
        assert_eq!(wf.labels.len(), 12);
        assert!(wf.labels.iter().all(|l| l.text == "Line"));
        assert!(wf.labels.iter().all(|l| l.color == EDGE_ORANGE));
    }

    #[test]
    fn each_segment_covers_all_four_corners() {
        let solid = extrude(&Sketch::rectangle(0.0, 0.0, 4.0, 4.0), 0.0, 1.0);
        let mut wf = Wireframe::default();
        wf.add_brep_edges(&solid, 5, EDGE_ORANGE);

        for seg in 0..verts(&wf) / 6 {
            let corners: Vec<(f32, f32)> = (0..6)
                .map(|k| {
                    let v = vertex(&wf, seg * 6 + k);
                    (v[9], v[10])
                })
                .collect();
            for expect in [(0.0, -1.0), (0.0, 1.0), (1.0, -1.0), (1.0, 1.0)] {
                assert!(corners.contains(&expect), "segment {seg} missing corner {expect:?}");
            }
            let first = &vertex(&wf, seg * 6)[0..6];
            for k in 1..6 {
                assert_eq!(&vertex(&wf, seg * 6 + k)[0..6], first, "endpoints must match");
            }
        }
    }

    #[test]
    fn sketch_arcs_are_sampled_and_tagged() {
        let profile = Sketch::circle(0.0, 0.0, 5.0).loops.remove(0);
        let mut wf = Wireframe::default();
        wf.add_sketch(&profile, (Vec3::new(0.0, 0.0, 3.0), Vec3::Z), 5, SKETCH_BLACK);

        assert_eq!(wf.labels.len(), profile.len());
        assert!(wf.labels.iter().all(|l| l.text == "Arc"));
        assert!(
            verts(&wf) / 6 > profile.len(),
            "arcs must subdivide, got {} segments for {} arcs",
            verts(&wf) / 6,
            profile.len()
        );
        for i in 0..verts(&wf) {
            assert!((vertex(&wf, i)[2] - 3.0).abs() < 1e-4, "vertex not lifted to plane");
        }
    }

    #[test]
    fn a_box_is_twelve_edges_over_its_eight_corners() {
        let (min, max) = (Vec3::new(1.0, 2.0, 3.0), Vec3::new(11.0, 22.0, 33.0));
        let mut wf = Wireframe::default();
        wf.add_box(min, max, OBJECT_BLUE);

        assert_eq!(verts(&wf), 12 * 6, "a box has twelve edges, each a quad");
        assert!(wf.labels.is_empty(), "a box carries no curve to name");

        let mut seen: Vec<(u32, u32)> = Vec::new();
        for seg in 0..verts(&wf) / 6 {
            let v = vertex(&wf, seg * 6);
            let corner = |o: usize| {
                let at = |k: usize, lo: f64, hi: f64| {
                    let c = f64::from(v[o + k]);
                    assert!(
                        (c - lo).abs() < 1e-6 || (c - hi).abs() < 1e-6,
                        "every endpoint is a corner of the box, but {c} is neither {lo} nor {hi}"
                    );
                    u32::from((c - hi).abs() < 1e-6)
                };
                at(0, min.x, max.x) | at(1, min.y, max.y) << 1 | at(2, min.z, max.z) << 2
            };
            let (a, b) = (corner(0), corner(3));
            assert_eq!(
                (a ^ b).count_ones(),
                1,
                "an edge of a box joins two corners differing on exactly one axis"
            );
            seen.push((a.min(b), a.max(b)));
        }
        seen.sort_unstable();
        seen.dedup();
        assert_eq!(seen.len(), 12, "the twelve edges must be distinct");
        for c in 0..8u32 {
            assert_eq!(
                seen.iter().filter(|(a, b)| *a == c || *b == c).count(),
                3,
                "corner {c} must meet three edges"
            );
        }
    }

    #[test]
    fn empty_profile_emits_nothing() {
        let mut wf = Wireframe::default();
        wf.add_sketch(&[], (Vec3::ZERO, Vec3::Z), 5, SKETCH_BLACK);
        assert!(wf.lines.is_empty());
        assert!(wf.labels.is_empty());
    }
}
