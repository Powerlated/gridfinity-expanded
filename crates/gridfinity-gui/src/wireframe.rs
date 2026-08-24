
use glam::Vec3;
use gridfinity_cad::kernel::build::ring_on_plane;
use gridfinity_cad::kernel::geom::Curve;
use gridfinity_cad::kernel::sketch::Seg;
use gridfinity_cad::kernel::topo::{Builder, Solid};



pub const SKETCH_BLACK: [f32; 3] = [0.05, 0.05, 0.06];
pub const EDGE_ORANGE: [f32; 3] = [1.0, 0.45, 0.05];

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
            self.lines.extend_from_slice(&[a.x, a.y, a.z, b.x, b.y, b.z]);
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
    fn empty_profile_emits_nothing() {
        let mut wf = Wireframe::default();
        wf.add_sketch(&[], (Vec3::ZERO, Vec3::Z), 5, SKETCH_BLACK);
        assert!(wf.lines.is_empty());
        assert!(wf.labels.is_empty());
    }
}
