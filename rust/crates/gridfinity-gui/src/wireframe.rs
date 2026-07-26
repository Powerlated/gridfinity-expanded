//! Line and label builders for the construction debugger's wireframe overlay.
//!
//! Both builders sample through [`gridfinity_cad::kernel::topo::Edge::sample`]
//! — the same sampler `tess.rs` drives — rather than rolling a second polyline
//! approximation. That keeps the overlay landing exactly on the shaded surface
//! instead of drifting from it, and is why this module needs no new geometry
//! code. Sampling to polylines is legitimate at this altitude: it sits *at* the
//! tessellation boundary, the same side of the line as `tess.rs`, not in the
//! modelling pipeline.
//!
//! ## Thick antialiased lines
//!
//! `glLineWidth` above 1.0 is not supported on core-profile desktop GL, so each
//! segment is expanded on the CPU into a **quad** (two triangles, six vertices)
//! and widened in the *vertex shader* in screen space. Each vertex therefore
//! carries both its own endpoint and the segment's other endpoint, plus a
//! `side` signum saying which way to push it off the centreline. Widening in
//! screen space (not world space) is what keeps a line the same visual weight
//! regardless of depth. The fragment shader feathers the edge with `fwidth`,
//! so lines are antialiased independently of any MSAA on the framebuffer.
//!
//! Vertex layout, 11 floats: `[p0(3), p1(3), color(3), end(1), side(1)]`.
//! Every vertex carries *both* endpoints and an `end` selector rather than its
//! own point plus the other one, so the shader can derive the screen-space
//! normal from a canonical `p0 → p1` direction. Deriving it from "this → other"
//! instead would flip the normal at the far end of the segment, and the
//! `across` varying would then interpolate through zero along one side of the
//! quad and tear a seam through the antialiasing.

use glam::Vec3;
use gridfinity_cad::kernel::build::ring_on_plane;
use gridfinity_cad::kernel::geom::Curve;
use gridfinity_cad::kernel::sketch::Seg;
use gridfinity_cad::kernel::topo::{Builder, Solid};

/// Floats per line vertex — keep in sync with `Renderer::upload_lines`.
pub const LINE_STRIDE: usize = 11;

/// Sketch profiles.
pub const SKETCH_BLACK: [f32; 3] = [0.05, 0.05, 0.06];
/// B-rep edges of the built solid.
pub const EDGE_ORANGE: [f32; 3] = [1.0, 0.45, 0.05];

/// A type tag floated at a curve's midpoint in the viewport.
pub struct Label {
    pub at: Vec3,
    pub text: &'static str,
    pub color: [f32; 3],
}

/// The overlay's CPU-side build product: one interleaved vertex buffer plus the
/// labels to paint over it with egui.
#[derive(Default)]
pub struct Wireframe {
    pub lines: Vec<f32>,
    pub labels: Vec<Label>,
}

impl Wireframe {
    /// Expand one segment into the two triangles of a screen-space quad.
    fn push_segment(&mut self, a: Vec3, b: Vec3, color: [f32; 3]) {
        // Four distinct corners as (end, side), wound as triangles 0-1-2, 2-1-3.
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

    /// Every B-rep edge of `solid`, sampled at the preview's curve resolution
    /// and tagged with its analytic curve type.
    ///
    /// `Edge::sample` yields `n + 1` points including both endpoints, so
    /// adjacent edges meet exactly and the wireframe closes where the solid does.
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

    /// One 2D profile lifted onto `plane`, sampled, and tagged per segment.
    ///
    /// The lift and the arc sampling both go through the production path: a
    /// scratch [`Builder`] plus [`ring_on_plane`] gives the real lift rule
    /// (including its degenerate-normal fallback), and reading each resulting
    /// edge back out gives exact arc interiors. The builder is discarded
    /// immediately — this runs on regenerate, not per frame.
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

/// A representative interior point of a sampled polyline, for label placement.
/// Two points means a straight run with no interior sample, so average them.
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
    use gridfinity_cad::kernel::build::extrude;
    use gridfinity_cad::kernel::sketch::Sketch;

    /// Vertices in the buffer (6 per expanded segment).
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

        // A box is 12 straight edges; each samples to a single segment, and each
        // segment becomes a 2-triangle quad = 6 vertices.
        assert_eq!(solid.edges.len(), 12);
        assert_eq!(verts(&wf), 12 * 6);
        assert_eq!(wf.labels.len(), 12);
        assert!(wf.labels.iter().all(|l| l.text == "Line"));
        assert!(wf.labels.iter().all(|l| l.color == EDGE_ORANGE));
    }

    /// The quad must span both endpoints and both sides. Getting this wrong
    /// (e.g. deriving the normal from "this -> other") collapses the quad or
    /// tears the antialiasing seam, and neither shows up as a compile error.
    #[test]
    fn each_segment_covers_all_four_corners() {
        let solid = extrude(&Sketch::rectangle(0.0, 0.0, 4.0, 4.0), 0.0, 1.0);
        let mut wf = Wireframe::default();
        wf.add_brep_edges(&solid, 5, EDGE_ORANGE);

        for seg in 0..verts(&wf) / 6 {
            let corners: Vec<(f32, f32)> = (0..6)
                .map(|k| {
                    let v = vertex(&wf, seg * 6 + k);
                    (v[9], v[10]) // end, side
                })
                .collect();
            for expect in [(0.0, -1.0), (0.0, 1.0), (1.0, -1.0), (1.0, 1.0)] {
                assert!(corners.contains(&expect), "segment {seg} missing corner {expect:?}");
            }
            // Both endpoints ride on every vertex, so the segment is
            // reconstructible from any corner — that is what lets the shader
            // build a canonical direction.
            let first = &vertex(&wf, seg * 6)[0..6];
            for k in 1..6 {
                assert_eq!(&vertex(&wf, seg * 6 + k)[0..6], first, "endpoints must match");
            }
        }
    }

    #[test]
    fn sketch_arcs_are_sampled_and_tagged() {
        // A circle profile is all arcs; each must subdivide into several
        // segments rather than being chorded straight across.
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
        // The profile was lifted onto z=3, not left at z=0.
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
