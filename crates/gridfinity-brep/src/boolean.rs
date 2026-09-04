//! Non-destructive Boolean assembly for analytic boundary representations.
//!
//! Independently written; the phase split follows OCCT General Fuse: validate,
//! find interference candidates, split, classify cells, rebuild. Design credit:
//! OCCT `BOPAlgo_PaveFiller` and `BOPAlgo_Builder` in
//! `vendor/occt/src/ModelingAlgorithms/TKBO/BOPAlgo/`.

use crate::build::{cap, ring};
use crate::curvedge::emit_edge;
use crate::geom::Surface;
use crate::math::Vec3;
use crate::sketch::Seg;
use crate::topo::{Builder, EdgeId, Solid};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Operation {
    Fuse,
    Cut,
    Common,
}

#[derive(Clone, Copy)]
struct Bounds {
    min: Vec3,
    max: Vec3,
}

impl Bounds {
    /// Exact vertex bounds of a nonempty solid.
    fn of(solid: &Solid) -> Result<Self, String> {
        let first = solid
            .verts
            .first()
            .ok_or_else(|| "boolean: argument has no vertices".to_string())?;
        let mut out = Self {
            min: first.point,
            max: first.point,
        };
        for vertex in &solid.verts[1..] {
            out.min = out.min.min(vertex.point);
            out.max = out.max.max(vertex.point);
        }
        Ok(out)
    }

    /// True only for positive-volume box overlap; touching uses the glue path.
    fn interferes(self, other: Self) -> bool {
        (0..3).all(|axis| self.max[axis] > other.min[axis] && other.max[axis] > self.min[axis])
    }

    fn separated(self, other: Self) -> bool {
        (0..3).any(|axis| self.max[axis] < other.min[axis] || other.max[axis] < self.min[axis])
    }
}

/// Boolean result when broad-phase classification proves no face splitting is needed.
pub fn boolean(a: &Solid, b: &Solid, operation: Operation) -> Result<Solid, String> {
    a.validate()
        .map_err(|error| format!("boolean object: {error}"))?;
    b.validate()
        .map_err(|error| format!("boolean tool: {error}"))?;
    let a_bounds = Bounds::of(a)?;
    let b_bounds = Bounds::of(b)?;
    if a_bounds.interferes(b_bounds) {
        return Err(format!(
            "boolean {operation:?}: interfering arguments require face splitting"
        ));
    }
    if !a_bounds.separated(b_bounds) && operation == Operation::Fuse {
        return fuse_touching_horizontal(a, b, a_bounds, b_bounds);
    }
    match operation {
        Operation::Fuse => glue_disjoint(a, b),
        Operation::Cut => Ok(a.clone()),
        Operation::Common => Err("boolean Common: disjoint arguments have an empty result".into()),
    }
}

/// Merge two solids which meet on one horizontal support plane. The two
/// same-domain caps are internal to the result: the upper cap is rebuilt with
/// the lower cap's material loop as another hole. This is the small exact
/// subset of OCCT's same-domain face treatment needed by stacked Part Studio
/// features. Design credit: OCCT `BOPAlgo_Builder::FillImagesFaces`, in
/// `vendor/occt/src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Builder_2.cxx`.
fn fuse_touching_horizontal(a: &Solid, b: &Solid, ab: Bounds, bb: Bounds) -> Result<Solid, String> {
    let (lower, upper, z) = if (ab.max.z - bb.min.z).abs() < 1e-8 {
        (a, b, ab.max.z)
    } else if (bb.max.z - ab.min.z).abs() < 1e-8 {
        (b, a, bb.max.z)
    } else {
        return Err(
            "boolean Fuse: touching arguments do not share a horizontal support plane".into(),
        );
    };
    let mut builder = Builder::new();
    let lower_caps = append_without_cap(&mut builder, lower, z, true)?;
    let upper_caps = append_without_cap(&mut builder, upper, z, false)?;
    if lower_caps.len() != 1 || upper_caps.len() != 1 {
        return Err("boolean Fuse: unclassified touching fuse needs one cap on each side".into());
    }
    let lower_cap = &lower_caps[0];
    let upper_cap = &upper_caps[0];
    if lower_cap.len() != 1 {
        return Err("boolean Fuse: lower contact cap must have one material loop".into());
    }
    if upper_cap.is_empty() {
        return Err("boolean Fuse: upper contact cap has no outer loop".into());
    }
    let mut holes: Vec<&[(EdgeId, bool)]> = upper_cap[1..].iter().map(Vec::as_slice).collect();
    holes.push(lower_cap[0].as_slice());
    builder.face_from(
        Surface::plane(Vec3::new(0.0, 0.0, z), -Vec3::Z),
        true,
        &upper_cap[0],
        &holes,
    );
    let result = builder.build();
    result
        .validate()
        .map_err(|e| format!("boolean touching fuse result: {e}"))?;
    Ok(result)
}

/// Same-domain fuse after the feature layer has classified the material still
/// exposed on the contact plane. This mirrors the PaveFiller/Builder handoff in
/// OCCT while keeping faces and topology identifiers entirely kernel-private.
pub(crate) fn fuse_touching_horizontal_region(
    upper: &Solid,
    lower: &Solid,
    z: f64,
    exposed: &[Vec<Seg>],
) -> Result<Solid, String> {
    let mut builder = Builder::new();
    append_without_cap(&mut builder, lower, z, true)?;
    append_without_cap(&mut builder, upper, z, false)?;
    for (outer, holes) in crate::slab::group_loops(exposed) {
        let outer = ring(&mut builder, &outer, z);
        let holes: Vec<_> = holes.iter().map(|one| ring(&mut builder, one, z)).collect();
        let refs: Vec<_> = holes.iter().collect();
        cap(&mut builder, z, false, &outer, &refs);
    }
    let result = builder.build();
    result
        .validate()
        .map_err(|e| format!("boolean same-domain fuse result: {e}"))?;
    Ok(result)
}

pub(crate) fn fuse_touching_horizontal_regions(
    upper: &Solid,
    lowers: &[Solid],
    z: f64,
    exposed: &[Vec<Seg>],
) -> Result<Solid, String> {
    let mut builder = Builder::new();
    for lower in lowers {
        append_without_cap(&mut builder, lower, z, true)?;
    }
    append_without_cap(&mut builder, upper, z, false)?;
    for (outer, holes) in crate::slab::group_loops(exposed) {
        let outer = ring(&mut builder, &outer, z);
        let holes: Vec<_> = holes.iter().map(|one| ring(&mut builder, one, z)).collect();
        let refs: Vec<_> = holes.iter().collect();
        cap(&mut builder, z, false, &outer, &refs);
    }
    let result = compact(builder.build().merge_coplanar_faces(&[]));
    result
        .validate()
        .map_err(|e| format!("boolean multi-tool same-domain fuse result: {e}"))?;
    Ok(result)
}

fn compact(solid: Solid) -> Solid {
    let mask = vec![true; solid.faces.len()];
    let mut builder = Builder::resume(&solid, &mask);
    for face in 0..solid.faces.len() {
        builder.copy_face(&solid, face);
    }
    builder.build_compact_unvalidated()
}

fn append_without_cap(
    builder: &mut Builder,
    solid: &Solid,
    z: f64,
    upward: bool,
) -> Result<Vec<Vec<Vec<(EdgeId, bool)>>>, String> {
    let vertices: Vec<usize> = solid
        .verts
        .iter()
        .map(|v| builder.vertex(v.point))
        .collect();
    let edges: Vec<(EdgeId, bool)> = solid
        .edges
        .iter()
        .map(|e| emit_edge(builder, vertices[e.v0], vertices[e.v1], e.curve, e.t0, e.t1))
        .collect();
    let mut caps = Vec::new();
    for face_id in 0..solid.faces.len() {
        let face = &solid.faces[face_id];
        let loops: Vec<Vec<(EdgeId, bool)>> = solid
            .face_loops(face_id)
            .map(|lp| {
                lp.iter()
                    .map(|&(e, fwd)| {
                        let (mapped, same) = edges[e];
                        (mapped, fwd == same)
                    })
                    .collect()
            })
            .collect();
        let is_cap = matches!(face.surface,
            Surface::Plane { origin, normal, .. }
                if (origin.z - z).abs() < 1e-8
                    && normal.vec().z.abs() > 1.0 - 1e-10
                    && (normal.vec().z > 0.0) == upward
        );
        if is_cap {
            caps.push(loops);
        } else {
            let inners: Vec<&[(EdgeId, bool)]> = loops[1..].iter().map(Vec::as_slice).collect();
            builder.face_from(face.surface, face.sense, &loops[0], &inners);
        }
    }
    if caps.is_empty() {
        Err("boolean Fuse: contact cap was not found".into())
    } else {
        Ok(caps)
    }
}

/// Copy disjoint arguments into a fresh, identifier-independent result.
fn glue_disjoint(a: &Solid, b: &Solid) -> Result<Solid, String> {
    let mut builder = Builder::new();
    append(&mut builder, a);
    append(&mut builder, b);
    let result = builder.build();
    result
        .validate()
        .map_err(|error| format!("boolean fuse result: {error}"))?;
    Ok(result)
}

/// Append one solid while remapping every topology identifier.
fn append(builder: &mut Builder, solid: &Solid) {
    let vertices: Vec<usize> = solid
        .verts
        .iter()
        .map(|v| builder.vertex(v.point))
        .collect();
    let edges: Vec<(EdgeId, bool)> = solid
        .edges
        .iter()
        .map(|e| emit_edge(builder, vertices[e.v0], vertices[e.v1], e.curve, e.t0, e.t1))
        .collect();
    for face_id in 0..solid.faces.len() {
        let face = &solid.faces[face_id];
        let loops: Vec<Vec<(EdgeId, bool)>> = solid
            .face_loops(face_id)
            .map(|lp| {
                lp.iter()
                    .map(|&(e, fwd)| {
                        let (mapped, same) = edges[e];
                        (mapped, fwd == same)
                    })
                    .collect()
            })
            .collect();
        let inners: Vec<&[(EdgeId, bool)]> = loops[1..].iter().map(Vec::as_slice).collect();
        builder.face_from(face.surface, face.sense, &loops[0], &inners);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{build::extrude, sketch::Sketch};

    #[test]
    fn disjoint_fuse_is_two_valid_shells() {
        let a = extrude(&Sketch::rectangle(0.0, 0.0, 10.0, 10.0), 0.0, 5.0);
        let b = extrude(&Sketch::rectangle(30.0, 0.0, 10.0, 10.0), 0.0, 5.0);
        let result = boolean(&a, &b, Operation::Fuse).expect("disjoint fuse");
        assert_eq!(
            result.shells().len(),
            2,
            "two disjoint boxes remain two shells"
        );
    }

    #[test]
    fn disjoint_cut_preserves_the_object() {
        let a = extrude(&Sketch::rectangle(0.0, 0.0, 10.0, 10.0), 0.0, 5.0);
        let b = extrude(&Sketch::rectangle(30.0, 0.0, 10.0, 10.0), 0.0, 5.0);
        let result = boolean(&a, &b, Operation::Cut).expect("disjoint cut");
        assert_eq!(
            result.faces.len(),
            a.faces.len(),
            "a disjoint tool changes no face"
        );
    }
}
