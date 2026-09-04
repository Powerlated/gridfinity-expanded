//! OCCT-shaped, ownership-safe entry points for the analytic kernel.
//!
//! The API intentionally treats a completed body as an opaque `Shape` and
//! exposes fallible modeling operations on it. This follows OCCT's separation
//! between `TopoDS_Shape` and builder algorithms such as
//! `BRepPrimAPI_MakePrism` and `BRepOffsetAPI_ThruSections`; see the credited
//! vendored headers beside the methods below. The implementation remains this
//! crate's original analytic topology and algorithms.

use crate::boolean::{self, Operation};
use crate::build::{self, Ring};
use crate::geom::{Surface, Surface::*};
use crate::math::Vec3;
use crate::mesh::Mesh;
use crate::sketch::{Seg, Sketch};
use crate::slab::{self, Op as SlabOp, Slab};
use crate::split::{self, Side};
use crate::topo::Solid;

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Profile {
    pub loops: Vec<Vec<Seg>>,
}

impl Profile {
    pub fn of(outer: Vec<Seg>) -> Self {
        Self { loops: vec![outer] }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Bounds {
    pub min: [f64; 3],
    pub max: [f64; 3],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Boolean {
    Cut,
    Fuse,
    Common,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FilletEdge {
    pub midpoint: [f64; 3],
    pub radius: f64,
}

/// An opaque completed body, analogous at the public boundary to OCCT's
/// `TopoDS_Shape` rather than to its mutable low-level topology containers.
#[derive(Clone, Debug)]
pub struct Shape {
    solid: Solid,
    planar: Option<PlanarCsg>,
    join_plane: Option<JoinPlane>,
    top_region: Option<(f64, Vec<Vec<Seg>>)>,
    composite: bool,
}

#[derive(Clone, Debug)]
struct JoinPlane {
    z: f64,
    exposed: Vec<Vec<Seg>>,
}

#[derive(Clone, Debug)]
enum PlanarCsg {
    Prism {
        profile: Profile,
        z: f64,
        dz: f64,
    },
    Boolean {
        object: Box<PlanarCsg>,
        tool: Box<PlanarCsg>,
        operation: Operation,
    },
}

impl PlanarCsg {
    /// Every height where the planar cell classification can change.
    fn heights(&self, out: &mut Vec<f64>) {
        match self {
            Self::Prism { z, dz, .. } => out.extend([*z, *z + *dz]),
            Self::Boolean { object, tool, .. } => {
                object.heights(out);
                tool.heights(out);
            }
        }
    }

    /// Exact 2D material region at a height strictly inside one z-cell.
    fn region_at(&self, z_at: f64) -> Vec<Vec<Seg>> {
        match self {
            Self::Prism { profile, z, dz } => {
                if z_at > *z && z_at < *z + *dz {
                    profile.loops.clone()
                } else {
                    Vec::new()
                }
            }
            Self::Boolean {
                object,
                tool,
                operation,
            } => {
                let a = object.region_at(z_at);
                let b = tool.region_at(z_at);
                match operation {
                    Operation::Fuse => crate::region2d::region_union(&a, &b),
                    Operation::Cut => crate::region2d::region_difference(&a, &b),
                    Operation::Common => crate::region2d::region_intersection(&a, &b),
                }
            }
        }
    }
}

impl Shape {
    pub fn from_solid(solid: Solid) -> Self {
        Self {
            solid,
            planar: None,
            join_plane: None,
            top_region: None,
            composite: false,
        }
    }

    pub fn as_solid(&self) -> &Solid {
        &self.solid
    }

    pub fn into_solid(self) -> Solid {
        self.solid
    }

    /// Linear sweep of a face-like profile.
    ///
    /// API shape inspired by OCCT `BRepPrimAPI_MakePrism`, documented in
    /// `vendor/occt/src/ModelingAlgorithms/TKPrim/BRepPrimAPI/BRepPrimAPI_MakePrism.hxx`.
    pub fn prism(profile: &Profile, z: f64, dz: f64) -> Result<Self, String> {
        let (outer, holes) = profile
            .loops
            .split_first()
            .ok_or_else(|| "a prism profile must contain an outer loop".to_string())?;
        if dz <= 0.0 || !dz.is_finite() {
            return Err(format!(
                "a prism height must be finite and positive, got {dz}"
            ));
        }
        let outer = Sketch::single(outer.clone());
        let holes: Vec<Sketch> = holes.iter().cloned().map(Sketch::single).collect();
        Ok(Self {
            solid: build::prism(&outer, &holes, z, z + dz),
            planar: Some(PlanarCsg::Prism {
                profile: profile.clone(),
                z,
                dz,
            }),
            join_plane: Some(JoinPlane {
                z,
                exposed: profile.loops.clone(),
            }),
            top_region: Some((z + dz, profile.loops.clone())),
            composite: false,
        })
    }

    /// Ruled solid through compatible section wires.
    ///
    /// API shape inspired by OCCT `BRepOffsetAPI_ThruSections`, documented in
    /// `vendor/occt/src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_ThruSections.hxx`.
    pub fn loft(sections: &[(&Profile, f64)]) -> Result<Self, String> {
        if sections.len() < 2 {
            return Err("a loft needs at least two sections".to_string());
        }
        if sections.iter().any(|(profile, _)| profile.loops.len() != 1) {
            return Err(
                "the analytic loft currently accepts one outer loop per section".to_string(),
            );
        }
        let sketches: Vec<Sketch> = sections
            .iter()
            .map(|(profile, _)| Sketch::single(profile.loops[0].clone()))
            .collect();
        let rings: Vec<Ring<'_>> = sections
            .iter()
            .zip(&sketches)
            .map(|((_, z), sketch)| Ring { z: *z, sketch })
            .collect();
        let top = sections.last().expect("a loft has sections");
        Ok(Self {
            solid: build::loft(&rings),
            planar: None,
            join_plane: None,
            top_region: Some((top.1, top.0.loops.clone())),
            composite: false,
        })
    }

    /// Keep the material behind a plane whose normal points toward the part
    /// removed. This mirrors the shape-in/shape-out style of OCCT's boolean
    /// builders while retaining the analytic kernel's exact half-space trim.
    pub fn cut_half_space(&self, origin: [f64; 3], normal: [f64; 3]) -> Result<Self, String> {
        let origin = Vec3::from_array(origin);
        let normal = Vec3::from_array(normal);
        if normal.length_squared() <= f64::EPSILON || !normal.is_finite() {
            return Err("a half-space normal must be finite and nonzero".to_string());
        }
        let plane = Surface::plane(origin, normal);
        split::trim_half_space(&self.solid, &plane, Side::Negative).map(Self::from_solid)
    }

    /// Non-destructive shape Boolean. API shape and operation names follow
    /// OCCT `BRepAlgoAPI_BooleanOperation`; design credit:
    /// `vendor/occt/src/ModelingAlgorithms/TKBO/BRepAlgoAPI/BRepAlgoAPI_BooleanOperation.hxx`.
    pub fn boolean(&self, other: &Self, operation: Boolean) -> Result<Self, String> {
        let operation = match operation {
            Boolean::Cut => Operation::Cut,
            Boolean::Fuse => Operation::Fuse,
            Boolean::Common => Operation::Common,
        };
        if operation == Operation::Fuse {
            if let Some(result) = same_domain_fuse(self, other)? {
                return Ok(result);
            }
            if let Some(result) = same_domain_fuse(other, self)? {
                return Ok(result);
            }
        }
        if self.composite
            && self.join_plane.is_some()
            && self.planar.is_some()
            && other.planar.is_some()
        {
            return composite_planar_boolean(self, other, operation);
        }
        if let (Some(object), Some(tool)) = (&self.planar, &other.planar) {
            return planar_boolean(object, tool, operation);
        }
        boolean::boolean(&self.solid, &other.solid, operation).map(Self::from_solid)
    }

    /// Whole-shape validity, following OCCT's `BRepCheck_Analyzer::IsValid`
    /// boundary. Credit: `vendor/occt/src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck_Analyzer.hxx`.
    pub fn is_valid(&self) -> Result<bool, String> {
        match self.solid.validate() {
            Ok(()) => Ok(true),
            Err(error) => Err(error),
        }
    }

    pub fn bounds(&self) -> Result<Bounds, String> {
        let first = self
            .solid
            .verts
            .first()
            .ok_or_else(|| "an empty shape has no bounds".to_string())?;
        let mut min = first.point;
        let mut max = first.point;
        for vertex in &self.solid.verts[1..] {
            min = min.min(vertex.point);
            max = max.max(vertex.point);
        }
        Ok(Bounds {
            min: min.to_array(),
            max: max.to_array(),
        })
    }

    pub fn shell_count(&self) -> Result<usize, String> {
        self.solid.validate()?;
        Ok(self.solid.shells().len())
    }

    /// Signed volume of each connected shell. Positive shells enclose material;
    /// negative shells bound internal voids, matching OCCT mass-property usage.
    pub fn shell_volumes(&self) -> Result<Vec<f64>, String> {
        self.solid.validate()?;
        self.solid
            .shells()
            .into_iter()
            .map(|shell| {
                let mut mask = vec![false; self.solid.faces.len()];
                for &face in &shell.faces {
                    mask[face] = true;
                }
                let mut builder = crate::topo::Builder::resume(&self.solid, &mask);
                for &face in &shell.faces {
                    builder.copy_face(&self.solid, face);
                }
                let one = builder.build_compact_unvalidated();
                one.validate()?;
                let mesh = crate::tess::tessellate(&one, 16).to_mesh();
                let magnitude = mesh
                    .triangles()
                    .map(|[a, b, c]| a.dot(b.cross(c)) / 6.0)
                    .sum::<f64>()
                    .abs();
                Ok(if shell.encloses_material {
                    magnitude
                } else {
                    -magnitude
                })
            })
            .collect()
    }

    /// Midpoints used as stable geometric edge names across feature rebuilds.
    pub fn edge_midpoints(&self) -> Result<Vec<[f64; 3]>, String> {
        self.solid.validate()?;
        let edge_faces = self.solid.edge_faces();
        Ok(self
            .solid
            .edges
            .iter()
            .enumerate()
            .filter_map(|(id, edge)| {
                let faces = &edge_faces[id];
                if faces.len() != 2 {
                    return None;
                }
                let midpoint = edge.curve.point((edge.t0 + edge.t1) * 0.5);
                let normal = |face: usize| {
                    let surface = &self.solid.faces[face].surface;
                    surface.normal(surface.project(midpoint))
                };
                (normal(faces[0]).cross(normal(faces[1])).length_squared() > 1e-16)
                    .then_some(midpoint.to_array())
            })
            .collect())
    }

    /// Round geometrically selected edges, resolving midpoint names only after
    /// all preceding features. This follows OCCT's generated-shape/history idea
    /// rather than exposing unstable topology indices. Design credit: OCCT
    /// `BRepFilletAPI_MakeFillet` in
    /// `vendor/occt/src/ModelingAlgorithms/TKFillet/BRepFilletAPI/`.
    pub fn fillet(&self, requests: &[FilletEdge], tolerance: f64) -> Result<Self, String> {
        if tolerance <= 0.0 || !tolerance.is_finite() {
            return Err(format!(
                "fillet selection tolerance must be finite and positive, got {tolerance}"
            ));
        }
        let mut selected = Vec::with_capacity(requests.len());
        let edge_faces = self.solid.edge_faces();
        for request in requests {
            let target = Vec3::from_array(request.midpoint);
            let mut matches = self
                .solid
                .edges
                .iter()
                .enumerate()
                .filter_map(|(id, edge)| {
                    let midpoint = edge.curve.point((edge.t0 + edge.t1) * 0.5);
                    ((midpoint - target).length() <= tolerance).then_some(id)
                });
            let edge = matches
                .next()
                .ok_or_else(|| format!("fillet: no edge at midpoint {:?}", request.midpoint))?;
            if matches.next().is_some() {
                return Err(format!(
                    "fillet: midpoint {:?} names more than one edge",
                    request.midpoint
                ));
            }
            if edge_faces[edge].iter().any(|&face| {
                surface_radius(&self.solid.faces[face].surface)
                    .is_some_and(|radius| radius <= 2.0 * request.radius + 1e-8)
            }) {
                continue;
            }
            selected.push((edge, request.radius));
        }
        let solid = crate::fillet::fillet_edges(&self.solid, &selected)?;
        Ok(Self {
            solid,
            planar: None,
            join_plane: self.join_plane.clone(),
            top_region: self.top_region.clone(),
            composite: self.composite,
        })
    }

    pub fn volume(&self) -> Result<f64, String> {
        self.solid.validate()?;
        let mesh = crate::tess::tessellate(&self.solid, 16).to_mesh();
        Ok(mesh
            .triangles()
            .map(|[a, b, c]| a.dot(b.cross(c)) / 6.0)
            .sum::<f64>()
            .abs())
    }

    /// Mesh by linear deflection, like OCCT's public meshing boundary. The
    /// analytic tessellator uses one angular resolution, derived conservatively
    /// from the largest curved surface in the body.
    pub fn tessellate(&self, deflection: f64) -> Result<Mesh, String> {
        if deflection <= 0.0 || !deflection.is_finite() {
            return Err(format!(
                "mesh deflection must be finite and positive, got {deflection}"
            ));
        }
        let radius = self
            .solid
            .faces
            .iter()
            .filter_map(|face| surface_radius(&face.surface))
            .fold(0.0_f64, f64::max);
        let segments = if radius <= deflection {
            1
        } else {
            let half_angle = (1.0 - deflection / radius).clamp(-1.0, 1.0).acos();
            (std::f64::consts::FRAC_PI_2 / (2.0 * half_angle))
                .ceil()
                .max(1.0) as usize
        };
        Ok(crate::tess::tessellate(&self.solid, segments).to_mesh())
    }
}

fn surface_radius(surface: &Surface) -> Option<f64> {
    match surface {
        Cylinder { radius, .. } | Sphere { radius, .. } => Some(*radius),
        Torus {
            major_r, minor_r, ..
        } => Some(major_r + minor_r),
        Cone { .. } | Plane { .. } => None,
    }
}

/// Exact Boolean of two vertical prisms through the kernel's planar cell
/// builder. This is the same split-then-select-cells decomposition described by
/// OCCT `BOPAlgo_CellsBuilder`; design credit:
/// `vendor/occt/src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_CellsBuilder.hxx`.
fn planar_boolean(
    object: &PlanarCsg,
    tool: &PlanarCsg,
    operation: Operation,
) -> Result<Shape, String> {
    let history = PlanarCsg::Boolean {
        object: Box::new(object.clone()),
        tool: Box::new(tool.clone()),
        operation,
    };
    let mut heights = Vec::new();
    history.heights(&mut heights);
    heights.sort_by(f64::total_cmp);
    heights.dedup_by(|a, b| (*a - *b).abs() < 1e-8);
    let slabs: Vec<(SlabOp, Slab)> = heights
        .windows(2)
        .filter_map(|span| {
            let region = history.region_at((span[0] + span[1]) * 0.5);
            (!region.is_empty()).then(|| (SlabOp::Union, Slab::new(region, span[0], span[1])))
        })
        .collect();
    if slabs.is_empty() {
        return Err(format!(
            "boolean {operation:?}: operation has an empty result"
        ));
    }
    let (_, classified_bands) = slab::plan_bands(&slabs)?;
    let bottom_region = classified_bands.first().cloned().unwrap_or_default();
    let top_region_value = classified_bands.last().cloned().unwrap_or_default();
    let solid = slab::build_slabs(&slabs)?;
    Ok(Shape {
        solid,
        planar: Some(history),
        join_plane: Some(JoinPlane {
            z: heights[0],
            exposed: bottom_region,
        }),
        top_region: Some((heights[heights.len() - 1], top_region_value)),
        composite: false,
    })
}

fn same_domain_fuse(upper: &Shape, lower: &Shape) -> Result<Option<Shape>, String> {
    let (Some(join), Some((top_z, top))) = (&upper.join_plane, &lower.top_region) else {
        return Ok(None);
    };
    if (join.z - top_z).abs() >= 1e-8 {
        return Ok(None);
    }
    let exposed = crate::region2d::region_difference(&join.exposed, top);
    let solid =
        boolean::fuse_touching_horizontal_region(&upper.solid, &lower.solid, join.z, &exposed)?;
    Ok(Some(Shape {
        solid,
        planar: upper.planar.clone(),
        join_plane: Some(JoinPlane { z: join.z, exposed }),
        top_region: upper.top_region.clone(),
        composite: true,
    }))
}

fn composite_planar_boolean(
    object: &Shape,
    tool: &Shape,
    operation: Operation,
) -> Result<Shape, String> {
    let upper = planar_boolean(
        object.planar.as_ref().expect("checked planar object"),
        tool.planar.as_ref().expect("checked planar tool"),
        operation,
    )?;
    let join = object.join_plane.as_ref().expect("checked join plane");
    let foundation = split::trim_half_space(
        &object.solid,
        &Surface::plane(Vec3::new(0.0, 0.0, join.z), Vec3::Z),
        Side::Negative,
    )?;
    let solid =
        boolean::fuse_touching_horizontal_region(&upper.solid, &foundation, join.z, &join.exposed)?;
    Ok(Shape {
        solid,
        planar: upper.planar,
        join_plane: object.join_plane.clone(),
        top_region: upper.top_region,
        composite: true,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn occt_shaped_prism_is_valid_and_measurable() {
        let profile = Profile::of(
            Sketch::rounded_rect(0.0, 0.0, 20.0, 10.0, 2.0)
                .loops
                .remove(0),
        );
        let shape = Shape::prism(&profile, 3.0, 5.0).expect("analytic prism");
        assert!(
            shape.is_valid().expect("validity"),
            "a constructed prism must be valid"
        );
        assert_eq!(
            shape.shell_count().expect("shell count"),
            1,
            "one prism must have one shell"
        );
        assert!(
            shape.tessellate(0.08).expect("mesh").tri_count() > 0,
            "a prism must tessellate"
        );
    }

    #[test]
    fn overlapping_prism_cut_is_an_exact_valid_feature() {
        let square = |x: f64| Profile::of(Sketch::rectangle(x, 0.0, 10.0, 10.0).loops.remove(0));
        let object = Shape::prism(&square(0.0), 0.0, 5.0).expect("object prism");
        let tool = Shape::prism(&square(5.0), 2.0, 5.0).expect("tool prism");
        let cut = object.boolean(&tool, Boolean::Cut).expect("prism cut");
        assert!(
            cut.is_valid().expect("validity"),
            "a prism cut must be valid"
        );
        assert!(
            (cut.volume().expect("volume") - 350.0).abs() < 1e-6,
            "the overlapping half-width removes 150 cubic millimetres"
        );
    }

    #[test]
    fn nested_planar_features_reclassify_all_z_cells() {
        let square = |x: f64, width: f64| {
            Profile::of(Sketch::rectangle(x, 0.0, width, 10.0).loops.remove(0))
        };
        let body = Shape::prism(&square(0.0, 20.0), 0.0, 10.0).expect("body");
        let pocket = Shape::prism(&square(0.0, 10.0), 4.0, 8.0).expect("pocket");
        let wall = Shape::prism(&square(0.0, 2.0), 4.0, 6.0).expect("wall");
        let result = body
            .boolean(&pocket, Boolean::Cut)
            .expect("pocket cut")
            .boolean(&wall, Boolean::Fuse)
            .expect("wall add");
        assert!(
            result.is_valid().expect("validity"),
            "nested planar features must build one valid cell complex"
        );
        assert!(
            (result.volume().expect("volume") - 1520.0).abs() < 1e-6,
            "the pocket removes 600 and the wall restores 120 cubic millimetres"
        );
    }
}
