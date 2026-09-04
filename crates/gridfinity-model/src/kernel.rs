//! A common model-backend boundary for the analytic and OCCT kernels.

use crate::gridfinity::Params;
use gridfinity_sketch::sketch::Seg;

pub type Profile = Vec<Vec<Seg>>;

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

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Bounds {
    pub min: [f64; 3],
    pub max: [f64; 3],
}

/// Onshape-like feature vocabulary available to the parametric model. Concrete
/// topology is absent by construction: each backend owns an opaque shape.
pub trait FeatureKernel {
    type Shape: KernelShape;

    const NAME: &'static str;

    fn prism(profile: &Profile, z: f64, dz: f64) -> Result<Self::Shape, String>;
    fn loft(sections: &[(Profile, f64)]) -> Result<Self::Shape, String>;
    fn boolean(
        object: &Self::Shape,
        tool: &Self::Shape,
        operation: Boolean,
    ) -> Result<Self::Shape, String>;
    fn cut_half_space(
        shape: &Self::Shape,
        origin: [f64; 3],
        normal: [f64; 3],
    ) -> Result<Self::Shape, String>;
    fn fillet(
        shape: &Self::Shape,
        edges: &[FilletEdge],
        tolerance: f64,
    ) -> Result<Self::Shape, String>;
    fn bounds(shape: &Self::Shape) -> Result<Bounds, String>;
    fn volume(shape: &Self::Shape) -> Result<f64, String>;
    fn shell_volumes(shape: &Self::Shape) -> Result<Vec<f64>, String>;
    fn edge_midpoints(shape: &Self::Shape) -> Result<Vec<[f64; 3]>, String>;
}

pub enum AnalyticFeatures {}

impl FeatureKernel for AnalyticFeatures {
    type Shape = gridfinity_brep::Shape;
    const NAME: &'static str = "analytic";

    fn prism(profile: &Profile, z: f64, dz: f64) -> Result<Self::Shape, String> {
        gridfinity_brep::Shape::prism(
            &gridfinity_brep::occt_api::Profile {
                loops: profile.clone(),
            },
            z,
            dz,
        )
    }

    fn loft(sections: &[(Profile, f64)]) -> Result<Self::Shape, String> {
        let profiles: Vec<gridfinity_brep::occt_api::Profile> = sections
            .iter()
            .map(|(loops, _)| gridfinity_brep::occt_api::Profile {
                loops: loops.clone(),
            })
            .collect();
        let borrowed: Vec<(&gridfinity_brep::occt_api::Profile, f64)> = profiles
            .iter()
            .zip(sections)
            .map(|(profile, (_, z))| (profile, *z))
            .collect();
        gridfinity_brep::Shape::loft(&borrowed)
    }

    fn boolean(
        object: &Self::Shape,
        tool: &Self::Shape,
        operation: Boolean,
    ) -> Result<Self::Shape, String> {
        let operation = match operation {
            Boolean::Cut => gridfinity_brep::occt_api::Boolean::Cut,
            Boolean::Fuse => gridfinity_brep::occt_api::Boolean::Fuse,
            Boolean::Common => gridfinity_brep::occt_api::Boolean::Common,
        };
        object.boolean(tool, operation)
    }

    fn cut_half_space(
        shape: &Self::Shape,
        origin: [f64; 3],
        normal: [f64; 3],
    ) -> Result<Self::Shape, String> {
        shape.cut_half_space(origin, normal)
    }
    fn fillet(
        shape: &Self::Shape,
        edges: &[FilletEdge],
        tolerance: f64,
    ) -> Result<Self::Shape, String> {
        let edges: Vec<_> = edges
            .iter()
            .map(|edge| gridfinity_brep::occt_api::FilletEdge {
                midpoint: edge.midpoint,
                radius: edge.radius,
            })
            .collect();
        shape.fillet(&edges, tolerance)
    }
    fn bounds(shape: &Self::Shape) -> Result<Bounds, String> {
        shape.bounds().map(|b| Bounds {
            min: b.min,
            max: b.max,
        })
    }
    fn volume(shape: &Self::Shape) -> Result<f64, String> {
        shape.volume()
    }
    fn shell_volumes(shape: &Self::Shape) -> Result<Vec<f64>, String> {
        shape.shell_volumes()
    }
    fn edge_midpoints(shape: &Self::Shape) -> Result<Vec<[f64; 3]>, String> {
        shape.edge_midpoints()
    }
}

pub enum OcctFeatures {}

impl FeatureKernel for OcctFeatures {
    type Shape = gridfinity_occt::Shape;
    const NAME: &'static str = "occt";

    fn prism(profile: &Profile, z: f64, dz: f64) -> Result<Self::Shape, String> {
        gridfinity_occt::Shape::prism(&occt_profile(profile), z, dz).map_err(|e| e.to_string())
    }

    fn loft(sections: &[(Profile, f64)]) -> Result<Self::Shape, String> {
        let profiles: Vec<_> = sections.iter().map(|(p, _)| occt_profile(p)).collect();
        let borrowed: Vec<_> = profiles
            .iter()
            .zip(sections)
            .map(|(p, (_, z))| (p, *z))
            .collect();
        gridfinity_occt::Shape::loft(&borrowed).map_err(|e| e.to_string())
    }

    fn boolean(
        object: &Self::Shape,
        tool: &Self::Shape,
        operation: Boolean,
    ) -> Result<Self::Shape, String> {
        let operation = match operation {
            Boolean::Cut => gridfinity_occt::Boolean::Cut,
            Boolean::Fuse => gridfinity_occt::Boolean::Fuse,
            Boolean::Common => gridfinity_occt::Boolean::Common,
        };
        object.boolean(tool, operation).map_err(|e| e.to_string())
    }

    fn cut_half_space(
        shape: &Self::Shape,
        origin: [f64; 3],
        normal: [f64; 3],
    ) -> Result<Self::Shape, String> {
        shape
            .cut_half_space(origin, normal)
            .map_err(|e| e.to_string())
    }
    fn fillet(
        shape: &Self::Shape,
        edges: &[FilletEdge],
        tolerance: f64,
    ) -> Result<Self::Shape, String> {
        let edges: Vec<_> = edges
            .iter()
            .map(|e| gridfinity_occt::FilletEdge {
                midpoint: e.midpoint,
                radius: e.radius,
            })
            .collect();
        shape.fillet(&edges, tolerance).map_err(|e| e.to_string())
    }
    fn bounds(shape: &Self::Shape) -> Result<Bounds, String> {
        shape
            .bounds()
            .map(|b| Bounds {
                min: b.min,
                max: b.max,
            })
            .map_err(|e| e.to_string())
    }
    fn volume(shape: &Self::Shape) -> Result<f64, String> {
        shape.volume().map_err(|e| e.to_string())
    }
    fn shell_volumes(shape: &Self::Shape) -> Result<Vec<f64>, String> {
        shape.shell_volumes().map_err(|e| e.to_string())
    }
    fn edge_midpoints(shape: &Self::Shape) -> Result<Vec<[f64; 3]>, String> {
        shape.edge_midpoints().map_err(|e| e.to_string())
    }
}

fn occt_profile(profile: &Profile) -> gridfinity_occt::Profile {
    gridfinity_occt::Profile {
        loops: profile
            .iter()
            .map(|one| {
                one.iter()
                    .map(|seg| match *seg {
                        Seg::Line { a, b } => gridfinity_occt::Seg::Line {
                            a: [a.x, a.y],
                            b: [b.x, b.y],
                        },
                        Seg::Arc {
                            a,
                            b,
                            center,
                            radius,
                            a0,
                            a1,
                        } => gridfinity_occt::Seg::Arc {
                            a: [a.x, a.y],
                            b: [b.x, b.y],
                            center: [center.x, center.y],
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

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MeshStats {
    pub vertices: usize,
    pub triangles: usize,
}

/// Kernel-neutral behavior of a completed body. Like OCCT's `TopoDS_Shape`,
/// this boundary keeps topology opaque after construction.
pub trait KernelShape {
    fn is_valid(&self) -> Result<bool, String>;
    fn mesh_stats(&self, deflection: f64) -> Result<MeshStats, String>;
}

/// The complete Gridfinity workload implemented by a CAD kernel.
///
/// Keeping the solid as an associated type makes callers statically generic:
/// benchmark loops have no enum dispatch and each kernel retains its native
/// topology representation.
pub trait Kernel {
    type Solid: KernelShape;

    const NAME: &'static str;

    fn build(params: &Params) -> Result<Self::Solid, String>;
}

pub enum Legacy {}

impl Kernel for Legacy {
    type Solid = gridfinity_brep::Shape;

    const NAME: &'static str = "analytic-brep";

    fn build(params: &Params) -> Result<Self::Solid, String> {
        crate::gridfinity::try_build_features::<AnalyticFeatures>(params)
    }
}

pub enum Occt {}

impl Kernel for Occt {
    type Solid = gridfinity_occt::Shape;

    const NAME: &'static str = "occt";

    fn build(params: &Params) -> Result<Self::Solid, String> {
        crate::gridfinity::try_build_occt(params)
    }
}

impl KernelShape for gridfinity_brep::Shape {
    fn is_valid(&self) -> Result<bool, String> {
        self.is_valid()
    }

    fn mesh_stats(&self, deflection: f64) -> Result<MeshStats, String> {
        let mesh = self.tessellate(deflection)?;
        Ok(MeshStats {
            vertices: mesh.positions.len(),
            triangles: mesh.indices.len() / 3,
        })
    }
}

impl KernelShape for gridfinity_occt::Shape {
    fn is_valid(&self) -> Result<bool, String> {
        self.is_valid().map_err(|error| error.to_string())
    }

    fn mesh_stats(&self, deflection: f64) -> Result<MeshStats, String> {
        let mesh = self
            .tessellate(deflection)
            .map_err(|error| error.to_string())?;
        Ok(MeshStats {
            vertices: mesh.positions.len(),
            triangles: mesh.indices.len() / 3,
        })
    }
}

#[derive(Clone, Copy, Debug)]
pub struct BenchResult {
    pub kernel: &'static str,
    pub builds: u32,
    pub build_time: std::time::Duration,
    pub tessellation_time: std::time::Duration,
    pub mesh: MeshStats,
}

/// Times identical parameters through a statically selected kernel backend.
pub fn benchmark<K: Kernel>(params: &Params, builds: u32) -> Result<BenchResult, String> {
    if builds == 0 {
        return Err("benchmark repetition count must be positive".to_string());
    }
    let started = std::time::Instant::now();
    let mut solid = None;
    for _ in 0..builds {
        solid = Some(std::hint::black_box(K::build(std::hint::black_box(
            params,
        ))?));
    }
    let build_time = started.elapsed();
    let solid = solid.expect("a positive repetition count produced a solid");
    if !solid.is_valid()? {
        return Err(format!("{} built an invalid shape", K::NAME));
    }
    let started = std::time::Instant::now();
    let mesh = std::hint::black_box(&solid).mesh_stats(0.08)?;
    let tessellation_time = started.elapsed();
    Ok(BenchResult {
        kernel: K::NAME,
        builds,
        build_time,
        tessellation_time,
        mesh,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn legacy_backend_runs_through_the_generic_boundary() {
        let result = benchmark::<Legacy>(&Params::rect(1, 1), 1).expect("legacy benchmark");
        assert!(result.mesh.triangles > 0);
    }

    #[test]
    fn neutral_prism_operation_drives_the_analytic_adapter() {
        use gridfinity_sketch::sketch::Sketch;
        let profile = Sketch::rectangle(0.0, 0.0, 10.0, 8.0).loops;
        let analytic = AnalyticFeatures::prism(&profile, 0.0, 3.0).expect("analytic prism");
        assert!(analytic.is_valid().expect("analytic validity"));
        assert!((AnalyticFeatures::volume(&analytic).expect("volume") - 240.0).abs() < 1e-8);
    }

    #[test]
    fn analytic_features_build_a_default_bin() {
        let body = crate::gridfinity::try_build_features::<AnalyticFeatures>(&Params::default())
            .expect("feature-only analytic bin");
        assert!(body.is_valid().expect("validity"));
    }
}
