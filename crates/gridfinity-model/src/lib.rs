//! Parametric Gridfinity bodies built unconditionally by Open CASCADE.

pub mod gridfinity;
pub mod kernel;
pub mod layout;
pub mod perf;
pub mod printers;
#[cfg(feature = "occt")]
#[path = "subbin_native.rs"]
pub mod subbin;
#[cfg(not(feature = "occt"))]
#[path = "subbin.rs"]
pub mod subbin;

pub use gridfinity::{Params, try_build_occt, try_build_pieces_occt};
pub use gridfinity_brep::audit::{
    AuditReport, Category, Defect, Severity, TessLeak, audit, tessellation_leaks,
};
#[cfg(not(feature = "occt"))]
pub use gridfinity_brep::tess::{Tessellation, tessellate, tessellate_shell};
#[cfg(not(feature = "occt"))]
pub use gridfinity_brep::{Shape as Solid, mesh::Mesh};
#[cfg(feature = "occt")]
pub use gridfinity_occt::{Mesh, Shape as Solid};

#[cfg(test)]
#[global_allocator]
static TEST_ALLOC: gridfinity_sketch::perf::CountingAlloc<mimalloc::MiMalloc> =
    gridfinity_sketch::perf::CountingAlloc::new(mimalloc::MiMalloc);
