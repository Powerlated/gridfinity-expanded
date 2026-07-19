//! The analytic-surface B-rep CAD kernel — no Gridfinity in here.
//!
//! Pipeline: [`sketch`] → [`build`] features → [`topo`] B-rep solid →
//! [`fillet`] → [`tess`] → [`mesh`] → STL.
//!
//! Everything upstream of [`tess`] is exact: analytic surfaces and curves,
//! closed-form intersections. Triangles are a terminal output format and are
//! never read back into modelling (see the hard rule in `CLAUDE.md`).
//!
//! [`segdiff`] and [`rectregion`] are the 2D region engines the model layer
//! builds footprints with; they depend only on [`math`] and [`sketch`], so
//! they live here rather than beside the Gridfinity code.

pub mod build;
pub mod fillet;
pub mod geom;
pub mod math;
pub mod mesh;
pub mod rectregion;
pub mod segdiff;
pub mod sketch;
pub mod tess;
pub mod topo;
