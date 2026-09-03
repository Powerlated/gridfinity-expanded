//! One or more solid bodies as a Parasolid XT transmit file.
//!
//! The crate owns the whole path from a body to the text: `math` and `geom` are
//! the analytic vocabulary Parasolid names -- a plane, a cylinder, a cone whose
//! single nappe is its own, a torus whose major radius is signed, an ellipse as
//! a principal pair, every direction a unit `Dir` -- and `topo` is the B-rep
//! stated in it, faces bounded by loops of directed edge uses. `occt` reads an
//! OCCT shape into exactly that, and `transmit` writes it out. Nothing here
//! approximates: geometry the format cannot state exactly is refused by name.
//!
//! `reader` and `validate` are the independent restatement of the format. They
//! re-parse the emitted text against a schema and check that every index
//! resolves and every chain a reader walks closes, so the writer is checked by
//! something other than itself. An import into a real CAD system is still the
//! acceptance test, because both halves share one author's reading of the
//! manual.

pub mod body;
pub mod geom;
pub mod hash;
pub mod isect;
pub mod math;
#[cfg(feature = "occt")]
pub mod occt;
pub mod orient;
pub mod reader;
pub mod surf;
pub mod text;
pub mod topo;
pub mod transmit;
pub mod validate;

pub use text::MM_PER_M;
pub use transmit::to_xt_text;
