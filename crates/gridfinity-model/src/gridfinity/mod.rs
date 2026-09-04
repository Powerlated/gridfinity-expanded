//! Gridfinity parameter planning and native OCCT body construction.

#[path = "baseplate_native.rs"]
mod baseplate_native;
mod cavity;
mod native;
mod opening;
mod outline;
mod params;
mod peg;
#[path = "pieces_native.rs"]
mod pieces_native;
mod spec;
mod wall;

use self::baseplate_native::*;
use self::cavity::*;
use self::native::*;
use self::opening::*;
use self::outline::*;
pub use self::params::*;
use self::peg::*;
pub use self::pieces_native::*;
pub use self::spec::*;
use self::wall::*;

/// Builds the complete body declared by `p` in OCCT.
pub fn try_build_occt(p: &Params) -> Result<gridfinity_occt::Shape, String> {
    try_build_features::<crate::kernel::OcctFeatures>(p)
}

/// Builds a bin using only the public Part-Studio-like feature vocabulary.
pub fn try_build_features<K: crate::kernel::FeatureKernel>(p: &Params) -> Result<K::Shape, String> {
    use crate::kernel::Boolean;
    if p.mode == Mode::Baseplate {
        return build_baseplate_features::<K>(p);
    }
    let mut body: Option<K::Shape> = None;
    for bin in &p.bins {
        if bin.cells.is_empty() {
            continue;
        }
        let one = build_closed_flat_bin::<K>(p, &bin.cells, &bin.pockets, bin.slope)?;
        body = Some(match body {
            Some(all) => K::boolean(&all, &one, Boolean::Fuse)
                .map_err(|e| format!("OCCT could not join logical bins: {e}"))?,
            None => one,
        });
    }
    body.ok_or_else(|| "a model with no cells has no OCCT body".to_string())
}

#[cfg(feature = "occt")]
pub fn build(p: &Params) -> gridfinity_occt::Shape {
    try_build_occt(p).expect("OCCT builds Gridfinity body")
}

#[cfg(feature = "occt")]
pub fn try_build(p: &Params) -> Result<gridfinity_occt::Shape, String> {
    try_build_occt(p)
}

#[cfg(feature = "occt")]
pub fn try_build_pieces(p: &Params) -> Result<Vec<OcctBinPiece>, String> {
    try_build_pieces_occt(p)
}

#[cfg(not(feature = "occt"))]
pub fn build(p: &Params) -> gridfinity_brep::Shape {
    try_build_features::<crate::kernel::AnalyticFeatures>(p)
        .expect("the analytic feature kernel builds the Gridfinity body")
}

#[cfg(not(feature = "occt"))]
pub fn try_build(p: &Params) -> Result<gridfinity_brep::Shape, String> {
    try_build_features::<crate::kernel::AnalyticFeatures>(p)
}
