//! The 2D vocabulary the layers above are planned in, and nothing three-
//! dimensional.
//!
//! A profile is a closed loop of exact lines and arcs (`sketch`), a loop is
//! read and reshaped at its corners (`round`), overlapping loops are resolved
//! by an exact sweep (`region2d`) or, where the input is rectilinear, on a
//! compressed coordinate grid (`rectregion`), and `nesting` decides which of
//! the resulting loops sit inside which. `math` holds the vector types, the one
//! angle wrap and the weld quantum they all agree on, `hash` the deterministic
//! maps that keep an iteration order out of the geometry, and `perf` the
//! instrumentation every layer reports into.
//!
//! Nothing here is a CAD kernel and nothing here knows what is built on it: no
//! module names a bin, a cell, a drawer, a solid or a face, and `Cargo.toml`
//! says so -- `glam` is the only dependency. What a caller does with a loop --
//! sweep it, cut with it, hand it to a kernel -- is the caller's own business.

pub mod hash;
pub mod math;
pub mod nesting;
pub mod perf;
pub mod rectregion;
pub mod region2d;
pub mod round;
pub mod sketch;

#[cfg(all(test, not(target_arch = "wasm32")))]
#[global_allocator]
static TEST_ALLOC: perf::CountingAlloc<mimalloc::MiMalloc> =
    perf::CountingAlloc::new(mimalloc::MiMalloc);
