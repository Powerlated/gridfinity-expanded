//! The Gridfinity model: a `Params` in, a watertight `Solid` out.
//!
//! This module is the wiring and the entry points, and the work is in its
//! submodules. `build`/`try_build` is the imperative path and `program` the same
//! construction as an inspectable, subset-runnable list of kernel ops; both go
//! through `plan::plan_piece` per logical bin, and both build each bin in one
//! `Builder` so its interface edges are shared rather than booleaned together.
//! `try_build_reporting` additionally returns what became of the model's blends,
//! which is the only way a caller can tell a bin that kept its fillets from one
//! that quietly gave them up.
//!
//! The submodules divide by what part of a bin they author: `spec` the standard
//! and the clamps, `params` the input, `outline` the outer profile, `cavity` the
//! void inside it, `opening` what an open edge does to that void, `wall` the
//! free-form dividers standing in it, `peg` the base beneath it, `stack` the
//! slab stacks the kernel builds a compartment from, `plan` the sequence that
//! calls all of them, `pieces` the carving that follows, and `baseplate` the
//! other thing a `Params` can ask for. Each names its own kernel imports and
//! reaches its siblings through `use super::*`, which is why every item they
//! share is `pub(super)` and why this file globs each submodule back in.

use gridfinity_brep::geom::Surface;
use gridfinity_brep::math::{Vec3, vec3_of};
use gridfinity_brep::program::{self, BlendReport, Program};
use gridfinity_brep::topo::{Builder, Loop, Solid};
use crate::layout::{GridCell, effective_walls};

mod baseplate;
mod cavity;
mod opening;
mod outline;
mod params;
mod peg;
mod pieces;
mod plan;
mod spec;
mod stack;
mod wall;

use self::baseplate::*;
use self::cavity::*;
use self::opening::*;
use self::outline::*;
pub use self::params::*;
use self::peg::*;
pub use self::pieces::*;
use self::plan::*;
pub use self::spec::*;
use self::stack::*;
use self::wall::*;

fn planar(b: &mut Builder, z: f64, up: bool, outer: Loop, inners: Vec<Loop>) {
    let surface = if up {
        Surface::plane_z(z)
    } else {
        Surface::plane(vec3_of(0.0, 0.0, z), -Vec3::Z)
    };
    b.face(surface, true, outer, inners);
}

pub fn build(p: &Params) -> Solid {
    try_build(p).expect("gridfinity program")
}

pub fn try_build(p: &Params) -> Result<Solid, String> {
    try_build_reporting(p).map(|(s, _)| s)
}

/// `try_build` plus what became of the model's blends. A baseplate asks for
/// none, so its report is empty rather than absent.
pub fn try_build_reporting(p: &Params) -> Result<(Solid, BlendReport), String> {
    let (solid, report) = match p.mode {
        Mode::Baseplate => (build_baseplate(p), BlendReport::default()),
        Mode::Bin => program::run_reporting(&program(p), |_| true)?,
    };
    if let Err(e) = solid.validate() {
        panic!("{:?} is not a closed manifold: {e}", p.mode);
    }
    let audited = crate::audit(&solid);
    assert!(
        audited.is_ok(),
        "{:?} is not geometrically sound:\n{audited}",
        p.mode
    );
    Ok((solid, report))
}

pub fn program(p: &Params) -> Program {
    let mut prog = Program::default();
    if p.mode != Mode::Bin {
        return prog;
    }
    for (bi, bin) in p.bins.iter().enumerate() {
        if bin.cells.is_empty() {
            continue;
        }
        let walls = effective_walls(&bin.cells, &bin.cells, &p.open_edges, &p.divider_edges);
        let tag = if p.bins.len() == 1 {
            "bin".to_string()
        } else {
            format!("bin {}", bi + 1)
        };
        plan_piece(p, &bin.cells, &bin.cells, walls, bin.slope, &bin.pockets, &tag, &mut prog);
    }
    prog
}

pub struct BinPiece {
    pub name: String,
    pub bin: usize,
    pub piece: usize,
    pub piece_count: usize,
    pub col: i32,
    pub row: i32,
    pub solid: Solid,
}

pub fn build_pieces(p: &Params) -> Vec<BinPiece> {
    try_build_pieces(p).expect("gridfinity piece program")
}

pub fn build_piece(
    p: &Params,
    bin_cells: &[GridCell],
    piece_cells: &[GridCell],
    slope: Option<BinSlope>,
    pockets: &[Pocket],
) -> Result<Solid, String> {
    let whole = build_bin_solid(p, bin_cells, slope, pockets)?;
    carve_to_cells(&whole, p.pitch, bin_cells, piece_cells)
}

pub fn build_bin_solid(
    p: &Params,
    bin_cells: &[GridCell],
    slope: Option<BinSlope>,
    pockets: &[Pocket],
) -> Result<Solid, String> {
    build_bin_solid_reporting(p, bin_cells, slope, pockets).map(|(s, _)| s)
}

/// `build_bin_solid` plus what became of the bin's blends.
pub fn build_bin_solid_reporting(
    p: &Params,
    bin_cells: &[GridCell],
    slope: Option<BinSlope>,
    pockets: &[Pocket],
) -> Result<(Solid, BlendReport), String> {
    let walls = effective_walls(bin_cells, bin_cells, &p.open_edges, &p.divider_edges);
    let mut prog = Program::default();
    plan_piece(p, bin_cells, bin_cells, walls, slope, pockets, "piece", &mut prog);
    let (solid, report) = program::run_reporting(&prog, |_| true)?;
    if let Err(e) = solid.validate() {
        panic!("a bin solid is not a closed manifold: {e}");
    }
    let audited = crate::audit(&solid);
    assert!(
        audited.is_ok(),
        "a bin solid is not geometrically sound:
{audited}"
    );
    Ok((solid, report))
}
