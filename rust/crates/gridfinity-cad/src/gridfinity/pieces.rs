//! Cutting a finished bin into the pieces a printer can fit.
//!
//! The bin is built once and carved, never authored per piece: `carve_to_cells`
//! trims the whole solid to the vertical prism over one piece's cell set, so a
//! divider at a seam becomes a wall in both halves and a plain seam cuts open.
//! The prism follows the cell set itself rather than its bounding box, because
//! two pieces' boxes can overlap and each claim the other's material, and it
//! reaches `REENTRANT_FILLET_OVERHANG` into empty neighbours along y so the
//! corner fillet that overhangs the grid is kept exactly once.
//! `try_build_pieces` is the whole path for one `Params`: build each logical
//! bin, partition it, carve.

use super::*;
use crate::kernel::math::Vec3;
use crate::kernel::rectregion::{RectF, trace_rects};
use crate::kernel::split::{Cut, Side, trim};
use crate::kernel::topo::Solid;
use crate::layout::{GridCell, partition_cells};

/// Cut one printable piece out of the finished bin: keep the material inside the
/// vertical prism over the piece's cells. A piece is any connected polyomino, not
/// necessarily a grid slab, so this must follow the cell set itself -- trimming to
/// the piece's bounding box duplicates material wherever one piece's box covers
/// another piece's cells.
pub fn carve_to_cells(
    whole: &Solid,
    bin_cells: &[GridCell],
    piece_cells: &[GridCell],
) -> Result<Solid, String> {
    if piece_cells.is_empty() {
        return Ok(whole.clone());
    }
    if piece_is_enclosed(bin_cells, piece_cells) {
        return Err(
            "a piece surrounded on every side by the rest of the bin is not supported: the cut \
             runs through the middle of faces it never reaches the boundary of, and trimming \
             cannot open a new hole in a face"
                .into(),
        );
    }
    let cell_rect = |c: &GridCell| {
        RectF::new(
            c.x as f32 * GRID_PITCH,
            c.y as f32 * GRID_PITCH,
            GRID_PITCH,
            GRID_PITCH,
        )
    };
    let mut rects: Vec<RectF> = piece_cells.iter().map(cell_rect).collect();
    for c in piece_cells {
        for step in [-1i32, 1] {
            let neighbour = GridCell {
                x: c.x,
                y: c.y + step,
            };
            if bin_cells.contains(&neighbour) {
                continue;
            }
            let y = if step > 0 {
                (c.y + 1) as f32 * GRID_PITCH
            } else {
                c.y as f32 * GRID_PITCH - REENTRANT_FILLET_OVERHANG
            };
            rects.push(RectF::new(
                c.x as f32 * GRID_PITCH,
                y,
                GRID_PITCH,
                REENTRANT_FILLET_OVERHANG,
            ));
        }
    }
    let loops: Vec<Vec<(f32, f32)>> = trace_rects(&rects, &[])
        .iter()
        .map(|lp| lp.pts.iter().map(|p| (p.x, p.y)).collect())
        .collect();
    if loops.is_empty() {
        return Err("a piece traced no boundary".into());
    }
    let cut = Cut::prism(&loops, Vec3::Z)?;
    if !straddles(whole, &cut) {
        return Ok(whole.clone());
    }
    trim(whole, &cut)
}

pub(super) fn piece_is_enclosed(bin_cells: &[GridCell], piece_cells: &[GridCell]) -> bool {
    if piece_cells.len() >= bin_cells.len() {
        return false;
    }
    piece_cells.iter().all(|c| {
        [(1, 0), (-1, 0), (0, 1), (0, -1)].iter().all(|&(dx, dy)| {
            bin_cells.contains(&GridCell {
                x: c.x + dx,
                y: c.y + dy,
            })
        })
    })
}

/// Whether the cut actually divides this solid. A piece whose prism covers the
/// whole bin needs no cut, and a split line that misses a piece's material is a
/// no-op rather than an error -- an L-shaped bin needs that.
pub(super) fn straddles(solid: &Solid, cut: &Cut) -> bool {
    solid
        .verts
        .iter()
        .any(|v| cut.side_of_point(v.point) == Side::Negative)
}

pub fn try_build_pieces(p: &Params) -> Result<Vec<BinPiece>, String> {
    if p.mode == Mode::Baseplate {
        return Ok(vec![BinPiece {
            name: "gridfinity-baseplate.stl".into(),
            bin: 0,
            piece: 0,
            piece_count: 1,
            col: 0,
            row: 0,
            solid: build_baseplate(p),
        }]);
    }
    let bins: Vec<(usize, &LogicalBin)> = p
        .bins
        .iter()
        .enumerate()
        .filter(|(_, b)| !b.cells.is_empty())
        .collect();
    let mut out = Vec::new();
    for (ord, (bi, bin)) in bins.iter().enumerate() {
        let parts = partition_cells(&bin.cells, &bin.split_lines);
        let stem = if bins.len() == 1 {
            "gridfinity-bin".to_string()
        } else {
            format!("gridfinity-bin-{}", ord + 1)
        };
        let whole = build_bin_solid(p, &bin.cells, bin.slope)?;
        for (i, part) in parts.iter().enumerate() {
            let solid = carve_to_cells(&whole, &bin.cells, &part.cells)?;
            let name = if parts.len() == 1 {
                format!("{stem}.stl")
            } else {
                format!("{stem}-piece-{}-of-{}.stl", i + 1, parts.len())
            };
            out.push(BinPiece {
                name,
                bin: *bi,
                piece: i,
                piece_count: parts.len(),
                col: part.col,
                row: part.row,
                solid,
            });
        }
    }
    Ok(out)
}
