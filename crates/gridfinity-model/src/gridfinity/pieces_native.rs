//! Native OCCT carving of finished bodies into printable pieces.

use super::*;
use crate::layout::{GridCell, GridFootprint, compartments, partition_cells};
use gridfinity_sketch::math::Vec2;
use gridfinity_sketch::rectregion::{RectF, trace_rects};

/// One printable OCCT body, named and positioned in the same partition as a
/// the model's printable-body metadata.
pub struct BinPiece<S> {
    pub name: String,
    pub bin: usize,
    pub piece: usize,
    pub piece_count: usize,
    pub col: i32,
    pub row: i32,
    pub solid: S,
}
pub type OcctBinPiece = BinPiece<gridfinity_occt::Shape>;
pub type AnalyticBinPiece = BinPiece<gridfinity_brep::Shape>;

/// Intersects `whole` with the vertical OCCT prism over `piece_cells`, using
/// the same reentrant-corner reach as legacy carving, and returns one positive
/// material shell per connected island of the piece.
pub fn carve_to_cells_features<K: crate::kernel::FeatureKernel>(
    whole: &K::Shape,
    pitch: f64,
    bin_cells: &[GridCell],
    piece_cells: &[GridCell],
) -> Result<K::Shape, String> {
    let cell_rect = |c: &GridCell| RectF::new(c.x as f64 * pitch, c.y as f64 * pitch, pitch, pitch);
    let mut rects: Vec<RectF> = piece_cells.iter().map(cell_rect).collect();
    for c in piece_cells {
        for step in [-1i32, 1] {
            let neighbour = GridCell {
                x: c.x,
                y: c.y + step,
            };
            if !bin_cells.contains(&neighbour) {
                let y = if step > 0 {
                    (c.y + 1) as f64 * pitch
                } else {
                    c.y as f64 * pitch - REENTRANT_FILLET_OVERHANG
                };
                rects.push(RectF::new(
                    c.x as f64 * pitch,
                    y,
                    pitch,
                    REENTRANT_FILLET_OVERHANG,
                ));
            }
        }
    }
    carve_with_rects::<K>(whole, &rects, piece_cells)
}

fn carve_baseplate_to_cells<K: crate::kernel::FeatureKernel>(
    whole: &K::Shape,
    p: &Params,
    bin_cells: &[GridCell],
    piece_cells: &[GridCell],
) -> Result<K::Shape, String> {
    let grid = GridFootprint::from_cells(bin_cells)
        .expect("a native baseplate piece belongs to a nonempty plate");
    let mut rects = Vec::with_capacity(piece_cells.len());
    for c in piece_cells {
        let flange = plate_cell_rect(*c, grid, p.pitch, p.plate_margin_x, p.plate_margin_y);
        let (mut x, mut y, mut w, mut h) = (flange.x, flange.y, flange.w, flange.h);
        let reach = baseplate_prism_reach(p.pitch);
        for (dx, dy) in [(-1i32, 0i32), (1, 0), (0, -1), (0, 1)] {
            if bin_cells.contains(&GridCell {
                x: c.x + dx,
                y: c.y + dy,
            }) {
                continue;
            }
            match (dx, dy) {
                (-1, 0) => {
                    x -= reach;
                    w += reach;
                }
                (1, 0) => w += reach,
                (0, -1) => {
                    y -= reach;
                    h += reach;
                }
                _ => h += reach,
            }
        }
        rects.push(RectF::new(x, y, w, h));
    }
    carve_with_rects::<K>(whole, &rects, piece_cells)
}

fn carve_with_rects<K: crate::kernel::FeatureKernel>(
    whole: &K::Shape,
    rects: &[RectF],
    piece_cells: &[GridCell],
) -> Result<K::Shape, String> {
    use crate::kernel::Boolean;
    use gridfinity_sketch::rectregion::{LoopStyle, shape_loop};
    use gridfinity_sketch::sketch::loop_area;

    let zero = |_: usize, _: Vec2, _: Vec2| 0.0;
    let loops: Vec<Vec<gridfinity_sketch::sketch::Seg>> = trace_rects(rects, &[])
        .iter()
        .map(|lp| {
            let loop_ = shape_loop(
                lp,
                &LoopStyle {
                    inset: &zero,
                    radius: &|_, _| 0.0,
                },
            );
            if lp.is_hole() && loop_area(&loop_) > 0.0 {
                gridfinity_sketch::sketch::reverse_loop(&loop_)
            } else if !lp.is_hole() && loop_area(&loop_) < 0.0 {
                gridfinity_sketch::sketch::reverse_loop(&loop_)
            } else {
                loop_
            }
        })
        .collect();
    let bounds = K::bounds(whole)
        .map_err(|e| format!("OCCT could not bound a body before carving it: {e}"))?;
    let z = bounds.min[2] - 1.0;
    let tool = prisms_of_region::<K>(&loops, z, bounds.max[2] - bounds.min[2] + 2.0)?;
    let piece = K::boolean(whole, &tool, Boolean::Common)
        .map_err(|e| format!("OCCT could not carve a printable piece: {e}"))?;
    let shells = K::shell_volumes(&piece)
        .map_err(|e| format!("OCCT could not inspect a carved piece: {e}"))?;
    let islands = compartments(piece_cells, &Default::default()).len();
    if shells.len() != islands || shells.iter().any(|volume| *volume <= 0.0) {
        return Err(format!(
            "an OCCT piece of {} cell(s) in {islands} island(s) has shell volumes {shells:?}",
            piece_cells.len()
        ));
    }
    Ok(piece)
}

/// Every printable piece declared by `p` as an OCCT body, built whole first and
/// intersected with each partition's exact cell-region prism. No legacy solid
/// is constructed or converted on this path.
pub fn try_build_pieces_features<K: crate::kernel::FeatureKernel>(
    p: &Params,
) -> Result<Vec<BinPiece<K::Shape>>, String> {
    assert!(
        p.pitch > 0.0,
        "an OCCT piece prism is measured in a positive grid pitch, not {} mm",
        p.pitch
    );
    if p.mode == Mode::Baseplate {
        let cells = p.all_cells();
        if cells.is_empty() {
            return Ok(Vec::new());
        }
        let mut lines = Vec::new();
        for line in p.bins.iter().flat_map(|bin| &bin.split_lines) {
            if !lines.contains(line) {
                lines.push(*line);
            }
        }
        let parts = partition_cells(&cells, &lines);
        let whole = build_baseplate_features::<K>(p)?;
        return parts
            .iter()
            .enumerate()
            .map(|(i, part)| {
                let name = if parts.len() == 1 {
                    "gridfinity-baseplate.stl".to_string()
                } else {
                    format!(
                        "gridfinity-baseplate-piece-{}-of-{}.stl",
                        i + 1,
                        parts.len()
                    )
                };
                Ok(BinPiece {
                    name,
                    bin: 0,
                    piece: i,
                    piece_count: parts.len(),
                    col: part.col,
                    row: part.row,
                    solid: carve_baseplate_to_cells::<K>(&whole, p, &cells, &part.cells)?,
                })
            })
            .collect();
    }

    let bins: Vec<(usize, &LogicalBin)> = p
        .bins
        .iter()
        .enumerate()
        .filter(|(_, bin)| !bin.cells.is_empty())
        .collect();
    let mut out = Vec::new();
    for (ord, (bi, bin)) in bins.iter().enumerate() {
        let parts = partition_cells(&bin.cells, &bin.split_lines);
        let stem = if bins.len() == 1 {
            "gridfinity-bin".to_string()
        } else {
            format!("gridfinity-bin-{}", ord + 1)
        };
        let whole = build_closed_flat_bin::<K>(p, &bin.cells, &bin.pockets, bin.slope)?;
        for (i, part) in parts.iter().enumerate() {
            out.push(BinPiece {
                name: if parts.len() == 1 {
                    format!("{stem}.stl")
                } else {
                    format!("{stem}-piece-{}-of-{}.stl", i + 1, parts.len())
                },
                bin: *bi,
                piece: i,
                piece_count: parts.len(),
                col: part.col,
                row: part.row,
                solid: carve_to_cells_features::<K>(&whole, p.pitch, &bin.cells, &part.cells)?,
            });
        }
    }
    Ok(out)
}

pub fn try_build_pieces_occt(p: &Params) -> Result<Vec<OcctBinPiece>, String> {
    try_build_pieces_features::<crate::kernel::OcctFeatures>(p)
}
