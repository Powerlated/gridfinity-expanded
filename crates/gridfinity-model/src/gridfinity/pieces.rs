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
//! bin, partition it, carve. `try_build_pieces_reporting` is the same path with
//! every bin's `BlendReport` summed, for a caller that needs to know whether the
//! rounding it asked for actually landed.

use super::*;
use gridfinity_brep::math::{Vec2, Vec3};
use gridfinity_brep::rectregion::{RectF, trace_rects};
use gridfinity_brep::sketch::point_in_polygon;
use gridfinity_brep::split::{Cut, Side, trim};
use gridfinity_brep::topo::Solid;
use crate::layout::{GridCell, GridFootprint, SplitLine, compartments, partition_cells};

/// Cut one printable piece out of the finished bin: keep the material inside the
/// vertical prism over the piece's cells. A piece is any connected polyomino, not
/// necessarily a grid slab, so this must follow the cell set itself -- trimming to
/// the piece's bounding box duplicates material wherever one piece's box covers
/// another piece's cells.
pub fn carve_to_cells(
    whole: &Solid,
    pitch: f64,
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
        RectF::new(c.x as f64 * pitch, c.y as f64 * pitch, pitch, pitch)
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
                (c.y + 1) as f64 * pitch
            } else {
                c.y as f64 * pitch - REENTRANT_FILLET_OVERHANG
            };
            rects.push(RectF::new(c.x as f64 * pitch, y, pitch, REENTRANT_FILLET_OVERHANG));
        }
    }
    let loops: Vec<Vec<Vec2>> = trace_rects(&rects, &[])
        .into_iter()
        .map(|lp| lp.pts)
        .collect();
    if loops.is_empty() {
        return Err("a piece traced no boundary".into());
    }
    let cut = Cut::prism(&loops, Vec3::Z)?;
    let piece = if straddles(whole, &cut) {
        trim(whole, &cut)?
    } else {
        whole.clone()
    };
    assert_piece_is_sound(&piece, piece_cells);
    Ok(piece)
}

/// Cut one printable piece out of the finished baseplate: keep the material
/// inside the vertical prism over the piece's cells, each cell reaching
/// `BASEPLATE_PRISM_REACH` into every neighbour the *bin* does not occupy.
///
/// That reach is the one difference from `carve_to_cells`, and it is what a
/// baseplate's outline forces. A plate is the full `GRID_PITCH` cell rectangle
/// where a bin is inset by `HALF_TOL`, so a prism traced from bare cell
/// rectangles puts its outer planes exactly in the plate's own outer faces --
/// a coincident-plane cut, which `trim` is never asked for on the bin path.
/// Growing outward only where the neighbour is outside the bin moves those
/// planes clear while leaving every interior seam plane exactly on its split
/// line, and it claims nothing: a baseplate's material lies wholly inside the
/// union of the bin's cell rectangles, so the ground the reach covers is empty.
///
/// `plate_margin_x`/`_y` are the plate's own, and each piece's rectangles are
/// grown by `plate_cell_rect` -- the same function `build_baseplate` traced the
/// outline from -- before the reach is applied, so the prism cannot disagree
/// with the material it is cutting. A flange side is a side with no neighbour in
/// `bin_cells`, so it is always a side the reach grows too, and the two asserts
/// below still hold.
pub fn carve_baseplate_to_cells(
    whole: &Solid,
    pitch: f64,
    bin_cells: &[GridCell],
    piece_cells: &[GridCell],
    plate_margin_x: f64,
    plate_margin_y: f64,
) -> Result<Solid, String> {
    if piece_cells.is_empty() {
        return Ok(whole.clone());
    }
    let grid = GridFootprint::from_cells(bin_cells)
        .expect("a plate with a piece to carve out of it has at least one cell");
    let mut rects: Vec<RectF> = Vec::with_capacity(piece_cells.len());
    for c in piece_cells {
        let flange = plate_cell_rect(*c, grid, pitch, plate_margin_x, plate_margin_y);
        let (mut x, mut y) = (flange.x, flange.y);
        let (mut w, mut h) = (flange.w, flange.h);
        let reach = baseplate_prism_reach(pitch);
        for (dx, dy) in [(-1i32, 0i32), (1, 0), (0, -1), (0, 1)] {
            let neighbour = GridCell {
                x: c.x + dx,
                y: c.y + dy,
            };
            if bin_cells.contains(&neighbour) {
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
    let loops: Vec<Vec<Vec2>> = trace_rects(&rects, &[])
        .into_iter()
        .map(|lp| lp.pts)
        .collect();
    if loops.is_empty() {
        return Err("a baseplate piece traced no boundary".into());
    }
    let covers = |c: &GridCell| {
        let centre = Vec2::new((c.x as f64 + 0.5) * pitch, (c.y as f64 + 0.5) * pitch);
        loops.iter().filter(|lp| point_in_polygon(lp, centre)).count() % 2 == 1
    };
    for c in piece_cells {
        assert!(
            covers(c),
            "the prism of a {}-cell baseplate piece does not cover its own cell ({}, {})",
            piece_cells.len(),
            c.x,
            c.y
        );
    }
    for c in bin_cells {
        assert!(
            piece_cells.contains(c) || !covers(c),
            "the prism of a baseplate piece reaches over cell ({}, {}), which belongs to another piece",
            c.x,
            c.y
        );
    }
    let cut = Cut::prism(&loops, Vec3::Z)?;
    let piece = if straddles(whole, &cut) {
        trim(whole, &cut)?
    } else {
        whole.clone()
    };
    assert_piece_is_sound(&piece, piece_cells);
    Ok(piece)
}

/// Asserts that `piece` is the sound, printable body its cell set describes.
///
/// A piece leaves here closed and manifold, geometrically sound under `audit`,
/// bounded by exactly one shell per connected island of `piece_cells` with
/// material inside every one of them, and carrying no vertex or edge that
/// nothing names. Cutting is the last thing that happens to a bin, so this is
/// the last point at which any of it can be stated -- past here the piece is
/// triangles or a transmit file, where a detached lump of material reads as
/// ordinary geometry.
///
/// The island count comes from `layout::compartments` over the cells with no
/// divider between any of them, which is the same connected-components pass the
/// cavity walk runs on: a piece cut into two arms must come back as two shells,
/// and a piece that is one polyomino must come back as one.
fn assert_piece_is_sound(piece: &Solid, piece_cells: &[GridCell]) {
    if let Err(e) = piece.validate() {
        panic!("a carved piece is not a closed manifold: {e}");
    }
    let audited = crate::audit(piece);
    assert!(
        audited.is_ok(),
        "a carved piece is not geometrically sound:
{audited}"
    );
    let orphan_verts = piece.orphan_vertices();
    assert!(
        orphan_verts.is_empty(),
        "a carved piece carries {} vertex(es) no edge names, the first at {:?}",
        orphan_verts.len(),
        piece.verts[orphan_verts[0]].point
    );
    let orphan_edges = piece.orphan_edges();
    assert!(
        orphan_edges.is_empty(),
        "a carved piece carries {} edge(s) no face uses, the first being edge {}",
        orphan_edges.len(),
        orphan_edges[0]
    );
    let shells = piece.shells();
    let voids: Vec<usize> = shells
        .iter()
        .enumerate()
        .filter(|(_, sh)| !sh.encloses_material)
        .map(|(i, _)| i)
        .collect();
    assert!(
        voids.is_empty(),
        "a carved piece encloses {} void(s) of {} shell(s); shell {} has the material outside it,          so it bounds a cavity sealed inside the part",
        voids.len(),
        shells.len(),
        voids[0]
    );
    let islands = compartments(piece_cells, &Default::default()).len();
    assert_eq!(
        shells.len(),
        islands,
        "a piece of {} cell(s) in {islands} island(s) is bounded by one shell per island, but          this one has {} -- material stands detached from the part",
        piece_cells.len(),
        shells.len()
    );
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


/// The baseplate over every bin's cells, cut into the pieces its bins' split
/// lines call for: built once and carved, exactly as a bin is, so a seam is a
/// boolean applied last rather than two plates authored side by side -- which
/// would round the convex corners the cut is supposed to leave square.
///
/// The lines are the union of every `LogicalBin::split_lines`, because a
/// baseplate is one body over `Params::all_cells` while the lines are stated
/// per bin; a `Params` carrying one bin, which is what `optimize` builds, is
/// simply that bin's lines. Every returned piece has passed
/// `assert_piece_is_sound`.
fn baseplate_pieces(p: &Params) -> Result<Vec<BinPiece>, String> {
    let cells = p.all_cells();
    if cells.is_empty() {
        return Ok(Vec::new());
    }
    let mut lines: Vec<SplitLine> = Vec::new();
    for bin in &p.bins {
        for line in &bin.split_lines {
            if !lines.contains(line) {
                lines.push(*line);
            }
        }
    }
    let parts = partition_cells(&cells, &lines);
    let whole = build_baseplate(p);
    let mut out = Vec::with_capacity(parts.len());
    for (i, part) in parts.iter().enumerate() {
        let name = if parts.len() == 1 {
            "gridfinity-baseplate.stl".to_string()
        } else {
            format!("gridfinity-baseplate-piece-{}-of-{}.stl", i + 1, parts.len())
        };
        out.push(BinPiece {
            name,
            bin: 0,
            piece: i,
            piece_count: parts.len(),
            col: part.col,
            row: part.row,
            solid: carve_baseplate_to_cells(
                &whole,
                p.pitch,
                &cells,
                &part.cells,
                p.plate_margin_x,
                p.plate_margin_y,
            )?,
        });
    }
    assert_eq!(
        out.len(),
        parts.len(),
        "a baseplate of {} cell(s) partitions into {} piece(s), so it must carve into as many",
        cells.len(),
        parts.len()
    );
    Ok(out)
}

pub fn try_build_pieces(p: &Params) -> Result<Vec<BinPiece>, String> {
    try_build_pieces_reporting(p).map(|(pieces, _)| pieces)
}

/// `try_build_pieces` plus what became of every bin's blends, summed across the
/// bins: how many rounds were asked for, how many the model could not resolve,
/// which edges it left sharp, and the first refusal that named a reason. A
/// caller that does not read this cannot tell a drawer that kept its floor
/// fillets from one that quietly gave them up, which is the whole difference
/// between a printable part and a printable part with sharp inside corners.
pub fn try_build_pieces_reporting(p: &Params) -> Result<(Vec<BinPiece>, BlendReport), String> {
    if p.mode == Mode::Baseplate {
        return Ok((baseplate_pieces(p)?, BlendReport::default()));
    }
    let bins: Vec<(usize, &LogicalBin)> = p
        .bins
        .iter()
        .enumerate()
        .filter(|(_, b)| !b.cells.is_empty())
        .collect();
    let mut out = Vec::new();
    let mut blends = BlendReport::default();
    for (ord, (bi, bin)) in bins.iter().enumerate() {
        let parts = partition_cells(&bin.cells, &bin.split_lines);
        let stem = if bins.len() == 1 {
            "gridfinity-bin".to_string()
        } else {
            format!("gridfinity-bin-{}", ord + 1)
        };
        let (whole, report) = build_bin_solid_reporting(p, &bin.cells, bin.slope, &bin.pockets)?;
        blends.requested += report.requested;
        blends.unresolved += report.unresolved;
        blends.dropped.extend(report.dropped);
        blends.refusal = blends.refusal.take().or(report.refusal);
        for (i, part) in parts.iter().enumerate() {
            let solid = carve_to_cells(&whole, p.pitch, &bin.cells, &part.cells)?;
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
    assert!(
        blends.made() + blends.unresolved + blends.dropped.len() == blends.requested,
        "the summed blend report claims {} made, {} unresolved and {} dropped of {} requested, \
         which is not a partition of the request",
        blends.made(),
        blends.unresolved,
        blends.dropped.len(),
        blends.requested
    );
    Ok((out, blends))
}
