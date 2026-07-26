//! Grid layout logic: cells, edges, split lines, and wall classification.
//!
//! Pure combinatorics — no geometry, no floats beyond the pitch constant. This
//! mirrors the edge/split semantics of the TypeScript reference so the same
//! `BinConfig` contracts (open/divider/seam edges, auto-split) port over.
//!
//! Edge convention (matches the reference):
//! - `V` edge at `(x,y)`: vertical segment on grid line `x·PITCH`, spanning
//!   `y·PITCH..(y+1)·PITCH`; separates cell `(x-1,y)` from cell `(x,y)`.
//! - `H` edge at `(x,y)`: horizontal segment on grid line `y·PITCH`, spanning
//!   `x·PITCH..(x+1)·PITCH`; separates cell `(x,y-1)` from cell `(x,y)`.

use std::collections::HashSet;

/// Canonical Gridfinity cell pitch (mm). Re-stated here so the layout logic is
/// self-contained; `gridfinity::GRID_PITCH` is the same value.
pub const PITCH: i32 = 42;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct GridCell {
    pub x: i32,
    pub y: i32,
}

/// Serialises as the reference's `'h' | 'v'`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Orientation {
    #[cfg_attr(feature = "serde", serde(rename = "h"))]
    H,
    #[cfg_attr(feature = "serde", serde(rename = "v"))]
    V,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct GridEdge {
    pub x: i32,
    pub y: i32,
    pub orientation: Orientation,
}

/// Serialises as the reference's `'x' | 'y'`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Axis {
    #[cfg_attr(feature = "serde", serde(rename = "x"))]
    X,
    #[cfg_attr(feature = "serde", serde(rename = "y"))]
    Y,
}

/// A split at a grid line. `Axis::X` → vertical line between columns `index-1`
/// and `index`; `Axis::Y` → horizontal line between rows `index-1` and `index`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SplitLine {
    pub axis: Axis,
    pub index: i32,
}

/// Axis-aligned footprint of a set of cells, in cell coordinates.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GridFootprint {
    pub min_x: i32,
    pub min_y: i32,
    pub width_cells: i32,
    pub depth_cells: i32,
}

impl GridFootprint {
    pub fn from_cells(cells: &[GridCell]) -> Option<GridFootprint> {
        if cells.is_empty() {
            return None;
        }
        let mut min_x = i32::MAX;
        let mut min_y = i32::MAX;
        let mut max_x = i32::MIN;
        let mut max_y = i32::MIN;
        for c in cells {
            min_x = min_x.min(c.x);
            min_y = min_y.min(c.y);
            max_x = max_x.max(c.x);
            max_y = max_y.max(c.y);
        }
        Some(GridFootprint {
            min_x,
            min_y,
            width_cells: max_x - min_x + 1,
            depth_cells: max_y - min_y + 1,
        })
    }

    /// Size in millimetres.
    pub fn mm(&self) -> (f32, f32) {
        (
            self.width_cells as f32 * PITCH as f32,
            self.depth_cells as f32 * PITCH as f32,
        )
    }
}

/// A contiguous piece produced by splitting a bin along `split_lines`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Piece {
    pub col: i32,
    pub row: i32,
    pub cells: Vec<GridCell>,
}

fn cell_set(cells: &[GridCell]) -> HashSet<GridCell> {
    cells.iter().copied().collect()
}

/// The two cells adjacent to an edge (in canonical order). For a perimeter edge
/// exactly one is present; for an internal edge both are.
fn edge_neighbours(e: GridEdge) -> [GridCell; 2] {
    match e.orientation {
        Orientation::V => [GridCell { x: e.x - 1, y: e.y }, GridCell { x: e.x, y: e.y }],
        Orientation::H => [GridCell { x: e.x, y: e.y - 1 }, GridCell { x: e.x, y: e.y }],
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EdgeClass {
    /// Both adjacent cells present — edge runs through the interior.
    Internal,
    /// Exactly one adjacent cell present — edge sits on the perimeter.
    Perimeter,
    /// Neither adjacent cell present — edge does not border the region.
    None,
}

pub fn classify_edge(cells: &[GridCell], e: GridEdge) -> EdgeClass {
    let s = cell_set(cells);
    let [a, b] = edge_neighbours(e);
    match (s.contains(&a), s.contains(&b)) {
        (true, true) => EdgeClass::Internal,
        (true, false) | (false, true) => EdgeClass::Perimeter,
        (false, false) => EdgeClass::None,
    }
}

/// The four canonical edges bounding a cell.
pub fn cell_edges(c: GridCell) -> [GridEdge; 4] {
    [
        GridEdge { x: c.x, y: c.y, orientation: Orientation::V },       // west
        GridEdge { x: c.x + 1, y: c.y, orientation: Orientation::V },    // east
        GridEdge { x: c.x, y: c.y, orientation: Orientation::H },        // south
        GridEdge { x: c.x, y: c.y + 1, orientation: Orientation::H },    // north
    ]
}

/// Perimeter edges of the region (exactly one adjacent cell present).
pub fn perimeter_edges(cells: &[GridCell]) -> Vec<GridEdge> {
    let s = cell_set(cells);
    let mut out = HashSet::new();
    for &c in cells {
        for e in cell_edges(c) {
            let [a, b] = edge_neighbours(e);
            let n = s.contains(&a) as u8 + s.contains(&b) as u8;
            if n == 1 {
                out.insert(e);
            }
        }
    }
    let mut v: Vec<GridEdge> = out.into_iter().collect();
    sort_edges(&mut v);
    v
}

/// Internal edges of the region (both adjacent cells present).
pub fn internal_edges(cells: &[GridCell]) -> Vec<GridEdge> {
    let s = cell_set(cells);
    let mut out = HashSet::new();
    for &c in cells {
        for e in cell_edges(c) {
            let [a, b] = edge_neighbours(e);
            if s.contains(&a) && s.contains(&b) {
                out.insert(e);
            }
        }
    }
    let mut v: Vec<GridEdge> = out.into_iter().collect();
    sort_edges(&mut v);
    v
}

pub fn sort_edges(edges: &mut [GridEdge]) {
    edges.sort_by(|a, b| {
        (a.orientation, a.x, a.y).cmp(&(b.orientation, b.x, b.y))
    });
}

/// How wall layout resolves for one piece of a bin.
#[derive(Clone, Debug, Default)]
pub struct EffectiveWalls {
    /// Perimeter edges that keep their outer wall.
    pub walled: HashSet<GridEdge>,
    /// Perimeter edges whose wall is removed (open to the outside), plus seam
    /// edges (between two pieces of the same bin) that are not dividers.
    pub open: HashSet<GridEdge>,
    /// Internal edges (within this piece, or seams) that carry a divider wall.
    pub dividers: HashSet<GridEdge>,
}

/// Resolve which perimeter edges are walled/open and which internal edges are
/// dividers for a piece, given the whole bin's cells and the user's open/divider
/// exceptions. Semantics mirror the reference:
/// - A piece-perimeter edge that is *internal to the whole bin* is a **seam**:
///   open by default (so glued pieces share a continuous cavity), walled iff a
///   divider sits on it.
/// - A true outer-perimeter edge is walled by default, open iff listed in
///   `open_edges`.
/// - A piece-internal edge is a divider iff listed in `divider_edges`.
pub fn effective_walls(
    piece_cells: &[GridCell],
    whole_bin_cells: &[GridCell],
    open_edges: &[GridEdge],
    divider_edges: &[GridEdge],
) -> EffectiveWalls {
    let open_set: HashSet<GridEdge> = open_edges.iter().copied().collect();
    let divider_set: HashSet<GridEdge> = divider_edges.iter().copied().collect();
    let mut out = EffectiveWalls::default();

    for e in perimeter_edges(piece_cells) {
        match classify_edge(whole_bin_cells, e) {
            EdgeClass::Internal => {
                // Seam between two pieces of the same bin. A divider placed on
                // a split line becomes a full wall on both adjacent pieces (it
                // is a piece-PERIMETER edge, so it gets a wall strip inset from
                // the seam plane, not a centred divider strip).
                if divider_set.contains(&e) {
                    out.walled.insert(e);
                } else {
                    out.open.insert(e);
                }
            }
            _ => {
                if open_set.contains(&e) {
                    out.open.insert(e);
                } else {
                    out.walled.insert(e);
                }
            }
        }
    }
    for e in internal_edges(piece_cells) {
        if divider_set.contains(&e) {
            out.dividers.insert(e);
        }
    }
    out
}

/// Number of split lines (along one axis) at or below coordinate `c`: this is
/// the chunk index a cell lands in along that axis.
fn chunk_index(c: i32, lines: &[i32]) -> i32 {
    lines.iter().filter(|&&l| l <= c).count() as i32
}

fn axis_lines(split_lines: &[SplitLine], axis: Axis) -> Vec<i32> {
    let mut v: Vec<i32> = split_lines
        .iter()
        .filter(|l| l.axis == axis)
        .map(|l| l.index)
        .collect();
    v.sort_unstable();
    v.dedup();
    v
}

/// Partition cells into pieces along the given split lines. Pieces are sorted
/// `(row, col)`; empty pieces are omitted. Lines that do not separate cells are
/// harmless (they just don't create extra chunks).
pub fn partition_cells(cells: &[GridCell], split_lines: &[SplitLine]) -> Vec<Piece> {
    let x_lines = axis_lines(split_lines, Axis::X);
    let y_lines = axis_lines(split_lines, Axis::Y);
    let s = cell_set(cells);
    let mut groups: std::collections::BTreeMap<(i32, i32), Vec<GridCell>> = Default::default();
    for &c in cells {
        let key = (chunk_index(c.x, &x_lines), chunk_index(c.y, &y_lines));
        groups.entry(key).or_default().push(c);
    }
    let mut out: Vec<Piece> = groups
        .into_iter()
        .filter(|(_, cs)| {
            // Keep only groups that actually contain real cells (a split line in
            // empty space yields no group anyway, but be defensive).
            cs.iter().any(|c| s.contains(c))
        })
        .map(|((col, row), mut cs)| {
            cs.sort_by(|a, b| (a.x, a.y).cmp(&(b.x, b.y)));
            Piece { col, row, cells: cs }
        })
        .collect();
    out.sort_by(|a, b| (a.row, a.col).cmp(&(b.row, b.col)));
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cells(coords: &[(i32, i32)]) -> Vec<GridCell> {
        coords.iter().map(|&(x, y)| GridCell { x, y }).collect()
    }

    #[test]
    fn footprint_of_2x2_block() {
        let f = GridFootprint::from_cells(&cells(&[(0, 0), (1, 0), (0, 1), (1, 1)])).unwrap();
        assert_eq!(f.width_cells, 2);
        assert_eq!(f.depth_cells, 2);
        assert_eq!(f.mm(), (84.0, 84.0));
    }

    #[test]
    fn perimeter_vs_internal_classification() {
        // Single 1×1 cell: all 4 edges are perimeter, none internal.
        let one = cells(&[(0, 0)]);
        assert_eq!(perimeter_edges(&one).len(), 4);
        assert!(internal_edges(&one).is_empty());
        // 2×1 block: 6 perimeter, 1 internal (the shared vertical edge at x=1).
        let two = cells(&[(0, 0), (1, 0)]);
        assert_eq!(perimeter_edges(&two).len(), 6);
        assert_eq!(internal_edges(&two).len(), 1);
    }

    #[test]
    fn seam_edges_default_open() {
        // Whole bin is a 2×1 block; split it at x=1. The piece-perimeter edge at
        // x=1 is internal to the whole bin → a seam → open by default.
        let whole = cells(&[(0, 0), (1, 0)]);
        let piece = cells(&[(0, 0)]);
        let w = effective_walls(&piece, &whole, &[], &[]);
        let seam = GridEdge { x: 1, y: 0, orientation: Orientation::V };
        assert!(w.open.contains(&seam));
        assert!(!w.dividers.contains(&seam));
    }

    #[test]
    fn seam_becomes_divider_when_requested() {
        let whole = cells(&[(0, 0), (1, 0)]);
        let piece = cells(&[(0, 0)]);
        let seam = GridEdge { x: 1, y: 0, orientation: Orientation::V };
        let w = effective_walls(&piece, &whole, &[], &[seam]);
        assert!(w.walled.contains(&seam), "seam divider becomes a full wall");
        assert!(!w.open.contains(&seam));
        assert!(!w.dividers.contains(&seam));
    }

    #[test]
    fn open_edge_removes_perimeter_wall() {
        let whole = cells(&[(0, 0)]);
        let open_south = GridEdge { x: 0, y: 0, orientation: Orientation::H };
        let w = effective_walls(&whole, &whole, &[open_south], &[]);
        assert!(w.open.contains(&open_south));
        assert_eq!(w.walled.len(), 3);
    }

    #[test]
    fn partition_single_x_split() {
        let whole = cells(&[(0, 0), (1, 0), (2, 0)]);
        let pieces = partition_cells(&whole, &[SplitLine { axis: Axis::X, index: 1 }]);
        assert_eq!(pieces.len(), 2);
        assert_eq!(pieces[0].cells.len(), 1); // col 0
        assert_eq!(pieces[1].cells.len(), 2); // cols 1..2
    }

    #[test]
    fn partition_stale_line_is_harmless() {
        // A split line at x=10 with no cells on either side creates no extra piece.
        let whole = cells(&[(0, 0), (1, 0)]);
        let pieces = partition_cells(&whole, &[SplitLine { axis: Axis::X, index: 10 }]);
        assert_eq!(pieces.len(), 1);
    }

    #[test]
    fn partition_two_axis_grid() {
        let whole = cells(&[(0, 0), (1, 0), (0, 1), (1, 1)]);
        let pieces = partition_cells(
            &whole,
            &[
                SplitLine { axis: Axis::X, index: 1 },
                SplitLine { axis: Axis::Y, index: 1 },
            ],
        );
        assert_eq!(pieces.len(), 4);
        for p in &pieces {
            assert_eq!(p.cells.len(), 1);
        }
    }
}
