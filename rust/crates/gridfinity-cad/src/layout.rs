use crate::kernel::hash::FxHashSet;

pub const PITCH: i32 = 42;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct GridCell {
    pub x: i32,
    pub y: i32,
}

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

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Axis {
    #[cfg_attr(feature = "serde", serde(rename = "x"))]
    X,
    #[cfg_attr(feature = "serde", serde(rename = "y"))]
    Y,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SplitLine {
    pub axis: Axis,
    pub index: i32,
}

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

    pub fn mm(&self) -> (f32, f32) {
        (
            self.width_cells as f32 * PITCH as f32,
            self.depth_cells as f32 * PITCH as f32,
        )
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Piece {
    pub col: i32,
    pub row: i32,
    pub cells: Vec<GridCell>,
}

pub type CellSet = FxHashSet<GridCell>;

pub fn cell_set(cells: &[GridCell]) -> CellSet {
    cells.iter().copied().collect()
}

fn edge_neighbours(e: GridEdge) -> [GridCell; 2] {
    match e.orientation {
        Orientation::V => [GridCell { x: e.x - 1, y: e.y }, GridCell { x: e.x, y: e.y }],
        Orientation::H => [GridCell { x: e.x, y: e.y - 1 }, GridCell { x: e.x, y: e.y }],
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EdgeClass {
    Internal,
    Perimeter,
    None,
}

pub fn classify_edge(cells: &[GridCell], e: GridEdge) -> EdgeClass {
    classify_edge_in(&cell_set(cells), e)
}

/// Same classification against a set the caller already owns. `classify_edge`
/// rebuilds the set on every call, which is quadratic when a whole boundary is
/// classified against one bin.
pub fn classify_edge_in(s: &CellSet, e: GridEdge) -> EdgeClass {
    let [a, b] = edge_neighbours(e);
    match (s.contains(&a), s.contains(&b)) {
        (true, true) => EdgeClass::Internal,
        (true, false) | (false, true) => EdgeClass::Perimeter,
        (false, false) => EdgeClass::None,
    }
}

pub fn cell_edges(c: GridCell) -> [GridEdge; 4] {
    [
        GridEdge {
            x: c.x,
            y: c.y,
            orientation: Orientation::V,
        },
        GridEdge {
            x: c.x + 1,
            y: c.y,
            orientation: Orientation::V,
        },
        GridEdge {
            x: c.x,
            y: c.y,
            orientation: Orientation::H,
        },
        GridEdge {
            x: c.x,
            y: c.y + 1,
            orientation: Orientation::H,
        },
    ]
}

/// Every cell edge, classified once. A cell edge is visited by at most two
/// cells, so emitting only from the lower-coordinate side dedupes without a set.
fn scan_edges(cells: &[GridCell], s: &CellSet, want_internal: bool) -> Vec<GridEdge> {
    let mut v: Vec<GridEdge> = Vec::new();
    for &c in cells {
        for e in cell_edges(c) {
            let [a, b] = edge_neighbours(e);
            let (ina, inb) = (s.contains(&a), s.contains(&b));
            if (ina && inb) != want_internal {
                continue;
            }
            if ina && inb && a != c {
                continue;
            }
            v.push(e);
        }
    }
    sort_edges(&mut v);
    v
}

pub fn perimeter_edges(cells: &[GridCell]) -> Vec<GridEdge> {
    scan_edges(cells, &cell_set(cells), false)
}

pub fn internal_edges(cells: &[GridCell]) -> Vec<GridEdge> {
    scan_edges(cells, &cell_set(cells), true)
}

pub fn sort_edges(edges: &mut [GridEdge]) {
    edges.sort_by(|a, b| (a.orientation, a.x, a.y).cmp(&(b.orientation, b.x, b.y)));
}

#[derive(Clone, Debug, Default)]
pub struct EffectiveWalls {
    pub walled: FxHashSet<GridEdge>,
    pub open: FxHashSet<GridEdge>,
    pub dividers: FxHashSet<GridEdge>,
}

/// The empty cells a cell set encloses: those unreachable from outside its
/// bounding box by 4-connected moves through empty cells. A bin's cavity is
/// planned from its cell rects, so it stops at the pitch line of an enclosed
/// hole while the hole's own material boundary sits `HALF_TOL` beyond it. An
/// opening onto such a boundary would leave the island loop and the hole loop
/// crossing rather than nested, so `effective_walls` refuses to honour it.
pub fn enclosed_holes(cells: &[GridCell]) -> CellSet {
    let s = cell_set(cells);
    if s.is_empty() {
        return CellSet::default();
    }
    let lo = GridCell {
        x: cells
            .iter()
            .map(|c| c.x)
            .min()
            .expect("a non-empty cell set has a least x")
            - 1,
        y: cells
            .iter()
            .map(|c| c.y)
            .min()
            .expect("a non-empty cell set has a least y")
            - 1,
    };
    let hi = GridCell {
        x: cells
            .iter()
            .map(|c| c.x)
            .max()
            .expect("a non-empty cell set has a greatest x")
            + 1,
        y: cells
            .iter()
            .map(|c| c.y)
            .max()
            .expect("a non-empty cell set has a greatest y")
            + 1,
    };
    let mut outside = CellSet::default();
    let mut stack = vec![lo];
    outside.insert(lo);
    while let Some(c) = stack.pop() {
        for n in [
            GridCell { x: c.x - 1, y: c.y },
            GridCell { x: c.x + 1, y: c.y },
            GridCell { x: c.x, y: c.y - 1 },
            GridCell { x: c.x, y: c.y + 1 },
        ] {
            if n.x < lo.x || n.x > hi.x || n.y < lo.y || n.y > hi.y {
                continue;
            }
            if s.contains(&n) || !outside.insert(n) {
                continue;
            }
            stack.push(n);
        }
    }
    let mut holes = CellSet::default();
    for y in lo.y..=hi.y {
        for x in lo.x..=hi.x {
            let c = GridCell { x, y };
            if !s.contains(&c) && !outside.contains(&c) {
                holes.insert(c);
            }
        }
    }
    holes
}

pub fn effective_walls(
    piece_cells: &[GridCell],
    whole_bin_cells: &[GridCell],
    open_edges: &[GridEdge],
    divider_edges: &[GridEdge],
) -> EffectiveWalls {
    let holes = enclosed_holes(whole_bin_cells);
    let onto_hole = |e: GridEdge| edge_neighbours(e).iter().any(|c| holes.contains(c));
    let open_set: FxHashSet<GridEdge> = open_edges
        .iter()
        .copied()
        .filter(|&e| !onto_hole(e))
        .collect();
    let divider_set: FxHashSet<GridEdge> = divider_edges.iter().copied().collect();
    let mut out = EffectiveWalls::default();

    let piece_set = cell_set(piece_cells);
    let bin_set = cell_set(whole_bin_cells);
    for e in scan_edges(piece_cells, &piece_set, false) {
        match classify_edge_in(&bin_set, e) {
            EdgeClass::Internal => {
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
    for e in scan_edges(piece_cells, &piece_set, true) {
        if divider_set.contains(&e) {
            out.dividers.insert(e);
        }
    }
    out
}

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
        .filter(|(_, cs)| cs.iter().any(|c| s.contains(c)))
        .map(|((col, row), mut cs)| {
            cs.sort_by(|a, b| (a.x, a.y).cmp(&(b.x, b.y)));
            Piece {
                col,
                row,
                cells: cs,
            }
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
        let one = cells(&[(0, 0)]);
        assert_eq!(perimeter_edges(&one).len(), 4);
        assert!(internal_edges(&one).is_empty());
        let two = cells(&[(0, 0), (1, 0)]);
        assert_eq!(perimeter_edges(&two).len(), 6);
        assert_eq!(internal_edges(&two).len(), 1);
    }

    #[test]
    fn seam_edges_default_open() {
        let whole = cells(&[(0, 0), (1, 0)]);
        let piece = cells(&[(0, 0)]);
        let w = effective_walls(&piece, &whole, &[], &[]);
        let seam = GridEdge {
            x: 1,
            y: 0,
            orientation: Orientation::V,
        };
        assert!(w.open.contains(&seam));
        assert!(!w.dividers.contains(&seam));
    }

    #[test]
    fn seam_becomes_divider_when_requested() {
        let whole = cells(&[(0, 0), (1, 0)]);
        let piece = cells(&[(0, 0)]);
        let seam = GridEdge {
            x: 1,
            y: 0,
            orientation: Orientation::V,
        };
        let w = effective_walls(&piece, &whole, &[], &[seam]);
        assert!(w.walled.contains(&seam), "seam divider becomes a full wall");
        assert!(!w.open.contains(&seam));
        assert!(!w.dividers.contains(&seam));
    }

    #[test]
    fn open_edge_removes_perimeter_wall() {
        let whole = cells(&[(0, 0)]);
        let open_south = GridEdge {
            x: 0,
            y: 0,
            orientation: Orientation::H,
        };
        let w = effective_walls(&whole, &whole, &[open_south], &[]);
        assert!(w.open.contains(&open_south));
        assert_eq!(w.walled.len(), 3);
    }

    #[test]
    fn a_ring_encloses_its_middle_cell_and_a_c_shape_does_not() {
        let ring = cells(&[
            (0, 0),
            (1, 0),
            (2, 0),
            (0, 1),
            (2, 1),
            (0, 2),
            (1, 2),
            (2, 2),
        ]);
        assert_eq!(
            enclosed_holes(&ring).into_iter().collect::<Vec<_>>(),
            vec![GridCell { x: 1, y: 1 }]
        );
        let c_shape = cells(&[(0, 0), (1, 0), (2, 0), (0, 1), (2, 1), (0, 2), (2, 2)]);
        assert!(enclosed_holes(&c_shape).is_empty());
    }

    #[test]
    fn an_opening_onto_an_enclosed_hole_stays_walled() {
        let ring = cells(&[
            (0, 0),
            (1, 0),
            (2, 0),
            (0, 1),
            (2, 1),
            (0, 2),
            (1, 2),
            (2, 2),
        ]);
        let onto_hole = GridEdge {
            x: 1,
            y: 1,
            orientation: Orientation::V,
        };
        let onto_outside = GridEdge {
            x: 0,
            y: 0,
            orientation: Orientation::H,
        };
        let w = effective_walls(&ring, &ring, &[onto_hole, onto_outside], &[]);
        assert!(w.walled.contains(&onto_hole));
        assert!(!w.open.contains(&onto_hole));
        assert!(w.open.contains(&onto_outside));
    }

    #[test]
    fn partition_single_x_split() {
        let whole = cells(&[(0, 0), (1, 0), (2, 0)]);
        let pieces = partition_cells(
            &whole,
            &[SplitLine {
                axis: Axis::X,
                index: 1,
            }],
        );
        assert_eq!(pieces.len(), 2);
        assert_eq!(pieces[0].cells.len(), 1);
        assert_eq!(pieces[1].cells.len(), 2);
    }

    #[test]
    fn partition_stale_line_is_harmless() {
        let whole = cells(&[(0, 0), (1, 0)]);
        let pieces = partition_cells(
            &whole,
            &[SplitLine {
                axis: Axis::X,
                index: 10,
            }],
        );
        assert_eq!(pieces.len(), 1);
    }

    #[test]
    fn partition_two_axis_grid() {
        let whole = cells(&[(0, 0), (1, 0), (0, 1), (1, 1)]);
        let pieces = partition_cells(
            &whole,
            &[
                SplitLine {
                    axis: Axis::X,
                    index: 1,
                },
                SplitLine {
                    axis: Axis::Y,
                    index: 1,
                },
            ],
        );
        assert_eq!(pieces.len(), 4);
        for p in &pieces {
            assert_eq!(p.cells.len(), 1);
        }
    }
}
