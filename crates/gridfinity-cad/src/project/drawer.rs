//! The drawer itself: how many Gridfinity cells fit in it, how much of it is
//! left over, and which rectangle inside the resulting bin the packer may use.
//!
//! Packing is millimetre-space and only the bin outline is cell-quantized:
//! `drawer_grid` floors the drawer's measurements to whole 42 mm cells and
//! reports the leftover millimetres as unusable margin, `drawer_cells` spells
//! that grid out as the bin's cell set, and `packing_area` is the cavity
//! interior inset by `packing_inset` -- the perimeter clearance plus the
//! perimeter wall. That inset treats the cavity as a plain rectangle and ignores
//! its corner rounding, so an object hard into a corner leans on its clearance.
//! `MIN_DRAWER_MM` and `max_drawer_mm` are the two ends of the range a drawer
//! measurement is worth stating in, and both are properties of `drawer_grid`'s
//! own clamping.

use super::rects::Rect;
use crate::gridfinity::{GRID_PITCH, HALF_TOL};
use crate::layout::GridCell;

/// The most cells a drawer bin is allowed along one axis.
pub const MAX_GRID: u32 = 40;

/// The smallest drawer measurement worth stating, in mm: one grid cell, below
/// which `drawer_grid` floors to zero cells and the drawer holds nothing.
pub const MIN_DRAWER_MM: f64 = GRID_PITCH as f64;

/// The largest drawer measurement worth stating, in mm: the point past which
/// `drawer_grid` clamps to `max_grid` and every further millimetre becomes
/// unusable margin rather than another cell.
pub fn max_drawer_mm(max_grid: u32) -> f64 {
    max_grid as f64 * GRID_PITCH as f64
}

/// How a drawer's measurements resolve into a bin: whole cells along each axis,
/// and the millimetres left over that no cell covers.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DrawerGrid {
    pub cols: u32,
    pub rows: u32,
    pub margin_x: f64,
    pub margin_y: f64,
}

/// The largest cell grid that fits the drawer, capped at `max_grid` on each
/// axis, and the drawer millimetres it leaves unused. Every millimetre past the
/// cap is margin, so the margin is below one pitch exactly when the axis is not
/// capped.
pub fn drawer_grid(width_mm: f64, depth_mm: f64, max_grid: u32) -> DrawerGrid {
    let pitch = GRID_PITCH as f64;
    let cells = |mm: f64| -> u32 {
        if mm <= 0.0 {
            return 0;
        }
        ((mm / pitch).floor() as u32).min(max_grid)
    };
    let (cols, rows) = (cells(width_mm), cells(depth_mm));
    let grid = DrawerGrid {
        cols,
        rows,
        margin_x: width_mm - cols as f64 * pitch,
        margin_y: depth_mm - rows as f64 * pitch,
    };
    assert!(
        grid.margin_x >= 0.0 && grid.margin_y >= 0.0,
        "a drawer of {width_mm} x {depth_mm} mm resolved to {cols} x {rows} cells, which is more \
         drawer than there is"
    );
    assert!(
        grid.cols == max_grid || grid.margin_x < pitch,
        "{} mm of margin is a whole further cell the {cols}-column grid did not take",
        grid.margin_x
    );
    assert!(
        grid.rows == max_grid || grid.margin_y < pitch,
        "{} mm of margin is a whole further cell the {rows}-row grid did not take",
        grid.margin_y
    );
    grid
}

/// How far inside the bin's outline the cavity's usable rectangle starts: the
/// perimeter clearance each side of the pitch line, plus the perimeter wall
/// standing on it.
pub fn packing_inset(perimeter_thickness: f64) -> f64 {
    assert!(
        perimeter_thickness > 0.0,
        "a bin with a {perimeter_thickness} mm perimeter has no wall to inset the cavity by"
    );
    HALF_TOL as f64 + perimeter_thickness
}

/// The rectangle the packer may fill: the drawer bin's cavity interior, in the
/// same millimetre coordinates as the bin's cells, zero-sized when the wall
/// leaves nothing.
pub fn packing_area(grid: DrawerGrid, perimeter_thickness: f64) -> Rect {
    let inset = packing_inset(perimeter_thickness);
    let pitch = GRID_PITCH as f64;
    let area = Rect::new(
        inset,
        inset,
        (grid.cols as f64 * pitch - inset * 2.0).max(0.0),
        (grid.rows as f64 * pitch - inset * 2.0).max(0.0),
    );
    assert!(
        area.right() <= grid.cols as f64 * pitch && area.bottom() <= grid.rows as f64 * pitch,
        "the packing area {area:?} reaches past the {} x {} cell bin it is inside",
        grid.cols,
        grid.rows
    );
    area
}

/// The grid spelled out as the bin's cell set, row by row from the origin.
pub fn drawer_cells(grid: DrawerGrid) -> Vec<GridCell> {
    let mut cells = Vec::with_capacity((grid.cols * grid.rows) as usize);
    for y in 0..grid.rows as i32 {
        for x in 0..grid.cols as i32 {
            cells.push(GridCell { x, y });
        }
    }
    cells
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn floors_a_drawer_to_whole_cells_and_reports_the_rest_as_margin() {
        let grid = drawer_grid(400.0, 300.0, MAX_GRID);
        assert_eq!((grid.cols, grid.rows), (9, 7));
        assert!((grid.margin_x - (400.0 - 9.0 * 42.0)).abs() < 1e-9);
        assert!((grid.margin_y - (300.0 - 7.0 * 42.0)).abs() < 1e-9);
    }

    #[test]
    fn holds_nothing_below_one_cell() {
        let grid = drawer_grid(MIN_DRAWER_MM - 0.1, 300.0, MAX_GRID);
        assert_eq!(grid.cols, 0);
        assert!(drawer_cells(grid).is_empty());
    }

    #[test]
    fn turns_every_millimetre_past_the_cap_into_margin() {
        let over = max_drawer_mm(MAX_GRID) + 100.0;
        let grid = drawer_grid(over, over, MAX_GRID);
        assert_eq!((grid.cols, grid.rows), (MAX_GRID, MAX_GRID));
        assert!((grid.margin_x - 100.0).abs() < 1e-9);
    }

    #[test]
    fn insets_the_packing_area_by_the_clearance_and_the_wall() {
        let grid = drawer_grid(400.0, 300.0, MAX_GRID);
        let area = packing_area(grid, 1.2);
        assert_eq!(area.x, packing_inset(1.2));
        assert_eq!(area.width, 9.0 * 42.0 - 2.0 * packing_inset(1.2));
    }

    #[test]
    fn lists_one_cell_per_grid_position() {
        let grid = drawer_grid(400.0, 300.0, MAX_GRID);
        assert_eq!(drawer_cells(grid).len(), (grid.cols * grid.rows) as usize);
    }
}
