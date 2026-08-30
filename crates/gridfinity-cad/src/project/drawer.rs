//! The drawer itself: how many Gridfinity cells fit in it, how much of it is
//! left over, and which rectangle inside the resulting bin the packer may use.
//!
//! Packing is millimetre-space and only the bin outline is cell-quantized:
//! `drawer_grid` floors the drawer's measurements to whole cells of the pitch it
//! is given -- `GRID_PITCH` for a standard grid, whatever the run asked for
//! otherwise -- and
//! reports the leftover millimetres as unusable margin, `drawer_cells` spells
//! that grid out as the bin's cell set, and `packing_area` is the cavity
//! interior inset by `packing_inset` -- the perimeter clearance plus the
//! perimeter wall. That inset treats the cavity as a plain rectangle and ignores
//! its corner rounding, so an object hard into a corner leans on its clearance.
//! `MIN_DRAWER_MM` and `max_drawer_mm` are the two ends of the range a drawer
//! measurement is worth stating in, and both are properties of `drawer_grid`'s
//! own clamping.

use super::rects::Rect;
use crate::gridfinity::HALF_TOL;
use crate::layout::GridCell;

/// The most cells a drawer bin is allowed along one axis.
pub const MAX_GRID: u32 = 40;

/// The smallest drawer measurement worth stating, in mm: one grid cell of
/// `pitch`, below which `drawer_grid` floors to zero cells and the drawer holds
/// nothing.
pub fn min_drawer_mm(pitch: f64) -> f64 {
    pitch
}

/// The largest drawer measurement worth stating, in mm: the point past which
/// `drawer_grid` clamps to `max_grid` and every further millimetre becomes
/// unusable margin rather than another cell.
pub fn max_drawer_mm(max_grid: u32, pitch: f64) -> f64 {
    max_grid as f64 * pitch
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
pub fn drawer_grid(width_mm: f64, depth_mm: f64, max_grid: u32, pitch: f64) -> DrawerGrid {
    assert!(pitch > 0.0, "a grid pitch is a positive number of millimetres, not {pitch}");
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
pub fn packing_area(grid: DrawerGrid, perimeter_thickness: f64, pitch: f64) -> Rect {
    let inset = packing_inset(perimeter_thickness);
    assert!(pitch > 0.0, "a grid pitch is a positive number of millimetres, not {pitch}");
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

/// The cavity of a bin standing on `cells`, as the rectangles whose union it is:
/// each cell's own square, drawn back by `inset` on every side whose neighbour is
/// not a cell of the bin.
///
/// No inset on a shared side, so the squares abut across the interior and the
/// union is one region rather than a grid of separate pockets. A side with no
/// neighbour is a side where the bin's outline stands, and the perimeter wall
/// plus its clearance -- which is what `inset` is -- stands inside it, so the
/// union is exactly the space a compartment may occupy. For a full `cols x rows`
/// rectangle of cells that union is `packing_area` itself, which the tests pin:
/// the two must not drift, one being the other's rectangular case.
pub fn cavity_region(cells: &[GridCell], pitch: f64, inset: f64) -> Vec<Rect> {
    assert!(pitch > 0.0, "a grid pitch is a positive number of millimetres, not {pitch}");
    assert!(
        inset >= 0.0 && inset * 2.0 < pitch,
        "a {inset} mm perimeter inset leaves no cavity in a {pitch} mm cell"
    );
    let held = |x: i32, y: i32| cells.contains(&GridCell { x, y });
    cells
        .iter()
        .map(|cell| {
            let west = if held(cell.x - 1, cell.y) { 0.0 } else { inset };
            let east = if held(cell.x + 1, cell.y) { 0.0 } else { inset };
            let south = if held(cell.x, cell.y - 1) { 0.0 } else { inset };
            let north = if held(cell.x, cell.y + 1) { 0.0 } else { inset };
            Rect::new(
                f64::from(cell.x) * pitch + west,
                f64::from(cell.y) * pitch + south,
                pitch - west - east,
                pitch - south - north,
            )
        })
        .collect()
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
    use crate::gridfinity::GRID_PITCH;

    /// A full rectangle of cells has `packing_area` for its cavity: every
    /// interior side is shared and takes no inset, so the squares merge into the
    /// one rectangle the packer is handed.
    #[test]
    fn a_rectangle_of_cells_is_the_packing_area() {
        let grid = drawer_grid(300.0, 210.0, MAX_GRID, GRID_PITCH);
        let region = cavity_region(&drawer_cells(grid), GRID_PITCH, packing_inset(1.2));
        let area = packing_area(grid, 1.2, GRID_PITCH);
        let bounds = super::super::rects::parts_bounds(&region);
        assert!(
            (bounds.x - area.x).abs() < 1e-9
                && (bounds.y - area.y).abs() < 1e-9
                && (bounds.width - area.width).abs() < 1e-9
                && (bounds.depth - area.depth).abs() < 1e-9,
            "the cavity of a full rectangle of cells spans {bounds:?}, not its packing area {area:?}"
        );
        let covered: f64 = super::super::rects::union_area(&region);
        assert!(
            (covered - area.area()).abs() < 1e-6,
            "the cavity of a {} x {} cell bin is {covered} mm2 where its packing area is {} mm2",
            grid.cols,
            grid.rows,
            area.area()
        );
    }

    /// A cell with no neighbour on a side is drawn back from it by the whole
    /// inset, and a cell that has one is not, which is what makes the two
    /// squares of an arm one span rather than two pockets.
    #[test]
    fn a_shared_side_carries_no_inset() {
        let cells = [GridCell { x: 0, y: 0 }, GridCell { x: 1, y: 0 }];
        let region = cavity_region(&cells, GRID_PITCH, 1.45);
        assert_eq!(region[0], Rect::new(1.45, 1.45, GRID_PITCH - 1.45, GRID_PITCH - 2.9));
        assert_eq!(
            region[1],
            Rect::new(GRID_PITCH, 1.45, GRID_PITCH - 1.45, GRID_PITCH - 2.9)
        );
    }

    /// A drawer measured in cells of half the standard pitch: the same 400 mm
    /// holds twice as many.
    #[test]
    fn a_finer_pitch_takes_more_cells_out_of_the_same_drawer() {
        let half = GRID_PITCH / 2.0;
        let grid = drawer_grid(400.0, 300.0, MAX_GRID, half);
        assert_eq!((grid.cols, grid.rows), (19, 14));
        assert!((grid.margin_x - (400.0 - 19.0 * half)).abs() < 1e-9);
        let area = packing_area(grid, 1.2, half);
        assert!(
            (area.width - (19.0 * half - 2.0 * packing_inset(1.2))).abs() < 1e-9,
            "the packing area follows the pitch the grid was measured in, not {area:?}"
        );
    }

    #[test]
    fn floors_a_drawer_to_whole_cells_and_reports_the_rest_as_margin() {
        let grid = drawer_grid(400.0, 300.0, MAX_GRID, GRID_PITCH);
        assert_eq!((grid.cols, grid.rows), (9, 7));
        assert!((grid.margin_x - (400.0 - 9.0 * 42.0)).abs() < 1e-9);
        assert!((grid.margin_y - (300.0 - 7.0 * 42.0)).abs() < 1e-9);
    }

    #[test]
    fn holds_nothing_below_one_cell() {
        let grid = drawer_grid(min_drawer_mm(GRID_PITCH) - 0.1, 300.0, MAX_GRID, GRID_PITCH);
        assert_eq!(grid.cols, 0);
        assert!(drawer_cells(grid).is_empty());
    }

    #[test]
    fn turns_every_millimetre_past_the_cap_into_margin() {
        let over = max_drawer_mm(MAX_GRID, GRID_PITCH) + 100.0;
        let grid = drawer_grid(over, over, MAX_GRID, GRID_PITCH);
        assert_eq!((grid.cols, grid.rows), (MAX_GRID, MAX_GRID));
        assert!((grid.margin_x - 100.0).abs() < 1e-9);
    }

    #[test]
    fn insets_the_packing_area_by_the_clearance_and_the_wall() {
        let grid = drawer_grid(400.0, 300.0, MAX_GRID, GRID_PITCH);
        let area = packing_area(grid, 1.2, GRID_PITCH);
        assert_eq!(area.x, packing_inset(1.2));
        assert_eq!(area.width, 9.0 * 42.0 - 2.0 * packing_inset(1.2));
    }

    #[test]
    fn lists_one_cell_per_grid_position() {
        let grid = drawer_grid(400.0, 300.0, MAX_GRID, GRID_PITCH);
        assert_eq!(drawer_cells(grid).len(), (grid.cols * grid.rows) as usize);
    }
}
