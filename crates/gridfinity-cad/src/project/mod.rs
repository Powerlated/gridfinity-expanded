//! Fitting a drawer: a drawer's measurements and the objects to organise in it,
//! in, and one Gridfinity bin's cell set plus the dividers between its
//! compartments, out.
//!
//! The pipeline is `drawer` -> `pack` -> `walls`, over the plan geometry in
//! `rects`. `drawer` decides how many cells the drawer holds and which rectangle
//! inside the resulting bin may be filled; `pack` places every object instance
//! inside that rectangle as a claim inflated by its clearance and half a
//! divider; `walls` turns the boundaries between those claims into ordinary
//! free-form inner walls. Nothing here touches the kernel -- the result is a
//! `Params` a caller assembles, and the drawer is one bin whose compartments are
//! divided by `InnerWall`s like any other.

pub mod drawer;
pub mod pack;
pub mod rects;
pub mod walls;

pub use drawer::{DrawerGrid, MAX_GRID, MIN_DRAWER_MM, drawer_cells, drawer_grid, max_drawer_mm, packing_area, packing_inset};
pub use pack::{PackEffort, PackInput, PackObject, PackResult, PackSearch, Placement, pack_layout};
pub use rects::{Rect, Rotation, parts_bounds, parts_connected, union_area};
pub use walls::{MIN_GENERATED_WALL_LENGTH, Point2, Wall, WallReport, layout_walls, layout_walls_reporting};
