//! Native OCCT Gridfinity baseplate construction.

use super::*;
use crate::kernel::KernelShape;
use crate::layout::{GridCell, GridFootprint};
use gridfinity_sketch::math::Vec2;
use gridfinity_sketch::nesting::containment;
use gridfinity_sketch::rectregion::{LoopStyle, RectF, shape_loop, trace_rects};
use gridfinity_sketch::sketch::{Aabb, Seg, loop_area, point_in_segs, reverse_loop, segs_bbox};

fn island_of(
    point: Vec2,
    outers: &[usize],
    loops: &[Vec<Seg>],
    containers: &[Vec<usize>],
) -> Option<usize> {
    outers
        .iter()
        .enumerate()
        .filter(|&(_, &l)| point_in_segs(point, &loops[l]))
        .max_by_key(|&(_, &l)| containers[l].len())
        .map(|(i, _)| i)
}

/// How far one cell's plate rectangle reaches past its own pitch square on each
/// of its four sides, as `(west, east, south, north)` millimetres: half the
/// plate margin wherever the cell sits on a side of `grid`'s bounding rectangle,
/// and nothing anywhere else.
///
/// The margins are *totals* per axis, split evenly between the two sides, so the
/// cell grid sits centred in the plate. A side facing into the grid's interior
/// -- a notch, an enclosed hole -- never grows, which is what keeps two pieces
/// of one plate from claiming the same empty cell. `grid` is the whole plate's
/// cell set even when `cell` belongs to one carved piece of it, so every piece
/// grows on the sides the finished plate grows on and on no others.
pub(super) fn plate_cell_overhang(
    cell: GridCell,
    grid: GridFootprint,
    margin_x: f64,
    margin_y: f64,
) -> (f64, f64, f64, f64) {
    assert!(
        margin_x >= 0.0 && margin_y >= 0.0,
        "a plate margin is how much wider than its grid the plate is, so it is not \
         {margin_x} x {margin_y} mm"
    );
    let (hx, hy) = (margin_x / 2.0, margin_y / 2.0);
    let (max_x, max_y) = (
        grid.min_x + grid.width_cells - 1,
        grid.min_y + grid.depth_cells - 1,
    );
    (
        if cell.x == grid.min_x { hx } else { 0.0 },
        if cell.x == max_x { hx } else { 0.0 },
        if cell.y == grid.min_y { hy } else { 0.0 },
        if cell.y == max_y { hy } else { 0.0 },
    )
}

/// One cell's plate rectangle: its pitch square grown by `plate_cell_overhang`.
pub(super) fn plate_cell_rect(
    cell: GridCell,
    grid: GridFootprint,
    pitch: f64,
    margin_x: f64,
    margin_y: f64,
) -> RectF {
    let (west, east, south, north) = plate_cell_overhang(cell, grid, margin_x, margin_y);
    let rect = RectF::new(
        cell.x as f64 * pitch - west,
        cell.y as f64 * pitch - south,
        pitch + west + east,
        pitch + south + north,
    );
    assert!(
        rect.x <= cell.x as f64 * pitch && rect.w >= pitch && rect.h >= pitch,
        "a plate rectangle grows a cell's pitch square outward, but cell ({}, {}) came back as \
         {rect:?}",
        cell.x,
        cell.y
    );
    rect
}

/// The plate declared by `p` as native OCCT solids: one rounded outline prism
/// per connected island, holed by the traced voids and by one four-section peg
/// socket per cell, with disjoint islands fused into the returned shape.
pub(super) fn build_baseplate_features<K: crate::kernel::FeatureKernel>(
    p: &Params,
) -> Result<K::Shape, String> {
    use crate::kernel::{Boolean, Profile};

    let cells = p.all_cells();
    if cells.is_empty() {
        return Err("a baseplate with no cells has no OCCT body".to_string());
    }
    let grid = GridFootprint::from_cells(&cells)
        .expect("a plate of at least one cell has a bounding rectangle");
    let traced = trace_rects(
        &cells
            .iter()
            .map(|c| plate_cell_rect(*c, grid, p.pitch, p.plate_margin_x, p.plate_margin_y))
            .collect::<Vec<_>>(),
        &[],
    );
    let inset = |_: usize, _: Vec2, _: Vec2| 0.0f64;
    let radius = |_: usize, convex: bool| if convex { OUTER_R } else { 0.0 };
    let outline: Vec<Vec<Seg>> = traced
        .iter()
        .map(|lp| {
            let segs = shape_loop(
                lp,
                &LoopStyle {
                    inset: &inset,
                    radius: &radius,
                },
            );
            if loop_area(&segs) < 0.0 && !lp.is_hole() {
                reverse_loop(&segs)
            } else {
                segs
            }
        })
        .collect();
    let bbox: Vec<Aabb> = outline.iter().map(|one| segs_bbox(one)).collect();
    let containers = containment(&outline, &bbox);
    let outers: Vec<usize> = (0..traced.len())
        .filter(|&i| !traced[i].is_hole())
        .collect();
    assert!(
        !outers.is_empty(),
        "a baseplate of {} cell(s) has no outer OCCT profile",
        cells.len()
    );

    let (w_bot, w_mid, w_top) = peg_widths(p.pitch);
    let mut result: Option<K::Shape> = None;
    for &outer_i in &outers {
        let holes: Vec<Vec<Seg>> = (0..traced.len())
            .filter(|&i| {
                traced[i].is_hole()
                    && island_of(outline[i][0].start(), &outers, &outline, &containers)
                        == outers.iter().position(|&candidate| candidate == outer_i)
            })
            .map(|i| outline[i].clone())
            .collect();
        let profile: Profile = std::iter::once(outline[outer_i].clone())
            .chain(holes)
            .collect();
        let mut island = K::prism(&profile, 0.0, PEG_HEIGHT)
            .map_err(|e| format!("OCCT could not extrude a baseplate island: {e}"))?;

        for &cell in &cells {
            let centre = Vec2::new(
                (cell.x as f64 + 0.5) * p.pitch,
                (cell.y as f64 + 0.5) * p.pitch,
            );
            let Some(owner) = island_of(centre, &outers, &outline, &containers) else {
                return Err(format!(
                    "cell ({}, {}) lies inside no OCCT baseplate island",
                    cell.x, cell.y
                ));
            };
            if outers[owner] != outer_i {
                continue;
            }
            let bottom = vec![peg_profile(cell, p.pitch, w_bot, PEG_R_BOTTOM)];
            let middle = vec![peg_profile(cell, p.pitch, w_mid, PEG_R_MID)];
            let top = vec![peg_profile(cell, p.pitch, w_top, OUTER_R)];
            let socket = K::loft(&[
                (bottom, 0.0),
                (middle.clone(), PEG_Z1),
                (middle, PEG_Z2),
                (top, PEG_HEIGHT),
            ])
            .map_err(|e| format!("OCCT could not loft a baseplate socket: {e}"))?;
            island = K::boolean(&island, &socket, Boolean::Cut)
                .map_err(|e| format!("OCCT could not cut a baseplate socket: {e}"))?;
        }
        result = Some(match result {
            Some(body) => K::boolean(&body, &island, Boolean::Fuse)
                .map_err(|e| format!("OCCT could not join baseplate islands: {e}"))?,
            None => island,
        });
    }
    let result = result.expect("at least one outer profile produces one OCCT island");
    if !result
        .is_valid()
        .map_err(|e| format!("OCCT could not validate the baseplate: {e}"))?
    {
        return Err("OCCT built an invalid baseplate".to_string());
    }
    Ok(result)
}

pub(super) fn build_baseplate_occt(p: &Params) -> Result<gridfinity_occt::Shape, String> {
    build_baseplate_features::<crate::kernel::OcctFeatures>(p)
}

#[cfg(all(test, feature = "occt"))]
mod occt_tests {
    use super::*;

    #[test]
    fn one_cell_is_one_valid_plate_with_one_open_socket() {
        let p = Params {
            mode: Mode::Baseplate,
            ..Params::rect(1, 1)
        };
        let plate = build_baseplate_occt(&p).expect("OCCT builds one plate cell");
        let bounds = plate.bounds().expect("bounds");
        assert!(
            (bounds.min[0] + 1e-7).abs() < 2e-7
                && (bounds.min[1] + 1e-7).abs() < 2e-7
                && (bounds.min[2] + 1e-7).abs() < 2e-7
                && (bounds.max[0] - (p.pitch + 1e-7)).abs() < 2e-7
                && (bounds.max[1] - (p.pitch + 1e-7)).abs() < 2e-7
                && (bounds.max[2] - (PEG_HEIGHT + 1e-7)).abs() < 2e-7,
            "one plate cell occupies its pitch square and peg height, got {bounds:?}"
        );
        let shells = plate.shell_volumes().expect("shell volumes");
        assert_eq!(shells.len(), 1, "one cell is one plate island");
        assert!(shells[0] > 0.0, "the plate shell encloses material");
        assert!(
            plate.volume().expect("volume") < p.pitch * p.pitch * PEG_HEIGHT,
            "the through socket removes material from the outline prism"
        );
    }

    #[test]
    fn separated_cell_islands_remain_two_printable_shells() {
        let p = Params {
            mode: Mode::Baseplate,
            bins: vec![LogicalBin {
                cells: vec![GridCell { x: 0, y: 0 }, GridCell { x: 2, y: 0 }],
                ..Default::default()
            }],
            ..Default::default()
        };
        let plate = build_baseplate_occt(&p).expect("OCCT builds both plate islands");
        let shells = plate.shell_volumes().expect("shell volumes");
        assert_eq!(shells.len(), 2, "two separated cells are two plate islands");
        assert!(
            shells.iter().all(|volume| *volume > 0.0),
            "each island shell encloses material: {shells:?}"
        );
    }
}
