//! Native OCCT construction of Gridfinity bins.
//!
//! The two-dimensional planners remain kernel-neutral. This module consumes
//! their cell outline and cavity loops as volumetric OCCT operations, so no
//! legacy B-rep topology crosses into the native path.

use super::*;
use crate::kernel::{Boolean, FeatureKernel, FilletEdge, KernelShape, OcctFeatures, Profile};
use crate::layout::{GridCell, effective_walls};
use gridfinity_sketch::sketch::{Seg, Sketch, loop_area, reverse_loop};

fn slope_span(cells: &[GridCell], pitch: f64, ux: f64, uy: f64) -> (f64, f64) {
    let mut min_a = f64::INFINITY;
    let mut max_a = f64::NEG_INFINITY;
    for c in cells {
        for (dx, dy) in [(0, 0), (1, 0), (0, 1), (1, 1)] {
            let a = ux * (c.x + dx) as f64 * pitch + uy * (c.y + dy) as f64 * pitch;
            min_a = min_a.min(a);
            max_a = max_a.max(a);
        }
    }
    (min_a, (max_a - min_a).max(1e-6))
}

fn uphill_unit(dir: SlopeDir) -> (f64, f64) {
    match dir {
        SlopeDir::PlusX => (-1.0, 0.0),
        SlopeDir::MinusX => (1.0, 0.0),
        SlopeDir::PlusY => (0.0, -1.0),
        SlopeDir::MinusY => (0.0, 1.0),
    }
}

/// A closed, flat bin over `cells`, built as an OCCT outer prism and peg lofts
/// minus its planned cavity prisms and optional fastener bores.
pub(super) fn build_closed_flat_bin<K: FeatureKernel>(
    p: &Params,
    cells: &[GridCell],
    pockets: &[Pocket],
    slope: Option<BinSlope>,
) -> Result<K::Shape, String> {
    if cells.is_empty() {
        return Err("a bin with no cells has no OCCT body".to_string());
    }
    let walls = effective_walls(cells, cells, &p.open_edges, &p.divider_edges);
    let openish = !walls.open.is_empty();
    let inset = |_: &crate::layout::GridEdge| HALF_TOL;
    let walled = |edge: &crate::layout::GridEdge| walls.walled.contains(edge);
    let mut shared = SharedWithPegs::default();
    let outline: Vec<Vec<Seg>> = boundary_steps(cells)
        .iter()
        .map(|steps| {
            author_outer_loop(steps, p.pitch, &inset, &walled, &mut shared)
                .into_iter()
                .map(|piece| piece.seg)
                .collect()
        })
        .collect();
    let mut body = prisms_of_region::<K>(&outline, PEG_HEIGHT, p.total_height() - PEG_HEIGHT)?;

    let (w_bot, w_mid, w_top) = peg_widths(p.pitch);
    let mut pegs = Vec::with_capacity(cells.len());
    for &cell in cells {
        let bottom = vec![peg_profile(cell, p.pitch, w_bot, PEG_R_BOTTOM)];
        let middle = vec![peg_profile(cell, p.pitch, w_mid, PEG_R_MID)];
        let top = vec![peg_profile(cell, p.pitch, w_top, OUTER_R)];
        let mut peg = K::loft(&[
            (bottom, 0.0),
            (middle.clone(), PEG_Z1),
            (middle, PEG_Z2),
            (top, PEG_HEIGHT),
        ])
        .map_err(|e| format!("OCCT could not loft a bin peg: {e}"))?;
        for tool in fastener_tools::<K>(p, cell)? {
            peg = K::boolean(&peg, &tool, Boolean::Cut)
                .map_err(|e| format!("OCCT could not cut a peg fastener recess: {e}"))?;
        }
        pegs.push(peg);
    }

    let wt = buildable_wall_thickness(p.wall_thickness, openish, slope.is_some());
    let rc = p.cavity_corner_radius.max((OUTER_R - wt).max(0.0));
    let fr = buildable_floor_fillet(
        p.floor_fillet,
        p.total_height() - (BASE_TOTAL_HEIGHT + FLOOR_THICKNESS),
        rc,
        slope.is_some(),
    );
    let ramp = slope.map(|sl| {
        let (ux, uy) = uphill_unit(sl.dir);
        let (min_a, span) = slope_span(cells, p.pitch, ux, uy);
        let depth = p.total_height() - (BASE_TOTAL_HEIGHT + FLOOR_THICKNESS);
        let rise = (sl
            .angle_deg
            .to_radians()
            .tan()
            .clamp(0.0, MAX_SLOPE_GRADIENT)
            * span)
            .min(depth - SLOPE_RIM_HEADROOM)
            .max(0.0);
        let gradient = if span > MIN_SLOPE_SPAN {
            rise / span
        } else {
            0.0
        };
        let origin = gridfinity_sketch::math::Vec3::new(
            0.0,
            0.0,
            BASE_TOTAL_HEIGHT + FLOOR_THICKNESS - gradient * min_a,
        );
        let normal =
            gridfinity_sketch::math::Vec3::new(-gradient * ux, -gradient * uy, 1.0).normalize();
        (origin, normal)
    });
    let cavities = if pockets.is_empty() {
        walked_cavity(cells, p.pitch, &walls, wt)
    } else {
        pocket_cavity(pockets)
    };
    let mut wall_tools = Vec::new();
    let spans = open_spans(cells, p.pitch, &walls);
    for (outer, holes) in cavities {
        let outer = if openish {
            shape_cavity_loop_open(&outer, rc, fr, &spans)
        } else {
            shape_cavity_loop(&outer, rc, fr)
        };
        let holes: Vec<Vec<Seg>> = holes
            .iter()
            .map(|hole| shape_cavity_loop(hole, rc, fr))
            .collect();
        let cavity: Profile = std::iter::once(outer.clone()).chain(holes).collect();
        let mut tool = K::prism(
            &cavity,
            BASE_TOTAL_HEIGHT + FLOOR_THICKNESS,
            p.total_height() - (BASE_TOTAL_HEIGHT + FLOOR_THICKNESS),
        )
        .map_err(|e| format!("OCCT could not extrude a bin cavity: {e}"))?;
        if let Some((origin, normal)) = ramp {
            tool = K::cut_half_space(
                &tool,
                [origin.x, origin.y, origin.z],
                [-normal.x, -normal.y, -normal.z],
            )
            .map_err(|e| format!("OCCT could not trim a cavity to its sloped floor: {e}"))?;
        }
        body = K::boolean(&body, &tool, Boolean::Cut)
            .map_err(|e| format!("OCCT could not hollow a bin cavity: {e}"))?;
        for wall in p.inner_walls.iter().filter(|_| !openish && slope.is_none()) {
            let Some(loop_) = inner_wall_quad_in(wall, fr, &outer) else {
                continue;
            };
            let height = wall
                .height
                .unwrap_or_else(|| p.total_height() - (BASE_TOTAL_HEIGHT + FLOOR_THICKNESS))
                .min(p.total_height() - (BASE_TOTAL_HEIGHT + FLOOR_THICKNESS));
            if height <= 0.0 {
                continue;
            }
            wall_tools.push(
                K::prism(&vec![loop_], BASE_TOTAL_HEIGHT + FLOOR_THICKNESS, height)
                    .map_err(|e| format!("OCCT could not extrude a free-form wall: {e}"))?,
            );
        }
    }
    for wall in wall_tools {
        body = K::boolean(&body, &wall, Boolean::Fuse)
            .map_err(|e| format!("OCCT could not join a free-form wall: {e}"))?;
    }

    if fr > MIN_USEFUL_BLEND {
        let floor_z = BASE_TOTAL_HEIGHT + FLOOR_THICKNESS;
        let bounds = K::bounds(&body)
            .map_err(|e| format!("OCCT could not bound the bin before filleting it: {e}"))?;
        let edges: Vec<FilletEdge> = K::edge_midpoints(&body)
            .map_err(|e| format!("OCCT could not list the cavity edges: {e}"))?
            .into_iter()
            .filter(|mid| {
                (mid[2] - floor_z).abs() < 1e-7
                    && (!openish
                        || (mid[0] - bounds.min[0]).abs() > 2e-7
                            && (mid[0] - bounds.max[0]).abs() > 2e-7
                            && (mid[1] - bounds.min[1]).abs() > 2e-7
                            && (mid[1] - bounds.max[1]).abs() > 2e-7)
            })
            .map(|midpoint| FilletEdge {
                midpoint,
                radius: fr,
            })
            .collect();
        assert!(
            !edges.is_empty(),
            "a bin asking for a {fr} mm floor fillet has no edge on its floor at z={floor_z}"
        );
        body = K::fillet(&body, &edges, 1e-7)
            .map_err(|e| format!("OCCT could not fillet the cavity floor: {e}"))?;
    }
    for peg in pegs {
        body = K::boolean(&body, &peg, Boolean::Fuse)
            .map_err(|e| format!("OCCT could not join a bin peg: {e}"))?;
    }
    if !body
        .is_valid()
        .map_err(|e| format!("OCCT could not validate the bin: {e}"))?
    {
        return Err("OCCT built an invalid bin".to_string());
    }
    Ok(body)
}

pub(super) fn build_closed_flat_bin_occt(
    p: &Params,
    cells: &[GridCell],
    pockets: &[Pocket],
    slope: Option<BinSlope>,
) -> Result<gridfinity_occt::Shape, String> {
    build_closed_flat_bin::<OcctFeatures>(p, cells, pockets, slope)
}

pub(super) fn prisms_of_region<K: FeatureKernel>(
    loops: &[Vec<Seg>],
    z: f64,
    height: f64,
) -> Result<K::Shape, String> {
    let mut outers: Vec<(Vec<Seg>, Vec<Vec<Seg>>)> = loops
        .iter()
        .filter(|one| loop_area(one) > 0.0)
        .map(|one| (one.clone(), Vec::new()))
        .collect();
    for hole in loops.iter().filter(|one| loop_area(one) < 0.0) {
        let point = hole[0].start();
        let owner = outers
            .iter()
            .enumerate()
            .filter(|(_, (outer, _))| gridfinity_sketch::sketch::point_in_segs(point, outer))
            .min_by(|(_, (a, _)), (_, (b, _))| {
                loop_area(a)
                    .partial_cmp(&loop_area(b))
                    .expect("finite outline areas compare")
            })
            .map(|(i, _)| i)
            .ok_or_else(|| "an OCCT outline hole lies inside no material island".to_string())?;
        outers[owner].1.push(reverse_loop(hole));
    }
    let mut result: Option<K::Shape> = None;
    for (outer, holes) in outers {
        let profile: Profile = std::iter::once(outer).chain(holes).collect();
        let island = K::prism(&profile, z, height)
            .map_err(|e| format!("OCCT could not extrude a bin outline: {e}"))?;
        result = Some(match result {
            Some(body) => K::boolean(&body, &island, Boolean::Fuse)
                .map_err(|e| format!("OCCT could not join bin islands: {e}"))?,
            None => island,
        });
    }
    result.ok_or_else(|| "a bin outline contains no material island".to_string())
}

fn fastener_tools<K: FeatureKernel>(p: &Params, cell: GridCell) -> Result<Vec<K::Shape>, String> {
    let sections: Vec<(f64, f64)> = match (p.magnet_holes, p.screw_holes) {
        (true, true) => vec![(MAGNET_RADIUS, MAGNET_DEPTH), (SCREW_RADIUS, SCREW_DEPTH)],
        (true, false) => vec![(MAGNET_RADIUS, MAGNET_DEPTH)],
        (false, true) => vec![(SCREW_RADIUS, SCREW_DEPTH)],
        (false, false) => return Ok(Vec::new()),
    };
    let centre = gridfinity_sketch::math::Vec2::new(
        (cell.x as f64 + 0.5) * p.pitch,
        (cell.y as f64 + 0.5) * p.pitch,
    );
    let inset = fastener_inset(p.pitch);
    let mut tools = Vec::new();
    for (dx, dy) in FASTENER_QUADRANTS {
        for &(radius, depth) in &sections {
            let loop_ = Sketch::circle(centre.x + dx * inset, centre.y + dy * inset, radius)
                .loops
                .remove(0);
            tools.push(
                K::prism(&vec![loop_], 0.0, depth)
                    .map_err(|e| format!("OCCT could not extrude a fastener recess: {e}"))?,
            );
        }
    }
    Ok(tools)
}

#[cfg(all(test, feature = "occt"))]
mod tests {
    use super::*;

    #[test]
    fn default_bin_is_a_native_valid_body_with_one_material_shell() {
        let p = Params::default();
        let bin = build_closed_flat_bin_occt(&p, &p.bins[0].cells, &[], None)
            .expect("OCCT builds the bin");
        let shells = bin.shell_volumes().expect("shell volumes");
        assert_eq!(shells.len(), 1, "the default bin is one printable body");
        assert!(shells[0] > 0.0, "the default bin shell encloses material");
        let bounds = bin.bounds().expect("bounds");
        assert!(
            (bounds.max[0] - bounds.min[0] - 83.5).abs() < 4e-7
                && (bounds.max[1] - bounds.min[1] - 83.5).abs() < 4e-7
                && bounds.min[2].abs() < 2e-7
                && (bounds.max[2] - p.total_height()).abs() < 2e-7,
            "the native default bin occupies its declared box, got {bounds:?}"
        );
    }

    #[test]
    fn native_dividers_add_material_between_compartments() {
        let plain = Params::rect(2, 1);
        let divided = Params {
            divider_edges: vec![crate::layout::GridEdge {
                x: 1,
                y: 0,
                orientation: crate::layout::Orientation::V,
            }],
            ..plain.clone()
        };
        let a =
            build_closed_flat_bin_occt(&plain, &plain.bins[0].cells, &[], None).expect("plain bin");
        let b = build_closed_flat_bin_occt(&divided, &divided.bins[0].cells, &[], None)
            .expect("divided bin");
        assert!(
            b.volume().expect("divided volume") > a.volume().expect("plain volume"),
            "a divider leaves material standing between the two cavities"
        );
    }

    #[test]
    fn native_fastener_recesses_remove_material_from_the_pegs() {
        let plain = Params::rect(1, 1);
        let holed = Params {
            magnet_holes: true,
            screw_holes: true,
            ..plain.clone()
        };
        let a =
            build_closed_flat_bin_occt(&plain, &plain.bins[0].cells, &[], None).expect("plain bin");
        let b =
            build_closed_flat_bin_occt(&holed, &holed.bins[0].cells, &[], None).expect("holed bin");
        assert!(
            b.volume().expect("holed volume") < a.volume().expect("plain volume"),
            "fastener recesses subtract material from a peg"
        );
    }

    #[test]
    fn native_stated_pockets_leave_everything_else_as_material() {
        let walked = Params::rect(2, 1);
        let pocketed = Params {
            bins: vec![LogicalBin {
                cells: walked.bins[0].cells.clone(),
                pockets: vec![Pocket {
                    x: 10.0,
                    y: 10.0,
                    width: 20.0,
                    depth: 20.0,
                }],
                ..Default::default()
            }],
            ..walked.clone()
        };
        let a = try_build_occt(&walked).expect("walked cavity");
        let b = try_build_occt(&pocketed).expect("stated pocket");
        assert!(
            b.volume().expect("pocketed volume") > a.volume().expect("walked volume"),
            "a small stated pocket leaves more surrounding material than a cell-wide cavity"
        );
    }

    #[test]
    fn native_full_and_partial_free_form_walls_add_material() {
        let plain = Params::rect(2, 2);
        let walled = Params {
            inner_walls: vec![
                InnerWall {
                    x1: 42.0,
                    y1: 12.0,
                    x2: 42.0,
                    y2: 72.0,
                    width: 2.0,
                    height: None,
                },
                InnerWall {
                    x1: 12.0,
                    y1: 42.0,
                    x2: 72.0,
                    y2: 42.0,
                    width: 2.0,
                    height: Some(6.0),
                },
            ],
            ..plain.clone()
        };
        let a = try_build_occt(&plain).expect("plain native bin");
        let b = try_build_occt(&walled).expect("walled native bin");
        assert!(
            b.volume().expect("walled volume") > a.volume().expect("plain volume"),
            "full- and partial-height walls add material inside the cavity"
        );
        assert!(b.is_valid().expect("validity"), "the walled bin is valid");
    }

    #[test]
    fn native_wall_opening_removes_material_and_keeps_one_valid_shell() {
        let closed = Params::rect(1, 1);
        let opened = Params {
            open_edges: vec![crate::layout::GridEdge {
                x: 0,
                y: 0,
                orientation: crate::layout::Orientation::H,
            }],
            ..closed.clone()
        };
        let a = try_build_occt(&closed).expect("closed native bin");
        let b = try_build_occt(&opened).expect("opened native bin");
        assert!(
            b.volume().expect("opened volume") < a.volume().expect("closed volume"),
            "opening a perimeter wall removes material"
        );
        let shells = b.shell_volumes().expect("shell volumes");
        assert_eq!(shells.len(), 1, "the opened bin remains one body");
        assert!(shells[0] > 0.0, "its shell still encloses material");
    }

    #[test]
    fn native_slope_raises_the_floor_without_changing_the_outer_box() {
        let flat = Params::rect(2, 1);
        let sloped = Params {
            bins: vec![LogicalBin {
                cells: flat.bins[0].cells.clone(),
                slope: Some(BinSlope {
                    angle_deg: 15.0,
                    dir: SlopeDir::PlusX,
                }),
                ..Default::default()
            }],
            ..flat.clone()
        };
        let a = try_build_occt(&flat).expect("flat native bin");
        let b = try_build_occt(&sloped).expect("sloped native bin");
        assert!(
            b.volume().expect("sloped volume") > a.volume().expect("flat volume"),
            "raising the cavity floor adds the ramp wedge to the bin"
        );
        let flat_bounds = a.bounds().expect("flat bounds");
        let slope_bounds = b.bounds().expect("sloped bounds");
        assert!(
            flat_bounds
                .min
                .iter()
                .chain(&flat_bounds.max)
                .zip(slope_bounds.min.iter().chain(&slope_bounds.max))
                .all(|(flat, sloped)| (flat - sloped).abs() < 1e-6),
            "the ramp changes the cavity and not the outside: {flat_bounds:?} against {slope_bounds:?}"
        );
    }

    #[test]
    fn native_bin_piece_prisms_conserve_the_whole_body() {
        let p = Params {
            bins: vec![LogicalBin {
                cells: vec![GridCell { x: 0, y: 0 }, GridCell { x: 1, y: 0 }],
                split_lines: vec![crate::layout::SplitLine {
                    axis: crate::layout::Axis::X,
                    index: 1,
                }],
                ..Default::default()
            }],
            ..Default::default()
        };
        let whole = try_build_occt(&p).expect("whole native bin");
        let pieces = try_build_pieces_occt(&p).expect("native pieces");
        assert_eq!(pieces.len(), 2, "one split line makes two pieces");
        let sum: f64 = pieces
            .iter()
            .map(|piece| piece.solid.volume().expect("piece volume"))
            .sum();
        let volume = whole.volume().expect("whole volume");
        assert!(
            (sum - volume).abs() < volume * 1e-9,
            "the pieces hold {sum} mm3 of the whole body's {volume} mm3"
        );
    }

    #[test]
    fn native_baseplate_piece_prisms_keep_the_outer_flange() {
        let p = Params {
            mode: Mode::Baseplate,
            plate_margin_x: 12.0,
            bins: vec![LogicalBin {
                cells: vec![GridCell { x: 0, y: 0 }, GridCell { x: 1, y: 0 }],
                split_lines: vec![crate::layout::SplitLine {
                    axis: crate::layout::Axis::X,
                    index: 1,
                }],
                ..Default::default()
            }],
            ..Default::default()
        };
        let whole = try_build_occt(&p).expect("whole native plate");
        let pieces = try_build_pieces_occt(&p).expect("native plate pieces");
        assert_eq!(pieces.len(), 2, "one split line makes two plate pieces");
        let sum: f64 = pieces
            .iter()
            .map(|piece| piece.solid.volume().expect("piece volume"))
            .sum();
        let volume = whole.volume().expect("whole volume");
        assert!(
            (sum - volume).abs() < volume * 1e-9,
            "the flanged pieces hold {sum} mm3 of the plate's {volume} mm3"
        );
    }
}
