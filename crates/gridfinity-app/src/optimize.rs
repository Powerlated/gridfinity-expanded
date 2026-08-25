//! The `optimize` subcommand: a TOML describing a drawer and the objects to
//! organise in it, in; printable geometry and an account of how it was reached,
//! out.
//!
//! `Args` is what the command line says, `Run` is everything one invocation
//! produced, and `fit` is the pipeline between them. The output path is checked
//! before any of it runs, because discovering it cannot be written after minutes
//! of packing and building is the same mistake reported far too late. `fit`: resolve the drawer to a
//! cell grid and a packing rectangle, pack the objects into it, turn the
//! boundaries between them into dividers, split the bin for the printer's bed,
//! build every piece, and read back what each piece is made of. Each stage's
//! result is kept on the `Run` rather than recomputed, so `report` prints what
//! was actually built. `run` is the whole
//! invocation and returns the `View` to show when `--view` was given -- the bin
//! and the boxes of the objects it was cut for -- so the window opens on the fit
//! already in memory rather than on a file written for
//! it. Failure at any stage is an `Err` and never a partial file -- `fit` runs
//! under the app's own `catch`, so an invariant the kernel asserts while
//! building arrives as a named error and exit 1 rather than a backtrace, and it
//! fires before the writer has touched anything.

use crate::export::{self, Format};
use crate::input::{self, Spec};
use crate::report;
use gridfinity_cad::gridfinity::{self, BinPiece, InnerWall, LogicalBin, Mode, Params};
use gridfinity_cad::kernel::math::Vec3;
use gridfinity_cad::kernel::program::BlendReport;
use gridfinity_cad::kernel::topo::Solid;
use gridfinity_cad::layout::{GridCell, Piece, SplitLine, partition_cells};
use gridfinity_cad::printers::compute_auto_split_lines;
use gridfinity_cad::project::drawer::{DrawerGrid, MAX_GRID, drawer_cells, drawer_grid, packing_area};
use gridfinity_cad::project::pack::{PackInput, PackResult, pack_layout};
use gridfinity_cad::project::rects::Rect;
use gridfinity_cad::project::walls::{WallReport, layout_walls_reporting};
use std::path::PathBuf;
use std::time::{Duration, Instant};

/// What one `optimize` invocation was asked to do.
#[derive(Debug, PartialEq, Eq)]
pub struct Args {
    pub input: PathBuf,
    pub format: Format,
    pub output: PathBuf,
    pub view: bool,
}

/// Everything one invocation produced, in the order the pipeline produced it.
pub struct Run {
    pub spec: Spec,
    pub grid: DrawerGrid,
    pub area: Rect,
    pub cells: Vec<GridCell>,
    pub result: PackResult,
    pub wall_report: WallReport,
    pub params: Params,
    pub split_lines: Vec<SplitLine>,
    pub parts: Vec<Piece>,
    pub pieces: Vec<BinPiece>,
    pub blends: BlendReport,
    pub soundness: Vec<PieceSoundness>,
    pub pack_time: Duration,
    pub build_time: Duration,
    pub export_time: Duration,
}

impl Run {
    /// How far each object's boxes were grown to become the area it claimed: its
    /// clearance plus the half divider standing on the claim boundary.
    pub fn claim_margin(&self) -> f64 {
        self.spec.clearance + self.spec.divider_thickness / 2.0
    }
}

/// One placed object's box, in the bin's own millimetre coordinates: what the
/// packer reserved for it, standing on the cavity floor.
///
/// `fits` is whether the object's stated height clears the cavity, which is the
/// same question the report's warnings answer in words -- a box that does not
/// fit still stands its full height, poking out of the bin it was packed into,
/// because hiding that is hiding the warning.
pub struct ObjectBox {
    pub min: Vec3,
    pub max: Vec3,
    pub fits: bool,
}

/// Everything `--view` opens the window on: the bin to rebuild, and the boxes of
/// the objects it was cut for.
pub struct View {
    pub params: Params,
    pub boxes: Vec<ObjectBox>,
}

/// Every placed instance's boxes, lifted from the packer's millimetre rectangles
/// into the bin's own coordinates: each part rectangle standing on the cavity
/// floor, rising by the height its object declared, or filling the cavity when
/// it declared none.
///
/// `packing_area` is already in the bin's coordinates, so a placement's
/// rectangles need no transform -- the rotation the packer chose is baked into
/// the rectangles themselves. An object made of several boxes contributes one
/// box per part rather than one bounding box over all of them, so an L-shaped
/// object reads as the L it is and not as the rectangle around it.
fn object_boxes(run: &Run) -> Vec<ObjectBox> {
    let depth = run.spec.cavity_depth();
    let floor = gridfinity::BASE_TOTAL_HEIGHT + gridfinity::FLOOR_THICKNESS;
    let mut out = Vec::new();
    for placement in &run.result.placements {
        let object = run
            .spec
            .objects
            .iter()
            .find(|o| o.pack.id == placement.object_id)
            .unwrap_or_else(|| {
                panic!(
                    "the packer placed {:?}, which is not one of the {} objects it was given",
                    placement.object_id,
                    run.spec.objects.len()
                )
            });
        let height = object.height.unwrap_or(depth);
        for part in &placement.parts {
            out.push(ObjectBox {
                min: Vec3::new(part.x, part.y, floor),
                max: Vec3::new(part.right(), part.bottom(), floor + height),
                fits: height <= depth,
            });
        }
    }
    out
}

/// What one built piece is made of, and what the audit had to say that was not
/// serious enough to stop it.
///
/// Every field here is read off a piece that has already passed the gate in
/// `carve_to_cells`, so `shells` is the piece's island count and the audit
/// carried no errors. It is printed so a run says out loud that the check
/// happened and on what -- a silent gate and a missing gate read the same.
pub struct PieceSoundness {
    pub name: String,
    pub shells: usize,
    pub faces: usize,
    pub edges: usize,
    pub verts: usize,
    pub warnings: usize,
}

/// What each built piece is made of, in the order the model built them.
fn soundness_of(pieces: &[BinPiece]) -> Vec<PieceSoundness> {
    pieces
        .iter()
        .map(|piece| {
            let solid: &Solid = &piece.solid;
            assert!(
                solid.orphan_vertices().is_empty() && solid.orphan_edges().is_empty(),
                "{} reached the report with geometry nothing names, which the carve gate refuses",
                piece.name
            );
            PieceSoundness {
                name: piece.name.clone(),
                shells: solid.shells().len(),
                faces: solid.faces.len(),
                edges: solid.edges.len(),
                verts: solid.verts.len(),
                warnings: gridfinity_cad::audit(solid).warnings().count(),
            }
        })
        .collect()
}

/// The command line as `Args`, or a message saying what is wrong with it. The
/// leading `optimize` has already been recognised by the caller.
pub fn parse_args(rest: &[String]) -> Result<Args, String> {
    let mut positional: Vec<String> = Vec::new();
    let mut format: Option<Format> = None;
    let mut view = false;
    let mut it = rest.iter();
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--view" => view = true,
            "--format" => {
                let name = it
                    .next()
                    .ok_or_else(|| "--format needs a value: stl or parasolid_x_t".to_string())?;
                format = Some(Format::from_name(name).ok_or_else(|| {
                    format!("--format {name:?} is not a format; it is stl or parasolid_x_t")
                })?);
            }
            other if other.starts_with("--") => {
                return Err(format!("{other:?} is not an option of `optimize`"));
            }
            other => positional.push(other.to_string()),
        }
    }
    let [input, output] = positional.as_slice() else {
        return Err(format!(
            "`optimize` takes an input file and an output path, but was given {} of them",
            positional.len()
        ));
    };
    Ok(Args {
        input: PathBuf::from(input),
        format: format.ok_or_else(|| "--format is required: stl or parasolid_x_t".to_string())?,
        output: PathBuf::from(output),
        view,
    })
}

/// The `Params` a fitted drawer builds as: one bin covering the drawer's cells,
/// carrying the generated dividers and cut for the printer's bed.
fn drawer_params(
    spec: &Spec,
    cells: &[GridCell],
    walls: &[InnerWall],
    splits: &[SplitLine],
) -> Params {
    Params {
        bins: vec![LogicalBin {
            cells: cells.to_vec(),
            split_lines: splits.to_vec(),
            slope: None,
        }],
        height_units: spec.height_units,
        wall_thickness: spec.wall_thickness,
        cavity_corner_radius: spec.fillet_radius,
        floor_fillet: spec.fillet_radius,
        magnet_holes: spec.magnets,
        screw_holes: spec.screws,
        open_edges: Vec::new(),
        divider_edges: Vec::new(),
        inner_walls: walls.to_vec(),
        mode: Mode::Bin,
    }
}

/// The whole pipeline for one validated run: pack, divide, split, build.
fn fit(spec: Spec) -> Result<Run, String> {
    let grid = drawer_grid(spec.drawer_width, spec.drawer_depth, MAX_GRID);
    if grid.cols == 0 || grid.rows == 0 {
        return Err(format!(
            "a drawer of {} x {} mm does not hold one 42 mm Gridfinity cell",
            spec.drawer_width, spec.drawer_depth
        ));
    }
    let cells = drawer_cells(grid);
    let area = packing_area(grid, spec.wall_thickness);

    let started = Instant::now();
    let result = pack_layout(PackInput {
        area,
        objects: spec.pack_objects(),
        divider_thickness: spec.divider_thickness,
        clearance: spec.clearance,
        effort: spec.effort,
    });
    let pack_time = started.elapsed();

    let (walls, wall_report) =
        layout_walls_reporting(&result.placements, &area, spec.divider_thickness);
    let inner_walls: Vec<InnerWall> = walls.iter().map(|w| w.to_inner_wall()).collect();
    let split_lines = compute_auto_split_lines(&cells, spec.printer);
    let params = drawer_params(&spec, &cells, &inner_walls, &split_lines);
    let parts = partition_cells(&cells, &split_lines);

    let started = Instant::now();
    let (pieces, blends) = gridfinity::try_build_pieces_reporting(&params)?;
    let build_time = started.elapsed();
    assert_eq!(
        pieces.len(),
        parts.len(),
        "the model built {} pieces for a partition of {} -- the report would name the wrong cells",
        pieces.len(),
        parts.len()
    );

    let soundness = soundness_of(&pieces);

    Ok(Run {
        spec,
        grid,
        area,
        cells,
        result,
        wall_report,
        params,
        split_lines,
        parts,
        pieces,
        blends,
        soundness,
        pack_time,
        build_time,
        export_time: Duration::ZERO,
    })
}

/// One `optimize` invocation, end to end: read, fit, write, report. Returns the
/// bin to open a window on, and the object boxes to draw in it, when `--view`
/// was given, and `None` otherwise.
pub fn run(rest: &[String]) -> Result<Option<View>, String> {
    let args = parse_args(rest)?;
    args.format.check_output(&args.output)?;
    let text = std::fs::read_to_string(&args.input)
        .map_err(|e| format!("could not read {}: {e}", args.input.display()))?;
    let spec = input::parse(&text).map_err(|e| format!("{}: {e}", args.input.display()))?;
    let mut run = crate::catch(|| fit(spec))?;

    let started = Instant::now();
    let written = match args.format {
        Format::Stl => export::write_stl_dir(&args.output, &run.pieces)?,
        Format::ParasolidXt => vec![export::write_xt(&args.output, &run.pieces)?],
    };
    run.export_time = started.elapsed();

    report::print(&run, &written);
    Ok(args.view.then(|| View {
        boxes: object_boxes(&run),
        params: run.params,
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn args(argv: &[&str]) -> Result<Args, String> {
        parse_args(&argv.iter().map(|s| s.to_string()).collect::<Vec<String>>())
    }

    #[test]
    fn reads_the_documented_invocation() {
        let a = args(&["in.toml", "--format", "stl", "out"]).expect("valid invocation");
        assert_eq!(a.input, PathBuf::from("in.toml"));
        assert_eq!(a.output, PathBuf::from("out"));
        assert_eq!(a.format, Format::Stl);
        assert!(!a.view);
    }

    #[test]
    fn takes_view_anywhere_among_the_arguments() {
        let a = args(&["--view", "in.toml", "--format", "parasolid_x_t", "out.x_t"])
            .expect("valid invocation");
        assert!(a.view);
        assert_eq!(a.format, Format::ParasolidXt);
    }

    #[test]
    fn refuses_an_invocation_missing_its_format() {
        let err = args(&["in.toml", "out"]).expect_err("--format is required");
        assert!(err.contains("--format"), "{err}");
    }

    #[test]
    fn refuses_a_format_that_is_not_wired() {
        let err = args(&["in.toml", "--format", "step", "out"])
            .expect_err("STEP is not an export format here");
        assert!(err.contains("step"), "{err}");
    }

    #[test]
    fn refuses_an_invocation_missing_a_path() {
        let err = args(&["in.toml", "--format", "stl"]).expect_err("both paths are required");
        assert!(err.contains("input file and an output path"), "{err}");
    }

    /// A drawer small enough to build quickly and busy enough to need dividers:
    /// two cells square, four objects that tile it.
    const SMALL: &str = "\
[drawer]
width = 84
depth = 84

[settings]
effort = \"quick\"

[[objects]]
name = \"block\"
quantity = 4
size = [34, 34]
";

    #[test]
    fn fits_a_drawer_end_to_end() {
        let spec = input::parse(SMALL).expect("the fixture is a valid run");
        let run = fit(spec).expect("a two-cell drawer of four blocks builds");

        assert_eq!((run.grid.cols, run.grid.rows), (2, 2));
        assert_eq!(run.cells.len(), 4);
        assert_eq!(
            run.result.placements.len(),
            4,
            "four 34 mm blocks fit an 84 mm drawer"
        );
        assert!(
            !run.result.walls.is_empty(),
            "four compartments in one bin need dividers between them"
        );
        assert_eq!(run.wall_report.generated, run.result.walls.len());
    }

    /// The same drawer, with the heights stated: one block that clears the
    /// cavity and one that does not.
    const TALL: &str = "[drawer]
width = 84
depth = 84

[settings]
effort = \"quick\"
height_units = 3

[[objects]]
name = \"low\"
quantity = 2
size = [34, 34, 5]

[[objects]]
name = \"high\"
quantity = 2
size = [34, 34, 200]
";

    #[test]
    fn every_placed_object_stands_on_the_cavity_floor_inside_the_packing_area() {
        let spec = input::parse(TALL).expect("the fixture is a valid run");
        let run = fit(spec).expect("a two-cell drawer of four blocks builds");
        let boxes = object_boxes(&run);

        assert_eq!(
            boxes.len(),
            run.result.placements.len(),
            "each of these objects is a single box, so it contributes exactly one"
        );
        let floor = gridfinity::BASE_TOTAL_HEIGHT + gridfinity::FLOOR_THICKNESS;
        for b in &boxes {
            assert!(
                (b.min.z - floor).abs() < 1e-9,
                "a packed object stands on the cavity floor at {floor}, not at {}",
                b.min.z
            );
            assert!(
                b.min.x >= run.area.x
                    && b.min.y >= run.area.y
                    && b.max.x <= run.area.right()
                    && b.max.y <= run.area.bottom(),
                "the packer placed {b_min:?}..{b_max:?} outside the {area:?} it packs into",
                b_min = b.min,
                b_max = b.max,
                area = run.area
            );
        }
        let too_tall: Vec<&ObjectBox> = boxes.iter().filter(|b| !b.fits).collect();
        assert_eq!(
            too_tall.len(),
            2,
            "the two 200 mm objects do not clear a {} mm cavity",
            run.spec.cavity_depth()
        );
        for b in too_tall {
            assert!(
                b.max.z - b.min.z > run.spec.cavity_depth(),
                "an object that does not fit still stands its full height, or the drawing                  hides the warning"
            );
        }
    }

    #[test]
    fn builds_one_piece_per_partition_cell_set() {
        let spec = input::parse(SMALL).expect("the fixture is a valid run");
        let run = fit(spec).expect("a two-cell drawer of four blocks builds");

        assert_eq!(run.pieces.len(), run.parts.len());
        assert!(
            run.split_lines.is_empty() && run.pieces.len() == 1,
            "an 84 mm bin fits every profile's bed, so it is not split"
        );
        let mut covered: Vec<GridCell> = run.parts.iter().flat_map(|p| p.cells.clone()).collect();
        covered.sort_by_key(|c| (c.y, c.x));
        let mut want = run.cells.clone();
        want.sort_by_key(|c| (c.y, c.x));
        assert_eq!(
            covered, want,
            "the pieces must partition the bin's cells exactly, covering each once"
        );
    }

    /// Every piece a run writes is one lump of material with nothing floating
    /// beside it. `carve_to_cells` asserts this as it produces each piece, so
    /// reaching here at all is most of the check; what this adds is that the
    /// report's own copy of the numbers agrees with the solids it was read off,
    /// since a report that recomputed them could disagree with what was written.
    #[test]
    fn every_piece_it_writes_is_one_sound_body() {
        let spec = input::parse(SMALL).expect("the fixture is a valid run");
        let run = fit(spec).expect("a two-cell drawer of four blocks builds");

        assert_eq!(run.soundness.len(), run.pieces.len());
        for (piece, sound) in run.pieces.iter().zip(&run.soundness) {
            assert_eq!(sound.name, piece.name);
            assert_eq!(
                sound.shells, 1,
                "{} is one rectangular slab of cells, so it is one shell",
                piece.name
            );
            assert_eq!(sound.faces, piece.solid.faces.len());
            assert_eq!(sound.edges, piece.solid.edges.len());
            assert_eq!(sound.verts, piece.solid.verts.len());
            assert!(
                piece.solid.shells().iter().all(|sh| sh.encloses_material),
                "{} bounds no sealed void",
                piece.name
            );
            assert!(
                piece.solid.orphan_vertices().is_empty()
                    && piece.solid.orphan_edges().is_empty(),
                "{} carries no geometry nothing names",
                piece.name
            );
        }
    }

    #[test]
    fn carries_the_generated_dividers_into_the_params_it_builds() {
        let spec = input::parse(SMALL).expect("the fixture is a valid run");
        let run = fit(spec).expect("a two-cell drawer of four blocks builds");

        assert_eq!(run.params.inner_walls.len(), run.result.walls.len());
        assert_eq!(run.params.bins.len(), 1);
        assert_eq!(run.params.bins[0].cells, run.cells);
        for (wall, inner) in run.result.walls.iter().zip(&run.params.inner_walls) {
            assert_eq!(inner.x1, wall.start.x);
            assert_eq!(inner.y1, wall.start.y);
            assert_eq!(inner.width, wall.width);
            assert_eq!(inner.height, None, "a generated divider is full height");
        }
    }

    #[test]
    fn refuses_a_drawer_too_small_for_one_cell() {
        let spec = input::parse("[drawer]\nwidth = 40\ndepth = 40\n").expect("a valid run");
        let Err(err) = fit(spec) else {
            panic!("40 mm does not hold a 42 mm cell, so there is no bin to build");
        };
        assert!(err.contains("42 mm Gridfinity cell"), "{err}");
    }

    #[test]
    fn refuses_an_output_path_of_the_wrong_kind_before_building_anything() {
        let dir = std::env::temp_dir();
        let err = Format::ParasolidXt
            .check_output(&dir)
            .expect_err("a transmit file is not a directory");
        assert!(err.contains("is a directory"), "{err}");
        assert!(
            Format::Stl.check_output(&dir).is_ok(),
            "an existing directory is exactly what --format stl wants"
        );
    }
}
