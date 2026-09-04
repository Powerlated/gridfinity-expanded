//! The `optimize` subcommand: a TOML describing a drawer and the objects to
//! organise in it, in; printable geometry and an account of how it was reached,
//! out.
//!
//! `Args` is what the command line says -- a `clap` struct, so the parsing,
//! the spellings and the help text are the declaration -- `Run` is everything
//! one invocation produced, and `fit` is the pipeline between them.
//! `Args::destination` is the one place the command line's two output questions
//! are answered: which format `-o` names, and whether `-o` can hold the format
//! that was named. The path is then checked before any geometry runs, because
//! discovering it cannot be written after minutes of packing and building is the
//! same mistake reported far too late. `fit`: resolve the drawer to a
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
use crate::grouping::{GroupPlan, Grouping, choose_groups, outer_pack, plan_group_bin};
use crate::input::{self, Object, Spec};
use crate::report;
use clap::{Parser, ValueEnum};
#[cfg(feature = "occt")]
use gridfinity_occt::Shape as Solid;
use gridfinity_model::gridfinity::{self, LogicalBin, Mode, Params, Pocket};
#[cfg(not(feature = "occt"))]
use gridfinity_model::gridfinity::BinPiece;
#[cfg(feature = "occt")]
use gridfinity_model::gridfinity::OcctBinPiece as BinPiece;
use gridfinity_model::layout::{GridCell, Piece, SplitLine, partition_cells};
use gridfinity_model::printers::{compute_auto_split_lines, compute_staggered_split_lines};
use gridfinity_model::subbin::{SubbinSpec, buildable_interior_corner};
#[cfg(not(feature = "occt"))]
use gridfinity_model::subbin::build_subbin;
#[cfg(feature = "occt")]
use gridfinity_model::subbin::build_subbin_occt as build_subbin;
use gridfinity_project::drawer::{
    DrawerGrid, MAX_GRID, cavity_region, drawer_cells, drawer_grid, packing_area, packing_inset,
};
use gridfinity_project::pack::{
    PackEffort, PackInput, PackObject, PackResult, Placement, pack_layout,
};
use gridfinity_project::rects::{
    Rect, Rotation, inflate_parts, parts_bounds, rotate_parts, translate_parts,
};
use gridfinity_project::settle::{Extents, Settle, Settled, clamp, settle};
use gridfinity_project::tidy::tidiness;
use gridfinity_project::walls::{WallReport, layout_walls_reporting};
use gridfinity_sketch::math::Vec3;
use gridfinity_sketch::round::MIN_ARC_R;
use std::collections::BTreeMap;
use std::path::PathBuf;
use std::time::{Duration, Instant};

/// Rounding outcomes retained in the run report. OCCT applies requested
/// fillets while constructing each body; unlike the retired command-stream
/// kernel it does not expose unresolved edge-selection bookkeeping.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct BlendReport {
    pub requested: usize,
    pub unresolved: usize,
    pub dropped: Vec<usize>,
    pub refusal: Option<String>,
}

impl BlendReport {
    pub fn is_clean(&self) -> bool {
        self.unresolved == 0 && self.dropped.is_empty()
    }

    pub fn made(&self) -> usize {
        self.requested
            .saturating_sub(self.unresolved)
            .saturating_sub(self.dropped.len())
    }
}

/// What to build out of the fitted drawer.
///
/// The answers produce entirely different sets of parts from one input file,
/// which is why the command line requires one rather than defaulting: a user
/// who has not said which they want has not said what they want built. `Auto`
/// is a statement in the same sense -- build the smallest bins the drawer can
/// be fitted with -- not the absence of one.
#[derive(Clone, Copy, Debug, PartialEq, Eq, ValueEnum)]
pub enum FitMode {
    /// One bin per object where the drawer holds them, and the whole drawer as
    /// one bin only where it does not. A discrete bin is the smaller print and
    /// the smaller thing to lose, so the large body is what a run falls back to
    /// rather than what it reaches for.
    #[value(name = "auto")]
    Auto,
    /// One discrete Gridfinity bin per object, each holding that object's whole
    /// quantity as its own compartments, all of them packed into the drawer and
    /// dropping into the one baseplate.
    #[value(name = "bins")]
    Bins,
    /// The same discrete bins, except that objects share one where sharing pays:
    /// a bin is a whole number of cells, so several small objects in one bin can
    /// stand on fewer cells than one bin each. `grouping.rs` decides where.
    #[value(name = "hybrid")]
    Hybrid,
    /// The whole drawer as one bin, hollowed to a compartment per packed object
    /// and solid everywhere else, so the material between two compartments is
    /// the divider.
    #[value(name = "walls")]
    Walls,
}

/// What a finished run was built as.
///
/// `FitMode::Auto` resolves to one of these before any geometry exists, so
/// nothing downstream of the plan carries an arm it can never take.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Built {
    /// One discrete bin per object, as `FitMode::Bins`.
    Bins,
    /// One discrete bin per group of objects, as `FitMode::Hybrid`. The same
    /// shape of run as `Bins` -- discrete bins packed into the drawer -- with
    /// more than one object allowed to share one.
    Hybrid,
    /// The whole drawer as one bin, as `FitMode::Walls`.
    Walls,
}

/// What one `optimize` invocation was asked to do.
///
/// An invocation must ask for at least one of the two things this command can
/// do -- write geometry (`-o`) or show it (`--view`) -- so `clap` requires `-o`
/// unless `--view` is present. Both together fit, write, and then open the
/// window on what was written.
#[derive(Parser, Debug, PartialEq, Eq)]
#[command(
    about = "Fit a drawer full of objects and write the geometry",
    long_about = "Packs the objects a TOML describes into the drawer it describes and states every compartment as a pocket -- as one drawer-wide bin with --mode walls, as one \nGridfinity bin per object with --mode bins, or with --mode auto as the bins where the drawer holds them and the one drawer-wide bin only where it does \nnot. Splits every body and its baseplate for the printer's bed, writes the geometry, and prints what it did."
)]
pub struct Args {
    /// The drawer's dimensions and the objects to organise in it
    pub input: PathBuf,

    /// What to build: `walls` states the whole drawer as one bin hollowed to a
    /// compartment per object, `bins` builds one Gridfinity bin per object,
    /// each holding its own instances, and `auto` builds those bins where the
    /// drawer holds them and falls back to the one drawer-wide bin where it
    /// does not
    #[arg(long, value_name = "MODE")]
    pub mode: FitMode,

    /// Where to write the geometry: a directory for `stl`, a `.x_t` file for
    /// `parasolid_x_t`. Required unless `--view` is given
    #[arg(short, long, value_name = "PATH", required_unless_present = "view")]
    pub output: Option<PathBuf>,

    /// The export format. Inferred from `--output` when that ends in `.x_t`,
    /// and required otherwise
    #[arg(long, value_name = "FORMAT")]
    pub format: Option<Format>,

    /// Open the fit in the construction debugger, with a wireframe box around
    /// every packed object -- red where the object stands taller than the
    /// compartment it was packed into
    #[arg(long)]
    pub view: bool,
}

impl Args {
    /// The format and path this invocation writes, or `None` when it asked only
    /// to look (`--view` with no `-o`).
    ///
    /// This is where `--format` and `-o` are reconciled, in both directions.
    /// An `-o` ending in `.x_t` names its own format, so `--format` may be left
    /// off; anything else names none and `--format` is required. A `--format`
    /// that is given is never overridden by the path -- it is checked against
    /// it, and a path that cannot hold it is an error rather than a silent
    /// reinterpretation of what the user asked for.
    pub fn destination(&self) -> Result<Option<(Format, PathBuf)>, String> {
        let Some(path) = self.output.clone() else {
            assert!(
                self.view,
                "an invocation with neither an output nor --view asks for nothing, which is what clap's required_unless_present rejects"
            );
            return Ok(None);
        };
        let format = self
            .format
            .or_else(|| Format::inferred_from(&path))
            .ok_or_else(|| {
                format!(
                    "--format is required: {} does not end in .x_t, so it names no format (it is stl or parasolid_x_t)",
                    path.display()
                )
            })?;
        format.check_extension(&path)?;
        Ok(Some((format, path)))
    }
}

/// Everything one invocation produced, in the order the pipeline produced it.
///
/// `claim_margin` is the packer's own `PackInput::margin`, kept rather than
/// restated: it is how the report and the drawn boxes get back from a
/// `Placement`'s claim to the object inside it, and a second derivation of it
/// silently disagrees the moment the two are fed different numbers.
///
/// Three fields are unions over however many bins the mode produced, which is
/// the identity for the single bin `walls` builds: `cells` is the whole
/// drawer's cells (the baseplate is the drawer's floor and spans the cells no
/// bin sits on), `split_lines` is every line any bin is cut on, and `parts` is
/// every bin's partition concatenated **in the order the pieces are built**, so
/// the nth piece is the body over the nth part. `result` is the placement of
/// every *instance* in drawer millimetres in both modes -- in `bins` the
/// instance's claim inside its own bin, composed with where that bin landed --
/// so everything reading it reads one thing.
pub struct Run {
    pub spec: Spec,
    pub built: Built,
    /// Why an `Auto` run built one drawer-wide bin: the refusal the
    /// bin-per-object plan came back with. `None` for a run that was told which
    /// mode to build, and for an `Auto` run that got its bins.
    pub fell_back: Option<String>,
    pub grid: DrawerGrid,
    pub area: Rect,
    pub cells: Vec<GridCell>,
    pub result: PackResult,
    pub floor_fillet: f64,
    pub claim_margin: f64,
    pub wall_report: WallReport,
    /// What settling the packed layout took: free bands of leftover absorbed
    /// into the compartments facing them, claim walls grown into the leftover no
    /// band reached, and slabs whose slack was evened out
    /// between their two ends. Zero and zero is a layout the packer already left
    /// square.
    pub absorbed: usize,
    pub evened: usize,
    pub grown: usize,
    pub clamped: usize,
    pub pockets: Vec<Pocket>,
    /// One entry per object that was given a bin of its own, in `params.bins`
    /// order. Empty in `walls` mode, where the drawer is the one bin.
    pub bins: Vec<FittedBin>,
    /// How the objects were grouped into bins, for a run that grouped them.
    /// `None` in every mode but `hybrid`, which is the only one that chooses.
    pub grouping: Option<Grouping>,
    /// One insert per placed instance of an object that asked for one, in
    /// placement order. Empty for a file that asks for none.
    pub subbins: Vec<BuiltSubbin>,
    pub params: Params,
    pub plate_params: Option<Params>,
    pub split_lines: Vec<SplitLine>,
    pub parts: Vec<Piece>,
    pub pieces: Vec<BinPiece>,
    pub plate_split_lines: Vec<SplitLine>,
    pub plate_parts: Vec<Piece>,
    pub plate_stagger_cost: usize,
    pub baseplate: Vec<BinPiece>,
    pub blends: BlendReport,
    pub soundness: Vec<PieceSoundness>,
    pub pack_time: Duration,
    pub build_time: Duration,
    pub export_time: Duration,
}

/// One body a run wrote or would write: the file it is named as, and the solid
/// it is.
///
/// The writers and the Soundness section read this rather than a `BinPiece`,
/// because an insert is neither a piece of a bin nor a piece of a baseplate --
/// it has no cells and no partition -- and everything downstream of the build
/// wants only the two things every body has.
pub struct Body<'a> {
    pub name: &'a str,
    pub solid: &'a Solid,
}

impl Run {
    /// Everything this run built, in the order it is written: the bin's pieces,
    /// then the baseplate's, then the inserts, each of the last two empty when
    /// the file asked for none. The export and the report both read this rather
    /// than any one list alone, so a file that is written is a file the
    /// soundness section accounts for.
    pub fn all_pieces(&self) -> Vec<Body<'_>> {
        fn bodies(pieces: &[BinPiece]) -> Vec<Body<'_>> {
            pieces
                .iter()
                .map(|p| Body {
                    name: &p.name,
                    solid: &p.solid,
                })
                .collect()
        }
        let mut out = bodies(&self.pieces);
        out.extend(bodies(&self.baseplate));
        out.extend(self.subbins.iter().map(|s| Body {
            name: &s.name,
            solid: &s.solid,
        }));
        out
    }

    /// Whether the stack holds itself together: every seam of the bin is spanned
    /// by a piece of the baseplate and every seam of the plate by a piece of the
    /// bin, which is exactly the two bodies sharing no cut line.
    ///
    /// True of a run with no baseplate to interlock with, and of one where
    /// either body is a single piece -- a piece that is cut nowhere spans every
    /// seam under it. False only where a staggered plan for the plate did not
    /// print, so it fell back to the bin's own lines and the assembly parts
    /// along one plane.
    pub fn interlocked(&self) -> bool {
        self.baseplate.is_empty()
            || self
                .plate_split_lines
                .iter()
                .all(|l| !self.split_lines.contains(l))
    }
}

/// One bin as it stands in the drawer: which objects it holds, the cells it
/// covers, how many compartments were hollowed into it, and where it is cut for
/// the bed.
///
/// The cells are the polyomino the object's packed claims actually reach, not
/// the rectangle around them, so an L-shaped object's bin is an L. They are in
/// the drawer's own grid coordinates, so they are exactly the cells of the
/// `LogicalBin` this describes.
pub struct FittedBin {
    /// The objects sharing this bin, sorted. One in `bins` mode, where a bin is
    /// an object; one or more in `hybrid`, where a bin is a group of them.
    pub objects: Vec<String>,
    pub cells: Vec<GridCell>,
    pub instances: usize,
    pub split_lines: Vec<SplitLine>,
}

impl FittedBin {
    /// What to call this bin: the objects sharing it, joined. A bin *is* what is
    /// in it, so a bin of one is named by that object and a shared one by all of
    /// them -- "bin 3" names nothing the reader of a report or a view wants.
    pub fn name(&self) -> String {
        assert!(
            !self.objects.is_empty(),
            "a fitted bin holds at least one object"
        );
        self.objects.join(" + ")
    }
}

/// One placed object's box, in the bin's own millimetre coordinates: what the
/// packer reserved for it, standing on the cavity floor.
///
/// `name` is the object's own, carried so the window can label the box with it:
/// a box is the one thing in the scene the viewer cannot identify by looking,
/// every one of them being a white rectangle. `instance` is which placement it
/// belongs to, so the several boxes of an L-shaped object are known to be one
/// object and named once rather than once per part. `bin` is which of
/// `Params::bins` it stands in, which is what the window explodes and clips it
/// against: in `bins` mode two boxes of one scene belong to bodies cut on
/// different lines, and a box taken apart on the wrong bin's seams travels away
/// from the bin it is in. `fits` is whether the object's stated height clears
/// the cavity, which is the same question the report's warnings answer in words
/// -- a box that does not fit still stands its full height, poking out of the
/// bin it was packed into, because hiding that is hiding the warning.
pub struct ObjectBox {
    pub name: String,
    pub instance: usize,
    pub bin: usize,
    pub min: Vec3,
    pub max: Vec3,
    pub fits: bool,
}

/// Everything `--view` opens the window on: the bin to rebuild, the boxes of the
/// objects it was cut for, and the baseplate under it -- `None` when the file
/// asked for none.
///
/// The plate travels as its own `Params` rather than as built geometry, like the
/// bin does, so the window rebuilds both from the same declarations the export
/// wrote from. It carries the plate's **own** split lines, which is what makes
/// the two bodies explode along different bands and shows the interlock: a piece
/// of each spans the other's seams.
pub struct View {
    pub params: Params,
    pub plate: Option<Params>,
    pub boxes: Vec<ObjectBox>,
    /// What to call each bin of `params`, one per bin and in the same order.
    /// Empty in `walls` mode, where the drawer is one bin and "bin" is the whole
    /// of what there is to say about it; in `bins` mode a bin *is* an object, so
    /// its name is the only thing telling two grey boxes apart.
    pub bin_names: Vec<String>,
    /// The file each piece of each bin would be written as: outer index into
    /// `params.bins`, inner in piece order, empty for a bin with no cells.
    ///
    /// Carried rather than re-derived. The window partitions a bin with the very
    /// same `partition_cells(&bin.cells, &bin.split_lines)` the exporter does,
    /// so a piece index means one body on both sides -- but the *name* is built
    /// in `gridfinity::pieces`, from a stem that is not the label (in `bins`
    /// mode a body reads "AAA batteries + caliper box" and writes
    /// `gridfinity-bin-1`), and a second copy of that rule in the viewer would
    /// drift the first time either changed.
    pub bin_files: Vec<Vec<String>>,
    /// The same for the baseplate's own pieces, in its own piece order. Empty
    /// for a run with no plate.
    pub plate_files: Vec<String>,
    /// Every insert the fit built, as the declaration that rebuilds it. It
    /// travels as a spec rather than as geometry for the same reason the plate
    /// travels as its own `Params`: the window rebuilds what the export wrote.
    pub subbins: Vec<PlacedSubbin>,
}

/// One insert as the window is given it: what to call it, the file it would be
/// written as, the declaration that rebuilds it, and which of `params.bins` it
/// stands in -- which is the bin whose seams it opens with.
pub struct PlacedSubbin {
    pub label: String,
    pub file: String,
    pub bin: usize,
    pub spec: SubbinSpec,
}

/// The name every built piece would be written as, gathered for the viewer:
/// one list per bin of `run.params.bins` in piece order, and the baseplate's
/// own list beside it.
///
/// Read off the finished `BinPiece`s rather than rebuilt, which is what keeps
/// the label and the file one string. `BinPiece::bin` indexes `params.bins` and
/// `BinPiece::piece` is the index into that bin's own `partition_cells`, so
/// slotting each name by the pair reproduces the exporter's order exactly; the
/// assertion states that every slot was filled, which is the same as saying the
/// two sides partitioned the bin the same way.
fn piece_files(run: &Run) -> (Vec<Vec<String>>, Vec<String>) {
    let mut bins: Vec<Vec<String>> = run
        .params
        .bins
        .iter()
        .map(|bin| vec![String::new(); partition_cells(&bin.cells, &bin.split_lines).len()])
        .collect();
    for piece in &run.pieces {
        let slot = bins
            .get_mut(piece.bin)
            .and_then(|files| files.get_mut(piece.piece))
            .unwrap_or_else(|| {
                panic!(
                    "{} is piece {} of bin {}, which the viewer's own partition of that bin does \
                     not have",
                    piece.name, piece.piece, piece.bin
                )
            });
        *slot = piece.name.clone();
    }
    assert!(
        bins.iter().flatten().all(|name| !name.is_empty()),
        "a bin has a piece the export never named, so the viewer and the exporter partitioned it \
         differently"
    );
    let plate = run.baseplate.iter().map(|p| p.name.clone()).collect();
    (bins, plate)
}

/// How far outside its compartment a drawn object may measure before the fit is
/// wrong rather than merely rounded: `Rect::right` and `Rect::bottom` quantise,
/// so a claim and the object inside it agree only to that quantum.
const BOX_IN_COMPARTMENT_MM: f64 = 1e-6;

/// Every placed instance's boxes, lifted from the packer's millimetre rectangles
/// into the bin's own coordinates: each part rectangle standing on the cavity
/// floor, rising by the height its object declared, or filling the cavity when
/// it declared none.
///
/// **A `Placement` carries the instance's *claim*, not the object.** A claim is
/// the object grown by `claim_margin` -- its clearance, the reserved fillet, and
/// the half divider that stands on the claim boundary -- so it reaches to the
/// divider centrelines and to the edge of the packing area by construction.
/// Drawing it is drawing a box that laps every wall the layout has, which is a
/// picture of the reservation and not of the object.
///
/// **An object given an insert is drawn standing in the insert**, not on the
/// compartment floor: the space holding it is the insert's own interior, and it
/// is lifted by the insert's floor. Everything else about the drawing is the
/// same, which is what keeps the two cases one function.
///
/// **And deflating the claim is not enough either, because `settle` grows it.**
/// Absorbing a strip of leftover into a compartment widens the claim after the
/// packer is done with it, so `claim - claim_margin` is the *compartment*, not
/// the object: on `examples/ikea-alex-drawer-1.toml`, whose `tidy_absorb` is
/// 100 mm, that draws a tape measure as the whole end of the drawer. The object
/// is its own declared boxes, `rotate_parts` by the quarter turn the packer
/// chose, and those are what is drawn -- **centred in the compartment**, since
/// the compartment is the only thing that says where the object is and the
/// object is free to sit anywhere in it. A layout that settled nothing draws
/// exactly what deflating the claim drew, which is what
/// `an_object_is_drawn_at_its_own_size_however_much_its_compartment_grew` holds
/// both halves of.
///
/// `packing_area` is already in the bin's coordinates, so nothing else needs a
/// transform -- the rotation the packer chose is baked into the rectangles
/// themselves. An object made of several boxes contributes one box per part
/// rather than one bounding box over all of them, so an L-shaped object reads as
/// the L it is and not as the rectangle around it.
fn object_boxes(run: &Run) -> Vec<ObjectBox> {
    let depth = run.spec.cavity_depth();
    let floor = gridfinity::BASE_TOTAL_HEIGHT + gridfinity::FLOOR_THICKNESS;
    let margin = run.claim_margin;
    let mut out = Vec::new();
    for (instance, placement) in run.result.placements.iter().enumerate() {
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
        let insert = run.subbins.iter().find(|s| s.instance == instance);
        let (holds, stands_on, clears) = match insert {
            Some(sub) => (
                Rect::new(
                    sub.spec.x + sub.spec.walls().0,
                    sub.spec.y + sub.spec.walls().1,
                    sub.spec.interior_width,
                    sub.spec.interior_depth,
                ),
                sub.spec.z + sub.spec.floor,
                sub.spec.interior_height,
            ),
            None => (
                parts_bounds(&inflate_parts(&placement.parts, -margin)),
                floor,
                depth,
            ),
        };
        let height = object.height.unwrap_or(clears);
        let turned = rotate_parts(&object.footprint, placement.rotation);
        let own = parts_bounds(&turned);
        assert!(
            insert.is_some() || turned.len() == placement.parts.len(),
            "an instance of {} is {} box(es), so its claim is {} and not {}",
            object.pack.name,
            turned.len(),
            turned.len(),
            placement.parts.len()
        );
        assert!(
            own.width <= holds.width + BOX_IN_COMPARTMENT_MM
                && own.depth <= holds.depth + BOX_IN_COMPARTMENT_MM,
            "{} measures {} x {} mm and the space holding it {} x {} mm, so the object does not \
             go in the space the fit reserved for it",
            object.pack.name,
            own.width,
            own.depth,
            holds.width,
            holds.depth
        );
        let parts = translate_parts(
            &turned,
            holds.x + 0.5 * (holds.width - own.width) - own.x,
            holds.y + 0.5 * (holds.depth - own.depth) - own.y,
        );
        for part in &parts {
            out.push(ObjectBox {
                name: object.pack.name.clone(),
                instance,
                bin: bin_of(&run.bins, &placement.object_id),
                min: Vec3::new(part.x, part.y, stands_on),
                max: Vec3::new(part.right(), part.bottom(), stands_on + height),
                fits: height <= clears,
            });
        }
    }
    out
}

/// One insert, built and standing where it belongs: what it holds, which bin it
/// stands in, the declaration that reproduces it, and the solid itself.
///
/// The spec travels beside the solid for the same reason the baseplate's
/// `Params` does -- `--view` rebuilds from the declaration the export wrote
/// from, rather than being handed geometry -- and the report reads its
/// measurements off it rather than re-deriving them from the placement.
pub struct BuiltSubbin {
    pub name: String,
    pub object: String,
    pub instance: usize,
    pub bin: usize,
    pub spec: SubbinSpec,
    pub solid: Solid,
}

/// Every placed instance of an object that asked for a subbin, as the insert it
/// asks for, built in the drawer's own millimetres.
///
/// The compartment is the placement's claim inset by half a divider -- the same
/// pocket `drawer_pockets` states -- and the insert is that pocket inset by
/// `subbin_clearance` on every side, so it stands that gap inside the
/// compartment at every height and its bottom chamfer clears the floor blend.
/// That gap is a part fitting a part and is not `clearance`, which is the room
/// an object is given inside its compartment.
/// An axis the file pinned keeps exactly the interior it stated; an axis it left
/// open takes whatever settling grew the compartment to, less the two walls.
/// Height comes from the file or fills the compartment.
///
/// One path for every `--mode`, because a `Placement` is in drawer millimetres
/// in all four of them.
fn build_subbins(
    spec: &Spec,
    result: &PackResult,
    bins: &[FittedBin],
    floor_fillet: f64,
) -> Result<Vec<BuiltSubbin>, String> {
    let wall = spec.subbin_wall_thickness;
    let floor_z = gridfinity::BASE_TOTAL_HEIGHT + gridfinity::FLOOR_THICKNESS;
    let inset = spec.divider_thickness / 2.0 + spec.subbin_clearance;
    let mut out: Vec<BuiltSubbin> = Vec::new();
    for (instance, placement) in result.placements.iter().enumerate() {
        let object = spec
            .objects
            .iter()
            .find(|o| o.pack.id == placement.object_id)
            .unwrap_or_else(|| {
                panic!(
                    "the packer placed {:?}, which is not one of the {} objects it was given",
                    placement.object_id,
                    spec.objects.len()
                )
            });
        let Some(subbin) = object.subbin else {
            continue;
        };
        assert_eq!(
            placement.parts.len(),
            1,
            "{} asks for an insert, so it is one box and its claim is one rectangle",
            object.pack.name
        );
        let claim = placement.parts[0];
        let outer = Rect::new(
            claim.x + inset,
            claim.y + inset,
            claim.width - 2.0 * inset,
            claim.depth - 2.0 * inset,
        );
        let [along, across] = [subbin.interior[0], subbin.interior[1]];
        let pinned = match placement.rotation {
            Rotation::Deg90 | Rotation::Deg270 => [across, along],
            Rotation::Deg0 | Rotation::Deg180 => [along, across],
        };
        let interior = [
            pinned[0].unwrap_or(outer.width - 2.0 * wall),
            pinned[1].unwrap_or(outer.depth - 2.0 * wall),
        ];
        for (axis, outer, interior) in [
            ("width", outer.width, interior[0]),
            ("depth", outer.depth, interior[1]),
        ] {
            assert!(
                outer - interior >= 2.0 * wall - 1e-6,
                "{}'s compartment is {outer} mm in {axis} around a {interior} mm interior, which \
                 leaves less than the {wall} mm wall the insert is built with",
                object.pack.name
            );
        }
        let chamfer = floor_fillet;
        let corner_r = spec.fillet_radius.max(chamfer + MIN_ARC_R);
        let built = SubbinSpec {
            x: outer.x,
            y: outer.y,
            z: floor_z,
            outer_width: outer.width,
            outer_depth: outer.depth,
            interior_width: interior[0],
            interior_depth: interior[1],
            interior_height: subbin.interior[2]
                .unwrap_or_else(|| (spec.cavity_depth() - wall).max(wall)),
            interior_corner_r: buildable_interior_corner(corner_r - wall, interior[0], interior[1]),
            floor: wall.max(chamfer),
            corner_r,
            chamfer,
        };
        let solid = build_subbin(&built)
            .map_err(|e| format!("the insert for {}: {e}", object.pack.name))?;
        out.push(BuiltSubbin {
            name: String::new(),
            object: object.pack.name.clone(),
            instance,
            bin: bin_of(bins, &placement.object_id),
            spec: built,
            solid,
        });
    }
    let total = out.len();
    for (i, subbin) in out.iter_mut().enumerate() {
        subbin.name = if total == 1 {
            "gridfinity-subbin.stl".to_string()
        } else {
            format!("gridfinity-subbin-{}-of-{total}.stl", i + 1)
        };
    }
    Ok(out)
}

/// Every body this run exports, rebuilt directly in OCCT from the declarations
/// the fitter produced: bin pieces, baseplate pieces, then inserts.
#[cfg(feature = "occt")]
fn occt_bodies(run: &Run) -> Vec<export::OcctBody<'_>> {
    run.all_pieces()
        .into_iter()
        .map(|body| export::OcctBody {
            name: body.name,
            shape: body.solid,
        })
        .collect()
}

/// Which of a run's bins an object stands in: the one holding it, and 0 for a
/// `walls` run, where the drawer is the one bin.
fn bin_of(bins: &[FittedBin], object_id: &str) -> usize {
    if bins.is_empty() {
        return 0;
    }
    bins.iter()
        .position(|b| b.objects.iter().any(|id| id == object_id))
        .unwrap_or_else(|| panic!("{object_id} was packed into a bin the fit does not carry"))
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
fn soundness_of(pieces: &[Body<'_>]) -> Vec<PieceSoundness> {
    pieces
        .iter()
        .map(|piece| {
            let solid: &Solid = piece.solid;
            #[cfg(not(feature = "occt"))]
            assert!(
                solid.orphan_vertices().is_empty() && solid.orphan_edges().is_empty(),
                "{} reached the report with geometry nothing names, which the carve gate refuses",
                piece.name
            );
            #[cfg(not(feature = "occt"))]
            let (shells, faces, edges, verts, warnings) = (
                solid.shells().len(),
                solid.faces.len(),
                solid.edges.len(),
                solid.verts.len(),
                gridfinity_model::audit(solid).warnings().count(),
            );
            #[cfg(feature = "occt")]
            let (shells, faces, edges, verts, warnings) = {
                assert!(
                    solid.is_valid().expect("OCCT validity query succeeds"),
                    "{} reached the report invalid",
                    piece.name
                );
                let shells = solid
                    .shell_volumes()
                    .expect("OCCT shell-volume query succeeds");
                assert!(
                    shells.iter().all(|volume| *volume > 0.0),
                    "{} reached the report with a non-material shell",
                    piece.name
                );
                let mesh = solid.tessellate(0.08).expect("valid OCCT body tessellates");
                (
                    shells.len(),
                    mesh.tri_count(),
                    mesh.indices.len(),
                    mesh.positions.len(),
                    0,
                )
            };
            PieceSoundness {
                name: piece.name.to_string(),
                shells,
                faces,
                edges,
                verts,
                warnings,
            }
        })
        .collect()
}

/// Every placed instance's compartment, as the pockets the bin is hollowed to:
/// each part rectangle of each claim, inset by half a divider on every side.
///
/// A `Placement` carries the **claim**, which reaches the divider centrelines
/// and the packing-area edge by construction, so insetting it by `t / 2` lands
/// the pocket wall exactly where a generated divider's face would have stood --
/// the compartment is the size it always was, and the object keeps the whole
/// `clearance + floor_fillet` the claim reserved beyond it. What changes is
/// everything *outside* a pocket: with the cavity stated rather than walked, the
/// drawer is solid there instead of being an open pocket of air no object was
/// packed into.
///
/// One pocket per part rather than one per placement, and they are allowed to
/// overlap: an object's parts touch along an edge, so their inset claims
/// overlap, and `pocket_cavity` unions them back into the one L-shaped
/// compartment the object wants.
fn drawer_pockets(placements: &[Placement], divider_thickness: f64) -> Vec<Pocket> {
    let inset = divider_thickness / 2.0;
    let mut out = Vec::new();
    for placement in placements {
        for part in &placement.parts {
            let (width, depth) = (part.width - inset * 2.0, part.depth - inset * 2.0);
            assert!(
                width > 0.0 && depth > 0.0,
                "a claim of {} x {} mm inset by half a {divider_thickness} mm divider leaves no compartment",
                part.width,
                part.depth
            );
            out.push(Pocket {
                x: part.x + inset,
                y: part.y + inset,
                width,
                depth,
            });
        }
    }
    out
}

/// The `Params` a drawer fitted in `walls` mode builds as: one bin covering the
/// drawer's cells, hollowed to one pocket per placed object and cut on `splits`
/// for the printer's bed.
///
/// Everything outside a pocket is material: the space between two compartments,
/// and the space no object was packed into.
fn drawer_params(
    spec: &Spec,
    cells: &[GridCell],
    pockets: &[Pocket],
    splits: &[SplitLine],
) -> Params {
    bins_params(
        spec,
        vec![LogicalBin {
            cells: cells.to_vec(),
            split_lines: splits.to_vec(),
            slope: None,
            pockets: pockets.to_vec(),
        }],
    )
}

/// The `Params` a run of any number of logical bins builds as: the bins as
/// given, and every dimension the file settled, in `Mode::Bin` with no plate
/// margin.
///
/// **No bin carries an `inner_wall`.** A divider is what is left between two
/// compartments when the cavity is walked, and every cavity here is stated
/// instead -- as pockets in a drawer-wide bin, or as the compartments of one
/// object's own bin -- so the material between two compartments is simply
/// material. That also keeps a fitted drawer off the free-form inner wall path,
/// which is the kernel's weakest surface.
fn bins_params(spec: &Spec, bins: Vec<LogicalBin>) -> Params {
    Params {
        bins,
        pitch: spec.pitch,
        height_units: spec.height_units,
        wall_thickness: spec.wall_thickness,
        cavity_corner_radius: spec.fillet_radius,
        floor_fillet: spec.fillet_radius,
        magnet_holes: spec.magnets,
        screw_holes: spec.screws,
        open_edges: Vec::new(),
        divider_edges: Vec::new(),
        inner_walls: Vec::new(),
        plate_margin_x: 0.0,
        plate_margin_y: 0.0,
        mode: Mode::Bin,
    }
}

/// Where the fitted drawer's baseplate is cut: its own plan, staggered off the
/// bin's `bin_splits` so the two bodies part on different lines, or the bin's
/// own lines when no staggered plan prints. Empty when the file asked for no
/// baseplate, which is the one case with nothing to cut.
///
/// **The seams are deliberately not shared.** A bin piece that spans a plate
/// seam pegs into both plate pieces at once and holds them together, and a plate
/// piece that spans a bin seam holds the bin pieces the same way, so a stack cut
/// on two staggered sets of lines constrains itself to move as one body: no
/// piece can leave without lifting the pieces it laps. Cut on one set, every
/// seam in the drawer lies in one plane and the whole stack parts along it.
///
/// The plate is planned in millimetres and against its own footprint rather than
/// the bin's, because it is not its cells: it stands half of `grid.margin_x` /
/// `_y` outside the grid at each end, and a plan measured in whole cells is the
/// reason a fitted plate used to be able to outgrow the bed its seams were
/// placed for. Falling back is a real outcome -- a drawer whose plate can only
/// be divided where the bin already is -- and `Run::interlocked` is what reads
/// it back, so the report can say the stack parts in one plane.
fn plate_splits(
    spec: &Spec,
    grid: DrawerGrid,
    cells: &[GridCell],
    bin_splits: &[SplitLine],
) -> Vec<SplitLine> {
    if !spec.baseplate {
        return Vec::new();
    }
    compute_staggered_split_lines(
        cells,
        spec.printer,
        spec.pitch,
        (grid.margin_x, grid.margin_y),
        bin_splits,
    )
    .unwrap_or_else(|| bin_splits.to_vec())
}

/// The pieces the baseplate would come to if it were free to be cut wherever it
/// liked: the bar its staggered plan is read against, so the report can say what
/// keeping off the bin's seams cost. One where no plan prints at all, which is
/// the bed refusing a single cell rather than anything staggering did.
fn plate_pieces_unstaggered(spec: &Spec, grid: DrawerGrid, cells: &[GridCell]) -> usize {
    compute_staggered_split_lines(
        cells,
        spec.printer,
        spec.pitch,
        (grid.margin_x, grid.margin_y),
        &[],
    )
    .map_or(1, |lines| partition_cells(cells, &lines).len())
}

/// The `Params` the fitted drawer's baseplate builds as: the bin's cells in
/// `Mode::Baseplate`, cut on `splits` -- the plate's *own* lines, staggered off
/// the bin's -- and carrying the drawer millimetres no cell covers as its plate
/// margin.
///
/// That margin is what makes the plate a snug fit. The bin is measured in whole
/// cells and the drawer is not, so a bin built on the grid alone leaves
/// `grid.margin_x` by `grid.margin_y` of slop for the whole stack to slide in;
/// the plate is the body that touches the drawer walls, so it is the body that
/// grows. It grows on the outside only, by half the margin on each side of the
/// grid, which spans the drawer exactly and leaves the bin, the packing area and
/// every compartment as they were.
fn baseplate_params(
    spec: &Spec,
    grid: DrawerGrid,
    cells: &[GridCell],
    splits: &[SplitLine],
) -> Params {
    Params {
        mode: Mode::Baseplate,
        plate_margin_x: grid.margin_x,
        plate_margin_y: grid.margin_y,
        ..drawer_params(spec, cells, &[], splits)
    }
}

/// What one mode's planner settled before any geometry was built: where every
/// instance landed, the compartments the model hollows, the bins it builds, and
/// what became of the dividers the packer derives.
struct Plan {
    result: PackResult,
    /// What settling the layout took: free bands absorbed into the compartments
    /// facing them, walls grown into what no band reached, and slabs whose slack
    /// was evened out. Summed over the bins
    /// in the modes that build several.
    absorbed: usize,
    evened: usize,
    grown: usize,
    clamped: usize,
    grouping: Option<Grouping>,
    pockets: Vec<Pocket>,
    params: Params,
    wall_report: WallReport,
    bins: Vec<FittedBin>,
}

/// The packing request for `objects` over `area`: the objects, and the margin
/// that turns each of them into the area it claims -- its clearance, the floor
/// fillet its compartment will be blended by, and the half divider that stands
/// on the claim boundary.
///
/// One function, so the margin a claim is grown by, the margin a pocket is inset
/// by and the margin the report deflates a claim by are one number rather than
/// three derivations of it.
pub(crate) fn claim_input(
    spec: &Spec,
    area: Rect,
    floor_fillet: f64,
    objects: Vec<PackObject>,
    effort: PackEffort,
) -> PackInput {
    PackInput {
        area,
        objects,
        divider_thickness: spec.divider_thickness,
        clearance: spec.clearance,
        floor_fillet,
        effort,
    }
}

/// The square one cell covers, in the millimetres of the grid it belongs to.
pub(crate) fn cell_rect(cell: GridCell, pitch: f64) -> Rect {
    Rect::new(
        f64::from(cell.x) * pitch,
        f64::from(cell.y) * pitch,
        pitch,
        pitch,
    )
}

/// The cells a list of pitch-sized squares standing on the grid names.
///
/// Every rectangle must be one cell of the lattice: the packer's positions are
/// area edges offset by the shape's own part offsets, and with a lattice-aligned
/// area and lattice-sized parts every one of them is a multiple of the pitch.
/// That is the property this asserts rather than assumes, because a placement
/// off the lattice would be a bin that does not sit in the baseplate under it.
fn cells_of_rects(rects: &[Rect], pitch: f64) -> Vec<GridCell> {
    rects
        .iter()
        .map(|r| {
            let (x, y) = (r.x / pitch, r.y / pitch);
            assert!(
                (x - x.round()).abs() < 1e-6
                    && (y - y.round()).abs() < 1e-6
                    && (r.width - pitch).abs() < 1e-6
                    && (r.depth - pitch).abs() < 1e-6,
                "a bin's footprint is whole cells of the {pitch} mm grid, but {r:?} is not one"
            );
            GridCell {
                x: x.round() as i32,
                y: y.round() as i32,
            }
        })
        .collect()
}

/// Every split line any bin of `params` is cut on, each named once, in the order
/// the bins state them.
fn all_split_lines(params: &Params) -> Vec<SplitLine> {
    let mut out: Vec<SplitLine> = Vec::new();
    for bin in &params.bins {
        for line in &bin.split_lines {
            if !out.contains(line) {
                out.push(*line);
            }
        }
    }
    out
}

/// Every bin's partition, concatenated in the order `try_build_pieces` emits the
/// pieces, so the nth piece is the body built over the nth part. A bin with no
/// cells contributes nothing, exactly as it builds nothing.
fn all_parts(params: &Params) -> Vec<Piece> {
    params
        .bins
        .iter()
        .filter(|b| !b.cells.is_empty())
        .flat_map(|b| partition_cells(&b.cells, &b.split_lines))
        .collect()
}

/// The claim each placement may grow to, in the **drawer's** frame: the object's
/// `max_size` turned by the quarter turn the packer chose and grown by the
/// margin that turned its size into a claim in the first place.
///
/// `None` on an axis the object is not held to. A quarter turn swaps the two,
/// because `max_size` is stated about the object and a `Placement` has already
/// been rotated -- the axis a battery must not roll along is the battery's, not
/// the drawer's.
pub(crate) fn claim_extents(
    placements: &[Placement],
    objects: &[Object],
    margin: f64,
) -> Vec<Extents> {
    assert!(
        margin >= 0.0,
        "a claim stands {margin} mm outside its object, which is not a margin"
    );
    placements
        .iter()
        .map(|placement| {
            let object = objects
                .iter()
                .find(|o| o.pack.id == placement.object_id)
                .unwrap_or_else(|| {
                    panic!(
                        "the packer placed {:?}, which is not one of the {} objects it was given",
                        placement.object_id,
                        objects.len()
                    )
                });
            let [along, across] = object.max_size;
            let turned = match placement.rotation {
                Rotation::Deg90 | Rotation::Deg270 => [across, along],
                Rotation::Deg0 | Rotation::Deg180 => [along, across],
            };
            turned.map(|most| most.map(|m| m + 2.0 * margin))
        })
        .collect()
}

/// A packed layout settled and then pulled back to the extents its objects ask
/// to be held to, which is the one order those two happen in: settling grows a
/// compartment into whatever leftover faces it, and `max_size` is how an object
/// says how much of that growth it wants.
pub(crate) fn settle_within(
    placements: &[Placement],
    cavity: &[Rect],
    spec: &Spec,
    margin: f64,
) -> (Settled, usize) {
    let settled = settle(
        placements,
        cavity,
        Settle {
            absorb: spec.tidy_absorb,
        },
    );
    let extents = claim_extents(&settled.placements, &spec.objects, margin);
    let held = clamp(&settled.placements, &extents);
    (
        Settled {
            placements: held.placements,
            ..settled
        },
        held.clamped,
    )
}

/// The whole drawer as one bin: every object packed into the one cavity, that
/// cavity stated as a pocket per placement, and the bin cut for the bed.
fn plan_walls(spec: &Spec, cells: &[GridCell], area: Rect, floor_fillet: f64) -> Plan {
    let input = claim_input(spec, area, floor_fillet, spec.pack_objects(), spec.effort);
    let input_margin = input.margin();
    let mut result = pack_layout(input);
    let cavity = cavity_region(cells, spec.pitch, packing_inset(spec.wall_thickness));
    let (settled, clamped) = settle_within(&result.placements, &cavity, spec, input_margin);
    result.placements = settled.placements;
    result.tidiness = tidiness(&result.placements, &area);
    let (walls, wall_report) =
        layout_walls_reporting(&result.placements, &area, spec.divider_thickness);
    result.walls = walls;
    let pockets = drawer_pockets(&result.placements, spec.divider_thickness);
    let split_lines = compute_auto_split_lines(cells, spec.printer, spec.pitch);
    let params = drawer_params(spec, cells, &pockets, &split_lines);
    Plan {
        result,
        absorbed: settled.absorbed,
        evened: settled.evened,
        grown: settled.grown,
        clamped,
        grouping: None,
        pockets,
        params,
        wall_report,
        bins: Vec::new(),
    }
}

/// `extra` moved by the same normalise, quarter turn and translation the packer
/// put `footprint` through to reach `placement`.
///
/// The packer normalises a shape, turns it and translates it, and hands back
/// only the result, so a caller carrying anything else in that shape's frame --
/// the compartments inside a bin -- has to reproduce the transform. Rotating the
/// two lists **as one** is what makes it the same transform: `rotate_parts`
/// normalises to the combined bounding box, which is the footprint's own,
/// because everything in `extra` lies inside the footprint. The assertion is the
/// point of the function: the footprint put through it must come back as the
/// packer's own answer, or the two lists have parted company.
fn place_in_drawer(footprint: &[Rect], extra: &[Rect], placement: &Placement) -> Vec<Rect> {
    assert_eq!(
        footprint.len(),
        placement.parts.len(),
        "the bin placed as {} box(es) was planned as {}",
        placement.parts.len(),
        footprint.len()
    );
    let mut combined: Vec<Rect> = footprint.to_vec();
    combined.extend_from_slice(extra);
    let turned = rotate_parts(&combined, placement.rotation);
    let corner = parts_bounds(&placement.parts);
    let mut moved = translate_parts(&turned, corner.x, corner.y);
    for (got, want) in moved.iter().zip(&placement.parts) {
        assert!(
            (got.x - want.x).abs() < 1e-6
                && (got.y - want.y).abs() < 1e-6
                && (got.width - want.width).abs() < 1e-6
                && (got.depth - want.depth).abs() < 1e-6,
            "reproducing the packer's placement gave {got:?} where it placed {want:?}"
        );
    }
    moved.split_off(footprint.len())
}

/// The two quarter turns composed: a compartment turned inside its bin, and the
/// bin turned inside the drawer.
fn compose(inner: Rotation, outer: Rotation) -> Rotation {
    let degrees = (inner.degrees() + outer.degrees()) % 360;
    Rotation::try_from(degrees)
        .unwrap_or_else(|e| panic!("{inner:?} after {outer:?} is not a quarter turn: {e}"))
}

/// The drawer laid out from bins already planned: the bins packed into it, every
/// instance composed through both levels into the drawer's own millimetres, and
/// the cavity stated as one pocket per claim.
///
/// The outer pack is a plain `pack_layout` whose area is the grid's own
/// rectangle and whose shapes are the bins' cells as pitch-sized squares, with
/// every margin zero because a Gridfinity bin already stands `HALF_TOL` inside
/// its cells. Every part offset, every extent and the area's origin are
/// multiples of the pitch, so every position the scan can reach is one too;
/// `cells_of_rects` asserts it rather than trusting it.
///
/// This is the whole of what `bins` and `hybrid` share, and all they differ in is
/// which groups they hand it: one object each, or whatever `choose_groups`
/// settled on. `Plan::result` therefore means in both exactly what it means in
/// `walls` -- the placement of every instance in drawer millimetres -- and the
/// pockets come from the composed claims by the same `drawer_pockets`, a pocket
/// being a claim inset by half a divider whichever frame the claim was solved in.
fn lay_out_bins(spec: &Spec, grid: DrawerGrid, plans: &[GroupPlan]) -> Result<Plan, String> {
    let mut seen: Vec<&str> = plans
        .iter()
        .flat_map(|p| p.objects.iter().map(String::as_str))
        .collect();
    let before = seen.len();
    seen.sort_unstable();
    seen.dedup();
    assert_eq!(
        (seen.len(), before),
        (spec.objects.len(), spec.objects.len()),
        "the bins to lay out must hold each of the run's {} object(s) exactly once",
        spec.objects.len()
    );

    let drawer = Rect::new(
        0.0,
        0.0,
        f64::from(grid.cols) * spec.pitch,
        f64::from(grid.rows) * spec.pitch,
    );
    let outer = outer_pack(spec, grid, plans, spec.effort);
    let iterations = outer.iterations + plans.iter().map(|p| p.iterations).sum::<usize>();

    let mut logical: Vec<LogicalBin> = Vec::new();
    let mut fitted: Vec<FittedBin> = Vec::new();
    let mut pockets: Vec<Pocket> = Vec::new();
    let mut placements: Vec<Placement> = Vec::new();
    let mut placed_by_object_id: BTreeMap<String, u32> = BTreeMap::new();
    for plan in plans {
        let id = plan.objects.join("+");
        let Some(placement) = outer.placements.iter().find(|p| p.object_id == id) else {
            return Err(format!(
                "the drawer has no room for {}'s own bin, which is {} cell(s); the bins already                  placed leave it nowhere to stand",
                plan.objects.join(" + "),
                plan.cells.len()
            ));
        };
        let footprint: Vec<Rect> = plan
            .cells
            .iter()
            .map(|c| cell_rect(*c, spec.pitch))
            .collect();
        let claims: Vec<Rect> = plan
            .placements
            .iter()
            .flat_map(|p| p.parts.clone())
            .collect();
        let moved = place_in_drawer(&footprint, &claims, placement);
        let mut taken = 0;
        let mut mine: Vec<Placement> = Vec::new();
        for local in &plan.placements {
            let parts = moved[taken..taken + local.parts.len()].to_vec();
            taken += local.parts.len();
            mine.push(Placement {
                object_id: local.object_id.clone(),
                instance: local.instance,
                rotation: compose(local.rotation, placement.rotation),
                parts,
            });
        }
        assert_eq!(
            taken,
            moved.len(),
            "every claim box of the bin is accounted for"
        );

        let cells = cells_of_rects(&placement.parts, spec.pitch);
        for cell in &cells {
            assert!(
                !logical.iter().any(|b| b.cells.contains(cell)),
                "{cell:?} is claimed by two bins, which cannot both stand in it"
            );
        }
        let split_lines = compute_auto_split_lines(&cells, spec.printer, spec.pitch);
        let mine_pockets = drawer_pockets(&mine, spec.divider_thickness);
        logical.push(LogicalBin {
            cells: cells.clone(),
            split_lines: split_lines.clone(),
            slope: None,
            pockets: mine_pockets.clone(),
        });
        fitted.push(FittedBin {
            objects: plan.objects.clone(),
            cells,
            instances: mine.len(),
            split_lines,
        });
        for id in &plan.objects {
            let held = mine.iter().filter(|p| &p.object_id == id).count() as u32;
            placed_by_object_id.insert(id.clone(), held);
        }
        pockets.extend(mine_pockets);
        placements.extend(mine);
    }

    Ok(Plan {
        result: PackResult {
            tidiness: tidiness(&placements, &drawer),
            placements,
            placed_by_object_id,
            iterations,
            walls: Vec::new(),
        },
        absorbed: plans.iter().map(|p| p.absorbed).sum(),
        evened: plans.iter().map(|p| p.evened).sum(),
        grown: plans.iter().map(|p| p.grown).sum(),
        clamped: plans.iter().map(|p| p.clamped).sum(),
        pockets,
        params: bins_params(spec, logical),
        wall_report: WallReport::default(),
        bins: fitted,
        grouping: None,
    })
}

/// One discrete Gridfinity bin per object, each sized to hold that object's whole
/// quantity as its own compartments, packed into the drawer.
///
/// Two levels of packing: each object into the smallest bin that holds it
/// (`plan_group_bin` over a group of one), and then the bins into the drawer
/// (`lay_out_bins`).
fn plan_bins(spec: &Spec, grid: DrawerGrid, floor_fillet: f64) -> Result<Plan, String> {
    let mut plans: Vec<GroupPlan> = Vec::new();
    for object in &spec.objects {
        plans.push(plan_group_bin(
            spec,
            &[object],
            grid,
            floor_fillet,
            spec.effort,
        )?);
    }
    lay_out_bins(spec, grid, &plans)
}

/// One Gridfinity bin per *group* of objects, the groups chosen by
/// `choose_groups`, packed into the drawer.
///
/// The only difference from `plan_bins` is which objects share a bin, and the
/// search that decides it can only return a grouping `better` than one bin per
/// object -- it starts there and asserts it before returning. So a hybrid fit
/// stands on the same two levels of packing, uses no more cells, and may fit a
/// drawer a bin per object cannot be laid out in.
fn plan_hybrid(spec: &Spec, grid: DrawerGrid, floor_fillet: f64) -> Result<Plan, String> {
    let groups = choose_groups(spec, grid, floor_fillet)?;
    let mut plan = lay_out_bins(spec, grid, &groups.plans)?;
    plan.grouping = Some(groups.grouping);
    Ok(plan)
}

/// `Ok` when every instance of every object was placed, and an error naming
/// every shortfall otherwise.
///
/// A run that cannot hold what it was given has not fitted the drawer, so it
/// fails rather than quietly building geometry missing compartments. Asked
/// before anything is built, so a refusal costs no work and leaves the disk
/// alone.
fn refuse_unplaced(spec: &Spec, result: &PackResult) -> Result<(), String> {
    let mut short: Vec<String> = Vec::new();
    for object in &spec.objects {
        let placed = result
            .placed_by_object_id
            .get(&object.pack.id)
            .copied()
            .unwrap_or(0);
        assert!(
            placed <= object.pack.quantity,
            "{} was placed {placed} times, more than the {} that were wanted",
            object.pack.name,
            object.pack.quantity
        );
        if placed < object.pack.quantity {
            short.push(format!(
                "{} ({placed} of {} placed)",
                object.pack.name, object.pack.quantity
            ));
        }
    }
    if short.is_empty() {
        return Ok(());
    }
    Err(format!(
        "the drawer does not hold everything it was given: {} -- state a bigger drawer, fewer \
         objects, or a smaller settings.clearance or settings.fillet_radius",
        short.join(", ")
    ))
}

/// One mode's plan for this drawer, checked against the quantities it was asked
/// for, or the reason that mode cannot fit the drawer.
///
/// The check belongs to the plan rather than to the pipeline after it, because
/// `Auto` chooses between the two modes on exactly this question: a plan that
/// leaves an instance unplaced has not fitted the drawer, whether it said so
/// itself or only left the shortfall to be counted. So a caller holding a plan
/// holds one that places every instance of every object.
fn plan_and_check(
    spec: &Spec,
    built: Built,
    grid: DrawerGrid,
    cells: &[GridCell],
    area: Rect,
    floor_fillet: f64,
) -> Result<Plan, String> {
    let plan = match built {
        Built::Walls => plan_walls(spec, cells, area, floor_fillet),
        Built::Bins => plan_bins(spec, grid, floor_fillet)?,
        Built::Hybrid => plan_hybrid(spec, grid, floor_fillet)?,
    };
    refuse_unplaced(spec, &plan.result)?;
    Ok(plan)
}

/// The whole pipeline for one validated run: pack, state the cavities, split,
/// build.
///
/// `mode` decides only the middle of it -- what the drawer is divided into --
/// and everything either side is shared: the drawer resolves to the same grid
/// and the same packing rectangle, the baseplate spans the same cells and is
/// staggered off whatever seams the bins came to, and every piece is built and
/// audited the same way. A run that cannot place everything it was given fails
/// here, before any of that work is done.
///
/// `FitMode::Auto` is resolved here and nowhere else: the discrete bins are
/// tried first -- grouped where grouping pays, which is `Hybrid` and can only
/// beat one bin per object -- and the drawer-wide bin is built only where that
/// plan is refused, because a discrete bin is the smaller print and the smaller
/// thing to lose. The refusal it fell back from is carried on the `Run`, so a reader
/// of the report is told which of the two was built and why. A drawer neither
/// plan can hold fails with the *walls* refusal, which is the message naming
/// the shortfall against the whole drawer rather than against one bin.
fn fit(spec: Spec, mode: FitMode) -> Result<Run, String> {
    let grid = drawer_grid(spec.drawer_width, spec.drawer_depth, MAX_GRID, spec.pitch);
    if grid.cols == 0 || grid.rows == 0 {
        return Err(format!(
            "a drawer of {} x {} mm does not hold one {} mm Gridfinity cell",
            spec.drawer_width, spec.drawer_depth, spec.pitch
        ));
    }
    let cells = drawer_cells(grid);
    let area = packing_area(grid, spec.wall_thickness, spec.pitch);
    let floor_fillet = spec.built_floor_fillet();
    let claim_margin = claim_input(&spec, area, floor_fillet, Vec::new(), spec.effort).margin();

    let started = Instant::now();
    let plan_for = |built| plan_and_check(&spec, built, grid, &cells, area, floor_fillet);
    let (built, fell_back, plan) = match mode {
        FitMode::Walls => (Built::Walls, None, plan_for(Built::Walls)?),
        FitMode::Bins => (Built::Bins, None, plan_for(Built::Bins)?),
        FitMode::Hybrid => (Built::Hybrid, None, plan_for(Built::Hybrid)?),
        FitMode::Auto => match plan_for(Built::Hybrid) {
            Ok(plan) => (Built::Hybrid, None, plan),
            Err(why) => (Built::Walls, Some(why), plan_for(Built::Walls)?),
        },
    };
    let pack_time = started.elapsed();

    let Plan {
        result,
        absorbed,
        evened,
        grown,
        clamped,
        grouping,
        pockets,
        params,
        wall_report,
        bins,
    } = plan;
    if params.bins.iter().all(|b| b.cells.is_empty()) && !spec.baseplate {
        return Err(
            "this run builds nothing: it has no object to give a bin to and settings.baseplate \
             is off"
                .to_string(),
        );
    }
    let split_lines = all_split_lines(&params);
    let parts = all_parts(&params);
    let plate_split_lines = plate_splits(&spec, grid, &cells, &split_lines);
    let plate_parts = if spec.baseplate {
        partition_cells(&cells, &plate_split_lines)
    } else {
        Vec::new()
    };
    let free_plate = plate_pieces_unstaggered(&spec, grid, &cells);
    assert!(
        !spec.baseplate || plate_parts.len() >= free_plate,
        "staggering the plate's seams off the bin's brought it to {} piece(s), fewer than the \
         {free_plate} it comes to when free to be cut anywhere",
        plate_parts.len()
    );
    let plate_stagger_cost = plate_parts.len().saturating_sub(free_plate);

    let plate_params = spec
        .baseplate
        .then(|| baseplate_params(&spec, grid, &cells, &plate_split_lines));

    let started = Instant::now();
    #[cfg(not(feature = "occt"))]
    let (pieces, legacy_blends) = gridfinity::try_build_pieces_reporting(&params)?;
    #[cfg(not(feature = "occt"))]
    let blends = BlendReport {
        requested: legacy_blends.requested,
        unresolved: legacy_blends.unresolved,
        dropped: (0..legacy_blends.dropped.len()).collect(),
        refusal: legacy_blends.refusal,
    };
    #[cfg(feature = "occt")]
    let pieces = gridfinity_model::try_build_pieces_occt(&params)?;
    #[cfg(feature = "occt")]
    let blends = BlendReport::default();
    let baseplate = match &plate_params {
        #[cfg(not(feature = "occt"))]
        Some(plate) => gridfinity::try_build_pieces(plate)?,
        #[cfg(feature = "occt")]
        Some(plate) => gridfinity_model::try_build_pieces_occt(plate)?,
        None => Vec::new(),
    };
    let subbins = build_subbins(&spec, &result, &bins, floor_fillet)?;
    let build_time = started.elapsed();
    assert_eq!(
        pieces.len(),
        parts.len(),
        "the model built {} pieces for a partition of {} -- the report would name the wrong cells",
        pieces.len(),
        parts.len()
    );
    assert_eq!(
        baseplate.len(),
        plate_parts.len(),
        "the baseplate is cut on its own staggered split lines, so it comes to {} piece(s), not {}",
        plate_parts.len(),
        baseplate.len()
    );

    let mut run = Run {
        spec,
        built,
        fell_back,
        grid,
        area,
        cells,
        result,
        floor_fillet,
        claim_margin,
        wall_report,
        absorbed,
        evened,
        grown,
        clamped,
        pockets,
        bins,
        grouping,
        subbins,
        params,
        plate_params,
        split_lines,
        parts,
        pieces,
        plate_split_lines,
        plate_parts,
        plate_stagger_cost,
        baseplate,
        blends,
        soundness: Vec::new(),
        pack_time,
        build_time,
        export_time: Duration::ZERO,
    };
    run.soundness = soundness_of(&run.all_pieces());
    Ok(run)
}

/// One `optimize` invocation, end to end: read, fit, write, report. Returns the
/// bin to open a window on, and the object boxes to draw in it, when `--view`
/// was given, and `None` otherwise.
///
/// An invocation with no `-o` writes nothing and reports an empty Output
/// section -- it fits, shows, and leaves the disk alone.
pub fn run(args: &Args) -> Result<Option<View>, String> {
    let destination = args.destination()?;
    if let Some((format, path)) = &destination {
        format.check_output(path)?;
    }
    let text = std::fs::read_to_string(&args.input)
        .map_err(|e| format!("could not read {}: {e}", args.input.display()))?;
    let spec = input::parse(&text).map_err(|e| format!("{}: {e}", args.input.display()))?;
    let mut run = crate::catch(|| fit(spec, args.mode))?;

    let started = Instant::now();
    #[cfg(not(feature = "occt"))]
    let written = {
        let all = run.all_pieces();
        match &destination {
            Some((Format::Stl, path)) => export::write_stl_dir(path, &all)?,
            Some((Format::ParasolidXt, path)) => vec![export::write_xt(path, &all)?],
            None => Vec::new(),
        }
    };
    #[cfg(feature = "occt")]
    let written = match &destination {
        Some((format, path)) => {
            let bodies = occt_bodies(&run);
            match format {
                Format::Stl => export::write_occt_stl_dir(path, &bodies)?,
                Format::ParasolidXt => vec![export::write_occt_xt(path, &bodies)?],
            }
        }
        None => Vec::new(),
    };
    run.export_time = started.elapsed();

    report::print(&run, &written);
    let (bin_files, plate_files) = piece_files(&run);
    let subbins = run
        .subbins
        .iter()
        .map(|s| PlacedSubbin {
            label: s.object.clone(),
            file: s.name.clone(),
            bin: s.bin,
            spec: s.spec,
        })
        .collect();
    Ok(args.view.then(|| View {
        boxes: object_boxes(&run),
        subbins,
        bin_names: run.bins.iter().map(FittedBin::name).collect(),
        bin_files,
        plate_files,
        params: run.params,
        plate: run.plate_params,
    }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use gridfinity_model::layout::{Axis, GridFootprint, compartments};
    #[cfg(not(feature = "occt"))]
    use gridfinity_sketch::math::Vec2;
    #[cfg(not(feature = "occt"))]
    use gridfinity_sketch::sketch::point_in_polygon;

    fn assert_positive_shells(solid: &Solid, name: &str) {
        #[cfg(feature = "occt")]
        assert!(
            solid
                .shell_volumes()
                .expect("OCCT shell query")
                .iter()
                .all(|volume| *volume > 0.0),
            "{name} bounds no sealed void"
        );
        #[cfg(not(feature = "occt"))]
        assert!(
            solid.shells().iter().all(|shell| shell.encloses_material),
            "{name} bounds no sealed void"
        );
    }

    /// The command line `clap` would see, parsed as `Args`, or the message it
    /// refuses with. The leading `optimize` is the subcommand's own name, which
    /// `clap` expects as argv[0] of a subcommand's argument list.
    ///
    /// A `--mode` is supplied when the caller states none, because every test
    /// but `refuses_an_invocation_that_does_not_state_a_mode` is about one of
    /// the other arguments and would say nothing by repeating it. That one
    /// parses without this helper, so the requirement is still tested.
    fn args(argv: &[&str]) -> Result<Args, String> {
        let mut all = vec!["optimize"];
        all.extend_from_slice(argv);
        if !argv.contains(&"--mode") {
            all.extend_from_slice(&["--mode", "walls"]);
        }
        Args::try_parse_from(all).map_err(|e| e.to_string())
    }

    /// The modes build entirely different sets of parts out of one file, so an
    /// invocation that has not said which it wants has not said what it wants
    /// built -- `auto` included, which asks for the smallest bins that fit and
    /// is therefore an answer rather than the absence of one.
    #[test]
    fn refuses_an_invocation_that_does_not_state_a_mode() {
        let err = Args::try_parse_from(["optimize", "in.toml", "-o", "out.x_t"])
            .map(|_| ())
            .expect_err("--mode is required")
            .to_string();
        assert!(err.contains("--mode"), "{err}");
    }

    #[test]
    fn reads_the_mode_the_command_line_states() {
        assert_eq!(
            args(&["in.toml", "--mode", "bins", "--view"])
                .expect("bins is a mode")
                .mode,
            FitMode::Bins
        );
        assert_eq!(
            args(&["in.toml", "--mode", "walls", "--view"])
                .expect("walls is a mode")
                .mode,
            FitMode::Walls
        );
        assert_eq!(
            args(&["in.toml", "--mode", "auto", "--view"])
                .expect("auto is a mode")
                .mode,
            FitMode::Auto
        );
        assert_eq!(
            args(&["in.toml", "--mode", "hybrid", "--view"])
                .expect("hybrid is a mode")
                .mode,
            FitMode::Hybrid
        );
        let err =
            args(&["in.toml", "--mode", "pockets", "--view"]).expect_err("there are four modes");
        assert!(err.contains("pockets"), "{err}");
    }

    /// What one invocation writes, as `(format, path)`, or the message saying
    /// why it can write nothing.
    fn destination(argv: &[&str]) -> Result<Option<(Format, PathBuf)>, String> {
        args(argv)?.destination()
    }

    #[test]
    fn reads_the_documented_invocation() {
        let a = args(&["in.toml", "--format", "stl", "-o", "out"]).expect("valid invocation");
        assert_eq!(a.input, PathBuf::from("in.toml"));
        assert_eq!(a.output, Some(PathBuf::from("out")));
        assert_eq!(a.format, Some(Format::Stl));
        assert!(!a.view);
        assert_eq!(
            a.destination().expect("stl into a directory"),
            Some((Format::Stl, PathBuf::from("out")))
        );
    }

    #[test]
    fn takes_view_anywhere_among_the_arguments() {
        let a = args(&["--view", "in.toml", "-o", "out.x_t"]).expect("valid invocation");
        assert!(a.view);
        assert_eq!(
            a.destination().expect("x_t names its own format"),
            Some((Format::ParasolidXt, PathBuf::from("out.x_t")))
        );
    }

    /// The one abbreviation the documented invocation leans on: a `.x_t` output
    /// says which format it is, so `--format` may be left off entirely.
    #[test]
    fn infers_parasolid_from_the_outputs_extension() {
        assert_eq!(
            destination(&["in.toml", "-o", "drawer.X_T"]).expect("the extension names the format"),
            Some((Format::ParasolidXt, PathBuf::from("drawer.X_T")))
        );
    }

    /// An STL run writes a *directory*, so its output carries no extension and
    /// there is nothing to infer from.
    #[test]
    fn refuses_an_output_that_names_no_format() {
        let err =
            destination(&["in.toml", "-o", "out"]).expect_err("a bare directory names no format");
        assert!(err.contains("--format is required"), "{err}");
    }

    #[test]
    fn refuses_an_output_whose_extension_contradicts_the_format() {
        let err = destination(&["in.toml", "--format", "parasolid_x_t", "-o", "out"])
            .expect_err("x_t must be written to a .x_t file");
        assert!(err.contains(".x_t"), "{err}");
        let err = destination(&["in.toml", "--format", "stl", "-o", "out/drawer.stl"])
            .expect_err("stl names a directory, not a file");
        assert!(err.contains("directory"), "{err}");
    }

    /// `--view` alone is a complete invocation: fit it, show it, write nothing.
    #[test]
    fn takes_a_view_only_invocation_and_writes_nothing() {
        let a = args(&["in.toml", "--view"]).expect("--view alone asks for something");
        assert_eq!(a.output, None);
        assert_eq!(a.destination().expect("nothing to write"), None);
    }

    #[test]
    fn refuses_an_invocation_that_neither_writes_nor_shows() {
        let err = args(&["in.toml"]).expect_err("-o is required without --view");
        assert!(err.contains("--output"), "{err}");
    }

    #[test]
    fn refuses_a_format_that_is_not_wired() {
        let err = args(&["in.toml", "--format", "step", "-o", "out"])
            .expect_err("STEP is not an export format here");
        assert!(err.contains("step"), "{err}");
    }

    #[test]
    fn refuses_an_invocation_missing_its_input() {
        let err = args(&["--format", "stl", "-o", "out"]).expect_err("the input is required");
        assert!(err.contains("INPUT"), "{err}");
    }

    /// A drawer small enough to build quickly and busy enough to need dividers:
    /// two cells square, four objects that tile it.
    ///
    /// 30 mm and not 34: a claim is the object plus `2 * (clearance +
    /// floor_fillet + divider/2)` = 7.16 mm at these settings, and two 34 mm
    /// blocks' claims are 82.32 mm across an 81.1 mm packing area. Reserving the
    /// fillet is what costs that, and the fixture is sized for what the drawer
    /// can really hold rather than for what fits in plan view.
    const SMALL: &str = "\
[drawer]
width = 84
depth = 84

[settings]
effort = \"quick\"

[[objects]]
name = \"block\"
quantity = 4
size = [30, 30]
";

    /// The same four blocks in a drawer half again as wide: they claim two thirds
    /// of it, so the column of leftover they cannot reach is far wider than the
    /// widest strip settling absorbs and survives the pass as material.
    #[cfg(not(feature = "occt"))]
    const ROOMY: &str = "[drawer]
width = 126
depth = 84

[settings]
effort = \"quick\"

[[objects]]
name = \"block\"
quantity = 4
size = [30, 30]
";

    /// The same drawer on a half-pitch grid: 21 mm cells, so the 84 mm drawer is
    /// four cells square rather than two, and the objects are sized for what one
    /// compartment of that grid holds.
    const HALF_PITCH: &str = "[drawer]
width = 84
depth = 84

[settings]
grid_size = 21
effort = \"quick\"

[[objects]]
name = \"block\"
quantity = 4
size = [15, 15]
";

    #[test]
    fn measures_the_drawer_in_the_grid_size_the_file_states() {
        let spec = input::parse(HALF_PITCH).expect("a stated grid size is a valid run");
        assert_eq!(spec.pitch, 21.0);
        let run = fit(spec, FitMode::Walls).expect("a drawer on a 21 mm grid builds");

        assert_eq!(
            (run.grid.cols, run.grid.rows),
            (4, 4),
            "an 84 mm drawer is four 21 mm cells across, not two 42 mm ones"
        );
        assert_eq!(
            run.params.pitch, 21.0,
            "the bin is built on the grid it was fitted on"
        );
        assert!(
            (run.area.width - (4.0 * 21.0 - 2.0 * run.area.x)).abs() < 1e-9,
            "the packing area follows the pitch, not {:?}",
            run.area
        );
        assert_eq!(run.result.placements.len(), 4, "four 15 mm blocks fit it");
    }

    /// A bin on a grid that is not the standard's is held to everything a
    /// standard one is: `carve_to_cells` asserts soundness as it produces each
    /// piece, so reaching the end of `fit` is most of the statement, and the
    /// baseplate under it has to come off the same grid.
    #[test]
    fn a_bin_on_a_stated_grid_is_as_sound_as_one_on_the_standard() {
        let spec = input::parse(HALF_PITCH).expect("a stated grid size is a valid run");
        let run = fit(spec, FitMode::Walls).expect("a drawer on a 21 mm grid builds");

        for (piece, sound) in run.all_pieces().iter().zip(&run.soundness) {
            assert_eq!(sound.shells, 1, "{} is one shell", piece.name);
            assert_positive_shells(piece.solid, piece.name);
        }
        assert_eq!(
            run.blends.made(),
            run.blends.requested,
            "a finer grid does not cost the compartments their floor fillets"
        );
        let (min, max) = run.pieces.iter().fold(
            (f64::INFINITY, f64::NEG_INFINITY),
            |(lo, hi), piece| {
                let (x0, x1, _, _) = extent(piece);
                (lo.min(x0), hi.max(x1))
            },
        );
        assert!(
            (min - 0.25).abs() < 0.2 && (max - (4.0 * 21.0 - 0.25)).abs() < 0.2,
            "a four-cell bin of 21 mm cells spans 0.25..83.75 mm, not {min}..{max}"
        );
    }

    #[test]
    fn fits_a_drawer_end_to_end() {
        let spec = input::parse(SMALL).expect("the fixture is a valid run");
        let run = fit(spec, FitMode::Walls).expect("a two-cell drawer of four blocks builds");

        assert_eq!((run.grid.cols, run.grid.rows), (2, 2));
        assert_eq!(run.cells.len(), 4);
        assert_eq!(
            run.result.placements.len(),
            4,
            "four 30 mm blocks fit an 84 mm drawer"
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
size = [30, 30, 5]

[[objects]]
name = \"high\"
quantity = 2
size = [30, 30, 200]
";

    #[test]
    fn every_placed_object_stands_on_the_cavity_floor_inside_the_packing_area() {
        let spec = input::parse(TALL).expect("the fixture is a valid run");
        let run = fit(spec, FitMode::Walls).expect("a two-cell drawer of four blocks builds");
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
                "an object that does not fit still stands its full height, or the drawing \
                 hides the warning"
            );
        }
    }

    /// Every compartment floor of a built piece, as the polygon its outer loop
    /// traces in XY.
    ///
    /// A cavity floor is a `Plane` with a vertical normal at the one height
    /// `BASE_TOTAL_HEIGHT + FLOOR_THICKNESS`, and nothing else in the model sits
    /// there -- the same identification `floor_fillet_coverage` makes. The loop
    /// is read off the *finished* solid, so it is the floor the floor fillet has
    /// already trimmed back, which is the whole point of asking.
    #[cfg(not(feature = "occt"))]
    fn compartment_floors(solid: &Solid) -> Vec<Vec<Vec2>> {
        let floor_z = gridfinity::BASE_TOTAL_HEIGHT + gridfinity::FLOOR_THICKNESS;
        let mut out = Vec::new();
        for fid in 0..solid.faces.len() {
            let Surface::Plane { origin, normal, .. } = solid.faces[fid].surface else {
                continue;
            };
            if normal.vec().z.abs() < 0.999 || (origin.z - floor_z).abs() > 1e-6 {
                continue;
            }
            let mut pts: Vec<Vec2> = Vec::new();
            for &(e, fwd) in solid.outer_edges(fid) {
                let edge = solid.edges[e];
                for p in edge.sample(fwd, edge.seg_count(24)) {
                    pts.push(Vec2::new(p.x, p.y));
                }
            }
            assert!(
                pts.len() >= 3,
                "a floor face traces {} points, which bounds no area",
                pts.len()
            );
            out.push(pts);
        }
        out
    }

    /// The check the drawn boxes are a picture of: every packed object's
    /// footprint lies inside a compartment floor the model actually built.
    ///
    /// This is the independent statement of what reserving the fillet is for. It
    /// asks the finished B-rep where the floor is rather than re-deriving it from
    /// the same margin the packer used, so a reservation that is too small fails
    /// here even though the packing is self-consistent. Without the reservation
    /// every corner of every object lands inside the blend.
    #[cfg(not(feature = "occt"))]
    #[test]
    fn every_packed_object_fits_the_compartment_floor_the_model_built() {
        let spec = input::parse(SMALL).expect("the fixture is a valid run");
        let run = fit(spec, FitMode::Walls).expect("a two-cell drawer of four blocks builds");
        assert_eq!(run.pieces.len(), 1, "a two-cell drawer needs no splitting");
        assert!(
            run.floor_fillet > run.spec.clearance,
            "the fixture must reserve more than its clearance, or it cannot tell the \
             reservation from the clearance: fillet {} against clearance {}",
            run.floor_fillet,
            run.spec.clearance
        );

        let floors = compartment_floors(&run.pieces[0].solid);
        assert_eq!(
            floors.len(),
            run.result.placements.len(),
            "a compartment floor is a packed object's pocket and nothing else; leftover area is material"
        );
        for b in object_boxes(&run) {
            for corner in [
                Vec2::new(b.min.x, b.min.y),
                Vec2::new(b.max.x, b.min.y),
                Vec2::new(b.max.x, b.max.y),
                Vec2::new(b.min.x, b.max.y),
            ] {
                assert!(
                    floors.iter().any(|f| point_in_polygon(f, corner)),
                    "the object corner {corner} stands on no compartment floor -- it is inside \
                     the floor fillet, so the object does not sit in the compartment \
                     packed for it"
                );
            }
        }
    }

    /// The bin's pegs need a grid to sit in, so a run builds one by default, on
    /// the bin's own cells. A two-cell drawer is cut nowhere, so both bodies are
    /// one piece and the stack is trivially interlocked.
    #[test]
    fn builds_the_baseplate_the_bin_drops_into() {
        let spec = input::parse(SMALL).expect("the fixture is a valid run");
        let run = fit(spec, FitMode::Walls).expect("a two-cell drawer of four blocks builds");

        assert_eq!(run.baseplate.len(), run.plate_parts.len());
        assert!(run.split_lines.is_empty() && run.plate_split_lines.is_empty());
        assert!(run.interlocked(), "an uncut stack is one piece already");
        assert_eq!(run.plate_stagger_cost, 0);
        assert_eq!(
            run.all_pieces().len(),
            run.pieces.len() + run.baseplate.len()
        );
        assert_eq!(
            run.soundness.len(),
            run.all_pieces().len(),
            "every piece written is a piece the soundness section accounts for"
        );
        for piece in &run.baseplate {
            assert!(piece.name.contains("baseplate"), "{}", piece.name);
            assert_eq!(soundness_of(&[Body { name: &piece.name, solid: &piece.solid }])[0].shells, 1);
            assert_positive_shells(&piece.solid, &piece.name);
        }
    }

    /// A drawer too long for the bed on one axis: seven cells across, so both
    /// the bin and the plate under it have to be cut.
    const LONG: &str = "[drawer]
width = 300
depth = 100

[settings]
effort = \"quick\"

[[objects]]
name = \"block\"
quantity = 6
size = [30, 30]
";

    /// Whether some piece of `parts` holds the cells on both sides of `line`.
    /// That is what it means for the piece to span the seam: it is pegged to
    /// both of the pieces the line separates, so neither can leave without it.
    fn spans(parts: &[Piece], line: SplitLine) -> bool {
        let along = |c: &GridCell| match line.axis {
            Axis::X => c.x,
            Axis::Y => c.y,
        };
        parts.iter().any(|p| {
            p.cells.iter().any(|c| along(c) == line.index - 1)
                && p.cells.iter().any(|c| along(c) == line.index)
        })
    }

    /// The whole point of staggering: the two bodies part on different lines, so
    /// every seam of each is spanned by a piece of the other and no piece of the
    /// stack can be lifted out on its own.
    #[test]
    fn the_baseplate_is_cut_beside_the_bins_seams_so_the_stack_holds_itself_together() {
        let spec = input::parse(LONG).expect("the fixture is a valid run");
        let run = fit(spec, FitMode::Walls).expect("a seven-cell drawer builds");

        assert!(
            !run.split_lines.is_empty(),
            "294 mm of cells does not print whole"
        );
        assert!(
            !run.plate_split_lines.is_empty(),
            "nor does the plate that spans them"
        );
        assert!(run.interlocked(), "the two bodies share a cut line");
        assert_eq!(
            run.plate_stagger_cost, 0,
            "staggering this plate costs it no piece"
        );
        for line in &run.split_lines {
            assert!(
                spans(&run.plate_parts, *line),
                "no baseplate piece spans the bin's seam at {line:?}, so the bin's pieces are \
                 held together by nothing"
            );
        }
        let plate = run
            .plate_params
            .as_ref()
            .expect("a fitted drawer ships its grid");
        assert_eq!(plate.mode, Mode::Baseplate);
        assert_eq!(
            plate.bins[0].split_lines, run.plate_split_lines,
            "the window rebuilds the plate on the lines the export cut it on"
        );
        for line in &run.plate_split_lines {
            assert!(
                spans(&run.parts, *line),
                "no bin piece spans the plate's seam at {line:?}, so the plate's pieces are held \
                 together by nothing"
            );
        }
    }

    /// The plate is measured where it reaches to, not over the cells it covers,
    /// so its own plan puts every piece on the bed -- the flange included.
    #[test]
    fn every_staggered_baseplate_piece_prints() {
        let spec = input::parse(LONG).expect("the fixture is a valid run");
        let run = fit(spec, FitMode::Walls).expect("a seven-cell drawer builds");

        assert_eq!(run.baseplate.len(), run.plate_parts.len());
        for piece in &run.baseplate {
            let (lo_x, width, lo_y, depth) = extent(piece);
            let fit = run.spec.printer.bed_fit_mm(width - lo_x, depth - lo_y);
            assert!(
                fit.fits,
                "{} measures {} x {} mm, which the bed the seams were placed for does not take",
                piece.name,
                width - lo_x,
                depth - lo_y
            );
        }
    }

    /// The same four blocks in a drawer the grid does *not* divide evenly: 100 mm
    /// is two 42 mm cells and 16 mm of margin on each axis.
    const OVERSIZE: &str = "[drawer]
width = 100
depth = 100

[settings]
effort = \"quick\"

[[objects]]
name = \"block\"
quantity = 4
size = [30, 30]
";

    /// The XY extent of a built piece, as `(min_x, max_x, min_y, max_y)`.
    fn extent(piece: &BinPiece) -> (f64, f64, f64, f64) {
        #[cfg(feature = "occt")]
        {
            let b = piece.solid.bounds().expect("built OCCT piece has bounds");
            return (b.min[0], b.max[0], b.min[1], b.max[1]);
        }
        #[cfg(not(feature = "occt"))]
        {
        piece.solid.verts.iter().fold(
            (
                f64::INFINITY,
                f64::NEG_INFINITY,
                f64::INFINITY,
                f64::NEG_INFINITY,
            ),
            |(lx, hx, ly, hy), v| {
                (
                    lx.min(v.point.x),
                    hx.max(v.point.x),
                    ly.min(v.point.y),
                    hy.max(v.point.y),
                )
            },
        )
        }
    }

    /// The plate is the body that touches the drawer walls, so it is the body
    /// that spans it: the drawer's leftover millimetres become a flange, half on
    /// each side, and the bin inside it is untouched.
    #[test]
    fn the_baseplate_spans_the_drawer_the_bin_is_only_cells_of() {
        let spec = input::parse(OVERSIZE).expect("the fixture is a valid run");
        let run = fit(spec, FitMode::Walls).expect("a two-cell drawer of four blocks builds");

        assert_eq!((run.grid.cols, run.grid.rows), (2, 2));
        assert!(
            (run.grid.margin_x - 16.0).abs() < 1e-9 && (run.grid.margin_y - 16.0).abs() < 1e-9,
            "a 100 mm drawer of 42 mm cells leaves 16 mm over, not {:?}",
            run.grid
        );
        assert_eq!(
            run.baseplate.len(),
            1,
            "a two-cell plate fits the bed whole"
        );
        let (lx, hx, ly, hy) = extent(&run.baseplate[0]);
        assert!(
            (hx - lx - run.spec.drawer_width).abs() < 0.3
                && (hy - ly - run.spec.drawer_depth).abs() < 0.3,
            "the plate measures {} x {} mm in a {} x {} mm drawer",
            hx - lx,
            hy - ly,
            run.spec.drawer_width,
            run.spec.drawer_depth
        );
        assert!(
            (lx + 8.0).abs() < 0.3 && (hx - (2.0 * gridfinity::GRID_PITCH + 8.0)).abs() < 0.3,
            "the flange is half the margin on each side, leaving the grid centred, not {lx}..{hx}"
        );

        assert_eq!(run.params.plate_margin_x, 0.0);
        assert_eq!(run.params.plate_margin_y, 0.0);
        let (blx, bhx, _, _) = extent(&run.pieces[0]);
        assert!(
            (blx - 0.25).abs() < 0.2
                && (bhx - (2.0 * gridfinity::GRID_PITCH - 0.25)).abs() < 0.2,
            "the bin is the cells it always was, {blx}..{bhx}"
        );
    }

    #[test]
    fn builds_no_baseplate_when_the_file_turns_it_off() {
        let text = SMALL.replace(
            "effort = \"quick\"",
            "effort = \"quick\"
baseplate = false",
        );
        let spec = input::parse(&text).expect("baseplate is a setting");
        let run = fit(spec, FitMode::Walls).expect("a two-cell drawer of four blocks builds");
        assert!(run.baseplate.is_empty());
        assert!(
            run.plate_params.is_none(),
            "there is no plate for --view to show either"
        );
        assert_eq!(run.all_pieces().len(), run.pieces.len());
    }

    /// The point of stating the cavity: space no object was packed into is
    /// material, not an open pocket of air nothing can reach. Read off the
    /// finished B-rep, so it is the solid that says so and not the packer's
    /// own bookkeeping.
    ///
    /// The fixture has leftover to find -- four 30 mm blocks in a drawer half
    /// again as wide as it is deep leave a column of it no compartment reaches,
    /// wider than settling will absorb -- and the sweep asserts it found some
    /// before asserting it is solid, since on a drawer packed full the check
    /// would pass vacuously.
    #[cfg(not(feature = "occt"))]
    #[test]
    fn the_space_no_object_was_packed_into_is_solid() {
        let spec = input::parse(
            &ROOMY.replace("effort = \"quick\"", "effort = \"quick\"\ntidy_absorb = 0"),
        )
        .expect("the fixture is a valid run");
        let run = fit(spec, FitMode::Walls).expect("a three-cell drawer of four blocks builds");
        let floors = compartment_floors(&run.pieces[0].solid);

        let claimed = |p: Vec2| {
            run.result.placements.iter().any(|pl| {
                pl.parts
                    .iter()
                    .any(|r| p.x >= r.x && p.x <= r.right() && p.y >= r.y && p.y <= r.bottom())
            })
        };
        let steps = 60;
        let mut unclaimed = 0;
        for i in 0..=steps {
            for j in 0..=steps {
                let p = Vec2::new(
                    run.area.x + run.area.width * f64::from(i) / f64::from(steps),
                    run.area.y + run.area.depth * f64::from(j) / f64::from(steps),
                );
                if claimed(p) {
                    continue;
                }
                unclaimed += 1;
                assert!(
                    !floors.iter().any(|f| point_in_polygon(f, p)),
                    "{p:?} is in the packing area, in no claim, and yet stands on a compartment floor"
                );
            }
        }
        assert!(
            unclaimed > 0,
            "the fixture packs the drawer full, so it cannot tell filled leftover from hollow"
        );
    }
    #[test]
    fn builds_one_piece_per_partition_cell_set() {
        let spec = input::parse(SMALL).expect("the fixture is a valid run");
        let run = fit(spec, FitMode::Walls).expect("a two-cell drawer of four blocks builds");

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
        let run = fit(spec, FitMode::Walls).expect("a two-cell drawer of four blocks builds");

        let pieces = run.all_pieces();
        assert_eq!(run.soundness.len(), pieces.len());
        for (piece, sound) in pieces.iter().zip(&run.soundness) {
            assert_eq!(sound.name, piece.name);
            assert_eq!(
                sound.shells, 1,
                "{} is one rectangular slab of cells, so it is one shell",
                piece.name
            );
            let mesh = piece.solid.tessellate(0.08).expect("sound body tessellates");
            assert_eq!(sound.faces, mesh.tri_count());
            assert_eq!(sound.edges, mesh.indices.len());
            assert_eq!(sound.verts, mesh.positions.len());
            assert_positive_shells(piece.solid, piece.name);
        }
    }

    /// The bin is hollowed to the pockets and carries no divider at all: a
    /// divider is what a *walked* cavity leaves between two compartments, and a
    /// stated cavity needs none.
    #[test]
    fn carries_the_packed_compartments_into_the_params_it_builds() {
        let spec = input::parse(SMALL).expect("the fixture is a valid run");
        let run = fit(spec, FitMode::Walls).expect("a two-cell drawer of four blocks builds");

        assert!(
            run.params.inner_walls.is_empty(),
            "a stated cavity needs no divider, but the bin carries {}",
            run.params.inner_walls.len()
        );
        assert_eq!(run.params.bins.len(), 1);
        assert_eq!(run.params.bins[0].cells, run.cells);
        assert_eq!(run.params.bins[0].pockets.len(), run.pockets.len());
        assert_eq!(
            run.pockets.len(),
            run.result.placements.len(),
            "every block of this fixture is a single box, so it is a single pocket"
        );
        let inset = run.spec.divider_thickness / 2.0;
        for (pocket, placement) in run.pockets.iter().zip(&run.result.placements) {
            let claim = placement.parts[0];
            assert!(
                (pocket.x - (claim.x + inset)).abs() < 1e-9
                    && (pocket.width - (claim.width - 2.0 * inset)).abs() < 1e-9,
                "a pocket is its claim inset by half a divider, but {pocket:?} came from {claim:?}"
            );
        }
    }

    #[test]
    fn refuses_a_drawer_too_small_for_one_cell() {
        let spec = input::parse("[drawer]\nwidth = 40\ndepth = 40\n").expect("a valid run");
        let Err(err) = fit(spec, FitMode::Walls) else {
            panic!("40 mm does not hold a 42 mm cell, so there is no bin to build");
        };
        assert!(err.contains("42 mm Gridfinity cell"), "{err}");
    }

    /// Two objects in a four-cell-square drawer, each wanting a bin of its own:
    /// two blocks that stack into one column of cells, and a single rod that
    /// needs the same column turned the other way up.
    const TWO: &str = "[drawer]
width = 200
depth = 200

[settings]
effort = \"quick\"

[[objects]]
name = \"block\"
quantity = 2
size = [30, 30]

[[objects]]
name = \"rod\"
size = [20, 60]
";

    /// One L-shaped object in a three-cell-square drawer: 100 x 100 mm with a
    /// 60 mm square bitten out of its far corner, so the cell in that corner is
    /// reached by nothing and the bin the object gets is an L.
    const ELL: &str = "[drawer]
width = 150
depth = 150

[settings]
effort = \"quick\"

[[objects]]
name = \"bracket\"
boxes = [
  { x = 0, y = 0, width = 100, depth = 40 },
  { x = 0, y = 40, width = 40, depth = 60 },
]
";

    /// One 30 x 20 mm object in a two-cell drawer, twice: once with the settling
    /// that grows its claim to the whole cavity and once with none at all. The two are the
    /// halves of what a drawn object is -- its own size, wherever its
    /// compartment ended up.
    const ROOMY_ONE: &str = "[drawer]
width = 100
depth = 100

[settings]
effort = \"quick\"
tidy_absorb = 100

[[objects]]
name = \"widget\"
size = [30, 20]
";

    /// One object in a roomy drawer, asking for an insert pinned on width and
    /// left open on depth, with settling free to grow the compartment as far as
    /// it likes.
    const INSERTED: &str = "[drawer]
width = 200
depth = 200

[settings]
effort = \"quick\"
tidy_absorb = 100

[[objects]]
name = \"battery\"
size =   [40, 60]
subbin = [40, \"\", 12]
";

    /// The pinned axis comes out at exactly the interior the file states, the
    /// open one fills whatever the compartment grew to, and the insert stands
    /// its own clearance inside that compartment on both.
    ///
    /// This is the whole of what a subbin promises, stated against the built
    /// geometry: the interior is read off the solid's own declaration, the
    /// compartment off the settled claim, and the two are compared rather than
    /// re-derived from one another.
    #[test]
    fn an_insert_is_the_interior_it_states_inside_the_compartment_it_stands_in() {
        let run = fit(
            input::parse(INSERTED).expect("the fixture is a valid run"),
            FitMode::Walls,
        )
        .expect("one object with an insert builds");

        assert_eq!(run.subbins.len(), 1, "one placed instance is one insert");
        let insert = &run.subbins[0];
        let spec = insert.spec;
        let wall = run.spec.subbin_wall_thickness;
        assert!(
            (spec.interior_width - 40.0).abs() < 1e-9,
            "the file pins the width at 40 mm, but the insert came out {} mm",
            spec.interior_width
        );
        assert!(
            (spec.interior_height - 12.0).abs() < 1e-9,
            "the file pins the height at 12 mm, but the insert came out {} mm",
            spec.interior_height
        );
        assert!(
            (spec.outer_width - (spec.interior_width + 2.0 * wall)).abs() < 1e-9
                && (spec.outer_depth - (spec.interior_depth + 2.0 * wall)).abs() < 1e-9,
            "a pinned axis leaves the walls their stated {wall} mm, but the insert is {} x {} mm \
             around {} x {} mm",
            spec.outer_width,
            spec.outer_depth,
            spec.interior_width,
            spec.interior_depth
        );

        let compartment = parts_bounds(&inflate_parts(
            &run.result.placements[0].parts,
            -run.spec.divider_thickness / 2.0,
        ));
        assert!(
            spec.interior_depth > 61.0,
            "settling did not grow this compartment past the 60 mm object, so the test cannot see \
             whether the open axis follows it -- the insert's interior is {} mm deep",
            spec.interior_depth
        );
        for (axis, outer, pocket) in [
            ("width", spec.outer_width, compartment.width),
            ("depth", spec.outer_depth, compartment.depth),
        ] {
            assert!(
                (pocket - outer - 2.0 * run.spec.subbin_clearance).abs() < 1e-9,
                "the insert is {outer} mm in {axis} inside a {pocket} mm compartment, which is not \
                 the {} mm clearance it is meant to stand inside",
                run.spec.subbin_clearance
            );
        }
        assert!(
            (spec.chamfer - run.floor_fillet).abs() < 1e-9 && spec.floor >= spec.chamfer,
            "the insert's chamfer is the compartment's own floor blend, standing in a floor at \
             least as thick"
        );

        let written: Vec<&str> = run.all_pieces().iter().map(|b| b.name).collect();
        assert!(
            written.contains(&insert.name.as_str()),
            "the insert is a body the run writes and the Soundness section accounts for, but              {written:?} does not name {}",
            insert.name
        );
        assert_eq!(run.soundness.len(), written.len());
    }

    /// The object stands in its insert, not on the compartment floor: centred in
    /// the interior and lifted by the insert's own floor.
    #[test]
    fn an_object_with_an_insert_is_drawn_standing_in_it() {
        let run = fit(
            input::parse(INSERTED).expect("the fixture is a valid run"),
            FitMode::Walls,
        )
        .expect("one object with an insert builds");
        let spec = run.subbins[0].spec;
        let boxes = object_boxes(&run);
        assert_eq!(boxes.len(), 1);
        let b = &boxes[0];
        assert!(
            (b.min.z - (spec.z + spec.floor)).abs() < 1e-9,
            "the battery stands at z = {} rather than on the insert floor at {}",
            b.min.z,
            spec.z + spec.floor
        );
        let (w, d) = (b.max.x - b.min.x, b.max.y - b.min.y);
        assert!(
            (w - 40.0).abs() < 1e-9 && (d - 60.0).abs() < 1e-9,
            "the battery is drawn {w} x {d} mm, not the 40 x 60 mm it was declared as"
        );
        assert!(
            b.min.x >= spec.x + spec.walls().0 - 1e-9
                && b.max.x <= spec.x + spec.outer_width - spec.walls().0 + 1e-9,
            "the battery is drawn outside the interior that holds it"
        );
    }

    /// A drawn object is the object, not the compartment the fit gave it.
    ///
    /// `settle` absorbs leftover into a claim after the packer is done with it,
    /// so `claim - claim_margin` is the compartment and can be many times the
    /// object -- at `tidy_absorb = 100` this one's compartment is the better
    /// part of the drawer. The box must still measure 30 x 20 mm, and must still
    /// stand inside that compartment.
    ///
    /// The second half is the regression guard in the other direction: with
    /// nothing absorbed the compartment *is* the object, and the box must be
    /// exactly where deflating the claim always put it.
    #[test]
    fn an_object_is_drawn_at_its_own_size_however_much_its_compartment_grew() {
        let settled = fit(
            input::parse(ROOMY_ONE).expect("the fixture is a valid run"),
            FitMode::Walls,
        )
        .expect("one small object in a 100 mm drawer builds");
        assert!(
            settled.absorbed > 0,
            "the fixture is meant to settle its one claim"
        );

        let boxes = object_boxes(&settled);
        assert_eq!(boxes.len(), 1);
        let b = &boxes[0];
        let (w, d) = (b.max.x - b.min.x, b.max.y - b.min.y);
        assert!(
            (w - 30.0).abs() < 1e-9 && (d - 20.0).abs() < 1e-9,
            "the widget is drawn {w} x {d} mm, not the 30 x 20 mm it was declared as"
        );
        let claim = parts_bounds(&settled.result.placements[0].parts);
        let compartment = parts_bounds(&inflate_parts(&[claim], -settled.claim_margin));
        assert!(
            compartment.width > w + 1.0 && compartment.depth > d + 1.0,
            "settling did not grow this compartment, so the test cannot see the defect"
        );
        assert!(
            b.min.x >= compartment.x - 1e-9
                && b.min.y >= compartment.y - 1e-9
                && b.max.x <= compartment.right() + 1e-9
                && b.max.y <= compartment.bottom() + 1e-9,
            "the widget is drawn outside the compartment the fit gave it"
        );

        let unsettled = fit(
            input::parse(&ROOMY_ONE.replace("tidy_absorb = 100", "tidy_absorb = 0"))
                .expect("the fixture is a valid run"),
            FitMode::Walls,
        )
        .expect("the same drawer builds without settling");
        assert_eq!(
            unsettled.absorbed, 0,
            "nothing is absorbed at tidy_absorb = 0"
        );
        let plain = object_boxes(&unsettled);
        let deflated = inflate_parts(
            &unsettled.result.placements[0].parts,
            -unsettled.claim_margin,
        );
        assert!(
            (plain[0].min.x - deflated[0].x).abs() < 1e-9
                && (plain[0].min.y - deflated[0].y).abs() < 1e-9,
            "with nothing absorbed the object is exactly the deflated claim, but it is drawn at \
             ({}, {}) against ({}, {})",
            plain[0].min.x,
            plain[0].min.y,
            deflated[0].x,
            deflated[0].y
        );
    }

    /// One long thin object in a drawer with room to spare, held to its own
    /// width so its compartment cannot grow sideways. The packer turns it, which
    /// is what the rotation of `max_size` is for.
    const SNUG: &str = "[drawer]
width = 200
depth = 200

[settings]
effort = \"quick\"
tidy_absorb = 100

[[objects]]
name = \"battery\"
size = [45, 124]
max_size = [45, \"\"]
";

    /// `max_size` is what an object says when growing its compartment would stop
    /// the compartment holding it still.
    ///
    /// Settling grows a compartment into whatever leftover faces it, which for a
    /// battery in a roomy drawer means a pocket it can lie over at an angle in.
    /// Held to its own 45 mm width, the compartment comes back to 45 mm plus the
    /// clearance and fillet the claim reserves -- and the axis left unstated
    /// still takes everything settling gave it, because a battery that slides
    /// along its own length is still a battery you can pick up by the end.
    #[test]
    fn an_object_held_to_a_max_size_keeps_its_compartment_snug() {
        let held = fit(
            input::parse(SNUG).expect("the fixture is a valid run"),
            FitMode::Walls,
        )
        .expect("one object in a 200 mm drawer builds");
        assert_eq!(held.clamped, 1, "the one compartment was pulled back");

        let boxes = object_boxes(&held);
        assert_eq!(boxes.len(), 1);
        let compartment = parts_bounds(&inflate_parts(
            &held.result.placements[0].parts,
            -held.claim_margin,
        ));
        let placed = &held.result.placements[0];
        let held_axis = match placed.rotation {
            Rotation::Deg90 | Rotation::Deg270 => compartment.depth,
            Rotation::Deg0 | Rotation::Deg180 => compartment.width,
        };
        assert!(
            (held_axis - 45.0).abs() < 1e-6,
            "the battery's compartment is {held_axis} mm across its own width, not the 45 mm it \
             was held to -- rotation {:?}",
            placed.rotation
        );

        let loose = fit(
            input::parse(&SNUG.replace("max_size = [45, \"\"]", "")).expect("valid"),
            FitMode::Walls,
        )
        .expect("the same drawer builds unheld");
        assert_eq!(loose.clamped, 0);
        let grew = parts_bounds(&inflate_parts(
            &loose.result.placements[0].parts,
            -loose.claim_margin,
        ));
        let loose_axis = match loose.result.placements[0].rotation {
            Rotation::Deg90 | Rotation::Deg270 => grew.depth,
            Rotation::Deg0 | Rotation::Deg180 => grew.width,
        };
        assert!(
            loose_axis > 45.0 + 1.0,
            "without the limit the same compartment grows to {loose_axis} mm, so the test can see \
             the difference"
        );
    }

    /// The names the viewer is handed are the exporter's own, slotted by bin and
    /// piece -- so a label and the file it names cannot come from two different
    /// rules, and the two partitions of a bin cannot drift apart without the
    /// gather panicking.
    #[test]
    fn the_viewer_is_handed_the_names_the_export_writes() {
        let spec = input::parse(LONG).expect("the fixture is a valid run");
        let run = fit(spec, FitMode::Walls).expect("a seven-cell drawer builds");
        assert!(
            !run.split_lines.is_empty(),
            "the fixture is a bin the bed makes us cut"
        );

        let (bins, plate) = piece_files(&run);
        assert_eq!(
            bins.len(),
            run.params.bins.len(),
            "one list per bin, indexed by bin"
        );
        assert_eq!(
            bins.iter().flatten().cloned().collect::<Vec<String>>(),
            run.pieces
                .iter()
                .map(|p| p.name.clone())
                .collect::<Vec<String>>(),
            "every piece the export writes, in the order it writes them"
        );
        assert_eq!(
            plate,
            run.baseplate
                .iter()
                .map(|p| p.name.clone())
                .collect::<Vec<String>>()
        );
        assert!(
            bins.iter().flatten().all(|f| f.ends_with(".stl")),
            "a piece is named by the file it would be written as"
        );
    }

    /// Every cell of every bin, so two bins can be asked whether they stand in
    /// the same place.
    fn all_bin_cells(run: &Run) -> Vec<GridCell> {
        run.bins.iter().flat_map(|b| b.cells.clone()).collect()
    }

    /// In `bins` mode a bin is an object: one `LogicalBin` per object, holding
    /// that object's whole quantity as its own compartments, and no two of them
    /// standing in the same cell of the drawer.
    #[test]
    fn gives_every_object_its_own_bin() {
        let spec = input::parse(TWO).expect("the fixture is a valid run");
        let grid = drawer_grid(200.0, 200.0, MAX_GRID, gridfinity::GRID_PITCH);
        let run = fit(spec, FitMode::Bins).expect("two objects in a four-cell drawer build");

        assert_eq!(
            run.bins
                .iter()
                .map(FittedBin::name)
                .collect::<Vec<String>>(),
            vec!["block".to_string(), "rod".to_string()],
            "one bin per object, in the order the file states them"
        );
        assert_eq!(run.params.bins.len(), run.bins.len());
        assert_eq!(
            run.bins.iter().map(|b| b.instances).sum::<usize>(),
            run.result.placements.len(),
            "every instance is a compartment of some bin"
        );
        assert_eq!(run.result.placements.len(), 3);
        assert_eq!(
            run.pockets.len(),
            3,
            "each of these objects is a single box, so it is a single pocket"
        );
        assert!(
            run.params.inner_walls.is_empty(),
            "a stated cavity needs no divider, but the run carries {}",
            run.params.inner_walls.len()
        );
        for (bin, logical) in run.bins.iter().zip(&run.params.bins) {
            assert_eq!(bin.cells, logical.cells);
            assert_eq!(logical.pockets.len(), bin.instances);
        }

        for b in object_boxes(&run) {
            let cell = GridCell {
                x: ((b.min.x + b.max.x) / 2.0 / gridfinity::GRID_PITCH).floor() as i32,
                y: ((b.min.y + b.max.y) / 2.0 / gridfinity::GRID_PITCH).floor() as i32,
            };
            assert!(
                run.bins[b.bin].cells.contains(&cell),
                "{}'s box is drawn against bin {}, which does not stand in {cell:?}",
                b.name,
                b.bin
            );
        }

        let cells = all_bin_cells(&run);
        let mut distinct = cells.clone();
        distinct.sort_unstable_by_key(|c| (c.x, c.y));
        distinct.dedup();
        assert_eq!(
            distinct.len(),
            cells.len(),
            "two bins stand in the same cell"
        );
        for cell in &cells {
            assert!(
                cell.x >= 0
                    && cell.y >= 0
                    && (cell.x as u32) < grid.cols
                    && (cell.y as u32) < grid.rows,
                "{cell:?} is outside the {} x {} cell drawer",
                grid.cols,
                grid.rows
            );
            assert!(
                run.cells.contains(cell),
                "{cell:?} has no baseplate under it"
            );
        }
    }

    /// The bin follows the object rather than the rectangle around it: a cell no
    /// compartment comes within a wall of is not part of the bin.
    #[test]
    fn an_l_shaped_object_gets_an_l_shaped_bin() {
        let spec = input::parse(ELL).expect("the fixture is a valid run");
        let run = fit(spec, FitMode::Bins).expect("an L-shaped bin builds");

        assert_eq!(run.bins.len(), 1);
        let cells = &run.bins[0].cells;
        let footprint = GridFootprint::from_cells(cells).expect("the bin has cells");
        assert_eq!(
            (footprint.width_cells, footprint.depth_cells),
            (3, 3),
            "the object needs three cells each way"
        );
        assert_eq!(
            cells.len(),
            8,
            "the corner the L does not reach is not part of the bin: {cells:?}"
        );
        assert_eq!(
            compartments(cells, &Default::default()).len(),
            1,
            "the trimmed bin is still one edge-connected shape"
        );
        for (piece, sound) in run.all_pieces().iter().zip(&run.soundness) {
            assert_eq!(sound.shells, 1, "{} is one shell", piece.name);
            assert_positive_shells(piece.solid, piece.name);
        }
    }

    /// The same check `walls` mode is held to, on a bin that is not the drawer:
    /// every object's footprint lies inside a compartment floor the model
    /// actually built, read off the finished B-rep rather than re-derived from
    /// the margin the packer used.
    #[cfg(not(feature = "occt"))]
    #[test]
    fn every_packed_object_fits_the_compartment_floor_of_its_own_bin() {
        let spec = input::parse(SMALL).expect("the fixture is a valid run");
        let run = fit(spec, FitMode::Bins).expect("a bin of four blocks builds");
        assert_eq!(run.bins.len(), 1);
        assert_eq!(
            run.pieces.len(),
            1,
            "a two-cell-square bin needs no splitting"
        );

        let floors = compartment_floors(&run.pieces[0].solid);
        assert_eq!(
            floors.len(),
            run.result.placements.len(),
            "a compartment floor is a packed object's pocket and nothing else"
        );
        for b in object_boxes(&run) {
            for corner in [Vec2::new(b.min.x, b.min.y), Vec2::new(b.max.x, b.max.y)] {
                assert!(
                    floors.iter().any(|f| point_in_polygon(f, corner)),
                    "the object corner {corner} stands on no compartment floor of its own bin"
                );
            }
        }
    }

    /// A drawer whose objects each fit a bin of their own is built as those
    /// bins: `auto` reaches for the discrete bins first -- grouped where
    /// grouping pays, which for these two it does not -- and the drawer-wide
    /// body is what it falls back to rather than what it prefers.
    #[test]
    fn auto_gives_every_object_its_own_bin_when_the_drawer_holds_them() {
        let spec = input::parse(TWO).expect("the fixture is a valid run");
        let run = fit(spec, FitMode::Auto).expect("two objects in a four-cell drawer build");

        assert_eq!(run.built, Built::Hybrid);
        assert!(
            run.fell_back.is_none(),
            "nothing was refused, so there is nothing to report having fallen back from: {:?}",
            run.fell_back
        );
        assert_eq!(
            run.bins
                .iter()
                .map(FittedBin::name)
                .collect::<Vec<String>>(),
            vec!["block".to_string(), "rod".to_string()],
            "the same bins --mode bins gives: neither fits in the other's cells"
        );
        assert_eq!(run.result.placements.len(), 3);
    }

    /// Four small objects in a four-cell drawer, each claiming 19.16 mm square:
    /// four of those claims stand in one cell's 39.1 mm packing area, so one bin
    /// per object costs four cells for what one bin holds.
    const SMALL_FOUR: &str = "[drawer]
width = 84
depth = 84

[settings]
effort = \"quick\"

[[objects]]
name = \"washers\"
size = [12, 12]

[[objects]]
name = \"nuts\"
size = [12, 12]

[[objects]]
name = \"grub screws\"
size = [12, 12]

[[objects]]
name = \"o-rings\"
size = [12, 12]
";

    /// The hybrid fit end to end: the objects that share a cell share a bin, and
    /// the bin the model builds really is hollowed to a compartment each.
    #[test]
    fn hybrid_puts_small_objects_that_share_a_cell_in_one_bin() {
        let spec = input::parse(SMALL_FOUR).expect("the fixture is a valid run");
        let apart = fit(spec, FitMode::Bins).expect("four small objects fit four bins");
        assert_eq!(apart.bins.len(), 4, "one bin per object is four bins");

        let spec = input::parse(SMALL_FOUR).expect("the fixture is a valid run");
        let run = fit(spec, FitMode::Hybrid).expect("and one bin between them");

        assert_eq!(run.built, Built::Hybrid);
        assert_eq!(run.bins.len(), 1, "all four objects share a bin");
        assert_eq!(run.bins[0].cells.len(), 1, "which stands on a single cell");
        assert_eq!(run.bins[0].instances, 4);
        assert_eq!(
            run.bins[0].name(),
            "grub screws + nuts + o-rings + washers",
            "a bin is named by everything in it"
        );
        assert_eq!(run.result.placements.len(), 4, "every instance is placed");
        assert_eq!(
            run.pockets.len(),
            4,
            "each object is one box, so the shared bin is hollowed four times"
        );
        assert!(
            run.grouping.is_some(),
            "a hybrid fit reports how it grouped; nothing else does"
        );
        assert!(
            run.bins.iter().map(|b| b.cells.len()).sum::<usize>()
                < apart.bins.iter().map(|b| b.cells.len()).sum::<usize>(),
            "grouping is worth doing here, or the fixture is not the case it claims to be"
        );
        for (piece, sound) in run.all_pieces().iter().zip(&run.soundness) {
            assert_eq!(sound.shells, 1, "{} is one shell", piece.name);
        }
    }

    /// And the other half: where sharing recovers no cell the objects keep their
    /// own bins, so `hybrid` is a decision and not a habit.
    #[test]
    fn hybrid_leaves_objects_apart_where_sharing_recovers_nothing() {
        let spec = input::parse(TWO).expect("the fixture is a valid run");
        let apart = fit(spec, FitMode::Bins).expect("two objects, two bins");
        let spec = input::parse(TWO).expect("the fixture is a valid run");
        let run = fit(spec, FitMode::Hybrid).expect("two objects, still two bins");

        assert_eq!(
            run.bins
                .iter()
                .map(FittedBin::name)
                .collect::<Vec<String>>(),
            apart
                .bins
                .iter()
                .map(FittedBin::name)
                .collect::<Vec<String>>()
        );
        assert_eq!(
            run.bins.iter().map(|b| b.cells.len()).sum::<usize>(),
            apart.bins.iter().map(|b| b.cells.len()).sum::<usize>(),
            "and it costs the drawer nothing to have asked"
        );
    }

    /// Four objects across three cells: a discrete bin is a whole number of
    /// cells, so each of these rounds up to a cell of its own and the four want
    /// one more than the drawer has, while all four claims stand side by side
    /// in the one drawer-wide packing area.
    ///
    /// 22 x 30 mm objects in a 126 x 42 mm drawer: a claim is the object plus
    /// `2 * (clearance + floor_fillet + divider / 2)` = 7.16 mm at these
    /// settings, so 29.16 x 37.16 mm fits the 39.1 mm square packing area of a
    /// single cell, and four of them span 116.64 mm of the drawer's own 123.1.
    const ROUNDS_UP: &str = "[drawer]
width = 126
depth = 42

[settings]
effort = \"quick\"

[[objects]]
name = \"awl\"
size = [22, 30]

[[objects]]
name = \"bradawl\"
size = [22, 30]

[[objects]]
name = \"chisel\"
size = [22, 30]

[[objects]]
name = \"drift\"
size = [22, 30]
";

    /// The whole point of `auto`: the one large body is built only where the
    /// small ones cannot be, and the run says which it built and why.
    ///
    /// Both halves of that are asserted directly on the same fixture rather than
    /// inferred from the automatic run, because the test is worth nothing unless
    /// `bins` really is refused here and `walls` really is not.
    #[test]
    fn auto_builds_one_drawer_wide_bin_only_when_the_bins_do_not_fit() {
        let spec = input::parse(ROUNDS_UP).expect("the fixture is a valid run");
        let refusal = fit(spec, FitMode::Hybrid)
            .err()
            .expect("four one-cell bins do not stand in three cells, grouped or not");
        let spec = input::parse(ROUNDS_UP).expect("the fixture is a valid run");
        assert!(
            fit(spec, FitMode::Bins).is_err(),
            "nor do they as a bin each -- no two of these claims share a cell, so grouping              has nothing to recover here"
        );

        let spec = input::parse(ROUNDS_UP).expect("the fixture is a valid run");
        let walls = fit(spec, FitMode::Walls).expect("four claims stand side by side in one bin");
        assert_eq!(walls.result.placements.len(), 4);

        let spec = input::parse(ROUNDS_UP).expect("the fixture is a valid run");
        let run = fit(spec, FitMode::Auto).expect("the drawer is fitted as one bin");
        assert_eq!(run.built, Built::Walls);
        assert_eq!(
            run.fell_back.as_deref(),
            Some(refusal.as_str()),
            "the run reports the refusal it fell back from, not a message of its own"
        );
        assert_eq!(
            run.result.placements.len(),
            4,
            "every object is placed, or the fallback fitted nothing"
        );
        assert_eq!(
            run.params.bins.len(),
            1,
            "the fallback is the one drawer-wide bin"
        );
        assert!(run.bins.is_empty(), "which is not a bin per object");
    }

    /// A run that cannot hold what it was given has not fitted the drawer, so it
    /// fails outright rather than building geometry missing compartments.
    #[test]
    fn refuses_a_run_whose_objects_do_not_all_fit() {
        let text = "[drawer]
width = 84
depth = 84

[settings]
effort = \"quick\"

[[objects]]
name = \"crowbar\"
size = [200, 30]
";
        let spec = input::parse(text).expect("the fixture is a valid run");
        let Err(err) = fit(spec, FitMode::Walls) else {
            panic!("a 200 mm object does not fit an 84 mm drawer");
        };
        assert!(
            err.contains("crowbar") && err.contains("0 of 1 placed"),
            "{err}"
        );

        let spec = input::parse(text).expect("the fixture is a valid run");
        let Err(err) = fit(spec, FitMode::Bins) else {
            panic!("nor does it fit a bin the drawer holds");
        };
        assert!(err.contains("crowbar"), "{err}");

        let spec = input::parse(text).expect("the fixture is a valid run");
        let Err(err) = fit(spec, FitMode::Hybrid) else {
            panic!("nor does grouping help an object that fits no bin at all");
        };
        assert!(err.contains("crowbar"), "{err}");

        let spec = input::parse(text).expect("the fixture is a valid run");
        let Err(err) = fit(spec, FitMode::Auto) else {
            panic!("neither plan holds a 200 mm object, so there is nothing to fall back to");
        };
        assert!(
            err.contains("crowbar") && err.contains("0 of 1 placed"),
            "an automatic run refuses with the drawer's own shortfall, not one bin's: {err}"
        );
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
