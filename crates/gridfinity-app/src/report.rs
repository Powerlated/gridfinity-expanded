//! What an `optimize` run tells the user about itself.
//!
//! `print` writes the whole report to stdout in twelve sections -- the drawer it
//! resolved, the objects it was given, how the packing went, where each instance
//! landed, the bin each object was given (in `bins` mode, where there is one per
//! object), the inserts it built for the objects that asked for one, what became
//! of the cavity, what became of the rounding, how the
//! bodies had to be split for the printer and how their seams interlock, what
//! each built piece is made of, the files written, and the warnings. Nothing here decides
//! anything: every number is read back off the finished `Run`, so the report
//! cannot disagree with the geometry. The section helpers (`heading`, `field`,
//! `row`) exist only so the columns line up; the measurement helpers (`mm`,
//! `mm2`, `bytes`, `secs`, `percent`) fix how each kind of quantity is spelled.

use crate::export::{Contents, Written};
use crate::grouping::score as grouping_score;
use crate::optimize::{Built, Run};
#[cfg(feature = "occt")]
use gridfinity_occt::Shape as Solid;
use gridfinity_model::layout::{Axis, GridFootprint, Piece, SplitLine};
use gridfinity_model::printers::{BED_MARGIN, BedFitResult, PrinterProfile, check_bed_fit};
use gridfinity_project::rects::{Rect, inflate_parts, parts_bounds, union_area};
use gridfinity_project::tidy::score as layout_score;
use std::time::Duration;

/// How wide the label column of a `field` line is.
const LABEL: usize = 18;

/// A millimetre measurement, to a tenth.
fn mm(value: f64) -> String {
    format!("{value:.1}")
}

/// An area in mm², to a whole millimetre and grouped in thousands.
fn mm2(value: f64) -> String {
    thousands(value.round() as i64)
}

/// A whole number with thousands separated, so a triangle count reads at a
/// glance.
fn thousands(value: i64) -> String {
    let digits = value.abs().to_string();
    let mut out = String::new();
    for (i, c) in digits.chars().enumerate() {
        if i > 0 && (digits.len() - i) % 3 == 0 {
            out.push(' ');
        }
        out.push(c);
    }
    if value < 0 {
        return format!("-{out}");
    }
    out
}

/// A file size in the largest unit that leaves a number above one.
fn bytes(count: usize) -> String {
    let n = count as f64;
    if n >= 1_048_576.0 {
        return format!("{:.1} MB", n / 1_048_576.0);
    }
    if n >= 1024.0 {
        return format!("{:.1} kB", n / 1024.0);
    }
    format!("{count} B")
}

/// An elapsed time in seconds, to a hundredth.
fn secs(d: Duration) -> String {
    format!("{:.2} s", d.as_secs_f64())
}

/// A share of a whole as a percentage, and zero when the whole is zero.
fn percent(part: f64, whole: f64) -> String {
    if whole <= 0.0 {
        return "0.0%".to_string();
    }
    format!("{:.1}%", 100.0 * part / whole)
}

/// A section heading with a blank line before it.
fn heading(title: &str) {
    println!();
    println!("{title}");
}

/// One labelled line of a section.
fn field(label: &str, value: &str) {
    println!("  {label:<LABEL$}{value}");
}

/// One row of a table, indented under its heading.
fn row(cells: &str) {
    println!("  {cells}");
}

/// The whole report for a finished run, in the order a reader wants it: what was
/// asked for, what happened, what was written, and what to worry about.
pub fn print(run: &Run, written: &[Written]) {
    drawer(run);
    objects(run);
    packing(run);
    placements(run);
    bins(run);
    subbins(run);
    dividers(run);
    rounding(run);
    printing(run);
    soundness(run);
    output(run, written);
    warnings(run);
}

/// The drawer as it resolved: the grid it holds, the millimetres it does not,
/// and the rectangle the packer was given.
fn drawer(run: &Run) {
    let pitch = run.spec.pitch;
    let (cols, rows) = (run.grid.cols, run.grid.rows);
    heading("Drawer");
    field(
        "requested",
        &format!(
            "{} x {} mm",
            mm(run.spec.drawer_width),
            mm(run.spec.drawer_depth)
        ),
    );
    field(
        "grid",
        &format!(
            "{cols} x {rows} cells of {} mm ({} x {} mm)",
            mm(pitch),
            mm(cols as f64 * pitch),
            mm(rows as f64 * pitch)
        ),
    );
    field(
        if run.spec.baseplate {
            "margin"
        } else {
            "unusable margin"
        },
        &format!(
            "{} mm across, {} mm deep{}",
            mm(run.grid.margin_x),
            mm(run.grid.margin_y),
            if run.spec.baseplate {
                " -- the baseplate spans it, so the stack is a snug fit"
            } else {
                ""
            }
        ),
    );
    field(
        "packing area",
        &format!(
            "{} x {} mm, inset {} mm from the bin outline",
            mm(run.area.width),
            mm(run.area.depth),
            run.area.x
        ),
    );
    field(
        "compartments",
        &format!(
            "{} mm deep ({} height units, {} mm tall overall)",
            mm(run.spec.cavity_depth()),
            run.spec.height_units,
            mm(run.spec.total_height())
        ),
    );
    field("fit", &fitted_as(run));
}

/// What the drawer was built as, and -- for a run that asked for the smallest
/// bins and could not have them -- the refusal that sent it to the one
/// drawer-wide bin.
///
/// A run told which mode to build says only which it built; there is nothing
/// else to say about a choice the command line made. An automatic run that fell
/// back has to say why, or the reader is left to guess whether the large body
/// was preferred or merely unavoidable.
fn fitted_as(run: &Run) -> String {
    let shared = run.bins.iter().filter(|b| b.objects.len() > 1).count();
    let built = match run.built {
        Built::Walls => format!(
            "the whole drawer as one bin, hollowed to {} compartment(s)",
            run.pockets.len()
        ),
        Built::Bins => format!("{} bin(s), one per object", run.bins.len()),
        Built::Hybrid if shared == 0 => format!(
            "{} bin(s), one per object -- no grouping paid",
            run.bins.len()
        ),
        Built::Hybrid => format!(
            "{} bin(s), {shared} of them shared by more than one object",
            run.bins.len()
        ),
    };
    match &run.fell_back {
        None => built,
        Some(why) => format!("{built} -- a bin per object was refused: {why}"),
    }
}

/// One row per object: how many were wanted, how many were placed, how big its
/// claim is, and which quarter turns the packer used.
///
/// The claim is what the packer worked with -- for an object given an insert
/// that is the insert's outer box less the fillet it hands back -- while the
/// area is the object's own, as the file declares it.
fn objects(run: &Run) {
    heading("Objects");
    if run.spec.objects.is_empty() {
        row("(none -- the drawer is one empty bin)");
        return;
    }
    row(&format!(
        "{:<24}{:>5}{:>8}  {:<18}{:>10}  {}",
        "name", "want", "placed", "claim mm", "area mm2", "turns"
    ));
    let margin = run.claim_margin;
    for object in &run.spec.objects {
        let placed = run
            .result
            .placed_by_object_id
            .get(&object.pack.id)
            .copied()
            .unwrap_or(0);
        let claim = claim_bounds(&object.pack.parts, margin);
        let mut turns: Vec<u16> = run
            .result
            .placements
            .iter()
            .filter(|p| p.object_id == object.pack.id)
            .map(|p| p.rotation.degrees())
            .collect();
        turns.sort_unstable();
        turns.dedup();
        let turns = if turns.is_empty() {
            "-".to_string()
        } else {
            turns
                .iter()
                .map(|t| format!("{t}"))
                .collect::<Vec<String>>()
                .join(", ")
        };
        row(&format!(
            "{:<24}{:>5}{:>8}  {:<18}{:>10}  {}",
            object.pack.name,
            object.pack.quantity,
            placed,
            format!("{} x {}", mm(claim.width), mm(claim.depth)),
            mm2(union_area(&object.footprint)),
            turns
        ));
    }
}

/// The object's boxes grown by the margin the packer claims around them, as one
/// bounding box.
fn claim_bounds(parts: &[Rect], margin: f64) -> Rect {
    let bounds = parts_bounds(parts);
    Rect::new(
        bounds.x - margin,
        bounds.y - margin,
        bounds.width + margin * 2.0,
        bounds.depth + margin * 2.0,
    )
}

/// How the search went: the budget it spent, how much of what was asked for it
/// placed, how much of the area that covers, and what it could not place.
///
/// The object area is the objects' own; the claimed area is what was packed, so
/// the gap between them is clearance, reserved fillet, dividers and the walls of
/// any insert.
fn packing(run: &Run) {
    let wanted: u32 = run.spec.objects.iter().map(|o| o.pack.quantity).sum();
    let placed = run.result.placements.len() as u32;
    let area = run.area.area();
    let object_area: f64 = run
        .result
        .placements
        .iter()
        .filter_map(|p| run.spec.objects.iter().find(|o| o.pack.id == p.object_id))
        .map(|o| union_area(&o.footprint))
        .sum();
    let claimed_area: f64 = run
        .result
        .placements
        .iter()
        .map(|p| union_area(&p.parts))
        .sum();
    heading("Packing");
    field(
        "effort",
        &format!(
            "{} ({} restarts, {} run)",
            run.spec.effort.name(),
            run.spec.effort.restarts(),
            run.result.iterations
        ),
    );
    field("instances", &format!("{placed} of {wanted} placed"));
    field(
        "object area",
        &format!(
            "{} mm2 of {} mm2 packing area ({} efficiency)",
            mm2(object_area),
            mm2(area),
            percent(object_area, area)
        ),
    );
    field(
        "claimed area",
        &format!(
            "{} mm2 including clearance, floor fillet and dividers ({} of the area)",
            mm2(claimed_area),
            percent(claimed_area, area)
        ),
    );
    tidiness(run);
    let margin = run.claim_margin;
    let mut any = false;
    for object in &run.spec.objects {
        let placed = run
            .result
            .placed_by_object_id
            .get(&object.pack.id)
            .copied()
            .unwrap_or(0);
        if placed >= object.pack.quantity {
            continue;
        }
        let claim = claim_bounds(&object.pack.parts, margin);
        field(
            if any { "" } else { "unplaced" },
            &format!(
                "{} x{} short -- its claim is {} x {} mm",
                object.pack.name,
                object.pack.quantity - placed,
                mm(claim.width),
                mm(claim.depth)
            ),
        );
        any = true;
    }
    if !any && wanted > 0 {
        field("unplaced", "none -- everything asked for fits");
    }
}

/// How the layout the search settled on reads, as the score it was chosen for and
/// the six terms behind it.
///
/// Every term is a fraction of its own worst case with 0 the tidiest, so they
/// print as percentages and the score is their weighted sum. Read off the
/// `PackResult`, which carries the winning pass's own reading: the number here
/// is the number the search minimised, not a second opinion about the layout.
fn tidiness(run: &Run) {
    let t = &run.result.tidiness;
    field(
        "tidiness",
        &format!(
            "{:.3} (unshared lines {}, runs {}, leftover in pieces {}, slivers {}, \
             objects apart {}, off centre {})",
            layout_score(t),
            percent(t.lines, 1.0),
            percent(t.runs, 1.0),
            percent(t.fragments, 1.0),
            percent(t.slivers, 1.0),
            percent(t.grouping, 1.0),
            percent(t.balance, 1.0)
        ),
    );
}

/// Where every instance ended up: the compartment interior it was given, in the
/// drawer's own millimetres, read down the drawer and then across it.
fn placements(run: &Run) {
    if run.result.placements.is_empty() {
        return;
    }
    heading("Placements");
    let margin = run.claim_margin;
    let mut rows: Vec<(f64, f64, String)> = run
        .result
        .placements
        .iter()
        .map(|p| {
            let interior = parts_bounds(&inflate_parts(&p.parts, -margin));
            (
                interior.y,
                interior.x,
                format!(
                    "{:<26}{:>4}{:>9}{:>9}{:>10}{:>9}{:>6}",
                    p.object_id,
                    format!("#{}", p.instance + 1),
                    mm(interior.x),
                    mm(interior.y),
                    mm(interior.width),
                    mm(interior.depth),
                    p.rotation.degrees()
                ),
            )
        })
        .collect();
    rows.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.total_cmp(&b.1)));
    row(&format!(
        "{:<26}{:>4}{:>9}{:>9}{:>10}{:>9}{:>6}",
        "name", "no.", "x", "y", "width", "depth", "turn"
    ));
    for (_, _, line) in rows {
        row(&line);
    }
}

/// The bin each object was given: what it covers, how many compartments are in
/// it, and whether it prints whole. Nothing in `walls` mode, where the drawer is
/// one bin and the Printing section already says everything about it.
///
/// A hybrid fit prints its `grouping` here too, the way `Packing` prints its
/// tidiness: the six terms are what decided which objects share a bin, and a
/// weight that reads oddly on a real drawer is only visible as its term.
fn bins(run: &Run) {
    if run.bins.is_empty() {
        return;
    }
    let printer = run.spec.printer;
    heading("Bins");
    if let Some(g) = &run.grouping {
        field(
            "grouping",
            &format!(
                "{:.2} cells (on {:.0}, air {:.1}, biggest {:.0}, cut {:.0}, shared {:.0}, oblong {:.1})",
                grouping_score(g),
                g.cells,
                g.air,
                g.largest,
                g.cut,
                g.shared,
                g.oblong
            ),
        );
    }
    row(&format!(
        "{:<28}{:>7}{:>9}{:>14}{:>14}  {}",
        "objects", "cells", "grid", "size mm", "compartments", "built as"
    ));
    for (index, bin) in run.bins.iter().enumerate() {
        let fit = check_bed_fit(&bin.cells, printer, run.spec.pitch);
        let footprint = GridFootprint::from_cells(&bin.cells)
            .map_or((0, 0), |f| (f.width_cells, f.depth_cells));
        row(&format!(
            "{:<28}{:>7}{:>9}{:>14}{:>14}  {}",
            bin.name(),
            bin.cells.len(),
            format!("{} x {}", footprint.0, footprint.1),
            format!("{} x {}", fit.bin_width, fit.bin_depth),
            bin.instances,
            built_as(run, index)
        ));
    }
}

/// What the report calls the files one bin becomes: the name the model gave its
/// piece, or the stem those pieces share when it was cut for the bed.
///
/// A bin *is* what is in it, and the model names its pieces by position rather
/// than by what is in them, so without this the report names every object and
/// every file and leaves the reader to pair them off by order.
fn built_as(run: &Run, index: usize) -> String {
    let Some(piece) = run.pieces.iter().find(|p| p.bin == index) else {
        return "-".to_string();
    };
    match piece.name.find("-piece-") {
        Some(cut) => format!("{}-piece-1..{}.stl", &piece.name[..cut], piece.piece_count),
        None => piece.name.clone(),
    }
}

/// The inserts the run built, one row each: what each holds, the interior it
/// came out at, the outer box that interior sits in, and the file it is written
/// as. Nothing for a file that asked for none.
///
/// Read off each built insert's own `SubbinSpec`, which is the declaration the
/// geometry was built from, so the measurements printed are the measurements cut.
fn subbins(run: &Run) {
    if run.subbins.is_empty() {
        return;
    }
    heading("Sub-bins");
    field(
        "walls",
        &format!(
            "{} mm walls, one thickness the whole way round -- the interior turns the \
             outer corner less the wall -- on a floor at least as thick as the {} mm \
             chamfer that clears the compartment's blend, standing {} mm inside it",
            mm(run.spec.subbin_wall_thickness),
            mm(run.floor_fillet),
            format!("{:.2}", run.spec.subbin_clearance)
        ),
    );
    row(&format!(
        "{:<28}{:>22}{:>22}  {}",
        "holds", "interior mm", "outer mm", "built as"
    ));
    for insert in &run.subbins {
        let spec = &insert.spec;
        row(&format!(
            "{:<28}{:>22}{:>22}  {}",
            insert.object,
            format!(
                "{} x {} x {}",
                mm(spec.interior_width),
                mm(spec.interior_depth),
                mm(spec.interior_height)
            ),
            format!(
                "{} x {} x {}",
                mm(spec.outer_width),
                mm(spec.outer_depth),
                mm(spec.height())
            ),
            insert.name
        ));
    }
}

/// What became of the cavity: how much of it was hollowed into compartments, how
/// much stands as material, what settling the packed layout took, and what
/// happened to the dividers the packer derives between two claims.
fn dividers(run: &Run) {
    let pocket_area: f64 = run.pockets.iter().map(|k| k.width * k.depth).sum();
    let pitch = run.spec.pitch;
    let (interior, whose) = match run.built {
        Built::Walls => (run.area.width * run.area.depth, "packing area"),
        Built::Bins | Built::Hybrid => (
            run.bins
                .iter()
                .map(|b| b.cells.len() as f64 * pitch * pitch)
                .sum(),
            "the bins cover",
        ),
    };
    heading("Compartments");
    field(
        "hollowed",
        &format!(
            "{} pocket(s) over {} placement(s), {} of the {} {whose}",
            run.pockets.len(),
            run.result.placements.len(),
            mm2(pocket_area),
            mm2(interior)
        ),
    );
    field(
        "solid",
        &format!(
            "{} of it, {}, is material: between the compartments and wherever nothing was packed",
            mm2((interior - pocket_area).max(0.0)),
            percent(interior - pocket_area, interior)
        ),
    );
    field(
        "settled",
        &match (run.absorbed, run.evened, run.grown) {
            (0, 0, 0) => {
                "nothing to tidy -- no strip worth absorbing, no slack to even out, and no \
                 compartment wall with room in front of it"
                    .to_string()
            }
            (absorbed, evened, grown) => format!(
                "{absorbed} strip(s) of leftover absorbed into the compartments facing them, \
                 {grown} wall(s) grown into leftover no strip covered, {evened} run(s) of slack \
                 evened between their two ends -- no wall moved more than {} mm{}",
                mm(run.spec.tidy_absorb),
                match run.clamped {
                    0 => String::new(),
                    n => format!(
                        ", and {n} compartment(s) pulled back afterwards to the size their \
                         object states, as a max_size or as a subbin"
                    ),
                }
            ),
        },
    );
    field(
        "walls",
        &match run.built {
            Built::Walls => format!(
                "none -- the cavity is stated, so the {} divider(s) the packer derived stand as material",
                run.wall_report.generated
            ),
            Built::Bins | Built::Hybrid => {
                "none -- the cavity of every bin is stated, so what stands between two \
                 compartments is that bin's own material"
                    .to_string()
            }
        },
    );
}

/// What became of the rounding the model asked for: an inside corner the kernel
/// could not blend comes out sharp, and nothing else in the report would say so.
fn rounding(run: &Run) {
    let blends = &run.blends;
    heading("Rounding");
    field(
        "floor fillets",
        &format!(
            "{} of {} built, at {} mm",
            blends.made(),
            blends.requested,
            mm(run.floor_fillet)
        ),
    );
    field(
        "reserved",
        &format!(
            "{} mm of floor at every compartment wall, so an object stands clear of the \
             blend it sits beside",
            mm(run.floor_fillet)
        ),
    );
    if blends.is_clean() {
        return;
    }
    field(
        "left sharp",
        &format!(
            "{} unresolved, {} dropped",
            blends.unresolved,
            blends.dropped.len()
        ),
    );
    if let Some(refusal) = &blends.refusal {
        field("refused because", refusal);
    }
}

/// Whether the bin fits the bed, where it had to be cut, and how each piece
/// fares.
fn printing(run: &Run) {
    let printer = run.spec.printer;
    heading("Printing");
    field(
        "printer",
        &if BED_MARGIN > 0.0 {
            format!(
                "{} ({} x {} mm bed, {} mm margin each side)",
                printer.name,
                printer.bed_width,
                printer.bed_depth,
                mm(BED_MARGIN)
            )
        } else {
            format!(
                "{} ({} x {} mm bed, used whole)",
                printer.name, printer.bed_width, printer.bed_depth
            )
        },
    );
    let (label, whole) = match run.built {
        Built::Walls => {
            let fit = check_bed_fit(&run.cells, printer, run.spec.pitch);
            (
                "whole bin",
                format!(
                    "{} x {} mm -- {}",
                    fit.bin_width,
                    fit.bin_depth,
                    if fit.fits {
                        "fits the bed"
                    } else {
                        "too big for the bed"
                    }
                ),
            )
        }
        Built::Bins | Built::Hybrid => {
            let cut = run
                .bins
                .iter()
                .filter(|b| !b.split_lines.is_empty())
                .count();
            (
                "bins",
                format!(
                    "{} bin(s) -- {}",
                    run.bins.len(),
                    if cut == 0 {
                        "every one prints whole".to_string()
                    } else {
                        format!("{cut} of them had to be cut for the bed")
                    }
                ),
            )
        }
    };
    field(label, &whole);
    field(
        "splits",
        &format!(
            "{} -> {} piece{}",
            cut_lines(&run.split_lines),
            run.pieces.len(),
            if run.pieces.len() == 1 { "" } else { "s" }
        ),
    );
    field(
        "baseplate",
        &if run.baseplate.is_empty() {
            "none -- settings.baseplate is off, so the bin's pegs have no grid to sit in"
                .to_string()
        } else {
            format!(
                "{} -> {} piece{}",
                cut_lines(&run.plate_split_lines),
                run.baseplate.len(),
                if run.baseplate.len() == 1 { "" } else { "s" }
            )
        },
    );
    if !run.baseplate.is_empty() {
        field("interlock", &interlock(run));
    }
    for (pieces, parts) in [
        (&run.pieces, &run.parts),
        (&run.baseplate, &run.plate_parts),
    ] {
        for (piece, part) in pieces.iter().zip(parts) {
            piece_row(&piece.name, part, &piece.solid, printer);
        }
    }
    for insert in &run.subbins {
        body_row(&insert.name, &insert.solid, printer);
    }
}

/// A set of split lines as the Printing section spells it: how many there are
/// and where each one falls, or that there are none.
fn cut_lines(lines: &[SplitLine]) -> String {
    let named: Vec<String> = lines
        .iter()
        .map(|l| {
            let axis = match l.axis {
                Axis::X => "x",
                Axis::Y => "y",
            };
            format!("{axis}={}", l.index)
        })
        .collect();
    format!(
        "{} cut line{} ({})",
        lines.len(),
        if lines.len() == 1 { "" } else { "s" },
        if named.is_empty() {
            "none".to_string()
        } else {
            named.join(", ")
        }
    )
}

/// What the bin's seams and the plate's seams do to each other, in a phrase: the
/// stack holds itself together when the two bodies share no cut line, because
/// every seam of each is then spanned by a piece of the other.
///
/// Read off `Run::interlocked`, so the line and the geometry cannot disagree.
/// The uncut cases are named separately because they are the strongest interlock
/// and read as an accident otherwise.
fn interlock(run: &Run) -> String {
    if !run.interlocked() {
        return "none -- no staggered plan for the plate prints, so it is cut on the bin's own \
                lines and the stack parts along them"
            .to_string();
    }
    if run.split_lines.is_empty() && run.plate_split_lines.is_empty() {
        return "whole -- neither body is cut, so the stack is already one piece".to_string();
    }
    if run.split_lines.is_empty() || run.plate_split_lines.is_empty() {
        return "every seam is spanned -- one body is uncut, so it laps all of the other's seams"
            .to_string();
    }
    "staggered -- the two bodies share no cut line, so each spans the other's seams and the \
     stack moves as one piece"
        .to_string()
}

/// How a piece is laid on the bed, in a phrase, or that it does not go on at
/// all. The angle is named because a piece that needs one needs it typed into
/// the slicer, and a row saying only "at an angle" leaves the reader to work out
/// which.
fn placement(fit: &BedFitResult) -> String {
    match (fit.fits, fit.rotated, fit.tilt_deg) {
        (false, _, _) => "DOES NOT FIT".to_string(),
        (_, true, _) => "fits, turned on the bed".to_string(),
        (_, _, Some(t)) => format!("fits, laid at {t:.1} degrees"),
        _ => "fits".to_string(),
    }
}

/// One row of the Printing table: what a piece is called, the cells it covers,
/// and whether the body built over them fits the bed, and how it has to be laid
/// down to. Each body reads its cells
/// off its own partition, because the plate is cut on lines staggered off the
/// bin's and so covers different cells; the millimetres are measured off each
/// piece's own solid, because neither body is its cells -- a bin is inset from
/// them and a fitted baseplate spans past them by the drawer's margin.
fn piece_row(name: &str, part: &Piece, solid: &Solid, printer: PrinterProfile) {
    let (width, depth) = footprint_mm(solid);
    let fit = printer.bed_fit_mm(width, depth);
    let footprint =
        GridFootprint::from_cells(&part.cells).map_or((0, 0), |f| (f.width_cells, f.depth_cells));
    row(&format!(
        "  {:<38}{} x {} cells  {} x {} mm  {}",
        name,
        footprint.0,
        footprint.1,
        fit.bin_width,
        fit.bin_depth,
        placement(&fit)
    ));
}

/// One row of the Printing table for a body that stands on no cells: its own
/// footprint measured off the finished solid, and how it lies on the bed.
///
/// An insert is cut for nothing -- it is a small box printed whole -- so where a
/// piece's row names the cells it covers, this one names what it is instead.
fn body_row(name: &str, solid: &Solid, printer: PrinterProfile) {
    let (width, depth) = footprint_mm(solid);
    let fit = printer.bed_fit_mm(width, depth);
    row(&format!(
        "  {:<38}{:>13}  {} x {} mm  {}",
        name,
        "insert",
        fit.bin_width,
        fit.bin_depth,
        placement(&fit)
    ));
}

/// Every file written, with its size and what it holds, and how long each stage
/// took.
/// What the pieces are made of, one row each.
///
/// Every piece listed here has already passed the gate in `carve_to_cells` --
/// closed, manifold, geometrically sound, one shell per island of its cells with
/// material inside it, and carrying nothing that no face or edge names. This
/// section says so with the numbers it holds, because a check that leaves no
/// trace in the output is indistinguishable from one that never ran.
fn soundness(run: &Run) {
    heading("Soundness");
    field(
        "checked",
        &format!(
            "{} piece(s): closed manifold, audit clean, one shell per island, no stray geometry",
            run.soundness.len()
        ),
    );
    row(&format!(
        "{:<38}{:>8}{:>10}{:>10}{:>10}{:>10}",
        "piece", "shells", "faces", "edges", "verts", "warnings"
    ));
    for p in &run.soundness {
        row(&format!(
            "{:<38}{:>8}{:>10}{:>10}{:>10}{:>10}",
            p.name,
            p.shells,
            thousands(p.faces as i64),
            thousands(p.edges as i64),
            thousands(p.verts as i64),
            p.warnings
        ));
    }
}

fn output(run: &Run, written: &[Written]) {
    heading("Output");
    for w in written {
        let contents = match w.contents {
            Contents::Triangles(n) => format!("{} triangles", thousands(n as i64)),
            Contents::Bodies(1) => "1 body".to_string(),
            Contents::Bodies(n) => format!("{n} bodies"),
        };
        row(&format!(
            "{:<52}{:>10}  {contents}",
            w.path.display(),
            bytes(w.bytes)
        ));
    }
    field(
        "timing",
        &format!(
            "pack {}   build {}   export {}",
            secs(run.pack_time),
            secs(run.build_time),
            secs(run.export_time)
        ),
    );
}

/// A built body's footprint in millimetres, `(width, depth)`, read off the
/// vertices of the solid itself.
///
/// The plate reaches past the cells it was cut on, so a footprint derived from a
/// cell count is not this one; the solid is the only thing that knows how big
/// the body really is.
fn footprint_mm(solid: &Solid) -> (f64, f64) {
    #[cfg(feature = "occt")]
    {
        let bounds = solid.bounds().expect("a reported OCCT body has bounds");
        return (bounds.max[0] - bounds.min[0], bounds.max[1] - bounds.min[1]);
    }
    #[cfg(not(feature = "occt"))]
    {
    assert!(
        !solid.verts.is_empty(),
        "a built piece with no vertices has no footprint to measure"
    );
    let (mut min_x, mut max_x) = (f64::INFINITY, f64::NEG_INFINITY);
    let (mut min_y, mut max_y) = (f64::INFINITY, f64::NEG_INFINITY);
    for v in &solid.verts {
        min_x = min_x.min(v.point.x);
        max_x = max_x.max(v.point.x);
        min_y = min_y.min(v.point.y);
        max_y = max_y.max(v.point.y);
    }
        (max_x - min_x, max_y - min_y)
    }
}

/// Everything worth a second look: rounding that did not land, objects that do
/// not fit the cavity's depth, a baseplate piece the bed cannot take, the extra
/// pieces staggering the plate's seams cost, a stack whose two bodies part on
/// one plane after all, and drawer margin big enough to have been another cell.
///
/// An instance the packer could not place is **not** here: a run that cannot
/// hold what it was given fails outright, before anything is built.
fn warnings(run: &Run) {
    let mut lines: Vec<String> = Vec::new();
    let depth = run.spec.cavity_depth();
    for object in &run.spec.objects {
        if let Some(height) = object.height
            && height > depth
        {
            lines.push(format!(
                "{} is {} mm tall, but a compartment is only {} mm deep -- raise \
                 settings.height_units",
                object.pack.name,
                mm(height),
                mm(depth)
            ));
        }
    }
    for insert in &run.subbins {
        let proud = insert.spec.height() - depth;
        if proud > 1e-9 {
            lines.push(format!(
                "{} holds {} and stands {} mm proud of the {} mm compartment it drops into -- \
                 raise settings.height_units to sink it",
                insert.name,
                insert.object,
                mm(proud),
                mm(depth)
            ));
        }
        let (width, insert_depth) = footprint_mm(&insert.solid);
        if !run.spec.printer.bed_fit_mm(width, insert_depth).fits {
            lines.push(format!(
                "{} measures {} x {} mm and does not fit the bed -- an insert is printed whole \
                 and is never cut, so state a smaller subbin",
                insert.name,
                mm(width),
                mm(insert_depth)
            ));
        }
    }
    if !run.blends.is_clean() {
        lines.push(format!(
            "{} of {} floor fillets could not be built; those inside corners are sharp",
            run.blends.requested - run.blends.made(),
            run.blends.requested
        ));
    }
    for piece in &run.baseplate {
        let (width, depth) = footprint_mm(&piece.solid);
        if !run.spec.printer.bed_fit_mm(width, depth).fits {
            lines.push(format!(
                "{} measures {} x {} mm and does not fit the bed -- no plan for the plate both \
                 prints and keeps off the bin's seams",
                piece.name,
                mm(width),
                mm(depth)
            ));
        }
    }
    if run.plate_stagger_cost > 0 {
        lines.push(format!(
            "keeping the baseplate's seams off the bin's cost it {} extra piece{} -- the bin's \
             own chunks are as wide as the bed takes, so the plate cannot match them and miss them",
            run.plate_stagger_cost,
            if run.plate_stagger_cost == 1 { "" } else { "s" }
        ));
    }
    if !run.interlocked() {
        lines.push(
            "the baseplate is cut on the bin's own lines, so every seam in the drawer lies in \
             one plane and the stack parts along it -- no piece of either body spans a seam of \
             the other"
                .to_string(),
        );
    }
    for (axis, margin) in [("width", run.grid.margin_x), ("depth", run.grid.margin_y)] {
        if margin >= 42.0 {
            lines.push(format!(
                "{} mm of the drawer's {axis} is margin, which is a whole cell the grid cap left \
                 out",
                mm(margin)
            ));
        }
    }
    if lines.is_empty() {
        return;
    }
    heading("Warnings");
    for line in lines {
        row(&format!("! {line}"));
    }
}
