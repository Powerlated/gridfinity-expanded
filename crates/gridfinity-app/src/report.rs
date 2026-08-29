//! What an `optimize` run tells the user about itself.
//!
//! `print` writes the whole report to stdout in ten sections -- the drawer it
//! resolved, the objects it was given, how the packing went, where each instance
//! landed, the dividers that came out of it, what became of the rounding, how the
//! bin had to be split for the printer, what each built piece is made of, the
//! files written, and the warnings. Nothing here decides
//! anything: every number is read back off the finished `Run`, so the report
//! cannot disagree with the geometry. The section helpers (`heading`, `field`,
//! `row`) exist only so the columns line up; the measurement helpers (`mm`,
//! `mm2`, `bytes`, `secs`, `percent`) fix how each kind of quantity is spelled.

use crate::export::{Contents, Written};
use crate::optimize::Run;
use gridfinity_cad::layout::{Axis, GridFootprint, Piece};
use gridfinity_cad::printers::{BED_MARGIN, PrinterProfile, check_bed_fit};
use gridfinity_cad::project::rects::{Rect, inflate_parts, parts_bounds, union_area};
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
    let pitch = 42.0;
    let (cols, rows) = (run.grid.cols, run.grid.rows);
    heading("Drawer");
    field(
        "requested",
        &format!("{} x {} mm", mm(run.spec.drawer_width), mm(run.spec.drawer_depth)),
    );
    field(
        "grid",
        &format!(
            "{cols} x {rows} cells ({} x {} mm)",
            mm(cols as f64 * pitch),
            mm(rows as f64 * pitch)
        ),
    );
    field(
        "unusable margin",
        &format!(
            "{} mm across, {} mm deep",
            mm(run.grid.margin_x),
            mm(run.grid.margin_y)
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
}

/// One row per object: how many were wanted, how many were placed, how big its
/// claim is, and which quarter turns the packer used.
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
            mm2(union_area(&object.pack.parts)),
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
fn packing(run: &Run) {
    let wanted: u32 = run.spec.objects.iter().map(|o| o.pack.quantity).sum();
    let placed = run.result.placements.len() as u32;
    let area = run.area.area();
    let object_area: f64 = run
        .result
        .placements
        .iter()
        .filter_map(|p| run.spec.objects.iter().find(|o| o.pack.id == p.object_id))
        .map(|o| union_area(&o.pack.parts))
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

/// The dividers the placements imply, and the boundary runs that did not become
/// one.
fn dividers(run: &Run) {
    let pocket_area: f64 = run.pockets.iter().map(|k| k.width * k.depth).sum();
    let interior = run.area.width * run.area.depth;
    heading("Compartments");
    field(
        "hollowed",
        &format!(
            "{} pocket(s) over {} placement(s), {} of the {} packing area",
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
        "walls",
        &format!(
            "none -- the cavity is stated, so the {} divider(s) the packer derived stand as material",
            run.wall_report.generated
        ),
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
    let whole = check_bed_fit(&run.cells, printer);
    heading("Printing");
    field(
        "printer",
        &format!(
            "{} ({} x {} mm bed, {} mm margin each side)",
            printer.name,
            printer.bed_width,
            printer.bed_depth,
            mm(f64::from(BED_MARGIN))
        ),
    );
    field(
        "whole bin",
        &format!(
            "{} x {} mm -- {}",
            whole.bin_width,
            whole.bin_depth,
            if whole.fits { "fits the bed" } else { "too big for the bed" }
        ),
    );
    let lines: Vec<String> = run
        .split_lines
        .iter()
        .map(|l| {
            let axis = match l.axis {
                Axis::X => "x",
                Axis::Y => "y",
            };
            format!("{axis}={}", l.index)
        })
        .collect();
    field(
        "splits",
        &format!(
            "{} cut line{} ({}) -> {} piece{}",
            run.split_lines.len(),
            if run.split_lines.len() == 1 { "" } else { "s" },
            if lines.is_empty() { "none".to_string() } else { lines.join(", ") },
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
                "{} piece{} on the same cut lines as the bin",
                run.baseplate.len(),
                if run.baseplate.len() == 1 { "" } else { "s" }
            )
        },
    );
    for pieces in [&run.pieces, &run.baseplate] {
        for (piece, part) in pieces.iter().zip(&run.parts) {
            piece_row(&piece.name, part, printer);
        }
    }
}

/// One row of the Printing table: what a piece is called, the cells it covers,
/// and whether that footprint fits the bed. Bin pieces and baseplate pieces are
/// cut on the same lines, so both read their cells off the same `Piece`.
fn piece_row(name: &str, part: &Piece, printer: PrinterProfile) {
    let fit = check_bed_fit(&part.cells, printer);
    let footprint =
        GridFootprint::from_cells(&part.cells).map_or((0, 0), |f| (f.width_cells, f.depth_cells));
    row(&format!(
        "  {:<38}{} x {} cells  {} x {} mm  {}",
        name,
        footprint.0,
        footprint.1,
        fit.bin_width,
        fit.bin_depth,
        match (fit.fits, fit.rotated) {
            (true, true) => "fits, turned on the bed",
            (true, false) => "fits",
            (false, _) => "DOES NOT FIT",
        }
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

/// Everything worth a second look: rounding that did not land, objects that do
/// not fit the cavity's depth, instances left unplaced, and drawer margin big
/// enough to have been another cell.
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
    if !run.blends.is_clean() {
        lines.push(format!(
            "{} of {} floor fillets could not be built; those inside corners are sharp",
            run.blends.requested - run.blends.made(),
            run.blends.requested
        ));
    }
    let wanted: u32 = run.spec.objects.iter().map(|o| o.pack.quantity).sum();
    let placed = run.result.placements.len() as u32;
    if placed < wanted {
        lines.push(format!(
            "{} of {wanted} instances did not fit; the bin was built without them",
            wanted - placed
        ));
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
