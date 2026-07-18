//! Printer bed profiles, bed-fit checks, and automatic split-line planning.
//!
//! Pure logic — no geometry. A bin that does not fit the print bed is split into
//! the fewest even pieces that do (mirrors the reference `printers.ts`).

use crate::layout::{Axis, GridCell, GridFootprint, Piece, SplitLine, partition_cells, PITCH};

/// Bed clearance (mm) kept on all sides of a part.
pub const BED_MARGIN: f32 = 5.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PrinterProfile {
    pub name: &'static str,
    pub bed_width: i32,
    pub bed_depth: i32,
}

/// The reference printer set (bed sizes in mm).
pub const PRINTER_PROFILES: &[PrinterProfile] = &[
    PrinterProfile { name: "Bambu Lab A1 Mini", bed_width: 180, bed_depth: 180 },
    PrinterProfile { name: "Bambu Lab P1S / X1C", bed_width: 256, bed_depth: 256 },
    PrinterProfile { name: "Creality Ender 3 / V2", bed_width: 220, bed_depth: 220 },
    PrinterProfile { name: "Creality K1", bed_width: 220, bed_depth: 220 },
    PrinterProfile { name: "Elegoo Centauri Carbon", bed_width: 256, bed_depth: 256 },
    PrinterProfile { name: "Prusa MK4 / MK3S+", bed_width: 250, bed_depth: 210 },
    PrinterProfile { name: "Prusa Mini+", bed_width: 180, bed_depth: 180 },
    PrinterProfile { name: "Voron 2.4 (250mm)", bed_width: 250, bed_depth: 250 },
    PrinterProfile { name: "Voron 2.4 (300mm)", bed_width: 300, bed_depth: 300 },
    PrinterProfile { name: "Custom", bed_width: 220, bed_depth: 220 },
];

/// The default printer (matches the reference: Prusa MK4 / MK3S+).
pub const DEFAULT_PRINTER: PrinterProfile = PrinterProfile { name: "Prusa MK4 / MK3S+", bed_width: 250, bed_depth: 210 };

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BedFitResult {
    pub fits: bool,
    pub bin_width: i32,
    pub bin_depth: i32,
    /// The part only fits after a 90° rotation.
    pub rotated: bool,
}

impl PrinterProfile {
    pub fn find(name: &str) -> Option<PrinterProfile> {
        PRINTER_PROFILES.iter().copied().find(|p| p.name == name)
    }
}

/// Maximum number of cells that fit along a bed dimension of `bed` mm, leaving
/// `BED_MARGIN` on each side: `floor((bed − 2·MARGIN) / PITCH)`.
fn max_cells_for_bed(bed: i32) -> i32 {
    ((bed as f32 - 2.0 * BED_MARGIN) / PITCH as f32).floor() as i32
}

fn fits(width_mm: f32, depth_mm: f32, bed_w: i32, bed_d: i32) -> bool {
    let w = bed_w as f32 - 2.0 * BED_MARGIN;
    let d = bed_d as f32 - 2.0 * BED_MARGIN;
    width_mm <= w && depth_mm <= d
}

/// Whether the footprint fits the bed in either orientation.
pub fn check_bed_fit(cells: &[GridCell], printer: PrinterProfile) -> BedFitResult {
    let f = GridFootprint::from_cells(cells).unwrap_or(GridFootprint {
        min_x: 0,
        min_y: 0,
        width_cells: 0,
        depth_cells: 0,
    });
    let (w_mm, d_mm) = (
        f.width_cells as f32 * PITCH as f32,
        f.depth_cells as f32 * PITCH as f32,
    );
    let normal = fits(w_mm, d_mm, printer.bed_width, printer.bed_depth);
    let rotated = fits(d_mm, w_mm, printer.bed_width, printer.bed_depth);
    BedFitResult {
        fits: normal || rotated,
        bin_width: (w_mm) as i32,
        bin_depth: (d_mm) as i32,
        rotated: !normal && rotated,
    }
}

/// Candidate split-line indices along one axis, evenly spaced so each resulting
/// chunk is at most `max_cells` cells wide. Returns the interior line indices
/// (the cut positions), so `n` lines ⇒ `n+1` chunks.
fn axis_split_indices(_min: i32, span_cells: i32, max_cells: i32) -> Vec<i32> {
    if max_cells >= span_cells || span_cells <= 0 {
        return Vec::new();
    }
    let chunks = if max_cells > 0 {
        (span_cells as f32 / max_cells as f32).ceil() as i32
    } else {
        span_cells
    };
    let n = chunks.min(span_cells).max(1);
    let mut out = Vec::with_capacity((n - 1) as usize);
    for i in 0..n - 1 {
        // Line position relative to min, then offset back into grid coords.
        let rel = ((i + 1) as f32 * span_cells as f32 / n as f32).round() as i32;
        out.push(rel);
    }
    out
}

/// Lower-bound span of the cells along one axis (min, span).
fn axis_span(cells: &[GridCell], axis: Axis) -> (i32, i32) {
    let mut min = i32::MAX;
    let mut max = i32::MIN;
    for c in cells {
        let v = if axis == Axis::X { c.x } else { c.y };
        min = min.min(v);
        max = max.max(v);
    }
    (min, max - min + 1)
}

/// Score a candidate plan: fewer pieces, then fewer lines, then smaller worst
/// piece. Lower is better.
fn score(pieces: &[Piece], line_count: usize) -> (usize, usize, i32) {
    let worst = pieces
        .iter()
        .map(|p| {
            let f = GridFootprint::from_cells(&p.cells);
            f.map(|f| f.width_cells * f.depth_cells).unwrap_or(0)
        })
        .max()
        .unwrap_or(0);
    (pieces.len(), line_count, worst)
}

fn build_plan(
    _cells: &[GridCell],
    min_x: i32,
    span_x: i32,
    min_y: i32,
    span_y: i32,
    max_w: i32,
    max_d: i32,
) -> (Vec<SplitLine>, usize) {
    let mut lines = Vec::new();
    for rel in axis_split_indices(min_x, span_x, max_w) {
        lines.push(SplitLine { axis: Axis::X, index: min_x + rel });
    }
    for rel in axis_split_indices(min_y, span_y, max_d) {
        lines.push(SplitLine { axis: Axis::Y, index: min_y + rel });
    }
    let n = lines.len();
    (lines, n)
}

/// Choose the split-line plan that fits the whole region on the bed with the
/// fewest pieces. Returns the plan (possibly empty if it already fits).
pub fn compute_auto_split_lines(cells: &[GridCell], printer: PrinterProfile) -> Vec<SplitLine> {
    if check_bed_fit(cells, printer).fits {
        return Vec::new();
    }
    let (min_x, span_x) = axis_span(cells, Axis::X);
    let (min_y, span_y) = axis_span(cells, Axis::Y);

    // Try both bed orientations; keep the lowest-scored plan.
    let mut best: Option<(Vec<SplitLine>, (usize, usize, i32))> = None;
    for (max_w, max_d) in [
        (max_cells_for_bed(printer.bed_width), max_cells_for_bed(printer.bed_depth)),
        (max_cells_for_bed(printer.bed_depth), max_cells_for_bed(printer.bed_width)),
    ] {
        let (lines, n) = build_plan(cells, min_x, span_x, min_y, span_y, max_w, max_d);
        let pieces = partition_cells(cells, &lines);
        let s = score(&pieces, n);
        match &best {
            Some((_, bs)) if &s >= bs => {}
            _ => best = Some((lines, s)),
        }
    }
    best.map(|(l, _)| l).unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cells(coords: &[(i32, i32)]) -> Vec<GridCell> {
        coords.iter().map(|&(x, y)| GridCell { x, y }).collect()
    }

    #[test]
    fn small_bin_fits_default_printer() {
        let c = cells(&[(0, 0), (1, 0), (0, 1), (1, 1)]);
        let r = check_bed_fit(&c, DEFAULT_PRINTER);
        assert!(r.fits);
        assert!(!r.rotated);
    }

    #[test]
    fn oversized_bin_does_not_fit_a1_mini() {
        // 6×6 cells = 252×252 mm; A1 Mini bed 180×180.
        let mut c = Vec::new();
        for x in 0..6 {
            for y in 0..6 {
                c.push(GridCell { x, y });
            }
        }
        let r = check_bed_fit(&c, PRINTER_PROFILES[0]);
        assert!(!r.fits);
    }

    #[test]
    fn auto_split_empty_when_already_fits() {
        let c = cells(&[(0, 0), (1, 0)]);
        assert!(compute_auto_split_lines(&c, DEFAULT_PRINTER).is_empty());
    }

    #[test]
    fn auto_split_halves_oversized_run() {
        // 8×1 cells = 336 mm along X. A1 Mini fits 4 cells (168 mm) per piece.
        let c: Vec<GridCell> = (0..8).map(|x| GridCell { x, y: 0 }).collect();
        let lines = compute_auto_split_lines(&c, PRINTER_PROFILES[0]); // 180×180
        assert!(!lines.is_empty(), "expected a split plan");
        let pieces = partition_cells(&c, &lines);
        // Every piece must individually fit the bed.
        for p in &pieces {
            assert!(check_bed_fit(&p.cells, PRINTER_PROFILES[0]).fits, "piece {:?} too big", p.cells);
        }
    }

    #[test]
    fn axis_split_indices_caps_chunk_size() {
        // span 10, max 3 ⇒ chunks = ceil(10/3)=4, indices split into 4 groups.
        let idx = axis_split_indices(0, 10, 3);
        // 3 interior lines ⇒ 4 chunks; each chunk ≤ 3 wide.
        assert_eq!(idx.len(), 3);
        let mut prev = 0;
        for &cut in &idx {
            assert!(cut - prev <= 3, "chunk {}..{} too wide", prev, cut);
            prev = cut;
        }
        assert!(10 - prev <= 3);
    }
}
