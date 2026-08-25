use crate::layout::{Axis, GridCell, GridFootprint, PITCH, Piece, SplitLine, partition_cells};

pub const BED_MARGIN: f64 = 5.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PrinterProfile {
    pub name: &'static str,
    pub bed_width: i32,
    pub bed_depth: i32,
}

pub const PRINTER_PROFILES: &[PrinterProfile] = &[
    PrinterProfile {
        name: "Bambu Lab A1 Mini",
        bed_width: 180,
        bed_depth: 180,
    },
    PrinterProfile {
        name: "Bambu Lab P1S / X1C",
        bed_width: 256,
        bed_depth: 256,
    },
    PrinterProfile {
        name: "Creality Ender 3 / V2",
        bed_width: 220,
        bed_depth: 220,
    },
    PrinterProfile {
        name: "Creality K1",
        bed_width: 220,
        bed_depth: 220,
    },
    PrinterProfile {
        name: "Elegoo Centauri Carbon",
        bed_width: 256,
        bed_depth: 256,
    },
    PrinterProfile {
        name: "Prusa MK4 / MK3S+",
        bed_width: 250,
        bed_depth: 210,
    },
    PrinterProfile {
        name: "Prusa Mini+",
        bed_width: 180,
        bed_depth: 180,
    },
    PrinterProfile {
        name: "Voron 2.4 (250mm)",
        bed_width: 250,
        bed_depth: 250,
    },
    PrinterProfile {
        name: "Voron 2.4 (300mm)",
        bed_width: 300,
        bed_depth: 300,
    },
    PrinterProfile {
        name: "Custom",
        bed_width: 220,
        bed_depth: 220,
    },
];

pub const DEFAULT_PRINTER: PrinterProfile = PrinterProfile {
    name: "Prusa MK4 / MK3S+",
    bed_width: 250,
    bed_depth: 210,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BedFitResult {
    pub fits: bool,
    pub bin_width: i32,
    pub bin_depth: i32,
    pub rotated: bool,
}

impl PrinterProfile {
    pub fn find(name: &str) -> Option<PrinterProfile> {
        PRINTER_PROFILES.iter().copied().find(|p| p.name == name)
    }
}

fn max_cells_for_bed(bed: i32) -> i32 {
    ((bed as f64 - 2.0 * BED_MARGIN) / PITCH as f64).floor() as i32
}

fn fits(width_mm: f64, depth_mm: f64, bed_w: i32, bed_d: i32) -> bool {
    let w = bed_w as f64 - 2.0 * BED_MARGIN;
    let d = bed_d as f64 - 2.0 * BED_MARGIN;
    width_mm <= w && depth_mm <= d
}

pub fn check_bed_fit(cells: &[GridCell], printer: PrinterProfile) -> BedFitResult {
    let f = GridFootprint::from_cells(cells).unwrap_or(GridFootprint {
        min_x: 0,
        min_y: 0,
        width_cells: 0,
        depth_cells: 0,
    });
    let (w_mm, d_mm) = (
        f.width_cells as f64 * PITCH as f64,
        f.depth_cells as f64 * PITCH as f64,
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

fn axis_split_indices(_min: i32, span_cells: i32, max_cells: i32) -> Vec<i32> {
    if max_cells >= span_cells || span_cells <= 0 {
        return Vec::new();
    }
    let chunks = if max_cells > 0 {
        (span_cells as f64 / max_cells as f64).ceil() as i32
    } else {
        span_cells
    };
    let n = chunks.min(span_cells).max(1);
    let mut out = Vec::with_capacity((n - 1) as usize);
    for i in 0..n - 1 {
        let rel = ((i + 1) as f64 * span_cells as f64 / n as f64).round() as i32;
        out.push(rel);
    }
    out
}

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
        lines.push(SplitLine {
            axis: Axis::X,
            index: min_x + rel,
        });
    }
    for rel in axis_split_indices(min_y, span_y, max_d) {
        lines.push(SplitLine {
            axis: Axis::Y,
            index: min_y + rel,
        });
    }
    let n = lines.len();
    (lines, n)
}

pub fn compute_auto_split_lines(cells: &[GridCell], printer: PrinterProfile) -> Vec<SplitLine> {
    if check_bed_fit(cells, printer).fits {
        return Vec::new();
    }
    let (min_x, span_x) = axis_span(cells, Axis::X);
    let (min_y, span_y) = axis_span(cells, Axis::Y);

    let mut best: Option<(Vec<SplitLine>, (usize, usize, i32))> = None;
    for (max_w, max_d) in [
        (
            max_cells_for_bed(printer.bed_width),
            max_cells_for_bed(printer.bed_depth),
        ),
        (
            max_cells_for_bed(printer.bed_depth),
            max_cells_for_bed(printer.bed_width),
        ),
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
        let c: Vec<GridCell> = (0..8).map(|x| GridCell { x, y: 0 }).collect();
        let lines = compute_auto_split_lines(&c, PRINTER_PROFILES[0]);
        assert!(!lines.is_empty(), "expected a split plan");
        let pieces = partition_cells(&c, &lines);
        for p in &pieces {
            assert!(
                check_bed_fit(&p.cells, PRINTER_PROFILES[0]).fits,
                "piece {:?} too big",
                p.cells
            );
        }
    }

    #[test]
    fn axis_split_indices_caps_chunk_size() {
        let idx = axis_split_indices(0, 10, 3);
        assert_eq!(idx.len(), 3);
        let mut prev = 0;
        for &cut in &idx {
            assert!(cut - prev <= 3, "chunk {}..{} too wide", prev, cut);
            prev = cut;
        }
        assert!(10 - prev <= 3);
    }
}
