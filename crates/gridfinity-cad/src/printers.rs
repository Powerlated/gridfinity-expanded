use crate::layout::{Axis, GridCell, GridFootprint, Piece, SplitLine, axis_lines, partition_cells};

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

    /// Whether a body of `width_mm` by `depth_mm` prints on this bed, and
    /// whether it has to be turned to do it, keeping `BED_MARGIN` clear on every
    /// side.
    ///
    /// This is the whole bed-fit question, asked of a body measured in
    /// millimetres. `check_bed_fit` is the same question asked of a cell set,
    /// and a cell count is the wrong measure for anything that reaches past its
    /// own cells -- a bin is inset from them, a fitted baseplate spans past
    /// them.
    pub fn bed_fit_mm(&self, width_mm: f64, depth_mm: f64) -> BedFitResult {
        assert!(
            width_mm >= 0.0 && depth_mm >= 0.0,
            "a body measures {width_mm} x {depth_mm} mm, which is not a footprint"
        );
        let normal = fits(width_mm, depth_mm, self.bed_width, self.bed_depth);
        let rotated = fits(depth_mm, width_mm, self.bed_width, self.bed_depth);
        BedFitResult {
            fits: normal || rotated,
            bin_width: width_mm.round() as i32,
            bin_depth: depth_mm.round() as i32,
            rotated: !normal && rotated,
        }
    }
}

fn max_cells_for_bed(bed: i32, pitch: f64) -> i32 {
    ((bed as f64 - 2.0 * BED_MARGIN) / pitch).floor() as i32
}

fn fits(width_mm: f64, depth_mm: f64, bed_w: i32, bed_d: i32) -> bool {
    let w = bed_w as f64 - 2.0 * BED_MARGIN;
    let d = bed_d as f64 - 2.0 * BED_MARGIN;
    width_mm <= w && depth_mm <= d
}

pub fn check_bed_fit(cells: &[GridCell], printer: PrinterProfile, pitch: f64) -> BedFitResult {
    let f = GridFootprint::from_cells(cells).unwrap_or(GridFootprint {
        min_x: 0,
        min_y: 0,
        width_cells: 0,
        depth_cells: 0,
    });
    let (w_mm, d_mm) = f.mm(pitch);
    printer.bed_fit_mm(w_mm, d_mm)
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

pub fn compute_auto_split_lines(
    cells: &[GridCell],
    printer: PrinterProfile,
    pitch: f64,
) -> Vec<SplitLine> {
    if check_bed_fit(cells, printer, pitch).fits {
        return Vec::new();
    }
    let (min_x, span_x) = axis_span(cells, Axis::X);
    let (min_y, span_y) = axis_span(cells, Axis::Y);

    let mut best: Option<(Vec<SplitLine>, (usize, usize, i32))> = None;
    for (max_w, max_d) in [
        (
            max_cells_for_bed(printer.bed_width, pitch),
            max_cells_for_bed(printer.bed_depth, pitch),
        ),
        (
            max_cells_for_bed(printer.bed_depth, pitch),
            max_cells_for_bed(printer.bed_width, pitch),
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

/// The millimetres a chunk of a split run measures along its axis: its cells at
/// `pitch`, plus half of `overhang` for each end of the whole run of `span`
/// cells that the chunk reaches. `p` and `q` are relative cell positions, so the
/// chunk holds cells `p..q`.
///
/// The overhang is what a body reaching past its own cells adds to the piece
/// that carries it -- a fitted baseplate stands half the drawer's leftover
/// millimetres outside the grid on each side of an axis -- and only the two
/// outermost chunks of a run ever carry any of it.
fn chunk_mm(p: i32, q: i32, span: i32, pitch: f64, overhang: f64) -> f64 {
    assert!(
        0 <= p && p < q && q <= span,
        "a chunk of a {span}-cell run holds cells {p}..{q}, which is not a run of at least one cell"
    );
    assert!(
        overhang >= 0.0,
        "a body reaches {overhang} mm past its cells, which is not a distance"
    );
    let ends = i32::from(p == 0) + i32::from(q == span);
    (q - p) as f64 * pitch + f64::from(ends) * overhang / 2.0
}

/// For every relative position `p` in `0..=span`, the fewest chunks the cells
/// `p..span` can be covered by when each chunk measures at most `limit_mm` and
/// every cut lands on a position `allowed` accepts, or `usize::MAX` where no
/// such cover exists. The entry for `span` itself is always 0.
///
/// A split plan along an axis *is* such a cover: the cuts are the chunk
/// boundaries, the piece count is the chunk count, and the plan with the fewest
/// pieces is the shortest cover.
fn chunk_cover(
    span: i32,
    pitch: f64,
    overhang: f64,
    limit_mm: f64,
    allowed: &dyn Fn(i32) -> bool,
) -> Vec<usize> {
    assert!(span > 0, "a run to cover has at least one cell, not {span}");
    assert!(
        pitch > 0.0,
        "a grid pitch is a positive number of millimetres, not {pitch}"
    );
    let mut from = vec![usize::MAX; span as usize + 1];
    from[span as usize] = 0;
    for p in (0..span).rev() {
        for q in p + 1..=span {
            if chunk_mm(p, q, span, pitch, overhang) > limit_mm {
                break;
            }
            if q < span && !allowed(q) {
                continue;
            }
            let rest = from[q as usize];
            if rest != usize::MAX {
                from[p as usize] = from[p as usize].min(rest + 1);
            }
        }
    }
    from
}

/// The cuts of one shortest cover, read forward off `from` and taken as near an
/// even division as that cover allows: at each step the chunk end that keeps the
/// piece count minimal and lies nearest the remaining cells shared out over the
/// remaining chunks.
///
/// Evenness is the tie-break only. Every position returned is one `allowed`
/// accepts and every chunk it bounds is inside `limit_mm`, because only ends
/// that keep `from` minimal are ever taken.
fn cover_cuts(
    span: i32,
    pitch: f64,
    overhang: f64,
    limit_mm: f64,
    allowed: &dyn Fn(i32) -> bool,
    from: &[usize],
) -> Vec<i32> {
    assert!(
        from[0] != usize::MAX,
        "a run of {span} cells that no set of chunks covers has no cuts to read off it"
    );
    let mut cuts = Vec::with_capacity(from[0] - 1);
    let mut p = 0;
    while from[p as usize] > 1 {
        let remaining = from[p as usize];
        let ideal = p as f64 + (span - p) as f64 / remaining as f64;
        let mut best: Option<i32> = None;
        for q in p + 1..span {
            if chunk_mm(p, q, span, pitch, overhang) > limit_mm {
                break;
            }
            if !allowed(q) || from[q as usize] != remaining - 1 {
                continue;
            }
            if best.is_none_or(|b| (q as f64 - ideal).abs() < (b as f64 - ideal).abs()) {
                best = Some(q);
            }
        }
        let q = best.unwrap_or_else(|| {
            panic!(
                "the cover says {remaining} chunks cover cells {p}..{span}, so one of them ends inside it"
            )
        });
        cuts.push(q);
        p = q;
    }
    assert_eq!(
        cuts.len() + 1,
        from[0],
        "a cover of {} chunks is bounded by {} cuts",
        from[0],
        from[0] - 1
    );
    cuts
}

/// Where a run of `span` cells is cut so every chunk prints within `limit_mm`,
/// no cut lands within `clearance` cells of a position in `avoid`, and the chunk
/// count is the smallest that allows. `None` when no such plan exists -- one
/// cell already overruns the bed, or every position that would divide the run
/// lies too near one the caller asked to keep clear.
///
/// Positions are relative to the run's first cell and a cut at `r` separates
/// cell `r - 1` from cell `r`, which is how `partition_cells` reads a line;
/// `avoid` is in those same relative positions. A `clearance` of 1 is the
/// interlock condition itself -- the two bodies do not part on the same line --
/// and asking for more buys a wider overlap at every seam.
fn plan_axis(
    span: i32,
    pitch: f64,
    overhang: f64,
    limit_mm: f64,
    avoid: &[i32],
    clearance: i32,
) -> Option<Vec<i32>> {
    assert!(
        clearance >= 1,
        "a cut kept {clearance} cells from another is not kept off it at all"
    );
    let clear = |q: i32| avoid.iter().all(|&a| (q - a).abs() >= clearance);
    let cover = chunk_cover(span, pitch, overhang, limit_mm, &clear);
    if cover[0] == usize::MAX {
        return None;
    }
    Some(cover_cuts(span, pitch, overhang, limit_mm, &clear, &cover))
}

/// The widest clearance, in cells, at which a run still covers in `fewest`
/// chunks: the largest one whose cover is no longer than the cover at clearance
/// 1, which is the plan that merely refuses to share a line.
///
/// Standing a seam further off its neighbour widens the overlap that ties the
/// two bodies together, so it is worth having -- but never at the price of
/// another piece, which is why the count at clearance 1 is the bar.
fn widest_clearance(
    span: i32,
    pitch: f64,
    overhang: f64,
    limit_mm: f64,
    avoid: &[i32],
    fewest: usize,
) -> i32 {
    assert!(
        fewest >= 1,
        "a run of {span} cells covers in at least one chunk, not {fewest}"
    );
    let mut best = 1;
    for clearance in 2..=span.max(1) {
        let clear = |q: i32| avoid.iter().all(|&a| (q - a).abs() >= clearance);
        if chunk_cover(span, pitch, overhang, limit_mm, &clear)[0] != fewest {
            break;
        }
        best = clearance;
    }
    best
}

/// Where a body over `cells` is cut so every piece prints on `printer` and no
/// cut coincides with one of `avoid`, or `None` when the two cannot both hold.
/// `overhang` is the millimetres the body reaches past its cells along each axis
/// in total -- `(0.0, 0.0)` for a body that is its cells, the drawer's leftover
/// margin for a fitted baseplate -- standing half outside each end.
///
/// This is `compute_auto_split_lines` with the two things a *second* body needs.
/// It is measured in millimetres rather than whole cells, because a body that
/// overhangs is not its cell count. And it is given the first body's lines to
/// keep off: cutting the two bodies on different lines is what makes the stack
/// hold itself together, because every seam of one is then spanned by a piece of
/// the other and no piece can leave without the pieces it laps.
///
/// An empty result means the whole body prints uncut, which is the strongest
/// interlock there is.
pub fn compute_staggered_split_lines(
    cells: &[GridCell],
    printer: PrinterProfile,
    pitch: f64,
    overhang: (f64, f64),
    avoid: &[SplitLine],
) -> Option<Vec<SplitLine>> {
    let (min_x, span_x) = axis_span(cells, Axis::X);
    let (min_y, span_y) = axis_span(cells, Axis::Y);
    assert!(
        span_x > 0 && span_y > 0,
        "a body of {} cells spans {span_x} x {span_y} cells, which is not a footprint",
        cells.len()
    );
    let relative = |axis: Axis, min: i32| -> Vec<i32> {
        axis_lines(avoid, axis).iter().map(|l| l - min).collect()
    };
    let (avoid_x, avoid_y) = (relative(Axis::X, min_x), relative(Axis::Y, min_y));
    let mut best: Option<(Vec<SplitLine>, usize)> = None;
    for (bed_x, bed_y) in [
        (printer.bed_width, printer.bed_depth),
        (printer.bed_depth, printer.bed_width),
    ] {
        let limit_x = f64::from(bed_x) - 2.0 * BED_MARGIN;
        let limit_y = f64::from(bed_y) - 2.0 * BED_MARGIN;
        let (Some(x), Some(y)) = (
            plan_axis(span_x, pitch, overhang.0, limit_x, &avoid_x, 1),
            plan_axis(span_y, pitch, overhang.1, limit_y, &avoid_y, 1),
        ) else {
            continue;
        };
        let wide_x = widest_clearance(span_x, pitch, overhang.0, limit_x, &avoid_x, x.len() + 1);
        let wide_y = widest_clearance(span_y, pitch, overhang.1, limit_y, &avoid_y, y.len() + 1);
        let x = plan_axis(span_x, pitch, overhang.0, limit_x, &avoid_x, wide_x)
            .expect("the widest clearance is one a plan was already found at");
        let y = plan_axis(span_y, pitch, overhang.1, limit_y, &avoid_y, wide_y)
            .expect("the widest clearance is one a plan was already found at");
        let pieces = (x.len() + 1) * (y.len() + 1);
        let mut lines: Vec<SplitLine> = x
            .iter()
            .map(|&r| SplitLine {
                axis: Axis::X,
                index: min_x + r,
            })
            .collect();
        lines.extend(y.iter().map(|&r| SplitLine {
            axis: Axis::Y,
            index: min_y + r,
        }));
        if best.as_ref().is_none_or(|(_, n)| pieces < *n) {
            best = Some((lines, pieces));
        }
    }
    let (lines, _) = best?;
    assert!(
        lines.iter().all(|l| !avoid.contains(l)),
        "a staggered plan shares a cut line with the body it was staggered against"
    );
    Some(lines)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gridfinity::GRID_PITCH;

    fn cells(coords: &[(i32, i32)]) -> Vec<GridCell> {
        coords.iter().map(|&(x, y)| GridCell { x, y }).collect()
    }

    #[test]
    fn small_bin_fits_default_printer() {
        let c = cells(&[(0, 0), (1, 0), (0, 1), (1, 1)]);
        let r = check_bed_fit(&c, DEFAULT_PRINTER, GRID_PITCH);
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
        let r = check_bed_fit(&c, PRINTER_PROFILES[0], GRID_PITCH);
        assert!(!r.fits);
    }

    #[test]
    fn auto_split_empty_when_already_fits() {
        let c = cells(&[(0, 0), (1, 0)]);
        assert!(compute_auto_split_lines(&c, DEFAULT_PRINTER, GRID_PITCH).is_empty());
    }

    #[test]
    fn auto_split_halves_oversized_run() {
        let c: Vec<GridCell> = (0..8).map(|x| GridCell { x, y: 0 }).collect();
        let lines = compute_auto_split_lines(&c, PRINTER_PROFILES[0], GRID_PITCH);
        assert!(!lines.is_empty(), "expected a split plan");
        let pieces = partition_cells(&c, &lines);
        for p in &pieces {
            assert!(
                check_bed_fit(&p.cells, PRINTER_PROFILES[0], GRID_PITCH).fits,
                "piece {:?} too big",
                p.cells
            );
        }
    }

    /// The condition the whole staggering exists for, asked of a plan directly:
    /// no seam of one body lies on a seam of the other.
    fn shares_a_line(a: &[SplitLine], b: &[SplitLine]) -> bool {
        a.iter().any(|l| b.contains(l))
    }

    #[test]
    fn a_staggered_plate_shares_no_cut_line_with_the_bin() {
        let c: Vec<GridCell> = (0..8)
            .flat_map(|x| (0..8).map(move |y| GridCell { x, y }))
            .collect();
        let bin = compute_auto_split_lines(&c, PRINTER_PROFILES[0], GRID_PITCH);
        assert!(!bin.is_empty(), "an 8 x 8 grid does not fit a 180 mm bed whole");
        let plate = compute_staggered_split_lines(
            &c,
            PRINTER_PROFILES[0],
            GRID_PITCH,
            (0.0, 0.0),
            &bin,
        )
        .expect("an 8 x 8 grid has room to be cut somewhere else");
        assert!(
            !shares_a_line(&plate, &bin),
            "the plate is cut at {plate:?} and the bin at {bin:?}, which part on the same plane"
        );
        for piece in partition_cells(&c, &plate) {
            assert!(
                check_bed_fit(&piece.cells, PRINTER_PROFILES[0], GRID_PITCH).fits,
                "the staggered piece {:?} does not print",
                piece.cells
            );
        }
    }

    #[test]
    fn a_plate_that_prints_whole_is_not_cut_at_all() {
        let c: Vec<GridCell> = (0..2)
            .flat_map(|x| (0..2).map(move |y| GridCell { x, y }))
            .collect();
        let bin = vec![SplitLine {
            axis: Axis::X,
            index: 1,
        }];
        let plate =
            compute_staggered_split_lines(&c, DEFAULT_PRINTER, GRID_PITCH, (0.0, 0.0), &bin)
                .expect("a two-cell plate prints whole");
        assert!(plate.is_empty(), "an uncut plate spans every seam of the bin over it");
    }

    /// The plate is not its cells: a fitted one stands half the drawer's
    /// leftover millimetres outside the grid at each end, and a plan measured in
    /// whole cells is how it came to outgrow the bed it was split for.
    #[test]
    fn an_overhanging_plate_is_measured_where_it_reaches_to() {
        let c: Vec<GridCell> = (0..4).map(|x| GridCell { x, y: 0 }).collect();
        let bare = compute_staggered_split_lines(&c, PRINTER_PROFILES[0], GRID_PITCH, (0.0, 0.0), &[])
            .expect("four 42 mm cells are 168 mm, which prints on a 180 mm bed");
        assert!(bare.is_empty(), "168 mm prints whole, so nothing is cut");
        let flanged =
            compute_staggered_split_lines(&c, PRINTER_PROFILES[0], GRID_PITCH, (40.0, 0.0), &[])
                .expect("half the flange at each end still divides somewhere");
        assert_eq!(
            flanged.len(),
            1,
            "168 mm of cells plus a 40 mm flange is 208 mm, which does not print whole"
        );
    }

    /// A run with nowhere left to be cut: two cells that must be divided, whose
    /// one dividing line is the one the caller asked to keep clear.
    #[test]
    fn a_run_with_no_line_left_to_take_has_no_staggered_plan() {
        let c: Vec<GridCell> = (0..2).map(|x| GridCell { x, y: 0 }).collect();
        let narrow = PrinterProfile {
            name: "one cell",
            bed_width: 52,
            bed_depth: 52,
        };
        let bin = vec![SplitLine {
            axis: Axis::X,
            index: 1,
        }];
        assert_eq!(
            compute_staggered_split_lines(&c, narrow, GRID_PITCH, (0.0, 0.0), &bin),
            None,
            "the only line that divides two cells is the bin's own"
        );
        assert!(
            compute_staggered_split_lines(&c, narrow, GRID_PITCH, (0.0, 0.0), &[]).is_some(),
            "with nothing to avoid, the same run divides at that line"
        );
    }

    /// A plan keeps off the line it was given and stays at the piece count that
    /// line would have bought: six cells in two four-cell chunks, cut beside the
    /// bin's seam rather than on it.
    #[test]
    fn a_staggered_seam_stands_as_far_off_the_bin_seam_as_the_piece_count_allows() {
        let c: Vec<GridCell> = (0..6).map(|x| GridCell { x, y: 0 }).collect();
        let bed = PrinterProfile {
            name: "four cells",
            bed_width: 178,
            bed_depth: 178,
        };
        let bin = compute_auto_split_lines(&c, bed, GRID_PITCH);
        assert_eq!(bin, vec![SplitLine { axis: Axis::X, index: 3 }]);
        let plate = compute_staggered_split_lines(&c, bed, GRID_PITCH, (0.0, 0.0), &bin)
            .expect("a six-cell run cut in two has three lines to choose from");
        assert_eq!(
            plate,
            vec![SplitLine { axis: Axis::X, index: 2 }],
            "the plate parts beside the bin's seam, not on it, and still in two pieces"
        );
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
