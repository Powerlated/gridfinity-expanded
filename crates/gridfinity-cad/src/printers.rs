use crate::layout::{Axis, GridCell, GridFootprint, Piece, SplitLine, axis_lines, partition_cells};

/// What is kept clear of each of the bed's four sides, in millimetres.
///
/// Zero: a bed's stated size is the size of it, and a body that measures inside
/// it prints. A caller wanting clearance says so by stating a smaller bed --
/// `optimize`'s `settings.bed = { width, depth }` -- which is the same number
/// arrived at where the person who knows their machine can see it, rather than
/// a silent 5 mm that refused a bin fitting the bed with 1.4 mm to spare.
pub const BED_MARGIN: f64 = 0.0;

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

/// Whether a body prints on a bed and how it has to be laid down to: `rotated`
/// for a quarter turn, `tilt_deg` for an angle between the two, in degrees. At
/// most one of them is set, and a body that lies flat sets neither.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BedFitResult {
    pub fits: bool,
    pub bin_width: i32,
    pub bin_depth: i32,
    pub rotated: bool,
    pub tilt_deg: Option<f64>,
}

impl PrinterProfile {
    pub fn find(name: &str) -> Option<PrinterProfile> {
        PRINTER_PROFILES.iter().copied().find(|p| p.name == name)
    }

    /// The bed's two dimensions in millimetres with `BED_MARGIN` taken off each
    /// of its four sides, in the profile's own order.
    ///
    /// This is the rectangle every fit question here is actually asked of; the
    /// bed itself is never compared against directly.
    pub fn usable(&self) -> (f64, f64) {
        let w = f64::from(self.bed_width) - 2.0 * BED_MARGIN;
        let d = f64::from(self.bed_depth) - 2.0 * BED_MARGIN;
        assert!(
            w > 0.0 && d > 0.0,
            "a {} x {} mm bed keeping {BED_MARGIN} mm clear on every side has no room left on it",
            self.bed_width,
            self.bed_depth
        );
        assert!(
            BED_MARGIN >= 0.0,
            "a bed cannot be larger than itself, so {BED_MARGIN} mm is not a margin"
        );
        (w, d)
    }

    /// Whether a body of `width_mm` by `depth_mm` prints on this bed at some
    /// orientation, and which one it needs: turned a quarter, or laid at an angle
    /// between the two, keeping `BED_MARGIN` clear on every side.
    ///
    /// This is the whole bed-fit question, asked of a body measured in
    /// millimetres. `check_bed_fit` is the same question asked of a cell set,
    /// and a cell count is the wrong measure for anything that reaches past its
    /// own cells -- a bin is inset from them, a fitted baseplate spans past
    /// them.
    ///
    /// `rotated` and `tilt_deg` name the placement the body needs, so at most one
    /// of them is set and neither is set for a body that lies flat. A slicer is
    /// free to turn a part to any angle, so a long thin body that clears the bed
    /// only across its diagonal still prints and must not be cut for the bed --
    /// and `tilt_deg` is the angle to type into the slicer to do it.
    ///
    /// Monotone: shrinking either side of a body that prints leaves a body that
    /// prints. That is what lets a whole split plan be judged by its widest chunk
    /// against its deepest.
    pub fn bed_fit_mm(&self, width_mm: f64, depth_mm: f64) -> BedFitResult {
        assert!(
            width_mm >= 0.0 && depth_mm >= 0.0,
            "a body measures {width_mm} x {depth_mm} mm, which is not a footprint"
        );
        let (bed_w, bed_d) = self.usable();
        let flat = fits_flat(width_mm, depth_mm, bed_w, bed_d);
        let turned = fits_flat(depth_mm, width_mm, bed_w, bed_d);
        let tilt_deg = if flat || turned {
            None
        } else {
            tilt_angle(width_mm, depth_mm, bed_w, bed_d)
        };
        assert!(
            tilt_deg.is_none_or(|t| (0.0..=90.0).contains(&t)),
            "a body is laid down within a quarter turn, not at {tilt_deg:?} degrees"
        );
        BedFitResult {
            fits: flat || turned || tilt_deg.is_some(),
            bin_width: width_mm.round() as i32,
            bin_depth: depth_mm.round() as i32,
            rotated: !flat && turned,
            tilt_deg,
        }
    }
}

/// Whether a `width_mm` x `depth_mm` body lies inside a `bed_w` x `bed_d`
/// rectangle with its sides parallel to the bed's. Both arguments already carry
/// whatever margin the caller keeps clear.
fn fits_flat(width_mm: f64, depth_mm: f64, bed_w: f64, bed_d: f64) -> bool {
    width_mm <= bed_w && depth_mm <= bed_d
}

/// The quarter turn a body is free to be laid down within, in radians. Both
/// bounding-box constraints are symmetric under a half turn and swap under a
/// quarter, so every distinct placement is somewhere in `[0, QUARTER_TURN]`.
const QUARTER_TURN: f64 = std::f64::consts::FRAC_PI_2;

/// How far the ladder in `tilt_angle` refines before it gives up on a round
/// number: eight halvings of the quarter turn, so down to 0.35 degrees.
const TILT_LADDER_DEPTH: u32 = 8;

/// The angles within `[0, QUARTER_TURN]` at which `R * cos(t - phase)` stays
/// within `limit`, as up to two runs in ascending order.
///
/// A bounding-box side of a turned body is exactly one such cosine, so this is
/// one bed side's whole demand. Two runs because the body may lean either way
/// out of the angle where that side is longest, and the arc between the two
/// leans is what the side refuses.
fn cosine_runs(limit: f64, phase: f64, r: f64) -> Vec<(f64, f64)> {
    assert!(
        r > 0.0 && (0.0..=QUARTER_TURN).contains(&phase),
        "a body of extent {r} at phase {phase} is not a turned rectangle's bounding side"
    );
    if r <= limit {
        return vec![(0.0, QUARTER_TURN)];
    }
    let k = (limit / r).clamp(-1.0, 1.0).acos();
    [(0.0, phase - k), (phase + k, QUARTER_TURN)]
        .into_iter()
        .map(|(lo, hi)| (lo.max(0.0), hi.min(QUARTER_TURN)))
        .filter(|(lo, hi)| lo <= hi)
        .collect()
}

/// Every angle, in radians, at which a `width_mm` x `depth_mm` body's bounding
/// box lies inside a `bed_w` x `bed_d` bed: up to two disjoint runs of
/// `[0, QUARTER_TURN]`, and empty when no angle takes it at all.
///
/// Turning a `p` x `q` body by `t` gives it a bounding box of
/// `p cos t + q sin t` by `p sin t + q cos t`, and each of those is
/// `R cos(t - phase)` for `R = hypot(p, q)` and the two phases `atan2(q, p)` and
/// `atan2(p, q)`. So each bed side admits the angles outside one arc, and the
/// answer is the intersection of the two -- which is what makes the fit
/// question exact and closed form rather than a search, and what lets an angle
/// be named as well as a yes or no.
///
/// A run reaching 0 is the flat placement and one reaching `QUARTER_TURN` is the
/// turned one; `bed_fit_mm` tests those two itself and asks here only about what
/// is left.
fn tilt_window(width_mm: f64, depth_mm: f64, bed_w: f64, bed_d: f64) -> Vec<(f64, f64)> {
    let (p, q) = (width_mm.max(depth_mm), width_mm.min(depth_mm));
    let (a, b) = (bed_w.max(bed_d), bed_w.min(bed_d));
    let r = p.hypot(q);
    if r <= 0.0 {
        return vec![(0.0, QUARTER_TURN)];
    }
    let mut runs = Vec::new();
    for long in cosine_runs(a, q.atan2(p), r) {
        for short in cosine_runs(b, p.atan2(q), r) {
            let (lo, hi) = (long.0.max(short.0), long.1.min(short.1));
            if lo <= hi {
                runs.push((lo, hi));
            }
        }
    }
    runs.sort_by(|x, y| x.partial_cmp(y).expect("an angle is a finite number of radians"));
    assert!(
        runs.len() <= 2,
        "a body's two bed constraints are one arc each, so they leave at most two runs, not {}",
        runs.len()
    );
    runs
}

/// The angle, in **degrees**, to lay a `width_mm` x `depth_mm` body down at so
/// it prints on a `bed_w` x `bed_d` bed, or `None` where no angle takes it.
///
/// Coarse angles first: 45 degrees, then 22.5 and 67.5, then halfway between
/// those again, and so on for `TILT_LADDER_DEPTH` halvings -- so a fit that a
/// round angle takes is reported as that round angle, which is what a slicer's
/// rotation box wants typed into it. Only a window too narrow for the ladder to
/// land in falls through to the midpoint of its first run, which always fits
/// because the window is exactly the angles that do.
///
/// The ladder decides nothing: `tilt_window` has already settled whether the
/// body prints, and this only picks which of the angles it named to report.
fn tilt_angle(width_mm: f64, depth_mm: f64, bed_w: f64, bed_d: f64) -> Option<f64> {
    let runs = tilt_window(width_mm, depth_mm, bed_w, bed_d);
    let holds = |t: f64| runs.iter().any(|&(lo, hi)| lo <= t && t <= hi);
    let &(lo, hi) = runs.first()?;
    for level in 1..=TILT_LADDER_DEPTH {
        let step = QUARTER_TURN / f64::from(1u32 << level);
        let mut t = step;
        while t < QUARTER_TURN {
            if holds(t) {
                return Some(t.to_degrees());
            }
            t += 2.0 * step;
        }
    }
    Some((0.5 * (lo + hi)).to_degrees())
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

/// The relative positions where a run of `span` cells is cut into `n` chunks as
/// evenly as whole cells allow: the `n - 1` positions `round(i * span / n)`.
///
/// Positions are relative to the run's first cell and a cut at `r` separates cell
/// `r - 1` from cell `r`, which is how `partition_cells` reads a line. The chunks
/// differ in length by at most one cell, so the longest is `ceil(span / n)`.
fn even_cuts(span: i32, n: i32) -> Vec<i32> {
    assert!(
        span > 0 && 1 <= n && n <= span,
        "a run of {span} cells does not divide into {n} chunks of at least one cell each"
    );
    let mut out = Vec::with_capacity((n - 1) as usize);
    for i in 0..n - 1 {
        out.push(((i + 1) as f64 * span as f64 / n as f64).round() as i32);
    }
    out
}

/// The longest chunk, in cells, that an even division of `span` into `n` makes.
fn widest_chunk(span: i32, n: i32) -> i32 {
    let cuts = even_cuts(span, n);
    let mut widest = 0;
    let mut p = 0;
    for &q in cuts.iter().chain(std::iter::once(&span)) {
        widest = widest.max(q - p);
        p = q;
    }
    assert_eq!(
        widest,
        (span + n - 1) / n,
        "an even division of {span} cells into {n} chunks makes its longest chunk ceil({span}/{n})"
    );
    widest
}

/// The lines cutting a body that spans `span_x` cells from `min_x` and `span_y`
/// from `min_y` into an `nx` by `ny` grid of chunks, each axis divided as evenly
/// as whole cells allow.
fn even_plan(min_x: i32, span_x: i32, nx: i32, min_y: i32, span_y: i32, ny: i32) -> Vec<SplitLine> {
    let mut lines = Vec::new();
    for rel in even_cuts(span_x, nx) {
        lines.push(SplitLine {
            axis: Axis::X,
            index: min_x + rel,
        });
    }
    for rel in even_cuts(span_y, ny) {
        lines.push(SplitLine {
            axis: Axis::Y,
            index: min_y + rel,
        });
    }
    assert_eq!(
        lines.len(),
        (nx - 1 + ny - 1) as usize,
        "an {nx} by {ny} grid of chunks is bounded by {} cut lines",
        nx - 1 + ny - 1
    );
    lines
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

/// Where a body over `cells` is cut so that every chunk prints on `printer`, in
/// the fewest pieces that allows. Empty when the whole body already prints,
/// which includes a body that prints only laid at an angle.
///
/// The two axes are chosen together, not one at a time against a per-axis cell
/// cap, because whether a chunk prints is a question about both of its
/// dimensions at once: a chunk longer than the bed still prints if it is narrow
/// enough to lie across it. A candidate `nx` by `ny` division is feasible when
/// its widest chunk against its deepest prints -- that pair is itself one of the
/// chunks, the chunks of a grid being every pairing of an x range with a y range,
/// and `bed_fit_mm` is monotone, so the one test covers all of them.
///
/// A body no division prints falls back to one chunk per cell, which is what a
/// per-axis cap of zero used to produce: a single cell overruns the bed, and the
/// caller reads that off the pieces rather than off an empty plan.
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
    assert!(
        span_x > 0 && span_y > 0,
        "a body of {} cells spans {span_x} x {span_y} cells, which is not a footprint",
        cells.len()
    );

    let mut best: Option<(Vec<SplitLine>, (usize, usize, i32))> = None;
    for nx in 1..=span_x {
        for ny in 1..=span_y {
            let w = f64::from(widest_chunk(span_x, nx)) * pitch;
            let d = f64::from(widest_chunk(span_y, ny)) * pitch;
            if !printer.bed_fit_mm(w, d).fits {
                continue;
            }
            let lines = even_plan(min_x, span_x, nx, min_y, span_y, ny);
            let s = score(&partition_cells(cells, &lines), lines.len());
            if best.as_ref().is_none_or(|(_, bs)| &s < bs) {
                best = Some((lines, s));
            }
        }
    }
    best.map(|(l, _)| l)
        .unwrap_or_else(|| even_plan(min_x, span_x, span_x, min_y, span_y, span_y))
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

/// Every length, in millimetres, that a chunk of a `span`-cell run can measure:
/// `chunk_mm` over every cell range of the run, deduplicated and ascending.
///
/// A split plan's feasibility is decided by its widest chunk against its
/// deepest, because the chunks of a grid are every pairing of an x range with a
/// y range and `bed_fit_mm` is monotone. Both of those are lengths in this list,
/// which is what makes pairing one axis's candidates against the other's a
/// complete search over the plans a pair of per-axis limits can express -- and a
/// pair of limits is the only thing `plan_axis` can be told.
///
/// Finite and exact, which a bound solved for as a real number is not: a limit
/// taken from this list either admits a chunk or refuses it outright, where a
/// bisected supremum sits a hair under the true one and refuses a chunk exactly
/// as wide as the bed.
fn candidate_limits(span: i32, pitch: f64, overhang: f64) -> Vec<f64> {
    assert!(span > 0, "a run to cover has at least one cell, not {span}");
    let mut out = Vec::new();
    for p in 0..span {
        for q in p + 1..=span {
            out.push(chunk_mm(p, q, span, pitch, overhang));
        }
    }
    out.sort_by(|a, b| {
        a.partial_cmp(b)
            .expect("a chunk of a run is a finite number of millimetres long")
    });
    out.dedup();
    assert!(
        !out.is_empty(),
        "a run of {span} cells has at least the whole run as a chunk"
    );
    out
}

/// The longest chunk, in millimetres, that cutting a `span`-cell run at `cuts`
/// leaves. `cuts` are relative positions in ascending order, as `plan_axis`
/// returns them.
fn widest_chunk_mm(span: i32, cuts: &[i32], pitch: f64, overhang: f64) -> f64 {
    let mut widest = 0.0f64;
    let mut p = 0;
    for &q in cuts.iter().chain(std::iter::once(&span)) {
        widest = widest.max(chunk_mm(p, q, span, pitch, overhang));
        p = q;
    }
    assert!(
        widest > 0.0,
        "a run of {span} cells cut at {cuts:?} has a longest chunk of some length"
    );
    widest
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
/// The two axes are bounded **together**. Whether a chunk prints is a question
/// about both of its dimensions at once -- a chunk longer than the bed still
/// prints if it is narrow enough to lie across it -- so the depth limit is swept
/// over the lengths a chunk of the run can measure and each is paired with the
/// longest width that prints against it. Every chunk of the resulting plan is
/// then inside both limits and prints, whichever chunk it is, and the plan with
/// the fewest pieces over the whole sweep is the one returned.
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
    let widths = candidate_limits(span_x, pitch, overhang.0);
    let mut best: Option<(Vec<SplitLine>, usize)> = None;
    for limit_y in candidate_limits(span_y, pitch, overhang.1) {
        let Some(&limit_x) = widths
            .iter()
            .rev()
            .find(|&&w| printer.bed_fit_mm(w, limit_y).fits)
        else {
            continue;
        };
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
        assert!(
            printer
                .bed_fit_mm(
                    widest_chunk_mm(span_x, &x, pitch, overhang.0),
                    widest_chunk_mm(span_y, &y, pitch, overhang.1)
                )
                .fits,
            "a plan bounded by {limit_x} x {limit_y} mm has a chunk that does not print, so the \
             two limits do not stand for the bed they came from"
        );
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
        assert!(
            !r.rotated && r.tilt_deg.is_none(),
            "a bin that lies flat needs no placement"
        );
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
        let bed = PrinterProfile { name: "170 square", bed_width: 170, bed_depth: 170 };
        let c: Vec<GridCell> = (0..4).map(|x| GridCell { x, y: 0 }).collect();
        let bare = compute_staggered_split_lines(&c, bed, GRID_PITCH, (0.0, 0.0), &[])
            .expect("four 42 mm cells are 168 mm, which prints on a 170 mm bed");
        assert!(bare.is_empty(), "168 mm prints whole, so nothing is cut");
        let flanged = compute_staggered_split_lines(&c, bed, GRID_PITCH, (40.0, 0.0), &[])
            .expect("half the flange at each end still divides somewhere");
        assert_eq!(
            flanged.len(),
            1,
            "168 mm of cells plus a 40 mm flange is 208 mm, which prints at no angle on a 170              mm bed"
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
    fn even_cuts_divides_a_run_as_evenly_as_whole_cells_allow() {
        let cuts = even_cuts(10, 4);
        assert_eq!(cuts.len(), 3);
        let mut prev = 0;
        for &cut in &cuts {
            assert!(cut - prev <= widest_chunk(10, 4), "chunk {prev}..{cut} is too wide");
            prev = cut;
        }
        assert!(10 - prev <= widest_chunk(10, 4));
    }

    /// The three placements, told apart. A body that lies flat is neither
    /// turned nor tilted, one that only clears the bed the other way round is
    /// turned and not tilted, and only a body neither axis-aligned placement
    /// takes is reported as diagonal.
    #[test]
    fn a_fit_names_the_one_placement_the_body_needs() {
        let bed = PrinterProfile { name: "oblong", bed_width: 250, bed_depth: 150 };
        let flat = bed.bed_fit_mm(200.0, 100.0);
        assert!(flat.fits && !flat.rotated && flat.tilt_deg.is_none());
        let turned = bed.bed_fit_mm(100.0, 200.0);
        assert!(turned.fits && turned.rotated && turned.tilt_deg.is_none());
        let tilted = bed.bed_fit_mm(260.0, 30.0);
        assert!(
            tilted.fits && !tilted.rotated && tilted.tilt_deg.is_some(),
            "260 x 30 mm clears neither 250 x 150 nor 150 x 250, and lies between the two"
        );
        assert!(
            !bed.bed_fit_mm(260.0, 30.0 + 40.0).fits,
            "and 40 mm deeper it does not lie anywhere"
        );
    }

    /// The angle is reported, not just the fact of one, and the ladder tries the
    /// round angles first: an 80 x 280 mm bin on a 256 mm bed has a window of
    /// 44.4 to 45.6 degrees, so it is reported as **45**, which is what a
    /// slicer's rotation box wants typed into it.
    #[test]
    fn a_tilted_body_is_told_the_roundest_angle_that_takes_it() {
        let bed = PrinterProfile { name: "256 square", bed_width: 256, bed_depth: 256 };
        assert_eq!(bed.bed_fit_mm(80.0, 280.0).tilt_deg, Some(45.0));
        let oblong = PrinterProfile { name: "oblong", bed_width: 250, bed_depth: 150 };
        let awkward = oblong
            .bed_fit_mm(260.0, 30.0)
            .tilt_deg
            .expect("260 x 30 mm lies across a 250 x 150 bed");
        assert!(
            (23.797..=28.386).contains(&awkward),
            "{awkward} degrees is outside the window that body has"
        );
    }

    /// Every angle the window names really does put the body inside the bed, and
    /// every angle outside it really does not -- the window is the whole answer,
    /// so nothing downstream has to re-derive it.
    #[test]
    fn the_window_is_exactly_the_angles_that_put_a_body_on_the_bed() {
        let (a, b) = (250.0, 150.0);
        for &(w, d) in &[(260.0, 30.0), (280.0, 80.0), (100.0, 100.0), (400.0, 10.0)] {
            let runs = tilt_window(w, d, a, b);
            let inside = |t: f64| runs.iter().any(|&(lo, hi)| lo <= t && t <= hi);
            for i in 0..=900 {
                let t = QUARTER_TURN * f64::from(i) / 900.0;
                let (p, q) = (w.max(d), w.min(d));
                let (bw, bd) = (p * t.cos() + q * t.sin(), p * t.sin() + q * t.cos());
                let on_bed = bw <= a && bd <= b;
                if inside(t) {
                    assert!(on_bed, "{w} x {d} mm is said to fit at {t} rad and does not");
                } else if on_bed {
                    let edge = runs.iter().any(|&(lo, hi)| {
                        (t - lo).abs() < 1e-3 || (t - hi).abs() < 1e-3
                    });
                    assert!(edge, "{w} x {d} mm fits at {t} rad and the window misses it");
                }
            }
        }
    }

    /// The criterion itself, on the bed the worked drawer is fitted for. A 320 mm
    /// strip 40 mm wide lies across it and a 300 mm one 100 mm wide does not, so
    /// the rule is not "anything shorter than the diagonal".
    #[test]
    fn a_long_thin_body_lies_across_a_bed_a_shorter_wide_one_cannot() {
        let bed = PrinterProfile { name: "256 square", bed_width: 256, bed_depth: 256 };
        let strip = bed.bed_fit_mm(320.0, 40.0);
        assert!(
            strip.fits && strip.tilt_deg.is_some(),
            "320 x 40 mm lies across a 256 mm square"
        );
        assert!(!bed.bed_fit_mm(300.0, 100.0).fits, "300 x 100 mm does not, at any angle");
        assert!(!bed.bed_fit_mm(370.0, 1.0).fits, "nothing reaches past the bed's own diagonal");
    }

    /// The property the whole per-axis planning rests on: a body that prints
    /// still prints once either of its sides is made smaller. Without it, capping
    /// the depth at `D` and the width at the widest that prints against `D` would
    /// not bound the chunks in between.
    #[test]
    fn a_body_that_prints_still_prints_when_it_shrinks() {
        let bed = PrinterProfile { name: "256 square", bed_width: 256, bed_depth: 256 };
        let mut w = 5.0;
        while w < 400.0 {
            let mut d = 5.0;
            while d < 400.0 {
                if bed.bed_fit_mm(w, d).fits {
                    assert!(
                        bed.bed_fit_mm(w - 1.0, d).fits && bed.bed_fit_mm(w, d - 1.0).fits,
                        "a {w} x {d} mm body prints, so a smaller one does"
                    );
                }
                d += 7.0;
            }
            w += 7.0;
        }
    }

    /// A body deeper than the bed is not refused out of hand -- it is the same
    /// question turned round, and a narrow enough one still lies across the
    /// diagonal.
    #[test]
    fn a_body_longer_than_the_bed_is_the_same_question_either_way_round() {
        let bed = PrinterProfile { name: "256 square", bed_width: 256, bed_depth: 256 };
        let deep = bed.bed_fit_mm(40.0, 320.0);
        let long = bed.bed_fit_mm(320.0, 40.0);
        assert!(deep.fits && deep.tilt_deg.is_some(), "40 x 320 mm lies across a 256 square");
        assert_eq!((deep.fits, deep.tilt_deg), (long.fits, long.tilt_deg));
    }

    /// The point of the whole change, as a split plan: a seven-cell strip is
    /// 280 mm long against a 246 mm bed and would be cut in two if it had to lie
    /// square, but it drops in at an angle whole.
    #[test]
    fn a_bin_that_only_prints_at_an_angle_is_not_cut() {
        let bed = PrinterProfile { name: "256 square", bed_width: 256, bed_depth: 256 };
        let c: Vec<GridCell> = (0..7).map(|x| GridCell { x, y: 0 }).collect();
        let fit = check_bed_fit(&c, bed, 40.0);
        assert!(fit.fits && fit.tilt_deg.is_some(), "280 x 40 mm needs the diagonal");
        assert!(
            compute_auto_split_lines(&c, bed, 40.0).is_empty(),
            "a bin that prints whole is not cut"
        );
        assert!(
            compute_staggered_split_lines(&c, bed, 40.0, (0.0, 0.0), &[])
                .expect("a plate that prints whole has a plan")
                .is_empty(),
            "and neither is the plate over it"
        );
    }

    /// A chunk of a cut body is judged the same way, so the planner divides a
    /// body only as far as the bed really requires: thirteen cells at 40 mm is
    /// 520 mm, too long for any angle, and comes back as a 280 mm chunk laid
    /// diagonally beside a 240 mm one rather than the three that a 246 mm
    /// axis-aligned cap asks for.
    #[test]
    fn a_chunk_that_only_prints_at_an_angle_is_not_divided_further() {
        let bed = PrinterProfile { name: "256 square", bed_width: 256, bed_depth: 256 };
        let c: Vec<GridCell> = (0..13).map(|x| GridCell { x, y: 0 }).collect();
        let lines = compute_auto_split_lines(&c, bed, 40.0);
        let pieces = partition_cells(&c, &lines);
        assert_eq!(pieces.len(), 2, "cut at {lines:?}");
        assert!(
            pieces
                .iter()
                .any(|p| check_bed_fit(&p.cells, bed, 40.0).tilt_deg.is_some()),
            "one of the two chunks is the 280 mm one, which prints only at an angle"
        );
        for p in &pieces {
            assert!(
                check_bed_fit(&p.cells, bed, 40.0).fits,
                "the piece {:?} prints",
                p.cells
            );
        }
    }
}
