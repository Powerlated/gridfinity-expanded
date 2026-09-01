//! Rectilinear plan geometry: the axis-aligned boxes an object is described by,
//! and the boundary runs a set of them encloses.
//!
//! Everything here is millimetres in the drawer's own plane, quantised to
//! `1/QUANTUM` so that two values a caller computed by different routes compare
//! equal. `Rect` is one box; a *part list* is the boxes of one object, which the
//! packer treats as a single rigid shape -- `parts_bounds`, `translate_parts`,
//! `normalize_parts`, `inflate_parts` and `rotate_parts` move and grow that
//! shape, and `parts_key` names it so two spellings of the same shape are
//! recognised as one. `RectGrid` is the coordinate lattice a part list induces,
//! which is what makes `union_area` measure the union rather than the sum and
//! `parts_connected` decide edge-connectivity without sampling. `Segment` and
//! `boundary_segments` trace the outside of a part list -- never the seam between
//! two of its own boxes -- and `merge_segments` collapses collinear and
//! duplicated runs on one line into one span.

use gridfinity_model::layout::Orientation;

/// Reciprocal of the coordinate quantum: every millimetre value this module
/// stores is a multiple of `1.0 / QUANTUM`.
pub const QUANTUM: f64 = 1e4;

/// The four quarter turns a claim may be placed at, as degrees anticlockwise in
/// the drawer plane.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(into = "u16", try_from = "u16"))]
pub enum Rotation {
    #[default]
    Deg0,
    Deg90,
    Deg180,
    Deg270,
}

/// Every rotation, in the order the packer tries them.
pub const ROTATIONS: [Rotation; 4] = [
    Rotation::Deg0,
    Rotation::Deg90,
    Rotation::Deg180,
    Rotation::Deg270,
];

impl From<Rotation> for u16 {
    /// The rotation as the degree count the wire format and the report both use.
    fn from(r: Rotation) -> u16 {
        match r {
            Rotation::Deg0 => 0,
            Rotation::Deg90 => 90,
            Rotation::Deg180 => 180,
            Rotation::Deg270 => 270,
        }
    }
}

impl TryFrom<u16> for Rotation {
    type Error = String;

    /// A degree count back to the quarter turn it names, or an error naming the
    /// value for anything that is not one of the four quarter turns.
    fn try_from(deg: u16) -> Result<Rotation, String> {
        match deg {
            0 => Ok(Rotation::Deg0),
            90 => Ok(Rotation::Deg90),
            180 => Ok(Rotation::Deg180),
            270 => Ok(Rotation::Deg270),
            other => Err(format!(
                "a placement is rotated by a quarter turn, so {other} degrees is not one of \
                 0/90/180/270"
            )),
        }
    }
}

impl Rotation {
    /// Whether this turn exchanges the two axes, so that a shape's bounding
    /// width and depth swap under it.
    pub fn swaps_axes(self) -> bool {
        self == Rotation::Deg90 || self == Rotation::Deg270
    }

    /// The turn as its degree count, for a report that prints it.
    pub fn degrees(self) -> u16 {
        u16::from(self)
    }
}

/// One axis-aligned box in the drawer plane: its minimum corner and its extent
/// along each axis, in millimetres.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Rect {
    pub x: f64,
    pub y: f64,
    pub width: f64,
    pub depth: f64,
}

impl Rect {
    /// A box from its minimum corner and extents, with every coordinate
    /// quantised.
    pub fn new(x: f64, y: f64, width: f64, depth: f64) -> Rect {
        Rect {
            x: quantize(x),
            y: quantize(y),
            width: quantize(width),
            depth: quantize(depth),
        }
    }

    /// The box's maximum x, quantised, so that a box's far edge and the near
    /// edge of the box abutting it are the same number rather than an ulp apart.
    pub fn right(&self) -> f64 {
        quantize(self.x + self.width)
    }

    /// The box's maximum y, quantised, for the same reason as `right`.
    pub fn bottom(&self) -> f64 {
        quantize(self.y + self.depth)
    }

    /// The box's area in mm², non-negative for any box with non-negative
    /// extents.
    pub fn area(&self) -> f64 {
        self.width * self.depth
    }
}

/// A millimetre value snapped to the module's coordinate quantum, so that two
/// values reached by different arithmetic compare equal when they name the same
/// point.
pub fn quantize(value: f64) -> f64 {
    (value * QUANTUM).round() / QUANTUM
}

/// Whether two boxes share interior area. Boxes that only touch along an edge or
/// at a corner do not overlap, which is what lets two claims abut on a divider
/// centreline.
pub fn rects_overlap(a: &Rect, b: &Rect) -> bool {
    a.x < b.right() && b.x < a.right() && a.y < b.bottom() && b.y < a.bottom()
}

/// Whether `inner` lies wholly within `outer`, touching allowed on any side.
pub fn rect_contains(outer: &Rect, inner: &Rect) -> bool {
    inner.x >= outer.x
        && inner.y >= outer.y
        && inner.right() <= outer.right()
        && inner.bottom() <= outer.bottom()
}

/// Whether every point of `inner` lies in the union of `cover`, touching allowed
/// on any side.
///
/// A union rather than a single box, because the region a claim must stay inside
/// is a bin's cavity and a bin's cells are a polyomino. The test is exact: the
/// two lists' own edges cut `inner` into a lattice whose every cell lies wholly
/// inside a member of `cover` or wholly outside all of them, so asking each
/// cell's centre answers for the cell.
pub fn rect_covered_by(inner: &Rect, cover: &[Rect]) -> bool {
    if inner.width <= 0.0 || inner.depth <= 0.0 {
        return true;
    }
    let cut = |lo: f64, hi: f64, edges: Vec<f64>| -> Vec<f64> {
        let mut out: Vec<f64> = edges
            .into_iter()
            .map(quantize)
            .filter(|v| *v > lo && *v < hi)
            .collect();
        out.push(lo);
        out.push(hi);
        out.sort_by(f64::total_cmp);
        out.dedup();
        out
    };
    let xs = cut(
        quantize(inner.x),
        inner.right(),
        cover.iter().flat_map(|r| [r.x, r.right()]).collect(),
    );
    let ys = cut(
        quantize(inner.y),
        inner.bottom(),
        cover.iter().flat_map(|r| [r.y, r.bottom()]).collect(),
    );
    assert!(
        xs.len() >= 2 && ys.len() >= 2,
        "a box of positive extent cuts into at least one lattice cell, but {inner:?} cut into none"
    );
    for row in 0..ys.len() - 1 {
        for col in 0..xs.len() - 1 {
            let mid = (
                (xs[col] + xs[col + 1]) / 2.0,
                (ys[row] + ys[row + 1]) / 2.0,
            );
            let held = cover.iter().any(|r| {
                r.x <= mid.0 && mid.0 <= r.right() && r.y <= mid.1 && mid.1 <= r.bottom()
            });
            if !held {
                return false;
            }
        }
    }
    true
}

/// The smallest box containing every part, or a zero box for an empty part list.
pub fn parts_bounds(parts: &[Rect]) -> Rect {
    if parts.is_empty() {
        return Rect::default();
    }
    let x = parts.iter().map(|p| p.x).fold(f64::INFINITY, f64::min);
    let y = parts.iter().map(|p| p.y).fold(f64::INFINITY, f64::min);
    let right = parts
        .iter()
        .map(Rect::right)
        .fold(f64::NEG_INFINITY, f64::max);
    let bottom = parts
        .iter()
        .map(Rect::bottom)
        .fold(f64::NEG_INFINITY, f64::max);
    Rect {
        x,
        y,
        width: right - x,
        depth: bottom - y,
    }
}

/// Every part moved by the same offset, extents unchanged.
pub fn translate_parts(parts: &[Rect], dx: f64, dy: f64) -> Vec<Rect> {
    parts
        .iter()
        .map(|p| Rect {
            x: quantize(p.x + dx),
            y: quantize(p.y + dy),
            ..*p
        })
        .collect()
}

/// The same shape moved so its bounding box's minimum corner is the origin.
pub fn normalize_parts(parts: &[Rect]) -> Vec<Rect> {
    let bounds = parts_bounds(parts);
    translate_parts(parts, -bounds.x, -bounds.y)
}

/// Every part grown by `margin` on all four sides, so the shape's bounding box
/// grows by `2 * margin` in each extent. A negative margin shrinks it, which is
/// how a claim is turned back into the object inside it.
pub fn inflate_parts(parts: &[Rect], margin: f64) -> Vec<Rect> {
    parts
        .iter()
        .map(|p| {
            Rect::new(
                p.x - margin,
                p.y - margin,
                p.width + margin * 2.0,
                p.depth + margin * 2.0,
            )
        })
        .collect()
}

/// The shape turned by `rotation` and normalised back to the first quadrant, so
/// the result's bounding box starts at the origin and its extents are the
/// original's, swapped for a quarter turn.
pub fn rotate_parts(parts: &[Rect], rotation: Rotation) -> Vec<Rect> {
    let turned: Vec<Rect> = parts
        .iter()
        .map(|p| match rotation {
            Rotation::Deg0 => *p,
            Rotation::Deg90 => Rect {
                x: -p.bottom(),
                y: p.x,
                width: p.depth,
                depth: p.width,
            },
            Rotation::Deg180 => Rect {
                x: -p.right(),
                y: -p.bottom(),
                width: p.width,
                depth: p.depth,
            },
            Rotation::Deg270 => Rect {
                x: p.y,
                y: -p.right(),
                width: p.depth,
                depth: p.width,
            },
        })
        .collect();
    normalize_parts(&turned)
}

/// A canonical name for a shape: the sorted list of its quantised parts, equal
/// for two part lists exactly when they cover the same boxes in the same place.
pub fn parts_key(parts: &[Rect]) -> String {
    let mut keys: Vec<String> = parts
        .iter()
        .map(|p| {
            format!(
                "{:.4},{:.4},{:.4},{:.4}",
                quantize(p.x),
                quantize(p.y),
                quantize(p.width),
                quantize(p.depth)
            )
        })
        .collect();
    keys.sort();
    keys.join("|")
}

/// The lattice a part list induces: every distinct part edge on each axis, and
/// for each cell of the resulting grid whether some part covers it.
pub struct RectGrid {
    pub xs: Vec<f64>,
    pub ys: Vec<f64>,
    pub filled: Vec<bool>,
}

/// Every value quantised, deduplicated and sorted ascending.
fn unique_sorted(values: impl Iterator<Item = f64>) -> Vec<f64> {
    let mut out: Vec<f64> = values.map(quantize).collect();
    out.sort_by(f64::total_cmp);
    out.dedup();
    out
}

/// The part list's lattice. A cell is filled when one part covers it entirely,
/// which is exact because every part edge is itself a lattice line.
pub fn rect_grid(parts: &[Rect]) -> RectGrid {
    let xs = unique_sorted(parts.iter().flat_map(|p| [p.x, p.right()]));
    let ys = unique_sorted(parts.iter().flat_map(|p| [p.y, p.bottom()]));
    let cols = xs.len().saturating_sub(1);
    let rows = ys.len().saturating_sub(1);
    let mut filled = vec![false; cols * rows];
    for row in 0..rows {
        for col in 0..cols {
            filled[row * cols + col] = parts.iter().any(|p| {
                quantize(p.x) <= xs[col]
                    && xs[col + 1] <= p.right()
                    && quantize(p.y) <= ys[row]
                    && ys[row + 1] <= p.bottom()
            });
        }
    }
    RectGrid { xs, ys, filled }
}

impl RectGrid {
    /// How many lattice columns the grid has: one fewer than its x lines.
    pub fn cols(&self) -> usize {
        self.xs.len().saturating_sub(1)
    }

    /// How many lattice rows the grid has: one fewer than its y lines.
    pub fn rows(&self) -> usize {
        self.ys.len().saturating_sub(1)
    }

    /// Whether the cell at these lattice indices is covered, `false` for any
    /// index outside the grid so that a boundary walk needs no special case.
    pub fn filled_at(&self, col: isize, row: isize) -> bool {
        if col < 0 || row < 0 || col >= self.cols() as isize || row >= self.rows() as isize {
            return false;
        }
        self.filled[row as usize * self.cols() + col as usize]
    }
}

/// The area in mm² the parts cover between them, counting overlapped area once.
pub fn union_area(parts: &[Rect]) -> f64 {
    let grid = rect_grid(parts);
    let mut area = 0.0;
    for row in 0..grid.rows() {
        for col in 0..grid.cols() {
            if !grid.filled_at(col as isize, row as isize) {
                continue;
            }
            area += (grid.xs[col + 1] - grid.xs[col]) * (grid.ys[row + 1] - grid.ys[row]);
        }
    }
    area
}

/// Whether the parts form one edge-connected region. Boxes meeting only at a
/// corner are not connected, and neither is a part list with no area at all.
pub fn parts_connected(parts: &[Rect]) -> bool {
    if parts.len() <= 1 {
        return true;
    }
    let grid = rect_grid(parts);
    let cols = grid.cols();
    let total = grid.filled.iter().filter(|f| **f).count();
    if total == 0 {
        return false;
    }
    let start = grid
        .filled
        .iter()
        .position(|f| *f)
        .expect("a grid with a filled cell has a first filled cell");
    let mut seen = vec![false; grid.filled.len()];
    seen[start] = true;
    let mut queue = vec![start];
    let mut reached = 1;
    while let Some(index) = queue.pop() {
        let col = (index % cols) as isize;
        let row = (index / cols) as isize;
        for (dx, dy) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
            let (nc, nr) = (col + dx, row + dy);
            if !grid.filled_at(nc, nr) {
                continue;
            }
            let next = nr as usize * cols + nc as usize;
            if seen[next] {
                continue;
            }
            seen[next] = true;
            reached += 1;
            queue.push(next);
        }
    }
    assert!(
        reached <= total,
        "the flood fill reached {reached} of {total} filled cells, so it counted a cell twice or \
         walked into one no part covers"
    );
    reached == total
}

/// One straight run on the boundary of a part list: the line it lies on and the
/// span of that line which is boundary. A `V` run lies on `x == coordinate` and
/// spans `start..end` in y; an `H` run lies on `y == coordinate` and spans in x.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Segment {
    pub orientation: Orientation,
    pub coordinate: f64,
    pub start: f64,
    pub end: f64,
}

impl Segment {
    /// The run's length in millimetres, non-negative for any run this module
    /// produces.
    pub fn length(&self) -> f64 {
        self.end - self.start
    }
}

/// Every lattice edge where the part list changes between covered and uncovered:
/// the outside of the shape and the boundary of any hole in it, but never the
/// seam between two of its own boxes.
pub fn boundary_segments(parts: &[Rect]) -> Vec<Segment> {
    let grid = rect_grid(parts);
    let (cols, rows) = (grid.cols() as isize, grid.rows() as isize);
    let mut out = Vec::new();
    for col in 0..=cols {
        for row in 0..rows {
            if grid.filled_at(col - 1, row) == grid.filled_at(col, row) {
                continue;
            }
            out.push(Segment {
                orientation: Orientation::V,
                coordinate: grid.xs[col as usize],
                start: grid.ys[row as usize],
                end: grid.ys[row as usize + 1],
            });
        }
    }
    for row in 0..=rows {
        for col in 0..cols {
            if grid.filled_at(col, row - 1) == grid.filled_at(col, row) {
                continue;
            }
            out.push(Segment {
                orientation: Orientation::H,
                coordinate: grid.ys[row as usize],
                start: grid.xs[col as usize],
                end: grid.xs[col as usize + 1],
            });
        }
    }
    out
}

/// The runs ordered by orientation, then line, then position along the line --
/// the order `merge_segments` returns and the one two callers can compare in.
pub fn sort_segments(segments: &mut [Segment]) {
    segments.sort_by(|a, b| {
        a.orientation
            .cmp(&b.orientation)
            .then(a.coordinate.total_cmp(&b.coordinate))
            .then(a.start.total_cmp(&b.start))
            .then(a.end.total_cmp(&b.end))
    });
}

/// The runs with every collinear or duplicated overlap on one line collapsed
/// into a single span, sorted. Two runs that merely touch end to end merge, so
/// two compartments meeting along one line share one divider rather than
/// stacking two.
pub fn merge_segments(segments: &[Segment]) -> Vec<Segment> {
    let mut sorted: Vec<Segment> = segments.to_vec();
    sort_segments(&mut sorted);
    let mut merged: Vec<Segment> = Vec::new();
    for segment in sorted {
        match merged.last_mut() {
            Some(run)
                if run.orientation == segment.orientation
                    && run.coordinate == segment.coordinate
                    && segment.start <= run.end =>
            {
                run.end = run.end.max(segment.end);
            }
            _ => merged.push(segment),
        }
    }
    for pair in merged.windows(2) {
        assert!(
            pair[0].orientation != pair[1].orientation
                || pair[0].coordinate != pair[1].coordinate
                || pair[0].end < pair[1].start,
            "two merged runs on the same line still meet: {:?} {} spans {}..{} and {}..{}",
            pair[0].orientation,
            pair[0].coordinate,
            pair[0].start,
            pair[0].end,
            pair[1].start,
            pair[1].end
        );
    }
    merged
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rect(x: f64, y: f64, width: f64, depth: f64) -> Rect {
        Rect::new(x, y, width, depth)
    }

    fn square() -> Vec<Rect> {
        vec![rect(0.0, 0.0, 20.0, 20.0)]
    }

    fn elbow() -> Vec<Rect> {
        vec![rect(0.0, 0.0, 10.0, 30.0), rect(0.0, 20.0, 40.0, 10.0)]
    }

    #[test]
    fn returns_to_the_original_shape_after_four_quarter_turns() {
        for parts in [square(), elbow()] {
            let mut turned = normalize_parts(&parts);
            for _ in 0..4 {
                turned = rotate_parts(&turned, Rotation::Deg90);
            }
            assert_eq!(parts_key(&turned), parts_key(&normalize_parts(&parts)));
        }
    }

    #[test]
    fn preserves_area_under_every_rotation_and_swaps_the_bounds_of_a_quarter_turn() {
        let original = parts_bounds(&elbow());
        for rotation in ROTATIONS {
            let turned = rotate_parts(&elbow(), rotation);
            assert!((union_area(&turned) - union_area(&elbow())).abs() < 1e-9);
            let bounds = parts_bounds(&turned);
            let (want_w, want_d) = if rotation.swaps_axes() {
                (original.depth, original.width)
            } else {
                (original.width, original.depth)
            };
            assert!((bounds.width - want_w).abs() < 1e-9, "{rotation:?} width");
            assert!((bounds.depth - want_d).abs() < 1e-9, "{rotation:?} depth");
        }
    }

    #[test]
    fn measures_the_union_rather_than_the_sum_of_overlapping_boxes() {
        let overlapping = vec![rect(0.0, 0.0, 10.0, 10.0), rect(5.0, 0.0, 10.0, 10.0)];
        assert!((union_area(&overlapping) - 150.0).abs() < 1e-9);
        assert!((union_area(&elbow()) - (10.0 * 30.0 + 30.0 * 10.0)).abs() < 1e-9);
    }

    #[test]
    fn grows_a_shape_by_the_same_margin_on_every_side() {
        let grown = inflate_parts(&square(), 2.0);
        assert_eq!(parts_bounds(&grown), rect(-2.0, -2.0, 24.0, 24.0));
        assert_eq!(
            parts_bounds(&inflate_parts(&grown, -2.0)),
            parts_bounds(&square())
        );
    }

    #[test]
    fn rejects_boxes_that_only_touch_at_a_corner() {
        assert!(parts_connected(&elbow()));
        assert!(parts_connected(&[
            rect(0.0, 0.0, 10.0, 10.0),
            rect(10.0, 0.0, 10.0, 10.0)
        ]));
        assert!(!parts_connected(&[
            rect(0.0, 0.0, 10.0, 10.0),
            rect(10.0, 10.0, 10.0, 10.0)
        ]));
        assert!(!parts_connected(&[
            rect(0.0, 0.0, 10.0, 10.0),
            rect(30.0, 0.0, 10.0, 10.0)
        ]));
    }

    #[test]
    fn traces_only_the_outside_of_a_shape_never_the_seam_between_its_own_boxes() {
        let merged = merge_segments(&boundary_segments(&[
            rect(0.0, 0.0, 10.0, 10.0),
            rect(10.0, 0.0, 10.0, 10.0),
        ]));
        let vertical: Vec<f64> = merged
            .iter()
            .filter(|s| s.orientation == Orientation::V)
            .map(|s| s.coordinate)
            .collect();
        let horizontal: Vec<f64> = merged
            .iter()
            .filter(|s| s.orientation == Orientation::H)
            .map(|s| s.coordinate)
            .collect();
        assert_eq!(vertical, vec![0.0, 20.0]);
        assert_eq!(horizontal, vec![0.0, 10.0]);
    }

    #[test]
    fn merges_collinear_and_duplicated_runs_into_one_span() {
        let run = |start: f64, end: f64| Segment {
            orientation: Orientation::V,
            coordinate: 5.0,
            start,
            end,
        };
        assert_eq!(
            merge_segments(&[run(0.0, 10.0), run(10.0, 20.0), run(0.0, 10.0), run(40.0, 50.0)]),
            vec![run(0.0, 20.0), run(40.0, 50.0)]
        );
    }
}
