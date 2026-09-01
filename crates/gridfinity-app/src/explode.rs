//! How a split bin is taken apart in the preview.
//!
//! The kernel abuts its carved pieces exactly, so a split bin drawn as it is
//! built is indistinguishable from an unsplit one. `Explosion` is the one place
//! that decides how far each piece stands off, and it decides it per *band*
//! rather than per piece: `partition_cells` labels every piece with the chunk
//! index it falls in along each axis, and every piece sharing an index moves the
//! same distance, so each cut opens by exactly `SPLIT_APART_MM` and no other
//! seam opens at all. `shift` is that offset, and `clip` cuts an object's box on
//! the same lines so the box travels with the piece it lies in rather than
//! straddling the gap.
//!
//! A bin and the baseplate under it each get their own `Explosion`, because they
//! are cut on their own lines -- `optimize` staggers the plate's off the bin's --
//! so the two bodies open their gaps in different places and a piece of each
//! visibly laps a seam of the other. `of` is the bin's way in, `new` the plain
//! cells-and-lines one the plate takes.

use gridfinity_model::gridfinity::LogicalBin;
use gridfinity_brep::math::Vec3 as KernelVec3;
use gridfinity_model::layout::{GridCell, Piece, SplitLine, partition_cells};
use glam::Vec3;

/// How far apart adjacent pieces of a split bin stand in the preview, in
/// millimetres. Far enough that every cut reads as a gap at a glance, near
/// enough that the bin still reads as one part.
pub const SPLIT_APART_MM: f32 = 3.0;

/// One band of a split bin along one axis: the millimetres its cells span, with
/// the outermost band open at its outer end so anything reaching past the bin's
/// outline still belongs to it.
#[derive(Clone, Copy, Debug)]
struct Band {
    lo: f64,
    hi: f64,
}

/// One bin taken apart: its pieces, and the band each piece belongs to along
/// each axis.
///
/// Built from the bin's own cells and split lines, so the pieces are the ones
/// the export writes and the bands are the chunks `partition_cells` sorted them
/// into. A bin with no split lines has one band per axis and every offset is
/// zero.
pub struct Explosion {
    pieces: Vec<Piece>,
    x: Vec<Band>,
    y: Vec<Band>,
    gap: f32,
}

/// The bands along one axis: for each chunk index in `keys`, the millimetres the
/// cells with that index span, with the first band open below and the last open
/// above.
///
/// `cell_span` gives a chunk's cell range along the axis; the range is turned
/// into millimetres by the grid pitch, which is what puts a band in the same
/// coordinates as the solid and the object boxes.
fn bands(mut spans: Vec<(i32, i32)>, pitch: f64) -> Vec<Band> {
    assert!(!spans.is_empty(), "a bin with cells has at least one band along every axis");
    assert!(pitch > 0.0, "a grid pitch is a positive number of millimetres, not {pitch}");
    let last = spans.len() - 1;
    spans
        .drain(..)
        .enumerate()
        .map(|(i, (first_cell, last_cell))| {
            assert!(
                first_cell <= last_cell,
                "a band runs from its first cell to its last, but {first_cell} is past {last_cell}"
            );
            Band {
                lo: if i == 0 { f64::NEG_INFINITY } else { first_cell as f64 * pitch },
                hi: if i == last {
                    f64::INFINITY
                } else {
                    (last_cell + 1) as f64 * pitch
                },
            }
        })
        .collect()
}

/// The centred offset of index `i` of `count`, in bands: half a gap either side
/// of the middle for an even count, whole gaps from the middle for an odd one.
/// A single band is centred on zero, which is why an axis with no cut moves
/// nothing.
fn band_offset(i: i32, count: usize, gap: f32) -> f32 {
    assert!(count > 0, "an axis of a bin with cells has at least one band, not {count}");
    assert!(
        i >= 0 && (i as usize) < count,
        "band {i} is not one of the {count} bands along this axis"
    );
    assert!(gap >= 0.0, "pieces stand apart or abut, so a gap is not {gap} mm");
    (i as f32 - (count - 1) as f32 / 2.0) * gap
}

impl Explosion {
    /// The bin's pieces and bands, read off its cells and its own split lines.
    pub fn of(bin: &LogicalBin, pitch: f64) -> Explosion {
        Explosion::new(&bin.cells, &bin.split_lines, pitch)
    }

    /// The same, for a body that is a cell set and a set of lines rather than a
    /// `LogicalBin`: a baseplate, whose cells are every bin's and whose lines
    /// are its own, staggered off the bin's so the two bodies part on different
    /// planes. Each body explodes along its own bands, which is what makes a
    /// piece of one visibly lap a seam of the other.
    pub fn new(cells: &[GridCell], splits: &[SplitLine], pitch: f64) -> Explosion {
        let pieces = partition_cells(cells, splits);
        assert!(
            !pieces.is_empty() || cells.is_empty(),
            "a bin with cells partitions into at least one piece"
        );
        let spans = |index: fn(&Piece) -> i32, along: fn(&GridCell) -> i32| {
            let mut keys: Vec<i32> = pieces.iter().map(index).collect();
            keys.sort_unstable();
            keys.dedup();
            keys.iter()
                .map(|&k| {
                    let along_axis = pieces
                        .iter()
                        .filter(|p| index(p) == k)
                        .flat_map(|p| p.cells.iter().map(along));
                    along_axis.fold((i32::MAX, i32::MIN), |(lo, hi), c| (lo.min(c), hi.max(c)))
                })
                .collect::<Vec<_>>()
        };
        let x = bands(spans(|p| p.col, |c| c.x), pitch);
        let y = bands(spans(|p| p.row, |c| c.y), pitch);
        Explosion { pieces, x, y, gap: SPLIT_APART_MM }
    }

    /// The same pieces and bands with every offset zero: the body shown as it
    /// is printed, its pieces abutting on the cut planes exactly as the kernel
    /// carved them.
    ///
    /// This is what the viewport's *Show gaps* toggle turns off, and it is the
    /// web viewer at explode 0. The pieces are still carved separately -- a
    /// collapsed explosion is not the unsplit solid, it is the split one closed
    /// up -- so what is on screen is still what the files hold.
    pub fn collapsed(mut self) -> Explosion {
        self.gap = 0.0;
        self
    }

    /// The pieces the bin partitions into, in the order `partition_cells`
    /// returns them.
    pub fn pieces(&self) -> &[Piece] {
        &self.pieces
    }

    /// Whether the bin is cut at all: one band along both axes is one piece, and
    /// nothing to take apart.
    pub fn is_split(&self) -> bool {
        self.x.len() > 1 || self.y.len() > 1
    }

    /// The millimetres a piece in band `(col, row)` is displaced by: its band's
    /// centred offset along each axis, and nothing in z. Adjacent bands differ
    /// by exactly `SPLIT_APART_MM`, so every cut opens by the same gap however
    /// many pieces the bin has and wherever in it they sit.
    pub fn shift(&self, col: i32, row: i32) -> Vec3 {
        Vec3::new(
            band_offset(col, self.x.len(), self.gap),
            band_offset(row, self.y.len(), self.gap),
            0.0,
        )
    }

    /// The millimetres the band containing the point `(x, y)` is displaced by.
    ///
    /// This is `shift` addressed by position rather than by piece, which is what
    /// a label on a *whole* item needs: the item is named once, at a point of
    /// its own, and has to travel with the piece that point stands on rather
    /// than being drawn where nothing is any more. The outermost bands are open
    /// outwards, so every point of the plane lands in exactly one band along
    /// each axis.
    pub fn shift_at(&self, x: f64, y: f64) -> Vec3 {
        let band_of = |bands: &[Band], v: f64| {
            bands
                .iter()
                .position(|b| v >= b.lo && v < b.hi)
                .unwrap_or_else(|| panic!("{v} mm lies in no band, but the outer bands are open"))
        };
        Vec3::new(
            band_offset(band_of(&self.x, x) as i32, self.x.len(), self.gap),
            band_offset(band_of(&self.y, y) as i32, self.y.len(), self.gap),
            0.0,
        )
    }

    /// The part of the axis-aligned box `min`..`max` that lies in band
    /// `(col, row)`, or `None` when the two do not overlap in both axes. The z
    /// range is the box's own: a cut is a vertical plane, so it takes nothing
    /// off the height.
    ///
    /// Clipping an object's box this way and displacing each part by `shift` of
    /// the same band is what makes an object cross a cut the way the bin does,
    /// instead of spanning the gap the pieces opened.
    pub fn clip(
        &self,
        col: i32,
        row: i32,
        min: KernelVec3,
        max: KernelVec3,
    ) -> Option<(KernelVec3, KernelVec3)> {
        assert!(
            min.x <= max.x && min.y <= max.y && min.z <= max.z,
            "a box runs from its minimum corner to its maximum, but {min} is not under {max}"
        );
        let x = &self.x[col as usize];
        let y = &self.y[row as usize];
        let (lo_x, hi_x) = (min.x.max(x.lo), max.x.min(x.hi));
        let (lo_y, hi_y) = (min.y.max(y.lo), max.y.min(y.hi));
        if hi_x - lo_x <= 0.0 || hi_y - lo_y <= 0.0 {
            return None;
        }
        Some((KernelVec3::new(lo_x, lo_y, min.z), KernelVec3::new(hi_x, hi_y, max.z)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use gridfinity_model::gridfinity::GRID_PITCH;
    use gridfinity_model::layout::{Axis, GridCell, SplitLine};

    fn bin(width: i32, depth: i32, splits: &[SplitLine]) -> LogicalBin {
        LogicalBin {
            cells: (0..width)
                .flat_map(|x| (0..depth).map(move |y| GridCell { x, y }))
                .collect(),
            split_lines: splits.to_vec(),
            ..Default::default()
        }
    }

    /// The ikea drawer's own partition: 6 x 12 cells cut at x=3, y=4 and y=8.
    fn ikea() -> Explosion {
        Explosion::of(
            &bin(6, 12, &[
                SplitLine { axis: Axis::X, index: 3 },
                SplitLine { axis: Axis::Y, index: 4 },
                SplitLine { axis: Axis::Y, index: 8 },
            ]),
            GRID_PITCH,
        )
    }

    #[test]
    fn an_uncut_bin_moves_nothing() {
        let whole = Explosion::of(&bin(3, 2, &[]), GRID_PITCH);
        assert!(!whole.is_split());
        assert_eq!(whole.pieces().len(), 1);
        assert_eq!(whole.shift(0, 0), Vec3::ZERO);
    }

    #[test]
    fn an_axis_with_no_cut_is_not_moved_along() {
        let across =
            Explosion::of(&bin(4, 3, &[SplitLine { axis: Axis::X, index: 2 }]), GRID_PITCH);
        for piece in across.pieces() {
            let shift = across.shift(piece.col, piece.row);
            assert_eq!(shift.y, 0.0, "nothing is cut along y, so nothing moves along y");
            assert_eq!(shift.x.abs(), SPLIT_APART_MM / 2.0, "the one cut opens by one gap");
        }
    }

    #[test]
    fn every_cut_opens_by_exactly_one_gap() {
        let e = ikea();
        assert_eq!(e.pieces().len(), 6, "two columns of three rows");
        for (a, b) in [((0, 0), (1, 0)), ((0, 0), (0, 1)), ((0, 1), (0, 2)), ((1, 1), (1, 2))] {
            let (from, to) = (e.shift(a.0, a.1), e.shift(b.0, b.1));
            assert!(
                ((to - from).length() - SPLIT_APART_MM).abs() < 1e-6,
                "neighbouring pieces {a:?} and {b:?} open by {}, not {SPLIT_APART_MM}",
                (to - from).length()
            );
        }
    }

    /// *Show gaps* off is the same partition standing still: the pieces are
    /// the ones the export writes, and every one of them is where the kernel
    /// carved it.
    #[test]
    fn a_collapsed_explosion_keeps_the_pieces_and_moves_none_of_them() {
        let apart = ikea();
        let together = ikea().collapsed();
        assert_eq!(
            together.pieces().len(),
            apart.pieces().len(),
            "closing the gaps does not weld the pieces back together"
        );
        assert!(together.is_split(), "a bin cut on three lines is still a bin cut on three lines");
        for piece in together.pieces() {
            assert_eq!(
                together.shift(piece.col, piece.row),
                Vec3::ZERO,
                "piece ({}, {}) abuts its neighbours, so it is not displaced",
                piece.col,
                piece.row
            );
        }
        assert_eq!(
            together.shift_at(3.5 * GRID_PITCH, 6.0 * GRID_PITCH),
            Vec3::ZERO,
            "a point moves with its band, and no band moves"
        );
    }

    /// The defect this replaced a radial displacement to fix: the middle row's
    /// pieces were pushed their whole distance along x while the rows above and
    /// below spent most of theirs on y, so the middle gap read four times wider
    /// than the others.
    #[test]
    fn every_row_is_pushed_the_same_distance_along_x() {
        let e = ikea();
        for col in 0..2 {
            let x: Vec<f32> = (0..3).map(|row| e.shift(col, row).x).collect();
            assert!(
                x.iter().all(|&v| (v - x[0]).abs() < 1e-6),
                "column {col} is displaced by {x:?} along x, which is not one distance"
            );
        }
        assert_eq!(e.shift(0, 1).x, -SPLIT_APART_MM / 2.0);
        assert_eq!(e.shift(1, 1).x, SPLIT_APART_MM / 2.0);
    }

    #[test]
    fn a_box_across_a_cut_clips_into_parts_that_sum_to_it() {
        let e = ikea();
        let pitch = GRID_PITCH;
        let (min, max) =
            (KernelVec3::new(pitch, pitch, 0.0), KernelVec3::new(5.0 * pitch, 2.0 * pitch, 10.0));
        let widths: Vec<f64> = (0..2)
            .filter_map(|col| e.clip(col, 0, min, max))
            .map(|(lo, hi)| hi.x - lo.x)
            .collect();
        assert_eq!(widths.len(), 2, "a box across the x cut lies in both columns");
        assert!(
            (widths.iter().sum::<f64>() - (max.x - min.x)).abs() < 1e-9,
            "the parts {widths:?} do not add up to the box's {} mm",
            max.x - min.x
        );
        for part in (0..2).filter_map(|col| e.clip(col, 0, min, max)) {
            assert_eq!((part.0.z, part.1.z), (min.z, max.z), "a cut takes nothing off the height");
        }
    }

    #[test]
    fn a_box_inside_one_band_survives_whole_and_reaches_no_other() {
        let e = ikea();
        let pitch = GRID_PITCH;
        let (min, max) = (
            KernelVec3::new(0.1 * pitch, 0.1 * pitch, 0.0),
            KernelVec3::new(2.9 * pitch, 3.9 * pitch, 10.0),
        );
        assert_eq!(e.clip(0, 0, min, max), Some((min, max)));
        assert_eq!(e.clip(1, 0, min, max), None);
        assert_eq!(e.clip(0, 1, min, max), None);
    }

    #[test]
    fn a_box_reaching_past_the_outline_still_belongs_to_the_outer_band() {
        let e = ikea();
        let pitch = GRID_PITCH;
        let (min, max) =
            (KernelVec3::new(-10.0, -10.0, 0.0), KernelVec3::new(pitch, pitch, 10.0));
        let (lo, hi) = e.clip(0, 0, min, max).expect("the outer band is open outwards");
        assert_eq!((lo.x, lo.y), (min.x, min.y), "the outer band keeps what hangs off the bin");
        assert_eq!((hi.x, hi.y), (max.x, max.y));
    }
}
