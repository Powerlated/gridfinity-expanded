//! Turning placed claims into the dividers between them.
//!
//! Every claim boundary that is not the packing area's own edge is a divider
//! centreline: `layout_walls` traces each placement's boundary runs, merges the
//! collinear and duplicated ones per line so two abutting compartments share one
//! divider rather than stacking two, drops the runs lying on the area boundary
//! (that is the bin's perimeter wall) and the runs shorter than
//! `MIN_GENERATED_WALL_LENGTH`, and extends what is left by half a divider
//! thickness at both ends. That extension is what closes the half-thickness gap
//! the kernel's region difference otherwise leaves at every junction, and it is
//! safe because the `t/2` band around a claim boundary is reserved by
//! construction and belongs to no compartment interior. `Wall` is the result in
//! the packer's own millimetre plane; `Wall::to_inner_wall` is the same divider
//! as the model's `InnerWall`.

use super::pack::Placement;
use super::rects::{Rect, Segment, boundary_segments, merge_segments, quantize};
use crate::gridfinity::InnerWall;
use crate::layout::Orientation;

/// The shortest generated divider worth building: a run below this is a sliver
/// of a claim boundary rather than a wall between two compartments.
pub const MIN_GENERATED_WALL_LENGTH: f64 = 5.0;

/// A point in the packer's millimetre plane.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Point2 {
    pub x: f64,
    pub y: f64,
}

/// One divider: the centreline it stands on, and the thickness it is built to,
/// centred on that line.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Wall {
    pub start: Point2,
    pub end: Point2,
    pub width: f64,
}

impl Wall {
    /// The same divider as the model's free-form inner wall, in the f32 the
    /// kernel works in and with no height, meaning full height.
    pub fn to_inner_wall(&self) -> InnerWall {
        InnerWall {
            x1: self.start.x as f32,
            y1: self.start.y as f32,
            x2: self.end.x as f32,
            y2: self.end.y as f32,
            width: self.width as f32,
            height: None,
        }
    }

    /// The divider's centreline length in millimetres.
    pub fn length(&self) -> f64 {
        (self.end.x - self.start.x).hypot(self.end.y - self.start.y)
    }
}

/// Whether the run lies on the packing area's own edge, where the bin's
/// perimeter wall already stands.
fn on_area_boundary(segment: &Segment, area: &Rect) -> bool {
    let coordinate = quantize(segment.coordinate);
    match segment.orientation {
        Orientation::V => coordinate == quantize(area.x) || coordinate == quantize(area.right()),
        Orientation::H => coordinate == quantize(area.y) || coordinate == quantize(area.bottom()),
    }
}

/// The run as a divider of the given width, extended by `extension` at both
/// ends.
fn to_wall(segment: &Segment, extension: f64, width: f64) -> Wall {
    let start = quantize(segment.start - extension);
    let end = quantize(segment.end + extension);
    match segment.orientation {
        Orientation::V => Wall {
            start: Point2 {
                x: segment.coordinate,
                y: start,
            },
            end: Point2 {
                x: segment.coordinate,
                y: end,
            },
            width,
        },
        Orientation::H => Wall {
            start: Point2 {
                x: start,
                y: segment.coordinate,
            },
            end: Point2 {
                x: end,
                y: segment.coordinate,
            },
            width,
        },
    }
}

/// How a set of claim boundaries divided up: the runs that became dividers, and
/// the two reasons a run did not.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct WallReport {
    pub generated: usize,
    pub on_boundary: usize,
    pub too_short: usize,
}

/// The dividers implied by a set of placed claims inside `area`: one per shared
/// or interior claim boundary, each extended half a thickness past both ends of
/// the span it divides.
pub fn layout_walls(placements: &[Placement], area: &Rect, divider_thickness: f64) -> Vec<Wall> {
    layout_walls_reporting(placements, area, divider_thickness).0
}

/// The dividers, and a count of every merged claim-boundary run that did not
/// become one: the runs lying on the packing area's own edge, where the bin's
/// perimeter wall already stands, and the runs below the minimum wall length.
pub fn layout_walls_reporting(
    placements: &[Placement],
    area: &Rect,
    divider_thickness: f64,
) -> (Vec<Wall>, WallReport) {
    let segments: Vec<Segment> = placements
        .iter()
        .flat_map(|p| boundary_segments(&p.parts))
        .collect();
    let half = divider_thickness / 2.0;
    let mut report = WallReport::default();
    let mut walls: Vec<Wall> = Vec::new();
    for segment in merge_segments(&segments) {
        if on_area_boundary(&segment, area) {
            report.on_boundary += 1;
        } else if segment.length() < MIN_GENERATED_WALL_LENGTH {
            report.too_short += 1;
        } else {
            walls.push(to_wall(&segment, half, divider_thickness));
        }
    }
    report.generated = walls.len();
    for wall in &walls {
        for point in [wall.start, wall.end] {
            assert!(
                point.x >= area.x - half
                    && point.x <= area.right() + half
                    && point.y >= area.y - half
                    && point.y <= area.bottom() + half,
                "generated divider endpoint {point:?} lies outside the packing area {area:?} \
                 grown by the half thickness {half} it is allowed to overhang"
            );
        }
        assert!(
            wall.length() >= MIN_GENERATED_WALL_LENGTH,
            "a divider of length {} survived the {MIN_GENERATED_WALL_LENGTH} mm minimum",
            wall.length()
        );
    }
    (walls, report)
}

#[cfg(test)]
mod tests {
    use super::super::pack::{PackEffort, PackInput, PackObject, pack_layout};
    use super::super::rects::{Rotation, parts_bounds};
    use super::*;

    const DIVIDER: f64 = 2.0;
    const STEP: f64 = 0.5;

    fn area() -> Rect {
        Rect::new(0.0, 0.0, 200.0, 200.0)
    }

    fn object(id: &str, width: f64, depth: f64, quantity: u32) -> PackObject {
        PackObject {
            id: id.to_string(),
            name: id.to_string(),
            parts: vec![Rect::new(0.0, 0.0, width, depth)],
            quantity,
        }
    }

    fn placement(parts: Vec<Rect>) -> Placement {
        Placement {
            object_id: "a".into(),
            instance: 0,
            rotation: Rotation::Deg0,
            parts,
        }
    }

    fn wall_rect(wall: &Wall) -> Rect {
        let half = wall.width / 2.0;
        if wall.start.x == wall.end.x {
            Rect::new(
                wall.start.x - half,
                wall.start.y.min(wall.end.y),
                wall.width,
                (wall.end.y - wall.start.y).abs(),
            )
        } else {
            Rect::new(
                wall.start.x.min(wall.end.x),
                wall.start.y - half,
                (wall.end.x - wall.start.x).abs(),
                wall.width,
            )
        }
    }

    fn centre(placement: &Placement) -> Point2 {
        let bounds = parts_bounds(&placement.parts);
        Point2 {
            x: bounds.x + bounds.width / 2.0,
            y: bounds.y + bounds.depth / 2.0,
        }
    }

    fn cell_of(point: Point2, area: &Rect) -> usize {
        let cols = (area.width / STEP).ceil() as usize;
        ((point.y - area.y) / STEP).floor() as usize * cols
            + ((point.x - area.x) / STEP).floor() as usize
    }

    fn reachable(from: Point2, walls: &[Wall], area: &Rect) -> Vec<bool> {
        let cols = (area.width / STEP).ceil() as usize;
        let rows = (area.depth / STEP).ceil() as usize;
        let blocks: Vec<Rect> = walls.iter().map(wall_rect).collect();
        let blocked = |col: usize, row: usize| {
            let x = area.x + (col as f64 + 0.5) * STEP;
            let y = area.y + (row as f64 + 0.5) * STEP;
            blocks
                .iter()
                .any(|r| x > r.x && x < r.right() && y > r.y && y < r.bottom())
        };
        let start = cell_of(from, area);
        let mut seen = vec![false; cols * rows];
        seen[start] = true;
        let mut queue = vec![start];
        while let Some(index) = queue.pop() {
            let col = (index % cols) as isize;
            let row = (index / cols) as isize;
            for (dx, dy) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
                let (nc, nr) = (col + dx, row + dy);
                if nc < 0 || nr < 0 || nc >= cols as isize || nr >= rows as isize {
                    continue;
                }
                let next = nr as usize * cols + nc as usize;
                if seen[next] || blocked(nc as usize, nr as usize) {
                    continue;
                }
                seen[next] = true;
                queue.push(next);
            }
        }
        seen
    }

    fn quad() -> (Vec<Placement>, Vec<Wall>) {
        let result = pack_layout(PackInput {
            area: area(),
            objects: vec![object("a", 80.0, 80.0, 4)],
            divider_thickness: DIVIDER,
            clearance: 0.5,
            effort: PackEffort::Quick,
        });
        let walls = layout_walls(&result.placements, &area(), DIVIDER);
        (result.placements, walls)
    }

    #[test]
    fn separates_every_pair_of_compartments() {
        let (placements, walls) = quad();
        assert_eq!(placements.len(), 4);
        for placement in &placements {
            let region = reachable(centre(placement), &walls, &area());
            for other in &placements {
                if std::ptr::eq(other, placement) {
                    continue;
                }
                assert!(
                    !region[cell_of(centre(other), &area())],
                    "compartment {} reaches compartment {}",
                    placement.instance,
                    other.instance
                );
            }
        }
    }

    #[test]
    fn emits_one_shared_divider_where_two_compartments_meet_not_two() {
        let (_, walls) = quad();
        let mut coordinates: Vec<f64> = walls
            .iter()
            .filter(|w| w.start.x == w.end.x)
            .map(|w| w.start.x)
            .collect();
        let count = coordinates.len();
        coordinates.sort_by(f64::total_cmp);
        coordinates.dedup();
        assert_eq!(coordinates.len(), count, "two dividers stacked on one line");
    }

    #[test]
    fn never_puts_a_divider_on_the_cavity_boundary() {
        let (_, walls) = quad();
        let a = area();
        for wall in &walls {
            let on_edge = if wall.start.x == wall.end.x {
                wall.start.x == a.x || wall.start.x == a.right()
            } else {
                wall.start.y == a.y || wall.start.y == a.bottom()
            };
            assert!(!on_edge, "divider {wall:?} stands on the cavity boundary");
        }
    }

    #[test]
    fn extends_every_divider_half_its_thickness_past_the_span_it_divides() {
        let single = layout_walls(
            &[placement(vec![Rect::new(0.0, 0.0, 50.0, 50.0)])],
            &area(),
            DIVIDER,
        );
        assert_eq!(
            single,
            vec![
                Wall {
                    start: Point2 { x: -1.0, y: 50.0 },
                    end: Point2 { x: 51.0, y: 50.0 },
                    width: DIVIDER,
                },
                Wall {
                    start: Point2 { x: 50.0, y: -1.0 },
                    end: Point2 { x: 50.0, y: 51.0 },
                    width: DIVIDER,
                },
            ]
        );
    }

    #[test]
    fn drops_runs_shorter_than_the_minimum_wall_length() {
        let sliver = layout_walls(
            &[placement(vec![Rect::new(0.0, 0.0, 3.0, 3.0)])],
            &area(),
            DIVIDER,
        );
        assert_eq!(sliver, Vec::new());
        assert!(MIN_GENERATED_WALL_LENGTH > 3.0);
    }

    #[test]
    fn emits_nothing_for_an_empty_layout() {
        assert_eq!(layout_walls(&[], &area(), DIVIDER), Vec::new());
    }
}
