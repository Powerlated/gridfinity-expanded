//! The grid canvas the bin is drawn on, and the three tabs that share it.
//!
//! One canvas, three readings of the same pointer: `Tab::Shape` paints cells
//! into the active bin, `Tab::Walls` toggles a perimeter opening or an internal
//! divider where the click lands on an edge and draws a free-form inner wall
//! where it does not, and `Tab::Cuts` toggles a split line. That is the web
//! app's `Sidebar` tab set (`ShapeTab`/`WallsTab`/`CutsTab`) and its
//! interactions, and the colours are `editor.css`'s, so a bin drawn here and
//! the same bin drawn in the browser are the same picture.
//!
//! `Editor` owns only what the canvas needs to interpret a gesture -- which tab
//! is showing, which bin is active, the drag in progress and the width the next
//! wall will be drawn at. Everything it edits lives in the `Params` passed in.

use eframe::egui::{self, Color32, CornerRadius, Pos2, Rect, Sense, Stroke, StrokeKind, Vec2};
use gridfinity_model::gridfinity::{InnerWall, Params};
use gridfinity_model::layout::{
    Axis, EdgeClass, GridCell, GridEdge, Orientation, SplitLine, classify_edge, perimeter_edges,
};

use crate::theme;
use crate::widgets;

const PITCH: f32 = 42.0;

/// `binColors.ts`'s palette, so bin 1 is the same blue in both front ends.
pub const BIN_COLORS: &[Color32] = &[
    Color32::from_rgb(0x25, 0x63, 0xeb),
    Color32::from_rgb(0x16, 0xa3, 0x4a),
    Color32::from_rgb(0xd9, 0x77, 0x06),
    Color32::from_rgb(0xdc, 0x26, 0x26),
    Color32::from_rgb(0x93, 0x33, 0xea),
    Color32::from_rgb(0x0d, 0x94, 0x88),
    Color32::from_rgb(0xdb, 0x27, 0x77),
    Color32::from_rgb(0x65, 0xa3, 0x0d),
];

/// The colour bin `index` is drawn in, wrapping as `binColor` hashes.
pub fn bin_color(index: usize) -> Color32 {
    BIN_COLORS[index % BIN_COLORS.len()]
}

/// Which editor is showing, named as the web sidebar names its tabs.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Tab {
    Shape,
    Walls,
    Cuts,
}

pub struct Editor {
    pub tab: Tab,
    pub active_bin: usize,
    drag_start: Option<Pos2>,
    drag_now: Option<Pos2>,
    pub wall_width: f64,
    pub wall_height: f64,
    pub wall_full: bool,
}

impl Default for Editor {
    fn default() -> Editor {
        Editor {
            tab: Tab::Shape,
            active_bin: 0,
            drag_start: None,
            drag_now: None,
            wall_width: 1.6,
            wall_height: 10.0,
            wall_full: true,
        }
    }
}

/// The canvas's mapping between the panel's pixels and the cell grid, settled
/// once per frame so every layer -- cells, edges, walls, cuts and the pointer
/// itself -- reads the same one. The web editors keep this in `editorCoords.ts`
/// for the same reason: two mappings desynchronise the moment one is edited.
struct Canvas {
    rect: Rect,
    cell: f32,
    cols: i32,
    rows: i32,
}

impl Canvas {
    /// The pixel rectangle of cell `(gx, gy)`. Row 0 is at the bottom, because
    /// the cells are the kernel's own y-up coordinates and the 3D view beside
    /// the canvas is drawn in them.
    fn cell_rect(&self, gx: i32, gy: i32) -> Rect {
        Rect::from_min_size(
            Pos2::new(
                self.rect.left() + gx as f32 * self.cell,
                self.rect.bottom() - (gy + 1) as f32 * self.cell,
            ),
            Vec2::splat(self.cell),
        )
    }

    /// A pixel position in cell units, the fractional part being where in the
    /// cell it fell.
    fn to_grid(&self, pos: Pos2) -> (f32, f32) {
        (
            (pos.x - self.rect.left()) / self.cell,
            (self.rect.bottom() - pos.y) / self.cell,
        )
    }

    /// The two ends of grid edge `e`.
    fn edge_pts(&self, e: &GridEdge) -> [Pos2; 2] {
        let x0 = self.rect.left() + e.x as f32 * self.cell;
        let y0 = self.rect.bottom() - e.y as f32 * self.cell;
        match e.orientation {
            Orientation::H => [Pos2::new(x0, y0), Pos2::new(x0 + self.cell, y0)],
            Orientation::V => [Pos2::new(x0, y0), Pos2::new(x0, y0 - self.cell)],
        }
    }

    /// A point in whole-bin millimetres at its pixel position.
    fn mm_to_pixel(&self, x: f64, y: f64) -> Pos2 {
        Pos2::new(
            self.rect.left() + x as f32 / PITCH * self.cell,
            self.rect.bottom() - y as f32 / PITCH * self.cell,
        )
    }

    /// The two ends of split line `sl` across the cells it cuts, given their
    /// bounding box.
    fn split_pts(&self, sl: SplitLine, b: (i32, i32, i32, i32)) -> [Pos2; 2] {
        let (min_x, max_x, min_y, max_y) = b;
        match sl.axis {
            Axis::X => {
                let x = self.rect.left() + sl.index as f32 * self.cell;
                [
                    Pos2::new(x, self.rect.bottom() - min_y as f32 * self.cell),
                    Pos2::new(x, self.rect.bottom() - (max_y + 1) as f32 * self.cell),
                ]
            }
            Axis::Y => {
                let y = self.rect.bottom() - sl.index as f32 * self.cell;
                [
                    Pos2::new(self.rect.left() + min_x as f32 * self.cell, y),
                    Pos2::new(self.rect.left() + (max_x + 1) as f32 * self.cell, y),
                ]
            }
        }
    }
}

impl Editor {
    /// The canvas drawn and one frame of pointer input applied to `p`,
    /// returning whether that input changed the model.
    ///
    /// The tab decides what a click means and what is drawn over the cells;
    /// nothing else about the canvas differs between them.
    pub fn canvas(&mut self, ui: &mut egui::Ui, p: &mut Params) -> bool {
        let mut changed = false;

        let (mut max_x, mut max_y) = (3i32, 3i32);
        for b in &p.bins {
            for c in &b.cells {
                max_x = max_x.max(c.x + 1);
                max_y = max_y.max(c.y + 1);
            }
        }
        let (cols, rows) = ((max_x + 2).min(14), (max_y + 2).min(14));

        let width = ui.available_width();
        let cell = (width / cols as f32).min(38.0);
        let size = Vec2::new(cols as f32 * cell, rows as f32 * cell);
        let (rect, resp) = ui.allocate_exact_size(size, Sense::click_and_drag());
        let canvas = Canvas { rect, cell, cols, rows };
        let painter = ui.painter_at(rect);
        let hover = resp.hover_pos().map(|pos| canvas.to_grid(pos));

        self.paint_cells(&painter, &canvas, p, hover);
        match self.tab {
            Tab::Shape => {}
            Tab::Walls => self.paint_walls(&painter, &canvas, p, hover),
            Tab::Cuts => self.paint_cuts(&painter, &canvas, p, hover),
        }

        match self.tab {
            Tab::Shape => {
                if resp.clicked() {
                    if let Some((fx, fy)) = resp.interact_pointer_pos().map(|q| canvas.to_grid(q)) {
                        let c = GridCell { x: fx.floor() as i32, y: fy.floor() as i32 };
                        if c.x >= 0 && c.y >= 0 {
                            changed |= toggle_cell(p, self.active_bin, c);
                        }
                    }
                }
            }
            // A click on an edge toggles it and a drag from anywhere else draws
            // a wall, which is `WallsTab`'s one canvas: "click a perimeter to
            // toggle an opening, drag inside the selected bin to add a wall".
            Tab::Walls => {
                if resp.clicked() {
                    if let Some((fx, fy)) = resp.interact_pointer_pos().map(|q| canvas.to_grid(q)) {
                        if let Some(e) = nearest_edge(fx, fy) {
                            changed |= toggle_edge(p, e);
                        }
                    }
                }
                if resp.drag_started() {
                    self.drag_start = resp.interact_pointer_pos().filter(|q| {
                        let (fx, fy) = canvas.to_grid(*q);
                        nearest_edge(fx, fy).is_none()
                    });
                }
                if resp.dragged() && self.drag_start.is_some() {
                    self.drag_now = resp.interact_pointer_pos();
                }
                if resp.drag_stopped() {
                    if let (Some(a), Some(b2)) = (self.drag_start, self.drag_now) {
                        let (ax, ay) = canvas.to_grid(a);
                        let (bx, by) = canvas.to_grid(b2);
                        let w = InnerWall {
                            x1: (ax * PITCH) as f64,
                            y1: (ay * PITCH) as f64,
                            x2: (bx * PITCH) as f64,
                            y2: (by * PITCH) as f64,
                            width: self.wall_width,
                            height: (!self.wall_full).then_some(self.wall_height),
                        };
                        if ((w.x2 - w.x1).powi(2) + (w.y2 - w.y1).powi(2)).sqrt() > 2.0 {
                            p.inner_walls.push(w);
                            changed = true;
                        }
                    }
                    self.drag_start = None;
                    self.drag_now = None;
                }
            }
            Tab::Cuts => {
                if resp.clicked() {
                    if let Some((fx, fy)) = resp.interact_pointer_pos().map(|q| canvas.to_grid(q)) {
                        changed |= toggle_split(p, self.active_bin, fx, fy);
                    }
                }
            }
        }
        changed
    }

    /// The empty grid and the painted cells over it: `.cell` and `.cell.is-on`
    /// from `editor.css`, plus its `:hover` outline on the cell a Shape click
    /// would land in.
    fn paint_cells(
        &self,
        painter: &egui::Painter,
        canvas: &Canvas,
        p: &Params,
        hover: Option<(f32, f32)>,
    ) {
        for gx in 0..canvas.cols {
            for gy in 0..canvas.rows {
                painter.rect(
                    canvas.cell_rect(gx, gy).shrink(1.5),
                    CornerRadius::same(2),
                    theme::DARK_6,
                    Stroke::new(1.0, theme::DARK_4),
                    StrokeKind::Inside,
                );
            }
        }
        // The Walls and Cuts canvases tint their cells rather than fill them,
        // so the edges and cut lines drawn over them stay the subject.
        let opacity = if self.tab == Tab::Shape { 1.0 } else { 0.35 };
        for (bi, b) in p.bins.iter().enumerate() {
            let col = bin_color(bi);
            let col = if bi == self.active_bin { col } else { col.gamma_multiply(0.55) };
            for c in &b.cells {
                painter.rect_filled(
                    canvas.cell_rect(c.x, c.y).shrink(1.5),
                    CornerRadius::same(2),
                    col.gamma_multiply(opacity),
                );
            }
        }
        if self.tab == Tab::Shape {
            if let Some((fx, fy)) = hover {
                if fx >= 0.0 && fy >= 0.0 {
                    painter.rect_stroke(
                        canvas.cell_rect(fx.floor() as i32, fy.floor() as i32).shrink(1.5),
                        CornerRadius::same(2),
                        Stroke::new(1.0, theme::DARK_2),
                        StrokeKind::Inside,
                    );
                }
            }
        }
    }

    /// The Walls overlay: every perimeter edge as the bin's boundary, the open
    /// ones dashed, every divider and free-form wall in the internal-wall
    /// colour, and the drag in progress as a dashed draft between its ends.
    /// `.edge-line--wall`, `.edge-line--open`, `.custom-wall` and
    /// `.custom-wall--draft`.
    fn paint_walls(
        &self,
        painter: &egui::Painter,
        canvas: &Canvas,
        p: &Params,
        hover: Option<(f32, f32)>,
    ) {
        let all: Vec<GridCell> = p.bins.iter().flat_map(|b| b.cells.iter().copied()).collect();
        let hovered = hover.and_then(|(fx, fy)| nearest_edge(fx, fy));
        for e in perimeter_edges(&all) {
            let lit = hovered == Some(e);
            if p.open_edges.contains(&e) {
                let color = if lit { theme::DARK_1 } else { theme::DARK_2 };
                widgets::dashed_line(painter, canvas.edge_pts(&e), Stroke::new(2.0, color), 4.0, 5.0);
            } else {
                let color = if lit { theme::GRAY_3 } else { theme::GRAY_5 };
                painter.line_segment(canvas.edge_pts(&e), Stroke::new(5.0, color));
            }
        }
        for e in &p.divider_edges {
            painter.line_segment(canvas.edge_pts(e), Stroke::new(4.0, theme::TEAL));
        }
        for w in &p.inner_walls {
            let px = (w.width.max(0.4) as f32 / PITCH * canvas.cell).max(2.5);
            painter.line_segment(
                [canvas.mm_to_pixel(w.x1, w.y1), canvas.mm_to_pixel(w.x2, w.y2)],
                Stroke::new(px, theme::TEAL),
            );
        }
        if let (Some(a), Some(b)) = (self.drag_start, self.drag_now) {
            widgets::dashed_line(painter, [a, b], Stroke::new(3.0, theme::TEAL_LIGHT), 5.0, 4.0);
            for end in [a, b] {
                painter.circle_filled(end, 4.0, theme::TEAL_PALE);
            }
        }
    }

    /// The Cuts overlay: every grid line the active bin could be cut on drawn
    /// faintly, the ones it is cut on drawn in the cut colour.
    /// `.cut-line-visible--inactive` and `--active`.
    fn paint_cuts(
        &self,
        painter: &egui::Painter,
        canvas: &Canvas,
        p: &Params,
        hover: Option<(f32, f32)>,
    ) {
        let Some(bin) = p.bins.get(self.active_bin) else { return };
        if bin.cells.is_empty() {
            return;
        }
        let b = bounds(&bin.cells);
        let hovered = hover.and_then(|(fx, fy)| candidate_split(&bin.cells, fx, fy));
        for sl in candidate_splits(&bin.cells) {
            if bin.split_lines.contains(&sl) {
                continue;
            }
            let lit = hovered == Some(sl);
            let stroke = if lit {
                Stroke::new(2.0, theme::YELLOW)
            } else {
                Stroke::new(1.0, theme::DARK_2)
            };
            widgets::dashed_line(painter, canvas.split_pts(sl, b), stroke, 2.0, 5.0);
        }
        for sl in &bin.split_lines {
            let pts = canvas.split_pts(*sl, b);
            widgets::dashed_line(painter, pts, Stroke::new(3.0, theme::YELLOW), 7.0, 4.0);
            painter.circle_filled(
                Pos2::new((pts[0].x + pts[1].x) / 2.0, (pts[0].y + pts[1].y) / 2.0),
                4.0,
                theme::YELLOW,
            );
        }
    }
}

fn bounds(cells: &[GridCell]) -> (i32, i32, i32, i32) {
    let mut b = (i32::MAX, i32::MIN, i32::MAX, i32::MIN);
    for c in cells {
        b.0 = b.0.min(c.x);
        b.1 = b.1.max(c.x);
        b.2 = b.2.min(c.y);
        b.3 = b.3.max(c.y);
    }
    b
}

fn toggle_cell(p: &mut Params, active: usize, c: GridCell) -> bool {
    if active >= p.bins.len() {
        return false;
    }
    let owner = p
        .bins
        .iter()
        .enumerate()
        .find_map(|(bi, b)| b.cells.iter().position(|&x| x == c).map(|k| (bi, k)));
    match owner {
        Some((bi, k)) => {
            if bi == active && p.bins[bi].cells.len() == 1 && p.bins.len() == 1 {
                return false;
            }
            p.bins[bi].cells.remove(k);
            if bi != active {
                p.bins[active].cells.push(c);
            }
        }
        Option::None => p.bins[active].cells.push(c),
    }
    true
}

fn nearest_edge(fx: f32, fy: f32) -> Option<GridEdge> {
    let dx = (fx - fx.round()).abs();
    let dy = (fy - fy.round()).abs();
    if dx.min(dy) > 0.3 {
        return None;
    }
    Some(if dx < dy {
        GridEdge { x: fx.round() as i32, y: fy.floor() as i32, orientation: Orientation::V }
    } else {
        GridEdge { x: fx.floor() as i32, y: fy.round() as i32, orientation: Orientation::H }
    })
}

fn toggle_edge(p: &mut Params, e: GridEdge) -> bool {
    let all: Vec<GridCell> = p.bins.iter().flat_map(|b| b.cells.iter().copied()).collect();
    match classify_edge(&all, e) {
        EdgeClass::Perimeter => toggle_in(&mut p.open_edges, e),
        EdgeClass::Internal => toggle_in(&mut p.divider_edges, e),
        EdgeClass::None => return false,
    }
    true
}

fn toggle_in(v: &mut Vec<GridEdge>, e: GridEdge) {
    if let Some(k) = v.iter().position(|&x| x == e) {
        v.remove(k);
    } else {
        v.push(e);
    }
}

/// Every line `cells` may be cut on: the grid lines strictly inside their
/// bounding box, on both axes. These are the faint candidates the Cuts canvas
/// draws, and the only lines `toggle_split` accepts.
fn candidate_splits(cells: &[GridCell]) -> Vec<SplitLine> {
    let (min_x, max_x, min_y, max_y) = bounds(cells);
    let mut out = Vec::new();
    for index in (min_x + 1)..=max_x {
        out.push(SplitLine { axis: Axis::X, index });
    }
    for index in (min_y + 1)..=max_y {
        out.push(SplitLine { axis: Axis::Y, index });
    }
    out
}

/// The candidate split a pointer `(fx, fy)` cell units into the canvas is over,
/// if it is over one at all.
fn candidate_split(cells: &[GridCell], fx: f32, fy: f32) -> Option<SplitLine> {
    if cells.is_empty() {
        return None;
    }
    let (min_x, max_x, min_y, max_y) = bounds(cells);
    let dx = (fx - fx.round()).abs();
    let dy = (fy - fy.round()).abs();
    if dx.min(dy) > 0.3 {
        return None;
    }
    if dx < dy {
        let index = fx.round() as i32;
        (index > min_x && index <= max_x).then_some(SplitLine { axis: Axis::X, index })
    } else {
        let index = fy.round() as i32;
        (index > min_y && index <= max_y).then_some(SplitLine { axis: Axis::Y, index })
    }
}

fn toggle_split(p: &mut Params, active: usize, fx: f32, fy: f32) -> bool {
    let Some(bin) = p.bins.get_mut(active) else { return false };
    let Some(sl) = candidate_split(&bin.cells, fx, fy) else { return false };
    if let Some(k) = bin.split_lines.iter().position(|&x| x == sl) {
        bin.split_lines.remove(k);
    } else {
        bin.split_lines.push(sl);
    }
    true
}
