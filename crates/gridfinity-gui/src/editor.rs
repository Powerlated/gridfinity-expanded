//! 2D layout editor canvas: paint polyomino bins cell by cell, toggle
//! open/divider edges, place split lines, and draw free-form inner walls —
//! the egui counterpart of the TS reference's shape/walls/split editors.

use eframe::egui::{self, Color32, Pos2, Rect, Sense, Stroke, Vec2};
use gridfinity_cad::gridfinity::{InnerWall, Params};
use gridfinity_cad::layout::{
    Axis, EdgeClass, GridCell, GridEdge, Orientation, SplitLine, classify_edge,
};

const PITCH: f32 = 42.0;

/// Distinguishable bin fill colors (mirrors the TS editor palette idea).
pub const BIN_COLORS: &[Color32] = &[
    Color32::from_rgb(0x4e, 0x79, 0xa7),
    Color32::from_rgb(0xf2, 0x8e, 0x2b),
    Color32::from_rgb(0x59, 0xa1, 0x4f),
    Color32::from_rgb(0xe1, 0x57, 0x59),
    Color32::from_rgb(0xb0, 0x7a, 0xa1),
    Color32::from_rgb(0x76, 0xb7, 0xb2),
    Color32::from_rgb(0xed, 0xc9, 0x48),
];

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Tool {
    /// Paint/unpaint the active bin's cells.
    Cells,
    /// Toggle open perimeter edges / internal divider edges.
    Edges,
    /// Toggle split lines of the active bin.
    Split,
    /// Drag to draw a free-form inner wall (mm coordinates).
    Walls,
}

pub struct Editor {
    pub tool: Tool,
    pub active_bin: usize,
    drag_start: Option<Pos2>,
    drag_now: Option<Pos2>,
    /// Width/height applied to newly drawn inner walls.
    pub wall_width: f32,
    pub wall_height: f32,
    pub wall_full: bool,
}

impl Default for Editor {
    fn default() -> Editor {
        Editor {
            tool: Tool::Cells,
            active_bin: 0,
            drag_start: None,
            drag_now: None,
            wall_width: 1.6,
            wall_height: 10.0,
            wall_full: true,
        }
    }
}

impl Editor {
    /// Show the canvas; returns true when `p` changed.
    pub fn canvas(&mut self, ui: &mut egui::Ui, p: &mut Params) -> bool {
        let mut changed = false;

        // View extent: every bin's cells plus one spare ring, at least 5×5.
        let (mut max_x, mut max_y) = (3i32, 3i32);
        for b in &p.bins {
            for c in &b.cells {
                max_x = max_x.max(c.x + 1);
                max_y = max_y.max(c.y + 1);
            }
        }
        let (cols, rows) = ((max_x + 2).min(14), (max_y + 2).min(14));

        let width = ui.available_width().min(340.0);
        let cell = (width / cols as f32).min(38.0);
        let size = Vec2::new(cols as f32 * cell, rows as f32 * cell);
        let (rect, resp) = ui.allocate_exact_size(size, Sense::click_and_drag());
        let painter = ui.painter_at(rect);

        // Grid cell (gx,gy) → screen rect; y-up so the canvas matches model mm.
        let cell_rect = |gx: i32, gy: i32| {
            Rect::from_min_size(
                Pos2::new(
                    rect.left() + gx as f32 * cell,
                    rect.bottom() - (gy + 1) as f32 * cell,
                ),
                Vec2::splat(cell),
            )
        };
        let to_grid = |pos: Pos2| -> (f32, f32) {
            (
                (pos.x - rect.left()) / cell,
                (rect.bottom() - pos.y) / cell,
            )
        };

        painter.rect_filled(rect, 2.0, ui.visuals().extreme_bg_color);
        for (bi, b) in p.bins.iter().enumerate() {
            let col = BIN_COLORS[bi % BIN_COLORS.len()];
            let col = if bi == self.active_bin {
                col
            } else {
                col.gamma_multiply(0.55)
            };
            for c in &b.cells {
                painter.rect_filled(cell_rect(c.x, c.y).shrink(1.0), 2.0, col);
            }
        }
        let grid_stroke = Stroke::new(1.0, ui.visuals().weak_text_color());
        for gx in 0..=cols {
            let x = rect.left() + gx as f32 * cell;
            painter.line_segment([Pos2::new(x, rect.top()), Pos2::new(x, rect.bottom())], grid_stroke);
        }
        for gy in 0..=rows {
            let y = rect.bottom() - gy as f32 * cell;
            painter.line_segment([Pos2::new(rect.left(), y), Pos2::new(rect.right(), y)], grid_stroke);
        }

        // Edge overlays: open perimeter edges red, dividers dark blue.
        let edge_pts = |e: &GridEdge| -> [Pos2; 2] {
            let x0 = rect.left() + e.x as f32 * cell;
            let y0 = rect.bottom() - e.y as f32 * cell;
            match e.orientation {
                Orientation::H => [Pos2::new(x0, y0), Pos2::new(x0 + cell, y0)],
                Orientation::V => [Pos2::new(x0, y0), Pos2::new(x0, y0 - cell)],
            }
        };
        for e in &p.open_edges {
            painter.line_segment(edge_pts(e), Stroke::new(4.0, Color32::from_rgb(0xd6, 0x33, 0x33)));
        }
        for e in &p.divider_edges {
            painter.line_segment(edge_pts(e), Stroke::new(4.0, Color32::from_rgb(0x28, 0x4a, 0x7a)));
        }

        // Split lines of every bin, over that bin's bounding rows/cols.
        for (bi, b) in p.bins.iter().enumerate() {
            let col = BIN_COLORS[bi % BIN_COLORS.len()].gamma_multiply(0.9);
            let (min_x, max_x, min_y, max_y) = bounds(&b.cells);
            for sl in &b.split_lines {
                let stroke = Stroke::new(3.0, col);
                match sl.axis {
                    Axis::X => {
                        let x = rect.left() + sl.index as f32 * cell;
                        let ya = rect.bottom() - min_y as f32 * cell;
                        let yb = rect.bottom() - (max_y + 1) as f32 * cell;
                        painter.line_segment([Pos2::new(x, ya + 0.0), Pos2::new(x, yb)], stroke);
                        painter.circle_filled(Pos2::new(x, (ya + yb) / 2.0), 4.0, col);
                    }
                    Axis::Y => {
                        let y = rect.bottom() - sl.index as f32 * cell;
                        let xa = rect.left() + min_x as f32 * cell;
                        let xb = rect.left() + (max_x + 1) as f32 * cell;
                        painter.line_segment([Pos2::new(xa, y), Pos2::new(xb, y)], stroke);
                        painter.circle_filled(Pos2::new((xa + xb) / 2.0, y), 4.0, col);
                    }
                }
            }
        }

        // Inner walls (mm → grid units).
        for w in &p.inner_walls {
            let a = Pos2::new(
                rect.left() + w.x1 / PITCH * cell,
                rect.bottom() - w.y1 / PITCH * cell,
            );
            let b2 = Pos2::new(
                rect.left() + w.x2 / PITCH * cell,
                rect.bottom() - w.y2 / PITCH * cell,
            );
            let px = (w.width.max(0.4) / PITCH * cell).max(2.0);
            painter.line_segment([a, b2], Stroke::new(px, Color32::from_rgb(0x8a, 0x5a, 0x2b)));
        }
        if let (Some(a), Some(b2)) = (self.drag_start, self.drag_now) {
            painter.line_segment([a, b2], Stroke::new(2.0, Color32::LIGHT_GREEN));
        }

        // ── Interaction ──────────────────────────────────────────────────
        match self.tool {
            Tool::Cells => {
                if resp.clicked() {
                    if let Some(pos) = resp.interact_pointer_pos() {
                        let (fx, fy) = to_grid(pos);
                        let c = GridCell { x: fx.floor() as i32, y: fy.floor() as i32 };
                        if c.x >= 0 && c.y >= 0 {
                            changed |= toggle_cell(p, self.active_bin, c);
                        }
                    }
                }
            }
            Tool::Edges | Tool::Split => {
                if resp.clicked() {
                    if let Some(pos) = resp.interact_pointer_pos() {
                        let (fx, fy) = to_grid(pos);
                        if self.tool == Tool::Edges {
                            if let Some(e) = nearest_edge(fx, fy) {
                                changed |= toggle_edge(p, e);
                            }
                        } else {
                            changed |= toggle_split(p, self.active_bin, fx, fy);
                        }
                    }
                }
            }
            Tool::Walls => {
                if resp.drag_started() {
                    self.drag_start = resp.interact_pointer_pos();
                }
                if resp.dragged() {
                    self.drag_now = resp.interact_pointer_pos();
                }
                if resp.drag_stopped() {
                    if let (Some(a), Some(b2)) = (self.drag_start, self.drag_now) {
                        let (ax, ay) = to_grid(a);
                        let (bx, by) = to_grid(b2);
                        let w = InnerWall {
                            x1: ax * PITCH,
                            y1: ay * PITCH,
                            x2: bx * PITCH,
                            y2: by * PITCH,
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
        }
        changed
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

/// Toggle a cell of the active bin; a cell owned by another bin moves to the
/// active bin. Bins may become empty (the builder skips them).
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
                return false; // never empty the entire layout
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

/// Snap a grid-space point to its nearest cell edge (within 0.3 cells).
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

/// Perimeter edge → toggle open; internal edge → toggle divider (the same
/// resolution the model applies via `classify_edge` against all cells).
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

/// Toggle the split line of the active bin nearest to a grid-space point.
fn toggle_split(p: &mut Params, active: usize, fx: f32, fy: f32) -> bool {
    let Some(bin) = p.bins.get_mut(active) else { return false };
    if bin.cells.is_empty() {
        return false;
    }
    let (min_x, max_x, min_y, max_y) = bounds(&bin.cells);
    let dx = (fx - fx.round()).abs();
    let dy = (fy - fy.round()).abs();
    if dx.min(dy) > 0.3 {
        return false;
    }
    let sl = if dx < dy {
        let idx = fx.round() as i32;
        if idx <= min_x || idx > max_x {
            return false; // must cut strictly inside the bin
        }
        SplitLine { axis: Axis::X, index: idx }
    } else {
        let idx = fy.round() as i32;
        if idx <= min_y || idx > max_y {
            return false;
        }
        SplitLine { axis: Axis::Y, index: idx }
    };
    if let Some(k) = bin.split_lines.iter().position(|&x| x == sl) {
        bin.split_lines.remove(k);
    } else {
        bin.split_lines.push(sl);
    }
    true
}
