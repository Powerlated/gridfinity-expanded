//! The model's input: what a caller asks the generator for.
//!
//! `Params` is a faithful port of the TypeScript reference's `BinConfig`, and
//! `LogicalBin` is one bin of it -- a polyomino cell set with an optional floor
//! slope, not a rectangle. The rest are the enumerations those two are built
//! from and the two conveniences that spell a rectangular bin out as cells
//! (`rect_cells`) and even divisions out as divider edges
//! (`divisions_to_edges`). Every type here is plain data: no validation, no
//! normalisation, no geometry. The UI constrains its own ranges and the model
//! takes what it is given.

use super::*;
use crate::layout::{GridCell, GridEdge, Orientation, SplitLine};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "camelCase"))]
pub enum Mode {
    #[default]
    Bin,
    Baseplate,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum SlopeDir {
    #[cfg_attr(feature = "serde", serde(rename = "+x"))]
    PlusX,
    #[cfg_attr(feature = "serde", serde(rename = "-x"))]
    MinusX,
    #[cfg_attr(feature = "serde", serde(rename = "+y"))]
    PlusY,
    #[cfg_attr(feature = "serde", serde(rename = "-y"))]
    MinusY,
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct BinSlope {
    #[cfg_attr(feature = "serde", serde(rename = "angle"))]
    pub angle_deg: f64,
    pub dir: SlopeDir,
}

/// One compartment stated outright, as the axis-aligned rectangle its cavity is
/// to be, in the bin's own millimetres from the origin of its cell grid.
///
/// A pocket is the *interior* -- the void, not a claim and not a divider
/// centreline -- so the material between two pockets is whatever they leave
/// between them and needs no wall to be named. Corners are rounded by the bin's
/// own `cavity_corner_radius` and the floor blended by its `floor_fillet`, the
/// same as a walked compartment's.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Pocket {
    pub x: f64,
    pub y: f64,
    pub width: f64,
    pub depth: f64,
}

#[derive(Clone, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "camelCase", default))]
pub struct LogicalBin {
    pub cells: Vec<GridCell>,
    pub split_lines: Vec<SplitLine>,
    pub slope: Option<BinSlope>,
    /// The compartments to hollow out, when the caller states them rather than
    /// letting the cell walk derive them. **Empty means the walk**, which is
    /// every hand-drawn bin; non-empty means the bin is solid everywhere these
    /// rectangles are not, which is what a fitted drawer wants -- the space no
    /// object was packed into is material, not an unreachable pocket of air.
    pub pockets: Vec<Pocket>,
}

impl LogicalBin {
    pub fn rect(gx: u32, gy: u32) -> LogicalBin {
        LogicalBin {
            cells: rect_cells(gx, gy),
            ..Default::default()
        }
    }
}

pub fn rect_cells(gx: u32, gy: u32) -> Vec<GridCell> {
    let mut out = Vec::new();
    for x in 0..gx.max(1) as i32 {
        for y in 0..gy.max(1) as i32 {
            out.push(GridCell { x, y });
        }
    }
    out
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct InnerWall {
    pub x1: f64,
    pub y1: f64,
    pub x2: f64,
    pub y2: f64,
    pub width: f64,
    pub height: Option<f64>,
}

#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "camelCase", default))]
pub struct Params {
    pub bins: Vec<LogicalBin>,
    /// The millimetres one grid cell spans, on both axes. `GRID_PITCH` is the
    /// standard and the default; anything else builds a bin of the same shape
    /// on a grid of that size, with every dimension the standard measures from
    /// a cell edge -- the peg profiles, the fastener bores, the baseplate's
    /// reach -- moving with it, and every absolute dimension -- heights, the
    /// outer corner radius, wall thickness -- staying as it is.
    pub pitch: f64,
    pub height_units: u32,
    pub wall_thickness: f64,
    pub cavity_corner_radius: f64,
    #[cfg_attr(feature = "serde", serde(rename = "innerFilletRadius"))]
    pub floor_fillet: f64,
    pub magnet_holes: bool,
    pub screw_holes: bool,
    pub open_edges: Vec<GridEdge>,
    pub divider_edges: Vec<GridEdge>,
    pub inner_walls: Vec<InnerWall>,
    /// Extra plate millimetres along x, in total: the plate's outline stands
    /// `plate_margin_x / 2` outside the cell grid on the -x and +x sides of the
    /// cell set's bounding rectangle, so the grid stays centred in the plate.
    /// Read only in `Mode::Baseplate`, where it is what makes a plate span the
    /// drawer it is fitted to rather than the whole cells the drawer floors to;
    /// an interior notch of the cell set never grows.
    pub plate_margin_x: f64,
    /// Extra plate millimetres along y, in total, exactly as `plate_margin_x`.
    pub plate_margin_y: f64,
    pub mode: Mode,
}

impl Default for Params {
    fn default() -> Params {
        Params {
            bins: vec![LogicalBin::rect(2, 2)],
            pitch: GRID_PITCH,
            height_units: 3,
            wall_thickness: 1.2,
            cavity_corner_radius: 2.5,
            floor_fillet: 3.0,
            magnet_holes: false,
            screw_holes: false,
            open_edges: Vec::new(),
            divider_edges: Vec::new(),
            inner_walls: Vec::new(),
            plate_margin_x: 0.0,
            plate_margin_y: 0.0,
            mode: Mode::Bin,
        }
    }
}

impl Params {
    /// This `Params` written as the Rust literal that rebuilds it, with every
    /// field left at its default omitted.
    ///
    /// The point is that a bin someone is looking at can be turned into a test
    /// without transcribing it. The fuzzer's repro printer emits exactly this --
    /// it calls this function -- so a bin exported from the debugger and a case
    /// the fuzzer shrank arrive in one format, and either can be pasted straight
    /// into a `#[test]`.
    pub fn rust_literal(&self) -> String {
        fn cell_list(cells: &[GridCell]) -> String {
            let cs: Vec<String> = cells
                .iter()
                .map(|c| format!("GridCell {{ x: {}, y: {} }}", c.x, c.y))
                .collect();
            format!("vec![{}]", cs.join(", "))
        }
        fn edge_list(edges: &[GridEdge]) -> String {
            let es: Vec<String> = edges
                .iter()
                .map(|e| {
                    format!(
                        "GridEdge {{ x: {}, y: {}, orientation: Orientation::{:?} }}",
                        e.x, e.y, e.orientation
                    )
                })
                .collect();
            format!("vec![{}]", es.join(", "))
        }

        let d = Params::default();
        let mut f: Vec<String> = Vec::new();

        let bins: Vec<String> = self
            .bins
            .iter()
            .map(|bin| {
                let mut binf: Vec<String> = vec![format!("cells: {}", cell_list(&bin.cells))];
                if !bin.split_lines.is_empty() {
                    let ls: Vec<String> = bin
                        .split_lines
                        .iter()
                        .map(|l| {
                            format!(
                                "SplitLine {{ axis: Axis::{:?}, index: {} }}",
                                l.axis, l.index
                            )
                        })
                        .collect();
                    binf.push(format!("split_lines: vec![{}]", ls.join(", ")));
                }
                if let Some(s) = bin.slope {
                    binf.push(format!(
                        "slope: Some(BinSlope {{ angle_deg: {:?}, dir: SlopeDir::{:?} }})",
                        s.angle_deg, s.dir
                    ));
                }
                if !bin.pockets.is_empty() {
                    let ps: Vec<String> = bin
                        .pockets
                        .iter()
                        .map(|k| {
                            format!(
                                "Pocket {{ x: {:?}, y: {:?}, width: {:?}, depth: {:?} }}",
                                k.x, k.y, k.width, k.depth
                            )
                        })
                        .collect();
                    binf.push(format!("pockets: vec![{}]", ps.join(", ")));
                }
                format!("LogicalBin {{ {}, ..Default::default() }}", binf.join(", "))
            })
            .collect();
        f.push(format!("bins: vec![{}]", bins.join(", ")));

        if self.pitch != d.pitch {
            f.push(format!("pitch: {:?}", self.pitch));
        }
        if self.height_units != d.height_units {
            f.push(format!("height_units: {}", self.height_units));
        }
        for (name, v, dv) in [
            ("wall_thickness", self.wall_thickness, d.wall_thickness),
            (
                "cavity_corner_radius",
                self.cavity_corner_radius,
                d.cavity_corner_radius,
            ),
            ("floor_fillet", self.floor_fillet, d.floor_fillet),
            ("plate_margin_x", self.plate_margin_x, d.plate_margin_x),
            ("plate_margin_y", self.plate_margin_y, d.plate_margin_y),
        ] {
            if v != dv {
                f.push(format!("{name}: {v:?}"));
            }
        }
        if self.magnet_holes {
            f.push("magnet_holes: true".into());
        }
        if self.screw_holes {
            f.push("screw_holes: true".into());
        }
        if self.mode != d.mode {
            f.push(format!("mode: Mode::{:?}", self.mode));
        }
        if !self.open_edges.is_empty() {
            f.push(format!("open_edges: {}", edge_list(&self.open_edges)));
        }
        if !self.divider_edges.is_empty() {
            f.push(format!("divider_edges: {}", edge_list(&self.divider_edges)));
        }
        if !self.inner_walls.is_empty() {
            let ws: Vec<String> = self
                .inner_walls
                .iter()
                .map(|w| {
                    let h = match w.height {
                        Some(h) => format!("Some({h:?})"),
                        None => "None".into(),
                    };
                    format!(
                        "InnerWall {{ x1: {:?}, y1: {:?}, x2: {:?}, y2: {:?}, width: {:?}, \
                         height: {h} }}",
                        w.x1, w.y1, w.x2, w.y2, w.width
                    )
                })
                .collect();
            f.push(format!("inner_walls: vec![{}]", ws.join(", ")));
        }

        let out = format!("Params {{ {}, ..Params::default() }}", f.join(", "));
        // The whole value of this string is that it pastes into a test and
        // compiles. A literal that lost its `bins` or its default tail still
        // looks like a config in a bug report and fails only for whoever tries
        // to use it, long after the bin it described is gone.
        assert!(
            out.starts_with("Params { bins: vec![") && out.ends_with(", ..Params::default() }"),
            "exported config is not a complete Params literal: {out}"
        );
        out
    }

    pub fn rect(gx: u32, gy: u32) -> Params {
        Params {
            bins: vec![LogicalBin::rect(gx, gy)],
            ..Default::default()
        }
    }

    pub fn divisions(mut self, div_x: u32, div_y: u32) -> Params {
        let (gx, gy) = self.grid_extent();
        self.divider_edges = divisions_to_edges(gx, gy, div_x, div_y);
        self
    }

    pub fn grid_extent(&self) -> (u32, u32) {
        let mut mx = 0;
        let mut my = 0;
        for b in &self.bins {
            for c in &b.cells {
                mx = mx.max(c.x + 1);
                my = my.max(c.y + 1);
            }
        }
        (mx.max(1) as u32, my.max(1) as u32)
    }

    pub fn all_cells(&self) -> Vec<GridCell> {
        let mut out = Vec::new();
        for b in &self.bins {
            out.extend(b.cells.iter().copied());
        }
        out
    }

    pub fn total_height(&self) -> f64 {
        BASE_TOTAL_HEIGHT + HEIGHT_PER_UNIT * self.height_units.max(1) as f64
    }
}

pub fn divisions_to_edges(gx: u32, gy: u32, dx: u32, dy: u32) -> Vec<GridEdge> {
    let mut out = Vec::new();
    if gx >= 2 {
        let n = dx.min(gx - 1) as i32;
        let span = gx as i32;
        for i in 0..n {
            let idx = ((i + 1) as f64 * span as f64 / (n + 1) as f64).round() as i32;
            for y in 0..gy as i32 {
                out.push(GridEdge {
                    x: idx,
                    y,
                    orientation: Orientation::V,
                });
            }
        }
    }
    if gy >= 2 {
        let n = dy.min(gy - 1) as i32;
        let span = gy as i32;
        for i in 0..n {
            let idx = ((i + 1) as f64 * span as f64 / (n + 1) as f64).round() as i32;
            for x in 0..gx as i32 {
                out.push(GridEdge {
                    x,
                    y: idx,
                    orientation: Orientation::H,
                });
            }
        }
    }
    out
}
