
use crate::kernel::build::{loop_of, ring, wall_between};
use crate::kernel::geom::Surface;
use crate::layout::{
    EdgeClass, EffectiveWalls, GridCell, GridEdge, Orientation, SplitLine,
    cell_edges, classify_edge_in, effective_walls, partition_cells,
};
use crate::kernel::split::{Cut, Side, trim};
use crate::kernel::math::{Vec2, Vec3, vec3_of};
use crate::kernel::rectregion::{LoopStyle, RectF, TracedLoop, shape_loop, trace_rects};
use crate::kernel::region2d::{chain_loops, loops_within, region_difference, split_regions};
use crate::kernel::program::{
    DirLoop as POpDirLoop, HoleProfile as PHoleProfile, Op as POp, PlaneRef as PPlaneRef, Program,
    run_all,
};
use crate::kernel::slab::{Op as SlabOp, Slab, SlabOpts, plan_bands};
use crate::kernel::sketch::{
    Aabb, Seg, Sketch, ccw_segs, loop_area, point_in_segs, reverse_loop, segs_bbox,
};
use crate::kernel::topo::{Builder, Loop, Solid};
use std::collections::{HashMap, HashSet};
use std::f32::consts::PI;

pub const GRID_PITCH: f32 = 42.0;
pub const HEIGHT_PER_UNIT: f32 = 7.0;
pub const BASE_TOTAL_HEIGHT: f32 = 7.0;
pub const PEG_HEIGHT: f32 = 4.75;
pub const PEG_Z1: f32 = 0.8;
pub const PEG_Z2: f32 = 2.6;
pub const OUTER_R: f32 = 3.75;
pub const FLOOR_THICKNESS: f32 = 1.2;
pub const HALF_TOL: f32 = 0.25;
const MAGNET_RADIUS: f32 = 3.25;
const MAGNET_DEPTH: f32 = 2.4;
const SCREW_RADIUS: f32 = 1.5;
const SCREW_DEPTH: f32 = 6.0;
const FASTENER_INSET: f32 = 13.0;

const PEG_W_BOTTOM: f32 = 35.6;
const PEG_W_MID: f32 = 37.2;
const PEG_W_TOP: f32 = 41.5;
const PEG_R_BOTTOM: f32 = 0.8;
const PEG_R_MID: f32 = 1.6;
const PEG_TANGENT: f32 = HALF_TOL + OUTER_R;

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
    pub angle_deg: f32,
    pub dir: SlopeDir,
}

#[derive(Clone, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "camelCase", default))]
pub struct LogicalBin {
    pub cells: Vec<GridCell>,
    pub split_lines: Vec<SplitLine>,
    pub slope: Option<BinSlope>,
}

impl LogicalBin {
    pub fn rect(gx: u32, gy: u32) -> LogicalBin {
        LogicalBin { cells: rect_cells(gx, gy), ..Default::default() }
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
    pub x1: f32,
    pub y1: f32,
    pub x2: f32,
    pub y2: f32,
    pub width: f32,
    pub height: Option<f32>,
}

#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "camelCase", default))]
pub struct Params {
    pub bins: Vec<LogicalBin>,
    pub height_units: u32,
    pub wall_thickness: f32,
    pub cavity_corner_radius: f32,
    #[cfg_attr(feature = "serde", serde(rename = "innerFilletRadius"))]
    pub floor_fillet: f32,
    pub magnet_holes: bool,
    pub screw_holes: bool,
    pub open_edges: Vec<GridEdge>,
    pub divider_edges: Vec<GridEdge>,
    pub inner_walls: Vec<InnerWall>,
    pub mode: Mode,
}

impl Default for Params {
    fn default() -> Params {
        Params {
            bins: vec![LogicalBin::rect(2, 2)],
            height_units: 3,
            wall_thickness: 1.2,
            cavity_corner_radius: 2.5,
            floor_fillet: 3.0,
            magnet_holes: false,
            screw_holes: false,
            open_edges: Vec::new(),
            divider_edges: Vec::new(),
            inner_walls: Vec::new(),
            mode: Mode::Bin,
        }
    }
}

impl Params {
    pub fn rect(gx: u32, gy: u32) -> Params {
        Params { bins: vec![LogicalBin::rect(gx, gy)], ..Default::default() }
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

    pub fn total_height(&self) -> f32 {
        BASE_TOTAL_HEIGHT + HEIGHT_PER_UNIT * self.height_units.max(1) as f32
    }
}

pub fn divisions_to_edges(gx: u32, gy: u32, dx: u32, dy: u32) -> Vec<GridEdge> {
    let mut out = Vec::new();
    if gx >= 2 {
        let n = dx.min(gx - 1) as i32;
        let span = gx as i32;
        for i in 0..n {
            let idx = ((i + 1) as f32 * span as f32 / (n + 1) as f32).round() as i32;
            for y in 0..gy as i32 {
                out.push(GridEdge { x: idx, y, orientation: Orientation::V });
            }
        }
    }
    if gy >= 2 {
        let n = dy.min(gy - 1) as i32;
        let span = gy as i32;
        for i in 0..n {
            let idx = ((i + 1) as f32 * span as f32 / (n + 1) as f32).round() as i32;
            for x in 0..gx as i32 {
                out.push(GridEdge { x, y: idx, orientation: Orientation::H });
            }
        }
    }
    out
}

fn planar(b: &mut Builder, z: f32, up: bool, outer: Loop, inners: Vec<Loop>) {
    let surface = if up {
        Surface::plane_z(z)
    } else {
        Surface::plane(vec3_of(0.0, 0.0, z), -Vec3::Z)
    };
    b.face(surface, true, outer, inners);
}

pub fn build(p: &Params) -> Solid {
    try_build(p).expect("gridfinity program")
}

pub fn try_build(p: &Params) -> Result<Solid, String> {
    let solid = match p.mode {
        Mode::Baseplate => build_baseplate(p),
        Mode::Bin => run_all(&program(p))?,
    };
    if let Err(e) = solid.validate() {
        panic!("{:?} is not a closed manifold: {e}", p.mode);
    }
    let report = crate::audit(&solid);
    assert!(report.is_ok(), "{:?} is not geometrically sound:\n{report}", p.mode);
    Ok(solid)
}

pub fn program(p: &Params) -> Program {
    let mut prog = Program::default();
    if p.mode != Mode::Bin {
        return prog;
    }
    for (bi, bin) in p.bins.iter().enumerate() {
        if bin.cells.is_empty() {
            continue;
        }
        let walls = effective_walls(&bin.cells, &bin.cells, &p.open_edges, &p.divider_edges);
        let tag = if p.bins.len() == 1 { "bin".to_string() } else { format!("bin {}", bi + 1) };
        plan_piece(p, &bin.cells, &bin.cells, walls, bin.slope, &tag, &mut prog);
    }
    prog
}

pub struct BinPiece {
    pub name: String,
    pub bin: usize,
    pub piece: usize,
    pub piece_count: usize,
    pub col: i32,
    pub row: i32,
    pub solid: Solid,
}

pub fn build_pieces(p: &Params) -> Vec<BinPiece> {
    try_build_pieces(p).expect("gridfinity piece program")
}

pub fn build_piece(
    p: &Params,
    bin_cells: &[GridCell],
    piece_cells: &[GridCell],
    slope: Option<BinSlope>,
) -> Result<Solid, String> {
    let whole = build_bin_solid(p, bin_cells, slope)?;
    carve_to_cells(&whole, bin_cells, piece_cells)
}

pub fn build_bin_solid(
    p: &Params,
    bin_cells: &[GridCell],
    slope: Option<BinSlope>,
) -> Result<Solid, String> {
    let walls = effective_walls(bin_cells, bin_cells, &p.open_edges, &p.divider_edges);
    let mut prog = Program::default();
    plan_piece(p, bin_cells, bin_cells, walls, slope, "piece", &mut prog);
    let solid = run_all(&prog)?;
    if let Err(e) = solid.validate() {
        panic!("a bin solid is not a closed manifold: {e}");
    }
    let report = crate::audit(&solid);
    assert!(report.is_ok(), "a bin solid is not geometrically sound:
{report}");
    Ok(solid)
}

const REENTRANT_FILLET_OVERHANG: f32 = 8.0;

/// Cut one printable piece out of the finished bin: keep the material inside the
/// vertical prism over the piece's cells. A piece is any connected polyomino, not
/// necessarily a grid slab, so this must follow the cell set itself -- trimming to
/// the piece's bounding box duplicates material wherever one piece's box covers
/// another piece's cells.
pub fn carve_to_cells(
    whole: &Solid,
    bin_cells: &[GridCell],
    piece_cells: &[GridCell],
) -> Result<Solid, String> {
    if piece_cells.is_empty() {
        return Ok(whole.clone());
    }
    if piece_is_enclosed(bin_cells, piece_cells) {
        return Err(
            "a piece surrounded on every side by the rest of the bin is not supported: the cut \
             runs through the middle of faces it never reaches the boundary of, and trimming \
             cannot open a new hole in a face"
                .into(),
        );
    }
    let cell_rect = |c: &GridCell| {
        RectF::new(c.x as f32 * GRID_PITCH, c.y as f32 * GRID_PITCH, GRID_PITCH, GRID_PITCH)
    };
    let mut rects: Vec<RectF> = piece_cells.iter().map(cell_rect).collect();
    for c in piece_cells {
        for step in [-1i32, 1] {
            let neighbour = GridCell { x: c.x, y: c.y + step };
            if bin_cells.contains(&neighbour) {
                continue;
            }
            let y = if step > 0 {
                (c.y + 1) as f32 * GRID_PITCH
            } else {
                c.y as f32 * GRID_PITCH - REENTRANT_FILLET_OVERHANG
            };
            rects.push(RectF::new(
                c.x as f32 * GRID_PITCH,
                y,
                GRID_PITCH,
                REENTRANT_FILLET_OVERHANG,
            ));
        }
    }
    let loops: Vec<Vec<(f32, f32)>> = trace_rects(&rects, &[])
        .iter()
        .map(|lp| lp.pts.iter().map(|p| (p.x, p.y)).collect())
        .collect();
    if loops.is_empty() {
        return Err("a piece traced no boundary".into());
    }
    let cut = Cut::prism(&loops, Vec3::Z)?;
    if !straddles(whole, &cut) {
        return Ok(whole.clone());
    }
    trim(whole, &cut)
}

fn piece_is_enclosed(bin_cells: &[GridCell], piece_cells: &[GridCell]) -> bool {
    if piece_cells.len() >= bin_cells.len() {
        return false;
    }
    piece_cells.iter().all(|c| {
        [(1, 0), (-1, 0), (0, 1), (0, -1)].iter().all(|&(dx, dy)| {
            bin_cells.contains(&GridCell { x: c.x + dx, y: c.y + dy })
        })
    })
}

/// Whether the cut actually divides this solid. A piece whose prism covers the
/// whole bin needs no cut, and a split line that misses a piece's material is a
/// no-op rather than an error -- an L-shaped bin needs that.
fn straddles(solid: &Solid, cut: &Cut) -> bool {
    solid.verts.iter().any(|v| cut.side_of_point(v.point) == Side::Negative)
}

pub fn try_build_pieces(p: &Params) -> Result<Vec<BinPiece>, String> {
    if p.mode == Mode::Baseplate {
        return Ok(vec![BinPiece {
            name: "gridfinity-baseplate.stl".into(),
            bin: 0,
            piece: 0,
            piece_count: 1,
            col: 0,
            row: 0,
            solid: build_baseplate(p),
        }]);
    }
    let bins: Vec<(usize, &LogicalBin)> =
        p.bins.iter().enumerate().filter(|(_, b)| !b.cells.is_empty()).collect();
    let mut out = Vec::new();
    for (ord, (bi, bin)) in bins.iter().enumerate() {
        let parts = partition_cells(&bin.cells, &bin.split_lines);
        let stem = if bins.len() == 1 {
            "gridfinity-bin".to_string()
        } else {
            format!("gridfinity-bin-{}", ord + 1)
        };
        let whole = build_bin_solid(p, &bin.cells, bin.slope)?;
        for (i, part) in parts.iter().enumerate() {
            let solid = carve_to_cells(&whole, &bin.cells, &part.cells)?;
            let name = if parts.len() == 1 {
                format!("{stem}.stl")
            } else {
                format!("{stem}-piece-{}-of-{}.stl", i + 1, parts.len())
            };
            out.push(BinPiece {
                name,
                bin: *bi,
                piece: i,
                piece_count: parts.len(),
                col: part.col,
                row: part.row,
                solid,
            });
        }
    }
    Ok(out)
}


#[derive(Clone, Copy, Debug)]
struct Step {
    from: (i32, i32),
    to: (i32, i32),
    edge: GridEdge,
}

impl Step {
    fn dir(&self) -> (i32, i32) {
        (self.to.0 - self.from.0, self.to.1 - self.from.1)
    }
}

fn boundary_steps(cells: &[GridCell]) -> Vec<Vec<Step>> {
    let set: HashSet<GridCell> = cells.iter().copied().collect();
    let present = |x: i32, y: i32| set.contains(&GridCell { x, y });
    let mut adj: HashMap<(i32, i32), Vec<Step>> = HashMap::new();
    for &c in cells {
        let (x, y) = (c.x, c.y);
        if !present(x, y - 1) {
            let s = Step { from: (x, y), to: (x + 1, y), edge: GridEdge { x, y, orientation: Orientation::H } };
            adj.entry(s.from).or_default().push(s);
        }
        if !present(x + 1, y) {
            let s = Step { from: (x + 1, y), to: (x + 1, y + 1), edge: GridEdge { x: x + 1, y, orientation: Orientation::V } };
            adj.entry(s.from).or_default().push(s);
        }
        if !present(x, y + 1) {
            let s = Step { from: (x + 1, y + 1), to: (x, y + 1), edge: GridEdge { x, y: y + 1, orientation: Orientation::H } };
            adj.entry(s.from).or_default().push(s);
        }
        if !present(x - 1, y) {
            let s = Step { from: (x, y + 1), to: (x, y), edge: GridEdge { x, y, orientation: Orientation::V } };
            adj.entry(s.from).or_default().push(s);
        }
    }

    let mut used: HashSet<((i32, i32), (i32, i32))> = HashSet::new();
    let mut starts: Vec<(i32, i32)> = adj.keys().copied().collect();
    starts.sort_unstable();
    let mut loops = Vec::new();
    for &start in &starts {
        loop {
            let Some(&first) = adj[&start].iter().find(|s| !used.contains(&(s.from, s.to))) else {
                break;
            };
            used.insert((first.from, first.to));
            let mut steps = vec![first];
            let mut cur = first;
            while cur.to != start {
                let din = cur.dir();
                let prefs = [(-din.1, din.0), din, (din.1, -din.0)];
                let mut next: Option<Step> = None;
                'outer: for d in prefs {
                    if let Some(cands) = adj.get(&cur.to) {
                        for &s in cands {
                            if s.dir() == d && !used.contains(&(s.from, s.to)) {
                                next = Some(s);
                                break 'outer;
                            }
                        }
                    }
                }
                let Some(nxt) = next else { break };
                used.insert((nxt.from, nxt.to));
                steps.push(nxt);
                cur = nxt;
            }
            loops.push(steps);
        }
    }
    loops
}

fn mm(p: (i32, i32)) -> Vec2 {
    Vec2::new(p.0 as f32 * GRID_PITCH, p.1 as f32 * GRID_PITCH)
}

fn left_of(d: (i32, i32)) -> Vec2 {
    Vec2::new(-d.1 as f32, d.0 as f32)
}

fn dirv(d: (i32, i32)) -> Vec2 {
    Vec2::new(d.0 as f32, d.1 as f32)
}


#[derive(Clone, Copy)]
struct OuterPiece {
    seg: Seg,
    shared: bool,
    edge: Option<GridEdge>,
}

#[derive(Default, Clone)]
struct SharedWithPegs {
    sides: HashSet<GridEdge>,
    corners: HashSet<(i32, i32)>,
}

fn author_outer_loop(
    steps: &[Step],
    inset: &dyn Fn(&GridEdge) -> f32,
    walled: &dyn Fn(&GridEdge) -> bool,
    shared: &mut SharedWithPegs,
) -> Vec<OuterPiece> {
    let n = steps.len();
    let mut pieces: Vec<OuterPiece> = Vec::new();
    for k in 0..n {
        let s = &steps[k];
        let s_next = &steps[(k + 1) % n];
        let d = dirv(s.dir());
        let nrm = left_of(s.dir());
        let ins = inset(&s.edge);
        let ins_next = inset(&s_next.edge);
        let from = mm(s.from);
        let to = mm(s.to);
        let is_std = (ins - HALF_TOL).abs() < 1e-6;

        let a = from + d * PEG_TANGENT + nrm * ins;
        let b = to - d * PEG_TANGENT + nrm * ins;
        pieces.push(OuterPiece { seg: Seg::Line { a, b }, shared: is_std, edge: Some(s.edge) });
        if is_std {
            shared.sides.insert(s.edge);
        }

        let d1 = dirv(s_next.dir());
        let n1 = left_of(s_next.dir());
        let cross = d.x * d1.y - d.y * d1.x;
        let start = to - d * PEG_TANGENT + nrm * ins;
        let end = to + d1 * PEG_TANGENT + n1 * ins_next;
        let both_std = is_std && (ins_next - HALF_TOL).abs() < 1e-6;
        let same_side = walled(&s.edge) == walled(&s_next.edge);
        if cross.abs() < 0.5 {
            pieces.push(OuterPiece { seg: Seg::Line { a: start, b: end }, shared: false, edge: None });
        } else if cross > 0.0 && both_std && same_side {
            let c = mm(s.to);
            let center = c + nrm * (ins + OUTER_R) + n1 * (ins_next + OUTER_R);
            let a0 = f32::atan2(start.y - center.y, start.x - center.x);
            let a1 = f32::atan2(end.y - center.y, end.x - center.x);
            let (a0, a1) = short_arc(a0, a1);
            pieces.push(OuterPiece {
                seg: Seg::Arc { a: start, b: end, center, radius: OUTER_R, a0, a1 },
                shared: true,
                edge: None,
            });
            shared.corners.insert(s.to);
        } else if cross < 0.0 && both_std && same_side {
            let q = mm(s.to) + nrm * ins + n1 * ins_next;
            let center = q - nrm * OUTER_R - n1 * OUTER_R;
            let t1 = center + nrm * OUTER_R;
            let t2 = center + n1 * OUTER_R;
            let a0 = f32::atan2(t1.y - center.y, t1.x - center.x);
            let a1 = f32::atan2(t2.y - center.y, t2.x - center.x);
            let (a0, a1) = short_arc(a0, a1);
            pieces.push(OuterPiece { seg: Seg::Line { a: start, b: t1 }, shared: false, edge: None });
            pieces.push(OuterPiece {
                seg: Seg::Arc { a: t1, b: t2, center, radius: OUTER_R, a0, a1 },
                shared: false,
                edge: None,
            });
            pieces.push(OuterPiece { seg: Seg::Line { a: t2, b: end }, shared: false, edge: None });
        } else {
            let q = mm(s.to) + nrm * ins + n1 * ins_next;
            pieces.push(OuterPiece { seg: Seg::Line { a: start, b: q }, shared: false, edge: None });
            pieces.push(OuterPiece { seg: Seg::Line { a: q, b: end }, shared: false, edge: None });
        }
    }
    pieces
}

fn short_arc(a0: f32, a1: f32) -> (f32, f32) {
    let mut d = a1 - a0;
    while d > PI {
        d -= 2.0 * PI;
    }
    while d < -PI {
        d += 2.0 * PI;
    }
    (a0, a0 + d)
}


const W_EPS: f32 = 1e-3;

fn v2_eq(a: Vec2, b: Vec2) -> bool {
    (a - b).length() < W_EPS
}

#[derive(Clone, Copy, Debug)]
struct OpenSpan {
    horiz: bool,
    coord: f32,
    lo: f32,
    hi: f32,
}

fn open_spans(cells: &[GridCell], walls: &EffectiveWalls) -> Vec<OpenSpan> {
    let set: HashSet<GridCell> = cells.iter().copied().collect();
    walls
        .open
        .iter()
        .filter(|e| edge_inside_cell(&set, e).is_some())
        .map(|e| {
            let p = GRID_PITCH;
            match e.orientation {
                Orientation::H => OpenSpan {
                    horiz: true,
                    coord: e.y as f32 * p,
                    lo: e.x as f32 * p,
                    hi: (e.x + 1) as f32 * p,
                },
                Orientation::V => OpenSpan {
                    horiz: false,
                    coord: e.x as f32 * p,
                    lo: e.y as f32 * p,
                    hi: (e.y + 1) as f32 * p,
                },
            }
        })
        .collect()
}

fn point_on_spans(spans: &[OpenSpan], pt: Vec2) -> bool {
    spans.iter().any(|s| {
        let (c, a) = if s.horiz { (pt.y, pt.x) } else { (pt.x, pt.y) };
        (c - s.coord).abs() < W_EPS && a > s.lo - W_EPS && a < s.hi + W_EPS
    })
}

fn seg_on_open(spans: &[OpenSpan], seg: &Seg) -> bool {
    let Seg::Line { a, b } = *seg else { return false };
    let horiz = (a.y - b.y).abs() < W_EPS;
    let vert = (a.x - b.x).abs() < W_EPS;
    if horiz == vert {
        return false;
    }
    let coord = if horiz { a.y } else { a.x };
    let (mut lo, mut hi) = if horiz { (a.x, b.x) } else { (a.y, b.y) };
    if lo > hi {
        std::mem::swap(&mut lo, &mut hi);
    }
    let mut covers: Vec<(f32, f32)> = spans
        .iter()
        .filter(|s| s.horiz == horiz && (s.coord - coord).abs() < W_EPS)
        .map(|s| (s.lo, s.hi))
        .collect();
    if covers.is_empty() {
        return false;
    }
    covers.sort_by(|x, y| x.0.total_cmp(&y.0));
    let mut cur = lo;
    for (l, h) in covers {
        if l <= cur + W_EPS && h > cur {
            cur = h;
        }
    }
    cur >= hi - W_EPS
}

struct OuterLoops {
    loops: Vec<Vec<OuterPiece>>,
    consumed: Vec<Vec<bool>>,
}

impl OuterLoops {
    fn new(loops: Vec<Vec<OuterPiece>>) -> OuterLoops {
        let consumed = loops.iter().map(|l| vec![false; l.len()]).collect();
        OuterLoops { loops, consumed }
    }

    fn pinch(&self, s: Vec2, d: Vec2) -> Option<(usize, Vec2)> {
        let mut best: Option<(usize, Vec2, f32)> = None;
        for (li, pieces) in self.loops.iter().enumerate() {
            for pc in pieces {
                let Seg::Line { a, b } = pc.seg else { continue };
                let e = b - a;
                let den = d.x * e.y - d.y * e.x;
                if den.abs() < 1e-6 {
                    continue;
                }
                let t = ((a.x - s.x) * e.y - (a.y - s.y) * e.x) / den;
                let u = ((a.x - s.x) * d.y - (a.y - s.y) * d.x) / den;
                let len = e.length();
                if !(-W_EPS..=1.0 + W_EPS / len).contains(&(u))
                    || !(-0.5..=PEG_TANGENT + 0.6).contains(&t)
                {
                    continue;
                }
                if best.map_or(true, |(_, _, bt)| t.abs() < bt.abs()) {
                    best = Some((li, s + d * t, t));
                }
            }
        }
        best.map(|(li, p, _)| (li, p))
    }

    fn split_at(&mut self, li: usize, p: Vec2, peg_splits: &mut HashMap<GridEdge, Vec<f32>>) {
        let pieces = &mut self.loops[li];
        for i in 0..pieces.len() {
            let Seg::Line { a, b } = pieces[i].seg else { continue };
            if v2_eq(a, p) || v2_eq(b, p) {
                let d = b - a;
                let t = (p - a).dot(d) / d.length_squared();
                if (-0.1..=1.1).contains(&t) && (a + d * t - p).length() < W_EPS {
                    return;
                }
                continue;
            }
            let d = b - a;
            let t = (p - a).dot(d) / d.length_squared();
            if !(0.0..=1.0).contains(&t) || (a + d * t - p).length() > W_EPS {
                continue;
            }
            let pc = pieces[i];
            if pc.shared {
                if let Some(e) = pc.edge {
                    let station = match e.orientation {
                        Orientation::H => p.x,
                        Orientation::V => p.y,
                    };
                    peg_splits.entry(e).or_default().push(station);
                }
            }
            pieces[i] = OuterPiece { seg: Seg::Line { a, b: p }, ..pc };
            pieces.insert(i + 1, OuterPiece { seg: Seg::Line { a: p, b }, ..pc });
            let was = self.consumed[li][i];
            self.consumed[li].insert(i + 1, was);
            return;
        }
        panic!("open-face pinch point {p:?} not on any straight outer piece");
    }

    fn consume_walk(&mut self, li: usize, from: Vec2, to: Vec2) -> Vec<Seg> {
        let pieces = &self.loops[li];
        let n = pieces.len();
        let start = (0..n)
            .find(|&i| v2_eq(pieces[i].seg.start(), from))
            .unwrap_or_else(|| panic!("no outer piece starts at {from:?}"));
        let mut out = Vec::new();
        for k in 0..n {
            let i = (start + k) % n;
            out.push(self.loops[li][i].seg);
            self.consumed[li][i] = true;
            if v2_eq(self.loops[li][i].seg.end(), to) {
                return out;
            }
        }
        panic!("outer walk from {from:?} never reached {to:?}");
    }

    fn consume_all_near(&mut self, probe: Vec2) -> Vec<Seg> {
        for (li, pieces) in self.loops.iter().enumerate() {
            let hit = pieces.iter().any(|pc| match pc.seg {
                Seg::Line { a, b } => {
                    let d = b - a;
                    let t = ((probe - a).dot(d) / d.length_squared()).clamp(0.0, 1.0);
                    (a + d * t - probe).length() < W_EPS
                }
                _ => false,
            });
            if hit {
                for c in &mut self.consumed[li] {
                    *c = true;
                }
                return self.loops[li].iter().map(|pc| pc.seg).collect();
            }
        }
        panic!("no outer loop passes near {probe:?}");
    }
}

struct CavityLoop {
    segs: Vec<Seg>,
    coincident: Vec<bool>,
}

impl CavityLoop {
    fn untouched(segs: Vec<Seg>) -> CavityLoop {
        let n = segs.len();
        CavityLoop { segs, coincident: vec![false; n] }
    }
    fn touched(&self) -> bool {
        self.coincident.iter().any(|&c| c)
    }
}

fn resolve_open_runs(
    shaped: Vec<Seg>,
    spans: &[OpenSpan],
    o: &mut OuterLoops,
    peg_splits: &mut HashMap<GridEdge, Vec<f32>>,
) -> CavityLoop {
    let on: Vec<bool> = shaped.iter().map(|s| seg_on_open(spans, s)).collect();
    if !on.iter().any(|&b| b) {
        return CavityLoop::untouched(shaped);
    }
    if on.iter().all(|&b| b) {
        let s0 = shaped[0].start();
        let s1 = shaped[0].end();
        let d = s1 - s0;
        let nrm = Vec2::new(-d.y, d.x).normalize();
        let mid = (s0 + s1) * 0.5;
        let probe = [mid, mid + nrm * HALF_TOL, mid - nrm * HALF_TOL]
            .into_iter()
            .find(|&q| {
                o.loops.iter().any(|pieces| {
                    pieces.iter().any(|pc| match pc.seg {
                        Seg::Line { a, b } => {
                            let e = b - a;
                            let t = ((q - a).dot(e) / e.length_squared()).clamp(0.0, 1.0);
                            (a + e * t - q).length() < W_EPS
                        }
                        _ => false,
                    })
                })
            })
            .expect("fully-open cavity: no outer loop found near its boundary");
        let segs = o.consume_all_near(probe);
        let n = segs.len();
        return CavityLoop { segs, coincident: vec![true; n] };
    }

    let start = on.iter().position(|&b| !b).unwrap();
    let n = shaped.len();
    let mut segs: Vec<Seg> = (0..n).map(|k| shaped[(start + k) % n]).collect();
    let on: Vec<bool> = (0..n).map(|k| on[(start + k) % n]).collect();

    let mut out: Vec<(Seg, bool)> = Vec::new();
    let mut i = 0;
    while i < n {
        if !on[i] {
            out.push((segs[i], false));
            i += 1;
            continue;
        }
        let mut j = i;
        while j < n && on[j] {
            j += 1;
        }
        let (prev_seg, _) = *out.last().expect("run preceded by a segment");
        let Seg::Line { a: pa, b: pb } = prev_seg else {
            panic!("open-run neighbour must be straight (got an arc before the run)");
        };
        let d_prev = (pa - pb).normalize();
        let (li_s, p_s) = o
            .pinch(pb, d_prev)
            .unwrap_or_else(|| panic!("no pinch for run start at {pb:?}"));
        o.split_at(li_s, p_s, peg_splits);

        let next_seg = if j < n { segs[j] } else { out[0].0 };
        let Seg::Line { a: na, b: nb } = next_seg else {
            panic!("open-run neighbour must be straight (got an arc after the run)");
        };
        let d_next = (nb - na).normalize();
        let (li_e, p_e) = o
            .pinch(na, d_next)
            .unwrap_or_else(|| panic!("no pinch for run end at {na:?}"));
        assert_eq!(li_s, li_e, "open run spans two outer loops");
        o.split_at(li_e, p_e, peg_splits);

        if let Some((Seg::Line { b, .. }, _)) = out.last_mut() {
            *b = p_s;
        }
        if j < n {
            if let Seg::Line { a, .. } = &mut segs[j] {
                *a = p_e;
            }
        } else if let (Seg::Line { a, .. }, _) = &mut out[0] {
            *a = p_e;
        }
        for pc in o.consume_walk(li_s, p_s, p_e) {
            out.push((pc, true));
        }
        i = j;
    }
    let (segs, coincident) = out.into_iter().unzip();
    CavityLoop { segs, coincident }
}

fn chain_fragments(mut frags: Vec<Vec<Seg>>) -> Vec<Vec<Seg>> {
    let mut out = Vec::new();
    while let Some(mut cur) = frags.pop() {
        loop {
            let end = cur.last().unwrap().end();
            if v2_eq(end, cur[0].start()) {
                break;
            }
            let next = frags
                .iter()
                .position(|f| v2_eq(f[0].start(), end))
                .unwrap_or_else(|| panic!("wall-sector chain stuck at {end:?}"));
            cur.extend(frags.swap_remove(next));
        }
        out.push(cur);
    }
    out
}


#[derive(Clone, Debug)]
struct Island {
    segs: Vec<Seg>,
    top: Option<f32>,
    fr: f32,
}

#[derive(Clone, Debug)]
struct Notch {
    quad: Vec<Seg>,
    contact: Vec<Seg>,
    top: f32,
}

#[derive(Clone, Debug)]
struct Banded {
    outline_a: Vec<Vec<(Seg, Option<usize>)>>,
    outline_b: Vec<Seg>,
    notches: Vec<Notch>,
}

fn inner_wall_quad(w: &InnerWall, r: f32) -> Option<Vec<Seg>> {
    let a = Vec2::new(w.x1, w.y1);
    let b = Vec2::new(w.x2, w.y2);
    let d = b - a;
    let len = d.length();
    if len < 0.1 {
        return None;
    }
    let u = d / len;
    let n = Vec2::new(-u.y, u.x);
    let hw = w.width.max(0.4) / 2.0;
    let (p0, p1, p2, p3) = (a - n * hw, b - n * hw, b + n * hw, a + n * hw);
    let sharp = vec![
        Seg::Line { a: p0, b: p1 },
        Seg::Line { a: p1, b: p2 },
        Seg::Line { a: p2, b: p3 },
        Seg::Line { a: p3, b: p0 },
    ];
    let mut corners = vec![p0, p1, p2, p3];
    let sharp = if loop_area(&sharp) < 0.0 {
        corners.reverse();
        reverse_loop(&sharp)
    } else {
        sharp
    };
    let r = r.min(hw).min(len / 2.0);
    if r < 0.02 {
        return Some(sharp);
    }
    let n_c = corners.len();
    let tangents: Vec<(Vec2, Vec2, Vec2)> = (0..n_c)
        .map(|i| {
            let v = corners[i];
            let din = (v - corners[(i + n_c - 1) % n_c]).normalize();
            let dout = (corners[(i + 1) % n_c] - v).normalize();
            let t_in = v - din * r;
            (t_in, v + dout * r, t_in + Vec2::new(-din.y, din.x) * r)
        })
        .collect();
    let mut out = Vec::with_capacity(n_c * 2);
    for i in 0..n_c {
        let (t_in, t_out, center) = tangents[i];
        let a0 = f32::atan2(t_in.y - center.y, t_in.x - center.x);
        let a1 = f32::atan2(t_out.y - center.y, t_out.x - center.x);
        let (a0, a1) = short_arc(a0, a1);
        out.push(Seg::Arc { a: t_in, b: t_out, center, radius: r, a0, a1 });
        let next_in = tangents[(i + 1) % n_c].0;
        if (next_in - t_out).length() > 1e-4 {
            out.push(Seg::Line { a: t_out, b: next_in });
        }
    }
    Some(out)
}

fn island_clears(island: &[Seg], outer: &[Seg], needed: f32) -> bool {
    needed <= 0.0 || !loops_within(island, outer, needed)
}

fn inner_wall_quad_in(w: &InnerWall, r: f32, outer: &[Seg]) -> Option<Vec<Seg>> {
    let sharp = inner_wall_quad(w, 0.0)?;
    if r < 0.02 {
        return Some(sharp);
    }
    let floats_free = sharp.iter().all(|s| point_in_segs(s.start(), outer));
    if floats_free { inner_wall_quad(w, r) } else { Some(sharp) }
}


fn peg_profile(c: GridCell, w: f32, r: f32) -> Vec<Seg> {
    let cx = (c.x as f32 + 0.5) * GRID_PITCH;
    let cy = (c.y as f32 + 0.5) * GRID_PITCH;
    ccw_segs(&Sketch::rounded_rect(cx, cy, w, w, r))
}

fn peg_seg_free(s: &Seg, c: GridCell, shared: &SharedWithPegs) -> bool {
    let cx = (c.x as f32 + 0.5) * GRID_PITCH;
    let cy = (c.y as f32 + 0.5) * GRID_PITCH;
    match *s {
        Seg::Line { a, b } => {
            let m = (a + b) * 0.5;
            let horiz = (a.y - b.y).abs() < W_EPS;
            let e = if horiz {
                let y = if m.y < cy { c.y } else { c.y + 1 };
                GridEdge { x: c.x, y, orientation: Orientation::H }
            } else {
                let x = if m.x < cx { c.x } else { c.x + 1 };
                GridEdge { x, y: c.y, orientation: Orientation::V }
            };
            !shared.sides.contains(&e)
        }
        Seg::Arc { center, .. } => {
            let lx = if center.x > cx { c.x + 1 } else { c.x };
            let ly = if center.y > cy { c.y + 1 } else { c.y };
            !shared.corners.contains(&(lx, ly))
        }
    }
}

fn split_peg_profile(
    segs: Vec<Seg>,
    c: GridCell,
    splits: &HashMap<GridEdge, Vec<f32>>,
) -> Vec<Seg> {
    let [west, east, south, north] = cell_edges(c);
    let cx = (c.x as f32 + 0.5) * GRID_PITCH;
    let cy = (c.y as f32 + 0.5) * GRID_PITCH;
    let mut out = Vec::with_capacity(segs.len());
    for s in segs {
        let Seg::Line { a, b } = s else {
            out.push(s);
            continue;
        };
        let horiz = (a.y - b.y).abs() < W_EPS;
        let e = if horiz {
            if a.y < cy { south } else { north }
        } else if a.x < cx {
            west
        } else {
            east
        };
        let Some(stations) = splits.get(&e) else {
            out.push(s);
            continue;
        };
        let coord = |p: Vec2| if horiz { p.x } else { p.y };
        let (c0, c1) = (coord(a), coord(b));
        let mut cuts: Vec<f32> = stations
            .iter()
            .copied()
            .filter(|&t| (t - c0.min(c1)) > W_EPS && (c0.max(c1) - t) > W_EPS)
            .collect();
        cuts.sort_by(|x, y| {
            if c1 > c0 { x.total_cmp(y) } else { y.total_cmp(x) }
        });
        cuts.dedup_by(|x, y| (*x - *y).abs() < W_EPS);
        let mut prev = a;
        for t in cuts {
            let p = if horiz { Vec2::new(t, a.y) } else { Vec2::new(a.x, t) };
            out.push(Seg::Line { a: prev, b: p });
            prev = p;
        }
        out.push(Seg::Line { a: prev, b });
    }
    out
}


const STRIP_OUT: f32 = 1.0;

fn edge_inside_cell(set: &HashSet<GridCell>, e: &GridEdge) -> Option<GridCell> {
    let (a, b) = match e.orientation {
        Orientation::V => (GridCell { x: e.x - 1, y: e.y }, GridCell { x: e.x, y: e.y }),
        Orientation::H => (GridCell { x: e.x, y: e.y - 1 }, GridCell { x: e.x, y: e.y }),
    };
    if set.contains(&a) { Some(a) } else if set.contains(&b) { Some(b) } else { None }
}

fn plan_cavity(cells: &[GridCell], walls: &EffectiveWalls, wall_thickness: f32) -> (Vec<RectF>, Vec<RectF>) {
    let p = GRID_PITCH;
    let t = HALF_TOL + wall_thickness;
    let set: HashSet<GridCell> = cells.iter().copied().collect();

    let pos: Vec<RectF> = cells
        .iter()
        .map(|c| RectF::new(c.x as f32 * p, c.y as f32 * p, p, p))
        .collect();
    let mut neg: Vec<RectF> = Vec::new();

    for e in &walls.walled {
        let Some(inside) = edge_inside_cell(&set, e) else { continue };
        match e.orientation {
            Orientation::H => {
                let below = inside.y == e.y - 1;
                let y0 = e.y as f32 * p - if below { t } else { STRIP_OUT };
                neg.push(RectF::new(e.x as f32 * p, y0, p, t + STRIP_OUT));
            }
            Orientation::V => {
                let left = inside.x == e.x - 1;
                let x0 = e.x as f32 * p - if left { t } else { STRIP_OUT };
                neg.push(RectF::new(x0, e.y as f32 * p, t + STRIP_OUT, p));
            }
        }
    }

    for e in &walls.dividers {
        match e.orientation {
            Orientation::H => neg.push(RectF::new(
                e.x as f32 * p,
                e.y as f32 * p - wall_thickness / 2.0,
                p,
                wall_thickness,
            )),
            Orientation::V => neg.push(RectF::new(
                e.x as f32 * p - wall_thickness / 2.0,
                e.y as f32 * p,
                wall_thickness,
                p,
            )),
        }
    }

    let mut lattice: HashSet<(i32, i32)> = HashSet::new();
    for c in cells {
        for l in [(c.x, c.y), (c.x + 1, c.y), (c.x, c.y + 1), (c.x + 1, c.y + 1)] {
            if !lattice.insert(l) {
                continue;
            }
            let quads = [(-1, -1), (0, -1), (-1, 0), (0, 0)];
            let absent: Vec<(i32, i32)> = quads
                .iter()
                .filter(|(qx, qy)| !set.contains(&GridCell { x: l.0 + qx, y: l.1 + qy }))
                .copied()
                .collect();
            if absent.len() != 1 {
                continue;
            }
            let (qx, qy) = absent[0];
            let v_edge = GridEdge { x: l.0, y: l.1 + qy, orientation: Orientation::V };
            let h_edge = GridEdge { x: l.0 + qx, y: l.1, orientation: Orientation::H };
            if !walls.walled.contains(&v_edge) || !walls.walled.contains(&h_edge) {
                continue;
            }
            neg.push(RectF::new(
                l.0 as f32 * p + if qx == 0 { -t } else { 0.0 },
                l.1 as f32 * p + if qy == 0 { -t } else { 0.0 },
                t,
                t,
            ));
        }
    }

    (pos, neg)
}


fn plan_piece(
    p: &Params,
    cells: &[GridCell],
    bin_cells: &[GridCell],
    walls: EffectiveWalls,
    slope: Option<BinSlope>,
    tag: &str,
    prog: &mut Program,
) {
    let _perf = crate::kernel::perf::scope(crate::kernel::perf::Metric::PlanPiece);
    let mut _g = crate::kernel::perf::scope(crate::kernel::perf::Metric::PlanOuter);
    let total_h = p.total_height();
    let floor_z = BASE_TOTAL_HEIGHT + FLOOR_THICKNESS;
    let openish = !walls.open.is_empty();
    let slope = if openish { None } else { slope };

    let bin_set = crate::layout::cell_set(bin_cells);
    let seam = |e: &GridEdge| classify_edge_in(&bin_set, *e) == EdgeClass::Internal;
    let inset = |e: &GridEdge| -> f32 { if seam(e) { 0.0 } else { HALF_TOL } };
    let walled = |e: &GridEdge| walls.walled.contains(e);
    let loops = boundary_steps(cells);
    let mut shared = SharedWithPegs::default();
    let outer_loops: Vec<Vec<OuterPiece>> = loops
        .iter()
        .map(|steps| author_outer_loop(steps, &inset, &walled, &mut shared))
        .collect();
    let mut o = OuterLoops::new(outer_loops);
    let spans = open_spans(cells, &walls);
    let mut peg_splits: HashMap<GridEdge, Vec<f32>> = HashMap::new();

    let wt = if openish {
        p.wall_thickness.max(0.4).min(PEG_TANGENT - 0.6)
    } else {
        p.wall_thickness.max(0.4)
    };
    drop(_g);
    let mut _g = crate::kernel::perf::scope(crate::kernel::perf::Metric::PlanCavity);
    let (pos, neg) = plan_cavity(cells, &walls, wt);
    let traced = trace_rects(&pos, &neg);
    let cavity_depth = total_h - floor_z;
    let rc = p.cavity_corner_radius.max(0.0);
    let mut fr = p.floor_fillet.min(cavity_depth - 0.05).max(0.0);
    if slope.is_some() {
        fr = 0.0;
    }
    if rc > 0.05 {
        fr = fr.min(rc - 0.02);
    } else {
        fr = 0.0;
    }

    let outers_traced: Vec<&TracedLoop> = traced.iter().filter(|l| !l.is_hole()).collect();
    let holes_of = |ol: &TracedLoop| -> Vec<&TracedLoop> {
        traced
            .iter()
            .filter(|l| l.is_hole() && point_in_rect_loop(l.pts[0], ol))
            .collect()
    };

    let mut planned: Vec<(CavityLoop, Vec<Island>, f32, Option<Banded>)> = Vec::new();
    let corner_r = (OUTER_R - wt).max(0.0);
    for ol in &outers_traced {
        let (convex_r, concave_r) =
            if slope.is_some() { (0.0, 0.0) } else { (rc.max(corner_r), fr) };
        let shape = if openish {
            shape_cavity_loop_open(ol, convex_r, concave_r, &spans)
        } else {
            shape_cavity_loop(ol, convex_r, concave_r)
        };
        let cl = if openish {
            resolve_open_runs(shape, &spans, &mut o, &mut peg_splits)
        } else {
            CavityLoop::untouched(shape)
        };
        let islands: Vec<Island> = holes_of(ol)
            .iter()
            .map(|il| Island { segs: shape_cavity_loop(il, rc, fr), top: None, fr: 0.0 })
            .collect();
        let full_walls: Vec<Vec<Seg>> = p
            .inner_walls
            .iter()
            .filter(|w| w.height.is_none_or(|h| h >= cavity_depth))
            .filter_map(|w| inner_wall_quad_in(w, fr, &cl.segs))
            .collect();
        let mut entries: Vec<(CavityLoop, Vec<Island>, Option<Banded>)> = Vec::new();
        if cl.touched() || full_walls.is_empty() {
            entries.push((cl, islands, None));
        } else {
            let mut region: Vec<Vec<Seg>> = vec![cl.segs.clone()];
            region.extend(islands.iter().map(|il| reverse_loop(&il.segs)));
            for q in &full_walls {
                region = region_difference(&region, std::slice::from_ref(q));
            }
            let mut outs: Vec<(Vec<Seg>, Vec<Island>)> = Vec::new();
            let mut hole_loops: Vec<Vec<Seg>> = Vec::new();
            for lp in region {
                let lp = round_sharp_corners(&lp, rc, fr);
                if loop_area(&lp) > 0.0 {
                    outs.push((lp, Vec::new()));
                } else {
                    hole_loops.push(lp);
                }
            }
            for h in hole_loops {
                let pt = h[0].start();
                let mut best: Option<usize> = None;
                for (i, (o, _)) in outs.iter().enumerate() {
                    if point_in_segs(pt, o)
                        && best
                            .is_none_or(|bi| loop_area(o).abs() < loop_area(&outs[bi].0).abs())
                    {
                        best = Some(i);
                    }
                }
                if let Some(bi) = best {
                    outs[bi].1.push(Island { segs: reverse_loop(&h), top: None, fr: 0.0 });
                }
            }
            for (o, isls) in outs {
                entries.push((CavityLoop::untouched(o), isls, None));
            }
        }
        let partial_walls: Vec<(&InnerWall, Vec<Seg>, f32)> = p
            .inner_walls
            .iter()
            .filter_map(|w| {
                let h = w.height?;
                if h >= cavity_depth {
                    return None;
                }
                Some((w, inner_wall_quad(w, 0.0)?, floor_z + h.max(0.5)))
            })
            .collect();
        'walls: for &(w, ref q, t) in &partial_walls {
            for (ecl, eisl, band) in &mut entries {
                if ecl.touched() {
                    continue;
                }
                let corners: Vec<Vec2> = q.iter().map(|s| s.start()).collect();
                let n_in = corners.iter().filter(|&&c| point_in_segs(c, &ecl.segs)).count();
                if n_in == 0 {
                    continue;
                }
                let clear = corners
                    .iter()
                    .all(|&c| eisl.iter().all(|il| !point_in_segs(c, &il.segs)));
                if !clear {
                    continue 'walls;
                }
                if n_in == corners.len() {
                    let rounded = inner_wall_quad(w, fr).expect("non-degenerate (filtered above)");
                    eisl.push(Island { segs: rounded, top: Some(t), fr: 0.0 });
                    continue 'walls;
                }
                if slope.is_some() {
                    continue 'walls;
                }
                let bd = band.get_or_insert_with(|| Banded {
                    outline_a: vec![ecl.segs.iter().map(|&s| (s, None)).collect()],
                    outline_b: ecl.segs.clone(),
                    notches: Vec::new(),
                });
                let ni = bd.notches.len();
                let qa: Vec<Vec<(Seg, Option<usize>)>> =
                    vec![q.iter().map(|&s| (s, Some(ni))).collect()];
                let sa = split_regions(&bd.outline_a, &qa);
                if sa.b_inside.is_empty() || sa.a_inside.is_empty() {
                    continue 'walls;
                }
                let mut kept = sa.a_outside.clone();
                kept.extend(sa.b_inside.iter().map(|&(s, t)| (s.reversed(), t)));
                bd.outline_a = chain_loops(kept);
                let ob: Vec<Vec<(Seg, ())>> = vec![
                    std::mem::take(&mut bd.outline_b).into_iter().map(|s| (s, ())).collect(),
                ];
                let qb: Vec<Vec<(Seg, ())>> = vec![q.iter().map(|&s| (s, ())).collect()];
                let sb = split_regions(&ob, &qb);
                let mut b_all = sb.a_outside;
                b_all.extend(sb.a_inside);
                bd.outline_b = chain_loops(b_all)
                    .pop()
                    .unwrap_or_else(|| ob[0].clone())
                    .into_iter()
                    .map(|(s, _)| s)
                    .collect();
                bd.notches.push(Notch {
                    quad: q.clone(),
                    contact: sa.a_inside.into_iter().map(|(s, _)| s).collect(),
                    top: t,
                });
                continue 'walls;
            }
        }
        const MIN_TORUS_MAJOR: f32 = 0.1;
        let clamp = |segs: &[Seg], ball_inside_convex: bool, loop_fr: &mut f32| {
            for s in segs {
                if let Seg::Arc { radius, .. } = s {
                    if *radius < *loop_fr + MIN_TORUS_MAJOR
                        && is_convex_arc(segs, s) == ball_inside_convex
                    {
                        *loop_fr = (*radius - MIN_TORUS_MAJOR).max(0.0);
                    }
                }
            }
            if fr > 0.0 && has_sharp_corner(segs) {
                *loop_fr = 0.0;
            }
        };
        for (cl, mut islands, banded) in entries {
            let mut loop_fr = fr;
            clamp(&cl.segs, true, &mut loop_fr);
            for isl in &islands {
                if !island_clears(&isl.segs, &cl.segs, loop_fr) {
                    loop_fr = 0.0;
                }
            }
            for isl in &mut islands {
                let mut island_fr = fr;
                clamp(&isl.segs, false, &mut island_fr);
                if !island_clears(&isl.segs, &cl.segs, island_fr + loop_fr) {
                    island_fr = 0.0;
                }
                isl.fr = island_fr;
            }
            if cl.touched() || banded.is_some() {
                loop_fr = 0.0;
            }
            if std::env::var("DIAG_LOOP").is_ok() {
                eprintln!("loop_fr={loop_fr} segs={} islands={}", cl.segs.len(), islands.len());
                let n = cl.segs.len();
                for i in 0..n {
                    let o = seg_tangent(&cl.segs[i], true);
                    let v = seg_tangent(&cl.segs[(i + 1) % n], false);
                    eprintln!("   {i:2} {:?} -> dot {:.5}", cl.segs[i], o.dot(v));
                }
            }
            planned.push((cl, islands, loop_fr, banded));
        }
    }

    drop(_g);
    let mut _g = crate::kernel::perf::scope(crate::kernel::perf::Metric::PlanOps);
    let peg_profiles: Vec<(GridCell, Vec<Seg>, Vec<Seg>, Vec<Seg>)> = cells
        .iter()
        .map(|&c| {
            (
                c,
                split_peg_profile(peg_profile(c, PEG_W_BOTTOM, PEG_R_BOTTOM), c, &peg_splits),
                split_peg_profile(peg_profile(c, PEG_W_MID, PEG_R_MID), c, &peg_splits),
                split_peg_profile(peg_profile(c, PEG_W_TOP, OUTER_R), c, &peg_splits),
            )
        })
        .collect();
    let outer_rings: Vec<(Vec<Seg>, Vec<bool>)> = o
        .loops
        .iter()
        .map(|pieces| {
            (pieces.iter().map(|p| p.seg).collect(), pieces.iter().map(|p| p.shared).collect())
        })
        .collect();
    let full_hi_rings: Vec<Vec<Seg>> =
        if openish { Vec::new() } else { outer_rings.iter().map(|(s, _)| s.clone()).collect() };

    let mut cav_ops: Vec<(String, POp)> = Vec::new();
    let mut fillet_edges: Vec<(Seg, f32, f32)> = Vec::new();
    let mut rim_holes: Vec<Vec<Seg>> = Vec::new();
    let mut island_tops: Vec<Vec<Seg>> = Vec::new();
    let mut touched: Vec<CavityLoop> = Vec::new();

    for (ci, (cl, island_shapes, loop_fr, banded)) in planned.into_iter().enumerate() {
        if cl.touched() {
            for isl in &island_shapes {
                island_tops.push(isl.segs.clone());
            }
            let floor_holes: Vec<POpDirLoop> = island_shapes
                .iter()
                .map(|i| (i.segs.clone(), false))
                .collect();
            cav_ops.push((
                format!("cavity {ci} (open): floor"),
                POp::PlanarFace {
                    plane: PPlaneRef::Z { z: floor_z, up: true },
                    outer: (cl.segs.clone(), true),
                    holes: floor_holes,
                },
            ));
            for (ii, isl) in island_shapes.iter().enumerate() {
                cav_ops.push((
                    format!("cavity {ci} (open): tower {ii}"),
                    POp::WallFaces {
                        lower: isl.segs.clone(),
                        upper: isl.segs.clone(),
                        z0: floor_z,
                        z1: total_h,
                        outward: true,
                    },
                ));
            }
            touched.push(cl);
            continue;
        }

        if let Some(bd) = banded {
            let (stack, opts, tops, rim, blends) =
                plan_cavity_banded(&bd, &island_shapes, floor_z, total_h);
            island_tops.extend(tops);
            rim_holes.extend(rim);
            fillet_edges.extend(blends);
            cav_ops.push((format!("cavity {ci}: banded slab stack"), POp::Slabs { stack, opts }));
            continue;
        }

        match slope {
            Some(sl) => {
                for isl in &island_shapes {
                    if isl.top.is_none() {
                        island_tops.push(isl.segs.clone());
                    }
                }
                let (ux, uy) = uphill_unit(sl.dir);
                let (min_a, span) = slope_span(bin_cells, ux, uy);
                let m = sl.angle_deg.to_radians().tan().clamp(0.0, 3.0);
                let cavity_depth = total_h - floor_z;
                let h_max = (m * span).min(cavity_depth - 0.5).max(0.0);
                let eff_m = if span > 1e-6 { h_max / span } else { 0.0 };
                let z_of = |pt: Vec2| floor_z + eff_m * (ux * pt.x + uy * pt.y - min_a);
                let origin = Vec3::new(cl.segs[0].start().x, cl.segs[0].start().y, z_of(cl.segs[0].start()));
                let normal = Vec3::new(-eff_m * ux, -eff_m * uy, 1.0).normalize();
                let floor_plane = PPlaneRef::Tilted { origin, normal };
                let top_plane = PPlaneRef::Z { z: total_h, up: true };

                cav_ops.push((
                    format!("cavity {ci}: sloped walls"),
                    POp::SlopedWall {
                        lower: cl.segs.clone(),
                        upper: cl.segs.clone(),
                        lower_plane: floor_plane.clone(),
                        upper_plane: top_plane.clone(),
                        outward: false,
                    },
                ));
                let mut floor_holes: Vec<POpDirLoop> = Vec::new();
                for (ii, isl) in island_shapes.iter().enumerate() {
                    let slope_max = isl
                        .segs
                        .iter()
                        .map(|s| z_of(s.start()))
                        .fold(floor_z, f32::max);
                    let t = isl.top.filter(|&t| t > slope_max + 0.2).unwrap_or(total_h);
                    let island_top_plane = PPlaneRef::Z { z: t, up: true };
                    cav_ops.push((
                        format!("cavity {ci}: sloped island {ii} walls"),
                        POp::SlopedWall {
                            lower: isl.segs.clone(),
                            upper: isl.segs.clone(),
                            lower_plane: floor_plane.clone(),
                            upper_plane: island_top_plane.clone(),
                            outward: true,
                        },
                    ));
                    if t < total_h {
                        cav_ops.push((
                            format!("cavity {ci}: sloped island {ii} top"),
                            POp::PlanarFace {
                                plane: island_top_plane,
                                outer: (isl.segs.clone(), true),
                                holes: vec![],
                            },
                        ));
                    }
                    floor_holes.push((isl.segs.clone(), false));
                }
                cav_ops.push((
                    format!("cavity {ci}: sloped floor"),
                    POp::PlanarFace {
                        plane: floor_plane,
                        outer: (cl.segs.clone(), false),
                        holes: floor_holes,
                    },
                ));
                rim_holes.push(cl.segs.clone());
            }
            None => {
                let (stack, opts, tops, rim, blends) =
                    plan_cavity_flat(&cl.segs, &island_shapes, floor_z, total_h, loop_fr);
                island_tops.extend(tops);
                rim_holes.extend(rim);
                fillet_edges.extend(blends);
                cav_ops.push((format!("cavity {ci}: slab stack"), POp::Slabs { stack, opts }));
            }
        }
    }

    let sector_segs: Vec<Vec<Seg>> =
        if openish { plan_wall_sectors(&o, &touched) } else { Vec::new() };
    let top_walls: Vec<Vec<Seg>> =
        if openish { sector_segs.clone() } else { full_hi_rings.clone() };

    let fastener_profile: Option<PHoleProfile> = match (p.magnet_holes, p.screw_holes) {
        (true, true) => Some(PHoleProfile::Counterbore {
            bore_r: SCREW_RADIUS,
            bore_d: SCREW_DEPTH,
            head_r: MAGNET_RADIUS,
            head_d: MAGNET_DEPTH,
        }),
        (true, false) => Some(PHoleProfile::Plain { radius: MAGNET_RADIUS, depth: MAGNET_DEPTH }),
        (false, true) => Some(PHoleProfile::Plain { radius: SCREW_RADIUS, depth: SCREW_DEPTH }),
        (false, false) => None,
    };
    for (ci, (c, s_bot, s_mid, s_top)) in peg_profiles.iter().enumerate() {
        let bot_name = format!("{tag}: peg {ci} bot");
        let mid_name = format!("{tag}: peg {ci} mid");
        let top_name = format!("{tag}: peg {ci} top");
        prog.push(
            format!("register {bot_name}"),
            POp::Sketch { name: bot_name.clone(), profile: s_bot.clone() },
        );
        prog.push(
            format!("register {mid_name}"),
            POp::Sketch { name: mid_name.clone(), profile: s_mid.clone() },
        );
        prog.push(
            format!("register {top_name}"),
            POp::Sketch { name: top_name.clone(), profile: s_top.clone() },
        );
        prog.push(
            format!("{tag}: peg {ci} loft"),
            POp::Loft {
                profiles: vec![
                    (bot_name.clone(), 0.0),
                    (mid_name.clone(), PEG_Z1),
                    (mid_name.clone(), PEG_Z2),
                    (top_name.clone(), PEG_HEIGHT),
                ],
                outward: true,
            },
        );
        let ccx = (c.x as f32 + 0.5) * GRID_PITCH;
        let ccy = (c.y as f32 + 0.5) * GRID_PITCH;
        if let Some(profile) = &fastener_profile {
            for (dx, dy) in [(-1.0, -1.0), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0)] {
                let hx = ccx + dx * FASTENER_INSET;
                let hy = ccy + dy * FASTENER_INSET;
                prog.push(
                    format!("{tag}: peg {ci} fastener ({hx:.0},{hy:.0})"),
                    POp::Hole {
                        at: Vec2::new(hx, hy),
                        from_z: 0.0,
                        profile: profile.clone(),
                    },
                );
            }
        }
        let mut cap_holes: Vec<POpDirLoop> = Vec::new();
        if let Some(profile) = &fastener_profile {
            let mouth_r = profile.mouth_radius();
            for (dx, dy) in [(-1.0, -1.0), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0)] {
                let hx = ccx + dx * FASTENER_INSET;
                let hy = ccy + dy * FASTENER_INSET;
                let mouth = Sketch::circle(hx, hy, mouth_r).loops.remove(0);
                cap_holes.push((mouth, false));
            }
        }
        prog.push(
            format!("{tag}: peg {ci} bottom cap"),
            POp::PlanarFace {
                plane: PPlaneRef::Z { z: 0.0, up: false },
                outer: (s_bot.clone(), false),
                holes: cap_holes,
            },
        );
    }

    drop(_g);
    let _g = crate::kernel::perf::scope(crate::kernel::perf::Metric::PlanStitch);
    for (li, (segs, _)) in outer_rings.iter().enumerate() {
        let z1 = if openish { floor_z } else { total_h };
        prog.push(
            format!("{tag}: outer wall {li}"),
            POp::Wall {
                lower: segs.clone(),
                upper: segs.clone(),
                z0: PEG_HEIGHT,
                z1,
                outward: true,
            },
        );
    }

    {
        let mut free: Vec<Seg> = Vec::new();
        for (c, _, _, s_top) in &peg_profiles {
            for seg in s_top {
                if peg_seg_free(seg, *c, &shared) {
                    free.push(*seg);
                }
            }
        }
        for (segs, shared_flags) in &outer_rings {
            for (k, seg) in segs.iter().enumerate() {
                if !shared_flags[k] {
                    free.push(seg.reversed());
                }
            }
        }
        let stitched = stitch_loops_2d(free);
        for (i, (outer, holes)) in stitched.into_iter().enumerate() {
            let label = if i == 0 {
                format!("{tag}: bridge underside")
            } else {
                format!("{tag}: bridge underside {i}")
            };
            let hole_loops: Vec<POpDirLoop> = holes.into_iter().map(|h| (h, true)).collect();
            prog.push(
                label,
                POp::PlanarFace {
                    plane: PPlaneRef::Z { z: PEG_HEIGHT, up: false },
                    outer: (outer, true),
                    holes: hole_loops,
                },
            );
        }
    }

    for (label, op) in cav_ops {
        prog.push(format!("{tag}: {label}"), op);
    }

    for (si, sl) in sector_segs.iter().enumerate() {
        prog.push(
            format!("{tag}: wall sector {si}"),
            POp::Wall {
                lower: sl.clone(),
                upper: sl.clone(),
                z0: floor_z,
                z1: total_h,
                outward: true,
            },
        );
    }
    {
        let (tw, it, rh) = (&top_walls, &island_tops, &rim_holes);
        let (tw, it, rh) = (tw.as_slice(), it.as_slice(), rh.as_slice());
        let mut outers: Vec<(Vec<Seg>, Vec<Vec<Seg>>)> = Vec::new();
        for segs in tw {
            if loop_area(segs) > 0.0 {
                outers.push((segs.clone(), Vec::new()));
            }
        }
        for segs in it {
            outers.push((segs.clone(), Vec::new()));
        }
        let holes: Vec<Vec<Seg>> = tw
            .iter()
            .filter(|s| loop_area(s) < 0.0)
            .cloned()
            .chain(rh.iter().cloned())
            .collect();
        for hole in &holes {
            let pt = hole[0].start();
            let mut best: Option<usize> = None;
            for (i, (outer, _)) in outers.iter().enumerate() {
                let a = loop_area(outer).abs();
                if point_in_segs(pt, outer)
                    && best.is_none_or(|bi| a < loop_area(&outers[bi].0).abs())
                {
                    best = Some(i);
                }
            }
            let bi = best.expect("total_h hole without a containing face");
            outers[bi].1.push(hole.clone());
        }
        for (i, (outer, holes)) in outers.into_iter().enumerate() {
            let label = if it.is_empty() && tw.len() == 1 {
                format!("{tag}: rim face")
            } else {
                format!("{tag}: rim face {i}")
            };
            let hole_loops: Vec<POpDirLoop> =
                holes.into_iter().map(|h| (h, true)).collect();
            prog.push(
                label,
                POp::PlanarFace {
                    plane: PPlaneRef::Z { z: total_h, up: true },
                    outer: (outer, true),
                    holes: hole_loops,
                },
            );
        }
    }

    if !fillet_edges.is_empty() {
        prog.push(format!("{tag}: floor fillet"), POp::Fillet { edges: fillet_edges });
    }
}

fn plan_wall_sectors(o: &OuterLoops, touched: &[CavityLoop]) -> Vec<Vec<Seg>> {
    let mut frags: Vec<Vec<Seg>> = Vec::new();
    for (li, pieces) in o.loops.iter().enumerate() {
        let cons = &o.consumed[li];
        let n = pieces.len();
        if cons.iter().all(|&c| c) {
            continue;
        }
        if !cons.iter().any(|&c| c) {
            frags.push(pieces.iter().map(|p| p.seg).collect());
            continue;
        }
        let start = cons.iter().position(|&c| c).unwrap();
        let mut run: Vec<Seg> = Vec::new();
        for k in 1..=n {
            let idx = (start + k) % n;
            if !cons[idx] {
                run.push(pieces[idx].seg);
            } else if !run.is_empty() {
                frags.push(std::mem::take(&mut run));
            }
        }
        if !run.is_empty() {
            frags.push(run);
        }
    }
    for cl in touched {
        let n = cl.segs.len();
        if cl.coincident.iter().all(|&c| c) {
            continue;
        }
        let start = cl.coincident.iter().position(|&c| c).unwrap();
        let mut run: Vec<Seg> = Vec::new();
        let flush = |run: &mut Vec<Seg>, frags: &mut Vec<Vec<Seg>>| {
            if !run.is_empty() {
                frags.push(run.iter().rev().map(|s| s.reversed()).collect());
                run.clear();
            }
        };
        for k in 1..=n {
            let idx = (start + k) % n;
            if !cl.coincident[idx] {
                run.push(cl.segs[idx]);
            } else {
                flush(&mut run, &mut frags);
            }
        }
        flush(&mut run, &mut frags);
    }

    chain_fragments(frags)
}

fn shape_cavity_loop_open(lp: &TracedLoop, rc: f32, rf: f32, spans: &[OpenSpan]) -> Vec<Seg> {
    let n = lp.pts.len();
    let suppressed: Vec<bool> = lp.pts.iter().map(|&p| point_on_spans(spans, p)).collect();
    let inset = |_: usize, _: Vec2, _: Vec2| 0.0f32;
    let radius = move |i: usize, convex: bool| {
        if suppressed[i] {
            return 0.0;
        }
        let mut r = if convex { rc } else { rf };
        let prev = (i + n - 1) % n;
        let next = (i + 1) % n;
        if suppressed[prev] {
            r = r.min(((lp.pts[i] - lp.pts[prev]).length() - 0.35).max(0.0));
        }
        if suppressed[next] {
            r = r.min(((lp.pts[next] - lp.pts[i]).length() - 0.35).max(0.0));
        }
        r
    };
    shape_loop(lp, &LoopStyle { inset: &inset, radius: &radius })
}

fn shape_cavity_loop(lp: &TracedLoop, rc: f32, rf: f32) -> Vec<Seg> {
    let inset = |_: usize, _: Vec2, _: Vec2| 0.0f32;
    let radius = move |_: usize, convex: bool| if convex { rc } else { rf };
    let segs = shape_loop(lp, &LoopStyle { inset: &inset, radius: &radius });
    if loop_area(&segs) < 0.0 { reverse_loop(&segs) } else { segs }
}

fn seg_tangent(s: &Seg, end: bool) -> Vec2 {
    match *s {
        Seg::Line { a, b } => (b - a).normalize(),
        Seg::Arc { a0, a1, .. } => {
            let t = if end { a1 } else { a0 };
            let dir = if a1 >= a0 { 1.0 } else { -1.0 };
            Vec2::new(-t.sin(), t.cos()) * dir
        }
    }
}

fn has_sharp_corner(shape: &[Seg]) -> bool {
    let n = shape.len();
    (0..n).any(|i| {
        let out = seg_tangent(&shape[i], true);
        let inn = seg_tangent(&shape[(i + 1) % n], false);
        out.dot(inn) < 0.9995
    })
}

fn is_convex_arc(shape: &[Seg], s: &Seg) -> bool {
    let ccw = loop_area(shape) > 0.0;
    match s {
        Seg::Arc { a0, a1, .. } => (a1 > a0) == ccw,
        _ => false,
    }
}


fn round_sharp_corners(segs: &[Seg], convex_r: f32, concave_r: f32) -> Vec<Seg> {
    let n = segs.len();
    if n < 2 || (convex_r <= 0.0 && concave_r <= 0.0) {
        return segs.to_vec();
    }
    let ccw = loop_area(segs) > 0.0;

    let mut trim = vec![0.0f32; n];
    let mut arc_r = vec![0.0f32; n];
    let mut tan_half = vec![0.0f32; n];
    for i in 0..n {
        let (cur, next) = (&segs[i], &segs[(i + 1) % n]);
        let (Seg::Line { .. }, Seg::Line { .. }) = (cur, next) else { continue };
        let d_in = seg_tangent(cur, true);
        let d_out = seg_tangent(next, false);
        let dot = d_in.dot(d_out).clamp(-1.0, 1.0);
        if dot > 1.0 - 1e-6 {
            continue;
        }
        if dot < -1.0 + 1e-6 {
            continue;
        }
        let cross = d_in.x * d_out.y - d_in.y * d_out.x;
        let r = if (cross > 0.0) == ccw { convex_r } else { concave_r };
        if r <= 0.0 {
            continue;
        }
        let phi = dot.acos();
        tan_half[i] = (phi / 2.0).tan();
        arc_r[i] = r;
        trim[i] = r * tan_half[i];
    }
    const MIN_ARC_R: f32 = 0.1;
    const USABLE: f32 = 0.98;
    let mut changed = true;
    while changed {
        changed = false;
        for i in 0..n {
            let Seg::Line { a, b } = segs[i] else { continue };
            let prev = (i + n - 1) % n;
            let want = trim[prev] + trim[i];
            let len = (b - a).length() * USABLE;
            if want <= len || want <= 0.0 {
                continue;
            }
            let k = len / want;
            for idx in [prev, i] {
                if trim[idx] > 0.0 {
                    trim[idx] *= k;
                    arc_r[idx] = trim[idx] / tan_half[idx].max(1e-6);
                    changed = true;
                }
            }
        }
        for i in 0..n {
            if arc_r[i] > 0.0 && arc_r[i] < MIN_ARC_R {
                arc_r[i] = 0.0;
                trim[i] = 0.0;
                changed = true;
            }
        }
    }

    let mut out: Vec<Seg> = Vec::with_capacity(n * 2);
    for i in 0..n {
        let prev = (i + n - 1) % n;
        let seg = segs[i];
        let seg = match seg {
            Seg::Line { a, b } => {
                let d = (b - a).normalize_or_zero();
                Seg::Line { a: a + d * trim[prev], b: b - d * trim[i] }
            }
            other => other,
        };
        out.push(seg);
        if trim[i] <= 0.0 || arc_r[i] <= 0.0 {
            continue;
        }
        let v = segs[i].end();
        let d_in = seg_tangent(&segs[i], true);
        let d_out = seg_tangent(&segs[(i + 1) % n], false);
        let cross = d_in.x * d_out.y - d_in.y * d_out.x;
        let p_in = v - d_in * trim[i];
        let p_out = v + d_out * trim[i];
        let nrm = if cross > 0.0 {
            Vec2::new(-d_in.y, d_in.x)
        } else {
            Vec2::new(d_in.y, -d_in.x)
        };
        let center = p_in + nrm * arc_r[i];
        let a0 = f32::atan2(p_in.y - center.y, p_in.x - center.x);
        let a1 = f32::atan2(p_out.y - center.y, p_out.x - center.x);
        let (a0, a1) = short_arc(a0, a1);
        out.push(Seg::Arc { a: p_in, b: p_out, center, radius: arc_r[i], a0, a1 });
    }
    out
}

fn plan_cavity_flat(
    shape: &[Seg],
    islands: &[Island],
    floor_z: f32,
    total_h: f32,
    loop_fr: f32,
) -> (Vec<(SlabOp, Slab)>, SlabOpts, Vec<Vec<Seg>>, Vec<Vec<Seg>>, Vec<(Seg, f32, f32)>) {
    let mut stack = vec![(SlabOp::Union, Slab::new(vec![shape.to_vec()], floor_z, total_h))];
    for isl in islands {
        stack.push((
            SlabOp::Difference,
            Slab::new(vec![isl.segs.clone()], floor_z, isl.top.unwrap_or(total_h)),
        ));
    }
    let mut blends: Vec<(Seg, f32, f32)> = Vec::new();
    if loop_fr > 0.01 {
        blends.extend(shape.iter().map(|s| (*s, floor_z, loop_fr)));
    }
    for isl in islands {
        if isl.fr > 0.01 {
            blends.extend(isl.segs.iter().map(|s| (*s, floor_z, isl.fr)));
        }
    }
    let top_band = plan_bands(&stack)
        .map(|(_, bands)| bands.last().cloned().unwrap_or_default())
        .unwrap_or_default();
    let tops: Vec<Vec<Seg>> =
        top_band.iter().filter(|l| loop_area(l) < 0.0).cloned().collect();
    let rim: Vec<Vec<Seg>> =
        top_band.iter().filter(|l| loop_area(l) > 0.0).cloned().collect();
    assert_eq!(
        tops.len() + rim.len(),
        top_band.len(),
        "a top-band loop has zero area, so it is neither void nor island"
    );
    (stack, SlabOpts { cavity: true, open_at: vec![total_h] }, tops, rim, blends)
}

fn plan_cavity_banded(
    bd: &Banded,
    islands: &[Island],
    floor_z: f32,
    total_h: f32,
) -> (Vec<(SlabOp, Slab)>, SlabOpts, Vec<Vec<Seg>>, Vec<Vec<Seg>>, Vec<(Seg, f32, f32)>) {
    const TRANSITION_R: f32 = 4.0;

    let mut stack = vec![(SlabOp::Union, Slab::new(vec![bd.outline_b.clone()], floor_z, total_h))];
    for n in &bd.notches {
        stack.push((SlabOp::Difference, Slab::new(vec![n.quad.clone()], floor_z, n.top)));
    }
    for isl in islands {
        stack.push((
            SlabOp::Difference,
            Slab::new(vec![isl.segs.clone()], floor_z, isl.top.unwrap_or(total_h)),
        ));
    }
    let opts = SlabOpts { cavity: true, open_at: vec![total_h] };

    let top_band = plan_bands(&stack)
        .map(|(_, bands)| bands.last().cloned().unwrap_or_default())
        .unwrap_or_default();
    let rim: Vec<Vec<Seg>> =
        top_band.iter().filter(|l| loop_area(l) > 0.0).cloned().collect();
    let tops: Vec<Vec<Seg>> =
        top_band.iter().filter(|l| loop_area(l) < 0.0).cloned().collect();
    assert_eq!(
        tops.len() + rim.len(),
        top_band.len(),
        "a top-band loop has zero area, so it is neither void nor island"
    );

    let mut blends: Vec<(Seg, f32, f32)> = Vec::new();
    for n in &bd.notches {
        let r = (total_h - n.top).min(TRANSITION_R);
        if r < 0.05 {
            continue;
        }
        blends.extend(n.contact.iter().map(|s| (*s, n.top, r)));
    }

    (stack, opts, tops, rim, blends)
}


fn slope_span(cells: &[GridCell], ux: f32, uy: f32) -> (f32, f32) {
    let mut min_a = f32::INFINITY;
    let mut max_a = f32::NEG_INFINITY;
    for c in cells {
        for (dx, dy) in [(0, 0), (1, 0), (0, 1), (1, 1)] {
            let x = (c.x + dx) as f32 * GRID_PITCH;
            let y = (c.y + dy) as f32 * GRID_PITCH;
            let a = ux * x + uy * y;
            min_a = min_a.min(a);
            max_a = max_a.max(a);
        }
    }
    (min_a, (max_a - min_a).max(1e-6))
}

fn uphill_unit(dir: SlopeDir) -> (f32, f32) {
    match dir {
        SlopeDir::PlusX => (-1.0, 0.0),
        SlopeDir::MinusX => (1.0, 0.0),
        SlopeDir::PlusY => (0.0, -1.0),
        SlopeDir::MinusY => (0.0, 1.0),
    }
}


fn point_in_rect_loop(pt: Vec2, lp: &TracedLoop) -> bool {
    let n = lp.pts.len();
    let mut inside = false;
    for i in 0..n {
        let a = lp.pts[i];
        let b = lp.pts[(i + 1) % n];
        if (a.y > pt.y) != (b.y > pt.y) {
            let x = a.x + (pt.y - a.y) / (b.y - a.y) * (b.x - a.x);
            if x > pt.x {
                inside = !inside;
            }
        }
    }
    inside
}


fn stitch_loops_2d(free: Vec<Seg>) -> Vec<(Vec<Seg>, Vec<Vec<Seg>>)> {
    let chained = chain_loops(free.into_iter().map(|s| (s, ())).collect());
    let loops: Vec<Vec<Seg>> =
        chained.into_iter().map(|lp| lp.into_iter().map(|(s, _)| s).collect()).collect();
    if loops.is_empty() {
        return Vec::new();
    }
    let bbox: Vec<Aabb> = loops.iter().map(|l| segs_bbox(l)).collect();
    let containers = containment(&loops, &bbox);
    let depth = |i: usize| containers[i].len();

    let mut out: Vec<(Vec<Seg>, Vec<Vec<Seg>>)> = Vec::new();
    let mut out_idx: HashMap<usize, usize> = HashMap::new();
    for (i, lp) in loops.iter().enumerate() {
        if depth(i) % 2 == 0 {
            out_idx.insert(i, out.len());
            out.push((lp.clone(), Vec::new()));
        }
    }
    for (i, lp) in loops.iter().enumerate() {
        if depth(i) % 2 == 1 {
            let owner = *containers[i]
                .iter()
                .filter(|&&j| depth(j) % 2 == 0)
                .max_by_key(|&&j| depth(j))
                .expect("hole loop without containing outer");
            let slot = out_idx[&owner];
            out[slot].1.push(lp.clone());
        }
    }
    out
}

/// For every loop, the loops that contain it.
///
/// A bin's bridge underside stitches into one loop per cell, and every one of
/// those has the same bounding-box area, so ordering candidates by area prunes
/// nothing and the scan is quadratic in cells. Bucketing the boxes on a uniform
/// grid keeps each query to its own neighbourhood; loops whose box spans an
/// unreasonable share of the grid are held aside and tested every time, which
/// bounds the insertion cost without losing candidates.
fn containment(loops: &[Vec<Seg>], bbox: &[Aabb]) -> Vec<Vec<usize>> {
    const MAX_CELLS: usize = 16;
    let n = loops.len();
    let all = bbox.iter().fold(Aabb::EMPTY, |a, b| a.union(*b));
    let side = (all.max - all.min).max_element();
    let k = (n as f32).sqrt().ceil().clamp(1.0, 256.0);
    let inv = if side > 0.0 { k / side } else { 0.0 };
    let (nx, ny) = (
        (((all.max.x - all.min.x) * inv) as usize + 1).min(256),
        (((all.max.y - all.min.y) * inv) as usize + 1).min(256),
    );
    let col = |x: f32| (((x - all.min.x) * inv).max(0.0) as usize).min(nx - 1);
    let row = |y: f32| (((y - all.min.y) * inv).max(0.0) as usize).min(ny - 1);

    let mut buckets: Vec<Vec<u32>> = vec![Vec::new(); nx * ny];
    let mut wide: Vec<u32> = Vec::new();
    for (j, b) in bbox.iter().enumerate() {
        let (i0, i1) = (col(b.min.x), col(b.max.x));
        let (j0, j1) = (row(b.min.y), row(b.max.y));
        if (i1 - i0 + 1) * (j1 - j0 + 1) > MAX_CELLS {
            wide.push(j as u32);
            continue;
        }
        for i in i0..=i1 {
            for r in j0..=j1 {
                buckets[i * ny + r].push(j as u32);
            }
        }
    }

    // A wide loop is tested by every query, and for a whole-bin outline that is
    // hundreds of segments each time. Bucketing its segments by the rows they
    // span leaves only the handful that can cross the query ray.
    let rows: Vec<Vec<Vec<u32>>> = wide
        .iter()
        .map(|&j| {
            let mut rs: Vec<Vec<u32>> = vec![Vec::new(); ny];
            for (si, s) in loops[j as usize].iter().enumerate() {
                let b = s.bbox();
                for r in row(b.min.y)..=row(b.max.y) {
                    rs[r].push(si as u32);
                }
            }
            rs
        })
        .collect();

    (0..n)
        .map(|i| {
            let pt = loops[i][0].start();
            let mut out: Vec<usize> = buckets[col(pt.x) * ny + row(pt.y)]
                .iter()
                .map(|&j| j as usize)
                .filter(|&j| j != i && bbox[j].contains(pt) && point_in_segs(pt, &loops[j]))
                .collect();
            for (w, &j) in wide.iter().enumerate() {
                let j = j as usize;
                if j == i || !bbox[j].contains(pt) {
                    continue;
                }
                crate::kernel::perf::count(crate::kernel::perf::Metric::PointInSegs);
                let hits: u32 = rows[w][row(pt.y)]
                    .iter()
                    .map(|&si| crate::kernel::sketch::seg_crossings(pt, &loops[j][si as usize]))
                    .sum();
                if hits % 2 == 1 {
                    out.push(j);
                }
            }
            out
        })
        .collect()
}


fn build_baseplate(p: &Params) -> Solid {
    let cells = p.all_cells();
    if cells.is_empty() {
        return Builder::new().build();
    }
    let mut b = Builder::new();

    let traced = trace_rects(
        &cells
            .iter()
            .map(|c| RectF::new(c.x as f32 * GRID_PITCH, c.y as f32 * GRID_PITCH, GRID_PITCH, GRID_PITCH))
            .collect::<Vec<_>>(),
        &[],
    );
    let inset = |_: usize, _: Vec2, _: Vec2| 0.0f32;
    let radius = |_: usize, convex: bool| if convex { OUTER_R } else { 0.0 };
    let mut outer_top: Vec<Loop> = Vec::new();
    let mut outer_bot: Vec<Loop> = Vec::new();
    let mut first_outer_top: Option<Loop> = None;
    let mut first_outer_bot: Option<Loop> = None;
    for lp in &traced {
        let segs = {
            let s = shape_loop(lp, &LoopStyle { inset: &inset, radius: &radius });
            if loop_area(&s) < 0.0 && !lp.is_hole() { reverse_loop(&s) } else { s }
        };
        let r_bot = ring(&mut b, &segs, 0.0);
        let r_top = ring(&mut b, &segs, PEG_HEIGHT);
        wall_between(&mut b, &segs, &segs, &r_bot, &r_top, 0.0, PEG_HEIGHT, true);
        if lp.is_hole() {
            outer_top.push(loop_of(&r_top, true));
            outer_bot.push(loop_of(&r_bot, false));
        } else if first_outer_top.is_none() {
            first_outer_top = Some(loop_of(&r_top, true));
            first_outer_bot = Some(loop_of(&r_bot, false));
        }
    }

    for c in &cells {
        let s_bot = peg_profile(*c, PEG_W_BOTTOM, PEG_R_BOTTOM);
        let s_mid = peg_profile(*c, PEG_W_MID, PEG_R_MID);
        let s_top = peg_profile(*c, PEG_W_TOP, OUTER_R);
        let r0 = ring(&mut b, &s_bot, 0.0);
        let r1 = ring(&mut b, &s_mid, PEG_Z1);
        let r2 = ring(&mut b, &s_mid, PEG_Z2);
        let r3 = ring(&mut b, &s_top, PEG_HEIGHT);
        wall_between(&mut b, &s_bot, &s_mid, &r0, &r1, 0.0, PEG_Z1, false);
        wall_between(&mut b, &s_mid, &s_mid, &r1, &r2, PEG_Z1, PEG_Z2, false);
        wall_between(&mut b, &s_mid, &s_top, &r2, &r3, PEG_Z2, PEG_HEIGHT, false);
        outer_top.push(loop_of(&r3, true));
        outer_bot.push(loop_of(&r0, false));
    }

    if let Some(t) = first_outer_top {
        planar(&mut b, PEG_HEIGHT, true, t, outer_top);
    }
    if let Some(bt) = first_outer_bot {
        planar(&mut b, 0.0, false, bt, outer_bot);
    }
    b.build()
}
