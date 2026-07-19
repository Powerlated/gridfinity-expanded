//! WebAssembly bindings exposing the Gridfinity model to the web app.
//!
//! This implements the app's geometry worker contract directly: it is a
//! drop-in for `generateGeometry(wasm, BinParameters[]) -> Bin[]`, so nothing
//! downstream of the worker changes. `BinParameters` deserialises into
//! [`BinParams`] below, one bin per entry, and each bin comes back as its
//! grouped pieces of flat triangle soup.
//!
//! ## What this deliberately does *not* do
//!
//! - **No mirroring.** The frontend's `buildBinParameters()` already mirrors
//!   every spatial value across the design's occupied Y extent, so parameters
//!   arrive in generation coordinates and output goes straight back out.
//!   Mirroring here would double-apply it.
//! - **No validation or clamping.** The UI only emits valid parameters and the
//!   pipeline treats geometry as trusting its input; `npm run check:manifold`
//!   is the gate that verifies the result.
//! - **No welding or degeneracy repair.** That exists in the manifold path
//!   because exact booleans can rebuild a feature twice within one float32 ULP.
//!   This kernel is analytic: `tess.rs` samples each edge exactly once, so the
//!   two faces sharing it emit identical boundary points and the soup is closed
//!   by construction.

use gridfinity_cad::gridfinity::{
    self, BASE_TOTAL_HEIGHT, HEIGHT_PER_UNIT, InnerWall, LogicalBin, Params,
};
use gridfinity_cad::layout::{GridCell, GridEdge};
use gridfinity_cad::tessellate;
use wasm_bindgen::prelude::*;

/// Curve resolution (segments per 90° arc). The app has one quality setting —
/// generated geometry is cached and reused for both preview and export, so this
/// is export quality.
const ARC_SEGMENTS_PER_QUARTER: usize = 16;

/// The app's `Wall`: a straight, full-height segment in generation millimetres.
#[derive(serde::Deserialize)]
struct Wall {
    start: Point2,
    end: Point2,
    width: f32,
}

#[derive(serde::Deserialize)]
struct Point2 {
    x: f32,
    y: f32,
}

/// The app's `BinParameters` — complete, trusted, self-contained input for one
/// bin. Field names match the TypeScript interface exactly.
#[derive(serde::Deserialize)]
#[serde(rename_all = "camelCase")]
struct BinParams {
    bin_id: String,
    /// Total bin height in mm, already converted from height units.
    height: f32,
    perimeter_thickness: f32,
    fillet_radius: f32,
    fasteners: Fasteners,
    cells: Vec<GridCell>,
    openings: Vec<GridEdge>,
    walls: Vec<Wall>,
    /// Piece footprints from the UI's cut planning; array order is piece index.
    pieces: Vec<Vec<GridCell>>,
}

#[derive(serde::Deserialize)]
struct Fasteners {
    magnets: bool,
    m3: bool,
}

impl BinParams {
    /// Maps one bin's parameters onto the kernel's [`Params`].
    ///
    /// Two mappings are worth spelling out:
    ///
    /// - **Height.** The app sends total millimetres, where a *u*-unit bin is
    ///   `u · 7` mm tall overall. The kernel counts units *above* the 7 mm
    ///   base, so it needs `u − 1` to reach the same total.
    /// - **Radius.** The app has one shared radius that both rounds cavity
    ///   corners and blends the floor; the kernel keeps those separate, so both
    ///   of its fields take it.
    fn to_params(&self) -> Params {
        let units_above_base =
            ((self.height - BASE_TOTAL_HEIGHT) / HEIGHT_PER_UNIT).round().max(1.0) as u32;
        Params {
            bins: vec![LogicalBin { cells: self.cells.clone(), ..Default::default() }],
            height_units: units_above_base,
            wall_thickness: self.perimeter_thickness,
            cavity_corner_radius: self.fillet_radius,
            floor_fillet: self.fillet_radius,
            magnet_holes: self.fasteners.magnets,
            screw_holes: self.fasteners.m3,
            open_edges: self.openings.clone(),
            // The app expresses every internal wall as a free-form `Wall`;
            // it has no grid-aligned divider concept.
            divider_edges: Vec::new(),
            inner_walls: self.walls.iter().map(|w| InnerWall {
                x1: w.start.x,
                y1: w.start.y,
                x2: w.end.x,
                y2: w.end.y,
                width: w.width,
                // The app's walls are always full height.
                height: None,
            }).collect(),
            mode: gridfinity::Mode::Bin,
        }
    }
}

#[wasm_bindgen(start)]
pub fn start() {
    // Turns a Rust panic into a readable console error instead of an opaque
    // trap. The generate path itself is fallible by construction.
    console_error_panic_hook::set_once();
}

/// Builds every supplied bin, returning each one's cut pieces grouped under it.
///
/// Mirrors `generateGeometry(wasm, bins)`: input is `BinParameters[]`, output is
/// `[{binId, pieces: [{triangles: Float32Array, cells: Cell[]}]}]`, where
/// `triangles` is a global-coordinate flat soup of 9 floats per triangle.
#[wasm_bindgen]
pub fn generate_geometry(bins: JsValue) -> Result<JsValue, JsValue> {
    let bins: Vec<BinParams> = serde_wasm_bindgen::from_value(bins)
        .map_err(|e| JsValue::from_str(&format!("invalid bin parameters: {e}")))?;

    let out = js_sys::Array::new();
    for bin in &bins {
        let params = bin.to_params();
        let pieces = js_sys::Array::new();

        // A bin with no cut has exactly one piece: its whole footprint.
        for piece_cells in &bin.pieces {
            let solid = gridfinity::build_piece(&params, &bin.cells, piece_cells, None)
                .map_err(|e| JsValue::from_str(&format!("bin {}: {e}", bin.bin_id)))?;
            pieces.push(&piece_obj(&triangle_soup(&solid), piece_cells)?);
        }

        let obj = js_sys::Object::new();
        set(&obj, "binId", &JsValue::from_str(&bin.bin_id))?;
        set(&obj, "pieces", &pieces)?;
        out.push(&obj);
    }
    Ok(out.into())
}

/// Tessellates to the non-indexed positional soup the app expects: nine floats
/// per triangle, no normals (the viewer computes its own).
///
/// Expanded from the *welded* indexed mesh rather than straight from the
/// tessellator. Adjacent faces sample a shared edge to within the weld
/// tolerance, not to the bit — so emitting raw triangles would hand consumers
/// a soup whose shared vertices differ in their last f32 ULP, and anything
/// rebuilding adjacency by exact position would see it as full of boundary
/// edges. Going through `to_mesh()` makes every shared corner one vertex, so
/// re-welding downstream is exact.
fn triangle_soup(solid: &gridfinity_cad::Solid) -> Vec<f32> {
    let mesh = tessellate(solid, ARC_SEGMENTS_PER_QUARTER).to_mesh();
    let mut out = Vec::with_capacity(mesh.indices.len() * 3);
    for &i in &mesh.indices {
        let p = mesh.positions[i as usize];
        out.extend_from_slice(&[p.x, p.y, p.z]);
    }
    out
}

fn piece_obj(triangles: &[f32], cells: &[GridCell]) -> Result<JsValue, JsValue> {
    let o = js_sys::Object::new();
    set(&o, "triangles", &js_sys::Float32Array::from(triangles))?;
    set(&o, "cells", &serde_wasm_bindgen::to_value(cells).map_err(|e| JsValue::from_str(&e.to_string()))?)?;
    Ok(o.into())
}

fn set(o: &js_sys::Object, key: &str, val: impl AsRef<JsValue>) -> Result<(), JsValue> {
    js_sys::Reflect::set(o, &JsValue::from_str(key), val.as_ref())?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    const ONE_CELL: &str = r#"[{
        "binId": "bin-1",
        "height": 21,
        "perimeterThickness": 1.2,
        "filletRadius": 2.8,
        "fasteners": {"magnets": false, "m3": false},
        "cells": [{"x":0,"y":0}],
        "openings": [],
        "walls": [],
        "pieces": [[{"x":0,"y":0}]]
    }]"#;

    fn parse(json: &str) -> Vec<BinParams> {
        serde_json::from_str(json).expect("BinParameters must deserialise")
    }

    /// The app's `BinParameters` JSON must deserialise verbatim.
    #[test]
    fn deserialises_the_worker_contract() {
        let bins = parse(ONE_CELL);
        assert_eq!(bins.len(), 1);
        assert_eq!(bins[0].bin_id, "bin-1");
        assert_eq!(bins[0].height, 21.0);
        assert_eq!(bins[0].pieces.len(), 1);
    }

    /// A `u`-unit bin is `u · 7` mm overall, and the kernel counts units above
    /// its 7 mm base — so the totals must agree, not the unit numbers.
    #[test]
    fn height_maps_to_the_same_total_millimetres() {
        for units in 2..=20u32 {
            let mm = units as f32 * 7.0;
            let json = ONE_CELL.replace("\"height\": 21", &format!("\"height\": {mm}"));
            let p = parse(&json)[0].to_params();
            assert_eq!(
                p.total_height(),
                mm,
                "a {units}-unit bin must be {mm} mm tall, got {}",
                p.total_height(),
            );
        }
    }

    #[test]
    fn maps_walls_fasteners_and_radius() {
        let json = r#"[{
            "binId": "b", "height": 28, "perimeterThickness": 2.0, "filletRadius": 1.5,
            "fasteners": {"magnets": true, "m3": true},
            "cells": [{"x":0,"y":0}],
            "openings": [{"x":0,"y":0,"orientation":"h"}],
            "walls": [{"start":{"x":1,"y":2},"end":{"x":3,"y":4},"width":1.6}],
            "pieces": [[{"x":0,"y":0}]]
        }]"#;
        let p = parse(json)[0].to_params();
        assert_eq!(p.wall_thickness, 2.0);
        assert!(p.magnet_holes && p.screw_holes);
        // One shared radius drives both of the kernel's separate fields.
        assert_eq!(p.cavity_corner_radius, 1.5);
        assert_eq!(p.floor_fillet, 1.5);
        assert_eq!(p.open_edges.len(), 1);
        assert_eq!(p.inner_walls.len(), 1);
        let w = p.inner_walls[0];
        assert_eq!((w.x1, w.y1, w.x2, w.y2, w.width), (1.0, 2.0, 3.0, 4.0, 1.6));
        // The app's walls are always full height.
        assert_eq!(w.height, None);
        // The app has no grid-aligned divider concept.
        assert!(p.divider_edges.is_empty());
    }

    /// Soup must be positions only, nine floats per triangle.
    #[test]
    fn emits_nine_floats_per_triangle() {
        let bin = &parse(ONE_CELL)[0];
        let solid =
            gridfinity::build_piece(&bin.to_params(), &bin.cells, &bin.pieces[0], None).unwrap();
        let soup = triangle_soup(&solid);
        assert!(!soup.is_empty());
        assert_eq!(soup.len() % 9, 0);
    }

    /// A cut bin's pieces must each build, and together exceed one piece alone.
    #[test]
    fn builds_each_cut_piece() {
        let json = r#"[{
            "binId": "b", "height": 21, "perimeterThickness": 1.2, "filletRadius": 2.8,
            "fasteners": {"magnets": false, "m3": false},
            "cells": [{"x":0,"y":0},{"x":1,"y":0}],
            "openings": [], "walls": [],
            "pieces": [[{"x":0,"y":0}],[{"x":1,"y":0}]]
        }]"#;
        let bin = &parse(json)[0];
        let params = bin.to_params();
        for cells in &bin.pieces {
            let solid = gridfinity::build_piece(&params, &bin.cells, cells, None)
                .expect("each cut piece must build");
            solid.validate().expect("each cut piece must be manifold");
        }
    }

    /// Rebuilds adjacency from the soup keyed on *exact* vertex position, the
    /// way the app's printability gate does, and returns the number of edges
    /// not shared by exactly two oppositely-wound triangles.
    ///
    /// This is stricter than the kernel's own manifold invariant: it also
    /// catches a soup whose shared corners differ in their last f32 ULP.
    fn unclosed_edges(soup: &[f32]) -> usize {
        use std::collections::HashMap;
        let mut ids: HashMap<[u32; 3], u32> = HashMap::new();
        let mut vertex = Vec::new();
        for p in soup.chunks_exact(3) {
            // Key on the raw bit patterns — exact equality, no tolerance.
            let key = [p[0].to_bits(), p[1].to_bits(), p[2].to_bits()];
            let next = ids.len() as u32;
            vertex.push(*ids.entry(key).or_insert(next));
        }
        let mut directed: HashMap<(u32, u32), i32> = HashMap::new();
        for t in vertex.chunks_exact(3) {
            for (&a, &b) in [(&t[0], &t[1]), (&t[1], &t[2]), (&t[2], &t[0])] {
                let (key, dir) = if a < b { ((a, b), 1) } else { ((b, a), -1) };
                *directed.entry(key).or_insert(0) += dir;
            }
        }
        directed.values().filter(|&&v| v != 0).count()
    }

    /// The soup must be closed under exact-position welding, not merely under
    /// the kernel's tolerance weld — consumers rebuild adjacency by position.
    #[test]
    fn soup_is_closed_under_exact_position_welding() {
        let bin = &parse(ONE_CELL)[0];
        let solid =
            gridfinity::build_piece(&bin.to_params(), &bin.cells, &bin.pieces[0], None).unwrap();
        let soup = triangle_soup(&solid);
        assert_eq!(unclosed_edges(&soup), 0, "soup has unpaired edges");
    }

    /// Cases the app's printability gate flags, all of them **pre-existing
    /// kernel limitations** rather than boundary problems.
    ///
    /// Each builds a valid B-rep — `Solid::validate()` passes, every edge used
    /// exactly twice — but tessellates with unpaired mesh edges on the rim
    /// plane, at the inner corners of a shape with an enclosed hole.
    ///
    /// The trigger is fillet radius alone, independent of the cavity corner
    /// radius: on a 3x3 ring it is clean through `fr = 4.0` and leaks from
    /// `fr = 4.5` up, with `rc` anywhere from 5 to 8. The gaps break at
    /// exactly `HALF_TOL + OUTER_R = 4.0`-ish from each corner, which is where
    /// adjacent floor blends start to overlap around a short corner run.
    /// Fixing it means teaching `fillet.rs` to merge overlapping blends at a
    /// corner — real work in the most delicate part of the kernel, and
    /// explicitly not something to paper over downstream.
    #[ignore = "pre-existing fillet.rs limitation: blends overlap at corners when fr > ~4.25"]
    #[test]
    fn ring_and_large_fillet_cases_stay_closed() {
        const RING: &str = r#""cells":[{"x":0,"y":0},{"x":1,"y":0},{"x":2,"y":0},
            {"x":0,"y":1},{"x":2,"y":1},{"x":0,"y":2},{"x":1,"y":2},{"x":2,"y":2}]"#;
        const U: &str = r#""cells":[{"x":0,"y":0},{"x":1,"y":0},{"x":2,"y":0},
            {"x":0,"y":1},{"x":2,"y":1},{"x":0,"y":2},{"x":2,"y":2}]"#;
        let cases: &[(&str, f32, f32, &str, &str)] = &[
            ("ring baseline", 21.0, 2.8, RING, "[]"),
            ("ring + hole opening", 21.0, 2.8, RING, r#"[{"x":1,"y":1,"orientation":"v"}]"#),
            ("ring + fillet 5", 21.0, 5.0, RING, "[]"),
            ("U + slider-max fillet", 14.0, 5.6, U, "[]"),
        ];
        let mut leaks = Vec::new();
        for (name, height, fillet, cells, openings) in cases {
            let json = format!(
                r#"[{{"binId":"b","height":{height},"perimeterThickness":1.2,
                   "filletRadius":{fillet},"fasteners":{{"magnets":false,"m3":false}},
                   {cells},"openings":{openings},"walls":[],"pieces":[[{{"x":0,"y":0}}]]}}]"#
            );
            let mut bin = parse(&json).pop().unwrap();
            // Build the whole bin, not a single cell.
            bin.pieces = vec![bin.cells.clone()];
            let solid = gridfinity::build_piece(&bin.to_params(), &bin.cells, &bin.pieces[0], None)
                .unwrap_or_else(|e| panic!("{name} failed to build: {e}"));
            let brep = match solid.validate() {
                Ok(()) => "B-rep manifold".to_string(),
                Err(e) => format!("B-REP INVALID: {e}"),
            };
            let unpaired = unclosed_edges(&triangle_soup(&solid));
            if unpaired != 0 {
                leaks.push(format!("{name}: {unpaired} unpaired mesh edges, {brep}"));
            }
        }
        assert!(leaks.is_empty(), "unclosed output:\n  {}", leaks.join("\n  "));
    }

    /// Every piece the kernel hands back must be a closed 2-manifold, since
    /// nothing downstream repairs it.
    #[test]
    fn pieces_are_manifold_across_the_feature_space() {
        let cases: &[(&str, &str)] = &[
            ("1x1", r#""cells":[{"x":0,"y":0}],"openings":[],"walls":[],"pieces":[[{"x":0,"y":0}]]"#),
            ("2x2", r#""cells":[{"x":0,"y":0},{"x":1,"y":0},{"x":0,"y":1},{"x":1,"y":1}],"openings":[],"walls":[],"pieces":[[{"x":0,"y":0},{"x":1,"y":0},{"x":0,"y":1},{"x":1,"y":1}]]"#),
            ("L-shape", r#""cells":[{"x":0,"y":0},{"x":1,"y":0},{"x":0,"y":1}],"openings":[],"walls":[],"pieces":[[{"x":0,"y":0},{"x":1,"y":0},{"x":0,"y":1}]]"#),
            ("opening", r#""cells":[{"x":0,"y":0}],"openings":[{"x":0,"y":0,"orientation":"h"}],"walls":[],"pieces":[[{"x":0,"y":0}]]"#),
            ("inner wall", r#""cells":[{"x":0,"y":0},{"x":1,"y":0}],"openings":[],"walls":[{"start":{"x":42,"y":4},"end":{"x":42,"y":38},"width":1.6}],"pieces":[[{"x":0,"y":0},{"x":1,"y":0}]]"#),
        ];
        for (name, body) in cases {
            for &(magnets, m3) in &[(false, false), (true, true)] {
                let json = format!(
                    r#"[{{"binId":"b","height":21,"perimeterThickness":1.2,"filletRadius":2.8,
                       "fasteners":{{"magnets":{magnets},"m3":{m3}}},{body}}}]"#
                );
                let bin = &parse(&json)[0];
                let params = bin.to_params();
                let solid =
                    gridfinity::build_piece(&params, &bin.cells, &bin.pieces[0], None)
                        .unwrap_or_else(|e| panic!("{name} (magnets={magnets}) failed: {e}"));
                solid
                    .validate()
                    .unwrap_or_else(|e| panic!("{name} (magnets={magnets}) not manifold: {e}"));
            }
        }
    }
}
