
use gridfinity_cad::gridfinity::{
    self, BASE_TOTAL_HEIGHT, HEIGHT_PER_UNIT, InnerWall, LogicalBin, Params,
};
use gridfinity_cad::layout::{GridCell, GridEdge};
use gridfinity_cad::tessellate;
use glam::Vec3;
use gridfinity_render::{append_smooth_shaded, color_of};
use wasm_bindgen::prelude::*;

#[cfg(target_arch = "wasm32")]
mod viewer;

const ARC_SEGMENTS_PER_QUARTER: usize = 16;

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

#[derive(serde::Deserialize)]
#[serde(rename_all = "camelCase")]
struct BinParams {
    bin_id: String,
    height: f32,
    perimeter_thickness: f32,
    fillet_radius: f32,
    fasteners: Fasteners,
    cells: Vec<GridCell>,
    openings: Vec<GridEdge>,
    walls: Vec<Wall>,
    pieces: Vec<Vec<GridCell>>,
}

#[derive(serde::Deserialize)]
struct Fasteners {
    magnets: bool,
    m3: bool,
}

impl BinParams {
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
            divider_edges: Vec::new(),
            inner_walls: self.walls.iter().map(|w| InnerWall {
                x1: w.start.x,
                y1: w.start.y,
                x2: w.end.x,
                y2: w.end.y,
                width: w.width,
                height: None,
            }).collect(),
            mode: gridfinity::Mode::Bin,
        }
    }
}

#[wasm_bindgen(start)]
pub fn start() {
    console_error_panic_hook::set_once();
}

#[wasm_bindgen]
pub fn generate_geometry(bins: JsValue) -> Result<JsValue, JsValue> {
    let bins: Vec<BinParams> = serde_wasm_bindgen::from_value(bins)
        .map_err(|e| JsValue::from_str(&format!("invalid bin parameters: {e}")))?;

    let out = js_sys::Array::new();
    for bin in &bins {
        let params = bin.to_params();
        let pieces = js_sys::Array::new();

        for piece_cells in &bin.pieces {
            let solid = gridfinity::build_piece(&params, &bin.cells, piece_cells, None)
                .map_err(|e| JsValue::from_str(&format!("bin {}: {e}", bin.bin_id)))?;
            pieces.push(&piece_obj(&render_vertices(&solid, ARC_SEGMENTS_PER_QUARTER), piece_cells)?);
        }

        let obj = js_sys::Object::new();
        set(&obj, "binId", &JsValue::from_str(&bin.bin_id))?;
        set(&obj, "pieces", &pieces)?;
        out.push(&obj);
    }
    Ok(out.into())
}

#[wasm_bindgen]
pub fn badapple_frame_count() -> usize {
    gridfinity_cad::badapple::frame_count()
}

#[wasm_bindgen]
pub fn badapple_fps() -> f64 {
    gridfinity_cad::badapple::FPS
}

#[wasm_bindgen]
pub fn badapple_bounds() -> js_sys::Float32Array {
    let (min, max) = gridfinity_cad::badapple::bounds();
    js_sys::Float32Array::from(&[min[0], min[1], min[2], max[0], max[1], max[2]][..])
}

#[wasm_bindgen]
pub fn badapple_frame_vertices(index: usize, rgb: u32) -> js_sys::Float32Array {
    let params = gridfinity_cad::badapple::cell_params();
    let color = color_of(rgb);
    let mut verts: Vec<f32> = Vec::new();
    let mut stage = |kernel: &[f32]| {
        append_smooth_shaded(&mut verts, kernel, Vec3::ZERO, color, false);
    };
    for cells in gridfinity_cad::badapple::components(gridfinity_cad::badapple::frame(index)) {
        match gridfinity::build_piece(&params, &cells, &cells, None) {
            Ok(solid) => stage(&render_vertices(&solid, 1)),
            Err(_) => {
                for cell in &cells {
                    let one = [*cell];
                    if let Ok(solid) = gridfinity::build_piece(&params, &one, &one, None) {
                        stage(&render_vertices(&solid, 1));
                    }
                }
            }
        }
    }
    js_sys::Float32Array::from(&verts[..])
}

fn render_vertices(solid: &gridfinity_cad::Solid, arc_segments: usize) -> Vec<f32> {
    tessellate(solid, arc_segments).welded_render_buffer()
}

fn piece_obj(vertices: &[f32], cells: &[GridCell]) -> Result<JsValue, JsValue> {
    let o = js_sys::Object::new();
    set(&o, "vertices", &js_sys::Float32Array::from(vertices))?;
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

    #[test]
    fn deserialises_the_worker_contract() {
        let bins = parse(ONE_CELL);
        assert_eq!(bins.len(), 1);
        assert_eq!(bins[0].bin_id, "bin-1");
        assert_eq!(bins[0].height, 21.0);
        assert_eq!(bins[0].pieces.len(), 1);
    }

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
        assert_eq!(p.cavity_corner_radius, 1.5);
        assert_eq!(p.floor_fillet, 1.5);
        assert_eq!(p.open_edges.len(), 1);
        assert_eq!(p.inner_walls.len(), 1);
        let w = p.inner_walls[0];
        assert_eq!((w.x1, w.y1, w.x2, w.y2, w.width), (1.0, 2.0, 3.0, 4.0, 1.6));
        assert_eq!(w.height, None);
        assert!(p.divider_edges.is_empty());
    }

    #[test]
    fn emits_whole_triangles_of_position_plus_normal_vertices() {
        let bin = &parse(ONE_CELL)[0];
        let solid =
            gridfinity::build_piece(&bin.to_params(), &bin.cells, &bin.pieces[0], None).unwrap();
        let verts = render_vertices(&solid, ARC_SEGMENTS_PER_QUARTER);
        assert!(!verts.is_empty());
        assert_eq!(verts.len() % (3 * gridfinity_render::KERNEL_STRIDE), 0);
    }

    #[test]
    fn every_emitted_vertex_carries_a_unit_normal() {
        let bin = &parse(ONE_CELL)[0];
        let solid =
            gridfinity::build_piece(&bin.to_params(), &bin.cells, &bin.pieces[0], None).unwrap();
        for v in render_vertices(&solid, ARC_SEGMENTS_PER_QUARTER)
            .chunks_exact(gridfinity_render::KERNEL_STRIDE)
        {
            let n = Vec3::new(v[3], v[4], v[5]).length();
            assert!((n - 1.0).abs() < 1e-3, "normal length {n} is not unit");
        }
    }

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

    fn unclosed_edges(verts: &[f32]) -> usize {
        use std::collections::HashMap;
        let mut ids: HashMap<[u32; 3], u32> = HashMap::new();
        let mut vertex = Vec::new();
        for p in verts.chunks_exact(gridfinity_render::KERNEL_STRIDE) {
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

    #[test]
    fn the_render_buffer_is_closed_under_exact_position_welding() {
        let bin = &parse(ONE_CELL)[0];
        let solid =
            gridfinity::build_piece(&bin.to_params(), &bin.cells, &bin.pieces[0], None).unwrap();
        let verts = render_vertices(&solid, ARC_SEGMENTS_PER_QUARTER);
        assert_eq!(unclosed_edges(&verts), 0, "render buffer has unpaired edges");
    }

    #[test]
    fn ring_and_large_fillet_cases_stay_closed() {
        const RING: &str = r#""cells":[{"x":0,"y":0},{"x":1,"y":0},{"x":2,"y":0},
            {"x":0,"y":1},{"x":2,"y":1},{"x":0,"y":2},{"x":1,"y":2},{"x":2,"y":2}]"#;
        const U: &str = r#""cells":[{"x":0,"y":0},{"x":1,"y":0},{"x":2,"y":0},
            {"x":0,"y":1},{"x":2,"y":1},{"x":0,"y":2},{"x":2,"y":2}]"#;
        let cases: &[(&str, f32, f32, &str, &str)] = &[
            ("ring baseline", 21.0, 2.8, RING, "[]"),
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
            bin.pieces = vec![bin.cells.clone()];
            let solid = gridfinity::build_piece(&bin.to_params(), &bin.cells, &bin.pieces[0], None)
                .unwrap_or_else(|e| panic!("{name} failed to build: {e}"));
            let brep = match solid.validate() {
                Ok(()) => "B-rep manifold".to_string(),
                Err(e) => format!("B-REP INVALID: {e}"),
            };
            let unpaired = unclosed_edges(&render_vertices(&solid, ARC_SEGMENTS_PER_QUARTER));
            if unpaired != 0 {
                leaks.push(format!("{name}: {unpaired} unpaired mesh edges, {brep}"));
            }
        }
        assert!(leaks.is_empty(), "unclosed output:\n  {}", leaks.join("\n  "));
    }

    #[ignore = "open/seam rim assembly double-subtracts a hole an opening merged into the cavity"]
    #[test]
    fn opening_on_a_hole_boundary_stays_closed() {
        const RING: &str = r#""cells":[{"x":0,"y":0},{"x":1,"y":0},{"x":2,"y":0},
            {"x":0,"y":1},{"x":2,"y":1},{"x":0,"y":2},{"x":1,"y":2},{"x":2,"y":2}]"#;
        let json = format!(
            r#"[{{"binId":"b","height":21,"perimeterThickness":1.2,
               "filletRadius":2.8,"fasteners":{{"magnets":false,"m3":false}},
               {RING},"openings":[{{"x":1,"y":1,"orientation":"v"}}],"walls":[],
               "pieces":[[{{"x":0,"y":0}}]]}}]"#
        );
        let mut bin = parse(&json).pop().unwrap();
        bin.pieces = vec![bin.cells.clone()];
        let solid =
            gridfinity::build_piece(&bin.to_params(), &bin.cells, &bin.pieces[0], None).unwrap();
        solid.validate().expect("B-rep stays manifold even while the mesh leaks");
        assert_eq!(unclosed_edges(&render_vertices(&solid, ARC_SEGMENTS_PER_QUARTER)), 0, "rim leaks at the opening");
    }

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
