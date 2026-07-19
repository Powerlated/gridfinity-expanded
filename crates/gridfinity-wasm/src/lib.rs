//! WebAssembly bindings exposing the Gridfinity model to the web app.
//!
//! The boundary is deliberately narrow: JSON in (the reference's `BinConfig`,
//! deserialised straight into [`Params`] — see the serde attributes there), and
//! `{vertProperties, triVerts}` meshes out, which is exactly the `BinMesh`
//! shape the app's worker already consumes. Nothing else about the kernel is
//! exposed, and nothing about the web app leaks in here.
//!
//! ## Coordinate handedness
//!
//! The app's shape editors map SVG y (downward) straight onto mm +y, so a part
//! built in ordinary right-handed coordinates would print as the chiral mirror
//! of the drawn layout. The reference compensates by mirroring every output
//! mesh across Y on the way out; [`MeshData::mirror_y`] reproduces that, and it
//! flips triangle winding to match, because mirroring reverses handedness and
//! an unflipped mesh would render (and slice) inside-out.

use gridfinity_cad::gridfinity::{self, GRID_PITCH, Params};
use gridfinity_cad::tessellate;
use wasm_bindgen::prelude::*;

/// A welded indexed mesh, flattened into the pair of typed-array-ready buffers
/// the web app calls `BinMesh`.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct MeshData {
    /// Flat xyz, three per vertex.
    pub positions: Vec<f32>,
    /// Three vertex indices per triangle.
    pub indices: Vec<u32>,
}

impl MeshData {
    /// Translates in the XY plane, leaving z alone.
    pub fn translate_xy(&mut self, dx: f32, dy: f32) {
        for v in self.positions.chunks_exact_mut(3) {
            v[0] += dx;
            v[1] += dy;
        }
    }

    /// Mirrors across the XZ plane (`y → offset − y`).
    ///
    /// Reflection reverses orientation, so every triangle's winding is flipped
    /// to keep normals pointing out of the solid.
    pub fn mirror_y(&mut self, offset: f32) {
        for v in self.positions.chunks_exact_mut(3) {
            v[1] = offset - v[1];
        }
        for t in self.indices.chunks_exact_mut(3) {
            t.swap(1, 2);
        }
    }

    /// Axis-aligned bounds as `(min, max)` over x and y; `None` when empty.
    pub fn xy_bounds(&self) -> Option<(f32, f32, f32, f32)> {
        let mut it = self.positions.chunks_exact(3);
        let first = it.next()?;
        let (mut lo_x, mut lo_y, mut hi_x, mut hi_y) = (first[0], first[1], first[0], first[1]);
        for v in it {
            lo_x = lo_x.min(v[0]);
            lo_y = lo_y.min(v[1]);
            hi_x = hi_x.max(v[0]);
            hi_y = hi_y.max(v[1]);
        }
        Some((lo_x, lo_y, hi_x, hi_y))
    }
}

/// The `id` field of the reference's `LogicalBin`, which is UI-owned and so is
/// not part of the kernel's [`gridfinity::LogicalBin`]. The viewer colour-matches
/// previews to the editors by it, so it is read separately and passed through.
#[derive(serde::Deserialize, Default)]
struct BinIdentity {
    #[serde(default)]
    id: Option<i32>,
}

#[derive(serde::Deserialize, Default)]
struct ConfigIdentity {
    #[serde(default)]
    bins: Vec<BinIdentity>,
}

#[wasm_bindgen(start)]
pub fn start() {
    // Turns a Rust panic into a readable console error instead of an opaque
    // "unreachable executed" trap. Cheap, and the generate path is otherwise
    // fallible-by-construction (see `try_build_pieces`).
    console_error_panic_hook::set_once();
}

/// Builds every printable piece of `config`, split-aware.
///
/// `res` is the curve resolution (segments per 90° arc): low for a live
/// preview, high for export.
///
/// Returns `{ pieces: [{name, col, row, mesh}], previews: [{bin, piece,
/// pieceCount, mesh}] }`. `pieces` are piece-local (bbox min at the origin,
/// print-ready); `previews` stay in whole-layout coordinates so the viewer can
/// assemble them. Both are mirrored to match the editors.
#[wasm_bindgen]
pub fn generate_bin_pieces(config: JsValue, res: usize) -> Result<JsValue, JsValue> {
    let params: Params = serde_wasm_bindgen::from_value(config.clone())
        .map_err(|e| JsValue::from_str(&format!("invalid config: {e}")))?;
    let ids: ConfigIdentity = serde_wasm_bindgen::from_value(config).unwrap_or_default();

    let built = gridfinity::try_build_pieces(&params).map_err(|e| JsValue::from_str(&e))?;

    // Whole-layout height, the mirror axis shared by every preview so the
    // assembled layout stays coherent. Derived from all cells, not per piece.
    let layout_h = params
        .all_cells()
        .iter()
        .map(|c| c.y)
        .max()
        .map_or(0.0, |max_y| (max_y + 1) as f32 * GRID_PITCH);

    let pieces = js_sys::Array::new();
    let previews = js_sys::Array::new();

    for bp in &built {
        let mesh = to_mesh_data(&tessellate(&bp.solid, res).to_mesh());

        // Preview: assembled layout coordinates.
        let mut preview = mesh.clone();
        preview.mirror_y(layout_h);
        let bin_id = ids
            .bins
            .get(bp.bin)
            .and_then(|b| b.id)
            .unwrap_or(bp.bin as i32);
        previews.push(&preview_obj(bin_id, bp.piece, bp.piece_count, &preview)?);

        // Export piece: dropped to the origin, then mirrored about its own
        // height so it is print-ready standalone.
        let mut piece = mesh;
        if let Some((lo_x, lo_y, _, hi_y)) = piece.xy_bounds() {
            piece.translate_xy(-lo_x, -lo_y);
            piece.mirror_y(hi_y - lo_y);
        }
        pieces.push(&piece_obj(&bp.name, bp.col, bp.row, &piece)?);
    }

    let out = js_sys::Object::new();
    set(&out, "pieces", &pieces)?;
    set(&out, "previews", &previews)?;
    Ok(out.into())
}

fn to_mesh_data(mesh: &gridfinity_cad::Mesh) -> MeshData {
    let mut positions = Vec::with_capacity(mesh.positions.len() * 3);
    for p in &mesh.positions {
        positions.extend_from_slice(&[p.x, p.y, p.z]);
    }
    MeshData { positions, indices: mesh.indices.clone() }
}

/// `{vertProperties: Float32Array, triVerts: Uint32Array}` — real typed arrays,
/// not plain JS arrays, so the app can transfer and reinterpret them directly.
fn mesh_obj(m: &MeshData) -> Result<JsValue, JsValue> {
    let o = js_sys::Object::new();
    set(&o, "vertProperties", &js_sys::Float32Array::from(&m.positions[..]))?;
    set(&o, "triVerts", &js_sys::Uint32Array::from(&m.indices[..]))?;
    Ok(o.into())
}

fn piece_obj(name: &str, col: i32, row: i32, m: &MeshData) -> Result<JsValue, JsValue> {
    let o = js_sys::Object::new();
    set(&o, "name", &JsValue::from_str(name))?;
    set(&o, "col", &JsValue::from(col))?;
    set(&o, "row", &JsValue::from(row))?;
    set(&o, "mesh", &mesh_obj(m)?)?;
    Ok(o.into())
}

fn preview_obj(bin: i32, piece: usize, count: usize, m: &MeshData) -> Result<JsValue, JsValue> {
    let o = js_sys::Object::new();
    set(&o, "bin", &JsValue::from(bin))?;
    set(&o, "piece", &JsValue::from(piece as u32))?;
    set(&o, "pieceCount", &JsValue::from(count as u32))?;
    set(&o, "mesh", &mesh_obj(m)?)?;
    Ok(o.into())
}

fn set(o: &js_sys::Object, key: &str, val: impl AsRef<JsValue>) -> Result<(), JsValue> {
    js_sys::Reflect::set(o, &JsValue::from_str(key), val.as_ref())?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tri(positions: Vec<f32>) -> MeshData {
        MeshData { positions, indices: vec![0, 1, 2] }
    }

    #[test]
    fn mirror_y_reflects_and_flips_winding() {
        let mut m = tri(vec![0.0, 0.0, 0.0, 1.0, 2.0, 0.0, 0.0, 4.0, 0.0]);
        m.mirror_y(10.0);
        assert_eq!(m.positions, vec![0.0, 10.0, 0.0, 1.0, 8.0, 0.0, 0.0, 6.0, 0.0]);
        // Winding must reverse, or the reflected solid renders inside-out.
        assert_eq!(m.indices, vec![0, 2, 1]);
    }

    #[test]
    fn mirror_y_is_an_involution_on_positions() {
        let original = tri(vec![1.0, 3.0, 5.0, 2.0, 7.0, 1.0, -4.0, 0.5, 2.0]);
        let mut m = original.clone();
        m.mirror_y(10.0);
        m.mirror_y(10.0);
        assert_eq!(m, original);
    }

    #[test]
    fn translate_xy_leaves_z_alone() {
        let mut m = tri(vec![0.0, 0.0, 9.0, 1.0, 2.0, 9.0, 0.0, 4.0, 9.0]);
        m.translate_xy(-1.0, 5.0);
        assert_eq!(m.positions, vec![-1.0, 5.0, 9.0, 0.0, 7.0, 9.0, -1.0, 9.0, 9.0]);
    }

    #[test]
    fn xy_bounds_spans_all_vertices() {
        let m = tri(vec![3.0, -1.0, 0.0, -2.0, 4.0, 0.0, 8.0, 0.5, 0.0]);
        assert_eq!(m.xy_bounds(), Some((-2.0, -1.0, 8.0, 4.0)));
        assert_eq!(MeshData::default().xy_bounds(), None);
    }

    /// The reference's `BinConfig` JSON must deserialise as-is: camelCase keys,
    /// string enums, `innerFilletRadius`, a null inner-wall height, and the
    /// UI-only `id`/`isManual` ignored rather than rejected.
    #[test]
    fn deserialises_the_reference_bin_config() {
        let json = r#"{
            "bins": [{
                "id": 7,
                "cells": [{"x":0,"y":0},{"x":1,"y":0}],
                "isManual": true,
                "splitLines": [{"axis":"x","index":1}],
                "slope": {"angle": 12.5, "dir": "-y"}
            }],
            "heightUnits": 4,
            "wallThickness": 1.2,
            "cavityCornerRadius": 2.5,
            "innerFilletRadius": 3.0,
            "magnetHoles": true,
            "screwHoles": false,
            "openEdges": [{"x":0,"y":1,"orientation":"h"}],
            "dividerEdges": [{"x":1,"y":0,"orientation":"v"}],
            "innerWalls": [{"x1":0,"y1":0,"x2":10,"y2":10,"width":1.6,"height":null}]
        }"#;

        let p: Params = serde_json::from_str(json).expect("reference config must deserialise");
        assert_eq!(p.height_units, 4);
        assert_eq!(p.floor_fillet, 3.0);
        assert!(p.magnet_holes && !p.screw_holes);
        assert_eq!(p.bins.len(), 1);
        assert_eq!(p.bins[0].cells.len(), 2);
        assert_eq!(p.bins[0].split_lines[0].axis, gridfinity_cad::layout::Axis::X);
        assert_eq!(p.bins[0].slope.unwrap().dir, gridfinity::SlopeDir::MinusY);
        assert_eq!(p.open_edges[0].orientation, gridfinity_cad::layout::Orientation::H);
        assert_eq!(p.divider_edges[0].orientation, gridfinity_cad::layout::Orientation::V);
        assert_eq!(p.inner_walls[0].height, None);
        // `mode` has no TS counterpart and must fall back rather than fail.
        assert_eq!(p.mode, gridfinity::Mode::Bin);

        let ids: ConfigIdentity = serde_json::from_str(json).unwrap();
        assert_eq!(ids.bins[0].id, Some(7));
    }

    /// End-to-end: the default config must produce a real, non-degenerate mesh.
    #[test]
    fn builds_pieces_for_the_default_config() {
        let p = Params::default();
        let built = gridfinity::try_build_pieces(&p).expect("default config must build");
        assert_eq!(built.len(), 1);
        let m = to_mesh_data(&tessellate(&built[0].solid, 4).to_mesh());
        assert!(!m.positions.is_empty() && m.indices.len() % 3 == 0);
    }
}
