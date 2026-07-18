# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A minimalistic **analytic-surface B-rep CAD kernel** (not CSG) plus a parametric **Gridfinity**
model built on it, with an **egui** front-end that previews the model in 3D and exports binary STL.
It reproduces the purpose of the TypeScript reference at `../gridfinity-expanded` (parametric
Gridfinity generator + watertight STL) as a small Rust workspace.

## Commands

```bash
cargo build                      # build both crates
cargo test -p gridfinity-cad     # engine unit tests (geometry correctness lives here)
cargo test -p gridfinity-cad <name> -- --nocapture   # run one test, e.g. default_bin_is_valid_watertight_and_sized
cargo run  -p gridfinity-gui     # launch the app (needs a display + OpenGL/glow)
cargo build --release
```

The GUI is `windows_subsystem="windows"` in release, so it opens a window and blocks. To smoke-test
that it starts without panicking (shader compile / GL context / first mesh upload all happen at
startup): `timeout 6 ./target/debug/gridfinity-gui.exe`.

## Workspace layout

Two crates (`Cargo.toml` = virtual workspace, edition 2024, resolver 3):

- **`crates/gridfinity-cad`** — the engine library. Deps: `glam` (math) + `earcutr` (used *only*
  for final planar-face-with-holes triangulation; the B-rep kernel itself is hand-rolled).
- **`crates/gridfinity-gui`** — the eframe/egui/glow app. Depends on `gridfinity-cad`.

## Engine architecture (the big picture)

Pipeline: **`sketch` → `build` (features) → `topo` (B-rep solid) → `fillet` → `tess` → `mesh` → STL.**

- **`geom.rs`** — analytic `Surface` (`Plane`/`Cylinder`/`Cone`/`Torus`/`Sphere`) and `Curve`
  (`Line`/`Circle`). Every radial surface and `Circle` carries an explicit `axis` (arbitrary
  direction); the `*_z` convenience constructors (`cylinder_z`, `cone_z`, `torus_z`,
  `Curve::circle_z`) cover the common +Z case, and everything stays closed-form. Each surface has
  `point`/`normal`/`project(uv)`; partial surfaces set `ref_dir` to their arc start so `project`
  angles stay wrap-free.
- **`topo.rs`** — the B-rep: `Vertex`/`Edge`/`Loop`/`Face`/`Solid`. **Not** half-edge with
  next/prev pointers — loops are explicit ordered `(EdgeId, forward)` lists, edges are shared.
  `Builder` interns vertices and edges (edge key = sorted endpoints **+ welded midpoint**, so a
  circle's two semicircle arcs don't collapse into one edge). `Solid::validate()` enforces the
  manifold invariant: **every edge used exactly twice, once in each direction** — assert it in
  tests after any construction change.
- **`sketch.rs`** — 2D profiles as closed loops of `Line`/`Arc` segments (`rectangle`,
  `rounded_rect`, `circle`). Corner radii are real arcs. Outer loops CCW.
- **`build.rs`** — features. Three primitives write into a shared `Builder`: `ring` (profile at a
  height), `wall_between` (side faces between two rings), `cap`/`loop_of` (planar caps).
  `extrude`/`prism`/`loft` wrap them. **Orientation convention:** author loops CCW; an `outward`
  flag says whether material is inside the loop (`true`) or it is a hole/cavity (`false`). `loft`
  turns arcs whose radius changes with height into `Cone` faces; a straight segment on a loft
  becomes a *slanted* `Plane` (its normal is computed from the actual 3D quad, not assumed
  vertical).
- **`fillet.rs`** — `blend_edges(&solid, &[(EdgeId, r)])`: true rolling-ball edge blending as a
  B-rep operator. Plane/plane edges become `Cylinder` blends, plane/coaxial-cylinder circle edges
  become `Torus` blends; adjacent faces are trimmed back to the exact tangent curves and quarter-
  circle connect arcs join neighbouring blends. Every blended vertex must be shared by exactly two
  blended edges (closed smooth chain; spherical corner patches unimplemented → `Err`). The whole
  solid is rebuilt through a fresh `Builder` and `validate()`d. Gotcha: when re-emitting an arc
  reversed, the angle range must be swapped too — `Builder::arc` trusts that its first vertex sits
  at the first angle.
- **`tess.rs`** — analytic faces → triangles. **Watertight by construction:** each edge is sampled
  once (cached by `EdgeId`), so the two faces sharing it emit identical boundary points. Winding is
  decided **once per face** (area-weighted vote against the analytic normal) — never per triangle,
  or curved faces get inconsistent internal edges. Non-planar 4-sided faces whose loop follows
  iso-u/iso-v lines (cylinder walls, cone chamfers, blend patches) take a structured-grid path
  (avoids earcut slivers from collinear boundary runs); everything else, including planar-with-
  holes, goes through `earcutr` with a zero-uv-area sliver filter, followed by
  `split_boundary_chords`: earcut drops collinear boundary vertices, so triangle edges that chord
  across boundary samples the neighbouring face emits individually are fanned back through the
  dropped points (only through vertices earcut didn't use — inserting used ones would duplicate
  triangles).
- **`rectregion.rs`** — rectilinear region engine: unions/differences of axis-aligned rects
  resolved on a compressed coordinate grid, traced material-on-the-left (outer CCW, holes CW), then
  per-corner arc rounding with clamping (`trace_rects`, `shape_loop`). The cavity planner and
  baseplate outline are built on it.
- **`gridfinity.rs`** — the parametric model + spec constants + `Params`, a faithful port of the
  TS reference's `BinConfig`. `Params.bins: Vec<LogicalBin>` holds polyomino cell sets (plus
  optional floor slope); `Params::rect(gx,gy)` is the rectangular convenience. **Each bin is built
  in one `Builder`** so interface edges are shared automatically — there is *no general boolean*.
  Model structure: a boundary walk traces each bin's cells into loops; the outer profile is the
  pitch lines inset `HALF_TOL=0.25` with `OUTER_R=3.75` convex corners, split at
  `PEG_TANGENT=4.0` from corners so peg-top edges weld with the wall's bottom ring. The base is
  **one chamfered connector peg per cell** (three lofted profiles, 0 → `PEG_HEIGHT=4.75`;
  `PEG_R_MID=1.6` so all three corner arcs share an axis and the chamfers are coaxial cones). Peg
  ring segments that don't weld to the wall, plus non-shared outer pieces, are stitched
  (`stitch_loops`) into planar bridge-underside faces at `PEG_HEIGHT` — loops geometrically
  contained in another become holes of that face (an interior peg's ring must be a hole, not a
  disk). The cavity plan (`plan_cavity`) mirrors TS `planCavity`: cell rects minus wall/divider
  strips plus concave-corner patches, traced through `rectregion` and rounded (`rc` convex, `fr`
  concave so the floor-fillet blend stays tangent-continuous). Bins are `42·n − 0.5` mm, the
  `Baseplate` is full `42·n` with a peg-shaped through-socket per cell; concentric magnet+screw
  becomes a stepped counterbore.

The compartment cavities are built **sharp**; the concave floor fillet is applied afterwards as a
true `blend_edges` rolling-ball blend over each compartment's floor-wall edge loop (skipped when
the clamped radius is 0 or the floor is sloped).

When adding geometry, keep the manifold invariant: any edge a new face introduces must be paired by
exactly one other face traversing it the opposite way. `Params` currently drives grid size, height,
wall/corner/fillet, magnet/screw holes, compartments/divider edges, floor slope, and Bin/Baseplate
mode. (Open-edges are intentionally not implemented in the constructive model.)

## GUI notes (important API gotcha)

This project pins **egui/eframe/egui_glow 0.35, which is a redesigned API**, not mainstream egui:

- `eframe::App::ui(&mut self, ui: &mut egui::Ui, frame)` — you get a root **`Ui`**, not a `Context`
  (there is no `update(ctx, ...)`).
- Panels are shown *inside* that root ui: `egui::Panel::left(id).show(ui, ...)` / `Panel::right` /
  `CentralPanel::default().show(ui, ...)`. There is **no `SidePanel`**.
- Scroll delta is `input.smooth_scroll_delta` (no `raw_scroll_delta`).
- `NativeOptions` still has `depth_buffer`/`renderer`/`viewport` like mainstream.

`viewport.rs` renders the mesh with one glow shader (smooth shading from analytic vertex normals)
via `egui::PaintCallback` + `egui_glow::CallbackFn`, inside a scissored depth-cleared, back-face
culled draw that restores GL state. Back-face culling relies on the engine's outward winding (see
the `meshes_have_outward_consistent_winding` test). `main.rs` binds `Params` to widgets, regenerates
(build → tessellate → upload VBO) on change, and exports STL via `rfd` + `Mesh::to_stl_binary`.
