# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A minimalistic **analytic-surface B-rep CAD kernel** (not CSG) plus a parametric **Gridfinity**
model built on it, with an **egui** front-end that previews the model in 3D and exports binary STL.
It reproduces the purpose of the TypeScript reference at `../gridfinity-expanded` (parametric
Gridfinity generator + watertight STL) as a small Rust workspace.

## Hard rule: no mesh operations before export/render

**Triangles are a terminal output format, never a modelling medium.** Everything upstream of
`tess.rs` is analytic B-rep — exact surfaces, exact curves, closed-form intersections — and the
mesh is produced once, at the very end, and never read back.

Prohibited anywhere in the modelling pipeline (`sketch` → `build` → `topo` → `fillet`, plus
`gridfinity.rs`, `region2d.rs`, `rectregion.rs`, `region.rs`):

- **Mesh booleans / CSG** of any kind, including pulling in a library for them.
- **Voxels, SDFs, marching cubes, point sampling,** or any discretised volume representation.
- **Remeshing, mesh healing, decimation, vertex-merge-to-fix-gaps,** or any "tessellate first,
  then repair the result" strategy. Watertightness is a property of the B-rep, guaranteed by the
  manifold invariant and by `tess.rs` sampling each edge exactly once — it is never something a
  post-process patches up.
- **Numerically approximating a curve or surface** (polyline/facet stand-ins) where a closed-form
  `Curve`/`Surface` is the correct answer. If the needed analytic primitive doesn't exist yet, add
  it to `geom.rs` — `Curve::Ellipse` was added exactly this way.
- **Reading a `Mesh` back into modelling.** `Mesh` is write-only downstream of `tessellate`.

Explicitly still allowed, because they sit *at or after* the tessellation boundary:

- `earcutr` inside `tess.rs`, for final planar-face-with-holes triangulation only.
- `weld_triangles`, `to_stl_binary`, `flat_vertices`, bounds — export and GL upload.
- Mesh-based *verification* in tests (`assert_watertight`, `signed_volume`): checking the analytic
  result, never producing it.

If a task looks like it needs a prohibited operation, that means either a missing analytic
primitive or a missing B-rep operator (e.g. blend runout in `fillet.rs`). **Stop and ask** — do not
reach for a mesh fallback.

## Commands

```bash
cargo build                      # build both crates
cargo test -p gridfinity-cad     # engine unit tests (geometry correctness lives here)
cargo test -p gridfinity-cad <name> -- --nocapture   # run one test, e.g. default_bin_is_valid_watertight_and_sized
cargo run  -p gridfinity-gui     # launch the app (needs a display + OpenGL/glow)
cargo build --release

# The geometry fuzzer (tests/fuzz.rs): random Params -> try_build -> validate -> audit
# -> tessellation_leaks, with failures grouped by signature and each shrunk to a
# paste-ready `Params` literal. Both profiles are #[ignore]d — they are tools, not gates.
FUZZ_CASES=2000 cargo test -p gridfinity-cad --test fuzz -- --ignored --nocapture
FUZZ_SEED=7 FUZZ_CASES=500 cargo test -p gridfinity-cad --test fuzz -- --ignored --nocapture
```

`fuzz_inner_walls` covers free-form inner walls (the divider/fillet work); `fuzz_params_broad`
covers shape, height, thicknesses, holes, dividers, slope and mode. Baseline at the default seed
is **52/150 failing, 6 distinct defects** — drop the `#[ignore]` and make it a gate once that
reaches zero. A run is deterministic per seed, but adding a generator arm reshuffles the stream,
so quote the *case literal* in a bug report, never "seed 7 case 412".

The GUI is `windows_subsystem="windows"` in release, so it opens a window and blocks. To smoke-test
that it starts without panicking (shader compile / GL context / first mesh upload all happen at
startup): `timeout 6 ./target/debug/gridfinity-gui.exe`.

## Workspace layout

Two crates (`Cargo.toml` = virtual workspace, edition 2024, resolver 3):

- **`crates/gridfinity-cad`** — the engine library. Deps: `glam` (math) + `earcutr` (used *only*
  for final planar-face-with-holes triangulation; the B-rep kernel itself is hand-rolled).
- **`crates/gridfinity-gui`** — the eframe/egui/glow app. Depends on `gridfinity-cad`.

Inside `gridfinity-cad`, the CAD engine lives in **`src/kernel/`** and the parametric model beside
it in `src/`:

- `src/kernel/` — `math`, `geom`, `sketch`, `topo`, `build`, `fillet`, `tess`, `mesh`, plus the 2D
  region engines `region2d` and `rectregion`. **Nothing here knows about Gridfinity**, and the
  dependency direction is one-way: no kernel module may import from the model layer. Paths are
  `crate::kernel::topo`, `gridfinity_cad::kernel::geom`, etc.
- `src/` — `gridfinity` (the model), `layout` (grid cells/edges), `region` (polyomino boundary
  tracing, grid-coupled), `printers` (bed fitting; pure logic).

`Mesh`, `Solid`, `Tessellation`/`tessellate` and `Params` stay re-exported at the crate root, so
the GUI is unaffected by the split.

## Engine architecture (the big picture)

Pipeline: **`sketch` → `build` (features) → `topo` (B-rep solid) → `fillet` → `tess` → `mesh` → STL.**
(All paths below are under `src/kernel/` unless noted.)

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
  tests after any construction change. `Solid::validate()` is the cheap topology check;
  [`audit`](gridfinity_cad::audit) is the heavy *geometric* soundness checker that also confirms
  each edge's curve lands on its vertices and lies on every face surface that references it.
  When a mesh leaks but `validate` passes, run `audit` first — it pins the failure to a
  specific edge/face if the B-rep is at fault, and rules the B-rep out otherwise (pointing at
  the tessellator).
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
  circle connect arcs join neighbouring blends. A vertex with **two** blended edges continues the
  chain; a vertex with **one** is a *runout* — the chain terminates against a third face, and the
  blend is trimmed by it instead of closed off: tangent curves extend to meet the plane, the exact
  cylinder/plane intersection (a `Curve::Ellipse`) becomes the trim curve, and it is spliced into
  the runout face's loop where its sharp corner was. The runout face is found by adjacency,
  skipping faces coplanar with the blended pair (a coplanar neighbour continues the surface rather
  than terminating the blend). Three or more blended edges at a vertex still needs a spherical
  corner patch → `Err`. The partial-height inner wall's top ramp is built this way. The whole
  solid is rebuilt through a fresh `Builder` and `validate()`d. Gotcha: when re-emitting an arc
  reversed, the angle range must be swapped too — `Builder::arc` trusts that its first vertex sits
  at the first angle.
  `blend_edges` is all-or-nothing, so `blend_best_effort` wraps it for callers that would rather
  have an ugly fillet than none: it groups the edges into connected chains, adds them one at a
  time while the result still blends, and bisects a failing chain (depth 3) to salvage contiguous
  runs — a dropped run just leaves runouts, which are ordinary geometry. It returns the edges left
  sharp, and **errs only when the input is at fault** (a blended edge missing or not shared by
  exactly two faces means the solid was already non-manifold, and degrading there would swap a
  loud error for a silently broken part). `program::run` applies blends through it, which is what
  lets a compartment-splitting divider keep the fillet on the corners that *do* close.
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
- **`program.rs`** — a model expressed as a **flat labelled list of ops** the kernel executes. A
  `Program` carries geometry (profiles, heights, `(seg, z)` blend selections) — never builder
  handles — so `run(prog, |i| bool)` can execute *any subset*: prefixes step through the
  construction, arbitrary masks toggle individual ops off. Blends collect during the run and apply
  once at the end (`blend_edges` consumes and rebuilds the whole solid). A partial subset is
  generally not manifold, by design — that's what makes the debugger useful. `Op::Custom` is the
  escape hatch for geometry with no kernel primitive (Gridfinity's bridge-underside stitching): it
  must re-derive every handle when it runs, which is cheap because `ring`/`seg_edge` intern.
- **`region2d.rs`** — exact 2D boolean algebra on seg-loop regions (outers CCW, holes CW):
  `region_union`/`region_difference`/`region_intersection`. All four Line/Arc intersection pairs
  are closed form (line/line by determinant, line/circle by the quadratic, circle/circle by the
  radical line); cut points are computed once and shared verbatim so selected pieces chain exactly.
  `presplit_regions` gives several booleans over the same inputs one common segmentation — required
  whenever their results must weld to each other. `split_regions` exposes the classified pieces with
  caller-supplied provenance tags, which is how the inner-wall planner names contact runs without
  ever comparing coordinates.
- **`slab.rs`** — the **restricted boolean**: `build_slabs(&[(Op, Slab)])`, where a `Slab` is a 2D
  region swept over a z-range. z endpoints cut the stack into bands, each band's cross-section is
  the 2D boolean of the slabs covering it, and the solid is assembled band by band — walls per band
  (bands are delimited by *all* breakpoints, so nothing changes inside one and verticals need no
  splitting), plus caps at each interface from `below − above` (facing up) and `above − below`
  (facing down), treating outside the stack as empty. **Loop directions are never reoriented**: the
  boolean emits every run material-on-the-left, which is exactly what both the wall and the cap
  need, and a loop can be a cap's hole *and* the outer of the band wall above it (a shoulder under
  a tower) — reversing it for one role breaks the other. Both operands being z-prisms keeps every
  intersection curve vertical or horizontal, which is what keeps this inside the analytic curve set
  (a general cylinder/cylinder boolean would need a quartic). Cones/spheres/tori are not
  expressible: a chamfered peg stays a `loft`, a rolling blend stays `fillet`.
  `emit_slabs` writes into an existing `Builder` so a stack can share edges with hand-built
  geometry; `SlabOpts::cavity` emits the stack as a void (material outside), and `open_at` skips
  the cap at an interface the caller closes itself. **Gotcha:** `wall_seg`'s `outward` flips only
  the surface normal, never the loop direction, so a cavity's walls traverse exactly like a solid's
  — `slab::emit_cap` therefore keeps solid-mode winding and flips only `Builder::face`'s `sense`.
  Using `build::cap` there instead would flip winding too and break the wall/cap pairing.
  Both cavity builders are stacks now. `build_cavity_flat` is the compartment void minus one slab
  per island tower; `build_cavity_banded` is the same plus one slab per partial-height inner wall
  (floor → that wall's top). A wall reaching the loop boundary, one fully inside it and one
  crossing it are all just differences — the band machinery caps each where its slab ends, so none
  of them needs its own code. The stack's returned top band **is** the rim opening, which the
  caller closes. Only the ramp blend still needs the planner: the contact runs name those edges by
  provenance rather than by coordinate.
- **`rectregion.rs`** — rectilinear region engine: unions/differences of axis-aligned rects
  resolved on a compressed coordinate grid, traced material-on-the-left (outer CCW, holes CW), then
  per-corner arc rounding with clamping (`trace_rects`, `shape_loop`). The cavity planner and
  baseplate outline are built on it.
- **`gridfinity.rs`** — the parametric model + spec constants + `Params`, a faithful port of the
  TS reference's `BinConfig`. `Params.bins: Vec<LogicalBin>` holds polyomino cell sets (plus
  optional floor slope); `Params::rect(gx,gy)` is the rectangular convenience. **Each bin is built
  in one `Builder`** so interface edges are shared automatically — there is *no general boolean*.
  Model structure: a boundary walk traces each bin's cells into loops; the outer profile is the
  pitch lines inset `HALF_TOL=0.25` with `OUTER_R=3.75` corners — convex ones welding with the
  corner cell's peg top, concave (reentrant) ones unshared — split at `PEG_TANGENT=4.0`
  (`= HALF_TOL + OUTER_R`, which is exactly what leaves a straight stub either side of a reentrant
  fillet) so peg-top edges weld with the wall's bottom ring. Reentrant corners **must** be rounded:
  the cavity rounds its own concave corners by the fillet radius `fr`, and a sharp outer one pokes
  out through that arc once `fr > wall_thickness · (2 + √2)`, leaving the rim strip between them a
  face whose hole crosses its own outer boundary. The base is
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

The model is exposed both imperatively (`build(&p)`) and as a kernel **`Program`**
(`program(&p)`): the same construction sequence, but inspectable and subset-runnable. The GUI's
construction debugger (right panel) drives that — every prefix runs, and any op can be toggled
off in isolation.

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

**Invalid geometry must never crash the app.** `main.rs` builds **one logical bin at a time**
(`gridfinity::build_piece` per `Params::bins` entry, not one `build` over the layout), so a bin the
model cannot produce is isolated to itself. Each build goes through `catch`, which converts an
`Err` *and* an unwind into a message — the model layer still panics on some degenerate parameter
combinations (e.g. `height_units: 1` with `wall_thickness: 0.4`), and an unwind out of `regenerate`
would take the window with it. `catch` also swaps in a silent panic hook for the duration, since the
message is shown in the UI and a slider dragged through a bad range would otherwise print a
backtrace per frame. A failed bin gets **placeholder geometry** (one plain rounded box per cell, at
the real footprint and height — featureless, so it can't be mistaken for a real build), and every
vertex carries a `bad` flag as a 7th float (`MESH_STRIDE`; the kernel's `render_buffer` still emits
6, and the GUI appends the flag). The fragment shader gives flagged vertices a pulsing red
rim-lit glow, and `paint_error_banner` names the bin and prints why. Export refuses while any bin is
failing rather than panicking on the way out.

`debugger.rs` is the construction debugger (right panel, toggled from the params panel). It calls
`gridfinity::program(&p)` to get the model's op list, caches per-prefix face counts for display,
and rebuilds the solid via `program::run(&prog, |i| enabled[i])` whenever the user steps or
toggles. The App's `regenerate` switches between `gridfinity::build(&p)` (debug off) and the
debugger's subset build (debug on) — both feed the same `tessellate` → VBO upload path.
