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

- `planar.rs`, called from `tess.rs`, for final planar-face-with-holes triangulation only.
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
is **36/150 failing, 6 distinct defects** — drop the `#[ignore]` and make it a gate once that
reaches zero. A run is deterministic per seed, but adding a generator arm reshuffles the stream,
so quote the *case literal* in a bug report, never "seed 7 case 412".

The GUI is `windows_subsystem="windows"` in release, so it opens a window and blocks. To smoke-test
that it starts without panicking (shader compile / GL context / first mesh upload all happen at
startup): `timeout 6 ./target/debug/gridfinity-gui.exe`.

## Workspace layout

Two crates (`Cargo.toml` = virtual workspace, edition 2024, resolver 3):

- **`crates/gridfinity-cad`** — the engine library. One dependency: `glam` (math). Everything
  else, B-rep kernel and triangulator alike, is hand-rolled.
- **`crates/gridfinity-gui`** — the eframe/egui/glow app. Depends on `gridfinity-cad`.

Inside `gridfinity-cad`, the CAD engine lives in **`src/kernel/`** and the parametric model beside
it in `src/`:

- `src/kernel/` — `math`, `geom`, `sketch`, `topo`, `build`, `fillet`, `tess`, `planar`, `mesh`, plus the 2D
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
  angles stay wrap-free. `u` enters a rotational surface only as `cos u · d0 + sin u · d1`, so
  `Prepared` also exposes that `radial` plus `point_at`/`normal_at` taking it pre-computed —
  `point`/`normal` are now thin wrappers over those, which keeps the two forms from drifting.
  `normal_ignores_v` says when a whole grid column shares one normal (every cylinder; a cone,
  whose normal depends on `v` only through its sign, which cannot change within a face).
- **`topo.rs`** — the B-rep: `Vertex`/`Edge`/`Loop`/`Face`/`Solid`. **Not** half-edge with
  next/prev pointers — loops are explicit ordered `(EdgeId, forward)` lists, edges are shared.
  Storage is **flat CSR, not nested `Vec`s**: the `Solid` holds one `loop_edges` arena plus a
  `loops` offset table, and a `Face` is `{ surface, sense, loop0, n_loops }` naming a contiguous
  loop range (outer first). `Loop` survives only as a *transient* input to `Builder::face`; read
  loops off the solid via `outer_edges`/`face_loops`/`inner_loops`/`n_inners`. Cloning a `Solid`
  (which `fillet_edges` does repeatedly) is therefore a few flat `memcpy`s. `Builder` interns
  vertices and edges (edge key = sorted endpoints **+ welded midpoint**, so a circle's two
  semicircle arcs don't collapse into one edge) and flattens each face's loops into the arena. `Solid::validate()` enforces the
  manifold invariant: **every edge used exactly twice, once in each direction** — assert it in
  tests after any construction change. `Solid::validate()` is the cheap topology check;
  [`audit`](gridfinity_cad::audit) is the heavy *geometric* soundness checker that also confirms
  each edge's curve lands on its vertices and lies on every face surface that references it.
  When a mesh leaks but `validate` passes, run `audit` first — it pins the failure to a
  specific edge/face if the B-rep is at fault, and rules the B-rep out otherwise (pointing at
  the tessellator).
  **Local rewrites:** `Builder::resume(solid, seed)` continues on top of an existing solid's
  vertex/edge arenas, keeping every id valid, and interns *only* the faces flagged in `seed` —
  the ones the caller means to re-emit. Everything else is re-filed by `copy_face`, which copies
  a face's loop spans verbatim. That is what makes a blend cost its own neighbourhood instead of
  the whole model (see `fillet.rs`); interning the whole solid in `resume` would cost exactly
  what rebuilding it did. Two gotchas, both load-bearing: `resume` must re-derive each edge's
  index key **the way the constructor that made it did** — `line` averages the two vertex
  positions, and evaluating the curve at mid-parameter instead is the same point in exact
  arithmetic but can land one weld quantum away in `f32`, which silently duplicates the edge and
  shows up as `edge N used fwd=1 bwd=0`. And a resumed builder strands the edges it replaced, so
  it must finish through `build_compact` (not `build`) to drop and renumber them. **Plain `build`
  must never compact:** callers pick blend and chamfer edges by id *before* building, and
  renumbering would silently repoint those selections at other edges.
- **`sketch.rs`** — 2D profiles as closed loops of `Line`/`Arc` segments (`rectangle`,
  `rounded_rect`, `circle`). Corner radii are real arcs. Outer loops CCW.
- **`build.rs`** — features. Three primitives write into a shared `Builder`: `ring` (profile at a
  height), `wall_between` (side faces between two rings), `cap`/`loop_of` (planar caps).
  `extrude`/`prism`/`loft` wrap them. **Orientation convention:** author loops CCW; an `outward`
  flag says whether material is inside the loop (`true`) or it is a hole/cavity (`false`). `loft`
  turns arcs whose radius changes with height into `Cone` faces; a straight segment on a loft
  becomes a *slanted* `Plane` (its normal is computed from the actual 3D quad, not assumed
  vertical).
- **`fillet.rs`** — `fillet_edges(&solid, &[(EdgeId, r)])`: true rolling-ball edge blending as a
  B-rep operator. Plane/plane edges become `Cylinder` blends, plane/coaxial-cylinder circle edges
  become `Torus` blends; adjacent faces are trimmed back to the exact tangent curves and quarter-
  circle connect arcs join neighbouring blends. A vertex with **two** blended edges continues the
  chain; a vertex with **one** is a *runout* — the chain terminates against a third face, and the
  blend is trimmed by it instead of closed off: tangent curves extend to meet the plane, the exact
  cylinder/plane intersection (a `Curve::Ellipse`) becomes the trim curve, and it is spliced into
  the runout face's loop where its sharp corner was. The runout face is found by adjacency,
  skipping faces coplanar with the blended pair (a coplanar neighbour continues the surface rather
  than terminating the blend). Three or more blended edges at a vertex still needs a spherical
  corner patch → `Err`. The partial-height inner wall's top ramp is built this way. The rebuild is
  **local**: an edge changes if it is blended or if either endpoint moves, a face changes if it
  names such an edge, and everything else is re-filed verbatim through `Builder::resume`/
  `copy_face`. This is sound because an edge's endpoints are the same seen from either side, so
  the two faces sharing a changed edge always agree that they changed and the untouched remainder
  stays internally consistent. It matters because a blend touches almost nothing: a floor fillet
  on a 370-cell bin moves **219 of 9889 faces**, and rebuilding all of them was ~90% of
  `fillet_edges`. The result still goes through a full `validate()` — the manifold gate is not
  narrowed to the neighbourhood, only the *work* is. Gotcha: when re-emitting an arc
  reversed, the angle range must be swapped too — `Builder::arc` trusts that its first vertex sits
  at the first angle.
  `fillet_edges` is all-or-nothing, so `fillet_best_effort` wraps it for callers that would rather
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
  (cheaper, and gives quad strips a predictable diagonal); everything else, including planar-with-
  holes, goes to [`planar`](#planarrs). The only triangles dropped are ones a weld would collapse
  anyway (two vertices on the same weld key): a flat triangle with three *distinct* vertices still
  has its three edges paired against its neighbours, so discarding it on area alone punches a slit
  in the mesh.
- **`planar.rs`** — planar polygon-with-holes triangulation: **monotone decomposition** (sweep top to
  bottom, diagonals at split/merge vertices) then the stack triangulation of each y-monotone piece.
  Chosen over ear clipping for what it guarantees, not for speed: **every boundary vertex the caller
  supplies appears in the output** (so the neighbouring face's samples still line up and nothing has
  to be fanned back in afterwards), holes are handled by the sweep rather than by bridging, and each
  interior edge is shared by exactly two triangles by construction. Degeneracy is the whole
  difficulty, because cavity floors are rectilinear and full of exactly-equal coordinates: the sweep
  order is lexicographic (decreasing y, then increasing x), which *is* an infinitesimal shear, so no
  two distinct vertices tie and no edge is horizontal in the sweep's frame; the orientation predicate
  is shear invariant, so the same cross product decides left-of-edge in both frames; and the face
  tracer names the edge it came in on **by index, through an explicit twin**, never by angle, which
  is what a diagonal lying along a boundary edge would break. Its tests assert the three properties
  that matter — exact area, full edge pairing, no dropped vertex — over rectilinear, many-hole,
  collinear-staircase and random inputs.
- **`program.rs`** — a model expressed as a **flat labelled list of ops** the kernel executes. A
  `Program` carries geometry (profiles, heights, `(seg, z)` blend selections) — never builder
  handles — so `run(prog, |i| bool)` can execute *any subset*: prefixes step through the
  construction, arbitrary masks toggle individual ops off. Blends collect during the run and apply
  once at the end (`fillet_edges` consumes and rebuilds the whole solid). A partial subset is
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
  Both sweeps are all-pairs over segments, so both **reject a pair on its bounding boxes** before
  solving it — the boxes are grown by `BOX_TOL`, which must stay above the 1e-3 that `on_seg`
  accepts, or a real crossing gets pruned and the boolean silently loses a cut. A `debug_assert`
  in each sweep re-solves every rejected pair and fails if it finds one, so debug builds and the
  fuzzer verify the prune continuously; that guard is the reason to trust it, since the failure
  mode is wrong topology rather than a crash. `loops_within(a, b, limit)` is the clearance
  predicate: callers wanting a verdict must use it rather than thresholding `min_loop_distance`,
  whose only early exit is an exact zero and which therefore always paid the full `|a|·|b|`.
  **Coincident boundary runs are classified explicitly**, not by the inside/outside point test: where
  the two boundaries run together the midpoint lies exactly *on* the other boundary, where even–odd
  is undefined. Those pieces go to `on_same`/`on_opposite` by relative traversal direction (A's copy
  only, so a shared run is represented once), and each boolean states which it keeps — union and
  intersection take `on_same`, difference takes `on_opposite`. Getting this wrong made `A − (A − N)`
  return *empty* whenever the operands shared most of their boundary, which is exactly the shape of
  a slab band interface, so a partial-height inner wall got no top cap and the solid was
  non-manifold.
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
true `fillet_edges` rolling-ball blend over each compartment's floor-wall edge loop (skipped when
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

**`kernel/perf.rs`** is the instrumentation. A fixed `Metric` set (region booleans, the seg/seg
solve, builder interning, blending, tessellation, slabs) backed by global relaxed atomics, plus a
`CountingAlloc<A>` the *binary* installs as `#[global_allocator]` (a library must not choose the
allocator for its dependents; the GUI installs it, and `lib.rs` installs it `cfg(test)` so
`perf_report` reads churn headlessly). **Off by default** — every entry point starts with one
relaxed load, so an uninstrumented build pays a predictable branch and nothing else. `count()` for
leaves too hot to time (`point_in_segs` runs millions of times; two `Instant::now()` calls would
cost more than the function), `scope()` for everything else. **Timings nest** — `split_regions`
includes the `seg_seg_points` beneath it — so the column does not sum to the wall time.
**Allocations attribute to the innermost open scope** (an allocation-free fixed-depth `Copy` scope
stack in a thread-local `Cell` — never a `Vec`, so pushing a scope can't re-enter the allocator);
that attribution is *exclusive*, unlike the nesting time column, and the shortfall against the
global total is unscoped construction churn. `perf_report` reports the **2nd** rebuild (the
slider-drag case). `cargo test -p gridfinity-cad perf_report -- --ignored --nocapture` prints the
table from the terminal.

That instrumentation drove a churn-first data-oriented pass: a `Solid` is now flat CSR arenas
(`loop_edges` + `loops` offsets, a compact `Face { loop0, n_loops }`), not per-`Face`/per-`Loop`
`Vec`s, so cloning it is a few `memcpy`s; `Solid::edge_faces` returns a two-pass CSR (`EdgeFaces`,
`ef[e]` slices) instead of a `Vec` per edge; and `fillet`/`chamfer`'s `rebuild_loop` takes that
`edge_faces` as a borrow rather than recomputing it once per face. Together those cut a default
rebuild's allocation churn ~77% (fillet_edges ~92%).

A second pass went after *work* rather than churn. The scaling harness is `tests/scale.rs`
(`scale_report` for the cost curve, `scale_features` for what the optional features cost,
`scale_profile` for the per-metric table at `SCALE_WH=48x48`, `tess_bench` for the tessellator
alone — all `#[ignore]`d tools, like `fuzz.rs`). Four changes, biggest first:

- **`fillet_edges` rebuilds only the faces the blend touches** (see `topo.rs`/`fillet.rs` above).
  `Builder::vertex` went 112836 calls → 25452 and `Builder::face` 19994 (≈2× over-emission) →
  10324 on a 370-cell bin. Emitting each blend patch through `face_from` off the stack rather than
  `Loop`/`face`, which heap-allocated three `Vec`s per patch, took a 1476-cell blend 19.4 → 12.2ms.
- **`region2d` rejects segment pairs on bounding boxes** before the exact solve, and
  `island_clears` asks `loops_within` (a threshold predicate that stops at the first close pair)
  instead of minimising with `min_loop_distance`. That function went 3.96ms → 64µs on a 660-cell
  bin, and `plan_piece` 11.7 → 4.8ms. These sweeps were the last quadratics in the pipeline.
- **The tessellator** got the `u`-dependent trig hoisted out of its inner loop, four duplicated
  `project` calls per grid face removed, one weld key per boundary point instead of three per
  triangle, and a pre-sized output buffer: 16.2 → 12.4ms on a 660-cell bin.

**Measure with `tess_bench`/`build_bench`, never a single rebuild.** This machine's single-shot
spread is ±30%, wide enough to hide a 10% change and to invent one. Two of the tessellator changes
above were first recorded here as null results on single-shot evidence; best-of-25 resolves them
as ~1.08×.

**Scalar micro-optimisation in `tess` is exhausted.** Three plausible ones were tried against
`tess_bench` and all measured flat, because LLVM already hoists them: a `Curve::prepare` mirroring
`Surface::prepare` (the per-sample `radial_frame` is loop-invariant), hoisting the four
`EdgeSamples` slices out of `tess_grid_face`'s grid fill, and batching `copy_face` into one
`memcpy` per run of untouched faces (1.8% at 1476 cells, and it would have made `Solid` promise
that a face's loops are contiguous *and* that faces' loop ranges are ordered — not part of its
contract, and a trap for any future operator that emits faces out of order). What remains is
memory traffic: the boundary-sample gather and writing 120724 × 72-byte `Tri`s (~8.7 MB). The next
real gain there is the output format, not the arithmetic.

No superlinear cost remains in the pipeline. Everything now scales linearly or better with cell
count; the items above `plan_piece` in the profile grow only through more compartments, meaning
more calls, not more work per call.

**`plan_piece` was investigated and left alone.** At 1476 cells it splits roughly `pegemit` 3.7ms
/ `stitch_loops_2d` 3.9ms / peg profiles 1.3ms / outer+cavity 2.2ms. Removing the loop clones in
`stitch_loops_2d` and moving sketch names into their `Op::Sketch` instead of cloning both measured
*neutral* — the clones are a few hundred bytes each and sit under this machine's noise. Keying
`Program`'s `sketches` map by step index instead of storing a second copy of each profile made
planning slightly cheaper and the **whole build ~3ms slower**, because `Program::sketch` then
chases into the large `steps` vec on every loft lookup during `run`; it was reverted. The only
real cost found is the ~5 per-cell `format!` labels, worth ~1ms of 11.9ms (consistent across four
alternating A/B rounds) — display-only text the construction debugger needs, and removing the
per-label allocation means giving `Program` a string arena and changing `Step`'s API for ~1.6% of
a rebuild. Not taken.

**Always A/B by alternating** stash/unstash within one session, several rounds. A single
before/after pair on this machine drifts enough to invent a 2.5ms improvement that a proper
alternating run shows to be 0.5ms — that nearly shipped the `sketches` regression above.

`perf_counters_see_a_real_build` needs an inner wall that *crosses* the compartment boundary. With
box rejection in place a free-standing wall yields no crossing pair at all, so `seg_seg_points`
legitimately never gets called and the metric never fires.

`debugger.rs` is the construction debugger (right panel, toggled from the params panel). It calls
`gridfinity::program(&p)` to get the model's op list, caches per-prefix face counts for display,
and rebuilds the solid via `program::run(&prog, |i| enabled[i])` whenever the user steps or
toggles. The App's `regenerate` switches between `gridfinity::build(&p)` (debug off) and the
debugger's subset build (debug on) — both feed the same `tessellate` → VBO upload path.
Its **Profile rebuilds** checkbox enables `perf` around one `build_solid` and shows wall time, the
per-metric table (heaviest first, bar scaled to the heaviest row since the timings nest) and
allocation count / churn / peak.
