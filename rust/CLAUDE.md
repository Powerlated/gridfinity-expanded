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
  manifold invariant and by `tess.rs` sampling each edge exactly once, from its own two vertices
  outward — it is never something a post-process patches up. Sampling once is only half of it:
  `EdgeSamples::build` overwrites the first and last sample of every edge with the stored
  `Vertex::point`, because `sample_into` evaluates the curve and a vertex is never exactly on the
  curves that meet there. At `WELD = 1e4` a 10-ulp radial residue on an f32 coordinate near 80 mm
  is most of a quantisation step, so the arc's endpoint and the line's endpoint welded to
  different keys and cracked the mesh open at the vertex. Taking the endpoint from the topology
  rather than the geometry is *not* a weld-to-fix-gaps: it is using the B-rep's own answer for a
  point the B-rep already decided the edges share.
- **Numerically approximating a curve or surface** (polyline/facet stand-ins) where a closed-form
  `Curve`/`Surface` is the correct answer. If the needed analytic primitive doesn't exist yet, add
  it to `geom.rs` — `Curve::Ellipse` was added exactly this way.
- **Reading a `Mesh` back into modelling.** `Mesh` is write-only downstream of `tessellate`.

Explicitly still allowed, because they sit *at or after* the tessellation boundary:

- `planar.rs`, called from `tess.rs`, for final planar-face-with-holes triangulation only.
- `weld_triangles`, `to_stl_binary`, `flat_vertices`, bounds — export and GL upload.
- `Tessellation::welded_render_buffer` — the web app's single buffer for both preview and STL. It
  is `render_buffer` (position + analytic normal per vertex, unindexed) with positions snapped to
  the `weld_key` representative and weld-degenerate triangles dropped, so it is watertight under
  exact comparison the way `to_mesh` is. This is a *quantisation at the tessellation boundary*, not
  mesh repair: nothing reads it back, and the raw `render_buffer` (unwelded, per-face samples) must
  not be exported — it leaks.
- Mesh-based *verification* in tests (`assert_watertight`, `signed_volume`): checking the analytic
  result, never producing it.

If a task looks like it needs a prohibited operation, that means either a missing analytic
primitive or a missing B-rep operator (e.g. blend runout in `fillet.rs`). **Stop and ask** — do not
reach for a mesh fallback.

## Hard rule: assert everything, and pay for it at runtime

**Assert to high hell.** Every invariant a function relies on gets a real `assert!`/`assert_eq!` at
the point it is relied on: preconditions on entry, postconditions before returning, topology and
orientation invariants after any construction step, geometric assumptions (non-degenerate normals,
on-surface points, sorted/deduplicated inputs, index bounds beyond what slicing already checks) at
the moment they are assumed. Prefer an assert with a message naming the offending values over a
silent `if` that patches the case up.

It is fine — expected, even — for most of the kernel's runtime to be spent inside asserts. A
build that runs slower and fails loudly at the defect is worth far more here than a fast one that
emits a leaking solid. Do not weaken or delete an assert to make something pass, do not trade one
for a fallback branch, and do not "optimise" one away without profiling evidence that it is the
bottleneck of something the user actually waits on.

Use `assert!`, not `debug_assert!`. The suite runs `--release`, which compiles `debug_assert!` out
entirely — a check nobody runs is not a check. Where a check is genuinely quadratic or worse and
only some callers want it, make it a runtime flag checked once per sweep (see `set_verify_prune`
in `isect.rs`), never a `debug_assert!`.

**An assert replaces the checking, never the exercising.** A test whose body only restates an
invariant now asserted in production loses its body, not its fixture: the assert only fires on
inputs something actually feeds it, so deleting the fixture removes coverage while the suite stays
green. Delete a test outright only when the input is redundantly covered too. The invariants that
now live in the code rather than in ~30 hand-picked fixtures:

- `Builder::build` — the manifold invariant, orphan edges excepted
- `orient::normalize` — no misoriented loop survives it
- `gridfinity::try_build` — the result is a closed manifold *and* audits clean (+37% on a build)
- `tessellate` — `tessellation_leaks` is empty (~4× the tessellation itself; still ~2 ms a bin)
- `triangulate` — the triangles tile the loops they came from
- `build_torus_blend` — the blend torus is not a ring
- `to_stl_binary` — the file is 84 + 50n bytes
- `region2d`, `uniforms` — `BOX_TOL` and every uniform block's layout, as `const` asserts that
  fail the *build*

The lib gate went 0.08 s → 1.9 s across these, which is the trade the rule above asks for.

## Commands

```bash
cargo build                      # build both crates
cargo test --release -p gridfinity-cad --lib   # the working gate: engine + model unit tests
cargo test --release -p gridfinity-cad --lib <name> -- --nocapture   # one test, e.g. default_bin_is_valid_watertight_and_sized
cargo test --release --workspace # full gate incl. the fuzzers -- slow, pre-PR only
cargo test --release --workspace -- --ignored --nocapture   # the benchmarks and perf reports
cargo run  -p gridfinity-gui     # launch the app (needs a display + a wgpu backend)
cargo build --release

# The geometry fuzzer (tests/fuzz.rs): random Params -> try_build -> validate -> audit
# -> tessellation_leaks, with failures grouped by signature and each shrunk to a
# paste-ready `Params` literal. Every test and fuzzer runs as a gate under
# `cargo test --release --workspace` -- a deliberate pre-PR run, not the every-change one
# (see AGENTS.md, Validation). The measuring targets do not: `tests/scale.rs`, the badapple
# timings in `gridfinity-gui`, `perf_report` and `alloc_report` are #[ignore]d, because a
# number that no assertion reads is not a gate and they dominated the workspace run.
FUZZ_CASES=2000 cargo test -p gridfinity-cad --test fuzz -- --nocapture
FUZZ_SEED=7 FUZZ_CASES=500 cargo test -p gridfinity-cad --test fuzz -- --nocapture
# One profile alone (they print interleaved otherwise, since test threads run in parallel):
cargo test --release -p gridfinity-cad --test fuzz fuzz_bin_shapes -- --exact --nocapture
```

### One fuzz path, seven profiles

There is **one** generator, checker, shrinker and repro printer. A profile is an `Options` value,
so an invariant added to the checker is immediately enforced by every profile that enables the
feature it covers — the four-way divergence this replaced had `fuzz_split_pieces` asserting shell
counts, trespass and cut-plane gaps that `fuzz_bin_shapes` never applied to the same solids, and a
repro printer that printed neither `open_edges` nor `split_lines`.

`Options` names what a case *generates* (`Shape`, `Walls`, `openings`, `dividers`, `vary_params`,
`slope`, `baseplate`, `Split`) and what it *insists on* (`require_blends`, `known`). Two of those
knobs exist because a checker cannot tell an over-ask from a defect on its own:

- **`Walls::Tidy` vs `Walls::Freeform` decides whether a refused blend is a defect.** Tidy is what
  the editor and the Projects packer emit — axis aligned, on a cell boundary or centre, spanning
  the bin, 0.8–3.0 mm. The model rounds every one of those cleanly, so a tidy profile can *require*
  the blends. A freeform wall sits at any angle and any offset and routinely leaves a sliver
  narrower than the floor fillet; there `fillet_best_effort` degrades **by design** and requiring
  blends would assert the impossible. Measured: a plain 2×2 is 150/150 blend-clean, and so is every
  canonical wall (centre divider at 1.2/3.0/8.0 mm, free-standing island, partial height); freeform
  walls drop 18–28 of 26–42 requested blends, independent of wall width.
- **`known` names a pre-existing, undiagnosed defect** so a profile stays a gate for everything
  else instead of being demoted to reporting wholesale. A known finding is still counted and still
  printed, tagged `KNOWN`; only `unexpected` fails the assert. Never add a signature to silence
  something new.

| profile | what it varies | gate |
| --- | --- | --- |
| `fuzz_inner_walls` | freeform walls on a fixed 2×2 | asserts |
| `fuzz_tidy_inner_walls` | tidy walls, up to 3 so they cross, `require_blends` | reports |
| `fuzz_wall_openings` | `open_edges` + `divider_edges` on rectangles, `require_blends` | asserts |
| `fuzz_openings_and_inner_walls` | both of the above at once, `require_blends` | asserts |
| `fuzz_bin_shapes` | polyominoes, flood-fill pieces | asserts |
| `fuzz_split_pieces` | polyominoes, `SplitLine`s + `partition_cells` | asserts |
| `fuzz_params_broad` | everything, incl. reentrant corners, slope and baseplate | reports |

**Wall openings were never fuzzed before.** `open_edges` flows to `layout::effective_walls` and an
opening deletes the wall the floor fillet was blending against — the runout case. The profile drew
blood on its first run (see the open-run panic below). Openings and dividers are drawn from
`layout::perimeter_edges` / `internal_edges` rather than synthesised: the old broad generator pushed
`GridEdge`s at random cell coordinates with a random orientation, and `effective_walls` consults the
divider set *only* for `EdgeClass::Internal`, so most of those dividers were no-ops.

`fuzz_split_pieces` asserts what "split" is supposed to mean rather than only that each piece is
sound:

- `partition_cells` reproduces an independently derived chunking of the cells,
- each piece is as many separate geometries as its cell set has islands (union-find over the welded
  mesh), so a chunk that falls into two arms must come back as two shells, not one,
- no piece keeps material standing over another piece's cells (the reentrant fillet's 8 mm overhang
  into *empty* cells is allowed -- it is what `REENTRANT_FILLET_OVERHANG` exists for),
- the pieces sum back to the whole bin's volume, and
- pieces whose cells adjoin **touch** on the cut plane, while pieces whose cells do not adjoin are
  measurably apart. Both distances come from XY footprints quantised to 0.05 mm.

**That last one is `Split::Lines` only, and generalising it was a mistake worth recording.** It
measures distance between the two meshes' *vertices*, which is a valid proxy for "the surfaces
meet" only when every piece is a grid slab — then two abutting pieces' cut faces share an outline
and land vertices in the same places. A ragged flood-fill piece breaks that: a three-cell row cut
against a single cell genuinely abuts at y=42 (both meshes reach exactly 42.00) yet the nearest
vertex pair stands 0.5 mm apart, because the row's cut face is subdivided differently. Applied to
`Split::Flood` it reported a defect that is not there. Volume conservation is what holds the flood
pieces to meeting exactly.

**Blends are observable now.** `program::run_reporting` returns a `BlendReport`
(`requested` / `unresolved` / `dropped`) beside the solid, and `gridfinity::try_build_reporting` /
`build_bin_solid_reporting` pass it through; `run` and `build_bin_solid` are wrappers, so no
existing caller changed. Without it a regression that stops rounding corners near an opening or an
inner wall passes every gate — the solid is still manifold, still audits clean, still tessellates
without leaks. Both counters are outcomes the model chooses on purpose (`find_seg_edge` returning
`None` leaves a selection unblended; `fillet_best_effort` would rather leave a corner sharp than
fail the build), which is exactly why only `require_blends` profiles treat them as failures.

**Status at the default seed: all seven pass.** `fuzz_wall_openings` and
`fuzz_openings_and_inner_walls` are clean at six seeds (default, 1, 7, 13, 42, 99) — that pair is
the gate for *adding a wall opening or an internal wall never breaks filleting*. `fuzz_bin_shapes`
and `fuzz_split_pieces` are clean of *unexpected* findings at all six, meeting only
`TRIM_SECTION_CURVE` at 1-3/120. `fuzz_inner_walls` is green at the default seed and finds 1-5/150
of the freeform defects below at others. `fuzz_params_broad` is at 9/400 / 6 defects.

Three defects this rewrite found, all pre-existing and none diagnosed:

- **An opening whose run abuts a reentrant fillet panics the open-run planner** — `open-run
  neighbour must be straight (got an arc before/after the run)`, 7/150 on L-shaped bins, plus a
  rarer `no pinch for run start`. `resolve_open_runs` casts a ray from the run's endpoint along the
  *direction of the adjacent straight segment* to pinch against the outer loop, then truncates that
  segment to the hit. At a reentrant corner the adjacent cavity segment is the concave fillet arc,
  and there is no line to cast along or to truncate. This is why the two asserting opening profiles
  use `Shape::Rect`; `Shape::SmallRect` in `fuzz_params_broad` is what produces the corner.
- **Two inner walls crossing at a cell centre hand the triangulator a face whose loops do not
  tile** (`CROSSING_WALL_TILING`), ~1-2% of tidy configurations. Characterised: **independent of
  tessellation density** (fails identically at segs 2 through 24, so it is a bad loop and not a
  sampling artifact, matching the `triangulate` note that a firing is a defect in whoever built the
  face), and knife-edge in position and width — on a 1×3 bin `h@84 w2.4 + v@21 w2.8` fails while
  `h@63`, `h@42`, `v@10` and widths 2.4 or 3.2 are all clean. A 5-loop face each time.
- **`fuzz_params_broad` is not reproducible run to run**, at `RAYON_NUM_THREADS=1` as well, so it
  is a seeding issue and not a threading one — the class the `fillet_edges` note below describes.
  The variation is confined to `tessellation leaks` findings: the same case lands in a different
  group between runs, moving counts (13 vs 14 distinct defects at 2000 cases). `tessellation_leaks`
  builds its `Vec` by iterating a `HashMap`, and `TessLeak`'s `Debug` carries `faces: [..]`, so the
  message a case fails with is hash-ordered. The gating profiles are stable across five runs.

`gridfinity-wasm`'s `opening_on_a_hole_boundary_stays_closed` is the workspace's one known-failing
test. The bin is a **ring** whose opening sits on the enclosed hole's boundary, which merges the
hole into the cavity — so the cavity's rim loop already swallows the hole, and the ring's own inner
loop is then added as a *second* hole covering the same area. The two overlap: outer 15722 mm²,
holes 1792 + 15141, a net of −1209 mm², leaving the face no interior. `audit` now reports it as
`LoopContainment` (the containment pass had only ever tested each hole against the *outer* loop,
never holes against each other), and `build_bin_solid` asserts the audit, so the failure names the
defect instead of surfacing 196 tessellation leaks. Dropping a hole that another hole contains is
**not** the fix — `divider_ring_island_is_watertight` has a legitimately nested pair and breaks.
The rim holes have to come from one region computation that already accounts for the merge, the way
`plan_cavity_flat`'s rim does.

A torus blend's tangent circles take their parameter range from **their own tangent points**
(`circle_span`), not from the edge being blended: once the blend radius exceeds the corner radius
the tangent circle lands on the far side of the axis, so the inherited range needed rotating by π
and reversing. Only the sweep *magnitude* comes from the source, because a full-turn blend's two
endpoints coincide and cannot tell 2π from 0.

**Both of a torus's parameters are angles**, and `tessellate` unwrapped only `u` for years. A blend
patch crossing the `v` seam therefore arrived at the triangulator as a torn polygon. `dist_to_surface`
had the matching bug in the other direction: it duplicated the ring-torus distance formula, whose
`perp` is unsigned, so on a **spindle** torus (`major_r` < `minor_r`, every corner blend) a point
with a negative radial coefficient measured exactly `2·major_r` off a surface it was sitting on.
`Surface::signed_distance` now takes the nearer of the two radial branches for spindles, and the
auditor delegates to it instead of keeping a copy.

**A fillet that would build a face whose boundary crosses itself is refused**, and
`fillet_best_effort` drops it — an unfilleted corner, not a build failure, which is the policy that
was already there for blends that fail to build at all. The check (`face_loops_self_intersect`)
runs over *every* face the rebuild touched, not just the blend faces: the runout rewrites the loops
of neighbouring walls too, and the bowties it left there were the larger half of the defect.
`fuzz_bin_shapes` went **47/120 → 0/120** (clean at eight seeds, and 1/1500 at `FUZZ_CASES=1500`)
across the four defects below. Fixing them moved `fuzz_inner_walls` 27 → 30/150 with **no new defect
class**: the corner clamp changes every bin's cavity slightly (rc 2.5 → 2.55 at the default wall),
so the same six defects catch a few more random wall placements. What `fuzz_bin_shapes` found:

- **A cut can split one face into disjoint regions**, and `trim` read the second as a hole of the
  first — a self-intersecting face that tessellates with leaks. Carving the middle cell of a strip
  leaves the rim as two separate strips; carving a corner leaves the fillet as two arcs. See
  `emit_trimmed_faces` under `split.rs`.
- **The cavity escaped the wall at a rounded corner** whenever `cavity_corner_radius <
  OUTER_R - wall_thickness`, panicking the rim planner outright below ~1.1 mm of wall. See the
  corner clamp under `gridfinity.rs`.
- **The reentrant-corner fillet overhangs the grid** and every piece shaved it off, losing ~54 mm^3
  per corner. See `carve_to_cells`.
- **A piece enclosed on all four sides is refused**, not mangled — the one case still unsupported.

Two rarer defects survive at 1500 cases (~0.07%, both in `trim`): a 4-leak tessellation failure on a
cut plane, and `no closed-form section curve for a face the cut crosses`. Neither is diagnosed.

**`fuzz_bin_shapes` targets `carve_to_cells`, not the model.** It grows a random connected
polyomino, severs a random subset of its internal adjacencies and flood-fills the remainder — the
same construction as `src/lib/cuts.ts`'s `partitionCells`, so pieces are arbitrary connected
polyominoes and not the grid slabs `layout::partition_cells` produces. It builds the bin **once**
via `build_bin_solid`, checks that whole solid first (failures are prefixed `whole `, so a
pre-existing model defect never reads as a split defect), then carves each piece and checks it
(prefixed `piece N `). Its sharpest invariant is **volume conservation**: the pieces' tessellated
volumes must sum to the whole bin's within `VOLUME_DRIFT`. That is the invariant the bounding-box
carve bug violated at 125%, and it is what caught the reentrant overhang at 0.2%; it sees material
that per-piece manifoldness checks pass straight over, in either direction.

**Measure a suspected volume loss against tessellation density before believing it.** Every one of
these numbers is a coarse mesh's estimate. The reentrant-overhang loss was confirmed real by
converging on -209 mm^3 from segs 4 to 48 while a rectangular split stayed at 0.01 mm^3, and the
mechanism was then pinned by correlating drift with reentrant-corner count (rect 0, L 1x -54,
T and S 2x -104) and with the parameters (flat in every cavity parameter, linear in height). A
vertex-only containment check will *not* see the overhang: the fillet's tangent points sit exactly
on the grid lines and only the arc between them escapes.

**What was left in `fuzz_inner_walls` (12/150, now 0 at the default seed and 1-5/150 at others).**
Three fixes took it from 30. Two were the same mistake in different places: an island's top face
came from the planner's own loop while the walls under it follow the slab band's segmentation, so
one raw island side faced several band edges and paired with none. Both paths take their tops from
the band now. The third stopped `seg_edge` interning a blend selection the boolean had split -- that
reported a missed selection as a non-manifold solid, and masked five that genuinely were. Later
fixes (the self-intersecting-face refusal, the sorted `bm` iteration) took the rest at the default
seed. The classes below are what the surviving off-seed failures still look like:

- **7 cases are the tessellator, not the model.** `validate` passes and `audit` reports **zero
  errors**, so the B-rep is sound and the mesh still leaks. They move with `wall_thickness` and
  `cavity_corner_radius` in no pattern -- a case leaks at 1.2/0.0, is clean at 1.5/0.0, leaks again
  at 2.0/0.0 -- which rules out one degenerate value and points at a general fragility where an
  inner wall meets the cavity's rounded corner.
- **2 cases are a real blend defect** on a **spindle torus** (`major_r` 1.45 < `minor_r` 4.0, the
  shape `orient.rs` already warns about): the blend edge's curve lands 2.05 mm from its own vertex
  and deviates 2.9 mm from the torus it is meant to lie on.
- **1 case is a genuinely non-manifold** input to the blend (`edge has 1 faces`), which is what
  that error is for.
- **2 cases still fail `validate` with `fwd=1 bwd=0`.**

The fuzzer only generates and shrinks to **edge-connected** bins. `gen_small_rect` could delete the
middle of a 1x3 and `shrink` could delete any cell, so either could hand the model a diagonally
connected bin, which `AGENTS.md` puts out of scope. It changed no counts, but the repros it prints
are trustworthy now, and were not before.

**`gridfinity-gui`'s `broken()` is coupled to the model's failure surface.** Its three
failure-path tests need a configuration that genuinely fails, and every fix here retires one, so it
has been re-pointed twice. It also needs a **hard** failure: `build_bin` only catches `Err` and
panics, so a solid that builds and then fails `validate` reads as success there.

**A `HashMap` keyed by edge must not decide a value keyed by vertex.** `fillet_edges` filled
`vinfo` (vertex -> moved tangent points) by iterating `bm`, a `std::collections::HashMap<EdgeId,
Fillet>`. A blend *chain* has two blended edges meeting at each interior vertex, so both wrote that
vertex and **the last writer won** — with `RandomState` seeded per process and per thread, which
one that was changed run to run. The two blends agree only to within an ulp, and an ulp at a
`weld_key` boundary is exactly the crack described under `topo.rs`, so `fuzz_inner_walls` failed on
about half of all runs (and at `RAYON_NUM_THREADS=1` too — this was never a threading bug, only a
seeding one). It now iterates `bm`'s keys sorted, the way lines 188 and 332 of the same function
already did. **Any iteration order that reaches geometry has to be sorted**: a std `HashMap` here is
a random number generator, and `--workspace` vs `-p gridfinity-cad` builds different binaries, so a
"passes alone, fails in the gate" report is a determinism smell before it is a feature-unification
one.

**The long test targets are `rayon`-parallel, and must stay deterministic anyway.** Every fuzz
profile shares one `sweep()`: cases are generated **sequentially** from the seeded `Rng` into a
`Vec` (so the case stream is byte-identical to the serial version), then checked with `par_iter`,
then grouped in `BTreeMap` order and shrunk in parallel. Generating per-case seeds in parallel
instead would reshuffle the stream and invalidate every recorded count.

Three phases, and only two of them can be parallel. Measured on 8c/16t: the check phase alone is
**7.1×** (2000 cases, 6.22 s → 0.88 s), and the heaviest profile end to end is **6.5×** (3000 cases,
12.15 s → 1.86 s) — about the ceiling for compute-bound work on eight physical cores, with the
serial generator the remainder. The shrink phase used to be the weak half, because
`entries.par_iter()` only spreads across *distinct signatures* (often one or two) while each shrink
is a long sequential search; its cell-removal sweep is `par_iter().find_map_first` now.
**`find_map_first`, never `find_map_any`:** it returns the match earliest in iteration order
whatever order the threads finish in, which is what keeps a shrunk repro a function of the seed
alone. The remaining `keep_if` chains stay sequential on purpose — each one edits the current best,
so they are genuinely dependent.

`catching` no
longer swaps the panic hook per case — with many threads inside `catch_unwind` that race silenced
the harness's own hook; `quiet_panics()` installs one silent hook per sweep, refcounted, and
restores the loud one before the report is asserted. `gridfinity-gui`'s `badapple::tests::
face_shapes` sweeps every distinct blob >= 40 cells in the first ten seconds of the clip: ~31
minutes of CPU, which is what made `cargo test --workspace` look hung. It is parallel (measured
13.6x on 16 threads) and de-duplicates blobs that repeat across consecutive frames, taking it to
~140 s. Because it saturates every core, anything sharing that binary must not have a wall-clock
budget tight enough to starve — `pipelined_worker_matches_serial_build` polled a fixed 2000 x 2 ms
and began failing under the load; its budget is an explicit deadline now.

**Reported leaks are picked by lexicographic minimum, not `leaks[0]`.** `tessellation_leaks` sorts
by `(a.z, a.x, a.y)`, and ties among those resolve by `HashMap` iteration order, so `leaks[0]` moved
between runs and silently reshuffled the defect grouping (`fuzz_params_broad` drifted 12↔13 groups
run to run). All three fuzzers are now byte-identical across repeated runs at a fixed seed; keep any
new failure message free of hash-ordered content or the grouping stops meaning anything. Run the suite with `--no-fail-fast`,
since a failing binary otherwise hides the ones after it. A run is deterministic per seed, but adding a generator arm reshuffles the stream,
so quote the *case literal* in a bug report, never "seed 7 case 412".

The GUI is `windows_subsystem="windows"` in release, so it opens a window and blocks. To smoke-test
that it starts without panicking (shader compile / GL context / first mesh upload all happen at
startup): `timeout 6 ./target/debug/gridfinity-gui.exe`.

## Workspace layout

Two crates (`Cargo.toml` = virtual workspace, edition 2024, resolver 3):

- **`crates/gridfinity-cad`** — the engine library. One dependency: `glam` (math). Everything
  else, B-rep kernel and triangulator alike, is hand-rolled.
- **`crates/gridfinity-gui`** — the eframe/egui/wgpu app. Depends on `gridfinity-cad`.

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
  (`Line`/`Circle`/`Ellipse`/`TorusSection`). `TorusSection` is a torus cut by a plane **parallel to
  its axis** — implicitly a quartic, which is why a plane split through a floor-fillet corner blend
  used to be inexpressible. It is not solved numerically: fixing the minor angle `t` fixes the ring
  radius `major + minor·cos t`, hence `cos u = offset / rad`, so the section parameterises exactly
  in `t` with `branch = ±1` picking the half. `torus_section_exists` reports where `|offset| <= rad`
  bounds the domain; outside it the section simply does not exist and the caller must not sample. Every radial surface and `Circle` carries an explicit `axis` (arbitrary
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
  semicircle arcs don't collapse into one edge). Both lookups are **exact bucket first, then the
  26 neighbours within `WELD_NEAR`**. The exact bucket must win unconditionally: a point may sit
  most of a bucket diagonal from its bucket's representative, so distance-testing the exact hit
  splits vertices the old lookup shared. The neighbour scan is what catches the opposite failure
  -- two solves of one corner landing 4e-6 mm apart but on either side of a bucket boundary,
  which interned as two vertices and left the solid non-manifold at an edge nothing paired with. and flattens each face's loops into the arena. `Solid::validate()` enforces the
  manifold invariant: **every edge used exactly twice, once in each direction**.
  `Builder::build` asserts it itself, via `validate_ignoring_unused_edges` — the interning arena
  can outlive edges no face kept, and only `compact_edges` drops them, so orphans are the one
  tolerated deviation. Three callers legitimately build something that is *not* yet a closed
  manifold and say so by name: `fillet_edges` and `chamfer_edges` build a candidate speculatively
  and reject it on a failed `validate` (`build_compact_unvalidated` / `build_unvalidated`), and
  `program::run_all` emits partial open shells for the step-through debugger. Everything else goes
  through `build`, so a construction bug panics at the builder that caused it instead of surfacing
  as a leak in a test far downstream. It is *not* the whole story: alternation holds just as well
  for a consistently-inverted shell, so `Builder::build` additionally establishes the **orientation
  invariant** via `orient::normalize` (see `orient.rs`). Loop directions off a built `Solid` are
  therefore material-consistent and may be relied on; the modules that emit loops (`slab`,
  `region2d`, `build`) still author in their own conventions and are re-wound at `build`. `Solid::validate()` is the cheap topology check;
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

**A shared cut point must be solved once, not once per side.** `presplit_regions` used to walk
*ordered* region pairs, so a crossing was solved twice — `seg_seg_points(sA, sB)` for A's cut list
and `seg_seg_points(sB, sA)` for B's. The line/line determinant is not ulp-symmetric under that
swap, so the two "shared" copies could differ by an f32 ulp; if that ulp straddled a `weld_key`
cell boundary (observed: `57.819447` → 578194 vs `57.81945` → 578195) the two segments interned
*two* vertices, every edge touching them dangled, and `validate` failed with `fwd=1 bwd=0`. It walks
unordered pairs now and pushes the identical `pt` into both segments' cut lists, which is what
"cut points are computed once and shared verbatim" was always supposed to mean. Halving the solves
also made it **faster than before the bug was known**: a 32x32 build went 19.4 → 17.1ms
(alternating A/B). Resist any fix for this class that reconciles *downstream* — snapping in
`Builder::vertex` was tried, cost ~10% of a build, and only hid which producer was wrong.

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
  decided **once per face** (area-weighted vote of the emitted triangles' geometric normals against
  the analytic ones) — never per triangle, or curved faces get inconsistent internal edges. That
  vote is literal, not the `uv_area · uv_orientation · sign` proxy it used to be: the proxy assumes
  a face's uv handedness is constant, which a **spindle torus** (floor-fillet blend, `major_r` <
  `minor_r`) violates, and it silently inverted those faces once loop winding was normalised.
  The structured-grid path tries **both edge rotations** when matching a quad's loop to its
  iso-u/iso-v roles, since which of the four edges comes first depends on where the loop starts;
  without the retry a normalised loop falls through to the planar path, which chords straight across
  a curved patch (a floor-fillet blend came out 0.6 mm off its own cylinder).
  `Edge::seg_count` subtracts a `1e-3` slack before `ceil`, so an arc whose sweep is a hair over an
  exact multiple of a quarter — which a connector's `TAU`-wrapped advance always is — does not gain
  a spurious segment. When it did, the two arcs bounding a trimmed patch disagreed on sample count
  and the grid path rejected the face. Non-planar 4-sided faces whose loop follows
  iso-u/iso-v lines (cylinder walls, cone chamfers, blend patches) take a structured-grid path
  (cheaper, and gives quad strips a predictable diagonal); everything else, including planar-with-
  holes, goes to [`planar`](#planarrs). The only triangles dropped are ones a weld would collapse
  anyway (two vertices on the same weld key): a flat triangle with three *distinct* vertices still
  has its three edges paired against its neighbours, so discarding it on area alone punches a slit
  in the mesh. `triangulate` asserts its own postcondition — `assert_tiles_the_loops` requires
  every loop edge to land in exactly one triangle and every other edge in exactly two. Neither the
  quad fast path nor `planar` can satisfy that for a loop that is **not simple in uv**, and before
  the assert existed they answered anyway: a self-intersecting wall-top quad came back as six
  triangles including a literal duplicate, which surfaced only as a `tessellation_leaks` report on
  a face far from the trim that authored the bad loop. The assert is where a bowtie loop is now
  caught, but it is not where such a loop is *fixed* — a firing is a defect in whoever built the
  face, not in the triangulator.
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
- **`split.rs`** — `trim_half_space(solid, plane, keep)`, the operator printer splits are being
  moved onto so a split divides the finished bin instead of each piece being authored from its own
  cell set (which is why a divider seam used to yield two `wall_thickness + HALF_TOL` walls where
  the intact bin has one centred `wall_thickness` strip). Every face is classified, straddling faces
  are trimmed to their section curves, the gaps are closed with connectors along `face ∩ plane`, and
  those connectors are chained into cap loops. All the analytic parts are closed form:
  `curve_plane_params` (line by ratio, circle/ellipse by `a·cos t + b·sin t = c`, a `TorusSection`
  never crosses since it already lies in the plane) and `param_of`, which inverts a point back to a
  parameter — **never by sampling**, which is the numerical approximation the kernel forbids.
  Two conventions are load-bearing and were each a bug first. The connector direction is
  `winding_normal × discard_normal` using the face's **true outward** normal (raw normal flipped by
  `sense`), which is well defined only because every loop now satisfies the orientation invariant
  below — before that invariant existed this had to use the raw normal, and a cavity wall sent the
  connector the wrong way. And trimmed pieces must take their end positions from the solid's
  **stored vertices**, not from `curve.point(t)` — the same point in exact arithmetic, but an f32
  ulp away, which duplicates the edge exactly as the `resume` note above describes.
  A connector's arc is emitted as `(t0, t0 + signed_advance)`, never `(param_of(from),
  param_of(to))`: `param_of` returns a principal value, so the raw pair can name the *other* way
  round the circle. `advance_along` already computes the correct directed advance (wrapping by
  `TAU` until positive), and that signed value is what the edge must be built from. Getting this
  wrong put a floor-fillet blend's quarter-arc on the 3/4 path — same endpoints, wrong surface —
  and cost 83 mm³ of double-counted material on a 2x1 bin.
  **Status: cuts a bin.** `cutting_a_box…`, `cutting_a_rounded_prism…` and
  `cutting_a_bin_gives_two_watertight_halves_that_conserve_volume` all assert both halves are
  manifold, mesh-closed and volume-conserving; the bin case conserves volume to 0.003 mm³ with
  floor fillets, cavity corner radii and holes all on, at every tessellation density.
  The operator is `trim(solid, &Cut)`; `trim_half_space` is the one-plane convenience over it. A
  `Cut` is a list of oriented planes, each with the `discard_normal` pointing into the material it
  removes, and it owns the three things that were previously hard-coded to a single plane:
  classification (`Cut::side_of`, where `On` means *on the cut surface*, not on one plane),
  crossings (`Cut::crossings`, tagging each parameter with the plane it crosses), and cap grouping
  (caps are emitted per plane).
  **A trimmed face may be several faces.** A cut can break one face into disjoint regions — trimming
  a bin's rim to its middle cell leaves the strip either side of the cavity with no path between
  them, and trimming a corner fillet leaves an arc either side of the removed span.
  `emit_trimmed_faces` groups the surviving loops by winding into outers and the holes each outer
  contains, the way `emit_caps` already grouped its cap loops; taking the first loop as the outer and
  the rest as its holes made the second region a *hole of the first*, a self-intersecting face that
  audits `LoopContainment` and tessellates with leaks. Two things it must get right: the winding test
  is signed area **times the face's `sense`**, since an inverted face's outer loop measures negative;
  and on a rotational surface `u` is an angle, so each loop's `u` is **unwrapped** first
  (`unwrap_u`) — a seam-crossing arc otherwise reads as a near-full-turn band whose signed area is
  meaningless, which is exactly how two disjoint arcs of a corner fillet measured as +137 and -8.
  A face whose loops do not classify cleanly falls back to first-loop-is-outer, which is the reading
  every uncut face already has, so the grouping can only ever improve on it.
  **Multi-plane cuts work.** `Cut::prism(&loops, axis)` sweeps a set of 2D loops into a prism of
  oriented planes, which is what carving an arbitrary polyomino needs. The piece that used to be
  missing is a connector that terminates on a *cut-surface edge* rather than on another chain:
  advancing along plane A must stop where A meets plane B and continue there. `window_exits` finds
  those stops, `runs_along_cut` recognises a section curve already lying in the cut surface, and
  `nearest_along_shared_edge` picks the continuation. Classification, crossings and per-plane caps
  were already general.
  A split is a boolean applied last rather than each piece being authored from its own cell set.
  `build_bin_solid` builds a logical bin once and `carve_to_cells` trims one printable piece out of
  it; every caller pairs them that way and builds each bin **once** — `try_build_pieces` for the
  GUI/STL path, `generate_geometry` for the web app, which carves all of a bin's pieces off the one
  solid rather than rebuilding and re-filleting the bin per piece. `build_piece` is the
  single-piece convenience that composes the two.
  **`carve_to_cells` trims to the piece's *cell set*, not its bounding box.** It traces the cells
  into boundary loops and trims to that prism, so a piece may be any connected polyomino. Carving
  to a bounding box was exact only for grid slabs: `layout::partition_cells` (the GUI/STL path)
  guarantees those, but the web app's `src/lib/cuts.ts` `partitionCells` is a flood fill over
  severed edges, so two pieces' boxes could overlap and their material was duplicated — a 2x2 bin
  split into `{(1,0)}` and the L-shaped `{(0,0),(0,1),(1,1)}` gave back the L as the *whole bin*,
  the two pieces summing to 125% of it. `a_staircase_piece_carves_to_its_cells_not_its_bounding_box`
  pins the partition. `trim` first classifies the solid's vertices, so a cut that
  misses a piece's material is a no-op rather than an error — an L-shaped bin needs that.
  Seam walls are *not* special-cased any more: the whole bin is built with its dividers and then
  cut, so a divider at a seam becomes a wall in both pieces and a plain seam cuts open, which is
  what `split_seam_divider_walls_both_pieces` and `seam_edges_default_open` already asserted.
  It takes the **bin's** cells as well as the piece's, because the prism is not quite the cell set.
  A reentrant corner is filleted by `OUTER_R` and that arc **overhangs the grid** into the empty
  cell in the notch — only its two tangent points sit on the grid lines. Trimming to the bare cell
  rectangles shaved the bulge off every piece and lost it outright: ~54 mm^3 per reentrant corner,
  so a split L lost 0.1% and a split rectangle lost nothing. Each cell therefore reaches
  `REENTRANT_FILLET_OVERHANG` into a neighbouring cell **the bin does not occupy**, and reaches
  **only along y**. One axis is not a simplification, it is the whole correctness argument: a
  reentrant corner always has a vertical neighbour among its three occupied cells, so one reach
  always covers the bulge, while reaching along both axes lets two pieces meet inside the same empty
  cell and both claim it — measured at *+186* mm^3 over-count on a split T.
  `carving_a_reentrant_bin_keeps_the_corner_fillet_that_overhangs_the_grid` pins it. A ~7 mm^3
  residual remains, flat in height so it is in the base, not the walls; it is not diagnosed.
  **A piece enclosed on all four sides by the rest of the bin is refused up front**
  (`piece_is_enclosed`). Its cut runs through the *interior* of faces without ever crossing their
  loops, and `trim_loop` only inspects loop edges, so no chain and no connector is ever produced —
  the 3x3 centre cell reaches `trim` with 32 vertices inside, 312 outside and **0 connectors**.
  Supporting it means teaching `trim` to open a new interior section loop in a face, which it cannot
  do; until then the error says so instead of failing deep in the cap emitter with `cut section does
  not close into a loop`.
- **`orient.rs`** — the **orientation invariant**: every loop is *material-consistent*, meaning that
  walking it with the face's true outward normal keeps the face's material on the left (outer loops
  positive, holes negative). `Builder::build` establishes it, so every solid the kernel hands out
  satisfies it and `misoriented_loops` is empty. This is what makes a boolean expressible.
  It was *not* true before: `region2d` emits its loops material-on-the-left in 2D, `slab`'s cavity
  mode reuses that winding and flips only `Builder::face`'s `sense`, so a bin's whole cavity shell
  — plus the rim face's inner loop that bounds it — traversed inverted relative to the outer shell.
  Edge alternation still held (an inverted shell paired with an inverted hole loop alternates
  fine), which is exactly why `validate` never caught it and why **propagation across shared edges
  cannot detect it**: the material side has to be measured geometrically.
  Two things make the measurement trustworthy. It uses the loop's **3D area vector** dotted with the
  analytic normal, not a signed area in uv — `uv_orientation()` assumes the uv parameterisation's
  handedness is constant over a face, and on the floor fillet's **spindle torus** (`major_r` 0.1 <
  `minor_r` 2.4) the ring radius `major + minor·cos v` changes sign inside the face, so uv area
  reports the wrong handedness. And the decision is made **per connected component of loops sharing
  an edge, not per loop**: alternation forces the flip set to be closed under edge sharing, so a
  component flips as a unit by an area-weighted vote. Deciding per loop instead strands any loop the
  measurement skips (a full-2π loop on a whole cylinder has no meaningful area vector) on the wrong
  side of its neighbours and breaks the manifold invariant — that was three fillet tests.
  Normalising is pure re-winding: it never moves a vertex, and `normalising_changes_no_geometry`
  pins that.
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
  radical line); cut points are computed once, for an *unordered* segment pair, and pushed verbatim
  into both sides' cut lists so selected pieces chain exactly (see the shared-cut-point note under
  `topo.rs` for why solving each side separately silently splits vertices).
  `presplit_regions` gives several booleans over the same inputs one common segmentation — required
  whenever their results must weld to each other. `split_regions` exposes the classified pieces with
  caller-supplied provenance tags, which is how the inner-wall planner names contact runs without
  ever comparing coordinates.
  Both sweeps are all-pairs over segments, so both **reject a pair on its bounding boxes** before
  solving it — the boxes are grown by `BOX_TOL`, which must stay above the 1e-3 that `on_seg`
  accepts, or a real crossing gets pruned and the boolean silently loses a cut. A verification pass
  in each sweep re-solves every rejected pair and fails if it finds one; that guard is the reason
  to trust the prune, since the failure mode is wrong topology rather than a crash. It is gated on
  `set_verify_prune`, a relaxed atomic checked **once per sweep** rather than once per pair, so it
  costs a predictable branch when off and re-solves quadratically only when a test asks for it. It
  was a `debug_assert!` per rejected pair until the suite moved to `--release`, which compiled it
  out entirely — the reason it is a runtime flag is that a check nobody runs is not a check. `loops_within(a, b, limit)` is the clearance
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
  Using `build::cap` there instead would flip winding too and break the wall/cap pairing. That
  stays true *during* emission; `Builder::build` then re-winds the finished cavity shell to satisfy
  the orientation invariant (see `orient.rs`), which is a pure re-winding and changes no geometry.
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
  **The outer cavity loop's convex radius is clamped up to `OUTER_R - wt`.** A convex outer corner
  is an arc of `OUTER_R` about a centre `HALF_TOL + OUTER_R` in from the pitch corner, so a cavity
  corner of radius `rc` leaves a wall there of `wt - (OUTER_R - wt - rc)(sqrt(2) - 1)`: below
  `OUTER_R - wt` the wall thins at the corner, and below `wt ~ 1.1` mm a sharp cavity **escapes the
  outer arc entirely**, at which point it is no longer inside the rim face it is a hole of and
  `plan_piece` panicked with `total_h hole without a containing face`. Clamping makes the two arcs
  concentric, which is what keeps the wall its own thickness the whole way round. It moves the
  default bin's cavity corner 2.5 → 2.55 mm and takes the corner wall 1.179 → 1.2 mm.
  `a_thin_walled_bin_keeps_its_cavity_inside_the_rounded_corner` sweeps the pair. **A sloped floor
  is left square** — it builds its cavity with no rounding at all, and clamping there breaks
  `sloped_bin_is_watertight_and_outward`, so thin-walled *sloped* bins can still hit the panic.

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

This project pins **egui/eframe/egui-wgpu 0.35, which is a redesigned API**, not mainstream egui:

- `eframe::App::ui(&mut self, ui: &mut egui::Ui, frame)` — you get a root **`Ui`**, not a `Context`
  (there is no `update(ctx, ...)`).
- Panels are shown *inside* that root ui: `egui::Panel::left(id).show(ui, ...)` / `Panel::right` /
  `CentralPanel::default().show(ui, ...)`. There is **no `SidePanel`**.
- Scroll delta is `input.smooth_scroll_delta` (no `raw_scroll_delta`).
- `NativeOptions` still has `renderer`/`viewport` like mainstream. **Leave `depth_buffer` at 0** —
  every depth-tested pass is offscreen, and a depth attachment on egui's own pass makes the
  depth-less blit pipeline incompatible with it, which is a `set_pipeline` validation panic.

`viewport.rs` is a thin egui adapter over the shared `gridfinity-render` crate. It implements
`egui_wgpu::CallbackTrait`: `prepare` runs the whole offscreen chain and returns its command buffer,
`paint` blits the finished image into egui's render pass. The callback carries the panel rect and
converts it to pixels itself, so `prepare` and `paint` cannot disagree about the viewport — the
renderer stores the viewport it presented and the blit reads it back. The params panel owns a
Low/Medium/High row calling `Renderer::set_quality`; quality is pinned, defaulting to `High`.

Both consumers now compile the **same WGSL through naga**, so the class of bug where the debugger
accepted a shader the browser rejected is gone. `cargo test -p gridfinity-render` parses and
validates every module, which is where a shader error surfaces. Still smoke-test browser-facing
changes with `npm run test:e2e` — validation does not catch a wrong picture.

**Upload before the paint callback is queued, not after.** `Panel::show` only *queues* the callback;
it runs later, inside `paint_primitives`. `ui()` used to call `regenerate` after the central panel
had been shown, so a frame that changed geometry drew the new mesh against the previous frame's
camera snapshot — and since the debugger repaints on demand, that mismatched frame stayed on screen
until the next input event. `regenerate`/`badapple_tick` now run before `CentralPanel::show`.

The web app drives the identical `Renderer` and the identical
`append_smooth_shaded` staging from `gridfinity-wasm`'s `Viewer`; the shading path is shared, and
the two consumers differ only in what they upload (the GUI adds a wireframe pass and the `bad`
flag; the web app adds per-bin colour, the explode displacement, and `FLAG_CUT` on the triangles
that lie in a split's cut plane). Back-face culling relies on the engine's outward winding (see
the `meshes_have_outward_consistent_winding` test). `main.rs` binds `Params` to widgets, regenerates
(build → tessellate → upload the vertex buffer) on change, and exports STL via `rfd` + `Mesh::to_stl_binary`.

**Invalid geometry must never crash the app.** `main.rs` builds **one logical bin at a time**
(`gridfinity::build_piece` per `Params::bins` entry, not one `build` over the layout), so a bin the
model cannot produce is isolated to itself. Each build goes through `catch`, which converts an
`Err` *and* an unwind into a message — the model layer still panics on some degenerate parameter
combinations (e.g. `height_units: 1` with `wall_thickness: 0.4`), and an unwind out of `regenerate`
would take the window with it. `catch` also swaps in a silent panic hook for the duration, since the
message is shown in the UI and a slider dragged through a bad range would otherwise print a
backtrace per frame. A failed bin gets **placeholder geometry** (one plain rounded box per cell, at
the real footprint and height — featureless, so it can't be mistaken for a real build), and every
vertex carries a `bad` flag in the last of `MESH_STRIDE`'s ten floats (position, analytic normal,
colour, flag — `MESH_STRIDE` is an alias of `gridfinity_render::VERTEX_STRIDE`, and
`append_smooth_shaded` expands the kernel's six-float `render_buffer` into it). The fragment shader gives flagged vertices a pulsing red
rim-lit glow, and `paint_error_banner` names the bin and prints why. Export refuses while any bin is
failing rather than panicking on the way out.

**`kernel/perf.rs`** is the instrumentation. A fixed `Metric` set (region booleans, the seg/seg
solve, builder interning, blending, tessellation, slabs) backed by global relaxed atomics, plus a
`CountingAlloc<A>` the *binary* installs as `#[global_allocator]` (a library must not choose the
allocator for its dependents; the GUI installs it, and `lib.rs` installs it `cfg(test)` so
`perf_report` reads churn headlessly). `A` is **mimalloc**, not `System` — see the allocator note
below; `gridfinity-cad` keeps it as a dev-dependency only, target-gated away from wasm, so that
benchmarks measure the allocator the shipping binary actually uses. **Off by default** — every entry point starts with one
relaxed load, so an uninstrumented build pays a predictable branch and nothing else. `count()` for
leaves too hot to time (`point_in_segs` runs millions of times; two `Instant::now()` calls would
cost more than the function), `scope()` for everything else. **Timings nest** — `split_regions`
includes the `seg_seg_points` beneath it — so the column does not sum to the wall time.
**Allocations attribute to the innermost open scope** (an allocation-free fixed-depth `Copy` scope
stack in a thread-local `Cell` — never a `Vec`, so pushing a scope can't re-enter the allocator);
that attribution is *exclusive*, unlike the nesting time column, and the shortfall against the
global total is unscoped construction churn. `perf_report` reports the **2nd** rebuild (the
slider-drag case). `cargo test --release -p gridfinity-cad --lib perf_report -- --ignored --nocapture`
prints the table from the terminal; like every other measuring target it is `#[ignore]`d.

That instrumentation drove a churn-first data-oriented pass: a `Solid` is now flat CSR arenas
(`loop_edges` + `loops` offsets, a compact `Face { loop0, n_loops }`), not per-`Face`/per-`Loop`
`Vec`s, so cloning it is a few `memcpy`s; `Solid::edge_faces` returns a two-pass CSR (`EdgeFaces`,
`ef[e]` slices) instead of a `Vec` per edge; and `fillet`/`chamfer`'s `rebuild_loop` takes that
`edge_faces` as a borrow rather than recomputing it once per face. Together those cut a default
rebuild's allocation churn ~77% (fillet_edges ~92%).

A second pass went after *work* rather than churn. The scaling harness is `tests/scale.rs` — every
test in it is `#[ignore]`d, so reach for it with
`cargo test --release -p gridfinity-cad --test scale -- --ignored --nocapture`
(`scale_report` for the cost curve, `scale_features` for what the optional features cost,
`scale_profile` for the per-metric table at `SCALE_WH=48x48`, `tess_bench` for the tessellator
alone). Four changes, biggest first:

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

## The allocator is the thing to watch, not the allocation sites

A 1476-cell rebuild makes **132794 allocations / 187 MB of churn**, averaging ~1.4 kB. On Windows'
`System` allocator that was ~12.5 ms of a ~118 ms rebuild, and the expensive half was **free**, not
`malloc` (13.4 ms vs 7.7 ms measured inside the allocator). Switching the binary's `CountingAlloc`
to wrap **mimalloc** took a 1476-cell build 69.5 → 44.9 ms and tessellation 12.8 → 9.6 ms, three
runs out of three — one line, and by a wide margin the largest single win in this file's history.

The lesson to keep: **chasing individual allocation sites here is not worth it.** `plan: peg loop`
alone makes 69474 of those allocations (47 per cell, over half the total), and the targeted fixes
that removed hundreds of them all measured flat. The count is spread thin across `Vec<Seg>` loops,
sketch wrappers and label `String`s, none of them a hotspot. `alloc_report`
(`SCALE_WH=48x48 cargo test -p gridfinity-cad alloc_report -- --nocapture`) prints the
per-scope allocation table; use it to confirm churn has not regressed, not to hunt for sites.

`perf_counters_see_a_real_build` needs an inner wall that *crosses* the compartment boundary. With
box rejection in place a free-standing wall yields no crossing pair at all, so `seg_seg_points`
legitimately never gets called and the metric never fires.

`badapple.rs` plays a 64x48 silhouette clip as bins, and its `Worker` is a **two-stage pipeline**:
a builder thread turns each frame's connected components into `Solid`s and streams them over a
depth-4 `sync_channel` to a tessellating thread, which accumulates the frame's vertices and emits a
`FrameResult`. Channel ordering does the synchronisation -- the `Piece::End` marker cannot overtake
the pieces before it, so no counting is needed. `Solid` is already `Send`, so this needed no kernel
change, and `pipelined_worker_matches_serial_build` asserts the pipeline reproduces the serial
`build_frame` vertex-for-vertex in the same order.
**The depth is the whole story.** Tessellation is only 29% of a frame at these parameters
(`height_units: 2`, no floor fillet, `arc_segs_per_quarter: 1`), so the ceiling is 1.41x, and with a
single frame in flight the pipeline drains at every frame boundary and *loses* ~8% to channel
overhead. `PIPELINE_DEPTH` frames in flight is what buys the overlap: measured 39.9 ms/frame serial,
43.3 at depth 1, 35.0 at depth 3. Raising the depth costs up to that many frames of display lag and
builds frames that a late `try_recv` may discard, so do not raise it without re-measuring.
`Worker::try_recv` returns how many results it collapsed, because the caller tracks frames in flight
and silently dropping one desynchronises that count into a spin.

`debugger.rs` is the construction debugger (right panel, toggled from the params panel). It calls
`gridfinity::program(&p)` to get the model's op list, caches per-prefix face counts for display,
and rebuilds the solid via `program::run(&prog, |i| enabled[i])` whenever the user steps or
toggles. The App's `regenerate` switches between `gridfinity::build(&p)` (debug off) and the
debugger's subset build (debug on) — both feed the same `tessellate` → vertex-buffer upload path.
Its **Profile rebuilds** checkbox enables `perf` around one `build_solid` and shows wall time, the
per-metric table (heaviest first, bar scaled to the heaviest row since the timings nest) and
allocation count / churn / peak.
