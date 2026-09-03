# Repository Development Guide

Sole operative LLM guide. Other agent files import it, never duplicate it. When the user asks to "remember" something, record it here — not in any other memory store.

This is an index, not a manual. It says where things live and which rules are not visible from the source. For how anything works, read the code.

**Never spawn subagents.** No Task/Agent tool calls, no Explore/Plan/general-purpose delegation, no background agents — not for searching, not for planning, not for review. Do the work inline with your own tools. A subagent starts cold, re-derives context that is already in the conversation, and reports back a summary that has to be re-verified anyway. This holds even when the task looks broad or the user says "thorough"; the only exception is the user explicitly asking for a subagent by name.

## Scope

A Rust workspace: an **egui parametric CAD application** for designing connected
Gridfinity bins and exporting printable parts, plus the headless `optimize` command
that fits a drawer. One binary serves both, and the same application runs on the
desktop and in the browser -- the page is a canvas and nothing else. There is no
TypeScript, no npm and no HTML UI; "the app" always means the egui one.

Pick local implementation details freely; ask before changing architecture,
user-visible semantics, compatibility policy, or scope. Keep changes narrow,
preserve unrelated working-tree changes, and inspect call sites first.

**The kernel is in scope, not off limits.** When a model defect traces back to a
missing capability in the kernel, extend it rather than degrading the model around
it or writing the case off as a known failure. Missing analytic primitives and
missing B-rep operators are the expected answer to a hard case (see
`crates/CLAUDE.md`'s "no mesh operations" rule, which says the same thing) -- do
not treat "that would need a kernel change" as a reason to stop.

## Structure

**Every `cargo` command runs from the repository root**, where the virtual
workspace lives. Nothing but `Cargo.toml`/`Cargo.lock`, the two guides,
`README.md`, `LICENSE`, `THIRD_PARTY_LICENSES.md`, `.gitignore`, `.cargo/`,
`.github/`, and the CMake files that build the vendored kernel sits there.

- `crates/gridfinity-brep` -- the legacy analytic B-rep kernel. `glam` is its only
  dependency and it names no bin, cell or drawer.
- `crates/gridfinity-occt` -- a small, exception-safe C ABI over the vendored
  **Open CASCADE** kernel in `vendor/occt`, behind the `occt` feature.
- `crates/gridfinity-xt` -- the Parasolid XT transmit writer, its reader and its
  validator, over its own analytic vocabulary; reads OCCT bodies. No dependency on
  `gridfinity-brep`.
- `crates/gridfinity-model` -- the Gridfinity model on the kernel.
- `crates/gridfinity-project` -- the drawer fitter on the model.
- `crates/gridfinity-render` -- the shared `wgpu` renderer, no egui and no wasm.
- `crates/gridfinity-app` -- **one binary**: the egui application with no
  arguments, the drawer fitter with `optimize`. `crates/gridfinity-app/web/` is the
  browser page it is served behind.
- `crates/gridfinity-web` -- builds that page: links the app and OCCT into one
  Emscripten module and stages `dist/`.
- `docs/` holds reference material that is not source (`xt_format.pdf`, the
  Parasolid manual `gridfinity-xt` is written against); `examples/` holds worked
  `optimize` inputs; `vendor/occt` is the pinned OCCT submodule; `third_party/`
  holds the locally patched `eframe`, `egui-wgpu` and `winit`.

`default-members` is `gridfinity-app`, so bare `cargo build`/`cargo test` narrow to
it -- pass `-p` or `--workspace`. **The kernel models in `f64`**; the only narrowing
to `f32` is at the four functions feeding binary STL and the wgpu vertex buffers.
`gridfinity-app` is mixed on purpose (egui screen space is `f32`).

## Conventions

Rust, edition 2024, four-space indent. Every `cargo` command runs from the repository root.

**Say as much as possible in function doc comments, and say it as a transformation from input to output.** A doc comment names what the function is given, what it returns, and the rule connecting them — the shape of the data, the coordinate space it is in, the units, what the caller must have already established, and what is true of the result afterwards. Write it in the indicative about *this* function's mapping, not as a summary of the algorithm inside and not as advice to the reader; if the body changes but the mapping does not, the comment should not need touching. A function whose mapping cannot be stated that way is doing more than one thing and gets split until each part can be.

**A file still carries one paragraph at the top**, describing what the file as a whole holds and how its functions fit together — the context that no single doc comment owns. A file that cannot be described in one paragraph is holding more than one thing and gets split.

**Bodies carry no inline comments.** Rationale that belongs to a step rather than to the signature goes where it is enforced rather than merely stated: into an **assertion message**, which is the repo's preference over a comment anyway — an invariant stated in an `assert!` is checked, one stated in a comment is not — or, when it is a campaign finding rather than a local fact, into `crates/CLAUDE.md`. Never delete reasoning to satisfy any of this; relocate it. Files predating the current rule convert as they are next edited, not in sweeps of their own.

The window's palette, type scale and control metrics come from `gridfinity-app/src/theme.rs` and nowhere else; the shared controls live in `widgets.rs`. No fixed design constant sits inline unless it is data-driven.

## Rules that the source does not show

- **A carved piece is checked where it is produced, not where it is written.** `carve_to_cells` asserts that every piece it returns is a closed manifold, audits clean, is bounded by exactly one shell per island of its cells with material inside every one of them, and carries no vertex or edge that nothing names. That covers the STL path, the X_T path, the preview and the `optimize` command at once, because a piece is only ever made in that one place. Do not add a second, weaker check downstream, and do not relax this one: a shell too many is material that broke off the part, and nothing after the carve can see it — a detached lump tessellates and welds like any other closed surface.
- **The panels alone enforce validity** — a control constrains its own range and its dependent values. There is no clamping layer and no validation pass behind them. A shape change resets that bin's openings, walls and cuts, then reseeds the required cuts.
- `build_bin_solid`/`carve_to_cells` is the sole production geometry path. Geometry must not plan cuts, name parts, inspect printers or validate input. Manifoldness is verified in the kernel and nowhere else; no second verifier may be reintroduced.
- Preview data may be grouped, coloured and positioned; export data must preserve coordinates, topology, orientation, and per-piece meaning. Both consume the identical `BinPiece.vertices` buffer (`Tessellation::welded_render_buffer()`); a piece never moves in the buffer. Never export the raw unwelded `render_buffer()` — it leaks.
- **Split pieces sit exactly where the kernel put them.** There is no kerf and no preview gap: carved pieces abut on the cut plane. *Show gaps* drives an *explode view* instead, and `explode.rs` owns it — the displacement is per **band**, `SPLIT_APART_MM` between adjacent bands, so each cut opens by exactly one gap rather than fanning the pieces radially. Nothing about it reaches the geometry or the export path.
- The camera is natively Z-up — never transform meshes for orientation.
- The alpha generator assumes every bin is edge-connected and valid. Add no geometry-side component normalization, repair, rejection, or fallback. Enclosed holes stay supported.
- Render quality is a **pinned user setting, never adaptive**. A frame-time controller would make the preview change appearance while the user is judging a part.
- **Reach for a permanent assertion before instrumentation.** When something is wrong, the default move is to write the invariant it violates into the code as a real `assert!` at the point it is relied on — not an `eprintln!`, an env-gated dump, or a scratch probe test. The assertion finds the defect just as well, names it at its source instead of two layers downstream, stays behind to catch the next one, and costs nothing to clean up. Temporary instrumentation is the fallback for when you genuinely cannot state the invariant yet, and it comes out before the change lands.
- **No test may allow a failure.** There is no expected-failure, known-defect or tolerated-signature mechanism anywhere in the suite, and none may be added — not anywhere in the suite. `tests/fuzz.rs` had one (`Options::known`: substrings of a failure message that were counted, printed and then excluded from the assert) and it is deleted. A forgiveness list is how a real defect keeps a green tick indefinitely, and the entry always outlives the diagnosis that justified it. A red profile naming a real bug is the correct state; fix the model, or leave it red and say so in the report.
- **A fillet that does not land is an error**, the way it is in a commercial modeller — not a corner quietly left sharp. `fillet_best_effort` degrades on purpose so the *user* still gets a part, but every fuzz profile holds the model to every blend it asked for, and `opening_keeps_the_fillet` additionally holds that no compartment the bin rounds when closed comes back sharp when a wall opening is added. It checks the *report* as well as the solid, because a clean `BlendReport` is vacuously clean at zero requested — a change that stops the model asking for the fillet outscores one that asks and is refused, so the gate alone rewards deleting the request. Never judge a cavity change on `FILLET_FAILED` counts without also reading `made()`.
- The Rust kernel asserts to high hell: every relied-on invariant gets a real `assert!` at the point it is relied on, and spending most of the runtime inside asserts is acceptable. Never `debug_assert!` — `--release` compiles it out. See `crates/CLAUDE.md`.
- **State every invariant, and state it mathematically.** Whenever a step relies on something being true — a normal is unit, a loop is simple, two half-edges leaving a vertex have distinct directions, a parameter lands inside its range, a boolean's output has the area its inputs imply — assert exactly that, at the point it is relied on, in the form a proof would state it. An assert that only checks a proxy (non-empty, non-NaN, "looks plausible") is worse than none: it passes while the property it stands for is violated. Prefer an exact predicate to a tolerance; where a tolerance is unavoidable, name the quantity it bounds and why that bound is the right one. A new operator is not finished until the properties it promises are asserted where it promises them.
- **Invalid geometry must never take the window down.** Each bin is built through `main.rs`'s one `catch`, which converts an `Err` *and* an unwind into a message and swaps in a silent panic hook for the duration; a failed bin gets featureless placeholder geometry flagged `FLAG_BAD`, and export refuses while any bin is failing.

## Projects

A *project* is a drawer plus the objects to organize in it. The pipeline is drawer → objects → pack → walls → one `BinDesign`. The window has no project mode of its own; `optimize` is where a project is fitted.

- **The drawer is one bin**, and `optimize` is the only thing that fits one. The web app had a Project mode that divided that bin with ordinary `Wall`s; it is deleted, and nothing has replaced it in the window yet.
- **`optimize` states the cavity instead of dividing it, in every one of its modes.** `--mode` is required and says what the drawer becomes: `walls` is one drawer-wide bin, `bins` is one Gridfinity bin per object, sized to hold that object's whole quantity as its own compartments and trimmed to the cells those compartments reach, so an L-shaped object gets an L-shaped bin, `hybrid` is the same bins with objects sharing one where sharing recovers cells (`crates/gridfinity-app/src/grouping.rs`), and `auto` is `hybrid` where the drawer holds it and `walls` where it does not. Both state the cavity the same way. A `LogicalBin` can carry `pockets` — the compartments outright, as rectangles — and when it does, `plan_cavities` authors those and never runs the cell walk. So the bin is **solid everywhere a pocket is not**: the space no object was packed into is material rather than an open pocket of air, and the material between two compartments is what the pockets leave standing rather than an `InnerWall`. A pocket is a placement's claim inset by `divider_thickness / 2`, which lands its wall exactly where the generated divider's face stood, so compartments are the size they always were and every object keeps the whole `clearance + floor_fillet` its claim reserved. Overlapping pockets merge, which is how a multi-box object gets its L-shaped compartment. **The web app's Project mode still divides** — it goes on emitting walls, so its leftover space is still hollow; changing that is a user-visible call.
- **The optimizer is Rust, and there is only one of it.** `rects.ts`/`pack.ts`/`walls.ts` were ported verbatim into `crates/gridfinity-project/src/` (see `crates/CLAUDE.md`) and the TypeScript originals deleted, because the `optimize` command needs the same packer and two copies would drift.
- **`PackResult` carries `walls`.** The packer derives them in Rust and returns them with the placements, and `buildDrawerBin`'s successor takes `result.walls` too. They are derived once, where the packing is known; do not re-derive them from the placements anywhere else.
- **The drawer's own bounds belong beside the function that clamps them**, in `project/drawer.rs`: `MIN_DRAWER_MM` (one cell, below which `drawer_grid` floors to zero cells) and `max_drawer_mm(max_grid)` (past which it clamps and every further millimetre is margin). Both are properties of `drawer_grid`'s own clamping, so a panel offering the drawer's size takes its limits from there rather than restating them.
- **Packing is millimetre-space; only the bin outline is cell-quantized.** The outline is just `floor(drawer / 42)` cells (capped at `MAX_GRID`), and the leftover millimetres are reported as unusable drawer margin. Compartments sit at arbitrary mm positions inside `packingArea()`, the cavity interior inset by `perimeterClearancePerSide + perimeterThickness`. That inset treats the cavity as a plain rectangle and ignores its ~2.5 mm corner rounding; the claim margin's reserved fillet is what an object hard into a corner now leans on instead of its clearance.
- An object is one or more **connected** axis-aligned boxes. Each instance claims its boxes inflated by `PackInput::margin` = `clearance + floorFillet + dividerThickness / 2`, so packing the claims without overlap is what keeps divider centrelines apart and leaves each compartment interior the object plus its clearance plus the reserved fillet.
- **The floor fillet is reserved because the object rests on the floor.** A concave blend of radius `r` between a compartment's wall and its floor takes `r` of floor away from every wall, so a compartment is its stated size at mid height and `r` smaller all round where the object actually sits: with `fillet_radius` 2.5 against a 0.5 mm clearance, every object sank 2 mm into the blend on every side. Reserving it is what makes a packed layout one the objects fit *into* rather than one they only fit the plan view of, and it covers the corner rounding with it — a compartment corner of radius `rc` bulges in by `rc * (1 - 1/√2)` ≈ `0.3 * rc`, and the model never builds a fillet larger than the corner it turns. It is not free: reserving 2.5 mm costs 5 mm of every compartment dimension, and `settings.fillet_radius` is the lever. **`optimize` reserves it; the web app's Project mode does not yet** — `PackInput.floor_fillet` is `serde(default)` 0.0, so a caller that has not been taught about it claims exactly what it always did.
- **A `Placement` carries the instance's *claim*, not the object.** It is the object grown by `margin`, so it reaches the divider centrelines and the packing-area edge by construction; `inflate_parts` by the negative margin is what puts the object back. `Run::claim_margin` is the packer's own `PackInput::margin`, kept rather than restated, because a second derivation disagrees silently the moment the two are fed different numbers — which is exactly how `--view` came to draw claims and read as objects intersecting every wall.
- **The optimizer's budget is an iteration count, never wall-clock.** `PackEffort::restarts()` per tier (**250 / 2 000 / 10 000**) plus the fixed `PACK_SEED` is the whole reason a given drawer and object list always produce the same layout; a time budget would make the result depend on the machine. `pack_once` accumulates its cheap keys — placements, claim area, spread — as it places, and does not re-derive them by re-measuring afterwards; the **tidiness** terms are the exception and cannot be accumulated, being properties of the whole arrangement, so they are measured once on the finished layout at the end of the same function and carried on the `PackResult` rather than recomputed by any caller.
- **Once everything fits, the search optimises for how the layout *looks*.** `project/tidy.rs` scores a finished layout six ways, each a fraction of its own worst case with 0 the tidiest: unshared divider `lines` and `runs` (from the same `merge_segments` that derives the walls, so the thing measured is the thing seen), leftover broken into `fragments`, `slivers` of leftover narrower than the narrowest claim placed, `grouping` of an object's instances, and `balance` of the layout's centre of area. `better` compares placements, then claim area, then that weighted score, then `spread` — so **a prettier layout can never cost a placed object**, and on a drawer everything fits in the third key is the one the whole budget is spent on. `balance` carries the smallest weight because it pulls against `fragments`: leftover gathered into one block means the claims crowd one end. A restart also varies the **scan axis** (rows-first or columns-first, `Scan`) and a per-instance **jitter** that declines the first few bands, because permuting the instance order alone reaches only bottom-left-greedy layouts — `examples/drawer.toml` exhausted that neighbourhood inside 250 restarts. The budget is spent in one call; nothing chunks it any more.
- **Once the search has chosen, `settle` tidies what it chose, and only `optimize` calls it.** `project/settle.rs` runs after packing, over the claims a mode has settled on: it **absorbs** every strip of leftover no wider than `settings.tidy_absorb` into the compartments facing it -- half each between two claims, the whole of it against the cavity wall -- **grows** every compartment wall that still has leftover in front of it, and **evens** the slack at the two ends of every slab, which is what centres a compartment in the bin around it. `tidy_absorb` is one lever with one meaning across all three: **how far a compartment wall may be pushed out**, so leftover survives exactly where it is wider than the walls facing it can travel, and `0` disables the pass. Then it **clamps**: an object stating `max_size` has its compartment pulled back to that afterwards, about its own centre, and the space given back becomes material. That is how an object says it must be held still -- a battery in a compartment 30 mm wider than itself lies over at an angle -- and it is stated in the object's own frame, so the quarter turn the packer chose swaps the two limits with it. `max_size` is `[width, depth]` with an **empty string for no limit on that axis**, is refused below the object's own size, and every file written without it is held to nothing. Together those are "a bin that is almost one pocket becomes exactly one pocket", "a compartment with air in front of it takes it", and "the leftover sits evenly instead of at one end". Every move is made across a **free band** -- a column or row of the slab no claim covers any part of -- and that restriction is the correctness argument, not a simplification: a free band separates the claims completely, so deleting, widening or narrowing it is a monotone remap of the band lines and cannot change which claims touch which. `settle_slab` does both operations on each axis and recurses into the blocks the surviving bands leave, which partitions the claims and so terminates. The band argument needs a whole free row or column, which a genuinely two-dimensional packing never has -- four objects interlocked in one bin cover every row and column of it -- so **the growth pass is what reaches that leftover**, pushing each of a claim's four walls into the space squarely in front of it, capped at `tidy_absorb` and refused wherever the grown claim would leave the cavity or meet another placement's claim. What is left after all three is leftover facing no wall squarely, which would need a compartment that is not a rectangle. A part never shrinks, so an object that fitted its compartment before the pass fits it after; a claim may move, and its object moves with it, being derived from the claim. A move that would put a claim outside the cavity is refused whole, which is how a bin whose cells are an L is settled without a non-rectangular slab: `cavity_region` states that cavity as the cells' own squares drawn back by `packing_inset` on every side with no neighbour, and `packing_area` is its rectangular case. It runs **after** `footprint_cells`, never before -- settling first would let a grown claim reach a cell that was about to be dropped and cost an L-shaped object its L-shaped bin -- and `PackResult::{walls, tidiness}` are re-derived from the settled placements, the one place the rule against re-deriving `tidiness` does not apply, because the search's own reading is of a layout that was not built. Only `optimize` calls it.
- **Every generated divider is extended by half its thickness at both ends.** Without it the kernel's `region_difference` leaves a half-thickness gap at each junction and adjacent compartments leak into each other. It is safe because the `t/2` band around a claim boundary is reserved by construction and belongs to no compartment interior. `layout_walls`'s own test flood-fills the packing area and asserts no compartment centre reaches another — that is the test that catches a junction gap.
- Collinear and duplicated boundary runs are merged per line, so two abutting compartments share **one** divider rather than two stacked ones. Runs lying on the packing-area boundary are dropped (that is the bin's own perimeter wall) and runs shorter than `MIN_GENERATED_WALL_LENGTH` are dropped with them.
- **The fitted drawer ships with its grid, cut beside the bin's seams rather than on them.** `optimize` builds a second `Params` in `Mode::Baseplate` over the same cells and its **own** `split_lines`, staggered off the bin's by `compute_staggered_split_lines`; `settings.baseplate = false` turns it off. **Staggering is what makes the stack hold itself together**: a bin piece spanning a plate seam pegs into both plate pieces and holds them, a plate piece spanning a bin seam holds the bin pieces, so no piece can leave without lifting the pieces it laps. Cut on one shared set of lines, every seam in the drawer lay in one plane and the whole stack parted along it. The plate's plan is measured in **millimetres against its own footprint** -- flange included -- rather than in whole cells, which is also what stopped a fitted plate outgrowing the bed the bin was split for; it takes the fewest pieces that both print and keep off the bin's lines, standing each seam as far from the bin's as that count allows. Where no staggered plan prints, the plate falls back to the bin's lines and `Run::interlocked()` is false, which the report's `interlock` field and a warning both say out loud. `Run::plate_split_lines` / `plate_parts` are the plate's own partition -- the Printing table reads each body's cells off its own -- and `plate_stagger_cost` is the extra pieces keeping off the bin's seams cost, warned about when it is not zero. **The plate is built to the drawer, not to the grid**: `Params::plate_margin_x`/`_y` carry `DrawerGrid`'s leftover millimetres onto it, and the plate's outline stands half of each outside the cells on that axis's two sides, so the grid stays centred and the plate's outer dimension is the drawer's own stated inside measurement. That is what makes the stack a snug fit -- the plate is the body that touches the drawer walls, so it is the body that grows, and the bin, the packing area and every compartment are untouched. Both piece lists reach the export and the report through `Run::all_pieces()` — a file that is written is a file the Soundness section accounts for.
- Inner walls are the kernel's weakest surface (`fuzz_inner_walls` / `fuzz_tidy_inner_walls`, see `crates/CLAUDE.md`), and a packed drawer emits dozens of them into one bin, all terminating where a divider meets the cavity's rounded corner. `a_drawer_bin_partitioned_into_compartments_is_watertight` is the gate: its 7×5 cells and 23 dividers are the **verbatim output** of `buildDrawerBin` for a 300 × 210 mm drawer holding nine objects, not a hand-written approximation of it. Re-dump and replace that fixture if the wall generator changes. A bin the kernel still cannot build surfaces as red `FLAG_BAD` geometry rather than crashing.

## Renderer

`crates/gridfinity-render` is a shared `wgpu` + `glam` crate with no egui or wasm deps, consumed by `gridfinity-app` on both of its targets. Keep front-end concerns out; change its camera, shaders or vertex format deliberately.

- One WGSL source per module in `shaders.rs`, compiled by naga for every backend. The unit tests parse and validate all three modules, so a broken shader fails `cargo test` rather than only the browser.
- `prepare()` records every offscreen pass into its own command buffer; `blit()` draws the finished image into the caller's render pass. The caller must not give that pass a depth attachment.
- The browser build reaches wgpu through Emscripten's GLES backend, which winit hands the canvas surface; `main.rs` follows eframe's winit runner there, not its wasm-bindgen `WebRunner`.
- `Rgba16Float` is probed once against the adapter; failing the probe drops the chain to `Rgba8Unorm` with bloom off.
- The blit decodes to linear when its destination format is sRGB, because the hardware re-encodes on write.
- The tenth vertex float is a **flag, not a boolean**: `FLAG_NONE` / `FLAG_BAD` (the pulsing red error rim) / `FLAG_CUT` (the slower, contrast-hued glow on a split's cut faces, hue-opposed to the piece's own colour so it reads against every bin colour). `fs_mesh` tests the wider band first. Adding a value means widening that ladder in `shaders.rs`, not reusing a threshold.
- **The scene is rendered at `Renderer::set_render_scale`, the blit stretches it back.** `prepare` rasterises into an offscreen of the viewport's size scaled by it and `blit` still covers the whole rectangle, so a fraction below 1 buys pixels back without moving anything on screen. The desktop `--view` window sets it from the display: `viewport.rs::render_scale` caps the 3D view at `MAX_RENDER_PIXELS_PER_POINT` (1.5) physical pixels per point, so a 2x or 3x screen renders the scene once rather than four or nine times over while egui keeps drawing the panels and their text at full density. The scene size is in the accumulation key, so changing the scale restarts accumulation rather than resolving a stale target, and `PostUniform::source_texel` is what the blit's FXAA steps by — the destination texel is not the source's once the two sizes differ. 
- **Medium and High shade a checkerboard half of the pixels per frame and read the other half back out of the accumulation buffer** (`Quality::checkerboard`, false at Low, which has no accumulation to read). A pixel's cell is `(floor(x) + floor(y)) & 1` in the *offscreen's* grid — scene, reflection, occlusion and accumulation targets are all `scene_viewport`-sized and rendered at origin `(0, 0)`, so one parity means one pixel in all of them. `checker_mode` is the schedule: sample 0 shades everything, so the first resolved frame is complete and nobody ever sees a half-black image; from sample 1 the parities alternate. `accumulation_weight` is the consequence — the running mean's divisor is that parity's own write count, `(n + 1) / 2 + 1`, not the frame count, and the non-checkerboard path keeps `1 / (n + 1)`. Sample budgets are unchanged, so High converges to 24 samples per pixel rather than 48 and Medium to 8 rather than 16: the trade bought is time to converge, not final quality. Checkerboarded are the two lit mesh draws (scene and reflection), both occlusion passes, and the accumulate; full rate are the shadow map, the depth/normal prepass (`fs_occlusion` samples it at arbitrary offsets, so it must be complete), and everything downstream of the accumulation buffer — bloom, resolve, FXAA, blit — because that buffer is always complete. It is derived from the *pinned* quality level and must never become a frame-time feedback loop, for the same reason the level itself is not adaptive.
- **A blur consuming a checkerboarded target steps by two texels**, which is the single rule governing the occlusion denoise and the reflection gloss blur. `blur_stride` names it; `gaussian_blur` also divides its sigma by the stride, so a doubled step with half the taps is the same physical blur read on one parity. Bloom keeps stride 1 — its source is the accumulation buffer.
- **A masked fragment returns black; it never discards.** `discard` would stop the depth write, and `fs_resolve` reads the scene depth for depth of field, so a checkerboarded depth buffer would blur half the pixels and not the other half. Returning `vec4<f32>(0.0)` keeps depth complete, keeps early-Z, and costs nothing because the accumulate pass never reads those pixels. `fs_accumulate` is the one pass a `discard` is right in — its target is blended, and leaving the running mean untouched is the whole intent. **The guard also sits after `fwidth`, not before it**: the checkerboard splits every 2×2 quad exactly two ways, so a derivative taken after the mask reads lanes that never ran. `the_only_derivative_in_the_mesh_shader_sits_in_uniform_control_flow` fails the build on the ordering.
- **`select` is not a branch — it evaluates every operand.** `select(a / b, fallback, b != 0)` still divides by zero, and the resulting NaN is free to reach the result through whatever arithmetic the backend lowers `select` to. Guard a division with `if`, or remove it. `no_select_guards_a_division_it_has_already_evaluated` fails the build on the pattern.
- **The GI bounce reads the frame it just wrote.** That feedback loop is contractive in magnitude (`GI_BOUNCE_STRENGTH` < 1) but not in NaN: one non-finite pixel re-seeds itself every frame and spreads by the sample radius, so the history sample is rejected against `GI_BOUNCE_HISTORY_CEILING` first. Every comparison against a NaN is false, which is what makes that one test reject it. Anything else that samples the previous frame needs the same guard.

Changing the geometry pipeline (`crates/gridfinity-model/src/gridfinity/`, `crates/gridfinity-project/`, `crates/gridfinity-model/src/{printers,subbin,layout}.rs`), the transmit writer (`crates/gridfinity-xt/`), the OCCT bridge (`crates/gridfinity-occt/`), the viewer (`crates/gridfinity-render/`, `crates/gridfinity-app/src/viewport.rs`), the browser build (`crates/gridfinity-web/`, `.cargo/config.toml`, `crates/gridfinity-app/web/`) or the `optimize` command (`crates/gridfinity-app/src/{optimize,grouping,input,export,report}.rs`) requires updating this guide in the same change.

## Validation

Every command runs from the repository root.

- `cargo build --release --workspace` — the whole workspace
- `cargo test --release -p gridfinity-brep --lib` — the legacy B-rep kernel suite
- `cargo test --release -p gridfinity-model --lib` — the model suite, the printability gate
- `cargo test --release -p gridfinity-model --test kernel_on_a_bin` — the kernel
  properties that want a *bin* to stand on
- `cargo test --release -p gridfinity-project --lib` — the drawer fitter
- `cargo test --release -p gridfinity-{brep,model,project} --test asserts` — assertion
  coverage, one run per crate, read off that crate's own AST with `syn`: no
  `debug_assert!`, no bare `.unwrap()` outside tests, every production assertion
  carries a message, and a per-file ratchet on functions that assert nothing. The
  ratchet fails in both directions. See `crates/CLAUDE.md`.
- `cargo test --release --workspace` — full gate; the fuzzers and benchmarks are
  `#[ignore]`d out of it
- `cargo test --release --workspace -- --ignored --nocapture` — the fuzzers,
  benchmarks and perf reports

The OCCT-backed crates need a built OCCT and are behind the `occt` feature:

```sh
cmake --preset occt-native
cmake --build --preset occt-native-install
OCCT_ROOT=target/occt-install/native cargo test --release -p gridfinity-occt --features occt
OCCT_ROOT=target/occt-install/native cargo test --release -p gridfinity-xt --features occt
```

`cargo run -p gridfinity-web --release` builds the browser page into `dist/`: it
configures and builds the `vendor/occt` submodule for the web on first use, links
the app and OCCT into one Emscripten module, and stages
`crates/gridfinity-app/web/` around it. It needs `wasm32-unknown-emscripten`,
Emscripten 6.0.5, wasm-bindgen-cli 0.2.126, CMake and Ninja; a local emsdk in
`target/emsdk` and wasm-bindgen in `target/tools/bin` are found automatically. The
link's flags live in `.cargo/config.toml`, each beside the reason it is needed.

`cargo run` opens the app; `cargo run -- optimize …` fits a drawer.

Build on every non-trivial code change.

Run the Rust suite for every print-affecting change. **Always pass `--release`** — the lib gate runs in 0.2s release against ~1.7s debug. `--release` compiles out `debug_assert!`, so nothing load-bearing may live in one.

**Fuzzing is opt-in, not routine.** All eight profiles in `crates/gridfinity-model/tests/fuzz.rs` are `#[ignore]`d, so `cargo test --workspace` does not run them and neither does CI; they run only with `-- --ignored`. Reach for one when you are working on what it covers, or as a deliberate campaign — `tests/fuzz.rs` is a single generate/check/shrink path and each `#[test]` is an `Options` value aimed at one corner of the model, so the name tells you the coverage: `fuzz_inner_walls`, `fuzz_tidy_inner_walls`, `fuzz_wall_openings`, `fuzz_openings_and_inner_walls`, `fuzz_stripped_polyominoes`, `fuzz_bin_shapes`, `fuzz_split_pieces`, `fuzz_params_broad`. Nothing about them is weakened by being opt-in: `gate()` still asserts zero failures and no profile forgives anything. Three of the eight are red on real defects — `fuzz_inner_walls` at 20/150, `fuzz_params_broad` at 8/400 and `fuzz_wall_openings` at 2/150 (see `crates/CLAUDE.md`'s profile table for the per-profile counts, which are the numbers to compare against) — check the table before assuming a failure is yours, and compare **case counts**, not distinct-defect counts: a new assertion that names a defect the old message absorbed raises the class count without a single extra failing case. The benchmarks (`--test scale`, `perf_report`, `alloc_report`) are `#[ignore]`d the same way. With those out, the whole workspace runs in ~5 s.

**Never run visual or browser verification yourself** — no browser automation, no screenshot capture, no ad-hoc scripts standing in for looking at the app. Build the page or the window and hand over the exact steps; the user drives it and reports back. There is no browser test suite: Playwright went with the npm project, and nothing has replaced it.

Complete means required commands finished with confirmed successful exit codes. Timeouts, truncated output, and partial runs are not successes. Final reports list checks run, results, and any required or relevant checks omitted.

## CI

`ci.yml` builds the workspace, runs the whole suite, builds OCCT for the host, runs
the two OCCT-backed suites, and builds the browser page. `deploy.yml` builds that
page and uploads `dist/` to Pages. There is no path classifier any more — it was a
Node script under `web/`, and CI is short enough now to run everything every time.

## Pull Requests

Work in a dedicated feature branch in a new worktree off latest `origin/main`, not on `main`; target `main`. Short imperative commit subjects. PRs describe user-visible changes, list validation commands, link issues, include screenshots/recordings for UI changes, call out printability and manifold implications for geometry/export changes, and carry any guide updates. Use `--body-file` for multiline `gh` bodies — escaped `\n` renders literally.

## Known Limitations

**Two wired export formats: STL (triangles, per part) and Parasolid X_T (analytic
B-rep, one multi-body file).** The X_T path rebuilds each bin once via
`build_bin_solid`/`carve_to_cells` and hands every piece to `gridfinity-xt`;
nothing on it is tessellated. The file is unitless metres (`mm / 1000`, `res_size`
1000, `res_linear` 1e-8), and measured deviation of kernel points from the emitted
analytic forms is ~3e-9 m, inside the declared resolution. A refusal (a surface
with a non-positive radius, a face whose points do not lie on it, an internal void)
surfaces as an error rather than a file.

**`gridfinity-xt` also reads OCCT bodies, and that is the migration's live edge.**
It owns the analytic vocabulary the format is written in and depends on
`gridfinity-brep` not at all. An edge OCCT holds as a B-spline is **not**
approximated: an edge is the intersection of the two faces meeting along it, which
the format states as INTERSECTION, so it crosses as a `Curve::Section` whose chart
names the branch while the two exact surfaces say where it is. A cut blended body
therefore transmits, which is what every printed piece is. Two things are still
refused by name: a **lofted or swept surface**, which has no analytic escape, and a
**seam edge** on a closed surface of revolution, used twice by one loop — which is
why no OCCT body can currently supply a CONE.

**Onshape imports all five ladder files clean.** Getting there took two rounds against a real reader, both
invisible to a green suite: the header had to be padded to the 80-character records a Parasolid frustrum
writes, and every *tilted* direction had to be normalised in f64 rather than the kernel's f32, which leaves
a unit vector 6.7e-8 off unit — six times the 1e-8 the file declares as its resolution. Only the second
faulted geometry, and only on non-axis-aligned normals, so `1-cube` passed while `2`..`5` each faulted on
a peg chamfer. See `crates/CLAUDE.md` for both, and for the validator tolerances that were loose enough to
hide the second. **An import is still the acceptance test and it is the user's to run** — nothing in this
repo can run a Parasolid reader — the writer's own round-trip tests share their author's reading of
the manual with the writer, so they agree with it rather than checking it. `gridfinity-brep/src/xt/reader` and
`.../xt/validate` are the independent restatement (`validate_xt` reports findings; the corruption tests
prove it catches them), and `writes_the_import_ladder` emits five files differing by one node class each so
that whichever a CAD system first refuses names the class at fault. Every field table but
INTERSECTION/CHART/LIMIT has been checked against the manual and is right. Do not assume a new X_T
behaviour works because the suite is green; the acceptance test is an import, and it is the user's to run.

**A `--mode bins` fit rounds every object up to whole cells, and can refuse a drawer that
`--mode walls` fits.** That rounding is what `--mode hybrid` recovers: several small objects
in one bin stand on fewer cells than one bin each, and `examples/drawer.toml` -- the worked
case a bin per object refuses outright -- fits as **three** grouped bins on 61 of its 63 cells.
Where even grouping does not fit, `--mode auto` builds the drawer-wide bin and reports the
refusal; an explicit `--mode bins` or `--mode hybrid` still refuses by name, because a user who
asked for the bins is not asking for the other body.

**Grouping is priced in cells, and the prices are the whole policy.** `grouping.rs` scores a
candidate partition as `cells + 0.25*air + 0.15*largest + 2.0*cut + 0.5*shared + 0.1*oblong`,
every term a number of **cells** so the trade reads as "is this worth a cell?" — `air` is the
unclaimed area inside the bins, `largest` the biggest bin, `cut` the bins the bed cannot take
whole, `shared` each bin's cells-per-object once for every object beyond the first in it, and
`oblong` how far from square the bins are. Fractions of their own worst cases were tried first,
the way `Tidiness` states its six, and are *wrong here*: a cell recovered is a small share of a
big drawer while an extra object is a large share of a handful of objects, so nothing grouped on
either worked example. A flat price per shared object is wrong in both directions at once — at
0.75 cells four bags of hardware would not share the one cell they all fit in, at 3.0 cells five
big tools still merged into one 68-cell body, which is the drawer-wide bin `walls` already
builds — so sharing is priced as a share of what is shared. Measured: the ikea drawer 82 → 75
cells and 5 bins → 3, `drawer-of-bins.toml` 33 → 24 cells and 4 bins → 2, `drawer.toml` from
refused to 61 cells in 3 bins. Pack time 0.13 s → 0.55 s on `drawer-of-bins.toml`, 1.45 s on the
ikea drawer. A discrete bin is a whole number of cells on each axis, and one object's
whole quantity goes in one bin, so a long thin object costs a bin the length of it and the cells
either side of it are that bin's own. `examples/drawer.toml` is exactly that case -- as discrete
bins its six objects want more cells than the 9 x 7 drawer has -- and the run refuses by name
rather than leaving an object out; `examples/drawer-of-bins.toml` is the same drawer with a list
that fits. Packing the *bins* tighter is a real improvement and is not
attempted: the outer pack is the ordinary `pack_layout` over their cell footprints, so it already
turns and interleaves L-shaped bins, but nothing reconsiders a bin's own size or shape once it is
chosen.

**A drawer fitted by `optimize` has no free-form inner walls at all**, and that is deliberate: its
cavity is stated as pockets, so it never enters the `InnerWall` path — the kernel's weakest surface,
which a packed drawer used to enter dozens of times per bin. The packer still derives its walls and
the report still counts them; they simply describe material now.

**A fitted baseplate is planned for the bed it actually reaches to, and staggering can cost it a
piece.** The plate has its own seams now, chosen in millimetres over its own footprint -- flange
included -- so it no longer inherits a plan measured in whole cells and no longer outgrows the bed the
bin was split for. What it can cost is an extra piece: where the bin's own chunks are already as wide
as the bed takes, no line both matches the bed and misses the bin's seams, so the plate divides one
step finer. The report warns with the number. The Printing table still measures every piece off its own
finished solid, and a plate that fits nothing is a named warning rather than a silent file. See
`crates/CLAUDE.md`.

**An insert is printed whole, and only ever holds one box.** `subbin` is refused on an object of
more than one box -- an insert is a rectangle, and an L-shaped object's compartment is not -- and
nothing splits an insert for the bed: one that does not fit is a named warning, not a cut. Both are
`optimize`-side limits; the body itself (`crates/gridfinity-model/src/subbin.rs`) is five rings and
six faces and would carve like anything else.

**A baseplate has no magnet or screw holes.** `build_baseplate` reads neither `magnet_holes` nor
`screw_holes` — the counterbore machinery is in the bin's `plan.rs` path only — so a drawer fitted
with `magnets = true` gets them in the bin and not in the grid under it. See `crates/CLAUDE.md`.

**A sloped bin takes no inner walls.** Its cavity is not a z-prism, so the island a free-form wall is
carved as cannot meet the tilted floor; the walls are dropped and the bin builds without them. See
`crates/CLAUDE.md`.

**An opened compartment takes no inner wall.** A wall the user drew across a compartment that has a
wall opening is dropped, full-height or partial-height; the bin builds without it. See
`crates/CLAUDE.md`.

**A sloped bin does take wall openings, and keeps its ramp.** It used to drop the slope for the whole
piece the moment any edge was open — a flat part for a user who asked for a ramp, built cleanly
enough that nothing but a fuzz profile saw it. The opened compartment's floor lies in the ramp now,
the standing wall stands on it, and a plinth carries the outline up to it. Its cavity corners are
still square. See `crates/CLAUDE.md`.

**A reentrant corner is rounded only when both its edges are walled.** Open both and the corner
squares: the outer fillet's arc stands 3.5 mm past both pitch lines, and with no wall there to be
the outside of, it was left standing alone as a 2.15 mm fin the full height of the bin. See
`crates/CLAUDE.md`.

**A wall opening whose run reaches a reentrant corner builds, and keeps its floor fillet.** It used
to panic in the open-run planner, which needed a straight perimeter run to pinch against; an opening
is a boolean now and needs none. The fillet was the second half of it: the cavity's rounded corner
there is an arc, so the blend is a torus, and the wall it rolls against tapers to zero thickness
where the opened cavity meets the outline — so the chain both terminates *on an arc* and terminates
where no face can take the curve. `runout_torus` and `RunoutEnd::Flat` are what close it. See
`crates/CLAUDE.md`.

**An opening onto an enclosed hole's boundary is ignored.** `layout::effective_walls` drops it and
keeps the wall, so the bin builds and stays closed without the doorway. See `crates/CLAUDE.md`.
