# Repository Development Guide

Sole operative LLM guide. Other agent files import it, never duplicate it. When the user asks to "remember" something, record it here — not in any other memory store.

This is an index, not a manual. It says where things live and which rules are not visible from the source. For how anything works, read the code.

**Never spawn subagents.** No Task/Agent tool calls, no Explore/Plan/general-purpose delegation, no background agents — not for searching, not for planning, not for review. Do the work inline with your own tools. A subagent starts cold, re-derives context that is already in the conversation, and reports back a summary that has to be re-verified anyway. This holds even when the task looks broad or the user says "thorough"; the only exception is the user explicitly asking for a subagent by name.

## Scope

React 19 + Vite 6 + TypeScript app generating printable Gridfinity STLs. "GUI" in user requests means this web app, not the egui debugger in `crates/gridfinity-app`. Pick local implementation details freely; ask before changing architecture, user-visible semantics, compatibility policy, or scope. Keep changes narrow, preserve unrelated working-tree changes, inspect call sites first — trace store, worker boundary, preview, and export paths.

**The kernel is in scope, not off limits.** When a model defect traces back to a missing capability in `crates/gridfinity-cad/src/kernel/`, extend the kernel rather than degrading the model around it or writing the case off as a known failure. Missing analytic primitives and missing B-rep operators are the expected answer to a hard case (see `crates/CLAUDE.md`'s "no mesh operations" rule, which says the same thing) — do not treat "that would need a kernel change" as a reason to stop.

## Structure

- **The repository is two projects side by side.** `web/` is the whole npm/Vite app — `package.json`, every config, `src/`, `public/`, `e2e/`, `scripts/`, and the gitignored `src/wasm/` the kernel compiles into — and **every `npm` command runs from `web/`**. `crates/` is the cargo workspace and **every `cargo` command runs from the repository root**. `docs/` holds reference material that is not source (`xt_format.pdf`, the Parasolid XT format manual `kernel/xt/` is written against); `examples/` holds worked `optimize` inputs. Nothing but `Cargo.toml`/`Cargo.lock`, the two guides, `README.md`, `LICENSE`, `.gitignore` and `.github/` sits at the root.
- `web/src/main.tsx` mounts; `web/src/App.tsx` = Mantine AppShell; `web/src/store.ts` = Zustand state + design commands; `web/src/theme.ts` = Mantine defaults.
- `web/src/lib/`: `types.ts` contracts, `gridfinitySpec.ts` normative dimensions, `coordinates.ts` editor→generation transforms, `binParameters.ts` per-bin worker input, `geometryCache.ts` triangle cache, `preview.ts` viewer layout, `edges.ts`, `cuts.ts` cut planning, `printers.ts` fit, `geometry/`, `export/printableObjects.ts` splitting+naming, `export/stl.ts`, `export/parasolid.ts` XT worker round trip + download.
- `web/src/lib/project/`: what is left of the Projects feature on this side — `rects.ts` render-time box measurements, `drawer.ts` drawer→grid+packing area, `layout.ts` the drawer bin, `defaults.ts` (incl. the `PACK_RESTARTS` labels), `storage.ts` localStorage. **The optimizer is not here**: it is `crates/gridfinity-cad/src/project/` and is reached through wasm.
- Geometry runs in `web/src/workers/geometry.worker.ts` via `web/src/hooks/useBinGeometry.ts`; layout optimization in `web/src/workers/pack.worker.ts` via `web/src/hooks/usePackLayout.ts`; Parasolid export in a short-lived `web/src/workers/export.worker.ts`, spawned per download by `web/src/lib/export/parasolid.ts` — deliberately not the geometry worker, whose pool and leading-edge scheduling must not gain a second message type.
- `web/src/components/`: left panel = Shape/Walls/Cuts editors; right panel = Dimensions/Features/Printer-fit/Display accordions; `viewer/ModelViewer.tsx` hosts the Rust renderer. `project/` holds the Project mode panels and canvases.
- The cargo workspace is rooted at the repository root and its crates live in `crates/` — the geometry kernel, the renderer, the wasm bindings, and `gridfinity-app`, which is **one binary**: the egui debugger with no arguments, and the headless drawer fitter with `optimize`. `default-members` points at it, so `cargo run` opens the app and `cargo run -- optimize …` fits a drawer; bare `cargo build`/`cargo test` narrow to that crate, so pass `-p` or `--workspace`. See `crates/CLAUDE.md`. Every `cargo` command runs from the repository root. **The kernel models in `f64`** — `Vec2`/`Vec3` are `DVec2`/`DVec3`, and the only narrowing to `f32` is at the four functions feeding binary STL and the wgpu vertex buffers. `gridfinity-app` is mixed on purpose (egui screen space is `f32`); everything else, the wasm boundary included, is `f64` end to end. **The kernel's analytic surfaces and curves are deliberately Parasolid's**, in Parasolid's own parameters -- a cone is one nappe, a torus's major radius is signed to name the spindle sheet, an ellipse is a principal pair, and every direction is an f64-unit `Dir`. That is what keeps `kernel/xt/` a transcription rather than a translation, and a change that makes a kernel surface diverge from the node it is written as pays for it there.
- Scripts in `web/scripts/`; Vitest beside source as `*.test.ts`; browser tests in `web/e2e/`.

## Conventions

TypeScript ESM, function components, two-space indent. `PascalCase.tsx`, `useName.ts`, camelCase helper exports. Shared contracts in `types.ts`, domain logic in `web/src/lib/`.

**Say as much as possible in function doc comments, and say it as a transformation from input to output.** A doc comment names what the function is given, what it returns, and the rule connecting them — the shape of the data, the coordinate space it is in, the units, what the caller must have already established, and what is true of the result afterwards. Write it in the indicative about *this* function's mapping, not as a summary of the algorithm inside and not as advice to the reader; if the body changes but the mapping does not, the comment should not need touching. A function whose mapping cannot be stated that way is doing more than one thing and gets split until each part can be.

**A file still carries one paragraph at the top**, describing what the file as a whole holds and how its functions fit together — the context that no single doc comment owns. A file that cannot be described in one paragraph is holding more than one thing and gets split.

**Bodies carry no inline comments.** Rationale that belongs to a step rather than to the signature goes where it is enforced rather than merely stated: into an **assertion message**, which is the repo's preference over a comment anyway — an invariant stated in an `assert!` is checked, one stated in a comment is not — or, when it is a campaign finding rather than a local fact, into `AGENTS.md` or `crates/CLAUDE.md`. Never delete reasoning to satisfy any of this; relocate it. Files predating the current rule convert as they are next edited, not in sweeps of their own.

Prefer Mantine controls/layout over custom UI. Cross-app control styling → `theme.ts`; global layout and library workarounds → `web/src/index.css`; SVG editor styles → `web/src/components/sidebar/editor.css`; Project-mode SVG styles → `web/src/components/project/project.css`. No fixed design constants in JSX unless data-driven.

## Rules that the source does not show

- Tabs read/write only through `useAppStore()` commands. Keep `Design`, `BinParameters`, `Bin` plain and structured-clone compatible.
- The UI alone enforces validity — controls constrain their own ranges and dependent values. No store clamping, no validation layer. A shape change resets that bin's openings, walls, and cuts, then reseeds required cuts.
- **A carved piece is checked where it is produced, not where it is written.** `carve_to_cells` asserts that every piece it returns is a closed manifold, audits clean, is bounded by exactly one shell per island of its cells with material inside every one of them, and carries no vertex or edge that nothing names. That covers the STL path, the X_T path, the preview and the `optimize` command at once, because a piece is only ever made in that one place. Do not add a second, weaker check downstream, and do not relax this one: a shell too many is material that broke off the part, and nothing after the carve can see it — a detached lump tessellates and welds like any other closed surface.
- `generateGeometry()` is the sole production geometry path. Geometry must not plan cuts, name parts, inspect printers, validate input, normalize coordinates, or localize. Manifoldness is verified in the Rust kernel only; no TypeScript manifold verifier exists or may be reintroduced.
- Preview data may be grouped, coloured and positioned; export data must preserve coordinates, topology, orientation, and per-piece meaning. Both consume the identical `BinPiece.vertices` buffer (`Tessellation::welded_render_buffer()`); a piece never moves in the buffer. Never export the raw unwelded `render_buffer()` — it leaks.
- **Split pieces sit exactly where the kernel put them.** There is no kerf and no preview gap: carved pieces abut on the cut plane. The viewer's "Show gaps" button drives an *explode view* instead — `previewLayout()` hands each piece a unit `apartDirection` (piece centroid away from the bin centroid), the wasm `Viewer` owns the displacement (`set_explode`, millimetres along that direction), and `ModelViewer` eases the distance per frame. Nothing about it reaches the geometry or the export path.
- Cut faces glow. `previewLayout()` also derives `cutSegments` — (axis, coordinate, spanStart, spanEnd) quadruples for every edge where a piece meets *another piece of the same bin* — and `add_piece` flags a triangle `FLAG_CUT` when all three of its vertices land on one. That works only because `welded_render_buffer()` emits per-triangle vertex triples; a welded index buffer would bleed the flag onto neighbouring faces.
- `Design` stays in editor row-down coordinates; `buildBinParameters()` mirrors every bin together. Partition pieces before mirroring so piece indexes keep editor meaning. The camera is natively Z-up — never transform meshes for orientation.
- `useBinGeometry` schedules on the leading edge, not a debounce. Every exit path from a run must settle the dirty flag, or the preview freezes.
- The alpha generator assumes every bin is edge-connected and valid. Add no geometry-side component normalization, repair, rejection, or fallback. Enclosed holes stay supported.
- Render quality is a **pinned user setting, never adaptive**. A frame-time controller would make the preview change appearance while the user is judging a part.
- **Reach for a permanent assertion before instrumentation.** When something is wrong, the default move is to write the invariant it violates into the code as a real `assert!` at the point it is relied on — not an `eprintln!`, an env-gated dump, or a scratch probe test. The assertion finds the defect just as well, names it at its source instead of two layers downstream, stays behind to catch the next one, and costs nothing to clean up. Temporary instrumentation is the fallback for when you genuinely cannot state the invariant yet, and it comes out before the change lands.
- **No test may allow a failure.** There is no expected-failure, known-defect or tolerated-signature mechanism anywhere in the suite, and none may be added — not in the fuzzer, not in Vitest, not in Playwright. `tests/fuzz.rs` had one (`Options::known`: substrings of a failure message that were counted, printed and then excluded from the assert) and it is deleted. A forgiveness list is how a real defect keeps a green tick indefinitely, and the entry always outlives the diagnosis that justified it. A red profile naming a real bug is the correct state; fix the model, or leave it red and say so in the report.
- **A fillet that does not land is an error**, the way it is in a commercial modeller — not a corner quietly left sharp. `fillet_best_effort` degrades on purpose so the *user* still gets a part, but every fuzz profile holds the model to every blend it asked for, and `opening_keeps_the_fillet` additionally holds that no compartment the bin rounds when closed comes back sharp when a wall opening is added. It checks the *report* as well as the solid, because a clean `BlendReport` is vacuously clean at zero requested — a change that stops the model asking for the fillet outscores one that asks and is refused, so the gate alone rewards deleting the request. Never judge a cavity change on `FILLET_FAILED` counts without also reading `made()`.
- The Rust kernel asserts to high hell: every relied-on invariant gets a real `assert!` at the point it is relied on, and spending most of the runtime inside asserts is acceptable. Never `debug_assert!` — `--release` compiles it out. See `crates/CLAUDE.md`.
- **State every invariant, and state it mathematically.** Whenever a step relies on something being true — a normal is unit, a loop is simple, two half-edges leaving a vertex have distinct directions, a parameter lands inside its range, a boolean's output has the area its inputs imply — assert exactly that, at the point it is relied on, in the form a proof would state it. An assert that only checks a proxy (non-empty, non-NaN, "looks plausible") is worse than none: it passes while the property it stands for is violated. Prefer an exact predicate to a tolerance; where a tolerance is unavoidable, name the quantity it bounds and why that bound is the right one. A new operator is not finished until the properties it promises are asserted where it promises them.
- `ModelViewer.tsx` publishes `data-render-quality` and `data-explode` imperatively via `dataset` inside the render loop. Routing per-frame reads through React state makes the camera stutter.

## Projects

A *project* is a drawer plus the objects to organize in it. The pipeline is drawer → objects → pack → walls → one `BinDesign`. `appMode` in the store switches the whole shell between `'bins'` and `'project'`; nothing else about the two modes is shared.

- **The drawer is one bin.** In the web app's Project mode it is divided by ordinary `Wall`s, which flow `store` → `buildBinParameters`/`mirrorWall` → `geometry.worker` → `gridfinity-wasm` `InnerWall` → kernel like any hand-drawn wall.
- **`optimize` states the cavity instead of dividing it, in every one of its modes.** `--mode` is required and says what the drawer becomes: `walls` is one drawer-wide bin, `bins` is one Gridfinity bin per object, sized to hold that object's whole quantity as its own compartments and trimmed to the cells those compartments reach, so an L-shaped object gets an L-shaped bin, `hybrid` is the same bins with objects sharing one where sharing recovers cells (`crates/gridfinity-app/src/grouping.rs`), and `auto` is `hybrid` where the drawer holds it and `walls` where it does not. Both state the cavity the same way. A `LogicalBin` can carry `pockets` — the compartments outright, as rectangles — and when it does, `plan_cavities` authors those and never runs the cell walk. So the bin is **solid everywhere a pocket is not**: the space no object was packed into is material rather than an open pocket of air, and the material between two compartments is what the pockets leave standing rather than an `InnerWall`. A pocket is a placement's claim inset by `divider_thickness / 2`, which lands its wall exactly where the generated divider's face stood, so compartments are the size they always were and every object keeps the whole `clearance + floor_fillet` its claim reserved. Overlapping pockets merge, which is how a multi-box object gets its L-shaped compartment. **The web app's Project mode still divides** — it goes on emitting walls, so its leftover space is still hollow; changing that is a user-visible call.
- **The optimizer is Rust, and there is only one of it.** `rects.ts`/`pack.ts`/`walls.ts` were ported verbatim into `crates/gridfinity-cad/src/project/` (see `crates/CLAUDE.md`) and the TypeScript originals deleted, because the `optimize` command needs the same packer and two copies would drift. `pack.worker.ts` drives the wasm `PackSearch` in the same 8-restart chunks it used to drive the TS one; the message contract, revision guarding and progress reporting are unchanged.
- **`PackResult` carries `walls`.** The packer derives them in Rust and returns them with the placements, because `LayoutCanvas` draws the dividers *during React render* and a wasm call cannot be awaited there. `buildDrawerBin` takes `result.walls` too. Do not reintroduce a TypeScript wall generator to avoid the round trip.
- **`web/src/lib/project/rects.ts` is render-time measurement only** — `partsBounds`, `inflateParts`, `unionArea`, `partsConnected`, `rectRight`/`rectBottom`, `rectGrid`. Its far-edge accessors quantize, matching the Rust `Rect::right`/`bottom`; without that a box can read as covering none of its own lattice cell and `unionArea` returns 0.
- **The drawer's own bounds live in `drawer.ts`, not in the panel.** `MIN_DRAWER_MM` (one cell, below which `drawerGrid` floors to zero cells) and `maxDrawerMm(maxGrid)` (past which it clamps and every further millimetre is margin) are the two `NumberInput` limits in `ProjectPanel`, which had them as a literal `42` and `MAX_GRID * 42`. Both are properties of `drawerGrid`'s own clamping, so they belong beside it — the general rule against fixed design constants in JSX, applied where the constant is really a statement about a function elsewhere.
- **Packing is millimetre-space; only the bin outline is cell-quantized.** The outline is just `floor(drawer / 42)` cells (capped at `MAX_GRID`), and the leftover millimetres are reported as unusable drawer margin. Compartments sit at arbitrary mm positions inside `packingArea()`, the cavity interior inset by `perimeterClearancePerSide + perimeterThickness`. That inset treats the cavity as a plain rectangle and ignores its ~2.5 mm corner rounding; the claim margin's reserved fillet is what an object hard into a corner now leans on instead of its clearance.
- An object is one or more **connected** axis-aligned boxes. Each instance claims its boxes inflated by `PackInput::margin` = `clearance + floorFillet + dividerThickness / 2`, so packing the claims without overlap is what keeps divider centrelines apart and leaves each compartment interior the object plus its clearance plus the reserved fillet.
- **The floor fillet is reserved because the object rests on the floor.** A concave blend of radius `r` between a compartment's wall and its floor takes `r` of floor away from every wall, so a compartment is its stated size at mid height and `r` smaller all round where the object actually sits: with `fillet_radius` 2.5 against a 0.5 mm clearance, every object sank 2 mm into the blend on every side. Reserving it is what makes a packed layout one the objects fit *into* rather than one they only fit the plan view of, and it covers the corner rounding with it — a compartment corner of radius `rc` bulges in by `rc * (1 - 1/√2)` ≈ `0.3 * rc`, and the model never builds a fillet larger than the corner it turns. It is not free: reserving 2.5 mm costs 5 mm of every compartment dimension, and `settings.fillet_radius` is the lever. **`optimize` reserves it; the web app's Project mode does not yet** — `PackInput.floor_fillet` is `serde(default)` 0.0, so a caller that has not been taught about it claims exactly what it always did.
- **A `Placement` carries the instance's *claim*, not the object.** It is the object grown by `margin`, so it reaches the divider centrelines and the packing-area edge by construction; `inflate_parts` by the negative margin is what puts the object back. `Run::claim_margin` is the packer's own `PackInput::margin`, kept rather than restated, because a second derivation disagrees silently the moment the two are fed different numbers — which is exactly how `--view` came to draw claims and read as objects intersecting every wall.
- **The optimizer's budget is an iteration count, never wall-clock.** `PackEffort::restarts()` per tier (**250 / 2 000 / 10 000**) plus the fixed `PACK_SEED` is the whole reason a given drawer and object list always produce the same layout; a time budget would make the result depend on the machine. `pack_once` accumulates its cheap keys — placements, claim area, spread — as it places, and does not re-derive them by re-measuring afterwards; the **tidiness** terms are the exception and cannot be accumulated, being properties of the whole arrangement, so they are measured once on the finished layout at the end of the same function and carried on the `PackResult` rather than recomputed by any caller. `PACK_RESTARTS` in `defaults.ts` only *labels* the tiers in the panel; the budget is the Rust table.
- **Once everything fits, the search optimises for how the layout *looks*.** `project/tidy.rs` scores a finished layout six ways, each a fraction of its own worst case with 0 the tidiest: unshared divider `lines` and `runs` (from the same `merge_segments` that derives the walls, so the thing measured is the thing seen), leftover broken into `fragments`, `slivers` of leftover narrower than the narrowest claim placed, `grouping` of an object's instances, and `balance` of the layout's centre of area. `better` compares placements, then claim area, then that weighted score, then `spread` — so **a prettier layout can never cost a placed object**, and on a drawer everything fits in the third key is the one the whole budget is spent on. `balance` carries the smallest weight because it pulls against `fragments`: leftover gathered into one block means the claims crowd one end. A restart also varies the **scan axis** (rows-first or columns-first, `Scan`) and a per-instance **jitter** that declines the first few bands, because permuting the instance order alone reaches only bottom-left-greedy layouts — `examples/drawer.toml` exhausted that neighbourhood inside 250 restarts. **Project mode in the web app gets all of this**, so its layouts move too; `pack.worker.ts` chunks by a share of the budget rather than a flat 8 restarts, or a bigger budget would be spent on `setTimeout` round trips.
- **Every generated divider is extended by half its thickness at both ends.** Without it the kernel's `region_difference` leaves a half-thickness gap at each junction and adjacent compartments leak into each other. It is safe because the `t/2` band around a claim boundary is reserved by construction and belongs to no compartment interior. `walls.test.ts` flood-fills the packing area and asserts no compartment centre reaches another — that is the test that catches a junction gap.
- Collinear and duplicated boundary runs are merged per line, so two abutting compartments share **one** divider rather than two stacked ones. Runs lying on the packing-area boundary are dropped (that is the bin's own perimeter wall) and runs shorter than `MIN_GENERATED_WALL_LENGTH` are dropped with them.
- Applying a layout **replaces `design.bins` entirely** with the single drawer bin, sizes the editor grid to the drawer, and returns to `'bins'` mode. Any project edit clears the layout, because a stale layout no longer describes the objects.
- Projects persist to `localStorage` under `gridfinity-expanded.projects.v1` through `subscribeProjectStorage()`, wired once from `main.tsx` — deliberately **not** zustand `persist` middleware, which would drag storage into `store.ts` and make the store tests touch a `localStorage` that does not exist under vitest's node environment. A missing key, unparseable JSON, or a different `version` all fall back to "no saved projects" silently; never throw and never block startup. Bump the version and add a migration rather than reinterpreting an old blob.
- **The fitted drawer ships with its grid, cut beside the bin's seams rather than on them.** `optimize` builds a second `Params` in `Mode::Baseplate` over the same cells and its **own** `split_lines`, staggered off the bin's by `compute_staggered_split_lines`; `settings.baseplate = false` turns it off. **Staggering is what makes the stack hold itself together**: a bin piece spanning a plate seam pegs into both plate pieces and holds them, a plate piece spanning a bin seam holds the bin pieces, so no piece can leave without lifting the pieces it laps. Cut on one shared set of lines, every seam in the drawer lay in one plane and the whole stack parted along it. The plate's plan is measured in **millimetres against its own footprint** -- flange included -- rather than in whole cells, which is also what stopped a fitted plate outgrowing the bed the bin was split for; it takes the fewest pieces that both print and keep off the bin's lines, standing each seam as far from the bin's as that count allows. Where no staggered plan prints, the plate falls back to the bin's lines and `Run::interlocked()` is false, which the report's `interlock` field and a warning both say out loud. `Run::plate_split_lines` / `plate_parts` are the plate's own partition -- the Printing table reads each body's cells off its own -- and `plate_stagger_cost` is the extra pieces keeping off the bin's seams cost, warned about when it is not zero. **The plate is built to the drawer, not to the grid**: `Params::plate_margin_x`/`_y` carry `DrawerGrid`'s leftover millimetres onto it, and the plate's outline stands half of each outside the cells on that axis's two sides, so the grid stays centred and the plate's outer dimension is the drawer's own stated inside measurement. That is what makes the stack a snug fit -- the plate is the body that touches the drawer walls, so it is the body that grows, and the bin, the packing area and every compartment are untouched. Both piece lists reach the export and the report through `Run::all_pieces()` — a file that is written is a file the Soundness section accounts for. Project mode in the web app has no baseplate: `gridfinity-wasm` hardcodes `Mode::Bin`, and exposing it is a user-visible call.
- **`ModelViewer` stays mounted in project mode**, covered by `.project-workspace` rather than unmounted, so switching modes never tears down and rebuilds the wgpu device. It keeps rendering behind the cover.
- Inner walls are the kernel's weakest surface (`fuzz_inner_walls` / `fuzz_tidy_inner_walls`, see `crates/CLAUDE.md`), and a packed drawer emits dozens of them into one bin, all terminating where a divider meets the cavity's rounded corner. `a_drawer_bin_partitioned_into_compartments_is_watertight` is the gate: its 7×5 cells and 23 dividers are the **verbatim output** of `buildDrawerBin` for a 300 × 210 mm drawer holding nine objects, not a hand-written approximation of it. Re-dump and replace that fixture if the wall generator changes. A bin the kernel still cannot build surfaces as red `FLAG_BAD` geometry rather than crashing.

## Renderer

`crates/gridfinity-render` is a shared `wgpu` + `glam` crate with no egui or wasm deps, consumed by the desktop app (`gridfinity-app`) and the web app (`gridfinity-wasm`). Keep front-end concerns out; change its camera, shaders, or vertex format only with both consumers in mind.

- One WGSL source per module in `shaders.rs`, compiled by naga for every backend. The unit tests parse and validate all three modules, so a broken shader fails `cargo test` rather than only the browser.
- `prepare()` records every offscreen pass into its own command buffer; `blit()` draws the finished image into the caller's render pass. The caller must not give that pass a depth attachment.
- Web targets WebGPU and falls back to WebGL2 automatically. `Viewer` is created through the async `create_viewer()`, not a constructor.
- `Rgba16Float` is probed once against the adapter; failing the probe drops the chain to `Rgba8Unorm` with bloom off.
- The blit decodes to linear when its destination format is sRGB, because the hardware re-encodes on write.
- The tenth vertex float is a **flag, not a boolean**: `FLAG_NONE` / `FLAG_BAD` (the pulsing red error rim) / `FLAG_CUT` (the slower, contrast-hued glow on a split's cut faces, hue-opposed to the piece's own colour so it reads against every bin colour). `fs_mesh` tests the wider band first. Adding a value means widening that ladder in `shaders.rs`, not reusing a threshold.
- **The scene is rendered at `Renderer::set_render_scale`, the blit stretches it back.** `prepare` rasterises into an offscreen of the viewport's size scaled by it and `blit` still covers the whole rectangle, so a fraction below 1 buys pixels back without moving anything on screen. The desktop `--view` window sets it from the display: `viewport.rs::render_scale` caps the 3D view at `MAX_RENDER_PIXELS_PER_POINT` (1.5) physical pixels per point, so a 2x or 3x screen renders the scene once rather than four or nine times over while egui keeps drawing the panels and their text at full density. The scene size is in the accumulation key, so changing the scale restarts accumulation rather than resolving a stale target, and `PostUniform::source_texel` is what the blit's FXAA steps by — the destination texel is not the source's once the two sizes differ. The web `Viewer` leaves the scale at 1 and still renders at the canvas's full `devicePixelRatio`.
- **Medium and High shade a checkerboard half of the pixels per frame and read the other half back out of the accumulation buffer** (`Quality::checkerboard`, false at Low, which has no accumulation to read). A pixel's cell is `(floor(x) + floor(y)) & 1` in the *offscreen's* grid — scene, reflection, occlusion and accumulation targets are all `scene_viewport`-sized and rendered at origin `(0, 0)`, so one parity means one pixel in all of them. `checker_mode` is the schedule: sample 0 shades everything, so the first resolved frame is complete and nobody ever sees a half-black image; from sample 1 the parities alternate. `accumulation_weight` is the consequence — the running mean's divisor is that parity's own write count, `(n + 1) / 2 + 1`, not the frame count, and the non-checkerboard path keeps `1 / (n + 1)`. Sample budgets are unchanged, so High converges to 24 samples per pixel rather than 48 and Medium to 8 rather than 16: the trade bought is time to converge, not final quality. Checkerboarded are the two lit mesh draws (scene and reflection), both occlusion passes, and the accumulate; full rate are the shadow map, the depth/normal prepass (`fs_occlusion` samples it at arbitrary offsets, so it must be complete), and everything downstream of the accumulation buffer — bloom, resolve, FXAA, blit — because that buffer is always complete. It is derived from the *pinned* quality level and must never become a frame-time feedback loop, for the same reason the level itself is not adaptive.
- **A blur consuming a checkerboarded target steps by two texels**, which is the single rule governing the occlusion denoise and the reflection gloss blur. `blur_stride` names it; `gaussian_blur` also divides its sigma by the stride, so a doubled step with half the taps is the same physical blur read on one parity. Bloom keeps stride 1 — its source is the accumulation buffer.
- **A masked fragment returns black; it never discards.** `discard` would stop the depth write, and `fs_resolve` reads the scene depth for depth of field, so a checkerboarded depth buffer would blur half the pixels and not the other half. Returning `vec4<f32>(0.0)` keeps depth complete, keeps early-Z, and costs nothing because the accumulate pass never reads those pixels. `fs_accumulate` is the one pass a `discard` is right in — its target is blended, and leaving the running mean untouched is the whole intent. **The guard also sits after `fwidth`, not before it**: the checkerboard splits every 2×2 quad exactly two ways, so a derivative taken after the mask reads lanes that never ran. `the_only_derivative_in_the_mesh_shader_sits_in_uniform_control_flow` fails the build on the ordering.
- **`select` is not a branch — it evaluates every operand.** `select(a / b, fallback, b != 0)` still divides by zero, and the resulting NaN is free to reach the result through whatever arithmetic the backend lowers `select` to. Guard a division with `if`, or remove it. `no_select_guards_a_division_it_has_already_evaluated` fails the build on the pattern.
- **The GI bounce reads the frame it just wrote.** That feedback loop is contractive in magnitude (`GI_BOUNCE_STRENGTH` < 1) but not in NaN: one non-finite pixel re-seeds itself every frame and spreads by the sample radius, so the history sample is rejected against `GI_BOUNCE_HISTORY_CEILING` first. Every comparison against a NaN is false, which is what makes that one test reject it. Anything else that samples the previous frame needs the same guard.

Changing the geometry pipeline (`web/src/lib/geometry/`, `web/src/lib/project/`, `crates/gridfinity-cad/src/project/`, `web/src/workers/geometry.worker.ts`, `web/src/workers/pack.worker.ts`, `web/src/workers/export.worker.ts`, `web/src/hooks/useBinGeometry.ts`, `web/src/hooks/usePackLayout.ts`, `web/src/lib/{binParameters,coordinates,geometryCache,preview,cuts,gridfinitySpec,edges}.ts`, `web/src/lib/export/printableObjects.ts`, `web/src/lib/export/parasolid.ts`) or the viewer (`web/src/components/viewer/ModelViewer.tsx`, `crates/gridfinity-render/`, `crates/gridfinity-wasm/src/viewer.rs`) or the `optimize` command (`crates/gridfinity-app/src/{optimize,grouping,input,export,report}.rs`) requires updating this guide in the same change.

## Validation

Every `npm` command below runs from `web/`; every `cargo` command from the repository root.

- `npm run lint` — Oxlint
- `npm run test` — Vitest
- `npm run build` — type-check + Vite build
- `npm run build:wasm` — rebuild `web/src/wasm/` from the Rust workspace, unconditionally
- `npm run dev` — runs `build-wasm.mjs --if-needed` first, so the dev server never serves a
  `web/src/wasm/` older than the kernel. Staleness is the newest mtime among the workspace's
  `.rs`/`.toml`/`.lock`/`.wgsl` files (`target/` excluded) against the oldest artifact in
  `web/src/wasm/`; a missing artifact always rebuilds. It is a **startup check, not a watcher** —
  editing Rust while the server runs does nothing until you restart it. A failing kernel build
  fails `npm run dev` rather than falling through to a stale artifact; `npm run dev:nowasm` skips
  the check when you want the UI without the Rust toolchain. `npm run build` does **not** check —
  `ci.yml` and `deploy.yml` each run `npm run build:wasm` as their own step beforehand.
- `cargo test --release -p gridfinity-cad --lib` — geometry kernel suite, the printability gate
- `cargo test --release -p gridfinity-cad --test asserts` — assertion coverage, read off the
  crate's own AST with `syn`: no `debug_assert!`, no bare `.unwrap()` outside tests, every production
  assertion carries a message, and a per-file ratchet on functions that assert nothing. The ratchet
  fails in both directions — add assertions and you must lower the budget. See `crates/CLAUDE.md`.
- `cargo test --release --workspace` — full gate; the fuzzers and benchmarks are `#[ignore]`d out of it
- `cargo test --release --workspace -- --ignored --nocapture` — the fuzzers, benchmarks and perf reports, all `#[ignore]`d so no ordinary run pays for them
- `npm run test:e2e` — Chromium Playwright smoke
- `npm run classify:changes -- <base> <head>` — CI gate classification
- `cargo run -- optimize <in.toml> --mode <auto|bins|hybrid|walls> -o <out> [--format <stl|parasolid_x_t>] [--view]` — headless drawer
  fitting. The command line is a **`clap`** declaration (`optimize::Args`), so the spellings, the help
  text and every refusal come from it. **`--mode` is mandatory and has no default**: `walls` builds the
  whole drawer as one bin hollowed to a compartment per object, `bins` builds one Gridfinity bin per
  object, `hybrid` builds those bins with **objects sharing one where sharing pays**, and the four
  produce entirely different sets of parts out of one file, so an invocation that
  has not said which it wants has not said what it wants built — `auto` says it too, and says
  **the smallest bins the drawer can be fitted with**: it plans a hybrid fit first and builds the one
  drawer-wide bin only where that plan is refused, because a discrete bin is the smaller print and
  the smaller thing to lose to a failed one. `hybrid` can only match or beat `bins` (its search starts
  from one bin per object and asserts it never returns worse), which is why `auto` prefers it. `FitMode` is what was asked for and `Built` what a
  finished `Run` is, so nothing downstream of the plan carries an arm it can never take; `Run::fell_back`
  is the bin plan's refusal an automatic run fell back from, and the report's Drawer section prints
  both. A drawer neither plan holds fails with the **walls** refusal, which names the shortfall against
  the whole drawer rather than against one bin. **An instance the packer cannot place
  fails the run** — `refuse_unplaced`, asked before any geometry is built, so a refusal writes nothing;
  an object taller than the cavity and a piece the bed cannot take are still warnings. `-o` names the output and **at least one of `-o` and `--view`
  must be given** — an invocation that neither writes nor shows asks for nothing, and `clap` refuses
  it. `--format` is inferred when `-o` ends in `.x_t` and required otherwise, because an STL run
  writes a *directory* of one file per piece and a directory's name declares nothing; a `--format`
  that is given is checked against the path rather than overridden by it, so `parasolid_x_t` demands
  a `.x_t` output and `stl` refuses one ending in `.stl` or `.x_t`. `--view` with no `-o` fits, opens
  the window, and writes nothing. `examples/drawer.toml` is a worked `walls` input and
  `examples/drawer-of-bins.toml` a worked `bins` one. **`settings.grid_size` is the cell pitch**, `GRID_PITCH`
  (42 mm) unless the file names another: the whole run is measured in it -- how many cells fit the drawer,
  the packing area, the bin, the baseplate and what fits the bed -- and the standard's dimensions that are
  measured *from a cell edge* (the three peg widths, the fastener bores, the baseplate's carving reach)
  move with it while every absolute one (heights, `OUTER_R`, wall thickness) stays put. Below
  `MIN_GRID_PITCH` the peg profile does not close and the run is refused; at or below
  `MIN_FASTENER_GRID_PITCH` (22.5 mm) a cell cannot hold four fastener bores, so `magnets`/`screws` are
  refused there too. A file that does not state it builds exactly what it always did.
  **Every measurement in the file is a `Length`** —
  a bare number is millimetres, a string carries its unit (`"2.1 in"`, `"40cm"`, `"1 ft"`) — so the
  drawer, the settings' thicknesses, the bed and every object size or box may be written as measured;
  `examples/ikea-alex-drawer-1.toml` is the imperial one. It writes the **baseplate** beside the bin unless `settings.baseplate = false` — a bin carries a connector peg under every cell and has nothing to sit in without one — and writes it at the **drawer's** size rather than the grid's, spanning the margin the Drawer section reports so the stack cannot slide. The plate is **cut beside the bin's seams, never on them**, so each body spans the other's seams and the assembly moves as one piece; the report's Printing section names both sets of lines and what they interlock to. `--view` shows the plate under the bin, in grey and **exploded along its own bands**, which is that interlock made visible -- a plate piece stands off where the bin's does not, so the piece of each that laps the other's seam reads at a glance. The right panel's *Display* section carries a *Baseplate* checkbox that turns it off, and it appears only for a fit that has one. Everything in that view is **named on screen** (`scene_labels`): the bin, the baseplate, and every object the packer placed, an object that does not clear the cavity in red. **One label per item, never per piece** -- a bin cut into six is still one bin, and an object crossing a cut or made of several boxes is still one object -- each riding the band its own centre stands in so it is drawn on the item rather than in a gap. Its report's **Soundness** section names what was checked and on what; a failure is a named error and exit 1, and never a partial file — `fit` runs under the app's `catch` and the STL writer tessellates every piece before writing any. `--view` opens the fit in the egui debugger. **A split bin previews as its carved pieces**, because the kernel abuts them exactly and an unexploded split bin looks like an unsplit one -- the viewport's *Show gaps* button closes them back up, and **every packed object as a solid white box** cut on the same lines and moved with the piece it lies in -- flagged `FLAG_BAD` (the pulsing red rim) where the object stands taller than its compartment. `explode.rs` owns both: the displacement is per *band*, `SPLIT_APART_MM` (3 mm) between adjacent bands, so each cut opens by exactly one gap rather than fanning the pieces radially. See `crates/CLAUDE.md`.

Lint + build on every non-trivial code change. Don't add Vitest coverage by default during rapid feature development; run existing Vitest when changing printer, cut-to-part, or export behavior it covers.

Run the Rust suite for every print-affecting change. **Always pass `--release`** — the lib gate runs in 0.2s release against ~1.7s debug. `--release` compiles out `debug_assert!`, so nothing load-bearing may live in one.

**Fuzzing is opt-in, not routine.** All eight profiles in `tests/fuzz.rs` are `#[ignore]`d, so `cargo test --workspace` does not run them and neither does CI; they run only with `-- --ignored`. Reach for one when you are working on what it covers, or as a deliberate campaign — `tests/fuzz.rs` is a single generate/check/shrink path and each `#[test]` is an `Options` value aimed at one corner of the model, so the name tells you the coverage: `fuzz_inner_walls`, `fuzz_tidy_inner_walls`, `fuzz_wall_openings`, `fuzz_openings_and_inner_walls`, `fuzz_stripped_polyominoes`, `fuzz_bin_shapes`, `fuzz_split_pieces`, `fuzz_params_broad`. Nothing about them is weakened by being opt-in: `gate()` still asserts zero failures and no profile forgives anything. Three of the eight are red on real defects — `fuzz_inner_walls` at 20/150, `fuzz_params_broad` at 8/400 and `fuzz_wall_openings` at 2/150 (see `crates/CLAUDE.md`'s profile table for the per-profile counts, which are the numbers to compare against) — check the table before assuming a failure is yours, and compare **case counts**, not distinct-defect counts: a new assertion that names a defect the old message absorbed raises the class count without a single extra failing case. The benchmarks (`--test scale`, `perf_report`, `alloc_report`) are `#[ignore]`d the same way. With those out, the whole workspace runs in ~5 s.

Use Playwright for every browser-visible change. **Never run visual or browser verification yourself** — no browser automation, no screenshot capture, no ad-hoc scripts standing in for looking at the app. Write and hand over the test or the exact steps; the user drives the browser and reports back.

Complete means required commands finished with confirmed successful exit codes. Timeouts, truncated output, and partial runs are not successes. Final reports list checks run, results, and any required or relevant checks omitted.

## CI Classification

CI always runs lint, Vitest, build. The classifier adds gates fail-safe:

- Playwright: runtime UI, entrypoints, styles, store, hooks, workers, shared types, dependencies, build config
- Rust: geometry, cut/part generation, STL export, geometry workers, geometry-consumed config, anything under `crates/`, and the workspace `Cargo.toml`/`Cargo.lock`. `web/src/lib/project/` counts — it authors the walls the kernel builds.
- Both: ambiguous shared runtime files
- Neither: docs-only, isolated test-only

Classify unrecognized paths conservatively. Leave `.github/workflows/deploy.yml` behavior alone unless deployment is explicitly in scope.

## Pull Requests

Work in a dedicated feature branch in a new worktree off latest `origin/main`, not on `main`; target `main`. Short imperative commit subjects. PRs describe user-visible changes, list validation commands, link issues, include screenshots/recordings for UI changes, call out printability and manifold implications for geometry/export changes, and carry any `AGENTS.md` updates. Use `--body-file` for multiline `gh` bodies — escaped `\n` renders literally.

## Known Limitations

**Two wired export formats: STL (triangles, per part) and Parasolid X_T (analytic B-rep, one multi-body file).** The X_T button rebuilds each bin once via `build_bin_solid`/`carve_to_cells` — the same pairing `generate_geometry` uses — and hands every piece's `Solid` to the kernel's `to_xt_text`; nothing on that path is tessellated. The file is unitless metres (`mm / 1000`, `res_size` 1000, `res_linear` 1e-8); measured f32 deviation of kernel points from the emitted analytic forms is ~3e-9 m, inside the declared resolution. A kernel refusal (a surface with a non-positive radius, a face whose points do not lie on it, an internal void) surfaces as red text beside the buttons rather than a download. See `crates/CLAUDE.md`'s `xt/` section.

**Onshape imports all five ladder files clean.** Getting there took two rounds against a real reader, both
invisible to a green suite: the header had to be padded to the 80-character records a Parasolid frustrum
writes, and every *tilted* direction had to be normalised in f64 rather than the kernel's f32, which leaves
a unit vector 6.7e-8 off unit — six times the 1e-8 the file declares as its resolution. Only the second
faulted geometry, and only on non-axis-aligned normals, so `1-cube` passed while `2`..`5` each faulted on
a peg chamfer. See `crates/CLAUDE.md` for both, and for the validator tolerances that were loose enough to
hide the second. **An import is still the acceptance test and it is the user's to run** — nothing in this
repo can run a Parasolid reader — the writer's own round-trip tests share their author's reading of
the manual with the writer, so they agree with it rather than checking it. `kernel/xt/reader` and
`kernel/xt/validate` are the independent restatement (`validate_xt` reports findings; the corruption tests
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
