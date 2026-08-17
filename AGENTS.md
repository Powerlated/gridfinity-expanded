# Repository Development Guide

Sole operative LLM guide. Other agent files import it, never duplicate it. When the user asks to "remember" something, record it here — not in any other memory store.

This is an index, not a manual. It says where things live and which rules are not visible from the source. For how anything works, read the code.

**Never spawn subagents.** No Task/Agent tool calls, no Explore/Plan/general-purpose delegation, no background agents — not for searching, not for planning, not for review. Do the work inline with your own tools. A subagent starts cold, re-derives context that is already in the conversation, and reports back a summary that has to be re-verified anyway. This holds even when the task looks broad or the user says "thorough"; the only exception is the user explicitly asking for a subagent by name.

## Scope

React 19 + Vite 6 + TypeScript app generating printable Gridfinity STLs. "GUI" in user requests means this web app, not the egui debugger in `rust/crates/gridfinity-gui`. Pick local implementation details freely; ask before changing architecture, user-visible semantics, compatibility policy, or scope. Keep changes narrow, preserve unrelated working-tree changes, inspect call sites first — trace store, worker boundary, preview, and export paths.

**The kernel is in scope, not off limits.** When a model defect traces back to a missing capability in `rust/crates/gridfinity-cad/src/kernel/`, extend the kernel rather than degrading the model around it or writing the case off as a known failure. Missing analytic primitives and missing B-rep operators are the expected answer to a hard case (see `rust/CLAUDE.md`'s "no mesh operations" rule, which says the same thing) — do not treat "that would need a kernel change" as a reason to stop.

## Structure

- `src/main.tsx` mounts; `src/App.tsx` = Mantine AppShell; `src/store.ts` = Zustand state + design commands; `src/theme.ts` = Mantine defaults.
- `src/lib/`: `types.ts` contracts, `gridfinitySpec.ts` normative dimensions, `coordinates.ts` editor→generation transforms, `binParameters.ts` per-bin worker input, `geometryCache.ts` triangle cache, `preview.ts` viewer layout, `edges.ts`, `cuts.ts` cut planning, `printers.ts` fit, `geometry/`, `export/printableObjects.ts` splitting+naming, `export/stl.ts`.
- `src/lib/project/`: the drawer-fitting Projects feature — `rects.ts` rectilinear geometry, `drawer.ts` drawer→grid+packing area, `pack.ts` the optimizer, `walls.ts` placements→dividers, `layout.ts` the drawer bin, `defaults.ts`, `storage.ts` localStorage.
- Geometry runs in `src/workers/geometry.worker.ts` via `src/hooks/useBinGeometry.ts`; layout optimization in `src/workers/pack.worker.ts` via `src/hooks/usePackLayout.ts`.
- `src/components/`: left panel = Shape/Walls/Cuts editors; right panel = Dimensions/Features/Printer-fit/Display accordions; `viewer/ModelViewer.tsx` hosts the Rust renderer. `project/` holds the Project mode panels and canvases.
- `rust/` is the geometry kernel and renderer workspace; see `rust/CLAUDE.md`.
- Scripts in `scripts/`; Vitest beside source as `*.test.ts`; browser tests in `e2e/`.

## Conventions

TypeScript ESM, function components, two-space indent. `PascalCase.tsx`, `useName.ts`, camelCase helper exports. Shared contracts in `types.ts`, domain logic in `src/lib/`.

**Say as much as possible in function doc comments, and say it as a transformation from input to output.** A doc comment names what the function is given, what it returns, and the rule connecting them — the shape of the data, the coordinate space it is in, the units, what the caller must have already established, and what is true of the result afterwards. Write it in the indicative about *this* function's mapping, not as a summary of the algorithm inside and not as advice to the reader; if the body changes but the mapping does not, the comment should not need touching. A function whose mapping cannot be stated that way is doing more than one thing and gets split until each part can be.

**A file still carries one paragraph at the top**, describing what the file as a whole holds and how its functions fit together — the context that no single doc comment owns. A file that cannot be described in one paragraph is holding more than one thing and gets split.

**Bodies carry no inline comments.** Rationale that belongs to a step rather than to the signature goes where it is enforced rather than merely stated: into an **assertion message**, which is the repo's preference over a comment anyway — an invariant stated in an `assert!` is checked, one stated in a comment is not — or, when it is a campaign finding rather than a local fact, into `AGENTS.md` or `rust/CLAUDE.md`. Never delete reasoning to satisfy any of this; relocate it. Files predating the current rule convert as they are next edited, not in sweeps of their own.

Prefer Mantine controls/layout over custom UI. Cross-app control styling → `theme.ts`; global layout and library workarounds → `src/index.css`; SVG editor styles → `src/components/sidebar/editor.css`; Project-mode SVG styles → `src/components/project/project.css`. No fixed design constants in JSX unless data-driven.

## Rules that the source does not show

- Tabs read/write only through `useAppStore()` commands. Keep `Design`, `BinParameters`, `Bin` plain and structured-clone compatible.
- The UI alone enforces validity — controls constrain their own ranges and dependent values. No store clamping, no validation layer. A shape change resets that bin's openings, walls, and cuts, then reseeds required cuts.
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
- The Rust kernel asserts to high hell: every relied-on invariant gets a real `assert!` at the point it is relied on, and spending most of the runtime inside asserts is acceptable. Never `debug_assert!` — `--release` compiles it out. See `rust/CLAUDE.md`.
- **State every invariant, and state it mathematically.** Whenever a step relies on something being true — a normal is unit, a loop is simple, two half-edges leaving a vertex have distinct directions, a parameter lands inside its range, a boolean's output has the area its inputs imply — assert exactly that, at the point it is relied on, in the form a proof would state it. An assert that only checks a proxy (non-empty, non-NaN, "looks plausible") is worse than none: it passes while the property it stands for is violated. Prefer an exact predicate to a tolerance; where a tolerance is unavoidable, name the quantity it bounds and why that bound is the right one. A new operator is not finished until the properties it promises are asserted where it promises them.
- `ModelViewer.tsx` publishes `data-render-quality`, `data-badapple-frame` and `data-explode` imperatively via `dataset` inside the render loop. Routing per-frame reads through React state makes the camera stutter. For the same reason, never route `#badapple` clip frames through React state or `add_piece`.

## Projects

A *project* is a drawer plus the objects to organize in it. The pipeline is drawer → objects → pack → walls → one `BinDesign`. `appMode` in the store switches the whole shell between `'bins'` and `'project'`; nothing else about the two modes is shared.

- **The drawer is one bin, and the optimizer writes its `walls`.** One compartment per placed object, divided by ordinary `Wall`s. This is why the feature is pure TypeScript: `Wall` already flows `store` → `buildBinParameters`/`mirrorWall` → `geometry.worker` → `gridfinity-wasm` `InnerWall` → kernel, so no Rust change was needed and none should be added for it.
- **The drawer's own bounds live in `drawer.ts`, not in the panel.** `MIN_DRAWER_MM` (one cell, below which `drawerGrid` floors to zero cells) and `maxDrawerMm(maxGrid)` (past which it clamps and every further millimetre is margin) are the two `NumberInput` limits in `ProjectPanel`, which had them as a literal `42` and `MAX_GRID * 42`. Both are properties of `drawerGrid`'s own clamping, so they belong beside it — the general rule against fixed design constants in JSX, applied where the constant is really a statement about a function elsewhere.
- **Packing is millimetre-space; only the bin outline is cell-quantized.** The outline is just `floor(drawer / 42)` cells (capped at `MAX_GRID`), and the leftover millimetres are reported as unusable drawer margin. Compartments sit at arbitrary mm positions inside `packingArea()`, the cavity interior inset by `perimeterClearancePerSide + perimeterThickness`. That inset treats the cavity as a plain rectangle and ignores its ~2.5 mm corner rounding; an object hard into a corner leans on its clearance.
- An object is one or more **connected** axis-aligned boxes. Each instance claims its boxes inflated by `clearance + dividerThickness / 2`, so packing the claims without overlap is what keeps divider centrelines apart and leaves each compartment interior exactly the object plus its clearance.
- **The optimizer's budget is an iteration count, never wall-clock.** `PACK_RESTARTS` per effort tier plus a fixed `PACK_SEED` is the whole reason a given drawer and object list always produce the same layout; a time budget would make the result depend on the machine. Measured cost on a 46-instance drawer: quick ~0.4 s, standard ~2.7 s, thorough ~11 s. `packOnce` accumulates its own score — do not re-derive it by re-measuring placements.
- **Every generated divider is extended by half its thickness at both ends.** Without it the kernel's `region_difference` leaves a half-thickness gap at each junction and adjacent compartments leak into each other. It is safe because the `t/2` band around a claim boundary is reserved by construction and belongs to no compartment interior. `walls.test.ts` flood-fills the packing area and asserts no compartment centre reaches another — that is the test that catches a junction gap.
- Collinear and duplicated boundary runs are merged per line, so two abutting compartments share **one** divider rather than two stacked ones. Runs lying on the packing-area boundary are dropped (that is the bin's own perimeter wall) and runs shorter than `MIN_GENERATED_WALL_LENGTH` are dropped with them.
- Applying a layout **replaces `design.bins` entirely** with the single drawer bin, sizes the editor grid to the drawer, and returns to `'bins'` mode. Any project edit clears the layout, because a stale layout no longer describes the objects.
- Projects persist to `localStorage` under `gridfinity-expanded.projects.v1` through `subscribeProjectStorage()`, wired once from `main.tsx` — deliberately **not** zustand `persist` middleware, which would drag storage into `store.ts` and make the store tests touch a `localStorage` that does not exist under vitest's node environment. A missing key, unparseable JSON, or a different `version` all fall back to "no saved projects" silently; never throw and never block startup. Bump the version and add a migration rather than reinterpreting an old blob.
- **`ModelViewer` stays mounted in project mode**, covered by `.project-workspace` rather than unmounted, so switching modes never tears down and rebuilds the wgpu device. It keeps rendering behind the cover.
- Inner walls are the kernel's weakest surface (`fuzz_inner_walls` / `fuzz_tidy_inner_walls`, see `rust/CLAUDE.md`), and a packed drawer emits dozens of them into one bin, all terminating where a divider meets the cavity's rounded corner. `a_drawer_bin_partitioned_into_compartments_is_watertight` is the gate: its 7×5 cells and 23 dividers are the **verbatim output** of `buildDrawerBin` for a 300 × 210 mm drawer holding nine objects, not a hand-written approximation of it. Re-dump and replace that fixture if the wall generator changes. A bin the kernel still cannot build surfaces as red `FLAG_BAD` geometry rather than crashing.

## Renderer

`rust/crates/gridfinity-render` is a shared `wgpu` + `glam` crate with no egui or wasm deps, consumed by the egui debugger (`gridfinity-gui`) and the web app (`gridfinity-wasm`). Keep front-end concerns out; change its camera, shaders, or vertex format only with both consumers in mind.

- One WGSL source per module in `shaders.rs`, compiled by naga for every backend. The unit tests parse and validate all three modules, so a broken shader fails `cargo test` rather than only the browser.
- `prepare()` records every offscreen pass into its own command buffer; `blit()` draws the finished image into the caller's render pass. The caller must not give that pass a depth attachment.
- Web targets WebGPU and falls back to WebGL2 automatically. `Viewer` is created through the async `create_viewer()`, not a constructor.
- `Rgba16Float` is probed once against the adapter; failing the probe drops the chain to `Rgba8Unorm` with bloom off.
- The blit decodes to linear when its destination format is sRGB, because the hardware re-encodes on write.
- The tenth vertex float is a **flag, not a boolean**: `FLAG_NONE` / `FLAG_BAD` (the pulsing red error rim) / `FLAG_CUT` (the slower, contrast-hued glow on a split's cut faces, hue-opposed to the piece's own colour so it reads against every bin colour). `fs_mesh` tests the wider band first. Adding a value means widening that ladder in `shaders.rs`, not reusing a threshold.
- **`select` is not a branch — it evaluates every operand.** `select(a / b, fallback, b != 0)` still divides by zero, and the resulting NaN is free to reach the result through whatever arithmetic the backend lowers `select` to. Guard a division with `if`, or remove it. `no_select_guards_a_division_it_has_already_evaluated` fails the build on the pattern.
- **The GI bounce reads the frame it just wrote.** That feedback loop is contractive in magnitude (`GI_BOUNCE_STRENGTH` < 1) but not in NaN: one non-finite pixel re-seeds itself every frame and spreads by the sample radius, so the history sample is rejected against `GI_BOUNCE_HISTORY_CEILING` first. Every comparison against a NaN is false, which is what makes that one test reject it. Anything else that samples the previous frame needs the same guard.

Changing the geometry pipeline (`src/lib/geometry/`, `src/lib/project/`, `src/workers/geometry.worker.ts`, `src/workers/pack.worker.ts`, `src/hooks/useBinGeometry.ts`, `src/hooks/usePackLayout.ts`, `src/lib/{binParameters,coordinates,geometryCache,preview,cuts,gridfinitySpec,edges}.ts`, `src/lib/export/printableObjects.ts`) or the viewer (`src/components/viewer/ModelViewer.tsx`, `rust/crates/gridfinity-render/`, `rust/crates/gridfinity-wasm/src/viewer.rs`) requires updating this guide in the same change.

## Validation

- `npm run lint` — Oxlint
- `npm run test` — Vitest
- `npm run build` — type-check + Vite build
- `npm run build:wasm` — rebuild `src/wasm/` from the Rust workspace, unconditionally
- `npm run dev` — runs `build-wasm.mjs --if-needed` first, so the dev server never serves a
  `src/wasm/` older than the kernel. Staleness is the newest mtime among the workspace's
  `.rs`/`.toml`/`.lock`/`.wgsl` files (`target/` excluded) against the oldest artifact in
  `src/wasm/`; a missing artifact always rebuilds. It is a **startup check, not a watcher** —
  editing Rust while the server runs does nothing until you restart it. A failing kernel build
  fails `npm run dev` rather than falling through to a stale artifact; `npm run dev:nowasm` skips
  the check when you want the UI without the Rust toolchain. `npm run build` does **not** check —
  `ci.yml` and `deploy.yml` each run `npm run build:wasm` as their own step beforehand.
- `cd rust && cargo test --release -p gridfinity-cad --lib` — geometry kernel suite, the printability gate
- `cd rust && cargo test --release -p gridfinity-cad --test asserts` — assertion coverage, read off the
  crate's own AST with `syn`: no `debug_assert!`, no bare `.unwrap()` outside tests, every production
  assertion carries a message, and a per-file ratchet on functions that assert nothing. The ratchet
  fails in both directions — add assertions and you must lower the budget. See `rust/CLAUDE.md`.
- `cd rust && cargo test --release --workspace` — full gate incl. fuzzers (slow; pre-PR only)
- `cd rust && cargo test --release --workspace -- --ignored --nocapture` — the benchmarks and perf reports, which are `#[ignore]`d so no ordinary run pays for them
- `npm run test:e2e` — Chromium Playwright smoke
- `npm run classify:changes -- <base> <head>` — CI gate classification

Lint + build on every non-trivial code change. Don't add Vitest coverage by default during rapid feature development; run existing Vitest when changing printer, cut-to-part, or export behavior it covers.

Run the Rust suite for every print-affecting change. **Always pass `--release`** — the lib gate runs in 0.2s release against ~1.7s debug. `--release` compiles out `debug_assert!`, so nothing load-bearing may live in one.

**Don't run the long targets by default.** `--test fuzz` is for a deliberate pre-PR run or CI; say in the report when it was skipped. Run one profile by name when the change is in what it covers -- `tests/fuzz.rs` is a single generate/check/shrink path and each `#[test]` is an `Options` value aimed at one corner of the model, so the name tells you the coverage: `fuzz_inner_walls`, `fuzz_tidy_inner_walls`, `fuzz_wall_openings`, `fuzz_openings_and_inner_walls`, `fuzz_stripped_polyominoes`, `fuzz_bin_shapes`, `fuzz_split_pieces`, `fuzz_params_broad`. Five of the eight are currently red on real defects (see `rust/CLAUDE.md`'s profile table) — check the table before assuming a failure is yours. The benchmarks (`--test scale`, the `gridfinity-gui` badapple timings, `perf_report`, `alloc_report`) are `#[ignore]`d and only run when you ask for `--ignored`.

Use Playwright for every browser-visible change. **Never run visual or browser verification yourself** — no browser automation, no screenshot capture, no ad-hoc scripts standing in for looking at the app. Write and hand over the test or the exact steps; the user drives the browser and reports back.

Complete means required commands finished with confirmed successful exit codes. Timeouts, truncated output, and partial runs are not successes. Final reports list checks run, results, and any required or relevant checks omitted.

## CI Classification

CI always runs lint, Vitest, build. The classifier adds gates fail-safe:

- Playwright: runtime UI, entrypoints, styles, store, hooks, workers, shared types, dependencies, build config
- Rust: geometry, cut/part generation, STL export, geometry workers, geometry-consumed config, anything under `rust/`. `src/lib/project/` counts — it authors the walls the kernel builds.
- Both: ambiguous shared runtime files
- Neither: docs-only, isolated test-only

Classify unrecognized paths conservatively. Leave `.github/workflows/deploy.yml` behavior alone unless deployment is explicitly in scope.

## Pull Requests

Work in a dedicated feature branch in a new worktree off latest `origin/main`, not on `main`; target `main`. Short imperative commit subjects. PRs describe user-visible changes, list validation commands, link issues, include screenshots/recordings for UI changes, call out printability and manifold implications for geometry/export changes, and carry any `AGENTS.md` updates. Use `--body-file` for multiline `gh` bodies — escaped `\n` renders literally.

## Known Limitations

STL is the only wired export format.

**A sloped bin takes no inner walls.** Its cavity is not a z-prism, so the island a free-form wall is
carved as cannot meet the tilted floor; the walls are dropped and the bin builds without them. See
`rust/CLAUDE.md`.

**A sloped bin does take wall openings, and keeps its ramp.** It used to drop the slope for the whole
piece the moment any edge was open — a flat part for a user who asked for a ramp, built cleanly
enough that nothing but a fuzz profile saw it. The opened compartment's floor lies in the ramp now,
the standing wall stands on it, and a plinth carries the outline up to it. Its cavity corners are
still square. See `rust/CLAUDE.md`.

**A reentrant corner is rounded only when both its edges are walled.** Open both and the corner
squares: the outer fillet's arc stands 3.5 mm past both pitch lines, and with no wall there to be
the outside of, it was left standing alone as a 2.15 mm fin the full height of the bin. See
`rust/CLAUDE.md`.

**A wall opening whose run reaches a reentrant corner builds, and keeps its floor fillet.** It used
to panic in the open-run planner, which needed a straight perimeter run to pinch against; an opening
is a boolean now and needs none. The fillet was the second half of it: the cavity's rounded corner
there is an arc, so the blend is a torus, and the wall it rolls against tapers to zero thickness
where the opened cavity meets the outline — so the chain both terminates *on an arc* and terminates
where no face can take the curve. `runout_torus` and `RunoutEnd::Flat` are what close it. See
`rust/CLAUDE.md`.

**An opening onto an enclosed hole's boundary is ignored.** `layout::effective_walls` drops it and
keeps the wall, so the bin builds and stays closed without the doorway. See `rust/CLAUDE.md`.
