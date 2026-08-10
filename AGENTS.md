# Repository Development Guide

Sole operative LLM guide. Other agent files import it, never duplicate it. When the user asks to "remember" something, record it here — not in any other memory store.

This is an index, not a manual. It says where things live and which rules are not visible from the source. For how anything works, read the code.

## Scope

React 19 + Vite 6 + TypeScript app generating printable Gridfinity STLs. "GUI" in user requests means this web app, not the egui debugger in `rust/crates/gridfinity-gui`. Pick local implementation details freely; ask before changing architecture, user-visible semantics, compatibility policy, or scope. Keep changes narrow, preserve unrelated working-tree changes, inspect call sites first — trace store, worker boundary, preview, and export paths.

## Structure

- `src/main.tsx` mounts; `src/App.tsx` = Mantine AppShell; `src/store.ts` = Zustand state + design commands; `src/theme.ts` = Mantine defaults.
- `src/lib/`: `types.ts` contracts, `gridfinitySpec.ts` normative dimensions, `coordinates.ts` editor→generation transforms, `binParameters.ts` per-bin worker input, `geometryCache.ts` triangle cache, `preview.ts` viewer layout, `edges.ts`, `cuts.ts` cut planning, `printers.ts` fit, `geometry/`, `export/printableObjects.ts` splitting+naming, `export/stl.ts`.
- Geometry runs in `src/workers/geometry.worker.ts` via `src/hooks/useBinGeometry.ts`.
- `src/components/`: left panel = Shape/Walls/Cuts editors; right panel = Dimensions/Features/Printer-fit/Display accordions; `viewer/ModelViewer.tsx` hosts the Rust renderer.
- `rust/` is the geometry kernel and renderer workspace; see `rust/CLAUDE.md`.
- Scripts in `scripts/`; Vitest beside source as `*.test.ts`; browser tests in `e2e/`.

## Conventions

TypeScript ESM, function components, two-space indent. `PascalCase.tsx`, `useName.ts`, camelCase helper exports. Shared contracts in `types.ts`, domain logic in `src/lib/`.

Prefer Mantine controls/layout over custom UI. Cross-app control styling → `theme.ts`; global layout and library workarounds → `src/index.css`; SVG editor styles → `src/components/sidebar/editor.css`. No fixed design constants in JSX unless data-driven.

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
- The Rust kernel asserts to high hell: every relied-on invariant gets a real `assert!` at the point it is relied on, and spending most of the runtime inside asserts is acceptable. Never `debug_assert!` — `--release` compiles it out. See `rust/CLAUDE.md`.
- `ModelViewer.tsx` publishes `data-render-quality`, `data-badapple-frame` and `data-explode` imperatively via `dataset` inside the render loop. Routing per-frame reads through React state makes the camera stutter. For the same reason, never route `#badapple` clip frames through React state or `add_piece`.

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

Changing the geometry pipeline (`src/lib/geometry/`, `src/workers/geometry.worker.ts`, `src/hooks/useBinGeometry.ts`, `src/lib/{binParameters,coordinates,geometryCache,preview,cuts,gridfinitySpec,edges}.ts`, `src/lib/export/printableObjects.ts`) or the viewer (`src/components/viewer/ModelViewer.tsx`, `rust/crates/gridfinity-render/`, `rust/crates/gridfinity-wasm/src/viewer.rs`) requires updating this guide in the same change.

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
- `cd rust && cargo test --release --workspace` — full gate incl. fuzzers (slow; pre-PR only)
- `cd rust && cargo test --release --workspace -- --ignored --nocapture` — the benchmarks and perf reports, which are `#[ignore]`d so no ordinary run pays for them
- `npm run test:e2e` — Chromium Playwright smoke
- `npm run classify:changes -- <base> <head>` — CI gate classification

Lint + build on every non-trivial code change. Don't add Vitest coverage by default during rapid feature development; run existing Vitest when changing printer, cut-to-part, or export behavior it covers.

Run the Rust suite for every print-affecting change. **Always pass `--release`** — the lib gate runs in 0.2s release against ~1.7s debug. `--release` compiles out `debug_assert!`, so nothing load-bearing may live in one.

**Don't run the long targets by default.** `--test fuzz` is for a deliberate pre-PR run or CI; say in the report when it was skipped. Run one fuzzer by name when the change is in what it covers. The benchmarks (`--test scale`, the `gridfinity-gui` badapple timings, `perf_report`, `alloc_report`) are `#[ignore]`d and only run when you ask for `--ignored`.

Use Playwright for every browser-visible change. **Never run visual or browser verification yourself** — no browser automation, no screenshot capture, no ad-hoc scripts standing in for looking at the app. Write and hand over the test or the exact steps; the user drives the browser and reports back.

Complete means required commands finished with confirmed successful exit codes. Timeouts, truncated output, and partial runs are not successes. Final reports list checks run, results, and any required or relevant checks omitted.

## CI Classification

CI always runs lint, Vitest, build. The classifier adds gates fail-safe:

- Playwright: runtime UI, entrypoints, styles, store, hooks, workers, shared types, dependencies, build config
- Rust: geometry, cut/part generation, STL export, geometry workers, geometry-consumed config, anything under `rust/`
- Both: ambiguous shared runtime files
- Neither: docs-only, isolated test-only

Classify unrecognized paths conservatively. Leave `.github/workflows/deploy.yml` behavior alone unless deployment is explicitly in scope.

## Pull Requests

Work in a dedicated feature branch in a new worktree off latest `origin/main`, not on `main`; target `main`. Short imperative commit subjects. PRs describe user-visible changes, list validation commands, link issues, include screenshots/recordings for UI changes, call out printability and manifold implications for geometry/export changes, and carry any `AGENTS.md` updates. Use `--body-file` for multiline `gh` bodies — escaped `\n` renders literally.

## Known Limitations

STL is the only wired export format.
