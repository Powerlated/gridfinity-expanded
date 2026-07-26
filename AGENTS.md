# Repository Development Guide

Sole operative LLM guide. Other agent files import it, never duplicate it. When the user asks to "remember" something, record it here — not in any other memory store.

## Scope

React 19 + Vite 6 + TypeScript app generating printable Gridfinity STLs. "GUI" in user requests means this web app, not the egui debugger in `rust/crates/gridfinity-gui`. Pick local implementation details freely; ask before changing architecture, user-visible semantics, compatibility policy, or scope. Keep changes narrow, preserve unrelated working-tree changes, inspect call sites first — trace store, worker boundary, preview, and export paths. Preview data may be grouped/colored/positioned; export data must preserve coordinates, topology, orientation, and per-piece meaning.

## Structure

`src/main.tsx` mounts; `src/App.tsx` = Mantine AppShell; `src/store.ts` = Zustand state + explicit design commands; `src/theme.ts` = Mantine defaults.

`src/lib/`: `types.ts` contracts, `gridfinitySpec.ts` normative dimensions, `coordinates.ts` editor→generation transforms, `binParameters.ts` per-bin worker input, `geometryCache.ts` per-bin triangle cache, `preview.ts` viewer-branch layout, `edges.ts`, `cuts.ts` cut planning, `printers.ts` fit, `geometry/`, `export/printableObjects.ts` splitting+naming, `export/stl.ts`.

Geometry runs in `src/workers/geometry.worker.ts` via `src/hooks/useBinGeometry.ts`. `src/components/`: left panel = Shape/Walls/Cuts editors; right panel = collapsed Dimensions/Features/Printer-fit accordions; `viewer/ModelViewer.tsx` hosts the Rust WebGL2 renderer drawing triangle soups directly. Scripts in `scripts/`; Vitest beside source as `*.test.ts`; browser tests in `e2e/`. This guide is the canonical spec and architecture record.

## Implementation

TypeScript ESM, function components, two-space indent. `PascalCase.tsx`, `useName.ts`, camelCase helper exports. Shared contracts in `types.ts`, domain logic in `src/lib/`.

Prefer Mantine controls/layout over custom UI. Cross-app control styling → `theme.ts`; global layout and documented library workarounds → `src/index.css`; SVG editor styles → `src/components/sidebar/editor.css`. No fixed design constants in JSX unless data-driven.

Tabs read/write only through `useAppStore()` commands. Keep `Design`, `BinParameters`, `Bin` plain and structured-clone compatible. A shape change resets that bin's openings, walls, and cuts, then reseeds required cuts. The UI alone enforces validity — controls constrain their own ranges and dependent values; no store clamping, no validation layer. The UI derives complete piece groups before invoking geometry. Preview offsets come from `previewLayout()` after generation, never worker input/output.

`generateGeometry()` is the sole production geometry path: takes trusted generation-ready `BinParameters[]`, builds each logical bin once, intersects with piece footprints, returns pieces grouped per `Bin`. Export splits them into named `PrintableObject`s via `toPrintableObjects()`. Author geometry with exact native `manifold-3d` `CrossSection`/`Manifold` ops — never coarse tolerances or padding (caused terraced fillets). Geometry must not plan cuts, name parts, inspect printers, validate input, normalize coordinates, or localize; manifoldness is verified in the Rust kernel only. `manifoldTriangles()` is the one extraction boundary: quantizes to serialized float32 with 1-micron weld and degenerate-facet repair — no other repair exists.

`Design` stays in editor row-down coordinates. `buildBinParameters()` mirrors every bin together across the design's maximum occupied row; parameters, geometry, and echoed `BinPiece.cells` share that generation frame. Partition pieces before mirroring so piece indexes keep editor meaning. The camera is natively Z-up: never transform meshes for orientation; default and reset orbit must face the layout as the editor does. Preview and STL export consume the identical global-coordinate triangle soup, with multipart spacing applied only via viewer transforms after cuts are mirrored into generation coordinates. Expand quantized output so each triangle owns its vertices and flat normal. Combine solids with manifold booleans; use `CrossSection.offset` for inward 2D offsets.

`useBinGeometry` schedules on the leading edge, not a debounce: a design change with no run in flight starts generation immediately, and a change arriving mid-run only sets a dirty flag that restarts once the in-flight run settles. That keeps the preview live while the user paints — the kernel is fast enough that a held gesture regenerates continuously — while coalescing bursts to at most one queued rebuild, so workers never accumulate a backlog whose tail is stale. Every exit path from a run (success, cache-only hit, empty design, worker error, superseded revision) must settle, or the dirty flag strands and the preview freezes.

The alpha generator assumes every bin is edge-connected and valid. Add no geometry-side component normalization, repair, rejection, fallback, or tests defining disconnected-bin behavior. Enclosed holes stay supported. Full spec/editing/cut/coordinate/invalid-input rules live in this guide; rule changes update it and the happy-path tests together.

Changing the geometry pipeline (`src/lib/geometry/`, `src/workers/geometry.worker.ts`, `src/hooks/useBinGeometry.ts`, `src/lib/{binParameters,coordinates,geometryCache,preview,cuts,gridfinitySpec,edges}.ts`, `src/lib/export/printableObjects.ts`) or the viewer (`src/components/viewer/ModelViewer.tsx`, `rust/crates/gridfinity-render/`, `rust/crates/gridfinity-wasm/src/viewer.rs`) requires updating the matching section of this guide in the same change.

`rust/crates/gridfinity-render` is a shared `glow`+`glam` crate with no egui or wasm deps, consumed by the egui debugger (`gridfinity-gui`) and the web app (`gridfinity-wasm`). Keep front-end concerns out; change its camera, shaders, or vertex format only with both consumers in mind.

The `#badapple` route is an easter egg, not part of the design pipeline. It plays through `src/hooks/useBadApple.ts` over a worker pool, and everything about it is shaped to keep the main thread free: workers call `badapple_frame_vertices()`, which bakes colour and flat-shaded normals in so the returned buffer goes straight to `Viewer::upload_vertices` as a plain GPU upload — the main thread never expands a triangle soup. Frames reach the viewer through the feed's `pending` ref, drained inside the existing render loop, and `data-badapple-frame` is set imperatively. Never route clip frames through React state or `add_piece`; either one re-introduces per-frame main-thread work and the camera visibly stutters.

## Validation

- `npm run lint` — Oxlint
- `npm run test` — Vitest
- `npm run build` — type-check + Vite build
- `cd rust && cargo test --workspace` — geometry kernel suite, the printability gate
- `npm run test:e2e` — Chromium Playwright smoke
- `npm run classify:changes -- <base> <head>` — CI gate classification

Lint + build on every non-trivial code change. Don't add Vitest coverage by default during rapid feature development; run existing Vitest when changing printer, cut-to-part, or export behavior it covers (CI always runs all of it).

Run the Rust suite for every print-affecting change: geometry, cut/part generation, STL serialization, walls, fasteners, worker generation, geometry-consumed config. Watertightness is a B-rep property asserted by `Solid::validate` and tessellation-leak checks; no TypeScript manifold verifier exists or may be reintroduced.

Use Playwright for every browser-visible change. Locally, equivalent manual browser verification is acceptable if the report names the method. In CI there is no fallback: if classification requires Playwright, a browser-test failure fails the check.

Never run visual or browser verification yourself — no browser automation, no screenshot capture, no ad-hoc scripts that stand in for looking at the app. Always ask the user to perform the visual check and report back what they saw. Write and hand over the test or the exact steps; the user drives the browser.

Complete means required commands finished with confirmed successful exit codes. Timeouts, truncated output, and partial runs are not successes. Final reports list checks run, results, and any required or relevant checks omitted.

## CI Classification

CI always runs lint, Vitest, build. The classifier adds gates fail-safe:

- Playwright: runtime UI, entrypoints, styles, store, hooks, workers, shared types, dependencies, build config
- Rust: geometry, cut/part generation, STL export, geometry workers, geometry-consumed config, anything under `rust/`
- Both: ambiguous shared runtime files
- Neither: docs-only, isolated test-only

Classify unrecognized paths conservatively. Leave `.github/workflows/deploy.yml` behavior alone unless deployment is explicitly in scope.

## Pull Requests

Work in a dedicated feature branch in a new worktree off latest `origin/main`, not on `main`; target `main`. Short imperative commit subjects. PRs describe user-visible changes, list validation commands, link issues, include screenshots/recordings for UI changes, call out printability and manifold implications for geometry/export changes, and carry any `AGENTS.md` updates for pipeline or viewer touches. Use `--body-file` for multiline `gh` bodies — escaped `\n` renders literally.

## Known Limitations

STL is the only wired export format.
