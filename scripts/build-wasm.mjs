/**
 * Compiles the Rust geometry kernel to WebAssembly into `src/wasm/`.
 *
 * The kernel workspace lives in `rust/` in this repository; set
 * `GRIDFINITY_KERNEL` to point somewhere else to build against a different
 * checkout. The output is a build artifact and is gitignored —
 * `npm run build`, `npm run dev`, and `npm run check:manifold` all depend on it
 * existing, so run this first after a fresh clone.
 *
 * Requires the Rust toolchain plus `wasm32-unknown-unknown` and `wasm-pack`:
 *   rustup target add wasm32-unknown-unknown
 *   cargo install wasm-pack
 */
import { spawnSync } from 'node:child_process';
import { existsSync } from 'node:fs';
import { resolve } from 'node:path';

const repoRoot = resolve(import.meta.dirname, '..');
const kernelRoot = resolve(repoRoot, process.env.GRIDFINITY_KERNEL ?? 'rust');
const cratePath = resolve(kernelRoot, 'crates/gridfinity-wasm');
const outDir = resolve(repoRoot, 'src/wasm');

if (!existsSync(cratePath)) {
  console.error(
    `Rust geometry kernel not found at ${cratePath}.\n` +
    'The kernel workspace should be at rust/; set GRIDFINITY_KERNEL to use another checkout.',
  );
  process.exit(1);
}

const result = spawnSync(
  'wasm-pack',
  ['build', cratePath, '--target', 'web', '--release', '--out-dir', outDir],
  { stdio: 'inherit', shell: true },
);

if (result.status !== 0) {
  console.error('\nwasm-pack build failed. Is wasm-pack installed and on PATH?');
  process.exit(result.status ?? 1);
}
