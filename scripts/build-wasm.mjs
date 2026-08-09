/**
 * Compiles the Rust geometry kernel to WebAssembly into `src/wasm/`.
 *
 * The kernel workspace lives in `rust/` in this repository; set
 * `GRIDFINITY_KERNEL` to point somewhere else to build against a different
 * checkout. The output is a build artifact and is gitignored, so a fresh clone
 * has none until this runs. `npm run build` needs it to already exist; CI and
 * the Pages deploy therefore run `npm run build:wasm` as a step of their own.
 *
 * With `--if-needed` it rebuilds only when `src/wasm/` is missing an artifact or
 * is older than the kernel's newest source file. `npm run dev` runs it that way,
 * so the dev server cannot serve geometry older than the kernel it was built
 * from. It is a startup check, not a watcher: editing Rust while the server runs
 * takes a restart.
 *
 * Requires the Rust toolchain plus `wasm32-unknown-unknown` and `wasm-pack`:
 *   rustup target add wasm32-unknown-unknown
 *   cargo install wasm-pack
 */
import { spawnSync } from 'node:child_process';
import { existsSync, readdirSync, statSync } from 'node:fs';
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

const SOURCE_EXTENSIONS = ['.rs', '.toml', '.lock', '.wgsl'];
const ARTIFACTS = ['gridfinity_wasm_bg.wasm', 'gridfinity_wasm.js', 'gridfinity_wasm.d.ts'];

function newestSourceChange(dir) {
  let newest = 0;
  for (const entry of readdirSync(dir, { withFileTypes: true })) {
    if (entry.name === 'target' || entry.name.startsWith('.')) continue;
    const path = resolve(dir, entry.name);
    if (entry.isDirectory()) {
      newest = Math.max(newest, newestSourceChange(path));
    } else if (SOURCE_EXTENSIONS.some((ext) => entry.name.endsWith(ext))) {
      newest = Math.max(newest, statSync(path).mtimeMs);
    }
  }
  return newest;
}

function staleness() {
  const missing = ARTIFACTS.filter((name) => !existsSync(resolve(outDir, name)));
  if (missing.length > 0) return `src/wasm/ is missing ${missing.join(', ')}`;
  const built = Math.min(...ARTIFACTS.map((name) => statSync(resolve(outDir, name)).mtimeMs));
  const changed = newestSourceChange(kernelRoot);
  return changed > built ? 'the Rust kernel changed since src/wasm/ was built' : null;
}

if (process.argv.includes('--if-needed')) {
  const reason = staleness();
  if (reason === null) {
    console.log('src/wasm/ is up to date with the Rust kernel.');
    process.exit(0);
  }
  console.log(`Rebuilding src/wasm/: ${reason}.`);
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
