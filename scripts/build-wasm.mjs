/**
 * Compiles the Rust geometry kernel to WebAssembly into `src/wasm/`.
 *
 * The cargo workspace is rooted at this repository's root, with the crates
 * under `crates/`; set `GRIDFINITY_KERNEL` to point somewhere else to build
 * against a different checkout. The output is a build artifact and is
 * gitignored, so a fresh clone has none until this runs. `npm run build` needs
 * it to already exist; CI and the Pages deploy therefore run `npm run
 * build:wasm` as a step of their own.
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
const kernelRoot = resolve(repoRoot, process.env.GRIDFINITY_KERNEL ?? '.');
const cratePath = resolve(kernelRoot, 'crates/gridfinity-wasm');
const outDir = resolve(repoRoot, 'src/wasm');

if (!existsSync(cratePath)) {
  console.error(
    `Rust geometry kernel not found at ${cratePath}.\n` +
    'The kernel workspace should be the repository root; set GRIDFINITY_KERNEL to use another checkout.',
  );
  process.exit(1);
}

const SOURCE_EXTENSIONS = ['.rs', '.toml', '.lock', '.wgsl'];
const KERNEL_SOURCES = ['crates', 'Cargo.toml', 'Cargo.lock'];
const ARTIFACTS = ['gridfinity_wasm_bg.wasm', 'gridfinity_wasm.js', 'gridfinity_wasm.d.ts'];

/**
 * Given an absolute path inside the kernel workspace, returns the newest mtime
 * in milliseconds among the kernel sources it covers: the file's own mtime when
 * it is a source file, the newest of its descendants when it is a directory,
 * and 0 when it is neither or does not exist. `target` and dot-entries are not
 * kernel sources and are not descended into.
 */
function newestSourceChange(path) {
  if (!existsSync(path)) return 0;
  if (!statSync(path).isDirectory()) {
    return SOURCE_EXTENSIONS.some((ext) => path.endsWith(ext)) ? statSync(path).mtimeMs : 0;
  }
  let newest = 0;
  for (const entry of readdirSync(path, { withFileTypes: true })) {
    if (entry.name === 'target' || entry.name.startsWith('.')) continue;
    newest = Math.max(newest, newestSourceChange(resolve(path, entry.name)));
  }
  return newest;
}

function staleness() {
  const missing = ARTIFACTS.filter((name) => !existsSync(resolve(outDir, name)));
  if (missing.length > 0) return `src/wasm/ is missing ${missing.join(', ')}`;
  const built = Math.min(...ARTIFACTS.map((name) => statSync(resolve(outDir, name)).mtimeMs));
  const changed = Math.max(
    ...KERNEL_SOURCES.map((name) => newestSourceChange(resolve(kernelRoot, name))),
  );
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
