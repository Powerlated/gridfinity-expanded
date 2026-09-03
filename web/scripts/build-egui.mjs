/** Build the egui application and the C++ OCCT bridge into one Emscripten WASM. */
import { spawnSync } from 'node:child_process';
import { copyFileSync, existsSync, mkdirSync, readdirSync, rmSync, statSync } from 'node:fs';
import { resolve } from 'node:path';

const webRoot = resolve(import.meta.dirname, '..');
const repoRoot = resolve(webRoot, '..');
const outDir = resolve(webRoot, 'src/wasm-egui');
const buildDir = resolve(repoRoot, 'target/wasm32-unknown-emscripten/release');
const outputWasm = resolve(outDir, 'gridfinity_app.wasm');
const sourceRoots = ['crates', 'cmake', 'vendor/occt', 'Cargo.toml', 'Cargo.lock', 'CMakePresets.json'];
const localEmsdk = resolve(repoRoot, 'target/emsdk');
process.env.EMSDK ??= localEmsdk;
process.env.PATH = [
  resolve(repoRoot, 'target/tools/bin'),
  resolve(process.env.EMSDK, 'upstream/emscripten'),
  process.env.EMSDK,
  process.env.PATH,
].join(process.platform === 'win32' ? ';' : ':');

function run(command, args, options = {}) {
  const result = spawnSync(command, args, { cwd: repoRoot, stdio: 'inherit', shell: process.platform === 'win32', ...options });
  if (result.status !== 0) throw new Error(`${command} failed with exit code ${result.status ?? 'unknown'}`);
}
function newest(path) {
  if (!existsSync(path)) return 0;
  const s = statSync(path); if (!s.isDirectory()) return s.mtimeMs;
  return Math.max(0, ...readdirSync(path, {withFileTypes:true}).filter(e => e.name !== 'target' && !e.name.startsWith('.')).map(e => newest(resolve(path,e.name))));
}
if (process.argv.includes('--if-needed') && existsSync(outputWasm) && statSync(outputWasm).mtimeMs >= Math.max(...sourceRoots.map(p => newest(resolve(repoRoot,p))))) {
  console.log('The unified egui + OCCT WASM is up to date.');
  process.exit(0);
}
for (const tool of [['emcc',['--version']], ['wasm-bindgen',['--version']]]) {
  const check=spawnSync(tool[0],tool[1],{stdio:'ignore',shell:process.platform==='win32'});
  if(check.status!==0) throw new Error(`${tool[0]} is required. Activate emsdk 6.0.5 and install wasm-bindgen-cli 0.2.126.`);
}
const occtRoot = process.env.OCCT_ROOT ?? resolve(repoRoot, 'target/occt-install/emscripten');
if (!existsSync(resolve(occtRoot, 'include/opencascade/TopoDS_Shape.hxx'))) {
  run('cmake', ['--preset', 'occt-web']);
  run('cmake', ['--build', '--preset', 'occt-web-install']);
}
// emcc links the module and, under -sWASM_BINDGEN (.cargo/config.toml), runs
// wasm-bindgen itself and merges the bindings into its own JS. There is no
// second pass to run here: it emits the loader and the module as one pair.
run('cargo', ['build', '--release', '--target', 'wasm32-unknown-emscripten', '-p', 'gridfinity-app', '--features', 'occt'], {env:{...process.env,OCCT_ROOT:occtRoot}});
mkdirSync(outDir,{recursive:true});
for (const stale of readdirSync(outDir)) rmSync(resolve(outDir, stale), {recursive:true});
for (const artifact of ['gridfinity-app.js', 'gridfinity_app.wasm']) {
  const built = resolve(buildDir, artifact);
  if (!existsSync(built)) throw new Error(`emcc did not emit ${artifact}`);
  copyFileSync(built, resolve(outDir, artifact));
}
const wasmFiles=readdirSync(outDir).filter(f=>f.endsWith('.wasm'));
if(wasmFiles.length!==1) throw new Error(`expected one final WASM, found ${wasmFiles.join(', ') || 'none'}`);
console.log(`Built one unified WASM: ${outputWasm}`);
