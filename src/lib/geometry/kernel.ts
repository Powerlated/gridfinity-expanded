/**
 * Geometry kernel boundary.
 *
 * Solid construction lives in the `gridfinity-parametric` Rust workspace and
 * runs here as WebAssembly. It is an analytic B-rep kernel: exact surfaces,
 * exact curves, closed-form intersections, tessellated once at the very end.
 * That is why nothing on this side welds vertices, repairs degenerate facets,
 * or otherwise post-processes the mesh — watertightness is a property of the
 * B-rep, and `tess.rs` samples each edge exactly once so the two faces sharing
 * it emit identical boundary points.
 *
 * The kernel receives complete, trusted `BinParameters` and returns each bin's
 * cut pieces grouped under it, exactly as the pipeline contract specifies. It
 * is given no cuts, printers, filenames, or presentation transforms, and
 * applies no mirroring of its own — `buildBinParameters()` has already mirrored
 * every spatial value into generation coordinates.
 *
 * The `.wasm` is a build artifact, not source: `npm run build:wasm` compiles it
 * from the Rust workspace into `src/wasm/`, which is gitignored.
 */
import init, { generate_geometry } from '../../wasm/gridfinity_wasm.js';
import type { Bin, BinParameters } from '../types';

/** Opaque handle proving the kernel finished loading. */
export interface GeometryKernel {
  readonly ready: true;
}

let cached: Promise<GeometryKernel> | null = null;

/**
 * Loads and instantiates the kernel, memoized so repeated calls share one
 * instance.
 *
 * `source` is how the caller locates the binary: browsers pass the bundler's
 * asset URL, Node passes the file's bytes. Omitting it uses wasm-bindgen's
 * default resolution relative to the module.
 */
export function initKernel(source?: string | BufferSource): Promise<GeometryKernel> {
  cached ??= init(source === undefined ? undefined : { module_or_path: source })
    .then(() => ({ ready: true }) as const);
  return cached;
}

/**
 * Builds finished solids from trusted parameters and returns cut pieces grouped
 * per bin.
 *
 * The kernel handle is threaded through so callers cannot invoke this before
 * `initKernel()` has resolved.
 */
export function generateGeometry(_kernel: GeometryKernel, bins: BinParameters[]): Bin[] {
  return generate_geometry(bins) as Bin[];
}
