/**
 * Geometry kernel boundary.
 *
 * Solid construction lives in the `crates/` Rust workspace and runs here as
 * WebAssembly. It is an analytic B-rep kernel: exact surfaces,
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
 * from the Rust workspace into `web/src/wasm/`, which is gitignored.
 */
import init, {
  PackSearch as WasmPackSearch,
  Viewer,
  badapple_bounds,
  badapple_fps,
  badapple_frame_count,
  badapple_frame_vertices,
  create_viewer,
  export_parasolid,
  generate_geometry,
} from '../../wasm/gridfinity_wasm.js';
import type { BadAppleClip, Bin, BinParameters, PackInput, PackResult } from '../types';

export type { Viewer };

/**
 * A drawer packing search in progress, owned by the kernel.
 *
 * The budget is a restart count, not a clock, so the caller spends it in chunks
 * and may read the incumbent layout between them. `free` releases the wasm
 * handle; the search is unusable afterwards.
 */
export interface PackSearch {
  readonly total: number;
  readonly done: number;
  step: (iterations: number) => boolean;
  result: () => PackResult;
  free: () => void;
}

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

/**
 * Builds every piece's solid again and returns all of them as one Parasolid
 * XT transmit file: an analytic B-rep, one body per printable piece, in
 * `toPrintableObjects` order. Nothing is tessellated on this path — the file
 * carries the exact surfaces and curves the kernel built.
 */
export function exportParasolid(_kernel: GeometryKernel, bins: BinParameters[]): string {
  return export_parasolid(bins);
}

/**
 * Starts a drawer packing search over `input`, with its first greedy pass
 * already run so `result()` is meaningful before any `step`.
 *
 * The optimizer itself lives in `crates/gridfinity-cad/src/project/`; this is
 * only the handle. The kernel handle is threaded through so callers cannot
 * start a search before `initKernel()` has resolved.
 */
export function createPackSearch(_kernel: GeometryKernel, input: PackInput): PackSearch {
  const search = new WasmPackSearch(input);
  return {
    get total() {
      return search.total;
    },
    get done() {
      return search.done;
    },
    step: (iterations: number) => search.step(iterations),
    result: () => search.result() as PackResult,
    free: () => search.free(),
  };
}

export function badAppleClip(_kernel: GeometryKernel): BadAppleClip {
  const b = badapple_bounds();
  return {
    frameCount: badapple_frame_count(),
    fps: badapple_fps(),
    bounds: {
      min: [b[0], b[1], b[2]],
      max: [b[3], b[4], b[5]],
    },
  };
}

/**
 * Builds one clip frame as a render-ready flat-shaded vertex buffer.
 *
 * The colour is baked in here, inside the worker, so the main thread never
 * expands a triangle soup: at 30 frames a second that expansion starved the
 * render loop and pointer handling, which showed up as camera lag.
 */
export function badAppleFrameVertices(
  _kernel: GeometryKernel,
  frame: number,
  rgb: number,
): Float32Array {
  return badapple_frame_vertices(frame, rgb);
}

/**
 * Creates the WebGL2 viewer that the Rust workspace also uses for its egui
 * debugger, bound to a canvas this side owns.
 *
 * The renderer is display-only: it consumes the same triangle soup the export
 * branch does, applies viewer-branch preview offsets, and never feeds anything
 * back into geometry.
 */
export function createViewer(
  _kernel: GeometryKernel,
  canvas: HTMLCanvasElement,
  clearRgb: number,
): Promise<Viewer> {
  return create_viewer(canvas, clearRgb);
}
