/**
 * Geometry kernel boundary.
 *
 * Solid construction lives in the `rust/` Rust workspace and runs here as
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
 * from the Rust workspace into `src/wasm/`, which is gitignored.
 */
import init, {
  Viewer,
  badapple_bounds,
  badapple_fps,
  badapple_frame_count,
  badapple_frame_vertices,
  generate_geometry,
} from '../../wasm/gridfinity_wasm.js';
import type { BadAppleClip, Bin, BinParameters } from '../types';

export type { Viewer };

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
): Viewer {
  return new Viewer(canvas, clearRgb);
}
