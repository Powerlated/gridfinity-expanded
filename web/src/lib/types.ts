export interface Cell {
  x: number;
  y: number;
}

export interface Point2 {
  x: number;
  y: number;
}

export interface GridPoint extends Point2 {}

export type EdgeOrientation = 'h' | 'v';

/**
 * Canonical unit grid edge in editor coordinates, where rows increase down.
 * A vertical edge at (x, y) runs from (x, y) to (x, y + 1); a horizontal
 * edge runs from (x, y) to (x + 1, y).
 */
export interface Edge {
  x: number;
  y: number;
  orientation: EdgeOrientation;
}

/** Straight, full-height wall in editor millimetres. */
export interface Wall {
  start: Point2;
  end: Point2;
  width: number;
}

/** Axis-aligned cut whose endpoints lie on exact grid points. */
export interface Cut {
  start: GridPoint;
  end: GridPoint;
}

export interface Rect {
  x: number;
  y: number;
  width: number;
  depth: number;
}

export type Rotation = 0 | 90 | 180 | 270;

export interface ProjectObject {
  id: string;
  name: string;
  parts: Rect[];
  quantity: number;
}

export interface Drawer {
  width: number;
  depth: number;
}

export interface Project {
  id: string;
  name: string;
  drawer: Drawer;
  dividerThickness: number;
  clearance: number;
  objects: ProjectObject[];
}

export interface Placement {
  objectId: string;
  instance: number;
  rotation: Rotation;
  parts: Rect[];
}

/**
 * How a finished layout reads, term by term, each a fraction in 0..=1 with 0 the
 * tidiest: unshared divider lines and runs, leftover broken into pieces, leftover
 * too narrow to hold anything, instances of one object standing apart, and the
 * layout's centre of area off the drawer's. The packer minimises their weighted
 * sum once everything asked for fits, and carries the winning reading back rather
 * than leaving it to be recomputed.
 */
export interface Tidiness {
  lines: number;
  runs: number;
  fragments: number;
  slivers: number;
  grouping: number;
  balance: number;
}

export interface PackResult {
  placements: Placement[];
  placedByObjectId: Record<string, number>;
  iterations: number;
  tidiness: Tidiness;
  /**
   * The dividers these placements imply, derived by the Rust packer alongside
   * them. They arrive with the result rather than being recomputed here because
   * the generator lives in the kernel and the canvases that draw them run
   * inside React render, which cannot await wasm.
   */
  walls: Wall[];
}

export type PackEffort = 'quick' | 'standard' | 'thorough';

export interface PackInput {
  area: Rect;
  objects: ProjectObject[];
  dividerThickness: number;
  clearance: number;
  effort: PackEffort;
}

export interface PackRequest {
  revision: number;
  input: PackInput;
}

export type PackResponse =
  | { ok: true; revision: number; done: boolean; progress: number; best: PackResult }
  | { ok: false; revision: number; error: string };

export interface FastenerSettings {
  magnets: boolean;
  m3: boolean;
}

export interface PrinterSettings {
  name: string;
  bedWidth: number;
  bedDepth: number;
}

export interface BinDesign {
  id: string;
  cells: Cell[];
  openings: Edge[];
  walls: Wall[];
  cuts: Cut[];
}

/** Plain editor-owned state; the UI only allows valid parameters. */
export interface Design {
  bins: BinDesign[];
  heightUnits: number;
  perimeterThickness: number;
  filletRadius: number;
  fasteners: FastenerSettings;
  printer: PrinterSettings;
}

/**
 * Complete, trusted, self-contained parameters for generating one bin.
 * Spatial values use generation coordinates, with editor Y mirrored across
 * the complete design's occupied height.
 */
export interface BinParameters {
  binId: string;
  /** Height in mm, already converted from height units. */
  height: number;
  perimeterThickness: number;
  filletRadius: number;
  fasteners: FastenerSettings;
  cells: Cell[];
  openings: Edge[];
  walls: Wall[];
  /** Piece footprints from cut planning; array order defines piece index. */
  pieces: Cell[][];
}

export const RENDER_VERTEX_STRIDE = 6;
export const RENDER_FLOATS_PER_TRIANGLE = 3 * RENDER_VERTEX_STRIDE;

export interface BinPiece {
  vertices: Float32Array;
  /** Generation-coordinate footprint cells, echoed for viewer-side layout. */
  cells: Cell[];
}

/** One generated logical bin with its cut pieces grouped together. */
export interface Bin {
  binId: string;
  pieces: BinPiece[];
}

/** One distinct printable part, split out of a bin and fully named. */
export interface PrintableObject {
  /** Complete STL filename. */
  name: string;
  vertices: Float32Array;
}

export interface BedFitResult {
  fits: boolean;
  width: number;
  depth: number;
  rotated: boolean;
}

export interface GenerateGeometryRequest {
  revision: number;
  bins: BinParameters[];
}

export type GenerateGeometryResponse =
  | { ok: true; revision: number; bins: Bin[] }
  | { ok: false; revision: number; error: string };

export interface ExportParasolidRequest {
  bins: BinParameters[];
}

export type ExportParasolidResponse =
  | { ok: true; xt: string }
  | { ok: false; error: string };
