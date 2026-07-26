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

export interface BinPiece {
  /** Global-coordinate flat triangle soup (9 floats per triangle). */
  triangles: Float32Array;
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
  triangles: Float32Array;
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

export interface BadAppleRequest {
  frame: number;
}

export type BadAppleResponse =
  | { ok: true; frame: number; triangles: Float32Array }
  | { ok: false; frame: number };

export interface BadAppleClip {
  frameCount: number;
  fps: number;
  bounds: { min: [number, number, number]; max: [number, number, number] };
}
