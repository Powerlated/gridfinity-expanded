import type { Rect } from '../types';

/**
 * Rectilinear plan geometry for the Project editors.
 *
 * These are the measurements the SVG canvases and panels take while a user is
 * drawing an object, all of them synchronous because they run inside React
 * render. The optimizer that consumes the same shapes lives in the Rust kernel
 * (`crates/gridfinity-cad/src/project/`) and is reached through the wasm
 * `PackSearch`; nothing here packs, rotates, or derives dividers.
 *
 * Coordinates are millimetres in the drawer plane, quantized to `QUANTUM` so a
 * box's far edge and the near edge of the box abutting it are the same number
 * rather than a float ulp apart — without that, `rectGrid` reads a box as
 * covering none of its own lattice cell and `unionArea` measures zero.
 */

const QUANTUM = 1e4;

export interface RectGrid {
  xs: number[];
  ys: number[];
  filled: boolean[];
}

/** A millimetre value snapped to the module's coordinate quantum. */
export function quantize(value: number): number {
  return Math.round(value * QUANTUM) / QUANTUM;
}

/** The box's maximum x in mm, quantized. */
export function rectRight(rect: Rect): number {
  return quantize(rect.x + rect.width);
}

/** The box's maximum y in mm, quantized. */
export function rectBottom(rect: Rect): number {
  return quantize(rect.y + rect.depth);
}

export const EMPTY_RECT: Rect = { x: 0, y: 0, width: 0, depth: 0 };

/** The smallest box containing every part, or a zero box for no parts. */
export function partsBounds(parts: Rect[]): Rect {
  if (parts.length === 0) return { ...EMPTY_RECT };
  const x = Math.min(...parts.map((part) => part.x));
  const y = Math.min(...parts.map((part) => part.y));
  return {
    x,
    y,
    width: Math.max(...parts.map(rectRight)) - x,
    depth: Math.max(...parts.map(rectBottom)) - y,
  };
}

/**
 * Every part grown by `margin` on all four sides, so the shape's bounding box
 * grows by `2 * margin` in each extent. A negative margin shrinks it, which is
 * how a placed claim is drawn back down to the object inside it.
 */
export function inflateParts(parts: Rect[], margin: number): Rect[] {
  return parts.map((part) => ({
    x: quantize(part.x - margin),
    y: quantize(part.y - margin),
    width: quantize(part.width + margin * 2),
    depth: quantize(part.depth + margin * 2),
  }));
}

function uniqueSorted(values: number[]): number[] {
  return [...new Set(values.map(quantize))].sort((a, b) => a - b);
}

/**
 * The lattice a part list induces: every distinct part edge on each axis, and
 * for each cell of the resulting grid whether one part covers it entirely.
 */
export function rectGrid(parts: Rect[]): RectGrid {
  const xs = uniqueSorted(parts.flatMap((part) => [quantize(part.x), rectRight(part)]));
  const ys = uniqueSorted(parts.flatMap((part) => [quantize(part.y), rectBottom(part)]));
  const cols = Math.max(0, xs.length - 1);
  const rows = Math.max(0, ys.length - 1);
  const filled = new Array<boolean>(cols * rows).fill(false);
  for (let row = 0; row < rows; row++) {
    for (let col = 0; col < cols; col++) {
      filled[row * cols + col] = parts.some((part) =>
        quantize(part.x) <= xs[col] && xs[col + 1] <= rectRight(part)
        && quantize(part.y) <= ys[row] && ys[row + 1] <= rectBottom(part));
    }
  }
  return { xs, ys, filled };
}

function gridColumns(grid: RectGrid): number {
  return Math.max(0, grid.xs.length - 1);
}

function gridRows(grid: RectGrid): number {
  return Math.max(0, grid.ys.length - 1);
}

function gridFilled(grid: RectGrid, col: number, row: number): boolean {
  const cols = gridColumns(grid);
  if (col < 0 || row < 0 || col >= cols || row >= gridRows(grid)) return false;
  return grid.filled[row * cols + col];
}

/** The area in mm² the parts cover between them, counting overlaps once. */
export function unionArea(parts: Rect[]): number {
  const grid = rectGrid(parts);
  let area = 0;
  for (let row = 0; row < gridRows(grid); row++) {
    for (let col = 0; col < gridColumns(grid); col++) {
      if (!gridFilled(grid, col, row)) continue;
      area += (grid.xs[col + 1] - grid.xs[col]) * (grid.ys[row + 1] - grid.ys[row]);
    }
  }
  return area;
}

/**
 * Whether the parts form one edge-connected region. Boxes meeting only at a
 * corner are not connected, and neither is a part list with no area at all.
 */
export function partsConnected(parts: Rect[]): boolean {
  if (parts.length <= 1) return true;
  const grid = rectGrid(parts);
  const cols = gridColumns(grid);
  const rows = gridRows(grid);
  const total = grid.filled.filter(Boolean).length;
  if (total === 0) return false;
  const start = grid.filled.indexOf(true);
  const seen = new Set<number>([start]);
  const queue = [start];
  while (queue.length > 0) {
    const index = queue.pop()!;
    const col = index % cols;
    const row = Math.floor(index / cols);
    for (const [dx, dy] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
      const nextCol = col + dx;
      const nextRow = row + dy;
      if (nextCol < 0 || nextRow < 0 || nextCol >= cols || nextRow >= rows) continue;
      const next = nextRow * cols + nextCol;
      if (seen.has(next) || !grid.filled[next]) continue;
      seen.add(next);
      queue.push(next);
    }
  }
  return seen.size === total;
}
