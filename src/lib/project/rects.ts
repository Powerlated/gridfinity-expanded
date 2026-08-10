import type { Rect, Rotation } from '../types';

export type Orientation = 'h' | 'v';

export interface Segment {
  orientation: Orientation;
  coordinate: number;
  start: number;
  end: number;
}

export interface RectGrid {
  xs: number[];
  ys: number[];
  filled: boolean[];
}

export const ROTATIONS: Rotation[] = [0, 90, 180, 270];

const QUANTUM = 1e4;

export function quantize(value: number): number {
  return Math.round(value * QUANTUM) / QUANTUM;
}

export function rectRight(rect: Rect): number {
  return rect.x + rect.width;
}

export function rectBottom(rect: Rect): number {
  return rect.y + rect.depth;
}

export function rectsOverlap(a: Rect, b: Rect): boolean {
  return a.x < rectRight(b) && b.x < rectRight(a)
    && a.y < rectBottom(b) && b.y < rectBottom(a);
}

export function rectContains(outer: Rect, inner: Rect): boolean {
  return inner.x >= outer.x && inner.y >= outer.y
    && rectRight(inner) <= rectRight(outer) && rectBottom(inner) <= rectBottom(outer);
}

export const EMPTY_RECT: Rect = { x: 0, y: 0, width: 0, depth: 0 };

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

export function translateParts(parts: Rect[], dx: number, dy: number): Rect[] {
  return parts.map((part) => ({ ...part, x: quantize(part.x + dx), y: quantize(part.y + dy) }));
}

export function normalizeParts(parts: Rect[]): Rect[] {
  const bounds = partsBounds(parts);
  return translateParts(parts, -bounds.x, -bounds.y);
}

export function inflateParts(parts: Rect[], margin: number): Rect[] {
  return parts.map((part) => ({
    x: quantize(part.x - margin),
    y: quantize(part.y - margin),
    width: quantize(part.width + margin * 2),
    depth: quantize(part.depth + margin * 2),
  }));
}

export function rotateParts(parts: Rect[], rotation: Rotation): Rect[] {
  return normalizeParts(parts.map((part) => {
    if (rotation === 90) {
      return { x: -rectBottom(part), y: part.x, width: part.depth, depth: part.width };
    }
    if (rotation === 180) {
      return { x: -rectRight(part), y: -rectBottom(part), width: part.width, depth: part.depth };
    }
    if (rotation === 270) {
      return { x: part.y, y: -rectRight(part), width: part.depth, depth: part.width };
    }
    return { ...part };
  }));
}

export function partsKey(parts: Rect[]): string {
  return [...parts]
    .map((part) => `${quantize(part.x)},${quantize(part.y)},${quantize(part.width)},${quantize(part.depth)}`)
    .sort()
    .join('|');
}

function uniqueSorted(values: number[]): number[] {
  return [...new Set(values.map(quantize))].sort((a, b) => a - b);
}

export function rectGrid(parts: Rect[]): RectGrid {
  const xs = uniqueSorted(parts.flatMap((part) => [part.x, rectRight(part)]));
  const ys = uniqueSorted(parts.flatMap((part) => [part.y, rectBottom(part)]));
  const cols = Math.max(0, xs.length - 1);
  const rows = Math.max(0, ys.length - 1);
  const filled = new Array<boolean>(cols * rows).fill(false);
  for (let row = 0; row < rows; row++) {
    for (let col = 0; col < cols; col++) {
      filled[row * cols + col] = parts.some((part) =>
        part.x <= xs[col] && xs[col + 1] <= rectRight(part)
        && part.y <= ys[row] && ys[row + 1] <= rectBottom(part));
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

export function boundarySegments(parts: Rect[]): Segment[] {
  const grid = rectGrid(parts);
  const cols = gridColumns(grid);
  const rows = gridRows(grid);
  const segments: Segment[] = [];
  for (let col = 0; col <= cols; col++) {
    for (let row = 0; row < rows; row++) {
      if (gridFilled(grid, col - 1, row) === gridFilled(grid, col, row)) continue;
      segments.push({
        orientation: 'v',
        coordinate: grid.xs[col],
        start: grid.ys[row],
        end: grid.ys[row + 1],
      });
    }
  }
  for (let row = 0; row <= rows; row++) {
    for (let col = 0; col < cols; col++) {
      if (gridFilled(grid, col, row - 1) === gridFilled(grid, col, row)) continue;
      segments.push({
        orientation: 'h',
        coordinate: grid.ys[row],
        start: grid.xs[col],
        end: grid.xs[col + 1],
      });
    }
  }
  return segments;
}

export function segmentKey(segment: Segment): string {
  return `${segment.orientation}:${quantize(segment.coordinate)}`;
}

export function sortSegments(segments: Segment[]): Segment[] {
  return [...segments].sort((a, b) =>
    a.orientation.localeCompare(b.orientation)
    || a.coordinate - b.coordinate
    || a.start - b.start);
}

export function mergeSegments(segments: Segment[]): Segment[] {
  const groups = new Map<string, Segment[]>();
  for (const segment of segments) {
    const key = segmentKey(segment);
    const group = groups.get(key);
    if (group) group.push(segment);
    else groups.set(key, [segment]);
  }
  const merged: Segment[] = [];
  for (const group of groups.values()) {
    const sorted = [...group].sort((a, b) => a.start - b.start || a.end - b.end);
    let run = { ...sorted[0] };
    for (const segment of sorted.slice(1)) {
      if (segment.start <= run.end) run.end = Math.max(run.end, segment.end);
      else {
        merged.push(run);
        run = { ...segment };
      }
    }
    merged.push(run);
  }
  return sortSegments(merged);
}
