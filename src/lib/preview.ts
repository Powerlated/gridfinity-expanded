import { GRIDFINITY_SPEC } from './gridfinitySpec';
import type { Bin, Cell, Point2 } from './types';

export interface PreviewPiece {
  binId: string;
  pieceIndex: number;
  vertices: Float32Array;
  apartDirection: Point2;
  cutSegments: Float32Array;
}

const NEIGHBOURS: Array<{ dx: number; dy: number }> = [
  { dx: 1, dy: 0 },
  { dx: -1, dy: 0 },
  { dx: 0, dy: 1 },
  { dx: 0, dy: -1 },
];

function centroid(cells: Cell[]): Point2 {
  if (cells.length === 0) return { x: 0, y: 0 };
  const sum = cells.reduce(
    (acc, cell) => ({ x: acc.x + cell.x, y: acc.y + cell.y }),
    { x: 0, y: 0 },
  );
  return { x: sum.x / cells.length, y: sum.y / cells.length };
}

export function apartDirectionFor(cells: Cell[], binCells: Cell[], pieceCount: number): Point2 {
  if (pieceCount <= 1) return { x: 0, y: 0 };
  const here = centroid(cells);
  const middle = centroid(binCells);
  const away = { x: here.x - middle.x, y: here.y - middle.y };
  const length = Math.hypot(away.x, away.y);
  if (length === 0) return { x: 0, y: 0 };
  return { x: away.x / length, y: away.y / length };
}

export function cutSegmentsFor(cells: Cell[], binCells: Cell[]): Float32Array {
  const pitch = GRIDFINITY_SPEC.gridPitch;
  const mine = new Set(cells.map((cell) => `${cell.x},${cell.y}`));
  const bin = new Set(binCells.map((cell) => `${cell.x},${cell.y}`));
  const out: number[] = [];
  for (const cell of cells) {
    for (const { dx, dy } of NEIGHBOURS) {
      const key = `${cell.x + dx},${cell.y + dy}`;
      if (mine.has(key) || !bin.has(key)) continue;
      if (dx !== 0) {
        out.push(0, (cell.x + (dx > 0 ? 1 : 0)) * pitch, cell.y * pitch, (cell.y + 1) * pitch);
      } else {
        out.push(1, (cell.y + (dy > 0 ? 1 : 0)) * pitch, cell.x * pitch, (cell.x + 1) * pitch);
      }
    }
  }
  return Float32Array.from(out);
}

export function previewLayout(bins: Bin[]): PreviewPiece[] {
  return bins.flatMap((bin) => {
    const binCells = bin.pieces.flatMap((piece) => piece.cells);
    return bin.pieces.map((piece, pieceIndex) => ({
      binId: bin.binId,
      pieceIndex,
      vertices: piece.vertices,
      apartDirection: apartDirectionFor(piece.cells, binCells, bin.pieces.length),
      cutSegments: cutSegmentsFor(piece.cells, binCells),
    }));
  });
}
