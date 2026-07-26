import { IMPLEMENTATION_ALLOWANCES } from './gridfinitySpec';
import { maximumOccupiedRow, mirrorCut } from './coordinates';
import type { Bin, Cell, Cut, Design, Point2 } from './types';

/** One flattened piece positioned for the multipart preview gap. */
export interface PreviewPiece {
  binId: string;
  /** 0-based piece index within its bin, for stable mesh naming. */
  pieceIndex: number;
  vertices: Float32Array;
  previewOffset: Point2;
}

export function previewOffsetFor(cells: Cell[], cuts: Cut[], pieceCount: number): Point2 {
  if (pieceCount <= 1) return { x: 0, y: 0 };
  const halfGap = IMPLEMENTATION_ALLOWANCES.multipartPreviewGap / 2;
  const verticalLines = new Set<number>();
  const horizontalLines = new Set<number>();
  for (const cut of cuts) {
    if (cut.start.x === cut.end.x) verticalLines.add(cut.start.x);
    else horizontalLines.add(cut.start.y);
  }
  let x = 0;
  let y = 0;
  for (const line of verticalLines) {
    if (cells.every((cell) => cell.x < line)) x -= halfGap;
    else if (cells.every((cell) => cell.x >= line)) x += halfGap;
  }
  for (const line of horizontalLines) {
    if (cells.every((cell) => cell.y < line)) y -= halfGap;
    else if (cells.every((cell) => cell.y >= line)) y += halfGap;
  }
  return { x, y };
}

/** Modifications for better viewing: flatten bins and attach multipart gap offsets. */
export function previewLayout(bins: Bin[], design: Design | null): PreviewPiece[] {
  const maximumRow = design ? maximumOccupiedRow(design) : null;
  return bins.flatMap((bin) => {
    const designCuts = design?.bins.find((candidate) => candidate.id === bin.binId)?.cuts ?? [];
    const cuts = designCuts.map((cut) => mirrorCut(cut, maximumRow!));
    return bin.pieces.map((piece, pieceIndex) => ({
      binId: bin.binId,
      pieceIndex,
      vertices: piece.vertices,
      previewOffset: previewOffsetFor(piece.cells, cuts, bin.pieces.length),
    }));
  });
}
