import type { Bin, PrintableObject } from '../types';

export function partFilename(
  binId: string,
  binCount: number,
  pieceIndex: number,
  pieceCount: number,
): string {
  const stem = binCount === 1 ? 'gridfinity-bin' : `gridfinity-${binId}`;
  return pieceCount === 1
    ? `${stem}.stl`
    : `${stem}-part-${pieceIndex + 1}-of-${pieceCount}.stl`;
}

/** Split bins into distinct printable objects, one fully named part per piece. */
export function toPrintableObjects(bins: Bin[]): PrintableObject[] {
  return bins.flatMap((bin) =>
    bin.pieces.map((piece, pieceIndex) => ({
      name: partFilename(bin.binId, bins.length, pieceIndex, bin.pieces.length),
      vertices: piece.vertices,
    })));
}
