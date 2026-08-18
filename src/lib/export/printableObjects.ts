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

/**
 * The one file a whole-design Parasolid export lands in. Unlike STL, XT is a
 * multi-body container, so every printable piece of every bin goes into this
 * single download and the name carries no part breakdown.
 */
export function designFilename(extension = 'x_t'): string {
  return `gridfinity.${extension}`;
}

/** Split bins into distinct printable objects, one fully named part per piece. */
export function toPrintableObjects(bins: Bin[]): PrintableObject[] {
  return bins.flatMap((bin) =>
    bin.pieces.map((piece, pieceIndex) => ({
      name: partFilename(bin.binId, bins.length, pieceIndex, bin.pieces.length),
      vertices: piece.vertices,
    })));
}
