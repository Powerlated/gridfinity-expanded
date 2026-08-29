import { partitionCells } from './cuts';
import { maximumOccupiedRow, mirrorCell, mirrorEdge, mirrorWall } from './coordinates';
import { gridfinityHeight } from './gridfinitySpec';
import type { BinParameters, Design } from './types';

/** Convert a validated design into self-contained per-bin generation parameters. */
export function buildBinParameters(design: Design): BinParameters[] {
  const height = gridfinityHeight(design.heightUnits);
  const maximumRow = maximumOccupiedRow(design);
  return design.bins.map((bin) => {
    // Derive identity-bearing pieces before the coordinate transform so their
    // indexes (and therefore export filenames) retain editor ordering.
    const pieces = partitionCells(bin.cells, bin.cuts);
    return {
      binId: bin.id,
      height,
      perimeterThickness: design.perimeterThickness,
      filletRadius: design.filletRadius,
      fasteners: design.fasteners,
      cells: bin.cells.map((cell) => mirrorCell(cell, maximumRow)),
      openings: bin.openings.map((edge) => mirrorEdge(edge, maximumRow)),
      walls: bin.walls.map((wall) => mirrorWall(wall, maximumRow)),
      pieces: pieces.map((piece) =>
        piece.cells.map((cell) => mirrorCell(cell, maximumRow))),
    };
  });
}
