import { GRIDFINITY_SPEC } from './gridfinitySpec';
import type { Cell, Cut, Design, Edge, Point2, Wall } from './types';

/** Largest occupied editor row shared by the complete design. */
export function maximumOccupiedRow(design: Design): number {
  return Math.max(0, ...design.bins.flatMap((bin) => bin.cells.map((cell) => cell.y)));
}

/** Mirror an occupied cell from editor row-down into generation coordinates. */
export function mirrorCell(cell: Cell, maximumRow: number): Cell {
  return { x: cell.x, y: maximumRow - cell.y };
}

/** Mirror a canonical grid edge from editor row-down into generation coordinates. */
export function mirrorEdge(edge: Edge, maximumRow: number): Edge {
  return {
    ...edge,
    y: edge.orientation === 'h' ? maximumRow + 1 - edge.y : maximumRow - edge.y,
  };
}

/** Mirror a grid-line point from editor row-down into generation coordinates. */
export function mirrorGridPoint(point: Point2, maximumRow: number): Point2 {
  return { x: point.x, y: maximumRow + 1 - point.y };
}

/** Mirror a cut into the same generation frame as its piece footprints. */
export function mirrorCut(cut: Cut, maximumRow: number): Cut {
  return {
    start: mirrorGridPoint(cut.start, maximumRow),
    end: mirrorGridPoint(cut.end, maximumRow),
  };
}

/** Mirror a millimetre point across the complete occupied design height. */
export function mirrorMillimetrePoint(point: Point2, maximumRow: number): Point2 {
  return {
    x: point.x,
    y: (maximumRow + 1) * GRIDFINITY_SPEC.gridPitch - point.y,
  };
}

/** Mirror a free-form millimetre wall into generation coordinates. */
export function mirrorWall(wall: Wall, maximumRow: number): Wall {
  return {
    ...wall,
    start: mirrorMillimetrePoint(wall.start, maximumRow),
    end: mirrorMillimetrePoint(wall.end, maximumRow),
  };
}
