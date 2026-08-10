import type { Placement, Rect, Wall } from '../types';
import {
  boundarySegments,
  mergeSegments,
  quantize,
  rectBottom,
  rectRight,
  type Segment,
} from './rects';

export const MIN_GENERATED_WALL_LENGTH = 5;

function onAreaBoundary(segment: Segment, area: Rect): boolean {
  const coordinate = quantize(segment.coordinate);
  return segment.orientation === 'v'
    ? coordinate === quantize(area.x) || coordinate === quantize(rectRight(area))
    : coordinate === quantize(area.y) || coordinate === quantize(rectBottom(area));
}

function toWall(segment: Segment, extension: number, width: number): Wall {
  const start = quantize(segment.start - extension);
  const end = quantize(segment.end + extension);
  return segment.orientation === 'v'
    ? { start: { x: segment.coordinate, y: start }, end: { x: segment.coordinate, y: end }, width }
    : { start: { x: start, y: segment.coordinate }, end: { x: end, y: segment.coordinate }, width };
}

export function layoutWalls(
  placements: Placement[],
  area: Rect,
  dividerThickness: number,
): Wall[] {
  const segments = placements.flatMap((placement) => boundarySegments(placement.parts));
  return mergeSegments(segments)
    .filter((segment) => !onAreaBoundary(segment, area))
    .filter((segment) => segment.end - segment.start >= MIN_GENERATED_WALL_LENGTH)
    .map((segment) => toWall(segment, dividerThickness / 2, dividerThickness));
}
