import { describe, expect, it } from 'vitest';
import {
  ROTATIONS,
  boundarySegments,
  inflateParts,
  mergeSegments,
  normalizeParts,
  partsBounds,
  partsConnected,
  partsKey,
  rotateParts,
  unionArea,
} from './rects';
import type { Rect } from '../types';

function rect(x: number, y: number, width: number, depth: number): Rect {
  return { x, y, width, depth };
}

const square = [rect(0, 0, 20, 20)];
const elbow = [rect(0, 0, 10, 30), rect(0, 20, 40, 10)];

describe('rectilinear object parts', () => {
  it('returns to the original shape after four quarter turns', () => {
    for (const parts of [square, elbow]) {
      let turned = normalizeParts(parts);
      for (let turn = 0; turn < 4; turn++) turned = rotateParts(turned, 90);
      expect(partsKey(turned)).toBe(partsKey(normalizeParts(parts)));
    }
  });

  it('preserves area under every rotation and swaps the bounds of a quarter turn', () => {
    for (const rotation of ROTATIONS) {
      const turned = rotateParts(elbow, rotation);
      expect(unionArea(turned)).toBeCloseTo(unionArea(elbow));
      const bounds = partsBounds(turned);
      const original = partsBounds(elbow);
      const swapped = rotation === 90 || rotation === 270;
      expect(bounds.width).toBeCloseTo(swapped ? original.depth : original.width);
      expect(bounds.depth).toBeCloseTo(swapped ? original.width : original.depth);
    }
  });

  it('measures the union rather than the sum of overlapping boxes', () => {
    expect(unionArea([rect(0, 0, 10, 10), rect(5, 0, 10, 10)])).toBeCloseTo(150);
    expect(unionArea(elbow)).toBeCloseTo(10 * 30 + 30 * 10);
  });

  it('grows a shape by the same margin on every side', () => {
    const grown = inflateParts(square, 2);
    expect(partsBounds(grown)).toEqual({ x: -2, y: -2, width: 24, depth: 24 });
    expect(partsBounds(inflateParts(grown, -2))).toEqual(partsBounds(square));
  });

  it('rejects boxes that only touch at a corner', () => {
    expect(partsConnected(elbow)).toBe(true);
    expect(partsConnected([rect(0, 0, 10, 10), rect(10, 0, 10, 10)])).toBe(true);
    expect(partsConnected([rect(0, 0, 10, 10), rect(10, 10, 10, 10)])).toBe(false);
    expect(partsConnected([rect(0, 0, 10, 10), rect(30, 0, 10, 10)])).toBe(false);
  });

  it('traces only the outside of a shape, never the seam between its own boxes', () => {
    const segments = mergeSegments(boundarySegments([rect(0, 0, 10, 10), rect(10, 0, 10, 10)]));
    expect(segments.filter((segment) => segment.orientation === 'v').map((segment) => segment.coordinate))
      .toEqual([0, 20]);
    expect(segments.filter((segment) => segment.orientation === 'h').map((segment) => segment.coordinate))
      .toEqual([0, 10]);
  });

  it('merges collinear and duplicated runs into one span', () => {
    const merged = mergeSegments([
      { orientation: 'v', coordinate: 5, start: 0, end: 10 },
      { orientation: 'v', coordinate: 5, start: 10, end: 20 },
      { orientation: 'v', coordinate: 5, start: 0, end: 10 },
      { orientation: 'v', coordinate: 5, start: 40, end: 50 },
    ]);
    expect(merged).toEqual([
      { orientation: 'v', coordinate: 5, start: 0, end: 20 },
      { orientation: 'v', coordinate: 5, start: 40, end: 50 },
    ]);
  });
});
