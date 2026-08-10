import { describe, expect, it } from 'vitest';
import { packLayout } from './pack';
import { rectContains, rectsOverlap } from './rects';
import type { PackInput, ProjectObject, Rect } from '../types';

const AREA: Rect = { x: 0, y: 0, width: 100, depth: 100 };

function object(
  id: string,
  width: number,
  depth: number,
  quantity = 1,
  parts?: Rect[],
): ProjectObject {
  return { id, name: id, quantity, parts: parts ?? [{ x: 0, y: 0, width, depth }] };
}

function input(objects: ProjectObject[], area = AREA): PackInput {
  return { area, objects, dividerThickness: 0, clearance: 0, effort: 'quick' };
}

function placedRects(objects: ProjectObject[], area = AREA): Rect[] {
  return packLayout(input(objects, area)).placements.flatMap((placement) => placement.parts);
}

describe('drawer packing', () => {
  it('fills an area that the objects tile exactly', () => {
    const result = packLayout(input([object('a', 20, 20, 25)]));
    expect(result.placements).toHaveLength(25);
    expect(result.placedByObjectId).toEqual({ a: 25 });
  });

  it('never overlaps two placements and never leaves the area', () => {
    const rects = placedRects([
      object('a', 30, 20, 6),
      object('b', 15, 45, 4),
      object('c', 10, 10, 12),
      object('d', 25, 25, 3, [{ x: 0, y: 0, width: 25, depth: 10 }, { x: 0, y: 10, width: 10, depth: 15 }]),
    ]);
    expect(rects.length).toBeGreaterThan(0);
    for (const rect of rects) expect(rectContains(AREA, rect)).toBe(true);
    for (let a = 0; a < rects.length; a++) {
      for (let b = a + 1; b < rects.length; b++) {
        expect(rectsOverlap(rects[a], rects[b])).toBe(false);
      }
    }
  });

  it('rotates an object that only fits the other way round', () => {
    const result = packLayout(input([object('a', 60, 20)], { x: 0, y: 0, width: 20, depth: 100 }));
    expect(result.placements).toHaveLength(1);
    expect([90, 270]).toContain(result.placements[0].rotation);
  });

  it('places no more than the requested quantity and reports the shortfall', () => {
    const result = packLayout(input([object('a', 60, 60, 4)]));
    expect(result.placements.length).toBeLessThanOrEqual(4);
    expect(result.placedByObjectId.a).toBe(result.placements.length);
    expect(result.placedByObjectId.a).toBeLessThan(4);
  });

  it('reserves the clearance and half a divider around every object', () => {
    const request = {
      ...input([object('a', 20, 20, 2)]),
      dividerThickness: 2,
      clearance: 0.5,
    };
    const result = packLayout(request);
    expect(result.placements).toHaveLength(2);
    for (const placement of result.placements) {
      expect(placement.parts[0].width).toBeCloseTo(23);
      expect(placement.parts[0].depth).toBeCloseTo(23);
    }
  });

  it('gives the same layout every time for the same input', () => {
    const objects = [object('a', 30, 20, 5), object('b', 12, 55, 4), object('c', 18, 18, 7)];
    expect(packLayout(input(objects))).toEqual(packLayout(input(objects)));
  });

  it('returns an empty layout when there is nothing to place', () => {
    expect(packLayout(input([]))).toEqual({ placements: [], placedByObjectId: {}, iterations: 0 });
  });
});
