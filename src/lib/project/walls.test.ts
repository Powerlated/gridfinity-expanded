import { describe, expect, it } from 'vitest';
import { packLayout } from './pack';
import { partsBounds, rectBottom, rectRight } from './rects';
import { MIN_GENERATED_WALL_LENGTH, layoutWalls } from './walls';
import type { PackInput, Placement, Point2, ProjectObject, Rect, Wall } from '../types';

const AREA: Rect = { x: 0, y: 0, width: 200, depth: 200 };
const DIVIDER = 2;
const CLEARANCE = 0.5;
const STEP = 0.5;

function object(id: string, width: number, depth: number, quantity: number): ProjectObject {
  return { id, name: id, quantity, parts: [{ x: 0, y: 0, width, depth }] };
}

function input(objects: ProjectObject[], area = AREA): PackInput {
  return {
    area,
    objects,
    dividerThickness: DIVIDER,
    clearance: CLEARANCE,
    effort: 'quick',
  };
}

function wallRect(wall: Wall): Rect {
  const half = wall.width / 2;
  return wall.start.x === wall.end.x
    ? {
      x: wall.start.x - half,
      y: Math.min(wall.start.y, wall.end.y),
      width: wall.width,
      depth: Math.abs(wall.end.y - wall.start.y),
    }
    : {
      x: Math.min(wall.start.x, wall.end.x),
      y: wall.start.y - half,
      width: Math.abs(wall.end.x - wall.start.x),
      depth: wall.width,
    };
}

function centre(placement: Placement): Point2 {
  const bounds = partsBounds(placement.parts);
  return { x: bounds.x + bounds.width / 2, y: bounds.y + bounds.depth / 2 };
}

function reachable(from: Point2, walls: Wall[], area: Rect): Set<number> {
  const cols = Math.ceil(area.width / STEP);
  const rows = Math.ceil(area.depth / STEP);
  const blocks = walls.map(wallRect);
  const blocked = (col: number, row: number) => {
    const x = area.x + (col + 0.5) * STEP;
    const y = area.y + (row + 0.5) * STEP;
    return blocks.some((rect) =>
      x > rect.x && x < rectRight(rect) && y > rect.y && y < rectBottom(rect));
  };
  const start = Math.floor((from.y - area.y) / STEP) * cols
    + Math.floor((from.x - area.x) / STEP);
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
      if (seen.has(next) || blocked(nextCol, nextRow)) continue;
      seen.add(next);
      queue.push(next);
    }
  }
  return seen;
}

function cellOf(point: Point2, area: Rect): number {
  const cols = Math.ceil(area.width / STEP);
  return Math.floor((point.y - area.y) / STEP) * cols + Math.floor((point.x - area.x) / STEP);
}

describe('generated dividers', () => {
  const quad = packLayout(input([object('a', 80, 80, 4)]));
  const walls = layoutWalls(quad.placements, AREA, DIVIDER);

  it('separates every pair of compartments', () => {
    expect(quad.placements).toHaveLength(4);
    for (const placement of quad.placements) {
      const region = reachable(centre(placement), walls, AREA);
      for (const other of quad.placements) {
        if (other === placement) continue;
        expect(region.has(cellOf(centre(other), AREA))).toBe(false);
      }
    }
  });

  it('emits one shared divider where two compartments meet, not two', () => {
    const vertical = walls.filter((wall) => wall.start.x === wall.end.x);
    const coordinates = vertical.map((wall) => wall.start.x);
    expect(new Set(coordinates).size).toBe(coordinates.length);
  });

  it('never puts a divider on the cavity boundary', () => {
    for (const wall of walls) {
      const onEdge = wall.start.x === wall.end.x
        ? wall.start.x === AREA.x || wall.start.x === rectRight(AREA)
        : wall.start.y === AREA.y || wall.start.y === rectBottom(AREA);
      expect(onEdge).toBe(false);
    }
  });

  it('extends every divider half its thickness past the span it divides', () => {
    const single = layoutWalls(
      [{ objectId: 'a', instance: 0, rotation: 0, parts: [{ x: 0, y: 0, width: 50, depth: 50 }] }],
      AREA,
      DIVIDER,
    );
    expect(single).toEqual([
      { start: { x: -1, y: 50 }, end: { x: 51, y: 50 }, width: DIVIDER },
      { start: { x: 50, y: -1 }, end: { x: 50, y: 51 }, width: DIVIDER },
    ]);
  });

  it('drops runs shorter than the minimum wall length', () => {
    const sliver = layoutWalls(
      [{ objectId: 'a', instance: 0, rotation: 0, parts: [{ x: 0, y: 0, width: 3, depth: 3 }] }],
      AREA,
      DIVIDER,
    );
    expect(sliver).toEqual([]);
    expect(MIN_GENERATED_WALL_LENGTH).toBeGreaterThan(3);
  });

  it('emits nothing for an empty layout', () => {
    expect(layoutWalls([], AREA, DIVIDER)).toEqual([]);
  });
});
