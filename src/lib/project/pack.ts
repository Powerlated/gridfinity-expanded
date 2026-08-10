import type {
  PackEffort,
  PackInput,
  PackResult,
  Placement,
  ProjectObject,
  Rect,
  Rotation,
} from '../types';
import {
  ROTATIONS,
  inflateParts,
  normalizeParts,
  partsBounds,
  partsKey,
  quantize,
  rectBottom,
  rectContains,
  rectRight,
  rectsOverlap,
  rotateParts,
  translateParts,
  unionArea,
} from './rects';

export const PACK_RESTARTS: Record<PackEffort, number> = {
  quick: 30,
  standard: 200,
  thorough: 800,
};

const PACK_SEED = 0x9e3779b9;
const STAGNATION_RESHUFFLE = 8;

interface Shape {
  rotation: Rotation;
  parts: Rect[];
  bounds: Rect;
}

interface Instance {
  objectId: string;
  instance: number;
  shapes: Shape[];
  key: string;
  area: number;
}

interface Attempt {
  parts: Rect[];
  rotation: Rotation;
  x: number;
  y: number;
}

interface Scored {
  placements: Placement[];
  count: number;
  area: number;
  spread: number;
}

function mulberry32(seed: number): () => number {
  let state = seed >>> 0;
  return () => {
    state = (state + 0x6d2b79f5) >>> 0;
    let value = Math.imul(state ^ (state >>> 15), 1 | state);
    value = (value + Math.imul(value ^ (value >>> 7), 61 | value)) ^ value;
    return ((value ^ (value >>> 14)) >>> 0) / 4294967296;
  };
}

function claimShapes(object: ProjectObject, margin: number): Shape[] {
  const base = normalizeParts(inflateParts(normalizeParts(object.parts), margin));
  const seen = new Set<string>();
  const shapes: Shape[] = [];
  for (const rotation of ROTATIONS) {
    const parts = rotateParts(base, rotation);
    const key = partsKey(parts);
    if (seen.has(key)) continue;
    seen.add(key);
    shapes.push({ rotation, parts, bounds: partsBounds(parts) });
  }
  return shapes;
}

function buildInstances(objects: ProjectObject[], margin: number): Instance[] {
  const instances: Instance[] = [];
  for (const object of objects) {
    if (object.parts.length === 0) continue;
    const shapes = claimShapes(object, margin);
    const key = partsKey(shapes[0].parts);
    const area = unionArea(shapes[0].parts);
    for (let instance = 0; instance < object.quantity; instance++) {
      instances.push({ objectId: object.id, instance, shapes, key, area });
    }
  }
  return instances;
}

function candidateAxis(base: number, limit: number, edges: number[], offsets: number[]): number[] {
  const values = new Set<number>();
  for (const edge of edges) {
    for (const offset of offsets) {
      const value = quantize(edge - offset);
      if (value >= base && value <= limit) values.add(value);
    }
  }
  values.add(base);
  return [...values].sort((a, b) => a - b);
}

function firstFit(shape: Shape, placed: Rect[], area: Rect, best: Attempt | null): Attempt | null {
  const limitX = quantize(rectRight(area) - shape.bounds.width);
  const limitY = quantize(rectBottom(area) - shape.bounds.depth);
  if (limitX < area.x || limitY < area.y) return null;
  const offsetsX = [...new Set(shape.parts.map((part) => part.x))];
  const offsetsY = [...new Set(shape.parts.map((part) => part.y))];
  const edgesX = [area.x, ...placed.map((rect) => rect.x), ...placed.map(rectRight)];
  const edgesY = [area.y, ...placed.map((rect) => rect.y), ...placed.map(rectBottom)];
  const xs = candidateAxis(area.x, limitX, edgesX, offsetsX);
  const ys = candidateAxis(area.y, limitY, edgesY, offsetsY);
  for (const y of ys) {
    if (best && y > best.y) break;
    for (const x of xs) {
      if (best && y === best.y && x >= best.x) break;
      const parts = translateParts(shape.parts, x, y);
      if (parts.some((part) => !rectContains(area, part))) continue;
      if (parts.some((part) => placed.some((other) => rectsOverlap(part, other)))) continue;
      return { parts, rotation: shape.rotation, x, y };
    }
  }
  return null;
}

function placeInstance(instance: Instance, placed: Rect[], area: Rect): Attempt | null {
  let best: Attempt | null = null;
  for (const shape of instance.shapes) {
    const found = firstFit(shape, placed, area, best);
    if (found) best = found;
  }
  return best;
}

function packOnce(order: Instance[], area: Rect): Scored {
  const placed: Rect[] = [];
  const placements: Placement[] = [];
  const blocked = new Set<string>();
  let claimed = 0;
  let spread = 0;
  for (const instance of order) {
    if (blocked.has(instance.key)) continue;
    const attempt = placeInstance(instance, placed, area);
    if (!attempt) {
      blocked.add(instance.key);
      continue;
    }
    placements.push({
      objectId: instance.objectId,
      instance: instance.instance,
      rotation: attempt.rotation,
      parts: attempt.parts,
    });
    placed.push(...attempt.parts);
    claimed += instance.area;
    spread += attempt.x + attempt.y;
  }
  return { placements, count: placements.length, area: claimed, spread };
}

function better(candidate: Scored, incumbent: Scored): boolean {
  if (candidate.count !== incumbent.count) return candidate.count > incumbent.count;
  if (candidate.area !== incumbent.area) return candidate.area > incumbent.area;
  return candidate.spread < incumbent.spread;
}

function perturb(order: Instance[], random: () => number, stagnation: number): Instance[] {
  const next = [...order];
  if (next.length < 2) return next;
  const swaps = stagnation >= STAGNATION_RESHUFFLE
    ? next.length
    : 1 + Math.floor(random() * 3);
  for (let index = 0; index < swaps; index++) {
    const a = Math.floor(random() * next.length);
    const b = Math.floor(random() * next.length);
    [next[a], next[b]] = [next[b], next[a]];
  }
  return next;
}

function toResult(scored: Scored, objects: ProjectObject[], iterations: number): PackResult {
  const placedByObjectId: Record<string, number> = {};
  for (const object of objects) placedByObjectId[object.id] = 0;
  for (const placement of scored.placements) {
    placedByObjectId[placement.objectId] = (placedByObjectId[placement.objectId] ?? 0) + 1;
  }
  return { placements: scored.placements, placedByObjectId, iterations };
}

export interface PackSearch {
  readonly total: number;
  readonly done: number;
  step: (iterations: number) => boolean;
  result: () => PackResult;
}

export function createPackSearch(input: PackInput): PackSearch {
  const margin = input.clearance + input.dividerThickness / 2;
  const instances = buildInstances(input.objects, margin);
  const total = instances.length === 0 ? 0 : PACK_RESTARTS[input.effort];
  const random = mulberry32(PACK_SEED);
  let order = [...instances].sort((a, b) => b.area - a.area || a.objectId.localeCompare(b.objectId));
  let best = (packOnce(order, input.area));
  let stagnation = 0;
  let done = 0;

  return {
    total,
    get done() {
      return done;
    },
    step(iterations: number): boolean {
      const until = Math.min(total, done + iterations);
      while (done < until) {
        done++;
        const candidateOrder = perturb(order, random, stagnation);
        const scored = (packOnce(candidateOrder, input.area));
        if (better(scored, best)) {
          best = scored;
          order = candidateOrder;
          stagnation = 0;
        } else {
          stagnation++;
        }
      }
      return done < total;
    },
    result(): PackResult {
      return toResult(best, input.objects, done);
    },
  };
}

export function packLayout(input: PackInput): PackResult {
  const search = createPackSearch(input);
  while (search.step(Number.MAX_SAFE_INTEGER));
  return search.result();
}
