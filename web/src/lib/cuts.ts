import { cellKey } from './edges';
import type { BinDesign, Cell, Cut } from './types';

export interface DisplayCell extends Cell {
  binId: string;
}

export interface CellPart {
  id: string;
  cells: Cell[];
}

export function flattenBins(bins: BinDesign[]): DisplayCell[] {
  return bins.flatMap((bin) => bin.cells.map((cell) => ({ ...cell, binId: bin.id })));
}

export function canonicalCut(cut: Cut): Cut {
  if (cut.start.x === cut.end.x) {
    return cut.start.y <= cut.end.y ? cut : { start: cut.end, end: cut.start };
  }
  return cut.start.x <= cut.end.x ? cut : { start: cut.end, end: cut.start };
}

export function cutKey(value: Cut): string {
  const cut = canonicalCut(value);
  return `${cut.start.x},${cut.start.y}:${cut.end.x},${cut.end.y}`;
}

export function sortCuts(cuts: Cut[]): Cut[] {
  const unique = new Map(cuts.map((cut) => [cutKey(cut), canonicalCut(cut)]));
  return [...unique.values()].sort((a, b) =>
    a.start.x - b.start.x || a.start.y - b.start.y ||
    a.end.x - b.end.x || a.end.y - b.end.y);
}

export function toggleCut(cuts: Cut[], cut: Cut): Cut[] {
  const key = cutKey(cut);
  return sortCuts(cuts.some((value) => cutKey(value) === key)
    ? cuts.filter((value) => cutKey(value) !== key)
    : [...cuts, cut]);
}

/** Unit internal edges covered by the exact cut segments. */
function severedEdges(cuts: Cut[]): Set<string> {
  const edges = new Set<string>();
  for (const value of cuts) {
    const cut = canonicalCut(value);
    if (cut.start.x === cut.end.x) {
      for (let y = cut.start.y; y < cut.end.y; y++) {
        edges.add(`v:${cut.start.x},${y}`);
      }
    } else if (cut.start.y === cut.end.y) {
      for (let x = cut.start.x; x < cut.end.x; x++) {
        edges.add(`h:${x},${cut.start.y}`);
      }
    }
  }
  return edges;
}

/** Connected cell components after cut-covered adjacencies are removed. */
export function partitionCells(cells: Cell[], cuts: Cut[]): CellPart[] {
  if (cells.length === 0) return [];
  if (cuts.length === 0) {
    return [{
      id: 'part-1',
      cells: [...cells].sort((a, b) => a.y - b.y || a.x - b.x),
    }];
  }
  const byKey = new Map(cells.map((cell) => [cellKey(cell), cell]));
  const severed = severedEdges(cuts);
  const unseen = new Set(byKey.keys());
  const parts: Cell[][] = [];

  while (unseen.size > 0) {
    const first = unseen.values().next().value as string;
    unseen.delete(first);
    const queue = [byKey.get(first)!];
    const part: Cell[] = [];
    while (queue.length > 0) {
      const cell = queue.shift()!;
      part.push(cell);
      const neighbors: Array<[Cell, string]> = [
        [{ x: cell.x - 1, y: cell.y }, `v:${cell.x},${cell.y}`],
        [{ x: cell.x + 1, y: cell.y }, `v:${cell.x + 1},${cell.y}`],
        [{ x: cell.x, y: cell.y - 1 }, `h:${cell.x},${cell.y}`],
        [{ x: cell.x, y: cell.y + 1 }, `h:${cell.x},${cell.y + 1}`],
      ];
      for (const [neighbor, edge] of neighbors) {
        const key = cellKey(neighbor);
        if (!severed.has(edge) && unseen.delete(key)) queue.push(byKey.get(key)!);
      }
    }
    part.sort((a, b) => a.y - b.y || a.x - b.x);
    parts.push(part);
  }

  return parts
    .sort((a, b) =>
      Math.min(...a.map((cell) => cell.y)) - Math.min(...b.map((cell) => cell.y)) ||
      Math.min(...a.map((cell) => cell.x)) - Math.min(...b.map((cell) => cell.x)))
    .map((part, index) => ({ id: `part-${index + 1}`, cells: part }));
}

interface UnitCut {
  orientation: 'h' | 'v';
  line: number;
  along: number;
}

function unitInternalCuts(cells: Cell[]): UnitCut[] {
  const set = new Set(cells.map(cellKey));
  const units: UnitCut[] = [];
  for (const cell of cells) {
    if (set.has(cellKey({ x: cell.x + 1, y: cell.y }))) {
      units.push({ orientation: 'v', line: cell.x + 1, along: cell.y });
    }
    if (set.has(cellKey({ x: cell.x, y: cell.y + 1 }))) {
      units.push({ orientation: 'h', line: cell.y + 1, along: cell.x });
    }
  }
  return units;
}

function mergeUnitCuts(units: UnitCut[]): Cut[] {
  const groups = new Map<string, UnitCut[]>();
  for (const unit of units) {
    const key = `${unit.orientation}:${unit.line}`;
    groups.set(key, [...(groups.get(key) ?? []), unit]);
  }
  const cuts: Cut[] = [];
  for (const group of groups.values()) {
    group.sort((a, b) => a.along - b.along);
    let start = group[0].along;
    let end = start + 1;
    const push = () => {
      const { orientation, line } = group[0];
      cuts.push(orientation === 'v'
        ? { start: { x: line, y: start }, end: { x: line, y: end } }
        : { start: { x: start, y: line }, end: { x: end, y: line } });
    };
    for (const unit of group.slice(1)) {
      if (unit.along === end) {
        end++;
      } else {
        push();
        start = unit.along;
        end = start + 1;
      }
    }
    push();
  }
  return sortCuts(cuts);
}

/** Maximal straight cut segments available on shared cell edges. */
export function availableCuts(cells: Cell[]): Cut[] {
  return mergeUnitCuts(unitInternalCuts(cells));
}

/** All segments required to sever a part at one grid-line bisection. */
function cutsAtLine(cells: Cell[], axis: 'x' | 'y', index: number): Cut[] {
  const orientation = axis === 'x' ? 'v' : 'h';
  return mergeUnitCuts(unitInternalCuts(cells).filter((unit) =>
    unit.orientation === orientation && unit.line === index));
}

/** Closest available cell-count bisection, with deterministic x-before-y ties. */
export function closestCellBisection(cells: Cell[]): Cut[] {
  if (cells.length < 2) return [];
  type Candidate = { cuts: Cut[]; imbalance: number; largest: number; axisOrder: number; index: number };
  const candidates: Candidate[] = [];
  for (const [axis, axisOrder] of [['x', 0], ['y', 1]] as const) {
    const values = cells.map((cell) => cell[axis]);
    const min = Math.min(...values);
    const max = Math.max(...values);
    for (let index = min + 1; index <= max; index++) {
      const before = values.filter((value) => value < index).length;
      const after = cells.length - before;
      const cuts = cutsAtLine(cells, axis, index);
      if (before === 0 || after === 0 || cuts.length === 0) continue;
      candidates.push({
        cuts,
        imbalance: Math.abs(before - after),
        largest: Math.max(before, after),
        axisOrder,
        index,
      });
    }
  }
  candidates.sort((a, b) =>
    a.imbalance - b.imbalance || a.largest - b.largest ||
    a.axisOrder - b.axisOrder || a.index - b.index);
  return candidates[0]?.cuts ?? [];
}

/**
 * Preserve existing cuts, then recursively bisect only non-fitting parts.
 * The alpha editor supplies connected valid bins; no disconnected-input
 * repair or policy is introduced here.
 */
export function addCutsUntilFit(
  cells: Cell[], existing: Cut[], fits: (partCells: Cell[]) => boolean,
): Cut[] {
  let cuts = sortCuts(existing);
  for (let guard = 0; guard < cells.length * 2; guard++) {
    const failing = partitionCells(cells, cuts).find((part) => !fits(part.cells));
    if (!failing) return cuts;
    const additions = closestCellBisection(failing.cells)
      .filter((cut) => !cuts.some((existingCut) => cutKey(existingCut) === cutKey(cut)));
    if (additions.length === 0) return cuts;
    cuts = sortCuts([...cuts, ...additions]);
  }
  return cuts;
}
