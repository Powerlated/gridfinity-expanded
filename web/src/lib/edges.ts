import type { Cell, Edge } from './types';

export type EdgeKind = 'perimeter' | 'internal' | 'none';

export function cellKey(cell: Cell): string {
  return `${cell.x},${cell.y}`;
}

export function edgeKey(edge: Edge): string {
  return `${edge.orientation}:${edge.x},${edge.y}`;
}

export function cellSet(cells: Cell[]): Set<string> {
  return new Set(cells.map(cellKey));
}

/** The two cells separated by an edge. */
export function adjacentCells(edge: Edge): [Cell, Cell] {
  return edge.orientation === 'v'
    ? [{ x: edge.x - 1, y: edge.y }, { x: edge.x, y: edge.y }]
    : [{ x: edge.x, y: edge.y - 1 }, { x: edge.x, y: edge.y }];
}

export function classifyEdge(cells: Set<string>, edge: Edge): EdgeKind {
  const [a, b] = adjacentCells(edge);
  const hasA = cells.has(cellKey(a));
  const hasB = cells.has(cellKey(b));
  if (hasA && hasB) return 'internal';
  if (hasA || hasB) return 'perimeter';
  return 'none';
}

export function edgeInsideCell(cells: Set<string>, edge: Edge): Cell | null {
  const [a, b] = adjacentCells(edge);
  const hasA = cells.has(cellKey(a));
  const hasB = cells.has(cellKey(b));
  if (hasA === hasB) return null;
  return hasA ? a : b;
}

export function cellEdges(cell: Cell): Edge[] {
  return [
    { orientation: 'v', x: cell.x, y: cell.y },
    { orientation: 'v', x: cell.x + 1, y: cell.y },
    { orientation: 'h', x: cell.x, y: cell.y },
    { orientation: 'h', x: cell.x, y: cell.y + 1 },
  ];
}

function edgesOfKind(cells: Cell[], kind: EdgeKind): Edge[] {
  const set = cellSet(cells);
  const seen = new Set<string>();
  const edges: Edge[] = [];
  for (const cell of cells) {
    for (const edge of cellEdges(cell)) {
      const key = edgeKey(edge);
      if (seen.has(key)) continue;
      seen.add(key);
      if (classifyEdge(set, edge) === kind) edges.push(edge);
    }
  }
  return sortEdges(edges);
}

export function perimeterEdges(cells: Cell[]): Edge[] {
  return edgesOfKind(cells, 'perimeter');
}

export function internalEdges(cells: Cell[]): Edge[] {
  return edgesOfKind(cells, 'internal');
}

export function sortEdges(edges: Edge[]): Edge[] {
  return [...edges].sort((a, b) =>
    a.orientation.localeCompare(b.orientation) || a.y - b.y || a.x - b.x);
}

export function toggleByKey<T>(items: T[], item: T, keyOf: (value: T) => string): T[] {
  const key = keyOf(item);
  const without = items.filter((value) => keyOf(value) !== key);
  return without.length === items.length ? [...without, item] : without;
}

export function toggleEdge(edges: Edge[], edge: Edge): Edge[] {
  return sortEdges(toggleByKey(edges, edge, edgeKey));
}
