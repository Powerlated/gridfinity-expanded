import { GRIDFINITY_DERIVED, GRIDFINITY_SPEC } from '../gridfinitySpec';
import type { Cell, Drawer, Rect } from '../types';

export interface DrawerGrid {
  cols: number;
  rows: number;
  marginX: number;
  marginY: number;
}

export function drawerGrid(drawer: Drawer, maxGrid: number): DrawerGrid {
  const pitch = GRIDFINITY_SPEC.gridPitch;
  const cols = Math.min(maxGrid, Math.max(0, Math.floor(drawer.width / pitch)));
  const rows = Math.min(maxGrid, Math.max(0, Math.floor(drawer.depth / pitch)));
  return {
    cols,
    rows,
    marginX: drawer.width - cols * pitch,
    marginY: drawer.depth - rows * pitch,
  };
}

export function packingInset(perimeterThickness: number): number {
  return GRIDFINITY_DERIVED.perimeterClearancePerSide + perimeterThickness;
}

export function packingArea(grid: DrawerGrid, perimeterThickness: number): Rect {
  const inset = packingInset(perimeterThickness);
  const pitch = GRIDFINITY_SPEC.gridPitch;
  return {
    x: inset,
    y: inset,
    width: Math.max(0, grid.cols * pitch - inset * 2),
    depth: Math.max(0, grid.rows * pitch - inset * 2),
  };
}

export function drawerCells(grid: DrawerGrid): Cell[] {
  const cells: Cell[] = [];
  for (let y = 0; y < grid.rows; y++) {
    for (let x = 0; x < grid.cols; x++) cells.push({ x, y });
  }
  return cells;
}
