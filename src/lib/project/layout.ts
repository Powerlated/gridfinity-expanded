import { cutsForPrinter } from '../printers';
import type { BinDesign, Drawer, PackResult, PrinterSettings } from '../types';
import { drawerCells, drawerGrid } from './drawer';

/**
 * The drawer bin: one bin covering as much of the drawer as whole Gridfinity
 * cells reach, its compartments divided by the walls the packer derived, and
 * cut wherever the printer's bed requires.
 */

export const DRAWER_BIN_ID = 'drawer';

/**
 * A drawer and a finished layout as the single `BinDesign` that realises them:
 * `floor(drawer / 42)` cells capped at `maxGrid`, the layout's own dividers as
 * the bin's walls, no openings, and the cuts `cutsForPrinter` needs to fit that
 * cell set on `printer`'s bed.
 */
export function buildDrawerBin(
  drawer: Drawer,
  result: PackResult,
  printer: PrinterSettings,
  maxGrid: number,
): BinDesign {
  const grid = drawerGrid(drawer, maxGrid);
  const cells = drawerCells(grid);
  return {
    id: DRAWER_BIN_ID,
    cells,
    openings: [],
    walls: result.walls,
    cuts: cutsForPrinter(cells, printer),
  };
}
