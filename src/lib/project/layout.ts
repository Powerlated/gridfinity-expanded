import { cutsForPrinter } from '../printers';
import type { BinDesign, PackResult, PrinterSettings, Project } from '../types';
import { drawerCells, drawerGrid, packingArea } from './drawer';
import { layoutWalls } from './walls';

export const DRAWER_BIN_ID = 'drawer';

export function buildDrawerBin(
  project: Project,
  result: PackResult,
  printer: PrinterSettings,
  perimeterThickness: number,
  maxGrid: number,
): BinDesign {
  const grid = drawerGrid(project.drawer, maxGrid);
  const cells = drawerCells(grid);
  const area = packingArea(grid, perimeterThickness);
  return {
    id: DRAWER_BIN_ID,
    cells,
    openings: [],
    walls: layoutWalls(result.placements, area, project.dividerThickness),
    cuts: cutsForPrinter(cells, printer),
  };
}
