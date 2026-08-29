import { addCutsUntilFit, partitionCells } from './cuts';
import { GRIDFINITY_SPEC, IMPLEMENTATION_ALLOWANCES } from './gridfinitySpec';
import type { BedFitResult, BinDesign, Cell, Cut, PrinterSettings } from './types';

export const PRINTER_PROFILES: PrinterSettings[] = [
  { name: 'Bambu Lab A1 Mini', bedWidth: 180, bedDepth: 180 },
  { name: 'Bambu Lab P1S / X1C', bedWidth: 256, bedDepth: 256 },
  { name: 'Creality Ender 3 / V2', bedWidth: 220, bedDepth: 220 },
  { name: 'Creality K1', bedWidth: 220, bedDepth: 220 },
  { name: 'Elegoo Centauri Carbon / Carbon 2', bedWidth: 256, bedDepth: 256 },
  { name: 'Prusa MK4 / MK3S+', bedWidth: 250, bedDepth: 210 },
  { name: 'Prusa Mini+', bedWidth: 180, bedDepth: 180 },
  { name: 'Voron 2.4 (250mm)', bedWidth: 250, bedDepth: 250 },
  { name: 'Voron 2.4 (300mm)', bedWidth: 300, bedDepth: 300 },
  { name: 'Custom', bedWidth: 220, bedDepth: 220 },
];

export function footprintCells(cells: Cell[]): { width: number; depth: number } {
  if (cells.length === 0) return { width: 0, depth: 0 };
  const xs = cells.map((cell) => cell.x);
  const ys = cells.map((cell) => cell.y);
  return {
    width: Math.max(...xs) - Math.min(...xs) + 1,
    depth: Math.max(...ys) - Math.min(...ys) + 1,
  };
}

export function checkBedFit(cells: Cell[], printer: PrinterSettings): BedFitResult {
  const footprint = footprintCells(cells);
  const width = footprint.width * GRIDFINITY_SPEC.gridPitch;
  const depth = footprint.depth * GRIDFINITY_SPEC.gridPitch;
  const margin = IMPLEMENTATION_ALLOWANCES.bedClearancePerSide * 2;
  const normal = width + margin <= printer.bedWidth && depth + margin <= printer.bedDepth;
  const rotated = depth + margin <= printer.bedWidth && width + margin <= printer.bedDepth;
  return { fits: normal || rotated, width, depth, rotated: !normal && rotated };
}

export function cutsForPrinter(cells: Cell[], printer: PrinterSettings, existing: Cut[] = []): Cut[] {
  return addCutsUntilFit(cells, existing, (part) => checkBedFit(part, printer).fits);
}

export interface DesignFitResult {
  allFit: boolean;
  parts: number;
  worst: BedFitResult;
}

export function checkDesignFit(bins: BinDesign[], printer: PrinterSettings): DesignFitResult {
  let worst: BedFitResult = { fits: true, width: 0, depth: 0, rotated: false };
  let parts = 0;
  let allFit = true;
  for (const bin of bins) {
    for (const part of partitionCells(bin.cells, bin.cuts)) {
      parts++;
      const fit = checkBedFit(part.cells, printer);
      if (!fit.fits) allFit = false;
      const area = fit.width * fit.depth;
      const worstArea = worst.width * worst.depth;
      if ((!fit.fits && worst.fits) || fit.fits === worst.fits && area >= worstArea) worst = fit;
    }
  }
  return { allFit, parts, worst };
}
