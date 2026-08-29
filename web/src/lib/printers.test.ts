import { describe, expect, it } from 'vitest';
import { partitionCells } from './cuts';
import { checkBedFit, checkDesignFit, cutsForPrinter } from './printers';
import type { BinDesign, Cell, PrinterSettings } from './types';

const prusaMk4: PrinterSettings = {
  name: 'Prusa MK4 / MK3S+',
  bedWidth: 250,
  bedDepth: 210,
};

function rectangle(width: number, depth: number, offsetX = 0, offsetY = 0): Cell[] {
  return Array.from({ length: depth }, (_, y) =>
    Array.from({ length: width }, (_, x) => ({ x: x + offsetX, y: y + offsetY })),
  ).flat();
}

function bin(cells: Cell[], cuts = []): BinDesign {
  return { id: 'bin-1', cells, cuts, openings: [], walls: [] };
}

describe('printer planning', () => {
  it('accepts a bin that fits only after a 90-degree rotation', () => {
    expect(checkBedFit(rectangle(4, 5), prusaMk4)).toEqual({
      fits: true,
      width: 168,
      depth: 210,
      rotated: true,
    });
  });

  it('recursively seeds fitting cuts and keeps representative plans bounded', () => {
    for (const [width, depth] of [[1, 1], [4, 5], [6, 5], [11, 9], [20, 13]]) {
      const cells = rectangle(width, depth);
      const cuts = cutsForPrinter(cells, prusaMk4);
      expect(partitionCells(cells, cuts).every((part) => checkBedFit(part.cells, prusaMk4).fits))
        .toBe(true);
      expect(cuts.length).toBeLessThan(cells.length);
    }
  });

  it('preserves an existing fitting cut plan', () => {
    const cells = rectangle(4, 1);
    const existing = [{ start: { x: 2, y: 0 }, end: { x: 2, y: 1 } }];
    expect(cutsForPrinter(cells, prusaMk4, existing)).toEqual(existing);
  });

  it('reports fit across multiple logical bins and their current parts', () => {
    const bins = [
      bin(rectangle(4, 5)),
      { ...bin(rectangle(6, 1, 10)), id: 'bin-2' },
    ];
    const result = checkDesignFit(bins, prusaMk4);
    expect(result).toMatchObject({ allFit: false, parts: 2 });
    expect(result.worst).toMatchObject({ width: 252, depth: 42 });
  });
});
