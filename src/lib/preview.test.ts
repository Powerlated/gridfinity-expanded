import { describe, expect, it } from 'vitest';
import { previewLayout } from './preview';
import type { Bin, Design } from './types';

const design: Design = {
  bins: [{
    id: 'bin-1',
    cells: [{ x: 0, y: 0 }, { x: 1, y: 0 }, { x: 2, y: 0 }],
    openings: [],
    walls: [],
    cuts: [{ start: { x: 1, y: 0 }, end: { x: 1, y: 1 } }],
  }],
  heightUnits: 3,
  perimeterThickness: 1.2,
  filletRadius: 2.8,
  fasteners: { magnets: false, m3: false },
  printer: { name: 'Editor only', bedWidth: 100, bedDepth: 100 },
};

const bins: Bin[] = [{
  binId: 'bin-1',
  pieces: [
    { vertices: new Float32Array(18), cells: [{ x: 0, y: 0 }] },
    { vertices: new Float32Array(18), cells: [{ x: 1, y: 0 }, { x: 2, y: 0 }] },
  ],
}];

describe('preview layout', () => {
  it('separates cut pieces with the multipart preview gap', () => {
    const pieces = previewLayout(bins, design);
    expect(pieces.map((piece) => piece.previewOffset)).toEqual([
      { x: -0.15, y: 0 },
      { x: 0.15, y: 0 },
    ]);
    expect(pieces.map((piece) => piece.pieceIndex)).toEqual([0, 1]);
  });

  it('leaves uncut bins at their model position', () => {
    const single: Bin[] = [{
      binId: 'bin-1',
      pieces: [{ vertices: new Float32Array(18), cells: [{ x: 0, y: 0 }] }],
    }];
    expect(previewLayout(single, design)[0].previewOffset).toEqual({ x: 0, y: 0 });
    expect(previewLayout(single, null)[0].previewOffset).toEqual({ x: 0, y: 0 });
    expect(previewLayout(single, { ...design, bins: [] })[0].previewOffset)
      .toEqual({ x: 0, y: 0 });
  });

  it('mirrors horizontal cuts before spacing generation-coordinate pieces', () => {
    const horizontalDesign: Design = {
      ...design,
      bins: [{
        id: 'bin-1',
        cells: [{ x: 0, y: 0 }, { x: 0, y: 1 }],
        openings: [],
        walls: [],
        cuts: [{ start: { x: 0, y: 1 }, end: { x: 1, y: 1 } }],
      }],
    };
    const horizontalBins: Bin[] = [{
      binId: 'bin-1',
      pieces: [
        { vertices: new Float32Array(18), cells: [{ x: 0, y: 1 }] },
        { vertices: new Float32Array(18), cells: [{ x: 0, y: 0 }] },
      ],
    }];

    expect(previewLayout(horizontalBins, horizontalDesign)
      .map((piece) => piece.previewOffset)).toEqual([
      { x: 0, y: 0.15 },
      { x: 0, y: -0.15 },
    ]);
  });
});
