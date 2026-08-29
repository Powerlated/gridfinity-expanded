import { describe, expect, it } from 'vitest';
import { previewLayout } from './preview';
import type { Bin } from './types';

const bins: Bin[] = [{
  binId: 'bin-1',
  pieces: [
    { vertices: new Float32Array(18), cells: [{ x: 0, y: 0 }] },
    { vertices: new Float32Array(18), cells: [{ x: 1, y: 0 }, { x: 2, y: 0 }] },
  ],
}];

describe('preview layout', () => {
  it('points every cut piece away from the bin centre', () => {
    const pieces = previewLayout(bins);
    expect(pieces.map((piece) => piece.apartDirection)).toEqual([
      { x: -1, y: 0 },
      { x: 1, y: 0 },
    ]);
    expect(pieces.map((piece) => piece.pieceIndex)).toEqual([0, 1]);
  });

  it('separates pieces along the axis they were cut on', () => {
    const stacked: Bin[] = [{
      binId: 'bin-1',
      pieces: [
        { vertices: new Float32Array(18), cells: [{ x: 0, y: 1 }] },
        { vertices: new Float32Array(18), cells: [{ x: 0, y: 0 }] },
      ],
    }];
    expect(previewLayout(stacked).map((piece) => piece.apartDirection)).toEqual([
      { x: 0, y: 1 },
      { x: 0, y: -1 },
    ]);
  });

  it('marks only the edges where a piece meets another piece', () => {
    const [left, right] = previewLayout(bins);
    expect(Array.from(left.cutSegments)).toEqual([0, 42, 0, 42]);
    expect(Array.from(right.cutSegments)).toEqual([0, 42, 0, 42]);
  });

  it('leaves an uncut bin with nowhere to move', () => {
    const single: Bin[] = [{
      binId: 'bin-1',
      pieces: [{ vertices: new Float32Array(18), cells: [{ x: 0, y: 0 }] }],
    }];
    expect(previewLayout(single)[0].apartDirection).toEqual({ x: 0, y: 0 });
  });

  it('keeps a piece centred on the whole bin where it is', () => {
    const row: Bin[] = [{
      binId: 'bin-1',
      pieces: [
        { vertices: new Float32Array(18), cells: [{ x: 0, y: 0 }] },
        { vertices: new Float32Array(18), cells: [{ x: 1, y: 0 }] },
        { vertices: new Float32Array(18), cells: [{ x: 2, y: 0 }] },
      ],
    }];
    expect(previewLayout(row).map((piece) => piece.apartDirection)).toEqual([
      { x: -1, y: 0 },
      { x: 0, y: 0 },
      { x: 1, y: 0 },
    ]);
  });
});
