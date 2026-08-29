import { describe, expect, it } from 'vitest';
import { partFilename, toPrintableObjects } from './printableObjects';
import { verticesToStl } from './stl';

describe('STL export ownership', () => {
  it('derives filenames from stable bin ids and part indices', () => {
    expect(partFilename('bin-1', 1, 0, 1)).toBe('gridfinity-bin.stl');
    expect(partFilename('bin-2', 3, 2, 4)).toBe('gridfinity-bin-2-part-3-of-4.stl');
  });

  it('splits grouped bin pieces into distinct named printable objects', () => {
    const vertices = new Float32Array(18);
    const printables = toPrintableObjects([
      { binId: 'bin-1', pieces: [{ vertices, cells: [{ x: 0, y: 0 }] }] },
      { binId: 'bin-2', pieces: [
        { vertices, cells: [{ x: 2, y: 0 }] },
        { vertices, cells: [{ x: 3, y: 0 }] },
      ] },
    ]);
    expect(printables.map((printable) => printable.name)).toEqual([
      'gridfinity-bin-1.stl',
      'gridfinity-bin-2-part-1-of-2.stl',
      'gridfinity-bin-2-part-2-of-2.stl',
    ]);
    expect(printables[0].vertices).toBe(vertices);
  });

  it('serializes render vertices without indexing or coordinate transforms', () => {
    const vertices = new Float32Array([
      0, 0, 0, 0, 0, 1,
      2, 0, 0, 0, 0, 1,
      0, 3, 0, 0, 0, 1,
    ]);
    const buffer = verticesToStl(vertices);
    const view = new DataView(buffer);

    expect(view.getUint32(80, true)).toBe(1);
    expect(view.getFloat32(96, true)).toBe(0);
    expect(view.getFloat32(108, true)).toBe(2);
    expect(view.getFloat32(124, true)).toBe(3);
  });
});
