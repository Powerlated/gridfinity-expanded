/** Production-path manifold and serialized-STL printability gate. */
import { buildBinParameters } from '../src/lib/binParameters';
import { trianglesToStl } from '../src/lib/export/stl';
import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';
import { generateGeometry, initKernel } from '../src/lib/geometry/kernel';
import { previewLayout } from '../src/lib/preview';
import { cutsForPrinter } from '../src/lib/printers';
import {
  GRIDFINITY_SPEC,
  gridfinityHeight,
  maximumFilletRadius,
} from '../src/lib/gridfinitySpec';
import type { BinDesign, Cell, Design, Edge, Wall } from '../src/lib/types';

interface Report {
  triangles: number;
  degenerate: number;
  boundaryEdges: number;
  nonManifoldEdges: number;
  orientationErrors: number;
  duplicateFaces: number;
  coincidentFaces: number;
  membranes: number;
}

function analyzeTriangleSoup(positions: Float32Array): Report {
  const indices = new Uint32Array(positions.length / 3);
  const vertices = new Map<string, number>();
  const vertexKeyAt = (vertex: number) => [0, 1, 2]
    .map((axis) => positions[vertex * 3 + axis])
    .join(',');
  for (let vertex = 0; vertex < indices.length; vertex++) {
    const key = vertexKeyAt(vertex);
    let id = vertices.get(key);
    if (id === undefined) {
      id = vertices.size;
      vertices.set(key, id);
    }
    indices[vertex] = id;
  }
  const undirected = new Map<string, number>();
  const directed = new Map<string, number>();
  const indexedFaces = new Map<string, number>();
  const geometricFaces = new Map<string, number>();
  let degenerate = 0;
  for (let index = 0; index < indices.length; index += 3) {
    const face = [indices[index], indices[index + 1], indices[index + 2]];
    const [a, b, c] = [index, index + 1, index + 2].map((vertex) => vertex * 3);
    const ux = positions[b] - positions[a];
    const uy = positions[b + 1] - positions[a + 1];
    const uz = positions[b + 2] - positions[a + 2];
    const vx = positions[c] - positions[a];
    const vy = positions[c + 1] - positions[a + 1];
    const vz = positions[c + 2] - positions[a + 2];
    const area = Math.hypot(
      uy * vz - uz * vy,
      uz * vx - ux * vz,
      ux * vy - uy * vx,
    ) / 2;
    if (new Set(face).size < 3 || area === 0) degenerate++;

    const indexedKey = [...face].sort((left, right) => left - right).join(',');
    indexedFaces.set(indexedKey, (indexedFaces.get(indexedKey) ?? 0) + 1);
    const geometricKey = [index, index + 1, index + 2]
      .map(vertexKeyAt).sort().join('|');
    geometricFaces.set(geometricKey, (geometricFaces.get(geometricKey) ?? 0) + 1);

    for (let edge = 0; edge < 3; edge++) {
      const from = face[edge];
      const to = face[(edge + 1) % 3];
      const key = from < to ? `${from}|${to}` : `${to}|${from}`;
      undirected.set(key, (undirected.get(key) ?? 0) + 1);
      directed.set(`${from}>${to}`, (directed.get(`${from}>${to}`) ?? 0) + 1);
    }
  }

  let boundaryEdges = 0;
  let nonManifoldEdges = 0;
  let orientationErrors = 0;
  for (const [edge, count] of undirected) {
    if (count < 2) {
      boundaryEdges++;
      continue;
    }
    if (count > 2) {
      nonManifoldEdges++;
      continue;
    }
    const [a, b] = edge.split('|');
    if ((directed.get(`${a}>${b}`) ?? 0) !== 1 ||
        (directed.get(`${b}>${a}`) ?? 0) !== 1) orientationErrors++;
  }
  const duplicates = (faces: Map<string, number>) =>
    [...faces.values()].reduce((total, count) => total + Math.max(0, count - 1), 0);
  return {
    triangles: indices.length / 3,
    degenerate,
    boundaryEdges,
    nonManifoldEdges,
    orientationErrors,
    duplicateFaces: duplicates(indexedFaces),
    coincidentFaces: duplicates(geometricFaces),
    membranes: countMembranes(positions),
  };
}

/**
 * Detect opposite-facing coplanar coverage that forms a zero-thickness sheet.
 * Edge pairing alone cannot see these internal membranes.
 */
function countMembranes(positions: Float32Array): number {
  const indices = Uint32Array.from({ length: positions.length / 3 }, (_, index) => index);
  type Triangle2 = [number, number, number, number, number, number];
  const planes = new Map<string, [Triangle2[], Triangle2[]]>();

  for (let index = 0; index < indices.length; index += 3) {
    const [p, q, r] = [
      indices[index] * 3,
      indices[index + 1] * 3,
      indices[index + 2] * 3,
    ];
    const ux = positions[q] - positions[p];
    const uy = positions[q + 1] - positions[p + 1];
    const uz = positions[q + 2] - positions[p + 2];
    const vx = positions[r] - positions[p];
    const vy = positions[r + 1] - positions[p + 1];
    const vz = positions[r + 2] - positions[p + 2];
    let nx = uy * vz - uz * vy;
    let ny = uz * vx - ux * vz;
    let nz = ux * vy - uy * vx;
    const length = Math.hypot(nx, ny, nz);
    if (length < 1e-9) continue;
    nx /= length;
    ny /= length;
    nz /= length;
    const flip = nx < -1e-6 ||
      (nx <= 1e-6 && (ny < -1e-6 || (ny <= 1e-6 && nz < 0)));
    const sign = flip ? -1 : 1;
    const distance = nx * positions[p] + ny * positions[p + 1] + nz * positions[p + 2];
    const key = [nx, ny, nz, distance]
      .map((value) => Math.round(sign * value * 1e3))
      .join(',');
    const dominant = Math.abs(nx) >= Math.abs(ny)
      ? (Math.abs(nx) >= Math.abs(nz) ? 0 : 2)
      : (Math.abs(ny) >= Math.abs(nz) ? 1 : 2);
    const [axis1, axis2] = dominant === 0 ? [1, 2] : dominant === 1 ? [0, 2] : [0, 1];
    let pair = planes.get(key);
    if (!pair) {
      pair = [[], []];
      planes.set(key, pair);
    }
    pair[flip ? 1 : 0].push([
      positions[p + axis1], positions[p + axis2],
      positions[q + axis1], positions[q + axis2],
      positions[r + axis1], positions[r + axis2],
    ]);
  }

  const containsInterior = (
    triangle: Triangle2,
    x: number,
    y: number,
    margin: number,
  ): boolean => {
    let sign = 0;
    for (let edge = 0; edge < 3; edge++) {
      const x1 = triangle[edge * 2 % 6];
      const y1 = triangle[(edge * 2 + 1) % 6];
      const x2 = triangle[(edge * 2 + 2) % 6];
      const y2 = triangle[(edge * 2 + 3) % 6];
      const edgeLength = Math.hypot(x2 - x1, y2 - y1);
      if (edgeLength < 1e-12) return false;
      const distance = ((x2 - x1) * (y - y1) - (y2 - y1) * (x - x1)) / edgeLength;
      if (Math.abs(distance) < margin) return false;
      if (sign === 0) sign = Math.sign(distance);
      else if (Math.sign(distance) !== sign) return false;
    }
    return true;
  };

  let membranes = 0;
  for (const [front, back] of planes.values()) {
    if (front.length === 0 || back.length === 0) continue;
    const [probe, cover] = front.length <= back.length ? [front, back] : [back, front];
    if (probe.some((triangle) => {
      const x = (triangle[0] + triangle[2] + triangle[4]) / 3;
      const y = (triangle[1] + triangle[3] + triangle[5]) / 3;
      return cover.some((candidate) => containsInterior(candidate, x, y, 1e-3));
    })) membranes++;
  }
  return membranes;
}

function stlBoundary(buffer: ArrayBuffer): { boundary: number; nonManifold: number } {
  const view = new DataView(buffer);
  const triangleCount = view.getUint32(80, true);
  const pointKey = (x: number, y: number, z: number) =>
    `${x},${y},${z}`;
  const edges = new Map<string, number>();
  let offset = 84;
  for (let triangle = 0; triangle < triangleCount; triangle++) {
    offset += 12;
    const read = () => {
      const value = view.getFloat32(offset, true);
      offset += 4;
      return value;
    };
    const points = [
      pointKey(read(), read(), read()),
      pointKey(read(), read(), read()),
      pointKey(read(), read(), read()),
    ];
    offset += 2;
    for (let edge = 0; edge < 3; edge++) {
      const from = points[edge];
      const to = points[(edge + 1) % 3];
      const key = from < to ? `${from}|${to}` : `${to}|${from}`;
      edges.set(key, (edges.get(key) ?? 0) + 1);
    }
  }
  let boundary = 0;
  let nonManifold = 0;
  for (const count of edges.values()) {
    if (count < 2) boundary++;
    else if (count > 2) nonManifold++;
  }
  return { boundary, nonManifold };
}

function rectangle(width: number, depth: number, offsetX = 0, offsetY = 0): Cell[] {
  return Array.from({ length: depth }, (_, y) =>
    Array.from({ length: width }, (_, x) => ({ x: x + offsetX, y: y + offsetY })),
  ).flat();
}

const irregular: Cell[] = [
  { x: 0, y: 0 }, { x: 1, y: 0 }, { x: 2, y: 0 },
  { x: 0, y: 1 }, { x: 1, y: 1 }, { x: 0, y: 2 },
];
const lShape: Cell[] = [
  { x: 0, y: 0 }, { x: 1, y: 0 }, { x: 0, y: 1 },
];
const uShape: Cell[] = [
  { x: 0, y: 0 }, { x: 1, y: 0 }, { x: 2, y: 0 },
  { x: 0, y: 1 }, { x: 2, y: 1 },
  { x: 0, y: 2 }, { x: 2, y: 2 },
];
const ring: Cell[] = [
  { x: 0, y: 0 }, { x: 1, y: 0 }, { x: 2, y: 0 },
  { x: 0, y: 1 }, { x: 2, y: 1 },
  { x: 0, y: 2 }, { x: 1, y: 2 }, { x: 2, y: 2 },
];
const h = (x: number, y: number): Edge => ({ orientation: 'h', x, y });
const v = (x: number, y: number): Edge => ({ orientation: 'v', x, y });
const wall = (x1: number, y1: number, x2: number, y2: number, width = 1.6): Wall => ({
  start: { x: x1, y: y1 },
  end: { x: x2, y: y2 },
  width,
});
const bin = (
  id: string,
  cells: Cell[],
  patch: Partial<BinDesign> = {},
): BinDesign => ({ id, cells, openings: [], walls: [], cuts: [], ...patch });

const printer = { name: 'Validation bed', bedWidth: 300, bedDepth: 300 };
const baseDesign: Omit<Design, 'bins'> = {
  heightUnits: 3,
  perimeterThickness: 1.2,
  filletRadius: 2.8,
  fasteners: { magnets: false, m3: false },
  printer,
};
const design = (bins: BinDesign[], patch: Partial<Design> = {}): Design => ({
  ...baseDesign,
  bins,
  ...patch,
});

const smallPrinter = { name: 'Small validation bed', bedWidth: 100, bedDepth: 100 };
const wideCells = rectangle(6, 1);
const wideCuts = cutsForPrinter(wideCells, smallPrinter);
const ringCuts = cutsForPrinter(ring, smallPrinter);

const cases: {
  name: string;
  value: Design;
  expectedParts?: number;
  expectsInteriorTop?: boolean;
}[] = [
  { name: '1x1 default', value: design([bin('bin-1', rectangle(1, 1))]) },
  { name: 'valid irregular', value: design([bin('bin-1', irregular)]) },
  { name: 'concave perimeter junction', value: design([bin('bin-1', lShape)]) },
  { name: 'valid U-shape', value: design([bin('bin-1', uShape)]) },
  { name: 'valid ring with hole', value: design([bin('bin-1', ring)]) },
  { name: 'outer opening', value: design([bin('bin-1', rectangle(2, 2), { openings: [h(0, 0)] })]) },
  { name: 'hole opening', value: design([bin('bin-1', ring, { openings: [v(1, 1)] })]) },
  { name: 'full-height wall', value: design([bin('bin-1', rectangle(2, 2), {
    walls: [wall(6, 42, 78, 42, 2)],
  })]), expectsInteriorTop: true },
  { name: 'T-shaped wall junction', value: design([bin('bin-1', rectangle(2, 2), {
    walls: [wall(5, 42, 79, 42, 2), wall(42, 5, 42, 42, 2)],
  })]), expectsInteriorTop: true },
  { name: 'cross-shaped wall junction', value: design([bin('bin-1', rectangle(2, 2), {
    walls: [wall(5, 21, 79, 21), wall(42, 5, 42, 79, 2)],
  })]), expectsInteriorTop: true },
  { name: 'magnet and M3 recesses', value: design([bin('bin-1', rectangle(2, 2))], {
    fasteners: { magnets: true, m3: true },
  }) },
  { name: 'zero shared fillet', value: design([bin('bin-1', irregular)], { filletRadius: 0 }) },
  { name: 'large shared fillet', value: design([bin('bin-1', ring)], { filletRadius: 5 }) },
  { name: 'minimum height, slider-max fillet', value: design([bin('bin-1', uShape)], {
    heightUnits: 2,
    filletRadius: maximumFilletRadius(gridfinityHeight(2)),
  }) },
  { name: 'multiple bins', value: design([
    bin('bin-1', rectangle(1, 2)),
    bin('bin-2', rectangle(1, 2, 2, 0)),
  ]), expectedParts: 2 },
  { name: 'recursive cuts', value: design([
    bin('bin-1', wideCells, { cuts: wideCuts }),
  ], { printer: smallPrinter }), expectedParts: 4 },
  { name: 'ring cuts', value: design([
    bin('bin-1', ring, { cuts: ringCuts }),
  ], { printer: smallPrinter }) },
  { name: 'wall sliced after construction', value: design([
    bin('bin-1', wideCells, { cuts: wideCuts, walls: [wall(8, 21, 244, 21)] }),
  ], { printer: smallPrinter }), expectedParts: 4 },
];

// Node has no bundler asset URL, so hand the kernel its bytes directly. The
// path is resolved from the working directory rather than `import.meta.url`
// because this script runs bundled out of `node_modules/.cache/`.
const wasm = await initKernel(
  readFileSync(resolve(process.cwd(), 'src/wasm/gridfinity_wasm_bg.wasm')),
);
let failed = false;
for (const fixture of cases) {
  try {
    const bins = generateGeometry(wasm, buildBinParameters(fixture.value));
    const parts = bins.flatMap((generatedBin) => generatedBin.pieces);
    if (fixture.expectedParts !== undefined && parts.length !== fixture.expectedParts) {
      throw new Error(`expected ${fixture.expectedParts} parts, received ${parts.length}`);
    }
    const filletRadius = fixture.value.filletRadius;
    const height = gridfinityHeight(fixture.value.heightUnits);
    for (const [partIndex, part] of parts.entries()) {
      const report = analyzeTriangleSoup(part.triangles);
      const serialized = stlBoundary(trianglesToStl(part.triangles));
      let minZ = Number.POSITIVE_INFINITY;
      let maxZ = Number.NEGATIVE_INFINITY;
      let horizontalTransitionFaces = 0;
      let interiorTopFaces = 0;
      const floorZ = GRIDFINITY_SPEC.baseHeight + GRIDFINITY_SPEC.floorThickness;
      const transitionTop = floorZ + filletRadius;
      for (let index = 0; index < part.triangles.length; index += 9) {
        const z = [part.triangles[index + 2], part.triangles[index + 5], part.triangles[index + 8]];
        minZ = Math.min(minZ, ...z);
        maxZ = Math.max(maxZ, ...z);
        const xMid = (part.triangles[index] + part.triangles[index + 3] +
          part.triangles[index + 6]) / 3;
        const yMid = (part.triangles[index + 1] + part.triangles[index + 4] +
          part.triangles[index + 7]) / 3;
        if (z.every((value) => Math.abs(value - height) < 1e-4) &&
            xMid > 5 && xMid < 79 && yMid > 5 && yMid < 79) {
          interiorTopFaces++;
        }
        const zMid = (z[0] + z[1] + z[2]) / 3;
        if (zMid <= floorZ + 1e-4 || zMid >= transitionTop - 1e-4) continue;
        const [a, b, c] = [index, index + 3, index + 6];
        const ux = part.triangles[b] - part.triangles[a];
        const uy = part.triangles[b + 1] - part.triangles[a + 1];
        const uz = part.triangles[b + 2] - part.triangles[a + 2];
        const vx = part.triangles[c] - part.triangles[a];
        const vy = part.triangles[c + 1] - part.triangles[a + 1];
        const vz = part.triangles[c + 2] - part.triangles[a + 2];
        const nx = uy * vz - uz * vy;
        const ny = uz * vx - ux * vz;
        const nz = ux * vy - uy * vx;
        const normalLength = Math.hypot(nx, ny, nz);
        // Any near-horizontal facet inside the transition band is a terrace step;
        // a smooth 32-segment sweep never tilts its lowest band this close to flat.
        if (normalLength > 1e-9 && Math.abs(nz) / normalLength > 0.9999) {
          horizontalTransitionFaces++;
        }
      }
      const coordinateError = Math.abs(minZ) > 1e-4 ||
        Math.abs(maxZ - height) > 1e-3;
      const topologyDefective = report.triangles === 0 ||
        report.degenerate || report.boundaryEdges || report.nonManifoldEdges ||
        report.orientationErrors || report.duplicateFaces || report.coincidentFaces || report.membranes ||
        serialized.boundary || serialized.nonManifold || coordinateError || horizontalTransitionFaces;
      const semanticError = fixture.expectsInteriorTop && interiorTopFaces === 0;
      const defective = topologyDefective || semanticError;
      if (defective) failed = true;
      console.log(
        `${`${fixture.name} [part ${partIndex + 1}]`.padEnd(48)} tris=${String(report.triangles).padStart(6)} ` +
        `boundary=${report.boundaryEdges} nonManifold=${report.nonManifoldEdges} ` +
        `orient=${report.orientationErrors} degen=${report.degenerate} ` +
        `dup=${report.duplicateFaces} coincident=${report.coincidentFaces} membrane=${report.membranes} ` +
        `flatFillet=${horizontalTransitionFaces} interiorTop=${interiorTopFaces} ` +
        `stl(bnd=${serialized.boundary},nm=${serialized.nonManifold}) coords=${coordinateError ? 'bad' : 'global'} ` +
        (defective ? '✗ DEFECTIVE' : '✓ clean'),
      );
    }
  } catch (error) {
    failed = true;
    console.log(`${fixture.name.padEnd(48)} ERROR: ${String(error)}`);
  }
}

try {
  const orientationDesign = design([
    bin('top', [{ x: 0, y: 0 }]),
    bin('bottom', [{ x: 0, y: 2 }]),
  ]);
  const orientationParameters = buildBinParameters(orientationDesign);
  if (orientationParameters[0].cells[0].y !== 2 || orientationParameters[1].cells[0].y !== 0) {
    throw new Error('parameter boundary did not mirror the shared occupied Y extent');
  }
  const orientationBins = generateGeometry(wasm, orientationParameters);
  const minY = (triangles: Float32Array) => {
    let value = Number.POSITIVE_INFINITY;
    for (let index = 1; index < triangles.length; index += 3) value = Math.min(value, triangles[index]);
    return value;
  };
  if (!(minY(orientationBins[1].pieces[0].triangles) < minY(orientationBins[0].pieces[0].triangles))) {
    throw new Error('geometry did not preserve mirrored generation coordinates');
  }
  const cutDesign = design([bin('bin-1', wideCells, { cuts: wideCuts })], { printer: smallPrinter });
  const cutBins = generateGeometry(wasm, buildBinParameters(cutDesign));
  const previewPieces = previewLayout(cutBins, cutDesign);
  const previewXs = [...new Set(previewPieces.map((piece) => piece.previewOffset.x))].sort((a, b) => a - b);
  if (previewXs.length < 2 || Math.abs(previewXs[1] - previewXs[0] - 0.3) > 1e-6) {
    throw new Error('multipart preview transforms do not create a 0.3 mm gap');
  }
  console.log('mirrored coordinate/orientation semantics'.padEnd(48) + ' ✓ pass');
} catch (error) {
  failed = true;
  console.log(`mirrored coordinate/orientation semantics`.padEnd(48) + ` ERROR: ${String(error)}`);
}

console.log(failed
  ? '\nRESULT: FAIL — defective production output detected.'
  : '\nRESULT: PASS — every production part and serialized STL is watertight and print-ready.');
process.exit(failed ? 1 : 0);
