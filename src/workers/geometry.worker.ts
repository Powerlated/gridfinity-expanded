/// <reference lib="webworker" />
// Vite resolves this to the hashed, base-path-aware asset URL for the binary.
import wasmUrl from '../wasm/gridfinity_wasm_bg.wasm?url';
import { generateGeometry, initKernel } from '../lib/geometry/kernel';
import type { GenerateGeometryRequest, GenerateGeometryResponse } from '../lib/types';

const kernelReady = initKernel(wasmUrl);

self.onmessage = async (event: MessageEvent<GenerateGeometryRequest>) => {
  const { bins: parameters, revision } = event.data;
  try {
    const kernel = await kernelReady;
    const bins = generateGeometry(kernel, parameters);
    const response: GenerateGeometryResponse = { ok: true, revision, bins };
    const transfer = bins.flatMap((bin) =>
      bin.pieces.map((piece) => piece.triangles.buffer as ArrayBuffer));
    self.postMessage(response, transfer);
  } catch {
    const response: GenerateGeometryResponse = {
      ok: false,
      revision,
      error: 'Geometry generation failed.',
    };
    self.postMessage(response);
  }
};
