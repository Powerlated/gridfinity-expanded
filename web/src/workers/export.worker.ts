/// <reference lib="webworker" />
// Vite resolves this to the hashed, base-path-aware asset URL for the binary.
import wasmUrl from '../wasm/gridfinity_wasm_bg.wasm?url';
import { exportParasolid, initKernel } from '../lib/geometry/kernel';
import type { ExportParasolidRequest, ExportParasolidResponse } from '../lib/types';

const kernelReady = initKernel(wasmUrl);

self.onmessage = async (event: MessageEvent<ExportParasolidRequest>) => {
  try {
    const kernel = await kernelReady;
    const xt = exportParasolid(kernel, event.data.bins);
    const response: ExportParasolidResponse = { ok: true, xt };
    self.postMessage(response);
  } catch (error) {
    const response: ExportParasolidResponse = {
      ok: false,
      error: `Parasolid export failed: ${String(error)}`,
    };
    self.postMessage(response);
  }
};
