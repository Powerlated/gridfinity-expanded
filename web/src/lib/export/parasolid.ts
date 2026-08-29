/**
 * Parasolid XT export: the on-demand worker round trip and the download.
 *
 * The export rebuilds every solid the preview already shows, so it runs off the
 * critical path in its own short-lived worker rather than the geometry
 * worker — that pool's leading-edge scheduling must not gain a second message
 * type. `exportParasolid()` spawns the worker, hands it complete
 * `BinParameters`, resolves the file text, and terminates it; the caller owns
 * nothing but the promise. `downloadParasolid()` then writes that text out as
 * one `.x_t`, a multi-body file holding every printable piece.
 */
import type { BinParameters, ExportParasolidResponse } from '../types';
import { downloadBuffer } from './stl';

export function exportParasolid(bins: BinParameters[]): Promise<string> {
  return new Promise((resolve, reject) => {
    const worker = new Worker(new URL('../../workers/export.worker.ts', import.meta.url), {
      type: 'module',
    });
    worker.onmessage = (event: MessageEvent<ExportParasolidResponse>) => {
      worker.terminate();
      if (event.data.ok) {
        resolve(event.data.xt);
      } else {
        reject(new Error(event.data.error));
      }
    };
    worker.onerror = (event) => {
      worker.terminate();
      reject(new Error(`Parasolid export worker failed: ${event.message}`));
    };
    worker.postMessage({ bins });
  });
}

export function downloadParasolid(xt: string, filename = 'gridfinity.x_t'): void {
  downloadBuffer(new TextEncoder().encode(xt).buffer as ArrayBuffer, filename, 'model/x_t');
}
