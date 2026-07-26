/// <reference lib="webworker" />
import wasmUrl from '../wasm/gridfinity_wasm_bg.wasm?url';
import { badAppleClip, badAppleFrame, initKernel } from '../lib/geometry/kernel';
import type { BadAppleRequest, BadAppleResponse, BadAppleClip } from '../lib/types';

const kernelReady = initKernel(wasmUrl);

void kernelReady.then((kernel) => {
  const clip: BadAppleClip = badAppleClip(kernel);
  self.postMessage({ ready: true, clip });
});

self.onmessage = async (event: MessageEvent<BadAppleRequest>) => {
  const { frame } = event.data;
  try {
    const kernel = await kernelReady;
    const triangles = badAppleFrame(kernel, frame);
    const response: BadAppleResponse = { ok: true, frame, triangles };
    self.postMessage(response, [triangles.buffer as ArrayBuffer]);
  } catch {
    const response: BadAppleResponse = { ok: false, frame };
    self.postMessage(response);
  }
};
