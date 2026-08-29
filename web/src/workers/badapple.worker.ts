/// <reference lib="webworker" />
import wasmUrl from '../wasm/gridfinity_wasm_bg.wasm?url';
import { badAppleClip, badAppleFrameVertices, initKernel } from '../lib/geometry/kernel';
import { BAD_APPLE_COLOR } from '../lib/badApple';
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
    const vertices = badAppleFrameVertices(kernel, frame, BAD_APPLE_COLOR);
    const response: BadAppleResponse = { ok: true, frame, vertices };
    self.postMessage(response, [vertices.buffer as ArrayBuffer]);
  } catch {
    const response: BadAppleResponse = { ok: false, frame };
    self.postMessage(response);
  }
};
