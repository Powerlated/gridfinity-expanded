/// <reference lib="webworker" />
// Vite resolves this to the hashed, base-path-aware asset URL for the binary.
import wasmUrl from '../wasm/gridfinity_wasm_bg.wasm?url';
import { createPackSearch, initKernel } from '../lib/geometry/kernel';
import type { PackSearch } from '../lib/geometry/kernel';
import type { PackRequest, PackResponse } from '../lib/types';

const CHUNK_ITERATIONS = 8;

const kernelReady = initKernel(wasmUrl);

let current: { revision: number; search: PackSearch } | null = null;

function post(revision: number, search: PackSearch, done: boolean) {
  const response: PackResponse = {
    ok: true,
    revision,
    done,
    progress: search.total === 0 ? 1 : search.done / search.total,
    best: search.result(),
  };
  self.postMessage(response);
}

function fail(revision: number) {
  const response: PackResponse = { ok: false, revision, error: 'Layout optimization failed.' };
  current = null;
  self.postMessage(response);
}

function run(revision: number) {
  if (current?.revision !== revision) return;
  const { search } = current;
  let more: boolean;
  try {
    more = search.step(CHUNK_ITERATIONS);
  } catch {
    fail(revision);
    return;
  }
  post(revision, search, !more);
  if (!more) {
    search.free();
    current = null;
    return;
  }
  setTimeout(() => run(revision), 0);
}

self.onmessage = async (event: MessageEvent<PackRequest>) => {
  const { revision, input } = event.data;
  try {
    const kernel = await kernelReady;
    current?.search.free();
    current = { revision, search: createPackSearch(kernel, input) };
  } catch {
    fail(revision);
    return;
  }
  run(revision);
};
