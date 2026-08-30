/// <reference lib="webworker" />
// Vite resolves this to the hashed, base-path-aware asset URL for the binary.
import wasmUrl from '../wasm/gridfinity_wasm_bg.wasm?url';
import { createPackSearch, initKernel } from '../lib/geometry/kernel';
import type { PackSearch } from '../lib/geometry/kernel';
import type { PackRequest, PackResponse } from '../lib/types';

/**
 * How many progress updates one search reports, whatever its budget.
 *
 * The chunk used to be a flat eight restarts, which was a fraction of the budget
 * when the budget was 200 and is a thousandth of it now. Every chunk costs a
 * macrotask the browser clamps and a whole `PackResult` posted back, so a fixed
 * chunk turns a bigger budget into scheduling rather than packing. A share of the
 * budget keeps the progress bar moving at the same rate it always did.
 */
const PROGRESS_UPDATES = 25;

/** The smallest chunk worth a round trip, for a budget too small to divide. */
const MIN_CHUNK_ITERATIONS = 8;

/** How many restarts to spend between progress reports for this search. */
function chunkFor(search: PackSearch): number {
  return Math.max(MIN_CHUNK_ITERATIONS, Math.ceil(search.total / PROGRESS_UPDATES));
}

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
    more = search.step(chunkFor(search));
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
