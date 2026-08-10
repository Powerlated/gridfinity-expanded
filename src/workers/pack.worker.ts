/// <reference lib="webworker" />
import { createPackSearch, type PackSearch } from '../lib/project/pack';
import type { PackRequest, PackResponse } from '../lib/types';

const CHUNK_ITERATIONS = 8;

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

function run(revision: number) {
  if (current?.revision !== revision) return;
  const { search } = current;
  let more: boolean;
  try {
    more = search.step(CHUNK_ITERATIONS);
  } catch {
    const response: PackResponse = { ok: false, revision, error: 'Layout optimization failed.' };
    current = null;
    self.postMessage(response);
    return;
  }
  post(revision, search, !more);
  if (!more) {
    current = null;
    return;
  }
  setTimeout(() => run(revision), 0);
}

self.onmessage = (event: MessageEvent<PackRequest>) => {
  const { revision, input } = event.data;
  try {
    current = { revision, search: createPackSearch(input) };
  } catch {
    const response: PackResponse = { ok: false, revision, error: 'Layout optimization failed.' };
    self.postMessage(response);
    return;
  }
  run(revision);
};
