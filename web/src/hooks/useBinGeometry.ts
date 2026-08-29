import { useEffect, useMemo, useRef, useState } from 'react';
import { buildBinParameters } from '../lib/binParameters';
import {
  geometryCacheKey,
  readCachedBin,
  writeCachedBin,
} from '../lib/geometryCache';
import type {
  Bin,
  BinParameters,
  Design,
  GenerateGeometryRequest,
  GenerateGeometryResponse,
} from '../lib/types';

const MAX_POOL_SIZE = 4;

export interface GeometryState {
  bins: Bin[];
  /** The design snapshot that produced `bins`, for viewer layout. */
  design: Design | null;
  generating: boolean;
  error: string | null;
}

interface PendingGeneration {
  revision: number;
  design: Design;
  /** Posted bin order; gathered results are reassembled to match. */
  binIds: string[];
  binsById: Map<string, Bin>;
  cacheKeysByBinId: Map<string, string>;
  remaining: number;
}

function poolSize(): number {
  return Math.min(MAX_POOL_SIZE, Math.max(1, (navigator.hardwareConcurrency ?? 2) - 1));
}

export function useBinGeometry(design: Design): GeometryState {
  const [state, setState] = useState<GeometryState>({
    bins: [],
    design: null,
    generating: false,
    error: null,
  });
  const parameters = useMemo(() => buildBinParameters(design), [design]);
  const workersRef = useRef<Worker[]>([]);
  const revisionRef = useRef(0);
  const pendingRef = useRef<PendingGeneration | null>(null);
  const latestRef = useRef({ design, parameters });
  const runningRef = useRef(false);
  const dirtyRef = useRef(false);
  const startRef = useRef<() => void>(() => undefined);
  const settleRef = useRef<() => void>(() => undefined);
  latestRef.current = { design, parameters };
  settleRef.current = () => {
    runningRef.current = false;
    if (!dirtyRef.current) return;
    dirtyRef.current = false;
    startRef.current();
  };

  useEffect(() => {
    const settle = () => settleRef.current();
    const fail = () => {
      pendingRef.current = null;
      setState((current) => ({
        ...current,
        generating: false,
        error: 'Geometry generation failed.',
      }));
      settle();
    };
    const workers = Array.from({ length: poolSize() }, () => {
      const worker = new Worker(
        new URL('../workers/geometry.worker.ts', import.meta.url),
        { type: 'module' },
      );
      worker.onmessage = (event: MessageEvent<GenerateGeometryResponse>) => {
        const response = event.data;
        const pending = pendingRef.current;
        if (response.revision !== revisionRef.current || pending?.revision !== response.revision) {
          return;
        }
        if (!response.ok) {
          fail();
          return;
        }
        for (const bin of response.bins) {
          pending.binsById.set(bin.binId, bin);
          const cacheKey = pending.cacheKeysByBinId.get(bin.binId);
          if (cacheKey) void writeCachedBin(cacheKey, bin).catch(() => undefined);
        }
        pending.remaining -= 1;
        if (pending.remaining > 0) return;
        pendingRef.current = null;
        setState({
          bins: pending.binIds.map((binId) => pending.binsById.get(binId)!),
          design: pending.design,
          generating: false,
          error: null,
        });
        settle();
      };
      worker.onerror = fail;
      return worker;
    });
    workersRef.current = workers;
    return () => {
      revisionRef.current += 1;
      workers.forEach((worker) => worker.terminate());
    };
  }, []);

  startRef.current = () => {
    const { design: current, parameters: currentParameters } = latestRef.current;
    const revision = ++revisionRef.current;
    runningRef.current = true;
    void (async () => {
      setState((previous) => ({ ...previous, generating: true, error: null }));
      if (currentParameters.length === 0) {
        pendingRef.current = null;
        setState({ bins: [], design: current, generating: false, error: null });
        settleRef.current();
        return;
      }

      const lookups = await Promise.all(currentParameters.map(async (bin) => {
        try {
          const key = await geometryCacheKey(bin);
          return { bin, key, cached: await readCachedBin(key, bin.binId) };
        } catch {
          return { bin, key: null, cached: null };
        }
      }));
      if (revision !== revisionRef.current) {
        settleRef.current();
        return;
      }

      const binsById = new Map<string, Bin>();
      const cacheKeysByBinId = new Map<string, string>();
      const missing: BinParameters[] = [];
      for (const lookup of lookups) {
        if (lookup.key) cacheKeysByBinId.set(lookup.bin.binId, lookup.key);
        if (lookup.cached) binsById.set(lookup.bin.binId, lookup.cached);
        else missing.push(lookup.bin);
      }

      if (missing.length === 0) {
        pendingRef.current = null;
        setState({
          bins: currentParameters.map((bin) => binsById.get(bin.binId)!),
          design: current,
          generating: false,
          error: null,
        });
        settleRef.current();
        return;
      }

      pendingRef.current = {
        revision,
        design: current,
        binIds: currentParameters.map((bin) => bin.binId),
        binsById,
        cacheKeysByBinId,
        remaining: missing.length,
      };
      const workers = workersRef.current;
      missing.forEach((bin, index) => {
        const request: GenerateGeometryRequest = { revision, bins: [bin] };
        workers[index % workers.length]?.postMessage(request);
      });
    })();
  };

  useEffect(() => {
    if (runningRef.current) {
      dirtyRef.current = true;
      return;
    }
    startRef.current();
  }, [design, parameters]);

  return state;
}
