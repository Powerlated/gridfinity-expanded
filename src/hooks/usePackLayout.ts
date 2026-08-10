import { useCallback, useEffect, useRef, useState } from 'react';
import { useAppStore } from '../store';
import type { PackInput, PackRequest, PackResponse } from '../lib/types';

const PACK_FAILED = 'Layout optimization failed.';

export interface PackState {
  running: boolean;
  progress: number;
  error: string | null;
  optimize: (input: PackInput) => void;
  cancel: () => void;
}

export function usePackLayout(): PackState {
  const setLayout = useAppStore((state) => state.setLayout);
  const [running, setRunning] = useState(false);
  const [progress, setProgress] = useState(0);
  const [error, setError] = useState<string | null>(null);
  const workerRef = useRef<Worker | null>(null);
  const revisionRef = useRef(0);
  const spawnRef = useRef<() => Worker>(() => undefined as unknown as Worker);

  spawnRef.current = () => {
    const worker = new Worker(
      new URL('../workers/pack.worker.ts', import.meta.url),
      { type: 'module' },
    );
    worker.onmessage = (event: MessageEvent<PackResponse>) => {
      const response = event.data;
      if (response.revision !== revisionRef.current) return;
      if (!response.ok) {
        setRunning(false);
        setError(response.error);
        return;
      }
      setLayout(response.best);
      setProgress(response.progress);
      setRunning(!response.done);
    };
    worker.onerror = () => {
      setRunning(false);
      setError(PACK_FAILED);
    };
    workerRef.current = worker;
    return worker;
  };

  useEffect(() => {
    const worker = spawnRef.current();
    return () => {
      revisionRef.current += 1;
      workerRef.current = null;
      worker.terminate();
    };
  }, []);

  const optimize = useCallback((input: PackInput) => {
    const revision = ++revisionRef.current;
    setRunning(true);
    setProgress(0);
    setError(null);
    const request: PackRequest = { revision, input };
    workerRef.current?.postMessage(request);
  }, []);

  const cancel = useCallback(() => {
    revisionRef.current += 1;
    workerRef.current?.terminate();
    spawnRef.current();
    setRunning(false);
  }, []);

  return { running, progress, error, optimize, cancel };
}
