import { useEffect, useRef, useState } from 'react';
import type { BadAppleClip, BadAppleRequest, BadAppleResponse } from '../lib/types';

const HASH = '#badapple';
const MAX_POOL_SIZE = 4;
const MAX_LEAD_FRAMES = 8;

export interface BadAppleState {
  active: boolean;
  clip: BadAppleClip | null;
  frame: number;
  triangles: Float32Array | null;
}

function hashIsBadApple(): boolean {
  return typeof window !== 'undefined' && window.location.hash.toLowerCase() === HASH;
}

function poolSize(): number {
  return Math.min(MAX_POOL_SIZE, Math.max(1, (navigator.hardwareConcurrency ?? 2) - 1));
}

export function useBadApple(): BadAppleState {
  const [active, setActive] = useState(hashIsBadApple);
  const [clip, setClip] = useState<BadAppleClip | null>(null);
  const [rendered, setRendered] = useState<{ frame: number; triangles: Float32Array } | null>(null);
  const clipRef = useRef<BadAppleClip | null>(null);

  useEffect(() => {
    const onHashChange = () => setActive(hashIsBadApple());
    window.addEventListener('hashchange', onHashChange);
    return () => window.removeEventListener('hashchange', onHashChange);
  }, []);

  useEffect(() => {
    if (!active) {
      setClip(null);
      setRendered(null);
      clipRef.current = null;
      return;
    }

    let disposed = false;
    const ready = new Map<number, Float32Array>();
    const idle: Worker[] = [];
    let nextFrame = 0;
    let displayed = -1;
    let startedAt = 0;
    let raf = 0;

    const workers = Array.from({ length: poolSize() }, () => {
      const worker = new Worker(
        new URL('../workers/badapple.worker.ts', import.meta.url),
        { type: 'module' },
      );
      worker.onmessage = (event: MessageEvent<BadAppleResponse | { ready: true; clip: BadAppleClip }>) => {
        if (disposed) return;
        const data = event.data;
        if ('ready' in data) {
          if (!clipRef.current) {
            clipRef.current = data.clip;
            setClip(data.clip);
            startedAt = performance.now();
          }
          idle.push(worker);
          pump();
          return;
        }
        if (data.ok) ready.set(data.frame, data.triangles);
        idle.push(worker);
        pump();
      };
      worker.onerror = () => {
        if (!disposed) idle.push(worker);
      };
      return worker;
    });

    const pump = () => {
      const info = clipRef.current;
      if (!info || disposed) return;
      while (
        idle.length > 0
        && nextFrame < info.frameCount
        && nextFrame - displayed < MAX_LEAD_FRAMES
      ) {
        const worker = idle.pop()!;
        const request: BadAppleRequest = { frame: nextFrame };
        worker.postMessage(request);
        nextFrame += 1;
      }
    };

    const tick = () => {
      raf = requestAnimationFrame(tick);
      const info = clipRef.current;
      if (!info || ready.size === 0) return;

      const elapsed = (performance.now() - startedAt) / 1000;
      const target = Math.floor(elapsed * info.fps);
      if (target >= info.frameCount) {
        startedAt = performance.now();
        nextFrame = 0;
        displayed = -1;
        ready.clear();
        pump();
        return;
      }

      let best = -1;
      for (const frame of ready.keys()) {
        if (frame <= target && frame > best) best = frame;
      }
      if (best < 0) {
        best = Math.min(...ready.keys());
      }

      const triangles = ready.get(best)!;
      for (const frame of [...ready.keys()]) {
        if (frame <= best) ready.delete(frame);
      }
      displayed = best;
      setRendered({ frame: best, triangles });
      pump();
    };
    raf = requestAnimationFrame(tick);

    return () => {
      disposed = true;
      cancelAnimationFrame(raf);
      workers.forEach((worker) => worker.terminate());
    };
  }, [active]);

  return {
    active,
    clip,
    frame: rendered?.frame ?? 0,
    triangles: rendered?.triangles ?? null,
  };
}
