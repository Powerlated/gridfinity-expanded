import { useEffect, useMemo, useRef, useState } from 'react';
import { Button, Text } from '@mantine/core';
import wasmUrl from '../../wasm/gridfinity_wasm_bg.wasm?url';
import { createViewer, initKernel } from '../../lib/geometry/kernel';
import type { Viewer } from '../../lib/geometry/kernel';
import { previewLayout } from '../../lib/preview';
import type { PreviewPiece } from '../../lib/preview';
import type { Bin, Design } from '../../lib/types';
import type { BadAppleFeed } from '../../hooks/useBadApple';
import { binColor } from '../sidebar/binColors';
import { RENDER_QUALITY_INDEX, renderQualityFromIndex, useAppStore } from '../../store';

const NO_PARTS: PreviewPiece[] = [];
const CLEAR_COLOR = 0x1c1c21;
const DEFAULT_CAMERA_YAW = 0.9;
const FACE_ORIENTATION = 'counter-clockwise';

function hexToRgb(hex: string): number {
  return Number.parseInt(hex.replace('#', ''), 16);
}

interface Props {
  bins: Bin[];
  design: Design | null;
  error: string | null;
  badApple?: BadAppleFeed | null;
}

export function ModelViewer({
  bins,
  design,
  error,
  badApple = null,
}: Props) {
  const designParts = useMemo(() => previewLayout(bins, design), [bins, design]);
  const active = badApple?.active ?? false;
  const parts = active ? NO_PARTS : designParts;
  const renderQuality = useAppStore((state) => state.renderQuality);
  const containerRef = useRef<HTMLDivElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const viewerRef = useRef<Viewer | null>(null);
  const feedRef = useRef<BadAppleFeed | null>(null);
  const badAppleFramedRef = useRef(false);
  const qualityRef = useRef(RENDER_QUALITY_INDEX[renderQuality]);
  const publishedQualityRef = useRef<string | null>(null);
  qualityRef.current = RENDER_QUALITY_INDEX[renderQuality];
  feedRef.current = active ? badApple : null;
  const [viewer, setViewer] = useState<Viewer | null>(null);
  const [kernelError, setKernelError] = useState<string | null>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    let disposed = false;
    let frame = 0;
    let observer: ResizeObserver | null = null;

    void initKernel(wasmUrl)
      .then((kernel) => createViewer(kernel, canvas, CLEAR_COLOR))
      .then((created) => {
        if (disposed) {
          created.destroy();
          return;
        }
        created.set_quality(qualityRef.current);
        viewerRef.current = created;
        setViewer(created);

        const resize = () => {
          const ratio = window.devicePixelRatio || 1;
          const width = Math.max(1, Math.round(canvas.clientWidth * ratio));
          const height = Math.max(1, Math.round(canvas.clientHeight * ratio));
          if (canvas.width !== width || canvas.height !== height) {
            canvas.width = width;
            canvas.height = height;
          }
          created.resize(width, height);
        };
        resize();
        observer = new ResizeObserver(resize);
        observer.observe(canvas);

        const tick = (time: number) => {
          frame = requestAnimationFrame(tick);
          const next = feedRef.current?.pending.current ?? null;
          if (next) {
            feedRef.current!.pending.current = null;
            created.upload_vertices(next.vertices);
            if (!badAppleFramedRef.current) {
              badAppleFramedRef.current = true;
              const clipBounds = feedRef.current?.clip?.bounds;
              if (clipBounds) {
                created.frame_bounds(
                  Float32Array.from(clipBounds.min),
                  Float32Array.from(clipBounds.max),
                );
              }
              created.look_down();
            }
            if (containerRef.current) {
              containerRef.current.dataset.badappleFrame = String(next.frame);
            }
          }
          const level = renderQualityFromIndex(created.quality());
          if (level !== publishedQualityRef.current && containerRef.current) {
            publishedQualityRef.current = level;
            containerRef.current.dataset.renderQuality = level;
          }
          created.render(time / 1000);
        };
        frame = requestAnimationFrame(tick);
      })
      .catch((cause: unknown) => {
        if (!disposed) setKernelError(String(cause));
      });

    return () => {
      disposed = true;
      cancelAnimationFrame(frame);
      observer?.disconnect();
      viewerRef.current?.destroy();
      viewerRef.current = null;
      setViewer(null);
    };
  }, []);

  useEffect(() => {
    viewer?.set_quality(RENDER_QUALITY_INDEX[renderQuality]);
  }, [renderQuality, viewer]);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || !viewer) return;

    let dragging = false;
    let lastX = 0;
    let lastY = 0;

    const onPointerDown = (event: PointerEvent) => {
      dragging = true;
      lastX = event.clientX;
      lastY = event.clientY;
      canvas.setPointerCapture(event.pointerId);
    };
    const onPointerMove = (event: PointerEvent) => {
      if (!dragging) return;
      viewer.orbit(event.clientX - lastX, event.clientY - lastY);
      lastX = event.clientX;
      lastY = event.clientY;
    };
    const onPointerUp = (event: PointerEvent) => {
      dragging = false;
      if (canvas.hasPointerCapture(event.pointerId)) canvas.releasePointerCapture(event.pointerId);
    };
    const onWheel = (event: WheelEvent) => {
      event.preventDefault();
      viewer.zoom(-event.deltaY);
    };

    canvas.addEventListener('pointerdown', onPointerDown);
    canvas.addEventListener('pointermove', onPointerMove);
    canvas.addEventListener('pointerup', onPointerUp);
    canvas.addEventListener('pointercancel', onPointerUp);
    canvas.addEventListener('wheel', onWheel, { passive: false });
    return () => {
      canvas.removeEventListener('pointerdown', onPointerDown);
      canvas.removeEventListener('pointermove', onPointerMove);
      canvas.removeEventListener('pointerup', onPointerUp);
      canvas.removeEventListener('pointercancel', onPointerUp);
      canvas.removeEventListener('wheel', onWheel);
    };
  }, [viewer]);

  useEffect(() => {
    if (!viewer || active) return;
    viewer.begin_scene();
    for (const part of parts) {
      viewer.add_piece(
        part.vertices,
        part.previewOffset.x,
        part.previewOffset.y,
        hexToRgb(binColor(part.binId)),
      );
    }
    viewer.commit_scene(true);
    badAppleFramedRef.current = false;
    delete containerRef.current?.dataset.badappleFrame;
  }, [active, parts, viewer]);

  return (
    <div
      ref={containerRef}
      className="viewer"
      data-part-count={parts.length}
      data-coordinate-orientation="generation-y-mirrored"
      data-default-camera-yaw={DEFAULT_CAMERA_YAW.toFixed(4)}
      data-face-orientation={FACE_ORIENTATION}
      data-mesh-topology="welded-vertex-normals"
      data-renderer="rust-webgl2"
      data-render-quality-mode={renderQuality}
      data-preview-offsets={parts.map((part) =>
        `${part.previewOffset.x.toFixed(2)},${part.previewOffset.y.toFixed(2)}`).join(';')}
    >
      <canvas ref={canvasRef} className="viewer-canvas" aria-label="3D bin preview" />
      <Button
        className="viewer-reset"
        size="compact-xs"
        variant="default"
        onClick={() => viewerRef.current?.reset_view()}
      >
        Reset view
      </Button>
      {(error || kernelError) && (
        <div className="viewer-overlay viewer-overlay--error">
          <Text size="sm" c="red">{error ?? kernelError}</Text>
        </div>
      )}
    </div>
  );
}
