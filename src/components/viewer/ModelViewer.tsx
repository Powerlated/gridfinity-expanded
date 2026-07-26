import { useEffect, useMemo, useRef, useState } from 'react';
import { Button, Text } from '@mantine/core';
import wasmUrl from '../../wasm/gridfinity_wasm_bg.wasm?url';
import { createViewer, initKernel } from '../../lib/geometry/kernel';
import type { Viewer } from '../../lib/geometry/kernel';
import { previewLayout } from '../../lib/preview';
import type { PreviewPiece } from '../../lib/preview';
import type { BadAppleClip, Bin, Design } from '../../lib/types';
import { binColor } from '../sidebar/binColors';

const NO_PARTS: PreviewPiece[] = [];
const CLEAR_COLOR = 0x1c1c21;
const BAD_APPLE_COLOR = 0xe9e9f2;
const DEFAULT_CAMERA_YAW = 0.9;
const FACE_ORIENTATION = 'counter-clockwise';

function hexToRgb(hex: string): number {
  return Number.parseInt(hex.replace('#', ''), 16);
}

interface Props {
  bins: Bin[];
  design: Design | null;
  error: string | null;
  badApple?: Float32Array | null;
  badAppleBounds?: BadAppleClip['bounds'] | null;
  badAppleFrame?: number | null;
}

export function ModelViewer({
  bins,
  design,
  error,
  badApple = null,
  badAppleBounds = null,
  badAppleFrame = null,
}: Props) {
  const designParts = useMemo(() => previewLayout(bins, design), [bins, design]);
  const parts = badApple ? NO_PARTS : designParts;
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const viewerRef = useRef<Viewer | null>(null);
  const badAppleFramedRef = useRef(false);
  const [viewer, setViewer] = useState<Viewer | null>(null);
  const [kernelError, setKernelError] = useState<string | null>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    let disposed = false;
    let frame = 0;
    let observer: ResizeObserver | null = null;

    void initKernel(wasmUrl)
      .then((kernel) => {
        if (disposed) return;
        const created = createViewer(kernel, canvas, CLEAR_COLOR);
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
    if (!viewer || badApple) return;
    viewer.begin_scene();
    for (const part of parts) {
      viewer.add_piece(
        part.triangles,
        part.previewOffset.x,
        part.previewOffset.y,
        hexToRgb(binColor(part.binId)),
      );
    }
    viewer.commit_scene(true);
    badAppleFramedRef.current = false;
  }, [badApple, parts, viewer]);

  useEffect(() => {
    if (!viewer || !badApple) return;
    viewer.begin_scene();
    viewer.add_piece(badApple, 0, 0, BAD_APPLE_COLOR);
    const framed = badAppleFramedRef.current;
    viewer.commit_scene(!framed && !badAppleBounds);
    if (!framed) {
      badAppleFramedRef.current = true;
      if (badAppleBounds) {
        viewer.frame_bounds(
          Float32Array.from(badAppleBounds.min),
          Float32Array.from(badAppleBounds.max),
        );
      }
      viewer.look_down();
    }
  }, [badApple, badAppleBounds, viewer]);

  return (
    <div
      className="viewer"
      data-part-count={parts.length}
      data-coordinate-orientation="generation-y-mirrored"
      data-default-camera-yaw={DEFAULT_CAMERA_YAW.toFixed(4)}
      data-face-orientation={FACE_ORIENTATION}
      data-mesh-topology="flat-triangle-soup"
      data-renderer="rust-webgl2"
      data-badapple-frame={badAppleFrame ?? undefined}
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
