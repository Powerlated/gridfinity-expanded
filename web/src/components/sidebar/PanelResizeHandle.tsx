import { useRef } from 'react';
import { useAppStore, type PanelSide } from '../../store';

/**
 * Drag strip on a side panel's inner edge (the edge facing the viewer);
 * live-updates that panel's width in the store. The left sidebar grows when
 * dragged right; the right settings panel grows when dragged left.
 */
export function PanelResizeHandle({ panel }: { panel: PanelSide }) {
  const width = useAppStore((s) => s.panelWidths[panel]);
  const setPanelWidth = useAppStore((s) => s.setPanelWidth);
  const dragStart = useRef({ x: 0, width: 0 });
  const grow = panel === 'sidebar' ? 1 : -1;

  const onPointerDown = (e: React.PointerEvent<HTMLDivElement>) => {
    e.currentTarget.setPointerCapture(e.pointerId);
    dragStart.current = { x: e.clientX, width };
    document.body.classList.add('no-select');
  };

  const onPointerMove = (e: React.PointerEvent<HTMLDivElement>) => {
    if (e.buttons === 0) return;
    setPanelWidth(panel, dragStart.current.width + grow * (e.clientX - dragStart.current.x));
  };

  const onPointerUp = (e: React.PointerEvent<HTMLDivElement>) => {
    e.currentTarget.releasePointerCapture(e.pointerId);
    document.body.classList.remove('no-select');
  };

  return (
    <div
      className={`panel-resize-handle panel-resize-handle--${panel === 'sidebar' ? 'right' : 'left'}`}
      onPointerDown={onPointerDown}
      onPointerMove={onPointerMove}
      onPointerUp={onPointerUp}
    />
  );
}
