import { useEffect, useRef, useState } from 'react';
import { Group, Stack, Text } from '@mantine/core';
import { partsConnected, rectBottom, rectRight } from '../../lib/project/rects';
import type { Point2, ProjectObject, Rect } from '../../lib/types';
import { useAppStore } from '../../store';
import { binColor } from '../sidebar/binColors';
import { Hint } from '../ui/Field';
import {
  OBJECT_GRID_MM,
  OBJECT_PAD,
  OBJECT_UNITS_PER_MM,
  objectFieldMm,
  objectMmToSvg,
  objectPointerToMm,
} from './projectCoords';
import './project.css';

const MIN_PART_MM = 3;
const PART_SNAP_MM = 4;

function snapAxis(value: number, guides: number[], field: number): number {
  const clamped = Math.min(field, Math.max(0, value));
  let nearest = Math.round(clamped);
  let distance = Math.abs(clamped - nearest);
  for (const guide of guides) {
    const candidate = Math.abs(clamped - guide);
    if (candidate < distance && candidate <= PART_SNAP_MM) {
      nearest = guide;
      distance = candidate;
    }
  }
  return nearest;
}

function draftRect(start: Point2, end: Point2): Rect {
  return {
    x: Math.min(start.x, end.x),
    y: Math.min(start.y, end.y),
    width: Math.abs(end.x - start.x),
    depth: Math.abs(end.y - start.y),
  };
}

function partRectProps(part: Rect) {
  return {
    x: objectMmToSvg(part.x),
    y: objectMmToSvg(part.y),
    width: part.width * OBJECT_UNITS_PER_MM,
    height: part.depth * OBJECT_UNITS_PER_MM,
  };
}

export function ObjectCanvas({ object }: { object: ProjectObject }) {
  const addObjectPart = useAppStore((state) => state.addObjectPart);
  const removeObjectPart = useAppStore((state) => state.removeObjectPart);
  const svgRef = useRef<SVGSVGElement>(null);
  const startRef = useRef<Point2 | null>(null);
  const [draft, setDraft] = useState<Rect | null>(null);
  const [selectedPart, setSelectedPart] = useState<number | null>(null);

  const field = objectFieldMm(object.parts);
  const view = OBJECT_PAD * 2 + field * OBJECT_UNITS_PER_MM;
  const guidesX = [0, ...object.parts.flatMap((part) => [part.x, rectRight(part)])];
  const guidesY = [0, ...object.parts.flatMap((part) => [part.y, rectBottom(part)])];
  const lines = Math.floor(field / OBJECT_GRID_MM);
  const color = binColor(object.id);

  useEffect(() => {
    if (selectedPart == null) return;
    const onKeyDown = (event: KeyboardEvent) => {
      const target = event.target as HTMLElement;
      if (target.tagName === 'INPUT' || target.tagName === 'TEXTAREA' || target.isContentEditable) return;
      if (event.key === 'Delete' || event.key === 'Backspace') {
        removeObjectPart(selectedPart);
        setSelectedPart(null);
      } else if (event.key === 'Escape') {
        setSelectedPart(null);
      }
    };
    window.addEventListener('keydown', onKeyDown);
    return () => window.removeEventListener('keydown', onKeyDown);
  }, [removeObjectPart, selectedPart]);

  function snapped(event: React.PointerEvent): Point2 {
    const point = objectPointerToMm(svgRef.current!, event);
    return {
      x: snapAxis(point.x, guidesX, field),
      y: snapAxis(point.y, guidesY, field),
    };
  }

  function beginPart(event: React.PointerEvent<SVGSVGElement>) {
    const start = snapped(event);
    startRef.current = start;
    setDraft({ x: start.x, y: start.y, width: 0, depth: 0 });
    setSelectedPart(null);
    event.currentTarget.setPointerCapture(event.pointerId);
  }

  function movePart(event: React.PointerEvent<SVGSVGElement>) {
    if (!startRef.current) return;
    setDraft(draftRect(startRef.current, snapped(event)));
  }

  function finishPart() {
    startRef.current = null;
    if (!draft) return;
    setDraft(null);
    if (draft.width < MIN_PART_MM || draft.depth < MIN_PART_MM) return;
    if (object.parts.length > 0 && !partsConnected([...object.parts, draft])) return;
    addObjectPart(draft);
  }

  return (
    <Stack className="no-select" gap="sm">
      <Hint>
        Drag to add a box. An object is one or more connected boxes, so a new box must
        touch or overlap the ones already drawn. Click a box to select it, then press
        Delete to remove it.
      </Hint>
      <svg
        ref={svgRef}
        viewBox={`0 0 ${view} ${view}`}
        className="editor-svg project-svg project-svg--object"
        style={{ aspectRatio: '1 / 1' }}
        onPointerDown={beginPart}
        onPointerMove={movePart}
        onPointerUp={finishPart}
      >
        <rect
          className="object-field"
          x={OBJECT_PAD}
          y={OBJECT_PAD}
          width={field * OBJECT_UNITS_PER_MM}
          height={field * OBJECT_UNITS_PER_MM}
        />
        {Array.from({ length: lines + 1 }, (_, index) => {
          const at = objectMmToSvg(index * OBJECT_GRID_MM);
          return (
            <g key={`grid-${index}`}>
              <line className="object-grid" x1={at} y1={OBJECT_PAD} x2={at} y2={view - OBJECT_PAD} />
              <line className="object-grid" x1={OBJECT_PAD} y1={at} x2={view - OBJECT_PAD} y2={at} />
            </g>
          );
        })}
        {object.parts.map((part, index) => (
          <rect
            key={`part-${index}`}
            {...partRectProps(part)}
            className={`object-part${selectedPart === index ? ' object-part--selected' : ''}`}
            fill={color}
            onPointerDown={(event) => event.stopPropagation()}
            onClick={() => setSelectedPart(index)}
          />
        ))}
        {draft && (
          <rect {...partRectProps(draft)} className="object-part--draft" />
        )}
      </svg>
      <Group gap="md">
        <Text>{object.parts.length} box{object.parts.length === 1 ? '' : 'es'}</Text>
        <Text>{field} mm field, {OBJECT_GRID_MM} mm grid</Text>
      </Group>
    </Stack>
  );
}
