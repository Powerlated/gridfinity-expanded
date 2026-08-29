import { useEffect, useMemo, useRef, useState } from 'react';
import { Button, CloseButton, Group, NumberInput, Paper, Stack, Text } from '@mantine/core';
import { flattenBins } from '../../../lib/cuts';
import { cellKey, edgeKey, perimeterEdges } from '../../../lib/edges';
import { GRIDFINITY_SPEC } from '../../../lib/gridfinitySpec';
import type { Edge, Point2, Wall } from '../../../lib/types';
import { useAppStore } from '../../../store';
import { Hint, Label } from '../../ui/Field';
import { EditorCanvas } from '../EditorCanvas';
import { CELL, gridToSvg, mmToSvg, pointerToMm } from '../editorCoords';

const POINT_SNAP_MM = 5;
const MIN_WALL_LENGTH = 5;
const WALL_WIDTH_INPUT = 64;

function edgePoints(edge: Edge) {
  const x = gridToSvg(edge.x);
  const y = gridToSvg(edge.y);
  return edge.orientation === 'h'
    ? { x1: x, y1: y, x2: x + CELL, y2: y }
    : { x1: x, y1: y, x2: x, y2: y + CELL };
}

function snapHalfMillimetre(value: number): number {
  return Math.round(value * 2) / 2;
}

function snapPoint(point: Point2, walls: Wall[]): Point2 {
  const endpoints = walls.flatMap((wall) => [wall.start, wall.end]);
  let nearest: Point2 | null = null;
  let distance = POINT_SNAP_MM;
  for (const endpoint of endpoints) {
    const candidate = Math.hypot(point.x - endpoint.x, point.y - endpoint.y);
    if (candidate <= distance) {
      nearest = endpoint;
      distance = candidate;
    }
  }
  const pitch = GRIDFINITY_SPEC.gridPitch;
  const grid = {
    x: Math.round(point.x / pitch) * pitch,
    y: Math.round(point.y / pitch) * pitch,
  };
  const gridDistance = Math.hypot(point.x - grid.x, point.y - grid.y);
  if (gridDistance <= distance) nearest = grid;
  return nearest ?? { x: snapHalfMillimetre(point.x), y: snapHalfMillimetre(point.y) };
}

export function WallsTab() {
  const design = useAppStore((state) => state.design);
  const selectedBinId = useAppStore((state) => state.selectedBinId);
  const gridCols = useAppStore((state) => state.gridCols);
  const gridRows = useAppStore((state) => state.gridRows);
  const toggleOpening = useAppStore((state) => state.toggleOpening);
  const resetSelectedWalls = useAppStore((state) => state.resetSelectedWalls);
  const addWall = useAppStore((state) => state.addWall);
  const updateWall = useAppStore((state) => state.updateWall);
  const removeWall = useAppStore((state) => state.removeWall);
  const selectedBin = design.bins.find((bin) => bin.id === selectedBinId);
  const selectedWalls = selectedBin?.walls ?? [];
  const cells = flattenBins(design.bins);
  const svgRef = useRef<SVGSVGElement>(null);
  const [draft, setDraft] = useState<Wall | null>(null);
  const [selectedWall, setSelectedWall] = useState<number | null>(null);

  const selectedCellSet = useMemo(
    () => new Set((selectedBin?.cells ?? []).map(cellKey)),
    [selectedBin],
  );
  const perimeter = useMemo(() => {
    const values = new Map<string, Edge>();
    for (const bin of design.bins) {
      for (const edge of perimeterEdges(bin.cells)) values.set(edgeKey(edge), edge);
    }
    return [...values.values()];
  }, [design.bins]);
  const openKeys = useMemo(
    () => new Set(design.bins.flatMap((bin) => bin.openings.map(edgeKey))),
    [design.bins],
  );

  function insideSelected(point: Point2): boolean {
    const pitch = GRIDFINITY_SPEC.gridPitch;
    const epsilon = 0.01;
    for (const x of [
      Math.floor((point.x - epsilon) / pitch),
      Math.floor((point.x + epsilon) / pitch),
    ]) {
      for (const y of [
        Math.floor((point.y - epsilon) / pitch),
        Math.floor((point.y + epsilon) / pitch),
      ]) {
        if (selectedCellSet.has(`${x},${y}`)) return true;
      }
    }
    return false;
  }

  function clampEndpoint(start: Point2, end: Point2): Point2 {
    const length = Math.hypot(end.x - start.x, end.y - start.y);
    const steps = Math.max(1, Math.ceil(length / 0.5));
    let last = start;
    for (let index = 1; index <= steps; index++) {
      const amount = index / steps;
      const point = {
        x: start.x + (end.x - start.x) * amount,
        y: start.y + (end.y - start.y) * amount,
      };
      if (!insideSelected(point)) return last;
      last = point;
    }
    return end;
  }

  useEffect(() => {
    if (selectedWall == null) return;
    const onKeyDown = (event: KeyboardEvent) => {
      const target = event.target as HTMLElement;
      if (target.tagName === 'INPUT' || target.tagName === 'TEXTAREA' || target.isContentEditable) return;
      if (event.key === 'Delete' || event.key === 'Backspace') {
        removeWall(selectedWall);
        setSelectedWall(null);
      } else if (event.key === 'Escape') {
        setSelectedWall(null);
      }
    };
    window.addEventListener('keydown', onKeyDown);
    return () => window.removeEventListener('keydown', onKeyDown);
  }, [removeWall, selectedWall]);

  if (cells.length === 0) return <Hint>Select cells in the Shape tab first.</Hint>;
  if (!selectedBin) return <Hint>Select an existing bin in the Shape tab to edit its walls.</Hint>;

  function pointFromEvent(event: React.PointerEvent): Point2 {
    return pointerToMm(svgRef.current!, event);
  }

  function beginWall(event: React.PointerEvent<SVGSVGElement>) {
    const raw = pointFromEvent(event);
    if (!insideSelected(raw)) {
      setSelectedWall(null);
      return;
    }
    const start = snapPoint(raw, selectedWalls);
    setDraft({ start, end: start, width: 1.2 });
    setSelectedWall(null);
    event.currentTarget.setPointerCapture(event.pointerId);
  }

  function moveWall(event: React.PointerEvent<SVGSVGElement>) {
    if (!draft) return;
    const snapped = snapPoint(pointFromEvent(event), selectedWalls);
    setDraft({ ...draft, end: clampEndpoint(draft.start, snapped) });
  }

  function finishWall() {
    if (!draft) return;
    if (Math.hypot(draft.end.x - draft.start.x, draft.end.y - draft.start.y) >= MIN_WALL_LENGTH) {
      addWall(draft);
    }
    setDraft(null);
  }

  return (
    <Stack className="no-select" gap="sm">
      <Hint>
        Click a perimeter—including an enclosed-hole perimeter—to toggle an opening.
        Shared coincident edges update both adjacent bins. Drag inside the selected bin
        to add a straight full-height wall.
      </Hint>
      <EditorCanvas
        ref={svgRef}
        className="editor-svg editor-svg--walls"
        gridCols={gridCols}
        gridRows={gridRows}
        cells={cells}
        onPointerDown={beginWall}
        onPointerMove={moveWall}
        onPointerUp={finishWall}
      >
        {perimeter.map((edge) => {
          const open = openKeys.has(edgeKey(edge));
          return (
            <g
              key={edgeKey(edge)}
              className="edge"
              onPointerDown={(event) => event.stopPropagation()}
              onClick={() => toggleOpening(edge)}
            >
              <line {...edgePoints(edge)} stroke="transparent" strokeWidth={12} strokeLinecap="round" />
              <line
                {...edgePoints(edge)}
                className={`edge-line edge-line--${open ? 'open' : 'wall'}`}
              />
            </g>
          );
        })}
        {design.bins.flatMap((bin) => bin.walls.map((wall, index) => {
          const selected = bin.id === selectedBin.id && index === selectedWall;
          const points = {
            x1: mmToSvg(wall.start.x),
            y1: mmToSvg(wall.start.y),
            x2: mmToSvg(wall.end.x),
            y2: mmToSvg(wall.end.y),
          };
          return (
            <g
              key={`${bin.id}:wall-${index}`}
              className="custom-wall-g"
              onPointerDown={(event) => event.stopPropagation()}
              onClick={() => {
                if (bin.id === selectedBin.id) setSelectedWall(index);
              }}
            >
              <line {...points} className="custom-wall-hit" />
              <line
                {...points}
                className={`custom-wall${selected ? ' custom-wall--selected' : ''}`}
                strokeWidth={Math.max(2.5, wall.width / GRIDFINITY_SPEC.gridPitch * CELL)}
              />
            </g>
          );
        }))}
        {draft && (
          <g>
            <line
              x1={mmToSvg(draft.start.x)}
              y1={mmToSvg(draft.start.y)}
              x2={mmToSvg(draft.end.x)}
              y2={mmToSvg(draft.end.y)}
              className="custom-wall--draft"
            />
            <circle
              className="custom-wall-endpoint"
              cx={mmToSvg(draft.start.x)}
              cy={mmToSvg(draft.start.y)}
              r={4}
            />
            <circle
              className="custom-wall-endpoint"
              cx={mmToSvg(draft.end.x)}
              cy={mmToSvg(draft.end.y)}
              r={4}
            />
          </g>
        )}
      </EditorCanvas>

      <Group gap="md">
        <Group gap={4} wrap="nowrap">
          <span className="legend-swatch legend-swatch--wall" />
          <Text>perimeter</Text>
        </Group>
        <Group gap={4} wrap="nowrap">
          <span className="legend-swatch legend-swatch--open" />
          <Text>opening</Text>
        </Group>
        <Group gap={4} wrap="nowrap">
          <span className="legend-swatch legend-swatch--custom" />
          <Text>internal wall</Text>
        </Group>
      </Group>
      <Button
        variant="default"
        disabled={selectedBin.openings.length === 0 && selectedWalls.length === 0}
        onClick={resetSelectedWalls}
      >
        Reset selected bin walls
      </Button>

      {selectedWalls.length > 0 && (
        <Stack gap="xs">
          <Label>Internal walls</Label>
          {selectedWalls.map((wall, index) => (
            <Paper
              key={`wall-${index}`}
              p={6}
              bg="dark.6"
              className={selectedWall === index ? 'wall-row--selected' : undefined}
              onClick={() => setSelectedWall(index)}
            >
              <Group gap="xs" wrap="nowrap">
                <Text flex={1} c="bright">
                  #{index + 1} · {Math.hypot(
                    wall.end.x - wall.start.x,
                    wall.end.y - wall.start.y,
                  ).toFixed(0)} mm
                </Text>
                <Text>width</Text>
                <NumberInput
                  w={WALL_WIDTH_INPUT}
                  hideControls
                  min={0.4}
                  max={8}
                  step={0.2}
                  value={wall.width}
                  onChange={(value) => {
                    const width = typeof value === 'number' ? value : Number.parseFloat(value);
                    if (Number.isFinite(width)) updateWall(index, { width });
                  }}
                  aria-label={`Width of wall ${index + 1}`}
                />
                <CloseButton
                  aria-label={`Delete wall ${index + 1}`}
                  onClick={(event) => {
                    event.stopPropagation();
                    removeWall(index);
                    setSelectedWall(null);
                  }}
                />
              </Group>
            </Paper>
          ))}
          <Hint>Each wall reaches the flat top rim; widths are configured independently.</Hint>
        </Stack>
      )}
    </Stack>
  );
}
