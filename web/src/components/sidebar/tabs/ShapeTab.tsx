import { useRef, useState } from 'react';
import { Button, ColorSwatch, Group, NumberInput, Stack, Text } from '@mantine/core';
import { flattenBins } from '../../../lib/cuts';
import { cellKey } from '../../../lib/edges';
import { GRIDFINITY_SPEC } from '../../../lib/gridfinitySpec';
import { footprintCells } from '../../../lib/printers';
import { MAX_GRID, minGridSize, useAppStore } from '../../../store';
import { Hint, Label } from '../../ui/Field';
import { binColor } from '../binColors';

function cellGap(cols: number, rows: number): number {
  if (cols > 14 || rows > 14) return 1;
  return cols > 8 ? 2 : 4;
}

const GRID_SIZE_INPUT_WIDTH = 64;

export function ShapeTab() {
  const design = useAppStore((state) => state.design);
  const selectedBinId = useAppStore((state) => state.selectedBinId);
  const gridCols = useAppStore((state) => state.gridCols);
  const gridRows = useAppStore((state) => state.gridRows);
  const selectBin = useAppStore((state) => state.selectBin);
  const startNewBin = useAppStore((state) => state.startNewBin);
  const paintCell = useAppStore((state) => state.paintCell);
  const removeSelectedCell = useAppStore((state) => state.removeSelectedCell);
  const setGridSize = useAppStore((state) => state.setGridSize);
  const [paintMode, setPaintMode] = useState<'add' | 'remove' | null>(null);
  const paintBinId = useRef<string | null>(null);

  const cells = flattenBins(design.bins);
  const cellBin = new Map(cells.map((cell) => [cellKey(cell), cell.binId]));
  const selectedExists = design.bins.some((bin) => bin.id === selectedBinId);

  function handlePointerDown(x: number, y: number) {
    const cell = { x, y };
    if (cellBin.get(cellKey(cell)) === selectedBinId) {
      setPaintMode('remove');
      removeSelectedCell(cell);
    } else {
      setPaintMode('add');
      paintCell(cell);
      paintBinId.current = useAppStore.getState().selectedBinId;
    }
  }

  function handlePointerEnter(x: number, y: number) {
    const cell = { x, y };
    if (paintMode === 'add') paintCell(cell, paintBinId.current ?? undefined);
    if (paintMode === 'remove' && cellBin.get(cellKey(cell)) === selectedBinId) {
      removeSelectedCell(cell);
    }
  }

  function endPaint() {
    setPaintMode(null);
    paintBinId.current = null;
  }

  function cellFromEvent(event: React.PointerEvent): { x: number; y: number } | null {
    const element = (event.target as HTMLElement).closest<HTMLElement>('[data-cell]');
    return element
      ? { x: Number(element.dataset.x), y: Number(element.dataset.y) }
      : null;
  }

  const footprint = footprintCells(cells);
  const min = minGridSize(cells);

  return (
    <Stack
      className="no-select"
      gap="sm"
      onPointerUp={endPaint}
      onPointerLeave={endPaint}
      onPointerCancel={endPaint}
    >
      <Group gap="xs">
        <Label>Grid</Label>
        <NumberInput
          w={GRID_SIZE_INPUT_WIDTH}
          hideControls
          min={min.cols}
          max={MAX_GRID}
          value={gridCols}
          onChange={(value) => setGridSize(Number(value), gridRows)}
          aria-label="Grid columns"
        />
        <Text span>×</Text>
        <NumberInput
          w={GRID_SIZE_INPUT_WIDTH}
          hideControls
          min={min.rows}
          max={MAX_GRID}
          value={gridRows}
          onChange={(value) => setGridSize(gridCols, Number(value))}
          aria-label="Grid rows"
        />
        <Text span>cells</Text>
      </Group>

      <Group gap="xs">
        {design.bins.map((bin, index) => (
          <Button
            key={bin.id}
            size="xs"
            variant="default"
            onClick={() => selectBin(bin.id)}
            style={selectedBinId === bin.id ? { borderColor: binColor(bin.id) } : undefined}
            leftSection={<ColorSwatch color={binColor(bin.id)} size={10} withShadow={false} />}
            aria-pressed={selectedBinId === bin.id}
          >
            Bin {index + 1}
          </Button>
        ))}
        <Button
          size="xs"
          variant="default"
          onClick={startNewBin}
          style={!selectedExists ? { borderColor: binColor(selectedBinId) } : undefined}
          leftSection={!selectedExists
            ? <ColorSwatch color={binColor(selectedBinId)} size={10} withShadow={false} />
            : undefined}
          aria-pressed={!selectedExists}
        >
          + New
        </Button>
      </Group>

      <Hint>
        Edge-connected painting modifies the selected bin. Painting elsewhere starts
        and selects a new bin. A held paint gesture stays in the same bin until release.
        Shape changes reset affected openings, walls, and cuts, then seed required cuts
        again.
      </Hint>
      <div
        className="cell-grid"
        style={{
          gridTemplateColumns: `repeat(${gridCols}, 1fr)`,
          aspectRatio: `${gridCols} / ${gridRows}`,
          gap: cellGap(gridCols, gridRows),
        }}
        onPointerDown={(event) => {
          const cell = cellFromEvent(event);
          if (cell) handlePointerDown(cell.x, cell.y);
        }}
        onPointerOver={(event) => {
          if (!paintMode) return;
          const cell = cellFromEvent(event);
          if (cell) handlePointerEnter(cell.x, cell.y);
        }}
      >
        {Array.from({ length: gridRows }, (_, row) =>
          Array.from({ length: gridCols }, (_, col) => {
            const binId = cellBin.get(cellKey({ x: col, y: row }));
            const selected = binId !== undefined;
            return (
              <button
                key={`${col}-${row}`}
                data-cell
                data-x={col}
                data-y={row}
                className={selected ? 'cell is-on' : 'cell'}
                style={selected
                  ? { background: binColor(binId), borderColor: binColor(binId) }
                  : undefined}
                aria-label={`Cell ${col},${row}`}
                aria-pressed={selected}
              />
            );
          }),
        )}
      </div>
      <Stack gap={2}>
        <Text>
          {cells.length} cell{cells.length !== 1 ? 's' : ''}
          {design.bins.length > 1 ? ` in ${design.bins.length} bins` : ''}
        </Text>
        {cells.length > 0 && (
          <Text>
            {footprint.width * GRIDFINITY_SPEC.gridPitch} ×{' '}
            {footprint.depth * GRIDFINITY_SPEC.gridPitch} mm layout footprint
          </Text>
        )}
      </Stack>
    </Stack>
  );
}
