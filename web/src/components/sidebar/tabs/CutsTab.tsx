import { Button, CloseButton, ColorSwatch, Group, Paper, Stack, Text } from '@mantine/core';
import { availableCuts, cutKey, flattenBins, partitionCells } from '../../../lib/cuts';
import { checkDesignFit } from '../../../lib/printers';
import type { Cut } from '../../../lib/types';
import { useAppStore } from '../../../store';
import { Hint, Label } from '../../ui/Field';
import { StatusBanner } from '../../ui/StatusBanner';
import { binColor } from '../binColors';
import { EditorCanvas } from '../EditorCanvas';
import { gridToSvg } from '../editorCoords';

function cutPoints(cut: Cut) {
  return {
    x1: gridToSvg(cut.start.x),
    y1: gridToSvg(cut.start.y),
    x2: gridToSvg(cut.end.x),
    y2: gridToSvg(cut.end.y),
  };
}

export function CutsTab() {
  const design = useAppStore((state) => state.design);
  const selectedBinId = useAppStore((state) => state.selectedBinId);
  const gridCols = useAppStore((state) => state.gridCols);
  const gridRows = useAppStore((state) => state.gridRows);
  const toggle = useAppStore((state) => state.toggleCut);
  const move = useAppStore((state) => state.moveCut);
  const reset = useAppStore((state) => state.resetCuts);
  const cells = flattenBins(design.bins);
  const fit = checkDesignFit(design.bins, design.printer);
  const selectedBin = design.bins.find((bin) => bin.id === selectedBinId);

  if (cells.length === 0) return <Hint>Select cells in the Shape tab first.</Hint>;

  return (
    <Stack className="no-select" gap="sm">
      <Hint>
        Cuts are editable grid-edge segments. Click a faint candidate to add it,
        click an active cut to remove it, or use the controls below to move and reset cuts.
      </Hint>

      <Stack gap="xs">
        {design.bins.map((bin, index) => {
          const parts = partitionCells(bin.cells, bin.cuts);
          return (
            <Group key={bin.id} justify="space-between" wrap="nowrap">
              <Group gap="xs" wrap="nowrap">
                <ColorSwatch color={binColor(bin.id)} size={12} withShadow={false} />
                <Text size="sm">Bin {index + 1}</Text>
              </Group>
              <Text>{parts.length} part{parts.length !== 1 ? 's' : ''}</Text>
            </Group>
          );
        })}
      </Stack>

      <EditorCanvas gridCols={gridCols} gridRows={gridRows} cells={cells}>
        {design.bins.flatMap((bin) => {
          const active = new Map(bin.cuts.map((cut) => [cutKey(cut), cut]));
          const candidates = new Map(availableCuts(bin.cells).map((cut) => [cutKey(cut), cut]));
          for (const [key, cut] of active) candidates.set(key, cut);
          return [...candidates].map(([key, cut]) => {
            const isActive = active.has(key);
            return (
              <g
                key={`${bin.id}:${key}`}
                className="cut-line"
                onClick={() => toggle(bin.id, cut)}
              >
                <line {...cutPoints(cut)} className="cut-line-hit" />
                <line
                  {...cutPoints(cut)}
                  className={`cut-line-visible cut-line-visible--${isActive ? 'active' : 'inactive'}`}
                  style={isActive ? { stroke: binColor(bin.id) } : undefined}
                />
              </g>
            );
          });
        })}
      </EditorCanvas>

      <StatusBanner ok={fit.allFit}>
        {fit.allFit
          ? `${fit.parts} part${fit.parts !== 1 ? 's' : ''} — every part fits the ${design.printer.name} bed.`
          : `The largest part (${fit.worst.width} × ${fit.worst.depth} mm) exceeds the ${design.printer.name} bed.`}
      </StatusBanner>

      {selectedBin && (
        <Stack gap="xs">
          <Group justify="space-between">
            <Label>Selected bin cuts</Label>
            <Button variant="default" onClick={() => reset(selectedBin.id)}>
              Reset cuts
            </Button>
          </Group>
          {selectedBin.cuts.length === 0 ? (
            <Hint>No cuts are required for the selected printer.</Hint>
          ) : selectedBin.cuts.map((cut, index) => (
            <Paper key={cutKey(cut)} p={6} bg="dark.6">
              <Group gap="xs" wrap="nowrap">
                <Text flex={1} c="bright">Cut {index + 1}</Text>
                <Button
                  variant="default"
                  size="compact-xs"
                  aria-label={`Move cut ${index + 1} earlier`}
                  onClick={() => move(selectedBin.id, index, -1)}
                >
                  ←
                </Button>
                <Button
                  variant="default"
                  size="compact-xs"
                  aria-label={`Move cut ${index + 1} later`}
                  onClick={() => move(selectedBin.id, index, 1)}
                >
                  →
                </Button>
                <CloseButton
                  aria-label={`Remove cut ${index + 1}`}
                  onClick={() => toggle(selectedBin.id, cut)}
                />
              </Group>
            </Paper>
          ))}
        </Stack>
      )}
    </Stack>
  );
}
