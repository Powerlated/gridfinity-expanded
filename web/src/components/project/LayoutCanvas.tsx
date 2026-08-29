import { Group, Stack, Text } from '@mantine/core';
import { drawerCells, drawerGrid, packingArea } from '../../lib/project/drawer';
import { DRAWER_BIN_ID } from '../../lib/project/layout';
import { inflateParts, partsBounds } from '../../lib/project/rects';
import type { PackResult, Project, Rect } from '../../lib/types';
import { MAX_GRID } from '../../store';
import { EditorCanvas } from '../sidebar/EditorCanvas';
import { binColor } from '../sidebar/binColors';
import { mmSpanToSvg, mmToSvg } from '../sidebar/editorCoords';
import { Hint } from '../ui/Field';
import './project.css';

interface LayoutCanvasProps {
  project: Project;
  layout: PackResult | null;
  perimeterThickness: number;
}

function rectProps(rect: Rect) {
  return {
    x: mmToSvg(rect.x),
    y: mmToSvg(rect.y),
    width: mmSpanToSvg(rect.width),
    height: mmSpanToSvg(rect.depth),
  };
}

export function LayoutCanvas({ project, layout, perimeterThickness }: LayoutCanvasProps) {
  const grid = drawerGrid(project.drawer, MAX_GRID);
  const cells = drawerCells(grid).map((cell) => ({ ...cell, binId: DRAWER_BIN_ID }));
  const area = packingArea(grid, perimeterThickness);
  const margin = project.clearance + project.dividerThickness / 2;
  const placements = layout?.placements ?? [];
  const walls = layout?.walls ?? [];
  const names = new Map(project.objects.map((object) => [object.id, object.name]));

  if (grid.cols === 0 || grid.rows === 0) {
    return <Hint>The drawer is smaller than one 42 mm Gridfinity cell.</Hint>;
  }

  return (
    <Stack className="no-select layout-canvas" gap="sm" data-divider-count={walls.length}>
      <Hint>
        The drawer holds {grid.cols} × {grid.rows} cells. Compartments are shown at their
        true size inside the bin cavity; the optimizer generates the dividers between them.
      </Hint>
      <EditorCanvas
        className="editor-svg project-svg"
        gridCols={grid.cols}
        gridRows={grid.rows}
        cells={cells}
      >
        <rect {...rectProps(area)} className="layout-area" />
        {placements.map((placement) => {
          const parts = inflateParts(placement.parts, -margin);
          const bounds = partsBounds(parts);
          const color = binColor(placement.objectId);
          return (
            <g key={`${placement.objectId}-${placement.instance}`}>
              {placement.parts.map((part, index) => (
                <rect key={`claim-${index}`} {...rectProps(part)} className="layout-claim" />
              ))}
              {parts.map((part, index) => (
                <rect
                  key={`part-${index}`}
                  {...rectProps(part)}
                  className="layout-part"
                  fill={color}
                />
              ))}
              <text
                className="layout-label"
                x={mmToSvg(bounds.x + bounds.width / 2)}
                y={mmToSvg(bounds.y + bounds.depth / 2)}
              >
                {names.get(placement.objectId) ?? placement.objectId}
              </text>
            </g>
          );
        })}
        {walls.map((wall, index) => (
          <line
            key={`wall-${index}`}
            x1={mmToSvg(wall.start.x)}
            y1={mmToSvg(wall.start.y)}
            x2={mmToSvg(wall.end.x)}
            y2={mmToSvg(wall.end.y)}
            className="layout-wall"
            strokeWidth={Math.max(2, mmSpanToSvg(wall.width))}
          />
        ))}
      </EditorCanvas>
      <Group gap="md">
        <Text>{placements.length} compartment{placements.length === 1 ? '' : 's'}</Text>
        <Text>{walls.length} divider{walls.length === 1 ? '' : 's'}</Text>
        <Text>
          {grid.marginX.toFixed(1)} × {grid.marginY.toFixed(1)} mm of drawer unused
        </Text>
      </Group>
    </Stack>
  );
}
