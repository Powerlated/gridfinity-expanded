import {
  Button,
  CloseButton,
  Group,
  NumberInput,
  Paper,
  ScrollArea,
  Stack,
  Text,
  TextInput,
} from '@mantine/core';
import { partsBounds, unionArea } from '../../lib/project/rects';
import type { ProjectObject } from '../../lib/types';
import { useAppStore } from '../../store';
import { binColor } from '../sidebar/binColors';
import { Hint, Label } from '../ui/Field';
import './project.css';

const QUANTITY_INPUT_WIDTH = 56;

function footprintLabel(object: ProjectObject): string {
  if (object.parts.length === 0) return 'empty';
  const bounds = partsBounds(object.parts);
  const size = `${bounds.width.toFixed(0)} × ${bounds.depth.toFixed(0)} mm`;
  return object.parts.length === 1
    ? size
    : `${size}, ${unionArea(object.parts).toFixed(0)} mm²`;
}

export function ObjectPanel() {
  const projects = useAppStore((state) => state.projects);
  const activeProjectId = useAppStore((state) => state.activeProjectId);
  const selectedObjectId = useAppStore((state) => state.selectedObjectId);
  const addObject = useAppStore((state) => state.addObject);
  const renameObject = useAppStore((state) => state.renameObject);
  const setObjectQuantity = useAppStore((state) => state.setObjectQuantity);
  const removeObject = useAppStore((state) => state.removeObject);
  const selectObject = useAppStore((state) => state.selectObject);
  const removeObjectPart = useAppStore((state) => state.removeObjectPart);

  const project = projects.find((value) => value.id === activeProjectId);
  const selected = project?.objects.find((object) => object.id === selectedObjectId);

  if (!project) {
    return (
      <ScrollArea h="100%" p="md">
        <Hint>Create a project first.</Hint>
      </ScrollArea>
    );
  }

  return (
    <ScrollArea h="100%" p="md">
      <Stack gap="sm">
        <Label>Objects</Label>
        <Hint>
          Select an object to draw its boxes on the canvas. Quantity is how many of it you
          want the optimizer to fit.
        </Hint>
        {project.objects.map((object) => (
          <Paper
            key={object.id}
            p={6}
            bg="dark.6"
            className={selectedObjectId === object.id ? 'object-row--selected' : undefined}
            onClick={() => selectObject(object.id)}
          >
            <Stack gap={4}>
              <Group gap="xs" wrap="nowrap">
                <span className="object-swatch" style={{ background: binColor(object.id) }} />
                <TextInput
                  flex={1}
                  size="xs"
                  value={object.name}
                  onChange={(event) => renameObject(object.id, event.currentTarget.value)}
                  aria-label={`Name of ${object.name}`}
                />
                <NumberInput
                  w={QUANTITY_INPUT_WIDTH}
                  size="xs"
                  hideControls
                  min={1}
                  max={200}
                  value={object.quantity}
                  onChange={(value) => {
                    const quantity = typeof value === 'number' ? value : Number.parseInt(value, 10);
                    if (Number.isFinite(quantity)) setObjectQuantity(object.id, quantity);
                  }}
                  aria-label={`Quantity of ${object.name}`}
                />
                <CloseButton
                  aria-label={`Delete ${object.name}`}
                  onClick={(event) => {
                    event.stopPropagation();
                    removeObject(object.id);
                  }}
                />
              </Group>
              <Text>{footprintLabel(object)}</Text>
            </Stack>
          </Paper>
        ))}
        <Button variant="default" onClick={addObject}>+ Object</Button>

        {selected && selected.parts.length > 0 && (
          <Stack gap="xs">
            <Label>Boxes of {selected.name}</Label>
            {selected.parts.map((part, index) => (
              <Paper key={`part-${index}`} p={6} bg="dark.6">
                <Group gap="xs" wrap="nowrap">
                  <Text flex={1} c="bright">
                    #{index + 1} · {part.width.toFixed(0)} × {part.depth.toFixed(0)} mm
                  </Text>
                  <CloseButton
                    aria-label={`Delete box ${index + 1}`}
                    onClick={() => removeObjectPart(index)}
                  />
                </Group>
              </Paper>
            ))}
          </Stack>
        )}
      </Stack>
    </ScrollArea>
  );
}
