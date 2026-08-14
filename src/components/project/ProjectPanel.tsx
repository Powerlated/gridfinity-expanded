import {
  Button,
  Group,
  NumberInput,
  Paper,
  Progress,
  ScrollArea,
  SegmentedControl,
  Select,
  Stack,
  Text,
  TextInput,
} from '@mantine/core';
import { usePackLayout } from '../../hooks/usePackLayout';
import { MIN_DRAWER_MM, drawerGrid, maxDrawerMm, packingArea } from '../../lib/project/drawer';
import { PACK_RESTARTS } from '../../lib/project/pack';
import type { PackEffort } from '../../lib/types';
import { MAX_GRID, useAppStore } from '../../store';
import { Hint, Label } from '../ui/Field';
import { StatusBanner } from '../ui/StatusBanner';

const NUMBER_INPUT_WIDTH = 88;

const EFFORTS: PackEffort[] = ['quick', 'standard', 'thorough'];

export function ProjectPanel() {
  const projects = useAppStore((state) => state.projects);
  const activeProjectId = useAppStore((state) => state.activeProjectId);
  const packEffort = useAppStore((state) => state.packEffort);
  const layout = useAppStore((state) => state.layout);
  const perimeterThickness = useAppStore((state) => state.design.perimeterThickness);
  const createProject = useAppStore((state) => state.createProject);
  const renameProject = useAppStore((state) => state.renameProject);
  const deleteProject = useAppStore((state) => state.deleteProject);
  const selectProject = useAppStore((state) => state.selectProject);
  const setDrawer = useAppStore((state) => state.setDrawer);
  const setDividerThickness = useAppStore((state) => state.setDividerThickness);
  const setClearance = useAppStore((state) => state.setClearance);
  const setPackEffort = useAppStore((state) => state.setPackEffort);
  const applyLayout = useAppStore((state) => state.applyLayout);
  const { running, progress, error, optimize, cancel } = usePackLayout();

  const project = projects.find((value) => value.id === activeProjectId);
  const grid = project ? drawerGrid(project.drawer, MAX_GRID) : null;
  const requested = project?.objects.reduce((total, object) => total + object.quantity, 0) ?? 0;
  const placed = layout?.placements.length ?? 0;

  function startOptimize() {
    if (!project || !grid) return;
    optimize({
      area: packingArea(grid, perimeterThickness),
      objects: project.objects,
      dividerThickness: project.dividerThickness,
      clearance: project.clearance,
      effort: packEffort,
    });
  }

  return (
    <ScrollArea h="100%" p="md">
      <Stack
        gap="sm"
        className="project-panel"
        data-pack-progress={running ? progress.toFixed(2) : 'idle'}
        data-placed-count={placed}
      >
        <Label>Project</Label>
        <Group gap="xs" wrap="nowrap">
          <Select
            flex={1}
            data={projects.map((value) => ({ value: value.id, label: value.name }))}
            value={activeProjectId}
            onChange={(value) => value && selectProject(value)}
            placeholder="No projects yet"
            aria-label="Active project"
          />
          <Button size="xs" variant="default" onClick={createProject}>+ New</Button>
        </Group>

        {!project && <Hint>Create a project to define a drawer and the objects to organize.</Hint>}

        {project && grid && (
          <>
            <TextInput
              value={project.name}
              onChange={(event) => renameProject(project.id, event.currentTarget.value)}
              aria-label="Project name"
            />

            <Label>Drawer</Label>
            <Group gap="xs" wrap="nowrap">
              <NumberInput
                w={NUMBER_INPUT_WIDTH}
                hideControls
                min={MIN_DRAWER_MM}
                max={maxDrawerMm(MAX_GRID)}
                step={10}
                value={project.drawer.width}
                onChange={(value) => {
                  const width = typeof value === 'number' ? value : Number.parseFloat(value);
                  if (Number.isFinite(width)) setDrawer({ ...project.drawer, width });
                }}
                aria-label="Drawer width"
              />
              <Text span>×</Text>
              <NumberInput
                w={NUMBER_INPUT_WIDTH}
                hideControls
                min={MIN_DRAWER_MM}
                max={maxDrawerMm(MAX_GRID)}
                step={10}
                value={project.drawer.depth}
                onChange={(value) => {
                  const depth = typeof value === 'number' ? value : Number.parseFloat(value);
                  if (Number.isFinite(depth)) setDrawer({ ...project.drawer, depth });
                }}
                aria-label="Drawer depth"
              />
              <Text span>mm</Text>
            </Group>
            <Hint>
              {grid.cols} × {grid.rows} cells fit, leaving {grid.marginX.toFixed(1)} ×{' '}
              {grid.marginY.toFixed(1)} mm unused.
            </Hint>

            <Label>Fit</Label>
            <Group gap="xs" wrap="nowrap">
              <Text flex={1}>Divider</Text>
              <NumberInput
                w={NUMBER_INPUT_WIDTH}
                hideControls
                min={0.4}
                max={8}
                step={0.2}
                value={project.dividerThickness}
                onChange={(value) => {
                  const thickness = typeof value === 'number' ? value : Number.parseFloat(value);
                  if (Number.isFinite(thickness)) setDividerThickness(thickness);
                }}
                aria-label="Divider thickness"
              />
            </Group>
            <Group gap="xs" wrap="nowrap">
              <Text flex={1}>Clearance</Text>
              <NumberInput
                w={NUMBER_INPUT_WIDTH}
                hideControls
                min={0}
                max={5}
                step={0.1}
                value={project.clearance}
                onChange={(value) => {
                  const clearance = typeof value === 'number' ? value : Number.parseFloat(value);
                  if (Number.isFinite(clearance)) setClearance(clearance);
                }}
                aria-label="Object clearance"
              />
            </Group>

            <Label>Optimizer</Label>
            <SegmentedControl
              fullWidth
              value={packEffort}
              onChange={(value) => setPackEffort(value as PackEffort)}
              data={EFFORTS.map((effort) => ({
                value: effort,
                label: `${effort[0].toUpperCase()}${effort.slice(1)}`,
              }))}
            />
            <Hint>
              {PACK_RESTARTS[packEffort]} attempts. The search is seeded, so the same drawer
              and objects always produce the same layout.
            </Hint>
            <Group gap="xs" wrap="nowrap">
              <Button
                flex={1}
                variant="default"
                disabled={project.objects.length === 0 || running}
                onClick={startOptimize}
              >
                Optimize
              </Button>
              {running && <Button variant="default" onClick={cancel}>Cancel</Button>}
            </Group>
            {running && (
              <Progress value={progress * 100} aria-label="Optimization progress" />
            )}
            {error && <StatusBanner ok={false}>{error}</StatusBanner>}

            {layout && (
              <>
                <Label>Result</Label>
                <StatusBanner ok={placed === requested}>
                  {placed} of {requested} placed
                </StatusBanner>
                <Stack gap={4}>
                  {project.objects.map((object) => (
                    <Paper key={object.id} p={6} bg="dark.6">
                      <Group gap="xs" wrap="nowrap">
                        <Text flex={1} c="bright">{object.name}</Text>
                        <Text>
                          {layout.placedByObjectId[object.id] ?? 0} / {object.quantity}
                        </Text>
                      </Group>
                    </Paper>
                  ))}
                </Stack>
                <Button
                  disabled={placed === 0 || running}
                  onClick={applyLayout}
                >
                  Apply to bin editor
                </Button>
                <Hint>
                  Applying replaces every bin in the editor with one drawer-sized bin whose
                  dividers come from this layout.
                </Hint>
              </>
            )}

            <Button variant="default" color="red" onClick={() => deleteProject(project.id)}>
              Delete project
            </Button>
          </>
        )}
      </Stack>
    </ScrollArea>
  );
}
