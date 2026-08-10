import { beforeEach, describe, expect, it } from 'vitest';
import { packLayout } from './lib/project/pack';
import { drawerGrid, packingArea } from './lib/project/drawer';
import { DRAWER_BIN_ID } from './lib/project/layout';
import type { Design } from './lib/types';
import { DEFAULT_DESIGN, MAX_GRID, useAppStore } from './store';

function copyDesign(): Design {
  return structuredClone(DEFAULT_DESIGN);
}

function seedProject() {
  const state = useAppStore.getState();
  state.createProject();
  useAppStore.getState().setDrawer({ width: 300, depth: 210 });
  useAppStore.getState().addObject();
  const objectId = useAppStore.getState().selectedObjectId!;
  useAppStore.getState().setObjectQuantity(objectId, 6);
  return objectId;
}

function optimize() {
  const state = useAppStore.getState();
  const project = state.projects.find((value) => value.id === state.activeProjectId)!;
  const grid = drawerGrid(project.drawer, MAX_GRID);
  state.setLayout(packLayout({
    area: packingArea(grid, state.design.perimeterThickness),
    objects: project.objects,
    dividerThickness: project.dividerThickness,
    clearance: project.clearance,
    effort: 'quick',
  }));
}

beforeEach(() => {
  useAppStore.setState({
    design: copyDesign(),
    selectedBinId: 'bin-1',
    projects: [],
    activeProjectId: null,
    selectedObjectId: null,
    layout: null,
    appMode: 'bins',
  });
});

describe('project commands', () => {
  it('creates a project, selects it, and seeds an object with one box', () => {
    const objectId = seedProject();
    const state = useAppStore.getState();
    const project = state.projects[0];
    expect(state.activeProjectId).toBe(project.id);
    expect(project.drawer).toEqual({ width: 300, depth: 210 });
    expect(project.objects).toHaveLength(1);
    expect(project.objects[0].id).toBe(objectId);
    expect(project.objects[0].parts).toHaveLength(1);
    expect(project.objects[0].quantity).toBe(6);
  });

  it('discards a stale layout whenever the project changes', () => {
    seedProject();
    optimize();
    expect(useAppStore.getState().layout).not.toBeNull();
    useAppStore.getState().setClearance(1);
    expect(useAppStore.getState().layout).toBeNull();
  });

  it('replaces every bin with one drawer bin and sizes the editor grid to the drawer', () => {
    seedProject();
    optimize();
    useAppStore.getState().applyLayout();

    const state = useAppStore.getState();
    expect(state.design.bins).toHaveLength(1);
    const bin = state.design.bins[0];
    expect(bin.id).toBe(DRAWER_BIN_ID);
    expect(state.selectedBinId).toBe(DRAWER_BIN_ID);
    expect(state.appMode).toBe('bins');
    expect(state.gridCols).toBe(7);
    expect(state.gridRows).toBe(5);
    expect(bin.cells).toHaveLength(35);
    expect(bin.openings).toEqual([]);
    expect(bin.walls.length).toBeGreaterThan(0);
    expect(bin.cuts.length).toBeGreaterThan(0);
  });

  it('leaves the design alone when there is no layout to apply', () => {
    seedProject();
    useAppStore.getState().applyLayout();
    expect(useAppStore.getState().design.bins).toEqual(copyDesign().bins);
  });

  it('drops the selection and the layout when the active project is deleted', () => {
    seedProject();
    optimize();
    const projectId = useAppStore.getState().activeProjectId!;
    useAppStore.getState().deleteProject(projectId);

    const state = useAppStore.getState();
    expect(state.projects).toEqual([]);
    expect(state.activeProjectId).toBeNull();
    expect(state.selectedObjectId).toBeNull();
    expect(state.layout).toBeNull();
  });
});
