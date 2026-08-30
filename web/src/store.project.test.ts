import { beforeEach, describe, expect, it } from 'vitest';
import { DRAWER_BIN_ID } from './lib/project/layout';
import type { Design, PackResult } from './lib/types';
import { DEFAULT_DESIGN, useAppStore } from './store';

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

/**
 * A finished layout, stated rather than computed: the optimizer lives in the
 * Rust kernel now and is reached through wasm, which vitest's node environment
 * does not load. What these tests exercise is the store's response to a layout,
 * so the layout is a fixture -- two compartments side by side with the divider
 * between them.
 */
function stubLayout(): PackResult {
  const objectId = useAppStore.getState().selectedObjectId!;
  return {
    placements: [
      { objectId, instance: 0, rotation: 0, parts: [{ x: 1.45, y: 1.45, width: 60, depth: 40 }] },
      { objectId, instance: 1, rotation: 0, parts: [{ x: 61.45, y: 1.45, width: 60, depth: 40 }] },
    ],
    placedByObjectId: { [objectId]: 2 },
    iterations: 30,
    tidiness: { lines: 0, runs: 0, fragments: 0, slivers: 0, grouping: 0, balance: 0 },
    walls: [
      { start: { x: 61.45, y: 0.85 }, end: { x: 61.45, y: 42.05 }, width: 1.2 },
      { start: { x: 0.85, y: 41.45 }, end: { x: 122.05, y: 41.45 }, width: 1.2 },
    ],
  };
}

function optimize() {
  useAppStore.getState().setLayout(stubLayout());
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
    expect(bin.walls).toEqual(stubLayout().walls);
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
