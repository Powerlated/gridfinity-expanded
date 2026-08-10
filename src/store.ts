import { create } from 'zustand';
import { availableCuts, cutKey, partitionCells, sortCuts, toggleCut } from './lib/cuts';
import { cellKey, classifyEdge, edgeKey, perimeterEdges, sortEdges } from './lib/edges';
import { DESIGN_DEFAULTS } from './lib/gridfinitySpec';
import { PRINTER_PROFILES, checkBedFit, cutsForPrinter } from './lib/printers';
import { PROJECT_DEFAULTS, newObject, newProject } from './lib/project/defaults';
import { drawerGrid } from './lib/project/drawer';
import { buildDrawerBin } from './lib/project/layout';
import type {
  BinDesign,
  Cell,
  Cut,
  Design,
  Drawer,
  Edge,
  FastenerSettings,
  PackEffort,
  PackResult,
  PrinterSettings,
  Project,
  ProjectObject,
  Rect,
  Wall,
} from './lib/types';

const DEFAULT_PRINTER = PRINTER_PROFILES.find(
  (printer) => printer.name === DESIGN_DEFAULTS.printerName,
)!;

export const PANEL_MIN_WIDTH = 220;
export const PANEL_MAX_WIDTH = 900;
export const MAX_GRID = 40;

export type PanelSide = 'sidebar' | 'settings';

export type AppMode = 'bins' | 'project';

export type RenderQuality = 'low' | 'medium' | 'high';

export const DEFAULT_RENDER_QUALITY: RenderQuality = 'high';

export const RENDER_QUALITY_INDEX: Record<RenderQuality, number> = {
  low: 0,
  medium: 1,
  high: 2,
};

export function renderQualityFromIndex(index: number): RenderQuality {
  const found = (Object.keys(RENDER_QUALITY_INDEX) as RenderQuality[])
    .find((level) => RENDER_QUALITY_INDEX[level] === index);
  return found ?? DEFAULT_RENDER_QUALITY;
}

const DEFAULT_PANEL_WIDTHS: Record<PanelSide, number> = {
  sidebar: 360,
  settings: 300,
};

function sortCells(cells: Cell[]): Cell[] {
  return [...cells].sort((a, b) => a.y - b.y || a.x - b.x);
}

function emptyBin(id: string): BinDesign {
  return { id, cells: [], openings: [], walls: [], cuts: [] };
}

function resetForShape(bin: BinDesign, cells: Cell[], printer: PrinterSettings): BinDesign {
  const sorted = sortCells(cells);
  return {
    ...bin,
    cells: sorted,
    openings: [],
    walls: [],
    cuts: cutsForPrinter(sorted, printer),
  };
}

function nextBinId(bins: BinDesign[]): string {
  const ids = new Set(bins.map((bin) => bin.id));
  let index = 1;
  while (ids.has(`bin-${index}`)) index++;
  return `bin-${index}`;
}

function isEdgeConnected(cell: Cell, cells: Cell[]): boolean {
  return cells.some((value) =>
    Math.abs(value.x - cell.x) + Math.abs(value.y - cell.y) === 1);
}

function cutOrientation(cut: Cut): 'h' | 'v' {
  return cut.start.x === cut.end.x ? 'v' : 'h';
}

function cutCenter(cut: Cut): { x: number; y: number } {
  return {
    x: (cut.start.x + cut.end.x) / 2,
    y: (cut.start.y + cut.end.y) / 2,
  };
}

function binPartsFit(bin: BinDesign, printer: PrinterSettings): boolean {
  return partitionCells(bin.cells, bin.cuts)
    .every((part) => checkBedFit(part.cells, printer).fits);
}

const initialCells: Cell[] = [
  { x: 0, y: 0 },
  { x: 1, y: 0 },
  { x: 0, y: 1 },
  { x: 1, y: 1 },
];

export const DEFAULT_DESIGN: Design = {
  bins: [{
    ...emptyBin('bin-1'),
    cells: initialCells,
    cuts: cutsForPrinter(initialCells, DEFAULT_PRINTER),
  }],
  heightUnits: DESIGN_DEFAULTS.heightUnits,
  perimeterThickness: DESIGN_DEFAULTS.perimeterThickness,
  filletRadius: DESIGN_DEFAULTS.filletRadius,
  fasteners: { ...DESIGN_DEFAULTS.fasteners },
  printer: { ...DEFAULT_PRINTER },
};

export function minGridSize(cells: Cell[]): { cols: number; rows: number } {
  return {
    cols: Math.max(4, ...cells.map((cell) => cell.x + 1)),
    rows: Math.max(4, ...cells.map((cell) => cell.y + 1)),
  };
}

interface AppState {
  design: Design;
  selectedBinId: string;
  gridCols: number;
  gridRows: number;
  panelWidths: Record<PanelSide, number>;
  renderQuality: RenderQuality;
  appMode: AppMode;
  projects: Project[];
  activeProjectId: string | null;
  selectedObjectId: string | null;
  packEffort: PackEffort;
  layout: PackResult | null;
  setAppMode: (appMode: AppMode) => void;
  createProject: () => void;
  renameProject: (id: string, name: string) => void;
  deleteProject: (id: string) => void;
  selectProject: (id: string) => void;
  setDrawer: (drawer: Drawer) => void;
  setDividerThickness: (dividerThickness: number) => void;
  setClearance: (clearance: number) => void;
  setPackEffort: (packEffort: PackEffort) => void;
  addObject: () => void;
  renameObject: (id: string, name: string) => void;
  setObjectQuantity: (id: string, quantity: number) => void;
  removeObject: (id: string) => void;
  selectObject: (id: string | null) => void;
  addObjectPart: (part: Rect) => void;
  updateObjectPart: (index: number, patch: Partial<Rect>) => void;
  removeObjectPart: (index: number) => void;
  setLayout: (layout: PackResult | null) => void;
  applyLayout: () => void;
  selectBin: (id: string) => void;
  startNewBin: () => void;
  paintCell: (cell: Cell, targetBinId?: string) => void;
  removeSelectedCell: (cell: Cell) => void;
  setHeightUnits: (heightUnits: number) => void;
  setPerimeterThickness: (perimeterThickness: number) => void;
  setFilletRadius: (filletRadius: number) => void;
  setFasteners: (patch: Partial<FastenerSettings>) => void;
  setPrinter: (printer: PrinterSettings) => void;
  setOpeningState: (edges: Edge[], open: boolean) => void;
  toggleOpening: (edge: Edge) => void;
  resetSelectedWalls: () => void;
  addWall: (wall: Wall) => void;
  updateWall: (index: number, patch: Partial<Wall>) => void;
  removeWall: (index: number) => void;
  toggleCut: (binId: string, cut: Cut) => void;
  moveCut: (binId: string, index: number, direction: -1 | 1) => void;
  resetCuts: (binId: string) => void;
  setGridSize: (cols: number, rows: number) => void;
  setPanelWidth: (panel: PanelSide, width: number) => void;
  setRenderQuality: (renderQuality: RenderQuality) => void;
}

function withActiveProject(
  state: AppState,
  update: (project: Project) => Project,
): Partial<AppState> {
  return {
    projects: state.projects.map((project) =>
      project.id === state.activeProjectId ? update(project) : project),
    layout: null,
  };
}

function withObject(
  state: AppState,
  id: string,
  update: (object: ProjectObject) => ProjectObject,
): Partial<AppState> {
  return withActiveProject(state, (project) => ({
    ...project,
    objects: project.objects.map((object) => object.id === id ? update(object) : object),
  }));
}

function activeProject(state: AppState): Project | undefined {
  return state.projects.find((project) => project.id === state.activeProjectId);
}

export const useAppStore = create<AppState>((set) => ({
  design: DEFAULT_DESIGN,
  selectedBinId: DEFAULT_DESIGN.bins[0].id,
  gridCols: 7,
  gridRows: 7,
  panelWidths: DEFAULT_PANEL_WIDTHS,
  renderQuality: DEFAULT_RENDER_QUALITY,
  appMode: 'bins',
  projects: [],
  activeProjectId: null,
  selectedObjectId: null,
  packEffort: PROJECT_DEFAULTS.effort,
  layout: null,

  setAppMode: (appMode) => set({ appMode }),

  createProject: () => set((state) => {
    const project = newProject(state.projects);
    return {
      projects: [...state.projects, project],
      activeProjectId: project.id,
      selectedObjectId: null,
      layout: null,
    };
  }),

  renameProject: (id, name) => set((state) => ({
    projects: state.projects.map((project) => project.id === id ? { ...project, name } : project),
  })),

  deleteProject: (id) => set((state) => {
    const projects = state.projects.filter((project) => project.id !== id);
    const activeProjectId = state.activeProjectId === id
      ? projects[0]?.id ?? null
      : state.activeProjectId;
    return { projects, activeProjectId, selectedObjectId: null, layout: null };
  }),

  selectProject: (id) => set({ activeProjectId: id, selectedObjectId: null, layout: null }),

  setDrawer: (drawer) => set((state) => withActiveProject(state, (project) => ({ ...project, drawer }))),

  setDividerThickness: (dividerThickness) => set((state) =>
    withActiveProject(state, (project) => ({ ...project, dividerThickness }))),

  setClearance: (clearance) => set((state) =>
    withActiveProject(state, (project) => ({ ...project, clearance }))),

  setPackEffort: (packEffort) => set({ packEffort, layout: null }),

  addObject: () => set((state) => {
    const project = activeProject(state);
    if (!project) return state;
    const object = newObject(project.objects);
    return {
      ...withActiveProject(state, (value) => ({ ...value, objects: [...value.objects, object] })),
      selectedObjectId: object.id,
    };
  }),

  renameObject: (id, name) => set((state) => withObject(state, id, (object) => ({ ...object, name }))),

  setObjectQuantity: (id, quantity) => set((state) =>
    withObject(state, id, (object) => ({ ...object, quantity }))),

  removeObject: (id) => set((state) => ({
    ...withActiveProject(state, (project) => ({
      ...project,
      objects: project.objects.filter((object) => object.id !== id),
    })),
    selectedObjectId: state.selectedObjectId === id ? null : state.selectedObjectId,
  })),

  selectObject: (id) => set({ selectedObjectId: id }),

  addObjectPart: (part) => set((state) => state.selectedObjectId
    ? withObject(state, state.selectedObjectId, (object) => ({
      ...object,
      parts: [...object.parts, part],
    }))
    : state),

  updateObjectPart: (index, patch) => set((state) => state.selectedObjectId
    ? withObject(state, state.selectedObjectId, (object) => ({
      ...object,
      parts: object.parts.map((part, partIndex) =>
        partIndex === index ? { ...part, ...patch } : part),
    }))
    : state),

  removeObjectPart: (index) => set((state) => state.selectedObjectId
    ? withObject(state, state.selectedObjectId, (object) => ({
      ...object,
      parts: object.parts.filter((_, partIndex) => partIndex !== index),
    }))
    : state),

  setLayout: (layout) => set({ layout }),

  applyLayout: () => set((state) => {
    const project = activeProject(state);
    if (!project || !state.layout) return state;
    const bin = buildDrawerBin(
      project,
      state.layout,
      state.design.printer,
      state.design.perimeterThickness,
      MAX_GRID,
    );
    const grid = drawerGrid(project.drawer, MAX_GRID);
    return {
      design: { ...state.design, bins: [bin] },
      selectedBinId: bin.id,
      gridCols: Math.max(4, grid.cols),
      gridRows: Math.max(4, grid.rows),
      appMode: 'bins',
    };
  }),

  selectBin: (id) => set({ selectedBinId: id }),

  startNewBin: () => set((state) => ({ selectedBinId: nextBinId(state.design.bins) })),

  paintCell: (cell, targetBinId) => set((state) => {
    const key = cellKey(cell);
    const owner = state.design.bins.find((bin) => bin.cells.some((value) => cellKey(value) === key));
    if (owner?.id === (targetBinId ?? state.selectedBinId)) return state;

    const selected = state.design.bins.find((bin) => bin.id === state.selectedBinId);
    const targetId = targetBinId ?? (selected && !isEdgeConnected(cell, selected.cells)
      ? nextBinId(state.design.bins)
      : state.selectedBinId);

    const bins = state.design.bins
      .map((bin) => {
        if (bin.id === owner?.id) {
          return resetForShape(bin, bin.cells.filter((value) => cellKey(value) !== key), state.design.printer);
        }
        if (bin.id === targetId) {
          return resetForShape(bin, [...bin.cells, cell], state.design.printer);
        }
        return bin;
      })
      .filter((bin) => bin.cells.length > 0);
    if (!bins.some((bin) => bin.id === targetId)) {
      bins.push(resetForShape(emptyBin(targetId), [cell], state.design.printer));
    }
    return { design: { ...state.design, bins }, selectedBinId: targetId };
  }),

  removeSelectedCell: (cell) => set((state) => {
    const key = cellKey(cell);
    const bins = state.design.bins
      .map((bin) => bin.id === state.selectedBinId
        ? resetForShape(bin, bin.cells.filter((value) => cellKey(value) !== key), state.design.printer)
        : bin)
      .filter((bin) => bin.cells.length > 0);
    return { design: { ...state.design, bins } };
  }),

  setHeightUnits: (heightUnits) => set((state) => ({
    design: { ...state.design, heightUnits },
  })),

  setPerimeterThickness: (perimeterThickness) => set((state) => ({
    design: { ...state.design, perimeterThickness },
  })),

  setFilletRadius: (filletRadius) => set((state) => ({
    design: { ...state.design, filletRadius },
  })),

  setFasteners: (patch) => set((state) => ({
    design: { ...state.design, fasteners: { ...state.design.fasteners, ...patch } },
  })),

  setPrinter: (printer) => set((state) => ({
    design: {
      ...state.design,
      printer,
      bins: state.design.bins.map((bin) => binPartsFit(bin, printer)
        ? bin
        : { ...bin, cuts: cutsForPrinter(bin.cells, printer, bin.cuts) }),
    },
  })),

  setOpeningState: (edges, open) => set((state) => ({
    design: {
      ...state.design,
      bins: state.design.bins.map((bin) => {
        const setOfCells = new Set(bin.cells.map(cellKey));
        const relevant = edges.filter((edge) => classifyEdge(setOfCells, edge) === 'perimeter');
        if (relevant.length === 0) return bin;
        const relevantKeys = new Set(relevant.map(edgeKey));
        const openings = open
          ? sortEdges([...bin.openings.filter((edge) => !relevantKeys.has(edgeKey(edge))), ...relevant])
          : bin.openings.filter((edge) => !relevantKeys.has(edgeKey(edge)));
        return { ...bin, openings };
      }),
    },
  })),

  toggleOpening: (edge) => set((state) => {
    const bordering = state.design.bins.filter((bin) =>
      classifyEdge(new Set(bin.cells.map(cellKey)), edge) === 'perimeter');
    const shouldOpen = bordering.some((bin) => !bin.openings.some((value) => edgeKey(value) === edgeKey(edge)));
    return {
      design: {
        ...state.design,
        bins: state.design.bins.map((bin) => {
          if (!bordering.some((value) => value.id === bin.id)) return bin;
          const without = bin.openings.filter((value) => edgeKey(value) !== edgeKey(edge));
          return { ...bin, openings: shouldOpen ? sortEdges([...without, edge]) : without };
        }),
      },
    };
  }),

  resetSelectedWalls: () => set((state) => {
    const selected = state.design.bins.find((bin) => bin.id === state.selectedBinId);
    if (!selected) return state;
    const perimeterKeys = new Set(perimeterEdges(selected.cells).map(edgeKey));
    return {
      design: {
        ...state.design,
        bins: state.design.bins.map((bin) => ({
          ...bin,
          openings: bin.openings.filter((edge) => !perimeterKeys.has(edgeKey(edge))),
          walls: bin.id === selected.id ? [] : bin.walls,
        })),
      },
    };
  }),

  addWall: (wall) => set((state) => ({
    design: {
      ...state.design,
      bins: state.design.bins.map((bin) => bin.id === state.selectedBinId
        ? { ...bin, walls: [...bin.walls, wall] }
        : bin),
    },
  })),

  updateWall: (index, patch) => set((state) => ({
    design: {
      ...state.design,
      bins: state.design.bins.map((bin) => bin.id === state.selectedBinId
        ? { ...bin, walls: bin.walls.map((wall, wallIndex) =>
          wallIndex === index ? { ...wall, ...patch } : wall) }
        : bin),
    },
  })),

  removeWall: (index) => set((state) => ({
    design: {
      ...state.design,
      bins: state.design.bins.map((bin) => bin.id === state.selectedBinId
        ? { ...bin, walls: bin.walls.filter((_, wallIndex) => wallIndex !== index) }
        : bin),
    },
  })),

  toggleCut: (binId, cut) => set((state) => ({
    design: {
      ...state.design,
      bins: state.design.bins.map((bin) => bin.id === binId
        ? { ...bin, cuts: toggleCut(bin.cuts, cut) }
        : bin),
    },
  })),

  moveCut: (binId, index, direction) => set((state) => ({
    design: {
      ...state.design,
      bins: state.design.bins.map((bin) => {
        if (bin.id !== binId || !bin.cuts[index]) return bin;
        const current = bin.cuts[index];
        const center = cutCenter(current);
        const candidates = availableCuts(bin.cells)
          .filter((cut) => cutOrientation(cut) === cutOrientation(current));
        if (candidates.length === 0) return bin;
        let candidateIndex = candidates.findIndex((cut) => cutKey(cut) === cutKey(current));
        if (candidateIndex < 0) {
          candidateIndex = candidates.reduce((best, cut, valueIndex) => {
            const point = cutCenter(cut);
            const bestPoint = cutCenter(candidates[best]);
            return Math.hypot(point.x - center.x, point.y - center.y) <
              Math.hypot(bestPoint.x - center.x, bestPoint.y - center.y)
              ? valueIndex : best;
          }, 0);
        }
        const nextIndex = Math.max(0, Math.min(candidates.length - 1, candidateIndex + direction));
        if (nextIndex === candidateIndex) return bin;
        const cuts = bin.cuts.filter((_, cutIndex) => cutIndex !== index);
        return { ...bin, cuts: sortCuts([...cuts, candidates[nextIndex]]) };
      }),
    },
  })),

  resetCuts: (binId) => set((state) => ({
    design: {
      ...state.design,
      bins: state.design.bins.map((bin) => bin.id === binId
        ? { ...bin, cuts: cutsForPrinter(bin.cells, state.design.printer) }
        : bin),
    },
  })),

  setGridSize: (cols, rows) => set((state) => {
    const cells = state.design.bins.flatMap((bin) => bin.cells);
    const min = minGridSize(cells);
    return {
      gridCols: Math.min(MAX_GRID, Math.max(min.cols, Math.round(cols))),
      gridRows: Math.min(MAX_GRID, Math.max(min.rows, Math.round(rows))),
    };
  }),

  setPanelWidth: (panel, width) => set((state) => ({
    panelWidths: {
      ...state.panelWidths,
      [panel]: Math.min(PANEL_MAX_WIDTH, Math.max(PANEL_MIN_WIDTH, Math.round(width))),
    },
  })),

  setRenderQuality: (renderQuality) => set({ renderQuality }),
}));
