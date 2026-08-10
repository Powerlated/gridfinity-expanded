import { DESIGN_DEFAULTS } from '../gridfinitySpec';
import type { PackEffort, Project, ProjectObject } from '../types';

export const PROJECT_DEFAULTS = {
  drawer: { width: 400, depth: 300 },
  dividerThickness: DESIGN_DEFAULTS.perimeterThickness,
  clearance: 0.5,
  effort: 'standard' as PackEffort,
  objectSize: { width: 40, depth: 30 },
} as const;

function nextId(prefix: string, existing: { id: string }[]): string {
  const ids = new Set(existing.map((value) => value.id));
  let index = 1;
  while (ids.has(`${prefix}-${index}`)) index++;
  return `${prefix}-${index}`;
}

export function newProject(existing: Project[]): Project {
  return {
    id: nextId('project', existing),
    name: `Project ${existing.length + 1}`,
    drawer: { ...PROJECT_DEFAULTS.drawer },
    dividerThickness: PROJECT_DEFAULTS.dividerThickness,
    clearance: PROJECT_DEFAULTS.clearance,
    objects: [],
  };
}

export function newObject(existing: ProjectObject[]): ProjectObject {
  return {
    id: nextId('object', existing),
    name: `Object ${existing.length + 1}`,
    parts: [{ x: 0, y: 0, ...PROJECT_DEFAULTS.objectSize }],
    quantity: 1,
  };
}
