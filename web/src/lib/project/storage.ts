import { useAppStore } from '../../store';
import type { Project } from '../types';

export const PROJECT_STORAGE_KEY = 'gridfinity-expanded.projects.v1';
export const PROJECT_STORAGE_VERSION = 1;

export interface ProjectStorage {
  version: number;
  projects: Project[];
  activeProjectId: string | null;
}

export function serializeProjectStorage(
  projects: Project[],
  activeProjectId: string | null,
): string {
  return JSON.stringify({ version: PROJECT_STORAGE_VERSION, projects, activeProjectId });
}

export function parseProjectStorage(raw: string | null): ProjectStorage | null {
  if (!raw) return null;
  try {
    const value = JSON.parse(raw) as Partial<ProjectStorage> | null;
    if (!value || value.version !== PROJECT_STORAGE_VERSION || !Array.isArray(value.projects)) {
      return null;
    }
    return {
      version: PROJECT_STORAGE_VERSION,
      projects: value.projects,
      activeProjectId: value.activeProjectId ?? null,
    };
  } catch {
    return null;
  }
}

export function loadProjectStorage(): ProjectStorage | null {
  try {
    return parseProjectStorage(window.localStorage.getItem(PROJECT_STORAGE_KEY));
  } catch {
    return null;
  }
}

export function subscribeProjectStorage(): () => void {
  const stored = loadProjectStorage();
  if (stored) {
    useAppStore.setState({
      projects: stored.projects,
      activeProjectId: stored.activeProjectId,
    });
  }
  return useAppStore.subscribe((state, previous) => {
    if (
      state.projects === previous.projects
      && state.activeProjectId === previous.activeProjectId
    ) {
      return;
    }
    try {
      window.localStorage.setItem(
        PROJECT_STORAGE_KEY,
        serializeProjectStorage(state.projects, state.activeProjectId),
      );
    } catch {
      return;
    }
  });
}
