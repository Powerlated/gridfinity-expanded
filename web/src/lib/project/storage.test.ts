import { describe, expect, it } from 'vitest';
import { PROJECT_STORAGE_VERSION, parseProjectStorage, serializeProjectStorage } from './storage';
import { newProject } from './defaults';

const project = newProject([]);

describe('project storage', () => {
  it('round-trips projects and the active selection', () => {
    expect(parseProjectStorage(serializeProjectStorage([project], project.id))).toEqual({
      version: PROJECT_STORAGE_VERSION,
      projects: [project],
      activeProjectId: project.id,
    });
  });

  it('ignores a blob it cannot read instead of throwing', () => {
    expect(parseProjectStorage(null)).toBeNull();
    expect(parseProjectStorage('')).toBeNull();
    expect(parseProjectStorage('not json')).toBeNull();
    expect(parseProjectStorage('null')).toBeNull();
    expect(parseProjectStorage('{"version":1}')).toBeNull();
  });

  it('ignores a blob written by a different version', () => {
    const raw = JSON.stringify({
      version: PROJECT_STORAGE_VERSION + 1,
      projects: [project],
      activeProjectId: project.id,
    });
    expect(parseProjectStorage(raw)).toBeNull();
  });
});
