import { lazy, Suspense } from 'react';
import { AppShell, Group, SegmentedControl, Text } from '@mantine/core';
import { Sidebar } from './components/sidebar/Sidebar';
import { SettingsPanel } from './components/sidebar/SettingsPanel';
import { PanelResizeHandle } from './components/sidebar/PanelResizeHandle';
import { ProjectPanel } from './components/project/ProjectPanel';
import { ObjectPanel } from './components/project/ObjectPanel';
import { ProjectWorkspace } from './components/project/ProjectWorkspace';
import { ExportMenu } from './components/ExportMenu';
import { useBadApple } from './hooks/useBadApple';
import { useBinGeometry } from './hooks/useBinGeometry';
import { useAppStore, type AppMode } from './store';

const ModelViewer = lazy(() => import('./components/viewer/ModelViewer').then((module) => ({
  default: module.ModelViewer,
})));

const MODES: { value: AppMode; label: string }[] = [
  { value: 'bins', label: 'Bin editor' },
  { value: 'project', label: 'Project' },
];

export default function App() {
  const design = useAppStore((s) => s.design);
  const panelWidths = useAppStore((s) => s.panelWidths);
  const appMode = useAppStore((s) => s.appMode);
  const setAppMode = useAppStore((s) => s.setAppMode);
  const { bins, generating, error } = useBinGeometry(design);
  const badApple = useBadApple();
  const project = appMode === 'project';

  return (
    <AppShell
      mode="static"
      className="app-shell"
      data-app-mode={appMode}
      header={{ height: 48 }}
      navbar={{ width: panelWidths.sidebar, breakpoint: 0 }}
      aside={{ width: panelWidths.settings, breakpoint: 0 }}
      padding={0}
    >
      <AppShell.Header>
        <Group h="100%" px="md" justify="space-between">
          <Text size="sm" fw={600} c="bright" lts="0.02em">
            gridfinity-expanded
          </Text>
          <SegmentedControl
            size="xs"
            value={appMode}
            onChange={(value) => setAppMode(value as AppMode)}
            data={MODES}
          />
          <ExportMenu bins={bins} generating={generating} />
        </Group>
      </AppShell.Header>
      <AppShell.Navbar className="app-panel">
        {project ? <ProjectPanel /> : <Sidebar />}
        <PanelResizeHandle panel="sidebar" />
      </AppShell.Navbar>
      <AppShell.Aside className="app-panel">
        {project ? <ObjectPanel /> : <SettingsPanel />}
        <PanelResizeHandle panel="settings" />
      </AppShell.Aside>
      <AppShell.Main className="app-main">
        <Suspense fallback={(
          <div className="viewer" role="status" aria-label="Loading 3D bin preview">
            <div className="viewer-overlay">
              <Text size="sm" c="dimmed">Loading 3D preview…</Text>
            </div>
          </div>
        )}>
          <ModelViewer
            bins={bins}
            error={error}
            badApple={badApple}
          />
        </Suspense>
        <div className={project ? undefined : 'project-pane--hidden'}>
          <ProjectWorkspace />
        </div>
      </AppShell.Main>
    </AppShell>
  );
}
