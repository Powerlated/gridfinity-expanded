import { lazy, Suspense } from 'react';
import { AppShell, Group, Text } from '@mantine/core';
import { Sidebar } from './components/sidebar/Sidebar';
import { SettingsPanel } from './components/sidebar/SettingsPanel';
import { PanelResizeHandle } from './components/sidebar/PanelResizeHandle';
import { ExportMenu } from './components/ExportMenu';
import { useBadApple } from './hooks/useBadApple';
import { useBinGeometry } from './hooks/useBinGeometry';
import { useAppStore } from './store';

const ModelViewer = lazy(() => import('./components/viewer/ModelViewer').then((module) => ({
  default: module.ModelViewer,
})));

export default function App() {
  const design = useAppStore((s) => s.design);
  const panelWidths = useAppStore((s) => s.panelWidths);
  const { bins, design: generatedDesign, generating, error } = useBinGeometry(design);
  const badApple = useBadApple();

  return (
    <AppShell
      mode="static"
      className="app-shell"
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
          <ExportMenu bins={bins} generating={generating} />
        </Group>
      </AppShell.Header>
      <AppShell.Navbar className="app-panel">
        <Sidebar />
        <PanelResizeHandle panel="sidebar" />
      </AppShell.Navbar>
      <AppShell.Aside className="app-panel">
        <SettingsPanel />
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
            design={generatedDesign}
            error={error}
            badApple={badApple.active ? badApple.triangles : null}
            badAppleBounds={badApple.active ? badApple.clip?.bounds ?? null : null}
            badAppleFrame={badApple.active ? badApple.frame : null}
          />
        </Suspense>
      </AppShell.Main>
    </AppShell>
  );
}
