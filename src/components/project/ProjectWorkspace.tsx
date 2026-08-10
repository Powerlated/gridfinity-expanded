import { useEffect, useState } from 'react';
import { ScrollArea, SegmentedControl, Stack } from '@mantine/core';
import { useAppStore } from '../../store';
import { Hint } from '../ui/Field';
import { LayoutCanvas } from './LayoutCanvas';
import { ObjectCanvas } from './ObjectCanvas';
import './project.css';

type ProjectView = 'Layout' | 'Object';

export function ProjectWorkspace() {
  const projects = useAppStore((state) => state.projects);
  const activeProjectId = useAppStore((state) => state.activeProjectId);
  const selectedObjectId = useAppStore((state) => state.selectedObjectId);
  const layout = useAppStore((state) => state.layout);
  const perimeterThickness = useAppStore((state) => state.design.perimeterThickness);
  const [view, setView] = useState<ProjectView>('Layout');

  useEffect(() => {
    if (selectedObjectId) setView('Object');
  }, [selectedObjectId]);

  const project = projects.find((value) => value.id === activeProjectId);
  const object = project?.objects.find((value) => value.id === selectedObjectId);

  return (
    <ScrollArea h="100%" p="md" className="project-workspace">
      <Stack gap="sm" maw={760} mx="auto">
        <SegmentedControl
          fullWidth
          value={view}
          onChange={(value) => setView(value as ProjectView)}
          data={['Layout', 'Object']}
        />
        {!project && <Hint>Create a project in the left panel to get started.</Hint>}
        {project && view === 'Layout' && (
          <LayoutCanvas
            project={project}
            layout={layout}
            perimeterThickness={perimeterThickness}
          />
        )}
        {project && view === 'Object' && !object && (
          <Hint>Select an object in the right panel to draw its boxes.</Hint>
        )}
        {project && view === 'Object' && object && <ObjectCanvas object={object} />}
      </Stack>
    </ScrollArea>
  );
}
