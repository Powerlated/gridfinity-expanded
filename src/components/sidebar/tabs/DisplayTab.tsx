import { SegmentedControl, Stack } from '@mantine/core';
import { useAppStore } from '../../../store';
import type { RenderQuality } from '../../../store';
import { Field, Hint } from '../../ui/Field';

const LEVELS: { label: string; value: RenderQuality }[] = [
  { label: 'Low', value: 'low' },
  { label: 'Medium', value: 'medium' },
  { label: 'High', value: 'high' },
];

export function DisplayTab() {
  const renderQuality = useAppStore((state) => state.renderQuality);
  const setRenderQuality = useAppStore((state) => state.setRenderQuality);

  return (
    <Stack gap="md">
      <Field label="Render quality">
        <SegmentedControl
          fullWidth
          size="xs"
          data={LEVELS}
          value={renderQuality}
          onChange={(value) => setRenderQuality(value as RenderQuality)}
        />
      </Field>
      <Hint>
        High adds the floor reflection, contact shadow, ambient occlusion, bloom and
        anti-aliasing; Low drops all of them. Preview only — it never changes exported
        geometry.
      </Hint>
    </Stack>
  );
}
