import { Stack, Switch } from '@mantine/core';
import { GRIDFINITY_SPEC } from '../../../lib/gridfinitySpec';
import { useAppStore } from '../../../store';
import { Label } from '../../ui/Field';

export function FeaturesTab() {
  const fasteners = useAppStore((state) => state.design.fasteners);
  const setFasteners = useAppStore((state) => state.setFasteners);
  const magnet = GRIDFINITY_SPEC.hardware.magnet;

  return (
    <Stack gap="sm">
      <Label>Base attachment</Label>
      <Switch
        label="Magnet recesses"
        description={`${magnet.recessDiameter} mm × ${magnet.recessDepth} mm, four per cell`}
        checked={fasteners.magnets}
        onChange={(event) => setFasteners({ magnets: event.currentTarget.checked })}
      />
      <Switch
        label="M3 recesses"
        description="M3 pilot recesses inside the same four base positions"
        checked={fasteners.m3}
        onChange={(event) => setFasteners({ m3: event.currentTarget.checked })}
      />
    </Stack>
  );
}
