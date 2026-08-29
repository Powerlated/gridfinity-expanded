import type { ReactNode } from 'react';
import { Group, Slider, Text } from '@mantine/core';
import { useId } from '@mantine/hooks';
import { Field } from './Field';

interface SliderFieldProps {
  label: ReactNode;
  min: number;
  max: number;
  step: number;
  value: number;
  onChange: (value: number) => void;
  /** Formatted value shown next to the slider, e.g. "1.2". */
  display: string;
  /** Dimmed suffix after the value, e.g. "mm" or "(25.75 mm)". */
  unit?: string;
  hint?: ReactNode;
  /** Extra controls rendered between the slider row and the hint. */
  children?: ReactNode;
}

/** Range slider with a formatted readout — the standard numeric control. */
export function SliderField({
  label, min, max, step, value, onChange, display, unit, hint, children,
}: SliderFieldProps) {
  const id = useId();
  return (
    <Field id={id} label={label} hint={hint}>
      <Group gap="sm" wrap="nowrap">
        <Slider
          id={id}
          flex={1}
          min={min}
          max={max}
          step={step}
          value={value}
          onChange={onChange}
          label={null}
        />
        <Text size="sm" c="bright" miw="5.5rem">
          {display}
          {unit && (
            <Text span>
              {' '}
              {unit}
            </Text>
          )}
        </Text>
      </Group>
      {children}
    </Field>
  );
}
