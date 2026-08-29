import type { ReactNode } from 'react';
import { Input, Text } from '@mantine/core';

// labelElement stays the default 'label' (not 'span') so it's a real <label
// for="...">, giving native click-to-focus on the linked control.
const LABEL_PROPS = { tt: 'uppercase' as const, fw: 700, lts: '0.05em' };

/** Uppercase section/field label used throughout the sidebar. */
export function Label({ children }: { children: ReactNode }) {
  return (
    <Text component="span" fw={700} tt="uppercase" lts="0.05em">
      {children}
    </Text>
  );
}

/** Muted helper text under a control or editor. */
export function Hint({ children }: { children: ReactNode }) {
  return <Text>{children}</Text>;
}

interface FieldProps {
  /** Id of the primary control, so the label follows Mantine's htmlFor convention. */
  id?: string;
  label: ReactNode;
  hint?: ReactNode;
  children: ReactNode;
}

/**
 * Labelled control block for controls without their own `label`/`description`
 * props (e.g. Slider). Built on Mantine's own `Input.Wrapper` — the same
 * primitive every native Mantine input uses under the hood for its `label`/
 * `description` props — instead of a bespoke label/hint composition, so this
 * follows the identical visual and semantic convention as the rest of the app.
 */
export function Field({ id, label, hint, children }: FieldProps) {
  return (
    <Input.Wrapper id={id} size="xs" label={label} description={hint} labelProps={LABEL_PROPS}>
      {children}
    </Input.Wrapper>
  );
}
