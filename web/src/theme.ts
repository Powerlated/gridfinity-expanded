import {
  createTheme,
  Button,
  NumberInput,
  Select,
  Slider,
  Switch,
  Tabs,
  Alert,
  Text,
  Menu,
} from '@mantine/core';

/**
 * The single source of truth for how every control in the app looks.
 *
 * Nothing in a component sets padding, font size, radius, or color inline —
 * those decisions are made once here (via each component's `defaultProps`) and
 * inherited everywhere. To restyle the app — bigger buttons, a different accent,
 * rounder corners — edit this file and nothing else.
 */
export const theme = createTheme({
  primaryColor: 'blue',
  fontFamily: 'system-ui, "Segoe UI", Roboto, sans-serif',
  defaultRadius: 'md',
  // App is a dense tool UI: default every control one step smaller than Mantine's.
  components: {
    Button: Button.extend({ defaultProps: { size: 'sm' } }),
    NumberInput: NumberInput.extend({ defaultProps: { size: 'xs' } }),
    Select: Select.extend({ defaultProps: { size: 'sm', allowDeselect: false } }),
    Slider: Slider.extend({ defaultProps: { size: 'sm' } }),
    Switch: Switch.extend({ defaultProps: { size: 'md' } }),
    Tabs: Tabs.extend({
      // 'pills' highlights the active tab with a filled background instead
      // of the 'default' variant's underline; radius 0 keeps them rectangular.
      defaultProps: { variant: 'pills', color: 'blue', radius: 0 },
      // The sidebar's tab strip needs to stay compact and scroll
      // horizontally rather than wrap; centralized here rather than as
      // per-instance overrides on <Tabs.List>/<Tabs.Tab>.
      styles: {
        tab: {
          padding: '0.75rem 0.75rem',
          margin: '0 0',
          fontSize: 'var(--mantine-font-size-sm)',
          // flex-grow lets tabs stretch to fill any leftover width in the
          // list (no gaps), while flex-shrink still lets them shrink back
          // to content size and scroll once the sidebar gets too narrow.
          flex: '1 1 auto',
        },
        // paddingBottom keeps the native horizontal scrollbar clear of the
        // tab labels instead of overlapping them.
        list: {
          flexWrap: 'nowrap',
          overflowX: 'auto',
          gap: 0,
          paddingBottom: '0.5rem',
        },
      },
    }),
    Alert: Alert.extend({ defaultProps: { variant: 'light' } }),
    Menu: Menu.extend({ defaultProps: { shadow: 'md', position: 'bottom-end' } }),
    // Every plain bit of copy defaults to caption-sized, muted text — this is a
    // dense sidebar UI, so that's what most labels/hints/captions actually want.
    // Callers that need the larger body size opt in with <Text size="sm">, and
    // emphasis with <Text c="bright">.
    Text: Text.extend({ defaultProps: { size: 'xs', c: 'dimmed' } }),
  },
});
