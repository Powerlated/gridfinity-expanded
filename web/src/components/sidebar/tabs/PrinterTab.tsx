import { NumberInput, Paper, Select, Stack } from '@mantine/core';
import { PRINTER_PROFILES, checkDesignFit } from '../../../lib/printers';
import { useAppStore } from '../../../store';
import { Hint } from '../../ui/Field';
import { StatusBanner } from '../../ui/StatusBanner';

export function PrinterTab() {
  const design = useAppStore((state) => state.design);
  const setPrinter = useAppStore((state) => state.setPrinter);
  const printer = design.printer;
  const isCustom = printer.name === 'Custom';
  const fit = checkDesignFit(design.bins, printer);

  return (
    <Stack gap="md">
      <Select
        label="Printer"
        data={PRINTER_PROFILES.map((profile) => profile.name)}
        value={printer.name}
        onChange={(name) => {
          const profile = PRINTER_PROFILES.find((value) => value.name === name);
          if (profile) setPrinter(profile);
        }}
      />
      {isCustom && (
        <Paper p="sm" bg="dark.6">
          <Stack gap="sm">
            {([['bedWidth', 'Bed width'], ['bedDepth', 'Bed depth']] as const)
              .map(([dimension, label]) => (
                <NumberInput
                  key={dimension}
                  label={label}
                  min={50}
                  max={1000}
                  suffix=" mm"
                  value={printer[dimension]}
                  onChange={(value) => setPrinter({
                    ...printer,
                    [dimension]: Number(value),
                  })}
                />
              ))}
          </Stack>
        </Paper>
      )}
      {design.bins.length === 0 ? (
        <Hint>Select cells in the Shape tab first.</Hint>
      ) : (
        <StatusBanner ok={fit.allFit}>
          {fit.allFit
            ? `${fit.parts} part${fit.parts !== 1 ? 's' : ''} fit the ${printer.name} bed (${printer.bedWidth} × ${printer.bedDepth} mm).`
            : `A part (${fit.worst.width} × ${fit.worst.depth} mm) exceeds the ${printer.name} bed (${printer.bedWidth} × ${printer.bedDepth} mm).`}
        </StatusBanner>
      )}
    </Stack>
  );
}
