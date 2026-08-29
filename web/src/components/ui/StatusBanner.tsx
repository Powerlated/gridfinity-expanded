import type { ReactNode } from 'react';
import { Alert } from '@mantine/core';

interface StatusBannerProps {
  ok: boolean;
  children: ReactNode;
}

/** Green (ok) / amber (warning) info box, e.g. for bed-fit results. */
export function StatusBanner({ ok, children }: StatusBannerProps) {
  return (
    <Alert color={ok ? 'green' : 'yellow'} icon={ok ? '✓' : '⚠'}>
      {children}
    </Alert>
  );
}
