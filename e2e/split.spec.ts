import { expect, test } from '@playwright/test';

test('the explode view separates a split bin and settles back', async ({ page }) => {
  await page.goto('/');

  const viewer = page.locator('.viewer');
  await expect(viewer).toHaveAttribute('data-render-quality', 'high', { timeout: 30_000 });
  await expect(viewer).toHaveAttribute('data-part-count', '1');

  const showGaps = page.getByRole('button', { name: 'Show gaps' });
  await expect(showGaps).toBeDisabled();

  await page.getByRole('tab', { name: 'Cuts' }).click();
  await page.locator('.cut-line').first().click();
  await expect(viewer).toHaveAttribute('data-part-count', '2', { timeout: 30_000 });
  await expect(viewer).not.toHaveAttribute('data-piece-directions', '0.00,0.00;0.00,0.00');

  await expect(showGaps).toBeEnabled();
  await showGaps.click();
  await expect(viewer).toHaveAttribute('data-explode', '24.00', { timeout: 10_000 });

  await page.getByRole('button', { name: 'Close up' }).click();
  await expect(viewer).toHaveAttribute('data-explode', '0.00', { timeout: 10_000 });
});
