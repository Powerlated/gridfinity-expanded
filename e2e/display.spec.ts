import { expect, test } from '@playwright/test';

async function openDisplaySection(page: import('@playwright/test').Page) {
  await page.getByRole('button', { name: 'Display' }).click();
}

test('starts at the richest render quality', async ({ page }) => {
  await page.goto('/');

  const viewer = page.locator('.viewer');
  await expect(viewer).toHaveAttribute('data-renderer', 'rust-webgl2', { timeout: 30_000 });
  await expect(viewer).toHaveAttribute('data-render-quality-mode', 'high');
  await expect(page.locator('.viewer-overlay--error')).toHaveCount(0);
  await expect(viewer).toHaveAttribute('data-render-quality', 'high', { timeout: 30_000 });
});

test('the display tab drives the renderer and never touches the geometry', async ({ page }) => {
  const errors: string[] = [];
  page.on('pageerror', (error) => errors.push(error.message));
  page.on('console', (message) => {
    if (message.type() === 'error') errors.push(message.text());
  });
  await page.goto('/');

  const viewer = page.locator('.viewer');
  await expect(viewer).toHaveAttribute('data-render-quality', 'high', { timeout: 30_000 });
  const parts = await viewer.getAttribute('data-part-count');
  const directions = await viewer.getAttribute('data-piece-directions');

  await openDisplaySection(page);

  for (const [label, level] of [['Low', 'low'], ['Medium', 'medium'], ['High', 'high']]) {
    await page.locator('label').filter({ hasText: new RegExp(`^${label}$`) }).click();
    await expect(viewer).toHaveAttribute('data-render-quality-mode', level);
    await expect(viewer).toHaveAttribute('data-render-quality', level, { timeout: 10_000 });
  }

  await expect(viewer).toHaveAttribute('data-part-count', parts ?? '');
  await expect(viewer).toHaveAttribute('data-piece-directions', directions ?? '');
  await expect(page.locator('canvas.viewer-canvas')).toBeVisible();
  expect(errors).toEqual([]);
});
