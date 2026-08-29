import { expect, test } from '@playwright/test';

test('renders the design viewer on the default route', async ({ page }) => {
  const errors: string[] = [];
  page.on('pageerror', (error) => errors.push(error.message));
  page.on('console', (message) => {
    if (message.type() === 'error') errors.push(message.text());
  });
  await page.goto('/');

  const viewer = page.locator('.viewer');
  await expect(viewer).toHaveAttribute('data-renderer', 'rust-webgl2', { timeout: 30_000 });
  await expect(viewer).toHaveAttribute('data-part-count', /[1-9]/, { timeout: 30_000 });
  await expect(viewer).not.toHaveAttribute('data-badapple-frame', /.*/);
  await expect(page.locator('canvas.viewer-canvas')).toBeVisible();
  expect(errors).toEqual([]);
});

test('plays the bad apple clip on the #badapple route', async ({ page }) => {
  const errors: string[] = [];
  page.on('pageerror', (error) => errors.push(error.message));
  page.on('console', (message) => {
    if (message.type() === 'error') errors.push(message.text());
  });
  await page.goto('/#badapple');

  const viewer = page.locator('.viewer');
  await expect(viewer).toHaveAttribute('data-badapple-frame', /\d+/, { timeout: 60_000 });
  await expect(viewer).toHaveAttribute('data-part-count', '0');
  await expect(page.locator('canvas.viewer-canvas')).toBeVisible();

  const frameOf = async () =>
    Number(await viewer.getAttribute('data-badapple-frame') ?? -1);
  const started = await frameOf();
  await expect.poll(frameOf, { timeout: 60_000 }).toBeGreaterThan(started);

  expect(errors).toEqual([]);
});

test('returns to the design viewer when the bad apple hash is cleared', async ({ page }) => {
  await page.goto('/#badapple');
  const viewer = page.locator('.viewer');
  await expect(viewer).toHaveAttribute('data-badapple-frame', /\d+/, { timeout: 60_000 });

  await page.evaluate(() => { window.location.hash = ''; });
  await expect(viewer).not.toHaveAttribute('data-badapple-frame', /.*/, { timeout: 30_000 });
  await expect(viewer).toHaveAttribute('data-part-count', /[1-9]/, { timeout: 30_000 });
});
