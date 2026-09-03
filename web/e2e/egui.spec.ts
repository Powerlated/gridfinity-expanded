import { expect, test } from '@playwright/test';

test('the egui application owns the browser canvas', async ({ page }) => {
  const errors: string[] = [];
  page.on('pageerror', error => errors.push(error.message));
  page.on('console', message => {
    if (message.type() === 'error') errors.push(message.text());
  });
  await page.goto('/');
  await expect(page.locator('#gridfinity-canvas')).toBeVisible();
  await expect(page.locator('#startup')).toHaveCount(0, { timeout: 60_000 });
  await expect(page.locator('#root')).toHaveCount(0);
  expect(errors).toEqual([]);
});
