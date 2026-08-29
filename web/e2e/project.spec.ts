import { expect, test } from '@playwright/test';

type Page = import('@playwright/test').Page;

async function switchMode(page: Page, label: string) {
  await page.locator('label').filter({ hasText: new RegExp(`^${label}$`) }).first().click();
}

async function drawBox(page: Page, from: [number, number], to: [number, number]) {
  const box = (await page.locator('.project-svg--object').boundingBox())!;
  const at = ([fx, fy]: [number, number]) =>
    [box.x + box.width * fx, box.y + box.height * fy] as const;
  await page.mouse.move(...at(from));
  await page.mouse.down();
  await page.mouse.move(...at(to), { steps: 8 });
  await page.mouse.up();
}

test('a project packs a drawer and applies it as one bin with dividers', async ({ page }) => {
  const errors: string[] = [];
  page.on('pageerror', (error) => errors.push(error.message));
  page.on('console', (message) => {
    if (message.type() === 'error') errors.push(message.text());
  });

  await page.goto('/');
  const shell = page.locator('.app-shell');
  const viewer = page.locator('.viewer');
  await expect(viewer).toHaveAttribute('data-render-quality', 'high', { timeout: 30_000 });
  await expect(viewer).toHaveAttribute('data-part-count', '1');

  await switchMode(page, 'Project');
  await expect(shell).toHaveAttribute('data-app-mode', 'project');

  await page.getByRole('button', { name: '+ New' }).click();
  await page.getByLabel('Drawer width').fill('300');
  await page.getByLabel('Drawer depth').fill('210');

  await page.getByRole('button', { name: '+ Object' }).click();
  await expect(page.locator('.project-svg--object')).toBeVisible();
  await drawBox(page, [0.2, 0.2], [0.6, 0.5]);

  const object = page.locator('.object-part');
  await expect(object).toHaveCount(2);

  await page.getByLabel(/^Quantity of /).fill('8');
  await switchMode(page, 'Layout');

  await page.getByRole('button', { name: 'Optimize' }).click();
  const panel = page.locator('.project-panel');
  await expect(panel).toHaveAttribute('data-pack-progress', 'idle', { timeout: 60_000 });
  await expect(panel).not.toHaveAttribute('data-placed-count', '0');
  await expect(page.locator('.layout-canvas')).not.toHaveAttribute('data-divider-count', '0');

  await page.getByRole('button', { name: 'Apply to bin editor' }).click();
  await expect(shell).toHaveAttribute('data-app-mode', 'bins');
  await expect(viewer).not.toHaveAttribute('data-part-count', '0', { timeout: 60_000 });
  await expect(page.locator('.viewer-overlay--error')).toHaveCount(0);

  await page.getByRole('tab', { name: 'Walls' }).click();
  await expect(page.locator('.custom-wall-g').first()).toBeVisible();

  await switchMode(page, 'Project');
  await switchMode(page, 'Bin editor');
  await expect(page.locator('canvas.viewer-canvas')).toBeVisible();
  expect(errors).toEqual([]);
});

test('projects survive a reload', async ({ page }) => {
  await page.goto('/');
  await expect(page.locator('.viewer')).toHaveAttribute('data-render-quality', 'high', {
    timeout: 30_000,
  });

  await page.locator('label').filter({ hasText: /^Project$/ }).first().click();
  await page.getByRole('button', { name: '+ New' }).click();
  await page.getByLabel('Project name').fill('Bench drawer');
  await page.getByLabel('Drawer width').fill('520');

  await page.reload();
  await page.locator('label').filter({ hasText: /^Project$/ }).first().click();
  await expect(page.getByLabel('Project name')).toHaveValue('Bench drawer');
  await expect(page.getByLabel('Drawer width')).toHaveValue('520');
});
