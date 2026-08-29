import { expect, test } from '@playwright/test';

async function waitForGeometry(page: import('@playwright/test').Page) {
  const viewer = page.locator('.viewer');
  await expect(viewer).toHaveAttribute('data-renderer', 'rust-webgl2', { timeout: 30_000 });
  await expect(viewer).toHaveAttribute('data-part-count', /[1-9]/, { timeout: 30_000 });
  await expect(page.getByRole('button', { name: 'Export X_T' })).toBeEnabled();
}

/**
 * How many solid bodies an XT file holds, read off its root node: the node at
 * index 1 is a single BODY for one part, or a POINTER_LIS_BLOCK whose
 * n_entries is the body count for several. Reading only the root keeps the spec
 * independent of every other node's field layout.
 */
function bodyCount(xt: string): number {
  const lines = xt.split('\n');
  const t = lines.findIndex((line) => line === 'T');
  expect(t, 'the format flag sequence is present').toBeGreaterThan(-1);
  const stream = lines.slice(t + 3).join('');
  expect(stream.startsWith('0 '), 'the node stream opens with the zero userfield size').toBe(true);
  expect(stream.endsWith('1 0'), 'the file ends at the 1 0 terminator').toBe(true);
  const tokens = stream.split(' ').filter(Boolean);
  const ty = Number(tokens[1]);
  if (ty === 12) {
    expect(tokens[2], 'a single-body file roots at the BODY node, index 1').toBe('1');
    return 1;
  }
  expect(ty, 'the root is a BODY or a body list').toBe(74);
  expect(tokens[3], 'a multi-body file roots at the list node, index 1').toBe('1');
  const count = Number(tokens[2]);
  expect(Number.isInteger(count) && count > 1).toBe(true);
  expect(Number(tokens[4]), 'the list carries its own entry count').toBe(count);
  expect(tokens[5], 'the list has no further blocks').toBe('0');
  return count;
}

test('exports the default design as one Parasolid body', async ({ page }) => {
  const errors: string[] = [];
  page.on('pageerror', (error) => errors.push(error.message));
  page.on('console', (message) => {
    if (message.type() === 'error') errors.push(message.text());
  });
  await page.goto('/');
  await waitForGeometry(page);

  const [download] = await Promise.all([
    page.waitForEvent('download'),
    page.getByRole('button', { name: 'Export X_T' }).click(),
  ]);
  expect(download.suggestedFilename()).toBe('gridfinity.x_t');
  const stream = await download.createReadableStream();
  const xt = await new Response(stream).text();

  expect(xt.startsWith('**'), 'the file opens with the XT header').toBe(true);
  expect(xt).toContain('PARASOLID');
  expect(xt).toContain('SCH_1200000_12006');
  expect(bodyCount(xt)).toBe(1);
  expect(errors).toEqual([]);
});

test('exports a split design as one body per piece', async ({ page }) => {
  const errors: string[] = [];
  page.on('pageerror', (error) => errors.push(error.message));
  await page.goto('/');
  await waitForGeometry(page);

  await page.getByRole('tab', { name: 'Cuts' }).click();
  await page.locator('.cut-line').first().click();
  const viewer = page.locator('.viewer');
  await expect(viewer).toHaveAttribute('data-part-count', '2', { timeout: 30_000 });
  await expect(page.getByRole('button', { name: 'Export X_T' })).toBeEnabled();

  const [download] = await Promise.all([
    page.waitForEvent('download'),
    page.getByRole('button', { name: 'Export X_T' }).click(),
  ]);
  expect(download.suggestedFilename()).toBe('gridfinity.x_t');
  const stream = await download.createReadableStream();
  const xt = await new Response(stream).text();

  expect(xt.startsWith('**')).toBe(true);
  expect(bodyCount(xt)).toBe(2);
  expect(errors).toEqual([]);
});
