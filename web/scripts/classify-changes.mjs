import { execFileSync } from 'node:child_process';

const [base, head] = process.argv.slice(2);
if (!base || !head) {
  console.error('Usage: node web/scripts/classify-changes.mjs <base> <head>');
  process.exit(2);
}

const ZERO_SHA = /^0+$/;
const diffArgs = ZERO_SHA.test(base)
  ? ['diff-tree', '--no-commit-id', '--name-only', '-r', head]
  : ['diff', '--name-only', `${base}...${head}`];
const paths = execFileSync('git', diffArgs, { encoding: 'utf8' })
  .split('\n')
  .filter(Boolean);

const documentation = /^(?:README(?:\.[^/]*)?|docs\/|\.agents\/|(?:[^/]+\/)*CLAUDE\.md)/;
const isolatedTest = /(?:^|\/)(?:__tests__\/.*|[^/]+\.(?:test|spec)\.[cm]?[jt]sx?)$/;
const playwrightOnly = /^web\/(?:e2e\/|playwright\.config\.[cm]?[jt]s$)/;
const geometry = /^(?:crates\/|Cargo\.(?:toml|lock)$|web\/src\/lib\/(?:geometry\/|project\/|cuts\.ts$|coordinates\.ts$|gridfinitySpec\.ts$|export\/stl\.ts$|types\.ts$)|web\/src\/workers\/geometry\.worker\.ts$|web\/src\/store\.ts$|web\/package(?:-lock)?\.json$)/;
const browserRuntime = /^web\/(?:src\/|public\/|index\.html$|package(?:-lock)?\.json$|vite\.config\.[cm]?[jt]s$|tsconfig(?:\.[^/]*)?\.json$|postcss\.config\.[cm]?[jt]s$)/;
const tooling = /^(?:\.github\/workflows\/|web\/scripts\/classify-changes\.mjs$|web\/vitest\.config\.[cm]?[jt]s$|\.gitignore$|LICENSE$)/;

let needsPlaywright = false;
let needsGeometry = false;

for (const path of paths) {
  if (documentation.test(path) || isolatedTest.test(path)) continue;
  if (playwrightOnly.test(path)) {
    needsPlaywright = true;
    continue;
  }
  if (geometry.test(path)) needsGeometry = true;
  if (browserRuntime.test(path)) needsPlaywright = true;
  if (!geometry.test(path) && !browserRuntime.test(path) && !tooling.test(path)) {
    needsPlaywright = true;
    needsGeometry = true;
  }
}

console.log(`playwright=${needsPlaywright}`);
console.log(`geometry=${needsGeometry}`);
