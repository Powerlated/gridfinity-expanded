import { defineConfig } from 'vite';

// Served from the custom domain's root (gridfinityexpanded.ashtonsouth.me),
// not the GitHub Pages project-page subpath — see public/CNAME. Asset URLs
// must therefore be root-relative in every build, CI or local.
export default defineConfig({
  base: '/',
});
