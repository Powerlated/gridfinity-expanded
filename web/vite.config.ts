import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';

// Served from the custom domain's root (gridfinityexpanded.ashtonsouth.me),
// not the GitHub Pages project-page subpath — see public/CNAME. Asset URLs
// must therefore be root-relative in every build, CI or local.
export default defineConfig({
  plugins: [react()],
  base: '/',
  worker: {
    format: 'es',
  },
});
