import './egui.css';
import init from './wasm-egui/gridfinity-app.js';

/* Started from an async function rather than at the top level: a top-level
   await forces every bundler target to es2022, and the page's job is only to
   hand the canvas to the module. */
async function start() {
  const startup = document.getElementById('startup');
  try {
    await init();
    startup.remove();
  } catch (error) {
    startup.textContent = `Could not start Gridfinity CAD: ${error}`;
    startup.dataset.failed = 'true';
    throw error;
  }
}

start();
