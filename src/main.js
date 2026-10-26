// Entry point: keep a copy of the page so it can be saved as a stand-alone
// file, then start the application.

import { App } from './ui/app.js';
import WASM_B64 from 'virtual:wasm-kernels';
import { initKernels, base64ToBytes } from './wasm/kernels.js';

// in-page fallback jobs use the WebAssembly kernels too (async compile on the main thread)
initKernels(base64ToBytes(WASM_B64));

try { window.__DUBURI_SOURCE__ = '<!doctype html>\n' + document.documentElement.outerHTML; } catch { /* ignore */ }

function boot() {
  const root = document.getElementById('app');
  const app = new App(root);
  window.__duburi = app; // handy from the browser console
  app.start().catch((e) => {
    console.error(e);
    root.insertAdjacentHTML('beforeend', `<div class="boot-error"><b>The simulator could not start.</b><br>${String(e && e.message ? e.message : e).replace(/[<>&]/g, '')}<br><br>Use a recent Chrome, Edge or Firefox with WebGL2 enabled.</div>`);
  });
}
if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', boot);
else boot();
