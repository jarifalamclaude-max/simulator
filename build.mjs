// Builds the whole simulator into one self-contained HTML file:
//   dist/duburi-sim.html     full document (open it directly in a browser)
//   dist/artifact.html       body fragment for hosting inside claude.ai
//   docs/index.html          the same page, served by GitHub Pages
// The worker is bundled first and inlined as a string (started from a blob).

import { build } from 'esbuild';
import { readFileSync, writeFileSync, mkdirSync } from 'node:fs';

const t0 = Date.now();
const common = { bundle: true, minify: true, platform: 'browser', target: ['es2020'], write: false, legalComments: 'none', loader: { '.param': 'text' }, logLevel: 'warning' };

// WebAssembly kernels (compiled from src/wasm/kernels.c by tools/build-wasm.mjs)
const wasmB64 = readFileSync('src/wasm/kernels.wasm').toString('base64');
const virtualWasm = {
  name: 'virtual-wasm',
  setup(b) {
    b.onResolve({ filter: /^virtual:wasm-kernels$/ }, () => ({ path: 'wasm-kernels', namespace: 'virtual-wasm' }));
    b.onLoad({ filter: /.*/, namespace: 'virtual-wasm' }, () => ({ contents: `export default ${JSON.stringify(wasmB64)};`, loader: 'js' }));
  },
};
const worker = await build({ ...common, entryPoints: ['src/worker/worker.js'], format: 'iife', plugins: [virtualWasm] });
const workerCode = worker.outputFiles[0].text;

const virtualWorker = {
  name: 'virtual-worker',
  setup(b) {
    b.onResolve({ filter: /^virtual:worker-source$/ }, () => ({ path: 'worker-source', namespace: 'virtual' }));
    b.onLoad({ filter: /.*/, namespace: 'virtual' }, () => ({ contents: `export default ${JSON.stringify(workerCode)};`, loader: 'js' }));
  },
};
const main = await build({ ...common, entryPoints: ['src/main.js'], format: 'iife', plugins: [virtualWorker, virtualWasm] });
const js = main.outputFiles[0].text.replace(/<\/script/gi, '<\\/script');
const css = readFileSync('src/ui/styles.css', 'utf8');
const version = JSON.parse(readFileSync('package.json', 'utf8')).version;

const fonts = 'https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500&family=IBM+Plex+Sans:wght@400;500;600&family=Saira+Semi+Condensed:wght@500;600;700&display=swap';
const body = `<title>DUBURI Simulator</title>
<meta name="description" content="DUBURI ROV simulator: ArduSub 4.5 control, 6-DOF hydrodynamics, BEM and lattice-Boltzmann CFD, gamepad flying and tuning.">
<meta name="generator" content="DUBURI Simulator ${version}">
<link rel="preconnect" href="https://fonts.googleapis.com"><link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" href="${fonts}">
<style>${css}</style>
<div id="app"><div class="boot">DUBURI · loading simulator…</div></div>
<noscript><p style="padding:24px;color:#e3eef3">The DUBURI simulator needs JavaScript and WebGL2.</p></noscript>
<script>${js}</script>
`;
const favicon = 'data:image/svg+xml,' + encodeURIComponent('<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 32 32"><rect width="32" height="32" rx="7" fill="#06131b"/><text x="16" y="23" font-family="Arial,sans-serif" font-weight="700" font-size="19" text-anchor="middle" fill="#ffb547">D</text></svg>');
const full = `<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<link rel="icon" href="${favicon}">
</head>
<body>
${body}</body>
</html>
`;
mkdirSync('dist', { recursive: true });
writeFileSync('dist/duburi-sim.html', full);
writeFileSync('dist/artifact.html', body);
// GitHub Pages (Settings > Pages > Deploy from a branch: main, /docs)
mkdirSync('docs', { recursive: true });
writeFileSync('docs/index.html', full);
writeFileSync('docs/.nojekyll', '');
console.log(`dist/duburi-sim.html ${(full.length / 1024).toFixed(0)} KB (worker ${(workerCode.length / 1024).toFixed(0)} KB) in ${Date.now() - t0} ms`);
