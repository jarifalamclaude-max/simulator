// Compares the WebGL2 lattice-Boltzmann solver with the CPU reference in a
// headless Chromium (software GPU). Usage: node tools/gpu-check.mjs
import { build } from 'esbuild';
import { createRequire } from 'node:module';
const require = createRequire(import.meta.url);
const { chromium } = require(process.env.PLAYWRIGHT_PATH || 'playwright');
const out = await build({ entryPoints: ['tools/gpu-check-entry.js'], bundle: true, format: 'iife', write: false, platform: 'browser' });
const browser = await chromium.launch({ executablePath: process.env.CHROME_PATH || undefined, args: ['--use-gl=swiftshader', '--enable-webgl', '--ignore-gpu-blocklist', '--enable-unsafe-swiftshader'] });
const page = await browser.newPage();
page.on('console', (m) => { if (!/GPU stall/.test(m.text())) console.log('page:', m.text()); });
await page.setContent('<!doctype html><html><body></body></html>');
await page.addScriptTag({ content: out.outputFiles[0].text });
for (const [rotate, interp] of [[false, false], [true, false], [false, true], [true, true]]) {
  const r = await page.evaluate(([rot, itp]) => window.gpuCheck({ steps: 80, rotate: rot, interp: itp }), [rotate, interp]);
  console.log(JSON.stringify({ interp: r.interp, rotate, F: r.F && r.F.rel, M: r.M && r.M.rel, state: r.state, gpuMs: r.gpuMs, reason: r.reason }));
  if (!r.ok) process.exitCode = 1;
  else if (r.F.rel > 2e-3 || r.M.rel > 5e-3 || r.state.maxDiff > 1e-5) { console.error('GPU and CPU solvers disagree'); process.exitCode = 1; }
}
await browser.close();
