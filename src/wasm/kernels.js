// Loader for the WebAssembly kernels (src/wasm/kernels.c). The bytes are
// handed in once per thread (the bundle embeds them; Node tests read the
// file). Workers compile synchronously; the main thread compiles async.
// Everything that uses the kernels falls back to plain JavaScript when they
// are not available.

let instance = null;
let pending = null;

const imports = { env: { log: Math.log } };

export function initKernels(bytes, { sync = false } = {}) {
  if (instance || !bytes) return instance;
  try {
    if (typeof WebAssembly === 'undefined') return null;
    if (sync) {
      instance = new WebAssembly.Instance(new WebAssembly.Module(bytes), imports);
      return instance;
    }
    pending = WebAssembly.instantiate(bytes, imports).then((r) => { instance = r.instance; return instance; }).catch((e) => { console.warn('WebAssembly kernels unavailable:', e.message); return null; });
    return pending;
  } catch (e) {
    console.warn('WebAssembly kernels unavailable:', e.message);
    return null;
  }
}
export function base64ToBytes(b64) {
  if (typeof atob === 'function') {
    const s = atob(b64);
    const u = new Uint8Array(s.length);
    for (let i = 0; i < s.length; i++) u[i] = s.charCodeAt(i);
    return u;
  }
  return new Uint8Array(Buffer.from(b64, 'base64'));
}
export function kernels() { return instance ? instance.exports : null; }
export function disableKernels() { instance = null; }
