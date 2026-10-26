// Compiles src/wasm/kernels.c to WebAssembly (needs clang with the wasm32
// target and wasm-ld). The .wasm is committed, so the normal build does not
// need a C toolchain.
import { execFileSync } from 'node:child_process';
import { statSync } from 'node:fs';
execFileSync('clang', ['--target=wasm32', '-O3', '-msimd128', '-mbulk-memory', '-fno-math-errno', '-nostdlib', '-Wl,--no-entry', '-Wl,--allow-undefined', '-Wl,--initial-memory=1310720', '-Wl,--max-memory=4294967296', '-Wl,--stack-first', '-Wl,-z,stack-size=1048576', '-o', 'src/wasm/kernels.wasm', 'src/wasm/kernels.c'], { stdio: 'inherit' });
console.log(`src/wasm/kernels.wasm ${(statSync('src/wasm/kernels.wasm').size / 1024).toFixed(1)} KB`);
