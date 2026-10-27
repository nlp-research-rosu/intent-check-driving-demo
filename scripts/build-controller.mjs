import { spawnSync } from 'node:child_process';
import { mkdirSync, statSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const windows = process.platform === 'win32';
const source = path.resolve(process.argv[2] ?? path.join(root, 'controller', 'placeholder_controller.c'));
let output = path.resolve(process.argv[3] ?? path.join(root, 'build', 'controller'));
if (windows && !output.endsWith('.exe')) output += '.exe';

try {
  if (!statSync(source).isFile()) throw new Error('not a file');
} catch {
  console.error(`Controller source not found: ${source}`);
  process.exit(1);
}
mkdirSync(path.dirname(output), { recursive: true });

if (windows) {
  // Reuse the Windows compiler discovery, including Visual Studio's environment setup.
  // Reject characters interpreted by cmd even inside quoted arguments.
  if ([root, source, output].some((value) => /["%\r\n]/.test(value))) {
    console.error('Windows build paths cannot contain quotes, percent signs, or line breaks.');
    process.exit(1);
  }
  const batch = path.join(root, 'build_controller.bat');
  const result = spawnSync(process.env.ComSpec ?? 'cmd.exe', [
    '/d', '/s', '/c', `""${batch}" "${source}" "${output}""`,
  ], { cwd: root, stdio: 'inherit', windowsHide: true, windowsVerbatimArguments: true });
  if (result.error) console.error(result.error.message);
  process.exit(result.status ?? 1);
}

const candidates = process.env.CC ? [process.env.CC] : ['cc', 'gcc', 'clang'];
for (const compiler of candidates) {
  const result = spawnSync(compiler, [
    '-O2', '-Wall', '-Wextra', '-Werror', '-std=c11', '-D_POSIX_C_SOURCE=200809L',
    '-I', path.join(root, 'controller'), path.join(root, 'controller', 'controller_host.c'),
    source, '-o', output, '-lm',
  ], { cwd: root, stdio: 'inherit' });
  if (result.error?.code === 'ENOENT') continue;
  if (result.error) console.error(result.error.message);
  if (result.status === 0) console.log(`Built ${output}`);
  process.exit(result.status ?? 1);
}

console.error('No C compiler found. Install the Xcode Command Line Tools on macOS, or GCC/Clang on Linux.');
process.exit(1);
