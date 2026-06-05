// gen-scene3d-video.mjs — render the 3D path/pose viewer to an mp4, offline.
//
// Serves the docs-site so a headless browser can load the same viewer module the
// docs island uses (scripts/scene3d-render.html), steps it frame-by-frame, screen-
// shots the canvas, and pipes the PNG sequence through ffmpeg. The video therefore
// matches the interactive web view exactly.
//
//   node docs-site/scripts/gen-scene3d-video.mjs --data <run.jsonl> --out <mp4> \
//        [--width 960] [--height 600] [--fps 20] [--keep]
//
// Needs: playwright (npm i -D playwright && npx playwright install chromium) and
// ffmpeg on PATH. Exits non-zero with a hint if either is missing.

import { createServer } from 'node:http';
import { readFile, mkdir, copyFile, rm, readdir } from 'node:fs/promises';
import { existsSync } from 'node:fs';
import { spawn } from 'node:child_process';
import { resolve, dirname, extname, join } from 'node:path';
import { fileURLToPath } from 'node:url';

const HERE = dirname(fileURLToPath(import.meta.url)); // docs-site/scripts
const ROOT = resolve(HERE, '..'); // docs-site (static-server root)

// ── args ────────────────────────────────────────────────────────────────────
const argv = process.argv.slice(2);
const opt = (name, def) => {
  const i = argv.indexOf(`--${name}`);
  return i >= 0 && i + 1 < argv.length ? argv[i + 1] : def;
};
const dataPath = opt('data');
const outPath = opt('out');
// Optional static scene (landmark cloud + camera). Defaults to scene.json next to
// the run.jsonl if present.
let scenePath = opt('scene');
if (!scenePath && dataPath) {
  const sibling = join(dirname(dataPath), 'scene.json');
  if (existsSync(sibling)) scenePath = sibling;
}
const W = Number(opt('width', 960));
const H = Number(opt('height', 600));
const FPS = Number(opt('fps', 20));
const KEEP = argv.includes('--keep');
if (!dataPath || !outPath) {
  console.error('usage: gen-scene3d-video.mjs --data <run.jsonl> --out <mp4> [--width --height --fps --keep]');
  process.exit(2);
}
if (!existsSync(dataPath)) {
  console.error(`gen-scene3d-video: --data not found: ${dataPath}`);
  process.exit(2);
}

const die = (m) => { console.error(`gen-scene3d-video: ${m}`); process.exit(1); };

// playwright is optional at install time — fail with a clear hint, not a stack.
let chromium;
try {
  ({ chromium } = await import('playwright'));
} catch {
  die('playwright not installed.\n  cd docs-site && npm i -D playwright && npx playwright install chromium');
}

// ── tiny static file server rooted at docs-site (ES-module MIME matters) ─────
const MIME = {
  '.html': 'text/html', '.js': 'text/javascript', '.mjs': 'text/javascript',
  '.json': 'application/json', '.jsonl': 'application/json', '.css': 'text/css',
  '.wasm': 'application/wasm', '.map': 'application/json',
};
const server = createServer(async (req, res) => {
  try {
    const urlPath = decodeURIComponent(new URL(req.url, 'http://x').pathname);
    const file = resolve(ROOT, '.' + urlPath);
    if (!file.startsWith(ROOT)) { res.writeHead(403).end(); return; } // no escaping root
    const body = await readFile(file);
    res.writeHead(200, { 'content-type': MIME[extname(file)] ?? 'application/octet-stream' });
    res.end(body);
  } catch {
    res.writeHead(404).end('not found');
  }
});
await new Promise((r) => server.listen(0, '127.0.0.1', r));
const port = server.address().port;
const base = `http://127.0.0.1:${port}`;

// Stage the run.jsonl inside the served root (the browser can only fetch from it).
const stageDir = join(HERE, '.render');
await mkdir(stageDir, { recursive: true });
const stagedData = join(stageDir, 'run.jsonl');
await copyFile(dataPath, stagedData);
const dataUrl = '/scripts/.render/run.jsonl';

let sceneUrl = '';
if (scenePath && existsSync(scenePath)) {
  await copyFile(scenePath, join(stageDir, 'scene.json'));
  sceneUrl = '/scripts/.render/scene.json';
}

// ── frames dir next to the output ────────────────────────────────────────────
const framesDir = join(stageDir, 'frames');
await rm(framesDir, { recursive: true, force: true });
await mkdir(framesDir, { recursive: true });

const log = (m) => console.log(`\x1b[1;36m==>\x1b[0m ${m}`);

let browser;
try {
  log('launching headless chromium (swiftshader WebGL)');
  browser = await chromium.launch({
    args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'],
  });
  const page = await browser.newPage({ viewport: { width: W, height: H }, deviceScaleFactor: 1 });
  page.on('console', (m) => { if (m.type() === 'error') console.error('  [page]', m.text()); });

  const sceneQ = sceneUrl ? `&scene=${encodeURIComponent(sceneUrl)}` : '';
  const url = `${base}/scripts/scene3d-render.html?data=${encodeURIComponent(dataUrl)}${sceneQ}&w=${W}&h=${H}`;
  await page.goto(url, { waitUntil: 'load' });
  await page.waitForFunction('window.__ready === true', { timeout: 30000 });
  const err = await page.evaluate('window.__error');
  if (err) die(`viewer failed to initialize: ${err}`);

  const n = await page.evaluate('window.__viewer.frameCount');
  if (!n || n < 1) die('viewer reports 0 frames — is run.jsonl empty / lacking est records?');
  log(`rendering ${n} frames at ${W}x${H}`);

  // Screenshot the stage (canvas + HUD + legend overlays), not the bare canvas,
  // so the metrics and over-confidence flag are captured in the video.
  const stage = page.locator('#stage');
  for (let i = 0; i < n; i++) {
    await page.evaluate((k) => window.__viewer.setFrame(k), i);
    // let the render loop paint the new frame (two rAFs = settled)
    await page.evaluate(() => new Promise((r) => requestAnimationFrame(() => requestAnimationFrame(r))));
    await stage.screenshot({ path: join(framesDir, `frame_${String(i).padStart(5, '0')}.png`) });
    if ((i + 1) % 25 === 0 || i + 1 === n) process.stdout.write(`\r    ${i + 1}/${n}`);
  }
  process.stdout.write('\n');
} finally {
  if (browser) await browser.close();
  server.close();
}

// ── encode ───────────────────────────────────────────────────────────────────
const frames = (await readdir(framesDir)).filter((f) => f.endsWith('.png'));
if (!frames.length) die('no frames captured');
await mkdir(dirname(resolve(outPath)), { recursive: true });
log(`encoding ${outPath} @ ${FPS}fps`);
await new Promise((res, rej) => {
  const ff = spawn('ffmpeg', [
    '-y', '-loglevel', 'error', '-framerate', String(FPS),
    '-i', join(framesDir, 'frame_%05d.png'),
    '-vf', 'pad=ceil(iw/2)*2:ceil(ih/2)*2', '-pix_fmt', 'yuv420p', resolve(outPath),
  ], { stdio: 'inherit' });
  // Don't block forever if ffmpeg stalls (corrupt frame / disk full): budget the
  // run time from the frame count plus a fixed buffer, then SIGKILL.
  const ms = (frames.length / FPS + 60) * 1000;
  const timer = setTimeout(() => { ff.kill('SIGKILL'); rej(new Error(`ffmpeg timed out after ${Math.ceil(ms / 1000)}s`)); }, ms);
  ff.on('error', (e) => { clearTimeout(timer); rej(new Error(`ffmpeg failed to start (${e.message}) — is it installed?`)); });
  ff.on('close', (c) => { clearTimeout(timer); c === 0 ? res() : rej(new Error(`ffmpeg exited ${c}`)); });
});

if (!KEEP) await rm(stageDir, { recursive: true, force: true });
log(`done → ${outPath}`);
