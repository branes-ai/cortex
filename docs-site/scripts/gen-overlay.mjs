// Composite the VIO pipeline's per-frame scene + metrics into overlay frames —
// the "metrics on the scene video" view. Reads <dir>/frames.jsonl (written by
// tools/src/vio_pipeline.cpp --video) and emits one self-contained SVG per frame
// into <dir>/frames/ : the camera image (embedded), the live feature tracks (and,
// for the synthetic source, the true projections so the additive camera noise is
// visible as the red↔gray offset), and a HUD of the live metrics.
//
//   node docs-site/scripts/gen-overlay.mjs <dir>
//
// To assemble a video (needs an SVG rasterizer + ffmpeg), e.g.:
//   for f in <dir>/frames/*.svg; do rsvg-convert "$f" -o "${f%.svg}.png"; done
//   ffmpeg -framerate 20 -i <dir>/frames/frame_%05d.png -pix_fmt yuv420p scene.mp4

import { readFileSync, writeFileSync, mkdirSync, existsSync } from 'node:fs';
import { resolve, dirname, isAbsolute } from 'node:path';

const dir = resolve(process.argv[2] ?? 'build/pipeline');
const framesPath = resolve(dir, 'frames.jsonl');
if (!existsSync(framesPath)) {
  console.error(`no frames.jsonl in ${dir} — run vio_pipeline --video --out ${dir}`);
  process.exit(1);
}
const outDir = resolve(dir, 'frames');
mkdirSync(outDir, { recursive: true });

const W = 752, H = 480; // EuRoC cam0 / synthetic scene size
const esc = (s) => String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
const num = (v, d = 2) => (typeof v === 'number' ? v.toFixed(d) : '—');

// Embed one frame's image as a data URI. Not cached: every frame is a distinct
// image (a per-frame synthetic scene, or a unique EuRoC frame), so a cache would
// only accumulate — on a long EuRoC run that is thousands of base64 PNGs in memory.
function dataUri(imgRef) {
  const p = isAbsolute(imgRef) ? imgRef : resolve(dir, imgRef);
  try {
    return `data:image/png;base64,${readFileSync(p).toString('base64')}`;
  } catch {
    return ''; // missing image → just draw the overlay on black
  }
}

const lines = readFileSync(framesPath, 'utf8').trim().split('\n').filter(Boolean);
let count = 0;
for (const line of lines) {
  let r;
  try { r = JSON.parse(line); } catch { continue; }
  const uri = dataUri(r.image);

  let body = '';
  body += `<rect width="${W}" height="${H}" fill="#000"/>`;
  if (uri) body += `<image href="${uri}" x="0" y="0" width="${W}" height="${H}"/>`;

  // True projections (synthetic only) — faint gray; the noise is the offset to red.
  for (const p of r.true ?? [])
    body += `<circle cx="${p[0]}" cy="${p[1]}" r="2.5" fill="none" stroke="#9aa" stroke-width="0.8" opacity="0.7"/>`;
  // Live feature tracks the filter consumes — red.
  for (const p of r.obs ?? [])
    body += `<circle cx="${p[0]}" cy="${p[1]}" r="3" fill="none" stroke="#ff3b30" stroke-width="1.3"/>`;

  // Metrics HUD (top-left).
  const hud = [
    `frame ${r.frame}   t=${num(r.t)}s`,
    `features: ${r.nfeat ?? '—'}`,
    `NIS: ${num(r.nis)}  ${r.nis > 0 ? (r.nis > 1.5 ? '(over-confident)' : r.nis < 0.5 ? '(conservative)' : '(consistent)') : ''}`,
    r.pos_err != null ? `pos err: ${num(r.pos_err)} m` : null,
    r.noise != null ? `sensor noise: x${num(r.noise, 1)}` : null,
  ].filter(Boolean);
  const boxH = 16 + hud.length * 18;
  body += `<rect x="8" y="8" width="234" height="${boxH}" rx="5" fill="#000" opacity="0.55"/>`;
  hud.forEach((h, i) => {
    body += `<text x="18" y="${30 + i * 18}" font-family="monospace" font-size="14" fill="#eee">${esc(h)}</text>`;
  });
  // Legend.
  body += `<text x="${W - 8}" y="${H - 12}" text-anchor="end" font-family="monospace" font-size="11" fill="#ddd">red = tracked feature${(r.true ?? []).length ? '   gray = true (noise = offset)' : ''}</text>`;

  const svg = `<svg xmlns="http://www.w3.org/2000/svg" width="${W}" height="${H}" viewBox="0 0 ${W} ${H}">${body}</svg>`;
  const name = `frame_${String(r.frame).padStart(5, '0')}.svg`;
  writeFileSync(resolve(outDir, name), svg);
  ++count;
}
console.log(`wrote ${count} overlay frames to ${outDir}`);
console.log(`rasterize+assemble (needs rsvg-convert + ffmpeg):`);
console.log(`  for f in ${outDir}/*.svg; do rsvg-convert "$f" -o "\${f%.svg}.png"; done`);
console.log(`  ffmpeg -framerate 20 -i ${outDir}/frame_%05d.png -pix_fmt yuv420p ${dir}/scene_overlay.mp4`);
