// gen-augmentation-figures.mjs — the FILTER-INTERNAL renderer for the S3 state-
// augmentation inspector (tools/src/s3_inspect.cpp, issue #378). Reads a run's
// captured augmentation and emits one self-contained SVG:
//
//   augment_covariance.svg — the covariance before and after the clone, as
//     side-by-side log|P| heatmaps, with the NEW clone block outlined in the
//     "after" matrix. The new block is a deterministic copy of the IMU pose, so
//     its marginal and its cross-covariance with the existing states equal the
//     pose's (reported in the HUD as ~0 residuals); the highlight makes the
//     stochastic clone literally appear.
//
//   node docs-site/scripts/gen-augmentation-figures.mjs <dir>
//
// Input (written by s3_inspect): <dir>/augmentation.json
//   {dim_before,dim_after,clone_offset,clone_dim,clone_marginal_err,clone_cross_err,
//    psd,n_clones_after,cov_before:[dim_before²],cov_after:[dim_after²]}

import { readFileSync, writeFileSync, existsSync } from 'node:fs';
import { resolve } from 'node:path';

const dir = resolve(process.argv[2] ?? 'build/stage_probes/S3');
const path = resolve(dir, 'augmentation.json');
if (!existsSync(path)) {
  console.error(`no augmentation.json in ${dir} — run s3_inspect --out ${dir} first`);
  process.exit(1);
}
let R;
try { R = JSON.parse(readFileSync(path, 'utf8')); } catch (e) { console.error(`augmentation.json parse error: ${e.message}`); process.exit(1); }

const esc = (s) => String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
function ramp(t) {
  t = Math.max(0, Math.min(1, t));
  const stops = [[0, [13, 8, 135]], [0.35, [42, 130, 180]], [0.6, [60, 170, 100]], [0.8, [240, 200, 60]], [1, [200, 30, 30]]];
  for (let i = 1; i < stops.length; i++)
    if (t <= stops[i][0]) {
      const [a0, c0] = stops[i - 1], [a1, c1] = stops[i], f = (t - a0) / (a1 - a0 || 1);
      const c = c0.map((v, k) => Math.round(v + f * (c1[k] - v)));
      return `rgb(${c[0]},${c[1]},${c[2]})`;
    }
  return 'rgb(200,30,30)';
}

const db = R.dim_before | 0, da = R.dim_after | 0;
const A = R.cov_before, B = R.cov_after;
if (!Array.isArray(A) || !Array.isArray(B) || A.length !== db * db || B.length !== da * da || da < 1) {
  console.error(`augmentation.json: cov_before/after are not ${db}²/${da}² matrices`);
  process.exit(1);
}
const at = (M, dim, i, j) => M[i * dim + j];

// Shared log scale over both matrices.
let amax = -Infinity, amin = Infinity;
for (const M of [A, B]) for (const v of M) { const a = Math.abs(v); if (a > 0) { if (a > amax) amax = a; if (a < amin) amin = a; } }
if (!(amax > 0)) { console.error('augmentation.json: covariance is all zero'); process.exit(1); }
const lo = Math.log10(Math.max(amin, amax * 1e-8)), hi = Math.log10(amax);
const col = (v) => { const a = Math.abs(v); if (!(a > 0)) return '#0a0a0a'; return ramp((Math.log10(a) - lo) / (hi - lo || 1)); };

const cell = Math.max(4, Math.min(14, Math.floor(300 / da)));
const gridA = db * cell, gridB = da * cell, oy = 64, ox = 40, gap = 56;
const W = ox + gridA + gap + gridB + 110, H = oy + gridB + 70;

function heat(M, dim, ox0, title) {
  let s = `<text x="${ox0}" y="${oy - 8}" font-family="monospace" font-size="13" fill="#eee">${esc(title)} (${dim}×${dim})</text>`;
  for (let i = 0; i < dim; i++)
    for (let j = 0; j < dim; j++)
      s += `<rect x="${ox0 + j * cell}" y="${oy + i * cell}" width="${cell}" height="${cell}" fill="${col(at(M, dim, i, j))}"/>`;
  // IMU block separator at 15
  if (dim > 15) {
    const p = 15 * cell;
    s += `<line x1="${ox0 + p}" y1="${oy}" x2="${ox0 + p}" y2="${oy + dim * cell}" stroke="#fff" stroke-opacity="0.18" stroke-width="0.6"/>`;
    s += `<line x1="${ox0}" y1="${oy + p}" x2="${ox0 + dim * cell}" y2="${oy + p}" stroke="#fff" stroke-opacity="0.18" stroke-width="0.6"/>`;
  }
  s += `<rect x="${ox0}" y="${oy}" width="${dim * cell}" height="${dim * cell}" fill="none" stroke="#555"/>`;
  return s;
}

let s = `<svg xmlns="http://www.w3.org/2000/svg" width="${W}" height="${H}" viewBox="0 0 ${W} ${H}"><rect width="${W}" height="${H}" fill="#0e1116"/>`;
s += `<text x="${ox}" y="28" font-family="monospace" font-size="15" fill="#eee">S3 state augmentation — covariance before / after the clone (log|P|)</text>`;
s += heat(A, db, ox, 'before');
const oxB = ox + gridA + gap;
s += heat(B, da, oxB, 'after');

// Outline the new clone block in the "after" matrix (rows/cols [off, off+cd)).
const off = R.clone_offset | 0, cd = R.clone_dim | 6;
const bx = oxB + off * cell, by = oy + off * cell, bs = cd * cell;
// the new block's row band and column band (cross-covariance), then the diagonal block
s += `<rect x="${oxB}" y="${by}" width="${da * cell}" height="${bs}" fill="none" stroke="#ffd60a" stroke-opacity="0.5" stroke-width="1" stroke-dasharray="3 2"/>`;
s += `<rect x="${bx}" y="${oy}" width="${bs}" height="${da * cell}" fill="none" stroke="#ffd60a" stroke-opacity="0.5" stroke-width="1" stroke-dasharray="3 2"/>`;
s += `<rect x="${bx}" y="${by}" width="${bs}" height="${bs}" fill="none" stroke="#ffd60a" stroke-width="2"/>`;
s += `<text x="${bx + bs + 4}" y="${by + bs / 2}" font-family="monospace" font-size="10" fill="#ffd60a">new clone</text>`;

// HUD.
const hud = [
  `dim ${db} → ${da}   (+${cd} clone block at ${off})`,
  `clone marginal err ${(R.clone_marginal_err ?? 0).toExponential(1)}  (= pose marginal)`,
  `clone cross err ${(R.clone_cross_err ?? 0).toExponential(1)}  (= pose cross-cov)`,
  `PSD ${R.psd ? 'yes' : 'NO'}   clones now ${R.n_clones_after ?? '—'}`,
];
hud.forEach((line, i) => { s += `<text x="${ox}" y="${oy + gridB + 22 + i * 15}" font-family="monospace" font-size="11" fill="#cde">${esc(line)}</text>`; });

writeFileSync(resolve(dir, 'augment_covariance.svg'), s + '</svg>');
console.log(`wrote ${dir}/augment_covariance.svg`);
