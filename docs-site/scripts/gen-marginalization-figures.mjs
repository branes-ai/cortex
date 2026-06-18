// gen-marginalization-figures.mjs — the FILTER-INTERNAL renderer for the S9
// marginalization inspector (tools/src/s9_inspect.cpp, issue #383). The inverse of
// gen-augmentation-figures.mjs. Reads a run's captured marginalization and emits:
//
//   marginalize_covariance.svg — the covariance before and after the clone drop,
//     as side-by-side log|P| heatmaps, with the DROPPED block outlined in the
//     "before" matrix. Marginalization is exact principal-submatrix extraction, so
//     the kept-state marginal is unchanged (HUD residual ~0); what is discarded is
//     the dropped clone's own uncertainty (σ) and its cross-covariance with the
//     kept states (the outlined row/column bands).
//
//   node docs-site/scripts/gen-marginalization-figures.mjs <dir>
//
// Input (written by s9_inspect): <dir>/marginalization.json
//   {dim_before,dim_after,dropped_offset,clone_dim,dropped_index,kept_marginal_err,
//    dropped_sigma,max_cross_dropped,psd,n_clones_before,n_clones_after,
//    cov_before:[dim_before²],cov_after:[dim_after²]}

import { readFileSync, writeFileSync, existsSync } from 'node:fs';
import { resolve } from 'node:path';

const dir = resolve(process.argv[2] ?? 'build/stage_probes/S9');
const path = resolve(dir, 'marginalization.json');
if (!existsSync(path)) {
  console.error(`no marginalization.json in ${dir} — run s9_inspect --out ${dir} first`);
  process.exit(1);
}
let R;
try { R = JSON.parse(readFileSync(path, 'utf8')); } catch (e) { console.error(`marginalization.json parse error: ${e.message}`); process.exit(1); }

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
if (!Array.isArray(A) || !Array.isArray(B) || A.length !== db * db || B.length !== da * da || db < 1) {
  console.error(`marginalization.json: cov_before/after are not ${db}²/${da}² matrices`);
  process.exit(1);
}
const at = (M, dim, i, j) => M[i * dim + j];

let amax = -Infinity, amin = Infinity;
for (const M of [A, B]) for (const v of M) { const a = Math.abs(v); if (a > 0) { if (a > amax) amax = a; if (a < amin) amin = a; } }
if (!(amax > 0)) { console.error('marginalization.json: covariance is all zero'); process.exit(1); }
const lo = Math.log10(Math.max(amin, amax * 1e-8)), hi = Math.log10(amax);
const col = (v) => { const a = Math.abs(v); if (!(a > 0)) return '#0a0a0a'; return ramp((Math.log10(a) - lo) / (hi - lo || 1)); };

const cell = Math.max(4, Math.min(14, Math.floor(300 / Math.max(db, 1))));
const gridA = db * cell, gridB = da * cell, oy = 64, ox = 40, gap = 56;
const W = ox + gridA + gap + gridB + 110, H = oy + gridA + 78;

function heat(M, dim, ox0, title) {
  let s = `<text x="${ox0}" y="${oy - 8}" font-family="monospace" font-size="13" fill="#eee">${esc(title)} (${dim}×${dim})</text>`;
  for (let i = 0; i < dim; i++)
    for (let j = 0; j < dim; j++)
      s += `<rect x="${ox0 + j * cell}" y="${oy + i * cell}" width="${cell}" height="${cell}" fill="${col(at(M, dim, i, j))}"/>`;
  if (dim > 15) {
    const p = 15 * cell;
    s += `<line x1="${ox0 + p}" y1="${oy}" x2="${ox0 + p}" y2="${oy + dim * cell}" stroke="#fff" stroke-opacity="0.18" stroke-width="0.6"/>`;
    s += `<line x1="${ox0}" y1="${oy + p}" x2="${ox0 + dim * cell}" y2="${oy + p}" stroke="#fff" stroke-opacity="0.18" stroke-width="0.6"/>`;
  }
  s += `<rect x="${ox0}" y="${oy}" width="${dim * cell}" height="${dim * cell}" fill="none" stroke="#555"/>`;
  return s;
}

let s = `<svg xmlns="http://www.w3.org/2000/svg" width="${W}" height="${H}" viewBox="0 0 ${W} ${H}"><rect width="${W}" height="${H}" fill="#0e1116"/>`;
s += `<text x="${ox}" y="28" font-family="monospace" font-size="15" fill="#eee">S9 marginalization — covariance before / after the clone drop (log|P|)</text>`;
s += heat(A, db, ox, 'before');
const oxB = ox + gridA + gap;
s += heat(B, da, oxB, 'after');

// Outline the DROPPED clone block in the "before" matrix (rows/cols [off, off+cd)).
const off = R.dropped_offset | 0, cd = (R.clone_dim ?? 6) | 0;
if (off + cd <= db) {
  const bx = ox + off * cell, by = oy + off * cell, bs = cd * cell;
  s += `<rect x="${ox}" y="${by}" width="${gridA}" height="${bs}" fill="none" stroke="#e6552d" stroke-opacity="0.5" stroke-width="1" stroke-dasharray="3 2"/>`;
  s += `<rect x="${bx}" y="${oy}" width="${bs}" height="${gridA}" fill="none" stroke="#e6552d" stroke-opacity="0.5" stroke-width="1" stroke-dasharray="3 2"/>`;
  s += `<rect x="${bx}" y="${by}" width="${bs}" height="${bs}" fill="none" stroke="#e6552d" stroke-width="2"/>`;
  s += `<text x="${bx + bs + 4}" y="${by + bs / 2}" font-family="monospace" font-size="10" fill="#e6552d">dropped</text>`;
}

const hud = [
  `dim ${db} → ${da}   (dropped clone ${R.dropped_index ?? 0} at offset ${off})`,
  `kept-marginal err ${(R.kept_marginal_err ?? 0).toExponential(1)}  (others' marginal UNCHANGED)`,
  `dropped σ ${(R.dropped_sigma ?? 0).toExponential(2)}, max cross-cov ${(R.max_cross_dropped ?? 0).toExponential(2)}  (info discarded)`,
  `PSD ${R.psd ? 'yes' : 'NO'}   clones ${R.n_clones_before ?? '—'} → ${R.n_clones_after ?? '—'}`,
];
hud.forEach((line, i) => { s += `<text x="${ox}" y="${oy + gridA + 22 + i * 15}" font-family="monospace" font-size="11" fill="#cde">${esc(line)}</text>`; });

writeFileSync(resolve(dir, 'marginalize_covariance.svg'), s + '</svg>');
console.log(`wrote ${dir}/marginalize_covariance.svg`);
