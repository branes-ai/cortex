// gen-calibration-figures.mjs — the FILTER-INTERNAL renderer for the S10 online-
// calibration inspector (tools/src/s10_inspect.cpp, issue #384). Reads a run's
// captured convergence curve and emits:
//
//   calib_convergence.svg — the in-state camera↔IMU extrinsic estimate converging
//     toward the dataset's true calibration: extrinsic-rotation error (deg) and
//     translation error (mm) over time, each inside the filter's own ±3σ band.
//     Seeded from a deliberately wrong extrinsic, a healthy filter pulls the error
//     down toward 0 while the band shrinks — and the error should stay inside it.
//
//   node docs-site/scripts/gen-calibration-figures.mjs <dir>
//
// Input (written by s10_inspect): <dir>/calibration.json
//   {has_calib,ref_rot_deg_init,ref_trans_mm_init,
//    curve:[{t,rot_err_deg,trans_err_mm,rot_sigma_deg,trans_sigma_mm}]}

import { readFileSync, writeFileSync, existsSync } from 'node:fs';
import { resolve } from 'node:path';

const dir = resolve(process.argv[2] ?? 'build/stage_probes/S10');
const path = resolve(dir, 'calibration.json');
if (!existsSync(path)) {
  console.error(`no calibration.json in ${dir} — run s10_inspect --out ${dir} first`);
  process.exit(1);
}
let R;
try { R = JSON.parse(readFileSync(path, 'utf8')); } catch (e) { console.error(`calibration.json parse error: ${e.message}`); process.exit(1); }
const curve = Array.isArray(R.curve) ? R.curve : [];
if (curve.length < 2) { console.error('calibration.json: need ≥2 samples'); process.exit(1); }

const esc = (s) => String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
const num = (v, d = 2) => (typeof v === 'number' && Number.isFinite(v) ? v.toFixed(d) : '—');

const W = 760, H = 380, M = { l: 64, r: 20, t: 44, b: 40 };
const panelH = (H - M.t - M.b - 24) / 2, pw = W - M.l - M.r;
const n = curve.length;
const tmax = curve[n - 1].t || (n - 1) || 1;
const X = (t) => M.l + (t / tmax) * pw;

function panel(oy, title, errKey, sigKey, unit) {
  let ymax = 1e-9;
  for (const s of curve) ymax = Math.max(ymax, s[errKey] ?? 0, 3 * (s[sigKey] ?? 0));
  ymax *= 1.1;
  const Y = (y) => oy + panelH - (Math.min(y, ymax) / ymax) * panelH;
  let g = `<rect x="${M.l}" y="${oy}" width="${pw}" height="${panelH}" fill="#0a0a0a" stroke="#444"/>`;
  g += `<text x="${M.l}" y="${oy - 6}" font-family="monospace" font-size="12" fill="#cde">${esc(title)}</text>`;
  // ±3σ band (the filter's own uncertainty about the extrinsic), from 0 up.
  let env = `M${X(curve[0].t).toFixed(1)},${Y(3 * (curve[0][sigKey] ?? 0)).toFixed(1)}`;
  for (let k = 1; k < n; k++) env += `L${X(curve[k].t).toFixed(1)},${Y(3 * (curve[k][sigKey] ?? 0)).toFixed(1)}`;
  env += `L${X(curve[n - 1].t).toFixed(1)},${Y(0).toFixed(1)}L${X(curve[0].t).toFixed(1)},${Y(0).toFixed(1)}Z`;
  g += `<path d="${env}" fill="#4fd1ff" fill-opacity="0.12" stroke="#4fd1ff" stroke-opacity="0.5" stroke-width="1"/>`;
  // error curve (orange); red dot where the error escapes its own 3σ.
  let d = '';
  for (let k = 0; k < n; k++) d += `${k ? 'L' : 'M'}${X(curve[k].t).toFixed(1)},${Y(curve[k][errKey] ?? 0).toFixed(1)}`;
  g += `<path d="${d}" fill="none" stroke="#fd8d3c" stroke-width="1.6"/>`;
  for (let k = 0; k < n; k++) {
    const e = curve[k][errKey] ?? 0;
    if (e > 3 * (curve[k][sigKey] ?? 0)) g += `<circle cx="${X(curve[k].t).toFixed(1)}" cy="${Y(e).toFixed(1)}" r="2" fill="#e6552d"/>`;
  }
  g += `<line x1="${M.l}" y1="${Y(0).toFixed(1)}" x2="${M.l + pw}" y2="${Y(0).toFixed(1)}" stroke="#39d98a" stroke-width="1" stroke-dasharray="4 3"/>`;
  g += `<text x="${M.l - 6}" y="${(oy + 10).toFixed(1)}" text-anchor="end" font-family="monospace" font-size="9" fill="#bbb">${num(ymax, 1)}</text>`;
  g += `<text x="${M.l - 6}" y="${(oy + panelH).toFixed(1)}" text-anchor="end" font-family="monospace" font-size="9" fill="#bbb">0</text>`;
  g += `<text x="${M.l + pw - 4}" y="${(oy + 12).toFixed(1)}" text-anchor="end" font-family="monospace" font-size="10" fill="#ddd">${esc(unit)}: ${num(curve[0][errKey], 2)} → ${num(curve[n - 1][errKey], 2)}</text>`;
  return g;
}

let s = `<svg xmlns="http://www.w3.org/2000/svg" width="${W}" height="${H}" viewBox="0 0 ${W} ${H}"><rect width="${W}" height="${H}" fill="#0e1116"/>`;
s += `<text x="20" y="26" font-family="monospace" font-size="15" fill="#eee">S10 online-calibration convergence — extrinsic estimate vs the dataset reference (seeded ${num(R.ref_rot_deg_init, 1)}° / ${num(R.ref_trans_mm_init, 0)} mm off)</text>`;
s += panel(M.t + 14, 'extrinsic rotation error (orange) inside ±3σ (blue); green = reference (0)', 'rot_err_deg', 'rot_sigma_deg', 'deg');
s += panel(M.t + 14 + panelH + 24, 'extrinsic translation error inside ±3σ', 'trans_err_mm', 'trans_sigma_mm', 'mm');
s += `<text x="${M.l}" y="${H - 14}" font-family="monospace" font-size="10" fill="#9ab">time → ; error should decay toward the green reference and stay inside its own (shrinking) blue ±3σ band</text>`;
writeFileSync(resolve(dir, 'calib_convergence.svg'), s + '</svg>');
console.log(`wrote ${dir}/calib_convergence.svg`);
