// gen-init-figures.mjs — the FILTER-INTERNAL renderer for the S1 initialization
// inspector (tools/src/s1_inspect.cpp, issue #376). Reads a run's captured init
// and emits two self-contained SVGs:
//
//   init_state.svg      — the recovered initial state (attitude / velocity / biases
//                         / scale) with the error vs EuRoC ground truth: the
//                         gravity-direction (roll/pitch) error, speed error,
//                         gyro-bias error, and — on the dynamic path — scale error.
//   init_covariance.svg — the seeded initial covariance as a log|P| heatmap, with
//                         the θ/p/v/bg/ba IMU blocks labelled, surfacing whether
//                         the seed is an isotropic σ·I or honestly structured.
//
//   node docs-site/scripts/gen-init-figures.mjs <dir>
//
// Input (written by s1_inspect): <dir>/init.json
//   {method,t,attitude_deg:{roll,pitch,yaw},velocity:[3],gyro_bias:[3],accel_bias:[3],
//    scale,cov:[dim*dim],dim,sigma:{theta_deg,pos_m,vel_ms,bg,ba},isotropic_seed,
//    gravity_residual,have_gt,gravity_dir_error_deg,velocity_error_ms,gyro_bias_error,
//    accel_bias_error,scale_error_pct,...}

import { readFileSync, writeFileSync, existsSync } from 'node:fs';
import { resolve } from 'node:path';

const dir = resolve(process.argv[2] ?? 'build/stage_probes/S1');
const initPath = resolve(dir, 'init.json');
if (!existsSync(initPath)) {
  console.error(`no init.json in ${dir} — run s1_inspect --out ${dir} first`);
  process.exit(1);
}
let R;
try { R = JSON.parse(readFileSync(initPath, 'utf8')); } catch (e) { console.error(`init.json parse error: ${e.message}`); process.exit(1); }

const esc = (s) => String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
const num = (v, d = 3) => (typeof v === 'number' && Number.isFinite(v) ? v.toFixed(d) : '—');

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
const svgOpen = (w, h) => `<svg xmlns="http://www.w3.org/2000/svg" width="${w}" height="${h}" viewBox="0 0 ${w} ${h}"><rect width="${w}" height="${h}" fill="#0e1116"/>`;

// ── init_state.svg : recovered state + GT errors ──────────────────────────────
function renderState() {
  const W = 640, H = 420;
  let s = svgOpen(W, H);
  s += `<text x="24" y="30" font-family="monospace" font-size="15" fill="#eee">S1 initialization — recovered initial state (method: ${esc(R.method)}, t=${num(R.t, 2)}s)</text>`;

  const a = R.attitude_deg ?? {}, v = R.velocity ?? [0, 0, 0];
  const lines = [
    `attitude  roll ${num(a.roll, 2)}°  pitch ${num(a.pitch, 2)}°  yaw ${num(a.yaw, 2)}°`,
    `velocity  [${v.map((x) => num(x, 2)).join(', ')}] m/s   |v| ${num(Math.hypot(...v), 2)}`,
    `gyro bias [${(R.gyro_bias ?? []).map((x) => num(x, 4)).join(', ')}] rad/s`,
    `accel bias [${(R.accel_bias ?? []).map((x) => num(x, 4)).join(', ')}] m/s²`,
    `scale ${num(R.scale, 4)}   gravity-residual ${num(R.gravity_residual, 4)}`,
  ];
  lines.forEach((ln, i) => { s += `<text x="24" y="${64 + i * 22}" font-family="monospace" font-size="13" fill="#cde">${esc(ln)}</text>`; });

  // Error-vs-GT bar chart (each bar normalized to its own "concern" scale so they're comparable at a glance).
  const by = 200;
  s += `<text x="24" y="${by - 6}" font-family="monospace" font-size="13" fill="#eee">${R.have_gt ? 'error vs ground truth' : '(no ground truth available)'}</text>`;
  if (R.have_gt) {
    const bars = [
      { label: 'gravity dir', val: R.gravity_dir_error_deg, unit: '°', scale: 5 },     // 5° = full bar
      { label: 'speed', val: R.velocity_error_ms, unit: ' m/s', scale: 0.5 },
      { label: 'gyro bias', val: R.gyro_bias_error, unit: ' rad/s', scale: 0.02 },
      { label: 'accel bias', val: R.accel_bias_error, unit: ' m/s²', scale: 0.2 },
    ];
    if (R.method === 'dynamic') bars.push({ label: 'scale', val: R.scale_error_pct, unit: '%', scale: 10 });
    const bx = 150, bw = 360, bh = 16, gap = 12;
    bars.forEach((b, i) => {
      const y = by + 14 + i * (bh + gap);
      const frac = Math.min(1, (b.val ?? 0) / b.scale);
      s += `<text x="${bx - 8}" y="${y + bh - 3}" text-anchor="end" font-family="monospace" font-size="11" fill="#ddd">${b.label}</text>`;
      s += `<rect x="${bx}" y="${y}" width="${bw}" height="${bh}" fill="#000" stroke="#333"/>`;
      s += `<rect x="${bx}" y="${y}" width="${(frac * bw).toFixed(1)}" height="${bh}" fill="${ramp(frac)}"/>`;
      s += `<text x="${bx + bw + 8}" y="${y + bh - 3}" font-family="monospace" font-size="11" fill="#eee">${num(b.val, 3)}${b.unit}</text>`;
    });
    s += `<text x="24" y="${H - 16}" font-family="monospace" font-size="10" fill="#9ab">bar = error ÷ a per-quantity reference (full bar ≈ a large error); yaw is an unobservable gauge and is omitted</text>`;
  }
  return s + '</svg>';
}

// ── init_covariance.svg : the seeded covariance heatmap ───────────────────────
function renderCovariance() {
  const dim = R.dim | 0;
  const flat = R.cov;
  if (!dim || !Array.isArray(flat) || flat.length !== dim * dim) {
    console.error(`init.json: cov is not a ${dim}×${dim} matrix, skipping init_covariance.svg`);
    return null;
  }
  const at = (i, j) => flat[i * dim + j];

  let amax = -Infinity, amin = Infinity;
  for (const x of flat) { const v = Math.abs(x); if (v > 0) { if (v > amax) amax = v; if (v < amin) amin = v; } }
  if (!(amax > 0)) { console.error('init.json: covariance is all zero, skipping init_covariance.svg'); return null; }
  const lo = Math.log10(Math.max(amin, amax * 1e-8)), hi = Math.log10(amax);
  const col = (x) => { const v = Math.abs(x); if (!(v > 0)) return '#0a0a0a'; return ramp((Math.log10(v) - lo) / (hi - lo || 1)); };

  const cell = Math.max(10, Math.min(26, Math.floor(360 / dim)));
  const grid = dim * cell, ox = 150, oy = 70;
  const W = ox + grid + 150, H = oy + grid + 60;
  let s = svgOpen(W, H);
  s += `<text x="24" y="30" font-family="monospace" font-size="15" fill="#eee">S1 initial covariance — log|P| (${dim}×${dim})  ${R.isotropic_seed ? '· ISOTROPIC σ·I seed' : '· structured'}</text>`;

  for (let i = 0; i < dim; i++)
    for (let j = 0; j < dim; j++)
      s += `<rect x="${ox + j * cell}" y="${oy + i * cell}" width="${cell}" height="${cell}" fill="${col(at(i, j))}"/>`;

  // IMU sub-block separators + labels: θ[0:3] p[3:6] v[6:9] bg[9:12] ba[12:15].
  const blocks = [['θ', 0], ['p', 3], ['v', 6], ['b_g', 9], ['b_a', 12]];
  for (const [, o] of blocks) {
    if (o === 0 || o >= dim) continue;
    const p = o * cell;
    s += `<line x1="${ox + p}" y1="${oy}" x2="${ox + p}" y2="${oy + grid}" stroke="#fff" stroke-opacity="0.25" stroke-width="0.8"/>`;
    s += `<line x1="${ox}" y1="${oy + p}" x2="${ox + grid}" y2="${oy + p}" stroke="#fff" stroke-opacity="0.25" stroke-width="0.8"/>`;
  }
  for (const [name, o] of blocks) {
    if (o >= dim) continue;
    const c = ox + (o + 1.5) * cell;
    s += `<text x="${c}" y="${oy - 6}" text-anchor="middle" font-family="monospace" font-size="12" fill="#9cf">${name}</text>`;
    s += `<text x="${ox - 8}" y="${oy + (o + 1.5) * cell + 4}" text-anchor="end" font-family="monospace" font-size="12" fill="#9cf">${name}</text>`;
  }
  s += `<rect x="${ox}" y="${oy}" width="${grid}" height="${grid}" fill="none" stroke="#555"/>`;

  // colorbar
  const cbx = ox + grid + 30, cbw = 16;
  for (let k = 0; k < 100; k++) s += `<rect x="${cbx}" y="${oy + (grid * (99 - k)) / 100}" width="${cbw}" height="${grid / 100 + 1}" fill="${ramp(k / 99)}"/>`;
  s += `<rect x="${cbx}" y="${oy}" width="${cbw}" height="${grid}" fill="none" stroke="#888"/>`;
  s += `<text x="${cbx + cbw + 4}" y="${oy + 8}" font-family="monospace" font-size="9" fill="#ddd">1e${hi.toFixed(0)}</text>`;
  s += `<text x="${cbx + cbw + 4}" y="${oy + grid}" font-family="monospace" font-size="9" fill="#ddd">1e${lo.toFixed(0)}</text>`;
  const sig = R.sigma ?? {};
  s += `<text x="24" y="${H - 16}" font-family="monospace" font-size="10" fill="#9ab">σ: θ ${num(sig.theta_deg, 2)}° · p ${num(sig.pos_m, 2)} m · v ${num(sig.vel_ms, 2)} m/s · b_g ${num(sig.bg, 3)} · b_a ${num(sig.ba, 3)}</text>`;
  return s + '</svg>';
}

writeFileSync(resolve(dir, 'init_state.svg'), renderState());
const cov = renderCovariance();
if (cov) writeFileSync(resolve(dir, 'init_covariance.svg'), cov);
console.log(`wrote ${dir}/init_state.svg${cov ? ', init_covariance.svg' : ' (covariance skipped)'}`);
