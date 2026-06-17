// gen-propagation-figures.mjs — the FILTER-INTERNAL renderer for the S2 IMU-
// propagation inspector (tools/src/s2_inspect.cpp, issue #377). Reads a run's
// per-step propagation record and emits two self-contained SVGs:
//
//   cov_growth.svg  — the covariance GROWTH over the window: a 15-error-state
//                     σ-vs-time heatmap (each row normalized to its own growth so
//                     the shape reads regardless of unit), with the θ/p/v/bg/ba
//                     blocks labelled, plus the pos/vel/att block-σ start→end HUD.
//   pose_drift.svg  — the propagated-vs-GT drift against the filter's own growing
//                     ±3σ envelope: position (mm) and attitude (deg). Drift that
//                     escapes the envelope means the propagated covariance
//                     under-covers the real dead-reckoning error (Q too small —
//                     the #212 candidate).
//
//   node docs-site/scripts/gen-propagation-figures.mjs <dir>
//
// Input (written by s2_inspect): <dir>/propagation.json
//   {duration_s,rate_hz,dim,have_gt,steps:[{t,pos_sigma_mm,vel_sigma_mm_s,
//    att_sigma_deg,diag_sigma:[15],p,v,q,pos_drift_mm,att_drift_deg,...}]}

import { readFileSync, writeFileSync, existsSync } from 'node:fs';
import { resolve } from 'node:path';

const dir = resolve(process.argv[2] ?? 'build/stage_probes/S2');
const path = resolve(dir, 'propagation.json');
if (!existsSync(path)) {
  console.error(`no propagation.json in ${dir} — run s2_inspect --out ${dir} first`);
  process.exit(1);
}
let R;
try { R = JSON.parse(readFileSync(path, 'utf8')); } catch (e) { console.error(`propagation.json parse error: ${e.message}`); process.exit(1); }
const steps = Array.isArray(R.steps) ? R.steps : [];
if (steps.length < 2) { console.error('propagation.json: need ≥2 steps'); process.exit(1); }

const esc = (s) => String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
const num = (v, d = 2) => (typeof v === 'number' && Number.isFinite(v) ? v.toFixed(d) : '—');
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

// ── cov_growth.svg : 15-state σ-vs-time heatmap (row-normalized) ───────────────
function renderGrowth() {
  const n = steps.length;
  const dim = 15;
  // labels for the 15 IMU error states
  const labels = ['θx', 'θy', 'θz', 'px', 'py', 'pz', 'vx', 'vy', 'vz', 'bgx', 'bgy', 'bgz', 'bax', 'bay', 'baz'];
  const valid = steps.every((s) => Array.isArray(s.diag_sigma) && s.diag_sigma.length >= dim);
  if (!valid) { console.error('propagation.json: steps lack a 15-wide diag_sigma'); return null; }
  // per-row max over time (for normalization)
  const rowMax = new Array(dim).fill(0);
  for (const s of steps) for (let i = 0; i < dim; i++) rowMax[i] = Math.max(rowMax[i], s.diag_sigma[i]);

  const cw = Math.max(2, Math.min(8, Math.floor(560 / n)));
  const rh = 16, ox = 56, oy = 54, gw = n * cw, gh = dim * rh;
  const W = ox + gw + 90, H = oy + gh + 56;
  let s = svgOpen(W, H);
  s += `<text x="20" y="28" font-family="monospace" font-size="15" fill="#eee">S2 covariance growth — per-state σ over the window (${num(R.duration_s, 2)}s, ${num(R.rate_hz, 0)} Hz; each row normalized)</text>`;

  for (let i = 0; i < dim; i++) {
    const mx = rowMax[i] > 0 ? rowMax[i] : 1;
    for (let k = 0; k < n; k++)
      s += `<rect x="${ox + k * cw}" y="${oy + i * rh}" width="${cw}" height="${rh}" fill="${ramp(steps[k].diag_sigma[i] / mx)}"/>`;
    s += `<text x="${ox - 6}" y="${oy + i * rh + rh - 4}" text-anchor="end" font-family="monospace" font-size="10" fill="#bbb">${labels[i]}</text>`;
  }
  // block separators after θ(3), p(6), v(9), bg(12)
  for (const b of [3, 6, 9, 12]) s += `<line x1="${ox}" y1="${oy + b * rh}" x2="${ox + gw}" y2="${oy + b * rh}" stroke="#fff" stroke-opacity="0.22" stroke-width="0.8"/>`;
  s += `<rect x="${ox}" y="${oy}" width="${gw}" height="${gh}" fill="none" stroke="#555"/>`;
  s += `<text x="${ox + gw / 2}" y="${oy + gh + 18}" text-anchor="middle" font-family="monospace" font-size="11" fill="#ddd">time → (${n} steps)</text>`;

  // start→end block σ HUD
  const f = steps[0], l = steps[n - 1];
  const hud = [
    `pos σ ${num(f.pos_sigma_mm)} → ${num(l.pos_sigma_mm)} mm`,
    `vel σ ${num(f.vel_sigma_mm_s)} → ${num(l.vel_sigma_mm_s)} mm/s`,
    `att σ ${num(f.att_sigma_deg)} → ${num(l.att_sigma_deg)} deg`,
  ];
  hud.forEach((line, i) => { s += `<text x="${ox + gw + 12}" y="${oy + 14 + i * 18}" font-family="monospace" font-size="11" fill="#cde">${esc(line)}</text>`; });
  return s + '</svg>';
}

// ── pose_drift.svg : drift vs the ±3σ envelope (position + attitude) ───────────
function renderDrift() {
  if (!R.have_gt) {
    let s = svgOpen(560, 120);
    s += `<text x="20" y="60" font-family="monospace" font-size="13" fill="#9ab">S2 pose drift — no ground truth in this run (skipped)</text>`;
    return s + '</svg>';
  }
  const n = steps.length;
  const W = 760, H = 380, M = { l: 64, r: 20, t: 44, b: 40 };
  const panelH = (H - M.t - M.b - 24) / 2, pw = W - M.l - M.r;
  const tmax = steps[n - 1].t || 1;
  const X = (t) => M.l + (t / tmax) * pw;
  let s = svgOpen(W, H);
  s += `<text x="20" y="26" font-family="monospace" font-size="15" fill="#eee">S2 propagated-vs-GT drift inside the ±3σ envelope</text>`;

  function panel(oy, title, driftKey, sigmaKey, unit) {
    let g = '';
    let ymax = 1e-9;
    for (const st of steps) ymax = Math.max(ymax, st[driftKey] ?? 0, 3 * (st[sigmaKey] ?? 0));
    ymax *= 1.1;
    const Y = (y) => oy + panelH - (Math.min(y, ymax) / ymax) * panelH;
    g += `<rect x="${M.l}" y="${oy}" width="${pw}" height="${panelH}" fill="#0a0a0a" stroke="#444"/>`;
    g += `<text x="${M.l}" y="${oy - 6}" font-family="monospace" font-size="12" fill="#cde">${esc(title)}</text>`;
    // 3σ envelope (filled) + its boundary
    let env = `M${X(steps[0].t).toFixed(1)},${Y(3 * (steps[0][sigmaKey] ?? 0)).toFixed(1)}`;
    for (let k = 1; k < n; k++) env += `L${X(steps[k].t).toFixed(1)},${Y(3 * (steps[k][sigmaKey] ?? 0)).toFixed(1)}`;
    env += `L${X(steps[n - 1].t).toFixed(1)},${Y(0).toFixed(1)}L${X(steps[0].t).toFixed(1)},${Y(0).toFixed(1)}Z`;
    g += `<path d="${env}" fill="#39d98a" fill-opacity="0.12" stroke="#39d98a" stroke-opacity="0.5" stroke-width="1"/>`;
    // drift line, points colored red where they escape 3σ
    let d = '';
    for (let k = 0; k < n; k++) d += `${k ? 'L' : 'M'}${X(steps[k].t).toFixed(1)},${Y(steps[k][driftKey] ?? 0).toFixed(1)}`;
    g += `<path d="${d}" fill="none" stroke="#fd8d3c" stroke-width="1.6"/>`;
    for (let k = 0; k < n; k++) {
      const dval = steps[k][driftKey] ?? 0;
      if (dval > 3 * (steps[k][sigmaKey] ?? 0))
        g += `<circle cx="${X(steps[k].t).toFixed(1)}" cy="${Y(dval).toFixed(1)}" r="2" fill="#e6552d"/>`;
    }
    g += `<text x="${M.l - 6}" y="${(oy + 10).toFixed(1)}" text-anchor="end" font-family="monospace" font-size="9" fill="#bbb">${num(ymax, 1)}</text>`;
    g += `<text x="${M.l - 6}" y="${(oy + panelH).toFixed(1)}" text-anchor="end" font-family="monospace" font-size="9" fill="#bbb">0</text>`;
    g += `<text x="${M.l + pw - 4}" y="${(oy + panelH - 6).toFixed(1)}" text-anchor="end" font-family="monospace" font-size="10" fill="#ddd">${unit}</text>`;
    return g;
  }
  s += panel(M.t + 14, 'position drift (orange) vs 3σ (green)', 'pos_drift_mm', 'pos_sigma_mm', 'mm');
  s += panel(M.t + 14 + panelH + 24, 'attitude drift vs 3σ', 'att_drift_deg', 'att_sigma_deg', 'deg');
  s += `<text x="${M.l}" y="${H - 14}" font-family="monospace" font-size="10" fill="#9ab">time → ; red dots = drift escapes the 3σ envelope (covariance under-covers the dead-reckoning error)</text>`;
  return s + '</svg>';
}

const g = renderGrowth();
if (g) writeFileSync(resolve(dir, 'cov_growth.svg'), g);
writeFileSync(resolve(dir, 'pose_drift.svg'), renderDrift());
console.log(`wrote ${dir}/pose_drift.svg${g ? ', cov_growth.svg' : ' (cov_growth skipped)'}`);
