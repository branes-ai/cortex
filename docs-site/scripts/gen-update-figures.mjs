// gen-update-figures.mjs — the FILTER-INTERNAL renderer tier for the S6 MSCKF-update
// inspector (tools/src/s6_inspect.cpp, issue #380). Reads a run's per-update records
// and the one dumped covariance, and emits three self-contained SVGs:
//
//   nis.svg         — NIS/dof over updates, with the χ² consistency band around 1
//                     and points coloured by the gate decision (accepted vs gated).
//                     This is the local image of the #212 over-confidence: NIS/dof
//                     riding above the band ⇒ the filter trusts vision more than the
//                     data supports (R under-modeled).
//   residuals.svg   — per-update RMS reprojection residual (px).
//   covariance.svg  — the dumped update's covariance before/after, side-by-side
//                     heatmaps (log|·|) with IMU/clone block separators.
//
//   node docs-site/scripts/gen-update-figures.mjs <dir>
//
// Inputs (written by s6_inspect):
//   <dir>/updates.jsonl   one record/line: {index,t,nis,dof,nis_over_dof,accepted,
//                         gated,valid,chi2_threshold,cov_trace_before,cov_trace_after,
//                         residual_rms_px,...}
//   <dir>/covariance.json {dim,imu_dim,clone_dim,before:[[...]],after:[[...]],...}

import { readFileSync, writeFileSync, existsSync } from 'node:fs';
import { resolve } from 'node:path';

const dir = resolve(process.argv[2] ?? 'build/stage_probes/S6');
const updPath = resolve(dir, 'updates.jsonl');
if (!existsSync(updPath)) {
  console.error(`no updates.jsonl in ${dir} — run s6_inspect --out ${dir} first`);
  process.exit(1);
}

const esc = (s) => String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
const num = (v, d = 2) => (typeof v === 'number' && Number.isFinite(v) ? v.toFixed(d) : '—');

// Perceptual-ish blue→green→yellow→red ramp, t∈[0,1] (matches the other generators).
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

const recs = readFileSync(updPath, 'utf8').trim().split('\n').filter(Boolean)
  .map((l) => { try { return JSON.parse(l); } catch { return null; } }).filter(Boolean);
if (!recs.length) { console.error('updates.jsonl had no records'); process.exit(1); }

const W = 760, H = 420;
const M = { l: 64, r: 24, t: 44, b: 48 };
const pw = W - M.l - M.r, ph = H - M.t - M.b;

// Median dof for the χ² normal-approximation band: Var(NIS/dof) ≈ 2/dof.
const dofs = recs.map((r) => r.dof).filter((d) => d > 0).sort((a, b) => a - b);
const dofMed = dofs.length ? dofs[Math.floor(dofs.length / 2)] : 5;
const bandHalf = 2 * Math.sqrt(2 / dofMed);  // ~95% band half-width around 1

function svgOpen(w, h) {
  return `<svg xmlns="http://www.w3.org/2000/svg" width="${w}" height="${h}" viewBox="0 0 ${w} ${h}"><rect width="${w}" height="${h}" fill="#0e1116"/>`;
}

// ── nis.svg : NIS/dof over updates, χ² band, gate colouring ────────────────────
function renderNis() {
  const n = recs.length;
  const ys = recs.map((r) => r.nis_over_dof).filter(Number.isFinite);
  const yMax = Math.max(2, 1 + bandHalf, ...ys) * 1.05;
  const X = (i) => M.l + (n <= 1 ? 0.5 : i / (n - 1)) * pw;
  const Y = (y) => M.t + ph - (Math.min(y, yMax) / yMax) * ph;

  let s = svgOpen(W, H);
  s += `<text x="${M.l}" y="26" font-family="monospace" font-size="14" fill="#eee">S6 update consistency — NIS / dof over ${n} updates</text>`;
  s += `<rect x="${M.l}" y="${M.t}" width="${pw}" height="${ph}" fill="#0a0a0a" stroke="#444"/>`;

  // χ² consistency band around 1.0 (green), and the target line.
  const yb0 = Y(Math.max(0, 1 - bandHalf)), yb1 = Y(1 + bandHalf), y1 = Y(1);
  s += `<rect x="${M.l}" y="${yb1.toFixed(1)}" width="${pw}" height="${(yb0 - yb1).toFixed(1)}" fill="#39d98a" fill-opacity="0.10"/>`;
  s += `<line x1="${M.l}" y1="${y1.toFixed(1)}" x2="${M.l + pw}" y2="${y1.toFixed(1)}" stroke="#39d98a" stroke-width="1" stroke-dasharray="4 3"/>`;
  s += `<text x="${M.l + pw - 4}" y="${(y1 - 4).toFixed(1)}" text-anchor="end" font-family="monospace" font-size="10" fill="#39d98a">NIS/dof = 1 (consistent)</text>`;

  // y gridlines + labels.
  for (let g = 0; g <= 4; g++) {
    const yv = (yMax / 4) * g, y = Y(yv);
    s += `<line x1="${M.l}" y1="${y.toFixed(1)}" x2="${M.l + pw}" y2="${y.toFixed(1)}" stroke="#222" stroke-width="0.7"/>`;
    s += `<text x="${M.l - 6}" y="${(y + 3).toFixed(1)}" text-anchor="end" font-family="monospace" font-size="10" fill="#bbb">${yv.toFixed(1)}</text>`;
  }

  // points coloured by gate decision; connect valid ones with a faint line.
  let path = '', started = false;
  for (let i = 0; i < n; i++) {
    const r = recs[i];
    if (!Number.isFinite(r.nis_over_dof)) continue;
    path += `${started ? 'L' : 'M'}${X(i).toFixed(1)},${Y(r.nis_over_dof).toFixed(1)}`;
    started = true;
  }
  s += `<path d="${path}" fill="none" stroke="#5a7" stroke-width="0.8" opacity="0.5"/>`;
  for (let i = 0; i < n; i++) {
    const r = recs[i];
    if (!Number.isFinite(r.nis_over_dof)) continue;
    const c = !r.valid ? '#888' : r.gated ? '#e6552d' : '#4fd1ff';
    s += `<circle cx="${X(i).toFixed(1)}" cy="${Y(r.nis_over_dof).toFixed(1)}" r="2.2" fill="${c}"/>`;
  }

  // axis title + legend + HUD.
  s += `<text x="${(M.l + pw / 2).toFixed(1)}" y="${H - 14}" text-anchor="middle" font-family="monospace" font-size="11" fill="#ddd">update index</text>`;
  const accepted = recs.filter((r) => r.accepted).length, gated = recs.filter((r) => r.gated).length;
  const validNod = recs.filter((r) => r.valid && r.dof > 0).map((r) => r.nis_over_dof);
  const mean = validNod.length ? validNod.reduce((a, b) => a + b, 0) / validNod.length : null;
  const hud = [`accepted ${accepted}   χ²-gated ${gated}`, `mean NIS/dof ${num(mean)}  (band ±${bandHalf.toFixed(2)} @dof≈${dofMed})`];
  s += `<rect x="${M.l + 6}" y="${M.t + 6}" width="270" height="40" rx="4" fill="#000" opacity="0.5"/>`;
  hud.forEach((line, i) => { s += `<text x="${M.l + 14}" y="${M.t + 23 + i * 16}" font-family="monospace" font-size="11" fill="#eee">${esc(line)}</text>`; });
  s += `<circle cx="${M.l + pw - 150}" cy="${M.t + 16}" r="3" fill="#4fd1ff"/><text x="${M.l + pw - 142}" y="${M.t + 20}" font-family="monospace" font-size="10" fill="#ddd">accepted</text>`;
  s += `<circle cx="${M.l + pw - 70}" cy="${M.t + 16}" r="3" fill="#e6552d"/><text x="${M.l + pw - 62}" y="${M.t + 20}" font-family="monospace" font-size="10" fill="#ddd">gated</text>`;
  return s + '</svg>';
}

// ── residuals.svg : per-update RMS reprojection residual (px) ──────────────────
function renderResiduals() {
  const n = recs.length;
  const ys = recs.map((r) => r.residual_rms_px).filter(Number.isFinite);
  const yMax = Math.max(0.5, ...ys) * 1.1;
  const X = (i) => M.l + (n <= 1 ? 0.5 : i / (n - 1)) * pw;
  const Y = (y) => M.t + ph - (Math.min(y, yMax) / yMax) * ph;
  let s = svgOpen(W, H);
  s += `<text x="${M.l}" y="26" font-family="monospace" font-size="14" fill="#eee">S6 reprojection residual (RMS px) over ${n} updates</text>`;
  s += `<rect x="${M.l}" y="${M.t}" width="${pw}" height="${ph}" fill="#0a0a0a" stroke="#444"/>`;
  for (let g = 0; g <= 4; g++) {
    const yv = (yMax / 4) * g, y = Y(yv);
    s += `<line x1="${M.l}" y1="${y.toFixed(1)}" x2="${M.l + pw}" y2="${y.toFixed(1)}" stroke="#222" stroke-width="0.7"/>`;
    s += `<text x="${M.l - 6}" y="${(y + 3).toFixed(1)}" text-anchor="end" font-family="monospace" font-size="10" fill="#bbb">${yv.toFixed(1)}</text>`;
  }
  let path = '', started = false;
  for (let i = 0; i < n; i++) {
    const r = recs[i];
    if (!Number.isFinite(r.residual_rms_px)) continue;
    path += `${started ? 'L' : 'M'}${X(i).toFixed(1)},${Y(r.residual_rms_px).toFixed(1)}`;
    started = true;
  }
  s += `<path d="${path}" fill="none" stroke="#fdbe85" stroke-width="1.2"/>`;
  s += `<text x="${(M.l + pw / 2).toFixed(1)}" y="${H - 14}" text-anchor="middle" font-family="monospace" font-size="11" fill="#ddd">update index</text>`;
  return s + '</svg>';
}

// ── covariance.svg : before/after heatmaps (log|·|) side-by-side ───────────────
function renderCovariance() {
  const covPath = resolve(dir, 'covariance.json');
  if (!existsSync(covPath)) return null;
  let cov;
  try { cov = JSON.parse(readFileSync(covPath, 'utf8')); } catch { return null; }
  const A = cov.before, B = cov.after;
  if (!Array.isArray(A) || !Array.isArray(B) || !A.length) return null;
  const N = A.length;
  const imuDim = cov.imu_dim ?? 15, cloneDim = cov.clone_dim ?? 6;
  if (!(cloneDim > 0)) {  // a 0/negative block stride would make the separator loop spin forever
    console.error(`covariance.json: invalid clone_dim ${cloneDim}, skipping covariance.svg`);
    return null;
  }

  // Shared log scale over both matrices. Validate the row-major shape so a
  // corrupt (scalar) row fails loudly instead of throwing "not iterable".
  let amax = -Infinity, amin = Infinity;
  for (const Mx of [A, B]) for (const row of Mx) {
    if (!Array.isArray(row)) {
      console.error('covariance.json: a matrix row is not an array, skipping covariance.svg');
      return null;
    }
    for (const v of row) {
      const a = Math.abs(v); if (a > 0) { if (a > amax) amax = a; if (a < amin) amin = a; }
    }
  }
  const lo = Math.log10(Math.max(amin, amax * 1e-6)), hi = Math.log10(amax);
  const col = (v) => { const a = Math.abs(v); if (!(a > 0)) return '#0a0a0a'; return ramp((Math.log10(a) - lo) / (hi - lo || 1)); };

  const cell = Math.max(2, Math.min(5, Math.floor(300 / N)));
  const grid = N * cell;
  const pad = 70, gap = 60, cbW = 16;
  const w = pad + grid + gap + grid + 90, h = 60 + grid + 40;

  function heat(Mx, ox, oy, title) {
    let s = `<text x="${ox}" y="${oy - 8}" font-family="monospace" font-size="13" fill="#eee">${esc(title)}</text>`;
    for (let i = 0; i < N; i++)
      for (let j = 0; j < N; j++)
        s += `<rect x="${ox + j * cell}" y="${oy + i * cell}" width="${cell}" height="${cell}" fill="${col(Mx[i][j])}"/>`;
    // block separators: IMU | (clones)
    const lines = [imuDim];
    for (let o = imuDim; o < N; o += cloneDim) lines.push(o);
    for (const o of lines) {
      const p = o * cell;
      s += `<line x1="${ox + p}" y1="${oy}" x2="${ox + p}" y2="${oy + grid}" stroke="#fff" stroke-opacity="0.18" stroke-width="0.6"/>`;
      s += `<line x1="${ox}" y1="${oy + p}" x2="${ox + grid}" y2="${oy + p}" stroke="#fff" stroke-opacity="0.18" stroke-width="0.6"/>`;
    }
    s += `<rect x="${ox}" y="${oy}" width="${grid}" height="${grid}" fill="none" stroke="#555"/>`;
    return s;
  }

  let s = svgOpen(w, h);
  s += `<text x="${pad}" y="28" font-family="monospace" font-size="14" fill="#eee">S6 covariance before / after update #${cov.update_index ?? 0}  (log|P|, ${N}×${N}; white lines = IMU/clone blocks)</text>`;
  s += heat(A, pad, 56, 'before');
  s += heat(B, pad + grid + gap, 56, 'after');
  // colorbar
  const cbx = pad + 2 * grid + gap + 24, cby = 56;
  for (let k = 0; k < 100; k++)
    s += `<rect x="${cbx}" y="${cby + (grid * (99 - k)) / 100}" width="${cbW}" height="${grid / 100 + 1}" fill="${ramp(k / 99)}"/>`;
  s += `<rect x="${cbx}" y="${cby}" width="${cbW}" height="${grid}" fill="none" stroke="#888"/>`;
  s += `<text x="${cbx + cbW + 4}" y="${cby + 8}" font-family="monospace" font-size="9" fill="#ddd">1e${hi.toFixed(0)}</text>`;
  s += `<text x="${cbx + cbW + 4}" y="${cby + grid}" font-family="monospace" font-size="9" fill="#ddd">1e${lo.toFixed(0)}</text>`;
  return s + '</svg>';
}

writeFileSync(resolve(dir, 'nis.svg'), renderNis());
writeFileSync(resolve(dir, 'residuals.svg'), renderResiduals());
const covSvg = renderCovariance();
if (covSvg) writeFileSync(resolve(dir, 'covariance.svg'), covSvg);

console.log(`wrote ${dir}/nis.svg, residuals.svg${covSvg ? ', covariance.svg' : ' (no covariance.json — skipped covariance.svg)'}`);
