// Render the S0 sensor-model probe artifacts (CSV) to SVG figures for
// qualitative inspection — the visual half of the S0 contract program
// (docs/arch/vio-pipeline-canonical.md, docs/assessments/s0-sensor-model-probe.md).
//
// Pipeline:  tests/sdk/sensor_model_probe.cpp  --(CORTEX_PROBE_OUT)-->  *.csv
//            this script                        --------------------->  *.svg
//
// Usage:
//   node docs-site/scripts/gen-sensor-model-figures.mjs [csvDir] [outDir]
//   csvDir default: build/sensor_model_probe
//   outDir default: docs/assessments/figures/s0
//
// Self-contained: a tiny dependency-free SVG builder, a perceptual color ramp,
// and a CSV reader. Unlike gen-benchmarks.mjs (a hard gate), a MISSING artifact
// here is a warning, not a failure — you may have run only a subset of the probe.

import { readFileSync, writeFileSync, mkdirSync, existsSync } from 'node:fs';
import { resolve, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';

const here = dirname(fileURLToPath(import.meta.url));
const repo = resolve(here, '..', '..');
const csvDir = resolve(repo, process.argv[2] ?? 'build/sensor_model_probe');
const outDir = resolve(repo, process.argv[3] ?? 'docs/assessments/figures/s0');
mkdirSync(outDir, { recursive: true });

// ── CSV ──────────────────────────────────────────────────────────────────
function readCsv(name) {
  const p = resolve(csvDir, name);
  if (!existsSync(p)) {
    console.warn(`  skip ${name} — not found in ${csvDir} (run the probe with CORTEX_PROBE_OUT set)`);
    return null;
  }
  const lines = readFileSync(p, 'utf8').trim().split('\n');
  const head = lines[0].split(',');
  return lines.slice(1).map((l) => {
    const c = l.split(',');
    const o = {};
    head.forEach((h, i) => {
      const n = Number(c[i]);
      o[h] = Number.isNaN(n) ? c[i] : n;
    });
    return o;
  });
}

// ── SVG primitives ─────────────────────────────────────────────────────────
const esc = (s) => String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');
function svgDoc(w, h, body, title) {
  return `<svg xmlns="http://www.w3.org/2000/svg" width="${w}" height="${h}" viewBox="0 0 ${w} ${h}" font-family="-apple-system,Segoe UI,Roboto,sans-serif">
<rect width="${w}" height="${h}" fill="#ffffff"/>
<text x="${w / 2}" y="22" text-anchor="middle" font-size="15" font-weight="600" fill="#1a1a1a">${esc(title)}</text>
${body}
</svg>`;
}
const txt = (x, y, s, o = {}) =>
  `<text x="${x}" y="${y}" font-size="${o.size ?? 11}" fill="${o.fill ?? '#333'}" text-anchor="${o.anchor ?? 'start'}"${o.weight ? ` font-weight="${o.weight}"` : ''}>${esc(s)}</text>`;
const line = (x1, y1, x2, y2, o = {}) =>
  `<line x1="${x1}" y1="${y1}" x2="${x2}" y2="${y2}" stroke="${o.stroke ?? '#ccc'}" stroke-width="${o.w ?? 1}"${o.dash ? ` stroke-dasharray="${o.dash}"` : ''}/>`;

// Perceptual-ish blue→green→yellow→red ramp (viridis/turbo-flavoured), t∈[0,1].
function ramp(t) {
  t = Math.max(0, Math.min(1, t));
  const stops = [
    [0.0, [13, 8, 135]],   // deep blue   = cold = "contract holds"
    [0.35, [42, 130, 180]],
    [0.6, [60, 170, 100]],
    [0.8, [240, 200, 60]],
    [1.0, [200, 30, 30]],  // red         = hot  = "contract violated"
  ];
  for (let i = 1; i < stops.length; i++) {
    if (t <= stops[i][0]) {
      const [a0, c0] = stops[i - 1];
      const [a1, c1] = stops[i];
      const f = (t - a0) / (a1 - a0 || 1);
      const c = c0.map((v, k) => Math.round(v + f * (c1[k] - v)));
      return `rgb(${c[0]},${c[1]},${c[2]})`;
    }
  }
  return 'rgb(200,30,30)';
}

// ── Image-plane heatmap (round-trip / Jacobian fields) ─────────────────────
// Colors each sample by log10(value), so spatial structure (distortion growing
// toward the FOV edge) is visible even when every sample passes the contract.
function heatmap(rows, { title, file, valueLabel, threshold }) {
  if (!rows || !rows.length) return;
  const W = 560, H = 470;
  const pad = { l: 48, r: 120, t: 40, b: 44 };
  const xs = rows.map((r) => r.x_px), ys = rows.map((r) => r.y_px);
  const imgW = Math.max(...xs), imgH = Math.max(...ys);
  const plotW = W - pad.l - pad.r, plotH = H - pad.t - pad.b;
  const sx = (x) => pad.l + (x / imgW) * plotW;
  const sy = (y) => pad.t + (y / imgH) * plotH;

  const vals = rows.map((r) => r.value).filter((v) => v > 0);
  const lo = vals.length ? Math.log10(Math.min(...vals)) : -14;
  const hi = vals.length ? Math.log10(Math.max(...vals)) : 0;
  const span = hi - lo || 1;
  const maxV = Math.max(...rows.map((r) => r.value));

  // cell size from grid spacing
  const ux = [...new Set(xs)].sort((a, b) => a - b);
  const cw = ux.length > 1 ? (sx(ux[1]) - sx(ux[0])) * 0.92 : 8;
  const ch = cw;

  let body = '';
  // frame
  body += `<rect x="${pad.l}" y="${pad.t}" width="${plotW}" height="${plotH}" fill="#0d0887" opacity="0.06" stroke="#bbb"/>`;
  for (const r of rows) {
    const lv = r.value > 0 ? Math.log10(r.value) : lo;
    body += `<rect x="${sx(r.x_px) - cw / 2}" y="${sy(r.y_px) - ch / 2}" width="${cw}" height="${ch}" fill="${ramp((lv - lo) / span)}"/>`;
  }
  // axes labels
  body += txt(pad.l + plotW / 2, H - 14, 'image x (px)', { anchor: 'middle' });
  body += `<text x="16" y="${pad.t + plotH / 2}" font-size="11" fill="#333" text-anchor="middle" transform="rotate(-90 16 ${pad.t + plotH / 2})">image y (px)</text>`;
  body += txt(pad.l, pad.t - 6, `0`, { size: 9, fill: '#888' });
  body += txt(pad.l + plotW, pad.t - 6, `${Math.round(imgW)}`, { size: 9, fill: '#888', anchor: 'end' });

  // colorbar
  const cbx = W - pad.r + 30, cby = pad.t, cbh = plotH, cbw = 16;
  const nseg = 40;
  for (let i = 0; i < nseg; i++) {
    const t = i / (nseg - 1);
    body += `<rect x="${cbx}" y="${cby + (1 - t) * cbh}" width="${cbw}" height="${cbh / nseg + 1}" fill="${ramp(t)}"/>`;
  }
  body += `<rect x="${cbx}" y="${cby}" width="${cbw}" height="${cbh}" fill="none" stroke="#999"/>`;
  body += txt(cbx + cbw + 4, cby + 8, `1e${hi.toFixed(0)}`, { size: 9 });
  body += txt(cbx + cbw + 4, cby + cbh, `1e${lo.toFixed(0)}`, { size: 9 });
  body += txt(cbx + cbw / 2, cby - 8, valueLabel, { size: 9, anchor: 'middle', fill: '#555' });
  // threshold marker on the bar
  if (threshold && Math.log10(threshold) <= hi && Math.log10(threshold) >= lo) {
    const ty = cby + (1 - (Math.log10(threshold) - lo) / span) * cbh;
    body += line(cbx - 3, ty, cbx + cbw + 3, ty, { stroke: '#000', w: 1.5 });
    body += txt(cbx + cbw + 4, ty + 3, `${threshold} (gate)`, { size: 8, fill: '#000' });
  }
  body += txt(pad.l, H - 4, `max = ${maxV.toExponential(2)} ${valueLabel}`, { size: 9, fill: '#555' });

  writeFileSync(resolve(outDir, file), svgDoc(W, H, body, title));
  console.log(`  wrote ${file}  (max ${maxV.toExponential(2)} ${valueLabel})`);
}

// ── Line chart (sensitivity curves, drift) ─────────────────────────────────
function lineChart(series, { title, file, xlabel, ylabel, logY = false, hline = null, hlineLabel = null }) {
  if (!series.length) return;
  const W = 560, H = 400;
  const pad = { l: 64, r: 150, t: 40, b: 50 };
  const plotW = W - pad.l - pad.r, plotH = H - pad.t - pad.b;
  const allX = series.flatMap((s) => s.points.map((p) => p.x));
  const allYraw = series.flatMap((s) => s.points.map((p) => p.y));
  const xmin = Math.min(...allX), xmax = Math.max(...allX);
  const tf = (y) => (logY ? Math.log10(Math.max(y, 1e-3)) : y);
  const ys = allYraw.map(tf).concat(hline != null ? [tf(hline)] : []);
  let ymin = Math.min(...ys, 0), ymax = Math.max(...ys);
  if (ymin === ymax) ymax = ymin + 1;
  const sx = (x) => pad.l + ((x - xmin) / (xmax - xmin || 1)) * plotW;
  const sy = (y) => pad.t + (1 - (tf(y) - ymin) / (ymax - ymin || 1)) * plotH;

  let body = `<rect x="${pad.l}" y="${pad.t}" width="${plotW}" height="${plotH}" fill="#fafafa" stroke="#ddd"/>`;
  // gridlines + y ticks
  const nticks = 5;
  for (let i = 0; i <= nticks; i++) {
    const yv = ymin + (i / nticks) * (ymax - ymin);
    const py = pad.t + (1 - i / nticks) * plotH;
    body += line(pad.l, py, pad.l + plotW, py, { stroke: '#eee' });
    const label = logY ? `1e${yv.toFixed(1)}` : (Math.abs(yv) >= 1000 ? yv.toExponential(1) : yv.toFixed(2));
    body += txt(pad.l - 6, py + 3, label, { anchor: 'end', size: 9, fill: '#666' });
  }
  // x ticks
  for (let i = 0; i <= 4; i++) {
    const xv = xmin + (i / 4) * (xmax - xmin);
    const px = sx(xv);
    body += line(px, pad.t + plotH, px, pad.t + plotH + 4, { stroke: '#999' });
    body += txt(px, pad.t + plotH + 16, xv.toFixed(xmax - xmin < 5 ? 1 : 0), { anchor: 'middle', size: 9, fill: '#666' });
  }
  // zero axis
  if (ymin < 0 && ymax > 0) body += line(pad.l, sy(0), pad.l + plotW, sy(0), { stroke: '#bbb', w: 1 });
  // hline (e.g. 1px budget)
  if (hline != null) {
    body += line(pad.l, sy(hline), pad.l + plotW, sy(hline), { stroke: '#000', w: 1, dash: '4 3' });
    body += txt(pad.l + plotW - 4, sy(hline) - 4, hlineLabel ?? `${hline}`, { anchor: 'end', size: 9, fill: '#000' });
  }
  const palette = ['#2a6fdb', '#22a06b', '#d1495b', '#e8902a', '#7048c4'];
  series.forEach((s, i) => {
    const col = palette[i % palette.length];
    const pts = s.points.map((p) => `${sx(p.x)},${sy(p.y)}`).join(' ');
    body += `<polyline points="${pts}" fill="none" stroke="${col}" stroke-width="2"/>`;
    // legend
    const ly = pad.t + 6 + i * 18;
    body += line(W - pad.r + 12, ly, W - pad.r + 30, ly, { stroke: col, w: 2.5 });
    body += txt(W - pad.r + 34, ly + 3, s.name, { size: 10, fill: '#333' });
  });
  body += txt(pad.l + plotW / 2, H - 12, xlabel, { anchor: 'middle' });
  body += `<text x="18" y="${pad.t + plotH / 2}" font-size="11" fill="#333" text-anchor="middle" transform="rotate(-90 18 ${pad.t + plotH / 2})">${esc(ylabel)}</text>`;

  writeFileSync(resolve(outDir, file), svgDoc(W, H, body, title));
  console.log(`  wrote ${file}`);
}

// ── Build the figures ──────────────────────────────────────────────────────
console.log(`S0 figures:  ${csvDir}  ->  ${outDir}`);

const rtMap = (rows) => rows && rows.map((r) => ({ x_px: r.x_px, y_px: r.y_px, value: r.residual_px }));
heatmap(rtMap(readCsv('roundtrip_radtan.csv')), {
  title: 'S0.1  EuRoC cam0 projection round-trip residual',
  file: 'roundtrip_radtan.svg', valueLabel: 'px', threshold: 0.01,
});
heatmap(rtMap(readCsv('roundtrip_fisheye.csv')), {
  title: 'S0.1  Fisheye projection round-trip residual (within FOV)',
  file: 'roundtrip_fisheye.svg', valueLabel: 'px', threshold: 0.01,
});
const jacMap = (rows) => rows && rows.map((r) => ({ x_px: r.x_px, y_px: r.y_px, value: r.lin_px_err }));
heatmap(jacMap(readCsv('jacobian_radtan.csv')), {
  title: 'S0.2  EuRoC cam0 Jacobian linearization error (1 cm scene move)',
  file: 'jacobian_radtan.svg', valueLabel: 'px', threshold: 0.1,
});
heatmap(jacMap(readCsv('jacobian_fisheye.csv')), {
  title: 'S0.2  Fisheye Jacobian linearization error (1 cm scene move)',
  file: 'jacobian_fisheye.svg', valueLabel: 'px', threshold: 0.1,
});

// Time-offset sensitivity (px vs ms, one line per motion regime)
const to = readCsv('timeoffset.csv');
if (to) {
  const regimes = [...new Set(to.map((r) => r.regime))];
  lineChart(
    regimes.map((rg) => ({
      name: `${rg} (${to.find((r) => r.regime === rg).px_per_ms.toFixed(2)} px/ms)`,
      points: to.filter((r) => r.regime === rg).map((r) => ({ x: r.dt_ms, y: r.error_px })),
    })),
    { title: 'S0.3  Cost of an unmodeled camera–IMU time offset', file: 'timeoffset.svg',
      xlabel: 'time offset t_d (ms)', ylabel: 'reprojection error (px)', hline: 1,
      hlineLabel: '1 px (1σ budget)' },
  );
}

// Extrinsic sensitivity — two charts (rotation, translation), line per depth
const ex = readCsv('extrinsic.csv');
if (ex) {
  const depths = [...new Set(ex.map((r) => r.depth_m))];
  lineChart(
    depths.map((d) => ({ name: `${d} m depth`, points: ex.filter((r) => r.type === 'rot_deg' && r.depth_m === d).map((r) => ({ x: r.perturb, y: r.error_px })) })),
    { title: 'S0.4  Reprojection bias vs extrinsic rotation error', file: 'extrinsic_rotation.svg',
      xlabel: 'extrinsic rotation error (deg)', ylabel: 'reprojection bias (px)' },
  );
  lineChart(
    depths.map((d) => ({ name: `${d} m depth`, points: ex.filter((r) => r.type === 'trans_mm' && r.depth_m === d).map((r) => ({ x: r.perturb, y: r.error_px })) })),
    { title: 'S0.4  Reprojection bias vs extrinsic translation error', file: 'extrinsic_translation.svg',
      xlabel: 'extrinsic translation error (mm)', ylabel: 'reprojection bias (px)' },
  );
}

// IMU dead-reckoning drift (log mm vs time, one line per case)
const im = readCsv('imu_drift.csv');
if (im) {
  const cases = [...new Set(im.map((r) => r.case))];
  lineChart(
    cases.map((c) => ({ name: c, points: im.filter((r) => r.case === c).map((r) => ({ x: r.t_s, y: r.pos_drift_mm })) })),
    { title: 'S0.5  IMU dead-reckoning drift — gravity leakage', file: 'imu_drift.svg',
      xlabel: 'time (s)', ylabel: 'position drift (mm, log)', logY: true },
  );
}

// ── S2 figures (IMU propagation) — rendered when the S2 artifacts are present ─
const grow = readCsv('prop_growth.csv');
if (grow) {
  lineChart(
    [
      { name: 'cortex (diagonal Q)', points: grow.map((r) => ({ x: r.t_s, y: r.pos_cortex_mm })) },
      { name: 'canonical Q_d', points: grow.map((r) => ({ x: r.t_s, y: r.pos_canon_mm })) },
    ],
    { title: 'S2  Position σ growth — cortex diagonal Q vs canonical Q_d', file: 'prop_growth_pos.svg',
      xlabel: 'propagation time (s)', ylabel: 'position σ (mm)' },
  );
}
const nees = readCsv('prop_nees.csv');
if (nees) {
  lineChart(
    [{ name: '6-DoF pose NEES', points: nees.map((r) => ({ x: r.q_scale, y: r.nees_pose })) }],
    { title: 'S2  Propagation-only NEES vs Q-scale (NEES proportional to 1/Q)', file: 'prop_nees.svg',
      xlabel: 'Q-scale multiplier', ylabel: 'NEES (target = dof)', hline: nees[0]?.dof ?? 6,
      hlineLabel: `dof = ${nees[0]?.dof ?? 6} (consistent)` },
  );
}
const gt = readCsv('prop_gt_injection.csv');
if (gt) {
  lineChart(
    [{ name: 'position error vs fine ref', points: gt.map((r) => ({ x: r.dt_s * 1000, y: r.pos_error_mm })) }],
    { title: 'S2  Mean-integration discretization error — O(dt)', file: 'prop_gt_error.svg',
      xlabel: 'IMU step dt (ms)', ylabel: 'position error (mm)' },
  );
}

// ── S1 figures (initialization) — rendered when the S1 artifacts are present ─
const stat = readCsv('init_static_sweep.csv');
if (stat) {
  lineChart(
    [{ name: 'leveling error (accepted)', points: stat.filter((r) => r.gate_accept === 1).map((r) => ({ x: r.accel_noise_std, y: r.rollpitch_err_deg })) }],
    { title: 'S1  Static init: roll/pitch leveling error vs accel noise', file: 'init_static.svg',
      xlabel: 'accel noise σ (m/s²)', ylabel: 'leveling error (deg)' },
  );
}
const exc = readCsv('init_excitation_sweep.csv');
if (exc) {
  lineChart(
    [{ name: 'resolved motion', points: exc.map((r) => ({ x: r.excitation_ms2, y: r.resolved_motion_m })) }],
    { title: 'S1  Dynamic init: scale-observability cliff (gate floor = 0.05 m)', file: 'init_excitation.svg',
      xlabel: 'acceleration excitation (m/s²)', ylabel: 'resolved metric motion (m)', hline: 0.05,
      hlineLabel: '0.05 m gate floor — below this, scale declined' },
  );
  lineChart(
    [{ name: 'scale error (accepted)', points: exc.filter((r) => r.gate_accept === 1).map((r) => ({ x: r.excitation_ms2, y: r.scale_err_pct })) }],
    { title: 'S1  Dynamic init: metric-scale recovery error vs excitation', file: 'init_scale_error.svg',
      xlabel: 'acceleration excitation (m/s²)', ylabel: 'scale error (%)' },
  );
}

// ── Equal-aspect 2-D path plot (trajectory overlay) ────────────────────────
function pathPlot(series, { title, file, xlabel, ylabel }) {
  if (!series.length) return;
  const W = 520, H = 500;
  const pad = { l: 60, r: 130, t: 40, b: 50 };
  const plotW = W - pad.l - pad.r, plotH = H - pad.t - pad.b;
  const xs = series.flatMap((s) => s.points.map((p) => p.x));
  const ys = series.flatMap((s) => s.points.map((p) => p.y));
  let xmin = Math.min(...xs), xmax = Math.max(...xs), ymin = Math.min(...ys), ymax = Math.max(...ys);
  // Equal aspect: expand the smaller range so a circle reads as a circle.
  const cx = (xmin + xmax) / 2, cy = (ymin + ymax) / 2;
  const span = Math.max(xmax - xmin, ymax - ymin, 1e-6) * 1.1;
  xmin = cx - span / 2; xmax = cx + span / 2; ymin = cy - span / 2; ymax = cy + span / 2;
  const sx = (x) => pad.l + ((x - xmin) / (xmax - xmin)) * plotW;
  const sy = (y) => pad.t + (1 - (y - ymin) / (ymax - ymin)) * plotH;
  let body = `<rect x="${pad.l}" y="${pad.t}" width="${plotW}" height="${plotH}" fill="#fafafa" stroke="#ddd"/>`;
  const palette = ['#222', '#d1495b', '#2a6fdb'];
  series.forEach((s, i) => {
    const col = palette[i % palette.length];
    const dash = s.dash ? ` stroke-dasharray="${s.dash}"` : '';
    body += `<polyline points="${s.points.map((p) => `${sx(p.x)},${sy(p.y)}`).join(' ')}" fill="none" stroke="${col}" stroke-width="2"${dash}/>`;
    body += `<circle cx="${sx(s.points[0].x)}" cy="${sy(s.points[0].y)}" r="4" fill="${col}"/>`;  // start
    const ly = pad.t + 6 + i * 18;
    body += line(W - pad.r + 12, ly, W - pad.r + 30, ly, { stroke: col, w: 2.5 });
    body += txt(W - pad.r + 34, ly + 3, s.name, { size: 10, fill: '#333' });
  });
  body += txt(pad.l + plotW / 2, H - 12, xlabel, { anchor: 'middle' });
  body += `<text x="18" y="${pad.t + plotH / 2}" font-size="11" fill="#333" text-anchor="middle" transform="rotate(-90 18 ${pad.t + plotH / 2})">${esc(ylabel)}</text>`;
  writeFileSync(resolve(outDir, file), svgDoc(W, H, body, title));
  console.log(`  wrote ${file}`);
}

// ── Pipeline noise-demo figures (vio_pipeline) ─────────────────────────────
const traj = readCsv('trajectory.csv');
if (traj) {
  pathPlot(
    [
      { name: 'ground truth', points: traj.map((r) => ({ x: r.gt_x, y: r.gt_y })) },
      { name: 'estimate', points: traj.map((r) => ({ x: r.est_x, y: r.est_y })), dash: '5 3' },
    ],
    { title: 'Pipeline  estimated vs ground-truth trajectory (top-down)', file: 'pipeline_trajectory.svg',
      xlabel: 'world x (m)', ylabel: 'world y (m)' },
  );
  lineChart(
    [{ name: 'position error', points: traj.map((r) => ({ x: r.t, y: r.pos_err })) }],
    { title: 'Pipeline  position error vs time', file: 'pipeline_error_time.svg',
      xlabel: 'time (s)', ylabel: 'position error (m)' },
  );
}
const nsw = readCsv('noise_sweep.csv');
if (nsw) {
  lineChart(
    [
      { name: 'ATE (RMS)', points: nsw.map((r) => ({ x: r.noise_scale, y: r.ate_rms_m })) },
      { name: 'final error', points: nsw.map((r) => ({ x: r.noise_scale, y: r.final_err_m })) },
    ],
    { title: 'Pipeline  robustness: trajectory error vs sensor-noise level', file: 'pipeline_noise_ate.svg',
      xlabel: 'sensor-noise scale (1 = filter’s assumed noise)', ylabel: 'error (m)', logY: true },
  );
  lineChart(
    [{ name: 'NIS (normalized)', points: nsw.map((r) => ({ x: r.noise_scale, y: r.nis_normalized })) }],
    { title: 'Pipeline  consistency (NIS) vs sensor-noise level', file: 'pipeline_noise_nis.svg',
      xlabel: 'sensor-noise scale', ylabel: 'NIS (1 = consistent)', hline: 1,
      hlineLabel: 'NIS = 1 (consistent)' },
  );
}

console.log('done.');
