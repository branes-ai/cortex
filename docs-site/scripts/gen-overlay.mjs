// Composite per-frame VIO data into overlay frames. Reads <dir>/frames.jsonl and
// emits one self-contained SVG per frame into <dir>/frames/.
//
// Two record shapes are supported (auto-detected per record):
//   • pipeline/scene (tools/src/vio_pipeline.cpp --video): camera image + live
//     feature tracks (+ synthetic true projections, so the additive camera noise
//     reads as the red↔gray offset) + a metrics HUD.
//   • S4 frontend inspector (tools/src/s4_inspect.cpp, has a `tracks` array): the
//     image-domain renderer tier — KLT flow vectors coloured by forward-backward
//     residual, FAST detections, a spatial-coverage grid, a pyramid schematic,
//     and a track-count HUD.
//   • S0 sensor-model inspector (tools/src/s0_inspect.cpp, `kind:"s0_camera"` /
//     `kind:"s0_imu"`): the distortion grid (ideal vs lens-warped) + displacement
//     field over the actual frame, and the per-channel IMU Allan-deviation /
//     noise plot. Written to distortion.svg / imu_allan.svg.
//   • S5 triangulation inspector (tools/src/s5_inspect.cpp, `kind:"s5_residuals"`):
//     per-frame reprojection residuals over the actual frame — observed pixel vs
//     the triangulated landmark's reprojection, the vector coloured by error. The
//     3-D landmark cloud + covariance ellipsoids render in the Scene3D viewer.
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

// Perceptual-ish blue→green→yellow→red ramp, t∈[0,1] (matches the figure generator).
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

const records = readFileSync(framesPath, 'utf8').trim().split('\n').filter(Boolean)
  .map((l) => { try { return JSON.parse(l); } catch { return null; } }).filter(Boolean);

// Global depth range (synthetic only) so the depth colour-map is consistent
// across the whole video — near features pop warm, far ones go cool.
let dmin = Infinity, dmax = -Infinity;
for (const r of records) for (const d of r.depth ?? []) { if (d < dmin) dmin = d; if (d > dmax) dmax = d; }
const haveDepth = Number.isFinite(dmin) && dmax > dmin;
// near (close, large parallax) → red; far → blue.
const depthColor = (d) => ramp(1 - (d - dmin) / (dmax - dmin));

// ── S4 frontend inspector tier ──────────────────────────────────────────────
// Draw the optical flow (prev→curr) coloured by forward-backward residual, the
// new FAST detections, a coverage grid shaded by track occupancy, a pyramid
// schematic, and a count HUD. `r.tracks` carries {u,v,pu,pv,fb,age,status}.
function frontendBody(r) {
  const w = r.width ?? W, h = r.height ?? H;
  const tracks = r.tracks ?? [];
  // Residual reference for the colour ramp: the gate if set, else 2 px.
  const fbRef = r.fb_max > 0 ? r.fb_max : 2.0;
  const fbColor = (fb) => (fb == null || fb < 0 ? '#888' : ramp(Math.min(1, fb / fbRef)));

  let s = '';

  // Coverage grid: shade cells holding ≥1 track (computed here from positions).
  const gc = r.grid?.cols ?? 8, gr = r.grid?.rows ?? 6;
  const occ = new Set();
  for (const t of tracks) {
    const cx = Math.min(gc - 1, Math.max(0, Math.floor((t.u / w) * gc)));
    const cy = Math.min(gr - 1, Math.max(0, Math.floor((t.v / h) * gr)));
    occ.add(cy * gc + cx);
  }
  for (let cy = 0; cy < gr; cy++)
    for (let cx = 0; cx < gc; cx++) {
      const x = (cx / gc) * w, y = (cy / gr) * h, cw = w / gc, ch = h / gr;
      const filled = occ.has(cy * gc + cx);
      s += `<rect x="${x.toFixed(1)}" y="${y.toFixed(1)}" width="${cw.toFixed(1)}" height="${ch.toFixed(1)}" fill="${filled ? '#39d98a' : '#000'}" fill-opacity="${filled ? 0.06 : 0}" stroke="#2a6" stroke-opacity="0.18" stroke-width="1"/>`;
    }

  // Optical-flow vectors (tracked) coloured by FB residual; new detections as
  // green crosses; lost tracks already absent from the record.
  for (const t of tracks) {
    if (t.status === 'new') {
      s += `<path d="M${(t.u - 4).toFixed(1)},${t.v.toFixed(1)} h8 M${t.u.toFixed(1)},${(t.v - 4).toFixed(1)} v8" stroke="#39d98a" stroke-width="1.3" opacity="0.9"/>`;
      continue;
    }
    const c = fbColor(t.fb);
    s += `<line x1="${t.pu.toFixed(1)}" y1="${t.pv.toFixed(1)}" x2="${t.u.toFixed(1)}" y2="${t.v.toFixed(1)}" stroke="${c}" stroke-width="1.6" opacity="0.9"/>`;
    s += `<circle cx="${t.u.toFixed(1)}" cy="${t.v.toFixed(1)}" r="2" fill="${c}"/>`;
  }

  // Pyramid schematic (bottom-right): nested rects, one per level.
  const sizes = r.pyramid?.sizes ?? [];
  if (sizes.length) {
    const px = w - 150, py = h - 20, scale = 120 / (sizes[0]?.[0] ?? w);
    for (let i = sizes.length - 1; i >= 0; i--) {
      const lw = sizes[i][0] * scale, lh = sizes[i][1] * scale;
      s += `<rect x="${px.toFixed(1)}" y="${(py - lh).toFixed(1)}" width="${lw.toFixed(1)}" height="${lh.toFixed(1)}" fill="none" stroke="#7af" stroke-opacity="${(0.4 + 0.5 * (i === 0 ? 1 : 0)).toFixed(2)}" stroke-width="1"/>`;
    }
    s += `<text x="${px}" y="${(py + 12).toFixed(1)}" font-family="monospace" font-size="9" fill="#9cf">pyramid ${sizes.length} lvl</text>`;
  }

  // FB-residual legend bar.
  const lx = w - 130, ly = 24, lw = 110, lh = 10;
  for (let k = 0; k < 22; k++)
    s += `<rect x="${lx + (k / 22) * lw}" y="${ly}" width="${lw / 22 + 1}" height="${lh}" fill="${ramp(k / 21)}"/>`;
  s += `<rect x="${lx}" y="${ly}" width="${lw}" height="${lh}" fill="none" stroke="#888"/>`;
  s += `<text x="${lx}" y="${ly - 3}" font-family="monospace" font-size="10" fill="#ddd">FB residual (px)</text>`;
  s += `<text x="${lx}" y="${ly + lh + 11}" font-family="monospace" font-size="9" fill="#ddd">0</text>`;
  s += `<text x="${lx + lw}" y="${ly + lh + 11}" text-anchor="end" font-family="monospace" font-size="9" fill="#ddd">${fbRef.toFixed(1)}</text>`;

  // Count HUD (top-left).
  const c = r.counts ?? {};
  const fbs = tracks.filter((t) => typeof t.fb === 'number' && t.fb >= 0).map((t) => t.fb);
  const fbMean = fbs.length ? fbs.reduce((a, b) => a + b, 0) / fbs.length : null;
  const hud = [
    `frame ${r.frame}   t=${num(r.t)}s`,
    `tracked ${c.tracked ?? '—'}  new ${c.new ?? '—'}  lost ${c.lost ?? '—'}`,
    `FB: cull ${c.fb_culled ?? 0}  mean ${num(fbMean)}px  gate ${num(r.fb_max, 1)}px`,
    `coverage ${r.grid?.occupied ?? occ.size}/${gc * gr} cells`,
  ];
  const boxH = 16 + hud.length * 18;
  s += `<rect x="8" y="8" width="288" height="${boxH}" rx="5" fill="#000" opacity="0.55"/>`;
  hud.forEach((line, i) => {
    s += `<text x="18" y="${30 + i * 18}" font-family="monospace" font-size="13" fill="#eee">${esc(line)}</text>`;
  });
  s += `<text x="8" y="${h - 12}" font-family="monospace" font-size="11" fill="#ddd">line = KLT flow (colour = FB residual)   + = new detection   grid = coverage</text>`;
  return s;
}

// ── S0 sensor-model inspector tier ────────────────────────────────────────
// Camera record (`kind:"s0_camera"`): draw the rectilinear IDEAL grid (faint,
// straight) and the lens-WARPED grid (the observed pixels, bowed), with a
// displacement vector per node coloured by distortion magnitude. The gap between
// the two grids IS the distortion the front end must invert. `r.samples` carries
// {u,v,iu,iv,dist,rt} in row-major (cols×rows) order.
function s0CameraBody(r) {
  const w = r.width ?? W, h = r.height ?? H;
  const cols = r.grid?.cols ?? 0, rows = r.grid?.rows ?? 0;
  const sm = r.samples ?? [];
  const ref = r.max_distortion_px > 0 ? r.max_distortion_px : 1;
  const dColor = (d) => ramp(Math.min(1, d / ref));
  let s = '';

  // The mesh lines need a full, ordered cols×rows grid; pinhole/radtan over a
  // normal FOV never drops a node, but guard so a sparse fisheye grid still draws
  // its vectors (below) even when the mesh is skipped.
  const full = cols > 1 && rows > 1 && sm.length === cols * rows;
  const at = (cx, cy) => sm[cy * cols + cx];
  if (full) {
    // Ideal rectilinear grid — straight, faint grey.
    for (let cy = 0; cy < rows; cy++) {
      let d = '';
      for (let cx = 0; cx < cols; cx++) d += `${cx ? 'L' : 'M'}${at(cx, cy).iu.toFixed(1)},${at(cx, cy).iv.toFixed(1)}`;
      s += `<path d="${d}" fill="none" stroke="#888" stroke-opacity="0.35" stroke-width="0.7"/>`;
    }
    for (let cx = 0; cx < cols; cx++) {
      let d = '';
      for (let cy = 0; cy < rows; cy++) d += `${cy ? 'L' : 'M'}${at(cx, cy).iu.toFixed(1)},${at(cx, cy).iv.toFixed(1)}`;
      s += `<path d="${d}" fill="none" stroke="#888" stroke-opacity="0.35" stroke-width="0.7"/>`;
    }
    // Lens-warped grid — the observed pixels, coloured mid-ramp.
    for (let cy = 0; cy < rows; cy++) {
      let d = '';
      for (let cx = 0; cx < cols; cx++) d += `${cx ? 'L' : 'M'}${at(cx, cy).u.toFixed(1)},${at(cx, cy).v.toFixed(1)}`;
      s += `<path d="${d}" fill="none" stroke="#39d98a" stroke-opacity="0.7" stroke-width="1"/>`;
    }
    for (let cx = 0; cx < cols; cx++) {
      let d = '';
      for (let cy = 0; cy < rows; cy++) d += `${cy ? 'L' : 'M'}${at(cx, cy).u.toFixed(1)},${at(cx, cy).v.toFixed(1)}`;
      s += `<path d="${d}" fill="none" stroke="#39d98a" stroke-opacity="0.7" stroke-width="1"/>`;
    }
  }
  // Displacement vectors (ideal → observed) coloured by distortion magnitude.
  for (const p of sm) {
    const c = dColor(p.dist);
    s += `<line x1="${p.iu.toFixed(1)}" y1="${p.iv.toFixed(1)}" x2="${p.u.toFixed(1)}" y2="${p.v.toFixed(1)}" stroke="${c}" stroke-width="1.4" opacity="0.95"/>`;
    s += `<circle cx="${p.u.toFixed(1)}" cy="${p.v.toFixed(1)}" r="1.6" fill="${c}"/>`;
  }

  // Distortion colour legend.
  const lx = w - 130, ly = 24, lw = 110, lh = 10;
  for (let k = 0; k < 22; k++) s += `<rect x="${lx + (k / 22) * lw}" y="${ly}" width="${lw / 22 + 1}" height="${lh}" fill="${ramp(k / 21)}"/>`;
  s += `<rect x="${lx}" y="${ly}" width="${lw}" height="${lh}" fill="none" stroke="#888"/>`;
  s += `<text x="${lx}" y="${ly - 3}" font-family="monospace" font-size="10" fill="#ddd">distortion (px)</text>`;
  s += `<text x="${lx}" y="${ly + lh + 11}" font-family="monospace" font-size="9" fill="#ddd">0</text>`;
  s += `<text x="${lx + lw}" y="${ly + lh + 11}" text-anchor="end" font-family="monospace" font-size="9" fill="#ddd">${ref.toFixed(1)}</text>`;

  // HUD.
  const hud = [
    `S0 camera: ${r.model ?? '—'}`,
    `distortion max ${num(r.max_distortion_px, 1)} px`,
    `round-trip max ${(r.max_roundtrip_px ?? 0).toExponential(1)} px ${r.max_roundtrip_px < 1e-2 ? '(contract OK)' : '(FAIL)'}`,
    `grid ${cols}×${rows}  (${sm.length} pts)`,
  ];
  const boxH = 16 + hud.length * 18;
  s += `<rect x="8" y="8" width="300" height="${boxH}" rx="5" fill="#000" opacity="0.55"/>`;
  hud.forEach((line, i) => { s += `<text x="18" y="${30 + i * 18}" font-family="monospace" font-size="13" fill="#eee">${esc(line)}</text>`; });
  s += `<text x="8" y="${h - 12}" font-family="monospace" font-size="11" fill="#ddd">grey = ideal pinhole   green = lens-warped   vector = distortion (colour = px)</text>`;
  return s;
}

// IMU record (`kind:"s0_imu"`): the per-channel Allan-deviation curve on log-log
// axes — the standard IMU noise characterization. Slope −½ is white noise (the
// N the filter's measurement noise needs); the flat floor is bias instability.
function s0ImuBody(r) {
  const chans = r.channels ?? [];
  const colours = ['#e6550d', '#fd8d3c', '#fdbe85', '#3182bd', '#6baed6', '#9ecae1']; // gyro xyz warm, accel xyz cool
  // Plot box.
  const m = { l: 70, r: 220, t: 50, b: 56 };
  const px0 = m.l, py0 = m.t, pw = W - m.l - m.r, ph = H - m.t - m.b;
  let s = '';

  // Data range (log-log) over all finite, positive points.
  let tmin = Infinity, tmax = -Infinity, smin = Infinity, smax = -Infinity;
  for (const c of chans)
    for (let i = 0; i < (c.taus ?? []).length; i++) {
      const t = c.taus[i], sd = (c.allan ?? [])[i];
      if (t > 0) { if (t < tmin) tmin = t; if (t > tmax) tmax = t; }
      if (sd > 0) { if (sd < smin) smin = sd; if (sd > smax) smax = sd; }
    }
  const ok = Number.isFinite(tmin) && Number.isFinite(smin) && tmax > tmin && smax > smin;
  const lx0 = Math.log10(tmin), lx1 = Math.log10(tmax), ly0 = Math.log10(smin), ly1 = Math.log10(smax);
  const X = (t) => px0 + ((Math.log10(t) - lx0) / (lx1 - lx0)) * pw;
  const Y = (sd) => py0 + ph - ((Math.log10(sd) - ly0) / (ly1 - ly0)) * ph;

  // Frame + decade gridlines.
  s += `<rect x="${px0}" y="${py0}" width="${pw}" height="${ph}" fill="#0a0a0a" stroke="#444"/>`;
  if (ok) {
    for (let e = Math.ceil(lx0); e <= Math.floor(lx1); e++) {
      const x = px0 + ((e - lx0) / (lx1 - lx0)) * pw;
      s += `<line x1="${x.toFixed(1)}" y1="${py0}" x2="${x.toFixed(1)}" y2="${(py0 + ph).toFixed(1)}" stroke="#333" stroke-width="0.7"/>`;
      s += `<text x="${x.toFixed(1)}" y="${(py0 + ph + 16).toFixed(1)}" text-anchor="middle" font-family="monospace" font-size="10" fill="#bbb">10^${e}</text>`;
    }
    for (let e = Math.ceil(ly0); e <= Math.floor(ly1); e++) {
      const y = py0 + ph - ((e - ly0) / (ly1 - ly0)) * ph;
      s += `<line x1="${px0}" y1="${y.toFixed(1)}" x2="${(px0 + pw).toFixed(1)}" y2="${y.toFixed(1)}" stroke="#333" stroke-width="0.7"/>`;
      s += `<text x="${(px0 - 8).toFixed(1)}" y="${(y + 3).toFixed(1)}" text-anchor="end" font-family="monospace" font-size="10" fill="#bbb">10^${e}</text>`;
    }
    // Per-channel curves.
    chans.forEach((c, ci) => {
      let d = '';
      let started = false;
      for (let i = 0; i < (c.taus ?? []).length; i++) {
        const t = c.taus[i], sd = (c.allan ?? [])[i];
        if (!(t > 0 && sd > 0)) continue;
        d += `${started ? 'L' : 'M'}${X(t).toFixed(1)},${Y(sd).toFixed(1)}`;
        started = true;
      }
      s += `<path d="${d}" fill="none" stroke="${colours[ci % colours.length]}" stroke-width="1.6" opacity="0.95"/>`;
    });
  } else {
    s += `<text x="${(px0 + pw / 2).toFixed(1)}" y="${(py0 + ph / 2).toFixed(1)}" text-anchor="middle" font-family="monospace" font-size="13" fill="#888">stream too short for an Allan curve</text>`;
  }

  // Axis titles.
  s += `<text x="${(px0 + pw / 2).toFixed(1)}" y="${H - 14}" text-anchor="middle" font-family="monospace" font-size="11" fill="#ddd">averaging time τ (s)</text>`;
  s += `<text x="16" y="${(py0 + ph / 2).toFixed(1)}" text-anchor="middle" font-family="monospace" font-size="11" fill="#ddd" transform="rotate(-90 16 ${(py0 + ph / 2).toFixed(1)})">Allan deviation σ_A(τ)</text>`;

  // Legend with each channel's white-noise density N.
  const lgx = W - m.r + 14;
  s += `<text x="${lgx}" y="${py0 + 4}" font-family="monospace" font-size="11" fill="#eee">white-noise density N</text>`;
  chans.forEach((c, ci) => {
    const y = py0 + 22 + ci * 18;
    s += `<line x1="${lgx}" y1="${y - 4}" x2="${lgx + 18}" y2="${y - 4}" stroke="${colours[ci % colours.length]}" stroke-width="2.4"/>`;
    s += `<text x="${lgx + 24}" y="${y}" font-family="monospace" font-size="10" fill="#ddd">${esc(c.name)}  ${(c.N ?? 0).toExponential(2)}</text>`;
  });

  // Title HUD.
  const hud = `S0 IMU noise — ${num(r.rate_hz, 0)} Hz · ${r.n_samples ?? '—'} samples · ${num(r.duration_s, 0)} s`;
  s += `<text x="${px0}" y="32" font-family="monospace" font-size="14" fill="#eee">${esc(hud)}</text>`;
  return s;
}

// ── S5 triangulation inspector tier (image-domain) ────────────────────────────
// Per-frame reprojection residuals: the observed pixel (white ring) vs the
// triangulated landmark's reprojection (coloured dot), the connecting vector
// coloured by error magnitude. A large, structured residual flags a bad
// triangulation (low parallax / outlier) the backend would otherwise trust.
function s5ResidualsBody(r) {
  const w = r.width ?? W, h = r.height ?? H;
  const res = r.residuals ?? [];
  const errs = res.map((p) => p.err).filter((e) => Number.isFinite(e));
  const ref = errs.length ? Math.max(2, ...errs) : 2;
  const eColor = (e) => (Number.isFinite(e) ? ramp(Math.min(1, e / ref)) : '#888');
  let s = '';

  for (const p of res) {
    const c = eColor(p.err);
    s += `<line x1="${p.u.toFixed(1)}" y1="${p.v.toFixed(1)}" x2="${p.pu.toFixed(1)}" y2="${p.pv.toFixed(1)}" stroke="${c}" stroke-width="1.6" opacity="0.95"/>`;
    s += `<circle cx="${p.u.toFixed(1)}" cy="${p.v.toFixed(1)}" r="3.5" fill="none" stroke="#fff" stroke-width="1" opacity="0.85"/>`;
    s += `<circle cx="${p.pu.toFixed(1)}" cy="${p.pv.toFixed(1)}" r="2" fill="${c}"/>`;
  }

  // Error colour legend.
  const lx = w - 130, ly = 24, lw = 110, lh = 10;
  for (let k = 0; k < 22; k++) s += `<rect x="${lx + (k / 22) * lw}" y="${ly}" width="${lw / 22 + 1}" height="${lh}" fill="${ramp(k / 21)}"/>`;
  s += `<rect x="${lx}" y="${ly}" width="${lw}" height="${lh}" fill="none" stroke="#888"/>`;
  s += `<text x="${lx}" y="${ly - 3}" font-family="monospace" font-size="10" fill="#ddd">reproj err (px)</text>`;
  s += `<text x="${lx}" y="${ly + lh + 11}" font-family="monospace" font-size="9" fill="#ddd">0</text>`;
  s += `<text x="${lx + lw}" y="${ly + lh + 11}" text-anchor="end" font-family="monospace" font-size="9" fill="#ddd">${ref.toFixed(1)}</text>`;

  // HUD.
  const mean = errs.length ? errs.reduce((a, b) => a + b, 0) / errs.length : null;
  const hud = [
    `S5 triangulation — frame ${r.frame}  t=${num(r.t)}s`,
    `landmarks reprojected: ${res.length}`,
    `reproj err: mean ${num(mean)}px  max ${num(ref, 1)}px`,
  ];
  const boxH = 16 + hud.length * 18;
  s += `<rect x="8" y="8" width="290" height="${boxH}" rx="5" fill="#000" opacity="0.55"/>`;
  hud.forEach((line, i) => { s += `<text x="18" y="${30 + i * 18}" font-family="monospace" font-size="13" fill="#eee">${esc(line)}</text>`; });
  s += `<text x="8" y="${h - 12}" font-family="monospace" font-size="11" fill="#ddd">white ring = observed   dot = reprojection   vector = residual (colour = px)</text>`;
  return s;
}

let count = 0;
for (const r of records) {
  if (r.kind === 's5_residuals') {
    const w = r.width ?? W, h = r.height ?? H;
    const uri = dataUri(r.image);
    let body = `<rect width="${w}" height="${h}" fill="#000"/>`;
    if (uri) body += `<image href="${uri}" x="0" y="0" width="${w}" height="${h}"/>`;
    body += s5ResidualsBody(r);
    const svg = `<svg xmlns="http://www.w3.org/2000/svg" width="${w}" height="${h}" viewBox="0 0 ${w} ${h}">${body}</svg>`;
    writeFileSync(resolve(outDir, `frame_${String(r.frame).padStart(5, '0')}.svg`), svg);
    ++count;
    continue;
  }
  if (r.kind === 's0_camera') {
    const w = r.width ?? W, h = r.height ?? H;
    const uri = dataUri(r.image);
    let body = `<rect width="${w}" height="${h}" fill="#000"/>`;
    if (uri) body += `<image href="${uri}" x="0" y="0" width="${w}" height="${h}"/>`;
    body += s0CameraBody(r);
    const svg = `<svg xmlns="http://www.w3.org/2000/svg" width="${w}" height="${h}" viewBox="0 0 ${w} ${h}">${body}</svg>`;
    writeFileSync(resolve(outDir, 'distortion.svg'), svg);
    ++count;
    continue;
  }
  if (r.kind === 's0_imu') {
    const body = `<rect width="${W}" height="${H}" fill="#000"/>${s0ImuBody(r)}`;
    const svg = `<svg xmlns="http://www.w3.org/2000/svg" width="${W}" height="${H}" viewBox="0 0 ${W} ${H}">${body}</svg>`;
    writeFileSync(resolve(outDir, 'imu_allan.svg'), svg);
    ++count;
    continue;
  }

  const uri = dataUri(r.image);

  // Frontend-inspector records carry their own dimensions; draw at those so the
  // overlay coordinates (raw pixels) line up with a non-EuRoC frame size.
  if (Array.isArray(r.tracks)) {
    const w = r.width ?? W, h = r.height ?? H;
    let body = `<rect width="${w}" height="${h}" fill="#000"/>`;
    if (uri) body += `<image href="${uri}" x="0" y="0" width="${w}" height="${h}"/>`;
    body += frontendBody(r);
    const svg = `<svg xmlns="http://www.w3.org/2000/svg" width="${w}" height="${h}" viewBox="0 0 ${w} ${h}">${body}</svg>`;
    writeFileSync(resolve(outDir, `frame_${String(r.frame).padStart(5, '0')}.svg`), svg);
    ++count;
    continue;
  }

  let body = '';
  body += `<rect width="${W}" height="${H}" fill="#000"/>`;
  if (uri) body += `<image href="${uri}" x="0" y="0" width="${W}" height="${H}"/>`;

  // True projections, colour-coded by depth (near = warm, far = cool) so the
  // parallax — near dots sweep faster than far ones — reads at a glance.
  const tru = r.true ?? [];
  const dep = r.depth ?? [];
  for (let i = 0; i < tru.length; i++) {
    const c = haveDepth && dep[i] != null ? depthColor(dep[i]) : '#9aa';
    body += `<circle cx="${tru[i][0]}" cy="${tru[i][1]}" r="3" fill="${c}" opacity="0.95"/>`;
  }
  // Live (noisy) feature tracks the filter consumes — thin white ring; the
  // offset from its coloured dot is the additive camera noise.
  for (const p of r.obs ?? [])
    body += `<circle cx="${p[0]}" cy="${p[1]}" r="4.5" fill="none" stroke="#fff" stroke-width="1" opacity="0.85"/>`;

  // Depth colour-bar legend (synthetic).
  if (haveDepth) {
    const lx = W - 130, ly = H - 70, lw = 110, lh = 12;
    for (let k = 0; k < 22; k++)
      body += `<rect x="${lx + (k / 22) * lw}" y="${ly}" width="${lw / 22 + 1}" height="${lh}" fill="${ramp(1 - k / 21)}"/>`;
    body += `<rect x="${lx}" y="${ly}" width="${lw}" height="${lh}" fill="none" stroke="#888"/>`;
    body += `<text x="${lx}" y="${ly - 4}" font-family="monospace" font-size="10" fill="#ddd">depth (m)</text>`;
    body += `<text x="${lx}" y="${ly + lh + 12}" font-family="monospace" font-size="9" fill="#ddd">near ${dmin.toFixed(1)}</text>`;
    body += `<text x="${lx + lw}" y="${ly + lh + 12}" text-anchor="end" font-family="monospace" font-size="9" fill="#ddd">far ${dmax.toFixed(1)}</text>`;
  }

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
  const legend = haveDepth
    ? 'dots = landmarks (colour = depth)   white ring = noisy track'
    : 'white ring = tracked feature';
  body += `<text x="8" y="${H - 12}" font-family="monospace" font-size="11" fill="#ddd">${legend}</text>`;

  const svg = `<svg xmlns="http://www.w3.org/2000/svg" width="${W}" height="${H}" viewBox="0 0 ${W} ${H}">${body}</svg>`;
  const name = `frame_${String(r.frame).padStart(5, '0')}.svg`;
  writeFileSync(resolve(outDir, name), svg);
  ++count;
}
console.log(`wrote ${count} overlay frames to ${outDir}`);
console.log(`rasterize+assemble (needs rsvg-convert + ffmpeg):`);
console.log(`  for f in ${outDir}/*.svg; do rsvg-convert "$f" -o "\${f%.svg}.png"; done`);
console.log(`  ffmpeg -framerate 20 -i ${outDir}/frame_%05d.png -pix_fmt yuv420p ${dir}/scene_overlay.mp4`);
