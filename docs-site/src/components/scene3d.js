// scene3d.js — a framework-agnostic Three.js viewer for the VIO pipeline's
// drone path + pose stream (run.jsonl). Renders the ground-truth and estimated
// trajectories, a moving 6-DOF pose triad, and — the point of the 3-D view — the
// filter's position-covariance ellipsoid, so the #212 over-confidence reads at a
// glance: when the ellipsoid is too small and the green GT marker sits outside it,
// the filter is sure about a wrong position.
//
// Used both ways:
//   • interactive — mounted in an Astro island (Scene3D.astro), orbit + scrub;
//   • offline     — the same page driven frame-by-frame by a headless browser to
//                   capture PNGs → ffmpeg (scripts/gen-scene3d-video.mjs).
//
// Input is the parsed array of run.jsonl records:
//   {type:"gt",  t, q:[w,x,y,z], p:[x,y,z]}
//   {type:"est", t, q:[w,x,y,z], p:[x,y,z], pos_err, pcov:[9 row-major]}

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { eigSym3 } from './scene3d-math.js';

// THREE quaternion (x,y,z,w) from a run.jsonl [w,x,y,z]; identity if absent.
function quat(q) {
  return q ? new THREE.Quaternion(q[1], q[2], q[3], q[0]) : new THREE.Quaternion();
}

/**
 * Mount the viewer.
 * @param {{canvas:HTMLCanvasElement, hud?:HTMLElement, records:Array, options?:object}} cfg
 * @returns {{setFrame:(i:number)=>void, frameCount:number, play:()=>void, pause:()=>void, dispose:()=>void, fitView:()=>void}}
 */
export function createViewer({ canvas, hud, records, options = {} }) {
  const sigmaK = options.sigmaK ?? 3; // ellipsoid radius in σ (3σ ≈ 97% per-axis)
  const gt = records.filter((r) => r.type === 'gt');
  const est = records.filter((r) => r.type === 'est');
  const n = Math.max(gt.length, est.length);

  // ── Renderer / scene / camera (world is z-up) ─────────────────────────────
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false });
  renderer.setPixelRatio(Math.min(globalThis.devicePixelRatio || 1, 2));
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0e1116);

  const camera = new THREE.PerspectiveCamera(50, 1, 0.01, 1000);
  camera.up.set(0, 0, 1);
  const controls = new OrbitControls(camera, canvas);
  controls.enableDamping = true;

  scene.add(new THREE.AmbientLight(0xffffff, 0.7));
  const key = new THREE.DirectionalLight(0xffffff, 0.8);
  key.position.set(2, 3, 5);
  scene.add(key);

  // ── World bounds (for grid + camera fit) over the GT path ─────────────────
  const pts = [...gt, ...est].map((r) => r.p).filter(Boolean);
  const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
  for (const p of pts) for (let k = 0; k < 3; k++) { lo[k] = Math.min(lo[k], p[k]); hi[k] = Math.max(hi[k], p[k]); }
  const ctr = lo.map((v, k) => (v + hi[k]) / 2);
  const span = Math.max(1e-3, ...lo.map((v, k) => hi[k] - v));

  // Ground grid in the world xy-plane at the floor of the trajectory.
  const grid = new THREE.GridHelper(Math.ceil(span * 2.5), 20, 0x335577, 0x223344);
  grid.rotation.x = Math.PI / 2; // GridHelper is xz by default → rotate into xy
  grid.position.set(ctr[0], ctr[1], lo[2]);
  scene.add(grid);

  // ── Trajectory polylines ──────────────────────────────────────────────────
  function polyline(recs, color) {
    const g = new THREE.BufferGeometry().setFromPoints(recs.map((r) => new THREE.Vector3(...r.p)));
    const line = new THREE.Line(g, new THREE.LineBasicMaterial({ color }));
    scene.add(line);
    return line;
  }
  polyline(gt, 0x35c759); // ground truth — green
  polyline(est, 0xff9f0a); // estimate — orange

  // ── Moving pose triads (est solid, gt faint) ──────────────────────────────
  const triadEst = new THREE.AxesHelper(span * 0.12);
  scene.add(triadEst);
  const triadGt = new THREE.AxesHelper(span * 0.09);
  triadGt.material.opacity = 0.35;
  triadGt.material.transparent = true;
  scene.add(triadGt);
  const gtMarker = new THREE.Mesh(
    new THREE.SphereGeometry(span * 0.015, 16, 12),
    new THREE.MeshBasicMaterial({ color: 0x35c759 }),
  );
  scene.add(gtMarker);

  // ── Covariance ellipsoid (unit sphere, scaled+oriented per frame) ─────────
  const ellipsoid = new THREE.Mesh(
    new THREE.SphereGeometry(1, 24, 18),
    new THREE.MeshPhongMaterial({ color: 0xff9f0a, transparent: true, opacity: 0.22, depthWrite: false }),
  );
  scene.add(ellipsoid);
  const ellipsoidWire = new THREE.LineSegments(
    new THREE.WireframeGeometry(new THREE.SphereGeometry(1, 16, 10)),
    new THREE.LineBasicMaterial({ color: 0xffd60a, transparent: true, opacity: 0.4 }),
  );
  scene.add(ellipsoidWire);

  function setEllipsoid(p, pcov) {
    if (!pcov) { ellipsoid.visible = ellipsoidWire.visible = false; return; }
    ellipsoid.visible = ellipsoidWire.visible = true;
    const { values, vectors } = eigSym3(pcov);
    // Orientation: columns are the principal axes; build a rotation matrix.
    const basis = new THREE.Matrix4().makeBasis(
      new THREE.Vector3(...vectors[0]),
      new THREE.Vector3(...vectors[1]),
      new THREE.Vector3(...vectors[2]),
    );
    const q = new THREE.Quaternion().setFromRotationMatrix(basis);
    const s = values.map((v) => sigmaK * Math.sqrt(Math.max(v, 0)));
    for (const e of [ellipsoid, ellipsoidWire]) {
      e.position.set(...p);
      e.quaternion.copy(q);
      e.scale.set(Math.max(s[0], 1e-4), Math.max(s[1], 1e-4), Math.max(s[2], 1e-4));
    }
    return s; // 1σ·k principal radii (m)
  }

  // ── Per-frame update ──────────────────────────────────────────────────────
  let frame = 0;
  function setFrame(i) {
    frame = Math.max(0, Math.min(n - 1, Math.round(i)));
    const e = est[Math.min(frame, est.length - 1)];
    const g = gt[Math.min(frame, gt.length - 1)];
    if (e?.p) { triadEst.position.set(...e.p); triadEst.quaternion.copy(quat(e.q)); }
    if (g?.p) {
      triadGt.position.set(...g.p); triadGt.quaternion.copy(quat(g.q));
      gtMarker.position.set(...g.p);
    }
    const radii = e ? setEllipsoid(e.p, e.pcov) : null;
    if (hud) {
      const sig = e?.pcov ? Math.sqrt((e.pcov[0] + e.pcov[4] + e.pcov[8]) / 3) : null;
      const overConf = e?.pos_err != null && sig != null && e.pos_err > sigmaK * sig;
      hud.innerHTML =
        `<div>frame ${frame + 1}/${n} &nbsp; t=${(e?.t ?? g?.t ?? 0).toFixed(2)}s</div>` +
        (e?.pos_err != null ? `<div>pos err: ${e.pos_err.toFixed(3)} m</div>` : '') +
        (sig != null ? `<div>pos 1σ: ${(sig * 100).toFixed(1)} cm</div>` : '') +
        (overConf
          ? `<div class="warn">⚠ over-confident — true error &gt; ${sigmaK}σ ellipsoid</div>`
          : '');
    }
    options.onFrame?.(frame);
  }

  // ── Camera fit + render loop ──────────────────────────────────────────────
  function fitView() {
    controls.target.set(...ctr);
    const d = span * 2.2;
    camera.position.set(ctr[0] + d, ctr[1] - d, lo[2] + d * 0.9);
    camera.near = span / 100; camera.far = span * 50; camera.updateProjectionMatrix();
    controls.update();
  }

  function resize() {
    const w = canvas.clientWidth || 800, h = canvas.clientHeight || 480;
    renderer.setSize(w, h, false);
    camera.aspect = w / h; camera.updateProjectionMatrix();
  }

  let playing = false, raf = 0, last = 0;
  function tick(ts) {
    raf = requestAnimationFrame(tick);
    if (playing && est.length > 1) {
      if (ts - last > 1000 / (options.fps ?? 20)) { last = ts; setFrame((frame + 1) % n); }
    }
    controls.update();
    renderer.render(scene, camera);
  }

  const ro = new ResizeObserver(resize);
  ro.observe(canvas);
  resize(); fitView(); setFrame(0);
  raf = requestAnimationFrame(tick);

  return {
    frameCount: n,
    setFrame,
    play() { playing = true; },
    pause() { playing = false; },
    fitView,
    dispose() { cancelAnimationFrame(raf); ro.disconnect(); controls.dispose(); renderer.dispose(); },
  };
}

// Parse a run.jsonl text blob into records (skips blank / malformed lines).
export function parseRunJsonl(text) {
  return text.trim().split('\n').filter(Boolean).map((l) => {
    try { return JSON.parse(l); } catch { return null; }
  }).filter(Boolean);
}
