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
export function createViewer({ canvas, hud, records, scene: sceneData, options = {} }) {
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

  // ── World bounds (for grid + camera fit): the path AND the landmark cloud, so
  // the drone and the scene it observes are both framed. ────────────────────
  const pts = [...gt, ...est].map((r) => r.p).filter(Boolean);
  if (sceneData?.landmarks) pts.push(...sceneData.landmarks);
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

  // ── Landmark cloud (the static 3-D scene the camera observes) ─────────────
  if (sceneData?.landmarks?.length) {
    const g = new THREE.BufferGeometry().setFromPoints(sceneData.landmarks.map((p) => new THREE.Vector3(...p)));
    const points = new THREE.Points(
      g,
      new THREE.PointsMaterial({ color: 0x8aa6bf, size: span * 0.012, sizeAttenuation: true, transparent: true, opacity: 0.85 }),
    );
    scene.add(points);
  }

  // ── Per-landmark covariance ellipsoids (the S5 triangulation tier) ────────
  // When the scene carries `landmark_cov` (parallel to `landmarks`, each a
  // row-major 3×3 position covariance in m²), draw one static σ-ellipsoid per
  // landmark — so triangulation uncertainty (large for low-parallax features)
  // reads directly. Reuses the same eigSym3 → orient+scale machinery as the
  // pose ellipsoid. Opt-in: absent field → nothing drawn.
  if (sceneData?.landmark_cov?.length && sceneData?.landmarks?.length) {
    const sharedGeo = new THREE.SphereGeometry(1, 12, 8);
    const mat = new THREE.MeshPhongMaterial({ color: 0x4fd1ff, transparent: true, opacity: 0.16, depthWrite: false });
    const n = Math.min(sceneData.landmarks.length, sceneData.landmark_cov.length);
    for (let i = 0; i < n; i++) {
      const cov = sceneData.landmark_cov[i];
      if (!cov || cov.some((v) => !Number.isFinite(v))) continue;
      const { values, vectors } = eigSym3(cov);
      const basis = new THREE.Matrix4().makeBasis(
        new THREE.Vector3(...vectors[0]),
        new THREE.Vector3(...vectors[1]),
        new THREE.Vector3(...vectors[2]),
      );
      const q = new THREE.Quaternion().setFromRotationMatrix(basis);
      const s = values.map((v) => sigmaK * Math.sqrt(Math.max(v, 0)));
      const e = new THREE.Mesh(sharedGeo, mat);
      e.position.set(...sceneData.landmarks[i]);
      e.quaternion.copy(q);
      e.scale.set(Math.max(s[0], 1e-5), Math.max(s[1], 1e-5), Math.max(s[2], 1e-5));
      scene.add(e);
    }
  }

  // ── Estimated-camera frustum (rebuilt per frame from the pose + extrinsics) ─
  let frustum = null, fRays = null, qImuCam = null, pImuCam = null, fDepth = 0;
  if (sceneData?.camera?.frustum_rays) {
    const cam = sceneData.camera;
    fRays = cam.frustum_rays.map((r) => new THREE.Vector3(...r));
    qImuCam = cam.R_imu_cam
      ? new THREE.Quaternion(cam.R_imu_cam[1], cam.R_imu_cam[2], cam.R_imu_cam[3], cam.R_imu_cam[0])
      : new THREE.Quaternion();
    pImuCam = new THREE.Vector3(...(cam.p_imu_cam ?? [0, 0, 0]));
    fDepth = span * 0.4;
    // 8 segments: 4 apex→corner + the 4 base edges → 16 vertices.
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(16 * 3), 3));
    frustum = new THREE.LineSegments(geo, new THREE.LineBasicMaterial({ color: 0x4fd1ff, transparent: true, opacity: 0.85 }));
    scene.add(frustum);
  }

  function setFrustum(estP, q) {
    if (!frustum) return;
    const qWorldImu = q;
    const camCenter = pImuCam.clone().applyQuaternion(qWorldImu).add(new THREE.Vector3(...estP));
    const qWorldCam = qWorldImu.clone().multiply(qImuCam);
    const c = fRays.map((r) => r.clone().applyQuaternion(qWorldCam).multiplyScalar(fDepth).add(camCenter));
    const a = camCenter;
    const segs = [a, c[0], a, c[1], a, c[2], a, c[3], c[0], c[1], c[1], c[2], c[2], c[3], c[3], c[0]];
    const pos = frustum.geometry.attributes.position;
    segs.forEach((v, i) => pos.setXYZ(i, v.x, v.y, v.z));
    pos.needsUpdate = true;
  }

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
    if (e?.p) {
      triadEst.position.set(...e.p);
      triadEst.quaternion.copy(quat(e.q));
      setFrustum(e.p, quat(e.q));
    }
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
