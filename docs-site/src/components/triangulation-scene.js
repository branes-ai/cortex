// triangulation-scene.js — a small, static Three.js illustration of the MSCKF
// camera measurement geometry (the worked example in the Camera Updaters page).
//
// It draws the two-view triangulation the math walks through: a world feature p_f,
// two camera poses (the clones) looking along +z, the two rays that meet at p_f,
// and a unit-distance image plane per camera with a dot at the normalized
// projection — so the parallax (the feature's image x differs between views) is
// visible directly. Numbers match the doc exactly (everything at y = 0, so the
// scene lays out on the ground plane). Orbit to rotate, scroll to zoom.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// ── The worked example (metres), identical to the page's numbers table ────────
const PF = [1.0, 0.0, 4.0];          // world feature
const CAM = [                        // camera centres = IMU + extrinsic p_ic (+5 cm x)
  { name: 'clone 0', c: [0.05, 0.0, 0.0], imu: [0.0, 0.0, 0.0], n: [0.2375, 0.0] },
  { name: 'clone 1', c: [0.25, 0.0, 0.0], imu: [0.2, 0.0, 0.0], n: [0.1875, 0.0] },
];
const COL = { feature: 0xff9f0a, ray: 0x35c759, cam: 0x4fd1ff, imu: 0x8aa6bf, plane: 0xffd60a };

// A text label as a camera-facing sprite (canvas texture).
function label(text, pos, { size = 0.34, color = '#e6edf3' } = {}) {
  const pad = 8, fs = 64;
  const cv = document.createElement('canvas');
  const ctx = cv.getContext('2d');
  ctx.font = `${fs}px ui-monospace, monospace`;
  cv.width = ctx.measureText(text).width + pad * 2;
  cv.height = fs + pad * 2;
  ctx.font = `${fs}px ui-monospace, monospace`;
  ctx.fillStyle = color;
  ctx.textBaseline = 'middle';
  ctx.fillText(text, pad, cv.height / 2);
  const tex = new THREE.CanvasTexture(cv);
  tex.anisotropy = 4;
  const spr = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, transparent: true, depthTest: false }));
  spr.scale.set((size * cv.width) / cv.height, size, 1);
  spr.position.set(...pos);
  return spr;
}

function lineSeg(points, color, opacity = 1) {
  const g = new THREE.BufferGeometry().setFromPoints(points.map((p) => new THREE.Vector3(...p)));
  return new THREE.LineSegments(g, new THREE.LineBasicMaterial({ color, transparent: opacity < 1, opacity }));
}

// A wireframe camera pyramid: apex at `c`, opening along +z (no rotation in the
// example), `d` deep with half-extent `h` in normalized image units.
function frustum(c, d = 0.7, h = 0.32, color = COL.cam) {
  const a = new THREE.Vector3(...c);
  const corner = (sx, sy) => new THREE.Vector3(c[0] + sx * h * d, c[1] + sy * h * d, c[2] + d);
  const k = [corner(1, 1), corner(-1, 1), corner(-1, -1), corner(1, -1)];
  const segs = [a, k[0], a, k[1], a, k[2], a, k[3], k[0], k[1], k[1], k[2], k[2], k[3], k[3], k[0]];
  const g = new THREE.BufferGeometry().setFromPoints(segs);
  return new THREE.LineSegments(g, new THREE.LineBasicMaterial({ color }));
}

/**
 * Mount the static triangulation scene.
 * @param {{canvas:HTMLCanvasElement, hud?:HTMLElement}} cfg
 * @returns {{fitView:()=>void, dispose:()=>void}}
 */
export function mountTriangulation({ canvas, hud }) {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
  renderer.setPixelRatio(Math.min(globalThis.devicePixelRatio || 1, 2));
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0e1116);
  scene.add(new THREE.AmbientLight(0xffffff, 0.8));
  const key = new THREE.DirectionalLight(0xffffff, 0.7);
  key.position.set(2, 4, -3);
  scene.add(key);

  const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 1000);
  const target = new THREE.Vector3(0.5, 0.0, 1.9);
  const home = new THREE.Vector3(3.6, 2.4, -2.2);
  const controls = new OrbitControls(camera, canvas);
  controls.enableDamping = true;
  controls.target.copy(target);

  // Ground plane (y = 0) + world axes with labels.
  const grid = new THREE.GridHelper(8, 16, 0x335577, 0x223344);
  grid.position.set(0.5, 0, 1.8);
  scene.add(grid);
  scene.add(new THREE.AxesHelper(0.6));
  scene.add(label('X', [0.72, 0, 0], { size: 0.22, color: '#ff6b6b' }));
  scene.add(label('Y', [0, 0.72, 0], { size: 0.22, color: '#7CFC9A' }));
  scene.add(label('Z (forward)', [0, 0, 0.78], { size: 0.22, color: '#4fd1ff' }));

  // Feature.
  scene.add(new THREE.Mesh(
    new THREE.SphereGeometry(0.07, 20, 14),
    new THREE.MeshStandardMaterial({ color: COL.feature, emissive: COL.feature, emissiveIntensity: 0.4 }),
  ).translateX(PF[0]).translateY(PF[1]).translateZ(PF[2]));
  scene.add(label('p_f = (1.0, 0, 4.0)', [PF[0] + 0.12, PF[1] + 0.18, PF[2]], { color: '#ffcf80' }));

  // Per camera: IMU box, extrinsic offset, frustum, ray to the feature, image
  // plane at unit depth with the projected dot.
  for (const cam of CAM) {
    const imuBox = new THREE.Mesh(
      new THREE.BoxGeometry(0.06, 0.06, 0.06),
      new THREE.MeshStandardMaterial({ color: COL.imu }),
    );
    imuBox.position.set(...cam.imu);
    scene.add(imuBox);
    scene.add(lineSeg([cam.imu, cam.c], COL.imu, 0.8)); // the 5 cm extrinsic p_ic
    scene.add(frustum(cam.c));
    scene.add(lineSeg([cam.c, PF], COL.ray)); // ray to feature (triangulation)
    scene.add(label(cam.name, [cam.c[0], cam.c[1] - 0.22, cam.c[2] - 0.05], { size: 0.26 }));

    // Unit-depth image plane (z = c_z + 1) with the normalized projection dot.
    const z = cam.c[2] + 1;
    const ext = 0.42;
    const planePts = [
      [cam.c[0] - ext, cam.c[1] - ext, z], [cam.c[0] + ext, cam.c[1] - ext, z],
      [cam.c[0] + ext, cam.c[1] - ext, z], [cam.c[0] + ext, cam.c[1] + ext, z],
      [cam.c[0] + ext, cam.c[1] + ext, z], [cam.c[0] - ext, cam.c[1] + ext, z],
      [cam.c[0] - ext, cam.c[1] + ext, z], [cam.c[0] - ext, cam.c[1] - ext, z],
    ];
    scene.add(lineSeg(planePts, COL.plane, 0.55));
    const dot = new THREE.Mesh(
      new THREE.SphereGeometry(0.03, 12, 10),
      new THREE.MeshBasicMaterial({ color: COL.plane }),
    );
    dot.position.set(cam.c[0] + cam.n[0], cam.c[1] + cam.n[1], z);
    scene.add(dot);
    scene.add(label(`x = ${cam.n[0]}`, [cam.c[0] + cam.n[0] + 0.06, cam.c[1] - 0.16, z], { size: 0.2, color: '#ffe9a8' }));
  }

  if (hud) hud.textContent = 'one feature, two views — the rays meet at p_f (triangulation); the image dots differ in x (parallax → depth)';

  function fitView() {
    camera.position.copy(home);
    controls.target.copy(target);
    controls.update();
  }
  function resize() {
    const w = canvas.clientWidth || 1, h = canvas.clientHeight || 1;
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }
  let raf = 0;
  function tick() {
    controls.update();
    renderer.render(scene, camera);
    raf = requestAnimationFrame(tick);
  }
  const ro = new ResizeObserver(resize);
  ro.observe(canvas);
  resize();
  fitView();
  tick();

  return {
    fitView,
    dispose() {
      cancelAnimationFrame(raf);
      ro.disconnect();
      controls.dispose();
      renderer.dispose();
    },
  };
}
