// scene3d-math.js — the three-free numerics behind the covariance ellipsoid, in
// their own module so they can be unit-tested headlessly (no WebGL/DOM) and
// reused by the offline video renderer.

/**
 * Symmetric 3×3 eigen-decomposition via cyclic Jacobi rotations.
 * @param {number[]} m row-major 3×3 (9 numbers), assumed symmetric.
 * @returns {{values:number[], vectors:number[][]}} eigenvalues and
 *          eigenvectors (vectors[i] is the unit eigenvector for values[i]).
 */
export function eigSym3(m) {
  const a = [
    [m[0], m[1], m[2]],
    [m[3], m[4], m[5]],
    [m[6], m[7], m[8]],
  ];
  const v = [
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
  ];
  for (let sweep = 0; sweep < 24; sweep++) {
    let p = 0, q = 1, off = Math.abs(a[0][1]);
    if (Math.abs(a[0][2]) > off) { off = Math.abs(a[0][2]); p = 0; q = 2; }
    if (Math.abs(a[1][2]) > off) { off = Math.abs(a[1][2]); p = 1; q = 2; }
    if (off < 1e-14) break;
    const app = a[p][p], aqq = a[q][q], apq = a[p][q];
    const phi = 0.5 * Math.atan2(2 * apq, aqq - app);
    const c = Math.cos(phi), s = Math.sin(phi);
    for (let k = 0; k < 3; k++) {
      const akp = a[k][p], akq = a[k][q];
      a[k][p] = c * akp - s * akq;
      a[k][q] = s * akp + c * akq;
    }
    for (let k = 0; k < 3; k++) {
      const apk = a[p][k], aqk = a[q][k];
      a[p][k] = c * apk - s * aqk;
      a[q][k] = s * apk + c * aqk;
    }
    for (let k = 0; k < 3; k++) {
      const vkp = v[k][p], vkq = v[k][q];
      v[k][p] = c * vkp - s * vkq;
      v[k][q] = s * vkp + c * vkq;
    }
  }
  return {
    values: [a[0][0], a[1][1], a[2][2]],
    // column i of v is eigenvector i → return as row-accessible vectors[i].
    vectors: [
      [v[0][0], v[1][0], v[2][0]],
      [v[0][1], v[1][1], v[2][1]],
      [v[0][2], v[1][2], v[2][2]],
    ],
  };
}
