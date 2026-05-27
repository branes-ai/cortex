---
title: Camera Models
description: Pinhole+radtan, equidistant fisheye, and unified omnidirectional projection models with analytical Jacobians.
---

`cameras.hpp` + `cameras/*` provide the projection models the front end uses to map
between 3D camera-frame points and image pixels. Each is header-only, type-generic,
and provides **project / unproject / distort / undistort** plus the analytical
`d(pixel)/d(point)` Jacobian.

## The three models

| Model | Header | Use |
|---|---|---|
| **Pinhole + radial-tangential** | `cameras/pinhole_radtan.hpp` | The standard EuRoC MAV model (Brown–Conrady 5-parameter distortion). The default for the VIO benchmarks. |
| **Equidistant fisheye** | `cameras/equidistant.hpp` | Kannala–Brandt model for wide-FOV lenses. |
| **Unified omnidirectional** | `cameras/unified.hpp` | Mei–Rives model for catadioptric / very-wide cameras. |

## Pinhole + radtan in detail

`PinholeRadtanCamera<T>` carries intrinsics `(fx, fy, cx, cy)` and a 5-parameter
radtan distortion `(k1, k2, p1, p2, k3)`:

- **`project(p)`** — normalizes `(x/z, y/z)`, applies distortion, scales by the
  intrinsics → pixel.
- **`unproject(px)`** — inverts intrinsics, undistorts, and returns a **unit bearing**
  (camera frame, +Z forward).
- **`project_jacobian(p)`** — the analytical 2×3 `d(pixel)/d(point)`.

The default-constructed camera is the **identity pinhole** (`fx=fy=1, cx=cy=0`, no
distortion), for which `unproject` round-trips normalized coordinates. This is what
lets tests and pre-rectified front ends feed normalized coordinates directly while
production code supplies a real calibration.

## How the estimator uses them

The MSCKF core never sees these models directly — it works in **normalized image
coordinates** (see [Layering Invariants](/architecture/layering/)). The
[`VioEstimator`](/vio/vio-estimator/) holds one camera model per camera id and uses
`unproject` to convert incoming pixel observations to bearings before handing them to
the [camera updater](/vio/camera-updaters/). Swapping a pinhole for a fisheye is
therefore a front-end configuration change, invisible to the estimator math.
