---
title: MSCKF Backend
description: How the backend wires initialization, propagation, the clone window, and camera updates into one VioBackend.
---

`MsckfBackend<T>` (`sdk/include/branes/sdk/msckf_backend.hpp`, #35) is the
`VioBackend` implementation that wires the Phase-3 pieces into one estimator. It is
the full-covariance MSCKF.

## The `VioBackend` interface

The backend implements a small abstraction (#34) that separates the front end from
the estimator:

```cpp
void initialize(const VioConfig&);
void process_imu(const ImuMeasurement<T>&);          // high rate
void process_camera(double t, span<const FrontendObservation<T>>);  // frame rate
NavState<T> current_state() const;
```

Swapping MSCKF for a different backend requires no front-end change — see the
[skeleton](#a-second-backend-the-sliding-window-skeleton) below.

## What it wires together

- **Initialization** — buffers an opening window of IMU samples and runs the
  [static initializer](/vio/imu/); if the platform never settles it falls back to an
  identity attitude (a moving start is left to dynamic VI init).
- **Propagation** — between IMU samples, per-sample mean + covariance propagation via
  the [`Propagator`](/vio/msckf-state/), driven by `dt` from the timestamps.
- **Clone window** — every camera frame augments a clone; the window is bounded by
  `config.max_clones` and marginalized **oldest-first**. Crucially, `process_camera`
  first **propagates the state to the image timestamp** (zero-order hold on the last
  IMU sample) *before* cloning, so each clone is anchored at exactly the frame time
  even on an asynchronous IMU/camera feed.
- **Feature management** — the standard MSCKF policy: a feature track is fed to the
  [camera updater](/vio/camera-updaters/) when it **ends** (no longer observed) or
  when the **oldest clone it touches is about to be dropped**.

## Robustness details

- Observations are **pixel coordinates** per the `VioBackend` contract; each camera's
  model unprojects them to bearings, keeping the estimator core intrinsics-agnostic.
- An **out-of-order or duplicate IMU sample** (`dt ≤ 0`) is dropped without advancing
  the filter time, so a later valid sample can't integrate across already-processed
  time.

## A second backend: the sliding-window skeleton

`SlidingWindowBackend<T>` (#36) is a deliberate placeholder implementing the same
`VioBackend` interface but performing no estimation — every operation raises a
`NotImplemented` marker, and a `kImplemented = false` constant advertises that. Its
purpose is to **prove the backend abstraction is real and substitutable** before
MSCKF accreted implementation-specific assumptions: a `VioEstimator` can be templated
on it and link. The real sliding-window-optimization math (keyframe marginalization,
factor-graph solve over the preintegration and reprojection factors) lands post-MVP.

## Validated

The backend's end-to-end test runs a stationary init then frames under acceleration
and asserts that it **produces finite, forward-moving poses** while the covariance
stays PSD and the clone window respects `max_clones`. Trajectory accuracy on real
data is the [EuRoC harness's](/benchmarks/accuracy/) job (#46).
