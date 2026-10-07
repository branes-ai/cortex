---
title: VioEstimator API
description: The top-level, lifecycle-gated façade that owns the front end and the selected backend.
---

`VioEstimator<T, Backend = MsckfBackend<T>>`
(`sdk/include/branes/sdk/vio_estimator.hpp`, #45) is the public façade the daemons
drive. It owns the visual [front end](/vio/front-end/) and the selected estimator
[backend](/vio/msckf-backend/), and exposes a small, **lifecycle-gated** API.

## The API

```cpp
void configure(const VioConfig&);              // fix parameters, init the backend
void activate();   void deactivate();          // managed lifecycle
void teardown();
void feed_imu(std::span<const ImuMeasurement<T>>);     // inputs by view
void feed_image(double t, cv::Image<const std::uint8_t>);
Pose      current_pose()  const;               // SE3 T_world_imu
NavState  current_state() const;
void      reset();
```

Inputs are **non-owning views** — an `Image` view for frames, a span for IMU batches;
the estimator copies nothing it doesn't need to retain.

## Lifecycle — mirroring the Rust RM

The estimator's states mirror the Resource Manager's managed lifecycle
([Resource Manager](/architecture/resource-manager/)):

```text
Unconfigured ──configure──▶ Inactive ──activate──▶ Active
                              ▲                       │
                              └────deactivate/reset───┘   teardown ─▶ (terminal)
```

Measurements are consumed **only while `Active`** — `feed_imu`/`feed_image` are no-ops
otherwise. This honors the "no dynamic reconfiguration on the hot path" rule:
parameters are fixed at `configure`, and `configure` is ignored while `Active`. To
change them, `deactivate()` then `configure()` (which also clears runtime state) and
`activate()` again. After `teardown()` the estimator is spent; construct a new one.

## What `feed_image` does

It runs the front-end loop (pyramid → KLT-track existing features → re-detect FAST
corners, suppressing near existing tracks → assign stable ids), producing one
`FrontendObservation` per surviving track, and hands them to the backend's
`process_camera`. A telemetry accessor, `num_tracked_features()`, exposes the current
track count.

## Backend substitutability

Because `Backend` is a template parameter constrained only by the `VioBackend`
interface, the estimator can be instantiated on `MsckfBackend<T>` (the default) or on
the [`SlidingWindowBackend<T>`](/vio/msckf-backend/#a-second-backend-the-sliding-window-skeleton)
skeleton — the front end and the public API are identical either way.

## Validated

Tests cover the **lifecycle gating** (feeds ignored unless Active; correct
transitions; `reset`/`teardown`) and a **full front-end → backend run** on translating
synthetic frames that detects and persistently tracks features, keeps the covariance
PSD and the clone window bounded, and produces a finite pose. The runnable EuRoC
replay example lives in the [dataset harness](/benchmarks/accuracy/) (#46).
