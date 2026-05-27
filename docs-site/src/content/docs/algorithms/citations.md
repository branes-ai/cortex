---
title: Clean-room Citations & Provenance
description: The published sources every algorithm was implemented from — the no-GPL provenance record.
---

Cortex has a **strict no-GPL rule**, including test oracles: every algorithm is
implemented **clean-room from published papers and mathematical definitions**, never
copied or adapted from GPL or otherwise restrictively-licensed source (in particular,
no `ov_eval` / OpenVINS / `evo` source contact for the metrics). This page is the
provenance record.

## Provenance table

| Component | Source(s) |
|---|---|
| MSCKF measurement model, null-space projection | Mourikis & Roumeliotis, *"A Multi-State Constraint Kalman Filter for Vision-aided Inertial Navigation"* (ICRA 2007) |
| Error-state propagation / indirect Kalman filter | Trawny & Roumeliotis, *"Indirect Kalman Filter for 3D Attitude Estimation"*; Solà, *"Quaternion kinematics for the error-state Kalman filter"* (2017) |
| IMU preintegration | Forster et al., *"On-Manifold Preintegration for Real-Time Visual-Inertial Odometry"* (RSS 2015 / T-RO 2017) |
| Dynamic visual-inertial initialization | Qin et al., *"VINS-Mono"* (T-RO 2018); Martinelli, *"Closed-Form Solution of Visual-Inertial Structure from Motion"* (IJCV 2014) |
| Lie groups (SO3/SE3/Sim3), Jacobians | Solà, Deray & Atchuthan, *"A micro Lie theory for state estimation in robotics"* (2018); Barfoot, *"State Estimation for Robotics"* |
| FAST corner detection | Rosten & Drummond, *"Machine learning for high-speed corner detection"* (ECCV 2006) |
| KLT tracking | Lucas & Kanade (1981); Bouguet, *"Pyramidal Implementation of the Lucas-Kanade Feature Tracker"* |
| Camera models | Brown–Conrady (radtan); Kannala & Brandt (equidistant fisheye, 2006); Mei & Rives (unified omnidirectional, 2007) |
| Trajectory alignment for ATE | Horn, *"Closed-form solution of absolute orientation using unit quaternions"* (JOSA-A 1987) |
| Symmetric eigendecomposition (Jacobi) | Golub & Van Loan, *"Matrix Computations"* (textbook cyclic Jacobi) |
| Non-linear least squares (GN / LM) | Nocedal & Wright, *"Numerical Optimization"*; Levenberg (1944); Marquardt (1963) |

## License compatibility

The third-party dependencies were audited up front (ADR-0001, *third-party license
compatibility*) and pulled in via CMake `FetchContent`: MTL5, Universal, yaml-cpp,
Catch2, Tracy, stb — all permissively licensed and compatible with the project's
distribution model. The one standing exception to `FetchContent` is MLIR/LLVM (when
introduced), which is consumed as pre-built binaries.

## Why this matters

The clean-room discipline is what lets Cortex be delivered as OCI containers and a
Yocto layer without license entanglement, and it's why, for example, the ATE metric
was re-derived from Horn's paper (with a Jacobi eigensolver) rather than borrowed from
an existing evaluation toolkit. When in doubt, the rule is: implement from the math,
cite the paper, write the test oracle independently.
