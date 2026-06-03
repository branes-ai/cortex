# VIO over-confidence — the consistency-sweep diagnosis (root cause of #212)

A record of the diagnostic chain that turned "the MSCKF drifts ~18% RPE and
nobody knows why" (#212) into a localized, evidence-backed root cause. It is the
payoff of the instrumentation program (epic #261): instead of editing across the
pipeline and reading end-to-end ATE, we drove two well-defined noise knobs and
read the filter-consistency instrument (`eval/consistency.hpp`,
`eval/nav_consistency.hpp`). Companion to
[`vio-diagnostic-methodology.md`](vio-diagnostic-methodology.md) (why ATE-only
debugging fails) and [`filter-consistency-nees-nis.md`](filter-consistency-nees-nis.md)
(what NEES/NIS mean).

**Bottom line up front:** the over-confidence is **not** a mistuned noise dial.
The true measurement noise *is* ~4× larger and the process noise ~10× larger than
configured — but setting them honestly makes the trajectory *worse*, not better.
Accuracy currently *requires* a dishonest (over-confident) covariance, which means
the over-confidence is **compensating for a structural estimator error**. The
fingerprint is motion-dependent, pointing at an EKF observability / linearization
inconsistency (the classic MSCKF spurious-information-gain). **#212 is an
estimator fix, not a config fix.**

---

## A. The symptom the instrument exposed

Running the instrumented `vio_euroc` benchmark, the MSCKF is **systemically
over-confident on every sequence** — invisible to ATE, which looked fine
(sub-metre on the well-tracked runs):

| sequence (static init) | ATE | NIS (E=1) | NEES (E=1) |
|---|---|---|---|
| MH_05_difficult | 0.79 m | 2.2 | 10 |
| V2_03_difficult | 0.29 m | **14.8** | **115** |

Per-block NEES localized it to **attitude**, worst under fast rotation
(V2_03 attitude=169 vs position=3.5; MH_05 attitude=7). The over-confidence
*scales with motion aggressiveness* — the first hint of a linearization/observability
cause rather than a constant mistune.

NEES ≫ NIS said the propagation side was the worse offender, but both were broken,
so we drove each noise independently to see what each one explains.

## B. The three sweeps

Two diagnostic knobs were added to the benchmark (read at runtime, no rebuild):
`CORTEX_Q_SCALE` multiplies the IMU process-noise densities, `CORTEX_R_SCALE`
multiplies the visual measurement noise (`VioConfig::camera_noise_normalized`,
newly wired from the previously-hardcoded `0.01`). All numbers are the run-average
**normalized** statistic (1.0 = consistent; >1 over-confident).

### B.1 Process-noise sweep — `Q` is ~10× too small

| Q_SCALE | V2_03 ATE | V2_03 NEES | V2_03 NIS | MH_05 ATE | MH_05 NEES | MH_05 NIS |
|---|---|---|---|---|---|---|
| 1  | 0.293 | 115.3 | 14.80 | 0.785 | 10.1 | 2.18 |
| 3  | 0.346 | 41.5  | 14.32 | 0.702 | 12.4 | 2.20 |
| 10 | 0.378 | **7.95** | 12.93 | 0.822 | **4.86** | 1.93 |
| 30 | 1.066 | 17.7  | 11.32 | 2.085 | 7.17 | 1.92 |

NEES has a clear minimum at **Q≈10** then climbs again (above the optimum the
estimate itself degrades). So the IMU densities are ~an order of magnitude
under-sized. **But NIS barely moves** (14.8→11.3 across a 30× change) — inflating
`P` via `Q` does not fix the innovation over-confidence. Two independent faults.

### B.2 Measurement-noise sweep — `R` is ~4× too small, and it is holding the filter up

| R_SCALE | V2_03 ATE | V2_03 NIS | V2_03 NEES | MH_05 ATE | MH_05 NIS | MH_05 NEES |
|---|---|---|---|---|---|---|
| 1  | 0.293 | 14.80 | 115 | 0.785 | 2.18 | 10 |
| 2  | 0.502 | 3.94  | 130 | 1.188 | 0.57 | 37 |
| 4  | 2.07  | **1.08** | 444 | 3.44 | 0.15 | 26 |
| 10 | **278 m** | 0.36 | 1542 | **2185 m** | 0.04 | 7019 |

NIS scales as a near-perfect **1/R_SCALE²** (14.8 → 3.9 → 1.08 = ÷4 ÷16). That
clean quadratic is the signature of an under-tuned *measurement noise*, not an
observability wall (which would not scale away). True normalized image noise is
~√14.8 ≈ **3.8×** the assumed 0.01 on V2_03, ~1.5× on MH_05 — i.e. motion-dependent.

**But making `R` honest diverges the filter:** ATE goes 0.29 m → 2 m → **278 m**.
The over-tight `R` is supplying the correction authority that keeps the trajectory
bounded. The default config has *good ATE and a lying covariance* because two
errors (tight `Q`, tight `R`) are **mutually compensating**.

### B.3 Joint sweep — no setting is both consistent and accurate

| Q | R | seq | ATE | NIS | NEES |
|---|---|---|---|---|---|
| 1  | 1 | V2_03 | **0.29** | 14.8 | 115 |
| 10 | 2 | V2_03 | 0.83 | 3.6 | **10.1** |
| 10 | 4 | V2_03 | 6.22 | **1.06** | 42 |
| 5  | 4 | V2_03 | 3.12 | 1.05 | 53 |
| 1  | 1 | MH_05 | **0.79** | 2.2 | 10 |
| 10 | 2 | MH_05 | 1.43 | 0.56 | 7.1 |
| 10 | 4 | MH_05 | 3.04 | 0.15 | **3.6** |

Read down the ATE column: **every step toward an honest covariance costs
accuracy.** The most accurate config is the most over-confident; the most
consistent configs (NIS≈1) carry 3–6 m ATE. The best balance (Q10R2) still leaves
V2_03 at NEES=10 / NIS=3.6 and triples the ATE.

## C. Why this is structural, not a re-tune

In a correctly-modelled filter the *true* noise values give consistency **and**
optimal accuracy simultaneously — that is the definition of optimality. Here,
accuracy *requires* over-confident noise. The only way that happens is if a
structural error is being masked by the over-confidence; relax the noise and the
underlying error surfaces as drift.

The fault is **motion-dependent** — the throughline of the whole investigation:
attitude NEES (7→169), NIS (2.2→14.8), and the residual inconsistency at honest
noise (MH_05 NEES 3.6 vs V2_03 42) all scale with aggressiveness. That fingerprints
an **EKF observability / linearization inconsistency** — the classic MSCKF
spurious-information-gain in the unobservable (yaw + position) directions, worst
under fast rotation. The standard remedies are **First-Estimates Jacobians (FEJ)**
or an **observability-constrained EKF (OC-EKF)**.

A motion-correlated *systematic* in the visual update (extrinsics, feature
triangulation bias, or — less likely on hardware-synchronized EuRoC — a
camera–IMU time offset) is a secondary candidate that would also produce
motion-scaling innovations.

## D. The discriminator — run, and resolved

One clean test separates the two remaining hypotheses before any estimator
surgery: **the mean and whiteness of the normalized innovations**
(`eval/innovation_whiteness.hpp`; per update it takes the signed
`S_t = Σ_k r_k/σ ~ N(0, dof)`).

- **zero-mean, inflated-variance, temporally non-white** → observability / Jacobian
  inconsistency → implement FEJ / OC-EKF.
- **a large non-zero mean** → a systematic (calibration / triangulation / time-sync)
  → fix the model, not the noise.

**Result** (baseline `Q1R1`):

| sequence | mean z | bias magnitude | lag-1 z | verdict |
|---|---|---|---|---|
| V2_03 (aggressive) | −4.64 (flagged) | **≈ −0.013σ** (~0.1 px) | +14.2 | non-white; bias negligible |
| MH_05 (mild) | +2.19 (not flagged) | — | −10.3 | non-white; zero-mean |

The innovations are **strongly non-white on both sequences** (|lag-1 z| ≫ 2.58):
the filter is leaving temporal structure in the residuals — the signature of an
observability / linearization inconsistency, exactly as the sweeps implied. The
mean is **effectively zero**: the only flagged bias (V2_03) is ~0.013σ — about a
tenth of a pixel — versus a NIS of 14.8 (innovation spread ~3.85σ), so the
systematic contributes ~0.3% of the over-confidence. No gross calibration error
(no swapped extrinsic, no large time offset) exists; the small bias is
motion-dependent (absent on MH_05) and a minor secondary at most.

**Conclusion: this is the observability/Jacobian case.** The fix is **FEJ /
OC-EKF** (#280). Success criterion: with FEJ in place the *honest* noise
(`Q≈10×`, `R≈4×`) finally yields **consistent NEES and NIS *and* bounded ATE**
together — the state no setting can reach today — and the innovation sequence
whitens.

> Caveat: `S_t` aggregates a whole track's multi-dimensional residual into one
> signed scalar processed in marginalization order, so read the lag-1 *sign* with
> care; the load-bearing finding is "significantly non-white," which holds on both
> sequences and both signs.

## E. Reproduction

```bash
cmake --preset sitl-release && cmake --build --preset sitl-release --target vio_euroc -j4
export CORTEX_EUROC_MH05=/path/to/MH_05_difficult/mav0
export CORTEX_EUROC_V203=/path/to/V2_03_difficult/mav0
# single point, or sweep CORTEX_Q_SCALE / CORTEX_R_SCALE (default 1 = baseline):
CORTEX_Q_SCALE=10 CORTEX_R_SCALE=2 \
  ./build/sitl-release/tests/vio_euroc "V2_03_difficult replay (moving start) converges"
```

The benchmark WARNs the aggregate NEES/NIS, the per-block NEES, and the active
scale factors. Note: the Release preset (`-O3`) replays a sequence in ~85 s; an
unoptimized build is ~25× slower — always sweep with the Release binary.
