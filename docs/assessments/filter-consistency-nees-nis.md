# Filter consistency: NEES and NIS, explained

A tutorial on the two standard tests for whether an estimator (an EKF/MSCKF like
our VIO filter) *believes the right amount* — and how we use them in cortex
(`eval/consistency.hpp`, `eval/nav_consistency.hpp`). Written for someone who has
not met NEES/NIS before. Companion to
[`vio-diagnostic-methodology.md`](./vio-diagnostic-methodology.md).

---

## 1. The one idea

A Kalman-type filter does not just produce an estimate. It produces an estimate
**and a statement of its own uncertainty** — the covariance matrix `P`. When the
VIO says "the camera is at position p," it is really saying "I'm 68% sure the
position is within this ellipsoid, 95% within this bigger one." `P` *is* that
ellipsoid.

A filter is **consistent** when that self-reported uncertainty matches reality:
when it says ±10 cm, it is actually wrong by about 10 cm — not 1 m (it was
**over-confident**, lying that it's sure), and not 1 mm (it was
**under-confident**, throwing away good information).

> **The weather-forecaster analogy.** A forecaster who says "70% chance of rain"
> is *well-calibrated* if, across all the days they said 70%, it actually rained
> about 70% of the time. If it rained 95% of those days, they were
> under-confident; if 30%, over-confident. **NEES and NIS are the calibration
> check for an estimator.** They don't ask "is the estimate close?" — they ask
> "is the estimate's *claimed uncertainty* honest?"

This matters because an over-confident filter is *dangerous*: it stops trusting
new measurements (it already "knows" the answer), so it locks onto its error and
diverges — silently, with a covariance that looks tight and healthy. ATE/RPE
can't see this. NEES/NIS can.

---

## 2. The math, gently

### 2.1 "How many sigmas off?" in one dimension

Suppose the filter estimates a scalar as `x̂` with standard deviation `σ`, and the
true value is `x`. The error is `e = x̂ − x`. The natural way to ask "is this
error consistent with the claimed `σ`?" is to measure it **in units of σ**:

```text
       e                 e²
  z = ───  (z-score),   ─── = z²  (its square)
       σ                 σ²
```

If the filter is honest, `e` is drawn from `N(0, σ²)`, so `z` is a standard
normal and **`z² = e²/σ²` has expected value 1**. Average `e²/σ²` over many
estimates: ≈ 1 means honest, ≫ 1 means the real errors are bigger than `σ`
claims (over-confident), ≪ 1 means smaller (under-confident).

### 2.2 The same thing in many dimensions: the Mahalanobis distance

Our state is a vector (position, velocity, attitude, biases…), and its
components are *correlated* — `P` is a full matrix, not just per-axis variances.
The multi-dimensional generalization of `e²/σ²` is the **squared Mahalanobis
distance**:

```text
  d² = eᵀ P⁻¹ e
```

`P⁻¹` "divides by the covariance" the way `1/σ²` did in 1D — it stretches the
error by the inverse uncertainty in every (possibly correlated) direction. This
is the single quadratic form at the heart of both tests; in code it is
`normalized_squared(e, P)` in `eval/consistency.hpp` (computed with a Cholesky
solve, never an explicit inverse).

### 2.3 Why the magic number is the dimension (the χ² fact)

Here is the one piece of probability that makes it all work. If `e ~ N(0, P)`
(the filter is consistent), then

```text
  d² = eᵀ P⁻¹ e   ~   χ²(n)          (chi-square with n degrees of freedom)
```

where `n` is the dimension of `e`. You can see why: `P⁻¹` can be factored so that
`d²` becomes a sum of `n` *independent* standard-normal squares, `z₁² + … + zₙ²`,
and that sum is the definition of a χ²(n) variable. The key property of χ²(n) is:

```text
  E[χ²(n)] = n        (mean equals the number of degrees of freedom)
  Var[χ²(n)] = 2n
```

So **the expected normalized-squared error equals the dimension of the thing you
are testing.** Divide `d²` by `n` and a consistent filter sits at **1.0**.
Everything below is just "measure `d²/n` and check it's near 1."

---

## 3. NEES vs NIS — two places to apply the same idea

The quadratic form `eᵀ P⁻¹ e` is identical; the two tests differ in *what error*
and *what covariance* you plug in.

### NEES — Normalized Estimation Error Squared (needs ground truth)

- **error** = `e = x̂ − x_true`, the estimate minus the true **state**.
- **covariance** = `P`, the filter's **state** covariance.
- **dof** = the state dimension (in our VIO core, 15: attitude 3, position 3,
  velocity 3, gyro bias 3, accel bias 3).
- Tests the *whole state estimate's* calibration. Per-state-block NEES (slice out
  attitude, or velocity, …) localizes **which** state is over/under-confident.
- Needs ground truth, so it is an **offline / evaluation** tool (we run it on
  EuRoC, which ships a ground-truth trajectory).

### NIS — Normalized Innovation Squared (no ground truth)

- **error** = `ν = z − ẑ`, the **innovation**: the actual measurement minus what
  the filter predicted it would be. (When the camera sees a feature, `ν` is the
  pixel residual between where the feature appeared and where the filter expected
  it.)
- **covariance** = `S = H P Hᵀ + R`, the **innovation covariance**: how uncertain
  the filter was about that prediction (state uncertainty `H P Hᵀ` plus
  measurement noise `R`).
- **dof** = the measurement dimension.
- The filter checks itself against the *incoming measurements* — **no ground
  truth required**, so NIS runs **online, always**. If the innovations are
  consistently bigger than `S` predicts, the filter is over-confident *right now*.

**Why keep both:** NIS you can run live on the robot (it's how the filter
self-monitors and how the χ² outlier gate works); NEES needs truth but tests the
full state including the directions measurements don't directly observe. NIS
**consistent** while NEES is **inconsistent** is itself a clue (the measurement
model is fine but a hidden state — a bias, a scale — is drifting un-modeled).

---

## 4. From one noisy number to a verdict: the χ² consistency test

A *single* `d²` is very noisy — χ²(n) is a broad distribution (its standard
deviation `√(2n)` is comparable to its mean `n`). One sample tells you almost
nothing. The test averages over a whole run.

By the additivity of χ², the **sum** of `N` independent samples (each χ² with `dofᵢ`
degrees of freedom) is itself χ² with `D = Σ dofᵢ` degrees of freedom. For a long
run `D` is large, and a large-`D` χ² is approximately Gaussian (central limit
theorem): `χ²(D) ≈ Normal(D, 2D)`. So the dof-normalized average `S/D` (which
should be 1) has a two-sided `(1−α)` **acceptance band**

```text
  1 ± z·√(2/D)            z = Φ⁻¹(1 − α/2)   (e.g. z = 1.96 for 95%)
```

This is the classic **Bar-Shalom NEES/NIS consistency test**. In code it is
`ConsistencyAccumulator`: you `add(d², dof)` each step, and `report()` returns
the normalized average and whether it is inside the band (`consistent`), above it
(`overconfident`), or below (`underconfident`). The band tightens as the run
grows — more evidence, less slack.

> **Separately: the per-update χ² gate.** The *same* statistic, used differently.
> Before applying a measurement, the filter checks `NIS ≤ χ²-threshold` (e.g.
> 5.99 for a 2-DoF pixel residual at 95%) and **rejects** the measurement if it
> is a wild outlier (a mistracked feature). That is a single-sample *gate*, not a
> run-average *consistency test* — but it computes the very same `νᵀ S⁻¹ ν`. Our
> MSCKF already does this gating; #264 simply also *logs* the value so we can run
> the consistency test on it.

---

## 5. How we apply it in cortex

| Piece | File | Role |
|---|---|---|
| `normalized_squared` / `nis` / `nees` | `eval/consistency.hpp` | the `eᵀP⁻¹e` quadratic form (Cholesky solve; throws on a non-PD covariance) |
| `ConsistencyAccumulator` / `ConsistencyReport` | `eval/consistency.hpp` | accumulate over a run; the χ² band verdict |
| per-update NIS | `msckf/camera_updater.hpp` → `MsckfBackend::nis_consistency()` | the gate's `γ = νᵀS⁻¹ν` is recorded every camera update |
| `nav_error` / `gauge_align` / `align_truth` / `core_covariance` | `eval/nav_consistency.hpp` | build the 15-vector state error vs ground truth, in the filter's convention and frame |
| EuRoC report | `tests/sdk/vio_euroc.cpp` | WARNs the run's NIS and NEES verdicts |

A EuRoC run now prints two lines, e.g.:

```text
  MH_05_difficult: NIS  over 1843 updates: normalized=1.07 (band [0.95, 1.05]) — OVER-confident
  MH_05_difficult: NEES over 2210 frames:  normalized=2.4  (band [0.97, 1.03]) — OVER-confident
```

### The two VIO-specific subtleties the code handles

1. **The error must be in the filter's tangent space and ordering.** The state
   error is `[δθ δp δv δbg δba]` with the attitude error being a rotation
   *on the manifold* — `δθ = Log(R̂⁻¹ R_true)` — not a naïve quaternion subtraction.
   `nav_error` builds exactly this so it matches the covariance `P`'s layout.

2. **The gauge.** A monocular-flavored VIO solution is only fixed up to **4
   unobservable degrees of freedom: global position + yaw about gravity.** (Roll
   and pitch *are* observable — the accelerometer feels gravity.) The estimate
   and the ground truth therefore live in different world frames, and you must
   remove *only* that 4-DoF gauge before differencing. `gauge_align` builds a
   **yaw-only** rotation + translation (the "posyaw" alignment of Zhang &
   Scaramuzza). Using a full 6-DoF SE(3) alignment would silently rotate away the
   *observable* roll/pitch error and report a flatteringly small NEES — a real
   trap (and a bug we caught in review).

---

## 6. Reading the verdict — what each outcome means

| Reading | Meaning | First suspects |
|---|---|---|
| `normalized ≈ 1`, in band | **consistent** — the covariance honestly bounds the error | (healthy) |
| `normalized > 1` (over-confident) | real errors exceed the claimed uncertainty | process noise `Q` too small; a wrong/sign-flipped Jacobian; an unmodeled bias making residuals systematically biased |
| `normalized < 1` (under-confident) | claimed uncertainty exceeds real errors | measurement noise `R` too large; over-inflated `Q`; the filter ignoring usable information |
| **NIS consistent but NEES ≫ 1** | innovations fine, *state* drifting un-modeled | a systematic scale/bias error the covariance doesn't represent (a prime #212 signature) |
| per-block NEES spikes in one state | that state is the culprit | e.g. attitude-block NEES climbing first ⇒ gyro-bias / propagation Jacobian |

The actionable point: **a divergence or a drift now has a *direction*.** Instead
of "it's wrong, edit something and re-measure ATE," you get "the filter is
over-confident, the attitude block first, from the second propagation" — which
points at a specific component and term (see the diagnostic-methodology doc).

---

## 7. Caveats worth knowing

- **It's a large-sample test.** The Gaussian band assumes many samples (a real
  run of hundreds–thousands of updates). On a handful of samples the band is
  meaningless. At the nominal `α`, even a perfectly consistent filter is flagged
  `1−α` of the time by chance — so treat a marginal verdict as a prompt to look,
  not a proof.
- **NEES needs ground truth; NIS does not.** That is why NIS is the always-on
  on-robot health signal and NEES is the offline (EuRoC) deep check.
- **The covariance and the error must be the same object** — same tangent space,
  same state ordering, same units. A NEES computed against a mismatched `P` is
  nonsense. (This is why `nav_error` and `core_covariance` are deliberately tied
  to `msckf::State`'s documented layout.)
- **A non-positive-definite covariance is itself a failure**, not something to
  paper over — `normalized_squared` throws on it, and the EuRoC report counts and
  surfaces any frames it had to skip.

---

## 8. References

- Y. Bar-Shalom, X.-R. Li, T. Kirubarajan, *Estimation with Applications to
  Tracking and Navigation* (2001) — the NEES/NIS consistency tests and the χ²
  acceptance bounds (the "average NEES" test).
- J. Hesch, D. Kottas, S. Bowman, S. Roumeliotis, *Consistency Analysis and
  Improvement of Vision-aided Inertial Navigation* (T-RO 2014) — VIO
  observability and filter consistency (the 4-DoF unobservable subspace).
- Z. Zhang, D. Scaramuzza, *A Tutorial on Quantitative Trajectory Evaluation for
  Visual(-Inertial) Odometry* (IROS 2018) — the yaw-only ("posyaw") gauge
  alignment for consistent VIO evaluation.
