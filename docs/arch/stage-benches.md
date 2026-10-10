# VIO stage test benches

Epic #444 §A, issue #453. A **stage bench** is a focused debug, test, characterization and research
environment for **one** pipeline transformation. It runs that stage on fixtures, in several arithmetic
types and implementation variants, and reports the stage's invariants (#445) in native units. The full
pipeline is the call sequence of the same transformations (#452, in
`sdk/include/branes/sdk/msckf/stages/`), so whatever a bench establishes about a stage holds where
the backend calls it.

A bench is one small executable. Editing a stage and re-running its bench takes seconds, not a EuRoC
replay.

## Layout

| Path | What |
|---|---|
| `tools/include/branes/tools/bench/bench.hpp` | the runner, the report, and the CLI (`bench_main<B>`) |
| `tools/include/branes/tools/bench/fixture.hpp` | the fixture model, the exact JSON codec, and the capture writer |
| `tools/include/branes/tools/bench/types.hpp` | the arithmetic types: `double`, `float`, `posit32` (posit<32,2>), `posit16` (posit<16,1>), and `long_double` as the wider-than-double reference |
| `tools/include/branes/tools/bench/variant.hpp` | the variant slot: named alternative implementations of a stage |
| `tools/include/branes/tools/bench/sweep.hpp` | declarative characterization sweeps and CSV tables |
| `tools/include/branes/tools/bench/s<N>_<transformation>_bench.hpp` | one stage's bench (the adapter) |
| `tools/benches/s<N>_<transformation>_bench.cpp` | the bench executable, a one-line `main` |
| `tools/benches/fixtures/s<N>_<transformation>/*.json` | the stage's committed fixtures |
| `tests/tools/stage_bench.cpp` | the Catch2 integration: the framework's guarantees, locked |

Benches so far:

| Bench | Issue | Variants | Sweep |
|---|---|---|---|
| `s0_sensor_model_bench` | #454 | `shipped` (Newton undistortion), `fixed_point` | distortion strength × radius → round trip, Jacobian, px per 1° extrinsic / per 1 ms offset |
| `s1_initialization_bench` | #454 | `shipped` (static), `gravity_align`, `dynamic` (VI alignment) | window × IMU noise × excitation → roll/pitch and scale error |
| `s2_propagation_bench` | #455 | `shipped` (first-order Φ, diagonal Q_d), `canonical_qd`, `sqrt_covariance` | Δt × dynamics × Q scale → position error, yaw leak, diagonal-vs-canonical position-σ gap, λ_min/‖P‖ |
| `s3_augmentation_bench` | #455 | `shipped`, `sqrt_covariance` | clones × conditioning → block residual, λ_min/‖P‖ |
| `s4_frontend_bench` | #456 | `shipped` (FB gate off), `fb_gate_1px`, `klt_window_7` | image noise × shift → survival, end-point RMS, FB median |
| `s5_triangulation_bench` | #456 | `shipped` (linear + Gauss-Newton), `linear_only`, `parallax_gate_2deg`, `midpoint_two_view`, `dlt_linear`, `inverse_depth_gn` | parallax × pixel noise → depth error per method, κ, gate rejection |
| `s6a_jacobians_bench` | #457 | `shipped` | observations × depth → H_f, H_x finite-difference error, max \|H_f\| |
| `s6b_nullspace_projection_bench` | #457 | `shipped` (Householder), `givens` | observations × depth → NᵀN − I, Nᵀ H_f, H₀·N per method |
| `s6c_compression_bench` | #457 | `shipped` (identity), `qr` | features stacked → rows in/out, normal-equation error |
| `s6d_gating_bench` | #457 | `shipped` (5 per dof), `chi2_95`, `gate_off` | dof × outlier size → inlier NIS/dof, acceptance per gate |
| `s6e_ekf_update_bench` | #457 | `shipped` (Joseph), `sqrt_array` | pixel noise × calibration σ → trace ratio, information along N, ‖δx‖, NIS/dof |
| `s9_marginalization_bench` | #453 (worked example), #458 | `shipped`, `direct_gather`, `policy_oldest`, `policy_second_newest` | clones × conditioning → residual, λ_min |
| `s10_online_calibration_bench` | #458 | `shipped` (extrinsic estimated), `fixed`, `folded_into_r` | extrinsic rotation error × prior → remaining error, NIS/dof per treatment |

Every stage S0–S6, S9 and S10 now has a bench. The S6 sub-steps share a scene (`s6_scene.hpp`): the synthetic world
run through S2 and S3, so the covariance is the one those stages produce, with each sub-step's input
built by running the shipped upstream sub-steps in double. A sub-step's bench (`S6a_…` to `S6e_…`)
prints the S6 contract.

A bench whose stage is slow in software arithmetic can set `static constexpr int kTimingReps` (S2
and S3 use 1). A bench whose stage is not generic in T declares `using SupportedTypes =
TypeList<...>` (S4: the KLT front end runs in double only, #449 decides whether it becomes
generic). Image fixtures are stored losslessly as base64 bytes (`pack(cv::Image)`,
`unpack_image`). Square-root variants start from a real filter state with `psd_factor`, a Cholesky that
tolerates the exactly singular covariance a fresh clone produces.

A posit is most precise near 1 and loses fraction bits as |x| moves away; at 1e-17 posit32 keeps
about 14 of its 28. A check on small quantities (a residual at the true state, a covariance entry
near σ²) uses `safety_at<T>(magnitude)`, which scales the safety factor by `eps_at<T>(magnitude) /
ε_T`: T's relative spacing at that size. It is 1 for IEEE types. Don't apply it to a lower bound
(an SPD margin): a larger safety factor makes those stricter.

## Fixtures

A fixture is a stage's input plus whatever its output is judged against. There are three kinds:

| Kind | Input | Judged against | How it is made |
|---|---|---|---|
| `known_answer` | synthetic | `expected`: an analytic output, computed independently of the stage | by hand, in the bench |
| `ground_truth` | built with truth injected upstream (#266) | `truth`: the true values the output must reproduce | a synthetic world run with the mean reset to truth |
| `captured` | a recorded stage boundary | `expected`: the output the stage produced in that run | `capture(...)` at the boundary; from real runs via #446 |

The runner adds two checks to the stage's own invariants:

- **`replay.bit_identical`** (captured fixtures): run in the arithmetic the fixture was captured in,
  the stage must reproduce the recorded output **bit for bit**. Any interesting moment of a real run
  becomes a regression test this way.
- **`known_answer.residual`**: the output must match `expected` within the type's arithmetic
  tolerance (`safety · n · ε_T · scale`, #445).

**Variant binding.** An `expected` output can belong to one implementation. The fixture's `variant`
field names it, and the runner applies the known-answer and replay checks only to that variant. An
empty `variant` means every variant must reproduce it. Captured fixtures record the variant that ran.
S1 needs this: its static and dynamic variants legitimately estimate different things from the same
input.

**Comparisons against double references.** Known answers, ground truth and stored rotations all pass
through `double`. So a comparison is never held tighter than double's ε, even when the stage ran in a
wider type (`tolerance_vs_double<T>`).

### File format

One JSON document per fixture, schema `branes.vio.fixture/1`:

```json
{"schema": "branes.vio.fixture/1", "stage": "S9_marginalization", "kind": "captured",
 "arithmetic": "double", "source": "…", "seed": 0, "description": "…",
 "input": {…}, "expected": {…}, "truth": {…}}
```

The payloads are built from a small vocabulary: numbers, vectors, row-major matrices
`{"rows","cols","data"}`, rotations as unit quaternions `[w,x,y,z]`, and the full MSCKF state
(navigation, calibration blocks, clone window, covariance P).

**Exactness.** The codec is bit-exact for every type the benches use:

- every number goes through `double`, which holds every float, posit<32,2> and double value
  exactly;
- nlohmann/json writes doubles in shortest round-trip form;
- non-finite values are written as the strings `"nan"`, `"inf"` and `"-inf"`;
- rotations are restored with `SO3::from_unit_quaternion`, without renormalizing, because
  renormalizing is not bit-idempotent.

`tests/tools/stage_bench.cpp` locks this for edge values (−0, denormals, max, ±inf, NaN) and for a
full captured state.

## Writing a bench

A bench is a struct describing one stage. The full contract is in the header comment of
`bench.hpp`; in short:

```cpp
struct S9MarginalizationBench {
    static constexpr std::string_view kStage = "S9_marginalization";
    static constexpr inv::Stage kInvStage = inv::Stage::S9_marginalization;
    static std::vector<Variant> variants();                        // "shipped" first
    template <class T> struct Input;  template <class T> struct Output;
    template <class T> static Input<T>  decode_input(const json&);
    template <class T> static Output<T> decode_output(const json&);
    template <class T> static json      encode_output(const Output<T>&);
    template <class T> static Output<T> run(const Input<T>&, std::string_view variant);
    template <class T> static std::vector<double> flatten(const Output<T>&);
    template <class T> static std::vector<inv::InvariantResult>
        invariants(const Input<T>&, const Output<T>&, const Fixture&);
    static std::vector<NamedFixture> builtin_fixtures();
    // optional characterization sweep
    static Sweep default_sweep();
    static std::vector<std::string> sweep_columns();
    template <class T> static std::vector<double> sweep_point(const Point&);
};
```

- **`run`** calls the stage's transformation, `stages::<sN_name>::apply`. A research variant is
  another branch of `run`, selected by name.
- **`invariants`** returns the stage's §B checks from `branes/sdk/eval/invariants.hpp`. The S9 bench
  checks:
  - the principal submatrix;
  - symmetry and positive-semidefiniteness;
  - the dimension and window bookkeeping;
  - that the mean is untouched;
  - for a ground-truth fixture, the kept clones against the truth.
- **`flatten`** lists every output number, for the comparison against `double` and the replay check.

## Running a bench

```bash
cmake --build build --target s9_marginalization_bench
./build/tools/s9_marginalization_bench                           # built-in fixtures, double/float/posit32
./build/tools/s9_marginalization_bench --types double,posit16    # choose arithmetic types
./build/tools/s9_marginalization_bench --variant all             # shipped vs research variants
./build/tools/s9_marginalization_bench --fixture F.json          # a fixture file (e.g. captured, #446)
./build/tools/s9_marginalization_bench --sweep clones=2,4,8 --csv DIR   # sweep → CSV per type
./build/tools/s9_marginalization_bench --capture DIR             # write the built-in fixtures as files
```

**Report.** The bench prints the stage contract from the registry (`vio_stage_contracts.hpp`). Then,
per fixture, variant and type, it prints:

- every invariant, with its value, threshold, unit and verdict;
- the largest deviation from the `double` run;
- the stage's run time.

**CSV output.** `--csv DIR` writes two kinds of file:

- `<stage>_report.csv`: one row per fixture × variant × type × invariant;
- `<stage>_sweep_<type>.csv`: one per type, columns = the sweep axes followed by the bench's metrics,
  ready for the docs-site figure scripts.

**Exit code.** The bench exits non-zero if any invariant fails, so it can gate CI.

## Edit-compile-run loop (measured)

Measured with `touch sdk/include/branes/sdk/msckf/stages/s9_marginalization.hpp`, then rebuilding
and running only the bench target. Machine: i7-12700K, gcc 13.3, `-j4`, 2026-10-08.

| Build | Incremental rebuild | Run: double + float | Run: + posit32 (default) | Run: all variants, + posit16 |
|---|---|---|---|---|
| default (no `-O`) | 4.3 s | 0.08 s | 9.1 s | — |
| Release (`-O3`) | 8.4 s | < 0.01 s | 1.1 s | 2.7 s |

The posit types are software arithmetic. Most of their time goes to the invariant checks, whose
Jacobi eigensolves run in the type under test, so use a Release build when you work in posits. In
either build, one stage edit costs seconds.

## Existing per-stage assets

The benches build on what is already there rather than replacing it at once:

- `sdk/include/branes/sdk/eval/*_probe.hpp`: the stage probes (sensor model, initialization,
  propagation, clone window, frontend, triangulation, update, calibration, observability). Their
  scenarios become a bench's fixtures and sweeps in #454–#458.
- `tools/src/s<N>_*.cpp` and `s<N>_inspect.cpp`: the contract printers, the CSV probes and the trace
  inspectors. A bench prints the same contract from the same registry, and its sweep CSVs feed the
  same figure scripts.
- `tests/sdk/` stage tests, e.g. `msckf_stages.cpp` for the stage transformations: these stay as the
  fast unit tests. The bench's Catch2 integration covers its fixtures.

## What the S0 and S1 benches found (#454)

- **The synthetic world's camera isn't quite EuRoC cam0.** Its tangential coefficient p2 is
  1.76187114e-2, where the EuRoC calibration has 1.76187114e-5. The world is self-consistent, but the
  ground-truth fixture, built with the real EuRoC intrinsics, disagreed by 0.1 in normalized
  coordinates until it used the world's camera.
- **The S1 seeding stage didn't compile for posits.** It narrowed `T` into a `double` diagnostic
  implicitly. That is now an explicit `static_cast`, which is unchanged for float and double.
- **The dynamic VI alignment is precision-fragile.** `ImuInitializer::try_dynamic` declines in
  `float` and `posit<32,2>` on fixtures where it resolves in `double` and `long double`, and in
  `posit<16,1>` it resolves with a 23.5% scale error. Its alignment solve forms the normal equations
  (squaring the condition number), adds a fixed `T(1e-9)` ridge, and then needs a Cholesky
  factorization, which breaks down once κ² exceeds 1/ε. This is #449/#450 territory; the default
  (static) path is green in every type.

## What the S2 and S3 benches found (#455)

- **The propagator didn't compile for posits.** `s.timestamp += dt` added a `T` into the `double`
  timestamp; #444 §E lists this exact site. It now uses an explicit `static_cast<double>(dt)`, which
  is identical for float and double.
- **The yaw direction leaks under propagation, as expected.** Φ preserves the global-translation
  directions of the unobservable subspace exactly, but the body-frame filter's first-order Φ,
  linearized at the current estimate, leaks the yaw direction at O(Δt²) per step. That is ≈2.4e-6
  per 5 ms step on the tumbling fixture, and the sweep shows it across Δt and dynamics. The bench
  reports it rather than gating it; it is the propagation half of the observability question
  (#212, #437).
- **Φ needed exposing.** `Propagator::transition` now returns Φ and Q_d. `propagate` is its sequence,
  and it is bit-identical on the synthetic full and square-root backend runs.

## What the S4 and S5 benches found (#456)

- **The front end needed extracting.** Its KLT tracking, forward-backward gate and FAST replenishment
  lived as private methods of `VioEstimator`. They are now `stages::s4_frontend::track`, which
  `VioEstimator` calls, and the result is bit-identical on EuRoC. The S4 inspector (`s4_inspect`)
  calls the stage too, rather than its own instrumented copy. Its JSONL output is byte-identical to
  the copy's on 600 MH_05 frames, with the gate off and at 1 px.
- **Border churn.** KLT drops any feature whose window doesn't fit at the coarsest pyramid level (a
  band about (half-window + 1)·2^(levels−1) px wide, ≈24 px at the defaults), while FAST detects up
  to 3 px from the edge. Detections in that band die on their first tracked frame and are replaced
  under new ids: about 85% of new detections on the bench's 160×120 frames. The bench reports this
  as `tracks.lost_on_first_frame`. Tracked in #474.
- **The forward-backward gate is off by default**, so the shipped front end keeps tracks that can't
  round-trip. The bench measures that (reported) and enforces it in the `fb_gate_1px` variant.
- **`CameraUpdater` didn't compile for posits.** It called `std::acos` and `std::sqrt` as qualified
  calls (#444 §E lists these lines). They now use ADL, which resolves to the same functions for float
  and double.
- **The synthetic world steps its velocity at the end of the warm-up.** The ground truth is at rest
  at t = 1.5 s and moving at ≈1 m/s right after it, which no IMU stream can produce. Pure IMU
  propagation from rest is off by ≈1 m/s after one frame. Elsewhere the world's IMU matches its truth
  to ≈1e-5 m per frame. The S5 captured fixture starts after the step. Tracked in #475.

## What the S6 benches found (#457)

- **The update adds no information along yaw and global position at a consistent linearization
  point.** The projected Jacobian annihilates the four unobservable directions to ≈1e-16 at the
  S2/S3-estimated state as well as at the true one, and the update's information-form correction
  Hᵀ S⁻¹ r has no component along them. That rules out this mechanism inside S6 on these fixtures; it
  does not rule out other sources of the #212 over-confidence. Linearization points drifting apart
  across stages are an identified one: the S2 bench measures a yaw leak in Φ (#455). Per-stage
  attribution on a real run is #480.
- **The shipped χ² gate is loose at low dof (#481).** It accepts γ ≤ 5·dof. Inliers are
  χ²-consistent (NIS/dof inside the band at every dof), but at 1–3 dof the gate admits 59–70% of
  measurements biased by 2σ, where a 95% χ² gate admits 22–44%. At 9 dof and above both reject them.
- **A square-root filter must carry its factor (#482).** The clone blocks are almost perfectly
  correlated with the IMU pose, so their Schur-complement pivots are below float's (and posit32's)
  resolution of P: factoring P in those types zeroes 36 of 51 pivots, where double zeroes the 6 exact
  ones cloning makes, and represents a different covariance. Measured that way, the square-root update
  looked ≈330× less precise than Joseph in posit32. The `sqrt_array` variant therefore factors the
  fixture's covariance in double and rounds the factor to T, as a filter that carried its factor
  would hold it; then it lands within 1.3–2.3× of Joseph in every type. The S2 and S3 benches'
  square-root variants still re-factor P in T (#482).
- **Givens and Householder null spaces agree** (the same HᵀH to 1e-13), and **QR compression** of a
  72-row stack keeps n + 1 = 52 rows with the normal equations preserved to 1e-16: batched updates are
  numerically safe.

## What the S9 and S10 benches found (#458)

- **A wrong extrinsic is nearly invisible to the update.** Over one 6-clone window (16 tracks), a
  constant camera↔IMU rotation error raises NIS only to ≈0.002 per dof at 5° and ≈8e-5 at 1°; the
  null-space projection absorbs almost all of it into the feature positions. A filter that trusts a
  wrong extrinsic cannot see it in NIS or the χ² gate, and folding the prior into R lowers NIS
  further. Estimated as state, the extrinsic moves toward the truth (1° → 0.955° with a 2° prior;
  5° → 3.98° with a 5° prior) while its covariance stays consistent (NEES 0.49 of a χ²₀.₉₉₉(6) bound of
  22.5), so recovery needs many windows of excitation. This is the "calibration treated as known"
  suspicion of #458, measured.
- **The window policy does not change the marginalization contract.** Removing the oldest or the
  second-newest clone keeps the principal-submatrix contract exactly; what a policy changes is which
  information the update had a chance to use first, which is a pipeline question (#447).
- Fixture expected outputs can now bind a list of variants (`"shipped,direct_gather"`), and the S9
  ground truth lists every clone's pose keyed by time, so policy variants are checked against it.
- The filter has no time-offset state, so the issue's t_d sweep has nothing to vary.

## Alternative triangulation methods (S5)

`stages::s5_triangulation::apply(state, updater, track, method)` runs a candidate instead of the
shipped solve; the estimator does not use them. They share the updater's camera model
(`CameraUpdater::camera_pose`, `projection_jacobians`) and ignore the parallax gate.

| Method | Variant | What it solves |
|---|---|---|
| `Shipped` | `shipped` | ray-perpendicular linear solve Σ(I − d̂d̂ᵀ)·p = Σ(I − d̂d̂ᵀ)·c, then Gauss-Newton on reprojection |
| `Midpoint` | `midpoint_two_view` | the best-conditioned pair of rays (largest sin of their angle); the midpoint of their common perpendicular |
| `Dlt` | `dlt_linear` | inhomogeneous DLT: algebraic least squares over every view (W = 1) |
| `InverseDepth` | `inverse_depth_gn` | anchored inverse depth (α, β, ρ) in the first camera, seeded by the DLT, Gauss-Newton on reprojection |

All four pass every fixture in every default type. The sweep (four clones, feature at 5 m, 16
draws; mean depth error in m, double) separates them:

| parallax | px noise | shipped | midpoint | DLT | inverse depth |
|---|---|---|---|---|---|
| 0.25° | 0.5 | 2.38 | fails | 1.60 | 2.41 |
| 1° | 2 | 2.40 | fails | 1.60 | 2.41 |
| 5° | 1 | 0.140 | 0.168 | 0.147 | 0.140 |
| 15° | 1 | 0.0455 | 0.0553 | 0.0465 | 0.0455 |

- **Inverse depth matches the shipped Gauss-Newton** to 1e-9 m wherever depth is observable. It
  minimizes the same reprojection error, so the parameterization changes the path, not the answer.
  Where it differs at grazing parallax, it is by the few percent of a different local minimum.
- **The DLT is biased toward the cameras.** That shrinkage beats the maximum-likelihood answer
  where depth is barely observable, and costs ≈5% where it is observable.
- **The two-view midpoint is the worst** and loses tracks at low parallax with noise. It is also
  the one method that loses precision with exact observations: the closed form divides by 1 − cos²
  of the parallax angle, which cancels catastrophically. In float it is off by 9 mm at 0.25°, where
  the others are within 1 µm.
- Every method loses the track at 0.25° with 1 px of noise; no parameterization recovers a depth
  the geometry doesn't hold. The parallax gate is the answer there, not the solver.

Capturing fixtures from the running backend and from EuRoC replays, selected by stage, frame range or
trigger, is #446. It writes fixtures with `bench::capture(...)` in the format above, so they load
into these benches unchanged.
