# VIO stage test benches

Epic #444 §A, issue #453. A **stage bench** is a focused debug, test, characterization and research
environment for **one** pipeline transformation. It runs that stage on fixtures, in several arithmetic
types and implementation variants, and reports the stage's invariants (#445) in native units. The full
pipeline is the call sequence of the same transformations (`sdk/include/branes/sdk/msckf/stages/`,
#452), so whatever a bench establishes about a stage holds where the backend calls it.

A bench is one small executable. Editing a stage and re-running its bench takes seconds, not a EuRoC
replay.

## Layout

| Path | What |
|---|---|
| `tools/include/branes/tools/bench/bench.hpp` | the runner, the report, and the CLI (`bench_main<B>`) |
| `tools/include/branes/tools/bench/fixture.hpp` | the fixture model, the exact JSON codec, and the capture writer |
| `tools/include/branes/tools/bench/types.hpp` | the arithmetic types: `double`, `float`, `posit32` (posit<32,2>), `posit16` (posit<16,1>) |
| `tools/include/branes/tools/bench/variant.hpp` | the variant slot: named alternative implementations of a stage |
| `tools/include/branes/tools/bench/sweep.hpp` | declarative characterization sweeps and CSV tables |
| `tools/include/branes/tools/bench/s<N>_<transformation>_bench.hpp` | one stage's bench (the adapter) |
| `tools/benches/s<N>_<transformation>_bench.cpp` | the bench executable, a one-line `main` |
| `tools/benches/fixtures/s<N>_<transformation>/*.json` | the stage's committed fixtures |
| `tests/tools/stage_bench.cpp` | the Catch2 integration: the framework's guarantees, locked |

The worked example is **S9_marginalization**: `s9_marginalization_bench.hpp` and
`tools/benches/s9_marginalization_bench.cpp`. The remaining stages get their benches in #454–#458.

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

Capturing fixtures from the running backend and from EuRoC replays, selected by stage, frame range or
trigger, is #446. It writes fixtures with `bench::capture(...)` in the format above, so they load
into these benches unchanged.
