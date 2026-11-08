#!/usr/bin/env bash
#
# euroc-consistency.sh — the EuRoC consistency gate (issue #605,
# docs/benchmarks/euroc-baseline.mdx "The consistency gate").
#
# Can the estimator be told the true measurement noise and get better rather
# than diverge? Since #613 every input comes from one truth: a cubic B-spline
# through the ground truth (vio_pipeline --imu gt). The IMU is synthesized from
# it at the real IMU timestamps with the filter's own noise densities, the KLT
# front end is replaced by ideal tracks (#602) seen from it with a known noise
# S (drawn in normalized coordinates, sigma S / fx, so it is exactly what a
# matched R says), and each run is scored against it (traj_eval --truth). R and
# Q are then the true noise. Before #613 the IMU was the real one and the truth
# the raw ground truth, whose error against the IMU (0.25-0.45 px over 0.25 s in
# the Vicon rooms) made matched R untrue and let the chi-square gate starve
# both filters (docs/methodology/specs/euroc-consistency-gate.yaml). Every
# backend runs with the same, shipped S6d gate. Each (sequence, S, seed) runs
# twice: with the filter's R matched to S, and with its default R. The verdict
# (bounds in scripts/euroc-consistency.json):
#
#   1. no divergence: every matched run's window 4-DoF ATE is under
#      diverge_ratio x OpenVINS mono's (a run that writes no trajectory counts
#      as diverged);
#   2. matched R beats the default R: at each S, the median window ratio of the
#      matched runs is below the default runs';
#   3. no yaw collapse: nothing observes yaw, so the filter's world-yaw sigma
#      should not fall below its value at the gauge anchor. The median over the
#      matched runs of (smallest yaw sigma / yaw sigma at the anchor) must be at
#      least yaw_sigma_ratio_min.
#
# A run whose trajectory exists but that traj_eval could not score is an
# evaluation failure, not a divergence: the gate then reports UNAVAILABLE and
# fails, and --record refuses to store it.
#
# The yaw ANEES is printed but not gated. Its error is measured from the anchor
# and its denominator is the current yaw variance alone; the correct variance
# of the change since the anchor needs the covariance between the two times,
# which neither filter exposes. So the ANEES is not a sound consistency test.
#
# Roll/pitch and position NEES and the normalized NIS are recorded but not
# gated. With the synthesized IMU they measure the estimator alone; NIS is the
# check that the noise model is true (~1 when it is and the filter is
# consistent).
#
#   scripts/euroc-consistency.sh                       # MSCKF, all 11 sequences
#   scripts/euroc-consistency.sh --backend riekf       # the R-IEKF (--invariant)
#   scripts/euroc-consistency.sh V1_01_easy MH_05_difficult
#   scripts/euroc-consistency.sh --record              # record this backend's results
#   scripts/euroc-consistency.sh --no-build ...        # reuse the built tools
#
# Environment overrides:
#   CORTEX_EUROC_ROOT         dataset root (default /srv/samba/sw-21/EuRoC-MAV-dataset)
#   CORTEX_PRESET             CMake preset to build and run (default sitl-release)
#   CORTEX_BIN                directory holding vio_pipeline and traj_eval (default
#                             build/<preset>/tools; implies --no-build)
#   CORTEX_CONSISTENCY_JOBS   runs at once (default 3; ideal-track runs read no images)
#   CORTEX_CONSISTENCY_OUT    output directory (default build/euroc-consistency)
#   CORTEX_CONSISTENCY_SIGMAS track noise levels, px (default "1 0.5")
#   CORTEX_CONSISTENCY_SEEDS  track seeds (default "1 2 3")
#
# A full run is 11 x 2 x 3 x 2 = 132 runs of ~11 s, ~8 minutes at 3 jobs.
#
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASELINE="$REPO/scripts/euroc-baseline.json"
GATE="$REPO/scripts/euroc-consistency.json"
DATASET_ROOT="${CORTEX_EUROC_ROOT:-/srv/samba/sw-21/EuRoC-MAV-dataset}"
PRESET="${CORTEX_PRESET:-sitl-release}"
JOBS="${CORTEX_CONSISTENCY_JOBS:-3}"
OUT_ROOT="${CORTEX_CONSISTENCY_OUT:-$REPO/build/euroc-consistency}"
SIGMAS="${CORTEX_CONSISTENCY_SIGMAS:-1 0.5}"
SEEDS="${CORTEX_CONSISTENCY_SEEDS:-1 2 3}"

build=1
record=0
backend=msckf
seqs=()
while [ $# -gt 0 ]; do
    case "$1" in
        --no-build) build=0 ;;
        --record) record=1 ;;
        --backend)
            [ $# -ge 2 ] || { echo "euroc-consistency: --backend needs msckf or riekf" >&2; exit 2; }
            backend="$2"
            shift
            ;;
        -h | --help)
            sed -n '2,62p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        -*)
            echo "euroc-consistency: unknown flag $1" >&2
            exit 2
            ;;
        *) seqs+=("$1") ;;
    esac
    shift
done
case "$backend" in
    msckf) BACKEND_ARGS="--observability-ledger" ;;
    riekf) BACKEND_ARGS="--invariant" ;;
    *)
        echo "euroc-consistency: --backend is msckf or riekf, not $backend" >&2
        exit 2
        ;;
esac
named=${#seqs[@]}
if [ "$named" -eq 0 ]; then
    mapfile -t seqs < <(python3 -c 'import json,sys; print("\n".join(json.load(open(sys.argv[1]))["sequences"]))' "$BASELINE")
fi
unknown="$(python3 -c 'import json,sys; known=json.load(open(sys.argv[1]))["sequences"]; print(" ".join(s for s in sys.argv[2:] if s not in known))' "$BASELINE" "${seqs[@]}")"
if [ -n "$unknown" ]; then
    echo "euroc-consistency: not in $BASELINE: $unknown" >&2
    exit 2
fi
if [ "$record" = 1 ] && { [ "$named" -gt 0 ] || [ -n "${CORTEX_CONSISTENCY_SIGMAS:-}" ] || [ -n "${CORTEX_CONSISTENCY_SEEDS:-}" ]; }; then
    echo "euroc-consistency: --record records the full gate; drop the sequence names and the SIGMAS/SEEDS overrides" >&2
    exit 2
fi

if [ -n "${CORTEX_BIN:-}" ]; then
    BIN="$CORTEX_BIN"
    build=0
else
    BIN="$REPO/build/$PRESET/tools"
fi
if [ "$build" = 1 ]; then
    cmake --build --preset "$PRESET" --target vio_pipeline traj_eval -j 4
fi
for t in vio_pipeline traj_eval; do
    [ -x "$BIN/$t" ] || { echo "euroc-consistency: $BIN/$t is missing (build it, or drop --no-build)" >&2; exit 2; }
done
# A missing dataset is a setup error, not a diverged run: check every
# sequence before running any, so --record can never store it as a result.
for s in "${seqs[@]}"; do
    if [ -z "$(find "$DATASET_ROOT" -type d -name mav0 -path "*/$s/*" -not -path '*__MACOSX*' 2>/dev/null | head -1)" ]; then
        echo "euroc-consistency: no mav0 for $s under $DATASET_ROOT (set CORTEX_EUROC_ROOT)" >&2
        exit 2
    fi
done
OUT="$OUT_ROOT/$backend"
mkdir -p "$OUT"

# run_one <sequence> <sigma> <seed> <matched|default>
run_one() {
    local seq="$1" sigma="$2" seed="$3" r="$4"
    local dir="$OUT/$seq/s${sigma}_seed${seed}_$r" mav0 from rarg=""
    mkdir -p "$dir"
    rm -f "$dir/trajectory.tum" "$dir/truth.tum" "$dir/consistency.json" "$dir/score_window.json"
    mav0="$(find "$DATASET_ROOT" -type d -name mav0 -path "*/$seq/*" -not -path '*__MACOSX*' 2>/dev/null | head -1)"
    if [ -z "$mav0" ]; then
        echo "$seq: no mav0 under $DATASET_ROOT" > "$dir/vio_pipeline.log"
        return 0
    fi
    [ "$r" = matched ] && rarg="--camera-sigma-px $sigma"
    # shellcheck disable=SC2086  # the flag strings split into arguments on purpose
    "$BIN/vio_pipeline" --source euroc --dataset "$mav0" --out "$dir" --tracks gt --imu gt --track-sigma "$sigma" \
        --track-seed "$seed" $rarg $BACKEND_ARGS > "$dir/vio_pipeline.log" 2>&1 || true
    from="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["sequences"][sys.argv[2]]["compare_from_s"])' "$BASELINE" "$seq")"
    "$BIN/traj_eval" --dataset "$mav0" --truth "$dir/truth.tum" --est "$dir/trajectory.tum" --from "$from" --json \
        > "$dir/score_window.json" 2>> "$dir/vio_pipeline.log" || true
}
export -f run_one
export OUT BIN BASELINE DATASET_ROOT BACKEND_ARGS

for s in "${seqs[@]}"; do
    for sigma in $SIGMAS; do
        for seed in $SEEDS; do
            for r in matched default; do
                echo "$s $sigma $seed $r"
            done
        done
    done
done | xargs -P "$JOBS" -L 1 bash -c 'run_one "$@"' _

commit="$(git -C "$REPO" rev-parse --short HEAD)"
python3 - "$BASELINE" "$GATE" "$OUT" "$backend" "$record" "$commit" "$SIGMAS" "$SEEDS" "${seqs[@]}" <<'EOF'
import datetime, json, math, os, statistics, sys

base_path, gate_path, out, backend, record, commit, sigmas, seeds = sys.argv[1:9]
seqs = sys.argv[9:]
record = record == "1"
sigmas, seeds = sigmas.split(), seeds.split()
base = json.load(open(base_path))
gate = json.load(open(gate_path))
B = gate["bounds"]


def load(p):
    try:
        return json.load(open(p))
    except (OSError, ValueError):
        return None


def fin(x):
    return isinstance(x, (int, float)) and math.isfinite(x)


def nis_of(log):
    """The run's normalized NIS, as vio_pipeline prints it."""
    try:
        for line in open(log):
            if line.strip().startswith("NIS (normalized)"):
                v = float(line.split(":", 1)[1])
                return v if math.isfinite(v) else None
    except (OSError, ValueError):
        pass
    return None


runs = {}
for s in seqs:
    ov = base["sequences"][s]["openvins"]["mono"]["ate_4dof"]
    for sg in sigmas:
        for sd in seeds:
            for r in ("matched", "default"):
                d = f"{out}/{s}/s{sg}_seed{sd}_{r}"
                sc, cj = load(f"{d}/score_window.json"), load(f"{d}/consistency.json")
                ate = sc.get("ate_4dof") if sc else None
                try:
                    has_traj = os.path.getsize(f"{d}/trajectory.tum") > 0
                except OSError:
                    has_traj = False
                pose = cj.get("pose", {}) if cj else {}
                led = cj.get("ledger") if cj else None
                ratio = ate / ov if fin(ate) else math.inf
                # No trajectory: the estimator failed (a divergence). A trajectory
                # without a finite score: the evaluation failed (unavailable).
                status = "scored" if fin(ate) else "eval_failed" if has_traj else "no_trajectory"
                ys0, ysm = pose.get("yaw_sigma_first"), pose.get("yaw_sigma_min")
                runs[f"{s}/{sg}/{sd}/{r}"] = {
                    "status": status,
                    "ate_4dof": round(ate, 4) if fin(ate) else None,
                    "ratio": round(ratio, 3) if fin(ratio) else None,
                    "rpe_rot_deg": round(sc["rpe_rot_deg"], 3) if sc and fin(sc.get("rpe_rot_deg")) else None,
                    "nees_yaw": pose.get("nees_yaw"),
                    "nees_rp": pose.get("nees_rp"),
                    "nees_pos": pose.get("nees_pos"),
                    "yaw_sigma_first": pose.get("yaw_sigma_first"),
                    "yaw_sigma_min": pose.get("yaw_sigma_min"),
                    "yaw_sigma_ratio": round(ysm / ys0, 4) if fin(ys0) and fin(ysm) and ys0 > 0 else None,
                    "yaw_err_max": pose.get("yaw_err_max"),
                    "nis": nis_of(f"{d}/vio_pipeline.log"),
                    "ledger_info_yaw": led.get("info_yaw") if led else None,
                    "ledger_share_s6": led.get("share_s6_mean_shift") if led else None,
                }


def ratio(k):
    v = runs[k]["ratio"]
    return v if v is not None else math.inf


def med(xs):
    xs = [x if fin(x) else math.inf for x in xs]
    return statistics.median(xs) if xs else math.inf


def g(x, w=7, p=2):
    return f"{x:{w}.{p}f}" if fin(x) and x < 1e4 else f"{x:{w}.0e}" if fin(x) else f"{'inf':>{w}}"


unavailable = sorted(k for k, v in runs.items() if v["status"] == "eval_failed")
print(f"\nConsistency gate, backend {backend}: window 4-DoF ATE / OpenVINS mono; ideal tracks of S px,")
print("R matched to S or the default; median over seeds. Yaw sigma ratio = min / anchor (target 1);")
print("yaw and roll/pitch ANEES and NIS are informational (NIS ~1 when the noise model is true).")
for sg in sigmas:
    print(f"\n  S = {sg} px")
    print(f"  {'sequence':17}{'matched':>8}{'default':>8}{'diverged':>9}{'yaw sig ratio':>14}{'yaw NEES':>9}{'rp NEES':>9}{'NIS':>7}")
    for s in seqs:
        m = [ratio(f"{s}/{sg}/{sd}/matched") for sd in seeds]
        d = [ratio(f"{s}/{sg}/{sd}/default") for sd in seeds]
        keys = [f"{s}/{sg}/{sd}/matched" for sd in seeds]
        na = sum(1 for k in keys if k in unavailable)  # unscored: neither diverged nor not
        dv = sum(1 for k in keys if k not in unavailable and ratio(k) >= B["diverge_ratio"])
        dcol = f"{dv}+{na}?" if na else str(dv)  # "+n?": n runs could not be scored
        ny = med([runs[f"{s}/{sg}/{sd}/matched"]["nees_yaw"] for sd in seeds])
        nr = med([runs[f"{s}/{sg}/{sd}/matched"]["nees_rp"] for sd in seeds])
        yr = med([runs[f"{s}/{sg}/{sd}/matched"]["yaw_sigma_ratio"] for sd in seeds])
        nn = med([runs[f"{s}/{sg}/{sd}/matched"]["nis"] for sd in seeds])
        print(f"  {s:17}{g(med(m), 8, 1)}{g(med(d), 8, 1)}{dcol:>6}/{len(seeds)}{g(yr, 14, 3)}{g(ny, 9, 1)}{g(nr, 9, 0)}"
              f"{g(nn, 7, 2)}")

verdict = {}
diverged = sorted(k for k in runs if k.endswith("/matched") and k not in unavailable and ratio(k) >= B["diverge_ratio"])
matched_unavailable = [k for k in unavailable if k.endswith("/matched")]
verdict["no_divergence"] = not diverged and not matched_unavailable
beats = {}
for sg in sigmas:
    m = med([ratio(f"{s}/{sg}/{sd}/matched") for s in seqs for sd in seeds])
    d = med([ratio(f"{s}/{sg}/{sd}/default") for s in seqs for sd in seeds])
    beats[sg] = {"matched": round(m, 3) if fin(m) else None, "default": round(d, 3) if fin(d) else None}
# A median over a missing default-R score would read it as infinity and let
# matched R "win": every default run must have been scored for the comparison.
default_missing = sorted(k for k, v in runs.items() if k.endswith("/default") and v["status"] != "scored")
verdict["matched_beats_default"] = not default_missing and all(
    v["matched"] is not None and v["default"] is not None and v["matched"] < v["default"] for v in beats.values())
yaw = med([runs[k]["nees_yaw"] for k in runs if k.endswith("/matched")])
yaw_ratio = statistics.median([runs[k]["yaw_sigma_ratio"] if runs[k]["yaw_sigma_ratio"] is not None else 0.0
                               for k in runs if k.endswith("/matched")])
verdict["no_yaw_collapse"] = yaw_ratio >= B["yaw_sigma_ratio_min"]
passed = all(verdict.values()) and not unavailable

c1 = "PASS" if verdict["no_divergence"] else "FAIL" if diverged else "UNAVAILABLE"
print(f"\n  1. no divergence (matched ratio < {B['diverge_ratio']}x): {c1}"
      + (f" — {len(matched_unavailable)} matched run(s) unscored" if matched_unavailable else "")
      + (f" — {len(diverged)} of {sum(1 for k in runs if k.endswith('/matched'))} diverged: "
         + ", ".join(diverged[:8]) + (" ..." if len(diverged) > 8 else "") if diverged else ""))
print(f"  2. matched R beats the default R (median ratio): {'PASS' if verdict['matched_beats_default'] else 'FAIL'} — "
      + "; ".join(f"S {k}: {g(v['matched'] or math.inf, 1, 2)} vs {g(v['default'] or math.inf, 1, 2)}" for k, v in beats.items()))
if default_missing:
    print(f"     default-R comparison unavailable: {len(default_missing)} default run(s) not scored: "
          + ", ".join(default_missing[:8]))
print(f"  3. no yaw collapse (median yaw sigma min/anchor >= {B['yaw_sigma_ratio_min']}): "
      f"{'PASS' if verdict['no_yaw_collapse'] else 'FAIL'} — {g(yaw_ratio, 1, 3)}")
nis_med = med([runs[k]["nis"] for k in runs if k.endswith("/matched")])
print(f"     (informational: median yaw ANEES {g(yaw, 1, 2)}, not a sound test, see the header; "
      f"median NIS of the matched runs {g(nis_med, 1, 2)})")
if unavailable:
    print(f"\n  UNAVAILABLE: traj_eval could not score {len(unavailable)} run(s) that wrote a trajectory: "
          + ", ".join(unavailable[:8]) + (" ..." if len(unavailable) > 8 else ""))
print(f"\n  gate: {'PASS' if passed else 'UNAVAILABLE' if unavailable else 'FAIL'}  "
      f"({len(seqs)} sequences, S in {sigmas}, seeds {seeds}; {out})")

if record and unavailable:
    print("  not recording: some runs could not be scored (an evaluation fault, not an estimator result)")
elif record:
    gate.setdefault("backends", {})[backend] = {
        "commit": commit,
        "date": datetime.date.today().isoformat(),
        "verdict": {**verdict, "pass": passed, "median_yaw_sigma_ratio": round(yaw_ratio, 4),
                    "median_yaw_nees": round(yaw, 3) if fin(yaw) else None,
                    "median_nis": round(nis_med, 3) if fin(nis_med) else None,
                    "truth": "spline (--imu gt, #613)",
                    "median_ratio": beats, "diverged": diverged},
        "runs": runs,
    }
    open(gate_path, "w").write(json.dumps(gate, indent=1) + "\n")
    print(f"  recorded {backend} in {gate_path} (commit {commit}); update the consistency-gate table in "
          "docs/benchmarks/euroc-baseline.mdx to match")
# The gate is a characterization until an estimator passes: fail the script so
# that a regression in a passing backend is caught.
sys.exit(0 if passed else 1)
EOF
