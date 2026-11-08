#!/usr/bin/env bash
#
# euroc-consistency.sh — the EuRoC consistency gate (issue #605,
# docs/benchmarks/euroc-baseline.mdx "The consistency gate").
#
# Can the estimator be told the true measurement noise and get better rather
# than diverge? Every run keeps the real EuRoC IMU and timestamps and replaces
# the KLT front end with ideal tracks (#602) of a known pixel noise S. Each
# (sequence, S, seed) runs twice: with the filter's R matched to S, and with
# its default R. The verdict (bounds in scripts/euroc-consistency.json):
#
#   1. no divergence: every matched run's window 4-DoF ATE is under
#      diverge_ratio x OpenVINS mono's (failed runs count as diverged);
#   2. matched R beats the default R: at each S, the median window ratio of the
#      matched runs is below the default runs';
#   3. yaw consistency: the median over the matched runs of the yaw ANEES
#      (gauge-anchored, world frame; target 1) is at most yaw_nees_max.
#
# Roll/pitch and position NEES are recorded but not gated: on EuRoC the ground
# truth's body frame and the accelerometer's gravity disagree by 0.1-0.9 deg
# even after the ground truth's own accelerometer bias is removed, which caps
# the roll/pitch NEES any filter can reach against it.
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
            sed -n '2,40p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
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
    rm -f "$dir/trajectory.tum" "$dir/consistency.json" "$dir/score_window.json"
    mav0="$(find "$DATASET_ROOT" -type d -name mav0 -path "*/$seq/*" -not -path '*__MACOSX*' 2>/dev/null | head -1)"
    if [ -z "$mav0" ]; then
        echo "$seq: no mav0 under $DATASET_ROOT" > "$dir/vio_pipeline.log"
        return 0
    fi
    [ "$r" = matched ] && rarg="--camera-sigma-px $sigma"
    # shellcheck disable=SC2086  # the flag strings split into arguments on purpose
    "$BIN/vio_pipeline" --source euroc --dataset "$mav0" --out "$dir" --tracks gt --track-sigma "$sigma" \
        --track-seed "$seed" $rarg $BACKEND_ARGS > "$dir/vio_pipeline.log" 2>&1 || true
    from="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["sequences"][sys.argv[2]]["compare_from_s"])' "$BASELINE" "$seq")"
    "$BIN/traj_eval" --dataset "$mav0" --est "$dir/trajectory.tum" --from "$from" --json \
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
import datetime, json, math, statistics, sys

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


runs = {}
for s in seqs:
    ov = base["sequences"][s]["openvins"]["mono"]["ate_4dof"]
    for sg in sigmas:
        for sd in seeds:
            for r in ("matched", "default"):
                d = f"{out}/{s}/s{sg}_seed{sd}_{r}"
                sc, cj = load(f"{d}/score_window.json"), load(f"{d}/consistency.json")
                ate = sc.get("ate_4dof") if sc else None
                pose = cj.get("pose", {}) if cj else {}
                led = cj.get("ledger") if cj else None
                ratio = ate / ov if fin(ate) else math.inf
                runs[f"{s}/{sg}/{sd}/{r}"] = {
                    "ate_4dof": round(ate, 4) if fin(ate) else None,
                    "ratio": round(ratio, 3) if fin(ratio) else None,
                    "rpe_rot_deg": round(sc["rpe_rot_deg"], 3) if sc and fin(sc.get("rpe_rot_deg")) else None,
                    "nees_yaw": pose.get("nees_yaw"),
                    "nees_rp": pose.get("nees_rp"),
                    "nees_pos": pose.get("nees_pos"),
                    "yaw_sigma_first": pose.get("yaw_sigma_first"),
                    "yaw_sigma_min": pose.get("yaw_sigma_min"),
                    "yaw_err_max": pose.get("yaw_err_max"),
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


print(f"\nConsistency gate, backend {backend}: window 4-DoF ATE / OpenVINS mono; ideal tracks of S px,")
print("R matched to S or the default; median over seeds. Yaw ANEES target 1.")
for sg in sigmas:
    print(f"\n  S = {sg} px")
    print(f"  {'sequence':17}{'matched':>8}{'default':>8}{'diverged':>9}{'yaw NEES':>9}{'rp NEES':>9}{'yaw sigma min':>14}")
    for s in seqs:
        m = [ratio(f"{s}/{sg}/{sd}/matched") for sd in seeds]
        d = [ratio(f"{s}/{sg}/{sd}/default") for sd in seeds]
        dv = sum(1 for x in m if x >= B["diverge_ratio"])
        ny = med([runs[f"{s}/{sg}/{sd}/matched"]["nees_yaw"] for sd in seeds])
        nr = med([runs[f"{s}/{sg}/{sd}/matched"]["nees_rp"] for sd in seeds])
        ys = med([runs[f"{s}/{sg}/{sd}/matched"]["yaw_sigma_min"] for sd in seeds])
        print(f"  {s:17}{g(med(m), 8, 1)}{g(med(d), 8, 1)}{dv:>6}/{len(seeds)}{g(ny, 9, 1)}{g(nr, 9, 0)}{g(ys, 14, 4)}")

verdict = {}
diverged = sorted(k for k in runs if k.endswith("/matched") and ratio(k) >= B["diverge_ratio"])
verdict["no_divergence"] = not diverged
beats = {}
for sg in sigmas:
    m = med([ratio(f"{s}/{sg}/{sd}/matched") for s in seqs for sd in seeds])
    d = med([ratio(f"{s}/{sg}/{sd}/default") for s in seqs for sd in seeds])
    beats[sg] = {"matched": round(m, 3) if fin(m) else None, "default": round(d, 3) if fin(d) else None}
verdict["matched_beats_default"] = all(
    v["matched"] is not None and v["matched"] < (v["default"] if v["default"] is not None else math.inf)
    for v in beats.values())
yaw = med([runs[k]["nees_yaw"] for k in runs if k.endswith("/matched")])
verdict["yaw_consistent"] = fin(yaw) and yaw <= B["yaw_nees_max"]
passed = all(verdict.values())

print(f"\n  1. no divergence (matched ratio < {B['diverge_ratio']}x): {'PASS' if verdict['no_divergence'] else 'FAIL'}"
      + (f" — {len(diverged)} of {sum(1 for k in runs if k.endswith('/matched'))} diverged: "
         + ", ".join(diverged[:8]) + (" ..." if len(diverged) > 8 else "") if diverged else ""))
print(f"  2. matched R beats the default R (median ratio): {'PASS' if verdict['matched_beats_default'] else 'FAIL'} — "
      + "; ".join(f"S {k}: {g(v['matched'] or math.inf, 1, 2)} vs {g(v['default'] or math.inf, 1, 2)}" for k, v in beats.items()))
print(f"  3. median yaw ANEES of the matched runs <= {B['yaw_nees_max']}: {'PASS' if verdict['yaw_consistent'] else 'FAIL'}"
      f" — {g(yaw, 1, 2)}")
print(f"\n  gate: {'PASS' if passed else 'FAIL'}  ({len(seqs)} sequences, S in {sigmas}, seeds {seeds}; {out})")

if record:
    gate.setdefault("backends", {})[backend] = {
        "commit": commit,
        "date": datetime.date.today().isoformat(),
        "verdict": {**verdict, "pass": passed, "median_yaw_nees": round(yaw, 3) if fin(yaw) else None,
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
