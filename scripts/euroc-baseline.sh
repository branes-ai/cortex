#!/usr/bin/env bash
#
# euroc-baseline.sh — the EuRoC regression benchmark
# (docs/benchmarks/euroc-baseline.mdx).
#
# Runs vio_pipeline on every EuRoC MAV sequence (or the ones named), scores
# each trajectory with traj_eval, and compares the scores with the recorded
# cortex baseline in scripts/euroc-baseline.json. It fails when a sequence
# gets worse than the baseline by more than the tolerance, and lists the ones
# that got better. The OpenVINS columns it prints are recorded targets; this
# script never runs another estimator or reads its output (ADR 0002).
#
#   scripts/euroc-baseline.sh                    # all 11 sequences
#   scripts/euroc-baseline.sh V1_01_easy MH_05_difficult
#   scripts/euroc-baseline.sh --update           # rerun, then record the new numbers
#   scripts/euroc-baseline.sh --no-build ...     # reuse the built tools
#
# Environment overrides:
#   CORTEX_EUROC_ROOT     dataset root (default /srv/samba/sw-21/EuRoC-MAV-dataset)
#   CORTEX_PRESET         CMake preset to build and run (default sitl-release)
#   CORTEX_BASELINE_JOBS  sequences run at once (default 2; each run is one core
#                         and ~1.5 GB of images read from disk)
#   CORTEX_BASELINE_OUT   output directory (default build/euroc-baseline)
#   CORTEX_BASELINE_TOL   relative tolerance before a change counts (default 0.10)
#
# A full serial run takes ~25 minutes on the development station.
#
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASELINE="$REPO/scripts/euroc-baseline.json"
DATASET_ROOT="${CORTEX_EUROC_ROOT:-/srv/samba/sw-21/EuRoC-MAV-dataset}"
PRESET="${CORTEX_PRESET:-sitl-release}"
JOBS="${CORTEX_BASELINE_JOBS:-2}"
OUT="${CORTEX_BASELINE_OUT:-$REPO/build/euroc-baseline}"
TOL="${CORTEX_BASELINE_TOL:-0.10}"

build=1
update=0
seqs=()
for a in "$@"; do
    case "$a" in
        --no-build) build=0 ;;
        --update) update=1 ;;
        -h | --help)
            sed -n '2,27p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        -*)
            echo "euroc-baseline: unknown flag $a" >&2
            exit 2
            ;;
        *) seqs+=("$a") ;;
    esac
done
named=${#seqs[@]}
if [ "$named" -eq 0 ]; then
    mapfile -t seqs < <(python3 -c 'import json,sys; print("\n".join(json.load(open(sys.argv[1]))["sequences"]))' "$BASELINE")
fi

if [ "$update" = 1 ] && [ "$named" -gt 0 ]; then
    echo "euroc-baseline: --update records a full run; drop the sequence names" >&2
    exit 2
fi

BIN="$REPO/build/$PRESET/tools"
if [ "$build" = 1 ]; then
    cmake --build --preset "$PRESET" --target vio_pipeline traj_eval -j 4
fi
for t in vio_pipeline traj_eval; do
    [ -x "$BIN/$t" ] || { echo "euroc-baseline: $BIN/$t is missing (build it, or drop --no-build)" >&2; exit 2; }
done
mkdir -p "$OUT"

# run_one <sequence>: vio_pipeline, then traj_eval over the whole run and over
# the window shared with the recorded OpenVINS mono score.
run_one() {
    local seq="$1"
    local mav0 from start
    mav0="$(find "$DATASET_ROOT" -type d -name mav0 -path "*/$seq/*" -not -path '*__MACOSX*' 2>/dev/null | head -1)"
    if [ -z "$mav0" ]; then
        echo "$seq: no mav0 under $DATASET_ROOT"
        return 0
    fi
    mkdir -p "$OUT/$seq"
    rm -f "$OUT/$seq/trajectory.tum" "$OUT/$seq/score.json" "$OUT/$seq/score_window.json"
    start=$(date +%s)
    "$BIN/vio_pipeline" --source euroc --dataset "$mav0" --out "$OUT/$seq" > "$OUT/$seq/vio_pipeline.log" 2>&1 || true
    from="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["sequences"][sys.argv[2]]["compare_from_s"])' "$BASELINE" "$seq")"
    "$BIN/traj_eval" --dataset "$mav0" --est "$OUT/$seq/trajectory.tum" --json > "$OUT/$seq/score.json" 2>> "$OUT/$seq/vio_pipeline.log" || true
    "$BIN/traj_eval" --dataset "$mav0" --est "$OUT/$seq/trajectory.tum" --from "$from" --json \
        > "$OUT/$seq/score_window.json" 2>> "$OUT/$seq/vio_pipeline.log" || true
    echo "$seq: $(( $(date +%s) - start )) s"
}
export -f run_one
export OUT BIN BASELINE DATASET_ROOT

printf '%s\n' "${seqs[@]}" | xargs -P "$JOBS" -I{} bash -c 'run_one "$1"' _ {}

commit="$(git -C "$REPO" rev-parse --short HEAD)"
python3 - "$BASELINE" "$OUT" "$TOL" "$update" "$commit" "${seqs[@]}" <<'EOF'
import datetime, json, math, sys

path, out, tol, update, commit, seqs = sys.argv[1], sys.argv[2], float(sys.argv[3]), sys.argv[4] == "1", sys.argv[5], sys.argv[6:]
base = json.load(open(path))
KEEP = ("ate_4dof", "ate_se3", "rpe_trans_m", "rpe_rot_deg", "start_s", "coverage_s")
GATED = ("ate_4dof", "rpe_trans_m", "rpe_rot_deg")
ABS = {"ate_4dof": 0.005, "rpe_trans_m": 0.005, "rpe_rot_deg": 0.05}  # noise floor (m, m, deg)


def load(p):
    try:
        j = json.load(open(p))
        return {k: j[k] for k in KEEP} if all(isinstance(j.get(k), (int, float)) and math.isfinite(j[k]) for k in KEEP) else None
    except (OSError, ValueError, KeyError):
        return None


def fmt(x):
    return f"{x:9.3f}" if x < 1000 else f"{x:9.0f}"


worse, better, failed, now = [], [], [], {}
print(f"\nATE 4-DoF (m); window = from OpenVINS mono's first pose. Tolerance {tol:.0%}.")
print(f"{'sequence':17}{'run':>9}{'baseline':>9}{'window':>9}{'OV mono':>9}{'OV stereo':>10}{'x mono':>8}  verdict")
for s in seqs:
    b = base["sequences"][s]
    run, win = load(f"{out}/{s}/score.json"), load(f"{out}/{s}/score_window.json")
    if run is None or win is None:
        failed.append(s)
        print(f"{s:17}{'—':>9}{fmt(b['cortex']['run']['ate_4dof'])}{'—':>9}  FAILED (see {out}/{s}/vio_pipeline.log)")
        continue
    now[s] = {"run": {k: round(v, 4) for k, v in run.items()}, "window": {k: round(v, 4) for k, v in win.items()}}
    verdict = []
    for k in GATED:
        old, new = b["cortex"]["run"][k], run[k]
        if new > old * (1 + tol) + ABS[k]:
            verdict.append(f"worse {k} {old:.3f}->{new:.3f}")
        elif new < old * (1 - tol) - ABS[k]:
            verdict.append(f"better {k} {old:.3f}->{new:.3f}")
    (worse if any(v.startswith("worse") for v in verdict) else better if verdict else []).append(s)
    ov = b["openvins"]
    print(f"{s:17}{fmt(run['ate_4dof'])}{fmt(b['cortex']['run']['ate_4dof'])}{fmt(win['ate_4dof'])}"
          f"{fmt(ov['mono']['ate_4dof'])}{fmt(ov['stereo']['ate_4dof']):>10}{win['ate_4dof'] / ov['mono']['ate_4dof']:8.1f}"
          f"  {'; '.join(verdict) or 'same'}")

print(f"\n{len(seqs) - len(failed)}/{len(seqs)} scored; worse: {worse or 'none'}; better: {better or 'none'}"
      + (f"; failed: {failed}" if failed else ""))
if update and not failed:
    for s, v in now.items():
        base["sequences"][s]["cortex"] = v
    base["cortex"]["commit"] = commit
    base["cortex"]["date"] = datetime.date.today().isoformat()
    open(path, "w").write(json.dumps(base, indent=1) + "\n")
    print(f"recorded the new cortex numbers in {path} (commit {commit}); update the table in "
          "docs/benchmarks/euroc-baseline.mdx to match")
elif update:
    print("not recording: some sequences failed")
sys.exit(1 if worse or failed else 0)
EOF
