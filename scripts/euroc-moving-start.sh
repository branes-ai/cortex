#!/usr/bin/env bash
#
# euroc-moving-start.sh — run the dataset-gated MH_05 / V2_03 moving-start
# dynamic-VI-init validation (epic #211) on the development station.
#
# Locates the published EuRoC ASL sequences (extracting the .zip on first use),
# points the gated tests at their `mav0` directories, builds an optimized
# (release) `vio_euroc`, and runs the `[dataset]` cases — which print the init
# method and ATE for each sequence (WARN lines), so the 1.5 m gate can be tuned.
#
#   scripts/euroc-moving-start.sh
#
# Environment overrides:
#   CORTEX_EUROC_ROOT  dataset root (default /srv/samba/sw-21/EuRoC-MAV-dataset)
#   CORTEX_PRESET      CMake preset to build/run (default sitl-release)
#
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DATASET_ROOT="${CORTEX_EUROC_ROOT:-/srv/samba/sw-21/EuRoC-MAV-dataset}"
PRESET="${CORTEX_PRESET:-sitl-release}"
WORK="$REPO/build/euroc-data"

# extract_seq <zip> <name> → prints the sequence's mav0 directory, extracting
# the ASL archive into build/euroc-data/<name> the first time.
extract_seq() {
    # Separate declarations: in a single `local a=$1 b=$2 c=$b`, bash expands
    # c's RHS before b is assigned, so under `set -u` `$name` reads the unset
    # *global* and aborts with "name: unbound variable".
    local zip="$1"
    local name="$2"
    local dest="$WORK/$name"
    if [ ! -d "$dest" ]; then
        if [ ! -f "$zip" ]; then
            echo "  (missing $zip — skipping $name)" >&2
            return 0
        fi
        echo "  extracting $name (one-time) ..." >&2
        mkdir -p "$dest"
        unzip -q "$zip" -d "$dest"
    fi
    find "$dest" -type d -name mav0 | head -1
}

echo "== locating sequences under $DATASET_ROOT =="
MH05="$(extract_seq "$DATASET_ROOT/machine_hall/MH_05_difficult.zip" MH_05_difficult)"
V203="$(extract_seq "$DATASET_ROOT/vicon_room2/V2_03_difficult.zip" V2_03_difficult)"
echo "  MH_05 mav0: ${MH05:-<not found>}"
echo "  V2_03 mav0: ${V203:-<not found>}"
if [ -z "${MH05}${V203}" ]; then
    echo "no sequences found — set CORTEX_EUROC_ROOT to the EuRoC dataset directory" >&2
    exit 1
fi

echo "== building $PRESET / vio_euroc =="
cmake --preset "$PRESET" >/dev/null
cmake --build --preset "$PRESET" --target vio_euroc

echo "== running the gated moving-start validation =="
export CORTEX_EUROC_MH05="${MH05:-}"
export CORTEX_EUROC_V203="${V203:-}"
"$REPO/build/$PRESET/tests/vio_euroc" "[dataset]"
