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

# locate_mav0 <sequence-dir> <name> → prints the sequence's `mav0` directory.
# Reuses an already-extracted mav0 (anywhere under the sequence dir or under
# build/euroc-data/<name>); otherwise extracts the first `.zip` found in the
# sequence dir into build/euroc-data/<name>. The EuRoC layout nests each
# sequence in its own directory (machine_hall/MH_05_difficult/…), so we search
# rather than assume a fixed zip path.
# Separate `local` decls on purpose: in `local a=$1 b=$2 c=$b`, bash expands c's
# RHS before b is assigned, so under `set -u` `$name` would read the unset
# global and abort with "name: unbound variable".
locate_mav0() {
    local seqdir="$1"
    local name="$2"
    local dest="$WORK/$name"
    local existing
    existing="$(find "$seqdir" "$dest" -maxdepth 3 -type d -name mav0 2>/dev/null | head -1 || true)"
    if [ -n "$existing" ]; then
        echo "$existing"
        return 0
    fi
    local zip
    zip="$(find "$seqdir" -maxdepth 1 -type f -name '*.zip' 2>/dev/null | head -1 || true)"
    if [ -z "$zip" ]; then
        echo "  (no extracted mav0 or .zip under $seqdir — skipping $name)" >&2
        return 0
    fi
    echo "  extracting $name (one-time) from $(basename "$zip") ..." >&2
    mkdir -p "$dest"
    unzip -q "$zip" -d "$dest"
    find "$dest" -maxdepth 3 -type d -name mav0 | head -1
}

echo "== locating sequences under $DATASET_ROOT =="
MH05="$(locate_mav0 "$DATASET_ROOT/machine_hall/MH_05_difficult" MH_05_difficult)"
V203="$(locate_mav0 "$DATASET_ROOT/vicon_room2/V2_03_difficult" V2_03_difficult)"
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
