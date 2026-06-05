#!/usr/bin/env bash
# vio_scene_video.sh — one-shot: run the VIO pipeline experiment, generate the
# per-frame scene + overlay data, and assemble it into an mp4 (and, with --sweep,
# the noise→robustness figures). See docs/assessments/vio-pipeline-howto.md.
#
#   scripts/vio_scene_video.sh [options]
#
# Options:
#   -o, --out DIR        output directory            (default: build/vio_scene)
#       --robot R        ground | drone | default    (default: ground)
#       --noise S        sensor-noise multiplier      (default: 1)
#       --source S       synthetic | euroc            (default: synthetic)
#       --dataset PATH   EuRoC mav0 root (for --source euroc)
#       --fps N          output video frame rate      (default: 20)
#       --sweep          also emit the noise→robustness curves (SVG)
#       --keep           keep intermediate frame SVG/PNG files
#       --build          (re)build vio_pipeline first
#   -h, --help           this help
#
# Needs: a built vio_pipeline, node, ffmpeg, and an SVG rasterizer
# (rsvg-convert | cairosvg | inkscape).

set -euo pipefail

# ── Locate the repo root (this script lives in <root>/scripts) ──────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# ── Defaults ────────────────────────────────────────────────────────────────
OUT="$ROOT/build/vio_scene"
ROBOT="ground"
NOISE="1"
SOURCE="synthetic"
DATASET=""
FPS="20"
SWEEP=0
KEEP=0
BUILD=0

usage() { sed -n '2,30p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit "${1:-0}"; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    -o|--out)     OUT="$2"; shift 2;;
    --robot)      ROBOT="$2"; shift 2;;
    --noise)      NOISE="$2"; shift 2;;
    --source)     SOURCE="$2"; shift 2;;
    --dataset)    DATASET="$2"; shift 2;;
    --fps)        FPS="$2"; shift 2;;
    --sweep)      SWEEP=1; shift;;
    --keep)       KEEP=1; shift;;
    --build)      BUILD=1; shift;;
    -h|--help)    usage 0;;
    *) echo "unknown option: $1" >&2; usage 1;;
  esac
done

say() { printf '\033[1;36m==>\033[0m %s\n' "$*"; }
die() { printf '\033[1;31merror:\033[0m %s\n' "$*" >&2; exit 1; }

BIN="$ROOT/build/tools/vio_pipeline"
OVERLAY="$ROOT/docs-site/scripts/gen-overlay.mjs"
FIGURES="$ROOT/docs-site/scripts/gen-sensor-model-figures.mjs"

# ── Tool checks ─────────────────────────────────────────────────────────────
command -v node   >/dev/null || die "node not found (needed for the overlay generator)"
command -v ffmpeg >/dev/null || die "ffmpeg not found (apt-get install ffmpeg)"

RASTER=""
if   command -v rsvg-convert >/dev/null; then RASTER="rsvg-convert"
elif command -v cairosvg     >/dev/null; then RASTER="cairosvg"
elif command -v inkscape     >/dev/null; then RASTER="inkscape"
else die "no SVG rasterizer found — install one of: librsvg2-bin (rsvg-convert), cairosvg, inkscape"
fi

if [[ $BUILD -eq 1 || ! -x "$BIN" ]]; then
  say "building vio_pipeline"
  cmake -B "$ROOT/build" -S "$ROOT" -DBUILD_TARGET_KPU=OFF >/dev/null
  cmake --build "$ROOT/build" --target vio_pipeline -j"$(nproc)" >/dev/null
fi
[[ -x "$BIN" ]] || die "vio_pipeline not built — run with --build (or: cmake --build build --target vio_pipeline)"

mkdir -p "$OUT"

# ── 1) run the experiment with --video (emits scene/ + frames.jsonl) ────────
say "running vio_pipeline (source=$SOURCE robot=$ROBOT noise=$NOISE)"
RUN_ARGS=(--source "$SOURCE" --video --out "$OUT")
[[ "$SOURCE" == "synthetic" ]] && RUN_ARGS+=(--robot "$ROBOT" --noise "$NOISE")
[[ -n "$DATASET" ]] && RUN_ARGS+=(--dataset "$DATASET")
"$BIN" "${RUN_ARGS[@]}"
[[ -s "$OUT/frames.jsonl" ]] || die "no frames.jsonl produced — for --source euroc, pass a valid --dataset"

# ── 2) the noise→robustness curves (optional) ───────────────────────────────
if [[ $SWEEP -eq 1 && "$SOURCE" == "synthetic" ]]; then
  say "noise sweep → robustness curves"
  "$BIN" --sweep --robot "$ROBOT" --out "$OUT" >/dev/null
  # The shared figure generator also tries the other stages' CSVs and warns when
  # they're absent — expected here, so quiet it (the pipeline figures still emit).
  node "$FIGURES" "$OUT" "$OUT/figures" >/dev/null 2>&1
fi

# ── 3) compose the per-frame overlay SVGs ───────────────────────────────────
say "compositing overlay frames (scene + features + metrics)"
node "$OVERLAY" "$OUT" >/dev/null
FRAMES="$OUT/frames"
shopt -s nullglob
SVGS=("$FRAMES"/frame_*.svg)
[[ ${#SVGS[@]} -gt 0 ]] || die "no overlay frames generated"

# ── 4) rasterize SVG → PNG ──────────────────────────────────────────────────
say "rasterizing ${#SVGS[@]} frames with $RASTER"
for f in "${SVGS[@]}"; do
  png="${f%.svg}.png"
  case "$RASTER" in
    rsvg-convert) rsvg-convert "$f" -o "$png" ;;
    cairosvg)     cairosvg "$f" -o "$png" ;;
    inkscape)     inkscape "$f" --export-type=png --export-filename="$png" >/dev/null 2>&1 ;;
  esac
done

# ── 5) assemble the mp4 (glob pattern — robust to the start frame number) ───
MP4="$OUT/scene.mp4"
say "encoding $MP4 @ ${FPS}fps"
ffmpeg -y -loglevel error -framerate "$FPS" -pattern_type glob -i "$FRAMES/frame_*.png" \
  -vf "pad=ceil(iw/2)*2:ceil(ih/2)*2" -pix_fmt yuv420p "$MP4"

# ── cleanup ─────────────────────────────────────────────────────────────────
if [[ $KEEP -eq 0 ]]; then
  rm -f "$FRAMES"/frame_*.svg "$FRAMES"/frame_*.png
  rmdir "$FRAMES" 2>/dev/null || true
fi

say "done"
echo "  video : $MP4"
[[ $SWEEP -eq 1 && "$SOURCE" == "synthetic" ]] && echo "  curves: $OUT/figures/"
echo "  data  : $OUT/{run.jsonl,trajectory.csv,frames.jsonl}"
