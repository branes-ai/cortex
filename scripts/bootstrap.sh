#!/usr/bin/env bash
# bootstrap.sh — set up a Linux dev environment for branes-ai/cortex.
#
# Run after a fresh clone:
#   scripts/bootstrap.sh                  # check + install per-user (rustup)
#   scripts/bootstrap.sh --install-deps   # also try `sudo apt install ...` for system tools
#
# Idempotent: re-runs are safe and skip steps that are already satisfied.

set -euo pipefail

INSTALL_DEPS=0
for arg in "$@"; do
    case "$arg" in
        --install-deps) INSTALL_DEPS=1 ;;
        -h|--help)
            sed -n '2,8p' "$0" | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        *) echo "unknown argument: $arg" >&2; exit 2 ;;
    esac
done

# Color helpers (degrade gracefully if not a TTY).
if [ -t 1 ]; then
    GREEN=$'\e[32m' YELLOW=$'\e[33m' RED=$'\e[31m' BOLD=$'\e[1m' NC=$'\e[0m'
else
    GREEN= YELLOW= RED= BOLD= NC=
fi

ok()   { printf "%s[ok]%s    %s\n"   "$GREEN" "$NC" "$*"; }
todo() { printf "%s[todo]%s  %s\n"   "$YELLOW" "$NC" "$*"; }
err()  { printf "%s[err]%s   %s\n"   "$RED" "$NC" "$*" >&2; }
step() { printf "\n%s== %s ==%s\n"   "$BOLD" "$*" "$NC"; }

MISSING=()

require() {
    local name="$1" hint="${2:-}"
    if command -v "$name" >/dev/null 2>&1; then
        ok "$name: $(command -v "$name")"
    else
        todo "$name not found${hint:+ — $hint}"
        MISSING+=("$name")
    fi
}

# ── System tools ────────────────────────────────────────────────────
step "System tools"
require git
require cmake "needs >= 3.25; install via 'apt install cmake' or kitware-cmake"
require ninja "install via 'apt install ninja-build'"
require cc    "C compiler; 'apt install build-essential'"
require c++   "C++ compiler; 'apt install build-essential'"
require clang-format "for formatting checks; 'apt install clang-format'"

# Validate CMake version if present.
if command -v cmake >/dev/null 2>&1; then
    cmake_ver=$(cmake --version | head -1 | awk '{print $3}')
    cmake_major=$(echo "$cmake_ver" | cut -d. -f1)
    cmake_minor=$(echo "$cmake_ver" | cut -d. -f2)
    if [ "$cmake_major" -lt 3 ] || { [ "$cmake_major" -eq 3 ] && [ "$cmake_minor" -lt 25 ]; }; then
        err "cmake $cmake_ver is too old (CMakePresets.json v6 needs >= 3.25)"
        MISSING+=("cmake>=3.25")
    fi
fi

# ── Optional: KPU cross-toolchain ──────────────────────────────────
step "KPU cross-toolchain (optional)"
if command -v aarch64-linux-gnu-gcc >/dev/null 2>&1 && \
   command -v aarch64-linux-gnu-g++ >/dev/null 2>&1; then
    ok "aarch64-linux-gnu cross-toolchain present"
else
    todo "aarch64-linux-gnu cross-toolchain missing — 'apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu' (only needed for kpu-cross preset)"
fi

# ── apt install pass (only if --install-deps) ──────────────────────
if [ "$INSTALL_DEPS" -eq 1 ] && [ "${#MISSING[@]}" -gt 0 ]; then
    step "Installing system packages via apt"
    sudo apt-get update
    sudo apt-get install -y \
        cmake ninja-build build-essential clang-format \
        gcc-aarch64-linux-gnu g++-aarch64-linux-gnu \
        pkg-config curl
fi

# ── Rust toolchain ─────────────────────────────────────────────────
step "Rust toolchain"
if command -v rustup >/dev/null 2>&1; then
    ok "rustup: $(rustup --version 2>/dev/null | head -1)"
else
    todo "installing rustup (per-user; no sudo)"
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --default-toolchain none
    # shellcheck source=/dev/null
    . "$HOME/.cargo/env"
fi

# rust-toolchain.toml at the repo root drives the install. Force it now
# so the first cmake configure doesn't pay the cost.
if [ -f "$(dirname "$0")/../rust-toolchain.toml" ]; then
    step "Pinned Rust toolchain (from rust-toolchain.toml)"
    (cd "$(dirname "$0")/.." && rustup show active-toolchain || rustup toolchain install)
fi

# ── Final report ───────────────────────────────────────────────────
step "Summary"
if [ "${#MISSING[@]}" -eq 0 ]; then
    ok "All required tools present."
    cat <<'NEXT'

Next steps:
  cmake --preset sitl-debug
  cmake --build --preset sitl-debug
  ctest  --preset sitl-debug

For the KPU cross-compile target:
  cmake --preset kpu-cross
  cmake --build --preset kpu-cross
NEXT
else
    err "Missing or out-of-date: ${MISSING[*]}"
    err "Re-run with --install-deps to apt-install them, or install manually."
    exit 1
fi
