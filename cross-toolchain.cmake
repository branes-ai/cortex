# cross-toolchain.cmake — aarch64-linux-gnu cross-compile for the KPU target.
#
# Activated via -DCMAKE_TOOLCHAIN_FILE=<path> or the `kpu-cross` preset.
# Requires gcc-aarch64-linux-gnu / g++-aarch64-linux-gnu on Debian/Ubuntu
# (or equivalent ARM cross-compiler on other distros), plus the matching
# Rust target installed (handled by rust-toolchain.toml).
#
# RISC-V variant is a follow-up — see #6 / issue body.

set(CMAKE_SYSTEM_NAME      Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# C / C++ compilers and binutils.
set(CMAKE_C_COMPILER   aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
set(CMAKE_AR           aarch64-linux-gnu-ar)
set(CMAKE_RANLIB       aarch64-linux-gnu-ranlib)
set(CMAKE_STRIP        aarch64-linux-gnu-strip)
set(CMAKE_LINKER       aarch64-linux-gnu-ld)

# Sysroot — opt-in. Set CROSS_SYSROOT (cmake var or env) to a custom KPU
# board rootfs to enable. When unset, we rely on the cross-compiler's
# built-in library/header paths, which is what Debian/Ubuntu's multiarch
# cross-toolchain packages (gcc-aarch64-linux-gnu) expect: their libm.so
# is a GNU ld linker script with absolute paths that conflicts with
# --sysroot. A real KPU rootfs will set CROSS_SYSROOT explicitly.
if(NOT DEFINED CROSS_SYSROOT AND DEFINED ENV{CROSS_SYSROOT})
    set(CROSS_SYSROOT "$ENV{CROSS_SYSROOT}")
endif()
if(DEFINED CROSS_SYSROOT)
    set(CMAKE_SYSROOT "${CROSS_SYSROOT}")
endif()

# Search-path discipline: never use host programs path for target
# libraries / headers, but allow find_program to reach host tools so
# we can still call cmake / cargo / clang-format etc. from the host.
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Tell Corrosion which Rust target triple to cross-compile to. The
# matching target is preinstalled by rust-toolchain.toml.
set(Rust_CARGO_TARGET "aarch64-unknown-linux-gnu" CACHE STRING
    "Rust target triple for cross-compilation (aarch64-linux-gnu).")

# Tell cargo which linker to use for the aarch64-unknown-linux-gnu
# target. Without this, cargo would default to the host `cc`, which
# can't produce aarch64 binaries.
set(ENV{CARGO_TARGET_AARCH64_UNKNOWN_LINUX_GNU_LINKER} "${CMAKE_C_COMPILER}")
