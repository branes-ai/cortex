#Requires -Version 5.1
<#
.SYNOPSIS
    bootstrap.ps1 — set up a Windows dev environment for branes-ai/cortex.

.DESCRIPTION
    Validates Visual Studio 2022 + CMake versions and installs rustup
    with the MSVC toolchain. Run after a fresh clone:

        scripts\bootstrap.ps1

    Idempotent: re-runs are safe and skip steps that are already satisfied.
#>

[CmdletBinding()]
param(
    [switch] $SkipRustup
)

$ErrorActionPreference = 'Stop'

function Write-Ok    ($msg) { Write-Host "[ok]   $msg" -ForegroundColor Green }
function Write-Todo  ($msg) { Write-Host "[todo] $msg" -ForegroundColor Yellow }
function Write-Err   ($msg) { Write-Host "[err]  $msg" -ForegroundColor Red }
function Write-Step  ($msg) { Write-Host ""; Write-Host "== $msg ==" -ForegroundColor Cyan }

$Missing = @()

function Require-Command ($name, $hint = '') {
    $cmd = Get-Command $name -ErrorAction SilentlyContinue
    if ($cmd) {
        Write-Ok "$name found: $($cmd.Source)"
    } else {
        Write-Todo "$name not found$(if ($hint) { ' — ' + $hint })"
        $script:Missing += $name
    }
}

# ── Visual Studio 2022 ──────────────────────────────────────────────
Write-Step "Visual Studio 2022"
$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
if (Test-Path $vswhere) {
    $vsInstall = & $vswhere -version "[17.0,18.0)" -property installationPath -latest 2>$null
    if ($vsInstall) {
        Write-Ok "Visual Studio 2022 at $vsInstall"
    } else {
        Write-Todo "vswhere ran but no VS 2022 installation found — install 'Desktop development with C++' workload + CMake Tools"
        $script:Missing += 'Visual Studio 2022'
    }
} else {
    Write-Todo "vswhere.exe not present — install Visual Studio 2022 with the 'Desktop development with C++' workload"
    $script:Missing += 'Visual Studio 2022'
}

# ── CMake ───────────────────────────────────────────────────────────
Write-Step "CMake"
Require-Command -name 'cmake' -hint 'install via VS Installer or kitware.com/cmake/download'
if (Get-Command cmake -ErrorAction SilentlyContinue) {
    $cmakeLine = (& cmake --version | Select-Object -First 1)
    if ($cmakeLine -match 'cmake version (\d+)\.(\d+)\.(\d+)') {
        $major = [int]$Matches[1]; $minor = [int]$Matches[2]
        if ($major -lt 3 -or ($major -eq 3 -and $minor -lt 25)) {
            Write-Err "cmake $($Matches[0]) is too old (CMakePresets.json v6 needs >= 3.25)"
            $script:Missing += 'cmake>=3.25'
        } else {
            Write-Ok "$cmakeLine"
        }
    }
}

# ── Ninja ───────────────────────────────────────────────────────────
Write-Step "Ninja generator"
Require-Command -name 'ninja' -hint 'bundled with VS 2022 (open the Developer Command Prompt) or `winget install Ninja-build.Ninja`'

# ── WSL2 (for kpu-cross preset) ─────────────────────────────────────
Write-Step "WSL2 (for kpu-cross preset; optional)"
if (Get-Command wsl -ErrorAction SilentlyContinue) {
    $wslList = & wsl --list --verbose 2>$null
    if ($LASTEXITCODE -eq 0) {
        Write-Ok 'WSL2 available; the kpu-cross preset can run inside a WSL2 Linux distro.'
    } else {
        Write-Todo "wsl.exe present but no distros installed — run 'wsl --install -d Ubuntu' if you plan to cross-compile for KPU."
    }
} else {
    Write-Todo "WSL not enabled — only required if you plan to cross-compile for the KPU silicon."
}

# ── Rust (rustup + MSVC toolchain) ─────────────────────────────────
Write-Step "Rust toolchain"
if ($SkipRustup) {
    Write-Todo 'Skipping rustup setup (--SkipRustup specified).'
} elseif (Get-Command rustup -ErrorAction SilentlyContinue) {
    Write-Ok "rustup: $(rustup --version | Select-Object -First 1)"
    # The rust-toolchain.toml at the repo root pins 1.83.0 and MSVC targets.
    # Force the install now so the first cmake configure doesn't pay the cost.
    $repoRoot = Resolve-Path "$PSScriptRoot\.."
    Push-Location $repoRoot
    try { & rustup show active-toolchain | Out-Null } finally { Pop-Location }
} else {
    Write-Todo 'rustup not found — installing rustup-init.exe (per-user, no admin)'
    $rustupInit = Join-Path $env:TEMP 'rustup-init.exe'
    Invoke-WebRequest -UseBasicParsing -Uri 'https://win.rustup.rs/x86_64' -OutFile $rustupInit
    & $rustupInit -y --default-toolchain none --default-host x86_64-pc-windows-msvc
    Write-Host 'Open a new terminal so rustup is on PATH, then re-run this script.'
    exit 0
}

# ── Summary ─────────────────────────────────────────────────────────
Write-Step "Summary"
if ($Missing.Count -eq 0) {
    Write-Ok 'All required tools present.'
    Write-Host ''
    Write-Host 'Next steps:'
    Write-Host '  cmake --preset sitl-debug'
    Write-Host '  cmake --build --preset sitl-debug'
    Write-Host '  ctest  --preset sitl-debug'
    Write-Host ''
    Write-Host 'Or open the cortex folder in Visual Studio 2022 and pick sitl-debug from the preset dropdown.'
} else {
    Write-Err "Missing or out-of-date: $($Missing -join ', ')"
    Write-Err 'Install the items above and re-run this script.'
    exit 1
}
