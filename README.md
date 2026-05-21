# cortex

Core Embodied AI Framework for perception

Create the `branes-ai/cortex` repository and establish the root folder structure. This strictly enforces the boundaries between your Rust hardware layer, your math, your algorithms, and your middleware.

```text
branes-ai/cortex/
├── .github/
│   └── workflows/
│       └── ci.yml               # GitHub Actions CI matrix
├── core/                        # Rust Resource Manager (HAL, Memory, KPU sync)
│   ├── Cargo.toml
│   ├── build.rs                 # cxx C++ header generation
│   └── src/
│       ├── lib.rs
│       └── memory_provider.rs   # The SITL vs SoC abstraction
├── math/                        # C++ MTL5 NLS Solvers & Data Types
│   ├── CMakeLists.txt
│   └── include/
│       └── branes/math/         # Header-only implementations
├── sdk/                         # C++ Spatial Operators (VIO, SLAM, 3DSG)
│   ├── CMakeLists.txt
│   ├── include/
│   │   └── branes/sdk/
│   └── src/
├── daemons/                     # C++ Zenoh Wrappers (The Application surface)
│   ├── CMakeLists.txt
│   └── src/
│       └── vio_daemon.cpp       # Zenoh pub/sub + YAML parsing + SDK invocation
├── config/                      # Unified YAML configuration schemas
│   └── default_bot.yaml
└── CMakeLists.txt               # The Master Build Script

```


