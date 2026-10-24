# cortex

![CodeRabbit Pull Request Reviews](https://img.shields.io/coderabbit/prs/github/branes-ai/cortex?utm_source=oss&utm_medium=github&utm_campaign=branes-ai%2Fcortex&labelColor=171717&color=FF570A&link=https%3A%2F%2Fcoderabbit.ai&label=CodeRabbit+Reviews)

Core Embodied AI Framework for perception

## Where cortex fits

```text
+--------------+ +-------------------+ +-----------+ +------------+
| racing-drone | | interceptor-drone | | isr-drone | | nano-drone | ...  Mission
+--------------+ +-------------------+ +-----------+ +------------+
        |                  |                 |              |
        v                  v                 v              v
+---------------------+ +---------------------+ +---------------------+
|    Branes Cortex    | |   Branes Synapse    | |    Branes Reflex    |  Platform
| perception (VIO,    | | tensor graph engine | | stabilization, MPC, |
| SLAM), VLA, world   | | for GGUF models,    | | state propagation,  |
| model, planning     | | static memory plan  | | fault recovery      |
+---------------------+ +---------------------+ +---------------------+
           |                       |                       |
           v                       v                       v
+---------------------------------+ +---------------------------------+
|              MTL5               | |            Universal            |  Math
| high-performance linear algebra | |       computer arithmetic       |
+---------------------------------+ +---------------------------------+
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
+-----------+ +-------------------+ +-------------------+ +-----------+
|    CPU    | |     CPU + GPU     | |     CPU + KPU     | |    KPU    |  Hardware
+-----------+ +-------------------+ +-------------------+ +-----------+
```

The Branes.AI stack has four tiers:

- **Mission**: one repo per vehicle and mission (`racing-drone`, `interceptor-drone`, `isr-drone`, `nano-drone`, ...). It owns the mission capability and composes platform components; algorithms do not live there.
- **Platform**: three sister repos, `cortex`, `synapse`, and `reflex`, that provide the functionality and services a mission is built from. None of them sits above the others.
- **Math**: [MTL5](https://github.com/stillwater-sc/mtl5) provides high-performance linear algebra and [Universal](https://github.com/stillwater-sc/universal) provides computer arithmetic (posits, intervals, mixed precision, hardware-aligned types).
- **Hardware**: the configurations the stack targets: CPU, CPU + GPU, CPU + KPU, and KPU. Today the stack runs on CPU; GPU and KPU support are planned, and the KPU is a stub until the silicon ships.

Arrows point from a tier to the tier below it that it builds on. They show how the tiers relate, not the dependencies of individual repos: each repo's build declares the libraries it actually uses.

| Repo | Biological analogue | Responsibility |
|---|---|---|
| **`cortex`** (this repo) | Cerebral cortex | Perception (VIO, LIO, SLAM, SfM), vision-language-action models, world building, path planning |
| [`synapse`](https://github.com/branes-ai/synapse) | The synapses: where signals are transmitted and computed | Tensor storage, graph construction, memory planning, and execution of ML models |
| [`reflex`](https://github.com/branes-ai/reflex) | Brainstem, spinal cord, autonomic system | Hard real-time control and safety: stabilization, IMU state propagation, MPC, fault recovery |

## Repository layout

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


