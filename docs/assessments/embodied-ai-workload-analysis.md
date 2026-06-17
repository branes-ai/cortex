# Embodied AI Workload Analysis

You are absolutely right. If we do not capture the exact physical dimensions of the data ingestion and the true arithmetic burden of the modern models (like massive multi-camera VLAs and dense 3D volumetrics), any architectural conclusions we draw for the SoC will be functionally useless.

To map out a true first-principles hardware architecture, we must start at the edge of the system: the sensors. The physical data streams dictate the I/O bus requirements (MIPI CSI-2, Ethernet, PCIe) and the memory bandwidth occupancy before a single mathematical operation even occurs.

Here is the comprehensive, ground-up rebuild of the computational graph, beginning with the sensor payload constraints for each domain.

---

### 1. High-Speed Drone Swarm (200 MPH / 90 m/s)

**The Physics Constraint:** At 90 m/s, a 30 Hz camera misses 3 meters of space between frames. To safely execute Non-linear Model Predictive Control (NMPC) and avoid catastrophic collisions, the perception-to-actuation loop must operate at **500 Hz** (2 ms latency limit).

#### Sensor Configuration & Stream Bandwidth

* **Vision:** 2x Forward-Facing Monochrome Cameras (4K Resolution: 3840x2160, 8-bit Global Shutter) at 500 FPS.
* **Inertial:** 1x 6-DOF IMU at 2,000 Hz.
* **Raw Ingestion Bandwidth:** * Pixels per frame = 16.58 Million.
* Data Rate: $16.58 \text{ MB} \times 500 \text{ Hz} = \mathbf{8.29 \text{ GB/s}}$.
* *Architectural Impact:* This requires a dedicated camera offload engine (like NVIDIA's PVA) directly tied to SRAM; writing 8.29 GB/s continuously to LPDDR will immediately choke standard edge memory buses.



#### Computational Graph Operators

**1.1 Dense Optical Flow (Perception)**

* **Math:** Lucas-Kanade dense pixel tracking, solving $I_x u + I_y v + I_t = 0$.
* **Data Structure:** 16.58 MB input frame $\rightarrow$ 33.1 MB feature gradient matrix (FP16).
* **Arithmetic Precision:** INT8 for pixel diffs, FP16 for gradients.
* **Compute Demand:** $\sim 500$ Ops per pixel. $16.58 \times 10^6 \times 500 = 8.29 \times 10^9 \text{ Ops}$.
* **Throughput Required:** $8.29 \text{ GOPS/frame} \times 500 \text{ Hz} = \mathbf{4.14 \text{ TOPS}}$ (Continuous).

**1.2 Non-Linear MPC (Control)**

* **Math:** Sequential Quadratic Programming (SQP) generating a Karush-Kuhn-Tucker (KKT) linear system for 40 trajectory steps.
* **Data Structure:** Dense KKT Matrix $\approx 1100 \times 1100$ (FP32) = 4.84 MB.
* **Arithmetic Precision:** Strictly **FP32** (Lower precision causes the physical flight model to destabilize and crash).
* **Compute Demand:** $\sim 2.66 \times 10^9 \text{ FLOPs}$ per solve.
* **Throughput Required:** $2.66 \text{ GFLOPS} \times 500 \text{ Hz} = \mathbf{1.33 \text{ TFLOPS (FP32)}}$. *(Note: At a typical 15% sparse-solver efficiency, this demands roughly 9 TFLOPS of peak SoC FP32 silicon).*

---

### 2. ISR Quadruped (Unstructured Terrain Mapping)

**The Physics Constraint:** Legged locomotion over rubble requires 1,000 Hz whole-body control, while spatial understanding requires omnidirectional multi-camera tokenization feeding a massive Vision-Language-Action (VLA) model at 30 Hz.

#### Sensor Configuration & Stream Bandwidth

* **Vision:** 6x RGB Cameras (1920x1080, 8-bit) at 30 FPS for 360-degree coverage.
* **Depth/LiDAR:** 1x 32-Channel 3D LiDAR (600,000 points/sec).
* **Inertial/Joints:** IMU + 12x Joint Encoders at 1,000 Hz.
* **Raw Ingestion Bandwidth:**
* Cameras: $6 \times 1920 \times 1080 \times 3 \text{ bytes} \times 30 \text{ FPS} = \mathbf{1.12 \text{ GB/s}}$.
* LiDAR: $600,000 \times 32 \text{ bytes} = \mathbf{19.2 \text{ MB/s}}$.
* *Architectural Impact:* While the bandwidth is lower than the drone, the *memory capacity* required to hold the generated semantic map is immense.



#### Computational Graph Operators

**2.1 VLA Multi-Camera Spatial Reasoning (Semantic Intelligence)**

* **Math:** Tokenizing 6 camera streams into 16x16 patches, generating a context window $N=6,144$ tokens, processed through a 2 Billion parameter scaled dot-product attention network.
* **Data Structure:** 2B Parameters (FP4/FP8) $\approx 1 \text{ GB}$. KV Cache $\approx 256 \text{ MB}$.
* **Arithmetic Precision:** **FP4 or FP8** for GEMMs; FP16 for softmax activations.
* **Compute Demand:** $2 \times (\text{Parameters}) \times (\text{Tokens}) = 2 \times 2 \times 10^9 \times 6,144 \approx 2.45 \times 10^{13} \text{ Ops}$ (24.5 TOPS per inference).
* **Throughput Required:** $24.5 \text{ TOPS} \times 30 \text{ Hz} = \mathbf{735 \text{ TOPS}}$. *(Factoring in 40% NPU efficiency, this requires an SoC with ~1.8 POPS peak performance, perfectly aligning with NVIDIA Thor's existence).*

**2.2 Dense Truncated Signed Distance Field (TSDF) Mapping**

* **Math:** Raycasting 600,000 LiDAR points and depth maps into a 3D voxel grid to identify physical structural boundaries.
* **Data Structure:** $512 \times 512 \times 256$ Voxel Grid (FP32) = 268 MB.
* **Arithmetic Precision:** **FP32** (Required for precise millimeter-level distance calculation).
* **Throughput Required:** $\mathbf{\sim 30 \text{ GFLOPS (FP32)}}$, heavily bottlenecked by continuous pseudo-random memory read/writes to the 268 MB voxel array.

---

### 3. Heavy Logistics UGV (Human-Centric Safety)

**The Physics Constraint:** Operating a multi-ton vehicle around human personnel requires aerospace-grade redundancy. It must fuse vastly different sensor modalities (Radar, LiDAR, Vision) into a unified Bird's Eye View (BEV) and run massively parallel trajectory predictions at 20-60 Hz.

#### Sensor Configuration & Stream Bandwidth

* **Vision:** 8x 4K RGB Cameras (3840x2160, 24-bit color) at 30 FPS.
* **LiDAR:** 4x 128-Channel Long-Range LiDARs (2.4M points/sec each).
* **Radar:** 6x High-Res Imaging Radars (100k points/sec each).
* **Raw Ingestion Bandwidth:**
* Cameras: $8 \times 8.29 \text{ MB} \times 3 \text{ bytes} \times 30 \text{ FPS} = \mathbf{5.97 \text{ GB/s}}$.
* Point Clouds: $(9.6 \text{M LiDAR} + 600\text{k Radar}) \times 32 \text{ bytes} = \mathbf{326 \text{ MB/s}}$.
* *Architectural Impact:* Over 6.3 GB/s of continuous ingestion via multi-gigabit Ethernet (e.g., 25GbE QSFP ports) and GMSL/CSI-2 lanes.



#### Computational Graph Operators

**3.1 Cross-Attention BEV Sensor Fusion**

* **Math:** Projecting multi-camera imagery and LiDAR/Radar points into a unified latent space via spatial cross-attention Transformers.
* **Data Structure:** Unified BEV Grid $400 \times 400 \times 256$ channels (FP16) = 81.9 MB.
* **Arithmetic Precision:** **INT8 / FP8**.
* **Compute Demand:** $\sim 180 \text{ TOPS}$ per combined frame fusion.
* **Throughput Required:** $180 \text{ TOPS} \times 30 \text{ Hz} = \mathbf{5,400 \text{ TOPS (5.4 POPS)}}$.

**3.2 Massively Parallel MPPI Path Planning (Safety)**

* **Math:** Generating 4,000 parallel candidate paths over an 8-second horizon, evaluated against dynamic actor probabilities.
* **Data Structure:** High-Res Costmap Matrix $1000 \times 1000$ (INT8), Trajectory Tensors $\approx 10 \text{ MB}$.
* **Arithmetic Precision:** **FP32** for vehicle dynamics, INT8 for map evaluation.
* **Throughput Required:** $\mathbf{2.0 \text{ GFLOPS (FP32)}}$, but demands ultra-wide SIMD lanes to compute all 4,000 paths concurrently without thread serialization.

---

### Consolidated First-Principles SoC Requirement Matrix

> **Note on Efficiency Mapping:** To convert "Raw Demand" to "Required SoC Silicon," deep learning operations (INT8/FP8/FP4) are scaled at ~40% efficiency, and dense/sparse physics solvers (FP32) are scaled at ~15% efficiency due to memory stalls and branch divergence.

| Mission Profile | Stream Ingestion (Raw Bandwidth) | Core Algorithmic Graph Step | Target Freq. | Data Precision | Raw Computation Demand | Implied SoC Silicon Peak Requirement |
| --- | --- | --- | --- | --- | --- | --- |
| **1. High-Speed Drone** | **8.29 GB/s** | Dense Optical Flow | 500 Hz | INT8/FP16 | 4.14 TOPS | **~10.3 TOPS** |
|  |  | KKT NMPC Solver | 500 Hz | FP32 | 1.33 TFLOPS | **~8.8 TFLOPS** |
| **2. ISR Quadruped** | **1.14 GB/s** | VLA Tokenization & Reasoning | 30 Hz | FP4/FP8 | 735 TOPS | **~1,837 TOPS (1.8 POPS)** |
|  |  | Dense TSDF Volumetric Mapping | 30 Hz | FP32 | 30 GFLOPS | **~200 GFLOPS** |
| **3. Heavy Logistics UGV** | **6.30 GB/s** | Cross-Attention Sensor Fusion | 30 Hz | INT8/FP8 | 5,400 TOPS | **~13,500 TOPS (13.5 POPS)** |
|  |  | Parallel MPPI Path Evaluator | 20 Hz | FP32 | 2.0 GFLOPS | **~13.3 GFLOPS** |

With the true math and physical sensor payloads exposed, the constraints of the SoC design are clear: you cannot just build a giant NPU for AI; you need high-bandwidth PHYs to ingest gigabytes of raw sensor data and massively powerful FP32 cores to handle the rigid-body physics.

Given this end-to-end data pipeline from raw ingestion to physical math, should we next define the specific on-chip memory hierarchy (L1/L2/L3 cache sizes) required to prevent these specific data structures from constantly spilling out to main system memory?

## Memory Hierarchies

Let's design some on-chip memory hierarchies, but to make that effective, we need to separate the different micro-architectures. Let's do a cache hierarchy for a multi-core CPU, or many-core GPUs with L2s that can share among SMs/CUs, Jetson-style architecture with custom compute tiles dedicated for specific functionality, such as, PVA, multi-core, DLAs, and GPU, and a KPU-style architecture that has designated software-managed scratch pads and system level schedules to move data around as the whole KPU is a push architecture that lacks any request/reply bus infrastructure to a global memory.

To evaluate and plan a custom SoC accelerator, we must map our workloads against four distinct on-chip memory and execution paradigms. Each architecture handles data movement differently—shifting the burden between hardware-managed prediction, shared silicon pools, heterogeneous offloading, and deterministic software-driven dataflow.

---

### Architecture A: Multi-Core CPU Cache Hierarchy

* **Mechanics:** Hardware-managed, reactive request/reply system. Features private L1 (Instruction/Data), private or shared L2, and a massive shared L3 Last-Level Cache (LLC). Relies on hardware prefetchers and directory-based cache coherency (e.g., MESI).

```
+-------------------------------------------------+
|               Shared L3 LLC (16MB - 32MB)       |
+-------------------------------------------------+
        ^                                 ^
        v                                 v
+-----------------+               +-----------------+
|    L2 Cache     |               |    L2 Cache     |
+-----------------+               +-----------------+
        ^                                 ^
        v                                 v
+-----------------+               +-----------------+
| L1D/L1I (Core0) |               | L1D/L1I (Core1) |
+-----------------+               +-----------------+

```

#### Workload Mapping & Bottlenecks

* **The Control/Physics Fit:** The **Drone's NMPC KKT Solver (4.84 MB)** and the **Quadruped's WBC Matrix (15 KB)** fit perfectly inside an 8MB–16MB L3 or L2 cache hierarchy. Because these solvers feature highly branched, non-vectorized serial loops, the hardware prefetchers easily hide memory latency.
* **The Sensor Stream Disaster:** Ingesting the **UGV's 5.97 GB/s camera stream** or the **Drone's 8.29 GB/s frame stream** directly through a CPU cache hierarchy results in total cache thrashing. The streaming video data lacks temporal reuse, meaning it completely evicts the vital control loop matrices from the L2/L3 cache, forcing costly round-trips to LPDDR.

---

### Architecture B: Many-Core GPU Hierarchy

* **Mechanics:** High-throughput, highly parallel request/reply system. Thousands of simple ALUs grouped into Streaming Multiprocessors (SMs) or Compute Units (CUs). Each SM features a private L1/Texture Cache and a configurable *Shared Memory* (hardware-accessible local scratchpad). All SMs route through a massive crossbar to a wide, shared L2 cache.

```
+-------------------------------------------------+
|               Shared L2 Cache (4MB - 16MB)      |
+-------------------------------------------------+
        ^                                 ^
        v                                 v
+-----------------+               +-----------------+
| SM0 (L1 / ShMem)|               | SM1 (L1 / ShMem)|
+-----------------+               +-----------------+

```

#### Workload Mapping & Bottlenecks

* **The Fusion Fit:** The **UGV's Cross-Attention Fusion (81.9 MB BEV Grid)** and the **Drone's Optical Flow (33.1 MB Gradient Matrix)** scale flawlessly across thousands of threads.
* **The Memory Tiling Constraint:** Because an 81.9 MB or 33.1 MB data structure exceeds a standard GPU L2 cache size (typically 4MB–16MB on edge SoCs), the workload *must be spatial-tiled*. The software must chunk the BEV grid or image into micro-tiles (e.g., $32 \times 32$ pixels) that fit into the SM's **128 KB Shared Memory/L1**. Threads process a tile, accumulate features locally, and write back to L2, preventing continuous LPDDR cache line misses.
* **The Latency Trap:** Attempting to run the **Quadruped's 1,000 Hz Whole Body Control** on this architecture introduces severe latency penalties. The GPU's deep execution pipelines and request/reply scheduling overhead fail when thread counts are low, causing the SMs to stall while waiting for single-element matrix inversions.

---

### Architecture C: Jetson-Style Heterogeneous Architecture

* **Mechanics:** Domain-Specific Acceleration (DSA). Computing is split into hardware tiles connected via an on-chip coherent Interconnect (Network-on-Chip) to a System MMU and a shared LPDDR pool.
* **PVA (Programmable Vision Accelerator):** Vector DSPs with local high-throughput SRAM for vision loops.
* **DLA (Deep Learning Accelerator):** Fixed-function tensor pipes optimized for static CNN/Transformer layers.
* **GPU/CPU:** General purpose parallel and serial engines.



```
+-------------------------------------------------------------+
|                 Coherent System Interconnect                |
+-------------------------------------------------------------+
   ^                 ^                 ^                 ^
   v                 v                 v                 v
+-----+           +-----+           +-----+           +-----+
| CPU |           | GPU |           | DLA |           | PVA |
+-----+           +-----+           +-----+           +-----+

```

#### Workload Mapping & Bottlenecks

* **The Pipeline Fit:** This architecture excels at managing complex pipelines like the **Quadruped**. The **PVA** ingests the raw 1.14 GB/s camera streams and downsamples them entirely within its local SRAM. The downsampled tokens are handed directly to the **DLA** to execute the heavy 735 TOPS VLA layers. Simultaneously, the **CPU** runs the 1,000 Hz WBC loop, completely isolated from the vision pipeline's memory churn.
* **The Inter-Tile Bottleneck:** The critical failure point is data handoff. If the PVA finishes processing an image and writes it back to LPDDR, and then the DLA must pull it back into its own local buffers, the system wastes massive bus bandwidth. The SoC must support **Zero-Copy unified memory mapping** and hardware-enforced cache snooping across the CPU/GPU/DLA boundaries to handle these pipelines effectively.

---

### Architecture D: KPU-Style Dataflow (Push Architecture)

* **Mechanics:** Completely deterministic, software-defined streaming architecture. There are **no hardware caches, no request/reply buses, and no hardware arbiters**. Each compute cluster contains its own software-managed local SRAM scratchpad. Data movement is treated as a continuous, unidirectional pipeline: a global system scheduler pushes data from an input engine, through a chain of compute tiles, directly to the output actuators via a deterministic, pipelined Network-on-Chip (NoC).

```
[Sensor Ingest] -> [Tile 0: SRAM] -> (Compute) -> [Tile 1: SRAM] -> (Compute) -> [Actuator Out]
                         ^                              ^
                         +------- (Software Scheduled) -+

```

#### Workload Mapping & Bottlenecks

* **The Ultra-Low Latency Fit:** This is the ideal architecture for the **200 MPH High-Speed Drone Swarm**. The 500 Hz control loop is mapped statically to the silicon. The 8.29 GB/s input camera data is streamed into Tile 0's scratchpad. As soon as a row of pixels is ready, it is *pushed* into the Optical Flow execution unit, which pushes gradients directly into Tile 1's scratchpad (Factor Graph), which immediately streams into the NMPC solver. Latency drops from milliseconds to microseconds because there are no cache-miss stalls or bus-arbitration delays.
* **The VLA Memory Choke:** This architecture struggles severely with large models like the **Quadruped's 1 GB VLA** or the **UGV's 160 MB Behavior Predictor**. Because there is no request/reply global memory infrastructure to load data dynamically on demand, the software scheduler must perform massive **double-buffered (ping-pong) DMA streams**. While Compute Engine A is calculating using Weight Block $N$ inside Scratchpad A, the global scheduler must be actively *pushing* Weight Block $N+1$ into Scratchpad B. If the model's structure features complex, non-linear skip connections, scheduling the data routing manually in software becomes an incredibly complex mathematical optimization challenge.

---

### Hardware Architecture Comparison Matrix

| Micro-Architecture Paradigm | Memory Handling Strategy | Ideal Workload Match | Worst-Case Failure Mode | On-Chip Memory Budget (SRAM) for Target Missions |
| --- | --- | --- | --- | --- |
| **A: Multi-Core CPU** | Reactive / Cache Coherent (Hardware Managed) | 500 Hz NMPC, 1000 Hz WBC (Serial Dynamics) | Camera stream ingestion causes total cache thrashing | Requires **16MB - 32MB L3 LLC** to isolate physics matrices from OS overhead. |
| **B: Many-Core GPU** | Parallel / Hardware L2 + Software Tiled L1 | 81.9 MB BEV Fusion, MPPI Path Planning | High-frequency low-parallelism loops stall the execution pipeline | Requires **8MB - 16MB L2** + **128KB Shared Memory per SM** for local spatial tiling. |
| **C: Jetson-Style DSA** | Heterogeneous Interconnect / Unified Memory | Complex multi-rate pipelines (e.g., ISR Quadruped) | High inter-tile handoff latency if zero-copy is mismanaged | Requires **4MB per PVA**, **8MB per DLA**, plus shared cache across CPU/GPU blocks. |
| **D: KPU-Style Dataflow** | Software-Managed Scratchpads / Unidirectional Push | Ultra-low latency, deterministic flight control (Drone) | Large weights (1GB+ VLA) require complex, multi-stage manual tiling | Requires massive distributed SRAM layout (**64MB - 128MB total scratchpad pool**) to handle ping-pong DMA buffers. |

How should we proceed with this architectural blueprint? We can explore sizing the specific compiler-driven DMA schedules for the KPU dataflow approach to manage the 160 MB behavior model, or we can focus on mapping the precise interconnect bandwidth needed to prevent inter-tile bottlenecks in the heterogeneous Jetson-style design.

## Workload Refinement

We need to further refine and embellish the workloads. We'll have GPS-denied deployments for all three, either deployments inside a warehouse or building or in war zones. This means we'll need VIO, LIO, SLAM, and multi-sensor versions including Radar and Sonar, and for accuracy in an urban environment we will have visual positioning. Please add these algorithms to the set and quantify their memory size, compute load, and loop frequency.

You are absolutely right to introduce this constraint. In modern deployments—whether deep inside a concrete warehouse or in a contested Electronic Warfare (EW) environment—GPS/GNSS signals are either degraded, jammed, or simply non-existent. Moving from a simple GPS serial UART read to fully autonomous GPS-denied localization introduces a massive layer of mathematical complexity and memory churn.

To solve this, the systems must build and maintain their own local reference frames while simultaneously aligning themselves to prior global maps (Visual Positioning System, or VPS) without satellite aid.

Here is the first-principles breakdown of the additional algorithms required to guarantee millimeter-accurate, GPS-denied localization.

---

### 1. High-Speed Drone Swarm (High-Frequency VIO & VPS)

A drone at 90 m/s cannot wait for dense 3D maps to construct; it needs immediate, tightly-coupled state estimation to prevent drifting into the ground, coupled with periodic global map matching to correct accumulated drift.

#### 1.4 Tightly-Coupled Visual-Inertial Odometry (VIO)

* **Math Formulation:** Minimizing a joint cost function of IMU pre-integration errors and visual reprojection errors over a sliding temporal window using non-linear least squares.

$$\min_{X} \left\{ \| r_p - H_p X \|^2 + \sum_{k} \| r_{I}(\hat{z}_{I_k}, X) \|^2_{P_I} + \sum_{c} \| r_{C}(\hat{z}_{C_c}, X) \|^2_{P_C} \right\}$$


* **Data Structures & Memory Footprint:**
* State covariance matrix (sliding window of 10-20 frames): **~1.5 MB** (FP32).
* Visual feature tracker buffers (FAST/ORB corners): **~2 MB**.


* **Loop Frequency:** 100 Hz vision updates (synchronizing with the 500 Hz IMU/Control loop).
* **Compute Demand:** Solving the Schur complement for marginalization and the Cholesky factorization. Requires **~15 GFLOPS (FP32)** per cycle.

#### 1.5 Visual Positioning System (Global Loop Closure)

* **Math Formulation:** Extracting high-dimensional feature descriptors from the current camera frame and querying a massive pre-computed global 3D map database (e.g., using NetVLAD or SuperPoint/SuperGlue algorithms).
* **Data Structures & Memory Footprint:**
* Global Map Descriptor Database: **500 MB to 2 GB** (Streamed from NVMe to LPDDR).
* Local matching tensors: **~35 MB** (INT8/FP16).


* **Loop Frequency:** 2 to 5 Hz (Only needed periodically to correct VIO drift).
* **Compute Demand:** Transformer-based feature matching requires **~40 TOPS (INT8/FP16)**.

---

### 2. ISR Quadruped (LIO & Indoor Semantic SLAM)

Indoors or in rubble, optical features degrade due to poor lighting, dust, or blank walls. The quadruped must rely heavily on LiDAR-Inertial Odometry (LIO) for robust short-term estimation, backed by a global Pose Graph for long-term SLAM consistency.

#### 2.4 Iterated LiDAR-Inertial Odometry (LIO)

* **Math Formulation:** Propagating system state via IMU and correcting it using an Iterated Error-State Kalman Filter (IESKF) matched against local LiDAR geometric features (planes and edges).

$$K = P H^T (H P H^T + R)^{-1}$$


* **Data Structures & Memory Footprint:**
* Incremental Voxel Map (iVox) / KD-Tree: **~85 MB** (Dynamically growing, FP32).
* Point Cloud Buffers: **~4 MB**.


* **Loop Frequency:** 50 Hz (Processing LiDAR sweeps).
* **Compute Demand:** Nearest-neighbor search and covariance matrix inversion. Requires **~8 GFLOPS (FP32)** per sweep.

#### 2.5 Pose Graph Optimization (PGO SLAM Backend)

* **Math Formulation:** Maintaining the history of the robot's trajectory and creating "loop closures" when the robot recognizes a previously visited location. It minimizes the Mahalanobis distance of the pose errors:

$$\arg \min_{x} \sum_{i,j} \| f(x_i, x_j) - z_{ij} \|_{\Sigma}^2$$


* **Data Structures & Memory Footprint:**
* Sparse Adjacency Matrix (Pose Graph): **~20 MB to 150 MB** (Grows linearly with mission time, FP32).


* **Loop Frequency:** 1 Hz (Runs asynchronously in the background).
* **Compute Demand:** Large-scale sparse matrix factorization (Levenberg-Marquardt). Highly bursty, requiring **~5 GFLOPS to 20 GFLOPS (FP32)** depending on graph size.

---

### 3. Heavy Logistics UGV (Multi-Modal SLAM & Urban VPS)

In urban canyons or war zones, a UGV faces dynamic obstacles, dust, smoke, and degraded maps. It requires absolute sensor redundancy—fusing Radar, LiDAR, Vision, Sonar, and Wheel Encoders into a single monolithic state estimator.

#### 3.4 Multi-Modal Odometry (Radar + Lidar + Sonar + IMU)

* **Math Formulation:** A highly dimensional Extended Kalman Filter (EKF) or Factor Graph that dynamically weights sensor trust based on environmental noise (e.g., heavily weighting Radar in smoke, weighting LiDAR in clear weather). Sonar is added for ultra-close (0.1m - 3m) blind-spot collision prevention.
* **Data Structures & Memory Footprint:**
* Multi-Sensor Covariance Matrices: **~12 MB** (FP32).
* Radar Cross-Section (RCS) Point Clouds + Sonar Echo Tensors: **~8 MB**.


* **Loop Frequency:** 60 Hz (Synchronized to the primary perception pipeline).
* **Compute Demand:** Joint optimization and continuous EKF updates across modalities. Requires **~25 GFLOPS (FP32)**.

#### 3.5 Urban Visual-LiDAR Positioning System (HD Map Localization)

* **Math Formulation:** Aligning the real-time fused BEV (Bird's Eye View) grid generated in Step 3.1 against a city-scale High-Definition (HD) Map containing semantic lane markers, traffic signs, and structural contours.
* **Data Structures & Memory Footprint:**
* Local HD Map Cache: **~250 MB** (INT8/FP16 map tiles streamed dynamically based on rough position).


* **Loop Frequency:** 10 Hz.
* **Compute Demand:** Dense tensor matching and Cross-Attention between live BEV grids and the HD Map. Requires **~120 TOPS (INT8/FP8)**.

---

### Consolidated GPS-Denied Workload Matrix

By adding these localization constraints, the hardware must now support massive database streaming (for VPS) and continuous matrix inversion (for LIO/VIO), pushing both the memory buses and the FP32 arithmetic pipelines to their absolute limits.

| Mission Profile | GPS-Denied Navigation Algorithm | Target Freq. | Dominant Arithmetic & Memory Cost | Raw Compute Demand | Implied SoC Silicon Req. (Peak) |
| --- | --- | --- | --- | --- | --- |
| **1. Drone Swarm** | 1.4 Tightly-Coupled VIO | 100 Hz | FP32 (3.5 MB cache) | 15.0 GFLOPS | **~100 GFLOPS (FP32)** |
|  | 1.5 Map-based VPS | 5 Hz | INT8/FP16 (500 MB streamed) | 40.0 TOPS | **~100 TOPS (INT8)** |
| **2. Quadruped** | 2.4 Iterated LIO | 50 Hz | FP32 (85 MB KD-Tree) | 8.0 GFLOPS | **~55 GFLOPS (FP32)** |
|  | 2.5 SLAM Pose Graph | 1 Hz | FP32 (150+ MB Graph) | 20.0 GFLOPS (Bursty) | **~133 GFLOPS (FP32)** |
| **3. Heavy UGV** | 3.4 Multi-Modal Odometry | 60 Hz | FP32 (20 MB Matrices) | 25.0 GFLOPS | **~166 GFLOPS (FP32)** |
|  | 3.5 Urban HD Map VPS | 10 Hz | INT8/FP8 (250 MB Tile Cache) | 120.0 TOPS | **~300 TOPS (INT8)** |

### Per Mission Profile

The tables below compile the first-principles computational graph and ingestion requirements for each of the three mission profiles.

To ensure architectural fidelity, the **Required SoC Peak Performance** column scales the raw algorithmic throughput by applying realistic hardware utilization factors based on workload characteristics: **40–45% efficiency for highly parallel Deep Learning / GEMM blocks**, and **15% efficiency for highly divergent, sequence-dependent Classical Control / Sparse Linear Algebra solvers**.

---

### 1. High-Speed Drone Swarm (200 MPH / 90 m/s)

* **Primary Mission Constraint:** Safety-critical 2 ms epoch budget to maintain less than 20 cm of blind physical displacement between control iterations.

| Algorithmic Step / Data Stream | Target Freq. | Memory Footprint / Data Structure Size | Arithmetic Precision | Raw Compute Demand | Required SoC Peak Performance | Primary Hardware Bottleneck & Dependency |
| --- | --- | --- | --- | --- | --- | --- |
| **Sensor Ingestion Stream** | 500 Hz | 16.58 MB / frame buffer | *N/A (I/O)* | *8.29 GB/s ingress* | **8.29 GB/s ingress** | Dedicated ISP & MIPI CSI-2 PHY line capture |
| **1.1 Dense Optical Flow** | 500 Hz | 33.10 MB (Gradient Matrix) | INT8 / FP16 | 4.14 TOPS | **10.35 TOPS** | Matrix Vector Units (SRAM to ALU bandwidth) |
| **1.2 Factor Graph Opt.** | 500 Hz | 0.36 MB (Info Matrix) | FP32 | 4.60 GFLOPS | **30.66 GFLOPS** | Low-latency serial cache / Vector SIMD registers |
| **1.3 Non-Linear MPC** | 500 Hz | 4.84 MB (KKT Matrix) | FP32 | 1.33 TFLOPS | **8.87 TFLOPS** | High TFLOPS dense FP32 execution blocks |
| **1.4 Tightly-Coupled VIO** | 100 Hz | 3.50 MB (Covariance & Features) | FP32 | 15 GFLOPS | **100 GFLOPS** | Sparse linear algebra / hardware prefetch efficiency |
| **1.5 Map-Based VPS** | 5 Hz | 500 MB – 2.00 GB (Global Map) | INT8 / FP16 | 40 TOPS | **100 TOPS** | LPDDR bandwidth (streaming descriptors from flash) |

---

### 2. ISR Quadruped (Unstructured Terrain & Indoor Mapping)

* **Primary Mission Constraint:** Multi-rate execution split between high-frequency mechanical stabilization (1,000 Hz) and dense, multi-camera semantic perception (30 Hz).

| Algorithmic Step / Data Stream | Target Freq. | Memory Footprint / Data Structure Size | Arithmetic Precision | Raw Compute Demand | Required SoC Peak Performance | Primary Hardware Bottleneck & Dependency |
| --- | --- | --- | --- | --- | --- | --- |
| **Sensor Ingestion Stream** | 30 Hz | 6.22 MB / frame block | *N/A (I/O)* | *1.14 GB/s ingress* | **1.14 GB/s ingress** | Multi-channel MIPI/Ethernet deserialization |
| **2.1 VLA Spatial Reasoning** | 30 Hz | 1.00 GB (Weights) + 256 MB (KV Cache) | FP4 / FP8 / FP16 | 735 TOPS | **1,837.50 TOPS** | Extreme LPDDR streaming bandwidth (Weight-bound) |
| **2.2 Dense TSDF Mapping** | 30 Hz | 268.00 MB (3D Voxel Grid) | FP32 | 30 GFLOPS | **200 GFLOPS** | Unstructured memory access (Cache miss penalties) |
| **2.3 Whole-Body Control** | 1000 Hz | 0.015 MB (Rigid Body Model) | FP32 | 0.45 GFLOPS | **3 GFLOPS** | Core-to-core synchronization & I/O latency |
| **2.4 Iterated LIO** | 50 Hz | 85.00 MB (Incremental KD-Tree) | FP32 | 8 GFLOPS | **53.33 GFLOPS** | Point-cloud search latency / hardware tree-traversal |
| **2.5 SLAM Pose Graph** | 1 Hz | 150.00 MB (Global Adjacency Matrix) | FP32 | 20 GFLOPS | **133.33 GFLOPS** | Highly bursty sparse matrix factorization logic |

---

### 3. Heavy Logistics UGV (Human-Centric Safe Operations)

* **Primary Mission Constraint:** Massive multi-modal sensory ingestion with high-throughput spatial cross-attention transformers running concurrently with deterministic trajectory safety checking.

| Algorithmic Step / Data Stream | Target Freq. | Memory Footprint / Data Structure Size | Arithmetic Precision | Raw Compute Demand | Required SoC Peak Performance | Primary Hardware Bottleneck & Dependency |
| --- | --- | --- | --- | --- | --- | --- |
| **Sensor Ingestion Stream** | 30-60 Hz | 205.00 MB / sensor buffer pool | *N/A (I/O)* | *6.30 GB/s ingress* | **6.30 GB/s ingress** | PCIe Gen4 / 25GbE infrastructure pipelines |
| **3.1 Cross-Attention Fusion** | 30 Hz | 122.42 MB (Input) + 81.90 MB (BEV Grid) | INT8 / FP8 / FP16 | 5,400 TOPS | **13,500 TOPS** | Heavy NPU execution / On-chip SRAM size constraints |
| **3.2 Behavior Prediction** | 20 Hz | 160.00 MB (Transformer Weights) | FP16 / BF16 | 240 GFLOPS | **533.33 GFLOPS** | Autoregressive decoding / Tensor Core efficiency |
| **3.3 Parallel MPPI Planner** | 20 Hz | 10.60 MB (Costmaps & Tensors) | FP32 / INT8 | 2 GFLOPS | **13.33 GFLOPS** | Massively parallel SIMD thread execution limits |
| **3.4 Multi-Modal Odometry** | 60 Hz | 20.00 MB (Sensor Covariances) | FP32 | 25 GFLOPS | **166.66 GFLOPS** | EKF state propagation / Deterministic timing loops |
| **3.5 Urban HD Map VPS** | 10 Hz | 250.00 MB (Local Map Tile Cache) | INT8 / FP8 | 120 TOPS | **300 TOPS** | Spatial cross-attention crossbar throughput |

## Operand Bandwidth

To isolate the true data delivery challenge of your SoC, analyzing the **raw operand bandwidth** (the internal dataflow required directly at the execution unit or register-file-to-ALU level) is an exceptional approach.

By ignoring caching, tiling, and data reuse algorithms, we can expose the unmitigated, absolute volume of data the execution units must consume and produce every single second.

### The First-Principles Operand Bandwidth Equation

A standard hardware compute operation (like a Fused Multiply-Accumulate, or MAC) counts as **2 Operations** (1 multiply, 1 add). A standard MAC requires **3 inputs** read from memory/registers ($A$, $B$, and $C$) and yields **1 output** written back ($D = A \times B + C$).

Therefore, for every 2 OPs executed, the system must move 4 operands. The raw operand bandwidth is governed strictly by the arithmetic precision used:

* **FP32 (4 bytes/operand):** 4 operands $\times$ 4 bytes / 2 OPs = **6.0 Bytes per FLOP**
* **FP16 / BF16 (2 bytes/operand):** 4 operands $\times$ 2 bytes / 2 OPs = **3.0 Bytes per FLOP**
* **INT8 / FP8 (1 byte/operand):** 4 operands $\times$ 1 byte / 2 OPs = **1.5 Bytes per OP**
* **FP4 (0.5 bytes/operand):** 4 operands $\times$ 0.5 bytes / 2 OPs = **0.75 Bytes per OP**

Multiplying the **Raw Compute Demand** by these precision coefficients yields the un-cached operand bandwidth required to keep the ALUs fully fed.

---

### 1. High-Speed Drone Swarm (200 MPH / 90 m/s)

| Algorithmic Step / Data Stream | Target Freq. | Raw Compute Demand | Arithmetic Precision | Raw Operand Bandwidth | Required SoC Peak Performance | Primary Hardware Bottleneck |
| --- | --- | --- | --- | --- | --- | --- |
| **Sensor Ingestion Stream** | 500 Hz | *8.29 GB/s ingress* | *N/A (I/O)* | **8.29 GB/s** | 8.29 GB/s ingress | Dedicated ISP / PHY line capture |
| **1.1 Dense Optical Flow** | 500 Hz | 4.14 TOPS | INT8 / FP16 | **6.21 TB/s** | 10.35 TOPS | Matrix Vector Unit RF-to-ALU bus |
| **1.2 Factor Graph Opt.** | 500 Hz | 4.60 GFLOPS | FP32 | **27.60 GB/s** | 30.66 GFLOPS | Vector SIMD register file width |
| **1.3 Non-Linear MPC** | 500 Hz | 1.33 TFLOPS | FP32 | **7.98 TB/s** | 8.87 TFLOPS | High TFLOPS dense FP32 execution blocks |
| **1.4 Tightly-Coupled VIO** | 100 Hz | 15 GFLOPS | FP32 | **90 GB/s** | 100 GFLOPS | Sparse linear algebra memory stalls |
| **1.5 Map-Based VPS** | 5 Hz | 40 TOPS | INT8 / FP16 | **60 TB/s** | 100 TOPS | LPDDR streaming to Tensor pipelines |

---

### 2. ISR Quadruped (Unstructured Terrain & Indoor Mapping)

| Algorithmic Step / Data Stream | Target Freq. | Raw Compute Demand | Arithmetic Precision | Raw Operand Bandwidth | Required SoC Peak Performance | Primary Hardware Bottleneck |
| --- | --- | --- | --- | --- | --- | --- |
| **Sensor Ingestion Stream** | 30 Hz | *1.14 GB/s ingress* | *N/A (I/O)* | **1.14 GB/s** | 1.14 GB/s ingress | Multi-channel MIPI deserialization |
| **2.1 VLA Spatial Reasoning** | 30 Hz | 735.00 TOPS | FP4 / FP8 | **735.00 TB/s** | 1,837.50 TOPS | Tensor Core weight-streaming bus |
| **2.2 Dense TSDF Mapping** | 30 Hz | 30 GFLOPS | FP32 | **180 GB/s** | 200 GFLOPS | Pseudo-random 3D memory crossbar |
| **2.3 Whole-Body Control** | 1000 Hz | 0.45 GFLOPS | FP32 | **2.70 GB/s** | 3 GFLOPS | Core-to-core I/O latency limits |
| **2.4 Iterated LIO** | 50 Hz | 8 GFLOPS | FP32 | **48 GB/s** | 53.33 GFLOPS | Point-cloud tree-traversal logic |
| **2.5 SLAM Pose Graph** | 1 Hz | 20 GFLOPS | FP32 | **120 GB/s** | 133.33 GFLOPS | Bursty sparse matrix solver lanes |

*Note: For step 2.1, an average coefficient of 1.0 Byte/OP is utilized to represent the mixed FP4/FP8 transformer execution footprint.*

---

### 3. Heavy Logistics UGV (Human-Centric Safe Operations)

| Algorithmic Step / Data Stream | Target Freq. | Raw Compute Demand | Arithmetic Precision | Raw Operand Bandwidth | Required SoC Peak Performance | Primary Hardware Bottleneck |
| --- | --- | --- | --- | --- | --- | --- |
| **Sensor Ingestion Stream** | 30-60 Hz | *6.30 GB/s ingress* | *N/A (I/O)* | **6.30 GB/s** | 6.30 GB/s ingress | PCIe Gen4 / 25GbE infrastructure |
| **3.1 Cross-Attention Fusion** | 30 Hz | 5,400 TOPS | INT8 / FP8 | **8,100 TB/s** | 13,500 TOPS | NPU cluster interconnect network |
| **3.2 Behavior Prediction** | 20 Hz | 240 GFLOPS | FP16 / BF16 | **720 GB/s** | 533.33 GFLOPS | Autoregressive decoding lanes |
| **3.3 Parallel MPPI Planner** | 20 Hz | 2 GFLOPS | FP32 | **12 GB/s** | 13.33 GFLOPS | Massively parallel SIMD execution |
| **3.4 Multi-Modal Odometry** | 60 Hz | 25 GFLOPS | FP32 | **150 GB/s** | 166.66 GFLOPS | EKF state propagation pipelines |
| **3.5 Urban HD Map VPS** | 10 Hz | 120 TOPS | INT8 / FP8 | **180 TB/s** | 300 TOPS | Spatial cross-attention crossbars |

---

### Architectural Takeaways for Your SoC Design

1. **The Terabyte vs. Petabyte Scale Split:** Looking at the raw numbers, neural workloads like **UGV Cross-Attention Fusion** demand an unmitigated **8.1 Petabytes per second (PB/s)** of internal operand delivery. This proves mathematically that your NPU tiles *must* feature massive internal register files and tightly clamped L1 SRAM nodes. If even 0.1% of that operand demand spills out to global LPDDR memory, the chip will instantly choke.
2. **The High-Precision FP32 Threat:** While the drone's **Non-Linear MPC** requires only 1.33 TFLOPS of compute, its raw operand delivery requirement sits at a staggering **7.98 TB/s**. Because this math cannot be compressed into lower precisions without crashing the physical drone, your SoC design cannot skimp on the routing crossbars or register-file bandwidth attached to your FP32 vector lanes.


Now that we have the complete, mathematically grounded computational graph—including raw sensor ingestion, semantic perception, high-frequency physics control, and GPS-denied localization—how would you like to proceed with mapping these consolidated demands onto the four specific micro-architecture paradigms (CPU, GPU, DSA, KPU) we discussed previously?


