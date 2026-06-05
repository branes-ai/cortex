# 3D visualization environment — requirement analysis & tool selection

A 3D environment to visualize drone **paths and pose** for **educational videos**
and **dissemination**. This records the requirement, what our data already
supports, the options we evaluated, and the direction we selected.

- Related: `docs/assessments/vio-pipeline-howto.md` (the 2D scene-video demo this
  extends), `docs/arch/vio-pipeline-canonical.md` (the #212 over-confidence story
  the uncertainty view supports).

---

## 1. The requirement, decomposed

"Visualize drone paths and pose for educational content and dissemination" is two
delivery modes that pull toward different tools:

1. **Offline rendered video (mp4)** — for talks, YouTube, embedding in docs.
   Must be scriptable, reproducible, and headless/CI-friendly (our current ethos:
   data → generator → ffmpeg).
2. **Interactive dissemination** — a viewer the audience can orbit and scrub,
   ideally embedded directly in the Astro/Starlight docs-site
   (`branes-ai.github.io/cortex/`).

The 3D scene needs: **GT + estimated trajectory** (two 3-D curves), the **current
6-DOF pose** (a moving body axis-triad / drone glyph), the **3-D landmarks**
(point cloud), a **camera frustum**, and — for the #212 story — a **position
covariance ellipsoid**.

---

## 2. What our data already supports

Most of the scene is already drivable from the pipeline's existing streams:

| Need | Source | Status |
|---|---|---|
| GT + est 6-DOF pose per frame | `run.jsonl` (`p[x,y,z]` + `q[w,x,y,z]`) | ready |
| Trajectory curves | `trajectory.csv` | ready |
| 3-D landmarks | `frames.jsonl` (`obs`/`true` px + `depth`) + known cam0 intrinsics → back-project | derivable |
| Camera frustum | intrinsics hardcoded (fx 458.654, fy 457.296, cx 367.215, cy 248.375) | derivable |
| **Position covariance ellipsoid** | not emitted (NIS/NEES are scalars) | **gap** |

**The one gap — covariance.** The backend has the per-state covariance; the demo
only dumps scalar NIS/NEES. Emitting the 3×3 position block per frame (a small
addition to `tools/src/vio_pipeline.cpp`) unlocks the uncertainty ellipsoid. This
is worth doing regardless of tool: an ellipsoid that is *too small* while the true
pose drifts outside it is the single most compelling visual for the #212
over-confidence work.

---

## 3. Options evaluated

| Tool | Web-embed | Video export | Robotics 3D primitives | Stack fit | Effort | License |
|---|---|---|---|---|---|---|
| **Three.js** (in Astro) | native islands | headless-Chrome frame capture → ffmpeg | build yourself (axes/lines/sprites trivial) | pure Node, reads `run.jsonl`, reuses ffmpeg | Medium | MIT |
| **Rerun** (rerun.io) | embeddable web viewer (.rrd) | viewer screen-record (not polished offline) | poses, point clouds, pinhole/frustum, ellipsoids, time-scrub — native | **C++ & Rust SDKs** — instrument `vio_pipeline.cpp` / RM directly | Low | Apache-2.0 |
| Open3D / PyVista | — | offscreen → PNG → ffmpeg, reproducible | line sets, frames, clouds | adds Python; headless GL on CI is painful (EGL/xvfb) | Medium | MIT/BSD |
| Blender (`bpy` headless) | — | top production quality, `blender -b -P` | manual scene build | heavy; separate render step | High | GPL (external renderer only) |
| Foxglove + MCAP | iframe | screen-record | strong | nudges toward MCAP; inspection-oriented | Low | MPL-2.0 |
| RViz2 (ROS 2) | — | screen-record | strong | pulls ROS, Linux-only, against our layering | Low | BSD |
| Plotly / matplotlib-3D | Plotly only | weak | weak | OK | Low | BSD |

---

## 4. Decision — a two-tier split

The two purposes are better served by two complementary tools than one compromise.

### Tier 1 — Three.js in the docs-site = the publication / dissemination layer
The only option that serves *both* delivery modes from one codebase and one data
format: interactive orbitable demos as Astro islands (nothing like it exists in
our docs today) **and** offline mp4 via Playwright headless-Chrome frame capture
into the ffmpeg backend we already drive. Pure Node, MIT, cross-platform, reads
`run.jsonl` directly. Strategic fit with the existing Node → ffmpeg pipeline.

### Tier 2 — Rerun = the internal / debug + technical-talk layer
The robotics-native microscope: instrument `vio_pipeline.cpp` with its C++ SDK
(and the Rust RM with its Rust SDK) for free time-scrubbing of poses, landmarks,
frustums, and covariance ellipsoids — directly accelerating the #212 consistency
hunt, not just producing videos. Lowest effort to stand up; its video output is
screen-capture-grade, so it is the engineer's tool, not the publish tool.

### Reserve — Blender
For a small number of hero explainer videos where production polish matters, used
purely as an external renderer (GPL is a non-issue there — separate process,
nothing linked into cortex).

**Not selected:** Open3D/PyVista (Python-on-CI headless-GL pain, no dissemination
upside), Foxglove/RViz (inspection tools, weak for narrated educational video,
ROS pull).

---

## 5. Implementation plan

**Phase 0 — close the data gap. [DONE]** `tools/src/vio_pipeline.cpp` now emits the
per-frame 3×3 world-position covariance block (`pcov`, row-major) on every `est`
record in `run.jsonl`, for both the synthetic and EuRoC sources. Both tiers
consume it. (On the sample drone run it confirms the #212 story directly: the
reported position σ collapses to ~13 cm while the true error grows past 2 m.)

**Phase 1 — Three.js viewer (Tier 1). [PROTOTYPE DONE]**
- `docs-site/src/components/scene3d.js` (viewer) + `scene3d-math.js` (three-free
  eigen-decomposition, headlessly unit-tested) + `Scene3D.astro` (vanilla island
  with orbit + scrub + play + HUD). Renders GT path, est path, moving GT/est pose
  triads, and the **position-covariance ellipsoid** (3σ), flagging over-confidence
  when the true pose escapes it.
- Docs page `vio/scene3d.mdx` embeds it against a shipped sample
  `public/data/vio_run.jsonl`; deploys with the docs-site to GitHub Pages.
- **Offline mp4 [DONE].** `docs-site/scripts/gen-scene3d-video.mjs` serves the
  docs-site, drives the same viewer module in headless chromium (swiftshader
  WebGL) via `scene3d-render.html`, screenshots each frame, and pipes the PNG
  sequence through ffmpeg. Wired into `scripts/vio_scene_video.sh --3d`. The video
  is the same viewer the web page uses, so they match. Verified: 250-frame
  960×600 12.5 s mp4 with real rendered content.
- **Next increments:** (a) landmark cloud + camera frustum (emit the synthetic
  world's 3-D landmarks); (b) growing-trail / playback polish; (c) a fixed-orbit
  camera option for the video.

**Phase 2 — Rerun instrumentation (Tier 2).**
- Add the Rerun C++ SDK (FetchContent) behind a build flag; log poses, landmark
  cloud, pinhole/frustum, and covariance from `vio_pipeline.cpp`. Confine to
  `tools/` — do not let it leak into `sdk/`/`math/` (layering rule).
- Use it interactively for the #212 stage-probe work; export `.rrd` for talks.

**Phase 3 — Blender (optional).** A `bpy` script importing `run.jsonl` for a polished
explainer render when a flagship video is needed.
