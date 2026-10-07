# hello_vio: a white-box SDK consumer

`hello_vio` runs visual-inertial odometry on a [EuRoC MAV](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)
sequence by calling the cortex C++ SDK directly, with no daemon, no Zenoh and no
Resource Manager. It is the smallest complete program that turns camera frames
and IMU samples into poses, and the starting point for anyone building on
the SDK ("Surface 1, the white box" in
[`docs/arch/cortex-repo.md`](../../docs/arch/cortex-repo.md)).

If you only want poses and don't want to write C++, use the daemon instead; see
`hello_zenoh` (issue #104, pending the Zenoh daemons of epic #75).

## Build and run

`hello_vio` builds with the rest of the tree (host builds only):

```bash
cmake --preset sitl-release
cmake --build --preset sitl-release --target hello_vio

# Download a EuRoC sequence (e.g. V1_01_easy, ASL format) and point at its mav0/:
./build/sitl-release/examples/hello_vio/hello_vio /data/euroc/V1_01_easy/mav0 200
```

Use a Release build for real sequences. The estimator is template-heavy and
a Debug build is slow (about 1m48s for 200 V1_01_easy frames).

The second argument is how many frames to replay (default 200, i.e. 10 s at
20 Hz). Output is one line per frame: timestamp, position, and orientation
quaternion of the IMU in the world frame.

On V1_01_easy:

```text
camera: fu=458.654 fv=457.296 cu=367.215 cv=248.375  k1=-0.28341 k2=0.07396  p_imu_cam=(-0.0216, -0.0647, 0.0098)
imu:    gyro noise 1.697e-04  accel noise 2.000e-03
sequence: 29120 IMU samples, 2912 frames (replaying up to 200)
        t[s]       x[m]       y[m]       z[m]        qw        qx        qy        qz
1403715273.2621     0.0000     0.0000     0.0000   1.00000   0.00000   0.00000   0.00000
...
1403715283.2121    -0.6894    -0.4249     0.1609   0.51264  -0.41210  -0.69493  -0.29058
replayed 200 frames
```

The pose stays at the origin while the estimator initializes (gravity
direction, biases from a stationary window), then the trajectory starts
moving. Timestamps are EuRoC's, in seconds.

CI runs `hello_vio` on every build against `testdata/mav0/`: the real EuRoC
`sensor.yaml` files with no frames (`ctest -R hello_vio.smoke`). That keeps
the configuration path and the estimator lifecycle tested without a dataset
download.

## Walkthrough

Everything is in [`main.cpp`](main.cpp), in four steps.

### 1. Configuration file → typed SDK structs

```cpp
const Backend::CameraCalibration cam = load_camera(mav0);   // cam0/sensor.yaml
const sdk::VioConfig config = load_vio_config(mav0);         // imu0/sensor.yaml
```

The SDK never parses files. It takes plain structs: a `CameraCalibration`
(pinhole intrinsics, radial-tangential distortion, camera→IMU extrinsics) and
a `VioConfig` (IMU noise model, sliding-window length, initialization options).
Turning a file into those structs is the application's job. Here it uses
yaml-cpp on the sensor files every EuRoC sequence ships. This is a
deliberate layering rule: SDK operators never see `yaml-cpp` types (see
`CONTRIBUTING.md`).

Two details are worth copying into your own loader:

- **OpenCV-written calibrations** start with `%YAML:1.0`, which is not valid
  YAML; `load_euroc_yaml()` drops that line, so both the dataset's own files
  and OpenCV re-exports load.
- **The extrinsics matter.** EuRoC's cam0 is rotated about 90° from the IMU. With an
  identity extrinsic the measurement model is grossly wrong and the filter
  diverges on fast motion. `T_BS` from `sensor.yaml` provides the real rotation and
  lever arm.

### 2. Construct the estimator and walk its lifecycle

```cpp
Estimator vio(Backend(std::vector<Backend::CameraCalibration>{cam}));
vio.configure(config);   // Unconfigured -> Inactive
vio.activate();          // Inactive     -> Active
...
vio.deactivate();        // Active       -> Inactive
vio.teardown();          // release everything
```

`VioEstimator<T, Backend>` is templated on the scalar type `T` (here `double`)
and on the backend (here the MSCKF; a sliding-window backend implements the
same interface). Parameters are fixed at `configure`. There is no live
parameter mutation; to change them, stop processing first:

- **Reconfigure the same estimator:** `deactivate()`, then `configure(new)`.
  This applies the new parameters and clears the runtime state, leaving it
  `Inactive`; call `activate()` to resume.
- **After `teardown()`:** teardown is terminal (`configure()` is ignored from
  there), so construct a new estimator.

See [ADR-0005](../../docs/adr/0005-managed-lifecycle-no-dynamic-reconfiguration.md).

### 3. Feed time-ordered measurements

```cpp
vio.feed_imu(span_of_imu_samples_up_to(frame.t_s));
vio.feed_image(frame.t_s, image.view());
```

For each frame: all IMU samples with timestamps up to the frame's, then the
frame itself (8-bit grayscale, as a non-owning `branes::cv::Image` view).
`euroc::parse_imu` and `euroc::parse_images` read the ASL CSV layout. For a live
sensor, feed the same two calls from your driver callbacks.

### 4. Read the pose back

```cpp
const auto pose = vio.current_pose();          // SE3: T_world_imu
pose.translation();                            // position [m]
pose.rotation().quaternion();                  // (w, x, y, z)
```

`current_state()` returns the full navigation state (velocity and biases too),
and the backend exposes its covariance for consistency checks; see
`branes/sdk/eval/` for the trajectory and NEES/NIS tools the test suite uses.

## Where to go next

- `branes::sdk::euroc::replay()` wraps steps 2–4 for a whole sequence.
- `tests/sdk/vio_euroc.cpp` runs full sequences against accuracy gates.
- The [docs site](https://branes-ai.github.io/cortex/) covers the VIO pipeline
  stage by stage.
