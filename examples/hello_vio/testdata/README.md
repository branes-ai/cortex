# hello_vio test fixtures

Both fixtures use the **real** EuRoC MAV `cam0/sensor.yaml` and
`imu0/sensor.yaml` (calibration only), in the ASL directory layout.

- `mav0/`: no measurements (header-only CSVs). `ctest -R hello_vio.smoke`
  checks the YAML → SDK-struct mapping and the estimator lifecycle.
- `frames/mav0/`: a tiny **synthetic** sequence: 5 frames at 20 Hz (160×120
  smooth texture shifting 1 px/frame) and 41 static IMU samples at 200 Hz
  (gravity on +x, no rotation). `ctest -R hello_vio.frames` runs `main.cpp`'s
  replay loop end to end. It is generated, so no dataset imagery is
  redistributed. Regenerate it with `gen_frames_fixture.py`.

Neither checks accuracy. Real-sequence accuracy gates are in
`tests/sdk/vio_euroc.cpp`.
