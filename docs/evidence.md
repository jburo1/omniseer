# Evidence and Verification Boundary

This page defines what current Omniseer evidence proves, what it does not prove,
and where the next verification work should land.

## CI Verified

GitHub Actions `ci` is the public software gate for the default branch. A green
run means the repository passed these portable checks:

- Ruff lint for the Python ROS/operator packages.
- ROS Kilted package dependency install, build, and tests for the portable core
  package set.
- Headless Gazebo smoke launch with CI-safe geometry and five boundary topics:
  `/clock`, `/imu`, `/scan`, `/range`, and
  `/mecanum_drive_controller/odometry`.
- Portable host vision tests for buffer pooling, JSONL telemetry, and rolling
  telemetry.
- Compile-only Teensy 4.1 firmware build through `scripts/omni build firmware`.
- Strict MkDocs documentation build.

CI does **not** prove camera capture, RGA/RKNN execution, NPU latency, real
sensor/motor behavior, firmware flashing, micro-ROS transport, experiment
recording, cloud synchronization, or hosted reporting.

## Locally Verified

Recent Radxa dev-container cleanup checks used the repository front door and
focused CI-equivalent commands:

- `scripts/omni test smoke-sim`
- `scripts/omni build firmware`
- `scripts/omni docs build`
- narrowed `rosdep check` for the smoke package set

These local checks confirm the scripts and portable workflows behave correctly in
the current development container. They are not a substitute for target-hardware
evidence.

## Hardware Verified

The native vision implementation includes target-oriented V4L2, RGA, RKNN,
post-processing, and telemetry paths. Component-level target checks and harnesses
support those implementation claims.

The local run bundle `runs/pipeline_001` records a completed target-hardware
Phase 3 native perception run on ROCK 5B+:

- duration: 15.5 s
- detections: 230 records, 235 observed `person` detections
- performance: 27 `/vision/perf` records
- resource telemetry: 16 `system.jsonl` records
- native pipeline telemetry: 1,067 `pipeline_telemetry.jsonl` records
- native telemetry status: 836 producer and 231 consumer samples, all `ok`
- recorded latency: producer total mean 1.12 ms, consumer infer mean 60.34 ms,
  consumer infer p95 64.47 ms

This proves the Phase 3 recorded perception path can persist detections,
performance summaries, system telemetry, and native stage telemetry into one
inspectable run bundle.

The currently published docs do **not** yet include a completed integrated
target-hardware verification record for:

- real teleop reaching `/mecanum_drive_controller/reference`
- gateway/preview/overlay operation during the same real run

Those records belong in the Phase 2 and Phase 3 verification checklists before
claiming the full operator-integrated hardware slice is complete.

## Planned / Not Yet Verified

The following remain planned or pending evidence:

- selected visual evidence capture inside run bundles
- laptop report generator for recorded runs
- provider-neutral cloud synchronization
- hosted static review dashboard
- target-hardware or hardware-in-the-loop CI
- tagged release packaging for firmware and robot software

These items should be described as planned until there is recorded local,
hardware, or CI evidence for them.
