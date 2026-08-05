# Verification Evidence

This page ties Omniseer implementation claims to the checks and run records that
support them.

## CI Verified

GitHub Actions is the public portable verification gate for the default branch.
Path-filtered workflows cover the relevant changed area:

- Ruff lint for the Python ROS/operator packages.
- ROS Kilted package dependency install, build, and tests for the portable core
  package set.
- Headless Gazebo smoke launch with CI-safe geometry and four asserted boundary
  topics: `/clock`, `/imu`, `/scan`, and
  `/mecanum_drive_controller/odometry`.
- Portable host vision tests for buffer pooling, JSONL telemetry, and rolling
  telemetry.
- Portable run-bundle tests for manifest/summary handling, recording helpers,
  inspection, retrieval edge cases, system telemetry parsing, evidence annotation,
  and static report generation.
- Compile-only Teensy 4.1 firmware build through `scripts/omni build firmware`
  when firmware paths change.
- Strict MkDocs documentation build when docs paths change.
- Hardware-independent portable runtime image build and smoke check when runtime
  packaging or dependency paths change, without camera, RKNN/RGA, LiDAR, or
  Teensy requirements.

CI is intentionally portable. Target-hardware evidence covers camera capture,
RGA/RKNN execution, NPU latency, robot IO, firmware flashing, micro-ROS transport,
operator behavior, and verified robot-runtime image publishing.

## Locally Verified

Recent Radxa dev-container cleanup checks used the repository front door and
focused CI-equivalent commands:

- `scripts/omni test smoke-sim`
- `scripts/omni build firmware`
- `scripts/omni docs build`
- narrowed `rosdep check` for the smoke package set

These local checks confirm the scripts and portable workflows behave correctly in
the current development container.

## Hardware Verified

The native vision implementation includes target-oriented V4L2, RGA, RKNN,
post-processing, and telemetry paths. Component-level target checks and harnesses
support those implementation claims.

A local target-hardware RunBundle named `runs/pipeline_001` was used to record a
completed Phase 3 native perception run on ROCK 5B+. The raw bundle is not
currently included in the public repository. The documented measurements from
that run are:

- duration: 15.5 s
- detections: 230 records, 235 observed `person` detections
- performance: 27 `/vision/perf` records
- resource telemetry: 16 `system.jsonl` records
- native pipeline telemetry: 1,067 `pipeline_telemetry.jsonl` records
- native telemetry status: 836 producer and 231 consumer samples, all `ok`
- recorded latency: producer total mean 1.12 ms, consumer infer mean 60.34 ms,
  consumer infer p95 64.47 ms

This documented local evidence records the native perception path persisting
detections, performance summaries, system telemetry, and native stage telemetry
into one run bundle.

Additional target-hardware records document integrated operator behavior such as:

- real teleop reaching `/mecanum_drive_controller/reference`
- gateway/preview/overlay operation during the same real run

## Run Evidence Records

RunBundles provide the project evidence format. A complete bundle can include:

- `manifest.yaml` with run configuration and provenance
- `provenance/` copies of small vocabulary, class, vision config, and experiment
  config inputs when available
- `detections.jsonl` with typed perception outputs
- `perf.jsonl` with ROS vision performance summaries
- `system.jsonl` with low-rate resource telemetry
- `pipeline_telemetry.jsonl` with native runtime stage samples
- `autonomy.jsonl` with bounded target-centering events when autonomy is enabled
- `evidence/` images and annotations
- `logs/bringup.log` with ROS launch stdout/stderr from recorded runs
- generated inspection summaries and static HTML reports
