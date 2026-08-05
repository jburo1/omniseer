# CI/CD Overview

This page documents the automation currently implemented in Omniseer, when it
runs, and what it does not prove.

## Current Automation

GitHub Actions is split into four small workflows with built-in path filters:

| Workflow | Triggered by | Purpose |
| --- | --- | --- |
| `ci` | `ros_ws/**`, `vision/**`, `tests/**`, `scripts/**`, `pyproject.toml`, `pytest.ini` | Normal portable software CI |
| `runtime` | `docker/runtime/**`, runtime scripts, `pyproject.toml`, `uv.lock`, ROS package manifests, or manual dispatch | Portable runtime container validation |
| `firmware` | `firmware/**`, `uros_ws/**` | Compile-only firmware check |
| `docs` | `docs/**`, `mkdocs.yml`, `scripts/docs/**` | Strict documentation build and `gh-pages` publish |

Each workflow runs only for pushes to `master`, except `runtime`, which also
supports manual dispatch before a robot release. Concurrency control cancels older
runs of the same workflow when a newer commit is pushed.

Documentation publishing is the only automatic deployment flow. GitHub Actions
does not publish runtime containers, firmware releases, robot images, experiment
bundles, or cloud dashboards.

## `ci`

The normal software workflow runs when portable software paths change. It does not
contain custom path-classification logic.

### `lint`

Runs on Ubuntu 24.04:

- existing pre-commit hooks over tracked files, including Ruff and Ruff format
- Shellcheck over tracked shell scripts

### `python-tests`

Runs root repository tests with:

```bash
python3 -m pytest -q tests
```

This covers script front doors, runtime workflow helpers, and other host-side unit
tests that are not part of the ROS workspace test graph.

### `ros`

Runs inside the ROS Kilted desktop-full image:

- installs dependencies from the selected portable package manifests with rosdep
- builds the portable ROS package set
- excludes `rf2o_laser_odometry`, `yolo_bringup`, and `yolo_ros`
- runs package, unit, XML, CMake, and C++ lint tests
- runs the headless Gazebo bringup smoke test

The smoke test launches CI-safe geometry and positively verifies that these
boundary topics appear with expected types:

| Topic | Type |
| --- | --- |
| `/clock` | `rosgraph_msgs/msg/Clock` |
| `/imu` | `sensor_msgs/msg/Imu` |
| `/scan` | `sensor_msgs/msg/LaserScan` |
| `/mecanum_drive_controller/odometry` | `nav_msgs/msg/Odometry` |

Normal simulation can publish `/range` through the scan-to-range adapter when
`start_range_adapter` is enabled; that topic is not currently asserted by the CI
smoke test.

The RKNN/RGA `omniseer_vision_bridge` is not compiled in GitHub Actions. That
hardware-specific bridge is compiled and verified by the ROCK 5B+
`robot-runtime` release process.

### `vision-host`

Runs directly on Ubuntu 24.04 and builds/runs portable native vision tests:

- `image_buffer_pool_test`
- `jsonl_telemetry_test`
- `rolling_telemetry_test`

RKNN, RGA, V4L2 camera, post-processing, text-embedding, and full pipeline tests
stay outside this lane because they require target SDKs, devices, or hardware.

## `runtime`

The runtime workflow validates portable container packaging. It runs on
packaging/dependency changes and can be manually dispatched before a robot
release.

It builds the hardware-independent runtime image:

```bash
scripts/omni build runtime-container \
  --target portable-runtime \
  --image omniseer/robot-runtime:portable-ci
```

The job checks that the image can source the ROS workspace and resolve expected
packages, then starts the image with camera, RKNN/RGA, LiDAR, Teensy,
boundary-topic waits, and pre-launch cleanup disabled. The smoke step requires a
positive ROS node readiness signal from `/twist_mux` before it passes; timeout
alone is not accepted.

This workflow does not authenticate to GHCR and does not push an image.

## `firmware`

The firmware workflow performs a compile-only Teensy 4.1 build through:

```bash
scripts/omni build firmware
```

It does not flash a board or validate motor, sensor, transport, timing, or
watchdog behavior.

## `docs`

The docs workflow installs the MkDocs dependencies and runs:

```bash
mkdocs build --strict
```

This catches navigation, Markdown, and plugin-level documentation build failures.
After the strict build passes, it deploys the site to `gh-pages` with:

```bash
mkdocs gh-deploy --clean --force --verbose
```

## Robot Runtime Releases

Hardware-specific `robot-runtime` images are built and published manually on the
ROCK 5B+:

```bash
git checkout <passed-master-commit>
scripts/omni runtime build
scripts/omni runtime verify
scripts/omni runtime verify --stage full
scripts/omni runtime push
```

`runtime push` requires a clean working tree, passed full verification, a matching
verified image ID, and matching verification/current git commits. It promotes the
verified local image to `robot-verified-g<full-commit-sha>`, optional
`--release-tag <tag>`, and moving `robot-verified`, then records release metadata
under `.omniseer/runtime/`.

## Local Equivalents

Use focused repository commands for local checks:

```bash
python3 -m pytest -q tests
scripts/omni build ros
scripts/omni test ros
scripts/omni test smoke-sim
scripts/omni test vision
scripts/omni build runtime-container --target portable-runtime --image omniseer/robot-runtime:portable
scripts/omni build firmware
scripts/omni docs build
```

## Verification Boundary

CI currently does not guarantee:

- camera capture or stable device enumeration
- RGA or RKNN execution
- detector accuracy or target-hardware latency
- real LiDAR, IMU, sonar, encoder, battery, or motor behavior
- firmware flashing or micro-ROS transport behavior
- long-duration simulation or robot soak behavior
- target-hardware experiment recording
- RKNN/RGA `omniseer_vision_bridge` compilation on generic runners
- hardware-specific runtime image build or release publishing

Those checks require the ROCK 5B+, target SDKs, devices, or later
hardware-in-the-loop infrastructure. CI evidence should not be presented as
real-hardware evidence.
