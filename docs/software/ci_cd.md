# CI/CD Overview

This page documents the automation currently implemented in Omniseer, when it runs,
and what it does not prove.

## Current Automation

There are two GitHub Actions workflows:

| Workflow | Purpose |
| --- | --- |
| `ci` | Lint, ROS build/tests, Gazebo smoke, portable vision tests, firmware compile, and strict docs build |
| `docs` | Build and publish MkDocs to the `gh-pages` branch |

Documentation publishing is the only current deployment flow. The repository does
not automatically publish containers, firmware releases, robot images, experiment
bundles, or cloud dashboards.

## CI Triggers

The `ci` workflow runs on:

- every push to `master`
- every push to `portfolio/**`
- pull requests when opened, updated, or reopened
- manual workflow dispatch

All six jobs run for every eligible event; the workflow currently has no path-based
job filtering. Jobs run independently in parallel. Concurrency control cancels an
older run when a newer commit is pushed to the same branch or pull request.

## CI Jobs

### `lint`

Runs inside `osrf/ros:kilted-desktop-full-noble`.

- creates an isolated Python virtual environment
- installs Ruff `0.13.3`, matching `pyproject.toml`
- checks `analysis`, `bringup`, and `robot_diag_control`

Ruff is not installed through the ROS dependency script. Package-level Ruff tests
skip when the binary is unavailable because the dedicated lint lane owns that check.

### `ros-core`

Runs inside the ROS Kilted desktop-full image.

- installs dependencies from selected package manifests with rosdep
- builds the portable ROS package set
- excludes `rf2o_laser_odometry`, `yolo_bringup`, and `yolo_ros` from this graph
- runs package, unit, XML, CMake, and C++ lint tests
- conditionally builds `omniseer_vision_bridge` only when RKNN and RGA SDKs exist

Validated build packages:

- `omniseer_gz_assets`
- `omniseer_msgs`
- `yolo_msgs`
- `omniseer_description`
- `analysis`
- `bringup`
- `robot_io_adapters`
- `robot_diag_control`
- `robot_diag_control_cpp`

The default GitHub image does not provide RKNN/RGA, so the native bridge conditional
normally skips there. This lane does not prove NPU execution.

### `bringup-smoke`

Builds a deliberately minimal package set and launches headless Gazebo with CI-safe
geometry. It verifies that the launch remains alive and that these boundary topics
appear with the expected types:

| Topic | Type |
| --- | --- |
| `/clock` | `rosgraph_msgs/msg/Clock` |
| `/imu` | `sensor_msgs/msg/Imu` |
| `/scan` | `sensor_msgs/msg/LaserScan` |
| `/range` | `sensor_msgs/msg/Range` |
| `/mecanum_drive_controller/odometry` | `nav_msgs/msg/Odometry` |

Rosdep discovers local runtime packages so it does not search for nonexistent binary
packages. Optional runtime packages that are not needed by the smoke launch are then
explicitly excluded from the colcon build graph.

### `vision-host`

Runs directly on Ubuntu 24.04 and explicitly installs CMake, a C++ compiler, Make, and
GTest. It builds and runs only portable tests:

- `image_buffer_pool_test`
- `jsonl_telemetry_test`
- `rolling_telemetry_test`

RKNN, RGA, V4L2 camera, post-processing, text-embedding, and full pipeline tests stay
outside this lane because they require target SDKs, devices, or hardware.

### `firmware-build`

Performs a compile-only Teensy 4.1 build using:

- PlatformIO `6.1.19` in `${HOME}/.platformio/penv`
- Teensy platform `5.1.0`
- micro-ROS PlatformIO pinned to commit
  `cfee17faffaa532363b7151dd13af6a85c69d3c1`

The job uploads firmware build outputs. It does not flash a board or validate motor,
sensor, transport, timing, or watchdog behavior.

### `docs-build`

Uses Python 3.12 and runs `mkdocs build --strict` on every CI event. This catches
navigation, Markdown, and plugin-level documentation build failures before merge.

## Documentation Deployment

The separate `docs` workflow runs when:

- `docs/**` changes on `master`
- `mkdocs.yml` changes on `master`
- it is manually dispatched

It checks out full history, builds the site strictly, and deploys with
`mkdocs gh-deploy` to `gh-pages`. Pull requests validate documentation through the CI
job but do not deploy it.

## Local Equivalents

Use the workflow as the authoritative command source. The representative local flow is:

For the common happy path, the root scripts layer now wraps the main local flows:

```bash
scripts/omni setup ros-deps
scripts/omni build ros
scripts/omni test ros
scripts/omni test smoke-sim
scripts/omni test vision
scripts/omni build firmware
scripts/omni docs build
```

The exact underlying commands remain:

```bash
python3 -m venv /tmp/omniseer-ruff
/tmp/omniseer-ruff/bin/python -m pip install ruff==0.13.3
/tmp/omniseer-ruff/bin/ruff check \
  ros_ws/src/analysis \
  ros_ws/src/bringup \
  ros_ws/src/robot_diag_control
```

```bash
bash scripts/ci/install_ros_workspace_deps.sh \
  ros_ws/src/omniseer_gz_assets \
  ros_ws/src/omniseer_msgs \
  ros_ws/src/yolo_ros/yolo_msgs \
  ros_ws/src/omniseer_description \
  ros_ws/src/analysis \
  ros_ws/src/bringup \
  ros_ws/src/robot_io_adapters \
  ros_ws/src/robot_diag_control \
  ros_ws/src/robot_diag_control_cpp

set +u
source /opt/ros/kilted/setup.bash
set -u
cd ros_ws
colcon build --merge-install \
  --packages-ignore rf2o_laser_odometry yolo_bringup yolo_ros \
  --packages-select \
    omniseer_gz_assets omniseer_msgs yolo_msgs omniseer_description \
    analysis bringup robot_io_adapters robot_diag_control robot_diag_control_cpp

set +u
source install/setup.bash
set -u
colcon test --merge-install \
  --packages-ignore rf2o_laser_odometry yolo_bringup yolo_ros \
  --packages-select \
    omniseer_description analysis bringup robot_io_adapters \
    robot_diag_control robot_diag_control_cpp
colcon test-result --all --verbose
```

The headless smoke equivalent uses a narrower build:

```bash
bash scripts/ci/install_ros_workspace_deps.sh \
  ros_ws/src/omniseer_gz_assets \
  ros_ws/src/omniseer_msgs \
  ros_ws/src/omniseer_description \
  ros_ws/src/analysis \
  ros_ws/src/bringup \
  ros_ws/src/robot_io_adapters \
  ros_ws/src/robot_diag_control_cpp

set +u
source /opt/ros/kilted/setup.bash
set -u
cd ros_ws
colcon build --merge-install \
  --packages-ignore \
    analysis rf2o_laser_odometry robot_diag_control_cpp yolo_bringup yolo_ros \
  --packages-select \
    omniseer_gz_assets omniseer_msgs omniseer_description bringup robot_io_adapters

set +u
source install/setup.bash
set -u
OMNISEER_RUN_SIM_SMOKE=1 \
  python3 -m pytest -vv -s src/bringup/test/test_sim_launch_smoke.py
```

```bash
cmake -S vision -B vision/build-verify -DVISION_BUILD_HARNESS=OFF
cmake --build vision/build-verify \
  --target image_buffer_pool_test jsonl_telemetry_test rolling_telemetry_test
ctest --test-dir vision/build-verify \
  -R 'image_buffer_pool_test|jsonl_telemetry_test|rolling_telemetry_test' \
  --output-on-failure

${HOME}/.platformio/penv/bin/platformio run -d firmware -e teensy41
mkdocs build --strict
```

## Verification Boundary

CI currently does not guarantee:

- camera capture or stable device enumeration
- RGA or RKNN execution
- detector accuracy or target-hardware latency
- real LiDAR, IMU, sonar, encoder, battery, or motor behavior
- firmware flashing or micro-ROS transport behavior
- long-duration simulation or robot soak behavior
- experiment recording, cloud synchronization, or hosted review
- release packaging or deployment

Those checks require target hardware, a self-hosted runner, or later delivery
infrastructure. CI evidence should not be presented as real-hardware evidence.

## Planned Extensions

- target-hardware or hardware-in-the-loop validation
- run-bundle schema and recorder tests
- provider-neutral cloud synchronization checks
- hosted report build/deployment
- tagged release packaging for firmware and robot software
