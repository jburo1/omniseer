# CI/CD Overview

This page explains the automation that currently exists in the Omniseer
repository and how it affects day-to-day development.

## What "CI/CD" means in this repo

There are currently two GitHub Actions workflows:

- `ci`: continuous integration for lint, build, unit/package tests, launch
  smoke, firmware compile, and docs validation
- `docs`: continuous deployment for the MkDocs documentation site

At the moment, Omniseer does **not** have a release pipeline for firmware,
containers, robot images, or production deployments. The only deployment flow
in the repository today is documentation publishing to GitHub Pages.

## Workflow Summary

| Workflow | File | Purpose |
| --- | --- | --- |
| CI | `.github/workflows/ci.yml` | Validate the main software path on GitHub Actions |
| Docs | `.github/workflows/docs.yml` | Build and publish the documentation site |

## CI Workflow

The CI workflow is defined in `.github/workflows/ci.yml`.

### When it runs

The workflow runs when:

- code is pushed to `master`
- a pull request is opened or updated
- the workflow is triggered manually from the GitHub Actions UI

This means CI acts as a gate for the default branch and for incoming PRs.

### What it does

The workflow is split into focused jobs:

| Job | Purpose |
| --- | --- |
| `lint` | Run Ruff against the Python helper/operator packages |
| `ros-core` | Install ROS workspace deps, build the validated package set, run unit/package tests, and conditionally build `omniseer_vision_bridge` when the RKNN/RGA SDKs are available |
| `bringup-smoke` | Launch headless simulation and verify the core sim/hardware boundary topics and type contracts |
| `vision-host` | Build and run the portable native vision tests |
| `firmware-build` | Run a compile-only PlatformIO build for the Teensy firmware target |
| `docs-build` | Build the MkDocs site in strict mode on every PR/push |

The validated ROS package set in `ros-core` now includes:

- `omniseer_gz_assets`
- `omniseer_msgs`
- `yolo_msgs`
- `omniseer_description`
- `analysis`
- `bringup`
- `robot_io_adapters`
- `robot_diag_control`
- `robot_diag_control_cpp`

The ROS test subset now includes:

- `omniseer_description`
- `analysis`
- `bringup`
- `robot_io_adapters`
- `robot_diag_control`
- `robot_diag_control_cpp`

The `bringup-smoke` job intentionally checks a narrow, stable contract instead
of trying to prove the full robot stack. It uses a dedicated minimal world and
CI-safe geometry path so the check does not depend on optional Gazebo models or
STL-heavy collision geometry. Today it verifies that headless sim bringup
publishes:

- `/clock`
- `/imu`
- `/scan`
- `/range`
- `/mecanum_drive_controller/odometry`

It also checks the expected message types on that boundary and fails if the
headless sim graph exits before those topics come up.

### Workflow hygiene

The CI workflow now includes a few guardrails that matter once hardware work
starts:

- `concurrency` cancels superseded runs on the same branch or PR
- each job has an explicit timeout
- ROS, vision, firmware, and smoke-test logs are uploaded as artifacts
- ROS dependency installation is driven from package manifests through
  `scripts/ci/install_ros_workspace_deps.sh`

### Why this helps

This workflow gives quick feedback when a change breaks:

- ROS package metadata or dependency declarations
- message generation for the selected interface packages
- Python linting in the helper and operator packages
- the C++ adapter and gateway test suites
- the package-level XML and test wiring in `analysis`, `bringup`, and the
  operator tooling
- the core sim/hardware boundary wiring before real hardware is in the loop
- compile-time firmware regressions on the main MCU target
- the portable part of the native vision runtime
- docs regressions that would otherwise only show up at deploy time

In practice, this reduces the chance that a broken branch reaches `master`
without anyone noticing.

### Current CI boundaries

The CI workflow is intentionally narrower than the full repository.

It does **not** currently guarantee:

- real hardware validation
- camera or sensor availability
- RKNN/RGA-backed execution inside the default GitHub runner image
- firmware flashing or micro-ROS deployment
- long-duration Gazebo or robot end-to-end runtime validation
- release packaging or artifact publishing

Those would require additional jobs, richer runners, or hardware-aware test
infrastructure.

## Docs Workflow

The docs workflow is defined in `.github/workflows/docs.yml`.

### When it runs

The workflow runs when:

- files under `docs/` change on `master`
- `mkdocs.yml` changes on `master`
- the workflow is triggered manually

### What it does

The job:

1. Checks out the repository with full history
2. Installs MkDocs and the Material theme
3. Builds the site in strict mode
4. Deploys the generated documentation to the `gh-pages` branch

This is the current "CD" piece of the repository.

## Developer Experience

From a contributor's perspective, the automation provides a few immediate
benefits:

- PRs get an automatic build/test signal without requiring every reviewer to
  reproduce the same checks locally
- dependency drift in ROS packages is more likely to fail early
- lightweight regressions in Python helpers, operator tools, and portable
  vision utilities are caught before merge
- the sim/hardware boundary gets a fast smoke signal before hardware
  integration starts absorbing debugging time
- firmware compile breaks show up before anyone plugs in a board
- docs changes can be published without a manual deployment step

The local equivalents of the current CI checks are:

```bash
ruff check ros_ws/src/analysis ros_ws/src/bringup ros_ws/src/robot_diag_control

source /opt/ros/kilted/setup.bash
cd ros_ws
colcon build --merge-install \
  --packages-select \
    omniseer_gz_assets \
    omniseer_msgs \
    yolo_msgs \
    omniseer_description \
    analysis \
    bringup \
    robot_io_adapters \
    robot_diag_control \
    robot_diag_control_cpp
colcon test --merge-install \
  --packages-select \
    omniseer_description \
    analysis \
    bringup \
    robot_io_adapters \
    robot_diag_control \
    robot_diag_control_cpp
colcon test-result --all --verbose
```

For the headless bringup smoke check:

```bash
source /opt/ros/kilted/setup.bash
cd ros_ws
colcon build --merge-install \
  --packages-select omniseer_gz_assets omniseer_msgs omniseer_description bringup robot_io_adapters
source install/setup.bash
OMNISEER_RUN_SIM_SMOKE=1 python3 -m pytest -vv -s src/bringup/test/test_sim_launch_smoke.py
```

For the portable host vision tests:

```bash
cmake -S vision -B vision/build-verify -DVISION_BUILD_HARNESS=OFF
cmake --build vision/build-verify \
  --target image_buffer_pool_test jsonl_telemetry_test rolling_telemetry_test
ctest --test-dir vision/build-verify \
  -R 'image_buffer_pool_test|jsonl_telemetry_test|rolling_telemetry_test' \
  --output-on-failure
```

For the compile-only firmware check:

```bash
platformio run -d firmware -e teensy41
```

For docs validation:

```bash
mkdocs build --strict
```

If the RKNN and RGA SDKs are installed locally, you can also build:

```bash
source /opt/ros/kilted/setup.bash
cd ros_ws
colcon build --merge-install --packages-select omniseer_vision_bridge
```

## Recommended Future Extensions

As subsystem documentation grows, this page should stay focused on repository
automation and point outward to the subsystem docs.

Likely future additions:

- a release workflow for tagged versions
- hardware-in-the-loop jobs on a self-hosted runner
- longer-running integration tests for launch flows and ROS graphs
- container image builds
- firmware flash/test automation
- artifact retention for logs, test reports, and generated packages
