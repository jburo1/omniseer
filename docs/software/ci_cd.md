# CI/CD Overview

This page explains the automation that currently exists in the Omniseer
repository and how it affects day-to-day development.

## What "CI/CD" means in this repo

There are currently two GitHub Actions workflows:

- `ci`: continuous integration for build, lint, and test validation
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

The CI job currently performs the following checks:

1. Installs the ROS 2 `kilted` build and test dependencies needed by the
   selected package set
2. Runs Ruff against the Python packages in `ros_ws/src/analysis` and
   `ros_ws/src/bringup`
3. Builds the main validated ROS package subset:
   - `omniseer_msgs`
   - `yolo_msgs`
   - `omniseer_description`
   - `analysis`
   - `bringup`
4. Builds `omniseer_vision_bridge` only when the RKNN and RGA SDKs are
   available in the runner environment
5. Runs the current ROS test subset:
   - `omniseer_description`
   - `analysis`
   - `bringup`
6. Builds and runs the portable native vision tests:
   - `image_buffer_pool_test`
   - `jsonl_telemetry_test`
   - `rolling_telemetry_test`

### Why this helps

This workflow gives quick feedback when a change breaks:

- ROS package metadata or dependency declarations
- message generation for the selected interface packages
- Python linting in the helper packages
- the frontier-selection and `omniseer_description` C++ tests
- the package-level XML and test wiring in `analysis` and `bringup`
- the portable part of the native vision runtime

In practice, this reduces the chance that a broken branch reaches `master`
without anyone noticing.

### Current CI boundaries

The CI workflow is intentionally narrower than the full repository.

It does **not** currently guarantee:

- real hardware validation
- camera or sensor availability
- RKNN/RGA-backed execution inside the default GitHub runner image
- firmware flashing or micro-ROS deployment
- Gazebo or robot end-to-end runtime validation
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
3. Builds the site
4. Deploys the generated documentation to the `gh-pages` branch

This is the current "CD" piece of the repository.

## Developer Experience

From a contributor's perspective, the automation provides a few immediate
benefits:

- PRs get an automatic build/test signal without requiring every reviewer to
  reproduce the same checks locally
- dependency drift in ROS packages is more likely to fail early
- lightweight regressions in Python helpers and portable vision utilities are
  caught before merge
- docs changes can be published without a manual deployment step

The local equivalents of the current CI checks are:

```bash
ruff check ros_ws/src/analysis ros_ws/src/bringup

source /opt/ros/kilted/setup.bash
cd ros_ws
colcon build --merge-install \
  --packages-select omniseer_msgs yolo_msgs omniseer_description analysis bringup
colcon test --merge-install \
  --packages-select omniseer_description analysis bringup
colcon test-result --all --verbose

cd /path/to/omniseer
cmake -S vision -B vision/build-verify -DVISION_BUILD_HARNESS=OFF
cmake --build vision/build-verify \
  --target image_buffer_pool_test jsonl_telemetry_test rolling_telemetry_test
ctest --test-dir vision/build-verify \
  -R 'image_buffer_pool_test|jsonl_telemetry_test|rolling_telemetry_test' \
  --output-on-failure
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
- integration tests for launch flows and ROS graphs
- container image builds
- firmware build and test automation
- artifact retention for logs, test reports, and generated packages
