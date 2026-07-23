# Scripts Front Door

_Status: implemented; command surface expected to grow additively as the project advances_

The root `scripts/omni` entrypoint is the supported front door for common local
workflows in this repository.

Use it when you want a stable human-facing command instead of remembering:

- which subdirectory owns a helper
- which workspace setup file must be sourced first
- which local build/test subset matches the documented workflow

The design intent is simple:

- one memorable top-level command
- thin wrappers around the existing build, test, run, and flash flows
- explicit profile selection for real-hardware bringup as the rollout expands
- additive growth without breaking familiar operator commands

This does **not** replace every package-local helper. Narrow package-owned scripts
still live close to their code. `scripts/omni` promotes the commands you are most
likely to run repeatedly.

## Core Usage

General shape:

```bash
scripts/omni <command> [subcommand] [args...]
```

Use help at any level:

```bash
scripts/omni --help
scripts/omni run --help
scripts/omni run real --help
```

Useful discovery command:

```bash
scripts/omni env
scripts/omni doctor
```

`env` prints resolved path/default settings. `doctor` checks local toolchains,
workspace setup, hardware SDKs, camera devices, installed ROS packages, and the
real vision asset paths used by the robot demo.

## Command Groups

### `setup`

Install local prerequisites needed for the portable ROS workflow.

Primary command:

```bash
scripts/omni setup ros-deps
```

Use this when:

- you are setting up a fresh machine or container
- ROS package dependencies have changed
- you want the local dependency set that matches the documented portable ROS flow

Notes:

- this wraps `scripts/ci/install_ros_workspace_deps.sh`
- it installs the default dependency set when no package paths are provided
- you can also pass explicit package paths when you want a narrower install


### `build`

Run the main local build flows without reassembling the underlying commands.

Commands:

```bash
scripts/omni build
scripts/omni build all
scripts/omni build strict
scripts/omni build ros --with-vision
scripts/omni build ros --without-vision
scripts/omni build vision
scripts/omni build firmware
```

Use `build` or `build all` when:

- you want the normal product build from one command
- you want ROS built first, native vision built when CMake is available, and
  firmware built when PlatformIO is available
- you want missing optional toolchains to warn instead of stopping the whole build

Use `build strict` when:

- you want the same product build but missing native vision or firmware toolchains
  should fail the command

Use `build ros` when:

- you changed ROS packages, launch files, configs, or message wiring
- you need the workspace install tree refreshed before a launch
- you are on the robot and want hardware-only ROS packages included when their
  SDK files are installed

Use `build ros --with-vision` when:

- you need `omniseer_vision_bridge` installed for native RKNN/RGA perception
- you want the build to fail loudly if RKNN/RGA development files are missing

Use `build ros --without-vision` when:

- you are on a robot image but intentionally want the portable ROS package set
- you are checking launch/package changes without rebuilding the native bridge

Use `build vision` when:

- you are iterating on the native vision code outside the ROS layer
- you want the default `vision/build` CMake flow

Use `build firmware` when:

- you changed Teensy firmware
- you want a compile-only firmware check before flashing

Suggestion:

- `build` is the best default before an operator run or handoff
- `build ros` is the best focused target when your change touches launch, bringup, adapters, gateway diagnostics, or ROS message surfaces
- `build ros --with-vision` requires RKNN/RGA development files and is expected to fail on hosts without the target SDKs
- `build vision` is better than a full ROS build when only the native C++ vision pipeline changed

### `runtime`

Build, record, verify, publish, and pull robot runtime container checkpoints.
This is a robot-local workflow; it does not require GitHub Actions to rebuild
the hardware-specific image.

Commands:

```bash
scripts/omni runtime build
scripts/omni runtime record
scripts/omni runtime verify
scripts/omni runtime verify --stage full
scripts/omni runtime push
scripts/omni runtime pull
```

Use `runtime record` for a manual operator evidence run. It starts the latest
local runtime image in the operator profile, records an indefinite runbundle
under `runs/operator_<UTC>`, samples system telemetry every second, and exits
when the container is stopped or interrupted. Pass extra launch args after `--`,
for example:

```bash
sudo scripts/omni runtime record -- start_lidar:=false
```

When these commands are launched from a devcontainer, the wrapper resolves the
workspace's host-side bind path before starting Docker so runbundles still appear
under this checkout's `runs/` directory. Override that detection with
`OMNISEER_RUNTIME_RUNS_HOST_ROOT=/host/path/to/omniseer/runs` if needed.

Use `runtime verify` without `--stage` for a safe container smoke check. Use
`--stage full` to run the real operator smoke path with run recording and image
provenance. `runtime push` publishes only after a passed full verification for
the same local image ID. Default builds are tagged
`robot-candidate-<UTC>-g<shortsha>`; push automatically promotes the same image
to immutable `robot-verified-<UTC>-g<shortsha>` and moving `robot-verified`
tags.
`runtime verify` runs Docker without an interactive TTY so it works under `sudo`,
SSH automation, and other non-interactive launch paths. For direct `runtime run`
commands, Docker TTY allocation defaults to `auto`; set
`OMNISEER_RUNTIME_DOCKER_TTY=always` or `never` to override it.

See [Robot Runtime Container](robot_runtime_container.md#checkpoint-and-promotion)
for the full pre-registry workflow, including runbundle inspection and report
generation.

### `test`

Run focused local verification flows.

Commands:

```bash
scripts/omni test ros
scripts/omni test vision
scripts/omni test smoke-sim
```

Use `test ros` when:

- you changed portable ROS packages
- you want the closest local equivalent to the documented ROS CI lane

Use `test vision` when:

- you changed portable native vision components
- you want the targeted host-side vision tests from the CI docs

Use `test smoke-sim` when:

- you changed simulation bringup or topic boundaries
- you want a headless Gazebo smoke check instead of a broad test sweep

Suggestion:

- start with the smallest relevant test first
- use `test smoke-sim` after sim launch or boundary-topic changes
- use `test ros` after wider ROS package changes

### `run`

Launch the operator-facing runtime workflows. Build explicitly first when code,
launch files, or generated interfaces changed.

Commands:

```bash
scripts/omni run sim
scripts/omni run real
scripts/omni run real --profile perception --record-run demo_001
scripts/omni run real --profile legacy-teleop smoke
scripts/omni run monitor --host <robot-ip>
scripts/omni run teleop
```

#### `run sim`

Launches the existing simulation bringup path.

Use this when:

- you want the normal local simulation entrypoint
- you want the bringup-owned cleanup behavior before launch

Example:

```bash
scripts/omni run sim headless:=true
```

#### `run real`

Launches a named real-hardware runtime profile.

Current supported profiles:

- `current`: default alias for the current supported real robot runtime
- `operator`: gateway, native vision, preview, bounded teleop, and recording
- `perception`: native vision and recording without the gateway operator surface
- `legacy-teleop`: older keyboard teleop path kept for hardware diagnostics

Examples:

```bash
scripts/omni run real
scripts/omni run real smoke
scripts/omni run real verify
scripts/omni run real --record-run demo_001
scripts/omni run real --profile perception --record-run demo_001
scripts/omni run real --profile legacy-teleop operator camera_device:=/dev/video11
```

If `--profile` is omitted, the command selects `current`. Today `current`
resolves to `operator`; that alias can move as the supported real runtime
changes.

The `operator` and `perception` profiles default to foreground `bringup`.
`legacy-teleop` defaults to its older background bringup plus keyboard teleop
mode.

Recording flags can be used with modes that launch real bringup:

```bash
scripts/omni run real --record
scripts/omni run real --record-run demo_001
scripts/omni run real --record-run demo_001 --record-out runs/demo_001
scripts/omni run real --record-run demo_001 \
  --record-container-image-ref ghcr.io/acme/omniseer:robot-v2 \
  --record-container-image-digest sha256:<digest> \
  --record-experiment-config experiments/container-smoke.yaml \
  --record-experiment-parameter profile=operator
```

The recorder is an optional sidecar. It writes a local bundle containing
`manifest.yaml`, `detections.jsonl`, `perf.jsonl`, optional automatic
`system.jsonl` resource telemetry, optional native
`pipeline_telemetry.jsonl`, representative native JPEG evidence frames under
`evidence/`, and `summary.json`. The first slice stores bundles on the robot;
laptop download, inspection, evidence annotation, and a simple local HTML report
are available through `scripts/omni runs`. Rich hosted review and cloud
synchronization remain later work.

`manifest.yaml` records the resolved real profile, mode, command, and launch
arguments. `system.jsonl` records low-rate CPU, memory, thermal, WiFi/network,
onboard battery, and `/battery` LiPo snapshots when those sources are available.

`--record-overwrite` removes and recreates the selected run directory before
launching ROS. The recorder then accepts an empty precreated directory, or one
where the native vision node has already opened `pipeline_telemetry.jsonl` or
`evidence/`, so startup ordering does not drop pipeline telemetry.

Containerized runs can provide the same provenance through
`OMNISEER_CONTAINER_IMAGE_REF`, `OMNISEER_CONTAINER_IMAGE_DIGEST`,
`OMNISEER_GIT_SHA`, `OMNISEER_EXPERIMENT_CONFIG`, and
`OMNISEER_EXPERIMENT_PARAMETERS`. Experiment parameters may be a JSON object or
comma/space-separated `key=value` pairs.

#### Real run modes

`operator`

- starts the `legacy-teleop` bringup in the background
- opens keyboard teleop in the current terminal

`smoke`

- starts the same bringup
- runs the passive teleop/perception verifier
- shuts the bringup down afterward

`bringup`

- launches only the selected profile bringup in the foreground

`teleop`

- launches only the stamped keyboard teleop publisher

`verify`

- runs only the passive verification helper against an existing ROS graph

Suggestions:

- run `scripts/omni build ros` before `run` when code or launch wiring changed
- use `run real` on the robot for the current supported real runtime
- use `run real smoke` for a quick integrated health check
- use `run real --profile legacy-teleop operator` for the old keyboard teleop path
- add `--record-run <run_id>` when the run should produce a local perception bundle

#### `run monitor`

Launches the Tk operator monitor from the laptop workspace:

```bash
scripts/omni run monitor --host <robot-ip>
```

The wrapper sources ROS and the workspace, refreshes status on startup, and lets
the GUI use the gateway host as the default preview host. Additional monitor
arguments are forwarded to `robot_monitor_gui`.

### `runs`

List and retrieve robot-side perception run bundles from the laptop workspace:

```bash
scripts/omni runs local-list --root runs
scripts/omni runs inspect runs/imported/demo_001
scripts/omni runs report runs/imported/demo_001
scripts/omni runs list
scripts/omni runs pull demo_001
```

`local-list`, `inspect`, and `annotate` operate on local bundle directories
through the `omniseer_experiments` tools. `annotate` derives
`evidence/annotated/*.jpg` from clean evidence frames and `evidence/evidence.jsonl`
without modifying the canonical `evidence/frames/*.jpg` inputs. `report`
generates missing annotations first, then writes a derived static HTML summary at
`report/index.html`, using annotated evidence when present and falling back to
clean evidence frames. The report includes an evidence summary, run
configuration, detection counts, performance and native pipeline telemetry,
resource samples with sample timestamps, error/drop counters, evidence links, and
inspection issues. `list` and `pull` operate on robot-side bundles over SSH and
validate pulled bundles locally.

The front door defaults to SSH target `radxa@192.168.1.178` and remote run root
`/home/radxa/apps/omniseer/runs`. Override those with `--host`, `--user`,
`--remote-root`, `OMNISEER_ROBOT_HOST`, `OMNISEER_ROBOT_USER`, or
`OMNISEER_ROBOT_RUNS_ROOT` when using a different robot, account, or checkout.

### `check`

Run passive validation commands that inspect an existing graph rather than building
or launching it.

Command:

```bash
scripts/omni check
scripts/omni check real-perception
```

Use this when:

- the real graph is already running
- you want to confirm the legacy teleop/perception boundary topics
- you want a non-driving verification helper

Suggestion:

- pair this with `run real --profile legacy-teleop smoke` for a quick end-to-end check
- use `OMNISEER_REQUIRE_DETECTIONS=1` when detections must be present for the run to count

### `doctor`

Report local environment state relevant to build and robot runs.

Command:

```bash
scripts/omni doctor
```

Use this when:

- a build or launch fails due to missing workspace, SDK, device, or asset state
- you are moving between laptop, container, and robot images
- you want to confirm whether `omniseer_vision_bridge` is installed and buildable

### `flash`

Run hardware flashing helpers.

Command:

```bash
scripts/omni flash teensy
```

Use this when:

- you need headless Teensy flashing over SSH or in a container
- you want the supported wrapper around the existing firmware flash helper

### `docs`

Build the documentation site locally.

Command:

```bash
scripts/omni docs build
```

Use this when:

- you changed docs or `mkdocs.yml`
- you want the local strict docs build without waiting for CI

### `clean`

Remove generated local build artifacts.

Commands:

```bash
scripts/omni clean
scripts/omni clean ros
scripts/omni clean vision
scripts/omni clean docs
scripts/omni clean all
```

Use this when:

- a local build tree is stale or confusing
- you want to force a clean rebuild of a specific layer

Suggestion:

- `clean` defaults to the ROS workspace, matching the `build` and `test` defaults
- prefer the narrowest clean target that solves the problem
- avoid `clean all` unless multiple build trees are genuinely suspect

## Suggested Workflows

### Fresh local ROS iteration

```bash
scripts/omni setup ros-deps
scripts/omni build ros
scripts/omni test ros
```

Use this for:

- normal ROS package development
- launch/config changes that need rebuild plus validation

### Simulation-focused iteration

```bash
scripts/omni build ros
scripts/omni test smoke-sim
scripts/omni run sim
```

Use this for:

- simulation bringup changes
- boundary-topic or launch-structure debugging

### Legacy Teleop Check

```bash
scripts/omni run real --profile legacy-teleop smoke
```

Use this for:

- the quickest legacy teleop health check
- validating teleop and perception boundaries before a longer session

### Current Real Operator Demo

Robot:

```bash
scripts/omni run real
```

Robot with local run-bundle recording:

```bash
scripts/omni run real --record-run demo_001
```

Laptop:

```bash
scripts/omni run monitor --host <robot-ip>
```

Active zero-motion acceptance verification from a second robot shell:

```bash
OMNISEER_REQUIRE_DETECTIONS=1 scripts/omni run real verify
```

Use this for the integrated gateway status, preview, perception, and bounded
teleop acceptance run described in `operator_integrated_demo.md`.

### Native vision iteration

```bash
scripts/omni build vision
scripts/omni test vision
```

Use this for:

- portable native vision changes
- tight local C++ iteration without rebuilding the ROS workspace first

### Firmware loop

```bash
scripts/omni build firmware
scripts/omni flash teensy
```

Use this for:

- firmware compile followed by hardware flashing

## Compatibility Notes

The new front door keeps a compatibility path for existing habits:

- `scripts/omni up <profile>` still builds ROS before launching and emits a
  deprecation warning
- `scripts/omni run real --phase 2` maps to `--profile legacy-teleop`
- `scripts/omni run real --phase 3` maps to `--profile operator`
- `scripts/phase2_real.sh` delegates to `scripts/omni run real --profile legacy-teleop`
- `scripts/check_real_teleop_perception.sh` delegates to `scripts/omni check real-perception`

Those compatibility wrappers exist to reduce churn. New docs and day-to-day usage
should prefer `scripts/omni`.

## Environment Overrides

Useful environment variables:

- `OMNISEER_ROS_SETUP`
- `OMNISEER_WS_SETUP`
- `OMNISEER_UROS_WS_SETUP`
- `OMNISEER_ROS_DISTRO`
- `OMNISEER_LATEST_STABLE_REAL_PHASE`
- `OMNISEER_VISION_PARAMS_FILE`
- `OMNISEER_BRINGUP_DELAY_SEC`
- `OMNISEER_BRINGUP_LOG`
- `OMNISEER_REQUIRE_DETECTIONS`

Use `scripts/omni env` to confirm what the front door will resolve on the current
machine.
