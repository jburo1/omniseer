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
- explicit phase selection for real-hardware bringup as the rollout expands
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
```

This prints the resolved repo root, ROS setup path, workspace setup path, and the
latest stable real phase.

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
scripts/omni build ros
scripts/omni build vision
scripts/omni build firmware
```

Use `build ros` when:

- you changed ROS packages, launch files, configs, or message wiring
- you need the workspace install tree refreshed before a launch

Use `build vision` when:

- you are iterating on the native vision code outside the ROS layer
- you want the default `vision/build` CMake flow

Use `build firmware` when:

- you changed Teensy firmware
- you want a compile-only firmware check before flashing

Suggestion:

- `build ros` is the best default when your change touches launch, bringup, adapters, gateway diagnostics, or ROS message surfaces
- `build vision` is better than a full ROS build when only the native C++ vision pipeline changed

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

Launch the operator-facing runtime workflows.

Commands:

```bash
scripts/omni run sim
scripts/omni run real --phase 0.5
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

#### `run real --phase <number>`

Launches the selected real-hardware rollout slice.

Current supported phase:

- `0.5`

Examples:

```bash
scripts/omni run real --phase 0.5
scripts/omni run real --phase 0.5 smoke
scripts/omni run real --phase 0.5 bringup camera_device:=/dev/video11
```

If `--phase` is omitted, the command selects the latest stable real phase and
prints which one it chose.

Why phases exist:

- the real stack is being completed incrementally
- different rollout slices may need different launch composition and checks
- the front door stays stable even as later phases are added

#### Real run modes

`phase05`

- starts the phase `0.5` bringup in the background
- opens keyboard teleop in the current terminal

`smoke`

- starts the same bringup
- runs the passive teleop/perception verifier
- shuts the bringup down afterward

`bringup`

- launches only the phase bringup in the foreground

`teleop`

- launches only the stamped keyboard teleop publisher

`verify`

- runs only the passive verification helper against an existing ROS graph

Suggestions:

- use `run real --phase 0.5` for interactive operator testing
- use `run real --phase 0.5 smoke` for a quick integrated health check
- use `run real --phase 0.5 bringup` when debugging launch or runtime issues

### `check`

Run passive validation commands that inspect an existing graph rather than building
or launching it.

Command:

```bash
scripts/omni check real-perception
```

Use this when:

- the real graph is already running
- you want to confirm the Phase `0.5` teleop/perception boundary topics
- you want a non-driving verification helper

Suggestion:

- pair this with `run real --phase 0.5 smoke` for a quick end-to-end check
- use `OMNISEER_REQUIRE_DETECTIONS=1` when detections must be present for the run to count

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
scripts/omni clean ros
scripts/omni clean vision
scripts/omni clean docs
scripts/omni clean all
```

Use this when:

- a local build tree is stale or confusing
- you want to force a clean rebuild of a specific layer

Suggestion:

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

### Real Phase `0.5` operator check

```bash
scripts/omni run real --phase 0.5 smoke
```

Use this for:

- the quickest integrated Phase `0.5` health check
- validating teleop and perception boundaries before a longer session

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

- `scripts/phase05_real.sh` delegates to `scripts/omni run real --phase 0.5`
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
