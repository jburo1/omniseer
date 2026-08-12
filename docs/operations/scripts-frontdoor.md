# Scripts Front Door

`scripts/omni` is the supported human-facing entrypoint for common Omniseer
setup, build, verification, run, runtime-container, RunBundle, docs, flashing,
environment, and cleanup tasks.

```bash
scripts/omni <command> [subcommand] [args...]
scripts/omni --help
scripts/omni <command> --help
scripts/omni env
scripts/omni doctor
```

Command groups:

| Group | Purpose |
| --- | --- |
| `setup` | Install local development dependencies. |
| `build` | Build ROS, native vision, firmware, and runtime-container artifacts. |
| `runtime` | Build, run, record, verify, push, and pull robot runtime container checkpoints. |
| `test` | Run targeted local verification checks. |
| `run` | Launch sim, real robot profiles, autonomy, monitor, and teleop surfaces. |
| `runs` | Inspect, annotate, report, build videos, list, and retrieve RunBundles. |
| `check` | Passively verify an already running graph. |
| `doctor` | Report local environment and dependency state. |
| `flash` | Run hardware flashing helpers. |
| `docs` | Build documentation and verify diagrams. |
| `clean` | Remove generated local build artifacts. |

## `setup`

Installs dependencies for the portable ROS development path.

```bash
scripts/omni setup ros-deps [package_path ...]
```

With no package paths, installs the default ROS dependency set. Explicit package
paths narrow the install.

Example:

```bash
scripts/omni setup ros-deps
```

## `build`

Builds supported local artifacts through repository-owned wrappers.

```bash
scripts/omni build
scripts/omni build all
scripts/omni build strict
scripts/omni build ros [--with-vision|--without-vision] [--with-yolo] [colcon args...]
scripts/omni build vision
scripts/omni build firmware
scripts/omni build runtime-container [options] [docker build args...]
```

Behavior:

- `build` and `build all`: ROS is required; native vision builds when CMake is
  available; firmware builds when PlatformIO is available and a Teensy is
  attached.
- `build strict`: same product build, but missing native vision or firmware
  toolchains fail the command.
- `build ros`: builds the default ROS package set and auto-includes
  `omniseer_vision_bridge` when RKNN/RGA development files are present.
- `build ros --with-vision`: requires the real-hardware vision bridge.
- `build ros --without-vision`: builds the portable ROS package set.
- `build ros --with-yolo`: also builds optional Python sim YOLO packages;
  `torch` and `ultralytics` must be available.
- `build runtime-container`: builds Docker target `robot-runtime` by default.
  The robot target requires RKNN SDK files.
- `build runtime-container --target portable-runtime`: builds the
  hardware-independent container target.

Runtime-container build options:

```bash
--image <name>                 Default: omniseer/robot-runtime:v2
--target <target>              robot-runtime or portable-runtime
--ros-distro <name>            Default: kilted
--micro-ros-agent-ref <ref>    Default: pinned upstream Kilted commit
--rknn-include <path>          Default: /usr/include/rknn_api.h
--rknn-lib <path>              Auto-detected from ldconfig when omitted
```

Examples:

```bash
scripts/omni build ros --with-yolo
scripts/omni build runtime-container --target portable-runtime --image omniseer/robot-runtime:portable
```

See [Robot Runtime Container](../robot-runtime/robot-runtime-container.md).

## `runtime`

Manages robot-local runtime container checkpoints and evidence runs.

```bash
scripts/omni runtime build [--image <base>] [--tag <tag>] [build args...]
scripts/omni runtime run [--image <base>] [--tag <tag>] [container command...]
scripts/omni runtime record [--image <base>] [--tag <tag>] [record args...] [-- launch args...]
scripts/omni runtime stop --run-id <id> [--time <seconds>]
scripts/omni runtime verify [--image <base>] [--tag <tag>] [--stage smoke|full]
scripts/omni runtime push [--image <base>] [--tag <tag>] [--release-tag <tag>]
scripts/omni runtime pull [--image <base>] [--tag <tag>]
```

Defaults:

- `--image`: `ghcr.io/jburo1/omniseer-robot-runtime`, or
  `OMNISEER_RUNTIME_IMAGE`.
- `runtime build`: creates `robot-candidate-<UTC>-g<shortsha>` when `--tag` is
  omitted and writes metadata under `.omniseer/runtime/`.
- `runtime run`, `record`, `verify`, and `push`: use the latest local runtime
  build metadata when `--tag` is omitted.
- `runtime pull`: defaults to `robot-verified` when `--tag` is omitted.

Record arguments:

```bash
--run-id <id>                         Default: operator_<UTC>
--notes <text>                        Store notes in manifest.yaml.
--classes <text>                      Store configured class names in manifest.yaml.
--system-interval-sec <seconds>       Default: 1.0
--experiment-config <name>            Default: operator-runtime
--experiment-parameter <key=value>    Repeatable; default: stage=manual-operator
```

Behavior and constraints:

- `runtime build` builds the robot hardware target through
  `build runtime-container --target robot-runtime`.
- `runtime record` starts the selected image with the real `operator` profile,
  writes `runs/<run_id>` through the Docker `/runs` mount, and records image
  reference and digest in the manifest.
- Stop recorded runs gracefully with Ctrl-C, monitor Stop Run, or
  `runtime stop --run-id <id>` so telemetry, evidence, `manifest.yaml`,
  `summary.json`, and `logs/bringup.log` are finalized.
- `runtime verify` defaults to `--stage smoke`, a bounded container smoke check
  with hardware-dependent launch components disabled.
- `runtime verify --stage full` runs the real operator smoke path with RunBundle
  recording (`--record-video` and the Rockchip preview encoder), then requires
  `runs inspect --require-complete` to accept the finalized bundle and
  `ffprobe` to identify its non-empty `video/source.ts` as H.264. It writes
  passed metadata only after those checks, and requires the robot runtime
  environment and target hardware dependencies.
- `runtime push` requires a clean git tree, passed full verification for the
  same local image ID, and matching verification/current git commits. It
  promotes to immutable `robot-verified-g<full-commit-sha>`, optional
  `--release-tag <tag>`, and moving `robot-verified`.
- `runtime pull` retrieves a registry image, defaulting to the moving verified
  tag.

Container boundaries:

- The checkpoint path is robot-local and does not request remote CI publishing.
- Runtime Docker commands bind `/dev`, `/run/udev`, host networking, host
  PID/IPC namespaces, and the repository `runs/` directory for hardware access
  and RunBundle persistence.
- Override host bind detection with `OMNISEER_RUNTIME_HOST_REPO_ROOT` or
  `OMNISEER_RUNTIME_RUNS_HOST_ROOT`.
- Set Docker TTY mode with `OMNISEER_RUNTIME_DOCKER_TTY=auto`, `always`, or
  `never`; verification forces `never`.

Examples:

```bash
scripts/omni runtime build
scripts/omni runtime record --run-id operator_001 --classes chair,bottle -- start_lidar:=false
scripts/omni runtime verify --stage full
scripts/omni runtime push --release-tag robot-demo-001
scripts/omni runtime pull
```

See [Robot Runtime Container](../robot-runtime/robot-runtime-container.md#checkpoint-and-promotion).

## `test`

Runs targeted local verification checks.

```bash
scripts/omni test
scripts/omni test ros
scripts/omni test vision
scripts/omni test smoke-sim
```

`test` defaults to `test ros`. Use `test ros` for portable ROS checks,
`test vision` for portable native vision checks, and `test smoke-sim` for the
headless simulation smoke check.

Examples:

```bash
scripts/omni test ros
scripts/omni test smoke-sim
```

## `run`

Launches operator-facing simulation, robot, autonomy, monitor, and teleop
commands. Build explicitly before running when code, launch files, configs, or
generated interfaces changed.

```bash
scripts/omni run sim [--yolo] [--yolo-device <device>] [--yolo-model <path>] [launch args...]
scripts/omni run real [--profile <name>] [recording flags] [--mode <mode>] [mode] [launch args...]
scripts/omni run autonomy --classes <classes> [options] [-- launch args...]
scripts/omni run monitor [monitor args...]
scripts/omni run teleop
```

### `run sim`

Sources ROS and the workspace, then launches `bringup sim.launch.py`.
Remaining arguments are forwarded as ROS launch arguments.

```bash
scripts/omni run sim headless:=true
scripts/omni run sim --yolo --yolo-device cpu
```

`--yolo` starts the optional Python/Ultralytics sim provider and
`/yolo/dbg_image` overlay. Build it first with
`scripts/omni build ros --with-yolo`. `--yolo-device <device>` forwards
`yolo_device:=...`; `--yolo-model <path>` forwards `yolo_model:=...`.

### `run real`

Launches real robot profiles, optionally with RunBundle recording.

Profiles:

| Profile | Behavior |
| --- | --- |
| `current` | Default alias. Currently resolves to `operator`. |
| `operator` | Gateway, native vision, preview, bounded teleop, and recording. |
| `perception` | Native vision and recording without the gateway operator surface. |
| `legacy-teleop` | Diagnostic compatibility profile. |

Modes:

| Mode | Behavior |
| --- | --- |
| `bringup` | Start the selected real profile in the foreground. |
| `smoke` | Start the selected profile, run its verifier, then stop. |
| `verify` | Run the selected profile verifier against an existing ROS graph. |
| `teleop` | Start only the stamped keyboard teleop publisher. |
| `operator` | Legacy-teleop profile only: start bringup, then open keyboard teleop. |

Recording flags:

```bash
--record
--record-run <run_id>
--record-out <path>
--record-system-interval-sec <seconds>
--record-notes <text>
--record-classes <text>
--record-model-family <text>
--record-model-variant <text>
--record-model-precision <text>
--record-model-backend <text>
--record-container-image-ref <ref>
--record-container-image-digest <digest>
--record-experiment-config <text>
--record-experiment-parameters <items>
--record-experiment-parameter <key=value>
--record-comparison-id <text>
--record-trial <text>
--record-workload-id <text>
--record-overwrite
```

Omitting `--profile` selects `current`. Omitting the mode for `operator`,
`perception`, or `current` selects foreground `bringup`. Recording flags require
a mode that launches real bringup. `--record` creates a timestamped run ID;
`--record-run <run_id>` uses the supplied ID; `--record-out` defaults to
`runs/<run_id>`.

Recorded RunBundles include manifest, summary, detections, performance,
available system and native pipeline telemetry, evidence, provenance inputs, and
`logs/bringup.log`. During recorded runs, `OMNISEER_BRINGUP_LOG` is replaced by
the bundle-local log path. Containerized runs can provide provenance through the
record flags or `OMNISEER_CONTAINER_IMAGE_REF`,
`OMNISEER_CONTAINER_IMAGE_DIGEST`, `OMNISEER_GIT_SHA`,
`OMNISEER_EXPERIMENT_CONFIG`, and `OMNISEER_EXPERIMENT_PARAMETERS`.
Use the model flags to record the explicit family, variant, precision, and
backend; they are never inferred from a filename. Recorded real runs also include
the bridge-emitted `provenance/resolved_vision_config.yaml`, which captures the
effective vision parameters after launch overrides.

Examples:

```bash
scripts/omni run real
scripts/omni run real smoke
scripts/omni run real verify
scripts/omni run real --record-run demo_001
scripts/omni run real --profile perception --record-run demo_001
```

### `run autonomy`

Starts the real `operator` profile with bounded target-centering autonomy
enabled from inside the Radxa devcontainer.

```bash
scripts/omni run autonomy --classes <class[,class...]> [options] [-- launch args...]
scripts/omni run autonomy --target <class> [options] [-- launch args...]
scripts/omni run autonomy <class> [options] [-- launch args...]
```

Options:

```bash
--classes <text>                 First class is centered; all classes are evidence candidates.
--target <class>                 One-class compatibility alias.
--run-id <id>                    Default: autonomy_<class>_<UTC>
--out <path>                     Default: runs/<run_id>
--notes <text>                   Store notes in manifest.yaml.
--system-interval-sec <seconds>  Default: 1.0
--evidence-interval-sec <s>      Default: 0.25
--no-overwrite                   Do not replace an existing output directory.
```

The command writes `classes.txt`, starts
`omniseer_autonomy/target_centering_node`, records through the real-run
recording path, and forwards launch arguments after `--`. The bounded behavior is
summarized in [System Architecture](../architecture/overview.md).

```bash
scripts/omni run autonomy --classes chair,backpack,bottle --run-id autonomy_chair_001 -- start_vision:=false
```

### `run monitor`

Launches the laptop Tk operator monitor and forwards arguments to
`robot_monitor_gui`.

```bash
scripts/omni run monitor --host 192.168.1.178 --ssh-user radxa
```

The monitor defaults to robot SSH target `radxa@192.168.1.178`. It starts
robot-side runs over SSH, using either `scripts/omni runtime record` or the
configured devcontainer command template, then retrieves completed bundles and
generates local reports. See [Operator Run Workflow](operator-run-workflow.md).

### `run teleop`

Starts the stamped keyboard teleop publisher.

```bash
scripts/omni run teleop
```

## `runs`

Inspects local RunBundles and retrieves robot-side RunBundles.

```bash
scripts/omni runs inspect <run_dir> [--json] [--require-complete]
scripts/omni runs annotate <run_dir> [--overwrite]
scripts/omni runs report <run_dir> [--overwrite]
scripts/omni runs local-list [--root <local-runs-root>]
scripts/omni runs list [retrieval args...]
scripts/omni runs pull <run_id> [retrieval args...]
```

Retrieval arguments:

```bash
--host <robot-ip>
--user <ssh-user>
--remote-root <robot-runs-root>
--import-root <local-import-root>
--out <local-run-dir>
--overwrite
```

Defaults: `--host 192.168.1.178`, `--user radxa`, and
`--remote-root /home/radxa/apps/omniseer/runs`; override them with
`OMNISEER_ROBOT_HOST`, `OMNISEER_ROBOT_USER`, and
`OMNISEER_ROBOT_RUNS_ROOT`.

`inspect`, `annotate`, `report`, and `local-list` operate on local bundle
directories. `annotate` creates derived annotated evidence without modifying
canonical evidence frames. `report` annotates missing evidence first, then
writes `report/index.html`. `list` and `pull` use SSH and validate pulled
bundles locally.

Examples:

```bash
scripts/omni runs list
scripts/omni runs pull demo_001
scripts/omni runs inspect runs/imported/demo_001
scripts/omni runs report runs/imported/demo_001
```

## `check`

Passively validates an already running graph.

```bash
scripts/omni check
scripts/omni check real-perception
```

`check` defaults to `real-perception`. The verifier checks stamped teleop,
detections, and vision performance topic types, then waits for representative
messages. `OMNISEER_TOPIC_TIMEOUT_SECONDS` controls wait time; default `15`.
`OMNISEER_REQUIRE_DETECTIONS=1` makes missing detections fatal.

```bash
OMNISEER_REQUIRE_DETECTIONS=1 scripts/omni check real-perception
```

## `doctor`

Reports local build, runtime, and robot dependency state.

```bash
scripts/omni doctor
```

Use it to inspect local toolchains, workspace setup, hardware SDK availability,
device visibility, installed ROS packages, and real vision asset paths.

## `flash`

Runs hardware flashing helpers.

```bash
scripts/omni flash teensy
```

This delegates to the headless Teensy flashing helper and changes firmware on
the connected controller.

## `docs`

Builds documentation and verifies tracked diagrams.

```bash
scripts/omni docs build
scripts/omni docs diagrams [--check]
scripts/omni docs check-diagram-links [--site-dir site]
```

`docs build` runs the strict local documentation build. `docs diagrams` renders
D2 sources under `docs/diagrams/` into tracked SVG assets under
`docs/assets/diagrams/`; `--check` verifies freshness. `docs check-diagram-links`
verifies embedded diagram links, defaulting to `site` unless `--site-dir` is
supplied. `OMNISEER_D2_BIN` can point to the pinned D2 renderer.

```bash
scripts/omni docs build
scripts/omni docs diagrams --check
```

## `clean`

Removes generated local build artifacts.

```bash
scripts/omni clean
scripts/omni clean ros
scripts/omni clean vision
scripts/omni clean docs
scripts/omni clean all
```

`clean` defaults to `clean ros`. Targets remove ROS workspace
`build/install/log` trees, native vision build directories, generated docs
outputs, or all of those targets.

## Environment Overrides

Use `scripts/omni env` to see resolved values for the current shell.

| Variable | Effect |
| --- | --- |
| `OMNISEER_ROS_SETUP`, `OMNISEER_WS_SETUP`, `OMNISEER_UROS_WS_SETUP` | Override ROS, workspace, and micro-ROS setup files. |
| `OMNISEER_ROS_DISTRO` | Override the ROS distro used by setup and builds. |
| `OMNISEER_DEFAULT_REAL_PROFILE`, `OMNISEER_CURRENT_REAL_PROFILE` | Override real-profile defaults. |
| `OMNISEER_VISION_PARAMS_FILE` | Override real vision bridge parameter file. |
| `OMNISEER_BRINGUP_DELAY_SEC`, `OMNISEER_BRINGUP_LOG` | Override real-run smoke wait and non-recording log path. |
| `OMNISEER_TOPIC_TIMEOUT_SECONDS`, `OMNISEER_REQUIRE_DETECTIONS` | Override passive check timing and detection requirements. |
| `OMNISEER_ROBOT_HOST`, `OMNISEER_ROBOT_USER`, `OMNISEER_ROBOT_RUNS_ROOT` | Override robot RunBundle retrieval target. |
| `OMNISEER_RUNTIME_IMAGE`, `OMNISEER_RUNTIME_METADATA_DIR` | Override runtime image base and metadata directory. |
| `OMNISEER_RUNTIME_HOST_REPO_ROOT`, `OMNISEER_RUNTIME_RUNS_HOST_ROOT` | Override runtime Docker host bind paths. |
| `OMNISEER_RUNTIME_DOCKER_TTY`, `OMNISEER_RUNTIME_SAFE_SMOKE_SEC` | Override runtime Docker TTY mode and smoke timeout. |
| `OMNISEER_CONTAINER_IMAGE_REF`, `OMNISEER_CONTAINER_IMAGE_DIGEST`, `OMNISEER_EXPERIMENT_CONFIG`, `OMNISEER_EXPERIMENT_PARAMETERS` | Record container and experiment provenance. |
| `OMNISEER_D2_BIN` | Override D2 renderer path for diagram commands. |

## Compatibility Note

`scripts/omni up` and legacy real-profile aliases remain available for older
runbooks. New operator commands should use
`scripts/omni run sim`, `scripts/omni run real --profile operator`, or
`scripts/omni run real --profile perception` directly.
