# Operator Run Workflow

This document explains the implemented operator monitor run workflow. The GUI
collects operator input and displays state; small run modules own command
building, process control, artifact handling, and validation.

## Purpose

The operator monitor is the laptop-side interface for remote perception runs. It:

- reads operator run settings from the Tk form
- starts a robot-side run over SSH
- streams the remote run log back into the GUI
- requests graceful stop, then falls back to runtime-container stop when needed
- retrieves the completed run bundle
- generates and opens the local HTML report

The monitor remains laptop-side tooling. Robot behavior belongs to `robot-core`
and the robot-side `scripts/omni runtime record` / `scripts/omni run real`
surfaces.

## Operator Modules

Python modules under `ros_ws/src/robot_diag_control/robot_diag_control`:

| Module | Owns | Should not own |
| --- | --- | --- |
| `monitor_gui.py` | Tk widgets, callbacks, timers, log display, button state application | SSH command construction, subprocess result interpretation, run artifact parsing |
| `run_settings.py` | Converting GUI form values into `RobotConnection`, `RunConfig`, and artifact context | Tk widgets or side effects beyond value normalization |
| `run_commands.py` | Stable command/path builders for remote start, stop, pull, and report | Subprocess execution or GUI state |
| `run_preparation.py` | Remote run directory creation and class-file upload before launch | Long-lived run process lifecycle |
| `run_lifecycle.py` | Process wrappers, process state checks, stop signal mechanics, run-control availability | Robot-specific SSH command construction |
| `run_manager.py` | Start/stop orchestration, runtime-stop fallback, completion interpretation | Tk widgets or artifact report generation |
| `run_artifacts.py` | Pulling run bundles, generating reports, artifact paths, artifact result messages | Tk widgets or robot process lifecycle |

This split keeps command construction, process lifecycle, and artifact handling
separate from Tk widgets.

## Start Sequence

1. `monitor_gui.py` collects current form values.
2. `run_settings.resolve_run_form(...)` returns:
   - `RobotConnection`
   - `RunConfig`
   - `RunArtifactContext`
3. `RunManager.start(...)` calls `prepare_remote_run(...)`.
4. `run_preparation.py` creates the remote run directory and uploads
   `classes.txt` when operator classes are provided.
5. `run_commands.build_remote_start_command(...)` builds the SSH command for the
   selected backend:
   - runtime container: `scripts/omni runtime record`
   - devcontainer: `docker exec` into the running robot devcontainer, then
     `scripts/omni run real --profile operator ...` from the container-visible
     workspace path
6. `run_lifecycle.start_remote_run_process(...)` starts the local SSH process.
7. The GUI starts the remote log reader and polls for completion.

## Stop Sequence

1. The GUI calls `RunManager.request_stop(...)`.
2. `run_lifecycle.request_remote_run_stop(...)` writes Ctrl-C to the SSH process
   stdin so the robot-side recorder can finalize the run bundle.
3. For the runtime-container backend, the GUI also calls
   `RunManager.request_runtime_stop(...)` immediately, which runs:

   ```bash
   scripts/omni runtime stop --run-id <run_id>
   ```

   A successful container stop, including the already-stopped case, moves the GUI
   to `STOPPED` and clears the stale SSH handle so the operator can start the
   next run without waiting on that old session.
4. For non-runtime backends, or if the runtime stop path is unavailable, the GUI
   starts a grace timer and then calls `RunManager.force_stop(...)` if the process
   does not exit.
5. `RunManager.completion(...)` converts the final exit state into a typed
   result:
   - operator-requested stop -> `STOPPED`
   - zero exit -> `STOPPED`
   - nonzero exit -> `FAILED`

The important invariant is that a graceful stop is preferred because the recorder
needs it to write final metadata such as `manifest.yaml` and `summary.json`.

## Artifact Sequence

1. `monitor_gui.py` calls `retrieve_run_artifacts(...)`.
2. `run_artifacts.py` executes the `scripts/omni runs pull <run_id>` command.
3. On success, the GUI calls `generate_run_report(...)`.
4. `run_artifacts.py` executes `scripts/omni runs report <imported_run>`.
5. The GUI opens `runs/imported/<run_id>/report/index.html` when present.

Artifact operations are local laptop actions after the robot run has completed.
They stay separate from robot process lifecycle and do not change robot-side
behavior.

## Detector Tuning

The Run form exposes detector controls that operators can adjust between runs
without changing hardware or model assumptions:

- `Score Threshold` -> `postprocess.score_threshold`
- `NMS IoU` -> `postprocess.nms_iou_threshold`
- `Max Detections` -> `postprocess.max_detections`

The command builder passes these as launch arguments and also records them as
experiment parameters in the run manifest. Operator-started runs also record the
selected `Experiment` dropdown label, either `Perception recording` or
`Autonomy: frame and capture target`, so the HTML report has a clear provenance
label even when no external experiment config file is used. Model paths, camera
device, capture size, model input size, class padding, and runner warmup remain
launch/config-file controls.

## Autonomy Run Type

The monitor can launch either a perception-only recording or the bounded
autonomy run type labeled `Autonomy: frame and capture target` in the
`Experiment` dropdown. The matching command-line helper is:

```bash
scripts/omni run autonomy --classes <class[,class...]>
scripts/omni run autonomy --target <class>
```

The monitor uses the runtime-container recording path for this run type and
appends launch arguments that start `omniseer_autonomy/target_centering_node`,
set the target to the first configured class in the operator `Class List`, pass
the full class list to native vision and run metadata, and write
`autonomy.jsonl` into the run bundle. When target centering reaches success or
failure, the autonomy node stops commanding motion, completes terminal logging
and capture handling, and exits cleanly; real launch then shuts down so the
recorder can finalize the run bundle without an operator Stop Run.

The autonomy behavior performs a bounded in-place visual scan for a configured
target class, acquires a stable detection, centers the target
horizontally using yaw, and uses small bounded forward or reverse motion to bring
the target's bounding-box area into the configured framing range. Forward motion
is blocked when the proximity range crosses the configured safety threshold. This
is bounded visual target acquisition and framing, not navigation-based object
search, global exploration, mapping, or semantic planning.

The node publishes `TwistStamped` commands to `/cmd_vel_autonomy`, relies on
`twist_mux` arbitration, and records terminal state into the bundle for the HTML
report. During scan, the target-centering node subscribes to `/odometry/filtered`
and stops with `scan_complete_no_target` after one observed revolution by default.
Navigation, SLAM, tracking packages, and camera servo control remain out of scope
for this run type.

The robot-side console log, retained in recorded bundles as `logs/bringup.log`,
includes each newly reached autonomy state and a terminal summary with the final
state, reason, states reached, target-loss count, timing milestones, final
centering error, and final confidence. The structured `autonomy.jsonl` stream
remains the canonical machine-readable event record.

## Verification

Local documentation and unit-test coverage can verify behavior that does not
require robot hardware:

- command construction
- form normalization
- pre-launch preparation ordering
- start/stop manager decisions
- runtime-stop fallback message formatting
- completion interpretation
- artifact pull/report result handling
- compact GUI-facing gRPC error messages

Full hardware acceptance requires a real operator run:

1. launch the monitor from the laptop
2. start a runtime-container run on the robot
3. stop it from the GUI
4. verify the run bundle finalizes
5. retrieve it
6. generate and open the report

For the operator profile, a robot-side acceptance check can be run against an
existing graph:

```bash
OMNISEER_REQUIRE_DETECTIONS=1 scripts/omni run real verify
```

This check requires live encoder, wheel-odometry, LiDAR, vision, and detection
messages; healthy gateway status; and zero vision errors. It also starts and
stops preview, runs a short headless overlay viewer smoke when laptop-side
dependencies are available, enables teleop, sends only a zero command, verifies
that the stamped controller reference receives it, and disables teleop. This is
an acceptance check for the operator profile, not a substitute for recorded
hardware run evidence.
