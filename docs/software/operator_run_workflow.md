# Operator Run Workflow

_Status: local GUI refactor in progress; target-hardware verification pending_

This document explains the operator monitor run workflow. The GUI collects operator
input and displays state; small run modules own command building, process control,
artifact handling, and validation.

## Purpose

The operator monitor is the laptop-side interface for remote perception runs. It:

- reads operator run settings from the Tk form
- starts a robot-side run over SSH
- streams the remote run log back into the GUI
- requests graceful stop, then falls back to runtime-container stop when needed
- retrieves the completed run bundle
- generates and opens the local HTML report

The monitor remains laptop-side tooling. Robot behavior still belongs to
`robot-core` and the robot-side `scripts/omni runtime record` / `run real`
surfaces.

## Ownership

Current Python modules under `robot_diag_control`:

| Module | Owns | Should not own |
| --- | --- | --- |
| `monitor_gui.py` | Tk widgets, callbacks, timers, log display, button state application | SSH command construction, subprocess result interpretation, run artifact parsing |
| `run_settings.py` | Converting GUI form values into `RobotConnection`, `RunConfig`, and artifact context | Tk widgets or side effects beyond value normalization |
| `run_commands.py` | Stable command/path builders for remote start, stop, pull, and report | Subprocess execution or GUI state |
| `run_preparation.py` | Remote run directory creation and class-file upload before launch | Long-lived run process lifecycle |
| `run_lifecycle.py` | Process wrappers, process state checks, stop signal mechanics, run-control availability | Robot-specific SSH command construction |
| `run_manager.py` | Start/stop orchestration, runtime-stop fallback, completion interpretation | Tk widgets or artifact report generation |
| `run_artifacts.py` | Pulling run bundles, generating reports, artifact paths, artifact result messages | Tk widgets or robot process lifecycle |

This split keeps the GUI readable and makes the operational decisions unit
testable without launching Tk.

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
They should stay separate from robot process lifecycle.

## Adding Experiment Behavior

Use this rule of thumb:

- new form value or default -> `run_settings.py`
- new SSH/runtime command argument -> `run_commands.py`
- new pre-launch file or directory setup -> `run_preparation.py`
- new run start/stop policy -> `run_manager.py`
- new process signal or process-state primitive -> `run_lifecycle.py`
- new retrieve/report artifact behavior -> `run_artifacts.py`
- new button, field, timer, or display text -> `monitor_gui.py`

Prefer additive changes to `RunConfig` and command builders. Do not make the GUI
assemble shell strings directly.

## Autonomy Run Type

The monitor can launch either a perception-only recording or the first bounded
autonomy experiment, `Autonomy: center first class`. The autonomy mode still uses
the runtime-container recording path, but appends launch arguments that start
`omniseer_autonomy/target_centering_node`, set the target to the first configured
class, and write `autonomy.jsonl` into the run bundle.

The v1 autonomy behavior is yaw-only: it publishes `TwistStamped` commands to
`/cmd_vel_autonomy`, relies on `twist_mux` arbitration, and records terminal
state into the bundle for the HTML report. During scan, the target-centering node
subscribes to `/odometry/filtered` and stops with `scan_complete_no_target` after
one observed revolution by default. Translation, navigation, SLAM, tracking
packages, and camera servo control remain out of scope for this run type.

## Verification

Current local coverage focuses on behavior that does not require robot hardware:

- command construction
- form normalization
- pre-launch preparation ordering
- start/stop manager decisions
- runtime-stop fallback message formatting
- completion interpretation
- artifact pull/report result handling
- compact GUI-facing gRPC error messages

Hardware verification is still required for the full path:

1. launch the monitor from the laptop
2. start a runtime-container run on the robot
3. stop it from the GUI
4. verify the run bundle finalizes
5. retrieve it
6. generate and open the report
