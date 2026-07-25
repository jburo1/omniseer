# Operator Run Workflow

_Status: local GUI refactor in progress; target-hardware verification pending_

This document explains the operator monitor run workflow after the first refactor
slices. It is meant to make future experiment work obvious: the GUI should collect
operator input and display state; small run modules should own command building,
process control, artifact handling, and validation.

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
   - devcontainer: `scripts/omni run real --profile operator ...`
6. `run_lifecycle.start_remote_run_process(...)` starts the local SSH process.
7. The GUI starts the remote log reader and polls for completion.

## Stop Sequence

1. The GUI calls `RunManager.request_stop(...)`.
2. `run_lifecycle.request_remote_run_stop(...)` writes Ctrl-C to the SSH process
   stdin so the robot-side recorder can finalize the run bundle.
3. The GUI starts a grace timer.
4. If the process does not exit, the GUI calls `RunManager.force_stop(...)`.
5. For the runtime-container backend, the GUI also calls
   `RunManager.request_runtime_stop(...)`, which runs:

   ```bash
   scripts/omni runtime stop --run-id <run_id>
   ```

6. `RunManager.completion(...)` converts the final exit state into a typed
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
