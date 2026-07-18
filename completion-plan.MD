# Omniseer Completion Plan: Edge-to-Cloud Perception Artifact

This document is the guiding plan for turning Omniseer into a complete, reviewable portfolio artifact.

The goal is not to add random robot features. The goal is to make the existing edge-perception system observable, reproducible, and easy for an engineering reviewer to understand.

## North Star

A reviewer should be able to clone the repository, read the README, open one hosted or local report, and understand:

> Omniseer deploys open-vocabulary perception on constrained robot hardware, records detections and runtime telemetry, and produces a reproducible experiment report suitable for laptop or cloud review.

The project should demonstrate the full loop:

```text
camera
  -> native V4L2/RGA/RKNN YOLO-World runtime
  -> ROS 2 detections and performance telemetry
  -> teleoperated real-robot verification
  -> structured experiment bundle
  -> local laptop report
  -> optional hosted/cloud review
```

## Critical Sequencing Principle

Do not build experiment scaffolding before the real teleop + perception path is proven.

The correct order is:

```text
clean public signal
  -> verify teleop + native YOLO detections + perf topics together
  -> record that known-good behavior
  -> analyze/report that behavior
  -> publish selected evidence
```

The recorder should be boring. It should not be where camera paths, model paths, class paths, command-topic mismatches, or missing launch integration are discovered.

## Scope Discipline

### In scope

- Reproducible edge-perception experiments.
- Real teleop + native RKNN YOLO-World verification.
- Recording `/yolo/detections` and `/vision/perf`.
- Capturing run metadata, timing, detection counts, and selected evidence.
- Generating a static local report from a recorded run.
- Publishing a boring hosted version of the report after local review works.
- Clearly documenting what is CI-verified, hardware-verified, and still planned.

### Out of scope for this completion plan

- Autonomous frontier exploration.
- Autonomous semantic seek-and-capture.
- Visual servoing.
- Grasping or physical object capture.
- Full fleet management.
- Browser-native live robot control.
- Cloud provider complexity before the local run bundle contract is stable.

Do not expand scope unless the current phase definition of done is already satisfied.

---

# Phase 1: Clean the Public Engineering Signal

## Purpose

Before adding features, make the repository look disciplined. A reviewer should not see avoidable red checks, inconsistent tooling, or unclear verification claims.

## Tasks

### 1.1 Align lint tooling

Ensure Ruff and pre-commit versions are consistent across:

- `pyproject.toml`
- `.pre-commit-config.yaml`
- `.github/workflows/ci.yml`
- local developer documentation

Recommended target: keep CI and `pyproject.toml` on the current Ruff version, then update pre-commit to match.

### 1.2 Restore green checks

All visible public checks should pass on the default branch.

Minimum checks:

- GitHub Actions `ci`
- docs build
- pre-commit.ci, if enabled

If a check is not intended to be authoritative, either fix it, disable it, or document why it is non-blocking. Do not leave unexplained red status.

### 1.3 Add verification-boundary documentation

Create or update a concise document such as:

```text
docs/evidence.md
```

The document should distinguish:

- CI-verified behavior
- locally/hardware-verified behavior
- unverified/planned behavior

It should avoid overstating hardware, RKNN, RGA, NPU, sensor, firmware-flash, or cloud-dashboard claims.

### 1.4 Add focused GitHub issues or TODO entries

Create a small visible backlog:

- `P1: fix pre-commit/CI consistency`
- `P2: verify real teleop + native YOLO detections`
- `P4: perception run bundle v0`
- `P5: laptop report generator`
- `P7: target hardware evidence run`
- `P8: hosted static report`

## Definition of Done

- Default branch has no avoidable red public status checks.
- Lint tooling is version-consistent.
- README and docs do not imply that CI proves hardware execution.
- The next implementation work is represented as explicit issues or checklist items.

## Suggested commit names

```text
fix(lint): align pre-commit ruff with CI
docs(evidence): document verification boundaries
chore(project): add completion backlog for perception artifact
```

---

# Phase 2: Real Teleop + Native Perception Verification

## Purpose

Prove the minimum integrated robot behavior before building experiment infrastructure.

The current codebase has substantial pieces:

- real bringup structure
- teleop command path through `twist_mux`
- native `omniseer_vision_bridge`
- `/yolo/detections`
- `/vision/perf`

But the integrated real slice must be made explicit and verified.

The required behavior is:

```text
operator keyboard teleop
  -> /cmd_vel_keyboard
  -> twist_mux
  -> /mecanum_drive_controller/reference
  -> MCU/motor command path

camera
  -> native V4L2/RGA/RKNN bridge
  -> /yolo/detections
  -> /vision/perf
```

This phase is complete only when both paths work at the same time on the real robot.

## Why this comes before the recorder

The experiment recorder should only record known-good topics.

If the recorder is built first, it may hide or confuse more important integration failures:

- camera device path wrong
- RKNN/RGA model paths unset
- class list path unset
- native bridge not launched
- topic type mismatch in teleop
- `/vision/perf` not publishing
- `/yolo/detections` not publishing
- real launch path not including the vision bridge

Do not start Phase 4 until this phase has a documented manual verification pass.

## Tasks

### 2.1 Add a real vision launch file

Add:

```text
ros_ws/src/bringup/launch/real_vision.launch.py
```

It should launch:

```text
package: omniseer_vision_bridge
executable: vision_bridge_node
```

Expose launch arguments for:

```text
camera.device
camera.width
camera.height
camera.buffer_count
pipeline.dst_width
pipeline.dst_height
models.detector_model_path
models.clip_model_path
models.clip_vocab_path
classes.path
classes.pad_token
producer.preflight_capture_wait_ms
runner.warmup_runs
postprocess.score_threshold
postprocess.nms_iou_threshold
postprocess.max_detections
frames.camera_frame_id
```

Do not bury these as magic constants. The paths should be reviewable from a config file or launch arguments.

### 2.2 Add a real vision config file

Add:

```text
ros_ws/src/bringup/config/vision_bridge.real.yaml
```

Minimum shape:

```yaml
vision_bridge:
  ros__parameters:
    camera.device: /dev/video12
    camera.width: 1280
    camera.height: 720
    camera.buffer_count: 4

    pipeline.dst_width: 640
    pipeline.dst_height: 640

    models.detector_model_path: ""
    models.clip_model_path: ""
    models.clip_vocab_path: ""

    classes.path: ""
    classes.pad_token: "nothing"

    producer.preflight_capture_wait_ms: 200
    runner.warmup_runs: 0

    postprocess.score_threshold: 0.25
    postprocess.nms_iou_threshold: 0.45
    postprocess.max_detections: 100

    frames.camera_frame_id: camera_optical_frame
```

Model and class paths may initially be empty in committed config if they are machine-local, but the verification document must show the concrete command or override used on the target robot.

### 2.3 Wire vision into real bringup

Add launch arguments to `real.launch.py`:

```text
start_vision:=true
vision_params_file:=vision_bridge.real.yaml
```

Then conditionally include `real_vision.launch.py`.

The native vision bridge belongs in the real hardware layer, not the common layer, because it depends on real camera/RGA/RKNN hardware.

### 2.4 Verify teleop command type compatibility

Check the command path carefully.

The intended path is:

```text
teleop_twist_keyboard
  -> /cmd_vel_keyboard
  -> twist_mux
  -> /mecanum_drive_controller/reference
```

Run:

```bash
ros2 topic type /cmd_vel_keyboard
ros2 topic type /mecanum_drive_controller/reference
```

If `/cmd_vel_keyboard` is `geometry_msgs/msg/Twist` but `twist_mux` expects stamped input, fix the mismatch.

Acceptable fixes:

- configure teleop to publish `geometry_msgs/msg/TwistStamped`, if supported
- add a small stamped command adapter
- change the mux config only if the rest of the command path remains correct

Do not proceed with a hidden command type mismatch.

### 2.5 Add a manual verification checklist

Add:

```text
docs/software/real_teleop_perception_check.md
```

It should include exact commands and expected observations.

Minimum command sequence:

```bash
ros2 launch bringup real.launch.py \
  start_nav:=false \
  start_slam:=false \
  start_rf2o:=false \
  start_gateway:=false \
  start_vision:=true \
  wait_for_boundary_topics:=false
```

In another terminal:

```bash
bash ros_ws/src/scripts/teleop.sh
```

Verification commands:

```bash
ros2 topic list

ros2 topic type /cmd_vel_keyboard
ros2 topic type /mecanum_drive_controller/reference
ros2 topic echo --once /mecanum_drive_controller/reference

ros2 topic type /yolo/detections
ros2 topic echo --once /yolo/detections
ros2 topic hz /yolo/detections

ros2 topic type /vision/perf
ros2 topic echo --once /vision/perf
ros2 topic hz /vision/perf
```

The doc should include a short troubleshooting table for:

- missing camera device
- missing RGA/RKNN libraries
- missing model path
- missing class list
- no detections despite `/vision/perf`
- teleop command topic type mismatch
- bridge fatal error

### 2.6 Optional smoke script

Add an optional shell script:

```text
scripts/check_real_teleop_perception.sh
```

It should not move the robot. It should only check topic presence/types/rates.

Example checks:

```text
/yolo/detections exists
/vision/perf exists
/mecanum_drive_controller/reference exists
/vision/perf publishes at least one message
```

Do not make this a hard GitHub CI requirement because it depends on target hardware.

## Definition of Done

- Real bringup can optionally start the native vision bridge.
- The robot can be teleoperated while `/yolo/detections` and `/vision/perf` publish.
- Topic types for teleop and motor command path are confirmed.
- A manual verification document exists with exact commands.
- At least one target-hardware verification pass has been performed and noted.

## Suggested commit names

```text
feat(bringup): add real vision bridge launch
feat(bringup): wire optional vision bridge into real bringup
fix(teleop): align keyboard command type with twist mux
docs(bringup): add real teleop perception verification checklist
chore(scripts): add real perception topic check helper
```

---

# Phase 3: Operator-Integrated Real Demo

## Purpose

Prove that the verified robot-side teleop/perception slice can be driven from
one laptop-facing workflow before adding experiment recording.

The required behavior is:

```text
laptop monitor GUI
  -> gateway status
  -> preview control + external SRT viewer
  -> bounded gateway teleop
  -> /cmd_vel_keyboard
  -> /mecanum_drive_controller/reference

native perception
  -> /vision/perf
  -> gateway vision status
  -> laptop monitor
```

This phase exists to make Phase 4 boring: the recorder should capture a known
good integrated operator workflow, not discover gateway, preview, teleop, or
visibility failures.

## Tasks

### 3.1 Add bounded gateway teleop

Extend the operator gateway API additively with teleop enable/disable, bounded
command submission, and teleop status in `SystemStatus`.

Keep ROS topic names out of the external contract. The C++ gateway should own
the internal adapter to the existing stamped command path.

Minimum safety behavior:

- teleop disabled by default
- explicit enable required before motion commands
- configured linear/angular bounds
- command rate limiting
- deadman timeout
- zero command on disable, timeout, and gateway shutdown

### 3.2 Upgrade the laptop monitor GUI

Use the existing Tk monitor as the Phase 3 laptop app.

It should provide:

- status polling for health, preview, vision, and teleop
- preview on/off/profile controls using the existing external SRT viewer
- bounded teleop enable/disable/stop and directional controls
- best-effort teleop disable when the app closes

Do not embed video in the Tk window in this phase. Keep the external
GStreamer viewer for lower risk.

### 3.3 Add an operator-integrated demo checklist

Add:

```text
docs/software/operator_integrated_demo.md
```

It should include exact robot and laptop commands, expected observations,
troubleshooting, and a target-hardware verification record.

## Definition of Done

- Gateway status includes teleop state and fresh vision status.
- The laptop monitor can enable preview and launch the external viewer.
- The laptop monitor can enable, command, stop, and disable bounded teleop.
- Teleop commands from the laptop reach `/mecanum_drive_controller/reference`.
- Closing the laptop app or disabling teleop sends a zero command.
- A target-hardware Phase 3 verification pass has been recorded.

## Suggested commit names

```text
feat(gateway): add bounded teleop control
feat(operator): add teleop controls to monitor gui
docs(operator): add integrated real demo checklist
docs(plan): insert phase 3 before run recording
```

---

# Phase 4: Structured Perception Run Bundle

## Purpose

Implement the missing local experiment artifact after Phase 3 proves the
operator-integrated source workflow.

The system should be able to record one verified perception run into a structured directory that can later be analyzed, copied, uploaded, or rendered.

## Entry Criteria

Do not begin this phase until:

- real teleop command path has been verified
- native bridge publishes `/yolo/detections`
- native bridge publishes `/vision/perf`
- teleop and perception can run at the same time
- `docs/software/real_teleop_perception_check.md` exists
- gateway-driven preview and bounded teleop have been verified from the laptop monitor
- `docs/software/operator_integrated_demo.md` exists

## Proposed package

Create a focused ROS package:

```text
ros_ws/src/omniseer_experiments/
```

This package should own experiment recording and run-bundle creation. It should not own native vision inference or robot control.

## Inputs

The first recorder version should subscribe to:

```text
/yolo/detections
/vision/perf
```

Optional later inputs:

```text
/battery
/tf
/odom or /mecanum_drive_controller/odometry
/operator_markers
```

Do not block Phase 4 on optional inputs.

## Output directory contract

A completed run should produce:

```text
runs/<run_id>/
  manifest.yaml
  detections.jsonl
  perf.jsonl
  summary.json
  evidence/
```

Optional later additions:

```text
runs/<run_id>/
  system.jsonl
  rosbag/
  notes.md
  failures.jsonl
  evidence/
    frames/
    crops/
    thumbnails/
```

## `manifest.yaml` contract

Minimum fields:

```yaml
schema_version: 1
run_id: demo_001
project: omniseer
robot: omniseer
sbc: ROCK 5B+
started_at: "2026-07-06T19:00:00-07:00"
ended_at: null
git_sha: "<current commit sha>"
ros_distro: kilted
model:
  detector: "yolo-world-rknn"
  detector_model_path: "<path or unset>"
  clip_model_path: "<path or unset>"
classes:
  - chair
  - backpack
  - fire_extinguisher
topics:
  detections: /yolo/detections
  perf: /vision/perf
verified_prerequisite:
  real_teleop_perception_check: true
notes: ""
```

Avoid inventing fields that are not useful yet. Keep the schema stable and boring.

## `detections.jsonl` contract

Each line should represent one received detection message or one detection event.

Recommended shape:

```json
{
  "schema_version": 1,
  "topic": "/yolo/detections",
  "recv_ts_ns": 0,
  "header_stamp": {"sec": 0, "nanosec": 0},
  "frame_id": "camera_frame",
  "detections": [
    {
      "class_id": 0,
      "class_name": "chair",
      "score": 0.82,
      "bbox": {
        "center_x": 512.0,
        "center_y": 300.0,
        "size_x": 120.0,
        "size_y": 180.0
      }
    }
  ]
}
```

## `perf.jsonl` contract

Each line should represent one `/vision/perf` message.

Recommended shape:

```json
{
  "schema_version": 1,
  "topic": "/vision/perf",
  "recv_ts_ns": 0,
  "header_stamp": {"sec": 0, "nanosec": 0},
  "producer_fps": 0.0,
  "consumer_fps": 0.0,
  "last_preprocess_ms": 0.0,
  "last_infer_ms": 0.0,
  "last_postprocess_ms": 0.0,
  "last_publish_ms": 0.0,
  "last_producer_total_ms": 0.0,
  "last_consumer_total_ms": 0.0,
  "produced_count": 0,
  "consumed_count": 0,
  "error_counts": {
    "no_writable_buffer": 0,
    "capture_retryable": 0,
    "capture_fatal": 0,
    "preprocess": 0,
    "infer": 0
  }
}
```

## CLI

Provide a command similar to:

```bash
ros2 run omniseer_experiments record_run \
  --run-id demo_001 \
  --classes chair backpack fire_extinguisher \
  --out runs/demo_001
```

Useful flags:

```text
--duration-sec <seconds>
--notes <text>
--overwrite
--include-rosbag false
```

Keep v0 simple. It is acceptable for v0 to stop on Ctrl-C and finalize the manifest/summary on shutdown.

## Summary generation

At the end of a run, generate:

```text
summary.json
```

Minimum fields:

```json
{
  "schema_version": 1,
  "run_id": "demo_001",
  "duration_sec": 60.0,
  "message_counts": {
    "detections": 100,
    "perf": 120
  },
  "detections_by_class": {
    "chair": 45,
    "backpack": 12
  },
  "confidence_by_class": {
    "chair": {"min": 0.52, "mean": 0.76, "max": 0.94}
  },
  "perf": {
    "producer_fps_mean": 0.0,
    "consumer_fps_mean": 0.0,
    "infer_ms_mean": 0.0,
    "infer_ms_p95": 0.0
  },
  "errors": {
    "capture_fatal": 0,
    "preprocess": 0,
    "infer": 0
  }
}
```

## Tests

Add tests that validate:

- run directory creation
- manifest schema
- JSONL writing
- summary generation from synthetic messages/data
- graceful shutdown/finalization
- no crash on empty run

Prefer pure Python tests for schema/summary logic. ROS integration tests can come later.

## Definition of Done

- A user can record a run from known-good ROS topics into `runs/<run_id>/`.
- The run contains `manifest.yaml`, `detections.jsonl`, `perf.jsonl`, and `summary.json`.
- Tests cover schema and summary behavior.
- README or docs include one reproducible command sequence.
- The docs make clear that Phase 3 must be completed before meaningful real runs are recorded.

## Suggested commit names

```text
feat(experiments): add perception run manifest schema
feat(experiments): record detections and perf jsonl
feat(experiments): generate run summary json
test(experiments): cover run bundle creation and summaries
docs(experiments): document local perception recording flow
```

---

# Phase 5: Local Laptop Report Generator

## Purpose

Turn a recorded run bundle into a human-readable report. This is what makes the project easy to review.

The report should answer:

1. Did the model run on the robot?
2. What latency and FPS did it achieve?
3. What classes did it detect?
4. What evidence supports success and failure?
5. What are the known limitations?

## Proposed location

Use the existing analysis area, or create a focused module:

```text
ros_ws/src/analysis/
```

or:

```text
tools/report_perception_run.py
```

Prefer the simplest location that fits the repository conventions.

## CLI

Provide:

```bash
python3 -m analysis.report_perception_run runs/demo_001 --out reports/demo_001
```

or:

```bash
ros2 run analysis report_perception_run runs/demo_001 --out reports/demo_001
```

## Output contract

```text
reports/<run_id>/
  index.html
  summary.json
  latency.png
  throughput.png
  detections_by_class.png
  confidence_by_class.png
  assets/
```

For v0, static HTML is enough. Do not build a web app.

## Report sections

The generated `index.html` should include:

### Run metadata

- run ID
- timestamp
- git SHA
- model information
- selected classes
- robot/hardware notes
- whether Phase 2 verification had been completed

### Performance

- producer FPS
- consumer FPS
- inference latency
- preprocess latency
- postprocess latency
- publish latency
- error counters

### Detection summary

- detections by class
- confidence statistics
- total detection messages
- first/last detection timestamps

### Evidence

For v0, it is acceptable to show:

- placeholder section if no frames/crops exist
- manually copied images from `evidence/`
- links to evidence files

Do not block report generation on frame capture.

### Known limitations

The report should state whether the run lacks:

- annotated frames
- ground truth
- resource telemetry
- hardware temperature
- cloud sync
- runtime class update support

This honesty is a strength.

## Plotting

Use simple Matplotlib plots:

- inference latency over time
- producer/consumer FPS over time
- detections by class
- confidence distribution by class

Do not over-style. The report should look clear, not fancy.

## Tests

Add tests for:

- loading a run bundle
- generating summary plots from synthetic JSONL
- generating `index.html`
- handling missing optional evidence
- handling empty detections

## Definition of Done

- A recorded run can be converted into a static local report.
- The report can be opened in a browser without a server.
- Missing optional evidence does not crash the generator.
- The report clearly distinguishes measured output from planned work.

## Suggested commit names

```text
feat(analysis): load perception run bundles
feat(analysis): render static perception report
feat(analysis): add latency and detection plots
test(analysis): cover report generation edge cases
docs(portfolio): add local report workflow
```

---

# Phase 6: System Resource Telemetry

## Purpose

Make the edge-deployment story stronger by recording system load alongside perception performance.

This phase should not modify the native hot path unless absolutely necessary.

## Design rule

Sample system resource telemetry outside the vision producer/consumer loops.

The resource sampler may run in the experiment recorder process or as a separate optional node.

## Data to capture

Minimum useful fields:

```text
timestamp
cpu_percent
memory_used_mb
memory_available_mb
soc_temp_c
```

Optional fields:

```text
process_rss_mb
battery_voltage
battery_percent
npu/rknn-specific counters if easily exposed
disk_free_mb
```

## Output

Add:

```text
runs/<run_id>/system.jsonl
```

Example line:

```json
{
  "schema_version": 1,
  "source": "system",
  "recv_ts_ns": 0,
  "cpu_percent": 38.2,
  "memory_used_mb": 812.0,
  "memory_available_mb": 7200.0,
  "soc_temp_c": 61.4
}
```

## Implementation guidance

Prefer simple Linux sources:

- `/proc/stat`
- `/proc/meminfo`
- `/sys/class/thermal/...`
- existing battery topic, if already available

Avoid heavy dependencies. Avoid polling too fast.

Recommended default sample rate:

```text
1 Hz
```

## Report integration

Add a system section to the report:

- CPU over time
- memory over time
- temperature over time
- any obvious throttling or thermal concerns

## Tests

Add tests for:

- parsing `/proc`-like sample fixtures
- missing temperature path
- missing battery source
- system JSONL writing
- report behavior when `system.jsonl` is absent

## Definition of Done

- Run bundles may include `system.jsonl`.
- Report generator displays resource telemetry when present.
- Missing system telemetry is treated as optional, not fatal.
- No high-cost system sampling occurs in the vision hot path.

## Suggested commit names

```text
feat(experiments): add optional system resource sampler
feat(analysis): include resource telemetry in reports
test(experiments): cover system sampler fallbacks
docs(experiments): document resource telemetry boundary
```

---

# Phase 7: Hardware Evidence Pack

## Purpose

Turn the software artifact into credible engineering evidence.

This phase produces real runs and documentation. It may not require much new code, but it is essential for hiring signal.

## Required runs

Create at least three real hardware runs.

### Run A: Static scene

Known objects placed in front of the robot.

Goal:

- show that selected classes are detected
- measure stable inference behavior
- collect clean screenshots/evidence

### Run B: Teleoperated pass

Drive the robot through an indoor scene.

Goal:

- show perception under motion and changing viewpoint
- measure latency/FPS during realistic operation
- collect representative successes and misses

### Run C: Negative/confusing scene

Use absent classes or visually confusing objects.

Goal:

- show false positives and limitations
- demonstrate engineering honesty
- create useful failure analysis

## Evidence directory

For each run:

```text
runs/<run_id>/evidence/
  README.md
  selected_frames/
  crops/
  screenshots/
```

If frame/crop extraction is not automated yet, manually selected evidence is acceptable for the first evidence pack. Be explicit when evidence is manually selected.

## Per-run interpretation

Each run should include:

```text
runs/<run_id>/notes.md
```

Template:

```markdown
# Run Notes: <run_id>

## Setup

- Location:
- Lighting:
- Classes:
- Robot state:
- Model/config:

## What worked

## What failed

## Representative evidence

## Metrics summary

## Follow-up fixes
```

## Documentation

Add a portfolio/demo page:

```text
docs/software/perception_demo.md
```

It should link to or describe:

- real teleop perception verification
- run commands
- generated reports
- hardware setup
- exact model/class configuration
- limitations

## Definition of Done

- Three real run bundles exist locally.
- At least one report is suitable to publish or show in an interview.
- The docs include an evidence-backed demo walkthrough.
- The repo clearly states which results were produced on target hardware.

## Suggested commit names

```text
docs(demo): add static-scene hardware run notes
docs(demo): add teleop perception run evidence
docs(demo): add negative-case failure analysis
docs(portfolio): publish perception demo walkthrough
```

---

# Phase 8: Hosted Static Review

## Purpose

Make the artifact easy to inspect without requiring a local ROS setup.

This phase should be boring. Prefer static files over a dynamic dashboard.

## Recommended approach

Use generated static HTML reports and publish them through the existing documentation/GitHub Pages path.

Potential output:

```text
docs/reports/demo_001/
  index.html
  latency.png
  throughput.png
  detections_by_class.png
```

or:

```text
site/reports/demo_001/
```

Choose the path that best fits the existing MkDocs flow.

## Do not overbuild

Avoid:

- login/auth
- databases
- cloud queues
- streaming dashboards
- live robot control
- provider-specific infrastructure
- frontend framework complexity

The hosted artifact only needs to prove that completed run bundles can be reviewed offboard.

## Optional sync command

A simple provider-neutral command may be added:

```bash
python3 -m analysis.export_report reports/demo_001 --target docs
```

or:

```bash
python3 -m analysis.sync_run runs/demo_001 --target local-pages
```

For v0, copying static files into a docs-published location is enough.

## Definition of Done

- One completed perception report is viewable through the hosted docs site or equivalent static hosting path.
- The hosted page clearly links back to the source repository and relevant run metadata.
- The report does not imply live cloud operation.
- The README links to the hosted demo report.

## Suggested commit names

```text
feat(analysis): export static reports for docs hosting
docs(demo): add hosted perception report
docs(readme): link portfolio demo report
```

---

# Cross-Cutting Engineering Rules

## Rule 1: Small commits

Each commit should do one thing.

Good:

```text
feat(experiments): add run manifest writer
test(experiments): validate run manifest schema
docs(experiments): document record_run usage
```

Bad:

```text
big update
more stuff
dashboard and recorder and cleanup
```

## Rule 2: Update docs with behavior changes

If a command, output schema, topic contract, or verification claim changes, update docs in the same PR/commit series.

## Rule 3: Prefer boring file formats

Use:

- YAML for run manifest
- JSONL for time-series events
- JSON for summaries
- static HTML for reports

Avoid premature databases.

## Rule 4: Preserve hot-path isolation

Do not put expensive logging, plotting, system polling, file I/O, or cloud upload behavior into the native vision producer/consumer hot path.

## Rule 5: Be explicit about planned versus implemented

Every public-facing doc should distinguish:

- implemented
- CI-verified
- hardware-verified
- planned
- deferred

This prevents credibility damage.

## Rule 6: Do not chase autonomy yet

Autonomous semantic search and capture are deferred. Do not work on them until the perception experiment artifact is complete.

## Rule 7: Every phase needs a demo command

Each phase should add or update at least one command that a reviewer can run or understand.

## Rule 8: Do not record unverified topics as evidence

If `/yolo/detections`, `/vision/perf`, or the teleop command path has not been verified in the current real bringup configuration, do not produce portfolio evidence from it.

---

# Final Portfolio Acceptance Criteria

The project is portfolio-complete when all of the following are true:

## Repository presentation

- README states the active deliverable clearly.
- README links to docs and at least one demo report.
- CI and docs checks are green.
- Verification boundaries are documented.

## Real teleop perception

- Real bringup can start the native vision bridge.
- Teleop command path has been verified.
- `/yolo/detections` publishes on the real robot.
- `/vision/perf` publishes on the real robot.
- The verification commands and observations are documented.

## Experiment recording

- A perception run can be recorded into a structured run bundle.
- The bundle includes manifest, detections, perf telemetry, and summary.
- Optional system telemetry is supported.

## Report generation

- A static report can be generated from a run bundle.
- The report includes performance plots and detection summaries.
- Missing optional evidence does not crash the report generator.

## Hardware evidence

- At least one real hardware run is documented.
- The report includes real measured latency/FPS numbers.
- Known limitations and failure cases are visible.

## Hosted review

- At least one report is viewable from the hosted documentation path.
- The hosted report is linked from the README.
- The hosted report is static and reproducible from repository commands.

---

# Recommended Immediate Next Step

Start with Phase 1, then Phase 2.

Do not start with cloud. Do not start with autonomy. Do not start with UI polish. Do not start with the recorder before the real source topics are verified.

Start here:

```text
fix(lint): align pre-commit ruff with CI
```

Then:

```text
feat(bringup): add real vision bridge launch
```

Then:

```text
docs(bringup): add real teleop perception verification checklist
```

The artifact becomes compelling when the first real teleop perception run is verified, then recorded.
