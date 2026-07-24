# Edge-to-Cloud Perception

_Status: active portfolio direction_

## Purpose

Define a focused edge-to-cloud ML artifact around the perception system already
implemented on the robot.

The goal is not autonomous object search. The goal is to demonstrate that an
open-vocabulary model can be deployed on constrained edge hardware, integrated with
ROS 2, measured under real operating conditions, and reviewed through a reproducible
offboard experiment workflow.

## Active Deliverable

An operator selects semantic classes, teleoperates the robot through a scene, and
records a perception experiment. YOLO-World runs on the ROCK 5B+ NPU. The resulting
detections, stage timings, performance summaries, and selected visual evidence form a
run bundle that can be reviewed on a laptop and later synchronized to a cloud-hosted
report.

```text
camera
  -> native V4L2/RGA/RKNN runtime
  -> /yolo/detections + /vision/perf
  -> experiment recorder
  -> local run bundle
  -> laptop review
  -> planned cloud sync and hosted report
```

Autonomous navigation, semantic search, viewpoint planning, and object capture are not
required for this deliverable.

## Current Implementation

### Edge inference

**Implemented with target-hardware run evidence:** the native runtime captures
NV12 frames, preprocesses them with RGA, prepares YOLO-World text embeddings, runs
RKNN detector inference, maps detections back to source coordinates, and publishes
bounded results. The local `runs/pipeline_001` bundle records detections,
performance summaries, system telemetry, and native pipeline telemetry from a
Phase 3 target run. The integrated real teleop plus operator preview verification
record is still pending.

The native bridge currently receives classes from `classes.path` during startup.
Changing classes requires restarting the bridge with a different class list. The
Python `yolo_ros` integration has a `SetClasses` service, but that service does not
reconfigure the native RKNN bridge.

### ROS contracts

**Implemented:** the native bridge publishes:

- `/yolo/detections` as `yolo_msgs/msg/DetectionArray`
- `/vision/perf` as `omniseer_msgs/msg/VisionPerfSummary`

The performance message reports producer and consumer rates, recent stage timings,
source age, processed counts, and pipeline error counters. It intentionally stays
focused on vision pipeline metrics; live CPU, memory, temperature, network, and power
diagnostics are surfaced separately through the robot gateway status snapshot.

### Local observability

**Implemented:** the native harness supports an annotated OpenCV preview, JSONL stage
telemetry, rolling statistics, and offline telemetry analysis. The ROS bridge is
headless and publishes detections and performance summaries rather than an annotated
image topic. The experiment workflow can record local run bundles containing
manifest metadata, detections, performance summaries, system telemetry, native
pipeline telemetry, evidence frames, and generated summaries.

### Operator connectivity

**Implemented:** the robot gateway exposes system status, platform diagnostics,
preview control, overlay snapshots, and bounded teleop over gRPC. A managed GStreamer
process exports an on-demand SRT video stream, and packaged Python tools receive it
on the laptop. This preview is a diagnostic camera stream; it is not yet a
frame-exact detection review surface.

### Offboard review

**Implemented:** `scripts/omni runs` can inspect local bundles, list and pull
robot-side bundles onto a laptop, annotate recorded evidence frames, and generate a
simple static HTML report. Retrieval preserves additive files inside the bundle so
new telemetry or evidence streams do not require a transport redesign.

## Remaining Product Loop

The next implementation work should focus on:

1. Native runtime class updates with explicit lifecycle and failure semantics.
2. Completed target-hardware verification records for the full operator-integrated
   slice: preview, gateway status, bounded teleop, detections, and stop behavior.
3. Curated hardware evidence packs with run notes, representative successes, false
   positives, misses, and limitations.
4. Provider-neutral synchronization of completed run bundles or generated reports to
   a hosted static review path.

The local run-bundle contract is intentionally file-based and additive. The cloud
provider and hosted publication path remain unspecified until selected reports are
ready to publish.

## Portfolio Success Criteria

The artifact is complete when a reviewer can:

- select classes without retraining the detector
- run the model on the robot NPU
- observe typed detections and performance summaries
- reproduce a recorded experiment from documented commands
- inspect latency and throughput over time
- inspect detections by class and confidence
- review representative successes, false positives, and missed detections
- fetch or open the same completed run through the offboard review workflow
- view at least one selected report from the hosted documentation path

Claims should be supported by measured output, captured evidence, and a documented
hardware/software configuration.

## Non-Goals

- autonomous frontier exploration
- autonomous semantic search
- visual servoing or grasp/capture behavior
- production fleet management
- browser-based live robot control
- choosing a cloud provider before the selected static-report workflow is ready

## Related Documentation

- [System Architecture](../architecture.md)
- [Evidence and Verification Boundary](../evidence.md)
- [Vision Pipeline](vision_pipeline.md)
- [Vision Telemetry](vision_telemetry_spec.md)
- [Robot Gateway](robot_gateway.md)
- [CI/CD Overview](ci_cd.md)
