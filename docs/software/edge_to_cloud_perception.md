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
  -> planned experiment recorder
  -> planned run bundle
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
headless and currently publishes detections and performance summaries rather than an
annotated image topic.

### Operator connectivity

**Implemented:** the robot gateway exposes system status, platform diagnostics,
preview control, overlay snapshots, and bounded teleop over gRPC. A managed GStreamer
process exports an on-demand SRT video stream, and packaged Python tools receive it
on the laptop. This preview is a diagnostic camera stream; it is not yet a
frame-exact detection review surface.

## Planned Experiment Loop

The next implementation phase should add:

1. Native runtime class updates with explicit lifecycle and failure semantics.
2. A recorder that correlates typed detections and performance summaries by time.
3. A structured run directory containing metadata, telemetry, detections, evidence,
   and an optional rosbag.
4. Selected annotated frames, crops, and operator-marked failure cases.
5. CPU, memory, temperature, network, and power samples recorded outside the vision hot path.
6. A laptop report showing latency, throughput, detections, confidence, and evidence.
7. Provider-neutral synchronization of completed run bundles to a hosted review path.

The exact run-bundle schema and cloud provider remain intentionally unspecified until
the local recording and review workflow proves what data is useful.

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

Claims should be supported by measured output, captured evidence, and a documented
hardware/software configuration.

## Non-Goals

- autonomous frontier exploration
- autonomous semantic search
- visual servoing or grasp/capture behavior
- production fleet management
- browser-based live robot control
- choosing a cloud provider before the run-bundle contract is validated locally

## Related Documentation

- [System Architecture](../architecture.md)
- [Evidence and Verification Boundary](../evidence.md)
- [Vision Pipeline](vision_pipeline.md)
- [Vision Telemetry](vision_telemetry_spec.md)
- [Robot Gateway](robot_gateway.md)
- [CI/CD Overview](ci_cd.md)
