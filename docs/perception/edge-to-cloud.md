# Edge-to-Cloud Perception

## Purpose

Document the implemented edge perception and review loop used by Omniseer robot
runs.

Omniseer deploys an open-vocabulary model on constrained edge hardware, integrates
it with ROS 2, measures it under robot operating conditions, and preserves the
result in a reproducible offboard experiment workflow.

## Runtime Flow

An operator selects semantic classes, runs the robot through a scene or starts the
bounded target-centering behavior, and records an experiment. YOLO-World runs on the
ROCK 5B+ NPU. The resulting detections, stage timings, performance summaries,
autonomy events, and selected visual evidence form a RunBundle that can be reviewed
on a laptop.

```text
camera
  -> native V4L2/RGA/RKNN runtime
  -> /yolo/detections + /vision/perf
  -> target-centering autonomy when enabled
  -> experiment recorder
  -> RunBundle
  -> laptop review
```

## Current Implementation

### Edge inference

The native runtime captures
NV12 frames, preprocesses them with RGA, prepares YOLO-World text embeddings, runs
RKNN detector inference, maps detections back to source coordinates, and publishes
bounded results. The local `runs/pipeline_001` bundle records detections,
performance summaries, system telemetry, and native pipeline telemetry from a
target-hardware run.

The native bridge currently receives classes from `classes.path` during startup.
Changing classes requires restarting the bridge with a different class list. The
Python `yolo_ros` integration has a `SetClasses` service, but that service does not
reconfigure the native RKNN bridge.

### ROS contracts

The native bridge publishes:

- `/yolo/detections` as `yolo_msgs/msg/DetectionArray`
- `/vision/perf` as `omniseer_msgs/msg/VisionPerfSummary`

The performance message reports producer and consumer rates, recent stage timings,
source age, processed counts, and pipeline error counters. It intentionally stays
focused on vision pipeline metrics; live CPU, memory, temperature, network, and power
diagnostics are surfaced separately through the robot gateway status snapshot.

### Local observability

The native harness supports an annotated OpenCV preview, JSONL stage
telemetry, rolling statistics, and offline telemetry analysis. The ROS bridge is
headless and publishes detections and performance summaries rather than an annotated
image topic. The experiment workflow can record local run bundles containing
manifest metadata, detections, performance summaries, system telemetry, native
pipeline telemetry, evidence frames, ROS launch stdout/stderr logs, and generated
summaries.

### Operator connectivity

The robot gateway exposes system status, platform diagnostics,
preview control, overlay snapshots, and bounded teleop over gRPC. A managed GStreamer
process exports an on-demand SRT video stream, and packaged Python tools receive it
on the laptop. This preview is a diagnostic camera stream.

### Offboard review

`scripts/omni runs` can inspect local bundles, list and pull
robot-side bundles onto a laptop, annotate recorded evidence frames, and generate a
simple static HTML report. Retrieval preserves additive files inside the bundle so
new telemetry or evidence streams do not require a transport redesign.

## Review Path

A reviewer can:

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

## Related Documentation

- [System Architecture](../architecture/overview.md)
- [Verification Evidence](../verification/evidence.md)
- [Vision Pipeline](vision-pipeline.md)
- [Vision Telemetry](vision-telemetry.md)
- [Robot Gateway](../gateway/robot-gateway.md)
- [CI/CD Overview](../verification/ci-cd.md)
