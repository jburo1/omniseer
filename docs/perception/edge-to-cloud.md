---
description: "On-robot runtime and offboard review flow for real-hardware Omniseer experiments on ROCK 5B+."
---

# Runtime Overview

Omniseer runs a real-hardware experiment on a ROCK 5B+ while preserving the
artifacts needed to inspect that run offboard. The robot-side runtime combines
native perception, the ROS 2 graph, bounded behavior, hardware I/O, and
recording; laptop tools remain outside the motion-critical path.

## End-to-End Runtime Flow

```text
operator laptop
  -> starts real-hardware bringup on the ROCK 5B+
  -> real I/O + micro-ROS agent + camera/LiDAR bring up the ROS 2 graph
  -> native V4L2/RGA/RKNN vision publishes detections and vision performance
  -> ROS consumers, twist_mux command arbitration, and optional bounded autonomy
  -> Teensy firmware receives the stamped command and owns motor safety/I/O
  -> RunBundle recorder preserves run data and evidence
  -> laptop retrieves the completed bundle for annotation, report, and review
```

## Robot-Side Runtime

Real-hardware bringup starts the configured camera, LiDAR, Teensy micro-ROS
connection, native vision bridge, and shared ROS 2 graph. Native perception
uses the ROCK 5B+ acceleration path to publish canonical detections and vision
performance summaries. It is a real-hardware provider; the simulation path has
different lower-level producers while consuming aligned ROS contracts.

ROS 2 carries the normalized sensor, odometry, perception, and command
interfaces between these components. `twist_mux` arbitrates stamped motion
commands before the Teensy boundary. Optional target acquisition and framing
are bounded ROS consumers of those interfaces; they do not replace firmware
command handling, timeout behavior, or motor safety.

The Teensy firmware owns motor control, mecanum kinematics, encoder, IMU, and
sonar acquisition, and publishes its real-I/O data through micro-ROS. The
robot gateway is an optional operator-facing boundary: it exposes normalized
status, preview control, overlays, and bounded teleoperation without exposing
the internal ROS graph. Preview and review tooling are diagnostic surfaces, not
dependencies of perception, arbitration, or firmware control.

When recording is enabled, the RunBundle recorder captures the configured ROS
and native evidence streams, launch context, and derived summaries. A completed
bundle can then be retrieved to the laptop, annotated, and rendered as a static
report without modifying its raw evidence.

## Detailed References

- [Vision Pipeline](vision-pipeline.md) — native real-time C++ capture,
  preprocessing, inference, publication, and failure behavior.
- [Vision Telemetry](vision-telemetry.md) — native telemetry records and timing
  semantics.
- [ROS / Sim-Real Interfaces](../robot-runtime/ros-packages.md) — launch
  composition, topics, providers, and command boundary.
- [Robot Runtime Container](../robot-runtime/robot-runtime-container.md) —
  packaged runtime lifecycle and verification.
- [Operator Run Workflow](../operations/operator-run-workflow.md) — laptop-side
  start, stop, retrieval, and report workflow.
- [Robot Gateway](../gateway/robot-gateway.md) — external gRPC status, preview,
  and teleoperation boundary.
- [Firmware & Robot I/O](../firmware/overview.md) — Teensy and micro-ROS
  responsibilities, safety behavior, and interfaces.
- [Verification Evidence](../verification/evidence.md) — RunBundle semantics and
  the evidence required for target-hardware claims.
