# Omniseer

[![CI](https://github.com/jburo1/omniseer/actions/workflows/ci.yml/badge.svg)](https://github.com/jburo1/omniseer/actions/workflows/ci.yml)
[![Docs](https://img.shields.io/github/deployments/jburo1/omniseer/github-pages?label=Docs)](https://jburo1.github.io/omniseer/)

Omniseer is an edge-to-cloud embodied AI project built around open-vocabulary
perception on a ROCK 5B+ mobile robot. The current system captures camera frames,
runs YOLO-World inference on the Rockchip NPU, publishes typed ROS 2 detections and
performance telemetry, and exposes optional operator diagnostics over gRPC and SRT.

The active portfolio deliverable is a reproducible perception evaluation loop:

```text
camera -> RKNN YOLO-World -> ROS detections + telemetry -> experiment bundle
                                                           -> laptop/cloud review
```

Robot-side inference, ROS publication, telemetry, simulation, firmware, gateway
control, and preview streaming are implemented. Runtime class updates in the native
RKNN bridge, structured experiment recording, cloud synchronization, and a hosted
review dashboard are planned next. Autonomous object search and capture are not part
of the active deliverable.

## Current Capabilities

- Hardware-accelerated V4L2 -> RGA -> RKNN producer/consumer vision pipeline.
- YOLO-World text-embedding preparation and bounded detection post-processing.
- Typed `/yolo/detections` and `/vision/perf` ROS 2 contracts.
- JSONL stage telemetry, rolling performance summaries, and offline analysis tools.
- ROS 2 simulation and real-hardware bringup with firmware and micro-ROS integration.
- gRPC system status and preview control with on-demand SRT video export.
- Six-lane GitHub CI covering lint, ROS, Gazebo smoke, portable vision, firmware, and docs.

## Documentation

- [Project documentation](https://jburo1.github.io/omniseer/)
- [System architecture](docs/architecture.md)
- [Edge-to-cloud perception](docs/software/edge_to_cloud_perception.md)
- [CI/CD overview](docs/software/ci_cd.md)

## Current Boundary

GitHub CI validates portable software and simulation contracts. Camera, RGA, RKNN,
NPU, sensor, motor, and firmware-flash behavior still require validation on the
target hardware. See the CI/CD documentation for the exact coverage boundary.
