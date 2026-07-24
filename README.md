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
                                                           -> laptop report
                                                           -> planned hosted review
```

Robot-side inference, ROS publication, telemetry, simulation, firmware, gateway
control, preview streaming, structured experiment recording, offboard run retrieval,
evidence annotation, and simple local HTML run reports are implemented. Runtime class
updates in the native RKNN bridge, cloud synchronization, and a hosted static review
path are planned next. Autonomous object search and capture are not part of the active
deliverable.

## Current Capabilities

- Hardware-accelerated V4L2 -> RGA -> RKNN producer/consumer vision pipeline.
- YOLO-World text-embedding preparation and bounded detection post-processing.
- Typed `/yolo/detections` and `/vision/perf` ROS 2 contracts.
- JSONL stage telemetry, rolling performance summaries, and offline analysis tools.
- Local perception run bundles with detections, performance summaries, system
  telemetry, native pipeline telemetry, evidence frames, inspection, retrieval,
  annotation, and static HTML reports.
- ROS 2 simulation and real-hardware bringup with firmware and micro-ROS integration.
- gRPC system status and preview control with on-demand SRT video export.
- Six-lane GitHub CI covering lint, ROS, Gazebo smoke, portable vision, firmware, and docs.

## Documentation

- [Project documentation](https://jburo1.github.io/omniseer/)
- [System architecture](docs/architecture.md)
- [Evidence and verification boundary](docs/evidence.md)
- [Edge-to-cloud perception](docs/software/edge_to_cloud_perception.md)
- [CI/CD overview](docs/software/ci_cd.md)

## Common Commands

The root `scripts/omni` entrypoint is the supported front door for common local
workflows:

```bash
scripts/omni build ros
scripts/omni test ros
scripts/omni run sim
scripts/omni run real
scripts/omni run real --record-run demo_001
scripts/omni runs inspect runs/demo_001
scripts/omni runs report runs/demo_001
scripts/omni check real-perception
scripts/omni flash teensy
scripts/omni docs build
```

For headless Teensy 4.1 flashing in Docker or over SSH, `scripts/omni flash teensy`
wraps the existing firmware helper.


## Current Boundary

GitHub CI validates portable software and simulation contracts. Target-hardware
camera, RGA, RKNN/NPU, and recording evidence exists in local run bundles, while
sensor, motor, firmware-flash, preview, and full operator-integrated behavior still
require explicit target-hardware verification records. See the CI/CD and evidence
documentation for the exact coverage boundary.
