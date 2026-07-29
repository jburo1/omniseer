# Omniseer

[![CI](https://github.com/jburo1/omniseer/actions/workflows/ci.yml/badge.svg)](https://github.com/jburo1/omniseer/actions/workflows/ci.yml)
[![Docs](https://img.shields.io/github/deployments/jburo1/omniseer/github-pages?label=Docs)](https://jburo1.github.io/omniseer/)

Omniseer is an embodied AI system for running open-vocabulary perception and
bounded target-centering behavior on a ROCK 5B+ mobile robot. The repository
contains the robot runtime, firmware integration, ROS 2 contracts, operator tools,
diagnostic gateway, preview transport, experiment recorder, and local report
workflow used to evaluate real robot runs.

The implemented loop connects robot execution to reviewable evidence:

```text
camera -> V4L2/RGA/RKNN YOLO-World -> ROS detections + telemetry
                                    -> bounded target centering
                                    -> RunBundle evidence
                                    -> laptop inspection and static report
```

The documentation is organized as three connected layers:

- **Architecture** describes the runtime components, boundaries, and data/control
  flows.
- **MkDocs pages** explain the engineering rationale, subsystem contracts, commands,
  and verification workflow.
- **RunBundles** preserve robot-run evidence: manifest data, detections, performance
  summaries, system telemetry, native pipeline telemetry, selected frames,
  annotations, inspection results, and static HTML reports.

## Current Capabilities

- Hardware-accelerated V4L2 -> RGA -> RKNN producer/consumer vision runtime.
- YOLO-World text-embedding preparation and bounded detection post-processing.
- Typed `/yolo/detections` and `/vision/perf` ROS 2 contracts.
- JSONL stage telemetry, rolling performance summaries, and offline analysis tools.
- Bounded target-centering autonomy through `/cmd_vel_autonomy` and `twist_mux`.
- Local RunBundles with detections, performance summaries, system
  telemetry, native pipeline telemetry, evidence frames, inspection, retrieval,
  annotation, and static HTML reports.
- ROS 2 simulation and real-hardware bringup with firmware and micro-ROS integration.
- gRPC system status and preview control with on-demand SRT video export.
- Six-lane GitHub CI covering lint, ROS, Gazebo smoke, portable vision, firmware, and docs.

## Documentation

- [Project documentation](https://jburo1.github.io/omniseer/)
- [System architecture](docs/architecture.md)
- [Verification evidence](docs/evidence.md)
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

GitHub CI validates portable software, simulation contracts, firmware compilation,
and documentation. RunBundle evidence records target-hardware camera, RGA,
RKNN/NPU, telemetry, and recording behavior. The evidence documentation separates
CI coverage, local checks, and target-hardware run records so implementation claims
stay tied to reproducible artifacts.
