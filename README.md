# Omniseer

[![CI](https://github.com/jburo1/omniseer/actions/workflows/ci.yml/badge.svg)](https://github.com/jburo1/omniseer/actions/workflows/ci.yml)
[![Docs](https://img.shields.io/github/deployments/jburo1/omniseer/github-pages?label=Docs)](https://jburo1.github.io/omniseer/)

Omniseer is an edge-to-cloud ROS 2 robotics platform that runs open-vocabulary
perception on a ROCK 5B+ mobile robot, performs bounded visual target acquisition
and framing, and records reproducible evidence from real robot runs.

[Documentation](https://jburo1.github.io/omniseer/) |
[Architecture](docs/architecture/overview.md) |
[Verification Evidence](docs/verification/evidence.md) |
[Operator Workflow](docs/operations/operator-run-workflow.md)

## What the Robot Does

Given a named target class, the autonomy mode performs a bounded visual target
acquisition and framing behavior:

```text
named target
  -> bounded visual scan
  -> stable target acquisition
  -> horizontal centering
  -> bounded distance adjustment
  -> proximity safety stop
  -> evidence capture and RunBundle report
```

The robot scans in place for a configured target class, acquires a stable
detection, centers the target horizontally using yaw, and uses small bounded
forward or reverse motion to bring the target into the configured visual framing
range. Forward motion is blocked when the proximity range crosses the configured
threshold. This is not navigation-based object search, room-scale semantic
exploration, global planning, or learned end-to-end control.

## System at a Glance

The main end-to-end path is:

```text
camera
  -> V4L2 capture
  -> RGA preprocessing
  -> RKNN YOLO-World inference
  -> typed ROS 2 detections and telemetry
  -> bounded autonomy
  -> RunBundle evidence
  -> laptop inspection and static report
```

Important boundaries are explicit:

- ROCK 5B+ edge compute runs the mission-critical robot workload.
- Native C++ perception owns V4L2 capture, RGA preprocessing, RKNN inference,
  post-processing, and detailed telemetry.
- ROS 2 sim and real bringup meet at typed detection, performance, command,
  odometry, sensor, and battery contracts.
- Teensy firmware and micro-ROS own low-level robot IO.
- The operator gateway and preview stream provide diagnostics without becoming
  dependencies of mission-critical perception or control.
- The robot runtime is packaged and verified as a containerized target runtime.
- RunBundle retrieval, annotation, inspection, and static reports happen offboard
  on the operator laptop.

The technically distinctive work is making that path fast enough for edge
execution, bounded enough for physical robot behavior, and measurable enough that
claims can be reviewed against recorded artifacts.

## Representative Engineering Evidence

| Evidence | What it supports | Public boundary |
| --- | --- | --- |
| GitHub Actions CI | Portable ROS package checks, Gazebo smoke boundary topics, portable vision tests, firmware compile, docs build, and hardware-independent runtime packaging | CI does not prove camera, RKNN/RGA, LiDAR, Teensy, or physical robot execution |
| RunBundle format and tooling | Reproducible run manifests, detections, performance telemetry, system telemetry, evidence frames, annotations, and static report generation | Tool support does not imply a public target-hardware perception or autonomy run |
| Implementation-backed target runtime | V4L2 capture, RGA preprocessing, RKNN inference, ROS bridge integration, and bounded autonomy source paths | Source and tests are public; target-hardware execution claims require named public artifacts |

## Engineering Contributions

- Hardware-accelerated edge perception runtime that keeps camera capture,
  preprocessing, inference, post-processing, and telemetry on the robot SBC.
- Explicit ROS sim/real boundary so portable simulation checks and real robot
  integration share typed contracts instead of separate ad hoc paths.
- Bounded visual autonomy that uses command arbitration and a proximity stop
  while keeping the behavior limited to target acquisition and framing.
- Reproducible RunBundle workflow that preserves run configuration, telemetry,
  detections, evidence frames, annotations, and static reports for offboard
  review.
- Containerized robot runtime packaging with a verification boundary that
  distinguishes portable image checks from target-hardware validation.
- Optional gateway and preview tooling isolated from mission-critical execution
  so operator diagnostics can degrade without changing the robot behavior path.
- Focused CI coverage for portable software with an explicit hardware evidence
  boundary.

## Repository Inspection Path

1. [System Architecture](docs/architecture/overview.md) for the implemented
   robot, operator, firmware, runtime, and evidence boundaries.
2. [Verification Evidence](docs/verification/evidence.md) for CI, local checks,
   target-hardware evidence, and what each artifact supports.
3. [Edge-to-Cloud Perception](docs/perception/edge-to-cloud.md) and
   [Vision Pipeline](docs/perception/vision-pipeline.md) for the native
   perception path.
4. [Operator Run Workflow](docs/operations/operator-run-workflow.md) for the
   run, stop, retrieve, inspect, and report path.
5. [CI/CD Overview](docs/verification/ci-cd.md) and
   [Scripts Front Door](docs/operations/scripts-frontdoor.md) for supported
   verification commands and automation limits.

## Reproduction and Operational Boundary

Portable checks cover documentation, Python/package tests, portable vision
components, firmware compilation, Gazebo smoke topics, and hardware-independent
runtime packaging through `scripts/omni` and GitHub Actions.

ROS and simulation are required for graph-level launch and smoke verification.
The full perception and robot behavior path requires the ROCK 5B+, RKNN/RGA SDKs,
camera, LiDAR/range data, Teensy firmware, micro-ROS transport, sensors, and the
physical robot. GitHub CI is intentionally portable and does not prove
target-hardware execution.

## Limitations

- Autonomy is a bounded visual scan and framing behavior, not room-scale
  navigation-based search or semantic exploration.
- Hardware-specific inference requires the target ROCK 5B+ platform and RKNN/RGA
  SDK availability.
- No public target-hardware RunBundle, static report, annotated frame, video, or
  measurement is linked from the current evidence catalog.
- No strong real robot photograph, public run video, or public report screenshot
  is currently committed, so the README avoids placeholder media.
