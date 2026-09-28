# Omniseer

[![CI](https://github.com/jburo1/omniseer/actions/workflows/ci.yml/badge.svg)](https://github.com/jburo1/omniseer/actions/workflows/ci.yml)
[![Docs](https://img.shields.io/github/deployments/jburo1/omniseer/github-pages?label=Docs)](https://jburo1.github.io/omniseer/)

<b>Physical edge AI for open-vocabulary perception and bounded visual autonomy on RK3588.</b>

Omniseer is a holonomic 4WD robot for developing and evaluating neural perception under real hardware constraints. A native C++ V4L2 → RGA → RKNN pipeline runs open-vocabulary YOLO-World on a ROCK 5B+ NPU, feeds detections into bounded visual autonomy, and records physical runs with model provenance, latency, telemetry, detections, video, and control traces.

An operator application on a remote machine parameterizes, launches, and synchronizes experiments between the onboard SBC and laptop. The project spans model conversion and quantization, hardware-accelerated inference, ROS 2 and Gazebo integration, physical target acquisition, profiling, controlled experiments, and failure analysis.

[Documentation](docs/index.md) · [Architecture](docs/architecture/overview.md) · [Physical target-acquisition RunBundle](studies/autonomy/v2l_fp_target_acquisition/README.md) · [Six-model detector comparison](studies/detector_comparison/scan_final_recal/README.md) · [INT8 quantization study](studies/quantization/yolo_world_v2l_int8/README.md)

<p align="center">
  <img src="docs/assets/evidence/robot-hero.webp" width="800" alt="Omniseer physical mobile robot with LiDAR and mecanum drive" />
</p>

<table align="center">
  <tr>
    <td><img src="assets/images/omniseer_profile.jpg" width="260" alt="Omniseer profile view" /></td>
    <td><img src="assets/images/top_down_90.jpg" width="260" alt="Omniseer top-down view rotated 90 degrees" /></td>
    <td><img src="assets/images/omniseer_wireframe.jpg" width="260" alt="Omniseer wireframe" /></td>
  </tr>
</table>

## What the Robot Does

An operator specifies an open-vocabulary target such as `person` or `backpack`. The robot scans, acquires a stable neural detection, centers it, adjusts distance to reach the requested framing, stops at a configured proximity limit, and preserves the run as reproducible evidence.

<video src="assets/omni_video.mp4" controls playsinline aria-label="Omniseer target-acquisition demonstration"></video>

## Experiments

<table>
  <tr>
    <td width="33%" align="center" valign="top">
      <a href="studies/autonomy/v2l_fp_target_acquisition/README.md">
        <img src="docs/assets/evidence/target-acquisition-v2l-fp-terminal.webp" alt="Terminal person detection from the v2-L FP target-acquisition RunBundle" width="300" />
      </a><br />
      <a href="studies/autonomy/v2l_fp_target_acquisition/README.md">v2-L FP target acquisition</a><br />
      Report generated from a successful run's data.
    </td>
    <td width="33%" align="center" valign="top">
      <a href="studies/detector_comparison/scan_final_recal/README.md">
        <img src="docs/assets/evidence/detector-comparison-poster.webp" alt="Six-panel controlled replay of YOLO-World detector configurations" width="300" />
      </a><br />
      <a href="studies/detector_comparison/scan_final_recal/README.md">Six-model detector comparison</a><br />
      A fixed-scene replay compares six YOLO-World RKNN configurations with the same vocabulary and post-processing; separate physical runs show their deployment throughput trade-offs.
    </td>
    <td width="33%" align="center" valign="top">
      <a href="studies/quantization/yolo_world_v2l_int8/README.md">
        <img src="docs/assets/evidence/v2l-int8-quantization-comparison.webp" alt="Side-by-side replay: recalibrated v2-L INT8 has no detections, while the TD01 mixed-precision hybrid detects a desk, potted plant, and subwoofer" width="300" />
      </a><br />
      <a href="studies/quantization/yolo_world_v2l_int8/README.md">v2-L INT8 quantization study</a><br />
      An RK3588 failure investigation asks whether v2-L INT8 preserves FP detection behavior, documenting its collapse and TD01 mixed precision's partial diagnostic recovery.
    </td>
  </tr>
</table>


## System at a Glance

```text
camera
  -> V4L2 capture
  -> RGA preprocessing
  -> RKNN YOLO-World inference
  -> typed ROS 2 detections and telemetry
  -> bounded target acquisition and framing
  -> RunBundle evidence
  -> laptop inspection and static report
```

The mission-critical path stays on the robot: native C++ owns capture, preprocessing, inference, post-processing, and telemetry; ROS 2 carries typed detection, performance, command, odometry, sensor, and battery contracts; Teensy firmware and micro-ROS own low-level I/O. The operator gateway, preview stream, reports, and offboard inspection are diagnostic or review tooling, not dependencies of mission-critical perception or control. The robot runtime is packaged and verified as a containerized target runtime.

## Representative Engineering Evidence

| Evidence | What it supports | Public boundary |
| --- | --- | --- |
| GitHub Actions CI | Portable ROS package checks, Gazebo smoke topics, portable vision tests, firmware compilation, docs build, and hardware-independent runtime packaging | Does not prove camera, RKNN/RGA, LiDAR, Teensy, or physical-robot execution |
| The three flagship studies above | Named physical-run, controlled replay, and target-hardware quantization claims | Their study-specific limitations apply; none is mAP, a replicated benchmark, or a general autonomy result |
| RunBundle tooling and target-runtime source | Reproducible manifests, telemetry, evidence frames, static reports, and the implemented V4L2/RGA/RKNN-to-ROS path | Tooling and source alone do not prove a particular target-hardware run |

## Engineering Contributions

- Native RK3588 perception and inference pipeline: V4L2 capture, RGA preprocessing, RKNN execution, post-processing, and detailed telemetry on the robot SBC.
- Bounded perception-to-control autonomy for target acquisition and framing, with command arbitration and a proximity stop.
- Reproducible RunBundle and evidence workflow that preserves configuration, telemetry, detections, media, annotations, and static reports for offboard review.
- Explicit simulation, real-hardware, and runtime-verification boundaries that keep portable checks distinct from target-hardware evidence.

## Repository Inspection Path

- [System Architecture](docs/architecture/overview.md) — robot, operator, firmware, runtime, and evidence boundaries.
- [Verification Evidence](docs/verification/evidence.md) — CI scope, target-hardware artifacts, and reproducibility limits.
- [Edge Perception and Offboard Review](docs/perception/edge-to-cloud.md) and [Vision Pipeline](docs/perception/vision-pipeline.md) — native perception and review path.
- [Operator Run Workflow](docs/operations/operator-run-workflow.md) — run, stop, retrieve, inspect, and report flow.
- [CI/CD Overview](docs/verification/ci-cd.md) and [Scripts Front Door](docs/operations/scripts-frontdoor.md) — supported checks and automation limits.
