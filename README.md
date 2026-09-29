# Omniseer

[![CI](https://github.com/jburo1/omniseer/actions/workflows/ci.yml/badge.svg)](https://github.com/jburo1/omniseer/actions/workflows/ci.yml)
[![Docs](https://img.shields.io/github/deployments/jburo1/omniseer/github-pages?label=Docs)](https://jburo1.github.io/omniseer/)

<b>Physical edge AI for open-vocabulary perception and bounded visual autonomy on RK3588.</b>

Omniseer is a holonomic 4WD robot for developing and evaluating neural perception under hardware constraints. A native C++ V4L2 → RGA → RKNN pipeline runs open-vocabulary YOLO-World on a ROCK 5B+ NPU, feeds detections into bounded visual autonomy, and records physical runs with model provenance, latency, telemetry, detections, video, and control traces.

An operator application on a remote machine parameterizes, launches, and synchronizes experiments between the onboard SBC and laptop. The project spans model conversion and quantization, hardware-accelerated inference, image processing and video encoding, ROS 2 and Gazebo integration, physical target acquisition, profiling, controlled experiments, and failure analysis.

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

https://github.com/user-attachments/assets/9afa632c-9b78-4978-9c06-8714d285d1c0

## Experiments

<table>
  <tr>
    <td width="33%" align="center" valign="top">
      <a href="https://jburo1.github.io/omniseer/verification/target-acquisition/">
        <img src="docs/assets/evidence/target-acquisition-v2l-fp-terminal.webp" alt="Terminal person detection from the v2-L FP target-acquisition RunBundle" width="300" />
      </a><br />
      <a href="https://jburo1.github.io/omniseer/verification/target-acquisition/">v2-L FP Target Acquisition</a><br />
      Report generated from a successful run's data.
    </td>
    <td width="33%" align="center" valign="top">
      <a href="https://jburo1.github.io/omniseer/verification/detector-comparison/">
        <img src="docs/assets/evidence/detector-comparison-poster.webp" alt="Six-panel controlled replay of YOLO-World detector configurations" width="300" />
      </a><br />
      <a href="https://jburo1.github.io/omniseer/verification/detector-comparison/">Six-Model Detector Comparison</a><br />
      A fixed-scene replay compares six YOLO-World RKNN configurations. Separate physical runs show their deployment trade-offs.
    </td>
    <td width="33%" align="center" valign="top">
      <a href="https://jburo1.github.io/omniseer/verification/int8-quantization-failure/">
        <img src="docs/assets/evidence/v2l-int8-quantization-comparison.webp" alt="Side-by-side replay: recalibrated v2-L INT8 has no detections, while the TD01 mixed-precision hybrid detects a desk, potted plant, and subwoofer" width="300" />
      </a><br />
      <a href="https://jburo1.github.io/omniseer/verification/int8-quantization-failure/">v2-L INT8 Quantization Failure Analysis</a><br />
      INT8 quantization caused the v2-L detector to collapse relative to FP16. Mixed precision surgery then partially recover the failure.
    </td>
  </tr>
</table>


## System at a Glance

<a href="https://jburo1.github.io/omniseer/"><img src="docs/assets/diagrams/explorer/system-explorer.svg" alt="Omniseer system architecture: PC, operator laptop, RK3588 robot runtime, firmware, hardware, and experiment evidence flow" width="100%"></a>

RKNN model construction happens offline on the development PC in a containerized environment. Simulation in Gazebo was used to prototype and validate key perception, navigation, and robot behavior before deploying the ROS 2 stack to physical hardware.

On the robot, native C++ owns capture, preprocessing, inference, post-processing, and telemetry. ROS 2 carries typed detection, performance, command, odometry, sensor, and battery contracts.Teensy firmware and micro-ROS own low-level I/O.

The operator gateway, preview stream, reports, and offboard inspection are diagnostic or review tooling, and are side carriaged so as to not interfere with the robot's hot path.

The robot runtime is packaged and verified as a containerized target runtime.


## Key Engineering Work

- **Native RK3588 perception pipeline** — C++ V4L2 capture, RGA preprocessing, RKNN inference, YOLO-World post-processing, and runtime telemetry on the robot SBC.
- **Hardware-aware model deployment** — converted, quantized, and evaluated YOLO-World variants on RK3588, including controlled FP/INT8 comparisons and mixed-precision failure analysis.
- **Perception-to-control autonomy** — bounded target acquisition and framing driven by neural detections, with command arbitration and proximity safety constraints.
- **Reproducible experiment infrastructure** — RunBundles preserve model provenance, configuration, detections, timing, telemetry, control traces, media, and generated reports for offline analysis.

More information can be found in the [project documentation](https://jburo1.github.io/omniseer/).
