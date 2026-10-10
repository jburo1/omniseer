# Omniseer

[![CI](https://github.com/jburo1/omniseer/actions/workflows/ci.yml/badge.svg)](https://github.com/jburo1/omniseer/actions/workflows/ci.yml)
[![Docs](https://img.shields.io/github/deployments/jburo1/omniseer/github-pages?label=Docs)](https://jburo1.github.io/omniseer/)

<b>Physical edge AI for open-vocabulary perception and target acquisition on an RK3588-equipped SBC.</b>

Omniseer is a holonomic 4WD robot for developing and evaluating neural perception on constrained hardware. A native C++ V4L2 → RGA → RKNN pipeline runs open-vocabulary YOLO-World on a ROCK 5B+ NPU, uses detections for target acquisition, and records physical runs with model provenance, latency, telemetry, detections, video, and control traces.

An operator application on a remote machine configures, launches, and synchronizes experiments between the onboard SBC and laptop. The project covers model conversion and quantization, hardware-accelerated inference, image processing and video encoding, ROS 2 and Gazebo integration, physical target acquisition, profiling, controlled experiments, and failure analysis.

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

An operator specifies an open-vocabulary target such as `person` or `backpack`. The robot scans, acquires a stable neural detection, centers it, adjusts distance to reach the requested framing, stops at a configured proximity limit, and records the run for reproducible analysis.

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
        <img src="docs/assets/evidence/v2l-int8-quantization-comparison.webp" alt="Side-by-side full-source replay: v2-L recalibrated INT8 has no detections, while v2-L TD01 mixed-precision detects a desk, potted plant, and subwoofer" width="300" />
      </a><br />
      <a href="https://jburo1.github.io/omniseer/verification/int8-quantization-failure/">v2-L INT8 Quantization Failure Analysis</a><br />
      v2-L recalibrated INT8 collapsed relative to v2-L FP. v2-L TD01 mixed-precision then partially recovered behavior as a diagnostic mitigation.
    </td>
  </tr>
</table>


## System at a Glance

<a href="https://jburo1.github.io/omniseer/"><img src="docs/assets/diagrams/explorer/system-explorer.svg" alt="Omniseer system architecture: PC, operator laptop, RK3588 robot runtime, firmware, hardware, and experiment evidence flow" width="100%"></a>


On the development PC, RKNN model construction happens offline in a containerized environment. Simulation in Gazebo was used to prototype and validate key perception, navigation, and robot behavior before deploying the ROS 2 stack to physical hardware.

On the robot, native C++ handles capture, preprocessing, inference, post-processing, and telemetry. ROS 2 carries typed detection, performance, command, odometry, sensor, and battery contracts. Teensy firmware and micro-ROS handle low-level I/O. The robot runtime is packaged and verified in a container.

On the laptop, the operator gateway, preview stream, reports, and offboard inspection are diagnostic or review tools that run outside the robot's critical perception-control path.


See the [project documentation](https://jburo1.github.io/omniseer/) for more detail.

I'm currently working on fine-tuning and deploying an image segmentation model onboard the NPU, NPU profiling, and validating EKF belief vs. ground truth robot motion.
