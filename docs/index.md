# Omniseer

Omniseer is an edge-to-cloud ROS 2 robotics platform that runs open-vocabulary
perception on a ROCK 5B+ mobile robot, performs bounded visual target acquisition
and framing, and records reproducible evidence from real robot runs.

The current autonomy behavior performs an in-place bounded visual scan for a
configured target class, acquires a stable detection, centers it horizontally,
uses small bounded distance adjustments to frame it, blocks forward motion at the
configured proximity threshold, and records terminal evidence. It is not
navigation-based object search or room-scale semantic exploration.

<div style="max-height: 620px; overflow: auto; margin: 0 auto 1.5rem;">
  <object data="assets/diagrams/explorer/system-explorer.svg" type="image/svg+xml" aria-label="Omniseer system explorer" width="1237" height="1133" style="display: block; width: 100%; max-width: 1000px; height: auto; margin: 0 auto;"></object>
</div>

## Start Here

| Reviewer goal | Start with |
| --- | --- |
| Understand the system | [System Architecture](architecture/overview.md) |
| Inspect implementation-backed evidence | [Verification Evidence](verification/evidence.md) |
| Inspect public target-acquisition evidence | [v2-M INT8 Target Acquisition RunBundle](https://github.com/jburo1/omniseer/tree/master/studies/autonomy/v2m_int8_target_acquisition) |
| Inspect target-hardware-derived detector evidence | [Detector Comparison](verification/detector-comparison.md) |
| Inspect the INT8 failure investigation | [v2-L INT8 Quantization Study](https://github.com/jburo1/omniseer/tree/master/studies/quantization/yolo_world_v2l_int8) |
| Understand edge perception | [Edge Perception and Offboard Review](perception/edge-to-cloud.md) and [Vision Pipeline](perception/vision-pipeline.md) |
| Operate or review a run | [Operator Run Workflow](operations/operator-run-workflow.md) and [Scripts Front Door](operations/scripts-frontdoor.md) |
| Inspect verification and CI | [CI/CD Overview](verification/ci-cd.md) |

## Evidence Boundary

GitHub CI verifies portable software, documentation, firmware compilation,
simulation smoke boundaries, portable vision tests, and hardware-independent
runtime packaging. Target-hardware behavior requires the ROCK 5B+, RKNN/RGA SDKs,
camera, sensors, Teensy, micro-ROS transport, and robot runtime.

The catalog links a public, target-hardware-derived controlled detector replay
and recomputable presence/visibility metrics in [Detector Comparison](verification/detector-comparison.md).
That comparison is not itself a public autonomy RunBundle or target-hardware timing
benchmark. The separate [v2-M INT8 target-acquisition study](https://github.com/jburo1/omniseer/tree/master/studies/autonomy/v2m_int8_target_acquisition)
contains one complete successful physical-run RunBundle and bounded autonomy-execution
evidence. Implementation-backed capabilities should not be read as public execution
evidence beyond their named artifacts.
