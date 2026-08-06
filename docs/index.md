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
| Understand edge perception | [Edge-to-Cloud Perception](perception/edge-to-cloud.md) and [Vision Pipeline](perception/vision-pipeline.md) |
| Operate or review a run | [Operator Run Workflow](operations/operator-run-workflow.md) and [Scripts Front Door](operations/scripts-frontdoor.md) |
| Inspect verification and CI | [CI/CD Overview](verification/ci-cd.md) |

## Evidence Boundary

GitHub CI verifies portable software, documentation, firmware compilation,
simulation smoke boundaries, portable vision tests, and hardware-independent
runtime packaging. Target-hardware behavior requires the ROCK 5B+, RKNN/RGA SDKs,
camera, sensors, Teensy, micro-ROS transport, and robot runtime.

The current evidence catalog does not link a public target-hardware RunBundle,
static report, annotated frame, video, or measurement. Implementation-backed
capabilities should not be read as public execution evidence unless a named
artifact is listed in [Verification Evidence](verification/evidence.md).
