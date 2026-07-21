# Omniseer

Omniseer is an edge-to-cloud embodied AI project centered on open-vocabulary
perception on a ROCK 5B+ mobile robot.

The implemented robot-side path captures camera frames, preprocesses them with RGA,
runs YOLO-World on the Rockchip NPU, and publishes typed detections and performance
telemetry through ROS 2. Simulation, firmware, real-hardware bringup, a gRPC gateway,
and on-demand SRT preview provide the supporting platform.

The active portfolio deliverable is narrower than autonomous object search:

- select semantic target classes
- run and measure onboard inference
- record detections, telemetry, and useful evidence
- review an experiment on a laptop and, later, through a cloud-hosted workflow

Structured run bundles are implemented as local robot artifacts. Native runtime
class updates, cloud synchronization, and a hosted review dashboard are
**planned**. A simple local HTML report is available for laptop review.
Autonomous seek and capture are explicitly deferred.

Start with:

- [Scripts Front Door](software/scripts_frontdoor.md) for the supported local command surface
- [Edge-to-Cloud Perception](software/edge_to_cloud_perception.md) for the active deliverable
- [System Architecture](architecture.md) for runtime boundaries and implementation status
- [Evidence and Verification Boundary](evidence.md) for what current checks prove
- [Vision Pipeline](software/vision_pipeline.md) for the native hot path
- [CI/CD Overview](software/ci_cd.md) for automated verification and its limits
- [Robot Gateway](software/robot_gateway.md) and [Preview Streaming](software/preview_streaming.md) for operator diagnostics
