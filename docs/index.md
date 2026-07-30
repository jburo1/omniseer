# Omniseer

<div style="max-height: 620px; overflow: auto; margin: 0 auto 1.5rem;">
  <object data="assets/diagrams/explorer/system-explorer.svg" type="image/svg+xml" aria-label="Omniseer system explorer" width="1237" height="1133" style="display: block; width: 100%; max-width: 1000px; height: auto; margin: 0 auto;"></object>
</div>

Omniseer is an embodied AI system for open-vocabulary perception, bounded
target-centering behavior, operator diagnostics, and reproducible robot-run
evidence on a ROCK 5B+ mobile robot.

The robot-side path captures camera frames, preprocesses them with RGA, runs
YOLO-World on the Rockchip NPU, publishes typed detections and performance telemetry
through ROS 2, and can run a bounded target-centering controller. Operator tools
provide gRPC status/control, on-demand SRT preview, RunBundle retrieval, inspection,
annotation, and static report generation.

The documentation has three connected layers:

- **Architecture** explains the major robot, operator, gateway, firmware, runtime,
  and evidence components.
- **MkDocs pages** document subsystem contracts, commands, design rationale, and
  verification workflows.
- **RunBundles** preserve executable evidence from robot runs: manifests,
  detections, telemetry, evidence frames, annotations, and generated reports.

Start with the architecture and evidence pages, then drill into the subsystem or
operator workflow that matches the task at hand:

- [Scripts Front Door](operations/scripts-frontdoor.md) for the supported local command surface
- [Edge-to-Cloud Perception](perception/edge-to-cloud.md) for the implemented perception and evidence loop
- [System Architecture](architecture/overview.md) for runtime boundaries and implementation status
- [Verification Evidence](verification/evidence.md) for CI, local, and target-hardware coverage
- [Vision Pipeline](perception/vision-pipeline.md) for the native hot path
- [CI/CD Overview](verification/ci-cd.md) for automated verification and its limits
- [Robot Runtime Container](robot-runtime/robot-runtime-container.md) for the v2 robot image workflow
- [Robot Gateway](gateway/robot-gateway.md) and [Preview Streaming](gateway/preview-streaming.md) for operator diagnostics
- [Operator Run Workflow](operations/operator-run-workflow.md) for the monitor run/start/stop/retrieve path
