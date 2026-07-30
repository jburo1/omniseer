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

- [Scripts Front Door](software/scripts_frontdoor.md) for the supported local command surface
- [Edge-to-Cloud Perception](software/edge_to_cloud_perception.md) for the implemented perception and evidence loop
- [System Architecture](architecture.md) for runtime boundaries and implementation status
- [Verification Evidence](evidence.md) for CI, local, and target-hardware coverage
- [Vision Pipeline](software/vision_pipeline.md) for the native hot path
- [CI/CD Overview](software/ci_cd.md) for automated verification and its limits
- [Robot Runtime Container](software/robot_runtime_container.md) for the v2 robot image workflow
- [Robot Gateway](software/robot_gateway.md) and [Preview Streaming](software/preview_streaming.md) for operator diagnostics
- [Operator Run Workflow](software/operator_run_workflow.md) for the monitor run/start/stop/retrieve path
