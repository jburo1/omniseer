# v2-M INT8 bounded target acquisition on ROCK 5B+

This study retains the original `v2m_int8_scene_1` RunBundle from one real ROCK
5B+ physical execution. It is a representative successful deployment, not a
benchmark aggregate. The deployed detector was YOLO-World v2-M INT8 using RKNN.
The same physical trial is one of the six independent runs summarized in the
[detector comparison](../../detector_comparison/scan_final_recal/README.md).

The bounded autonomy controller scanned for `person`, acquired and centered the
target, framed it, and captured terminal evidence. It succeeded with reason
`framed` at 51.1 s (first detection 24.5 s), with no target-loss episodes. Mean
consumer throughput was 10.04 FPS; inference p50/p95 were 96.22/106.47 ms; and
source-age p95 was 139.42 ms.

The unmodified bundle is under [`run/`](run/). Start with
[`run/manifest.yaml`](run/manifest.yaml), [`run/autonomy.jsonl`](run/autonomy.jsonl),
[`run/summary.json`](run/summary.json), and the existing
[`run/report/index.html`](run/report/index.html). Raw detections, performance,
system, and native pipeline telemetry are beside them; terminal evidence is in
[`run/evidence/`](run/evidence/); video, rosbag, and copied provenance are retained
in their original subdirectories. [`checksums.sha256`](checksums.sha256) inventories
every tracked bundle artifact.

Manifest provenance: Git SHA
`b90feb60e3a1a84c0a4d8d402a656d0b8bc42ef7`; runtime container
`ghcr.io/jburo1/omniseer-robot-runtime@sha256:1280cc86229b5acb6f30ad43629da687f1b816ed380b634ab8110729755562c5`.

This run establishes one bounded perception-to-acquisition/framing completion on
the recorded robot configuration. It does not establish mAP, general detector
accuracy, statistical significance, navigation-based semantic search, global
exploration, or learned control. See the
[six-model detector comparison](../../detector_comparison/scan_final_recal/README.md)
for the broader controlled replay and six-trial context.
