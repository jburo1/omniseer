# v2-L FP target acquisition on ROCK 5B+

**One complete, successful physical-robot execution.** On a ROCK 5B+,
YOLO-World v2-L FP ran through RKNN while the bounded controller scanned for
`person`, acquired the target, centered and framed it, then completed with
reason `framed`.

[![Terminal `person` detection and framing capture](run/evidence/annotated/capture_frame_3184.jpg)](run/evidence/annotated/capture_frame_3184.jpg)

*The existing terminal evidence capture: frame 3184, recorded with reason
`target_framed`. The controller's corresponding terminal event is in the
[autonomy trace](run/autonomy.jsonl).*

## Headline results

| Outcome | Measurement |
| --- | --- |
| Target acquisition | First `person` detection at **25.4 s**; completed at **56.1 s** with `framed` |
| Tracking continuity | **0 target-loss episodes** |
| On-robot throughput | Mean consumer throughput: **2.64 FPS** |
| RKNN inference | p50/p95: **380.25 / 400.98 ms** |
| Freshness | Source-age p95: **438.76 ms** |

## Inspect the run

The unmodified [`run/`](run/) directory is the complete `v2l_fp_scene_1`
RunBundle. Useful starting points:

- [Manifest and provenance](run/manifest.yaml) — ROCK 5B+, model/backend,
  launch configuration, Git SHA, runtime image digest, and copied inputs.
- [Autonomy trace](run/autonomy.jsonl) — scan, acquisition/framing, terminal
  `framed` result, and target-loss counters.
- [Summary](run/summary.json) and [static report](run/report/index.html) —
  compact measurements and a review surface derived from the bundle.
- [Performance telemetry](run/perf.jsonl), [native pipeline telemetry](run/pipeline_telemetry.jsonl),
  and [system telemetry](run/system.jsonl) — timing, throughput, freshness, and
  recorded system context.
- [Evidence index](run/evidence/evidence.jsonl) and [captured frames](run/evidence/frames/) —
  raw visual evidence; [annotated frames](run/evidence/annotated/) are review
  copies.
- [Overlay video](run/video/overlay.mp4), [source video](run/video/source.mp4),
  [rosbag](run/rosbag/), and [artifact checksums](checksums.sha256).

## Scope and provenance

This is a representative successful deployment, not a benchmark aggregate. The
recorded model is YOLO-World v2-L FP with RKNN; its configuration, model hash,
classes, launch parameters, Git SHA
`b90feb60e3a1a84c0a4d8d402a656d0b8bc42ef7`, and runtime container digest are
preserved in the [manifest](run/manifest.yaml). The outcome and times above come
from the [autonomy trace](run/autonomy.jsonl); performance values come from the
[summary](run/summary.json), [static report](run/report/index.html), and telemetry.

One run does not establish mAP, general detector accuracy, statistical
significance, navigation-based semantic search, global exploration, or learned
control. For broader controlled replay and six independent physical-run context,
see the [six-detector study](../../detector_comparison/scan_final_recal/README.md).
