# v2-L FP target acquisition: retained RunBundle

**Canonical engineering write-up:** [Target Acquisition](https://jburo1.github.io/omniseer/verification/target-acquisition/).
That GitHub Pages page is the authoritative engineering narrative, results,
interpretation, and limitations. This repository directory is the retained
evidence and reproducibility index for that physical-robot case study.

## Directory contents

The unmodified [`run/`](run/) directory is the complete `v2l_fp_scene_1`
RunBundle: one successful ROCK 5B+ execution of bounded visual target
acquisition and framing with YOLO-World v2-L FP through RKNN.

[![Terminal `person` detection and framing capture](run/evidence/annotated/capture_frame_3184.jpg)](run/evidence/annotated/capture_frame_3184.jpg)

- [Manifest and provenance](run/manifest.yaml), including hardware,
  model/backend, launch configuration, Git SHA, runtime image digest, and
  copied inputs.
- [Autonomy trace](run/autonomy.jsonl), [summary](run/summary.json), and
  [current static report](run/report-updated/index.html) for the recorded
  state transitions, terminal result, and compact measurements.
- [Performance telemetry](run/perf.jsonl), [native pipeline telemetry](run/pipeline_telemetry.jsonl),
  and [system telemetry](run/system.jsonl) for timing, throughput, freshness,
  and system context.
- [Evidence index](run/evidence/evidence.jsonl), raw [captured frames](run/evidence/frames/),
  and review-copy [annotated frames](run/evidence/annotated/). The terminal
  capture above corresponds to the `target_framed` event in the autonomy trace.
- [Overlay video](run/video/overlay.mp4), [source video](run/video/source.mp4),
  [rosbag](run/rosbag/), and [artifact checksums](checksums.sha256).

## Reproducibility boundary

This preserved RunBundle is a representative successful deployment, not a
benchmark aggregate. Consult the
[canonical write-up](https://jburo1.github.io/omniseer/verification/target-acquisition/)
for the engineering interpretation and limitations; for broader detector
coverage and independent physical-run context, see the
[six-detector study](../../detector_comparison/scan_final_recal/README.md).
