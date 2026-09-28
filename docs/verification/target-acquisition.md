---
description: "One public ROCK 5B+ v2-L FP target-acquisition RunBundle: bounded visual framing completed in 56.1 seconds with zero target-loss episodes."
---

# v2-L FP Target Acquisition

## Engineering question

Can open-vocabulary YOLO-World v2-L FP running through RKNN on a ROCK 5B+
support a complete, reviewable execution of Omniseer's bounded visual
target-acquisition and framing behavior?

## Method / experimental setup

This is one physical-robot case study, recorded as the public
`v2l_fp_scene_1` RunBundle. The recorded model is YOLO-World v2-L FP with the
RKNN backend. The bounded controller was configured to find `person`: it
scanned, acquired a stable detection, centered and framed it, then stopped at
the terminal condition `framed`. The bundle records the model/backend and
launch configuration, Git revision, runtime image digest, copied inputs,
autonomy trace, detections, timing telemetry, visual evidence, media, and
rosbag. The manifest is the authoritative provenance record for this run.

## Results

<video controls preload="metadata" poster="../../assets/evidence/target-acquisition-v2l-fp-terminal.webp" width="100%">
  <source src="../../runs/v2l_fp_scene_1/video/overlay.mp4" type="video/mp4" />
  Your browser cannot play this video. <a href="../../runs/v2l_fp_scene_1/video/overlay.mp4">Open the target-acquisition overlay video</a>.
</video>

The controller detected `person`, completed framing, and stopped with terminal
reason `framed`.

| Measurement | Observed value |
| --- | ---: |
| First `person` detection | **25.4 s** |
| Terminal `framed` success | **56.1 s** |
| Target-loss episodes | **0** |
| Mean consumer throughput | **2.64 FPS** |
| RKNN inference p50 / p95 | **380.25 / 400.98 ms** |
| Source-age p95 | **438.76 ms** |

## Engineering interpretation

This artifact demonstrates one end-to-end execution of bounded target
acquisition and framing on the stated hardware and model configuration. It
establishes that the recorded execution completed reviewably; it does not turn
one success into a performance benchmark or a general perception claim. For
fixed-source detector coverage and six independent physical case-study
summaries, see [Six-Model Detector Comparison](detector-comparison.md).

## Evidence and reproducibility

The complete RunBundle is available in this documentation site. Start with the
<a href="../../runs/v2l_fp_scene_1/report/">static run report</a> and follow
its links to the manifest, autonomy trace, summaries, telemetry, evidence
frames, overlay and source video, rosbag, and checksums. The terminal capture and its
`target_framed` event can be cross-checked against the evidence index and
`autonomy.jsonl` in that bundle.

The repository copy of the [raw RunBundle directory](https://github.com/jburo1/omniseer/tree/master/studies/autonomy/v2l_fp_target_acquisition/run)
is a secondary artifact-inspection path, not a separate study narrative.

## Limitations

This is one representative successful run, not a benchmark aggregate or a
claim of statistical significance, general detector accuracy, navigation-based
search, global exploration, or learned control.
