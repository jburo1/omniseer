---
description: "Six YOLO-World configurations on a repaired 360° scene: controlled presence/visibility coverage plus independent ROCK 5B+ physical-run summaries."
---

# Detector Comparison

## Engineering question

With one repaired, frame-aligned 360° source scene, how do the six final
YOLO-World RKNN detector configurations differ when vocabulary and
post-processing are held fixed—and how do their independent physical
case-study summaries inform deployment trade-offs?

[![Six-panel controlled replay poster](https://github.com/jburo1/omniseer/raw/master/studies/detector_comparison/scan_final_recal/evidence/controlled_replay_poster.jpg)](https://github.com/jburo1/omniseer/blob/master/studies/detector_comparison/scan_final_recal/evidence/controlled_replay_2x3.mp4)

*Controlled replay of the same repaired source frames through all six detector
configurations. The full study publishes provenance, JSONLs, visibility
annotations, a comparison report, and independent physical-run summaries.*

The controlled replay is a 3-row × 2-column layout, pairing FP with INT8 or
Hybrid by model size. It holds the repaired source frames, vocabulary, and
post-processing fixed while changing only the detector configuration. Its leading
visible-frame result is v2-M FP at 2,367 / 5,679 (41.7%), narrowly ahead of v2-L
FP at 41.6%. This is one-scene presence/visibility evidence, not mAP,
bounding-box recall, latency evidence, or general detector accuracy.

## Independent physical runs

The physical presentation grid is a 2-row × 3-column display of independent
runs; its panels do not start together and it is not a controlled replay.

| Configuration | Outcome | First detection | Success | Mean consumer FPS | Inference p50 | Inference p95 |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| v2-S FP | success | 24.6 s | 53.4 s | 7.90 | 117.28 ms | 153.95 ms |
| v2-S INT8 | success | 24.6 s | 50.5 s | 16.54 | 59.19 ms | 60.76 ms |
| v2-M FP | success | 25.2 s | 53.9 s | 4.37 | 225.12 ms | 251.43 ms |
| v2-M INT8 | success | 24.5 s | 51.1 s | 10.04 | 96.22 ms | 106.47 ms |
| v2-L FP | success | 25.4 s | 56.1 s | 2.64 | 380.25 ms | 400.98 ms |
| v2-L Hybrid | success | 24.9 s | 57.8 s | 4.65 | 205.55 ms | 229.90 ms |

v2-S INT8 has the highest author-reported throughput among the six physical
case-study summaries. The v2-M INT8 RunBundle is publicly inspectable; the
other five physical RunBundles remain local, so their timing values and derived
ratios cannot be independently recomputed from the public repository. Against
their FP counterparts, v2-S INT8 improves inference p50/p95 by 1.98×/2.53× and
throughput by 2.09×; v2-M INT8 by 2.34×/2.36× and 2.30×; and v2-L Hybrid by
1.85×/1.74× and 1.76×. v2-M INT8 is therefore the strongest observed
coverage/throughput compromise for this scene. v2-L Hybrid is
faster than v2-L FP, but has lower
controlled coverage and higher observed memory use. All six trials succeeded
without target loss; their 50.5–57.8 s success times vary far less than their
59.19–380.25 ms inference p50 values, suggesting scan/control timing is also
material in this bounded trial.

These are one independent physical run per configuration, not replicated
benchmark estimates or proof of causal model differences. Detection coverage
comes from fixed-source replay; runtime behavior comes from the independent
trials. The retained thermal throttle field is unavailable, so no-throttling is
not established by this evidence.

The tracked comparison video is target-hardware-derived public evidence. The
complete RunBundles, source transport stream, and raw physical manifests remain
ignored local evidence. JSONLs plus visibility annotations are sufficient to
recompute controlled metrics; rerendering the video also requires the retained
source stream.

[Open the full detector-comparison study and reproducibility artifacts on GitHub](https://github.com/jburo1/omniseer/tree/master/studies/detector_comparison/scan_final_recal){ .md-button .md-button--primary }
