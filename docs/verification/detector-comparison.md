---
description: "Six YOLO-World configurations on a repaired 360° scene: controlled presence/visibility coverage plus independent ROCK 5B+ physical-run summaries."
---

# Six-Model Detector Comparison

## Engineering question

With one repaired, frame-aligned 360° source scene, how do the six final
YOLO-World RKNN detector configurations differ when vocabulary and
post-processing are held fixed—and how do their independent physical
case-study summaries inform deployment trade-offs?

## Method / experimental setup

The controlled replay sends exactly the same 1,222 repaired source frames
(inclusive frames 0–1221) through each configuration: v2-S FP, v2-S INT8,
v2-M FP, v2-M INT8, v2-L FP, and v2-L Hybrid. It holds source repair, class
vocabulary, score threshold (0.25), NMS IoU threshold (0.45), and maximum
detections (100) fixed; detector configuration is the only intended variable.
Manual visibility annotations define 19 in-vocabulary class-interval sets and
the 5,679 visible-frame denominator. `laptop`, `computer monitor`, and `sofa`
were outside the comparison vocabulary and were excluded rather than counted
as negative evidence.

The physical-run grid is a separate 2-row × 3-column presentation of six
independent ROCK 5B+ runs. Panels begin at their own run start and the grid
ends at the shortest overlay, so it is supporting end-to-end case-study
evidence, not a frame-aligned replay or a latency benchmark.

<video controls preload="metadata" poster="../assets/evidence/detector-comparison-poster.webp" width="100%">
  <source src="https://media.githubusercontent.com/media/jburo1/omniseer/master/studies/detector_comparison/scan_final_recal/evidence/controlled_replay_2x3.mp4" type="video/mp4" />
  Your browser cannot play this video. <a href="https://github.com/jburo1/omniseer/blob/master/studies/detector_comparison/scan_final_recal/evidence/controlled_replay_2x3.mp4">Open the controlled replay media</a>.
</video>

*The controlled replay is a 3-row × 2-column layout pairing FP with INT8 or
Hybrid by model size. It is target-hardware-derived evidence from the same
repaired source frames for every configuration.*

## Results

### Controlled presence/visibility replay

| Configuration | Visible-frame detections | Rate | Absent dog frames | Absent cat frames |
| --- | ---: | ---: | ---: | ---: |
| v2-S FP | 2,126 / 5,679 | 37.4% | 34 | 0 |
| v2-S INT8 | 1,853 / 5,679 | 32.6% | 6 | 0 |
| v2-M FP | 2,367 / 5,679 | 41.7% | 45 | 0 |
| v2-M INT8 | 2,055 / 5,679 | 36.2% | 23 | 0 |
| v2-L FP | 2,363 / 5,679 | 41.6% | 33 | 0 |
| v2-L Hybrid | 1,440 / 5,679 | 25.4% | 0 | 0 |

Every configuration detected `person` on all 293 / 293 annotated visible
frames. v2-M FP led aggregate visible-frame coverage by a narrow margin over
v2-L FP. The zero absent-dog count for v2-L Hybrid must be read alongside its
lowest aggregate visible-frame rate; it is not a general precision claim.

### Independent physical-run summaries

| Configuration | Outcome | First detection | Success | Target loss | Mean consumer FPS | Inference p50 / p95 | Source-age p95 | Memory-used p95 |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| v2-S FP | success | 24.6 s | 53.4 s | 0 | 7.90 | 117.28 / 153.95 ms | 181.50 ms | 2955.47 MB |
| v2-S INT8 | success | 24.6 s | 50.5 s | 0 | 16.54 | 59.19 / 60.76 ms | 95.48 ms | 2959.68 MB |
| v2-M FP | success | 25.2 s | 53.9 s | 0 | 4.37 | 225.12 / 251.43 ms | 282.81 ms | 3349.19 MB |
| v2-M INT8 | success | 24.5 s | 51.1 s | 0 | 10.04 | 96.22 / 106.47 ms | 139.42 ms | 2999.90 MB |
| v2-L FP | success | 25.4 s | 56.1 s | 0 | 2.64 | 380.25 / 400.98 ms | 438.76 ms | 3306.73 MB |
| v2-L Hybrid | success | 24.9 s | 57.8 s | 0 | 4.65 | 205.55 / 229.90 ms | 262.32 ms | 4157.62 MB |

All six trials completed without target loss. The retained throttle field was
unavailable for every trial, so this evidence does not establish that thermal
throttling was absent.

## Engineering interpretation

v2-M FP has the highest controlled coverage, while v2-S INT8 has the highest
author-reported throughput among the physical summaries. v2-M INT8 is the
strongest observed coverage/throughput compromise for this scene: 36.2%
controlled coverage and 10.04 FPS in its independent run. Relative to their
FP counterparts, v2-S INT8 improves inference p50/p95 by 1.98×/2.53× and
throughput by 2.09×; v2-M INT8 by 2.34×/2.36× and 2.30×; v2-L Hybrid by
1.85×/1.74× and 1.76×. v2-L Hybrid is faster than v2-L FP but has lower
controlled coverage and the highest observed memory use.

Success times span 50.5–57.8 s, much less than the 59.19–380.25 ms inference
p50 range, which suggests scan/control timing is also material in this bounded
trial. The ratios describe independent end-to-end measurements; they do not
establish causal model differences.

## Evidence and reproducibility

The tracked replay provenance, six replay JSONLs, visibility annotations, and
comparison report make the controlled metrics independently recomputable.
Rerendering the video additionally requires the retained source stream. The
complete RunBundles, source transport stream, and raw physical manifests remain
ignored local evidence. Apart from the public v2-L FP RunBundle, the physical
timing values and derived ratios are author-reported summaries from retained
local bundles and cannot be independently recomputed from the public
repository.

For raw-artifact inspection, use the repository's [replay provenance](https://github.com/jburo1/omniseer/blob/master/studies/detector_comparison/scan_final_recal/evidence/replay_provenance.json),
[replay JSONLs](https://github.com/jburo1/omniseer/tree/master/studies/detector_comparison/scan_final_recal/evidence/replay_jsonl),
[visibility annotations](https://github.com/jburo1/omniseer/blob/master/studies/detector_comparison/scan_final_recal/visibility.txt),
and [physical-run summary](https://github.com/jburo1/omniseer/blob/master/studies/detector_comparison/scan_final_recal/physical_runs.yaml).

## Limitations

This is one-scene presence/visibility evidence, not mAP, bounding-box recall,
or general detector accuracy. Controlled replay is not latency benchmarking;
the physical results are one unreplicated run per configuration and cannot
establish statistical significance or causal performance differences. Detection
coverage comes from fixed-source replay, while runtime behavior comes from the
independent trials; physical-run detections must not be used as controlled
accuracy comparisons.
