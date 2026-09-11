# Detector comparison: recalibrated 360° scene

The canonical [six-model YOLO-World comparison](https://github.com/jburo1/omniseer/blob/master/studies/detector_comparison/scan_final_recal/README.md)
publishes a controlled replay, six replay JSONLs, visibility annotations,
repaired provenance, a comparison report, and a summary of six independent
physical ROCK 5B+ runs.

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

v2-S INT8 has the highest observed throughput. Against their FP counterparts,
v2-S INT8 improves inference p50/p95 by 1.98×/2.53× and throughput by 2.09×;
v2-M INT8 by 2.34×/2.36× and 2.30×; and v2-L Hybrid by 1.85×/1.74× and
1.76×. v2-M INT8 is therefore the most plausible coverage/throughput
compromise for this scene. v2-L Hybrid is faster than v2-L FP, but has lower
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
