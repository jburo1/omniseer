# Results: recalibrated 360° detector comparison

## Controlled replay

The six saved replay streams each contain 1,222 records for inclusive source frames 0–1221. Metrics below were independently recomputed from those JSONLs and [visibility annotations](visibility.txt).

| Configuration | Visible-frame detections | Rate | Absent dog frames | Absent cat frames |
| --- | ---: | ---: | ---: | ---: |
| v2-S FP | 2,126 / 5,679 | 37.4% | 34 | 0 |
| v2-S INT8 | 1,853 / 5,679 | 32.6% | 6 | 0 |
| v2-M FP | 2,367 / 5,679 | 41.7% | 45 | 0 |
| v2-M INT8 | 2,055 / 5,679 | 36.2% | 23 | 0 |
| v2-L FP | 2,363 / 5,679 | 41.6% | 33 | 0 |
| v2-L Hybrid | 1,440 / 5,679 | 25.4% | 0 | 0 |

Every configuration detected `person` on all 293 / 293 annotated visible frames. The 5,679 denominator is the inclusive total over 19 in-vocabulary class-interval sets. `laptop`, `computer monitor`, and `sofa` were excluded because they were outside the comparison vocabulary; they were not treated as negative evidence.

v2-M FP led the aggregate rate by a narrow margin over v2-L FP. Both FP configurations exceeded their corresponding INT8 configurations in this fixed scene. The no-absent-dog result for v2-L Hybrid must be read together with its lowest aggregate visible-frame rate, not as a general precision claim.

## Independent physical runs

Each row is one independent physical ROCK 5B+ run. The values were checked against the retained local trial bundles and the published [comparison report](evidence/comparison_report.html).

| Configuration | Outcome | First detection | Success | Target-loss episodes | Mean consumer FPS | Inference p50 | Inference p95 | Source-age p95 | CPU p95 | Memory-used p95 | SoC temperature p95 | Thermal-throttling status |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| v2-S FP | success | 24.6 s | 53.4 s | 0 | 7.90 | 117.28 ms | 153.95 ms | 181.50 ms | 32.92% | 2955.47 MB | 48.08 C | not recorded (signal unavailable) |
| v2-S INT8 | success | 24.6 s | 50.5 s | 0 | 16.54 | 59.19 ms | 60.76 ms | 95.48 ms | 36.74% | 2959.68 MB | 48.08 C | not recorded (signal unavailable) |
| v2-M FP | success | 25.2 s | 53.9 s | 0 | 4.37 | 225.12 ms | 251.43 ms | 282.81 ms | 46.12% | 3349.19 MB | 52.69 C | not recorded (signal unavailable) |
| v2-M INT8 | success | 24.5 s | 51.1 s | 0 | 10.04 | 96.22 ms | 106.47 ms | 139.42 ms | 31.94% | 2999.90 MB | 49.00 C | not recorded (signal unavailable) |
| v2-L FP | success | 25.4 s | 56.1 s | 0 | 2.64 | 380.25 ms | 400.98 ms | 438.76 ms | 43.27% | 3306.73 MB | 52.69 C | not recorded (signal unavailable) |
| v2-L Hybrid | success | 24.9 s | 57.8 s | 0 | 4.65 | 205.55 ms | 229.90 ms | 262.32 ms | 45.50% | 4157.62 MB | 50.85 C | not recorded (signal unavailable) |

Compared with each FP counterpart, v2-S INT8 is 1.98× faster at inference p50, 2.53× faster at p95, and 2.09× higher in throughput; v2-M INT8 is 2.34×/2.36×/2.30×; and v2-L Hybrid is 1.85×/1.74×/1.76×. The ratios describe these independent end-to-end runs, not controlled replay timing.

Time to success spans 50.5–57.8 s while inference p50 spans 59.19–380.25 ms, consistent with scan/control timing being a material component of this bounded trial. All six runs succeeded without target loss. Their missing throttle signal means the table does not establish that thermal throttling was absent.

These results measure class presence against manual visibility ranges in one scene. They are not mAP, object-localization or bounding-box recall, a calibrated precision/recall evaluation, latency benchmarking from the controlled replay, or evidence of detector accuracy in other scenes, viewpoints, lighting, instances, or vocabularies. The physical-run measurements are not replicated benchmark estimates or proof of causal model differences.
