# Final v2-L 300-frame evaluation

FP is a behavioral reference, not ground truth. Matching is class-aware, one-to-one IoU >= 0.50.

| model | active frames | detections | FP retention | whole-process s | whole-process fps |
|---|---:|---:|---:|---:|---:|
| v2-L FP | 300 | 1301 | — | 118.751 | 2.526 |
| v2-L recalibrated INT8 | 0 | 0 | 0.0% | 58.653 | 5.115 |
| v2-L TD01 base hybrid | 295 | 1030 | 60.6% | 75.084 | 3.996 |

## v2-L recalibrated INT8 vs FP

- FP detection retention: 0.0% (0/1301)
- Unmatched FP detections: 1301; extra candidate detections: 0
- Matched IoU mean/median: —/—

## v2-L TD01 base hybrid vs FP

- FP detection retention: 60.6% (789/1301)
- Unmatched FP detections: 512; extra candidate detections: 241
- Matched IoU mean/median: 0.976/0.981

Whole-process time includes initialization, decode, preprocessing, inference, postprocessing, and serialization. It is not inference latency.
