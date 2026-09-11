# Results: recalibrated 360° detector comparison

The six saved replay streams each contain 1,222 records for inclusive source
frames 0–1221. Metrics below were independently recomputed from those JSONLs
and [visibility annotations](visibility.txt).

| Configuration | Visible-frame detections | Rate | Absent dog frames | Absent cat frames |
| --- | ---: | ---: | ---: | ---: |
| v2-S FP | 2,126 / 5,679 | 37.4% | 34 | 0 |
| v2-S INT8 | 1,853 / 5,679 | 32.6% | 6 | 0 |
| v2-M FP | 2,367 / 5,679 | 41.7% | 45 | 0 |
| v2-M INT8 | 2,055 / 5,679 | 36.2% | 23 | 0 |
| v2-L FP | 2,363 / 5,679 | 41.6% | 33 | 0 |
| v2-L Hybrid | 1,440 / 5,679 | 25.4% | 0 | 0 |

Every configuration detected `person` on all 293 / 293 annotated visible
frames. The 5,679 denominator is the inclusive total over 19 in-vocabulary
class-interval sets. `laptop`, `computer monitor`, and `sofa` were excluded
because they were outside the comparison vocabulary; they were not treated as
negative evidence.

v2-M FP led the aggregate rate by a narrow margin over v2-L FP. Both FP
configurations exceeded their corresponding INT8 configurations in this fixed
scene. The no-absent-dog result for v2-L Hybrid must be read together with its
lowest aggregate visible-frame rate, not as a general precision claim.

These results measure class presence against manual visibility ranges in one
scene. They are not mAP, object-localization or bounding-box recall, a calibrated
precision/recall evaluation, latency benchmarking, or evidence of detector
accuracy in other scenes, viewpoints, lighting, instances, or vocabularies.
