---
description: "One public ROCK 5B+ v2-M INT8 target-acquisition RunBundle: bounded visual framing completed in 51.1 seconds with zero target-loss episodes."
---

# Target Acquisition

## Engineering question

Can open-vocabulary YOLO-World v2-M INT8 running through RKNN on a ROCK 5B+ support a complete, reviewable execution of Omniseer's bounded visual target-acquisition and framing behavior?

## Observed physical run

The public `v2m_int8_scene_1` RunBundle records one successful physical-robot execution for the configured `person` target. The controller scanned, acquired a stable detection, centered and framed it, then stopped with terminal reason `framed`.

[![Terminal person detection and framing capture](https://github.com/jburo1/omniseer/raw/master/studies/autonomy/v2m_int8_target_acquisition/run/evidence/annotated/capture_frame_2924.jpg)](https://github.com/jburo1/omniseer/blob/master/studies/autonomy/v2m_int8_target_acquisition/run/evidence/annotated/capture_frame_2924.jpg)

*Terminal capture from frame 2924. The matching `target_framed` event is retained in the public autonomy trace.*

| Measurement | Observed value |
| --- | ---: |
| First `person` detection | **24.5 s** |
| Terminal `framed` success | **51.1 s** |
| Target-loss episodes | **0** |
| Mean consumer throughput | **10.04 FPS** |
| RKNN inference p50 / p95 | **96.22 / 106.47 ms** |
| Source-age p95 | **139.42 ms** |

## Engineering conclusion

This artifact demonstrates one end-to-end execution of bounded target acquisition and framing on the stated hardware and model configuration. The full RunBundle preserves the configuration, provenance, autonomy trace, detections, timing telemetry, evidence frames, media, rosbag, and derived report needed to inspect that execution.

[Open the complete study and RunBundle on GitHub](https://github.com/jburo1/omniseer/tree/master/studies/autonomy/v2m_int8_target_acquisition){ .md-button .md-button--primary }

## Limitations

This is one representative successful run, not a benchmark aggregate or a claim of statistical significance, general detector accuracy, navigation-based search, global exploration, or learned control. For fixed-source detector coverage and six physical case-study summaries, see [Detector Comparison](detector-comparison.md).
