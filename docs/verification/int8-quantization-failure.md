---
description: "RK3588 investigation: recalibrated YOLO-World v2-L INT8 retained no detections; a mixed-precision probe recovered partial behavior and localized the failure."
---

# INT8 Quantization Failure Analysis

## Engineering question

Could recalibrated YOLO-World v2-L INT8 preserve useful open-vocabulary behavior on RK3588, and could a targeted mixed-precision layout identify a practical recovery path?

## Final RK3588 evaluation

The canonical evaluation replays 300 frozen representative frames through the existing detector path on RK3588. FP is the behavioral reference rather than ground truth; retention means same-class, one-to-one matches at IoU >= 0.50.

| Model | Active frames | Detections | FP detections retained |
| --- | ---: | ---: | ---: |
| v2-L FP | 300 | 1,301 | — |
| v2-L recalibrated INT8 | 0 | 0 | **0 / 1,301 (0.0%)** |
| v2-L TD01 mixed precision | 295 | 1,030 | **789 / 1,301 (60.6%)** |

[Watch the retained INT8-versus-TD01 RK3588 replay](https://github.com/jburo1/omniseer/blob/master/studies/quantization/yolo_world_v2l_int8/evidence/scan_final_v2l_int8_vs_hybrid.mp4){ .md-button .md-button--primary }

## Engineering conclusion

Recalibrated v2-L INT8 is unsuitable for this model and conversion contract: it collapsed before post-processing in the final replay. TD01 is diagnostic evidence of partial recovery, not an FP-equivalent or validated production replacement. The healthy narrow exMatMul micrograph and failed expanded projection context locate the issue in the classification/projection path and its precision arrangement; the exact Toolkit2/backend mechanism remains unresolved.

[Open the complete study, results, and provenance on GitHub](https://github.com/jburo1/omniseer/tree/master/studies/quantization/yolo_world_v2l_int8){ .md-button }

## Limitations

This is a fixed 300-frame RK3588 comparison with recorded models, thresholds, and replay path. It is not ground-truth precision or recall, broad deployment validation, isolated latency measurement, or proof of a proprietary Toolkit2/backend root cause. The frozen source stream and extracted frames are identified by hashes but are non-retained external inputs, so independently rerunning the evaluation requires access to those source artifacts.
