---
description: "Supporting RKNN conversion, diagnostics, host-analysis procedure, and hashes for the YOLO-World v2-L INT8 investigation."
---

# YOLO-World v2 INT8 Quantization Technical Appendix

This is a supporting technical appendix to the
[v2-L INT8 Quantization Failure Analysis](../verification/int8-quantization-failure.md).
Read that page for the engineering question, final results, interpretation,
decision, evidence boundaries, and limitations. This appendix preserves the
2026-08-29--31 RKNN conversion contract, layer-level diagnostics, host-analysis
procedure and hashes.

## Supporting technical conclusions

- v2-L recalibrated INT8 has a classification collapse before YOLO
  postprocessing. The final RK3588 replay produced zero detections on all 300
  representative frames.
- TD0 did not recover the failure. TD01 substantially recovered the 80x80 and
  40x40 classification heads; expanding FP16 across the whole neck did not
  materially improve further.
- The remaining failure localizes around the lowered projection / exMatMul
  path under particular hybrid-precision arrangements. The exact proprietary
  Toolkit2/backend mechanism remains unresolved.
- The isolated exMatMul micrograph remained healthy. The expanded
  projection-context reproducer instead collapsed exactly on RK3588 for the
  failed hybrid layout, while its FP16 and healthy-hybrid controls remained
  non-constant. This distinguishes the required projection context from an
  isolated matrix multiply alone.
- The final v2-L TD01 mixed-precision result is a useful experimental/diagnostic
  mitigation, but is not FP-equivalent or a validated production replacement:
  it produced 1,030 total detections, of which 789 matched 1,301 v2-L
  FP-reference detections (60.6% retention), and had severe
  class-specific degradation.

## Supporting final RK3588 evaluation detail

The canonical final evidence is
`studies/quantization/yolo_world_v2l_int8/evidence/final_multiframe_rk3588/results/`, indexed with
hashes and file-retention guidance in
`studies/quantization/yolo_world_v2l_int8/evidence/EVIDENCE_INDEX.md`. It replays 300 frozen
representative frames using the existing `vision_replay` detector path, no
warmup, score threshold 0.25, and NMS IoU threshold 0.45.

FP is a behavioral reference, not ground truth. FP-relative agreement uses
the same class, IoU >= 0.50, and deterministic one-to-one matching: eligible
pairs are ordered by descending IoU, then FP index, then candidate index and
accepted greedily.

| Model | Active frames | Detections | Matched FP detections | FP detection retention | Whole-process replay |
| --- | ---: | ---: | ---: | ---: | ---: |
| v2-L FP | 300 | 1,301 | — | — | 118.751 s / 2.526 fps |
| v2-L recalibrated INT8 | 0 | 0 | 0 / 1,301 | 0.0% | 58.653 s / 5.115 fps |
| v2-L TD01 mixed-precision | 295 | 1,030 | 789 / 1,301 | 60.6% | 75.084 s / 3.996 fps |

The timing column is whole-process replay time: model/text initialization,
PNG decode, CPU letterbox preprocessing, RKNN inference, postprocessing, and
JSONL serialization. It is **not** isolated RKNN inference latency.

For TD01, the strong FP detection-retention classes were person (81/86,
94.2%), potted plant (119/127, 93.7%), tripod (58/63, 92.1%), cup (47/52,
90.4%), garbage can (120/137, 87.6%), and bottle (66/79, 83.5%). Severe
FP-relative degradation remained for handbag (2/48, 4.2%), bag (4/59, 6.8%),
cellphone (0/5), book (18/73, 24.7%), subwoofer (7/27, 25.9%), and desk
(37/124, 29.8%). These values are neither ground-truth precision nor recall.

## Decision context

The canonical decision is recorded in the
[v2-L INT8 Quantization Failure Analysis](../verification/int8-quantization-failure.md):
v2-L FP remains the quality reference; recalibrated v2-L INT8 is unsuitable;
and v2-L TD01 mixed-precision is an experimental diagnostic mitigation, not a validated production
replacement. The proprietary Toolkit2/backend root cause remains unresolved.
Future work should reopen the root-cause investigation only for a vendor bug
report or a production-quality v2-L mixed-precision deployment; TD01's 60.6%
retention alone is not a reason to reopen it.

## Reproducible host analysis

Build the pinned host model-builder image and run any variant:

```bash
scripts/omni model image
scripts/omni model analyze --variant v2s
scripts/omni model analyze --variant v2m
scripts/omni model analyze --variant v2l
```

`model analyze` runs only the RKNN Toolkit2 host simulator. It neither opens
ADB nor contacts a board. It rebuilds the selected ONNX as INT8 and writes a
throwaway report and snapshots to `artifacts/quant_analysis/<variant>/`; that
directory must otherwise be empty so a report cannot be mistaken for a new
run. Delete it before rerunning the same variant.

The controlled input is
`models/source/yolo_world/calibration/bus.jpg`. Analysis uses the runtime text
input `[1,80,512]` `float32`: `person`, `bus`, then 78 `nothing` rows. This is
deliberately distinct from the 80-label calibration vocabulary used to build
the representative dataset.

## Fixed inputs and conversion contract

| Item | Value |
| --- | --- |
| Builder | RKNN Toolkit2 `2.1.0+708089d1`, source `deaba85fc437a28db0b0c29f27d8929f4c5816a1` |
| YOLO-World exporter | Rockchip fork `b8b0fe9beffa9564306a798f6e443c9fe88057af` |
| Target conversion contract | RK3588; `images=[1,3,640,640]`, `texts=[1,80,512]` |
| Image conversion | `mean=[0,0,0]`, `std=[255,255,255]`; named `images`, `texts` inputs |
| Quantization | Toolkit `build(do_quantization=True)` using `models/source/yolo_world/calibration/dataset.txt` |
| Calibration set | 100 robot frames plus one generated fixed-shape CLIP text input per dataset entry |
| Postprocess thresholds for controlled runtime comparisons | score `0.25`; NMS IoU `0.45` |
