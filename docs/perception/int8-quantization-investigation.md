# YOLO-World v2 INT8 Quantization Investigation

This record preserves the conclusions of the 2026-08-29--31 RKNN
quantization investigation. Raw Toolkit simulator snapshots are intentionally
ignored under `artifacts/quant_analysis/`; the compact final RK3588 evaluation
evidence is retained in the v2-L INT8 study and indexed in
`studies/quantization/yolo_world_v2l_int8/evidence/EVIDENCE_INDEX.md`.

## Conclusions

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
- The final TD01 result is a useful experimental/diagnostic mixed-precision
  mitigation, but is not FP-equivalent or a validated production replacement:
  it retained 789 of 1,301 FP detections (60.6%) and had severe
  class-specific degradation.

## Final representative RK3588 evaluation

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
| v2-L TD01 base hybrid | 295 | 1,030 | 789 / 1,301 | 60.6% | 75.084 s / 3.996 fps |

The timing column is whole-process replay time: model/text initialization,
PNG decode, CPU letterbox preprocessing, RKNN inference, postprocessing, and
JSONL serialization. It is **not** isolated RKNN inference latency.

For TD01, the strong FP detection-retention classes were person (81/86,
94.2%), potted plant (119/127, 93.7%), tripod (58/63, 92.1%), cup (47/52,
90.4%), garbage can (120/137, 87.6%), and bottle (66/79, 83.5%). Severe
FP-relative degradation remained for handbag (2/48, 4.2%), bag (4/59, 6.8%),
cellphone (0/5), book (18/73, 24.7%), subwoofer (7/27, 25.9%), and desk
(37/124, 29.8%). These values are neither ground-truth precision nor recall.

## Final engineering decision

- v2-L FP remains the quality reference.
- Recalibrated v2-L INT8 is unsuitable.
- TD01 is an experimental/diagnostic mixed-precision mitigation, not a
  validated production replacement.
- The proprietary Toolkit2/backend root cause remains unresolved.
- The root-cause investigation is closed. Future work should reopen it only
  when a vendor bug report or a production-quality v2-L mixed-precision
  deployment is specifically required; TD01's 60.6% retention alone is not a
  reason to reopen it.

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

The exact local input hashes recorded during the investigation were:

```text
yolo_world_v2_s.onnx  0b5f46756f56fbaba1178413fb5bc0db2660353adc821dd0744161c9cba2bf41
yolo_world_v2_m.onnx  f3aeb4c93001d9faef04de68f573693238322030b0a2c1b0e0228bb460f31326
yolo_world_v2_l.onnx  b7d5cd79a887ff6d78d711a3676acba427545e28383de363913b5ecd63e6649a
dataset.txt           4d55fef03384759c8753f83bbaed24714687c13b6a17f43a62db3ec0c1a0120d
calibration_text_outp.npy
                      9cfb78cef21252785897d52519c91af583a7a7117e4fd541716563b360698d8e
bus.jpg               33b198a1d2839bb9ac4c65d61f9e852196793cae9a0781360859425f6022b69c
```

The v2-S, v2-M, and v2-L ONNX artifacts above had sizes 51,067,624,
113,524,529, and 187,339,766 bytes, respectively. Regenerate and record new
hashes whenever the exporter, source checkpoint, calibration dataset, CLIP
snapshot, or controlled input changes.

## Raw traces and retained final evidence

The investigation's raw Toolkit output remains intentionally excluded from
version control. Do not delete experimental artifacts as part of this
investigation closeout. The final 300-frame result bundle is distinct compact
evidence and is retained as described in the evidence index; the frozen source
and PNG frames remain reproducible local inputs rather than Git content.

Earlier host-generated snapshot directories are candidates for later cleanup
only after the evidence review. The broad
`artifacts/quant_analysis/v2l/` directory remains generated local output;
the Git-retained evidence index and final result bundle are under `studies/`.
Model artifacts, calibration inputs, source changes, and findings required to
recreate the investigation are retained elsewhere.
