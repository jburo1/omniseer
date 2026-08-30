# YOLO-World v2 INT8 Quantization Investigation

This record preserves the conclusions of the 2026-08-29 RKNN quantization
investigation. Raw Toolkit simulator snapshots are intentionally ignored under
`artifacts/quant_analysis/`; they are reproducible diagnostic output, not
source evidence.

## Conclusions

- The original controlled S/M/L FP-versus-INT8 comparison found viable INT8
  behavior for v2-S and v2-M, while v2-L INT8 diverged materially from its FP
  result.
- Recalibration with 100 representative robot frames preserved the viable S/M
  result but did not make v2-L INT8 deployment-ready.
- The native image input binding was corrected to bind RGB888 as `UINT8`,
  `NHWC`, and `pass_through=0`, allowing RKNN runtime quantization instead of
  passing RGB pixels through as the model's declared floating input type.
- On the controlled `bus.jpg` input, v2-M is a healthy layer-wise control.
  It is evidence for this investigation, not a declaration of a final
  supported model baseline.
- The v2-L INT8 simulator error becomes material around
  `neck/top_down_layers.0` and compounds through later top-down, bottom-up,
  and detection-head layers. v2-L INT8 is therefore not deployment-ready;
  v2-S and v2-M remain viable pending hardware validation.
- An initial v2-L hybrid experiment kept the two
  `neck/top_down_layers.0/final_conv` outputs in FP16. Its current status is
  diagnostic-only: it has not completed a controlled comparison or hardware
  validation and is not a deployment candidate.
- All conclusions above remain host-simulator results. Real RK3588 hardware
  inference, latency, thermal behavior, and end-to-end detection validation
  are still pending.

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

## Discarded raw traces

The investigation's raw Toolkit output is intentionally excluded from version
control. After this document and its hashes have been reviewed, these local
generated directories are safe to delete:

```text
artifacts/quant_analysis/v2m/
artifacts/quant_analysis/v2l/
```

They contain host-generated snapshots and reports only; the model artifacts,
calibration inputs, source changes, and findings required to recreate the
investigation are retained elsewhere.
