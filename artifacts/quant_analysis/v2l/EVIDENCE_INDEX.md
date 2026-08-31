# v2-L RKNN quantization evidence index

Frozen inventory for the investigation wrap-up.  This index names existing
artifacts only; it does not make a deployment-readiness claim and it is not an
experiment record.  Paths are repository-relative.  Generated model and host
artifact directories remain ignored; this index and the small RK3588 output
collection are deliberately force-added to Git.

## Canonical detector artifacts

| Evidence | Canonical path | SHA-256 / provenance |
| --- | --- | --- |
| FP v2-L reference | `artifacts/models/yolo_world_v2_l_fp.rknn` | `b6112550be6a4b2a019619053a3015914f99a5ba7216c078115a6dfbdef75df3` |
| v2-L source ONNX | `artifacts/models/yolo_world_v2_l.onnx` | `b7d5cd79a887ff6d78d711a3676acba427545e28383de363913b5ecd63e6649a` |
| Original v2-L INT8 | `artifacts/models/yolo_world_v2_l_i8.rknn` | `3b8dbbbf02952cc9c83b9287c54fdde7a7f5d923d22605594ddbac27284b87a8` |
| Recalibrated v2-L INT8 failure | `artifacts/models/yolo_world_v2_l_i8_recal.rknn` | `337012ef7690c39dbbbb0fc73ce4396a5c9ff2411032cca47fb012b66045faef` |
| Original TD0 hybrid | `runs/model_artifacts/yolo_world_v2_l_hybrid.rknn` plus `_config/` | `6a590575f5de60ecdca4a5b01346932eab05aa6b3479f876b53b3c4bf51fd2f5` |
| TD01 base hybrid | `runs/model_artifacts/yolo_world_v2_l_hybrid_td01.rknn` plus `_config/` | `20e43523ab4221fd755553030dbc58943f457839d5583b1f2f29954489c2ef92` |
| Current practical TD01 mitigation | `runs/model_artifacts/yolo_world_v2_l_hybrid_td01_clspreds0_mm_inputs_fp16.rknn` plus `_config/` | model `df72d337ad03a7b90c5a96f2b44495fd25f5747094019b038b2823d5133d398e`; config `d9f93c95d849307c86bf5f48705ff31002b81ff779aa2693d8b8e5badcbdc365` |
| Full-neck hybrid | `runs/model_artifacts/yolo_world_v2_l_hybrid_neck.rknn` plus `_config/` | `fa197bed3d9e0a70bf0803b0139bf3350c4efbef895f03d44b2000a618905439` |

The TD01 mitigation config is derived from the stored TD01 output-FP16 control
and retains the `cls_preds.0.2` projection/reshape/exMatMul boundary plus
`texts_tp-rs` in FP16.  Keep the `.rknn` and all three generated step-1 files
(`.model`, `.data`, `.quantization.cfg`) together.

## Classification-path localization evidence

Retain both rows: the exMatMul micrograph is the narrow control, and the
projection-context reproducer supplies the real v2-L classifier context.

| Evidence | Canonical path | Required contents |
| --- | --- | --- |
| exMatMul control | `artifacts/quant_analysis/v2l/cls80_exmatmul_toolkit210_final/` | `report.json`, run log, ONNX, three RKNN variants, step-1 configs, input/reference/output arrays |
| final minimal hardware reproducer | `artifacts/quant_analysis/v2l/cls80_projection_context_toolkit210_run2/` | `report.json`, `cls80_projection_context_*.rknn`, ONNX, step-1 configs, projection/image/text/reference/output arrays |
| reproducer source | `tools/model/reproduce_cls80_exmatmul.py`; `tools/model/reproduce_cls80_projection_context.py` | source must remain with the artifacts |

`cls80_projection_context_toolkit210_run2/report.json` records Toolkit2
`2.1.0+708089d1`, the full-ONNX hash above, input image hash
`33b198a1d2839bb9ac4c65d61f9e852196793cae9a0781360859425f6022b69c`,
and text-input hash
`f1eb7a48d7cb69fbc5aec5f943b2f3b752ebbe5f6d85ed8a3e4ed7c271bf3311`.
It also records the exact RKNN artifact hashes and comparison metrics.

## Inputs, host diagnostics, and version contract

- Calibration inputs: `models/source/calibration_images/` (100 frames),
  `models/source/yolo_world/calibration/dataset.txt`, and
  `models/source/yolo_world/calibration/calibration_text_outp.npy`.
- Controlled host input: `models/source/yolo_world/calibration/bus.jpg` and
  `artifacts/quant_analysis/v2l/texts_person_bus_padded.npy`.
- Host evidence: `artifacts/quant_analysis/v2l/error_analysis.txt`,
  `map_name_to_file.txt`, and
  `hybrid/topdown0_block_fp16/host_detection_compare.log` with its scripts.
- Builder contract: Toolkit2 `2.1.0+708089d1`, Toolkit source
  `deaba85fc437a28db0b0c29f27d8929f4c5816a1`, Rockchip exporter
  `b8b0fe9beffa9564306a798f6e443c9fe88057af`; see
  `docs/perception/int8-quantization-investigation.md` for the already
  recorded ONNX, calibration, and input hashes.

## RK3588 one-frame evidence

`artifacts/quant_analysis/rk3588_v2l_20260831/` is an intact copy of the
Radxa directory of that name.  It contains the corrected single-frame AVI and
frame hashes, plus paired `.log` and `.jsonl` output for FP, recalibrated INT8,
TD0, TD01, full-neck, the TD01 output control, and each classifier-path bisect.
The current mitigation is represented by
`v2l_hybrid_td01_clspreds0_mm_inputs_fp16.{log,jsonl}`.  The local copy was
byte-compared with the board after transfer.

The RK3588 logs capture actual tensor metadata and output statistics.
`rk3588_environment.txt` records the board kernel, runtime `2.3.0`
(`c949ad889d`, 2024-11-07), and board checkout.  The normal
`/proc/driver/rknpu/version` path was absent, so the NPU driver version remains
unknown and must be captured before a hardware-version-sensitive conclusion.
No further board copy is required for the evidence currently present there.

## Non-canonical duplicates / failed probes

Keep them locally until final documentation review, but do not use them as the
citation target: `cls80_exmatmul_toolkit210/`, `*_run2`, `*_run3`, `*_run4`,
and `cls80_projection_context_toolkit210_run1/` are earlier or duplicate host
generations.  `*_invalid*`, `*_range_only_invalid*`, `*_pre_guide_fix_invalid*`,
and the other named TD01 progression artifacts under `runs/model_artifacts/`
are bisect/probe material, not the practical mitigation.
