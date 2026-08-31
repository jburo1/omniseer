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
| TD01 base hybrid (diagnostic mitigation) | `runs/model_artifacts/yolo_world_v2_l_hybrid_td01.rknn` plus `_config/` | `20e43523ab4221fd755553030dbc58943f457839d5583b1f2f29954489c2ef92` |
| Full-neck hybrid | `runs/model_artifacts/yolo_world_v2_l_hybrid_neck.rknn` plus `_config/` | `fa197bed3d9e0a70bf0803b0139bf3350c4efbef895f03d44b2000a618905439` |

TD01 is a diagnostic mixed-precision mitigation, not an FP-equivalent or
production-validated replacement. Keep its `.rknn` and all three generated
step-1 files (`.model`, `.data`, `.quantization.cfg`) together. The
`yolo_world_v2_l_hybrid_td01_clspreds0_mm_inputs_fp16.rknn` artifact (model
`df72d337ad03a7b90c5a96f2b44495fd25f5747094019b038b2823d5133d398e`; config
`d9f93c95d849307c86bf5f48705ff31002b81ff779aa2693d8b8e5badcbdc365`) is a
failed classifier-path localization probe, not a mitigation or deployment
candidate.

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

### Final RK3588 reproduction

The final hardware reproduction is
`artifacts/quant_analysis/v2l/cls80_projection_context_toolkit210_run2/rk3588_execution_20260831/`.
Its canonical, Git-retained machine-readable comparison is `metrics.json`
(SHA-256 `77f8634499127aa1bd02a1228489b8dbd756d632df78d73b59ad7f7c82d688fb`).
It identifies RK3588 as the platform, the three exact model hashes, output
shapes/statistics, host/reference comparison metrics, and raw-output hashes.

On RK3588, only the failed-layout hybrid (`failed_fp16_island`) collapsed to
an exactly constant output (value `-11.993370056152344`, stddev `0.0`).  The
FP16 control and healthy-hybrid control remained non-constant (stddev
`5.930371901237637` and `5.876957691347017`, respectively).  The raw output
arrays remain in that directory as recoverable local evidence; their hashes,
recorded in `metrics.json`, are:

| Variant | Raw output file | SHA-256 |
| --- | --- | --- |
| Failed-layout hybrid | `failed_fp16_island_hardware_output.npy` | `42f4f41a873f54315a82837519d86d3f607550ec03f2dbe41a93f8175eb49ac4` |
| FP16 control | `full_fp16_hardware_output.npy` | `675fe8131019bf2402b0c7f7712b57539856c056075d495f0f261d0c0c1404bc` |
| Healthy-hybrid control | `healthy_int8_matmul_hardware_output.npy` | `b5a0092a00b6641cdf8710999d59ea189bea24d71e6682f276f1281c36296206` |

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
The TD01 base hybrid is the mitigation in this one-frame collection.  The
`v2l_hybrid_td01_clspreds0_mm_inputs_fp16.{log,jsonl}` pair is a
classifier-path localization probe, not mitigation evidence.  The local copy
was byte-compared with the board after transfer.

The RK3588 logs capture actual tensor metadata and output statistics.
`rk3588_environment.txt` records the board kernel, runtime `2.3.0`
(`c949ad889d`, 2024-11-07), and board checkout.  The normal
`/proc/driver/rknpu/version` path was absent, so the NPU driver version remains
unknown and must be captured before a hardware-version-sensitive conclusion.
No further board copy is required for the evidence currently present there.

## Canonical final 300-frame RK3588 evaluation

`artifacts/quant_analysis/v2l/final_multiframe_eval/results/` is the
canonical final representative evaluation. The retained compact evidence is
`summary.json`, `summary.md`, the three normalized `v2l_*.jsonl` files, and
the corresponding `raw/*_replay.jsonl`, `*.wall_time.txt`, `*.stdout.log`,
and `*.stderr.log` process records. `summary.json` binds every retained JSONL
file to its SHA-256, exact model hash, replay command, runtime setting, and
whole-process timing method.

The 300-frame JSONL results independently confirm the following FP-relative
comparison. FP is a behavioral reference, not ground truth. Matching requires
the same class and IoU >= 0.50, enumerates all eligible pairs, sorts by
descending IoU then FP index then candidate index, and greedily accepts
deterministic one-to-one pairs.

| Model | Active frames | Detections | Matched FP detections | FP detection retention | Whole-process replay |
| --- | ---: | ---: | ---: | ---: | ---: |
| v2-L FP | 300 | 1,301 | — | — | 118.751 s / 2.526 fps |
| v2-L recalibrated INT8 | 0 | 0 | 0 / 1,301 | 0.0% | 58.653 s / 5.115 fps |
| v2-L TD01 base hybrid | 295 | 1,030 | 789 / 1,301 | 60.6% | 75.084 s / 3.996 fps |

These are whole-process replay measurements, including initialization, PNG
decode, CPU letterbox preprocessing, RKNN inference, postprocessing, and
JSONL serialization. They are explicitly **not** isolated RKNN inference
latency.

TD01's notable FP detection retention was person 81/86 (94.2%), potted plant
119/127 (93.7%), tripod 58/63 (92.1%), cup 47/52 (90.4%), garbage can 120/137
(87.6%), and bottle 66/79 (83.5%). Its notable degradation was handbag 2/48
(4.2%), bag 4/59 (6.8%), cellphone 0/5, book 18/73 (24.7%), subwoofer 7/27
(25.9%), and desk 37/124 (29.8%). These are FP-relative agreement measures,
not ground-truth precision or recall.

The frozen dataset manifest is retained in Git at
`artifacts/quant_analysis/v2l/final_multiframe_eval/manifest.json` (SHA-256
`3a33a81643c31e4fb72b8a8406c825858baef2abef10b112e136cc6ca3a4c055`).
`summary.json` records that same manifest SHA-256,
`3a33a81643c31e4fb72b8a8406c825858baef2abef10b112e136cc6ca3a4c055`, source
SHA-256 `9cede604070b3bbdbcb31f6040a84c9637f4894522b480a46b6faf7fdca40058`,
and the task-class SHA-256 `86034945de1c68662432a8677b5c08fb1c965cb2069d5e5b1f634571f5b91e3b`.
The source transport stream (`runs/scan_final/video/source.ts`) and its 300
lossless PNG frames are intentionally not retained in Git: they are external,
reproducible artifacts identified by their recorded hashes and the manifest's
deterministic extraction recipe. Do not force-add either the source stream or
the PNG sequence. No experimental artifacts are deleted by this index.

## Non-canonical duplicates / failed probes

Keep them locally until final documentation review, but do not use them as the
citation target: `cls80_exmatmul_toolkit210/`, `*_run2`, `*_run3`, `*_run4`,
and `cls80_projection_context_toolkit210_run1/` are earlier or duplicate host
generations.  `*_invalid*`, `*_range_only_invalid*`, `*_pre_guide_fix_invalid*`,
and the other named TD01 progression artifacts under `runs/model_artifacts/`
are bisect/probe material, not the practical mitigation.
