# `scan_final` v2-L INT8 vs Hybrid replay video

- Output: `scan_final_v2l_int8_vs_hybrid.mp4`
  (`0f15bd410ba7521178e4b6aed3679b25df0419165c57e19fa7a23877a8760325`)
- Source RunBundle: `runs/scan_final`; `video/source.ts`
  (`9cede604070b3bbdbcb31f6040a84c9637f4894522b480a46b6faf7fdca40058`).
  The comparator decodes this immutable transport stream once and applies the
  established in-memory `rockchip_preview_circular_wrap_v1` correction.
- Left model: `runs/model_artifacts/yolo_world_v2_l_i8_recal.rknn`
  (`337012ef7690c39dbbbb0fc73ce4396a5c9ff2411032cca47fb012b66045faef`).
- Right model: `runs/model_artifacts/yolo_world_v2_l_hybrid_td01.rknn`
  (`20e43523ab4221fd755553030dbc58943f457839d5583b1f2f29954489c2ef92`).
- Classes: `config/classes/task.txt`; score threshold `0.25`, NMS IoU `0.45`,
  maximum detections `100`.
- Rendered source frames: `1222` at `30 fps`; 2560×720 H.264/yuv420p.

Command used from `/omniseer`:

```bash
scripts/omni runs compare runs/scan_final --v2l-int8-vs-hybrid --output studies/quantization/yolo_world_v2l_int8/evidence/scan_final_v2l_int8_vs_hybrid.mp4 --classes config/classes/task.txt
```

The accompanying JSON provenance and one JSONL stream per panel were emitted
by the same controlled comparator invocation.
