# YOLO-World v2 Model Deployment

This host-only workflow produces the detector model consumed by Omniseer's
native RKNN vision path:

```text
YOLO-World v2-S, v2-M, or v2-L `.pth` -> ONNX (images + texts) -> RK3588 `.rknn`
```

It does not add training, export, ONNX, or RKNN Toolkit dependencies to
`docker/runtime/Dockerfile`. The checked-in runtime model remains unchanged.

## Pinned builder

Build the dedicated host-side image once from the checkout:

```bash
scripts/omni model image
```

The image pins:

- `airockchip/YOLO-World` commit
  `b8b0fe9beffa9564306a798f6e443c9fe88057af` (the Rockchip RKNN exporter);
- `airockchip/rknn-toolkit2` v2.1.0 commit
  `deaba85fc437a28db0b0c29f27d8929f4c5816a1`, including
  `rknn_toolkit2-2.1.0+708089d1` for CPython 3.8;
- the Rockchip model-zoo v2.1.0 commit
  `c2b7d00714b4e5d21266ab3003f3ca687ba0d57b` conversion contract:
  `images=[1,3,640,640]` and `texts=[1,80,512]` for `rk3588`.

Docker is run against the host daemon. When invoked in a devcontainer the
wrapper derives the host repository bind path; set
`OMNISEER_MODEL_HOST_REPO_ROOT=/host/path/to/omniseer` if that mount cannot be
detected.

## Local inputs

Download the pinned, ignored source assets once from a fresh checkout:

```bash
scripts/omni model assets
```

This uses `curl` with immutable Hugging Face revisions for
`wondervictor/YOLO-World` (`4340b03f4f59f46279a6581bbb818e0f77765d4d`) and
`openai/clip-vit-base-patch32` (`3d74acf9a28c67741b2f4f2ea7635f0aaf6f0268`),
plus Rockchip Model Zoo commit
`bad6c7334531becaf90a561988519b7bec34d0ab` for `bus.jpg` and
`coco_text_outp.npy`. It creates the required one-line local calibration
dataset. Existing non-empty files are retained, downloads use `.part` files
until complete, and every required asset is validated before the command
reports success. `models/` remains ignored.

The only variant-specific mapping is:

| Variant | Checkpoint | Rockchip export config | Artifact stem |
| --- | --- | --- | --- |
| `v2s` | `yolo_world_v2_s_obj365v1_goldg_pretrain-55b943ea.pth` | `configs/pretrain/yolo_world_v2_s_vlpan_bn_2e-3_100e_4x8gpus_obj365v1_goldg_train_lvis_minival.py` | `yolo_world_v2_s` |
| `v2m` | `yolo_world_v2_m_obj365v1_goldg_pretrain-c6237d5b.pth` | `configs/pretrain/yolo_world_v2_m_vlpan_bn_2e-3_100e_4x8gpus_obj365v1_goldg_train_lvis_minival.py` | `yolo_world_v2_m` |
| `v2l` | `yolo_world_v2_l_obj365v1_goldg_pretrain-a82b1fe3.pth` | `configs/pretrain/yolo_world_v2_l_vlpan_bn_2e-3_100e_4x8gpus_obj365v1_goldg_train_lvis_minival.py` | `yolo_world_v2_l` |

```text
models/source/yolo_world/v2s/yolo_world_v2_s_obj365v1_goldg_pretrain-55b943ea.pth
models/source/yolo_world/v2m/yolo_world_v2_m_obj365v1_goldg_pretrain-c6237d5b.pth
models/source/yolo_world/v2l/yolo_world_v2_l_obj365v1_goldg_pretrain-a82b1fe3.pth
```

Rockchip's exporter also instantiates the CLIP text encoder in order to
reparameterize the supplied COCO text prompts. `model assets` places the
complete official `openai/clip-vit-base-patch32` snapshot at:

```text
models/source/clip-vit-base-patch32/
  config.json
  pytorch_model.bin
  tokenizer_config.json
  tokenizer.json
  merges.txt
  vocab.json
  special_tokens_map.json
  preprocessor_config.json
```

The wrapper checks the model configuration and weights and forces Hugging Face
offline mode. Use `--clip-model <directory>` to choose another local snapshot.

## Export ONNX

```bash
scripts/omni model export \
  --variant v2s \
  --weights models/source/yolo_world/v2s/yolo_world_v2_s_obj365v1_goldg_pretrain-55b943ea.pth
```

For v2-M or v2-L, use its matching pinned Rockchip config through the same command:

```bash
scripts/omni model export \
  --variant v2m \
  --weights models/source/yolo_world/v2m/yolo_world_v2_m_obj365v1_goldg_pretrain-c6237d5b.pth
```

```bash
scripts/omni model export \
  --variant v2l \
  --weights models/source/yolo_world/v2l/yolo_world_v2_l_obj365v1_goldg_pretrain-a82b1fe3.pth
```

This invokes Rockchip's `deploy/export_onnx.py`, the matching 640 config, ONNX
opset 12 (required for this graph's `einsum` operator), and the checked-in
80 COCO prompt list. It validates ONNX with `onnx.checker`
and rejects any model not having exactly the native runtime interface:

- `images`: `[1,3,640,640]`;
- `texts`: `[1,80,512]`;
- six NCHW outputs: 80-class and 4-box tensors at 80x80, 40x40, and 20x20.

The default results are `artifacts/models/yolo_world_v2_s.onnx`,
`artifacts/models/yolo_world_v2_m.onnx`, and
`artifacts/models/yolo_world_v2_l.onnx`, respectively. `--variant` defaults to
`v2s` for backward compatibility.

## Compile RKNN

FP/non-quantized RKNN needs only the validated ONNX:

```bash
scripts/omni model compile \
  --variant v2s \
  --onnx artifacts/models/yolo_world_v2_s.onnx \
  --precision fp
```

For INT8, `model assets` creates the Rockchip YOLO-World calibration directory
and dataset format. The pinned Model Zoo conversion contract is commit
`c2b7d00714b4e5d21266ab3003f3ca687ba0d57b`; it references
`coco_text_outp.npy` but does not contain that file. Source only that missing
calibration embedding from Rockchip Model Zoo commit
`bad6c7334531becaf90a561988519b7bec34d0ab`
(`examples/yolo_world/model/coco_text_outp.npy`). The command keeps it local
and ignored, alongside the calibration image, preserving the dataset line
exactly and validating that the embedding is a NumPy array of shape
`[1,80,512]`:

```text
models/source/yolo_world/calibration/
  dataset.txt             # contains: bus.jpg coco_text_outp.npy
  bus.jpg
  coco_text_outp.npy      # Rockchip commit bad6c733...; [1,80,512] CLIP output
```

Then compile:

```bash
scripts/omni model compile \
  --variant v2s \
  --onnx artifacts/models/yolo_world_v2_s.onnx \
  --precision int8
```

This is the Rockchip converter configuration: `target_platform=rk3588`, RGB
normalization `mean=[0,0,0]`, `std=[255,255,255]`, and fixed named inputs
`images` and `texts`. The FP output is
`artifacts/models/yolo_world_v2_s_fp.rknn`; the INT8 output is
`artifacts/models/yolo_world_v2_s_i8.rknn`. With `--variant v2m`, the same
defaults are `artifacts/models/yolo_world_v2_m_fp.rknn` and
`artifacts/models/yolo_world_v2_m_i8.rknn`; with `--variant v2l`, they are
`artifacts/models/yolo_world_v2_l_fp.rknn` and
`artifacts/models/yolo_world_v2_l_i8.rknn`.

## Full build

After `model assets` and the builder image are in place, run:

```bash
scripts/omni model build \
  --variant v2s \
  --weights models/source/yolo_world/v2s/yolo_world_v2_s_obj365v1_goldg_pretrain-55b943ea.pth \
  --precision int8
```

```bash
scripts/omni model build \
  --variant v2m \
  --weights models/source/yolo_world/v2m/yolo_world_v2_m_obj365v1_goldg_pretrain-c6237d5b.pth \
  --precision int8
```

```bash
scripts/omni model build \
  --variant v2l \
  --weights models/source/yolo_world/v2l/yolo_world_v2_l_obj365v1_goldg_pretrain-a82b1fe3.pth \
  --precision int8
```

Use `--precision fp` for a non-quantized model. Existing generated artifacts
are deliberately not overwritten; choose a new `--output` / `--onnx-output` or
remove the exact generated artifact intentionally before rebuilding.

The commands fail before compilation for a missing checkpoint, local CLIP
snapshot, calibration asset, or incompatible ONNX contract, and fail after the
tools run if a generated artifact is missing or empty.
