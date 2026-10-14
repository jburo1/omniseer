# YOLO11 model building

This isolated host workflow builds YOLO11 detection artifacts. Existing
YOLO-World commands and dependencies are unchanged. It does not change the
robot runtime, ROS integration, or training pipeline.

## Export and FP compilation

```bash
scripts/omni model yolo11 assets
scripts/omni model yolo11 image
scripts/omni model yolo11 export --weights models/source/yolo11/yolo11n.pt
scripts/omni model yolo11 compile --onnx artifacts/models/yolo11/yolo11n.onnx --precision fp
```

The single linux/amd64 builder pins Python 3.10.16, Rockchip's
[YOLO11 exporter](https://github.com/airockchip/ultralytics_yolo11/blob/0692e9297670acf4cc6d0cec773d7a9493cb8a5f/RKOPT_README.md)
at `0692e9297670acf4cc6d0cec773d7a9493cb8a5f` (Ultralytics **8.3.9**, based on
upstream `50497218c25682458ea35b02dcc5d8a364f34591`), PyTorch 2.2.0 CPU,
ONNX 1.14.1, NumPy 1.26.4, and RKNN Toolkit2 **2.3.0**.
The fork supplies Ultralytics; installing upstream Ultralytics would replace
the Rockchip export changes. These pinned versions satisfy the fork's requirements.
The CPython 3.10 wheel requires Torch 1.10.1–2.4.0, ONNX >=1.10.0,
protobuf 4.21.6–4.25.4 and NumPy <=1.26.4. PyPI release-level metadata
instead lists older requirements (including protobuf 3.20.3); using that
protobuf pin fails dependency resolution. The image pins the compatible wheel by SHA-256, locks resolved Python dependencies,
and runs `pip check` and imports both tools. Export plus FP compilation
is the compatibility check; no separate exporter/compiler images are needed.

`assets` downloads the official Ultralytics `v8.3.0` release checkpoint into
`models/source/yolo11/yolo11n.pt`, validating SHA-256
`0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1`.
Its JSON sidecar records the release URL and source revision
`0f20d143ac8f9e01322f85dcb5c10dc9b7ca1153`. The release URL is pinned by
content hash because GitHub release assets can be replaced. Repeated asset
requests validate existing content and provenance.

`export` accepts local trusted `.pt` YOLO11 Detect checkpoints: either official
ordered COCO-80 classes or the ordered fine-tuned vocabulary **person, book,
power drill** (`power_drill` is also accepted). Other tasks, architectures,
class counts, or orders fail. Fine-tuned checkpoints retain their own class
metadata; this workflow does not relabel pretrained COCO classes.

Rockchip's `format="rknn"` exports **ONNX**, using opset 12 and fixed batch 1,
640x640 CPU FP32 input. It removes DFL decoding and adds score sums. The ONNX
validator checks the graph, infers shapes, and requires exactly:

- `images`: float32 `[1,3,640,640]`, RGB normalized to [0,1];
- nine ordered float32 NCHW outputs, three per grid (80, 40, 20):
  box distributions `[1,64,H,W]` (`reg_max=16`), class probabilities
  `[1,80|3,H,W]`, and score sums `[1,1,H,W]`.

ONNX Runtime also exercises a deterministic normalized RGB input, checks
finite outputs, class probability ranges, and agreement with score sums.
It does not fabricate static output shapes. YOLO-World's six-output contract
and text embeddings are inapplicable. CPU decoding, thresholding and NMS would
be required by a future YOLO11 runtime consumer.

`compile` targets RK3588, with mean 0 and standard deviation 255 for each
channel. `fp` disables quantization; Toolkit2 chooses its non-quantized internal
representation (typically FP16). Outputs default to
`artifacts/models/yolo11/<checkpoint-stem>.onnx` and
`<onnx-stem>_fp.rknn` or `<onnx-stem>_int8.rknn`.
`--output` selects a new path beneath that directory. Existing artifacts,
sidecars, and reservations are refused. Choose a new output name for a new
experiment. Failed operations clean their staging directory; interrupted
processes can leave a `.lock` requiring inspection before removal.

Each generated artifact has a `.json` sidecar with SHA-256, input hash and
provenance, class/tensor contracts, tool source hashes, exporter revision,
package versions, actual Docker image ID, and compiler/export options.
The source checkpoint is never modified. Source models and generated artifacts
remain ignored by Git.

## INT8 calibration without held-out leakage

Use a separate representative set of robot-camera images drawn from training
captures or a dedicated calibration acquisition. Include person, book and
power drill, background scenes, scale, lighting and viewing-angle variation.
Do not use validation or test captures for calibration or tuning. No sample
dataset is downloaded automatically, and YOLO-World calibration is not reused.

Create a manifest under a common dataset root (all paths must resolve beneath
its directory):

```json
{
  "split": "calibration",
  "source": "dedicated robot-camera calibration capture; acquisition/split revision",
  "representativeness": "objects, background, distances, lighting and viewpoints sampled",
  "images": ["calibration/frame001.jpg", "calibration/frame002.jpg"],
  "excluded_splits": {
    "validation": ["validation/frame001.jpg"],
    "test": ["test/frame001.jpg"]
  }
}
```

List **all** validation and test images; both lists must be non-empty. The
builder rejects duplicate calibration content and any SHA-256 intersection
with those held-out lists, including renamed copies. Dataset ownership must
ensure the lists are complete and exclude related captures from the same
held-out sequences; hashes cannot establish semantic independence or scene
representativeness.

```bash
scripts/omni model yolo11 compile \
  --onnx artifacts/models/yolo11/yolo11n.onnx --precision int8 \
  --calibration-manifest models/source/yolo11/calibration/manifest.json
```

The builder validates the manifest before invoking Toolkit, makes temporary
centered 640x640 letterboxed images (bilinear resize, padding 114), then invokes
Toolkit's image calibration loader with normalization by 255. The sidecar
records the manifest hash, original and prepared image hashes, source,
representativeness description, held-out image hashes and preprocessing.
Prepared images and Toolkit's dataset list are temporary derived artifacts.

## Verification boundary

Host ONNX validation and RKNN compilation do not establish ROCK 5B+ execution,
NPU accuracy, latency, or physical-robot compatibility. Toolkit2 2.3.0 matches
the requested recorded runtime version, but the installed driver/library and
nine-output YOLO11 consumer still require board verification. No runtime
promotion or hardware operation is part of these commands.

## Host verification record (2026-10-13)

The final image `omniseer/model-builder:yolo11-rknn-toolkit2-2.3.0` is
`sha256:9e22b437d749b3201914890801f24740261a52b5ac667663c05aff409b256f2b`.
Its pinned exporter, Ultralytics 8.3.9 and Toolkit2 2.3.0 successfully exported
and compiled the official COCO-80 checkpoint on an x86_64 host.

```bash
scripts/omni model yolo11 image
scripts/omni model yolo11 assets
scripts/omni model yolo11 export --weights models/source/yolo11/yolo11n.pt --output artifacts/models/yolo11/verified/yolo11n.onnx
scripts/omni model yolo11 compile --onnx artifacts/models/yolo11/verified/yolo11n.onnx --precision fp --output artifacts/models/yolo11/verified/yolo11n_fp.rknn
```

| Artifact | SHA-256 |
| --- | --- |
| `models/source/yolo11/yolo11n.pt` | `0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1` |
| `artifacts/models/yolo11/verified/yolo11n.onnx` | `1ee1e506dcd7c688c51f1b8cb6b2f06a935c3a306ba23776680d8802a7266a38` |
| `artifacts/models/yolo11/verified/yolo11n_fp.rknn` | `651bb676a6d08a93be0cac23b64f63728f65fb6a672a43db95e86322e4328da5` |
| `artifacts/models/yolo11/yolo11n.onnx` | `1ee1e506dcd7c688c51f1b8cb6b2f06a935c3a306ba23776680d8802a7266a38` |
| `artifacts/models/yolo11/yolo11n_fp.rknn` | `b8e63006b84d1349d1aaa62004f11bbbd7a918f049abb1e35ac42a9c2e5bfbca` |

The top-level ONNX/RKNN pair was generated during implementation and retained;
the `verified/` pair uses the final locked image and tool sources. The ONNX
hash reproduced across exports. RKNN hashes differ between these builds;
bit-for-bit RKNN reproducibility was not established. Each pair has complete
JSON provenance.

Focused tests (`tests/test_model_toolchain.py` and
`tests/test_yolo11_toolchain.py`) passed: **30 tests** in the pinned Python 3.10
builder, including ONNX 1.14.1 and calibration preprocessing tests. The portable
host suite also passed with Python 3.12 and ONNX 1.16.2. Ruff, Bash syntax,
`git diff --check`, and `scripts/omni docs build` passed (temporary documentation
tool environment, pinned D2 v0.7.1). Re-export to an existing destination was
refused as expected.

INT8 split/preprocessing guards were tested with synthetic images; no
representative calibration dataset or INT8 artifact was produced. Actual
fine-tuned three-class checkpoint export, board execution, NPU accuracy and
latency, and physical robot behavior remain unverified. Three-class ONNX
contracts and class ordering were tested portably.
