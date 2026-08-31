#!/usr/bin/env python3
"""Minimal Toolkit2 reproduction with YOLO-World's real 80x80 class context.

This analysis-only tool extracts the activation immediately before
``cls_preds.0.2`` from a deterministic full-model representative input, then
builds only this classification slice:

  cls_preds.0.2 1x1 Conv -> Einsum -> Exp(logit_scale) -> Mul -> Add(bias)

Toolkit2 lowers the Einsum to the observed reshape/transpose/exMatMul/
reshape/transpose sequence.  No production model, runtime, or hybrid artifact
is modified.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import sys
from collections.abc import Iterable
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import onnx
import onnxruntime as ort
from onnx import TensorProto, checker, helper, numpy_helper
from rknn.api import RKNN

TOOLKIT_VERSION = "2.1.0+708089d1"
PROJECTION_INPUT = "/bbox_head/head_module/cls_preds.0/cls_preds.0.1/activate/Mul_output_0"
PROJECTION_OUTPUT = "/bbox_head/head_module/cls_preds.0/cls_preds.0.2/Conv_output_0"
EINSUM_OUTPUT = "/bbox_head/head_module/cls_contrasts.0/Einsum_output_0"
ADD_OUTPUT = "/bbox_head/head_module/cls_contrasts.0/Add_output_0"
PROJECTION_WEIGHT = "onnx::Conv_1856"
PROJECTION_BIAS = "onnx::Conv_1857"
LOGIT_SCALE = "baseModel.bbox_head.head_module.cls_contrasts.0.logit_scale"
CONTRAST_BIAS = "baseModel.bbox_head.head_module.cls_contrasts.0.bias"
PROJECTION_INPUT_SHAPE = (1, 256, 80, 80)
TEXT_SHAPE = (1, 80, 512)
OUTPUT_SHAPE = (1, 80, 80, 80)


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir", type=Path, required=True, help="Empty directory for all reproducible artifacts."
    )
    parser.add_argument("--onnx", type=Path, default=root / "artifacts/models/yolo_world_v2_l.onnx")
    parser.add_argument("--image", type=Path, default=root / "models/source/yolo_world/calibration/bus.jpg")
    parser.add_argument("--texts", type=Path, default=root / "artifacts/quant_analysis/v2l/texts_person_bus_padded.npy")
    return parser.parse_args()


def require_empty_directory(path: Path) -> None:
    if path.exists() and (not path.is_dir() or any(path.iterdir())):
        raise ValueError(f"output directory must be empty: {path}")


def require_success(ret: int, operation: str) -> None:
    if ret != 0:
        raise RuntimeError(f"{operation} failed with code {ret}")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def preprocess_image(path: Path) -> tuple[np.ndarray, dict[str, Any]]:
    source = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if source is None:
        raise ValueError(f"cannot read representative image: {path}")
    height, width = source.shape[:2]
    scale = min(640.0 / width, 640.0 / height)
    resized_width, resized_height = round(width * scale), round(height * scale)
    pad_x, pad_y = (640 - resized_width) // 2, (640 - resized_height) // 2
    letterboxed = np.full((640, 640, 3), 114, dtype=np.uint8)
    letterboxed[pad_y : pad_y + resized_height, pad_x : pad_x + resized_width] = cv2.resize(
        source, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR
    )
    rgb = cv2.cvtColor(letterboxed, cv2.COLOR_BGR2RGB)
    # The RKNN full-model experiment used this uint8 RGB image with std=255.
    # ONNX Runtime receives the equivalent normalized NCHW tensor.
    nchw = np.moveaxis(rgb, -1, 0)[np.newaxis, ...].astype(np.float32) / 255.0
    return nchw, {
        "source_shape": [int(height), int(width), 3],
        "letterbox": {"scale": scale, "resized": [resized_height, resized_width], "pad": [pad_y, pad_x], "value": 114},
    }


def dependency_slice(model: onnx.ModelProto, output_name: str) -> onnx.ModelProto:
    """Return the original topologically ordered node closure for one tensor."""
    producer = {output: node for node in model.graph.node for output in node.output}
    needed_nodes: set[str] = set()
    needed_inputs: set[str] = set()

    def visit(value: str) -> None:
        node = producer.get(value)
        if node is None:
            needed_inputs.add(value)
            return
        if node.name in needed_nodes:
            return
        needed_nodes.add(node.name)
        for node_input in node.input:
            if node_input:
                visit(node_input)

    visit(output_name)
    initializers = {initializer.name: initializer for initializer in model.graph.initializer}
    graph_inputs = {value.name: value for value in model.graph.input}
    nodes = [node for node in model.graph.node if node.name in needed_nodes]
    inputs = [graph_inputs[name] for name in needed_inputs if name in graph_inputs]
    initializers_needed = [initializers[name] for name in needed_inputs if name in initializers]
    if {value.name for value in inputs} != {"images", "texts"}:
        raise ValueError(
            f"unexpected full-model dependency inputs for projection activation: {[value.name for value in inputs]}"
        )
    graph = helper.make_graph(
        nodes,
        "cls80_projection_activation_extract",
        inputs,
        [helper.make_tensor_value_info(output_name, TensorProto.FLOAT, list(PROJECTION_INPUT_SHAPE))],
        initializer=initializers_needed,
    )
    sliced = helper.make_model(
        graph, producer_name="omniseer-cls80-projection-context", opset_imports=model.opset_import
    )
    checker.check_model(sliced)
    return sliced


def extract_representative_activation(
    full_onnx: Path, image: Path, texts: np.ndarray, output_dir: Path
) -> tuple[np.ndarray, dict[str, Any]]:
    full_model = onnx.load(full_onnx)
    sliced = dependency_slice(full_model, PROJECTION_INPUT)
    slice_path = output_dir / "full_model_to_projection_input.onnx"
    onnx.save(sliced, slice_path)
    input_image, preprocessing = preprocess_image(image)
    np.save(output_dir / "source_image_nchw_normalized.npy", input_image)
    session = ort.InferenceSession(str(slice_path), providers=["CPUExecutionProvider"])
    output = session.run([PROJECTION_INPUT], {"images": input_image, "texts": texts})[0]
    if tuple(output.shape) != PROJECTION_INPUT_SHAPE:
        raise RuntimeError(f"unexpected real projection input shape: {output.shape}")
    output = np.asarray(output, dtype=np.float32)
    np.save(output_dir / "projection_input.npy", output)
    return output, preprocessing


def initializer(model: onnx.ModelProto, name: str) -> onnx.TensorProto:
    for value in model.graph.initializer:
        if value.name == name:
            return value
    raise ValueError(f"full model lacks required initializer: {name}")


def write_context_onnx(full_onnx: Path, output_path: Path) -> dict[str, Any]:
    full = onnx.load(full_onnx)
    projection_input = helper.make_tensor_value_info(
        "projection_input", TensorProto.FLOAT, list(PROJECTION_INPUT_SHAPE)
    )
    texts = helper.make_tensor_value_info("texts", TensorProto.FLOAT, list(TEXT_SHAPE))
    scores = helper.make_tensor_value_info("scores", TensorProto.FLOAT, list(OUTPUT_SHAPE))
    nodes = [
        helper.make_node(
            "Conv",
            ["projection_input", PROJECTION_WEIGHT, PROJECTION_BIAS],
            [PROJECTION_OUTPUT],
            name="/bbox_head/head_module/cls_preds.0/cls_preds.0.2/Conv",
            dilations=[1, 1],
            group=1,
            kernel_shape=[1, 1],
            pads=[0, 0, 0, 0],
            strides=[1, 1],
        ),
        helper.make_node(
            "Einsum",
            [PROJECTION_OUTPUT, "texts"],
            [EINSUM_OUTPUT],
            name="/bbox_head/head_module/cls_contrasts.0/Einsum",
            equation="bchw,bkc->bkhw",
        ),
        helper.make_node(
            "Exp",
            [LOGIT_SCALE],
            ["/bbox_head/head_module/cls_contrasts.0/Exp_output_0"],
            name="/bbox_head/head_module/cls_contrasts.0/Exp",
        ),
        helper.make_node(
            "Mul",
            [EINSUM_OUTPUT, "/bbox_head/head_module/cls_contrasts.0/Exp_output_0"],
            ["/bbox_head/head_module/cls_contrasts.0/Mul_output_0"],
            name="/bbox_head/head_module/cls_contrasts.0/Mul",
        ),
        helper.make_node(
            "Add",
            ["/bbox_head/head_module/cls_contrasts.0/Mul_output_0", CONTRAST_BIAS],
            ["scores"],
            name="/bbox_head/head_module/cls_contrasts.0/Add",
        ),
    ]
    graph = helper.make_graph(
        nodes,
        "cls80_projection_context",
        [projection_input, texts],
        [scores],
        initializer=[
            initializer(full, name) for name in (PROJECTION_WEIGHT, PROJECTION_BIAS, LOGIT_SCALE, CONTRAST_BIAS)
        ],
    )
    model = helper.make_model(
        graph, producer_name="omniseer-cls80-projection-context", opset_imports=[helper.make_opsetid("", 12)]
    )
    checker.check_model(model)
    onnx.save(model, output_path)
    return {
        name: {
            "shape": list(numpy_helper.to_array(initializer(full, name)).shape),
            "sha256": hashlib.sha256(initializer(full, name).SerializeToString()).hexdigest(),
        }
        for name in (PROJECTION_WEIGHT, PROJECTION_BIAS, LOGIT_SCALE, CONTRAST_BIAS)
    }


def save_inputs_and_reference(
    onnx_path: Path, full_onnx: Path, image: Path, texts_path: Path, output_dir: Path
) -> tuple[Path, Path, np.ndarray, dict[str, Any]]:
    texts = np.load(texts_path, allow_pickle=False).astype(np.float32, copy=False)
    if tuple(texts.shape) != TEXT_SHAPE:
        raise ValueError(f"unexpected real texts shape: {texts.shape}")
    projection_input, preprocessing = extract_representative_activation(full_onnx, image, texts, output_dir)
    saved_texts = output_dir / "texts.npy"
    np.save(saved_texts, texts)
    reference = (
        ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
        .run(["scores"], {"projection_input": projection_input, "texts": texts})[0]
        .astype(np.float32)
    )
    np.save(output_dir / "reference.npy", reference)
    (output_dir / "dataset.txt").write_text(f"{output_dir / 'projection_input.npy'} {saved_texts}\n", encoding="utf-8")
    return output_dir / "projection_input.npy", saved_texts, reference, preprocessing


def configure_and_load(onnx_path: Path) -> RKNN:
    rknn = RKNN(verbose=False)
    require_success(rknn.config(target_platform="rk3588"), "RKNN config")
    require_success(
        rknn.load_onnx(
            model=str(onnx_path),
            inputs=["projection_input", "texts"],
            input_size_list=[list(PROJECTION_INPUT_SHAPE), list(TEXT_SHAPE)],
        ),
        "RKNN load_onnx",
    )
    return rknn


def run_simulator(rknn: RKNN, projection_path: Path, texts_path: Path) -> np.ndarray:
    require_success(rknn.init_runtime(target=None), "RKNN host simulator initialization")
    outputs = rknn.inference(
        inputs=[np.load(projection_path, allow_pickle=False), np.load(texts_path, allow_pickle=False)],
        data_format=["nchw", "nchw"],
    )
    if len(outputs) != 1 or tuple(outputs[0].shape) != OUTPUT_SHAPE:
        raise RuntimeError(f"unexpected simulator output: {[tuple(output.shape) for output in outputs]}")
    return np.asarray(outputs[0], dtype=np.float32)


def build_full_fp(onnx_path: Path, artifact_path: Path, projection_path: Path, texts_path: Path) -> np.ndarray:
    rknn = configure_and_load(onnx_path)
    try:
        require_success(rknn.build(do_quantization=False), "full-FP RKNN build")
        output = run_simulator(rknn, projection_path, texts_path)
        require_success(rknn.export_rknn(str(artifact_path)), "full-FP RKNN export")
        return output
    finally:
        rknn.release()


def custom_layers(config_path: Path) -> set[str]:
    return set(
        re.findall(
            r"^    (\S+): float16$",
            config_path.read_text(encoding="utf-8").split("quantize_parameters:\n", 1)[0],
            re.MULTILINE,
        )
    )


def add_custom_layers(config_path: Path, layers: Iterable[str]) -> None:
    contents = config_path.read_text(encoding="utf-8")
    custom, parameters = contents.split("quantize_parameters:\n", 1)
    if custom.rstrip().endswith("{}"):
        custom = custom.rstrip()[:-2] + "\n"
    existing = custom_layers(config_path)
    custom += "".join(f"    {layer}: float16\n" for layer in sorted(set(layers) - existing))
    config_path.write_text(custom + "quantize_parameters:\n" + parameters, encoding="utf-8")


def lowered_names(config_path: Path) -> dict[str, str]:
    contents = config_path.read_text(encoding="utf-8")
    names = {
        "projection": PROJECTION_OUTPUT,
        "projection_reshape": f"{PROJECTION_OUTPUT}_rs",
        "matmul": f"{PROJECTION_OUTPUT}_rs_mm",
        "post_matmul_reshape": f"{PROJECTION_OUTPUT}_rs_mm_rs",
        "text_operand": "texts_tp-rs",
    }
    missing = [value for value in names.values() if f"    {value}:" not in contents]
    if missing:
        raise ValueError(f"Toolkit2 step-1 config lacks expected lowered tensors: {', '.join(missing)}")
    return names


def build_hybrid(
    name: str,
    onnx_path: Path,
    output_dir: Path,
    artifact_path: Path,
    fp16_keys: set[str],
    projection_path: Path,
    texts_path: Path,
) -> tuple[np.ndarray, dict[str, str]]:
    workdir = output_dir / f"{name}_config"
    workdir.mkdir()
    rknn = configure_and_load(onnx_path)
    try:
        cwd = Path.cwd()
        try:
            os.chdir(workdir)
            require_success(
                rknn.hybrid_quantization_step1(dataset=str(output_dir / "dataset.txt")), f"{name} hybrid step 1"
            )
        finally:
            os.chdir(cwd)
        config_path = workdir / f"{onnx_path.stem}.quantization.cfg"
        names = lowered_names(config_path)
        add_custom_layers(config_path, {names[key] for key in fp16_keys})
        require_success(
            rknn.hybrid_quantization_step2(
                str(workdir / f"{onnx_path.stem}.model"), str(workdir / f"{onnx_path.stem}.data"), str(config_path)
            ),
            f"{name} hybrid step 2",
        )
        output = run_simulator(rknn, projection_path, texts_path)
        require_success(rknn.export_rknn(str(artifact_path)), f"{name} RKNN export")
        return output, names
    finally:
        rknn.release()


def stats(values: np.ndarray) -> dict[str, Any]:
    data = np.asarray(values, dtype=np.float64)
    return {
        "min": float(data.min()),
        "max": float(data.max()),
        "mean": float(data.mean()),
        "stddev": float(data.std()),
        "near_constant": bool(float(data.std()) < 1e-7 or float(data.max() - data.min()) < 1e-6),
    }


def compare(candidate: np.ndarray, reference: np.ndarray) -> dict[str, Any]:
    error = np.abs(np.asarray(candidate, dtype=np.float64) - np.asarray(reference, dtype=np.float64))
    return {
        "max_absolute_error": float(error.max()),
        "mean_absolute_error": float(error.mean()),
        "output": stats(candidate),
    }


def topology(artifact_path: Path, names: dict[str, str]) -> dict[str, Any]:
    raw = artifact_path.read_bytes()
    text = raw.decode("latin-1", errors="ignore")
    candidates = [
        f"Conv:{names['projection']}",
        f"Reshape:{names['projection_reshape']}",
        f"exMatMul:{names['matmul']}",
        f"Reshape:{names['post_matmul_reshape']}",
        f"Transpose:{names['post_matmul_reshape']}_tp",
        f"{names['projection_reshape']}__cvt_float16_int8",
        f"{names['text_operand']}__cvt_int8_float16",
        f"{names['post_matmul_reshape']}__cvt_float16_int8",
    ]
    return {
        "sha256": sha256(artifact_path),
        "bytes": artifact_path.stat().st_size,
        "toolkit_version_marker": TOOLKIT_VERSION in text,
        "markers": {candidate: candidate in text for candidate in candidates},
    }


def artifact_record(path: Path, root: Path) -> dict[str, Any]:
    return {"path": str(path.relative_to(root)), "bytes": path.stat().st_size, "sha256": sha256(path)}


def main() -> int:
    args = parse_args()
    try:
        args.output_dir, args.onnx, args.image, args.texts = (
            path.resolve() for path in (args.output_dir, args.onnx, args.image, args.texts)
        )
        require_empty_directory(args.output_dir)
        for path in (args.onnx, args.image, args.texts):
            if not path.is_file() or not path.stat().st_size:
                raise ValueError(f"required input is missing or empty: {path}")
        args.output_dir.mkdir(parents=True)
        context_onnx = args.output_dir / "cls80_projection_context.onnx"
        constants = write_context_onnx(args.onnx, context_onnx)
        projection_path, texts_path, reference, preprocessing = save_inputs_and_reference(
            context_onnx, args.onnx, args.image, args.texts, args.output_dir
        )
        artifacts = {
            name: args.output_dir / f"cls80_projection_context_{name}.rknn"
            for name in ("full_fp16", "healthy_int8_matmul", "failed_fp16_island")
        }
        outputs: dict[str, np.ndarray] = {
            "full_fp16": build_full_fp(context_onnx, artifacts["full_fp16"], projection_path, texts_path)
        }
        # Match the full-model progression: projection + its lowered reshape
        # stay FP16, but exMatMul is INT8 in the healthy variant.
        outputs["healthy_int8_matmul"], names = build_hybrid(
            "healthy_int8_matmul",
            context_onnx,
            args.output_dir,
            artifacts["healthy_int8_matmul"],
            {"projection", "projection_reshape"},
            projection_path,
            texts_path,
        )
        # This is the full-model failed island: projection through the
        # post-MatMul reshape are FP16. Toolkit2 decides the exact text/input
        # and output conversion placement during step 2.
        outputs["failed_fp16_island"], _ = build_hybrid(
            "failed_fp16_island",
            context_onnx,
            args.output_dir,
            artifacts["failed_fp16_island"],
            {"projection", "projection_reshape", "matmul", "post_matmul_reshape"},
            projection_path,
            texts_path,
        )
        for name, output in outputs.items():
            np.save(args.output_dir / f"{name}_output.npy", output)
        manifest_paths = [
            context_onnx,
            args.output_dir / "full_model_to_projection_input.onnx",
            projection_path,
            texts_path,
            args.output_dir / "source_image_nchw_normalized.npy",
            args.output_dir / "reference.npy",
            args.output_dir / "dataset.txt",
        ]
        for name, artifact in artifacts.items():
            manifest_paths.extend([artifact, args.output_dir / f"{name}_output.npy"])
            if name != "full_fp16":
                manifest_paths.extend(
                    args.output_dir / f"{name}_config" / f"{context_onnx.stem}{suffix}"
                    for suffix in (".model", ".data", ".quantization.cfg")
                )
        report = {
            "toolkit_expected": TOOLKIT_VERSION,
            "source": {
                "full_onnx": str(args.onnx),
                "full_onnx_sha256": sha256(args.onnx),
                "image": str(args.image),
                "image_sha256": sha256(args.image),
                "texts": str(args.texts),
                "texts_sha256": sha256(args.texts),
                "preprocessing": preprocessing,
            },
            "architecture": {
                "added_relative_to_exmatmul_micrograph": (
                    "real cls_preds.0.2 1x1 Conv (exported weight+bias) upstream; "
                    "real cls_contrasts.0 Exp(logit_scale), Mul, Add(bias) downstream"
                ),
                "projection_input": list(PROJECTION_INPUT_SHAPE),
                "projection_output": list((1, 512, 80, 80)),
                "texts": list(TEXT_SHAPE),
                "output": list(OUTPUT_SHAPE),
                "einsum": "bchw,bkc->bkhw",
                "lowering": "reshape/transpose/exMatMul/reshape/transpose",
                "constants": constants,
            },
            "reference": stats(reference),
            "requested_precision_layout": {
                "full_fp16": "non-quantized float16",
                "healthy_int8_matmul": "projection output and lowered reshape FP16; exMatMul remains INT8",
                "failed_fp16_island": "projection output, lowered reshape, exMatMul, and post-MatMul reshape FP16",
            },
            "variants": {
                name: {
                    "comparison_to_onnx_float32": compare(output, reference),
                    "comparison_to_full_fp16": compare(output, outputs["full_fp16"]),
                    "artifact": topology(artifacts[name], names),
                }
                for name, output in outputs.items()
            },
            "artifact_manifest": {
                record["path"]: record for record in (artifact_record(path, args.output_dir) for path in manifest_paths)
            },
        }
        (args.output_dir / "report.json").write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0
    except (OSError, RuntimeError, ValueError, onnx.checker.ValidationError) as exc:
        print(f"cls80 projection-context reproduction failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
