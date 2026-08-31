#!/usr/bin/env python3
"""Minimal RKNN Toolkit2 reproduction for YOLO-World's 80x80 class Einsum.

This is deliberately an analysis-only tool.  It creates a fixed-shape ONNX
micrograph, compiles it with the pinned Toolkit2 image, and compares the host
simulator outputs.  It does not affect the robot runtime or production model
build entrypoints.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from pathlib import Path
from typing import Any

import numpy as np
import onnx
from onnx import TensorProto, checker, helper
from rknn.api import RKNN

VISUAL_SHAPE = (1, 512, 80, 80)
TEXT_SHAPE = (1, 80, 512)
OUTPUT_SHAPE = (1, 80, 80, 80)
TOOLKIT_VERSION = "2.1.0+708089d1"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True, help="Empty directory for reproducible outputs.")
    return parser.parse_args()


def require_empty_directory(path: Path) -> None:
    if path.exists() and (not path.is_dir() or any(path.iterdir())):
        raise ValueError(f"output directory must be empty: {path}")


def deterministic_inputs() -> tuple[np.ndarray, np.ndarray]:
    """Produce non-constant values with stable, modest quantization ranges."""
    visual_index = np.arange(np.prod(VISUAL_SHAPE), dtype=np.float32).reshape(VISUAL_SHAPE)
    text_index = np.arange(np.prod(TEXT_SHAPE), dtype=np.float32).reshape(TEXT_SHAPE)
    visual = (0.125 * np.sin(visual_index * 0.00071) + 0.075 * np.cos(visual_index * 0.00193)).astype(np.float32)
    text = (0.14 * np.cos(text_index * 0.00311) - 0.06 * np.sin(text_index * 0.00127)).astype(np.float32)
    return visual, text


def write_onnx(path: Path) -> None:
    """Write bchw,bkc->bkhw; Toolkit2 performs the observed lowering itself."""
    visual = helper.make_tensor_value_info("visual", TensorProto.FLOAT, VISUAL_SHAPE)
    texts = helper.make_tensor_value_info("texts", TensorProto.FLOAT, TEXT_SHAPE)
    scores = helper.make_tensor_value_info("scores", TensorProto.FLOAT, OUTPUT_SHAPE)
    einsum = helper.make_node(
        "Einsum",
        ["visual", "texts"],
        ["scores"],
        name="cls80_einsum",
        equation="bchw,bkc->bkhw",
    )
    graph = helper.make_graph([einsum], "cls80_exmatmul_repro", [visual, texts], [scores])
    model = helper.make_model(
        graph,
        producer_name="omniseer-cls80-exmatmul-repro",
        opset_imports=[helper.make_opsetid("", 12)],
    )
    checker.check_model(model)
    onnx.save(model, path)


def save_inputs(output_dir: Path) -> tuple[Path, Path, np.ndarray]:
    visual, texts = deterministic_inputs()
    visual_path = output_dir / "visual.npy"
    text_path = output_dir / "texts.npy"
    np.save(visual_path, visual)
    np.save(text_path, texts)
    reference = np.einsum("bchw,bkc->bkhw", visual, texts, optimize=True)
    np.save(output_dir / "reference.npy", reference)
    (output_dir / "dataset.txt").write_text(f"{visual_path} {text_path}\n", encoding="utf-8")
    return visual_path, text_path, reference


def require_success(ret: int, operation: str) -> None:
    if ret != 0:
        raise RuntimeError(f"{operation} failed with code {ret}")


def configure_and_load(onnx_path: Path) -> RKNN:
    rknn = RKNN(verbose=False)
    require_success(rknn.config(target_platform="rk3588"), "RKNN config")
    require_success(
        rknn.load_onnx(
            model=str(onnx_path),
            inputs=["visual", "texts"],
            input_size_list=[list(VISUAL_SHAPE), list(TEXT_SHAPE)],
        ),
        "RKNN load_onnx",
    )
    return rknn


def run_simulator(rknn: RKNN, visual_path: Path, text_path: Path) -> np.ndarray:
    require_success(rknn.init_runtime(target=None), "RKNN host simulator initialization")
    visual = np.load(visual_path, allow_pickle=False)
    texts = np.load(text_path, allow_pickle=False)
    outputs = rknn.inference(inputs=[visual, texts], data_format=["nchw", "nchw"])
    if len(outputs) != 1 or tuple(outputs[0].shape) != OUTPUT_SHAPE:
        raise RuntimeError(f"unexpected simulator output: {[tuple(output.shape) for output in outputs]}")
    return np.asarray(outputs[0], dtype=np.float32)


def build_full_fp(onnx_path: Path, artifact_path: Path, visual_path: Path, text_path: Path) -> np.ndarray:
    rknn = configure_and_load(onnx_path)
    try:
        require_success(rknn.build(do_quantization=False), "full-FP RKNN build")
        output = run_simulator(rknn, visual_path, text_path)
        require_success(rknn.export_rknn(str(artifact_path)), "full-FP RKNN export")
        return output
    finally:
        rknn.release()


def read_custom_layers(config_path: Path) -> set[str]:
    contents = config_path.read_text(encoding="utf-8")
    before_parameters = contents.split("quantize_parameters:\n", 1)[0]
    return set(re.findall(r"^    (\S+): float16$", before_parameters, flags=re.MULTILINE))


def add_custom_layers(config_path: Path, layers: set[str]) -> None:
    contents = config_path.read_text(encoding="utf-8")
    custom, parameters = contents.split("quantize_parameters:\n", 1)
    existing = read_custom_layers(config_path)
    # Toolkit2 emits an inline empty map when step 1 has no custom layers.
    # Expand it before adding indented entries so the YAML remains valid.
    if custom.rstrip().endswith("{}"):
        custom = custom.rstrip()[:-2] + "\n"
    custom += "".join(f"    {layer}: float16\n" for layer in sorted(layers - existing))
    config_path.write_text(custom + "quantize_parameters:\n" + parameters, encoding="utf-8")


def generated_names(config_path: Path) -> dict[str, str]:
    """Resolve Toolkit2's generated names and fail instead of guessing."""
    contents = config_path.read_text(encoding="utf-8")
    required = {
        "visual_operand": "visual_rs",
        "text_operand": "texts_tp-rs",
        "matmul": "visual_rs_mm",
        "post_matmul_reshape": "visual_rs_mm_rs",
    }
    missing = [name for name in required.values() if f"    {name}:" not in contents]
    if missing:
        raise ValueError(f"Toolkit2 step-1 config lacks expected lowered tensors: {', '.join(missing)}")
    return required


def build_hybrid(
    name: str,
    onnx_path: Path,
    dataset_path: Path,
    output_dir: Path,
    artifact_path: Path,
    fp16_layers: set[str],
    visual_path: Path,
    text_path: Path,
) -> np.ndarray:
    workdir = output_dir / f"{name}_config"
    workdir.mkdir()
    rknn = configure_and_load(onnx_path)
    try:
        original_cwd = Path.cwd()
        try:
            # Toolkit2 writes its three reproducible step-1 records to cwd.
            import os

            os.chdir(workdir)
            require_success(rknn.hybrid_quantization_step1(dataset=str(dataset_path)), f"{name} hybrid step 1")
        finally:
            os.chdir(original_cwd)
        config_path = workdir / f"{onnx_path.stem}.quantization.cfg"
        names = generated_names(config_path)
        add_custom_layers(config_path, {names[layer] for layer in fp16_layers})
        require_success(
            rknn.hybrid_quantization_step2(
                str(workdir / f"{onnx_path.stem}.model"),
                str(workdir / f"{onnx_path.stem}.data"),
                str(config_path),
            ),
            f"{name} hybrid step 2",
        )
        output = run_simulator(rknn, visual_path, text_path)
        require_success(rknn.export_rknn(str(artifact_path)), f"{name} RKNN export")
        return output
    finally:
        rknn.release()


def stats(values: np.ndarray) -> dict[str, Any]:
    values = np.asarray(values, dtype=np.float64)
    return {
        "min": float(values.min()),
        "max": float(values.max()),
        "mean": float(values.mean()),
        "stddev": float(values.std()),
        "unique_sample_count": int(np.unique(values.reshape(-1)[:100_000]).size),
        "near_constant": bool(float(values.std()) < 1e-7 or float(values.max() - values.min()) < 1e-6),
    }


def comparison(candidate: np.ndarray, reference: np.ndarray) -> dict[str, Any]:
    error = np.abs(np.asarray(candidate, dtype=np.float64) - np.asarray(reference, dtype=np.float64))
    return {
        "max_absolute_error": float(error.max()),
        "mean_absolute_error": float(error.mean()),
        "output": stats(candidate),
    }


def rknn_topology(artifact_path: Path) -> dict[str, Any]:
    """Extract the narrow conversion/topology evidence available in an RKNN artifact."""
    raw = artifact_path.read_bytes()
    text = raw.decode("latin-1", errors="ignore")
    needles = (
        "exMatMul:visual_rs_mm",
        "Reshape:visual_rs",
        "Reshape:visual_rs_mm_rs",
        "Transpose:visual_rs_mm_rs_tp",
        "texts_rs__cvt_int8_float16",
        "visual_rs__cvt_float16_int8",
        "visual_rs_mm_rs__cvt_float16_int8",
    )
    return {
        "sha256": hashlib.sha256(raw).hexdigest(),
        "bytes": len(raw),
        "markers": {needle: needle in text for needle in needles},
        "toolkit_version_marker": TOOLKIT_VERSION in text,
    }


def main() -> int:
    args = parse_args()
    try:
        args.output_dir = args.output_dir.resolve()
        require_empty_directory(args.output_dir)
        args.output_dir.mkdir(parents=True)
        onnx_path = args.output_dir / "cls80_exmatmul.onnx"
        write_onnx(onnx_path)
        visual_path, text_path, reference = save_inputs(args.output_dir)

        outputs: dict[str, np.ndarray] = {}
        artifacts = {
            "full_fp16": args.output_dir / "cls80_exmatmul_full_fp16.rknn",
            "healthy_int8_matmul": args.output_dir / "cls80_exmatmul_healthy_int8_matmul.rknn",
            "failed_fp16_island": args.output_dir / "cls80_exmatmul_failed_fp16_island.rknn",
        }
        outputs["full_fp16"] = build_full_fp(onnx_path, artifacts["full_fp16"], visual_path, text_path)
        outputs["healthy_int8_matmul"] = build_hybrid(
            "healthy_int8_matmul",
            onnx_path,
            args.output_dir / "dataset.txt",
            args.output_dir,
            artifacts["healthy_int8_matmul"],
            {"visual_operand"},
            visual_path,
            text_path,
        )
        outputs["failed_fp16_island"] = build_hybrid(
            "failed_fp16_island",
            onnx_path,
            args.output_dir / "dataset.txt",
            args.output_dir,
            artifacts["failed_fp16_island"],
            {"visual_operand", "text_operand", "matmul", "post_matmul_reshape"},
            visual_path,
            text_path,
        )

        for name, values in outputs.items():
            np.save(args.output_dir / f"{name}_output.npy", values)
        report = {
            "toolkit_expected": TOOLKIT_VERSION,
            "architecture": {
                "onnx": "Einsum bchw,bkc->bkhw; Toolkit2 lowers it to reshape/transpose/exMatMul/reshape/transpose.",
                "visual_input": list(VISUAL_SHAPE),
                "text_input": list(TEXT_SHAPE),
                "output": list(OUTPUT_SHAPE),
                "expected_lowered_shapes": {
                    "visual_operand": [1, 512, 1, 6400],
                    "text_operand": [1, 512, 1, 80],
                    "matmul_result": [1, 6400, 1, 80],
                    "post_matmul_reshape": [80, 80, 1, 80],
                },
                "permutations": {"text": [0, 3, 2, 1], "output": [2, 3, 0, 1]},
            },
            "reference": stats(reference),
            "variants": {
                name: {
                    "comparison_to_numpy_float32": comparison(values, reference),
                    "comparison_to_full_fp16": comparison(values, outputs["full_fp16"]),
                    "artifact": rknn_topology(artifacts[name]),
                }
                for name, values in outputs.items()
            },
            "requested_precision_layout": {
                "full_fp16": "Toolkit2 non-quantized float16 model; no conversion around exMatMul.",
                "healthy_int8_matmul": "visual_rs FP16, then FP16->INT8 before exMatMul; text and result remain INT8.",
                "failed_fp16_island": (
                    "visual_rs, texts_tp-rs, exMatMul result, and post-MatMul reshape requested FP16; "
                    "Toolkit2 converts text INT8->FP16 before its transpose and post-MatMul reshape FP16->INT8."
                ),
            },
        }
        (args.output_dir / "report.json").write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"cls80 exMatMul reproduction failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
