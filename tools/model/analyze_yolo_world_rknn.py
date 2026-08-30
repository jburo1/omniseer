#!/usr/bin/env python3
"""Run RKNN Toolkit2 layer-wise INT8 accuracy analysis for YOLO-World v2."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
from generate_yolo_world_calibration_embedding import (
    CLASS_CAPACITY,
    EMBEDDING_WIDTH,
    generate_padded_embedding,
)
from rknn.api import RKNN

IMAGE_INPUT_SHAPE = [1, 3, 640, 640]
TEXT_INPUT_SHAPE = [1, CLASS_CAPACITY, EMBEDDING_WIDTH]
RUNTIME_CLASSES = ["person", "bus"]
RUNTIME_PAD_TOKEN = "nothing"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run RK3588 layer-wise RKNN INT8 accuracy analysis.")
    parser.add_argument("--variant", choices=("v2s", "v2m", "v2l"), required=True, help="YOLO-World v2 variant.")
    parser.add_argument("--onnx", type=Path, required=True, help="YOLO-World v2 ONNX model.")
    parser.add_argument(
        "--calibration-dir", type=Path, required=True, help="Existing INT8 representative calibration directory."
    )
    parser.add_argument("--clip-model", type=Path, required=True, help="Local CLIP ViT-B/32 snapshot.")
    parser.add_argument(
        "--analysis-image", type=Path, required=True, help="Single controlled image used for layer-wise analysis."
    )
    parser.add_argument("--output-dir", type=Path, required=True, help="Empty output directory for Toolkit snapshots.")
    return parser.parse_args()


def require_nonempty(path: Path, description: str) -> None:
    if not path.is_file() or path.stat().st_size == 0:
        raise ValueError(f"{description} is missing or empty: {path}")


def require_empty_output_dir(path: Path) -> None:
    if path.exists() and not path.is_dir():
        raise ValueError(f"analysis output path is not a directory: {path}")
    allowed_pre_analysis_files = {"texts_person_bus_padded.npy"}
    existing = {entry.name for entry in path.iterdir()} if path.exists() else set()
    if existing - allowed_pre_analysis_files:
        raise ValueError(f"analysis output directory must be empty to preserve raw Toolkit output: {path}")


def save_runtime_text_input(output_dir: Path, clip_model: Path) -> Path:
    text_path = output_dir / "texts_person_bus_padded.npy"
    values = generate_padded_embedding(RUNTIME_CLASSES, clip_model, RUNTIME_PAD_TOKEN)
    if values.dtype != np.float32 or list(values.shape) != TEXT_INPUT_SHAPE:
        raise ValueError(
            f"runtime text tensor has dtype/shape {values.dtype}/{list(values.shape)}, "
            f"expected float32/{TEXT_INPUT_SHAPE}"
        )
    if text_path.exists():
        existing = np.load(text_path, allow_pickle=False)
        if existing.dtype != np.float32 or list(existing.shape) != TEXT_INPUT_SHAPE:
            raise ValueError(
                f"existing runtime text tensor has dtype/shape {existing.dtype}/{list(existing.shape)}, "
                f"expected float32/{TEXT_INPUT_SHAPE}: {text_path}"
            )
        if not np.array_equal(existing, values):
            raise ValueError(f"existing runtime text tensor does not match the current CLIP embedding: {text_path}")
        print(f"Analysis texts (reused): {text_path}")
        return text_path

    with text_path.open("xb") as output:
        np.save(output, values)
    print(
        f"Analysis texts: {text_path} ({RUNTIME_CLASSES!r}, "
        f"then {CLASS_CAPACITY - len(RUNTIME_CLASSES)} x {RUNTIME_PAD_TOKEN!r})"
    )
    return text_path


def main() -> int:
    args = parse_args()
    try:
        require_nonempty(args.onnx, f"YOLO-World {args.variant} ONNX model")
        require_nonempty(args.calibration_dir / "dataset.txt", "INT8 calibration dataset")
        require_nonempty(args.analysis_image, "analysis image")
        require_empty_output_dir(args.output_dir)
        if not args.clip_model.is_dir():
            raise ValueError(f"local CLIP model directory is missing: {args.clip_model}")

        args.output_dir.mkdir(parents=True, exist_ok=True)
        texts = save_runtime_text_input(args.output_dir, args.clip_model)

        rknn = RKNN(verbose=False)
        try:
            print(f"--> Configuring {args.variant} model")
            ret = rknn.config(
                target_platform="rk3588",
                mean_values=[[0, 0, 0]],
                std_values=[[255, 255, 255]],
            )
            if ret != 0:
                raise RuntimeError(f"RKNN config failed with code {ret}")

            print("--> Loading model")
            ret = rknn.load_onnx(
                model=str(args.onnx),
                inputs=["images", "texts"],
                input_size_list=[IMAGE_INPUT_SHAPE, TEXT_INPUT_SHAPE],
            )
            if ret != 0:
                raise RuntimeError(f"RKNN load_onnx failed with code {ret}")

            print("--> Building INT8 model")
            ret = rknn.build(do_quantization=True, dataset=str(args.calibration_dir / "dataset.txt"))
            if ret != 0:
                raise RuntimeError(f"RKNN build failed with code {ret}")

            print("--> Running host-simulator layer-wise accuracy analysis")
            ret = rknn.accuracy_analysis(
                inputs=[str(args.analysis_image), str(texts)],
                output_dir=str(args.output_dir),
            )
            if ret != 0:
                raise RuntimeError(f"RKNN accuracy_analysis failed with code {ret}")
        finally:
            rknn.release()
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"{args.variant} RKNN accuracy analysis failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
