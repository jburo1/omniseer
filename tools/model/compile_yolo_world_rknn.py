#!/usr/bin/env python3
"""Compile the fixed YOLO-World v2-S detector contract with RKNN Toolkit2."""

import argparse
import sys
from pathlib import Path

from rknn.api import RKNN

IMAGE_INPUT_SHAPE = [1, 3, 640, 640]
TEXT_INPUT_SHAPE = [1, 80, 512]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compile an Omniseer-compatible YOLO-World v2-S ONNX model for RK3588."
    )
    parser.add_argument("onnx", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--precision", choices=("fp", "i8"), required=True)
    parser.add_argument(
        "--dataset",
        type=Path,
        help="Rockchip yolo_world dataset.txt; required for --precision i8.",
    )
    return parser.parse_args()


def require_nonempty(path: Path, description: str) -> None:
    if not path.is_file() or path.stat().st_size == 0:
        raise ValueError(f"{description} is missing or empty: {path}")


def main() -> int:
    args = parse_args()
    try:
        require_nonempty(args.onnx, "ONNX model")
        if args.precision == "i8":
            if args.dataset is None:
                raise ValueError("--dataset is required for --precision i8")
            require_nonempty(args.dataset, "INT8 calibration dataset")

        args.output.parent.mkdir(parents=True, exist_ok=True)
        if args.output.exists():
            args.output.unlink()

        rknn = RKNN(verbose=False)
        try:
            print("--> Config model")
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

            print("--> Building model")
            ret = rknn.build(
                do_quantization=args.precision == "i8",
                dataset=str(args.dataset) if args.precision == "i8" else None,
            )
            if ret != 0:
                raise RuntimeError(f"RKNN build failed with code {ret}")

            print("--> Exporting model")
            ret = rknn.export_rknn(str(args.output))
            if ret != 0:
                raise RuntimeError(f"RKNN export_rknn failed with code {ret}")
        finally:
            rknn.release()

        require_nonempty(args.output, "generated RKNN artifact")
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"model compile failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
