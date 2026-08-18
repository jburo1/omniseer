#!/usr/bin/env python3
"""Validate the fixed ONNX interface consumed by Omniseer's native runtime."""

import argparse
import sys
from collections.abc import Iterable
from pathlib import Path
from typing import Optional

import onnx

EXPECTED_INPUTS = {"images": [1, 3, 640, 640], "texts": [1, 80, 512]}
EXPECTED_GRIDS = (80, 40, 20)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Validate the Omniseer YOLO-World v2-S ONNX interface.")
    parser.add_argument("onnx", type=Path)
    return parser.parse_args()


def tensor_shape(value_info: onnx.ValueInfoProto) -> Optional[list[int]]:
    dims = value_info.type.tensor_type.shape.dim
    shape: list[int] = []
    for dim in dims:
        if not dim.HasField("dim_value"):
            return None
        shape.append(dim.dim_value)
    return shape


def format_shapes(values: Iterable[onnx.ValueInfoProto]) -> str:
    return ", ".join(f"{value.name}={tensor_shape(value)}" for value in values)


def main() -> int:
    args = parse_args()
    if not args.onnx.is_file() or args.onnx.stat().st_size == 0:
        print(f"ONNX model is missing or empty: {args.onnx}", file=sys.stderr)
        return 1

    try:
        model = onnx.load(str(args.onnx))
        onnx.checker.check_model(model)
    except (OSError, onnx.checker.ValidationError, ValueError) as exc:
        print(f"ONNX validation failed: {exc}", file=sys.stderr)
        return 1

    inputs = {value.name: tensor_shape(value) for value in model.graph.input}
    if set(inputs) != set(EXPECTED_INPUTS):
        print(
            "ONNX inputs must be exactly images and texts; found " + format_shapes(model.graph.input),
            file=sys.stderr,
        )
        return 1
    for name, expected_shape in EXPECTED_INPUTS.items():
        if inputs[name] != expected_shape:
            print(
                f"ONNX input {name} must have shape {expected_shape}; found {inputs[name]}",
                file=sys.stderr,
            )
            return 1

    outputs = [tensor_shape(value) for value in model.graph.output]
    expected_outputs = {(1, 80, grid, grid) for grid in EXPECTED_GRIDS} | {
        (1, 4, grid, grid) for grid in EXPECTED_GRIDS
    }
    if len(outputs) != 6 or set(map(tuple, filter(None, outputs))) != expected_outputs:
        print(
            "ONNX outputs must be the 80-class and 4-box NCHW tensors at "
            f"80x80, 40x40, and 20x20; found {format_shapes(model.graph.output)}",
            file=sys.stderr,
        )
        return 1

    print(f"validated Omniseer YOLO-World ONNX contract: {args.onnx}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
