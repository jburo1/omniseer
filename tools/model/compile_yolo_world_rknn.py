#!/usr/bin/env python3
"""Compile the fixed YOLO-World v2 detector contract with RKNN Toolkit2."""

import argparse
import os
import re
import shutil
import sys
from contextlib import contextmanager
from pathlib import Path
from typing import Optional

import numpy as np
from rknn.api import RKNN

IMAGE_INPUT_SHAPE = [1, 3, 640, 640]
TEXT_INPUT_SHAPE = [1, 80, 512]
CALIBRATION_TEXT_EMBEDDING = "calibration_text_outp.npy"
HYBRID_LAYERS = [
    [
        "/neck/upsample_layers.0/Resize_output_0",
        "/neck/Concat_output_0",
    ],
    [
        "/neck/top_down_layers.0/Concat_output_0",
        "/neck/top_down_layers.0/final_conv/conv/Conv_output_0",
    ],
    [
        "/neck/top_down_layers.0/final_conv/conv/Conv_output_0",
        "/neck/top_down_layers.0/final_conv/activate/Mul_output_0",
    ],
    [
        "/neck/top_down_layers.0/final_conv/activate/Mul_output_0",
        "/neck/top_down_layers.1/final_conv/activate/Mul_output_0",
    ],
]
# These are the generated terminal classification output names.  Keeping only
# these entries FP16 asks Toolkit2 to insert output conversion without
# promoting the cls_preds or cls_contrasts computations which feed them.
CLASSIFICATION_OUTPUTS = ("1577_int8", "1579_int8", "1581_int8")
CLASSIFICATION_PROJECTION_TENSORS = (
    "/bbox_head/head_module/cls_preds.0/cls_preds.0.2/Conv_output_0",
    "/bbox_head/head_module/cls_preds.0/cls_preds.0.2/Conv_output_0_rs",
    "/bbox_head/head_module/cls_preds.0/cls_preds.0.2/Conv_output_0_rs_mm",
    "/bbox_head/head_module/cls_preds.0/cls_preds.0.2/Conv_output_0_rs_mm_rs",
)
# Toolkit2 lowers the shared text embedding to this reshape/transpose result
# before it feeds the classification exMatMul operators.  This is the narrowest
# generated-config name available at that operand boundary.
CLASSIFICATION_TEXT_MATMUL_TENSOR = "texts_tp-rs"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Compile an Omniseer-compatible YOLO-World v2 ONNX model for RK3588.")
    parser.add_argument("onnx", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--precision", choices=("fp", "i8", "hybrid"), required=True)
    parser.add_argument(
        "--dataset",
        type=Path,
        help="Rockchip yolo_world dataset.txt; required for --precision i8 or hybrid.",
    )
    parser.add_argument(
        "--hybrid-workdir",
        type=Path,
        help="Empty directory in which Toolkit2 writes the reproducible hybrid configuration.",
    )
    parser.add_argument(
        "--hybrid-template-workdir",
        type=Path,
        help="Existing Toolkit2 step-1 work directory to use as the exact hybrid baseline.",
    )
    return parser.parse_args()


def require_nonempty(path: Path, description: str) -> None:
    if not path.is_file() or path.stat().st_size == 0:
        raise ValueError(f"{description} is missing or empty: {path}")


def require_text_embedding(dataset: Path) -> None:
    """Validate the fixed calibration text input before RKNN consumes it."""
    embedding = dataset.parent / CALIBRATION_TEXT_EMBEDDING
    require_nonempty(embedding, "INT8 text embedding")
    try:
        values = np.load(embedding, allow_pickle=False)
    except (OSError, ValueError) as exc:
        raise ValueError(f"INT8 text embedding is not a valid NumPy file: {embedding}") from exc
    if list(values.shape) != TEXT_INPUT_SHAPE:
        raise ValueError(
            f"INT8 text embedding has shape {list(values.shape)}, expected {TEXT_INPUT_SHAPE}: {embedding}"
        )


@contextmanager
def working_directory(path: Path):
    previous = Path.cwd()
    os.chdir(path)
    try:
        yield
    finally:
        os.chdir(previous)


def require_empty_hybrid_workdir(path: Path) -> None:
    if path.exists() and (not path.is_dir() or any(path.iterdir())):
        raise ValueError(f"hybrid work directory must be empty: {path}")
    path.mkdir(parents=True, exist_ok=True)


def augment_td01_classification_projection(config: Path) -> None:
    """Add the 80x80 projection and its text operand at the MatMul boundary."""
    contents = config.read_text(encoding="utf-8")
    custom, parameters = contents.split("quantize_parameters:\n", 1)
    existing = set(re.findall(r"^    (\S+): float16$", custom, flags=re.MULTILINE))
    expected_head_entries = set(CLASSIFICATION_PROJECTION_TENSORS)
    unexpected_head_entries = [
        entry for entry in existing if "/bbox_head/head_module/" in entry and entry not in expected_head_entries
    ]
    if unexpected_head_entries:
        raise ValueError(
            "hybrid template must be the output-FP16 control with no head FP16 entries: "
            + ", ".join(sorted(unexpected_head_entries))
        )
    missing_outputs = [entry for entry in CLASSIFICATION_OUTPUTS if entry not in existing]
    if missing_outputs:
        raise ValueError(
            "hybrid template is missing validated FP16 classification outputs: " + ", ".join(missing_outputs)
        )
    missing_projection_entries = [entry for entry in CLASSIFICATION_PROJECTION_TENSORS if entry not in existing]
    custom += "".join(f"    {entry}: float16\n" for entry in missing_projection_entries)
    if CLASSIFICATION_TEXT_MATMUL_TENSOR not in existing:
        custom += f"    {CLASSIFICATION_TEXT_MATMUL_TENSOR}: float16\n"
    config.write_text(custom + "quantize_parameters:\n" + parameters, encoding="utf-8")


def build_hybrid(rknn: RKNN, onnx: Path, dataset: Path, workdir: Path, template_workdir: Optional[Path] = None) -> None:
    """Use Toolkit2's supported two-step hybrid quantization workflow."""
    require_empty_hybrid_workdir(workdir)
    model_input = f"{onnx.stem}.model"
    data_input = f"{onnx.stem}.data"
    model_quantization_cfg = f"{onnx.stem}.quantization.cfg"
    with working_directory(workdir):
        if template_workdir is None:
            print(f"--> Generating hybrid configuration in {workdir}")
            ret = rknn.hybrid_quantization_step1(dataset=str(dataset), custom_hybrid=HYBRID_LAYERS)
            if ret != 0:
                raise RuntimeError(f"RKNN hybrid_quantization_step1 failed with code {ret}")
        else:
            print(f"--> Copying exact hybrid baseline from {template_workdir}")
            for filename in (model_input, data_input, model_quantization_cfg):
                require_nonempty(template_workdir / filename, f"RKNN hybrid baseline {filename}")
                shutil.copy2(template_workdir / filename, workdir / filename)
        for filename in (model_input, data_input, model_quantization_cfg):
            require_nonempty(workdir / filename, f"RKNN hybrid configuration {filename}")
        augment_td01_classification_projection(workdir / model_quantization_cfg)
        print("--> Building hybrid model")
        ret = rknn.hybrid_quantization_step2(model_input, data_input, model_quantization_cfg)
        if ret != 0:
            raise RuntimeError(f"RKNN hybrid_quantization_step2 failed with code {ret}")


def main() -> int:
    args = parse_args()
    try:
        require_nonempty(args.onnx, "ONNX model")
        if args.precision in {"i8", "hybrid"}:
            if args.dataset is None:
                raise ValueError(f"--dataset is required for --precision {args.precision}")
            require_nonempty(args.dataset, "INT8 calibration dataset")
            require_text_embedding(args.dataset)
        if args.precision == "hybrid" and args.hybrid_workdir is None:
            raise ValueError("--hybrid-workdir is required for --precision hybrid")

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

            if args.precision == "hybrid":
                build_hybrid(
                    rknn,
                    args.onnx,
                    args.dataset,
                    args.hybrid_workdir,
                    args.hybrid_template_workdir,
                )
            else:
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
