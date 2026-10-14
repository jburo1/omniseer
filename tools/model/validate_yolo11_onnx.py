#!/usr/bin/env python3
"""Validate Rockchip YOLO11 detection outputs, independently of YOLO-World."""

from __future__ import annotations

import ast
from pathlib import Path

import onnx

COCO_NAMES = (
    "person,bicycle,car,motorcycle,airplane,bus,train,truck,boat,traffic light,fire hydrant,stop sign,"
    "parking meter,bench,bird,cat,dog,horse,sheep,cow,elephant,bear,zebra,giraffe,backpack,umbrella,handbag,"
    "tie,suitcase,frisbee,skis,snowboard,sports ball,kite,baseball bat,baseball glove,skateboard,surfboard,"
    "tennis racket,bottle,wine glass,cup,fork,knife,spoon,bowl,banana,apple,sandwich,orange,broccoli,carrot,"
    "hot dog,pizza,donut,cake,chair,couch,potted plant,bed,dining table,toilet,tv,laptop,mouse,remote,"
    "keyboard,cell phone,microwave,oven,toaster,sink,refrigerator,book,clock,vase,scissors,teddy bear,"
    "hair drier,toothbrush"
).split(",")


def validate_names(names: list[str]) -> None:
    if not all(isinstance(name, str) for name in names):
        raise ValueError("class names must be strings")
    normalized = [name.replace("_", " ") for name in names]
    if normalized != COCO_NAMES and normalized != ["person", "book", "power drill"]:
        raise ValueError("expected COCO-80 or ordered classes: person, book, power drill")


def validate(path: Path, names: list[str] | None = None) -> dict:
    model = onnx.load(str(path), load_external_data=False)
    onnx.checker.check_model(model, full_check=True)
    if any(t.data_location == onnx.TensorProto.EXTERNAL for t in model.graph.initializer):
        raise ValueError("external ONNX tensor data is unsupported; use a self-contained model")
    if names is None:
        metadata = {entry.key: entry.value for entry in model.metadata_props}
        raw = ast.literal_eval(metadata.get("names", "{}"))
        if not isinstance(raw, dict) or set(raw) != set(range(len(raw))):
            raise ValueError("ONNX must record contiguous class names in metadata")
        names = [raw[i] for i in range(len(raw))]
    validate_names(names)
    inferred = onnx.shape_inference.infer_shapes(model, strict_mode=True)

    def tensor(value):
        tt = value.type.tensor_type
        if tt.elem_type != onnx.TensorProto.FLOAT or any(not d.HasField("dim_value") for d in tt.shape.dim):
            raise ValueError(f"expected static float32 tensor: {value.name}")
        return {"name": value.name, "shape": [d.dim_value for d in tt.shape.dim], "dtype": "float32"}

    inputs = [tensor(v) for v in inferred.graph.input]
    outputs = [tensor(v) for v in inferred.graph.output]
    if inputs != [{"name": "images", "shape": [1, 3, 640, 640], "dtype": "float32"}]:
        raise ValueError(f"expected single images=[1,3,640,640] input; found {inputs}")
    expected = [[1, channels, grid, grid] for grid in (80, 40, 20) for channels in (64, len(names), 1)]
    if [v["shape"] for v in outputs] != expected:
        raise ValueError(f"expected nine ordered box-distribution/class/score-sum tensors {expected}; found {outputs}")
    if [(x.domain, x.version) for x in model.opset_import] != [("", 12)]:
        raise ValueError("expected Rockchip export ONNX opset 12")
    return {"inputs": inputs, "outputs": outputs, "classes": names, "opset": 12, "reg_max": 16}


def smoke_inference(path: Path, contract: dict) -> dict:
    """Exercise the graph on a deterministic normalized RGB input on the host."""
    import numpy as np
    import onnxruntime as ort

    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    options.inter_op_num_threads = 1
    session = ort.InferenceSession(str(path), sess_options=options, providers=["CPUExecutionProvider"])
    outputs = session.run(None, {"images": np.zeros((1, 3, 640, 640), dtype=np.float32)})
    for value, expected in zip(outputs, contract["outputs"]):
        if list(value.shape) != expected["shape"] or value.dtype != np.float32 or not np.isfinite(value).all():
            raise ValueError(f"invalid ONNX inference output: {expected['name']}")
    for offset in (0, 3, 6):
        classes, score_sum = outputs[offset + 1 : offset + 3]
        if np.any(classes < 0) or np.any(classes > 1):
            raise ValueError("class outputs must be probabilities")
        if not np.allclose(score_sum, np.clip(classes.sum(axis=1, keepdims=True), 0, 1), atol=1e-5):
            raise ValueError("score-sum branch disagrees with class probabilities")
    return {
        "provider": "CPUExecutionProvider",
        "input": "all-zero normalized RGB",
        "finite_outputs": True,
        "class_probabilities_and_score_sums": "validated",
    }
