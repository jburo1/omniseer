#!/usr/bin/env python3
"""Generate the fixed 80-row YOLO-World INT8 calibration text input."""

from __future__ import annotations

import argparse
import os
from pathlib import Path

import numpy as np
import torch
from transformers import AutoTokenizer, CLIPTextModelWithProjection

CLASS_CAPACITY = 80
EMBEDDING_WIDTH = 512


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate a YOLO-World calibration text embedding from 80 class names."
    )
    parser.add_argument("--classes", type=Path, required=True, help="Comment-tolerant newline-delimited class list.")
    parser.add_argument("--clip-model", type=Path, required=True, help="Local CLIP ViT-B/32 model snapshot.")
    parser.add_argument("--output", type=Path, required=True, help="Output NumPy .npy file.")
    return parser.parse_args()


def read_classes(path: Path) -> list[str]:
    if not path.is_file() or path.stat().st_size == 0:
        raise ValueError(f"calibration class list is missing or empty: {path}")
    classes = [line.strip() for line in path.read_text(encoding="utf-8").splitlines()]
    classes = [label for label in classes if label and not label.startswith("#")]
    if len(classes) != CLASS_CAPACITY:
        raise ValueError(f"calibration class list has {len(classes)} labels, expected {CLASS_CAPACITY}: {path}")
    if len({label.casefold() for label in classes}) != CLASS_CAPACITY:
        raise ValueError(f"calibration class list contains duplicate labels: {path}")
    return classes


def generate_embedding(classes: list[str], clip_model: Path) -> np.ndarray:
    if not clip_model.is_dir():
        raise ValueError(f"local CLIP model directory is missing: {clip_model}")
    tokenizer = AutoTokenizer.from_pretrained(clip_model, local_files_only=True)
    model = CLIPTextModelWithProjection.from_pretrained(clip_model, local_files_only=True).eval()
    with torch.inference_mode():
        inputs = tokenizer(text=classes, return_tensors="pt", padding=True)
        values = model(**inputs).text_embeds
        values = values / values.norm(p=2, dim=-1, keepdim=True)
    embedding = values.cpu().numpy().astype(np.float32, copy=False)
    if embedding.shape != (CLASS_CAPACITY, EMBEDDING_WIDTH):
        raise ValueError(
            f"CLIP text embedding has shape {list(embedding.shape)}, expected [{CLASS_CAPACITY}, {EMBEDDING_WIDTH}]"
        )
    return np.expand_dims(embedding, axis=0)


def main() -> int:
    args = parse_args()
    try:
        classes = read_classes(args.classes)
        embedding = generate_embedding(classes, args.clip_model)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        temporary_output = args.output.with_name(f"{args.output.name}.part")
        if temporary_output.exists():
            raise ValueError(f"incomplete calibration embedding already exists: {temporary_output}")
        try:
            with temporary_output.open("wb") as output:
                np.save(output, embedding)
            os.replace(temporary_output, args.output)
        finally:
            temporary_output.unlink(missing_ok=True)
        print(f"Calibration text embedding: {args.output} ({len(classes)} labels)")
    except (OSError, ValueError) as exc:
        print(f"calibration text embedding generation failed: {exc}", file=os.sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
