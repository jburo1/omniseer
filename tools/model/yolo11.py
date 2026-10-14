#!/usr/bin/env python3
"""Isolated YOLO11 assets, Rockchip export, and RK3588 compilation."""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
import os
import platform
import shutil
import sys
import tempfile
import urllib.request
from contextlib import contextmanager
from pathlib import Path

EXPORTER_REVISION = "0692e9297670acf4cc6d0cec773d7a9493cb8a5f"
ASSET_REVISION = "0f20d143ac8f9e01322f85dcb5c10dc9b7ca1153"
ASSET_URL = "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt"
ASSET_SHA256 = "0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1"


def sha256(path: Path) -> str:
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest() if hasattr(hashlib, "file_digest") else _hash(stream)


def _hash(stream) -> str:
    digest = hashlib.sha256()
    for block in iter(lambda: stream.read(1024 * 1024), b""):
        digest.update(block)
    return digest.hexdigest()


def write_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


@contextmanager
def artifact(output: Path):
    """Reserve destination and publish only validated results without replacement."""
    output.parent.mkdir(parents=True, exist_ok=True)
    sidecar = Path(str(output) + ".json")
    lock = Path(str(output) + ".lock")
    for path in (output, sidecar, lock):
        if path.exists() or path.is_symlink():
            raise ValueError(f"refusing to overwrite existing artifact: {path}")
    with lock.open("x"):
        try:
            with tempfile.TemporaryDirectory(prefix=".yolo11-", dir=output.parent) as directory:
                temporary = Path(directory) / output.name
                metadata = {}
                yield temporary, metadata
                if not temporary.is_file() or temporary.stat().st_size == 0:
                    raise ValueError("builder produced no artifact")
                metadata["artifact"] = {"filename": output.name, "sha256": sha256(temporary)}
                temporary_sidecar = Path(directory) / "provenance.json"
                write_json(temporary_sidecar, metadata)
                os.link(temporary_sidecar, sidecar)
                try:
                    os.link(temporary, output)
                except BaseException:
                    sidecar.unlink()
                    raise
        finally:
            lock.unlink()


def assets(directory: Path) -> None:
    output = directory / "yolo11n.pt"
    metadata = {
        "source_url": ASSET_URL,
        "source_release": "v8.3.0",
        "source_revision": ASSET_REVISION,
        "sha256": ASSET_SHA256,
    }
    sidecar = Path(str(output) + ".json")
    if output.exists():
        if sha256(output) != ASSET_SHA256 or not sidecar.is_file():
            raise ValueError("existing checkpoint hash/provenance mismatch; refusing replacement")
        existing = json.loads(sidecar.read_text())
        if any(existing.get(key) != value for key, value in metadata.items()):
            raise ValueError("existing checkpoint provenance mismatch")
    else:
        with artifact(output) as (temporary, provenance):
            with urllib.request.urlopen(ASSET_URL, timeout=60) as source, temporary.open("xb") as target:
                shutil.copyfileobj(source, target)
            if sha256(temporary) != ASSET_SHA256:
                raise ValueError("downloaded checkpoint SHA-256 mismatch")
            provenance.update(metadata)
    print(f"validated checkpoint: {output} sha256={ASSET_SHA256}")


def versions() -> dict:
    expected = {"ultralytics": "8.3.9", "rknn-toolkit2": "2.3.0", "torch": "2.2.0+cpu", "onnx": "1.14.1"}
    actual = {name: importlib.metadata.version(name) for name in expected}
    if actual != expected:
        raise ValueError(f"incompatible builder versions: {actual}; expected {expected}")
    return {"python": platform.python_version(), **actual}


def provenance(input_path: Path, operation: str) -> dict:
    parent = Path(str(input_path) + ".json")
    result = {
        "operation": operation,
        "input": {"filename": input_path.name, "sha256": sha256(input_path)},
        "versions": versions(),
        "packages": dict(sorted((d.metadata["Name"], d.version) for d in importlib.metadata.distributions())),
        "exporter_repository": "https://github.com/airockchip/ultralytics_yolo11",
        "exporter_revision": EXPORTER_REVISION,
        "builder_image_id": os.environ.get("OMNISEER_YOLO11_IMAGE_ID"),
        "tool_sha256": {
            p.name: sha256(p) for p in (Path(__file__), Path(__file__).with_name("validate_yolo11_onnx.py"))
        },
    }
    if parent.exists():
        result["input_provenance"] = json.loads(parent.read_text())
        if result["input_provenance"].get("artifact", {}).get("sha256") != sha256(input_path):
            raise ValueError("input artifact does not match its provenance SHA-256")
    return result


def export(weights: Path, output: Path) -> None:
    from ultralytics import YOLO
    from validate_yolo11_onnx import smoke_inference, validate, validate_names

    metadata = provenance(weights, "export")
    with artifact(output) as (temporary, record):
        # The fork writes next to its checkpoint. Copy into staging to preserve inputs.
        staged = temporary.parent / "checkpoint.pt"
        shutil.copyfile(weights, staged)
        model = YOLO(str(staged), task="detect")
        if model.task != "detect" or model.model.model[-1].__class__.__name__ != "Detect":
            raise ValueError("only YOLO11 Detect checkpoints are supported")
        if not any(layer.__class__.__name__ == "C3k2" for layer in model.model.model):
            raise ValueError("checkpoint is not a YOLO11 architecture")
        names = [model.names[i] for i in range(len(model.names))]
        validate_names(names)
        generated = model.export(format="rknn", imgsz=640, batch=1, device="cpu", dynamic=False, half=False)
        if not generated or not Path(generated).is_file():
            raise ValueError("Rockchip exporter failed")
        # export_rknn omits class metadata; add it without rewriting output shapes.
        import onnx

        graph = onnx.load(str(generated))
        onnx.helper.set_model_props(
            graph, {"names": str(dict(enumerate(names))), "exporter_revision": EXPORTER_REVISION}
        )
        onnx.save(graph, str(temporary))
        contract = validate(temporary)
        record.update(
            metadata,
            smoke_inference=smoke_inference(temporary, contract),
            contract=contract,
            export_options={"format": "rknn", "imgsz": 640, "batch": 1, "device": "cpu"},
        )


def calibration(manifest: Path, workdir: Path) -> tuple[Path, dict]:
    """Validate split provenance and hash disjointness; letterbox dedicated images."""
    import cv2

    data = json.loads(manifest.read_text())
    if data.get("split") != "calibration" or not data.get("source") or not data.get("representativeness"):
        raise ValueError("calibration manifest requires split=calibration, source, and representativeness")
    excluded = data.get("excluded_splits", {})
    if set(excluded) != {"validation", "test"} or not all(excluded.values()):
        raise ValueError("provide non-empty validation and test image lists in excluded_splits")
    root = manifest.parent.resolve()

    def entries(paths):
        if not isinstance(paths, list) or not paths:
            raise ValueError("image lists must be non-empty")
        result = []
        for item in paths:
            path = (root / item).resolve()
            if not path.is_relative_to(root) or not path.is_file():
                raise ValueError(f"image must exist beneath manifest directory: {item}")
            result.append((path, sha256(path)))
        return result

    images = entries(data.get("images"))
    excluded_images = {split: entries(paths) for split, paths in excluded.items()}
    hashes = [digest for _, digest in images]
    if len(set(hashes)) != len(hashes):
        raise ValueError("duplicate calibration content")
    held_out = {digest for items in excluded_images.values() for _, digest in items}
    if held_out.intersection(hashes):
        raise ValueError("calibration images overlap validation/test by SHA-256")
    records, dataset = [], workdir / "dataset.txt"
    paths = []
    for index, (path, digest) in enumerate(images):
        image = cv2.imread(str(path))
        if image is None:
            raise ValueError(f"invalid calibration image: {path}")
        height, width = image.shape[:2]
        scale = min(640 / height, 640 / width)
        resized = cv2.resize(image, (round(width * scale), round(height * scale)), interpolation=cv2.INTER_LINEAR)
        h, w = resized.shape[:2]
        image = cv2.copyMakeBorder(
            resized,
            (640 - h) // 2,
            (641 - h) // 2,
            (640 - w) // 2,
            (641 - w) // 2,
            cv2.BORDER_CONSTANT,
            value=(114, 114, 114),
        )
        target = workdir / f"calibration-{index:06d}.png"
        if not cv2.imwrite(str(target), image):
            raise ValueError("failed to prepare calibration image")
        paths.append(str(target))
        records.append({"path": str(path.relative_to(root)), "sha256": digest, "letterboxed_sha256": sha256(target)})
    dataset.write_text("\n".join(paths) + "\n")
    return dataset, {
        "manifest_sha256": sha256(manifest),
        "source": data["source"],
        "representativeness": data["representativeness"],
        "split": "calibration",
        "images": records,
        "excluded_splits": {
            split: [{"path": str(p.relative_to(root)), "sha256": h} for p, h in items]
            for split, items in excluded_images.items()
        },
        "preprocessing": "centered 640x640 letterbox, bilinear resize, RGB, padding=114; RKNN normalizes by 255",
    }


def compile_model(onnx_path: Path, output: Path, precision: str, manifest: Path | None) -> None:
    from rknn.api import RKNN
    from validate_yolo11_onnx import smoke_inference, validate

    contract = validate(onnx_path)
    metadata = provenance(onnx_path, "compile")
    metadata["smoke_inference"] = smoke_inference(onnx_path, contract)
    if precision == "int8" and manifest is None:
        raise ValueError("INT8 requires --calibration-manifest")
    if precision == "fp" and manifest is not None:
        raise ValueError("FP does not use calibration")
    with artifact(output) as (temporary, record):
        dataset, calibration_record = calibration(manifest, temporary.parent) if manifest else (None, None)
        options = {"target_platform": "rk3588", "mean_values": [[0, 0, 0]], "std_values": [[255, 255, 255]]}
        rknn = RKNN(verbose=False)
        try:

            def check(code, step):
                if code != 0:
                    raise RuntimeError(f"RKNN {step} failed: {code}")

            check(rknn.config(**options), "config")
            check(rknn.load_onnx(model=str(onnx_path)), "load_onnx")
            check(rknn.build(do_quantization=precision == "int8", dataset=str(dataset) if dataset else None), "build")
            check(rknn.export_rknn(str(temporary)), "export_rknn")
        finally:
            rknn.release()
        record.update(
            metadata, contract=contract, precision=precision, compiler_options=options, calibration=calibration_record
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    asset_parser = commands.add_parser("assets")
    asset_parser.add_argument("--directory", type=Path, required=True)
    export_parser = commands.add_parser("export")
    export_parser.add_argument("--weights", type=Path, required=True)
    export_parser.add_argument("--output", type=Path, required=True)
    compile_parser = commands.add_parser("compile")
    compile_parser.add_argument("--onnx", type=Path, required=True)
    compile_parser.add_argument("--output", type=Path, required=True)
    compile_parser.add_argument("--precision", choices=("fp", "int8"), required=True)
    compile_parser.add_argument("--calibration-manifest", type=Path)
    args = parser.parse_args()
    try:
        if args.command != "assets":
            if args.output.is_symlink():
                raise ValueError("refusing to overwrite existing artifact symlink")
            suffix = ".onnx" if args.command == "export" else ".rknn"
            if args.output.suffix != suffix:
                raise ValueError(f"output must end in {suffix}")
        if args.command == "assets":
            assets(args.directory)
        elif args.command == "export":
            export(args.weights.resolve(), args.output.resolve())
            print(f"ONNX artifact: {args.output} sha256={sha256(args.output)}")
        else:
            compile_model(args.onnx.resolve(), args.output.resolve(), args.precision, args.calibration_manifest)
            print(f"RKNN artifact: {args.output} sha256={sha256(args.output)}")
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"YOLO11 {args.command} failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
