"""Portable contract tests; no exporter, Toolkit, Docker daemon or robot needed."""

import importlib.util
import json
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("yolo11_builder", REPO / "tools/model/yolo11.py")
builder = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(builder)


def cli(*args):
    return subprocess.run(["scripts/omni", "model", "yolo11", *args], cwd=REPO, text=True, capture_output=True)


def test_frontdoor_and_fail_fast(tmp_path):
    assert cli("--help").returncode == 0
    assert "--calibration-manifest" in cli("--help").stdout
    assert "requires --weights" in cli("export").stderr
    assert "--precision must be fp or int8" in cli("compile", "--onnx", "missing.onnx").stderr
    assert "path does not exist" in cli("export", "--weights", "missing.pt").stderr


def test_artifact_preserves_existing_and_cleans_failure(tmp_path):
    output = tmp_path / "model.onnx"
    with pytest.raises(RuntimeError), builder.artifact(output) as (stage, _):
        stage.write_bytes(b"incomplete")
        raise RuntimeError("export failed")
    assert not output.exists()
    assert not Path(str(output) + ".lock").exists()
    with builder.artifact(output) as (stage, record):
        stage.write_bytes(b"model")
        record["operation"] = "test"
    original = output.read_bytes()
    with pytest.raises(ValueError, match="refusing to overwrite"), builder.artifact(output):
        pass
    assert output.read_bytes() == original
    assert json.loads(Path(str(output) + ".json").read_text())["artifact"]["sha256"] == builder.sha256(output)


def test_assets_hash_validation_without_download(tmp_path):
    (tmp_path / "yolo11n.pt").write_bytes(b"wrong weights")
    with pytest.raises(ValueError, match="hash/provenance mismatch"):
        builder.assets(tmp_path)
    assert (tmp_path / "yolo11n.pt").read_bytes() == b"wrong weights"


def validator():
    pytest.importorskip("onnx")
    spec = importlib.util.spec_from_file_location("yolo11_contract", REPO / "tools/model/validate_yolo11_onnx.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def make_onnx(path, names, channels=None, extra_input=False):
    import onnx
    from onnx import TensorProto, helper

    shapes = [[1, c, g, g] for g in (80, 40, 20) for c in (64, len(names), 1)]
    if channels:
        shapes[0][1] = channels
    outputs, nodes = [], []
    for i, shape in enumerate(shapes):
        name = f"output{i}"
        outputs.append(helper.make_tensor_value_info(name, TensorProto.FLOAT, shape))
        # ConstantOfShape provides real shape inference, avoiding fake output metadata.
        shape_tensor = helper.make_tensor(f"shape{i}", TensorProto.INT64, [4], shape)
        nodes.append(helper.make_node("Constant", [], [f"shape{i}"], value=shape_tensor))
        nodes.append(helper.make_node("ConstantOfShape", [f"shape{i}"], [name]))
    inputs = [helper.make_tensor_value_info("images", TensorProto.FLOAT, [1, 3, 640, 640])]
    if extra_input:
        inputs.append(helper.make_tensor_value_info("texts", TensorProto.FLOAT, [1, 80, 512]))
    model = helper.make_model(
        helper.make_graph(nodes, "test", inputs, outputs), opset_imports=[helper.make_opsetid("", 12)]
    )
    helper.set_model_props(model, {"names": str(dict(enumerate(names)))})
    onnx.save(model, path)


@pytest.mark.parametrize("count", [3, 80])
def test_onnx_accepts_actual_nine_output_contract(tmp_path, count):
    module = validator()
    names = module.COCO_NAMES if count == 80 else ["person", "book", "power drill"]
    path = tmp_path / "model.onnx"
    make_onnx(path, names)
    assert len(module.validate(path)["outputs"]) == 9


@pytest.mark.parametrize("options", [{"channels": 4}, {"extra_input": True}])
def test_onnx_rejects_world_layout(tmp_path, options):
    module = validator()
    path = tmp_path / "model.onnx"
    make_onnx(path, module.COCO_NAMES, **options)
    with pytest.raises(ValueError):
        module.validate(path)


def test_class_mapping_rejects_wrong_order():
    with pytest.raises(ValueError, match="ordered classes"):
        validator().validate_names(["book", "person", "power drill"])


def calibration_manifest(tmp_path):
    cv2 = pytest.importorskip("cv2")
    import numpy as np

    for i, name in enumerate(["calibration", "validation", "test"]):
        cv2.imwrite(str(tmp_path / f"{name}.png"), np.full((200, 400, 3), i * 50, dtype=np.uint8))
    manifest = tmp_path / "manifest.json"
    data = {
        "split": "calibration",
        "source": "dedicated training captures",
        "representativeness": "robot camera; varied lighting",
        "images": ["calibration.png"],
        "excluded_splits": {"validation": ["validation.png"], "test": ["test.png"]},
    }
    manifest.write_text(json.dumps(data))
    return manifest, data


def test_calibration_letterboxes_and_records_hashes(tmp_path):
    manifest, _ = calibration_manifest(tmp_path)
    work = tmp_path / "work"
    work.mkdir()
    dataset, record = builder.calibration(manifest, work)
    import cv2

    image = cv2.imread(dataset.read_text().strip())
    assert image.shape == (640, 640, 3)
    assert image[0, 0].tolist() == [114, 114, 114]
    assert record["images"][0]["sha256"] == builder.sha256(tmp_path / "calibration.png")


def test_calibration_rejects_content_leakage_and_missing_splits(tmp_path):
    manifest, data = calibration_manifest(tmp_path)
    (tmp_path / "validation.png").write_bytes((tmp_path / "calibration.png").read_bytes())
    with pytest.raises(ValueError, match="overlap validation/test"):
        builder.calibration(manifest, tmp_path)
    data["excluded_splits"] = {}
    manifest.write_text(json.dumps(data))
    with pytest.raises(ValueError, match="validation and test"):
        builder.calibration(manifest, tmp_path)


def test_frontdoor_uses_shared_docker_mount_and_resolves_image_id(monkeypatch, tmp_path):
    import os
    import tempfile

    log = tmp_path / "docker.jsonl"
    docker = tmp_path / "docker"
    docker.write_text(
        "#!/usr/bin/env python3\nimport json, os, sys\n"
        "with open(os.environ['YOLO11_TEST_DOCKER_LOG'], 'a') as f: f.write(json.dumps(sys.argv[1:])+'\\n')\n"
        "if sys.argv[1:3] == ['image', 'inspect']: print('sha256:test-image')\n"
    )
    docker.chmod(0o755)
    monkeypatch.setenv("PATH", str(tmp_path) + os.pathsep + os.environ["PATH"])
    monkeypatch.setenv("YOLO11_TEST_DOCKER_LOG", str(log))
    with tempfile.TemporaryDirectory(dir=REPO) as directory:
        weights = Path(directory) / "custom.pt"
        weights.write_bytes(b"checkpoint")
        result = cli("export", "--weights", str(weights))
    assert result.returncode == 0, result.stderr
    calls = [json.loads(line) for line in log.read_text().splitlines()]
    invocation = calls[-1]
    assert invocation[:3] == ["run", "--rm", "--user"]
    assert "sha256:test-image" in invocation
    assert "OMNISEER_YOLO11_IMAGE_ID=sha256:test-image" in invocation
    assert "/workspace/artifacts/models/yolo11/custom.onnx" in invocation
    assert any("dst=/workspace" in argument for argument in invocation)


def test_frontdoor_rejects_output_outside_yolo11_before_docker():
    import tempfile

    with tempfile.TemporaryDirectory(dir=REPO) as directory:
        weights = Path(directory) / "checkpoint.pt"
        weights.write_bytes(b"checkpoint")
        result = cli("export", "--weights", str(weights), "--output", "artifacts/models/world.onnx")
    assert result.returncode != 0
    assert "outputs must be under artifacts/models/yolo11/" in result.stderr


def test_compiler_failure_releases_toolkit_and_publishes_nothing(monkeypatch, tmp_path):
    import sys
    import types

    module = validator()
    monkeypatch.setattr(module, "smoke_inference", lambda *_: {})
    monkeypatch.setitem(sys.modules, "validate_yolo11_onnx", module)
    monkeypatch.setattr(builder, "versions", lambda: {})
    released = []

    class FailingRKNN:
        def __init__(self, **_):
            pass

        def config(self, **_):
            return 0

        def load_onnx(self, **_):
            return 0

        def build(self, **_):
            return 7

        def release(self):
            released.append(True)

    monkeypatch.setitem(sys.modules, "rknn", types.ModuleType("rknn"))
    api = types.ModuleType("rknn.api")
    api.RKNN = FailingRKNN
    monkeypatch.setitem(sys.modules, "rknn.api", api)
    source = tmp_path / "model.onnx"
    output = tmp_path / "model.rknn"
    make_onnx(source, module.COCO_NAMES)
    with pytest.raises(RuntimeError, match="build failed: 7"):
        builder.compile_model(source, output, "fp", None)
    assert released == [True]
    assert not output.exists()
    assert not Path(str(output) + ".json").exists()
    assert not Path(str(output) + ".lock").exists()
