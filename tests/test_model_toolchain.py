import struct
import subprocess
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]


def resolve_variant(variant: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [
            "bash",
            "-c",
            (
                'source scripts/model.sh; model_select_variant "$1"; '
                "printf '%s|%s|%s|%s.onnx|%s_fp.rknn|%s_i8.rknn\\n' "
                '"$model_variant" "$model_checkpoint_name" "$model_config_path" '
                '"$model_artifact_stem" "$model_artifact_stem" "$model_artifact_stem"'
            ),
            "model-variant-test",
            variant,
        ],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )


def test_model_help_is_available_from_omni_front_door() -> None:
    result = subprocess.run(
        ["scripts/omni", "model", "--help"],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "model assets" in result.stdout
    assert "model export [--variant v2s|v2m] --weights" in result.stdout
    assert "model compile [--variant v2s|v2m] --onnx" in result.stdout
    assert "model build [--variant v2s|v2m] --weights" in result.stdout


def test_model_variant_v2s_resolves_expected_checkpoint_config_and_artifacts() -> None:
    result = resolve_variant("v2s")

    assert result.returncode == 0, result.stderr
    assert result.stdout == (
        "v2s|yolo_world_v2_s_obj365v1_goldg_pretrain-55b943ea.pth|"
        "configs/pretrain/yolo_world_v2_s_vlpan_bn_2e-3_100e_4x8gpus_obj365v1_goldg_train_lvis_minival.py|"
        "yolo_world_v2_s.onnx|yolo_world_v2_s_fp.rknn|yolo_world_v2_s_i8.rknn\n"
    )


def test_model_variant_v2m_resolves_expected_checkpoint_config_and_artifacts() -> None:
    result = resolve_variant("v2m")

    assert result.returncode == 0, result.stderr
    assert result.stdout == (
        "v2m|yolo_world_v2_m_obj365v1_goldg_pretrain-c6237d5b.pth|"
        "configs/pretrain/yolo_world_v2_m_vlpan_bn_2e-3_100e_4x8gpus_obj365v1_goldg_train_lvis_minival.py|"
        "yolo_world_v2_m.onnx|yolo_world_v2_m_fp.rknn|yolo_world_v2_m_i8.rknn\n"
    )


def test_model_variant_rejects_unsupported_value() -> None:
    result = subprocess.run(
        ["scripts/omni", "model", "export", "--variant", "v2x"],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode != 0
    assert "unsupported YOLO-World model variant: v2x; expected v2s or v2m" in result.stderr


def test_model_export_defaults_to_v2s() -> None:
    with tempfile.TemporaryDirectory(dir=REPO_ROOT) as temporary_directory:
        checkpoint = Path(temporary_directory) / "yolo_world_v2_m_obj365v1_goldg_pretrain-c6237d5b.pth"
        clip_model = Path(temporary_directory) / "clip"
        checkpoint.write_bytes(b"checkpoint")
        clip_model.mkdir()
        (clip_model / "config.json").write_text("{}", encoding="utf-8")
        (clip_model / "pytorch_model.bin").write_bytes(b"weights")
        result = subprocess.run(
            ["scripts/omni", "model", "export", "--weights", str(checkpoint), "--clip-model", str(clip_model)],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
            check=False,
        )

    assert result.returncode != 0
    assert "unsupported checkpoint; expected yolo_world_v2_s_obj365v1_goldg_pretrain-55b943ea.pth" in result.stderr


def test_model_export_rejects_missing_checkpoint_before_docker() -> None:
    missing_weights = "models/source/yolo_world/yolo_world_v2_s_obj365v1_goldg_pretrain-55b943ea.pth"
    result = subprocess.run(
        ["scripts/omni", "model", "export", "--weights", missing_weights],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode != 0
    assert "path does not exist" in result.stderr


def test_model_assets_pin_expected_sources_and_revisions() -> None:
    model_script = (REPO_ROOT / "scripts" / "model.sh").read_text(encoding="utf-8")

    assert "4340b03f4f59f46279a6581bbb818e0f77765d4d" in model_script
    assert "3d74acf9a28c67741b2f4f2ea7635f0aaf6f0268" in model_script
    assert "bad6c7334531becaf90a561988519b7bec34d0ab" in model_script
    assert "huggingface.co/wondervictor/YOLO-World/resolve/${model_yolo_world_revision}" in model_script
    assert "huggingface.co/openai/clip-vit-base-patch32/resolve/${model_clip_revision}" in model_script
    assert "examples/yolo_world/model" in model_script
    assert "bus.jpg coco_text_outp.npy\\n" in model_script


def test_model_assets_does_not_overwrite_non_empty_file() -> None:
    with tempfile.TemporaryDirectory() as temporary_directory:
        destination = Path(temporary_directory) / "existing.pth"
        destination.write_bytes(b"keep-this-checkpoint")
        result = subprocess.run(
            [
                "bash",
                "-c",
                'source scripts/model.sh; model_download_missing "$1" "$2"',
                "model-assets-test",
                "https://example.invalid/not-downloaded",
                str(destination),
            ],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
            check=False,
        )

        assert result.returncode == 0, result.stderr
        assert destination.read_bytes() == b"keep-this-checkpoint"


def write_float32_npy(path: Path, shape: tuple[int, int, int]) -> None:
    header = f"{{'descr': '<f4', 'fortran_order': False, 'shape': {shape}, }}".encode("ascii")
    header += b" " * ((16 - (10 + len(header) + 1) % 16) % 16) + b"\n"
    data_size = 4 * shape[0] * shape[1] * shape[2]
    path.write_bytes(b"\x93NUMPY\x01\x00" + struct.pack("<H", len(header)) + header + b"\x00" * data_size)


def run_calibration_check(calibration_dir: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [
            "bash",
            "-c",
            'source scripts/model.sh; model_require_calibration "$1"',
            "model-calibration-test",
            str(calibration_dir),
        ],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )


def test_model_calibration_rejects_missing_or_invalid_embedding() -> None:
    with tempfile.TemporaryDirectory() as temporary_directory:
        calibration_dir = Path(temporary_directory)
        (calibration_dir / "dataset.txt").write_text("bus.jpg coco_text_outp.npy\n", encoding="utf-8")
        (calibration_dir / "bus.jpg").write_bytes(b"image")

        missing = run_calibration_check(calibration_dir)
        assert missing.returncode != 0
        assert "text embedding is missing" in missing.stderr

        write_float32_npy(calibration_dir / "coco_text_outp.npy", (1, 79, 512))
        invalid = run_calibration_check(calibration_dir)
        assert invalid.returncode != 0
        assert "valid NumPy file with shape [1,80,512]" in invalid.stderr


def test_model_calibration_dataset_contract_is_exact() -> None:
    with tempfile.TemporaryDirectory() as temporary_directory:
        calibration_dir = Path(temporary_directory)
        (calibration_dir / "dataset.txt").write_text("bus.jpg coco_text_outp.npy\nextra\n", encoding="utf-8")
        (calibration_dir / "bus.jpg").write_bytes(b"image")
        write_float32_npy(calibration_dir / "coco_text_outp.npy", (1, 80, 512))

        result = run_calibration_check(calibration_dir)

        assert result.returncode != 0
        assert "dataset.txt must contain exactly: bus.jpg coco_text_outp.npy" in result.stderr


def test_model_builder_pins_rockchip_revisions_and_keeps_runtime_separate() -> None:
    dockerfile = (REPO_ROOT / "docker" / "model-builder" / "Dockerfile").read_text(encoding="utf-8")
    runtime_dockerfile = (REPO_ROOT / "docker" / "runtime" / "Dockerfile").read_text(encoding="utf-8")

    assert "b8b0fe9beffa9564306a798f6e443c9fe88057af" in dockerfile
    assert "deaba85fc437a28db0b0c29f27d8929f4c5816a1" in dockerfile
    assert "rknn_toolkit2-2.1.0+708089d1" in dockerfile
    assert "deploy/export_onnx.py" not in runtime_dockerfile


def test_model_deployment_documents_calibration_embedding_provenance() -> None:
    deployment_doc = (REPO_ROOT / "docs" / "perception" / "yolo-world-model-deployment.md").read_text(encoding="utf-8")

    assert "c2b7d00714b4e5d21266ab3003f3ca687ba0d57b" in deployment_doc
    assert "bad6c7334531becaf90a561988519b7bec34d0ab" in deployment_doc
    assert "examples/yolo_world/model/coco_text_outp.npy" in deployment_doc
