import random
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
    assert "model calibration [--images-dir <dir>] [--classes <file>] [--clip-model <dir>]" in result.stdout
    assert "model rknn-debug" not in result.stdout
    assert "model analyze [--variant v2s|v2m|v2l] [--onnx <model.onnx>]" in result.stdout
    assert "model export [--variant v2s|v2m|v2l] --weights" in result.stdout
    assert "model compile [--variant v2s|v2m|v2l] --onnx" in result.stdout
    assert "model build [--variant v2s|v2m|v2l] --weights" in result.stdout


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


def test_model_variant_v2l_resolves_expected_checkpoint_config_and_artifacts() -> None:
    result = resolve_variant("v2l")

    assert result.returncode == 0, result.stderr
    assert result.stdout == (
        "v2l|yolo_world_v2_l_obj365v1_goldg_pretrain-a82b1fe3.pth|"
        "configs/pretrain/yolo_world_v2_l_vlpan_bn_2e-3_100e_4x8gpus_obj365v1_goldg_train_lvis_minival.py|"
        "yolo_world_v2_l.onnx|yolo_world_v2_l_fp.rknn|yolo_world_v2_l_i8.rknn\n"
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
    assert "unsupported YOLO-World model variant: v2x; expected v2s, v2m, or v2l" in result.stderr


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
    assert "huggingface.co/wondervictor/YOLO-World/resolve/${model_yolo_world_revision}" in model_script
    assert "huggingface.co/openai/clip-vit-base-patch32/resolve/${model_clip_revision}" in model_script
    assert "calibration_text_outp.npy" in model_script
    assert "coco_text_outp.npy" not in model_script


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
        (calibration_dir / "dataset.txt").write_text("frame_001.jpg calibration_text_outp.npy\n", encoding="utf-8")
        (calibration_dir / "frame_001.jpg").write_bytes(b"image")

        missing = run_calibration_check(calibration_dir)
        assert missing.returncode != 0
        assert "text embedding is missing" in missing.stderr

        write_float32_npy(calibration_dir / "calibration_text_outp.npy", (1, 79, 512))
        invalid = run_calibration_check(calibration_dir)
        assert invalid.returncode != 0
        assert "valid NumPy file with shape [1,80,512]" in invalid.stderr


def test_model_calibration_accepts_multiple_images_paired_with_the_calibration_embedding() -> None:
    with tempfile.TemporaryDirectory() as temporary_directory:
        temporary_path = Path(temporary_directory)
        calibration_dir = temporary_path / "calibration"
        calibration_dir.mkdir()
        (calibration_dir / "dataset.txt").write_text(
            "../robot/frame_001.jpg calibration_text_outp.npy\n", encoding="utf-8"
        )
        robot_images = temporary_path / "robot"
        robot_images.mkdir()
        (robot_images / "frame_001.jpg").write_bytes(b"image")
        write_float32_npy(calibration_dir / "calibration_text_outp.npy", (1, 80, 512))

        result = run_calibration_check(calibration_dir)

        assert result.returncode == 0, result.stderr


def test_model_calibration_rejects_an_entry_with_a_different_text_embedding() -> None:
    with tempfile.TemporaryDirectory() as temporary_directory:
        calibration_dir = Path(temporary_directory)
        (calibration_dir / "dataset.txt").write_text("frame_001.jpg another_text_embedding.npy\n", encoding="utf-8")
        (calibration_dir / "frame_001.jpg").write_bytes(b"image")
        write_float32_npy(calibration_dir / "calibration_text_outp.npy", (1, 80, 512))

        result = run_calibration_check(calibration_dir)

        assert result.returncode != 0
        assert "must pair every image with calibration_text_outp.npy" in result.stderr


def test_model_calibration_generates_a_dataset_from_images_dir() -> None:
    with tempfile.TemporaryDirectory(dir=REPO_ROOT) as temporary_directory:
        temporary_path = Path(temporary_directory)
        calibration_dir = temporary_path / "calibration"
        images_dir = temporary_path / "images"
        calibration_dir.mkdir()
        images_dir.mkdir()
        write_float32_npy(calibration_dir / "calibration_text_outp.npy", (1, 80, 512))
        (images_dir / "frame_010.jpg").write_bytes(b"image")
        (images_dir / "frame_002.png").write_bytes(b"image")
        (images_dir / "ignore.txt").write_text("not an image", encoding="utf-8")

        result = subprocess.run(
            [
                "bash",
                "-c",
                'source scripts/model.sh; model_write_calibration_dataset "$1" "$2"',
                "model-calibration-dataset-test",
                str(images_dir),
                str(calibration_dir),
            ],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
            check=False,
        )

        assert result.returncode == 0, result.stderr
        assert (calibration_dir / "dataset.txt").read_text(encoding="utf-8") == (
            "../images/frame_002.png calibration_text_outp.npy\n../images/frame_010.jpg calibration_text_outp.npy\n"
        )


def read_class_list(path: Path) -> list[str]:
    return [
        line.strip()
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.startswith("#")
    ]


def test_calibration_class_list_contains_task_classes_and_reproducible_coco_fillers() -> None:
    task_classes = read_class_list(REPO_ROOT / "config" / "classes" / "task.txt")
    coco_classes = read_class_list(REPO_ROOT / "config" / "classes" / "coco80.txt")
    calibration_classes = read_class_list(REPO_ROOT / "config" / "classes" / "calibration.txt")
    task_keys = {label.casefold() for label in task_classes}
    candidates = [label for label in coco_classes if label.casefold() not in task_keys]
    expected_fillers = random.Random(0).sample(candidates, 80 - len(task_classes))

    assert len(calibration_classes) == 80
    assert len({label.casefold() for label in calibration_classes}) == 80
    assert calibration_classes[: len(task_classes)] == task_classes
    assert calibration_classes[len(task_classes) :] == expected_fillers


def test_model_builder_pins_rockchip_revisions_and_keeps_runtime_separate() -> None:
    dockerfile = (REPO_ROOT / "docker" / "model-builder" / "Dockerfile").read_text(encoding="utf-8")
    runtime_dockerfile = (REPO_ROOT / "docker" / "runtime" / "Dockerfile").read_text(encoding="utf-8")

    assert "b8b0fe9beffa9564306a798f6e443c9fe88057af" in dockerfile
    assert "deaba85fc437a28db0b0c29f27d8929f4c5816a1" in dockerfile
    assert "rknn_toolkit2-2.1.0+708089d1" in dockerfile
    assert "deploy/export_onnx.py" not in runtime_dockerfile


def test_model_analysis_supports_all_variants_without_board_dependencies() -> None:
    model_script = (REPO_ROOT / "scripts" / "model.sh").read_text(encoding="utf-8")
    dockerfile = (REPO_ROOT / "docker" / "model-builder" / "Dockerfile").read_text(encoding="utf-8")
    analysis_script = (REPO_ROOT / "tools" / "model" / "analyze_yolo_world_rknn.py").read_text(encoding="utf-8")

    assert "apt-get install --no-install-recommends -y adb" not in dockerfile
    assert "rknn-debug" not in model_script
    assert "--network host" not in model_script
    assert "analyze_yolo_world_rknn.py" in model_script
    assert "--variant v2s|v2m|v2l" in model_script
    assert "artifacts/models/${model_artifact_stem}.onnx" in model_script
    assert "artifacts/quant_analysis/${model_variant}" in model_script
    assert 'choices=("v2s", "v2m", "v2l")' in analysis_script
    assert 'target_platform="rk3588"' in analysis_script
    assert "mean_values=[[0, 0, 0]]" in analysis_script
    assert "std_values=[[255, 255, 255]]" in analysis_script
    assert 'inputs=["images", "texts"]' in analysis_script
    assert "input_size_list=[IMAGE_INPUT_SHAPE, TEXT_INPUT_SHAPE]" in analysis_script
    assert "do_quantization=True" in analysis_script
    assert 'target="rk3588"' not in analysis_script
    assert 'RUNTIME_CLASSES = ["person", "bus"]' in analysis_script
    assert 'RUNTIME_PAD_TOKEN = "nothing"' in analysis_script
    assert "generate_padded_embedding" in analysis_script


def test_model_deployment_documents_calibration_embedding_provenance() -> None:
    deployment_doc = (REPO_ROOT / "docs" / "perception" / "yolo-world-model-deployment.md").read_text(encoding="utf-8")

    assert "config/classes/calibration.txt" in deployment_doc
    assert "calibration_text_outp.npy" in deployment_doc
