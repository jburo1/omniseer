import subprocess
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
    result = subprocess.run(
        [
            "scripts/omni",
            "model",
            "export",
            "--weights",
            "models/source/yolo_world/v2m/yolo_world_v2_m_obj365v1_goldg_pretrain-c6237d5b.pth",
        ],
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
