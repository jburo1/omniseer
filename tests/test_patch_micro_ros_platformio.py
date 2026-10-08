import importlib.util
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
PATCH_PATH = REPO_ROOT / "firmware/scripts/patch_micro_ros_platformio.py"


def _patch_module():
    spec = importlib.util.spec_from_file_location("patch_micro_ros_platformio", PATCH_PATH)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_micro_ros_patch_disables_dependency_installation_idempotently(tmp_path: Path) -> None:
    patch = _patch_module()
    package_dir = tmp_path / ".pio/libdeps/teensy41/micro_ros_platformio"
    library_builder = package_dir / "microros_utils/library_builder.py"
    extra_script = package_dir / "extra_script.py"
    library_builder.parent.mkdir(parents=True)
    library_builder.write_text(patch.ORIGINAL, encoding="utf-8")
    extra_script.write_text(patch.DEPENDENCY_ORIGINAL, encoding="utf-8")

    assert patch.patch_library_builder(tmp_path, "teensy41")
    assert patch.patch_dependency_installer(tmp_path, "teensy41")
    assert "rmw_test_fixture/COLCON_IGNORE" in library_builder.read_text(encoding="utf-8")
    patched_extra_script = extra_script.read_text(encoding="utf-8")
    assert "Missing micro-ROS Python dependencies" in patched_extra_script
    assert "env.Execute('$PYTHONEXE -m pip install" not in patched_extra_script

    assert patch.patch_library_builder(tmp_path, "teensy41")
    assert patch.patch_dependency_installer(tmp_path, "teensy41")
