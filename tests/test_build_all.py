import os
import shutil
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]


def _write_executable(path: Path, content: str) -> None:
    path.write_text(content, encoding="utf-8")
    path.chmod(0o755)


def _stage_build_scripts(tmp_path: Path) -> Path:
    scripts_root = tmp_path / "scripts"
    build_root = scripts_root / "build"
    lib_root = scripts_root / "lib"
    build_root.mkdir(parents=True)
    lib_root.mkdir(parents=True)
    shutil.copy(REPO_ROOT / "scripts/build/all.sh", build_root / "all.sh")
    shutil.copy(REPO_ROOT / "scripts/lib/common.sh", lib_root / "common.sh")
    shutil.copy(REPO_ROOT / "scripts/lib/log.sh", lib_root / "log.sh")
    for name in ("ros", "vision", "firmware"):
        _write_executable(
            build_root / f"{name}.sh",
            "\n".join(
                [
                    "#!/usr/bin/env bash",
                    "set -euo pipefail",
                    f"printf '{name}\\n' >>\"${{BUILD_LOG}}\"",
                ]
            )
            + "\n",
        )
    return build_root / "all.sh"


def _build_env(tmp_path: Path) -> dict[str, str]:
    bin_root = tmp_path / "bin"
    bin_root.mkdir()
    _write_executable(bin_root / "cmake", "#!/usr/bin/env bash\nexit 0\n")
    _write_executable(bin_root / "platformio", "#!/usr/bin/env bash\nexit 0\n")
    env = os.environ.copy()
    env["PATH"] = f"{bin_root}:{env['PATH']}"
    env["BUILD_LOG"] = str(tmp_path / "build.log")
    env["OMNISEER_TEENSY_DEVICE_GLOBS"] = str(tmp_path / "missing-teensy-*")
    return env


def _run_build_all(tmp_path: Path, env: dict[str, str], *args: str) -> subprocess.CompletedProcess[str]:
    script = _stage_build_scripts(tmp_path)
    return subprocess.run(
        [str(script), *args],
        cwd=tmp_path,
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )


def _build_log(env: dict[str, str]) -> str:
    path = Path(env["BUILD_LOG"])
    return path.read_text(encoding="utf-8") if path.exists() else ""


def test_build_all_skips_firmware_when_platformio_exists_but_no_teensy_is_attached(tmp_path: Path) -> None:
    env = _build_env(tmp_path)

    result = _run_build_all(tmp_path, env)

    assert result.returncode == 0, result.stderr
    assert _build_log(env) == "ros\nvision\n"
    assert "no Teensy detected; skipping firmware in build all" in result.stderr


def test_build_all_builds_firmware_when_teensy_is_attached(tmp_path: Path) -> None:
    env = _build_env(tmp_path)
    teensy = tmp_path / "usb-Teensyduino_USB_Serial_fake-if00"
    teensy.touch()
    env["OMNISEER_TEENSY_DEVICE_GLOBS"] = str(teensy)

    result = _run_build_all(tmp_path, env)

    assert result.returncode == 0, result.stderr
    assert _build_log(env) == "ros\nvision\nfirmware\n"


def test_build_all_strict_builds_firmware_without_attached_teensy(tmp_path: Path) -> None:
    env = _build_env(tmp_path)

    result = _run_build_all(tmp_path, env, "--strict")

    assert result.returncode == 0, result.stderr
    assert _build_log(env) == "ros\nvision\nfirmware\n"
