import os
import shutil
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]


def _write_executable(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")
    path.chmod(0o755)


def _stage_flash_script(tmp_path: Path) -> tuple[Path, Path]:
    firmware_dir = tmp_path / "firmware"
    scripts_dir = tmp_path / "scripts"
    (firmware_dir / "scripts").mkdir(parents=True)
    (scripts_dir / "lib").mkdir(parents=True)
    shutil.copy(
        REPO_ROOT / "firmware/scripts/flash_teensy_headless.sh",
        firmware_dir / "scripts/flash_teensy_headless.sh",
    )
    (firmware_dir / "scripts/patch_micro_ros_platformio.py").write_text("raise SystemExit(0)\n", encoding="utf-8")
    shutil.copy(REPO_ROOT / "scripts/lib/common.sh", scripts_dir / "lib/common.sh")
    return firmware_dir / "scripts/flash_teensy_headless.sh", firmware_dir


def _flash_env(tmp_path: Path, firmware_dir: Path) -> dict[str, str]:
    bin_dir = tmp_path / "bin"
    home_dir = tmp_path / "home"
    command_log = tmp_path / "commands.log"
    pio = bin_dir / "platformio"
    python = bin_dir / "platformio-python"
    loader = home_dir / ".platformio/packages/tool-teensy/teensy_loader_cli"
    loader_attempts = tmp_path / "loader-attempts"

    _write_executable(
        pio,
        "#!/usr/bin/env bash\n"
        'printf \'pio %s\\n\' "$*" >>"${COMMAND_LOG}"\n'
        '[[ "${FLASH_BUILD_FAIL:-0}" != 1 ]] || exit 1\n'
        'mkdir -p "${FAKE_FIRMWARE_DIR}/.pio/build/teensy41"\n'
        'printf fresh >"${FAKE_FIRMWARE_DIR}/.pio/build/teensy41/firmware.hex"\n',
    )
    _write_executable(
        python,
        "#!/usr/bin/env bash\n"
        'printf \'platformio-python %s\\n\' "$*" >>"${COMMAND_LOG}"\n'
        '[[ "${FLASH_MISSING_PYTHON_DEPS:-0}" != 1 ]]\n',
    )
    _write_executable(
        bin_dir / "python3",
        '#!/usr/bin/env bash\nprintf \'python3 %s\\n\' "$*" >>"${COMMAND_LOG}"\n',
    )
    _write_executable(
        bin_dir / "timeout",
        '#!/usr/bin/env bash\nprintf \'timeout %s\\n\' "$*" >>"${COMMAND_LOG}"\nshift 3\nexec "$@"\n',
    )
    _write_executable(
        loader,
        "#!/usr/bin/env bash\n"
        'printf \'loader %s\\n\' "$*" >>"${COMMAND_LOG}"\n'
        'attempts=$(cat "${LOADER_ATTEMPTS}" 2>/dev/null || printf 0)\n'
        "attempts=$((attempts + 1))\n"
        'printf %s "${attempts}" >"${LOADER_ATTEMPTS}"\n'
        'if [[ "${attempts}" == 1 && -n "${FLASH_LOADER_FIRST_EXIT:-}" ]]; then\n'
        '  exit "${FLASH_LOADER_FIRST_EXIT}"\n'
        "fi\n"
        'exit "${FLASH_LOADER_EXIT:-0}"\n',
    )
    _write_executable(
        home_dir / ".platformio/packages/tool-teensy/teensy_reboot",
        '#!/usr/bin/env bash\nprintf \'reboot %s\\n\' "$*" >>"${COMMAND_LOG}"\n',
    )

    env = os.environ.copy()
    env.update(
        {
            "PATH": f"{bin_dir}:{env['PATH']}",
            "HOME": str(home_dir),
            "PIO_BIN": str(pio),
            "PLATFORMIO_PYTHON": str(python),
            "COMMAND_LOG": str(command_log),
            "FAKE_FIRMWARE_DIR": str(firmware_dir),
            "LOADER_ATTEMPTS": str(loader_attempts),
        }
    )
    return env


def _run_flash(script: Path, env: dict[str, str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [str(script)],
        cwd=script.parents[2],
        env=env,
        capture_output=True,
        text=True,
        check=False,
    )


def _command_log(env: dict[str, str]) -> str:
    path = Path(env["COMMAND_LOG"])
    return path.read_text(encoding="utf-8") if path.exists() else ""


def test_flash_compiles_then_uses_only_direct_bounded_loader(tmp_path: Path) -> None:
    script, firmware_dir = _stage_flash_script(tmp_path)
    env = _flash_env(tmp_path, firmware_dir)

    result = _run_flash(script, env)

    assert result.returncode == 0, result.stderr
    commands = _command_log(env)
    assert "pip install" not in commands
    assert "pkg install" not in commands
    assert "pio run -d" in commands
    assert "-t upload" not in commands
    assert "timeout --foreground --kill-after=5s 30s" in commands
    assert "loader -mmcu=imxrt1062 -w -v -s" in commands
    assert "reboot " not in commands
    assert (firmware_dir / ".pio/build/teensy41/firmware.hex").read_text(encoding="utf-8") == "fresh"


def test_flash_missing_python_prerequisites_prevents_build_and_upload(tmp_path: Path) -> None:
    script, firmware_dir = _stage_flash_script(tmp_path)
    env = _flash_env(tmp_path, firmware_dir)
    env["FLASH_MISSING_PYTHON_DEPS"] = "1"

    result = _run_flash(script, env)

    assert result.returncode == 2
    assert "micro-ROS Python dependencies are missing" in result.stderr
    commands = _command_log(env)
    assert "pio " not in commands
    assert "loader " not in commands


def test_flash_build_failure_removes_stale_hex_and_never_uploads(tmp_path: Path) -> None:
    script, firmware_dir = _stage_flash_script(tmp_path)
    env = _flash_env(tmp_path, firmware_dir)
    stale_hex = firmware_dir / ".pio/build/teensy41/firmware.hex"
    stale_hex.parent.mkdir(parents=True)
    stale_hex.write_text("stale", encoding="utf-8")
    env["FLASH_BUILD_FAIL"] = "1"

    result = _run_flash(script, env)

    assert result.returncode == 1
    assert not stale_hex.exists()
    commands = _command_log(env)
    assert "pio run -d" in commands
    assert "loader " not in commands


def test_flash_reports_a_bounded_bootloader_wait_timeout(tmp_path: Path) -> None:
    script, firmware_dir = _stage_flash_script(tmp_path)
    env = _flash_env(tmp_path, firmware_dir)
    env["FLASH_LOADER_EXIT"] = "124"

    result = _run_flash(script, env)

    assert result.returncode == 124
    assert "Timed out waiting for the Teensy bootloader after 30s" in result.stderr


def test_flash_retries_a_failed_direct_upload_without_rebooting_again(tmp_path: Path) -> None:
    script, firmware_dir = _stage_flash_script(tmp_path)
    env = _flash_env(tmp_path, firmware_dir)
    env["FLASH_LOADER_FIRST_EXIT"] = "1"

    result = _run_flash(script, env)

    assert result.returncode == 0, result.stderr
    assert "retrying once without another reboot" in result.stderr
    loader_calls = [line for line in _command_log(env).splitlines() if line.startswith("loader ")]
    assert len(loader_calls) == 2
    assert loader_calls[0].endswith(" -w -v -s " + str(firmware_dir / ".pio/build/teensy41/firmware.hex"))
    assert loader_calls[1].endswith(" -w -v " + str(firmware_dir / ".pio/build/teensy41/firmware.hex"))
