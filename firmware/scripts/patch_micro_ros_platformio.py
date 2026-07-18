#!/usr/bin/env python3
"""Patch micro_ros_platformio for local kilted/rolling build compatibility."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ORIGINAL = """        # Fix build: Ignore rmw_test_fixture_implementation in rolling
        touch_command = ''
        if self.distro in ('rolling', 'kilted'):
            touch_command = 'touch src/ament_cmake_ros/rmw_test_fixture_implementation/COLCON_IGNORE && '
"""

REPLACEMENT = """        # Ignore dev-only RMW test fixtures that pull in host ROS packages the
        # PlatformIO helper environment does not fully provide.
        touch_command = ''
        if self.distro in ('rolling', 'kilted'):
            touch_command = (
                'touch src/ament_cmake_ros/rmw_test_fixture/COLCON_IGNORE && '
                'touch src/ament_cmake_ros/rmw_test_fixture_implementation/COLCON_IGNORE && '
            )
"""


def patch_library_builder(project_dir: Path, pioenv: str) -> bool:
    target = (
        project_dir / ".pio" / "libdeps" / pioenv / "micro_ros_platformio" / "microros_utils" / "library_builder.py"
    )
    if not target.exists():
        print(f"patch skipped: {target} does not exist yet", file=sys.stderr)
        return False

    content = target.read_text()
    if "rmw_test_fixture/COLCON_IGNORE" in content:
        return True

    if ORIGINAL not in content:
        raise RuntimeError(f"expected patch anchor not found in {target}")

    target.write_text(content.replace(ORIGINAL, REPLACEMENT))
    return True


def touch_existing_ignore_files(project_dir: Path, pioenv: str) -> None:
    base = (
        project_dir / ".pio" / "libdeps" / pioenv / "micro_ros_platformio" / "build" / "dev" / "src" / "ament_cmake_ros"
    )
    for relative in ("rmw_test_fixture", "rmw_test_fixture_implementation"):
        ignore_path = base / relative / "COLCON_IGNORE"
        if ignore_path.parent.exists():
            ignore_path.touch()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--project-dir", required=True)
    parser.add_argument("--pioenv", default="teensy41")
    args = parser.parse_args()

    project_dir = Path(args.project_dir).resolve()
    patch_library_builder(project_dir, args.pioenv)
    touch_existing_ignore_files(project_dir, args.pioenv)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
