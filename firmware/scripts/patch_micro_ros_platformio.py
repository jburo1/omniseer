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

DEPENDENCY_ORIGINAL = (
    '    pip_packages = [x.split("==")[0] for x in os.popen('
    "'{} -m pip freeze'.format(env['PYTHONEXE'])).read().split('\\n')]\n"
    '    required_packages = ["catkin-pkg", "lark-parser", "colcon-common-extensions", '
    '"importlib-resources", "pyyaml", "pytz", "markupsafe==2.0.1", "empy==3.3.4"]\n'
    "    if all([x in pip_packages for x in required_packages]):\n"
    '        print("All required Python pip packages are installed")\n'
    "\n"
    "    for p in [x for x in required_packages if x not in pip_packages]:\n"
    "        print('Installing {} with pip at PlatformIO environment'.format(p))\n"
    "        env.Execute('$PYTHONEXE -m pip install {}'.format(p))\n"
)

DEPENDENCY_REPLACEMENT = """    required_packages = {
        \"catkin-pkg\": None,
        \"lark-parser\": None,
        \"colcon-common-extensions\": None,
        \"importlib-resources\": None,
        \"pyyaml\": None,
        \"pytz\": None,
        \"markupsafe\": \"2.0.1\",
        \"empy\": \"3.3.4\",
    }
    installed_packages = {}
    for package in os.popen('{} -m pip freeze'.format(env['PYTHONEXE'])).read().splitlines():
        if \"==\" in package:
            name, version = package.split(\"==\", 1)
            installed_packages[name.lower().replace(\"_\", \"-\")] = version
    missing_packages = [
        name if version is None else \"{}=={}\".format(name, version)
        for name, version in required_packages.items()
        if name not in installed_packages or (version is not None and installed_packages[name] != version)
    ]
    if missing_packages:
        message = (
            \"Missing micro-ROS Python dependencies: {}. Install them before building with: {} -m pip install {}\"
        )
        raise RuntimeError(
            message.format(
                \" \".join(missing_packages), env['PYTHONEXE'], \" \".join(missing_packages)
            )
        )
    print(\"All required Python pip packages are installed\")
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


def patch_dependency_installer(project_dir: Path, pioenv: str) -> bool:
    target = project_dir / ".pio" / "libdeps" / pioenv / "micro_ros_platformio" / "extra_script.py"
    if not target.exists():
        print(f"patch skipped: {target} does not exist yet", file=sys.stderr)
        return False

    content = target.read_text()
    if "Missing micro-ROS Python dependencies" in content:
        return True

    if DEPENDENCY_ORIGINAL not in content:
        raise RuntimeError(f"expected dependency patch anchor not found in {target}")

    target.write_text(content.replace(DEPENDENCY_ORIGINAL, DEPENDENCY_REPLACEMENT))
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
    patch_dependency_installer(project_dir, args.pioenv)
    touch_existing_ignore_files(project_dir, args.pioenv)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
