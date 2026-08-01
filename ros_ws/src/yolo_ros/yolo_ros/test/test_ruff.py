# test/test_ruff.py
import pathlib
import shutil
import subprocess

import pytest


def test_ruff_clean():
    if shutil.which("ruff") is None:
        pytest.skip("Ruff is validated by the dedicated lint lane")

    pkg = pathlib.Path(__file__).resolve().parents[1]
    res = subprocess.run(["ruff", "check", str(pkg)], capture_output=True, text=True)
    assert res.returncode == 0, f"Ruff findings:\n{res.stdout}\n{res.stderr}"
