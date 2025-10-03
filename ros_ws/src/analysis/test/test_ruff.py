# test/test_ruff.py
import pathlib
import subprocess


def test_ruff_clean():
    pkg = pathlib.Path(__file__).resolve().parents[1]
    res = subprocess.run(["ruff", "check", str(pkg)], capture_output=True, text=True)
    assert res.returncode == 0, f"Ruff findings:\n{res.stdout}\n{res.stderr}"
