import pathlib
import shutil
import subprocess
import unittest


class TestRuff(unittest.TestCase):
    @unittest.skipUnless(shutil.which("ruff"), "Ruff is validated by the dedicated lint lane")
    def test_ruff_clean(self) -> None:
        pkg = pathlib.Path(__file__).resolve().parents[1]
        res = subprocess.run(["ruff", "check", str(pkg)], capture_output=True, text=True)
        self.assertEqual(res.returncode, 0, f"Ruff findings:\n{res.stdout}\n{res.stderr}")
