import importlib.util
import os
import pty
import sys
import tempfile
import unittest
from pathlib import Path


def _load_wait_for_teensy_module():
    module_path = Path(__file__).resolve().parents[1] / "bringup" / "wait_for_teensy.py"
    spec = importlib.util.spec_from_file_location("bringup_wait_for_teensy", module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class WaitForTeensyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls._module = _load_wait_for_teensy_module()

    def test_wait_for_device_accepts_pty_backed_serial_path(self) -> None:
        master_fd, slave_fd = pty.openpty()
        self.addCleanup(os.close, master_fd)
        self.addCleanup(os.close, slave_fd)

        device_path = os.ttyname(slave_fd)
        result = self._module.wait_for_device(device_path, timeout_sec=0.1)

        self.assertTrue(result.ready, result.reason)
        self.assertIn(device_path, result.reason)

    def test_wait_for_device_times_out_when_path_never_appears(self) -> None:
        missing_path = Path(tempfile.gettempdir()) / "omniseer-teensy-missing"
        if missing_path.exists():
            missing_path.unlink()

        result = self._module.wait_for_device(str(missing_path), timeout_sec=0.0)

        self.assertFalse(result.ready)
        self.assertIn("Timed out", result.reason)
        self.assertIn(str(missing_path), result.reason)

    def test_wait_for_device_rejects_non_serial_regular_file(self) -> None:
        with tempfile.NamedTemporaryFile() as handle:
            result = self._module.wait_for_device(handle.name, timeout_sec=0.0)

        self.assertFalse(result.ready)
        self.assertIn("not a character device", result.reason)
