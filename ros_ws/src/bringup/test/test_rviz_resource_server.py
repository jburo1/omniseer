import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


def _load_rviz_resource_server_module():
    module_path = Path(__file__).resolve().parents[1] / "bringup" / "rviz_resource_server.py"
    spec = importlib.util.spec_from_file_location("bringup_rviz_resource_server", module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class RvizResourceServerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls._module = _load_rviz_resource_server_module()

    def test_resolve_package_uri_uses_ament_share_directory(self) -> None:
        with tempfile.TemporaryDirectory() as share_dir:
            with mock.patch.object(self._module, "get_package_share_directory", return_value=share_dir):
                resolved_path = self._module._resolve_resource_path("package://omniseer_description/meshes/chassis.stl")

        self.assertEqual(resolved_path, Path(share_dir) / "meshes" / "chassis.stl")

    def test_resolve_file_uri_decodes_local_path(self) -> None:
        resolved_path = self._module._resolve_resource_path("file:///tmp/rviz%20mesh.stl")

        self.assertEqual(resolved_path, Path("/tmp/rviz mesh.stl"))

    def test_unsupported_uri_scheme_fails_loudly(self) -> None:
        with self.assertRaisesRegex(ValueError, "unsupported RViz resource URI scheme"):
            self._module._resolve_resource_path("model://omniseer/chassis.stl")


if __name__ == "__main__":
    unittest.main()
