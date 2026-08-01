"""Serve local RViz resources through the Kilted RViz resource service."""

from __future__ import annotations

from array import array
from pathlib import Path
from urllib.parse import unquote, urlparse

import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rclpy.node import Node
from rviz_resource_interfaces.srv import GetResource


def _resolve_resource_path(resource_path: str) -> Path:
    parsed = urlparse(resource_path)

    if parsed.scheme == "package":
        if not parsed.netloc:
            raise ValueError(f"package URI missing package name: {resource_path}")
        relative_path = unquote(parsed.path).lstrip("/")
        return Path(get_package_share_directory(parsed.netloc)) / relative_path

    if parsed.scheme == "file":
        return Path(unquote(parsed.path))

    if not parsed.scheme:
        return Path(resource_path)

    raise ValueError(f"unsupported RViz resource URI scheme: {parsed.scheme}")


def _etag_for(path: Path) -> str:
    stat = path.stat()
    return f"{stat.st_mtime_ns:x}-{stat.st_size:x}"


class RvizResourceServer(Node):
    def __init__(self) -> None:
        super().__init__("rviz_resource_server")
        self.create_service(GetResource, "/rviz/get_resource", self._handle_get_resource)

    def _handle_get_resource(
        self,
        request: GetResource.Request,
        response: GetResource.Response,
    ) -> GetResource.Response:
        try:
            path = _resolve_resource_path(request.path)
            current_etag = _etag_for(path)
            response.expanded_path = path.as_uri()
            response.etag = current_etag

            if request.etag and request.etag == current_etag:
                response.status_code = GetResource.Response.NOT_MODIFIED
                return response

            response.body = array("B", path.read_bytes())
            response.status_code = GetResource.Response.OK
            return response
        except (FileNotFoundError, IsADirectoryError, OSError, PackageNotFoundError, ValueError) as exc:
            response.status_code = GetResource.Response.ERROR
            response.error_reason = str(exc)
            return response


def main() -> None:
    rclpy.init()
    node = RvizResourceServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
