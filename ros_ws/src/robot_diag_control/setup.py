from glob import glob

from setuptools import find_packages, setup

package_name = "robot_diag_control"
share_dir = f"share/{package_name}"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (share_dir, ["package.xml"]),
        (f"{share_dir}/proto", glob("robot_diag_control/api/*.proto")),
    ],
    install_requires=["setuptools", "grpcio>=1.71.2,<2", "protobuf>=5.29.0,<6"],
    zip_safe=True,
    maintainer="selfsim",
    maintainer_email="jonas.buro1@gmail.com",
    description="Python client tools and shared protobuf contract for the operator gateway.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "robot_gateway_cli = robot_diag_control.cli:main",
            "robot_monitor_gui = robot_diag_control.monitor_gui:main",
            "robot_monitor_shell = robot_diag_control.monitor_shell:main",
            "robot_preview_viewer = robot_diag_control.preview_viewer:main",
        ],
    },
)
