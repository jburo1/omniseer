from setuptools import find_packages, setup

package_name = "analysis"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="selfsim",
    maintainer_email="jonas.buro1@gmail.com",
    description="Analysis + misc. tooling",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "path_recorder = analysis.path_recorder:main",
            "scan_to_range = analysis.scan_to_range:main",
        ],
    },
)
