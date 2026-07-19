from setuptools import find_packages, setup

package_name = "omniseer_experiments"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="selfsim",
    maintainer_email="jonas.buro1@gmail.com",
    description="Experiment recording and run-bundle tooling for Omniseer perception runs.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "record_run = omniseer_experiments.record_run:main",
        ],
    },
)
