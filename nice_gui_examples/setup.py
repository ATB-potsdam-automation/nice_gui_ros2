import os
from glob import glob

from setuptools import find_packages, setup

package_name = "nice_gui_examples"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.[yaml]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Daniel Suhr",
    maintainer_email="dsuhr@atb-potsdam.de",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    include_package_data=True,
)
