import os
from glob import glob

from setuptools import find_packages, setup

package_name = "nice_gui_core"

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
        (
            os.path.join("share", package_name, "config/cards"),
            glob(os.path.join("config/cards", "*.[yaml]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ros",
    maintainer_email="tschuette@atb-potsdam.de",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ui = nice_gui_core.ui:main",
        ],
    },
    include_package_data=True,
    package_data={
        "nice_gui_core": ["static/*"],
    },
)
