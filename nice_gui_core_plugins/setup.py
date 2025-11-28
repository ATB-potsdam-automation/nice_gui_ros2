import os
from glob import glob

from setuptools import find_packages, setup

package_name = "nice_gui_core_plugins"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.[yaml]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Daniel Suhr",
    maintainer_email="dsuhr@atb-potsdam.de",
    description="Core plugins for Nice GUI",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "nice_gui.cards": [
            "LinkCard = nice_gui_core_plugins.cards.link_card:LinkCard",
            "ControlCard = nice_gui_core_plugins.cards.control_card:ControlCard",
            "BatteryStateCard = nice_gui_core_plugins.cards.battery_state_card:BatteryStateCard",
            "DiagnosticCard = nice_gui_core_plugins.cards.diagnostic_card:DiagnosticCard",
            "GPSInfoCard = nice_gui_core_plugins.cards.gps_info_card:GPSInfoCard",
            "GridVisualizationCard = nice_gui_core_plugins.cards.grid_visualization_card:GridVisualizationCard",
            "ImageCard = nice_gui_core_plugins.cards.image_card:ImageCard",
            "OdometryCard = nice_gui_core_plugins.cards.odometry_card:OdometryCard",
            "SceneCard = nice_gui_core_plugins.cards.scene_card:SceneCard",
        ],
    },
    include_package_data=True,
    package_data={},
)
