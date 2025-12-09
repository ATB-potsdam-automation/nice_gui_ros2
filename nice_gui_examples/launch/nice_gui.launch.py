import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():

    namespace = LaunchConfiguration("namespace")

    use_sim_time_env = os.getenv("USE_SIM_TIME")

    config_file = LaunchConfiguration("config_file")

    config_directory = os.path.join(
        get_package_share_directory("nice_gui_examples"), "config"
    )
    config_file_launch_arg = DeclareLaunchArgument(
        "config_file", default_value=os.path.join(config_directory, "default.yaml")
    )

    namespace_launch_arg = DeclareLaunchArgument("namespace", default_value="")

    node = Node(
        package="nice_gui_core",
        namespace=namespace,
        executable="ui",
        name="ui",
        parameters=[config_file],
    )

    return LaunchDescription(
        [
            SetParameter("use_sim_time", use_sim_time_env),
            config_file_launch_arg,
            namespace_launch_arg,
            node,
        ]
    )
