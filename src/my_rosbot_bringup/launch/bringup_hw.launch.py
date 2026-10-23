"""Launch the official ROSbot XL hardware stack with project defaults."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    hardware_launch = os.path.join(
        get_package_share_directory("rosbot_bringup"), "launch", "rosbot_xl.yaml"
    )
    hardware = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(hardware_launch),
        launch_arguments={
            "configuration": LaunchConfiguration("configuration"),
            "arm_activate": "False",
            "mecanum": "True",
            "namespace": LaunchConfiguration("namespace"),
            "tf_namespace_bridge": "True",
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("configuration", default_value="basic"),
            DeclareLaunchArgument("namespace", default_value=""),
            hardware,
        ]
    )
