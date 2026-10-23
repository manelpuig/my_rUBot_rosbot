"""Launch ROSbot XL hardware behind the common project interface."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    project_launch_dir = os.path.join(
        get_package_share_directory("my_rosbot_bringup"), "launch"
    )
    official_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(project_launch_dir, "bringup_hw.launch.py")
        ),
        launch_arguments={
            "configuration": LaunchConfiguration("configuration"),
            "namespace": "rosbot",
        }.items(),
    )
    adapter = Node(
        package="my_rosbot_control",
        executable="interface_adapter",
        name="rosbot_interface_adapter",
        output="screen",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("configuration", default_value="basic"),
            official_stack,
            adapter,
        ]
    )
