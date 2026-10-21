"""Launch the official ROSbot XL autonomy sensor configuration."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    project_launch = os.path.join(
        get_package_share_directory("my_rosbot_bringup"),
        "launch",
        "bringup_sw.launch.py",
    )
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(project_launch),
        launch_arguments={
            "configuration": "autonomy",
            "arm_activate": "False",
            "rviz": LaunchConfiguration("rviz"),
            "gz_headless_mode": LaunchConfiguration("gz_headless_mode"),
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz", default_value="True", choices=["True", "False"]
            ),
            DeclareLaunchArgument(
                "gz_headless_mode",
                default_value="False",
                choices=["True", "False"],
            ),
            simulation,
        ]
    )
