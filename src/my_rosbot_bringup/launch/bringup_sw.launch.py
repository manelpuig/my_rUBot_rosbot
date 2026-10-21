"""Launch the official ROSbot XL simulation with project defaults."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    simulation_launch = os.path.join(
        get_package_share_directory("rosbot_gazebo"),
        "launch",
        "simulation.yaml",
    )
    simulation = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(simulation_launch),
        launch_arguments={
            "robot_model": "rosbot_xl",
            "use_sim": "True",
            "configuration": LaunchConfiguration("configuration"),
            "arm_activate": LaunchConfiguration("arm_activate"),
            "rviz": LaunchConfiguration("rviz"),
            "gz_headless_mode": LaunchConfiguration("gz_headless_mode"),
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("configuration", default_value="basic"),
            DeclareLaunchArgument(
                "arm_activate", default_value="False", choices=["True", "False"]
            ),
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
