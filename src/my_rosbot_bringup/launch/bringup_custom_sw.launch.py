"""Launch ROSbot XL simulation behind the common project interface."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    simulation_launch = os.path.join(
        get_package_share_directory("rosbot_gazebo"), "launch", "simulation.yaml"
    )
    official_stack = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(simulation_launch),
        launch_arguments={
            "robot_model": "rosbot_xl",
            "use_sim": "True",
            "configuration": LaunchConfiguration("configuration"),
            "arm_activate": "False",
            "mecanum": "True",
            "namespace": "rosbot",
            "tf_namespace_bridge": "True",
            "rviz": LaunchConfiguration("rviz"),
            "gz_headless_mode": LaunchConfiguration("gz_headless_mode"),
        }.items(),
    )
    adapter = Node(
        package="my_rosbot_control",
        executable="interface_adapter",
        name="rosbot_interface_adapter",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("configuration", default_value="basic"),
            DeclareLaunchArgument("rviz", default_value="True"),
            DeclareLaunchArgument("gz_headless_mode", default_value="False"),
            official_stack,
            adapter,
        ]
    )
