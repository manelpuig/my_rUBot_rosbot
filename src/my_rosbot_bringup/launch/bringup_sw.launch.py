from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    bringup_pkg = get_package_share_directory("my_rosbot_bringup")

    bridges_yaml = os.path.join(
        bringup_pkg, "config", "gz_bridges.yaml"
    )

    rviz_config = os.path.join(
        bringup_pkg, "rviz", "rosbot_sim.rviz"
    )

    rosbot_gazebo_pkg = get_package_share_directory("rosbot_gazebo")
    sim_launch = os.path.join(
        rosbot_gazebo_pkg, "launch", "simulation.launch.py"
    )

    rosbot_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={
            "robot_model": "rosbot_xl",
        }.items(),
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="extra_ros_gz_bridge",
        output="screen",
        parameters=[{"config_file": bridges_yaml}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription([
        rosbot_sim,
        gz_bridge,
        rviz,
    ])