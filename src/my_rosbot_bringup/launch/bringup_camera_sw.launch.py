from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    robot_model = LaunchConfiguration("robot_model")
    use_rviz    = LaunchConfiguration("use_rviz")

    bringup_share = FindPackageShare("my_rosbot_bringup")
    rosbot_gazebo_share = FindPackageShare("rosbot_gazebo")

    # ---- Your extra bridges (RGBD + LIDAR + clock) ----
    bridges_yaml = PathJoinSubstitution([bringup_share, "config", "gz_bridges.yaml"])
    rviz_config  = PathJoinSubstitution([bringup_share, "rviz", "rosbot_sim.rviz"])

    # ---- Your custom xacro (wrapper) that adds RGBD sensor/plugin ----
    # IMPORTANT: this xacro MUST call the official rosbot_xl.urdf.xacro with:
    #   use_sim:=true  and include_camera_mount:=true
    custom_xacro = PathJoinSubstitution([bringup_share, "urdf", "rosbot_xl_with_rgbd.urdf.xacro"])

    robot_description = {
        "robot_description": Command([
            "xacro ", custom_xacro,
            " robot_model:=", robot_model,
            " use_sim:=true",
            " include_camera_mount:=true",
        ])
    }

    # Robot State Publisher (your robot_description)
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    # Spawn + simulation from official rosbot_gazebo launch
    sim_launch = PathJoinSubstitution([rosbot_gazebo_share, "launch", "simulation.launch.py"])
    rosbot_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={
            "robot_model": robot_model,
        }.items(),
    )

    # Extra ros_gz_bridge using YAML
    extra_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="extra_ros_gz_bridge",
        output="screen",
        parameters=[{"config_file": bridges_yaml}],
    )

    # RViz (optional; in Docker sovint petarà per GLX. Deixa-ho desactivat per defecte)
    rviz = Node(
        condition=None,  # we gate it manually below with IfCondition-like behavior via simple argument usage
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # Simple gating without importing IfCondition (keeps file minimal)
    # If you prefer proper gating, I can rewrite with IfCondition.
    actions = [
        DeclareLaunchArgument("robot_model", default_value="rosbot_xl"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        LogInfo(msg=["Spawning ROSbot model: ", robot_model]),
        rsp,
        rosbot_sim,
        extra_bridge,
    ]

    if str(use_rviz.perform({})) == "true":
        actions.append(rviz)

    return LaunchDescription(actions)
