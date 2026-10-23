# my_rUBot_rosbot

Project-specific ROS 2 Jazzy packages for the ROSbot XL. The official robot
description, simulation, drivers, controllers and localization remain in the
upstream [Husarion ROSbot stack](https://github.com/husarion/rosbot_ros).

## Packages

| Package | Purpose |
| --- | --- |
| `my_rosbot_bringup` | Standard Husarion and common-interface launch wrappers |
| `my_rosbot_control` | Motion test and ROSbot/common topic adapter |

## Installation

Follow [Documentation/01_rosbot_setup.md](Documentation/01_rosbot_setup.md) to
create the native Ubuntu 24.04 and ROS 2 Jazzy workspace.

## Simulation

Start the basic ROSbot XL configuration:

```bash
ros2 launch my_rosbot_bringup bringup_sw.launch.py
```

The wrapper accepts the official configuration name and display options:

```bash
ros2 launch my_rosbot_bringup bringup_sw.launch.py \
  configuration:=basic \
  arm_activate:=False \
  rviz:=True \
  gz_headless_mode:=False
```

Start the official `autonomy` sensor configuration:

```bash
ros2 launch my_rosbot_bringup bringup_camera_sw.launch.py
```

The camera launch no longer injects a second robot description or duplicates
Gazebo bridges. Sensors are selected through the official Husarion component
configuration.

The physical robot currently has no camera; the optional future camera is a
Gemini 2. For the normalized teaching interface, use:

```bash
ros2 launch my_rosbot_bringup bringup_custom_sw.launch.py
```

For hardware, validate the official stack first and then choose one wrapper:

```bash
# Official Husarion API
ros2 launch my_rosbot_bringup bringup_hw.launch.py

# Common API: /cmd_vel (Twist), /odom, /wheel/odom, /imu and /scan
ros2 launch my_rosbot_bringup bringup_custom_hw.launch.py
```

Do not run the standard and custom hardware bringups simultaneously. See the
setup guide for the complete topic mapping and verification procedure.

## Motion test

Raise the physical robot's wheels or use a clear simulation area before
running a movement test:

```bash
ros2 run my_rosbot_control drive_test --ros-args \
  -p vx:=0.3 \
  -p vy:=0.0 \
  -p wz:=0.0 \
  -p duration:=5.0 \
  -p publish_rate:=10.0
```

The node publishes on the relative `cmd_vel` topic, sends several zero-velocity
commands at the end and exits. `vx`, `vy`, and `wz` map to `linear.x`,
`linear.y`, and `angular.z`. Use a ROS remapping or namespace when required.

## License

MIT. See [LICENSE](LICENSE).
