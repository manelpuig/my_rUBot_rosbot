# ROSbot XL setup

This guide defines the reference ROSbot XL development environment:

- Ubuntu 24.04
- ROS 2 Jazzy
- a native workspace at `~/rosbot_ws`
- the official Husarion ROSbot sources pinned to release `1.3.0`
- the custom robot and sensor repositories

The native Ubuntu 24.04 installation is the recommended, reproducible
workflow. Docker on an Ubuntu 22.04 host remains an alternative.

## 1. References

- [Husarion](https://husarion.com/)
- [Husarion tutorials](https://husarion.com/tutorials/)
- [Official `rosbot_ros` release 1.3.0](https://github.com/husarion/rosbot_ros/tree/1.3.0)

## 2. Native Ubuntu 24.04 installation

### 2.1. Obtain this repository and run the installer

The installation script is stored in this repository. On a fresh Ubuntu 24.04
computer, first clone the repository into a temporary development location:

```bash
mkdir -p ~/repositories
cd ~/repositories
git clone https://github.com/manelpuig/my_rUBot_rosbot.git
cd my_rUBot_rosbot/Documentation/Files/ROSbot_install
```

Run the installer:

```bash
chmod +x install_rosbot_native.sh
./install_rosbot_native.sh
```

The installer can be run again after an interrupted installation. It keeps
existing repositories and build products, verifies that `rosbot_ros` is at
tag `1.3.0` (commit `72f60ed7b3154bf2988526a095cdc7c92786ef71`),
and never removes workspace content automatically. It deliberately stops when
an existing checkout is at another revision instead of changing user work.

The script:

1. Installs ROS 2 Jazzy and its development tools.
2. Creates `~/rosbot_ws/src`.
3. Clones the official `husarion/rosbot_ros` tag `1.3.0`.
4. Imports the official hardware and simulation repositories with `vcs`.
5. Installs dependencies with `rosdep`.
6. Builds the workspace with `colcon`.
7. Adds the ROS 2 and workspace setup commands to `~/.bashrc`.

Open a new terminal after the script finishes, or load the environment:

```bash
source /opt/ros/jazzy/setup.bash
source ~/rosbot_ws/install/setup.bash
```

Verify the base environment:

```bash
echo "$ROS_DISTRO"
ros2 pkg prefix rosbot_gazebo
colcon list --base-paths ~/rosbot_ws/src
```

Expected results:

- `ROS_DISTRO` is `jazzy`.
- `rosbot_gazebo` is found in `~/rosbot_ws/install`.
- The official ROSbot packages appear in the package list.

Do not source a Humble workspace in the same terminal. Each ROS distribution
must use its own terminal and its own `build`, `install`, and `log`
directories.

### 2.2. Validate the official simulation

Always validate the official stack before adding the custom repositories:

```bash
source /opt/ros/jazzy/setup.bash
source ~/rosbot_ws/install/setup.bash

ros2 launch rosbot_gazebo simulation.yaml \
  robot_model:=rosbot_xl \
  use_sim:=True \
  configuration:=basic \
  arm_activate:=False \
  rviz:=True
```

The `robot_model` argument is required when `ROBOT_MODEL` is not defined.
Supported configurations include `basic`, `telepresence`, `autonomy`,
`manipulation`, `manipulation_pro`, and `custom`.

For example:

```bash
ros2 launch rosbot_gazebo simulation.yaml \
  robot_model:=rosbot_xl \
  use_sim:=True \
  configuration:=manipulation_pro \
  arm_activate:=True \
  rviz:=True
```

While the simulation is running, verify it from a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/rosbot_ws/install/setup.bash

ros2 node list
ros2 topic list | sort
ros2 topic echo /odometry/wheels --once
```

The basic simulation should provide `/cmd_vel`, `/odometry/wheels`,
`/imu/data`, `/tf`, and `/tf_static`, among other topics.

![ROSbot XL Pro in Gazebo](./Images/ROSbot_xl_pro.png)

![ROSbot XL Pro in RViz](./Images/ROSbot_xl_pro_rviz.png)

### 2.3. Add the custom robot and sensor repositories

Use one workspace for the complete ROSbot XL Jazzy system:

```text
~/rosbot_ws/
|-- src/
|   |-- official Husarion repositories
|   |-- my_rUBot_rosbot/
|   |-- ENS160_air_quality/
|   `-- my_rUBot_sensors/
|-- build/
|-- install/
`-- log/
```

Clone the custom repositories:

```bash
cd ~/rosbot_ws/src
git clone https://github.com/manelpuig/my_rUBot_rosbot.git
git clone https://github.com/manelpuig/ENS160_air_quality.git
git clone https://github.com/manelpuig/my_rUBot_sensors.git
```

Install their dependencies and build the complete workspace:

```bash
cd ~/rosbot_ws
source /opt/ros/jazzy/setup.bash

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

List the expected custom packages:

```bash
colcon list | grep -E \
  'my_rosbot|ens160|rubot_sensor|rubot_ens160'
```

Expected packages:

- `my_rosbot_bringup`
- `my_rosbot_control`
- `ens160_interfaces`
- `ens160_air_quality_ros`
- `rubot_sensor_bringup`
- `rubot_ens160_mapping`

### 2.4. Build and validate incrementally

First validate the generic ENS160 packages:

```bash
cd ~/rosbot_ws
colcon build --symlink-install \
  --packages-select ens160_interfaces ens160_air_quality_ros

source install/setup.bash
ros2 pkg executables ens160_air_quality_ros
ros2 interface list | grep ens160
```

Then validate the custom ROSbot packages:

```bash
colcon build --symlink-install \
  --packages-select my_rosbot_bringup my_rosbot_control

source install/setup.bash
ros2 pkg executables my_rosbot_control
```

The control package should provide:

```text
my_rosbot_control drive_test
my_rosbot_control interface_adapter
```

The project supports two bringup modes. Always validate the standard Husarion
mode first; it keeps the official topic names and message types unchanged.
The custom mode runs the official stack under `/rosbot` and exposes a small,
stable interface shared with the other teaching robots.

Standard Husarion simulation, through the local wrapper:

```bash
ros2 launch my_rosbot_bringup bringup_sw.launch.py
```

Custom compatible simulation:

```bash
ros2 launch my_rosbot_bringup bringup_custom_sw.launch.py
```

The robot currently has no camera. Do not use
`bringup_camera_sw.launch.py` for the physical robot. That legacy simulation
wrapper selects Husarion's `autonomy` sensor configuration and does not
represent the future Gemini 2 installation.

### 2.5. Standard and custom topic interfaces

The standard mode is the reference for diagnostics and vendor support:

| Purpose | Official topic | Type |
| --- | --- | --- |
| Velocity command | `/cmd_vel` | `geometry_msgs/msg/TwistStamped` |
| Filtered odometry | `/odometry/filtered` | `nav_msgs/msg/Odometry` |
| Wheel odometry | `/odometry/wheels` | `nav_msgs/msg/Odometry` |
| IMU | `/imu/data` | `sensor_msgs/msg/Imu` |
| Lidar | `/scan` | `sensor_msgs/msg/LaserScan` |

The custom mode preserves that stack under `/rosbot` and exposes:

| Common topic | Type | Official endpoint |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `/rosbot/manual/cmd_vel` (`TwistStamped`) |
| `/odom` | `nav_msgs/msg/Odometry` | `/rosbot/odometry/filtered` |
| `/wheel/odom` | `nav_msgs/msg/Odometry` | `/rosbot/odometry/wheels` |
| `/imu` | `sensor_msgs/msg/Imu` | `/rosbot/imu/data` |
| `/scan` | `sensor_msgs/msg/LaserScan` | `/rosbot/scan` |

The adapter adds the timestamp to velocity commands and republishes the sensor
messages without changing their frame IDs. Because the official stack runs in
the `rosbot` namespace, its TF bridge exposes frames such as `rosbot/odom` and
`rosbot/base_link` on global `/tf`. A future common repository must therefore
make `odom_frame` and `base_frame` configurable instead of assuming
`odom`/`base_footprint`. Topic names are normalized; frame names remain unique
and safe for multi-robot use.

### 2.6. Test ROSbot movement

`drive_test` publishes a `TwistStamped` command on the relative `cmd_vel`
topic for a limited time and then stops the robot. Use it with the standard
Husarion bringup. Its main parameters are:

| Parameter | Twist field | Unit | Default |
| --- | --- | --- | --- |
| `vx` | `linear.x` | m/s | `0.0` |
| `vy` | `linear.y` | m/s | `0.0` |
| `wz` | `angular.z` | rad/s | `0.0` |
| `duration` | Test duration | s | `5.0` |
| `publish_rate` | Publication frequency | Hz | `10.0` |

Example: move forward for three seconds:

```bash
ros2 run my_rosbot_control drive_test --ros-args \
  -p vx:=0.2 -p vy:=0.0 -p wz:=0.0 -p duration:=3.0
```

Example: move laterally to the left:

```bash
ros2 run my_rosbot_control drive_test --ros-args \
  -p vx:=0.0 -p vy:=0.2 -p wz:=0.0 -p duration:=3.0
```

Example: rotate counterclockwise:

```bash
ros2 run my_rosbot_control drive_test --ros-args \
  -p vx:=0.0 -p vy:=0.0 -p wz:=0.5 -p duration:=3.0
```

The node sends several zero-velocity commands before exiting. For a physical
robot, first raise the wheels or use a clear test area and keep an emergency
stop method available.

With the custom bringup, test the common `Twist` interface instead:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p stamped:=false
```

### 2.7. Development checkpoints

At each stage, record this information before changing source files:

```bash
git status --short --branch
echo "$ROS_DISTRO"
printenv AMENT_PREFIX_PATH
printenv COLCON_PREFIX_PATH
colcon list
```

After a failed build, keep the complete summary and the first relevant error.
Warnings containing an old workspace path usually indicate that another
workspace is sourced from `~/.bashrc`.

Avoid partial ROS package upgrades. Binary packages built on different dates
can produce `undefined symbol` errors. A fresh native installation built from
the official source workspace is the reference environment for this project.

## 3. Docker alternative for an Ubuntu 22.04 host

The University of Barcelona image provides a ROS 2 Jazzy environment for
simulation and laboratory use:

- Image: `manelpuig/ros2-jazzy-ub-rosbot`
- Configuration directory: `my_rUBot_rosbot/docker_jazzy`

Review the Compose configuration before starting it:

- Set `ROS_DOMAIN_ID` to the group number.
- Use `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` for simulation.
- Use `ROS_AUTOMATIC_DISCOVERY_RANGE=OFF` and configure
  `ROS_STATIC_PEERS` for laboratory use.
- Set `CYCLONEDDS_URI=file:///config/cyclonedds_pc.xml` when required.

On a Linux host:

```bash
cd ~/my_rUBot_rosbot/docker_jazzy
xhost +local:root
docker compose up
```

VS Code can then attach to the container. Inside it, use the official
simulation command from section 2.2.

## 4. Physical ROSbot setup

### 4.1. Connection settings

Default connection details:

- User: `husarion`
- Password: `husarion`
- Hostname: `husarion`
- Ethernet address: `192.168.77.2`

Connect through Ethernet:

```bash
ssh husarion@192.168.77.2
```

or, when mDNS is available:

```bash
ssh husarion@husarion.local
```

Configure Wi-Fi using the robot's current network-management procedure. If it
uses Netplan, inspect the existing files before editing:

```bash
ls -l /etc/netplan
```

Do not assume a fixed Netplan filename. Network credentials and assigned IP
addresses must not be committed to this repository.

### 4.2. Initial communication test

The web interface is normally available at:

```text
http://192.168.77.2:8080/ui
```

Use the actual Wi-Fi address when the robot is connected wirelessly.

First start and verify the standard official Husarion bringup on the robot:

```bash
source /opt/ros/jazzy/setup.bash
source ~/rosbot_ws/install/setup.bash
ros2 launch rosbot_bringup rosbot_xl.yaml configuration:=basic
```

The equivalent project wrapper is:

```bash
ros2 launch my_rosbot_bringup bringup_hw.launch.py
```

For a manual test with the standard interface:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p stamped:=true
```

After the official bringup works, stop it and test the custom compatible
bringup. Do not run the two bringups at the same time:

```bash
ros2 launch my_rosbot_bringup bringup_custom_hw.launch.py
```

From the control PC on the same `ROS_DOMAIN_ID`, verify and drive the common
interface:

```bash
ros2 topic info /cmd_vel
ros2 topic echo /odom --once
ros2 topic echo /imu --once
ros2 topic echo /scan --once
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p stamped:=false
```

Before moving the physical robot, raise its wheels or place it in a clear test
area and be ready to stop the command.

## 5. Future Gemini 2 camera

The current ROSbot has no camera. A Gemini 2 may be added later as a separate
camera driver and optional launch include. Its image, camera-info and depth
topics should be parameterized or remapped to the future common robot
interface. The current hardware bringups intentionally do not start an OAK-D,
Gemini or other camera driver.

## 6. Planned sensor integration

The ROSbot XL and ENS160 integration will use the same `~/rosbot_ws` workspace.
The planned combined simulation interface is:

```bash
ros2 launch rubot_sensor_bringup ens160_robot.launch.py \
  robot:=rosbot \
  mode:=simulation \
  start_sensor:=false
```

The intended physical interface is:

```bash
ros2 launch rubot_sensor_bringup ens160_robot.launch.py \
  robot:=rosbot \
  mode:=hardware \
  start_sensor:=true
```

These commands document the target design. Do not use them until the matching
`my_rUBot_sensors` implementation has been added and validated.
