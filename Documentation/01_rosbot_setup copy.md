# Rosbot

Webgraphy:
- https://husarion.com/
- https://husarion.com/tutorials/
- https://github.com/husarion/rosbot_ros/tree/jazzy


## 1. ROSbot repository

To work with ROSbot, you can use:
- Local PC Ubuntu24
- Local PC Ubuntu22 with Docker configuration

### 1.1 Local PC Ubuntu24

Whem working in a PC with Ubuntu24, the best procedure is:
- launch a `install_rosbot_native.sh` installation file

````bash
chmod +x install_rosbot_native.sh
./install_rosbot_native.sh
````

- Clone your custom repository
````bash
cd $HOME/rosbot_ws/src
git clone https://github.com/user/my_rUBot_rosbot.git
cd ..
colcon build
````

### 1.2 Local PC Ubuntu22 with Docker configuration
We have designed a University of Barcelona custom Docker-based ROS 2 jazzy environment to simplify student access to ROS 2 and ensure platform-independent workflows in robotics courses.

A proper Docker Image has been created with the custom configuration on Dockerfile and uploaded to my DockerHub account (https://hub.docker.com/r/manelpuig/ros2-jazzy-ub-rosbot).


**PC-ubuntu/linux** will work on SIM and LAB use. docker-compose.yaml is configured by default for LAB use.
- In `~/my_rUBot_rosbot/docker` review on:
    - `docker-compose.yaml` file: 
        - `ROS_DOMAIN_ID=1` variable to match your Group number.
        - `ROS_AUTOMATIC_DISCOVERY_RANGE` SUBNET (SIM use) or OFF (LAB use).
        - `ROS_STATIC_PEERS` not set (SIM use) or set with your robot IP (LAB Use).
        - Be sure to include: CYCLONEDDS_URI=file:///config/cyclonedds_pc.xml

- Open a terminal in `~/my_rUBot_rosbot/docker` and run:
    ````bash
    xhost +local:root            # only in case of Host Ubuntu to allow X11 for Docker 
    docker compose up
    ````
- Verify the environment variables are correctly set by checking the container startup output.
- In Host VScode you can `attach VScode`.

Then inside the container you can verify the simulation functionalities.
From the official repository you can test first in simulation:
- Launch the `basic` configuration
````bash
ros2 launch rosbot_gazebo simulation.yaml \
  robot_model:=rosbot_xl \
  use_sim:=True \
  configuration:=basic \
  arm_activate:=False \
  rviz:=True
````
- The `configuration` parameter can be:
````bash
basic
telepresence
autonomy
manipulation
manipulation_pro
custom
````
- Launch the `manipulation pro`:
````bash
ros2 launch rosbot_gazebo simulation.yaml \
  robot_model:=rosbot_xl \
  use_sim:=True \
  configuration:=manipulation_pro \
  arm_activate:=True \
  rviz:=True
````
![](./Images/ROSbot_xl_pro.png)

![](./Images/ROSbot_xl_pro_rviz.png)

In our custom repository we have created 2 new packages:
- my_rosbot_bringup
- my_rosbot_control

To simulate:
```bash
ros2 launch rosbot_gazebo simulation.launch.py
```

For our custom:
```bash
ros2 launch my_rosbot_bringup bringup_custom_sw.launch.py
ros2 run my_rosbot_control teleop_simple 
```

## 2. ROSbot Hardware setup

### 2.1. Connection settings

The ROSbot by default has:
- user: husarion
- pass: husarion
- Hostname: husarion
- IP address: 192.168.77.2

- First connection with ethernet cable via SSH protocol:
```bash
ssh husarion@192.168.77.2
or
ssh husarion@husarion.local
```
- Change the `netplan` configuration file to add your wifi:
```bash
sudo nano /etc/netplan/01-network-manager-all.yaml
```
> add your SSID and password in official configuration file
- We have assigned the wifi address: 192.168.1.5


### 2.2. First test

Manual ROSbot driving with teleop_twist_keyboard.
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

The web-based user interface is ready to use out-of-the-box. Simply open the following URL in your web browser:
```bash
http://192.168.1.5:8080/ui
```
