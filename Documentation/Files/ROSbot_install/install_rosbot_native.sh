#!/usr/bin/env bash
set -e

ROS_DISTRO=jazzy
WORKSPACE=$HOME/rosbot_ws

echo "=== Installing ROS 2 Jazzy Desktop + ROSbot workspace ==="

sudo apt update
sudo apt install -y software-properties-common curl gnupg lsb-release

sudo add-apt-repository universe -y

sudo apt update
sudo apt install -y curl gnupg

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade -y

sudo apt install -y \
  ros-${ROS_DISTRO}-desktop \
  ros-dev-tools \
  python3-rosdep \
  python3-vcstool \
  python3-colcon-common-extensions \
  git \
  wget \
  build-essential

sudo apt install -y \
  ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
  ros-${ROS_DISTRO}-teleop-twist-keyboard \
  ros-${ROS_DISTRO}-xacro \
  ros-${ROS_DISTRO}-robot-state-publisher \
  ros-${ROS_DISTRO}-joint-state-publisher \
  ros-${ROS_DISTRO}-joint-state-publisher-gui \
  ros-${ROS_DISTRO}-rviz2 \
  ros-${ROS_DISTRO}-ros-gz \
  ros-${ROS_DISTRO}-navigation2 \
  ros-${ROS_DISTRO}-nav2-bringup \
  ros-${ROS_DISTRO}-slam-toolbox

sudo rosdep init 2>/dev/null || true
rosdep update

mkdir -p ${WORKSPACE}/src
cd ${WORKSPACE}

git clone -b jazzy https://github.com/husarion/rosbot_ros.git src/rosbot_ros

source /opt/ros/${ROS_DISTRO}/setup.bash

vcs import src < src/rosbot_ros/rosbot/rosbot_hardware.repos
vcs import src < src/rosbot_ros/rosbot/rosbot_simulation.repos

rosdep install \
  --from-paths src \
  --ignore-src \
  -r \
  -y

colcon build --symlink-install

if ! grep -q "ROSbot native workspace" ~/.bashrc; then
cat << 'EOF' >> ~/.bashrc

# ROSbot native workspace
source /opt/ros/jazzy/setup.bash
source $HOME/rosbot_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
EOF
fi

echo "=== Finished ==="
echo "Now run:"
echo "source ~/.bashrc"